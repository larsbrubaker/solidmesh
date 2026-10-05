//! The needle half of the working mesh: triangles with no height, the T-vertices among them, and
//! the counters that say what a build met. Ports `Bevel/LevelLine/LevelLineMesh.Needles.cs` (a
//! partial of `LevelLineMesh`, whose fields live in `level_line_mesh.rs`): so far its constant, the
//! no-height test, the left-alone count, the silent discard, the chain-edge test, the re-tiling
//! through a T-vertex and the repair pass (`RepairNeedles` / `RepairNeedlesAround`, which start a
//! pass, land with another package).

use std::collections::VecDeque;

use vector_math::{cs_math, Vector3};

use super::{
    FieldCounters, LevelLineMesh, LevelLineTriangle, LevelLineWork, NeedleMeter, WorkError,
};

/// How far inside its long edge a needle's apex has to project before the needle is repaired, as a
/// fraction of that edge. An apex any nearer an end than this is that end for every purpose the
/// cut has, and re-tiling through it would mint a zero-length edge - and, on the sliver a repair
/// leaves, would start the repair round again.
pub const APEX_INSIDE: f64 = 1e-6;

impl LevelLineMesh {
    /// How many DISTINCT needles the repair found and left standing - a boundary one, one whose
    /// apex is at an END of its long edge, and the slivers repair itself leaves behind.
    pub fn needles_left_alone(&self) -> i32 {
        self.left_alone.len() as i32
    }

    /// True when the triangle stands no higher than `floor` over its own longest edge - a needle,
    /// whose three corners are collinear and whose apex can therefore stand INSIDE another edge.
    pub fn has_no_height(&self, t: LevelLineTriangle, floor: f64) -> bool {
        let a = self.vertices[t.a as usize];
        let b = self.vertices[t.b as usize];
        let c = self.vertices[t.c as usize];
        let longest = cs_math::max_f64(
            (b - a).length(),
            cs_math::max_f64((c - b).length(), (a - c).length()),
        );

        // Twice the area over the longest edge is the least of the three heights.
        longest <= 0.0 || Vector3::cross(&(b - a), c - a).length() / longest <= floor
    }

    /// Takes a triangle off its edges and marks it removed WITHOUT counting it in the band - the
    /// repair's own retired needles, which are not part of the gap.
    pub fn discard(&mut self, triangle: i32) {
        if self.removed[triangle as usize] {
            return;
        }

        self.detach(triangle, self.triangles[triangle as usize]);
        self.removed[triangle as usize] = true;
    }

    /// True when the vertex pair `a b` lies on an edge of the PICK, read through the INPUT edge
    /// both descend from: a piece of a split chain edge is still the chain edge.
    pub fn edge_lies_on_a_chain(&self, a: i32, b: i32) -> bool {
        let Some(is_chain_edge) = &self.is_chain_edge else {
            return false;
        };

        let behind = Self::merge(self.input_edge_behind(a), self.input_edge_behind(b));
        behind.0 >= 0 && behind.0 != behind.1 && is_chain_edge(behind.0, behind.1)
    }
}

impl LevelLineMesh {
    /// Re-tiles every triangle on edge a-b through vertex `at` (which stands inside it), dropping
    /// the needle that names `at` already; false where `at` is an end, there is no such edge, or
    /// every triangle on it names `at` (a BOUNDARY needle, which needs nothing).
    pub fn retile_through(&mut self, a: i32, b: i32, at: i32) -> bool {
        if at == a || at == b {
            return false;
        }

        let Some(standing) = self.edge_triangles.get(&Self::key(a, b)).cloned() else {
            return false;
        };

        let something_to_retile = standing.iter().any(|&parent| {
            let t = self.triangles[parent as usize];
            t.a != at && t.b != at && t.c != at
        });
        if !something_to_retile {
            return false;
        }

        if let Some(clean) = &mut self.answered_clean {
            clean.remove(&Self::key(a, b));
        }

        for parent in standing {
            let t = self.triangles[parent as usize];
            if t.a == at || t.b == at || t.c == at {
                // The needle itself: re-tiling it would give two triangles with two corners in one
                // place.
                self.discard(parent);
                continue;
            }

            let other = Self::opposite_corner(t, a, b);
            let (x, y) = Self::directed_edge(t, a, b);
            self.replace_triangle(parent, LevelLineTriangle::new(x, at, other, t.tag));
            self.add_triangle(LevelLineTriangle::new(at, y, other, t.tag));
        }

        true
    }

    /// The triangle's own direction along edge a-b, as (x, y).
    fn directed_edge(t: LevelLineTriangle, a: i32, b: i32) -> (i32, i32) {
        if (t.a == a && t.b == b) || (t.a == b && t.b == a) {
            (t.a, t.b)
        } else if (t.b == a && t.c == b) || (t.b == b && t.c == a) {
            (t.b, t.c)
        } else {
            (t.c, t.a)
        }
    }

    /// The repair pass, wrapped in the needle meter's call record when the meter is on;
    /// `around` false is the whole-mesh pass.
    pub fn metered_repair_from(
        &mut self,
        pending: &mut VecDeque<i32>,
        touched: &mut Vec<i32>,
        around: bool,
    ) -> Result<(), WorkError> {
        if !FieldCounters::meter_on() {
            return self.repair_from(pending, touched);
        }

        NeedleMeter::start_call();
        let result = self.repair_from(pending, touched);
        // C#'s `finally`: the call is closed whether or not the pass ran out its budget.
        NeedleMeter::end_call(around);
        result
    }

    /// Repairs the pending needles until none is left or the pass runs out its own bound (a mesh
    /// so tangled that repairing one keeps making another: flagged, never hung), charging the work
    /// budget once per dequeue.
    pub fn repair_from(
        &mut self,
        pending: &mut VecDeque<i32>,
        touched: &mut Vec<i32>,
    ) -> Result<(), WorkError> {
        let mut left = (4 * self.triangles.len() as i64) + 64;
        while let Some(triangle) = pending.pop_front() {
            // Charged like every other worklist in the module - once per dequeue - because this one can
            // spend real time: a repair pass's bound grows with the mesh it is growing.
            LevelLineWork::charge(1)?;

            if left <= 0 {
                self.repair_hit_its_bound = true;
                FieldCounters::note_repair_hit_its_bound();
                return Ok(());
            }

            left -= 1;
            if triangle as usize >= self.triangles.len() || self.removed[triangle as usize] {
                continue;
            }

            self.repair_one_needle(triangle, pending, touched);
        }

        Ok(())
    }

    /// Repairs one needle by re-tiling its neighbour across its long edge through the apex,
    /// dropping the needle - or leaves it standing (counted) where it is a boundary or end needle,
    /// its long edge is the pick's, the neighbour is on another side of the tagging, or the
    /// re-tiling would itself make a needle (so every repair strictly lowers the needle count).
    pub fn repair_one_needle(
        &mut self,
        needle: i32,
        pending: &mut VecDeque<i32>,
        touched: &mut Vec<i32>,
    ) {
        let t = self.triangles[needle as usize];
        if !self.has_no_height(t, self.needle_floor) {
            return;
        }

        // The apex is the corner NOT on the longest edge.
        let mut corner = 0;
        let mut longest = -1.0;
        for k in 0..3 {
            let length = self.length(t.corner(k), t.corner((k + 1) % 3));
            if length > longest {
                longest = length;
                corner = k;
            }
        }

        if longest <= 0.0 {
            self.leave_the_needle(needle);
            return;
        }

        let a = t.corner(corner);
        let b = t.corner((corner + 1) % 3);
        let apex = t.corner((corner + 2) % 3);

        // The long edge is the one a repair DELETES. Where it is the pick itself the needle stands.
        if self.edge_lies_on_a_chain(a, b) {
            self.leave_the_needle_on_its_chain(needle);
            return;
        }

        let along = self.vertices[b as usize] - self.vertices[a as usize];
        let at = (self.vertices[apex as usize] - self.vertices[a as usize]).dot(along)
            / along.length_squared();
        if at <= APEX_INSIDE || at >= 1.0 - APEX_INSIDE {
            self.leave_the_needle(needle);
            return;
        }

        let standing = self.triangles_on(a, b);
        if standing.len() != 2 {
            self.leave_the_needle(needle);
            return;
        }

        let across = if standing[0] == needle {
            standing[1]
        } else {
            standing[0]
        };
        let neighbour = self.triangles[across as usize];
        let other = Self::opposite_corner(neighbour, a, b);
        if other == apex {
            // The neighbour is the needle again, face to face.
            self.leave_the_needle(needle);
            return;
        }

        // A needle and its neighbour on DIFFERENT sides of the tagging is the band-reach frontier;
        // the replacements would both carry the neighbour's tag and could sever the pick's side.
        if neighbour.tag != t.tag {
            self.leave_the_needle(needle);
            return;
        }

        let (x, y) = Self::directed_edge(neighbour, a, b);

        // NO REPAIR MAY MAKE A NEEDLE: four nearly collinear points have no tiling without one, and
        // the re-tiling's own needle's repair flips it straight back (7.2 million repairs on the Many
        // Holes witness). Refusing here bounds the worklist by construction.
        if self.has_no_height(
            LevelLineTriangle::new(x, apex, other, neighbour.tag),
            self.needle_floor,
        ) || self.has_no_height(
            LevelLineTriangle::new(apex, y, other, neighbour.tag),
            self.needle_floor,
        ) {
            self.leave_the_needle(needle);
            return;
        }

        // The edge is about to stop existing; its two halves are edges the needle already carried.
        if let Some(clean) = &mut self.answered_clean {
            clean.remove(&Self::key(a, b));
        }

        if FieldCounters::meter_on() {
            let pa = self.vertices[a as usize];
            let height = Vector3::cross(
                &(self.vertices[b as usize] - pa),
                self.vertices[apex as usize] - pa,
            )
            .length()
                / longest;
            NeedleMeter::note_repair(
                needle,
                apex,
                a,
                b,
                across,
                height,
                longest,
                self.needle_floor,
            );
        }

        self.discard(needle);
        self.replace_triangle(
            across,
            LevelLineTriangle::new(x, apex, other, neighbour.tag),
        );
        let added = self.triangles.len() as i32;
        self.add_triangle(LevelLineTriangle::new(apex, y, other, neighbour.tag));
        if FieldCounters::meter_on() {
            NeedleMeter::note_repair_left(
                across,
                self.has_no_height(self.triangles[across as usize], self.needle_floor),
            );
            NeedleMeter::note_repair_left(
                added,
                self.has_no_height(self.triangles[added as usize], self.needle_floor),
            );
        }

        self.needles_repaired += 1;
        FieldCounters::note_needle(true);

        touched.push(across);
        touched.push(added);

        // Handed back to the worklist so the queue's own height test stays the one place that
        // decides.
        pending.push_back(across);
        pending.push_back(added);
    }
}
