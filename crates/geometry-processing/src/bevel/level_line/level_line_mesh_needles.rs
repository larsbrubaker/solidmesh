//! The needle half of the working mesh: triangles with no height, the T-vertices among them, and
//! the counters that say what a build met. Ports `Bevel/LevelLine/LevelLineMesh.Needles.cs` (a
//! partial of `LevelLineMesh`, whose fields live in `level_line_mesh.rs`): so far its constant, the
//! no-height test, the left-alone count, the silent discard and the chain-edge test; the repair
//! lands with later packages.

use vector_math::{cs_math, Vector3};

use super::{LevelLineMesh, LevelLineTriangle};

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
