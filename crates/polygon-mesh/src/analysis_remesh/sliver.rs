//! The last stage of a remesh for analysis: the triangles still under the angle floor after the
//! passes get the same guarded edits aimed at them (flip, then collapse the shortest edge, then
//! split the longest, then move a free apex), a few rounds, until nothing improves.
//!
//! The repair aims at the soft target; only the hard floor can refuse a result (`mod.rs` says
//! why). A corner under either limit is forced, and left alone, only where the input's own
//! geometry leaves no triangulation that could do better, and only while it is no narrower than
//! the input's own angle there (the corner may be rounding away from it by 1 part in a million):
//! a corner at an input tip that a coarsened chord narrowed is the remesh's doing. The input
//! fixes two kinds of limit, and only these are measured, always on the input surface (after
//! `clean`), never on the mesh being built:
//! - The creases at a vertex cut its fan into sectors, and every triangulation that keeps the
//!   creases has at least one corner in each sector. A sector whose total input angle is itself
//!   under the floor forces the corner in it, and the triangle's small corner must be exactly
//!   that corner (between the sector's two crease edges) for the triangle to count as forced.
//!   This is why a corner between two coarsened chord edges is compared with the input sector,
//!   not with the chords: a chord that narrows a sector is the remesh's own doing, not a feature.
//! - A vertex with no sector to measure (no crease, or one crease ending there) is a closed fan,
//!   and a closed fan needs at least three corners, so its best worst corner is a third of its
//!   whole angle. A whole fan totalling under three times a limit cannot reach that limit at all
//!   (a needle's tip), so corners at it are forced at that limit when no narrower than its
//!   smallest input corner, and only then: a fan totalling 6 degrees cannot reach the 5 degree
//!   aim but can reach the 1 degree floor, so its corners are the repair's to open up to the
//!   floor. A fan under three aims is also a feature point in its own right: it is pinned where
//!   the input put it (`crease`), so a tip stays at its tip while the triangles around it change.
//!
//! Anything else under the floor is the remesh's to fix or the caller's to hear about through
//! `AngleFloorNotMet`, however thin the input triangles were: a rounding sliver on a flat face is
//! geometry only the triangulation invented, and the fan around it has a sector angle of 360.
//!
//! A triangle under the hard floor whose longest edge is past its target also gets that edge
//! split with no shape guard, as the last resort before moving a free apex. The long thin facets
//! of a needle cannot be opened any other way: halving one makes other thin triangles, so the
//! guarded split refuses, but only the vertices it adds give the collapses around the tip
//! anything to work with.

use super::engine::{worst_cos, Engine, P, SPLIT_ABOVE};
use super::AnalysisRemeshError;
use crate::g3::vector3d::distance;

/// Rounds of the repair: each round handles every remaining bad triangle once; past this they
/// are chasing each other rather than converging.
const MAX_ROUNDS: u32 = 16;
/// How far [`Engine::search_vertex`] looks, as fractions of the shortest edge at the vertex, and
/// in how many tangent directions per fraction.
const SEARCH_STEPS: [f64; 3] = [0.4, 0.2, 0.08];
const SEARCH_DIRECTIONS: usize = 8;
/// The vertex search runs only for triangles at least this fraction of the floor.
const SEARCH_NEAR_FLOOR: f64 = 0.6;
/// Corner comparisons with the input's own angle allow this much for rounding, as a ratio.
const ANGLE_ROUNDING: f64 = 1.0 - 1e-6;
/// The repair's work budget: edits per triangle of the mesh it started on, and a floor for small
/// meshes.
const REPAIR_EDITS_PER_TRIANGLE: usize = 3;
const MIN_REPAIR_EDITS: usize = 1024;
/// The repair never splits an edge shorter than this fraction of the local target: past that the
/// corner cannot be helped by more resolution, only by a move the budget should go to instead.
const REPAIR_SHORTEST_DIVISOR: f64 = 256.0;

impl Engine<'_> {
    /// Whether every corner of `t` under `cos_limit` is forced by the input's own geometry, with
    /// `limit_degrees` the same limit in degrees: see the module header for the rule.
    pub(super) fn is_forced(&self, t: i32, cos_limit: f64, limit_degrees: f64) -> bool {
        let verts = self.mesh().get_triangle(t);
        let points = self.triangle_points(t);
        let mut any = false;
        for i in 0..3 {
            let (at, x, y) = (verts[i], verts[(i + 1) % 3], verts[(i + 2) % 3]);
            let cos = self.corner_cos(points[i], points[(i + 1) % 3], points[(i + 2) % 3]);
            if cos <= cos_limit {
                continue;
            }
            if !self.corner_is_forced(at, x, y, cos, limit_degrees) {
                return false;
            }
            any = true;
        }
        any
    }

    /// Whether the corner at `at` wound `x`, `y`, whose cosine is `cos`, cannot be opened: the
    /// vertex is still where the input put it, the input angle there is itself under the limit,
    /// and the corner is no narrower than the input's own angle there. A corner narrower than
    /// the input was is the remesh's own doing, however small the input angle is.
    fn corner_is_forced(&self, at: i32, x: i32, y: i32, cos: f64, limit_degrees: f64) -> bool {
        if !self.is_at_input_position(at) {
            return false;
        }
        let corner = libm::acos(cos.min(1.0)).to_degrees();
        if let Some(sector) = self.crease.input_sector_angle(at, x, y) {
            return sector < limit_degrees && corner >= sector * ANGLE_ROUNDING;
        }
        self.crease
            .closed_fan_angle(at)
            .is_some_and(|whole| whole < 3.0 * limit_degrees)
            && corner >= self.crease.smallest_input_corner(at) * ANGLE_ROUNDING
    }

    fn corner_cos(&self, at: [f64; 3], p: [f64; 3], q: [f64; 3]) -> f64 {
        use crate::g3::vector3d::{dot, length, sub};
        let (u, v) = (sub(p, at), sub(q, at));
        let lengths = length(u) * length(v);
        if lengths <= 0.0 {
            1.0
        } else {
            dot(u, v) / lengths
        }
    }

    /// Whether the repair should work on `t`: a corner under the aim that the input does not
    /// force at the aim, or a corner under the hard floor that the input does not force at the
    /// floor. The second clause is what keeps a fan forced at the aim (too narrow for three aim
    /// corners) in the repair's reach while it can still be opened up to the floor.
    fn needs_repair(&self, t: i32, aim: f64) -> bool {
        let worst = self.triangle_worst_cos(t);
        (worst > self.cos_aim && !self.is_forced(t, self.cos_aim, aim))
            || (worst > self.cos_floor
                && !self.is_forced(t, self.cos_floor, self.opts.min_angle_floor_degrees))
    }

    /// Works the triangles below the aim in rounds; stops when a round changes nothing or the
    /// run's work budget is spent.
    pub(super) fn repair_slivers(&mut self) -> Result<(), AnalysisRemeshError> {
        if self.repair_edits_left.is_none() {
            let triangles = self.mesh().triangle_count();
            self.repair_edits_left =
                Some((triangles * REPAIR_EDITS_PER_TRIANGLE).max(MIN_REPAIR_EDITS));
        }
        for _ in 0..MAX_ROUNDS {
            let aim = self.opts.aim_degrees();
            let bad: Vec<i32> = self
                .mesh()
                .triangle_indices()
                .filter(|&t| self.needs_repair(t, aim))
                .collect();
            if bad.is_empty() {
                return Ok(());
            }
            let mut fixed = 0;
            for t in bad {
                self.tick()?;
                if self.repair_edits_left == Some(0) {
                    return Ok(());
                }
                if self.mesh().is_triangle(t)
                    && self.triangle_worst_cos(t) > self.cos_aim
                    && self.repair_triangle(t)?
                {
                    fixed += 1;
                    if let Some(left) = self.repair_edits_left.as_mut() {
                        *left = left.saturating_sub(1);
                    }
                }
            }
            if fixed == 0 {
                return Ok(());
            }
        }
        Ok(())
    }

    /// Moves free vertex `v` to whichever nearby point on the input sheet most improves the
    /// worst corner of its fan, trying eight tangent directions at three step sizes. This is the
    /// repair's last resort: the centroid is usually a good place, but a vertex just under the
    /// floor needs the move that targets the small corner, which the centroid rarely is.
    fn search_vertex(&mut self, v: i32) -> bool {
        use crate::g3::vector3d::{cross, length, normalized};
        if self.crease.is_vertex(v) || self.mesh().get_vtx_edge_count(v) < 3 {
            return false;
        }
        let normal = self.vertex_normal(v);
        if length(normal) <= 0.0 {
            return false;
        }
        // Any direction not parallel to the normal gives a tangent basis.
        let axis = if normal[0].abs() < 0.9 {
            [1.0, 0.0, 0.0]
        } else {
            [0.0, 1.0, 0.0]
        };
        let u = normalized(cross(normal, axis));
        let w = cross(normal, u);
        let p = self.mesh().vertex(v);
        let shortest = self
            .mesh()
            .vtx_vertices(v)
            .iter()
            .map(|&n| distance(p, self.mesh().vertex(n)))
            .fold(f64::INFINITY, f64::min);
        let mut best: Option<(f64, P)> = None;
        let current_worst = self.fan_worst_cos(v, p);
        for fraction in SEARCH_STEPS {
            let step = shortest * fraction;
            for direction in 0..SEARCH_DIRECTIONS {
                let angle = core::f64::consts::TAU * direction as f64 / SEARCH_DIRECTIONS as f64;
                let (c, s) = (libm::cos(angle), libm::sin(angle));
                let candidate = [
                    p[0] + (u[0] * c + w[0] * s) * step,
                    p[1] + (u[1] * c + w[1] * s) * step,
                    p[2] + (u[2] * c + w[2] * s) * step,
                ];
                let Some((on_surface, _)) = self.sheets().nearest(candidate, normal, step) else {
                    continue;
                };
                let worst = self.fan_worst_cos(v, on_surface);
                if worst < current_worst
                    && best.is_none_or(|(b, _)| worst < b)
                    && self.vertex_move_allowed(v, on_surface)
                {
                    best = Some((worst, on_surface));
                }
            }
            if let Some((_, position)) = best {
                self.set_vertex_position(v, position);
                return true;
            }
        }
        false
    }

    /// The worst corner cosine of the fan of `v` if `v` were at `at`.
    fn fan_worst_cos(&self, v: i32, at: P) -> f64 {
        self.mesh()
            .vtx_triangles(v)
            .into_iter()
            .map(|t| {
                let verts = self.mesh().get_triangle(t);
                let [a, b, c] = verts.map(|x| if x == v { at } else { self.mesh().vertex(x) });
                worst_cos(a, b, c)
            })
            .fold(-1.0, f64::max)
    }

    /// Tries to lift `t` above the floor; returns whether any edit was made.
    fn repair_triangle(&mut self, t: i32) -> Result<bool, AnalysisRemeshError> {
        let verts = self.mesh().get_triangle(t);
        let edges = self.mesh().get_tri_edges(t);
        let lengths = edges.map(|e| {
            let [a, b] = self.mesh().get_edge_v(e);
            distance(self.mesh().vertex(a), self.mesh().vertex(b))
        });
        let mut by_length: Vec<usize> = (0..3).collect();
        // Longest first; equal lengths fall back to edge order so the sequence is fixed.
        by_length.sort_by(|&x, &y| {
            lengths[y]
                .partial_cmp(&lengths[x])
                .unwrap_or(std::cmp::Ordering::Equal)
                .then(edges[x].cmp(&edges[y]))
        });

        for &i in &by_length {
            let [_, _, t0, t1] = self.mesh().get_edge(edges[i]);
            if self.try_flip(edges[i], t0, t1, true)? {
                return Ok(true);
            }
        }
        for &i in by_length.iter().rev() {
            if self.try_collapse(edges[i])? {
                return Ok(true);
            }
        }
        // Split the crease edges first: a coarsened crease regains its real polyline points, and
        // a straight one gains the vertices the far side of a thin wall needs to flip towards.
        let mut split_order = by_length.clone();
        split_order.sort_by_key(|&i| {
            let [a, b] = self.mesh().get_edge_v(edges[i]);
            !self.crease.has(a, b)
        });
        for &i in &split_order {
            let [a, b] = self.mesh().get_edge_v(edges[i]);
            let (pa, pb) = (self.mesh().vertex(a), self.mesh().vertex(b));
            let middle = [
                (pa[0] + pb[0]) * 0.5,
                (pa[1] + pb[1]) * 0.5,
                (pa[2] + pb[2]) * 0.5,
            ];
            if distance(pa, pb) >= self.target_at(middle)? / REPAIR_SHORTEST_DIVISOR
                && self.split_edge_guarded(edges[i], true)?
            {
                return Ok(true);
            }
        }
        // A needle under the hard floor whose longest edge is past its target has no shape left
        // to protect: the guarded split above refuses because halving it makes other thin
        // triangles, but only that puts vertices along a long thin tip for the collapses and
        // later rounds to work with. The edge-length test keeps this from running away.
        let longest = edges[by_length[0]];
        let [a, b] = self.mesh().get_edge_v(longest);
        let (pa, pb) = (self.mesh().vertex(a), self.mesh().vertex(b));
        let middle = [
            (pa[0] + pb[0]) * 0.5,
            (pa[1] + pb[1]) * 0.5,
            (pa[2] + pb[2]) * 0.5,
        ];
        if self.triangle_worst_cos(t) > self.cos_floor
            && distance(pa, pb) > SPLIT_ABOVE * self.target_at(middle)?
            && self.split_edge_guarded(longest, false)?
        {
            return Ok(true);
        }
        // The search is costly and only useful close to the floor, where a small move can tip the
        // corner over it; a deep sliver needs a flip or a split, not a nudge.
        let close_to_floor = libm::acos(self.triangle_worst_cos(t).min(1.0)).to_degrees()
            >= self.opts.aim_degrees() * SEARCH_NEAR_FLOOR;
        for v in verts {
            if self.smooth_vertex(v) || (close_to_floor && self.search_vertex(v)) {
                return Ok(true);
            }
        }
        Ok(false)
    }
}
