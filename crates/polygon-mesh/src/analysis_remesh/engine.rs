//! The remesh passes of [`super::remesh_for_analysis`] on a working `DMesh3`: split long or
//! off-surface edges, collapse short ones, flip for valence, smooth free vertices in their
//! tangent plane, and refine any triangle whose interior strays from the input. Every edit is
//! refused unless the triangles it makes keep their facing, are no worse shaped than the local
//! worst (or above the floor), and pass the deviation guard against the same-facing input sheet
//! (`sheets`): the sheet test is what keeps one wall of a thin part from reading as the other.
//!
//! The deviation guard samples each new triangle at a grid of points and holds them to a
//! fraction of the caller's bound ([`GUARD_FRACTION`]), so whatever curvature the grid steps over
//! between samples still has room under the bound. A set of triangles that are coplanar before
//! and after an edit with the moved point in their plane covers the same region as before, so its
//! deviation cannot change and the sampling is skipped: that shortcut is what makes a mostly flat
//! CAD part affordable. `verify` then re-establishes the bound after the last pass and measures
//! the returned surface more densely.
//!
//! `sliver` drives the same edits at the triangles still below the floor. Edges are visited in
//! ascending id over a snapshot taken at the start of each pass and vertices in ascending id, so
//! a run is a fixed sequence of edits and two runs are identical.

use crate::g3::dmesh3::{DMesh3, MeshResult, INVALID_ID};
use crate::g3::dmesh3_queries::orient_tri_edge;
use crate::g3::dmesh_aabb_tree3::DMeshAABBTree3;
use crate::g3::vector3d::{add, cross, distance, dot, length, normalized, scale, sub};

use super::crease::CreaseNet;
use super::sheets::Sheets;
use super::{AnalysisRemeshError, AnalysisRemeshOptions};

/// An edge longer than this over its target is split. Together with [`COLLAPSE_BELOW`] it keeps
/// both halves of a split edge above the collapse threshold at a steady target, so a split is
/// not undone by the next collapse pass.
pub(super) const SPLIT_ABOVE: f64 = 1.33;
/// An edge shorter than this over its target is collapsed.
pub(super) const COLLAPSE_BELOW: f64 = 0.66;
/// The smoothing step, as a fraction of the way to the one-ring centroid.
const SMOOTH_STEP: f64 = 0.5;
/// A new triangle's normal must keep this dot with the old one: below it the edit folded the
/// surface over. A collapse may turn a face, but never by more than about 78 degrees.
const MIN_NORMAL_DOT: f64 = 0.2;
/// The margin by which a local worst must improve to count as an improvement, so a change that
/// only re-rounds an equal shape cannot keep the sliver pass going.
const IMPROVEMENT: f64 = 1e-9;
/// Edits between cancel checks: the check is an atomic read, but a pass makes tens of thousands
/// of edits and only needs to notice a cancel promptly.
const CANCEL_STRIDE: u32 = 256;
/// A split of an over-long edge may worsen the local worst corner as long as it stays at least
/// this fraction of the floor: enough to take the one step past a pair of slivers.
const SPLIT_SHAPE_LENIENCY: f64 = 0.85;
/// Below this fraction of the floor a corner is a needle, not a near miss: a split that worsens
/// it at all is refused however long the edge is.
const SPLIT_SLIVER_LIMIT: f64 = 0.4;
/// The deviation guard holds samples to this fraction of the caller's bound: the rest is the
/// allowance for what the sample grid steps over between its points.
const GUARD_FRACTION: f64 = 0.8;
/// Subdivisions per triangle side of the guard's sample grid.
const GUARD_GRID: usize = 4;
/// Two unit normals closer than this dot are the same plane for the coplanar shortcut.
const COPLANAR_DOT: f64 = 1.0 - 1e-12;

pub(super) type P = [f64; 3];

/// The cosine of the smallest corner angle of a triangle: 1 for a degenerate one, lower is better.
pub(super) fn worst_cos(a: P, b: P, c: P) -> f64 {
    let corner = |p: P, q: P, r: P| -> f64 {
        let (u, v) = (sub(q, p), sub(r, p));
        let lengths = length(u) * length(v);
        if lengths <= 0.0 {
            1.0
        } else {
            dot(u, v) / lengths
        }
    };
    corner(a, b, c).max(corner(b, c, a)).max(corner(c, a, b))
}

fn midpoint(a: P, b: P) -> P {
    scale(add(a, b), 0.5)
}

fn raw_normal(a: P, b: P, c: P) -> P {
    cross(sub(b, a), sub(c, a))
}

/// The barycentric sample points of a triangle with `grid` subdivisions per side, corners left
/// out: a triangle's corners are vertices already held on the surface.
pub(super) fn sample_grid(a: P, b: P, c: P, grid: usize) -> Vec<P> {
    let mut points = Vec::with_capacity((grid + 1) * (grid + 2) / 2);
    for i in 0..=grid {
        for j in 0..=(grid - i) {
            let k = grid - i - j;
            if i == grid || j == grid || k == grid {
                continue;
            }
            let (u, v, w) = (
                i as f64 / grid as f64,
                j as f64 / grid as f64,
                k as f64 / grid as f64,
            );
            points.push([
                a[0] * u + b[0] * v + c[0] * w,
                a[1] * u + b[1] * v + c[1] * w,
                a[2] * u + b[2] * v + c[2] * w,
            ]);
        }
    }
    points
}

/// Whether every triangle (given by its three corners) lies in one plane, tolerating only the
/// rounding of the coordinates themselves.
fn coplanar(triangles: &[[P; 3]], extra: Option<P>) -> bool {
    let Some(first) = triangles.first() else {
        return true;
    };
    let reference = raw_normal(first[0], first[1], first[2]);
    if length(reference) <= 0.0 {
        return false;
    }
    let unit = normalized(reference);
    let origin = first[0];
    let scale_of = |p: P| p[0].abs().max(p[1].abs()).max(p[2].abs());
    let tolerance = 1e-12 * (1.0 + scale_of(origin));
    let in_plane = |p: P| dot(sub(p, origin), unit).abs() <= tolerance;
    let triangles_ok = triangles.iter().all(|t| {
        let n = raw_normal(t[0], t[1], t[2]);
        length(n) > 0.0
            && dot(normalized(n), unit) >= COPLANAR_DOT
            && t.iter().all(|&p| in_plane(p))
    });
    triangles_ok && extra.is_none_or(in_plane)
}

/// One planned collapse: `remove` merges into `keep`, which then sits at `pos`.
struct CollapsePlan {
    keep: i32,
    remove: i32,
    pos: P,
    /// The next chain vertex when this removes a crease chain's interior vertex.
    chain_next: Option<i32>,
}

/// The working state of one remesh.
pub(super) struct Engine<'a> {
    mesh: DMesh3,
    sheets: Sheets<'a>,
    pub(super) crease: CreaseNet,
    pub(super) opts: &'a AnalysisRemeshOptions<'a>,
    pub(super) cos_floor: f64,
    /// The cosine of the angle the edits aim for: the soft target, at least the floor.
    pub(super) cos_aim: f64,
    /// Where the input vertices were when the net was built, to tell whether one is still there.
    input_positions: Vec<P>,
    /// Sliver-repair edits left before the repair stops: a work budget, so it can never chase a
    /// corner forever.
    pub(super) repair_edits_left: Option<usize>,
    guard_bound: f64,
    since_cancel_check: u32,
}

impl<'a> Engine<'a> {
    pub(super) fn new(
        mesh: DMesh3,
        tree: &'a DMeshAABBTree3<'a>,
        crease: CreaseNet,
        opts: &'a AnalysisRemeshOptions<'a>,
    ) -> Self {
        let input_positions = (0..mesh.max_vertex_id().max(0))
            .map(|v| {
                if mesh.is_vertex(v) {
                    mesh.vertex(v)
                } else {
                    [f64::NAN; 3]
                }
            })
            .collect();
        Self {
            mesh,
            sheets: Sheets::new(tree),
            crease,
            opts,
            cos_floor: libm::cos(opts.min_angle_floor_degrees.to_radians()),
            cos_aim: libm::cos(opts.aim_degrees().to_radians()),
            input_positions,
            repair_edits_left: None,
            guard_bound: opts.max_deviation * GUARD_FRACTION,
            since_cancel_check: 0,
        }
    }

    pub(super) fn mesh(&self) -> &DMesh3 {
        &self.mesh
    }

    pub(super) fn sheets(&self) -> &Sheets<'a> {
        &self.sheets
    }

    /// The passes, the sliver repair and the final refinement to the bound; returns the passes.
    pub(super) fn run(&mut self) -> Result<u32, AnalysisRemeshError> {
        self.check_cancel()?;
        let mut passes = 0;
        while passes < self.opts.max_passes {
            self.check_cancel()?;
            let edits = self.split_pass()?
                + self.collapse_pass()?
                + self.flip_pass()?
                + self.smooth_pass()?;
            passes += 1;
            if edits == 0 {
                break;
            }
        }
        self.repair_slivers()?;
        self.establish_bound()?;
        self.check_cancel()?;
        Ok(passes)
    }

    pub(super) fn check_cancel(&self) -> Result<(), AnalysisRemeshError> {
        match self.opts.cancel {
            Some(token) if token.is_cancelled() => Err(AnalysisRemeshError::Cancelled),
            _ => Ok(()),
        }
    }

    /// Counts an edit and checks the cancel flag every [`CANCEL_STRIDE`] edits.
    pub(super) fn tick(&mut self) -> Result<(), AnalysisRemeshError> {
        self.since_cancel_check += 1;
        if self.since_cancel_check >= CANCEL_STRIDE {
            self.since_cancel_check = 0;
            self.check_cancel()?;
        }
        Ok(())
    }

    /// The target length at `p`, clamped to the caller's limits, or the refusal of a bad answer.
    pub(super) fn target_at(&self, p: P) -> Result<f64, AnalysisRemeshError> {
        let value = (self.opts.target_edge_length)(p);
        if !value.is_finite() || value <= 0.0 {
            return Err(AnalysisRemeshError::InvalidTargetSize { at: p, value });
        }
        Ok(value.clamp(self.opts.min_edge_length, self.opts.max_edge_length))
    }

    /// Whether input vertex `v` is still where the input put it.
    pub(super) fn is_at_input_position(&self, v: i32) -> bool {
        self.input_positions
            .get(v as usize)
            .is_some_and(|&p| p == self.mesh.vertex(v))
    }

    pub(super) fn triangle_points(&self, t: i32) -> [P; 3] {
        self.mesh.get_triangle(t).map(|v| self.mesh.vertex(v))
    }

    pub(super) fn triangle_worst_cos(&self, t: i32) -> f64 {
        let [a, b, c] = self.triangle_points(t);
        worst_cos(a, b, c)
    }

    fn edges_snapshot(&self) -> Vec<i32> {
        self.mesh.edge_indices().collect()
    }

    /// The combined normal of an edge's two faces, or `None` where they cancel out.
    fn edge_normal(&self, t0: i32, t1: i32) -> Option<P> {
        let [a, b, c] = self.triangle_points(t0);
        let mut sum = raw_normal(a, b, c);
        if t1 >= 0 {
            let [d, e, f] = self.triangle_points(t1);
            sum = add(sum, raw_normal(d, e, f));
        }
        (length(sum) > 0.0).then(|| normalized(sum))
    }

    /// Whether every guard sample of triangle `a b c` lies within the guard fraction of the
    /// bound of the input sheet facing the same way.
    fn triangle_on_sheet(&self, a: P, b: P, c: P) -> bool {
        let n = raw_normal(a, b, c);
        if length(n) <= 0.0 {
            return false;
        }
        let unit = normalized(n);
        sample_grid(a, b, c, GUARD_GRID)
            .into_iter()
            .all(|p| self.sheets.distance(p, unit, self.guard_bound) <= self.guard_bound)
    }

    fn split_pass(&mut self) -> Result<u32, AnalysisRemeshError> {
        let mut splits = 0;
        for eid in self.edges_snapshot() {
            if !self.mesh.is_edge(eid) {
                continue;
            }
            self.tick()?;
            let [a, b, t0, t1] = self.mesh.get_edge(eid);
            let (pa, pb) = (self.mesh.vertex(a), self.mesh.vertex(b));
            let len = distance(pa, pb);
            let middle = midpoint(pa, pb);
            let target = self.target_at(middle)?;
            let is_crease = self.crease.has(a, b);
            let deviates = if is_crease {
                self.crease.deviation(a, b) > self.opts.max_deviation
            } else {
                self.edge_normal(t0, t1).is_none_or(|normal| {
                    self.sheets.distance(middle, normal, self.guard_bound) > self.guard_bound
                })
            };
            if (len > SPLIT_ABOVE * target || deviates) && self.split_edge_guarded(eid, true)? {
                splits += 1;
            }
        }
        // Sag lives in triangle interiors, so an edge check alone can miss it.
        let triangles: Vec<i32> = self.mesh.triangle_indices().collect();
        for t in triangles {
            if !self.mesh.is_triangle(t) {
                continue;
            }
            self.tick()?;
            let [a, b, c] = self.triangle_points(t);
            if !self.triangle_on_sheet(a, b, c) && self.split_longest_edge(t, true)? {
                splits += 1;
            }
        }
        Ok(splits)
    }

    /// Splits the longest edge of `t`, the one that most shortens where the triangle sags.
    pub(super) fn split_longest_edge(
        &mut self,
        t: i32,
        shape_guard: bool,
    ) -> Result<bool, AnalysisRemeshError> {
        let edges = self.mesh.get_tri_edges(t);
        let mut best = edges[0];
        let mut best_len = -1.0;
        for e in edges {
            let [a, b] = self.mesh.get_edge_v(e);
            let len = distance(self.mesh.vertex(a), self.mesh.vertex(b));
            if len > best_len {
                best_len = len;
                best = e;
            }
        }
        self.split_edge_guarded(best, shape_guard)
    }

    /// Splits `eid` unless the halves would be under the minimum edge length, the new vertex
    /// has no same-facing sheet near the edge, the new triangles would fold over, or (with
    /// `shape_guard`) they would be worse shaped than the local worst and under the floor. The
    /// refinement to the bound turns the shape guard off and lets the sliver pass recover shape.
    pub(super) fn split_edge_guarded(
        &mut self,
        eid: i32,
        shape_guard: bool,
    ) -> Result<bool, AnalysisRemeshError> {
        let [a, b, t0, t1] = self.mesh.get_edge(eid);
        let (pa, pb) = (self.mesh.vertex(a), self.mesh.vertex(b));
        let len = distance(pa, pb);
        if len * 0.5 < self.opts.min_edge_length {
            return Ok(false);
        }
        let is_crease = self.crease.has(a, b);
        let pos = if is_crease {
            match self.crease.split_position(a, b) {
                Some(position) => position,
                None => return Ok(false),
            }
        } else {
            let Some(normal) = self.edge_normal(t0, t1) else {
                return Ok(false);
            };
            match self.sheets.nearest(midpoint(pa, pb), normal, len) {
                Some((point, _)) => point,
                None => return Ok(false),
            }
        };

        let mut old_worst = -1.0_f64;
        let mut new_worst = -1.0_f64;
        for t in [t0, t1] {
            if t < 0 {
                continue;
            }
            let verts = self.mesh.get_triangle(t);
            let opposite = verts
                .into_iter()
                .find(|&v| v != a && v != b)
                .unwrap_or(verts[0]);
            let po = self.mesh.vertex(opposite);
            old_worst = old_worst.max(worst_cos(pa, pb, po));
            new_worst = new_worst
                .max(worst_cos(pa, pos, po))
                .max(worst_cos(pos, pb, po));
            // Oriented with the triangle's own winding so a fold reads as a flipped normal.
            let [x, y, z] = verts.map(|v| self.mesh.vertex(v));
            let old_n = raw_normal(x, y, z);
            if length(old_n) > 0.0 {
                let old_unit = normalized(old_n);
                for corner in [[pa, pos, po], [pos, pb, po]] {
                    let oriented = if self.same_winding(&verts, a, b) {
                        corner
                    } else {
                        [corner[1], corner[0], corner[2]]
                    };
                    let n = raw_normal(oriented[0], oriented[1], oriented[2]);
                    if length(n) <= 0.0 || dot(normalized(n), old_unit) < MIN_NORMAL_DOT {
                        return Ok(false);
                    }
                }
            }
        }
        if shape_guard && new_worst > self.cos_aim && new_worst > old_worst + IMPROVEMENT {
            // An edge past its target that a split would only just worsen is split anyway: a
            // strip of two slivers (a hole wall facet split along its diagonal) takes one split
            // that is momentarily no better before the flip that fixes it can exist. Only while
            // the result is still within reach of the floor: a needle far under it is not walked
            // further down.
            let overlong = len > SPLIT_ABOVE * self.target_at(midpoint(pa, pb))?;
            let floor = self.opts.aim_degrees();
            let (new_angle, old_angle) = (
                libm::acos(new_worst.min(1.0)).to_degrees(),
                libm::acos(old_worst.min(1.0)).to_degrees(),
            );
            let near_floor = new_angle >= SPLIT_SHAPE_LENIENCY * floor
                || (old_angle >= SPLIT_SLIVER_LIMIT * floor && new_angle >= 0.9 * old_angle);
            if !(overlong && near_floor) {
                return Ok(false);
            }
        }

        let (result, info) = self.mesh.split_edge(eid, 0.5);
        if result != MeshResult::Ok {
            return Ok(false);
        }
        self.mesh.set_vertex(info.v_new, pos);
        if is_crease {
            self.crease.record_split(a, b, info.v_new);
        }
        let triangles = self.mesh.triangle_count();
        if triangles > self.opts.max_triangles {
            return Err(AnalysisRemeshError::TriangleBudgetExceeded {
                triangles,
                limit: self.opts.max_triangles,
            });
        }
        Ok(true)
    }

    /// Whether triangle `verts` walks from `a` to `b`.
    fn same_winding(&self, verts: &[i32; 3], a: i32, b: i32) -> bool {
        (0..3).any(|i| verts[i] == a && verts[(i + 1) % 3] == b)
    }

    fn collapse_pass(&mut self) -> Result<u32, AnalysisRemeshError> {
        let mut collapses = 0;
        for eid in self.edges_snapshot() {
            if !self.mesh.is_edge(eid) {
                continue;
            }
            self.tick()?;
            let [a, b] = self.mesh.get_edge_v(eid);
            let (pa, pb) = (self.mesh.vertex(a), self.mesh.vertex(b));
            if distance(pa, pb) >= COLLAPSE_BELOW * self.target_at(midpoint(pa, pb))? {
                continue;
            }
            if self.try_collapse(eid)? {
                collapses += 1;
            }
        }
        Ok(collapses)
    }

    /// The collapses `eid` allows, in the order to try them: a crease edge only merges along
    /// its chain, a free edge ending on a crease keeps the crease vertex, and two crease
    /// vertices joined by a free edge never merge (that would pinch two creases together).
    fn collapse_plans(&self, eid: i32, a: i32, b: i32) -> Vec<CollapsePlan> {
        let (pa, pb) = (self.mesh.vertex(a), self.mesh.vertex(b));
        if self.crease.has(a, b) {
            let mut plans = Vec::new();
            for (keep, remove, pos) in [(a, b, pa), (b, a, pb)] {
                let Some(next) = self.crease.chain_neighbour(&self.mesh, remove, keep) else {
                    continue;
                };
                if next == keep || self.mesh.find_edge(keep, next) != INVALID_ID {
                    continue;
                }
                let within = self
                    .crease
                    .merged_deviation(keep, remove, next)
                    .is_some_and(|d| d <= self.opts.max_deviation);
                if within {
                    plans.push(CollapsePlan {
                        keep,
                        remove,
                        pos,
                        chain_next: Some(next),
                    });
                }
            }
            return plans;
        }
        let (a_crease, b_crease) = (self.crease.is_vertex(a), self.crease.is_vertex(b));
        let plan = match (a_crease, b_crease) {
            (true, true) => return Vec::new(),
            (true, false) => (a, b, pa),
            (false, true) => (b, a, pb),
            (false, false) => {
                let [_, _, t0, t1] = self.mesh.get_edge(eid);
                let pos = self.edge_normal(t0, t1).and_then(|normal| {
                    self.sheets
                        .nearest(midpoint(pa, pb), normal, distance(pa, pb))
                });
                let Some((position, _)) = pos else {
                    return Vec::new();
                };
                (a.min(b), a.max(b), position)
            }
        };
        vec![CollapsePlan {
            keep: plan.0,
            remove: plan.1,
            pos: plan.2,
            chain_next: None,
        }]
    }

    /// Collapses `eid` if a plan passes the guard. Returns whether it collapsed.
    pub(super) fn try_collapse(&mut self, eid: i32) -> Result<bool, AnalysisRemeshError> {
        let [a, b, t0, t1] = self.mesh.get_edge(eid);
        for plan in self.collapse_plans(eid, a, b) {
            if !self.collapse_allowed(&plan, t0, t1)? {
                continue;
            }
            let (result, _info) = self.mesh.collapse_edge(plan.keep, plan.remove);
            if result != MeshResult::Ok {
                continue;
            }
            self.mesh.set_vertex(plan.keep, plan.pos);
            if let Some(next) = plan.chain_next {
                self.crease
                    .record_chain_collapse(plan.keep, plan.remove, next);
            }
            return Ok(true);
        }
        Ok(false)
    }

    fn collapse_allowed(
        &self,
        plan: &CollapsePlan,
        t0: i32,
        t1: i32,
    ) -> Result<bool, AnalysisRemeshError> {
        let &CollapsePlan {
            keep, remove, pos, ..
        } = plan;
        let mut triangles = self.mesh.vtx_triangles(remove);
        triangles.extend(self.mesh.vtx_triangles(keep));
        triangles.sort_unstable();
        triangles.dedup();

        let mut old_worst = -1.0_f64;
        let mut new_worst = -1.0_f64;
        let mut old_triangles = Vec::with_capacity(triangles.len());
        let mut new_triangles = Vec::with_capacity(triangles.len());
        for t in triangles {
            let old = self.triangle_points(t);
            old_worst = old_worst.max(worst_cos(old[0], old[1], old[2]));
            old_triangles.push(old);
            if t == t0 || t == t1 {
                continue;
            }
            let verts = self.mesh.get_triangle(t);
            if verts.contains(&keep) && verts.contains(&remove) {
                return Ok(false);
            }
            let moved = verts.map(|v| {
                if v == keep || v == remove {
                    pos
                } else {
                    self.mesh.vertex(v)
                }
            });
            let old_normal = raw_normal(old[0], old[1], old[2]);
            let new_normal = raw_normal(moved[0], moved[1], moved[2]);
            if length(old_normal) > 0.0
                && (length(new_normal) <= 0.0
                    || dot(normalized(old_normal), normalized(new_normal)) < MIN_NORMAL_DOT)
            {
                return Ok(false);
            }
            new_worst = new_worst.max(worst_cos(moved[0], moved[1], moved[2]));
            for (i, &v) in verts.iter().enumerate() {
                if v == keep || v == remove {
                    continue;
                }
                let middle = midpoint(pos, moved[i]);
                if distance(pos, moved[i]) > SPLIT_ABOVE * self.target_at(middle)? {
                    return Ok(false);
                }
            }
            new_triangles.push(moved);
        }
        if !(new_worst <= self.cos_aim || new_worst < old_worst - IMPROVEMENT) {
            return Ok(false);
        }
        Ok(coplanar(&old_triangles, Some(pos))
            || new_triangles
                .iter()
                .all(|t| self.triangle_on_sheet(t[0], t[1], t[2])))
    }

    /// The target number of edges at a vertex: six on a smooth surface, four on a straight
    /// crease (half the fan is on each side of it), three at a corner of creases.
    fn valence_target(&self, v: i32) -> i32 {
        match self.crease.degree(v) {
            0 => 6,
            2 => 4,
            _ => 3,
        }
    }

    fn flip_pass(&mut self) -> Result<u32, AnalysisRemeshError> {
        let mut flips = 0;
        for eid in self.edges_snapshot() {
            if !self.mesh.is_edge(eid) {
                continue;
            }
            self.tick()?;
            let [a, b, t0, t1] = self.mesh.get_edge(eid);
            if t1 < 0 || self.crease.has(a, b) {
                continue;
            }
            let [c, d] = self.mesh.get_edge_opposing_v(eid);
            let error = |v: i32, delta: i32, engine: &Self| {
                (engine.mesh.get_vtx_edge_count(v) + delta - engine.valence_target(v)).abs()
            };
            let before =
                error(a, 0, self) + error(b, 0, self) + error(c, 0, self) + error(d, 0, self);
            let after =
                error(a, -1, self) + error(b, -1, self) + error(c, 1, self) + error(d, 1, self);
            if after < before && self.try_flip(eid, t0, t1, false)? {
                flips += 1;
            }
        }
        Ok(flips)
    }

    /// Flips `eid` when the new triangles keep their facing and the deviation guard and, for a
    /// valence flip, are no worse than the local worst or above the floor; with `strict` they
    /// must strictly beat it, which is what the sliver pass wants.
    pub(super) fn try_flip(
        &mut self,
        eid: i32,
        t0: i32,
        t1: i32,
        strict: bool,
    ) -> Result<bool, AnalysisRemeshError> {
        let [mut a, mut b, _, _] = self.mesh.get_edge(eid);
        if self.crease.has(a, b) || t1 < 0 {
            return Ok(false);
        }
        let [c, d] = self.mesh.get_edge_opposing_v(eid);
        if self.mesh.find_edge(c, d) != INVALID_ID {
            return Ok(false);
        }
        let tri = self.mesh.get_triangle(t0);
        orient_tri_edge(&mut a, &mut b, &tri);
        let (pa, pb, pc, pd) = (
            self.mesh.vertex(a),
            self.mesh.vertex(b),
            self.mesh.vertex(c),
            self.mesh.vertex(d),
        );
        let old_worst = worst_cos(pa, pb, pc).max(worst_cos(pb, pa, pd));
        let new_worst = worst_cos(pc, pd, pb).max(worst_cos(pd, pc, pa));
        let n0 = raw_normal(pa, pb, pc);
        let n1 = raw_normal(pb, pa, pd);
        let f0 = raw_normal(pc, pd, pb);
        let f1 = raw_normal(pd, pc, pa);
        for old in [n0, n1] {
            if length(old) <= 0.0 {
                continue;
            }
            for new in [f0, f1] {
                if length(new) <= 0.0 || dot(normalized(old), normalized(new)) < MIN_NORMAL_DOT {
                    return Ok(false);
                }
            }
        }
        let accepted = if strict {
            new_worst < old_worst - IMPROVEMENT
        } else {
            new_worst <= self.cos_aim || new_worst <= old_worst
        };
        if !accepted {
            return Ok(false);
        }
        let same_plane = coplanar(&[[pa, pb, pc], [pb, pa, pd]], None);
        if !same_plane
            && (!self.triangle_on_sheet(pc, pd, pb) || !self.triangle_on_sheet(pd, pc, pa))
        {
            return Ok(false);
        }
        let (result, _) = self
            .mesh
            .flip_edge(eid)
            .map_err(|e| AnalysisRemeshError::BrokenTopology(e.0))?;
        Ok(result == MeshResult::Ok)
    }

    fn smooth_pass(&mut self) -> Result<u32, AnalysisRemeshError> {
        let mut moved = 0;
        let vertices: Vec<i32> = self.mesh.vertex_indices().collect();
        for v in vertices {
            self.tick()?;
            if self.smooth_vertex(v) {
                moved += 1;
            }
        }
        Ok(moved)
    }

    /// Moves free vertex `v` towards its one-ring centroid in its tangent plane, then back onto
    /// the input sheet it faces, unless that leaves its fan worse shaped, folded over or past
    /// the deviation guard: a vertex moved onto the line of a crease edge would make a triangle
    /// of no area with that edge.
    pub(super) fn smooth_vertex(&mut self, v: i32) -> bool {
        if self.crease.is_vertex(v) || self.mesh.get_vtx_edge_count(v) < 3 {
            return false;
        }
        let ring = self.mesh.vtx_vertices(v);
        let p = self.mesh.vertex(v);
        let mut sum = [0.0; 3];
        for &n in &ring {
            sum = add(sum, self.mesh.vertex(n));
        }
        let center = scale(sum, 1.0 / ring.len() as f64);
        let delta = sub(center, p);
        let normal = self.vertex_normal(v);
        if length(normal) <= 0.0 {
            return false;
        }
        let tangential = sub(delta, scale(normal, dot(delta, normal)));
        let candidate = add(p, scale(tangential, SMOOTH_STEP));
        let reach = distance(p, candidate) + self.guard_bound;
        let Some((on_surface, _)) = self.sheets.nearest(candidate, normal, reach) else {
            return false;
        };
        if !self.vertex_move_allowed(v, on_surface) {
            return false;
        }
        self.mesh.set_vertex(v, on_surface);
        true
    }

    pub(super) fn vertex_normal(&self, v: i32) -> P {
        let mut sum = [0.0; 3];
        for t in self.mesh.vtx_triangles(v) {
            let [a, b, c] = self.triangle_points(t);
            sum = add(sum, raw_normal(a, b, c));
        }
        if length(sum) > 0.0 {
            normalized(sum)
        } else {
            sum
        }
    }

    /// Moves `v` to `position`, for an edit that has already passed its guards.
    pub(super) fn set_vertex_position(&mut self, v: i32, position: P) {
        self.mesh.set_vertex(v, position);
    }

    pub(super) fn vertex_move_allowed(&self, v: i32, to: P) -> bool {
        let mut old_worst = -1.0_f64;
        let mut new_worst = -1.0_f64;
        let mut old_triangles = Vec::new();
        let mut new_triangles = Vec::new();
        for t in self.mesh.vtx_triangles(v) {
            let verts = self.mesh.get_triangle(t);
            let old = self.triangle_points(t);
            let moved = verts.map(|x| if x == v { to } else { self.mesh.vertex(x) });
            let old_normal = raw_normal(old[0], old[1], old[2]);
            let new_normal = raw_normal(moved[0], moved[1], moved[2]);
            if length(old_normal) > 0.0
                && (length(new_normal) <= 0.0
                    || dot(normalized(old_normal), normalized(new_normal)) < MIN_NORMAL_DOT)
            {
                return false;
            }
            old_worst = old_worst.max(worst_cos(old[0], old[1], old[2]));
            new_worst = new_worst.max(worst_cos(moved[0], moved[1], moved[2]));
            old_triangles.push(old);
            new_triangles.push(moved);
        }
        if !(new_worst <= self.cos_aim.max(old_worst) || new_worst < old_worst - IMPROVEMENT) {
            return false;
        }
        coplanar(&old_triangles, Some(to))
            || new_triangles
                .iter()
                .all(|t| self.triangle_on_sheet(t[0], t[1], t[2]))
    }
}
