//! The first step of a remesh for analysis: taking out, in the `f64` working copy, the faces too
//! thin to have a normal at the coordinate scale ([`reliable_normal`]), before anything measures
//! the surface.
//!
//! A thin face's normal is rounding, so nothing downstream can use it: a crease test across it
//! would read noise or nothing, and the repair would chase its corner down to rounding scale. The
//! face is really a place where its corners nearly coincide or one corner nearly lies on the
//! opposite side, and that is how it is removed. A needle (its apex beside one end of the longest
//! side) collapses the apex into that end. A cap (its apex beside the middle of the longest side)
//! splits the longest side at the apex's foot and collapses the apex into the new vertex, so the
//! apex's fan re-attaches on the longest side and a crease running along that side keeps every
//! vertex it had. Each removal moves one vertex by about the face's own height, and that distance
//! is charged against [`CLEAN_FRACTION`] of the caller's bound, summed along a vertex's history;
//! a removal the bound cannot pay for, or that the mesh topology refuses, is left alone.
//!
//! The same removal then takes out strips: a facet (faces joined across edges that bend less than
//! the crease angle) lying between two creases, narrower than [`CLEAN_FRACTION`] of the bound,
//! such as a chamfer or step a few micrometres wide left along an edge of an exported part.
//! `strip_faces` gives the exact test: every face narrow, the facet's mean width (twice its area
//! over its border) under the limit, and a crease border with two different neighbouring facets
//! each running many times that width. A facet with a face wider than the limit (a cap
//! narrowing to a sharp tip), with a mean width over it (a finely faceted wall), or set into a
//! single surrounding surface (a noise facet, however thin) is not a strip and stays. The width
//! test is a mean, so a long strip with a small wider tab in its plane made of narrow faces still
//! counts: its tab is reshaped like the rest, each vertex moved within the budget or left where
//! it is. The fraction is the clean-up's whole movement budget: a strip that
//! narrow is removed by moving its vertices no further than thin-face removal may, a quarter of
//! the bound, which leaves the remesh three quarters of it. The caller's bound says a feature
//! that narrow need not show, and left in it cannot be meshed: both of its sides are creases, so
//! any vertex the remesh puts inside it lies on two crease lines at once and the triangles it
//! makes with them have no width. Removing the strip merges its two creases into one sharp edge,
//! and the deviation measures compare its input faces with the nearest output surface of any
//! facing (`sheets`). A strip removal is held to more than a thin face's: every face it moves
//! must keep a normal and its facing, since these faces had a usable normal to begin with. A
//! strip a removal cannot take out under those checks stays as it was. Thin faces go first, so a
//! strip is measured on faces that all have normals, and again after any strip was removed.

use std::collections::{HashMap, HashSet};

use crate::g3::dmesh3::{DMesh3, MeshResult};
use crate::g3::vector3d::{distance, dot, sub};

use super::engine::P;
use super::sheets::reliable_normal;
use super::{AnalysisRemeshError, AnalysisRemeshOptions};

/// The share of the caller's bound the clean-up may spend moving vertices.
const CLEAN_FRACTION: f64 = 0.25;
/// Passes over the faces: a sliver flanked by a sliver needs a pass per layer.
const MAX_PASSES: u32 = 32;
/// How many times its own mean width a facet's crease border with one neighbouring facet must
/// run for that to count as a side of a strip. A strip has two such sides, one along each of
/// the facets it separates, each as long as the strip, so for a chamfer the ratio is its length
/// over its width, in the thousands. A compact facet cannot reach it: a square's side is twice
/// its mean width and an equilateral triangle's about three and a half times, so eight leaves
/// room for a short, irregular facet without admitting one that is about as long as it is wide.
const STRIP_SIDE_RATIO: f64 = 8.0;
/// Faces handled between cancel polls.
const CANCEL_STRIDE: usize = 256;

fn is_thin(mesh: &DMesh3, t: i32) -> bool {
    let [a, b, c] = mesh.get_triangle(t).map(|v| mesh.vertex(v));
    reliable_normal(a, b, c).is_none()
}

/// Whether face `t` has a usable normal and is no wider than `max_height` over its longest side.
fn is_narrow(mesh: &DMesh3, t: i32, max_height: f64) -> bool {
    use crate::g3::vector3d::{cross, length};
    let [a, b, c] = mesh.get_triangle(t).map(|v| mesh.vertex(v));
    if reliable_normal(a, b, c).is_none() {
        return false;
    }
    let longest = distance(a, b).max(distance(b, c)).max(distance(c, a));
    longest > 0.0 && length(cross(sub(b, a), sub(c, a))) / longest <= max_height
}

/// The faces of every strip: a facet (faces joined across edges that bend by no more than the
/// crease angle, `cos_sharp` being its cosine) in which every face is narrow, no wider on
/// average than `max_height`, and creased along two sides: its crease border with each of at
/// least two different neighbouring facets runs [`STRIP_SIDE_RATIO`] times its mean width. A
/// small facet set into one surrounding surface, however thin, borders a single facet and is a
/// noise facet, not a strip between two creases. A facet that is wide anywhere, such as a cap narrowing to a sharp
/// tip, is a real feature and is not a strip. In ascending face id.
fn strip_faces(mesh: &DMesh3, max_height: f64, cos_sharp: f64) -> Vec<i32> {
    use crate::g3::vector3d::{cross, length};
    let count = mesh.max_triangle_id().max(0) as usize;
    let mut parent: Vec<usize> = (0..count).collect();
    fn root(parent: &mut [usize], mut x: usize) -> usize {
        while parent[x] != x {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        x
    }
    let normal = |t: i32| {
        let [a, b, c] = mesh.get_triangle(t).map(|v| mesh.vertex(v));
        reliable_normal(a, b, c)
    };
    let mut bordered = vec![false; count];
    for e in mesh.edge_indices() {
        let [_, _, t0, t1] = mesh.get_edge(e);
        if t0 < 0 || t1 < 0 {
            continue;
        }
        match (normal(t0), normal(t1)) {
            (Some(n0), Some(n1)) if dot(n0, n1) >= cos_sharp => {
                let (r0, r1) = (
                    root(&mut parent, t0 as usize),
                    root(&mut parent, t1 as usize),
                );
                // The smaller root wins so the grouping does not depend on edge order.
                let (low, high) = (r0.min(r1), r0.max(r1));
                parent[high] = low;
            }
            (Some(_), Some(_)) => {
                bordered[t0 as usize] = true;
                bordered[t1 as usize] = true;
            }
            _ => {}
        }
    }
    // A facet's width is twice its area over its border length: a strip's border is its two
    // long sides, so this is its mean width, while a disc gives its radius and a cylinder wall
    // its height however finely they are faceted.
    let mut wide = vec![false; count];
    let mut has_crease = vec![false; count];
    let mut area = vec![0.0_f64; count];
    let mut border = vec![0.0_f64; count];
    for t in mesh.triangle_indices() {
        let r = root(&mut parent, t as usize);
        if !is_narrow(mesh, t, max_height) {
            wide[r] = true;
        }
        if bordered[t as usize] {
            has_crease[r] = true;
        }
        let [a, b, c] = mesh.get_triangle(t).map(|v| mesh.vertex(v));
        area[r] += 0.5 * length(cross(sub(b, a), sub(c, a)));
    }
    for e in mesh.edge_indices() {
        let [a, b, t0, t1] = mesh.get_edge(e);
        let length_of = distance(mesh.vertex(a), mesh.vertex(b));
        for (t, other) in [(t0, t1), (t1, t0)] {
            if t < 0 {
                continue;
            }
            let r = root(&mut parent, t as usize);
            if other < 0 || root(&mut parent, other as usize) != r {
                border[r] += length_of;
            }
        }
    }
    // The crease border each facet shares with each neighbouring facet. Lookup only, so map
    // order cannot reach the result.
    let mut sides: HashMap<(usize, usize), f64> = HashMap::new();
    for e in mesh.edge_indices() {
        let [a, b, t0, t1] = mesh.get_edge(e);
        if t0 < 0 || t1 < 0 {
            continue;
        }
        let creased =
            matches!((normal(t0), normal(t1)), (Some(n0), Some(n1)) if dot(n0, n1) < cos_sharp);
        if !creased {
            continue;
        }
        let length_of = distance(mesh.vertex(a), mesh.vertex(b));
        let (r0, r1) = (
            root(&mut parent, t0 as usize),
            root(&mut parent, t1 as usize),
        );
        *sides.entry((r0, r1)).or_insert(0.0) += length_of;
        *sides.entry((r1, r0)).or_insert(0.0) += length_of;
    }
    let mut long_sides = vec![0_u32; count];
    for (&(r, _), &run) in &sides {
        // `area` and `border` are only read for a facet with a crease border, which has length.
        if border[r] > 0.0 && run >= STRIP_SIDE_RATIO * (2.0 * area[r] / border[r]) {
            long_sides[r] += 1;
        }
    }
    mesh.triangle_indices()
        .filter(|&t| {
            let r = root(&mut parent, t as usize);
            !wide[r]
                && has_crease[r]
                && 2.0 * area[r] <= max_height * border[r]
                && long_sides[r] >= 2
        })
        .collect()
}

/// How much each vertex has already been moved, and the vertices a removal has failed on.
struct Cleaner<'a> {
    mesh: &'a mut DMesh3,
    limit: f64,
    moved: HashMap<i32, f64>,
    stuck: HashSet<i32>,
    /// Whether every moved face must keep a usable normal: set while removing strips.
    strict: bool,
    /// Every face the strip pass set out to remove, with its corners at the time.
    attempted: Vec<(i32, [P; 3])>,
}

impl Cleaner<'_> {
    fn charge(&self, v: i32) -> f64 {
        self.moved.get(&v).copied().unwrap_or(0.0)
    }

    /// Whether moving `v` to `to` keeps every face around it, other than the ones touching
    /// `other` that the collapse deletes, facing the way it did.
    fn fan_survives(&self, v: i32, other: i32, to: P) -> bool {
        self.mesh.vtx_triangles(v).into_iter().all(|t| {
            let verts = self.mesh.get_triangle(t);
            if verts.contains(&other) {
                return true;
            }
            let [a, b, c] = verts.map(|x| self.mesh.vertex(x));
            let [d, e, f] = verts.map(|x| if x == v { to } else { self.mesh.vertex(x) });
            match (reliable_normal(a, b, c), reliable_normal(d, e, f)) {
                (Some(old), Some(new)) => dot(old, new) > 0.0,
                (Some(_), None) => !self.strict,
                (None, _) => true,
            }
        })
    }

    /// Collapses `apex` into `keep` if the bound can pay for it and the fan keeps its facing.
    fn collapse(&mut self, apex: i32, keep: i32) -> bool {
        let at = self.mesh.vertex(keep);
        let spent = self.charge(apex) + distance(self.mesh.vertex(apex), at);
        if spent > self.limit || !self.fan_survives(apex, keep, at) {
            return false;
        }
        let (result, _) = self.mesh.collapse_edge(keep, apex);
        if result != MeshResult::Ok {
            return false;
        }
        self.mesh.set_vertex(keep, at);
        let carried = self.charge(keep).max(spent);
        self.moved.insert(keep, carried);
        true
    }

    /// Removes thin face `t`; returns whether the mesh changed.
    fn remove(&mut self, t: i32) -> bool {
        let edges = self.mesh.get_tri_edges(t);
        let lengths = edges.map(|e| {
            let [a, b] = self.mesh.get_edge_v(e);
            distance(self.mesh.vertex(a), self.mesh.vertex(b))
        });
        let longest = (0..3).fold(
            0,
            |best, i| {
                if lengths[i] > lengths[best] {
                    i
                } else {
                    best
                }
            },
        );
        let eid = edges[longest];
        let [a, b] = self.mesh.get_edge_v(eid);
        let verts = self.mesh.get_triangle(t);
        let Some(apex) = verts.into_iter().find(|&v| v != a && v != b) else {
            return false;
        };
        if self.stuck.contains(&apex) {
            return false;
        }
        let (pa, pb, pc) = (
            self.mesh.vertex(a),
            self.mesh.vertex(b),
            self.mesh.vertex(apex),
        );
        let along = sub(pb, pa);
        let span = dot(along, along);
        if span <= 0.0 {
            return self.collapse(apex, a);
        }
        let s = (dot(sub(pc, pa), along) / span).clamp(0.0, 1.0);
        let foot = [
            pa[0] + along[0] * s,
            pa[1] + along[1] * s,
            pa[2] + along[2] * s,
        ];
        let height = distance(pc, foot);
        let (near, near_distance) = if s < 0.5 {
            (a, distance(pc, pa))
        } else {
            (b, distance(pc, pb))
        };
        // Beside an end: the short side is what is thin, so the ends merge.
        if near_distance <= height * 2.0 + f64::EPSILON * span.sqrt() && self.collapse(apex, near) {
            return true;
        }
        if self.charge(apex) + height > self.limit {
            self.stuck.insert(apex);
            return false;
        }
        let (result, info) = self.mesh.split_edge(eid, s.clamp(0.25, 0.75));
        if result != MeshResult::Ok {
            self.stuck.insert(apex);
            return false;
        }
        self.mesh.set_vertex(info.v_new, foot);
        let parent_charge = self.charge(a).max(self.charge(b));
        self.moved.insert(info.v_new, parent_charge);
        if self.collapse(apex, info.v_new) {
            return true;
        }
        self.stuck.insert(apex);
        true
    }
}

/// Removes every thin face, then every strip, the bound allows.
///
/// Returns the corners of every strip face the clean-up removed, for [`absorbed_faces`]: empty
/// when no strip was touched.
pub(super) fn remove_thin_faces(
    mesh: &mut DMesh3,
    options: &AnalysisRemeshOptions<'_>,
) -> Result<Vec<[P; 3]>, AnalysisRemeshError> {
    let limit = options.max_deviation * CLEAN_FRACTION;
    let mut cleaner = Cleaner {
        mesh,
        limit,
        moved: HashMap::new(),
        stuck: HashSet::new(),
        strict: false,
        attempted: Vec::new(),
    };
    let thin = |mesh: &DMesh3| {
        mesh.triangle_indices()
            .filter(|&t| is_thin(mesh, t))
            .collect()
    };
    cleaner.remove_all(options, thin, is_thin)?;
    let thin_stuck = std::mem::take(&mut cleaner.stuck);
    cleaner.strict = true;
    let cos_sharp = libm::cos(options.sharp_edge_degrees.to_radians());
    let strips = |mesh: &DMesh3| strip_faces(mesh, limit, cos_sharp);
    let mut removed = Vec::new();
    if cleaner.remove_all(options, strips, |mesh, t| is_narrow(mesh, t, limit))? {
        // A face the pass set out to remove is gone, or no longer the face it was, once its
        // removal went through; one whose removal was refused is still there unchanged.
        for &(t, corners) in &cleaner.attempted {
            let intact = cleaner.mesh.is_triangle(t)
                && cleaner.mesh.get_triangle(t).map(|v| cleaner.mesh.vertex(v)) == corners;
            if !intact {
                removed.push(corners);
            }
        }
        // A strip removal that splits a side at the apex's foot and then cannot collapse the
        // apex leaves the split's faces behind, and one of them can be thin when the foot lands
        // beside an end: the thin-face pass runs again so nothing after the clean-up sees a face
        // with no normal. Only after a strip was touched, and with the vertices the first pass
        // gave up on still given up on, so a part with no strip is cleaned exactly as before.
        cleaner.strict = false;
        cleaner.stuck = thin_stuck;
        cleaner.remove_all(options, thin, is_thin)?;
    }
    Ok(removed)
}

/// The input faces the clean-up removed as strips, by triangle id of `mesh` (the input), found by
/// their exact corners in `removed` (what [`remove_thin_faces`] returned): what the deviation
/// measures compare with the nearest output surface of any facing (`sheets`). A strip the
/// clean-up left in place is not among them and is measured like any other face.
pub(super) fn absorbed_faces(mesh: &DMesh3, removed: &[[P; 3]]) -> Vec<bool> {
    let key = |corners: [P; 3]| {
        let mut bits = corners.map(|p| p.map(f64::to_bits));
        bits.sort_unstable();
        bits
    };
    // Lookup only, so set order cannot reach the result.
    let wanted: HashSet<[[u64; 3]; 3]> = removed.iter().map(|&c| key(c)).collect();
    let mut absorbed = vec![false; mesh.max_triangle_id().max(0) as usize];
    if wanted.is_empty() {
        return absorbed;
    }
    for t in mesh.triangle_indices() {
        if wanted.contains(&key(mesh.get_triangle(t).map(|v| mesh.vertex(v)))) {
            absorbed[t as usize] = true;
        }
    }
    absorbed
}

impl Cleaner<'_> {
    /// Removes the faces `pick` lists, in its ascending face id, pass after pass until a pass
    /// changes nothing; a listed face an earlier removal changed is removed only if `wanted`
    /// still holds for it. Returns whether the mesh changed.
    fn remove_all(
        &mut self,
        options: &AnalysisRemeshOptions<'_>,
        pick: impl Fn(&DMesh3) -> Vec<i32>,
        wanted: impl Fn(&DMesh3, i32) -> bool,
    ) -> Result<bool, AnalysisRemeshError> {
        let mut any = false;
        for _ in 0..MAX_PASSES {
            let picked = pick(self.mesh);
            let mut changed = false;
            for (count, t) in picked.into_iter().enumerate() {
                if count % CANCEL_STRIDE == 0 {
                    if let Some(token) = options.cancel {
                        if token.is_cancelled() {
                            return Err(AnalysisRemeshError::Cancelled);
                        }
                    }
                }
                if self.mesh.is_triangle(t) && wanted(self.mesh, t) {
                    if self.strict {
                        let corners = self.mesh.get_triangle(t).map(|v| self.mesh.vertex(v));
                        self.attempted.push((t, corners));
                    }
                    changed |= self.remove(t);
                }
            }
            any |= changed;
            if !changed {
                break;
            }
        }
        Ok(any)
    }
}
