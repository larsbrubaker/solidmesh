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
/// Faces handled between cancel polls.
const CANCEL_STRIDE: usize = 256;

fn is_thin(mesh: &DMesh3, t: i32) -> bool {
    let [a, b, c] = mesh.get_triangle(t).map(|v| mesh.vertex(v));
    reliable_normal(a, b, c).is_none()
}

/// How much each vertex has already been moved, and the vertices a removal has failed on.
struct Cleaner<'a> {
    mesh: &'a mut DMesh3,
    limit: f64,
    moved: HashMap<i32, f64>,
    stuck: HashSet<i32>,
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
                _ => true,
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

/// Removes every thin face the bound allows.
pub(super) fn remove_thin_faces(
    mesh: &mut DMesh3,
    options: &AnalysisRemeshOptions<'_>,
) -> Result<(), AnalysisRemeshError> {
    let limit = options.max_deviation * CLEAN_FRACTION;
    let mut cleaner = Cleaner {
        mesh,
        limit,
        moved: HashMap::new(),
        stuck: HashSet::new(),
    };
    for _ in 0..MAX_PASSES {
        let thin: Vec<i32> = cleaner
            .mesh
            .triangle_indices()
            .filter(|&t| is_thin(cleaner.mesh, t))
            .collect();
        let mut changed = false;
        for (count, t) in thin.into_iter().enumerate() {
            if count % CANCEL_STRIDE == 0 {
                if let Some(token) = options.cancel {
                    if token.is_cancelled() {
                        return Err(AnalysisRemeshError::Cancelled);
                    }
                }
            }
            if cleaner.mesh.is_triangle(t) && is_thin(cleaner.mesh, t) {
                changed |= cleaner.remove(t);
            }
        }
        if !changed {
            break;
        }
    }
    Ok(())
}
