//! The flush end at a COVE-TO-ROUND corner: where a rim cut at its sense flips
//! (`chain_on_the_mesh::split_at_setback_steps` under `WorkBudget::sense_flips_to_cut`) has a
//! cove run and a round run meeting end to end, each run stops flush on one flat plane through the
//! corner, and the two flat ends meet there in a short sharp seam. Each run stops on the plane of
//! the OTHER run's side face, the face its band runs into at the corner, as a square crease end
//! stops on the face it meets ([`against_the_other_side_face`]): that plane holds the edges the
//! mesh already has there, so the cut leaves no sliver beside a facet seam. Where there is no such
//! face, where it does not part the runs (this run's next vertex off it on the band's side, the
//! other's in it or past it), or where it would cut into that face past the corner (a faceted
//! curved face whose facets about the corner stand on both sides of it), the plane is read along
//! the crease running in ([`seam_plane`]). [`flip_corner_plane_paths`] says which rule ran. The
//! clip is a stopped end's (`stopped_end_clip.rs`): an [`EndRayBound`] of no reach marked
//! `stopped`, so the band stops on the plane and the fill cuts the end flat there.

use std::collections::HashSet;

use vector_math::Vector3;

use super::super::float_grid;
use super::chain_on_the_mesh::{chain_edges, per_edge_convexity};
use super::end_face_sense::{face_normal, Corners, COPLANAR_TOLERANCE, SEPARATE_END_FACE_DEGREES};
use super::{ChainField, EndRayBound, LevelLineWork};

/// Which rule set a flip corner's plane ([`flip_corner_plane_paths`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SeamPlanePath {
    /// The other run's side face ([`against_the_other_side_face`]).
    OtherSideFace,
    /// The crease running in and the round's side-face edge leaving the corner.
    CreaseAndEdge,
    /// The crease running in and the runs' bisector.
    CreaseAndBisector,
    /// The mitre between the runs.
    Mitre,
}

/// Each chain's flip-corner clips, as (first end, last end): an end where the chain meets another
/// chain of the pick at a vertex where one's crease is a cove and the other's a round, at the flips
/// the build cuts (`WorkBudget::sense_flips_to_cut`) and nowhere else.
pub fn flip_corner_clips(
    positions: &[Vector3],
    indices: &[Corners],
    chains: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: &dyn Fn(i32, i32) -> bool,
) -> Vec<(Option<EndRayBound>, Option<EndRayBound>)> {
    with_paths(positions, indices, chains, closed, is_feature_edge)
        .into_iter()
        .map(|(first, last)| (first.map(|c| c.0), last.map(|c| c.0)))
        .collect()
}

/// Which rule set each flip-corner clip's plane, as (first end, last end), read as
/// [`flip_corner_clips`] reads the clips.
pub fn flip_corner_plane_paths(
    positions: &[Vector3],
    indices: &[Corners],
    chains: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: &dyn Fn(i32, i32) -> bool,
) -> Vec<(Option<SeamPlanePath>, Option<SeamPlanePath>)> {
    with_paths(positions, indices, chains, closed, is_feature_edge)
        .into_iter()
        .map(|(first, last)| (first.map(|c| c.1), last.map(|c| c.1)))
        .collect()
}

/// One end's clip and the rule that set its plane.
type ClipAndPath = Option<(EndRayBound, SeamPlanePath)>;

fn with_paths(
    positions: &[Vector3],
    indices: &[Corners],
    chains: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: &dyn Fn(i32, i32) -> bool,
) -> Vec<(ClipAndPath, ClipAndPath)> {
    let mut clips: Vec<(ClipAndPath, ClipAndPath)> = vec![(None, None); chains.len()];
    let to_cut: Vec<i32> = LevelLineWork::current()
        .map(|budget| budget.sense_flips_to_cut.borrow().clone())
        .unwrap_or_default();
    if to_cut.is_empty() {
        return clips;
    }

    // Each open chain's end edges and their convexity: (end vertex, next vertex, convex).
    let ends: Vec<Option<[(i32, i32, bool); 2]>> = chains
        .iter()
        .enumerate()
        .map(|(k, chain)| {
            if closed[k] || chain.len() < 2 {
                return None;
            }
            let edges = chain_edges(chain, false);
            // The pick was validated and split on these same edges, every one standing on two
            // triangles, so the reading cannot fail; were it to, the chain's ends take no clip.
            let convex = per_edge_convexity(positions, indices, &edges).ok()?;
            let n = chain.len();
            Some([
                (chain[0], chain[1], convex[0]),
                (chain[n - 1], chain[n - 2], convex[convex.len() - 1]),
            ])
        })
        .collect();

    let picked: HashSet<(i32, i32)> = chains
        .iter()
        .enumerate()
        .flat_map(|(k, chain)| chain_edges(chain, closed[k]))
        .map(|(a, b)| (a.min(b), a.max(b)))
        .collect();

    for k in 0..chains.len() {
        let Some(mine) = ends[k] else { continue };
        for (at, &(vertex, next, convex)) in mine.iter().enumerate() {
            if !to_cut.contains(&vertex) {
                continue;
            }
            // The other run ending at this vertex with the other sense.
            let other = (0..chains.len()).filter(|&j| j != k).find_map(|j| {
                ends[j].and_then(|theirs| {
                    theirs
                        .iter()
                        .find(|&&(v, _, c)| v == vertex && c != convex)
                        .copied()
                })
            });
            let Some((_, their_next, _)) = other else {
                continue;
            };

            let corner = positions[vertex as usize];
            let mine_out = (positions[next as usize] - corner).get_normal();
            let (normal, tilt, path) =
                against_the_other_side_face(positions, indices, vertex, next, their_next)
                    .map(|(normal, tilt)| (normal, tilt, SeamPlanePath::OtherSideFace))
                    .unwrap_or_else(|| {
                        let round_next = if convex { next } else { their_next };
                        seam_plane(
                            positions,
                            indices,
                            &picked,
                            is_feature_edge,
                            vertex,
                            next,
                            their_next,
                            round_next,
                        )
                    });
            // Faced on past this run's end: away from its own run.
            let faced = if normal.dot(mine_out) < 0.0 {
                normal
            } else {
                -normal
            };
            let bound = Some((
                EndRayBound {
                    reach_per_setback: 0.0,
                    plane_normal: faced,
                    stopped: true,
                    plane_tilt: tilt,
                    ..EndRayBound::default()
                },
                path,
            ));
            if at == 0 {
                clips[k].0 = bound;
            } else {
                clips[k].1 = bound;
            }
        }
    }

    clips
}

/// The plane a run ending at corner `vertex` (its next vertex `mine_next`) stops flush on: the
/// plane of the OTHER run's side face there, the face this run's band runs into at the corner, as
/// a square crease end stops on the face it meets. That face is the triangle on the other run's
/// end edge (towards `their_next`) with no coplanar triangle on this run's end edge (the two runs
/// share their other face). As (unit normal, tilt), the tilt read off the face's stored vertices as
/// a stopped end's is (`stopped_end_clip::stop_at`); `None` where there is no such face or it does
/// not part the two runs.
pub fn against_the_other_side_face(
    positions: &[Vector3],
    indices: &[Corners],
    vertex: i32,
    mine_next: i32,
    their_next: i32,
) -> Option<(Vector3, f64)> {
    let v = positions[vertex as usize];
    let on_edge = |p: i32, q: i32| -> Vec<Corners> {
        indices
            .iter()
            .copied()
            .filter(|&(x, y, z)| [x, y, z].contains(&p) && [x, y, z].contains(&q))
            .collect()
    };
    let mine: Vec<Vector3> = on_edge(vertex, mine_next)
        .iter()
        .map(|&t| face_normal(positions, t))
        .collect();
    let face = on_edge(vertex, their_next).into_iter().find(|&t| {
        let n = face_normal(positions, t);
        !mine.iter().any(|m| m.dot(n) >= 1.0 - COPLANAR_TOLERANCE)
    })?;
    let normal = face_normal(positions, face);
    let others: Vec<i32> = [face.0, face.1, face.2]
        .into_iter()
        .filter(|&w| w != vertex)
        .collect();
    let step = others
        .iter()
        .fold(float_grid::step_at_point(v), |step, &o| {
            step.max(float_grid::step_at_point(positions[o as usize]))
        });
    let lean = |far: i32| 3.0_f64.sqrt() * step / (positions[far as usize] - v).length();
    let spread = (positions[others[0] as usize] - v)
        .get_normal()
        .cross((positions[others[1] as usize] - v).get_normal())
        .length();
    let tilt = (lean(others[0]) + lean(others[1])) / spread;
    let on = |w: i32| ChainField::on_a_stopped_plane(v, normal, tilt, positions[w as usize]);
    // Faced on past this run's end, away from its own run.
    let mine = normal.dot(positions[mine_next as usize] - v);
    let faced = if mine < 0.0 { normal } else { -normal };
    // The plane parts the runs: this run's next vertex stands off it on the band's side, and the
    // other run's does not stand on the band's side (it runs along its own face, in the plane).
    let parts =
        !on(mine_next) && (on(their_next) || faced.dot(positions[their_next as usize] - v) > 0.0);
    // And it does not cut into the other run's side face past the corner: on a faceted curved
    // face the next facets about the corner all stand on one side of this facet's plane, or in it.
    let fan: Vec<i32> = indices
        .iter()
        .filter(|&&(x, y, z)| {
            [x, y, z].contains(&vertex)
                && face_normal(positions, (x, y, z)).dot(normal)
                    >= SEPARATE_END_FACE_DEGREES.to_radians().cos()
        })
        .flat_map(|&(x, y, z)| [x, y, z])
        .filter(|&w| w != vertex && !on(w))
        .collect();
    let sides = |above: bool| {
        fan.iter()
            .any(|&w| (normal.dot(positions[w as usize] - v) > 0.0) == above)
    };
    let cuts_into_the_face = sides(true) && sides(false);
    (parts && !cuts_into_the_face).then_some((normal, tilt))
}

/// The seam plane at corner `vertex`, between the runs leaving it towards `mine_next` and
/// `their_next`, as (unit normal, tilt). It runs along edges the mesh already has: the one unpicked
/// crease running into the corner, and the edge leaving the corner across the ROUND run's side
/// face most nearly opposite that crease (the face that carries on past the corner below the
/// cove), so the band's boundary there follows existing edges and leaves no sliver beside a facet
/// seam. Where that plane does not part the two runs (the crease and the edge nearly in line, or
/// the plane holding a run, as where the round's side face itself holds the crease), it is turned
/// to hold the crease and the runs' bisector; with no single crease, it is the mitre between the
/// runs. The tilt is how far the stored plane may lean off the true one from the float storage of
/// the vertices it is read from, as a stopped end's (`stopped_end_clip::stop_at`).
#[allow(clippy::too_many_arguments)]
fn seam_plane(
    positions: &[Vector3],
    indices: &[Corners],
    picked: &HashSet<(i32, i32)>,
    is_feature_edge: &dyn Fn(i32, i32) -> bool,
    vertex: i32,
    mine_next: i32,
    their_next: i32,
    round_next: i32,
) -> (Vector3, f64, SeamPlanePath) {
    let v = positions[vertex as usize];
    let a = (positions[mine_next as usize] - v).get_normal();
    let b = (positions[their_next as usize] - v).get_normal();
    let lean = |far: i32, step: f64| 3.0_f64.sqrt() * step / (positions[far as usize] - v).length();
    let step_of = |others: &[i32]| {
        others
            .iter()
            .fold(float_grid::step_at_point(v), |step, &o| {
                step.max(float_grid::step_at_point(positions[o as usize]))
            })
    };
    // A plane parts the runs where each run's next vertex stands off it on its own side, read by
    // the one rule for standing on a stopped end's plane (`ChainField::on_a_stopped_plane`).
    let parts = |n: Vector3, tilt: f64| {
        let n = n.get_normal();
        let off = |w: i32| !ChainField::on_a_stopped_plane(v, n, tilt, positions[w as usize]);
        let (da, db) = (n.dot(a), n.dot(b));
        off(mine_next) && off(their_next) && (da > 0.0) != (db > 0.0)
    };

    let mitre = || {
        let between = a - b;
        let step = step_of(&[mine_next, their_next]);
        (
            between.get_normal(),
            (lean(mine_next, step) + lean(their_next, step)) / between.length(),
        )
    };

    let mut creases: Vec<i32> = Vec::new();
    for &(x, y, z) in indices {
        for (u, w) in [(x, y), (y, z), (z, x)] {
            let other = if u == vertex {
                w
            } else if w == vertex {
                u
            } else {
                continue;
            };
            if !picked.contains(&(vertex.min(other), vertex.max(other)))
                && is_feature_edge(vertex, other)
                && !creases.contains(&other)
            {
                creases.push(other);
            }
        }
    }
    if creases.len() != 1 {
        return (mitre().0, mitre().1, SeamPlanePath::Mitre);
    }

    let crease_far = creases[0];
    let crease = (positions[crease_far as usize] - v).get_normal();

    // The round's side face at the corner: the triangle on its end edge the cove's end edge has no
    // coplanar triangle beside (the two runs share the other face).
    let on_edge = |p: i32, q: i32| -> Vec<Corners> {
        indices
            .iter()
            .copied()
            .filter(|&(x, y, z)| [x, y, z].contains(&p) && [x, y, z].contains(&q))
            .collect()
    };
    let cove_next = if round_next == mine_next {
        their_next
    } else {
        mine_next
    };
    let cove_normals: Vec<Vector3> = on_edge(vertex, cove_next)
        .iter()
        .map(|&t| face_normal(positions, t))
        .collect();
    let side = on_edge(vertex, round_next).into_iter().find(|&t| {
        let n = face_normal(positions, t);
        !cove_normals
            .iter()
            .any(|c| c.dot(n) >= 1.0 - COPLANAR_TOLERANCE)
    });

    let mut leaving: Option<(f64, i32)> = None;
    if let Some(side) = side {
        let side_normal = face_normal(positions, side);
        for &(x, y, z) in indices {
            if ![x, y, z].contains(&vertex)
                || face_normal(positions, (x, y, z)).dot(side_normal) < 1.0 - COPLANAR_TOLERANCE
            {
                continue;
            }
            for w in [x, y, z] {
                if w == vertex || w == round_next || w == crease_far {
                    continue;
                }
                let along = (positions[w as usize] - v).get_normal().dot(crease);
                // The most nearly opposite the crease; a tie goes to the lower index.
                let better = match leaving {
                    None => true,
                    Some((best, at)) => along < best || (along == best && w < at),
                };
                if better {
                    leaving = Some((along, w));
                }
            }
        }
    }

    if let Some((_, edge_far)) = leaving {
        let edge = (positions[edge_far as usize] - v).get_normal();
        let normal = crease.cross(edge);
        if normal.length_squared() > 0.0 {
            let step = step_of(&[crease_far, edge_far]);
            let tilt = (lean(crease_far, step) + lean(edge_far, step)) / normal.length();
            if parts(normal, tilt) {
                return (normal.get_normal(), tilt, SeamPlanePath::CreaseAndEdge);
            }
        }
    }

    let between = a + b;
    let normal = crease.cross(between.get_normal());
    if between.length_squared() > 0.0 && normal.length_squared() > 0.0 {
        let step = step_of(&[crease_far, mine_next, their_next]);
        let tilt = (lean(crease_far, step) + lean(mine_next, step) + lean(their_next, step))
            / normal.length();
        if parts(normal, tilt) {
            return (normal.get_normal(), tilt, SeamPlanePath::CreaseAndBisector);
        }
    }

    let (normal, tilt) = mitre();
    (normal, tilt, SeamPlanePath::Mitre)
}
