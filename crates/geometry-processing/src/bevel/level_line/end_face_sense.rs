//! The SENSE of an open chain's end face against the chain: whether the creases where the end face
//! meets the chain's two sides fold the same way as the chain itself (a cube's edge onto its end
//! face: convex-convex; a pocket's floor edge onto the end walls: concave-concave) or the opposite
//! way (an L block's inner, concave, edge onto its L-shaped top, which meets the inner walls at
//! CONVEX creases). Its readings over plain positions and triangles - the fold test
//! ([`is_convex`]), the crease sense
//! ([`is_opposite_sense_crease`]), which ends take an end ray ([`oblique_ends`]), the rays' bounds
//! ([`end_ray_bounds`]), the convex ends' planes ([`convex_end_faces`]) and whether an end face is
//! the opposite sense on a tagged triangle list ([`is_opposite_sense`]) - and the crease sense over
//! a tagged `LevelLineMesh`.
//!
//! The sense is read exactly as `ForeignEdgeRoom.IsAKnife` reads a fold: an edge is convex where
//! the far triangle's opposite corner lies behind the near triangle's plane. An OPPOSITE-SENSE end
//! face is not trimmed by the bead but GROWS over its section, so it is not an end region of the
//! band: it is not refined, rooted or cut, and it does not bound the room.

use vector_math::{ieee_math, Vector3};

use super::corner_cap_flat_ends::use_flat_ends;
use super::end_face_sense_oblique::ObliqueEndReader;
use super::triangle_edges::{self, StandingMap};
use super::{
    chain_on_the_mesh, end_face_crease_plane, end_plane_trim, BandProfile,
    ChainEdgeNotOnTwoTriangles, EndRayBound,
};

/// One triangle by its three corner indices into the positions.
pub type Corners = (i32, i32, i32);

/// Relative 1e-6 - the catalogue's faces are flat only to
/// float noise.
pub const COPLANAR_TOLERANCE: f64 = 1e-6;

/// The fold, off a side, past which a face at the
/// chain's end is a SEPARATE end face that takes the end ray. Under it the face is the side
/// carrying on and the end fades. 30 degrees reads a faceted curve of 12 or more facets per circle
/// as one wall (a 24-facet boss's next facet folds 15 degrees off, and read at a tighter bend
/// threshold it would otherwise take a ray), while real end faces fold far sharper (the sheared
/// prism's fold 69.3 and 110.7, Fillet Thin Front's 66.9, Center Flows Through's 66.9 and 90).
pub const SEPARATE_END_FACE_DEGREES: f64 = 30.0;

/// Whether two unit directions whose dot product is `cosine` - two faces' normals, or a crease and
/// the next leg leaving it - stand within [`SEPARATE_END_FACE_DEGREES`] of each other: the faceted
/// curve CARRYING ON. The one reading of that limit for every caller (the oblique end, the stopped
/// end, the square crease's reach and a growing neighbour's next facet). Read on the cosine with a
/// slack of [`COPLANAR_TOLERANCE`], so a turn of exactly 30 degrees, a 12-gon's, carries on
/// whichever way its coordinates round; NaN, a broken direction, never carries on.
pub fn carries_on(cosine: f64) -> bool {
    cosine >= SEPARATE_END_FACE_DEGREES.to_radians().cos() - COPLANAR_TOLERANCE
}

/// The lean off square past which a wall facing BACK
/// over a chain it meets the opposite way takes the end ray. Under it the wall is read as square
/// and grows (flow rung 1): Top & Round's rim end stands 4.5 degrees off square and builds only
/// that way, while Fillet Thin Front's walls lean 45 degrees. The number is measured between
/// cases, not derived: it sits well above 4.5 (Top & Round), below the leaning L's 30 (which the
/// grow path already builds, so either reading serves it) and far below 45 (Fillet Thin Front,
/// which needs the ray).
pub const BACK_FACING_WALL_LEAN_DEGREES: f64 = 15.0;

fn at(positions: &[Vector3], i: i32) -> Vector3 {
    positions[i as usize]
}

fn has(t: Corners, v: i32) -> bool {
    t.0 == v || t.1 == v || t.2 == v
}

/// The distinct corners of `a`
/// that are also corners of `b`, in `a`'s order.
pub fn shared_corners(a: Corners, b: Corners) -> Vec<i32> {
    let mut shared = Vec::with_capacity(3);
    for c in [a.0, a.1, a.2] {
        if has(b, c) && !shared.contains(&c) {
            shared.push(c);
        }
    }

    shared
}

/// The triangle's unit normal by its winding.
pub fn face_normal(positions: &[Vector3], t: Corners) -> Vector3 {
    (at(positions, t.1) - at(positions, t.0))
        .cross(at(positions, t.2) - at(positions, t.0))
        .get_normal()
}

/// Whether the fold from `near` to `far` across their shared edge (one
/// end of which is `on_edge`) is convex: the far triangle's corners lie behind the near plane.
pub fn is_convex(positions: &[Vector3], near: Corners, far: Corners, on_edge: i32) -> bool {
    let a = at(positions, near.0);
    let normal = (at(positions, near.1) - a).cross(at(positions, near.2) - a);
    let origin = at(positions, on_edge);

    // The corner of the far triangle that is not on the shared edge is the one farthest off the
    // near plane; summing all three is the same sign, since the two shared corners lie on the
    // plane.
    let off = (at(positions, far.0) - origin).dot(normal)
        + (at(positions, far.1) - origin).dot(normal)
        + (at(positions, far.2) - origin).dot(normal);
    off < 0.0
}

/// Whether a side's bounding
/// crease folds the OPPOSITE way to the chain - the neighbour across it GROWS under the blend
/// rather than being trimmed (flow rung 2: a rib standing on a convex edge's side meets it at
/// concave creases). `chain_near` and `chain_far` are the two sides' triangles on a chain edge
/// through `chain_vertex`; `side` is the picked side's triangle on the crease through
/// `crease_vertex` and `far` the untagged one across it.
///
/// A knife is never opposite sense here, whatever the chain: the knife rule (zero-width flow, see
/// `ForeignEdgeRoom.IsAKnife`) owns that crease and stays exactly as it is.
pub fn is_opposite_sense_crease(
    positions: &[Vector3],
    chain_near: Corners,
    chain_far: Corners,
    chain_vertex: i32,
    side: Corners,
    far: Corners,
    crease_vertex: i32,
) -> bool {
    let crease_is_convex = is_convex(positions, side, far, crease_vertex);
    if crease_is_convex && face_normal(positions, side).dot(face_normal(positions, far)) < 0.0 {
        return false;
    }

    crease_is_convex != is_convex(positions, chain_near, chain_far, chain_vertex)
}

/// Facing is the wall normal . the chain's outward direction,
/// below zero here; square is -1.
pub fn leans_off_square(facing: f64) -> bool {
    (-facing).clamp(-1.0, 1.0).acos() * 180.0 / std::f64::consts::PI > BACK_FACING_WALL_LEAN_DEGREES
}

/// Whether `normal` lies in the plane of one of `sides`: its dot with a side's normal is within
/// the coplanar tolerance of 1.
fn in_a_side_plane(sides: &[Vector3], normal: Vector3) -> bool {
    sides
        .iter()
        .any(|s| s.dot(normal) >= 1.0 - COPLANAR_TOLERANCE)
}

/// Whether either side's own face carries on past the
/// end plane along its rail: the rail's direction from the end vertex V (`rails` at unit setback),
/// carried along the chain just past where it meets the end face's plane, still lies on a triangle
/// in that side's plane. Near V the faces are planar, so the test point is taken a thousandth of
/// V's shortest edge out - it reads the faces' shape at V, not the rail at any particular size.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn side_runs_past_the_end_plane(
    positions: &[Vector3],
    indices: &[Corners],
    end: i32,
    outward: Vector3,
    end_face: Vector3,
    rails: &[Vector3],
    side_normals: &[Vector3],
) -> bool {
    let v = at(positions, end);
    let n = if end_face.dot(outward) > 0.0 {
        end_face
    } else {
        -end_face
    };
    let slant = n.dot(outward);
    if !(slant > 0.0) {
        return false;
    }

    let mut shortest = f64::MAX;
    for &t in indices {
        if has(t, end) {
            for c in [t.0, t.1, t.2] {
                if c != end {
                    shortest = ieee_math::min_f64(shortest, (at(positions, c) - v).length());
                }
            }
        }
    }

    let inside = |a: Vector3, b: Vector3, c: Vector3, normal: Vector3, p: Vector3| {
        (b - a).cross(p - a).dot(normal) >= 0.0
            && (c - b).cross(p - b).dot(normal) >= 0.0
            && (a - c).cross(p - c).dot(normal) >= 0.0
    };

    let scale = 1e-3 * shortest;
    for s in 0..rails.len() {
        let away = rails[s] - v;
        // where the rail meets the end plane, then as far again past it along the chain
        let to_the_plane = -n.dot(away) / slant;
        let past = v + ((away + (outward * (ieee_math::max_f64(0.0, to_the_plane) + 1.0))) * scale);
        let normal = side_normals[s];
        for &t in indices {
            let a = at(positions, t.0);
            let face = face_normal(positions, t);
            if face.dot(normal) < 1.0 - COPLANAR_TOLERANCE
                || (past - a).dot(normal).abs() > 1e-3 * scale
            {
                continue;
            }

            if inside(a, at(positions, t.1), at(positions, t.2), face, past) {
                return true;
            }
        }
    }

    false
}

/// The bound on each rayed end's ray (see [`EndRayBound`]): how far
/// along the chain the round's row at the end vertex must be carried for every point of it to meet
/// the end face's plane, per unit of setback. `None` at an end with no ray, or whose sides or end
/// face cannot be read.
pub fn end_ray_bounds(
    positions: &[Vector3],
    indices: &[Corners],
    chain: &[i32],
    rays: (bool, bool),
) -> (Option<EndRayBound>, Option<EndRayBound>) {
    let first = if rays.0 {
        end_ray_bound_at(positions, indices, chain[0], chain[1])
    } else {
        None
    };
    let last = if rays.1 {
        end_ray_bound_at(
            positions,
            indices,
            chain[chain.len() - 1],
            chain[chain.len() - 2],
        )
    } else {
        None
    };
    (first, last)
}

/// The bound on one end's ray, for [`end_ray_bounds`]; `None` where the end cannot be read.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn end_ray_bound_at(
    positions: &[Vector3],
    indices: &[Corners],
    end: i32,
    next: i32,
) -> Option<EndRayBound> {
    let v = at(positions, end);
    let outward = (v - at(positions, next)).get_normal();
    let mut rails = Vec::new();
    let mut side_normals = Vec::new();
    let mut side_triangles = Vec::new();
    let mut end_triangles = Vec::new();
    let mut end_face: Option<Vector3> = None;
    let mut end_face_area = 0.0;
    for &t in indices {
        if !has(t, end) {
            continue;
        }

        let normal = face_normal(positions, t);
        if has(t, next) {
            // The side's direction away from the chain, square to it: where its rail runs at unit
            // setback. A triangle with no third corner has no side to read, so the end cannot
            // be read.
            let third = [t.0, t.1, t.2]
                .into_iter()
                .find(|&c| c != end && c != next)?;
            let mut away = at(positions, third) - v;
            away = away - (outward * away.dot(outward));
            rails.push(v + away.get_normal());
            side_normals.push(normal);
            side_triangles.push(t);
        }
    }

    // The end face: the LARGEST triangle at the end that is not in a side's plane. ObliqueEnds has
    // found them all one plane, but only to the input's float noise; the first in the index order
    // made a mirror pair's two ends read different triangles of one face, so one end hole stood
    // 2.5e-8 off one plane and went to the faired filler while the other closed flat
    // (HoleFillOneLoopCornerTests.TheMirroredHolesAreMirrored). The largest does not depend on the
    // order; it only orients, and stands in for, the plane through the two creases
    // (EndFaceCreasePlane), which is what the rails meet.
    for &t in indices {
        if !has(t, end) || has(t, next) {
            continue;
        }

        let normal = face_normal(positions, t);
        if !in_a_side_plane(&side_normals, normal) {
            let area = (at(positions, t.1) - at(positions, t.0))
                .cross(at(positions, t.2) - at(positions, t.0))
                .length();
            if area > end_face_area {
                end_face = Some(normal);
                end_face_area = area;
            }

            end_triangles.push(t);
        } else {
            // A side's own triangle at the end (a fanned side has several): its crease with the
            // end face may be on it rather than on the chain edge's triangle.
            side_triangles.push(t);
        }
    }

    let fallback = end_face?;
    if rails.len() != 2 {
        return None;
    }

    let end_face =
        end_face_crease_plane::through(positions, end, &side_triangles, &end_triangles, fallback);

    // Bounded only where the ray is a CONCAVE chain's onto an end face that leans back over it
    // (see ObliqueEnds). Every other ray keeps its old reach, which the sheared prism's room and
    // far-end rails are read through. With flat ends a concave chain's ray onto an end face facing
    // ON along it is bounded too, but only where a SIDE'S OWN FACE carries on past the end plane
    // along its rail (SideRunsPastTheEndPlane): the tent tip's floor runs on in front of the
    // tent's other wall, so an unbounded ray lays a spur of fillet over it. Where each side stops
    // at the end face the rails meet only the end face past the plane, and that face GROWS over
    // the bead, so the ray keeps its reach. Convex rays keep their reach.
    if is_convex(positions, side_triangles[0], side_triangles[1], end) {
        return None;
    }

    if end_face.dot(outward) >= 0.0
        && !(use_flat_ends()
            && side_runs_past_the_end_plane(
                positions,
                indices,
                end,
                outward,
                end_face,
                &rails,
                &side_normals,
            ))
    {
        return None;
    }

    let n = if end_face.dot(outward) > 0.0 {
        end_face
    } else {
        -end_face
    };
    let slant = n.dot(outward);
    if !(slant > 0.0) {
        return None;
    }

    // Either way out of the plane: a row point behind it is carried on to it, one before it was met.
    let farthest = ieee_math::max_f64(
        end_plane_trim::farthest_out(rails[0], rails[1], v, n, BandProfile::Fillet),
        end_plane_trim::farthest_out(rails[0], rails[1], v, -n, BandProfile::Fillet),
    );
    // SAME sense where every side-to-end crease through the end folds the chain's way (concave):
    // the end face is trimmed to the round's section (EndPlaneTrim). Opposite sense, it grows and
    // clips instead.
    let mut creases = 0;
    let mut same_sense = true;
    for &side in &side_triangles {
        for &other in &end_triangles {
            let shared = shared_corners(side, other);
            if shared.len() == 2 && shared.contains(&end) {
                creases += 1;
                same_sense &= !is_convex(positions, side, other, end);
            }
        }
    }

    same_sense &= creases > 0;

    Some(EndRayBound {
        reach_per_setback: ieee_math::max_f64(0.0, farthest) / slant,
        plane_normal: n,
        same_sense,
        ..EndRayBound::default()
    })
}

/// A plane as a point on it and its unit normal.
pub type PointPlane = (Vector3, Vector3);

/// The end face's plane at each CONVEX rayed end - the end vertex
/// and the face's normal faced on along the chain - for the room reading alone
/// (`BandRoom.RoomOfCutMesh`): a convex ray keeps its full reach ([`end_ray_bounds`] bounds concave
/// rays only), but the convex band is clipped at this plane, so a foreign edge wholly beyond it
/// can never meet the band through the ray. `None` at an end with no ray, a concave end, or one
/// whose sides or end face cannot be read.
pub fn convex_end_faces(
    positions: &[Vector3],
    indices: &[Corners],
    chain: &[i32],
    rays: (bool, bool),
) -> (Option<PointPlane>, Option<PointPlane>) {
    let face_at = |end: i32, next: i32| -> Option<PointPlane> {
        let v = at(positions, end);
        let outward = (v - at(positions, next)).get_normal();
        let mut side_triangles = Vec::new();
        let mut others = Vec::new();
        for &t in indices {
            if !has(t, end) {
                continue;
            }

            if has(t, next) {
                side_triangles.push(t);
            } else {
                others.push(t);
            }
        }

        if side_triangles.len() != 2
            || !is_convex(positions, side_triangles[0], side_triangles[1], end)
        {
            return None;
        }

        let side_normals: Vec<Vector3> = side_triangles
            .iter()
            .map(|&t| face_normal(positions, t))
            .collect();
        for t in others {
            let normal = face_normal(positions, t);
            if in_a_side_plane(&side_normals, normal) {
                continue;
            }

            let slant = normal.dot(outward);
            if slant.abs() <= COPLANAR_TOLERANCE {
                return None;
            }

            return Some((v, if slant > 0.0 { normal } else { -normal }));
        }

        None
    };

    let first = if rays.0 {
        face_at(chain[0], chain[1])
    } else {
        None
    };
    let last = if rays.1 {
        face_at(chain[chain.len() - 1], chain[chain.len() - 2])
    } else {
        None
    };
    (first, last)
}

/// Which ends of an open chain end on ONE flat end face that is
/// OBLIQUE to the chain - the ends whose field carries an end ray (see `ChainField.EndRay`). An end
/// square to the chain, an end with no end face (a crease dying out) or with several faces past the
/// two sides reads false, so its field is the plain one, bit for bit.
pub fn oblique_ends(
    positions: &[Vector3],
    indices: &[Corners],
    chain: &[i32],
    closed: bool,
    sharp_turn_degrees: f64,
) -> (bool, bool) {
    if closed || chain.len() < 2 {
        return (false, false);
    }

    let reader = ObliqueEndReader {
        positions,
        indices,
        sharp_turn_degrees,
    };
    (
        reader.is_oblique_end(chain[0], chain[1]),
        reader.is_oblique_end(chain[chain.len() - 1], chain[chain.len() - 2]),
    )
}

/// The working-mesh form - the
/// chain edge's two triangles, and the crease's filled triangle as its side and the other as its
/// far face. `false` where either edge is not two triangles or the crease is not between a filled
/// and an unfilled one.
pub fn is_opposite_sense_crease_on(
    mesh: &super::LevelLineMesh,
    chain_edge: (i32, i32),
    edge: (i32, i32),
) -> bool {
    let on_chain = mesh.triangles_on(chain_edge.0, chain_edge.1);
    let on_crease = mesh.triangles_on(edge.0, edge.1);
    if on_chain.len() != 2 || on_crease.len() != 2 {
        return false;
    }

    let first = mesh.triangles[on_crease[0] as usize];
    let second = mesh.triangles[on_crease[1] as usize];
    if (first.tag == 0) == (second.tag == 0) {
        return false;
    }

    let corners = |t: super::LevelLineTriangle| (t.a, t.b, t.c);
    let (side, far) = if first.tag != 0 {
        (first, second)
    } else {
        (second, first)
    };

    is_opposite_sense_crease(
        &mesh.vertices,
        corners(mesh.triangles[on_chain[0] as usize]),
        corners(mesh.triangles[on_chain[1] as usize]),
        chain_edge.0,
        corners(side),
        corners(far),
        edge.0,
    )
}

/// True when EVERY crease incident to `end` between a picked side
/// (tag 1 or 2) and an untagged triangle folds the opposite way to `chain_edge`, and there is at
/// least one. Since flow rung 3 an oblique end face is taken too: its end carries the end ray, so
/// the rows run on square to the chain until the end face's plane clips them. The chain edge not
/// standing on two triangles is an error.
pub fn is_opposite_sense(
    positions: &[Vector3],
    indices: &[Corners],
    standing: &StandingMap,
    tags: &[i32],
    chain_edge: (i32, i32),
    end: i32,
) -> Result<bool, ChainEdgeNotOnTwoTriangles> {
    let (left, right) = chain_on_the_mesh::sides_of(indices, standing, chain_edge.0, chain_edge.1)?;
    let chain_is_convex = is_convex(
        positions,
        indices[left as usize],
        indices[right as usize],
        chain_edge.0,
    );
    let mut creases = 0;
    for (i, &t) in indices.iter().enumerate() {
        if tags[i] != 0 || !has(t, end) {
            continue;
        }

        for (a, b) in triangle_edges::corners(t) {
            if a != end && b != end {
                continue;
            }

            let Some(across) = standing.get(&triangle_edges::key(a, b)) else {
                continue;
            };

            for &j in across {
                if j as usize == i || (tags[j as usize] != 1 && tags[j as usize] != 2) {
                    continue;
                }

                creases += 1;
                if is_convex(positions, indices[j as usize], t, a) == chain_is_convex {
                    return Ok(false);
                }
            }
        }
    }

    Ok(creases > 0)
}

/// At each growing end of the chain (`first_grows`,
/// `last_grows`), the side-to-end-face creases running from it along the end face's one plane -
/// the stretch a growing crease runs along. A crease is an edge between one of the chain's own
/// sides and an end face, with a real fold between them; `own_side` says which tags are the
/// chain's own sides (`None` reads the one-chain tagging: sides 1 and 2, the end face untagged).
pub fn growing_crease_stretch(
    mesh: &super::LevelLineMesh,
    chain_edges: &[(i32, i32)],
    first_grows: bool,
    last_grows: bool,
    own_side: Option<&dyn Fn(i32) -> bool>,
) -> std::collections::HashSet<(i32, i32)> {
    use std::collections::{HashMap, HashSet};

    let mut stretch = HashSet::new();
    if !first_grows && !last_grows {
        return stretch;
    }

    let normal_of = |triangle: i32| {
        let t = mesh.triangles[triangle as usize];
        let a = mesh.position(t.a);
        (mesh.position(t.b) - a)
            .cross(mesh.position(t.c) - a)
            .get_normal()
    };

    // Every side-to-end CREASE: one picked side, one end-face triangle, and a real fold.
    let mut end_face_normal: HashMap<(i32, i32), Vector3> = HashMap::new();
    let mut creases_at: HashMap<i32, Vec<(i32, i32)>> = HashMap::new();
    for edge in mesh.edges() {
        let standing = mesh.triangles_on(edge.0, edge.1);
        if standing.len() != 2 {
            continue;
        }

        let first = mesh.triangles[standing[0] as usize].tag;
        let second = mesh.triangles[standing[1] as usize].tag;
        let first_is_the_end_face = match own_side {
            None => {
                if (first == 0) == (second == 0)
                    || (first != 0 && first > 2)
                    || (second != 0 && second > 2)
                {
                    continue;
                }
                first == 0
            }
            Some(own_side) => {
                // Several chains: the end face may be untagged or ANOTHER chain's side.
                let first_own = first != 0 && own_side(first);
                let second_own = second != 0 && own_side(second);
                if first_own == second_own {
                    continue;
                }
                !first_own
            }
        };

        let (end_t, side_t) = if first_is_the_end_face {
            (standing[0], standing[1])
        } else {
            (standing[1], standing[0])
        };
        let end = normal_of(end_t);
        if 1.0 - end.dot(normal_of(side_t)) <= COPLANAR_TOLERANCE {
            continue;
        }

        end_face_normal.insert(edge, end);
        for v in [edge.0, edge.1] {
            creases_at.entry(v).or_default().push(edge);
        }
    }

    let mut ends = Vec::new();
    if first_grows {
        ends.push(chain_edges[0].0);
    }
    if last_grows {
        ends.push(chain_edges[chain_edges.len() - 1].1);
    }

    for end_vertex in ends {
        let Some(at_end) = creases_at.get(&end_vertex) else {
            continue;
        };

        for &start in at_end {
            let plane = end_face_normal[&start];
            let mut walking = vec![start];
            while let Some(edge) = walking.pop() {
                if 1.0 - end_face_normal[&edge].dot(plane) > COPLANAR_TOLERANCE
                    || !stretch.insert(edge)
                {
                    continue;
                }

                for v in [edge.0, edge.1] {
                    walking.extend(creases_at[&v].iter().copied());
                }
            }
        }
    }

    stretch
}
