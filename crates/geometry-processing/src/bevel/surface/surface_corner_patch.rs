//! The corner where THREE ROUND strips meet at one vertex of the part:
//! each one ends on the plane through the
//! ball centre square to its own edge, and the spherical triangle between the three end rings is
//! stitched in as the patch - the rung 3 reading of `BevelCornerPatches`, built as surface and
//! shared by vertex rather than built as a solid and subtracted. Here are its nested record, one
//! strip arriving at the corner ([`SurfaceCornerPatchArrival`]), the ball's centre
//! ([`try_find_the_ball_centre`]), the cut of one strip's end at the ball's plane
//! ([`try_cut_the_end`]) and the ordering of the three end rings round the patch
//! ([`try_order_the_sides`]), and the close itself ([`try_close`]).

use std::rc::Rc;

use solidmesh_math::Vector3;

use super::{surface_corner_mitre, surface_corner_patch_rows, SurfaceStripStation};
use super::{SurfaceEdgePlan, SurfaceStrip, SurfaceVertexPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};
use crate::topology::EdgeConvexity;

/// One strip arriving at the corner - which
/// strip, what it was planned as, and the two vertices the walkback left where its own two tangent
/// lines ended.
#[derive(Debug, Clone)]
pub struct SurfaceCornerPatchArrival {
    /// The strip arriving.
    pub strip: Rc<SurfaceStrip>,
    /// The edge plan the strip was built from, held by value.
    pub edge: SurfaceEdgePlan,
    /// The last vertex of the left face's tangent run - rail 0 of the end ring.
    pub on_the_left: i32,
    /// The same for the right face, which is the ring's last rail.
    pub on_the_right: i32,
}

/// One strip cut at the plane through the ball centre square to its own edge,
/// as the ring of vertices across it and where along the edge each rail meets that plane; `Err` is
/// why it would not cut. The ring's two outer rails have to land on the
/// vertices the walkback left at the ends of that strip's tangent runs - the ball's own touch points
/// on the two faces - or the patch would not be rimmed by the faces it stands between. The inner
/// rails' points are added to `pool`.
pub fn try_cut_the_end(
    pool: &mut SurfaceVertexPool,
    arrival: &SurfaceCornerPatchArrival,
    centre: Vector3,
    corner: Vector3,
) -> Result<(Vec<i32>, Vec<f64>), BevelAdvisory> {
    let rail_count = arrival.strip.rail_count();
    let mut ring = vec![0i32; rail_count as usize];
    let mut meets_at = vec![0.0; rail_count as usize];

    for rail in 0..rail_count {
        let Some(param) = arrival.strip.try_cross(rail, centre, arrival.strip.along()) else {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCornerNotSupported,
                vec![corner.into()],
            ));
        };

        meets_at[rail as usize] = param;

        let at = arrival.strip.point_on(rail, param);

        if rail == 0 || rail == rail_count - 1 {
            let vertex = if rail == 0 {
                arrival.on_the_left
            } else {
                arrival.on_the_right
            };

            if (pool.position_of(vertex) - at).length() > pool.near_enough() {
                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
                    vec![at.into()],
                ));
            }

            ring[rail as usize] = vertex;
            continue;
        }

        ring[rail as usize] = pool.add(at);
    }

    Ok((ring, meets_at))
}

/// How near two face normals have to agree to be one face, and two setbacks to be one plane - a
/// dimensionless floor.
pub const FACE_NORMALS_AGREE: f64 = 1e-6;

impl SurfaceCornerPatchArrival {
    /// A strip arriving at the corner, with the edge plan it was built from.
    pub fn new(
        strip: Rc<SurfaceStrip>,
        edge: SurfaceEdgePlan,
        on_the_left: i32,
        on_the_right: i32,
    ) -> Self {
        SurfaceCornerPatchArrival {
            strip,
            edge,
            on_the_left,
            on_the_right,
        }
    }
}

/// The centre of the ball of radius `size` tangent to the three faces the `arrivals` end on: the
/// corner moved by the solution of `n . x = -size` over the three face normals (the minus because
/// the normals face out of the material and the centre is inside it). Refused where two edges end
/// on one face at different setbacks, where the arrivals name other than three faces, or where the
/// three do not cut a point out between them.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)` refuses a NaN as well.
pub fn try_find_the_ball_centre(
    arrivals: &[SurfaceCornerPatchArrival],
    corner: Vector3,
    size: f64,
) -> Result<Vector3, BevelAdvisory> {
    let not_supported = || {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![corner.into()],
        )
    };

    let mut normals: Vec<Vector3> = Vec::with_capacity(3);
    let mut offsets: Vec<f64> = Vec::with_capacity(3);

    for arrival in arrivals {
        for normal in [
            arrival.edge.edge.left_normal(),
            arrival.edge.edge.right_normal(),
        ] {
            let offset = size;

            let already = normals
                .iter()
                .position(|face| face.dot(normal) > 1.0 - FACE_NORMALS_AGREE);

            let Some(already) = already else {
                normals.push(normal.get_normal());
                offsets.push(offset);
                continue;
            };

            if (offsets[already] - offset).abs() > size * FACE_NORMALS_AGREE {
                // Two edges ending on one face, set back by different amounts: there is no single
                // plane both of them can end on and no one ball for the patch to be part of.
                return Err(not_supported());
            }
        }
    }

    if normals.len() != 3 {
        // Three edges over anything but three faces - two of them sharing both their faces, or a
        // corner the flood reads as fewer surfaces than it has.
        return Err(not_supported());
    }

    let determinant = normals[0].dot(normals[1].cross(normals[2]));

    if !(determinant.abs() > FACE_NORMALS_AGREE) {
        // Three faces that do not cut a point out between them - two of them parallel, or all
        // three through one line - so there is no corner here to put a ball in.
        return Err(not_supported());
    }

    // Cramer, written out: the inverse of a 3x3 whose rows are the normals, applied to the
    // offsets.
    let wanted = Vector3::new(-offsets[0], -offsets[1], -offsets[2]);

    let first = normals[1].cross(normals[2]);
    let second = normals[2].cross(normals[0]);
    let third = normals[0].cross(normals[1]);

    Ok(corner + (((first * wanted.x) + (second * wanted.y) + (third * wanted.z)) / determinant))
}

/// The three end `rings` joined end to start round the patch - each turned over where it meets
/// the last by its own far end - starting from the first; `None` where they do not close into
/// one loop.
pub fn try_order_the_sides(rings: &[Vec<i32>]) -> Option<Vec<Vec<i32>>> {
    let mut sides = Vec::with_capacity(rings.len());
    sides.push(rings[0].clone());

    let mut used = vec![false; rings.len()];

    used[0] = true;

    for _ in 1..rings.len() {
        let ending = *sides.last()?.last()?;
        let mut found = false;

        for next in 0..rings.len() {
            if found {
                break;
            }
            if used[next] {
                continue;
            }

            let ring = &rings[next];

            if ring[0] == ending {
                sides.push(ring.clone());
                used[next] = true;
                found = true;
            } else if ring[ring.len() - 1] == ending {
                sides.push(ring.iter().rev().copied().collect());
                used[next] = true;
                found = true;
            }
        }

        if !found {
            return None;
        }
    }

    if sides.last()?.last()? != &sides[0][0] {
        return None;
    }

    Some(sides)
}

/// The corner where three strips meet - a chamfer's closed by the mitre
/// ([`surface_corner_mitre::try_close`]); a round's each cut at the plane through the ball's
/// centre (mirrored through the corner where all three are concave), the three end rings ordered
/// round the patch and the spherical triangle stitched in. Returns each strip's station, or why
/// the corner is not one this patch closes.
pub fn try_close(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    arrivals: &[SurfaceCornerPatchArrival],
    corner: Vector3,
    size: f64,
    round_profile: bool,
) -> Result<Vec<SurfaceStripStation>, BevelAdvisory> {
    let not_supported = || {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![corner.into()],
        )
    };

    if arrivals.len() != 3 {
        return Err(not_supported());
    }

    if !round_profile {
        return surface_corner_mitre::try_close(pool, arrivals, corner);
    }

    let all_concave = arrivals
        .iter()
        .all(|a| a.edge.convexity() == EdgeConvexity::Concave);

    let mut centre = match try_find_the_ball_centre(arrivals, corner, size) {
        Ok(centre) => centre,
        Err(failure) => {
            if all_concave {
                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::SurfaceJunctionOfThreeCreases,
                    vec![corner.into()],
                ));
            }
            return Err(failure);
        }
    };

    if all_concave {
        centre = corner + (corner - centre);
    }

    let mut rings = Vec::with_capacity(arrivals.len());
    let mut meets_at = Vec::with_capacity(arrivals.len());
    for arrival in arrivals {
        let (ring, meets) = try_cut_the_end(pool, arrival, centre, corner)?;
        rings.push(ring);
        meets_at.push(meets);
    }

    let Some(sides) = try_order_the_sides(&rings) else {
        return Err(not_supported());
    };

    if !surface_corner_patch_rows::try_emit(pool, faces, &sides, centre, size, all_concave) {
        return Err(not_supported());
    }

    Ok(rings
        .into_iter()
        .zip(meets_at)
        .map(|(ring, meets)| SurfaceStripStation::clean(ring, meets))
        .collect())
}
