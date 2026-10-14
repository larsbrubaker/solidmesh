//! A CONVEX bite taken out of one flat face of the part, bounded by a ring of points lying in that
//! face rather than by a single tangent line. A module of free functions: what is here are the
//! readings that put the vertices standing on a piece's sides into it
//! ([`with_the_vertices_standing_on_it`], which reads the pieces across its sides off
//! [`SurfaceBiteSharedSides`]) and take the ones lying straight through back out
//! ([`without_the_straight_through`]), and the tests a bite is guarded by, and the bite itself
//! ([`try_take`]).
//!
//! The walkback's own cut is a half space through a straight tangent line, which is all a strip
//! along one edge ever needs (`SurfaceFacetCut`). A mixed corner's blend needs more: its footprint
//! on the cap face is bounded by the arc the rolling ball traces, and on the wall by the square the
//! bead's own stop plane leaves standing. Both are convex rings, so both are the intersection of the
//! half planes their own sides name.

use indexmap::IndexSet;

use vector_math::{cs_math, Vector3};

use super::{SurfaceBiteSharedSides, SurfaceVertexPool};

/// One piece with the vertices that STAND ON its sides put into
/// it - any vertex the faces this bite cuts already carry, lying within the ruler of one of the
/// piece's sides and strictly between that side's ends, becomes a corner of the ring at that place.
/// `shared` and `of_piece` are `None` and `-1` where there is no other piece; `carried` is an
/// [`IndexSet`] so it is walked in the order its vertices were added.
///
/// Which is the T junction this class would otherwise leave. A cut line is drawn where the bite's
/// own rim says, and nothing stops it running through a vertex the part already had: the face on
/// the far side of that line keeps the vertex as its own corner, while the piece on this side spans
/// it with one straight side, and the two then meet along an edge one of them has cut in two.
/// Measured on "Every Chamfer Works": the cap's own vertex at (19.5913, -0.5759, 20) stands 2e-5
/// off the sector's first side, and the six edges that leaves open are all of what the rim still
/// had open.
///
/// The candidates are the vertices of the faces this bite is IN THE WAY OF, and not the whole pool:
/// a vertex that stands on a side of a piece cut out of a face is a vertex of that face or of one
/// beside it in the same plane, and all of those are among the faces the bite is already walking.
///
/// Each side's finds are sorted stably, so finds
/// at one place keep `carried`'s order (two of them would be one weld, so the tie does not arise).
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn with_the_vertices_standing_on_it(
    pool: &SurfaceVertexPool,
    piece: Vec<i32>,
    carried: &IndexSet<i32>,
    shared: Option<&SurfaceBiteSharedSides>,
    of_piece: i32,
) -> Vec<i32> {
    if piece.len() < 2 || (carried.is_empty() && shared.is_none()) {
        return piece;
    }

    let mut grown = Vec::with_capacity(piece.len() + 2);

    // What the ring already carries, which is every corner of it and every vertex put in so far.
    // Two adjoining sides both carry a vertex standing near the corner they meet at - it is within
    // the ruler of each and strictly between the ends of each - and put in twice the ring walks out
    // to it and back along a side of no length. Only looked up.
    let mut already: std::collections::HashSet<i32> = piece.iter().copied().collect();

    for corner in 0..piece.len() {
        let from = pool.position_of(piece[corner]);
        let to = pool.position_of(piece[(corner + 1) % piece.len()]);
        let along = to - from;

        grown.push(piece[corner]);

        if !(along.length_squared() > 0.0) {
            continue;
        }

        // Where the piece across this side is one of this bite's own, WHICH of ITS corners this side
        // carries is read off the neighbour rather than measured - the only way the two of them are
        // sure to carry the same sequence (SurfaceBiteSharedSides). It only ever takes candidates
        // AWAY: what the neighbour has along the side that this piece has not is an ordinary T
        // junction, and the sewing is what puts those back.
        let read = shared.and_then(|shared| {
            shared.try_the_chain_across(
                pool,
                of_piece,
                piece[corner],
                piece[(corner + 1) % piece.len()],
            )
        });

        let mut standing: Vec<(f64, i32)> = Vec::new();

        for &vertex in carried {
            if already.contains(&vertex) {
                // Already in this ring, wherever it stands.
                continue;
            }

            if let Some((across, neighbour)) = &read {
                if named_by(neighbour, vertex) && !named_by(across, vertex) {
                    // The neighbour has said which of its OWN corners this side carries. One it did not
                    // name is a corner of some other side of the neighbour, standing within the ruler
                    // of this one because the two are one line to within that ruler - and put in here
                    // it claims an edge the neighbour has not got. A vertex of a face the bite never
                    // cut is named by nobody, and that is still a T junction this side has to take in.
                    continue;
                }
            }

            let at = pool.position_of(vertex);
            let how_far = (at - from).dot(along) / along.length_squared();

            if !(how_far > 0.0) || !(how_far < 1.0) {
                continue;
            }

            if (at - (from + (along * how_far))).length() > pool.near_enough() {
                continue;
            }

            standing.push((how_far, vertex));
        }

        // Every find is strictly inside (0, 1), so no NaN reaches the comparison.
        standing.sort_by(|first, second| first.0.total_cmp(&second.0));

        for (_, vertex) in standing {
            grown.push(vertex);
            already.insert(vertex);
        }
    }

    grown
}

/// Whether a piece names `vertex` as one of its own corners.
fn named_by(piece: &[i32], vertex: i32) -> bool {
    piece.contains(&vertex)
}

/// `piece` without the corners at or past `split_from` (the bite's own new vertices) that lie
/// straight through between their standing neighbours, dropped one at a time down to a triangle;
/// then any dropped corner the ring stopped carrying while the dropping went on is put back.
pub fn without_the_straight_through(
    pool: &SurfaceVertexPool,
    piece: &[i32],
    split_from: i32,
) -> Vec<i32> {
    let mut dropped = vec![false; piece.len()];
    let mut standing = piece.len();
    let mut going = true;

    while going && standing > 3 {
        going = false;

        for corner in 0..piece.len() {
            if dropped[corner] || piece[corner] < split_from {
                continue;
            }

            if !on_the_side_between(pool, piece, &dropped, corner) {
                continue;
            }

            dropped[corner] = true;
            standing -= 1;
            going = true;

            break;
        }
    }

    // And back in goes anything the ring stopped carrying while the dropping went on.
    let mut restoring = true;

    while restoring {
        restoring = false;

        for corner in 0..piece.len() {
            if !dropped[corner] || on_the_side_between(pool, piece, &dropped, corner) {
                continue;
            }

            dropped[corner] = false;
            standing += 1;
            restoring = true;
        }
    }

    let mut kept = Vec::with_capacity(standing);

    for (corner, &vertex) in piece.iter().enumerate() {
        if !dropped[corner] {
            kept.push(vertex);
        }
    }

    kept
}

/// Whether `corner` lies strictly between its two standing neighbours, within the pool's ruler of
/// the side they span.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` refuses a NaN as well.
pub fn on_the_side_between(
    pool: &SurfaceVertexPool,
    piece: &[i32],
    dropped: &[bool],
    corner: usize,
) -> bool {
    let (Some(before), Some(after)) = (
        standing(piece, dropped, corner, -1),
        standing(piece, dropped, corner, 1),
    ) else {
        return false;
    };
    if before == after {
        return false;
    }

    let from = pool.position_of(piece[before]);
    let run = pool.position_of(piece[after]) - from;

    if !(run.length_squared() > 0.0) {
        return false;
    }

    let at = pool.position_of(piece[corner]) - from;
    let along = at.dot(run) / run.length_squared();

    along > 0.0 && along < 1.0 && (at - (run * along)).length() < pool.near_enough()
}

/// The nearest corner from `corner` stepping `step` (+1 or -1) round the ring that is not
/// dropped, or `None` where every other corner is.
pub fn standing(piece: &[i32], dropped: &[bool], corner: usize, step: i64) -> Option<usize> {
    let count = piece.len() as i64;

    for walked in 1..count {
        let at = ((corner as i64 + (step * walked)) % count + count) % count;

        if !dropped[at as usize] {
            return Some(at as usize);
        }
    }

    None
}

/// Whether `piece` covers nothing: fewer than three corners, or every corner within the pool's
/// ruler of the line through its two furthest-apart corners.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` refuses a NaN as well.
pub fn covers_nothing(pool: &SurfaceVertexPool, piece: &[i32]) -> bool {
    if piece.len() < 3 {
        return true;
    }

    let mut from = pool.position_of(piece[0]);
    let mut to = from;
    let mut longest = 0.0;

    for one in 0..piece.len() {
        for other in (one + 1)..piece.len() {
            let apart = (pool.position_of(piece[other]) - pool.position_of(piece[one])).length();

            if apart > longest {
                longest = apart;
                from = pool.position_of(piece[one]);
                to = pool.position_of(piece[other]);
            }
        }
    }

    if !(longest > 0.0) {
        return true;
    }

    let along = (to - from) / longest;

    piece.iter().all(|&vertex| {
        let at = pool.position_of(vertex) - from;

        !((at - (along * at.dot(along))).length() > pool.near_enough())
    })
}

/// Whether every point of `rim` stands inside every half plane of `region` (each a point and its
/// inward direction), to a slack measured against the RING's own size rather than the part's:
/// this is arithmetic on these few points, and what it has to forgive is the rounding in their
/// own dot products.
pub fn it_is_convex(rim: &[Vector3], region: &[(Vector3, Vector3)]) -> bool {
    let mut least = rim[0];
    let mut most = rim[0];

    for &at in rim {
        least = Vector3::component_min(least, at);
        most = Vector3::component_max(most, at);
    }

    let slack = 1e-9 * cs_math::max_f64(1.0, (most - least).length());

    for &at in rim {
        for &(point, inward) in region {
            if (at - point).dot(inward) < -slack {
                return false;
            }
        }
    }

    true
}

/// Whether `face` lies in the bite's plane (through `plane_point`, facing `plane_normal`, to the
/// pool's ruler) and its bounding box overlaps the bite's (`low`..`high`).
pub fn is_in_the_bites_way(
    pool: &SurfaceVertexPool,
    face: (i32, i32, i32),
    plane_normal: Vector3,
    plane_point: Vector3,
    low: Vector3,
    high: Vector3,
) -> bool {
    let mut lowest = pool.position_of(face.0);
    let mut highest = lowest;

    for vertex in [face.0, face.1, face.2] {
        let at = pool.position_of(vertex);

        if (at - plane_point).dot(plane_normal).abs() > pool.near_enough() {
            return false;
        }

        lowest = Vector3::component_min(lowest, at);
        highest = Vector3::component_max(highest, at);
    }

    !(highest.x < low.x
        || lowest.x > high.x
        || highest.y < low.y
        || lowest.y > high.y
        || highest.z < low.z
        || lowest.z > high.z)
}

/// The convex region `rim` (in the plane of `plane_normal`
/// through `plane_point`) bitten out of `faces` - every face in the bite's way clipped to what
/// lies outside the region, its pieces snapped to `snap_to` (the profile, for pool vertices from
/// `split_from` on), the vertices standing on their sides carried in, straight-through vertices
/// dropped, and each piece that covers anything fanned facing its own face. Refused
/// (`SurfaceCornerNotSupported`) for a rim of fewer than three points, a side with no half plane,
/// or a rim that is not convex.
#[allow(clippy::too_many_arguments)]
pub fn try_take(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    rim: &[Vector3],
    plane_normal: Vector3,
    plane_point: Vector3,
    snap_to: &[i32],
    split_from: i32,
) -> Result<(), crate::bevel::BevelAdvisory> {
    use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

    let not_supported = |at: Vector3| {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![at.into()],
        )
    };

    if rim.len() < 3 {
        return Err(not_supported(plane_point));
    }

    let mut middle = Vector3::ZERO;
    for &at in rim {
        middle += at;
    }
    middle /= rim.len() as f64;

    let mut region = Vec::with_capacity(rim.len());
    let mut low = rim[0];
    let mut high = rim[0];
    for side in 0..rim.len() {
        let from = rim[side];
        let to = rim[(side + 1) % rim.len()];
        let Some(half_plane) =
            super::surface_end_gives_up_the_section::try_half_plane(from, to, plane_normal, middle)
        else {
            return Err(not_supported(from));
        };
        region.push(half_plane);
        low = Vector3::component_min(low, from);
        high = Vector3::component_max(high, from);
    }

    if !it_is_convex(rim, &region) {
        return Err(not_supported(rim[0]));
    }

    let near = pool.near_enough();
    low -= Vector3::new(near, near, near);
    high += Vector3::new(near, near, near);

    let mut carried = IndexSet::new();
    for &face in faces.iter() {
        if is_in_the_bites_way(pool, face, plane_normal, plane_point, low, high) {
            carried.insert(face.0);
            carried.insert(face.1);
            carried.insert(face.2);
        }
    }

    let mut clipped: Vec<(Vector3, Vec<i32>)> = Vec::new();
    let mut places: Vec<((i32, i32, i32), i32, usize)> = Vec::with_capacity(faces.len());
    for &face in faces.iter() {
        if !is_in_the_bites_way(pool, face, plane_normal, plane_point, low, high) {
            places.push((face, -1, 0));
            continue;
        }

        let (pieces, _) = super::surface_end_gives_up_the_section::pieces_outside_of(
            &[face.0, face.1, face.2],
            pool,
            &region,
            Some(snap_to),
        );
        let face_normal = (pool.position_of(face.1) - pool.position_of(face.0))
            .cross(pool.position_of(face.2) - pool.position_of(face.0));
        let facing = if face_normal.length_squared() > 0.0 {
            face_normal
        } else {
            plane_normal
        };
        let first = clipped.len();
        for piece in pieces {
            clipped.push((
                facing,
                super::surface_end_notch::snapped_to_the_profile(&piece, snap_to, pool, split_from),
            ));
        }
        places.push((face, first as i32, clipped.len() - first));
    }

    let rings: Vec<Vec<i32>> = clipped.iter().map(|(_, ring)| ring.clone()).collect();
    let shared = SurfaceBiteSharedSides::new(rings);
    let mut rebuilt = Vec::with_capacity(faces.len());
    for &(face, from, count) in &places {
        if from < 0 {
            rebuilt.push(face);
            continue;
        }

        for (piece, (facing, ring)) in clipped.iter().enumerate().skip(from as usize).take(count) {
            let snapped = without_the_straight_through(
                pool,
                &with_the_vertices_standing_on_it(
                    pool,
                    ring.clone(),
                    &carried,
                    Some(&shared),
                    piece as i32,
                ),
                split_from,
            );
            if covers_nothing(pool, &snapped) {
                continue;
            }

            super::surface_fan::emit_facing(&snapped, pool, &mut rebuilt, *facing);
        }
    }

    *faces = rebuilt;
    Ok(())
}
