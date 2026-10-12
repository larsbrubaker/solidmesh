//! Triangulating the rings a smooth-side cut keeps (`Bevel/Sections/SectionSmoothSide.Triangulate.cs`,
//! part of the C# `internal static partial class SectionSmoothSide`; free functions here beside
//! [`super::section_smooth_side`]'s types). A kept ring goes to the ears
//! ([`crate::bevel::surface::surface_ring_ears::try_triangulate_the_ring`]) once every run of points
//! KNOWN to lie on the straight segment between its own neighbours is lifted out, and each such run is
//! put back afterwards as a fan.
//!
//! WHY THE EARS MAY NOT BE HANDED THEM. A SPLIT station's boundary point is placed ON the join
//! between its two unsplit neighbours' points (`BevelSection::left_boundary_is_join_placed`). Where
//! all three stand inside ONE flat facet that join is a straight segment, so the middle point is
//! collinear with the other two by construction - the cross product read 2.6e-16 on the barrel's
//! second 36-station mouth. The ears take such a triple as their final remainder, an area of 1.3e-16,
//! and wind it by its own normal, which at that area is rounding: the triangle came out backwards and
//! its three directed edges were each run twice by the rebuild.
//!
//! KNOWN, NOT MEASURED. The reading is by IDENTITY throughout: the point is join-placed (the builder
//! said so), and its two RING neighbours are exactly its two POLYLINE neighbours, so no fold crossing
//! was minted between it and either of them and all three lie in this one facet. No position is
//! compared and no tolerance is used.

use indexmap::IndexMap;
use vector_math::{Vector3, Vector3Float};

use super::section_smooth_side::{SmoothSideCut, SmoothSideRefusal};
use super::SectionPool;
use crate::bevel::surface::surface_ring_ears;
use crate::bevel::surface::SurfaceVertexPool;

/// Each boundary vertex's two neighbours along the boundary polyline, `(before, after)`.
pub type PolylineNeighbours = IndexMap<i32, (i32, i32)>;

/// One collinear run lifted out of a ring: its two standing ends and the points between them.
pub type CollinearRun = (i32, i32, Vec<i32>);

/// C# `TryTriangulateAKeptRing`: triangulates one kept ring into `triangles`, having first lifted out
/// every run of points known to lie on the straight segment between the ring's own neighbours of that
/// run; the reduced ring is ear-clipped and the one triangle running the directed edge each run
/// spanned is replaced by a fan from its third vertex over the run, same winding. Those fan triangles
/// have real area - their apex is off the line - so nothing here rests on a degenerate normal.
pub fn try_triangulate_a_kept_ring(
    pool: &SectionPool,
    cut: &SmoothSideCut,
    along_the_boundary: &PolylineNeighbours,
    ring: &[i32],
    plane: Vector3,
    triangles: &mut Vec<(i32, i32, i32)>,
) -> Result<(), SmoothSideRefusal> {
    let (reduced, runs) = the_ring_without_its_collinear_runs(cut, along_the_boundary, ring);

    if runs.is_empty() {
        if try_triangulate(pool, ring, plane, triangles) {
            return Ok(());
        }

        return Err(SmoothSideRefusal::ARingWillNotTriangulate);
    }

    if reduced.len() < 3 {
        return Err(SmoothSideRefusal::ARingIsNothingButACollinearRun);
    }

    let mut drawn = Vec::new();

    if !try_triangulate(pool, &reduced, plane, &mut drawn) {
        return Err(SmoothSideRefusal::ARingWillNotTriangulate);
    }

    for (end0, end1, through) in &runs {
        if !try_fan_over_the_run(&mut drawn, *end0, *end1, through) {
            return Err(SmoothSideRefusal::ACollinearRunHasNoTriangleToFan);
        }
    }

    triangles.extend(drawn);

    Ok(())
}

/// C# `ThePolylineNeighbours`: each boundary vertex's two neighbours along the boundary polyline, in
/// chain order (wrapping, as C# does whether the boundary is closed or not).
pub fn the_polyline_neighbours(cut: &SmoothSideCut) -> PolylineNeighbours {
    let mut along = IndexMap::new();
    let count = cut.boundary.len();

    for i in 0..count {
        along.insert(
            cut.boundary[i],
            (
                cut.boundary[(i + count - 1) % count],
                cut.boundary[(i + 1) % count],
            ),
        );
    }

    along
}

/// C# `TheRingWithoutItsCollinearRuns`: the ring with every collinear run lifted out of it, leaving
/// each run's two ends adjacent, and the runs themselves for [`try_fan_over_the_run`] to put back.
pub fn the_ring_without_its_collinear_runs(
    cut: &SmoothSideCut,
    along_the_boundary: &PolylineNeighbours,
    ring: &[i32],
) -> (Vec<i32>, Vec<CollinearRun>) {
    let mut runs = Vec::new();

    let mut lifted = vec![false; ring.len()];
    let mut standing = 0;

    for (i, is_lifted) in lifted.iter_mut().enumerate() {
        *is_lifted = lies_on_the_segment_between_its_neighbours(cut, along_the_boundary, ring, i);

        if !*is_lifted {
            standing += 1;
        }
    }

    if standing == ring.len() {
        return (ring.to_vec(), runs);
    }

    if standing == 0 {
        // Every point of the ring reads as lifted, which leaves no ends to fan between; hand the ring
        // on whole and let the ears answer for it.
        runs.clear();

        return (ring.to_vec(), runs);
    }

    let mut from = 0;

    while lifted[from] {
        from += 1;
    }

    let mut reduced: Vec<i32> = Vec::new();
    let mut run: Option<Vec<i32>> = None;

    for step in 0..ring.len() {
        let i = (from + step) % ring.len();

        if lifted[i] {
            run.get_or_insert_with(Vec::new).push(ring[i]);

            continue;
        }

        if let Some(through) = run.take() {
            runs.push((reduced[reduced.len() - 1], ring[i], through));
        }

        reduced.push(ring[i]);
    }

    if let Some(through) = run {
        runs.push((reduced[reduced.len() - 1], reduced[0], through));
    }

    (reduced, runs)
}

/// C# `LiesOnTheSegmentBetweenItsNeighbours`: whether a ring vertex is a join-placed point whose two
/// RING neighbours are exactly its two POLYLINE neighbours - the one reading under which it is known,
/// without measuring, to stand on the straight segment between them.
pub fn lies_on_the_segment_between_its_neighbours(
    cut: &SmoothSideCut,
    along_the_boundary: &PolylineNeighbours,
    ring: &[i32],
    at: usize,
) -> bool {
    let vertex = ring[at];

    if !cut.join_placed.contains(&vertex) {
        return false;
    }

    let Some(&(neighbour_before, neighbour_after)) = along_the_boundary.get(&vertex) else {
        return false;
    };

    let before = ring[(at + ring.len() - 1) % ring.len()];
    let after = ring[(at + 1) % ring.len()];

    (before == neighbour_before && after == neighbour_after)
        || (before == neighbour_after && after == neighbour_before)
}

/// C# `TryFanOverTheRun`: replaces the one triangle running the directed edge `end0` to `end1` by a
/// fan from its third vertex over the run that edge stands in place of.
pub fn try_fan_over_the_run(
    drawn: &mut Vec<(i32, i32, i32)>,
    end0: i32,
    end1: i32,
    through: &[i32],
) -> bool {
    for t in 0..drawn.len() {
        let (v0, v1, v2) = drawn[t];
        let apex = if v0 == end0 && v1 == end1 {
            v2
        } else if v1 == end0 && v2 == end1 {
            v0
        } else if v2 == end0 && v0 == end1 {
            v1
        } else {
            -1
        };

        if apex < 0 {
            continue;
        }

        let mut fan = Vec::with_capacity(through.len() + 1);
        let mut here = end0;

        for &point in through {
            fan.push((apex, here, point));
            here = point;
        }

        fan.push((apex, here, end1));

        drawn.splice(t..t + 1, fan);

        return true;
    }

    false
}

/// C# `TryTriangulate`: ear-clips one ring of pool points in `plane`, appending pool-index triangles.
///
/// One local vertex per DISTINCT vertex of the ring rather than one per POSITION in it: a bridged
/// ring walks the vertices of its slit twice, and the only thing telling that slit from a ring that
/// crosses itself is the two sides being the same edge run either way round. That is an identity
/// between indices, and numbering the two occurrences apart loses it.
pub fn try_triangulate(
    pool: &SectionPool,
    ring: &[i32],
    plane: Vector3,
    triangles: &mut Vec<(i32, i32, i32)>,
) -> bool {
    let mut local_of: IndexMap<i32, i32> = IndexMap::new();
    let mut at: Vec<Vector3Float> = Vec::new();
    let mut back: Vec<i32> = Vec::new();
    let mut indices = vec![0; ring.len()];

    for (i, &vertex) in ring.iter().enumerate() {
        let local = match local_of.get(&vertex) {
            Some(&local) => local,
            None => {
                let local = at.len() as i32;
                local_of.insert(vertex, local);
                at.push(Vector3Float::from_vector3(pool.position_of(vertex)));
                back.push(vertex);
                local
            }
        };

        indices[i] = local;
    }

    let mut drawn = Vec::new();

    if !surface_ring_ears::try_triangulate_the_ring(
        &indices,
        &SurfaceVertexPool::new(&at),
        &mut drawn,
        plane,
        false,
    ) {
        return false;
    }

    for (v0, v1, v2) in drawn {
        triangles.push((back[v0 as usize], back[v1 as usize], back[v2 as usize]));
    }

    true
}
