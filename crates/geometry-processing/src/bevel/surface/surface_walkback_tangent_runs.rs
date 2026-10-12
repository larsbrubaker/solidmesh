//! What each cut of the face walkback left of its tangent line as a boundary to sew to: the
//! region's own CUT EDGES chained into contiguous runs ([`gather_tangent_runs`], [`runs_along`]),
//! and the stretches standing on a rail the cut BORROWED from a sibling
//! ([`continuations_along`]). Ports the run half of `Bevel/Surface/SurfaceWalkbackBands.cs`
//! (`GatherTangentRuns`, `ContinuationsAlong`, `TheRimTheBandConsumed`, `RunsAlong`, `RunsOn`,
//! `OneStationPerPooledPoint`); the band's reach and its standing-feature guard are
//! `surface_walkback_bands`.
//!
//! C#'s `List.Sort` with a comparison is .NET's unstable introsort, and vertices standing at one
//! distance along the line tie; the sorts here run [`introspective_sort`] so ties land where .NET
//! leaves them.

use std::cmp::Ordering;
use std::collections::{HashMap, HashSet};

use indexmap::IndexMap;
use vector_math::cs_sort::introspective_sort;

use super::surface_band_runs_off_the_face_beyond::{consumes_the_face_to_its_rim, BandSpan};
use super::{SurfaceFacetCut, SurfaceLine, SurfaceRegionWalkback, SurfaceVertexPool};
use crate::topology::FaceGroup;

/// One region's kept triangles, as the walkback emitted them.
pub type KeptFaces = Vec<(i32, i32, i32)>;

/// One per-cut table of runs: for each `(edge index, left)`, contiguous runs of vertex indices -
/// the shape `SurfaceWalkbackResult` publishes its tables in.
pub type CutRuns = IndexMap<(i32, bool), Vec<Vec<i32>>>;

/// C# `GatherTangentRuns`: what each cut left of its tangent line as a boundary of the region it
/// cut - the region's own CUT EDGES (edges of kept triangles with both ends standing on the line)
/// chained into contiguous runs, each ordered the way the chain walks the line - keyed by
/// `(edge index, left)`.
///
/// Runs, not one list, and read off edges rather than off vertices: the tangent line is infinite,
/// so on a region with a notch or a bore it comes out of the region and goes back in, and a vertex
/// that merely GRAZES the line is in no edge and so in no run.
///
/// `beveled_mesh_edges` and `chain_vertices` are the mesh edges this pass is beveling and their
/// ends (`SurfaceFaceWalkback.TheChainsOwn`). Where `kept_range_of_facet` and
/// `rail_continuations` are both given, each cut's borrowed-rail stretches are written into
/// `rail_continuations` as well.
pub fn gather_tangent_runs(
    walkbacks: &IndexMap<i32, SurfaceRegionWalkback>,
    faces_of_region: &IndexMap<i32, KeptFaces>,
    pool: &SurfaceVertexPool,
    beveled_mesh_edges: Option<&HashSet<i32>>,
    chain_vertices: Option<&HashSet<i32>>,
    kept_range_of_facet: Option<&IndexMap<i32, (i32, i32)>>,
    mut rail_continuations: Option<&mut CutRuns>,
) -> CutRuns {
    let mut tangent_runs = IndexMap::new();
    let no_faces = KeptFaces::new();

    for walkback in walkbacks.values() {
        let kept = faces_of_region.get(&walkback.key).unwrap_or(&no_faces);

        for (index, cut) in walkback.cuts.iter().enumerate() {
            let consumed = the_rim_the_band_consumed(
                &walkback.region,
                cut,
                beveled_mesh_edges,
                chain_vertices,
                pool,
            );

            tangent_runs.insert(
                (cut.edge_index, cut.left),
                runs_along(cut, kept, pool, consumed.as_deref()),
            );

            if let (Some(continuations), Some(kept_range_of_facet)) =
                (rail_continuations.as_deref_mut(), kept_range_of_facet)
            {
                continuations.insert(
                    (cut.edge_index, cut.left),
                    continuations_along(walkback, index, kept_range_of_facet, kept, pool),
                );
            }
        }
    }

    tangent_runs
}

/// C# `ContinuationsAlong`: the stretches of a cut's rim that stand on a BORROWED rail rather than
/// on the cut's own tangent line, one run per contiguous stretch, in the cut's own walk order.
///
/// At a facet the cut reached across a crease, `SurfaceBandOutline.AsDrawnOn` draws the band's far
/// side along the SIBLING cut's line, so the rim the cut left there stands on that line and on no
/// line of the cut's own - and [`runs_along`] has no way to see it. That is the segment a mitre at
/// the end of such a cut has to meet. The band ENDS are still the cut's, so a continuation stops
/// where the cut's own band does.
pub fn continuations_along(
    walkback: &SurfaceRegionWalkback,
    cut_index: usize,
    kept_range_of_facet: &IndexMap<i32, (i32, i32)>,
    kept: &[(i32, i32, i32)],
    pool: &SurfaceVertexPool,
) -> Vec<Vec<i32>> {
    let Some(Some(borrowed_on)) = walkback.rail_borrowed_on.get(cut_index) else {
        return Vec::new();
    };

    let cut = &walkback.cuts[cut_index];

    // Grouped by the sibling whose rail was borrowed: two facets that borrowed the SAME rail hold
    // one stretch of one line between them, and reading them apart would cut that stretch in two
    // at the crease they share.
    let mut facets_of: IndexMap<i32, KeptFaces> = IndexMap::new();

    for (face_index, &borrowed) in borrowed_on {
        let Some(&(from, count)) = kept_range_of_facet.get(face_index) else {
            continue;
        };

        if count == 0 {
            // The band took the whole facet: nothing of it stands to read a rim off.
            continue;
        }

        let faces = facets_of.entry(borrowed).or_default();

        for face in from..from + count {
            faces.push(kept[face as usize]);
        }
    }

    let mut continuations = Vec::new();

    for (&borrowed, faces) in &facets_of {
        for run in runs_on(
            cut,
            &walkback.cuts[borrowed as usize].tangent_line,
            faces,
            pool,
            None,
        ) {
            if run.len() >= 2 {
                continuations.push(run);
            }
        }
    }

    // In the cut's own walk order, the order everything downstream reads a run in.
    let direction = cut.tangent_line.direction;
    introspective_sort(
        &mut continuations,
        &mut |first: &Vec<i32>, second: &Vec<i32>| {
            cs_compare(
                pool.position_of(first[0]).dot(direction),
                pool.position_of(second[0]).dot(direction),
            )
        },
    );

    continuations
}

/// C# `TheRimTheBandConsumed`: the region's own rim, where this cut CONSUMED the region to it - and
/// `None` everywhere else. A band that reaches exactly as far as its face does leaves no triangle
/// of that face standing, so what stands where the tangent line would have been cut is the rim
/// itself, already in the pool as the part's own (`consumes_the_face_to_its_rim`). The whole rim is
/// handed over: [`runs_on`] keeps only the segments standing on the line and inside the band.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)`, which refuses a NaN as well.
fn the_rim_the_band_consumed(
    region: &FaceGroup,
    cut: &SurfaceFacetCut,
    beveled_mesh_edges: Option<&HashSet<i32>>,
    chain_vertices: Option<&HashSet<i32>>,
    pool: &SurfaceVertexPool,
) -> Option<Vec<(i32, i32)>> {
    let span = cut.chain_to - cut.chain_from;
    let length = span.length();

    if !(length > 0.0) {
        // An end cut, or any other cut with no chain edge behind it: there is no span to read a
        // band's reach over.
        return None;
    }

    // C# passes its optional sets through as null, which the reading takes as empty.
    let none = HashSet::new();
    let band = BandSpan {
        from: cut.chain_from,
        along: span / length,
        length,
        inward: cut.inward,
        setback: (cut.tangent_line.point - cut.chain_from).dot(cut.inward),
    };

    if !consumes_the_face_to_its_rim(
        Some(region),
        &band,
        beveled_mesh_edges.unwrap_or(&none),
        chain_vertices.unwrap_or(&none),
        pool.near_enough(),
    ) {
        return None;
    }

    Some(
        region
            .boundary_edges()
            .iter()
            .map(|edge| (edge.start_vertex, edge.end_vertex))
            .collect(),
    )
}

/// C# `RunsAlong`: the contiguous runs of one cut's line through one region's kept triangles.
/// `consumed_rim` is the region's own rim where the band consumed the face to it, whose segments
/// stand in for the kept triangles' edges there, or `None` in the ordinary case.
pub fn runs_along(
    cut: &SurfaceFacetCut,
    kept: &[(i32, i32, i32)],
    pool: &SurfaceVertexPool,
    consumed_rim: Option<&[(i32, i32)]>,
) -> Vec<Vec<i32>> {
    runs_on(cut, &cut.tangent_line, kept, pool, consumed_rim)
}

/// C# `RunsOn`: the same reading, taken along a line that need not be the cut's own: the cut still
/// says where the band ENDS and which way along the run is forward, and `line` says which edges
/// stand on the rail.
fn runs_on(
    cut: &SurfaceFacetCut,
    line: &SurfaceLine,
    kept: &[(i32, i32, i32)],
    pool: &SurfaceVertexPool,
    consumed_rim: Option<&[(i32, i32)]>,
) -> Vec<Vec<i32>> {
    let mut on_the_line: HashMap<i32, bool> = HashMap::new();
    let mut next_to: IndexMap<i32, Vec<i32>> = IndexMap::new();

    let mut stands_on_it = |vertex: i32| -> bool {
        *on_the_line
            .entry(vertex)
            .or_insert_with(|| line.distance_to(pool.position_of(vertex)) <= pool.near_enough())
    };

    // Inside the band's own ends, not merely on the line. Where the tangent line lands ON A CREASE
    // of the mesh (SurfaceWallSection.RestsOnTheCrease) the crease is an edge of the part for its
    // whole length, and the stretch of it past the band's end is the rim of the piece the corner's
    // own blend bites away, not the strip's seam. Asked of the two VERTICES rather than of the
    // middle between them, so the choice is made the same way every time and the way the strip is
    // built: the seam stops inside the band.
    let inside_the_band = |first: i32, second: i32| -> bool {
        cut.ends.iter().all(|end| {
            !(end.signed_distance(pool.position_of(first)) < -pool.near_enough()
                || end.signed_distance(pool.position_of(second)) < -pool.near_enough())
        })
    };

    let mut join_if_it_stands_on_the_line = |first: i32, second: i32| {
        if first == second
            || !stands_on_it(first)
            || !stands_on_it(second)
            || !inside_the_band(first, second)
        {
            return;
        }

        join(&mut next_to, first, second);
        join(&mut next_to, second, first);
    };

    for &(v0, v1, v2) in kept {
        join_if_it_stands_on_the_line(v0, v1);
        join_if_it_stands_on_the_line(v1, v2);
        join_if_it_stands_on_the_line(v2, v0);
    }

    // And the RIM, where the band consumed the face to it: the region kept no triangle to read the
    // line off, and its rim is what stands there instead.
    for &(first, second) in consumed_rim.unwrap_or(&[]) {
        join_if_it_stands_on_the_line(first, second);
    }

    // Everything joined to everything it can reach: one group per stretch of the line the region
    // holds. They are all on one line, so sorting a group by how far along it each vertex stands
    // puts the stretch back in order without walking the chain edge by edge.
    let mut group_of: HashMap<i32, usize> = HashMap::new();
    let mut groups: Vec<Vec<i32>> = Vec::new();

    for &vertex in next_to.keys() {
        if group_of.contains_key(&vertex) {
            continue;
        }

        let mut group = Vec::new();
        let mut walking = vec![vertex];

        while let Some(at) = walking.pop() {
            if group_of.contains_key(&at) {
                continue;
            }

            group_of.insert(at, groups.len());
            group.push(at);

            for &neighbour in &next_to[&at] {
                walking.push(neighbour);
            }
        }

        groups.push(group);
    }

    let direction = cut.tangent_line.direction;
    let along = |vertex: i32| pool.position_of(vertex).dot(direction);

    for group in &mut groups {
        introspective_sort(group, &mut |&first: &i32, &second: &i32| {
            cs_compare(along(first), along(second))
        });
    }

    introspective_sort(&mut groups, &mut |first: &Vec<i32>, second: &Vec<i32>| {
        cs_compare(along(first[0]), along(second[0]))
    });

    groups
        .iter()
        .map(|group| one_station_per_pooled_point(line, group, pool))
        .collect()
}

/// C#'s local `Join`: notes that `from` is joined to `to` along the line.
fn join(next_to: &mut IndexMap<i32, Vec<i32>>, from: i32, to: i32) {
    let neighbours = next_to.entry(from).or_insert_with(|| Vec::with_capacity(2));

    if !neighbours.contains(&to) {
        neighbours.push(to);
    }
}

/// C# `OneStationPerPooledPoint`: one station per POOLED point - where two vertices of a run stand
/// closer together than the pool's `near_enough`, they are one point as far as everything
/// downstream is concerned, and the station kept is the one NEAREST the line, which is the one the
/// cut crossed at. Two vertices the pool would have minted as one otherwise have the end cut's cap
/// and the stitch's window disagree about the sliver between them, covering it twice.
fn one_station_per_pooled_point(
    line: &SurfaceLine,
    ordered: &[i32],
    pool: &SurfaceVertexPool,
) -> Vec<i32> {
    let mut stations: Vec<i32> = Vec::with_capacity(ordered.len());

    for &vertex in ordered {
        if let Some(last) = stations.last_mut() {
            if (pool.position_of(*last) - pool.position_of(vertex)).length() <= pool.near_enough() {
                if line.distance_to(pool.position_of(vertex))
                    < line.distance_to(pool.position_of(*last))
                {
                    *last = vertex;
                }

                continue;
            }
        }

        stations.push(vertex);
    }

    stations
}

/// C#'s `double.CompareTo`, which orders a NaN before every number.
fn cs_compare(a: f64, b: f64) -> Ordering {
    match (a.is_nan(), b.is_nan()) {
        (true, true) => Ordering::Equal,
        (true, false) => Ordering::Less,
        (false, true) => Ordering::Greater,
        (false, false) => a.partial_cmp(&b).unwrap_or(Ordering::Equal),
    }
}
