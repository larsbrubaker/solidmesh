//! The two-pass rewrite of the mesh the walkback cuts (`Bevel/Surface/SurfaceWalkbackRewrite.cs`,
//! C# `internal static class`, free functions here): every crossing of a tangent plane with a facet
//! edge, welded into the pool and listed against that edge ([`try_find_crossings`]); the ring each
//! face is rebuilt from with those crossings inserted ([`ring_of`]); and the finished mesh,
//! renumbered with the vertices no face uses any more dropped ([`assemble`]).
//!
//! Split out of `SurfaceFaceWalkback`, which was over the file-length limit. The walkback is the
//! decision - which region is cut where - and this is the mechanics of carrying that decision into
//! triangles without leaving a T junction or a crack behind. Nothing here is new.

use indexmap::IndexMap;
use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::{Vector3, Vector3Float};

use super::{
    surface_cut_clip, surface_ring, SurfaceFacetCut, SurfaceRegionWalkback,
    SurfaceStandingFeatureWithheld, SurfaceVertexPool, SurfaceWalkbackRuns,
};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// The vertices split into each mesh edge, keyed by the edge's `(low, high)` vertex pair (C#'s
/// `splitsOnEdge`).
pub type SurfaceSplitsOnEdge = IndexMap<(i32, i32), Vec<i32>>;

/// C# `TryFindCrossings`: every point where a region's cut crosses one of its own facet edges,
/// welded into the pool and listed against that mesh edge so that BOTH faces on it are rebuilt
/// through the same vertex.
///
/// Refuses a crossing on an edge the mesh does not close - one the graph gives other than two faces
/// on. There is no second face there to rebuild through the new vertex, so splitting would leave
/// exactly the crack the two-pass rebuild exists to avoid. The region rims are already known to be
/// sound by this point; this is the same question asked of the edges INSIDE a region.
pub fn try_find_crossings(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    walkbacks: &IndexMap<i32, SurfaceRegionWalkback>,
    pool: &mut SurfaceVertexPool,
) -> Result<SurfaceSplitsOnEdge, BevelAdvisory> {
    let mut splits_on_edge = SurfaceSplitsOnEdge::new();

    let on_the_cut = surface_cut_clip::on_the_cut(pool);

    let mut edge_of_pair: std::collections::HashMap<(i32, i32), i32> =
        std::collections::HashMap::with_capacity(graph.edge_count().max(0) as usize);

    for edge in 0..graph.edge_count() {
        let (v0, v1) = (graph.get_vertex0(edge), graph.get_vertex1(edge));
        edge_of_pair.insert((v0.min(v1), v0.max(v1)), edge);
    }

    for walkback in walkbacks.values() {
        // C# iterates the walkback's `HashSet` of faces, which (built from the region's faces and
        // never changed) yields them in the region's own order; the Rust set does not keep an
        // order, so the region is read instead, filtered to the set.
        for &face_index in walkback.region.faces() {
            if !walkback.faces.contains(&face_index) {
                continue;
            }

            let face = &mesh.faces[face_index as usize];

            for (first, second) in [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)] {
                // Always from the lower index, so the two faces on an edge ask for the same point
                // in the same arithmetic and land on it to the bit rather than to the weld.
                let low = first.min(second);
                let high = first.max(second);

                let from = pool.position_of(low);
                let to = pool.position_of(high);

                for (index, cut) in walkback.cuts.iter().enumerate() {
                    if !walkback.reaches[index].contains(&face_index) {
                        continue;
                    }

                    // The tangent plane and, where the band has them, its end planes: each of
                    // them splits this facet, and each split has to be known to the face on the
                    // far side of the mesh edge it lands on.
                    for (at_from, at_to, is_tangent) in crossings(cut, from, to) {
                        if (at_from > on_the_cut && at_to < -on_the_cut)
                            || (at_from < -on_the_cut && at_to > on_the_cut)
                        {
                            let at = from + ((to - from) * (at_from / (at_from - at_to)));

                            // A tangent line crossing outside the band's own ends is a point on a
                            // line the cut never reaches: made a vertex, it joins the run the strip
                            // sews to and drags the end of the strip out past the corner the chain
                            // stopped at. The END planes are not filtered that way - each of them
                            // splits the whole facet, and the piece on the far side has to know
                            // where the split landed on its own edges.
                            if is_tangent && !inside_the_ends(cut, at, on_the_cut) {
                                continue;
                            }

                            match edge_of_pair.get(&(low, high)) {
                                Some(&mesh_edge) if graph.get_face_count(mesh_edge) == 2 => {}
                                _ => {
                                    return Err(BevelAdvisory::new(
                                        BevelAdvisoryCode::FaceRegionRimIsPartial,
                                        vec![1.into()],
                                    ));
                                }
                            }

                            let split =
                                the_split_on_the_edge(&splits_on_edge, (low, high), at, pool);
                            remember(&mut splits_on_edge, (low, high), split);
                        }
                    }
                }
            }
        }
    }

    Ok(splits_on_edge)
}

/// How far a mesh edge's two ends stand from each plane a cut is bounded by - its tangent plane
/// first, then any end of its band.
fn crossings(cut: &SurfaceFacetCut, from: Vector3, to: Vector3) -> Vec<(f64, f64, bool)> {
    let mut found = Vec::with_capacity(1 + cut.ends.len());

    found.push((cut.signed_distance(from), cut.signed_distance(to), true));

    for end in &cut.ends {
        found.push((end.signed_distance(from), end.signed_distance(to), false));
    }

    found
}

/// Whether a point stands between the two planes a band's cut stops at, if it has any.
fn inside_the_ends(cut: &SurfaceFacetCut, at: Vector3, on_the_cut: f64) -> bool {
    for end in &cut.ends {
        if end.signed_distance(at) < -on_the_cut {
            return false;
        }
    }

    true
}

/// The vertex a crossing at `at` IS: the one a cut before it already split into this very edge
/// within the pool's ruler of that place, or a new one where none stands there.
///
/// A SPLIT ON A MESH EDGE IS A FUNCTION OF THE EDGE, not of which plane asked for it. Where a
/// closed chain turns, the two bands either side of the corner are cut off on tangent planes that
/// meet at the corner's mitre point, and where that point stands on a facet's own edge each band
/// computes ITS crossing of the edge from its own plane: the same point arrived at two ways, and
/// the two ways disagree by more than the weld - 4.4e-5 against a 2.96e-5 weld on the three chord
/// half disc, which is a cross product of two nearly parallel planes divided by a nearly zero
/// denominator and not a rounding a weld can be widened to cover. Two vertices then stand where the
/// construction has one place, the seam the strip sews to and the face the cap left carry one each,
/// and the shell comes out open along the tangent line. So the memo is read before anything is
/// minted, the same rule `SurfaceSplitsOnASegment` keeps for a stretch and `SurfaceCutClip.ClipBy`
/// for a cut's own profile. The ruler and not the weld, because what is being asked is whether a
/// point that has been through a plane crossing still lies where another plane crossing put it; and
/// two splits of ONE edge nearer than the ruler are a crack whichever planes drew them, since every
/// reading downstream calls them one place.
fn the_split_on_the_edge(
    splits_on_edge: &SurfaceSplitsOnEdge,
    edge: (i32, i32),
    at: Vector3,
    pool: &mut SurfaceVertexPool,
) -> i32 {
    if let Some(splits) = splits_on_edge.get(&edge) {
        let mut nearest = pool.near_enough();
        let mut already = -1;

        // The NEAREST and not the first: an edge a band's tangent plane and its end plane both
        // cross carries more than one split, and taking the first within the ruler would answer by
        // the order the cuts happened to be listed in.
        for &split in splits {
            let how_far = (pool.position_of(split) - at).length();

            if how_far <= nearest {
                nearest = how_far;
                already = split;
            }
        }

        if already >= 0 {
            return already;
        }
    }

    pool.add(at)
}

fn remember(splits_on_edge: &mut SurfaceSplitsOnEdge, edge: (i32, i32), vertex: i32) {
    let splits = splits_on_edge
        .entry(edge)
        .or_insert_with(|| Vec::with_capacity(1));

    if !splits.contains(&vertex) {
        splits.push(vertex);
    }
}

/// C# `RingOf`: a face as a ring of vertices with whatever was split into its edges inserted, each
/// in its place along the edge. Three vertices for a face nothing crossed, which is nearly all.
pub fn ring_of(
    mesh: &Mesh,
    face_index: i32,
    splits_on_edge: &SurfaceSplitsOnEdge,
    pool: &SurfaceVertexPool,
) -> Vec<i32> {
    let face = &mesh.faces[face_index as usize];
    let corners = [face.v0, face.v1, face.v2];
    let mut ring = Vec::with_capacity(3);

    for corner in 0..corners.len() {
        let from = corners[corner];
        let to = corners[(corner + 1) % corners.len()];

        ring.push(from);

        let Some(splits) = splits_on_edge.get(&(from.min(to), from.max(to))) else {
            continue;
        };

        let start = pool.position_of(from);
        let run = pool.position_of(to) - start;
        let length_squared = run.length_squared();

        #[allow(clippy::neg_cmp_op_on_partial_ord)]
        if !(length_squared > 0.0) {
            continue;
        }

        let mut along: Vec<(f64, i32)> = Vec::with_capacity(splits.len());

        for &split in splits {
            let place = (pool.position_of(split) - start).dot(run) / length_squared;

            if place > 0.0 && place < 1.0 {
                along.push((place, split));
            }
        }

        // C#'s `List.Sort` on the place alone; two splits at exactly one place are one vertex in
        // every fixture, and the vertex index breaks the tie so the order never depends on it.
        along.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));

        for (_, vertex) in along {
            ring.push(vertex);
        }
    }

    surface_ring::without_repeats(&ring)
}

/// C# `Assemble`: the new mesh, with the vertices no face uses any more dropped and everything
/// renumbered - the walked-back corners of the gap are nobody's now, and an unused vertex still
/// counts in a bounding box. No `CleanAndMerge`: every vertex the rewrite shares was shared by
/// index as it was made (`SurfaceVertexPool`), so there is nothing left to weld and nothing to
/// renumber blind.
///
/// `tangent_runs`, `rail_continuations` and `band_corners` are carried into the mesh's own
/// numbering in place, as are the withheld features' rings, creases and withins.
pub fn assemble(
    pool: &SurfaceVertexPool,
    faces: &[(i32, i32, i32)],
    tangent_runs: &mut SurfaceWalkbackRuns,
    withheld: &[SurfaceStandingFeatureWithheld],
    rail_continuations: Option<&mut SurfaceWalkbackRuns>,
    band_corners: Option<&mut SurfaceWalkbackRuns>,
) -> Mesh {
    let pool_vertices = pool.vertices();
    let mut renumbered = vec![-1i32; pool_vertices.len()];
    let mut vertices: Vec<Vector3Float> = Vec::with_capacity(pool_vertices.len());

    let keep = |vertex: i32, renumbered: &mut Vec<i32>, vertices: &mut Vec<Vector3Float>| {
        if renumbered[vertex as usize] < 0 {
            renumbered[vertex as usize] = vertices.len() as i32;
            vertices.push(pool_vertices[vertex as usize]);
        }
    };

    for &(v0, v1, v2) in faces {
        for vertex in [v0, v1, v2] {
            keep(vertex, &mut renumbered, &mut vertices);
        }
    }

    // A withheld region contributes no face here, so none of its ring would survive the gathering
    // above - and its extension is built from exactly those points. They are kept by name before
    // the mesh is made, which is the only place a vertex may be added without a face to use it.
    for feature in withheld {
        for face in &feature.faces {
            if let Some(ring) = face.ring.borrow().as_ref() {
                for &vertex in ring {
                    keep(vertex, &mut renumbered, &mut vertices);
                }
            }
        }
    }

    let mut mesh = Mesh::new();

    for &(v0, v1, v2) in faces {
        mesh.faces.add(
            renumbered[v0 as usize],
            renumbered[v1 as usize],
            renumbered[v2 as usize],
            &vertices,
        );
    }

    mesh.vertices = vertices;

    for feature in withheld {
        for face in &feature.faces {
            let carried = face
                .ring
                .borrow()
                .as_ref()
                .map(|ring| ring.iter().map(|&v| renumbered[v as usize]).collect());

            if let Some(carried) = carried {
                *face.ring.borrow_mut() = Some(carried);
            }

            // Every reach's run, not just the first: a feature two bands reached lost its footing
            // on one run of crease per band, and a run left in the pool's numbering would index
            // the mesh by a vertex that is not the one it names.
            let mut creases = face.creases.borrow_mut();
            let mut withins = face.withins.borrow_mut();

            for reach in 0..creases.len() {
                let (from, to) = creases[reach];
                creases[reach] = (renumbered[from as usize], renumbered[to as usize]);

                for step in withins[reach].iter_mut() {
                    *step = renumbered[*step as usize];
                }
            }
        }
    }

    let into_the_meshs_own_numbering = |runs: &mut SurfaceWalkbackRuns| {
        for moved in runs.values_mut() {
            for run in moved.iter_mut() {
                *run = run
                    .iter()
                    .filter(|&&vertex| renumbered[vertex as usize] >= 0)
                    .map(|&vertex| renumbered[vertex as usize])
                    .collect();
            }
        }
    };

    into_the_meshs_own_numbering(tangent_runs);

    // The borrowed-rail continuations are read off the same pool and have to be carried over the
    // same way, or they would name the mesh by vertices that are not the ones they mean
    // (SurfaceWalkbackResult.RailContinuations).
    if let Some(rail_continuations) = rail_continuations {
        into_the_meshs_own_numbering(rail_continuations);
    }

    // And the band corners, for the same reason (SurfaceWalkbackResult.BandCorners).
    if let Some(band_corners) = band_corners {
        into_the_meshs_own_numbering(band_corners);
    }

    mesh
}
