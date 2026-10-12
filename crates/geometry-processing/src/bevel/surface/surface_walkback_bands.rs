//! The two questions the face walkback (`SurfaceFaceWalkback`) asks about a BAND - how far across
//! its region one cut is allowed to reach ([`limit_each_cut_to_the_band_its_own_edge_reaches`]),
//! and whether anything is standing where it is about to reach
//! ([`try_check_nothing_is_standing_in_the_band`]). Ports the band half of
//! `Bevel/Surface/SurfaceWalkbackBands.cs`; the reading it takes afterwards of what each cut left
//! behind as a boundary to sew to (`GatherTangentRuns` and its helpers) is
//! `surface_walkback_tangent_runs`. A region under the knife is [`SurfaceRegionWalkback`].
//!
//! The regions' face sets are walked in the REGION's own face order rather than over the
//! `HashSet` [`SurfaceRegionWalkback::faces`]: C#'s `HashSet<int>` built from that list iterates
//! in insertion order, and the order reaches a float sum ([`normal_of`]).

use std::collections::{HashMap, HashSet};

use indexmap::IndexMap;
use polygon_mesh::{Face, Mesh};
use vector_math::Vector3;

use super::surface_cut_clip;
use super::{SurfaceFacetCut, SurfaceLine, SurfaceRegionWalkback, SurfaceVertexPool};
use crate::bevel::bevel_chain_linker;
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode, BevelSweptChain};

/// C# `LimitEachCutToTheBandItsOwnEdgeReaches`: which of a region's facets each cut is allowed to
/// take material off - the ones the band starting at the chain edge itself can spread to across
/// material the cut removes - appended to each walkback's `reaches` in cut order.
///
/// The half space a cut is written as has no end (`SurfaceFacetCut`), and on a plain face that is
/// right - one picked edge of a cube's top walks the whole top back, rim to rim. On a face with a
/// notch bitten out of its side it is wrong: the same half space also reaches the material on the
/// FAR side of the notch and takes a second band off it, which no strip goes anywhere near.
///
/// The band the edge really asks for is the material joined to the edge. Two facets' removed
/// pieces touch exactly when the shared edge between them has an end on the removed side, so the
/// spread is a flood over facets with that as its one condition. On anything without a notch every
/// facet of the band is reached and nothing changes.
pub fn limit_each_cut_to_the_band_its_own_edge_reaches(
    mesh: &Mesh,
    walkbacks: &mut IndexMap<i32, SurfaceRegionWalkback>,
    pool: &SurfaceVertexPool,
) {
    let on_the_cut = surface_cut_clip::on_the_cut(pool);

    for walkback in walkbacks.values_mut() {
        let mut faces_on: HashMap<(i32, i32), Vec<i32>> = HashMap::new();

        for &face_index in walkback.region.faces() {
            for (first, second) in corner_pairs(&mesh.faces[face_index as usize]) {
                let key = (first.min(second), first.max(second));
                faces_on
                    .entry(key)
                    .or_insert_with(|| Vec::with_capacity(2))
                    .push(face_index);
            }
        }

        let mut all_reaches = Vec::with_capacity(walkback.cuts.len());

        for cut in &walkback.cuts {
            let mut reaches = HashSet::new();
            let mut walking: Vec<i32> = Vec::new();

            if walkback.faces.contains(&cut.seed_face) {
                walking.push(cut.seed_face);
            }

            for also_seeded in the_facets_the_bands_far_side_runs_through(mesh, walkback, cut, pool)
            {
                walking.push(also_seeded);
            }

            while let Some(at) = walking.pop() {
                if !reaches.insert(at) {
                    continue;
                }

                for (first, second) in corner_pairs(&mesh.faces[at as usize]) {
                    if !cut.removes_part_of(
                        pool.position_of(first),
                        pool.position_of(second),
                        on_the_cut,
                    ) {
                        // Nothing of this shared edge is given up, so the two facets' pieces do
                        // not touch along it and the band does not cross here. A band with ends
                        // gives up nothing outside them, which is what stops one picked facet of
                        // a cylinder's wall from carrying the cut round the rest of it.
                        continue;
                    }

                    if let Some(neighbours) = faces_on.get(&(first.min(second), first.max(second)))
                    {
                        for &neighbour in neighbours {
                            if neighbour != at {
                                walking.push(neighbour);
                            }
                        }
                    }
                }
            }

            if let Some(may_touch) = &cut.may_touch {
                // A rolled cut says which facets of the wall are its own; the flood can only take
                // material off those (SurfaceFacetCut.MayTouch).
                reaches.retain(|face| may_touch.contains(face));
            }

            all_reaches.push(reaches);
        }

        walkback.reaches.extend(all_reaches);
    }
}

/// C# `TheFacetsTheBandsFarSideRunsThrough`: the facets of the region the band's FAR SIDE runs
/// across - the stretch of the cut's tangent line between its two band end planes - which are
/// seeded into the flood beside `SurfaceFacetCut::seed_face`.
///
/// The seed facet is not enough on a loop whose setback steps round it: a hole mouth drilled at a
/// rake sets back by 0.97 at one end of the loop and 0.17 at the other, the mitre point then stands
/// about half a chord past the corner, the band is a slab over the NEXT chord along, and a flood
/// from the seed alone never leaves it. Only where the stretch is bounded BOTH ways: a cut with a
/// free end is an endless half space on purpose, and seeding along an endless line would put the
/// cut back on the far side of every notch and hole (`SurfaceWalkbackGuardTests`).
fn the_facets_the_bands_far_side_runs_through(
    mesh: &Mesh,
    walkback: &SurfaceRegionWalkback,
    cut: &SurfaceFacetCut,
    pool: &SurfaceVertexPool,
) -> Vec<i32> {
    let Some((from, to)) = try_the_stretch_between_the_band_ends(cut, pool) else {
        return Vec::new();
    };

    walkback
        .region
        .faces()
        .iter()
        .copied()
        .filter(|&face_index| {
            the_stretch_crosses_the_facet(
                &mesh.faces[face_index as usize],
                &cut.tangent_line,
                from,
                to,
                pool,
            )
        })
        .collect()
}

/// C# `TryTheStretchBetweenTheBandEnds`: the window of the cut's tangent line that lies inside
/// every one of its band ends, or `None` where there is no such window or it runs off to infinity
/// at either end.
fn try_the_stretch_between_the_band_ends(
    cut: &SurfaceFacetCut,
    pool: &SurfaceVertexPool,
) -> Option<(f64, f64)> {
    let mut from = f64::NEG_INFINITY;
    let mut to = f64::INFINITY;

    for end in &cut.ends {
        let at_the_point = (cut.tangent_line.point - end.point).dot(end.inward);
        let slope = cut.tangent_line.direction.dot(end.inward);

        if slope.abs() < 1e-12 {
            // The line runs ALONG this end plane, so it is inside it everywhere or nowhere.
            if at_the_point < -pool.near_enough() {
                return None;
            }

            continue;
        }

        let crossing = -at_the_point / slope;

        if slope > 0.0 {
            from = from.max(crossing);
        } else {
            to = to.min(crossing);
        }
    }

    (!from.is_infinite() && !to.is_infinite() && to > from).then_some((from, to))
}

/// C# `TheStretchCrossesTheFacet`: whether the stretch `from`..`to` of `line` passes through a
/// facet - the segment clipped by the facet's three sides in the facet's own plane, with what
/// survives required to lie in that plane.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)`, which refuses a NaN as well.
fn the_stretch_crosses_the_facet(
    face: &Face,
    line: &SurfaceLine,
    mut from: f64,
    mut to: f64,
    pool: &SurfaceVertexPool,
) -> bool {
    let corners = [
        pool.position_of(face.v0),
        pool.position_of(face.v1),
        pool.position_of(face.v2),
    ];

    let mut normal = (corners[1] - corners[0]).cross(corners[2] - corners[0]);

    if !(normal.length_squared() > 0.0) {
        return false;
    }

    normal = normal.get_normal();

    for side in 0..3 {
        let at = corners[side];
        let mut facing = normal.cross(corners[(side + 1) % 3] - at);

        if !(facing.length_squared() > 0.0) {
            return false;
        }

        facing = facing.get_normal();

        if facing.dot(corners[(side + 2) % 3] - at) < 0.0 {
            facing = -facing;
        }

        let at_the_point = (line.point - at).dot(facing);
        let slope = line.direction.dot(facing);

        if slope.abs() < 1e-12 {
            if at_the_point < -pool.near_enough() {
                return false;
            }

            continue;
        }

        // Inside by the pool's own nearness rather than strictly, so a stretch running down the
        // very side of a facet counts as through it - the same ruler every other reading of this
        // phase is taken with.
        let crossing = (-at_the_point - pool.near_enough()) / slope;

        if slope > 0.0 {
            from = from.max(crossing);
        } else {
            to = to.min(crossing);
        }
    }

    if !(to > from) {
        return false;
    }

    // And in the facet's own plane: the tangent line lies in the region's plane, so a facet of a
    // region that is really one surface holds it exactly. Read on the middle of what survived the
    // clip, which is the deepest the stretch gets into this facet.
    let middle = line.point + (line.direction * ((from + to) / 2.0));

    (middle - corners[0]).dot(normal).abs() <= pool.near_enough()
}

/// C# `TryCheckNothingIsStandingInTheBand`: whether anything is STANDING on a face where its band
/// is about to be taken off - a boss, a rib, the mouth of a bore. The footprint of such a feature is
/// a hole in the region, an inner loop of its rim, and a band that reaches that loop takes away the
/// material the feature's walls stand on. Refused rather than handled (rung 9 of the plan).
///
/// Asked of each rim EDGE, not of its corners: a cut with band ends removes a convex box, and a
/// segment crosses one with both of its ends outside it whenever they are outside by different
/// faces. Asked of the BAND, not of the half space (`try_find_removed_point_in_the_band`), so a hole
/// behind the band in a face that wraps round a bore is no part of it. And a loop that is the chain
/// of the plan whose cut reached it - a boss's foot walked outward from - is exempt
/// ([`stands_on_a_chain_being_bevelled`]).
///
/// Which rim loop is the outside is read by area in the region's own plane - the biggest loop is
/// the outline and the rest are holes - which is exact on the planar regions this phase cuts.
pub fn try_check_nothing_is_standing_in_the_band(
    mesh: &Mesh,
    walkbacks: &IndexMap<i32, SurfaceRegionWalkback>,
    pool: &SurfaceVertexPool,
) -> Result<(), BevelAdvisory> {
    let on_the_cut = surface_cut_clip::on_the_cut(pool);

    for walkback in walkbacks.values() {
        let loops = bevel_chain_linker::link(walkback.region.boundary_edges());

        if loops.len() < 2 {
            continue;
        }

        let normal = normal_of(mesh, walkback.region.faces(), pool);

        let mut widest = usize::MAX;
        let mut widest_area = f64::NEG_INFINITY;

        for (index, chain) in loops.iter().enumerate() {
            let area = area_of(chain, normal);

            if area > widest_area {
                widest_area = area;
                widest = index;
            }
        }

        for (index, chain) in loops.iter().enumerate() {
            if index == widest {
                continue;
            }

            for cut in &walkback.cuts {
                for edge in &chain.edges {
                    let (from, to) = (edge.from(), edge.to());

                    let Some(inside) = cut.try_find_removed_point_in_the_band(from, to, on_the_cut)
                    else {
                        continue;
                    };

                    if stands_on_a_chain_being_bevelled(walkback, cut, from, to, pool.near_enough())
                    {
                        continue;
                    }

                    // Named by a corner of the rim where there is one to name, which is the place
                    // a reader looks for, and by the stretch itself where the band crosses the edge
                    // between its two ends.
                    let met = if cut.removes_from_the_band(from, on_the_cut) {
                        from
                    } else if cut.removes_from_the_band(to, on_the_cut) {
                        to
                    } else {
                        inside
                    };

                    return Err(BevelAdvisory::new(
                        BevelAdvisoryCode::SurfaceCutMeetsStandingFeature,
                        vec![met.into()],
                    ));
                }
            }
        }
    }

    Ok(())
}

/// C# `StandsOnAChainBeingBevelled`: whether a whole rim edge lies along one chain edge of the
/// plan `reaching` was made for - the exemption a boss's foot needs, its wall being the chain's
/// other face walked back and sewn to the same strip. Of the SAME PLAN, so another plan's band
/// into the foot is not excused; and one chain edge holds BOTH ends, so the answer is about the
/// removed stretch rather than two points.
fn stands_on_a_chain_being_bevelled(
    walkback: &SurfaceRegionWalkback,
    reaching: &SurfaceFacetCut,
    from: Vector3,
    to: Vector3,
    near: f64,
) -> bool {
    walkback.cuts.iter().any(|cut| {
        cut.plan_index == reaching.plan_index
            && cut.stands_on_the_chain_edge(from, near)
            && cut.stands_on_the_chain_edge(to, near)
    })
}

/// C# `NormalOf`: the region's own facing, as the sum of its faces' normals, `faces` in the order
/// C#'s set iterates them.
pub fn normal_of(mesh: &Mesh, faces: &[i32], pool: &SurfaceVertexPool) -> Vector3 {
    let mut summed = Vector3::ZERO;

    for &face_index in faces {
        let face = &mesh.faces[face_index as usize];
        let from = pool.position_of(face.v0);

        summed += (pool.position_of(face.v1) - from).cross(pool.position_of(face.v2) - from);
    }

    if summed.length_squared() > 0.0 {
        summed.get_normal()
    } else {
        Vector3::UNIT_Z
    }
}

/// C# `AreaOf`: how much a closed rim loop encloses, seen along `normal`.
pub fn area_of(chain: &BevelSweptChain, normal: Vector3) -> f64 {
    let mut summed = Vector3::ZERO;

    for edge in &chain.edges {
        summed += edge.from().cross(edge.to());
    }

    summed.dot(normal).abs() / 2.0
}

/// C# `CornerPairs`: the three corner pairs of a face, in its own winding.
pub fn corner_pairs(face: &Face) -> [(i32, i32); 3] {
    [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)]
}
