//! Port of `SurfaceBoreMouthTests.cs`, the STRAIGHT half of the partial class
//! `SurfaceBoreMouthTests`: the mouth of a straight round bore, rounded all the way round - the
//! first convex INNER loop the surface corpus builds, whose band grows outward across the face from
//! a hole in it. The blend is a rolling ball's surface facet by facet: each rim edge has its own
//! centre line, parallel to its own chord, a radius below the top face and a radius outward from its
//! own facet's plane.
//!
//! Also holds the class's build-side readings that the raked half declares
//! (`AssertNothingStandsInTheBore`, `TheRakedBoresFacetPlanes`,
//! `AssertEveryRakedBlendVertexStandsARadiusFromItsOwnCentreLine`, `RakedCentreLines`,
//! `TheUpperRakedMouth`), which the other raked-bore classes call. The class's constants and the
//! raked mouth's walkback are in `surface_raked_bore_mouth_tests.rs`.

#![allow(dead_code)]

use geometry_processing::bevel::surface::{surface_mesh_closure, SurfaceLine, SurfaceVertexPool};
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelSelectionKind, FaceGroup, FeatureEdge, FeatureEdgeSet,
};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::bore_test_meshes;
use crate::mesh_measurements::shell_count;
use crate::surface_raked_bore_mouth_tests::{
    raked_axis, AS_NEAR_AS_A_FLOAT_HOLDS, BEND_THRESHOLD_DEGREES, RAKED_RADIUS, RAKED_THICKNESS,
    SEGMENTS, SIDES, SIZE,
};
use crate::surface_test_build::{expect_built, try_bevel, TryBevelOptions};

/// The straight fixture's bore radius.
pub const BORE_RADIUS: f64 = 3.0;

/// The straight fixture's plate thickness.
pub const THICKNESS: f64 = 10.0;

/// A 0.4 round on the whole of a straight bore's upper mouth: forty edges, one setback, and a loop
/// that closes on itself with no end anywhere.
#[test]
fn a_straight_bores_mouth_is_rounded_all_the_way_round() {
    let part = bore_test_meshes::create_plate_with_a_round_bore(SIDES, BORE_RADIUS, THICKNESS, 6.0);
    let features = FeatureEdgeSet::create_with_graph(
        &part,
        &part.get_mesh_edge_graph(),
        BEND_THRESHOLD_DEGREES,
    );
    let mouth = the_upper_mouth(&features);

    let group = bevel_edge_group_resolver::create_from_edge(
        &features,
        mouth,
        BevelSelectionKind::TangentChain,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("the mouth's edge is in the set");

    let (resolutions, _) =
        bevel_edge_group_resolver::resolve_all(std::slice::from_ref(&group), &part);
    assert_eq!(
        resolutions[0].edges.len(),
        SIDES as usize,
        "a tangent chain seeded on the mouth walks the whole mouth and stops there - instead {}",
        resolutions[0].edges.len()
    );

    let mesh = expect_built(
        try_bevel(
            &part,
            &[group],
            SIZE,
            SEGMENTS,
            &CancelToken::new(),
            TryBevelOptions::default(),
        ),
        "a hole mouth is a convex inner loop and nothing more",
    );

    let (open, open_at) = surface_mesh_closure::open_edge_count(&mesh);
    assert_eq!(
        open, 0,
        "a loop all the way round leaves a closed solid, and this one is open at {open_at:?}"
    );

    assert!(mesh.is_manifold());
    assert_eq!(shell_count(&mesh), 1);

    assert_every_blend_vertex_stands_a_radius_from_its_own_centre_line(&features, &mesh);
    assert_no_two_vertices_are_the_same_place_read_twice(&part, &mesh);
}

/// That the mouth reads every place ONCE: no two published vertices stand farther apart than the
/// pool's weld and nearer than its ruler. A pair in that band is one point two constructions each
/// wrote their own version of, the shape of a crack no later reading can find.
pub fn assert_no_two_vertices_are_the_same_place_read_twice(part: &Mesh, built: &Mesh) {
    let weld = SurfaceVertexPool::weld_distance_for(&part.vertices);
    let ruler = SurfaceVertexPool::near_enough_for(&part.vertices);
    let at: Vec<Vector3> = built.vertices.iter().map(|v| v.as_vector3()).collect();
    let mut twins = 0;
    let mut worst = 0.0_f64;

    for one in 0..at.len() {
        for other in (one + 1)..at.len() {
            let apart = (at[one] - at[other]).length();

            if apart > weld && apart < ruler {
                twins += 1;
                worst = worst.max(apart);
            }
        }
    }

    assert_eq!(
        twins, 0,
        "one place is read once, and {twins} pairs stand between the weld {weld:e} and the ruler \
         {ruler:e}, the worst of them {worst:e} apart"
    );
}

/// That the blend is a rolling ball's surface facet by facet - each rim edge's own centre line, a
/// radius below the top face and a radius outward from its own facet's plane.
pub fn assert_every_blend_vertex_stands_a_radius_from_its_own_centre_line(
    features: &FeatureEdgeSet,
    built: &Mesh,
) {
    let top = THICKNESS / 2.0;

    let lines = centre_lines(features);

    assert_eq!(
        lines.len(),
        SIDES as usize,
        "the mouth of a forty sided bore is forty chords"
    );

    // The blend's own vertices: below the top face's tangent ring and above the wall's, which on a
    // right angled mouth are a radius apart in z.
    let blend: Vec<Vector3> = built
        .vertices
        .iter()
        .map(|v| v.as_vector3())
        .filter(|at| {
            at.z < top - AS_NEAR_AS_A_FLOAT_HOLDS && at.z > top - SIZE + AS_NEAR_AS_A_FLOAT_HOLDS
        })
        .collect();

    assert!(
        !blend.is_empty(),
        "a chorded round has rows between its two tangent rings"
    );

    for at in blend {
        // How far the BEST line misses the radius by, not how near the nearest is: which line a
        // vertex belongs to is what the strip knows and a measurement of the mesh does not.
        let missed_by = best_miss(&lines, at);

        assert!(
            missed_by.abs() <= AS_NEAR_AS_A_FLOAT_HOLDS,
            "{at:?} is on the blend, so it stands a radius from the centre line of the edge it \
             belongs to (missed by {missed_by})"
        );
    }
}

/// How far the best of the lines misses a radius at `at`: C#'s `lines.Min(|DistanceTo - Size|)`.
fn best_miss(lines: &[SurfaceLine], at: Vector3) -> f64 {
    lines
        .iter()
        .map(|line| (line.distance_to(at) - SIZE).abs())
        .fold(f64::INFINITY, f64::min)
}

/// The forty lines the rolling ball's centre runs along, one per mouth edge: parallel to that
/// edge's own chord, a radius below the top face and a radius OUTWARD from its own facet.
pub fn centre_lines(features: &FeatureEdgeSet) -> Vec<SurfaceLine> {
    the_upper_mouth_edges(features)
        .into_iter()
        .map(|edge| {
            let middle = (edge.start + edge.end) / 2.0;

            // The chord's own midpoint stands at the facet's apothem from the axis, so a step of
            // one more radius straight out from the axis through it is a radius clear of that
            // facet's plane.
            let outward = Vector3::new(middle.x, middle.y, 0.0);

            SurfaceLine::new(
                outward.get_normal() * (outward.length() + SIZE)
                    - Vector3::new(0.0, 0.0, SIZE - (THICKNESS / 2.0)),
                (edge.end - edge.start).get_normal(),
            )
        })
        .collect()
}

/// The index of one edge of the upper mouth, to pick the tangent chain by.
pub fn the_upper_mouth(features: &FeatureEdgeSet) -> i32 {
    (0..features.count())
        .find(|&i| is_the_upper_mouth(&features.edges()[i]))
        .map(|i| i as i32)
        .expect("the fixture has no upper mouth")
}

/// Every edge of the upper mouth, which is what the chain resolves to.
pub fn the_upper_mouth_edges(features: &FeatureEdgeSet) -> Vec<FeatureEdge> {
    features
        .edges()
        .iter()
        .filter(|edge| is_the_upper_mouth(edge))
        .cloned()
        .collect()
}

/// An edge of the upper mouth: both ends on the top face and both a bore radius from the axis,
/// which the outer rim's own edges are not.
pub fn is_the_upper_mouth(edge: &FeatureEdge) -> bool {
    on_the_mouth(edge.start) && on_the_mouth(edge.end)
}

pub fn on_the_mouth(at: Vector3) -> bool {
    (at.z - (THICKNESS / 2.0)).abs() < 1e-6
        && (Vector3::new(at.x, at.y, 0.0).length() - BORE_RADIUS).abs() < 1e-4
}

/// A bevel only ever takes material away, so nothing it publishes may stand inside the drilled
/// hole. Read against the drilled PRISM and not the drill's cylinder, because the hole is forty
/// flats: a point is in the hole when it stands inside EVERY facet plane at once.
pub fn assert_nothing_stands_in_the_bore(features: &FeatureEdgeSet, built: &Mesh) {
    let facets = the_raked_bores_facet_planes(features);

    assert_eq!(facets.len(), SIDES as usize);

    for at in built.vertices.iter().map(|v| v.as_vector3()) {
        // Inside the hole means inside EVERY flat at once: a point on a flat reads zero there, a
        // point in the material reads negative on at least one, and only a point in the hole reads
        // positive on all forty.
        let how_far_in = facets
            .iter()
            .map(|(point, normal)| (at - *point).dot(*normal))
            .fold(f64::INFINITY, f64::min);

        assert!(
            how_far_in <= AS_NEAR_AS_A_FLOAT_HOLDS,
            "{at:?} stands inside the bore ({how_far_in}), which the bevel cannot have put material \
             back into"
        );
    }
}

/// The forty flats the raked bore is really drilled as, each a point on it and its outward normal,
/// outward being out of the material and INTO the hole. Read off the part's OWN mouth chords
/// rather than built from the radius and the rake, because the polygon's phase is the fixture's.
pub fn the_raked_bores_facet_planes(features: &FeatureEdgeSet) -> Vec<(Vector3, Vector3)> {
    let axis = raked_axis();

    the_upper_raked_mouth_edges(features)
        .into_iter()
        .map(|edge| {
            let middle = (edge.start + edge.end) / 2.0;
            let mut normal = (edge.end - edge.start)
                .get_normal()
                .cross(axis)
                .get_normal();

            if normal.dot(middle - (axis * middle.dot(axis))) > 0.0 {
                // Pointed OUT of the material, which is toward the axis rather than away from it.
                normal = -normal;
            }

            (middle, normal)
        })
        .collect()
}

/// The raked blend, read the same way as the straight one but with the centre line solved for:
/// the drop and the step out are `a = r / (1 - n.z)` and `b = r + a n.z` for that facet's own
/// outward normal. Everything between the top face and a radius below it, MINUS what stands on the
/// bore's own wall - the rim the blend lands on, which steps down the creases between chords and is
/// on no bead (`SurfaceRakedBoreBandOutlineTests` reads where it stands).
pub fn assert_every_raked_blend_vertex_stands_a_radius_from_its_own_centre_line(
    features: &FeatureEdgeSet,
    built: &Mesh,
) {
    let top = RAKED_THICKNESS / 2.0;

    let lines = raked_centre_lines(features);

    assert_eq!(lines.len(), SIDES as usize);

    let facets = the_raked_bores_facet_planes(features);

    let blend: Vec<Vector3> = built
        .vertices
        .iter()
        .map(|v| v.as_vector3())
        .filter(|at| {
            at.z < top - AS_NEAR_AS_A_FLOAT_HOLDS && at.z > top - SIZE + AS_NEAR_AS_A_FLOAT_HOLDS
        })
        .filter(|at| {
            !facets.iter().any(|(point, normal)| {
                (*at - *point).dot(*normal).abs() <= AS_NEAR_AS_A_FLOAT_HOLDS
            })
        })
        .collect();

    // Pinned so that taking the wall out cannot quietly take the reading with it: 1175 of the
    // band's 1301 vertices stand off every facet and are read here, and the 126 on the wall are its
    // rim.
    assert_eq!(
        blend.len(),
        1175,
        "the blend proper is what this reads, and the rim it lands on is the wall's"
    );

    for at in blend {
        let missed_by = best_miss(&lines, at);

        assert!(
            missed_by.abs() <= AS_NEAR_AS_A_FLOAT_HOLDS,
            "{at:?} is on the blend, so it stands a radius from the centre line of the edge it \
             belongs to (missed by {missed_by})"
        );
    }
}

/// The forty lines the rolling ball's centre runs along round the raked mouth, each read off its
/// own chord and its own facet.
pub fn raked_centre_lines(features: &FeatureEdgeSet) -> Vec<SurfaceLine> {
    let axis = raked_axis();

    the_upper_raked_mouth_edges(features)
        .into_iter()
        .map(|edge| {
            let middle = (edge.start + edge.end) / 2.0;
            let chord = (edge.end - edge.start).get_normal();

            // The facet's own plane contains the chord and the drill's axis, so its normal is their
            // cross - turned to face AWAY from the axis, which is the side the material is on.
            let mut facet = chord.cross(axis).get_normal();
            let outward = middle - (axis * middle.dot(axis));

            if facet.dot(outward) < 0.0 {
                facet = -facet;
            }

            // A radius below z = top and a radius out from that plane at once: drop a, step out b.
            let drop = SIZE / (1.0 - facet.z);
            let step_out = SIZE + (drop * facet.z);

            SurfaceLine::new(
                middle - Vector3::new(0.0, 0.0, drop) + (facet * step_out),
                chord,
            )
        })
        .collect()
}

pub fn the_upper_raked_mouth(features: &FeatureEdgeSet) -> i32 {
    (0..features.count())
        .find(|&i| {
            on_the_raked_mouth(features.edges()[i].start)
                && on_the_raked_mouth(features.edges()[i].end)
        })
        .map(|i| i as i32)
        .expect("the raked fixture has no upper mouth")
}

pub fn the_upper_raked_mouth_edges(features: &FeatureEdgeSet) -> Vec<FeatureEdge> {
    features
        .edges()
        .iter()
        .filter(|edge| on_the_raked_mouth(edge.start) && on_the_raked_mouth(edge.end))
        .cloned()
        .collect()
}

/// On the upper mouth: on the top face, and a bore radius from the drill's own axis - which the
/// plate's outer rim is not.
pub fn on_the_raked_mouth(at: Vector3) -> bool {
    let axis = raked_axis();

    (at.z - (RAKED_THICKNESS / 2.0)).abs() < 1e-6
        && ((at - (axis * at.dot(axis))).length() - RAKED_RADIUS).abs() < 1e-4
}
