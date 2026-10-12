//! Port of `SurfaceRakedBoreMouthClaimTests.cs`, a part of the partial class
//! `SurfaceBoreMouthTests`: the raked bore's claim - a 0.4 round on the whole upper mouth of a bore
//! leaned 45 degrees through a 3 mm plate is closed, manifold, one shell, stands nowhere inside the
//! drilled hole, and every blend vertex stands a radius from its own chord's centre line. The C#
//! file's two handoff comments (the order of the two crossings on one chord, the patch fanned from
//! the far corner, a fan wound once for its whole ring) record how the mouth closed; the readings
//! are in `surface_bore_mouth_tests.rs`, the constants in `surface_raked_bore_mouth_tests.rs`.

use geometry_processing::bevel::surface::surface_mesh_closure;
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelSelectionKind, FaceGroup, FeatureEdgeSet,
};
use polygon_mesh::CancelToken;

use crate::mesh_measurements::shell_count;
use crate::surface_bore_mouth_tests::{
    assert_every_raked_blend_vertex_stands_a_radius_from_its_own_centre_line,
    assert_nothing_stands_in_the_bore, the_upper_raked_mouth,
};
use crate::surface_raked_bore_mouth_tests::{
    the_raked_part, BEND_THRESHOLD_DEGREES, SEGMENTS, SIDES, SIZE,
};
use crate::surface_test_build::{expect_built, try_bevel, TryBevelOptions};

#[test]
fn a_raked_bores_mouth_is_rounded_all_the_way_round() {
    let part = the_raked_part();
    let features = FeatureEdgeSet::create_with_graph(
        &part,
        &part.get_mesh_edge_graph(),
        BEND_THRESHOLD_DEGREES,
    );
    let group = bevel_edge_group_resolver::create_from_edge(
        &features,
        the_upper_raked_mouth(&features),
        BevelSelectionKind::TangentChain,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("the mouth's edge is in the set");

    assert_eq!(
        bevel_edge_group_resolver::resolve_all(std::slice::from_ref(&group), &part).0[0]
            .edges
            .len(),
        SIDES as usize,
        "a leaned mouth is still one closed loop of forty chords"
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
        "the setback changing round the loop is the mitre's business",
    );

    let (open, open_at) = surface_mesh_closure::open_edge_count(&mesh);
    assert_eq!(
        open, 0,
        "a loop all the way round leaves a closed solid, and this one is open at {open_at:?}"
    );
    assert!(mesh.is_manifold());
    assert_eq!(shell_count(&mesh), 1);

    assert_nothing_stands_in_the_bore(&features, &mesh);
    assert_every_raked_blend_vertex_stands_a_radius_from_its_own_centre_line(&features, &mesh);
}
