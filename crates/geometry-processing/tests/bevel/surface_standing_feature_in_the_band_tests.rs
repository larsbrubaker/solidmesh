//! "Encroaching Side.mcx" in SURFACE mode - a CONVEX band whose footprint on the wall it lands on
//! CONTAINS the foot of a feature standing there.
//!
//! The pick is the chevron's right knife tip - the vertical edge at (20, 0) where the flat face
//! `x = 20` meets the right slope at 45 degrees - and what stands on that slope is the nose, whose
//! ridge runs out onto it 7.071 from the tip edge. A band off a 45 degree tip reaches r/tan 22.5
//! across each of its two faces, which at the fixture's own 3.4 is 8.208: the nose's foot is INSIDE
//! the band. That construction is BUILT - the bite the nose takes out of the strip
//! (`SurfaceStripBittenByAFeature`) - and what is left here is the pair of sizes either side of the
//! foot, which is what says the two readings meet rather than overlap. Built through the shared
//! `SurfaceTestBuild` port.

use solidmesh::bevel::surface::surface_mesh_closure;
use solidmesh::topology::FeatureEdgeSet;
use solidmesh_math::Vector3;
use solidmesh_mesh::CancelToken;

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::nose_in_notch_cut;
use crate::nose_in_notch_test_meshes as nose_in_notch;
use crate::surface_test_build::{try_bevel, TryBevelOptions};

/// The fixture's own radius: reach 3.4/tan 22.5 = 8.208, against the foot's 7.071.
const THE_RADIUS_THAT_COVERS_THE_FOOT: f64 = 3.4;

/// The largest of the sizes measured to clear the foot: 2.9/tan 22.5 = 7.001.
const THE_RADIUS_THAT_CLEARS_IT: f64 = 2.9;

const SEGMENTS: i32 = 8;

/// The knife tip's vertical edge, floor end - the corner (20, 0) of the cube.
fn the_tip_edge_foot() -> Vector3 {
    Vector3::new(nose_in_notch::SIZE, 0.0, 0.0)
}

/// The same edge's ceiling end.
fn the_tip_edge_top() -> Vector3 {
    Vector3::new(nose_in_notch::SIZE, 0.0, nose_in_notch::SIZE)
}

/// THE SIZE THAT COVERS THE FOOT BUILDS: what used to be named there
/// (`SurfaceStandingFeatureInTheBand`) is now the bite the nose takes out of the middle of the
/// strip.
#[test]
fn a_nose_foot_inside_a_convex_band_is_built_rather_than_named() {
    let part = nose_in_notch::create();
    let features = FeatureEdgeSet::create(&part);
    let tip = nose_in_notch_cut::anchor(&features, the_tip_edge_foot(), the_tip_edge_top());

    let outcome = try_bevel(
        &part,
        &[tip],
        THE_RADIUS_THAT_COVERS_THE_FOOT,
        SEGMENTS,
        &CancelToken::new(),
        TryBevelOptions::default(),
    );

    let mesh = match outcome.result {
        Ok(built) => built.mesh,
        Err(reason) => panic!(
            "the band reaches 8.208 across the slope and the nose's foot stands 7.071 out on it, \
             which is a bite out of the strip's middle - instead {reason:?}"
        ),
    };

    let (open, open_at) = surface_mesh_closure::open_edge_count(&mesh);
    assert_eq!(
        open, 0,
        "nothing may be left open, and something is at {open_at:?}"
    );
}

/// AND THE SIZE BESIDE IT STILL BUILDS: at 2.9 the band stops just short of the foot, so the same
/// pick is an ordinary convex strip running the whole height of the tip.
#[test]
fn a_smaller_band_that_clears_the_foot_builds_closed() {
    let part = nose_in_notch::create();
    let features = FeatureEdgeSet::create(&part);
    let tip = nose_in_notch_cut::anchor(&features, the_tip_edge_foot(), the_tip_edge_top());

    let outcome = try_bevel(
        &part,
        &[tip],
        THE_RADIUS_THAT_CLEARS_IT,
        SEGMENTS,
        &CancelToken::new(),
        TryBevelOptions::default(),
    );

    let mesh = match outcome.result {
        Ok(built) => built.mesh,
        Err(reason) => panic!(
            "the band's reach of 7.001 stops short of the foot at 7.071 - instead {reason:?}"
        ),
    };

    let (open, open_at) = surface_mesh_closure::open_edge_count(&mesh);
    assert_eq!(
        open, 0,
        "nothing may be left open, and something is at {open_at:?}"
    );

    assert!(mesh.is_manifold());

    assert_eq!(shell_count(&mesh), 1);

    assert!(
        signed_volume(&mesh) < nose_in_notch::VOLUME,
        "a round on a convex tip takes material away, whatever its size"
    );
}
