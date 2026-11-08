//! The bead of "Bottom Center Edge.mcx" - one
//! concave crease of the nose-in-notch part beaded at radius 6, whose far end is the knife-edge
//! ridge tip where the picked crease hairpins into the UNSELECTED upper crease beside it.
//!
//! The Onshape control's bead is the exact radius 6 cylinder tangent to the right slope and the
//! lower nose face, trimmed at EACH end by the plane of the one face beyond that end: the left
//! slope `y = x` where the crease dies on the notch's centre crease, and the UPPER nose face's
//! plane at the ridge tip. So the upper nose face GAINS the region between the ridge and the trim
//! curve, while at the centre crease the left slope GIVES the section up and has nothing within
//! 1.2 mm of the crease's old corner (`surface_end_gives_up_the_section`, pinned by the second
//! test).

use std::sync::Arc;

use solidmesh::bevel::surface::{
    surface_bevel_builder, surface_fan, surface_mesh_closure, SurfaceBevelPlan,
};
use solidmesh_math::Vector3;
use solidmesh_mesh::{CancelToken, Mesh};

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::nose_in_notch_test_meshes;
use crate::surface_walkback_checks::{corners, regions_of};

/// The fixture's own radius and tessellation.
const RADIUS: f64 = 6.0;

const SEGMENTS: i32 = 8;

/// How near a named corner the mesh has to come. The four below are exact - they are
/// intersections of planes and tangent lines, not samples of an arc - so this is only float
/// storage and the control's own four decimals.
const NEAR: f64 = 1e-3;

/// Where the bead's rail on the right slope crosses the UPPER crease's line, 5.138 mm beyond the
/// ridge tip at (15, 5, 10): the far corner of the bead, and the control's (18.287, 1.713, 7.8087).
fn past_the_tip() -> Vector3 {
    Vector3::new(18.287242, 1.713143, 7.808801)
}

/// Where the bead's rail on the lower nose face crosses the ridge itself - the control's
/// (9.8429, 5, 10), the near corner of the same end.
fn on_the_ridge() -> Vector3 {
    Vector3::new(9.843153, 5.0, 10.0)
}

/// The far end's two corners, on the left slope: the control's (10, 10, 2.284) and
/// (7.4215, 7.4215, 8.3857).
fn on_the_centre_crease() -> Vector3 {
    Vector3::new(10.0, 10.0, 2.284083)
}

fn on_the_lower_left_crease() -> Vector3 {
    Vector3::new(7.421708, 7.421708, 8.385774)
}

/// Where the crease's own corner stood before the bead - the point the control has nothing within
/// 1.2 mm of, because its left slope gives the whole section between that corner and the trim
/// conic UP rather than keeping it.
fn the_old_corner() -> Vector3 {
    nose_in_notch_test_meshes::lower_crease_end()
}

/// How far from [`the_old_corner`] the nearest vertex has to be: 1.0 mm, comfortably inside the
/// control's own 1.2 and far outside anything float storage could move a vertex by.
const CLEAR_OF_THE_OLD_CORNER: f64 = 1.0;

#[test]
fn a_bead_running_past_a_knife_edge_tip_is_trimmed_by_the_face_across_the_unselected_crease() {
    let built = the_lower_right_crease_beaded(
        "the bead runs out past the ridge tip onto the upper nose face",
    );

    assert_eq!(
        surface_mesh_closure::open_edge_count(&built).0,
        0,
        "a bead trimmed by the face beyond its end leaves a closed solid"
    );
    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);

    assert!(
        signed_volume(&built) > nose_in_notch_test_meshes::VOLUME,
        "a concave bead adds material"
    );

    for (name, at) in [
        ("past the tip", past_the_tip()),
        ("on the ridge", on_the_ridge()),
        ("on the centre crease", on_the_centre_crease()),
        ("on the lower left crease", on_the_lower_left_crease()),
    ] {
        let nearest = nearest_to(&built, at);

        assert!(
            nearest < NEAR,
            "the control has the bead's corner {name} at {at:?}, and the nearest vertex is \
             {nearest:.5} away"
        );
    }
}

#[test]
fn the_end_dying_on_the_centre_crease_gives_the_section_up_rather_than_capping_it() {
    let built = the_lower_right_crease_beaded("the bead builds");

    assert_eq!(
        surface_mesh_closure::open_edge_count(&built).0,
        0,
        "taking the section off the left slope leaves the solid as closed as adding it did"
    );
    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);

    let mut flat = 0;

    for face in 0..built.faces.len() as i32 {
        let corners = corners(&built, face);

        if (corners[1] - corners[0])
            .cross(corners[2] - corners[0])
            .length()
            <= surface_fan::SMALLEST_TRIANGLE
        {
            flat += 1;
        }
    }

    assert_eq!(
        flat, 0,
        "a subtraction that leaves needles has cut in the wrong place"
    );

    // The trim conic's own two corners, which the cap already got right and the subtraction must
    // keep.
    for (name, at) in [
        ("on the centre crease", on_the_centre_crease()),
        ("on the lower left crease", on_the_lower_left_crease()),
    ] {
        let nearest = nearest_to(&built, at);

        assert!(
            nearest < NEAR,
            "the control has the trim conic's corner {name} at {at:?}, and the nearest vertex is \
             {nearest:.5} away"
        );
    }

    let still_there = nearest_to(&built, the_old_corner());

    assert!(
        still_there > CLEAR_OF_THE_OLD_CORNER,
        "the control has nothing within 1.2 mm of the crease's old corner {:?}, and this has one \
         {still_there:.5} away",
        the_old_corner()
    );
}

/// The nose-in-notch part's lower right crease, beaded at [`RADIUS`].
fn the_lower_right_crease_beaded(why: &str) -> Mesh {
    let part = nose_in_notch_test_meshes::create();
    let (regions, _) = regions_of(&Arc::new(part.clone()));
    let regions = Arc::new(regions);
    let (chains, names) = nose_in_notch_test_meshes::creases(&part);

    assert_eq!(names[3], "lower right");

    let plan = SurfaceBevelPlan::try_create(&chains[3], &regions, RADIUS, true)
        .unwrap_or_else(|reason| panic!("{:?}", reason.code()));

    surface_bevel_builder::try_build_one(
        &part,
        &part.get_mesh_edge_graph(),
        &regions,
        plan,
        SEGMENTS,
        &CancelToken::new(),
    )
    .unwrap_or_else(|reason| panic!("{why}: {reason:?}"))
}

/// The distance from `at` to the nearest vertex of `mesh`.
fn nearest_to(mesh: &Mesh, at: Vector3) -> f64 {
    mesh.vertices
        .iter()
        .map(|&vertex| (Vector3::from_vector3_float(vertex) - at).length())
        .fold(f64::INFINITY, f64::min)
}
