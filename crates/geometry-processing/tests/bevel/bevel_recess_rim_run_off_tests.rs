//! The pyramid recess's upper left crease (`pyramid_recess_test_meshes`), planned at 3 mm over 8
//! segments and built through the surface construction with the ROOM's own reading handed to the
//! build - its left wall runs off over a stretch, dies on the knife edge, and the bead turns onto
//! the cube's top and dies past the apex. Every case runs either way up (`mirror_z`) and at either
//! place (the box frame, or where the rebuild has the part).

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_chain_join, surface_mesh_closure, surface_room,
    SurfaceBandRunsOffOverAStretch, SurfaceBevelPlan, SurfaceBuildOptions,
};
use geometry_processing::topology::{FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::csg::manifold_import;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::pyramid_recess_test_meshes as recess;

const RADIUS: f64 = 3.0;

const SEGMENTS: i32 = 8;

/// What the control and the boolean path both measure the beaded part at.
const THE_CONTROLS_VOLUME: f64 = 6346.26;

/// Where rail zero dies on the knife edge - the station the room's own stretch ends at.
fn where_the_wall_dies() -> Vector3 {
    Vector3::new(0.0, 0.0, 17.1306)
}

/// Where the end turns from the mouth plane onto the cube's top face.
fn where_the_end_turns() -> Vector3 {
    Vector3::new(2.0745, 0.0, 20.0)
}

/// Where the bead dies past the apex, on the recess's ridge.
fn where_the_bead_dies_past_the_apex() -> Vector3 {
    Vector3::new(7.2934, 10.0, 10.0316)
}

/// The whole crease, built: closed, manifold, one solid, and a solid the CSG kernel takes as it
/// stands - which is what a Combine after this fillet would hand it.
fn the_crease_builds_and_the_kernel_takes_it(mirror_z: bool, where_the_rebuild_has_the_part: bool) {
    let built = the_crease(mirror_z, where_the_rebuild_has_the_part);

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
    assert_eq!(
        open, 0,
        "every edge is used twice, once each way - instead open at {open_at:?}"
    );

    assert!(built.is_manifold());

    assert_eq!(
        shell_count(&built),
        1,
        "the bead and the part are one solid"
    );

    // The strict import, the same marshalling `surface_result_imports_into_manifold_tests` reads
    // with no weld-and-retry, so a defect in the build cannot be welded away.
    if let Err(rejected) = manifold_import::import_strict(&built) {
        panic!(
            "the kernel takes the published mesh, which a Combine after a Fillet would hand it - \
             {rejected:?}"
        );
    }

    let volume = signed_volume(&built);
    assert!(
        (volume - THE_CONTROLS_VOLUME).abs() <= 0.5,
        "the control measures {THE_CONTROLS_VOLUME} - instead {volume:.3}"
    );
}

#[test]
fn the_crease_builds_and_the_kernel_takes_it_false_false() {
    the_crease_builds_and_the_kernel_takes_it(false, false);
}

#[test]
fn the_crease_builds_and_the_kernel_takes_it_true_false() {
    the_crease_builds_and_the_kernel_takes_it(true, false);
}

#[test]
fn the_crease_builds_and_the_kernel_takes_it_false_true() {
    the_crease_builds_and_the_kernel_takes_it(false, true);
}

#[test]
fn the_crease_builds_and_the_kernel_takes_it_true_true() {
    the_crease_builds_and_the_kernel_takes_it(true, true);
}

/// And it is the shape the design worked out: one vertex where the wall dies on the knife edge, one
/// where the end turns onto the top face, one where the bead dies past the apex - each of them
/// once, because every one of them is a point three rims share.
fn the_ends_stand_where_the_design_puts_them(mirror_z: bool, where_the_rebuild_has_the_part: bool) {
    let built = the_crease(mirror_z, where_the_rebuild_has_the_part);

    for (what, at) in [
        ("the wall dies on the knife edge", where_the_wall_dies()),
        ("the end turns onto the top face", where_the_end_turns()),
        (
            "the bead dies past the apex",
            where_the_bead_dies_past_the_apex(),
        ),
    ] {
        let standing = built
            .vertices
            .iter()
            .filter(|vertex| {
                (in_the_box_frame(
                    Vector3::from_vector3_float(**vertex),
                    mirror_z,
                    where_the_rebuild_has_the_part,
                ) - at)
                    .length()
                    < 1e-3
            })
            .count();

        assert_eq!(
            standing, 1,
            "{what} at {at:?}, and exactly one vertex stands there - instead {standing}"
        );
    }
}

#[test]
fn the_ends_stand_where_the_design_puts_them_false_false() {
    the_ends_stand_where_the_design_puts_them(false, false);
}

#[test]
fn the_ends_stand_where_the_design_puts_them_true_false() {
    the_ends_stand_where_the_design_puts_them(true, false);
}

#[test]
fn the_ends_stand_where_the_design_puts_them_false_true() {
    the_ends_stand_where_the_design_puts_them(false, true);
}

#[test]
fn the_ends_stand_where_the_design_puts_them_true_true() {
    the_ends_stand_where_the_design_puts_them(true, true);
}

/// And the bead stays INSIDE the part's own outside: nothing in front of the recess's mouth and
/// nothing above the cube's top, which is what the two hull planes the end leaves through mean.
fn nothing_stands_outside_the_cube(mirror_z: bool, where_the_rebuild_has_the_part: bool) {
    let built = the_crease(mirror_z, where_the_rebuild_has_the_part);
    let mut outside = Vector3::ZERO;
    let mut how_many = 0;

    for vertex in &built.vertices {
        let at = in_the_box_frame(
            Vector3::from_vector3_float(*vertex),
            mirror_z,
            where_the_rebuild_has_the_part,
        );

        if at.y < -1e-4 || at.z > recess::SIZE + 1e-4 {
            how_many += 1;
            outside = at;
        }
    }

    assert_eq!(
        how_many, 0,
        "the mouth and the top are planes of the part's own hull - instead {how_many} vertices \
         are past them, one at {outside:?}"
    );
}

#[test]
fn nothing_stands_outside_the_cube_false_false() {
    nothing_stands_outside_the_cube(false, false);
}

#[test]
fn nothing_stands_outside_the_cube_true_false() {
    nothing_stands_outside_the_cube(true, false);
}

#[test]
fn nothing_stands_outside_the_cube_false_true() {
    nothing_stands_outside_the_cube(false, true);
}

#[test]
fn nothing_stands_outside_the_cube_true_true() {
    nothing_stands_outside_the_cube(true, true);
}

/// And the cube's LEFT face gains exactly one vertex: K*, where the wall the bead ate dies on the
/// knife edge the two of them share. A point standing part way along that face's own edge without
/// being sewn into it is a crack, and one that never arrived is a strip ending somewhere else.
fn the_left_face_gains_the_one_point_the_bead_dies_at(
    mirror_z: bool,
    where_the_rebuild_has_the_part: bool,
) {
    let part = the_part(mirror_z, where_the_rebuild_has_the_part);
    let built = the_crease(mirror_z, where_the_rebuild_has_the_part);

    let before = on_the_left_face(&part, mirror_z, where_the_rebuild_has_the_part);
    let after = on_the_left_face(&built, mirror_z, where_the_rebuild_has_the_part);

    assert_eq!(
        after as i64 - before as i64,
        1,
        "the left face carried {before} vertices and now carries {after}, and the one it gains is K*"
    );

    let mut carried = 0;

    for face in built.faces.iter() {
        for corner in [face.v0, face.v1, face.v2] {
            let at = in_the_box_frame(
                Vector3::from_vector3_float(built.vertices[corner as usize]),
                mirror_z,
                where_the_rebuild_has_the_part,
            );
            if (at - where_the_wall_dies()).length() < 1e-3 {
                carried += 1;
            }
        }
    }

    assert!(
        carried >= 4,
        "and it is SEWN in rather than standing on somebody's edge - instead {carried} faces \
         carry it"
    );
}

#[test]
fn the_left_face_gains_the_one_point_the_bead_dies_at_false_false() {
    the_left_face_gains_the_one_point_the_bead_dies_at(false, false);
}

#[test]
fn the_left_face_gains_the_one_point_the_bead_dies_at_true_false() {
    the_left_face_gains_the_one_point_the_bead_dies_at(true, false);
}

#[test]
fn the_left_face_gains_the_one_point_the_bead_dies_at_false_true() {
    the_left_face_gains_the_one_point_the_bead_dies_at(false, true);
}

#[test]
fn the_left_face_gains_the_one_point_the_bead_dies_at_true_true() {
    the_left_face_gains_the_one_point_the_bead_dies_at(true, true);
}

/// How many vertices of a mesh stand on the cube's left face, x = 0 in the box frame.
fn on_the_left_face(mesh: &Mesh, mirror_z: bool, where_the_rebuild_has_the_part: bool) -> usize {
    mesh.vertices
        .iter()
        .filter(|vertex| {
            in_the_box_frame(
                Vector3::from_vector3_float(**vertex),
                mirror_z,
                where_the_rebuild_has_the_part,
            )
            .x
            .abs()
                < 1e-4
        })
        .count()
}

/// A point of the built part read back in the box frame the design's numbers are written in - the
/// offset taken off and the mirror turned back.
fn in_the_box_frame(at: Vector3, mirror_z: bool, where_the_rebuild_has_the_part: bool) -> Vector3 {
    let moved = at
        - if where_the_rebuild_has_the_part {
            recess::fixture_offset()
        } else {
            Vector3::ZERO
        };

    if mirror_z {
        recess::mirror_z(moved)
    } else {
        moved
    }
}

/// The fixture's own fourteen vertices, either way up and at either place.
fn the_part(mirror_z: bool, where_the_rebuild_has_the_part: bool) -> Mesh {
    recess::create_from_the_fixtures_own_triangles(
        if where_the_rebuild_has_the_part {
            recess::fixture_offset()
        } else {
            Vector3::ZERO
        },
        mirror_z,
    )
}

/// The upper left crease, planned at 3 mm over 8 segments and built through the surface
/// construction - with the ROOM's own reading handed to the build, which is how the pass really
/// runs it.
fn the_crease(mirror_z: bool, where_the_rebuild_has_the_part: bool) -> Mesh {
    let part = the_part(mirror_z, where_the_rebuild_has_the_part);
    let graph = part.get_mesh_edge_graph();
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(part.clone()),
        Some(graph.clone()),
        FeatureEdge::default_flat_angle_degrees(),
    ));
    let features = FeatureEdgeSet::create(&part);

    let place = |box_frame_point: Vector3| -> Vector3 {
        (if mirror_z {
            recess::mirror_z(box_frame_point)
        } else {
            box_frame_point
        }) + if where_the_rebuild_has_the_part {
            recess::fixture_offset()
        } else {
            Vector3::ZERO
        }
    };

    let rim = place(recess::rim_end());
    let apex = place(recess::apex_end());

    let creases: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| {
            ((edge.start - rim).length() < 1e-3 && (edge.end - apex).length() < 1e-3)
                || ((edge.end - rim).length() < 1e-3 && (edge.start - apex).length() < 1e-3)
        })
        .copied()
        .collect();
    assert_eq!(creases.len(), 1, "exactly one: the upper left crease");

    let mut chains = surface_chain_join::join(&bevel_chain_linker::link(&creases));
    assert_eq!(chains.len(), 1, "exactly one chain");
    let chain = chains.remove(0);

    let plan: Rc<SurfaceBevelPlan> = SurfaceBevelPlan::try_create(&chain, &regions, RADIUS, true)
        .unwrap_or_else(|reason| panic!("the crease has to plan at {RADIUS}: {:?}", reason.code()));

    let plans = [plan];
    let mut stretches: Vec<SurfaceBandRunsOffOverAStretch> = Vec::new();
    let token = CancelToken::new();

    let fit = surface_room::try_fit(&plans, RADIUS, &token, Some(&mut stretches))
        .expect("nothing cancels");
    if !fit.fits {
        panic!(
            "and the room has to allow it: {:?} allowing {}",
            fit.refusal.map(|why| why.code()),
            fit.largest_fitting_size
        );
    }

    assert_eq!(
        stretches.len(),
        1,
        "and the left wall has to run off over its stretch: {} were read",
        stretches.len()
    );

    surface_bevel_builder::try_build(
        &part,
        &graph,
        &regions,
        &plans,
        SEGMENTS,
        &token,
        SurfaceBuildOptions {
            crossings: Some(&fit.crossings),
            ran_off_over_a_stretch: Some(&stretches),
            ..crate::surface_test_build::build_options()
        },
    )
    .unwrap_or_else(|reason| panic!("and it has to build: {reason:?}"))
    .mesh
}
