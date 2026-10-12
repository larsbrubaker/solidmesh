//! Port of `SurfaceRoomAcrossACurvedWallTests.cs`: the room on the CURVED side of a crease - a half
//! disc's vertical seam, where the flat chord face meets the tessellated wall. The wall turns 9
//! degrees a facet, under the 10 degree bend threshold this is read at, so it is one smooth surface,
//! and the room across it is the run along that surface to the seam at the other end of the arc,
//! not the width of the one facet the crease happens to touch.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::surface::{surface_room, surface_room_measure, SurfaceBevelPlan};
use geometry_processing::bevel::{
    bevel_chain_linker, BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode,
};
use geometry_processing::topology::{FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::math_helper::degrees_to_radians;
use vector_math::{Matrix4X4, Vector3, Vector3Float};

use crate::topology_test_meshes;

/// A mesh keeps its vertices as floats, so a run of 31 mm is a run of 31 mm to about this.
const NEAR: f64 = 1e-3;

/// The bend threshold the fixture is set to, which the 9 degree facet joins are under.
const THRESHOLD_DEGREES: f64 = 10.0;

/// 20 chords of 2 * 10 * sin(4.5): the half turn of wall, 31.29.
fn across_the_wall() -> f64 {
    20.0 * 2.0 * 10.0 * degrees_to_radians(4.5).sin()
}

/// The seam of a half disc 20 mm across, 20 segments of arc: the flat side reads the 20 mm chord
/// face it is on, and the round side reads the whole half turn of wall rather than the first chord.
#[test]
fn the_room_across_a_smooth_wall_is_the_run_along_it_to_the_far_seam() {
    let prism = topology_test_meshes::create_arc_prism(10.0, 20.0, 20);
    let plan = seam_plan_of(&prism, 3.0);

    // The BANDS' rooms. The seam's two free ends notch the caps beyond them as well, and a notch is
    // bounded by another chain's footprint rather than by any room of its own.
    let mut rooms: Vec<f64> = surface_room_measure::measure(&[plan], None, None)
        .into_iter()
        .filter(|cut| !cut.corner_footprint())
        .map(|cut| cut.room)
        .collect();
    rooms.sort_by(f64::total_cmp);

    assert_eq!(
        rooms.len(),
        2,
        "the seam lands on the chord face on one side and on the wall on the other"
    );

    assert!(
        (rooms[0] - 20.0).abs() <= NEAR,
        "the chord face is 20 mm across, and it measured {}",
        rooms[0]
    );
    assert!(
        (rooms[1] - across_the_wall()).abs() <= NEAR,
        "the wall runs {} around to the other seam, and it measured {}",
        across_the_wall(),
        rooms[1]
    );
}

/// And so a fillet bigger than one facet of that wall FITS on it: at 3 mm the seam's setback is 3.25
/// into each side, which a 1.56 mm chord cannot hold and 31 mm of wall can.
#[test]
fn a_fillet_wider_than_one_facet_of_the_wall_still_fits_on_it() {
    let prism = topology_test_meshes::create_arc_prism(10.0, 20.0, 20);

    let fit = surface_room::try_fit(&[seam_plan_of(&prism, 3.0)], 3.0, &CancelToken::new(), None)
        .expect("not cancelled");

    assert!(
        fit.fits,
        "3 mm of fillet has a half turn of wall to land in - {:?}, which allowed only {}",
        fit.refusal, fit.largest_fitting_size
    );
}

/// And a fillet nothing here can hold is still refused with the size that fits - by the PLAN now,
/// because no circle larger than the one inscribed in the half disc (radius / 2, 5 mm) can be
/// tangent to the wall at a facet it touches.
#[test]
fn the_flat_side_of_the_same_seam_still_refuses_what_it_cannot_hold() {
    let prism = topology_test_meshes::create_arc_prism(10.0, 20.0, 20);

    let refusal = match try_seam_plan_of(&prism, 40.0, Matrix4X4::IDENTITY) {
        Ok(_) => panic!("a 40 mm fillet has to be refused by the plan"),
        Err(refusal) => refusal,
    };
    assert_eq!(refusal.code(), BevelAdvisoryCode::SizeExceedsFaceWidth);
    match refusal.args()[2] {
        BevelAdvisoryArg::Double(named) => assert!(
            (named - 5.0).abs() <= 0.05,
            "a circle bigger than the disc's own inscribed 5 mm touches nothing, and it named {named}"
        ),
        ref other => panic!("the size that fits is a double, not {other:?}"),
    }

    // And the room reading itself, asked of a plan that CAN be drawn: 40 mm is still too much for
    // the chord face, and what the fit allows times what that face gives up per unit of size is the
    // face's own 20 mm.
    let fit = surface_room::try_fit(
        &[seam_plan_of(&prism, 3.0)],
        40.0,
        &CancelToken::new(),
        None,
    )
    .expect("not cancelled");

    assert!(
        !fit.fits,
        "40 mm has neither the wall nor the chord face to land in"
    );
    assert_eq!(
        fit.refusal.as_ref().map(BevelAdvisory::code),
        Some(BevelAdvisoryCode::SizeExceedsFaceWidth)
    );

    let largest = fit.largest_fitting_size;
    assert!(
        (largest * setback_per_size(&prism) - 20.0).abs() <= NEAR,
        "the chord face gives up its own 20 mm and no more, and the fit allowed {largest}"
    );
}

/// The walk reads every facet's normal off its VERTICES, not off the mesh's cached float normal: a
/// mesh whose vertices have been moved without a recalculation still measures the same half turn
/// of wall, because a rigid turn changes no distance on it.
#[test]
fn the_run_is_the_same_on_a_mesh_whose_cached_normals_went_stale() {
    let mut turned = topology_test_meshes::create_arc_prism(10.0, 20.0, 20);
    let turn = Matrix4X4::create_rotation_x(degrees_to_radians(37.0));

    for vertex in turned.vertices.iter_mut() {
        // Straight onto the vertex list, which is what leaves the cached face normals behind.
        *vertex = Vector3Float::from_vector3(Vector3::from_vector3_float(*vertex).transform(turn));
    }

    let mut rooms: Vec<f64> =
        surface_room_measure::measure(&[seam_plan_of_moved(&turned, 3.0, turn)], None, None)
            .into_iter()
            .map(|cut| cut.room)
            .collect();
    rooms.sort_by(f64::total_cmp);

    let wall = turned.faces[0];
    let at = |index: i32| Vector3::from_vector3_float(turned.vertices[index as usize]);
    let from_the_vertices = (at(wall.v1) - at(wall.v0))
        .cross(at(wall.v2) - at(wall.v0))
        .get_normal();

    assert!(
        from_the_vertices.dot(Vector3::from_vector3_float(wall.normal)) < 0.999,
        "the fixture is only worth anything while the cached normals ARE stale, and nothing in the reading may quietly refresh them"
    );

    assert!(
        (rooms[1] - across_the_wall()).abs() <= NEAR,
        "turning the part turns the wall with it, and the run measured {}",
        rooms[1]
    );
}

/// The room is the SHORTEST section along the chain edge, not the one at its middle. On a half cone
/// from a 10 mm radius at the bottom to 2 mm at the top the run around to the other seam shrinks
/// with the radius, so the reading has to come from the narrow end of the seam.
#[test]
fn the_room_is_the_narrowest_section_along_the_edge_and_not_the_one_at_its_middle() {
    let cone = half_prism(10.0, 2.0, 20.0, 20);

    // The cut on the WALL, told from the one on the chord face by which way it reaches.
    let walls: Vec<f64> = surface_room_measure::measure(&[seam_plan_of(&cone, 1.0)], None, None)
        .into_iter()
        .filter(|cut| cut.inward.y > 0.5)
        .map(|cut| cut.room)
        .collect();
    assert_eq!(walls.len(), 1, "one cut reaches around the cone");
    let wall = walls[0];

    // A quarter, the middle and three quarters along the seam the cone's radius is 8, 6 and 4: a
    // half turn of about 25, 19 and 13, the last a little longer for the tilt of the section plane.
    assert!(
        wall < 16.0,
        "three quarters up the seam the wall is only about 13 round, and it read {wall}"
    );
    assert!(
        wall > 11.0,
        "it is not shorter than the narrowest section either, and it read {wall}"
    );
}

/// A section passing exactly through a VERTEX carries on across that vertex's fan instead of
/// stopping there: the half disc with one upright wall edge split at its middle is the same solid,
/// so the run has to come out the same 31.29 as before.
#[test]
fn a_section_through_a_vertex_carries_on_across_its_fan() {
    let split =
        split_an_upright_wall_edge(&topology_test_meshes::create_arc_prism(10.0, 20.0, 20), 5);

    let mut rooms: Vec<f64> =
        surface_room_measure::measure(&[seam_plan_of(&split, 3.0)], None, None)
            .into_iter()
            .map(|cut| cut.room)
            .collect();
    rooms.sort_by(f64::total_cmp);

    assert!(
        (rooms[1] - across_the_wall()).abs() <= NEAR,
        "the wall is the same wall with one more vertex on it, and it measured {}",
        rooms[1]
    );
}

/// The plan for the seam at (10, 0), flooded at the fixture's own 10 degree threshold.
fn seam_plan_of(mesh: &Mesh, size: f64) -> Rc<SurfaceBevelPlan> {
    seam_plan_of_moved(mesh, size, Matrix4X4::IDENTITY)
}

/// The same, for a part that has been moved: the seam is looked for where `moved` puts it.
fn seam_plan_of_moved(mesh: &Mesh, size: f64, moved: Matrix4X4) -> Rc<SurfaceBevelPlan> {
    match try_seam_plan_of(mesh, size, moved) {
        Ok(plan) => plan,
        Err(reason) => panic!(
            "the seam has to plan before its room can be measured: {:?}",
            reason.code()
        ),
    }
}

/// The same, where the plan's own refusal is what is being read rather than an obstacle.
fn try_seam_plan_of(
    mesh: &Mesh,
    size: f64,
    moved: Matrix4X4,
) -> Result<Rc<SurfaceBevelPlan>, BevelAdvisory> {
    let features = FeatureEdgeSet::create_with_angle(mesh, THRESHOLD_DEGREES);
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(mesh.clone()),
        Some(mesh.get_mesh_edge_graph()),
        THRESHOLD_DEGREES,
    ));

    let seam: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| is_the_seam(edge, moved))
        .copied()
        .collect();

    let mut chains = bevel_chain_linker::link(&seam);
    assert_eq!(chains.len(), 1, "the seam links into one chain");

    SurfaceBevelPlan::try_create(&chains.remove(0), &regions, size, true)
}

/// How far the seam's fillet sets back into the CHORD FACE per unit of size, read on a plan drawn
/// at three, and told from the wall's by which way the cut reaches: across the flat.
fn setback_per_size(mesh: &Mesh) -> f64 {
    let flat: Vec<f64> = surface_room_measure::measure(&[seam_plan_of(mesh, 3.0)], None, None)
        .into_iter()
        .filter(|cut| !cut.corner_footprint() && cut.inward.y.abs() < 0.5)
        .map(|cut| cut.setback_per_size)
        .collect();
    assert_eq!(flat.len(), 1, "one band lands on the chord face");

    flat[0]
}

/// The crease where the arc runs into the chord that closes it, read where `moved` puts it.
fn is_the_seam(edge: &FeatureEdge, moved: Matrix4X4) -> bool {
    at_the_seam(edge.start, moved) && at_the_seam(edge.end, moved)
}

/// Whether one end of an edge stands on the half of the chord plane the seam is on - y = 0 and x
/// positive, in the part's own space.
fn at_the_seam(point: Vector3, moved: Matrix4X4) -> bool {
    let built = point.transform(moved.inverted());

    built.y.abs() < 1e-3 && built.x > 1e-3
}

/// Half of a prism or cone, upright: a tessellated wall from `bottom_radius` to `top_radius` over a
/// half turn, closed by the flat face on the chord, with a cap at each end.
fn half_prism(bottom_radius: f64, top_radius: f64, height: f64, segments: i32) -> Mesh {
    let at = |step: i32, top: bool| {
        let angle = std::f64::consts::PI * f64::from(step) / f64::from(segments);
        let radius = if top { top_radius } else { bottom_radius };

        Vector3::new(
            radius * angle.cos(),
            radius * angle.sin(),
            if top { height } else { 0.0 },
        )
    };

    let mut mesh = Mesh::new();

    for i in 0..segments {
        // Wound so the wall faces away from the axis, which is what makes the seam a convex crease.
        mesh.create_face(&[at(i, false), at(i + 1, false), at(i + 1, true)]);
        mesh.create_face(&[at(i, false), at(i + 1, true), at(i, true)]);
    }

    // The caps as fans from the first corner of each, so their straight side is the one chord edge
    // the flat face below is built on and no T junction is left between them.
    for i in 1..segments {
        mesh.create_face(&[at(0, false), at(i + 1, false), at(i, false)]);
        mesh.create_face(&[at(0, true), at(i, true), at(i + 1, true)]);
    }

    mesh.create_face(&[at(0, false), at(0, true), at(segments, true)]);
    mesh.create_face(&[at(0, false), at(segments, true), at(segments, false)]);

    mesh.clean_and_merge();

    mesh
}

/// The same solid with one upright edge of its wall split at its middle: every facet on that edge
/// is cut in two at the new vertex, so the surface is unchanged.
fn split_an_upright_wall_edge(mesh: &Mesh, at_segment: i32) -> Mesh {
    let angle = std::f64::consts::PI * f64::from(at_segment) / 20.0;
    let low = Vector3::new(10.0 * angle.cos(), 10.0 * angle.sin(), 0.0);
    let high = Vector3::new(low.x, low.y, 20.0);
    let middle = (low + high) / 2.0;

    let is = |vertex: Vector3Float, at: Vector3| {
        (Vector3::from_vector3_float(vertex) - at).length() < 1e-4
    };

    let mut split = Mesh::new();

    for face in mesh.faces.iter() {
        let corners = [
            mesh.vertices[face.v0 as usize],
            mesh.vertices[face.v1 as usize],
            mesh.vertices[face.v2 as usize],
        ];
        let mut on_the_edge = None;

        for i in 0..3 {
            if (is(corners[i], low) && is(corners[(i + 1) % 3], high))
                || (is(corners[i], high) && is(corners[(i + 1) % 3], low))
            {
                on_the_edge = Some(i);
            }
        }

        let Some(i) = on_the_edge else {
            split.create_face(&corners.map(Vector3::from_vector3_float));
            continue;
        };

        let a = Vector3::from_vector3_float(corners[i]);
        let b = Vector3::from_vector3_float(corners[(i + 1) % 3]);
        let c = Vector3::from_vector3_float(corners[(i + 2) % 3]);

        split.create_face(&[a, middle, c]);
        split.create_face(&[middle, b, c]);
    }

    split.clean_and_merge();

    split
}
