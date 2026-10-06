//! Port of `SurfaceRoomTests.cs`: rung 8's room, measured against numbers anybody can pace out on
//! the fixture - how far it is across a face from the edge being beveled to the far side of that
//! face, what two bevels landing on one face leave each other, and the largest size that fits in
//! what is left. This is the 2D relaxation's model (`PathBevel.RelaxRadii`) read in 3D.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{surface_room, surface_room_measure, SurfaceBevelPlan};
use geometry_processing::bevel::{BevelAdvisoryArg, BevelAdvisoryCode};
use geometry_processing::topology::{FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::math_helper::degrees_to_radians;
use vector_math::{Vector2, Vector3};

use crate::topology_test_meshes::create_isosceles_prism;

/// A mesh keeps its vertices as floats, so a distance of 20 is a distance of 20 to about this.
const NEAR: f64 = 1e-5;

/// One edge of a cube's top: the room is the whole 20 across the top face and the whole 20 down
/// the front one; the two edges running square across from its own corners are not walls.
#[test]
fn one_picked_edge_of_a_cube_has_the_whole_face_on_both_sides_of_it() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let plan = plan_of(&cube, &[top_front_edge(&cube)], 3.0, true);

    let cuts = surface_room_measure::measure(std::slice::from_ref(&plan), None, None);

    assert_eq!(
        cuts.len(),
        4,
        "one edge lands on two regions, so it makes two bands - and its two free ends notch the side face beyond each of them (SurfaceRoomEndCut)"
    );
    assert_eq!(
        cuts.iter().filter(|cut| cut.corner_footprint()).count(),
        2,
        "the two notches are corner footprints, seated at the edge's own two ends"
    );

    for cut in cuts.iter().filter(|cut| !cut.corner_footprint()) {
        assert!(
            (cut.room - 20.0).abs() <= NEAR,
            "a 20 mm cube face is 20 mm across, and this one measured {}",
            cut.room
        );
        assert!(
            (cut.setback_per_size - 1.0).abs() <= NEAR,
            "a 90 degree wedge sets a round back by its radius"
        );
    }

    let fit = try_fit(&[plan], 3.0);
    assert!(fit.fits, "3 mm has 20 to land in - {:?}", fit.refusal);
    assert!((fit.largest_fitting_size - 3.0).abs() <= NEAR);
}

/// The same cube edge at a radius the face cannot hold: refused, with the 20 that would have fitted.
#[test]
fn a_radius_wider_than_its_face_is_refused_with_the_width_of_the_face() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let plan = plan_of(&cube, &[top_front_edge(&cube)], 25.0, true);

    let fit = try_fit(&[plan], 25.0);
    assert!(!fit.fits);
    assert!(
        (fit.largest_fitting_size - 20.0).abs() <= NEAR,
        "the setback of a 90 degree round IS its radius, so what fits across 20 mm of face is 20"
    );
    let refusal = fit.refusal.expect("a refusal");
    assert_eq!(
        refusal.code(),
        BevelAdvisoryCode::SizeExceedsFaceWidth,
        "nothing is coming the other way across the face: it is simply too wide a cut"
    );
    assert!(
        (double_arg(&refusal.args()[2]) - 20.0).abs() <= NEAR,
        "the advisory carries the size that fits, which is what the panel quotes"
    );
}

/// A 5 mm wide face: the room across it is 5, so a fillet fits up to a 5 mm radius and no further.
#[test]
fn a_narrow_face_gives_up_its_own_width_and_no_more() {
    let slab = platonic_solids::create_cube(20.0, 5.0, 20.0);
    let plan = plan_of(&slab, &[top_edge_at(&slab, 2.5)], 4.0, true);

    let mut rooms: Vec<f64> =
        surface_room_measure::measure(std::slice::from_ref(&plan), None, None)
            .iter()
            .map(|cut| cut.room)
            .collect();
    rooms.sort_by(f64::total_cmp);

    assert!(
        (rooms[0] - 5.0).abs() <= NEAR,
        "the top of the slab is 5 mm across and measured {}",
        rooms[0]
    );
    assert!(
        (rooms[1] - 20.0).abs() <= NEAR,
        "the side of it is the full 20 mm down"
    );

    let fit = try_fit(&[plan], 4.0);
    assert!(fit.fits);
    assert!((fit.largest_fitting_size - 4.0).abs() <= NEAR);

    let too_big = plan_of(&slab, &[top_edge_at(&slab, 2.5)], 6.0, true);

    let refused = try_fit(&[too_big], 6.0);
    assert!(!refused.fits);
    assert!(
        (refused.largest_fitting_size - 5.0).abs() <= NEAR,
        "5 is the whole width of the face, and it said {} - {:?}",
        refused.largest_fitting_size,
        refused.refusal
    );
}

/// Both long edges of that 5 mm face picked: the two share the 5 in proportion to their tangent
/// distances - equal here, so 2.5 each.
#[test]
fn two_parallel_edges_on_one_face_split_the_room_between_them() {
    let slab = platonic_solids::create_cube(20.0, 5.0, 20.0);

    let plans = [
        plan_of(&slab, &[top_edge_at(&slab, 2.5)], 4.0, true),
        plan_of(&slab, &[top_edge_at(&slab, -2.5)], 4.0, true),
    ];

    let fit = try_fit(&plans, 4.0);
    assert!(!fit.fits);
    assert!(
        (fit.largest_fitting_size - 2.5).abs() <= NEAR,
        "half of the 5 mm face each, and it said {}",
        fit.largest_fitting_size
    );
    assert_eq!(
        fit.refusal.expect("a refusal").code(),
        BevelAdvisoryCode::SizeCollidesAcrossFace,
        "this is two bevels running into each other, not one that is simply too wide"
    );

    let exact = try_fit(
        &[
            plan_of(&slab, &[top_edge_at(&slab, 2.5)], 2.5, true),
            plan_of(&slab, &[top_edge_at(&slab, -2.5)], 2.5, true),
        ],
        2.5,
    );
    assert!(exact.fits, "2.5 each is exactly the face, which fits");
}

/// A whole rim - four edges of one 20 mm face: the two edges facing each other share its 20 and
/// get 10 apiece; the two running square to each one do not compete with it at all.
#[test]
fn a_face_rim_shares_the_face_with_the_edge_opposite_and_not_with_the_ones_beside_it() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let plan = plan_of(&cube, &top_rim_edges(&cube), 3.0, true);

    assert_eq!(plan.edges.len(), 4);
    assert!(plan.is_closed);

    let fit = try_fit(&[plan], 3.0);
    assert!(
        fit.fits,
        "a 3 mm round on a 20 mm rim is the ladder's own rung 2 - {:?}",
        fit.refusal
    );

    let too_big = plan_of(&cube, &top_rim_edges(&cube), 12.0, true);

    let refused = try_fit(&[too_big], 12.0);
    assert!(!refused.fits);
    assert!(
        (refused.largest_fitting_size - 10.0).abs() <= NEAR,
        "the two edges across the top from each other take half of it each, and it said {}",
        refused.largest_fitting_size
    );
}

/// A chamfer sets back by its size whatever the wedge, and a round by `r / tan(half the wedge)`,
/// so the same face answers the two of them with different sizes.
#[test]
fn the_fitting_size_is_read_through_the_wedge_and_not_straight_off_the_room() {
    let prism = create_isosceles_prism(60.0, 20.0, 10.0);
    let apex = apex_edge(&prism);

    // Asked for far more than the prism can give, so that what comes back is the room's own answer
    // rather than the request handed straight back.
    let round = plan_of(&prism, &[apex], 40.0, true);
    let straight = plan_of(&prism, &[apex], 40.0, false);

    let room = surface_room_measure::measure(std::slice::from_ref(&round), None, None)
        .iter()
        .map(|cut| cut.room)
        .fold(f64::INFINITY, f64::min);

    let round_fits = try_fit(&[round], 40.0).largest_fitting_size;
    let straight_fits = try_fit(&[straight], 40.0).largest_fitting_size;

    assert!(
        (straight_fits - room).abs() <= 1e-4,
        "a chamfer's setback is its size, so what fits is the room itself - {room}"
    );
    assert!(
        (round_fits - (room * degrees_to_radians(30.0).tan())).abs() <= 1e-4,
        "a round reaches r / tan(30) into a 60 degree wedge, so only {room} * tan(30) of radius fits"
    );
}

/// A triangular face, picked on one side of it: the room is the ALTITUDE of the triangle from that
/// side, not infinity. A 6 by 8 right triangle picked on its hypotenuse has an altitude of 4.8.
#[test]
fn a_triangular_face_is_worth_its_altitude_and_not_an_infinity_of_room() {
    // Tall enough that the side face beside the hypotenuse is nowhere near the binding constraint.
    let prism = triangular_prism(
        Vector2::new(0.0, 0.0),
        Vector2::new(0.0, 6.0),
        Vector2::new(8.0, 0.0),
        60.0,
    );
    let hypotenuse = top_edge_between(&prism, Vector2::new(0.0, 6.0), Vector2::new(8.0, 0.0));
    let plan = plan_of(&prism, &[hypotenuse], 6.0, true);

    // The BANDS' rooms; the chain's two ends notch the faces beyond them as well.
    let mut rooms: Vec<f64> =
        surface_room_measure::measure(std::slice::from_ref(&plan), None, None)
            .iter()
            .filter(|cut| !cut.corner_footprint())
            .map(|cut| cut.room)
            .collect();
    rooms.sort_by(f64::total_cmp);

    assert_eq!(rooms.len(), 2);
    assert!(
        (rooms[0] - 4.8).abs() <= NEAR,
        "the altitude of a 6 by 8 right triangle from its hypotenuse is 6 * 8 / 10, and the face measured {}",
        rooms[0]
    );
    assert!(
        (rooms[1] - 60.0).abs() <= NEAR,
        "the wall below that edge is the full height of the prism"
    );

    let fit = try_fit(&[plan], 6.0);
    assert!(
        !fit.fits,
        "a 90 degree round sets back by its radius, and 6 of setback does not fit in 4.8 of triangle"
    );
    assert!(
        (fit.largest_fitting_size - 4.8).abs() <= NEAR,
        "the refusal names the size that fits, and it said {}",
        fit.largest_fitting_size
    );
    assert_eq!(
        fit.refusal.expect("a refusal").code(),
        BevelAdvisoryCode::SizeExceedsFaceWidth
    );

    let smaller = try_fit(&[plan_of(&prism, &[hypotenuse], 4.0, true)], 4.0);
    assert!(
        smaller.fits,
        "4 of setback is inside the 4.8 the triangle holds, so the triangle is not refusing everything either"
    );
}

/// Nothing the same pass is beveling onto one face is a WALL on that face: a cube's top rim picked
/// whole has no wall on the top at all, and each band may take the half of the 20 the band facing
/// it leaves.
#[test]
fn nothing_the_pass_is_beveling_onto_the_face_is_a_wall_on_it() {
    let cube = platonic_solids::create_cube(20.0, 20.0, 20.0);
    let plan = plan_of(&cube, &top_rim_edges(&cube), 3.0, true);

    // The four cuts landing on the TOP face.
    for cut in surface_room_measure::measure(std::slice::from_ref(&plan), None, None)
        .iter()
        .filter(|cut| cut.inward.z.abs() < 0.5)
    {
        assert!(
            cut.room >= 20.0 - NEAR,
            "no rim of the top face is a wall to it, and this measured {}",
            cut.room
        );
    }

    let fit = try_fit(&[plan], 3.0);
    assert!(
        fit.fits,
        "3 mm a side is well inside the 20 the four bands share - {:?}",
        fit.refusal
    );
    assert!((fit.largest_fitting_size - 3.0).abs() <= NEAR);

    let too_big = plan_of(&cube, &top_rim_edges(&cube), 12.0, true);

    let refused = try_fit(&[too_big], 12.0);
    assert!(
        !refused.fits,
        "two 12 mm setbacks cannot both fit across a 20 mm face"
    );
    assert!(
        (refused.largest_fitting_size - 10.0).abs() <= NEAR,
        "the band facing it takes the other half of the 20, and the reading said {}",
        refused.largest_fitting_size
    );
    assert_eq!(
        refused.refusal.expect("a refusal").code(),
        BevelAdvisoryCode::SizeCollidesAcrossFace,
        "what is in the way is the bevel coming back across the face, not a wall"
    );
}

/// C# `SurfaceRoom.TryFit`, with a token nothing cancels.
fn try_fit(plans: &[Rc<SurfaceBevelPlan>], size: f64) -> surface_room::SurfaceRoomFit {
    surface_room::try_fit(plans, size, &CancelToken::new(), None).expect("nothing cancels")
}

fn double_arg(arg: &BevelAdvisoryArg) -> f64 {
    match arg {
        BevelAdvisoryArg::Double(value) => *value,
        other => panic!("expected a double argument, got {other}"),
    }
}

/// One plan for the given feature edges of a mesh, at the pick's own flat angle.
fn plan_of(
    mesh: &Mesh,
    edge_indices: &[usize],
    size: f64,
    round_profile: bool,
) -> Rc<SurfaceBevelPlan> {
    let features = FeatureEdgeSet::create(mesh);
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(mesh.clone()),
        Some(mesh.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    ));
    let picked: Vec<FeatureEdge> = edge_indices
        .iter()
        .map(|&index| features.edges()[index])
        .collect();
    let mut chains = bevel_chain_linker::link(&picked);
    assert_eq!(chains.len(), 1);
    let chain = chains.remove(0);

    SurfaceBevelPlan::try_create(&chain, &regions, size, round_profile).unwrap_or_else(|reason| {
        panic!("the fixture has to plan before its room can be measured: {reason:?}")
    })
}

/// C#'s `Enumerable.Single` over the feature edges' indices.
fn single(features: &FeatureEdgeSet, is_it: impl Fn(&FeatureEdge) -> bool) -> usize {
    let found: Vec<usize> = (0..features.count())
        .filter(|&i| is_it(&features.edges()[i]))
        .collect();
    assert_eq!(found.len(), 1, "exactly one feature edge matches");
    found[0]
}

fn top_of(mesh: &Mesh) -> f64 {
    mesh.vertices
        .iter()
        .map(|vertex| vertex.z as f64)
        .fold(f64::NEG_INFINITY, f64::max)
}

/// The top front edge of a cube, running in x at the lowest y of the top face.
fn top_front_edge(mesh: &Mesh) -> usize {
    let features = FeatureEdgeSet::create(mesh);
    let top = top_of(mesh);
    let front = mesh
        .vertices
        .iter()
        .map(|vertex| vertex.y as f64)
        .fold(f64::INFINITY, f64::min);

    single(&features, |edge| {
        on_z(edge, top) && (edge.start.y - front).abs() < 1e-4 && (edge.end.y - front).abs() < 1e-4
    })
}

/// The top edge of a slab running in x at the named y.
fn top_edge_at(mesh: &Mesh, y: f64) -> usize {
    let features = FeatureEdgeSet::create(mesh);
    let top = top_of(mesh);

    single(&features, |edge| {
        on_z(edge, top) && (edge.start.y - y).abs() < 1e-4 && (edge.end.y - y).abs() < 1e-4
    })
}

/// All four edges of a cube's top face.
fn top_rim_edges(mesh: &Mesh) -> Vec<usize> {
    let features = FeatureEdgeSet::create(mesh);
    let top = top_of(mesh);

    (0..features.count())
        .filter(|&i| on_z(&features.edges()[i], top))
        .collect()
}

/// The apex edge of the isosceles prism: the one crease lying on the z axis.
fn apex_edge(mesh: &Mesh) -> usize {
    let features = FeatureEdgeSet::create(mesh);

    single(&features, |edge| {
        Vector2::new(edge.start.x, edge.start.y).length() < 1e-9
            && Vector2::new(edge.end.x, edge.end.y).length() < 1e-9
    })
}

/// A prism on the given triangle, extruded along z the way `create_isosceles_prism` builds its
/// own: the outline runs CLOCKWISE in xy, and the soup is welded.
fn triangular_prism(a: Vector2, b: Vector2, c: Vector2, height: f64) -> Mesh {
    let outline = [a, b, c];
    let bottom = -height / 2.0;
    let top = height / 2.0;

    let mut mesh = Mesh::new();

    mesh.create_face(&[
        Vector3::from_vector2(outline[0], bottom),
        Vector3::from_vector2(outline[1], bottom),
        Vector3::from_vector2(outline[2], bottom),
    ]);

    mesh.create_face(&[
        Vector3::from_vector2(outline[0], top),
        Vector3::from_vector2(outline[2], top),
        Vector3::from_vector2(outline[1], top),
    ]);

    for i in 0..outline.len() {
        let from = outline[i];
        let to = outline[(i + 1) % outline.len()];

        mesh.create_face(&[
            Vector3::from_vector2(from, bottom),
            Vector3::from_vector2(from, top),
            Vector3::from_vector2(to, top),
            Vector3::from_vector2(to, bottom),
        ]);
    }

    mesh.clean_and_merge();

    mesh
}

/// The edge of the top face running between the two named xy corners.
fn top_edge_between(mesh: &Mesh, a: Vector2, b: Vector2) -> usize {
    let features = FeatureEdgeSet::create(mesh);
    let top = top_of(mesh);

    single(&features, |edge| {
        on_z(edge, top)
            && ((at(edge.start, a) && at(edge.end, b)) || (at(edge.start, b) && at(edge.end, a)))
    })
}

fn at(point: Vector3, xy: Vector2) -> bool {
    Vector2::new(point.x - xy.x, point.y - xy.y).length() < 1e-4
}

fn on_z(edge: &FeatureEdge, z: f64) -> bool {
    (edge.start.z - z).abs() < 1e-4 && (edge.end.z - z).abs() < 1e-4
}
