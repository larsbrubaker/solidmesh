//! The ball at one chain point (`ball_station`), held to centres and contacts worked out by hand:
//! flat faces meeting at 90 and 60 degrees, concave and convex; a cylinder standing on a plate,
//! read mid-facet so the faceted answer is exact; a floor narrower than the ball's reach, where
//! the contact must slide onto the floor's rim; and a sphere sunk into a plate, curved on one
//! side, held to the smooth answer within the facets' chord.

use geometry_processing::bevel::rolling_ball::{
    ball_station, ball_station_reading, flat_face_centre, BallStation, BallStationError,
    ContactFeature, HalfSpace, RollingBallSide, SideReading,
};
use vector_math::Vector3;

use crate::surfaces::{cylinder_wall, rectangle, sphere};

const RADIUS: f64 = 5.0;

/// Exact answers, give or take the solve's own tolerance.
const EXACT: f64 = 1e-9;

fn close(a: Vector3, b: Vector3, within: f64) -> bool {
    (a - b).length() <= within
}

/// Two sides of one part, sharing its vertex list.
fn sides(
    positions: &[Vector3],
    a: &[[i32; 3]],
    b: &[[i32; 3]],
) -> (RollingBallSide, RollingBallSide) {
    (
        RollingBallSide::new(positions, a),
        RollingBallSide::new(positions, b),
    )
}

/// Solves from the flat-face estimate nudged off by `nudge`, so Newton has work to do.
fn solve(
    (a, b): &(RollingBallSide, RollingBallSide),
    chain_point: Vector3,
    tangent: Vector3,
    normals: (Vector3, Vector3),
    convex: bool,
    nudge: Vector3,
) -> BallStation {
    let start = flat_face_centre(chain_point, normals.0, normals.1, RADIUS, convex).unwrap();
    ball_station(a, b, chain_point, tangent, start + nudge, RADIUS).unwrap()
}

/// A floor (z = 0, x from 0 to `floor_width`) and a wall (x = 0, z from 0 to 30), meeting along y.
fn floor_and_wall(floor_width: f64) -> (RollingBallSide, RollingBallSide) {
    let mut positions = Vec::new();
    let floor = rectangle(
        &mut positions,
        Vector3::new(0.0, -10.0, 0.0),
        Vector3::new(floor_width, 0.0, 0.0),
        Vector3::new(0.0, 20.0, 0.0),
        6,
        4,
    );
    let wall = rectangle(
        &mut positions,
        Vector3::new(0.0, -10.0, 0.0),
        Vector3::new(0.0, 20.0, 0.0),
        Vector3::new(0.0, 0.0, 30.0),
        4,
        6,
    );
    sides(&positions, &floor, &wall)
}

#[test]
fn in_a_square_concave_corner_the_ball_sits_a_radius_from_both_faces() {
    let sides = floor_and_wall(30.0);
    let station = solve(
        &sides,
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        (Vector3::UNIT_Z, Vector3::UNIT_X),
        false,
        Vector3::new(0.7, 0.0, -1.1),
    );
    assert!(
        close(station.centre, Vector3::new(5.0, 1.0, 5.0), EXACT),
        "{station:?}"
    );
    assert!(close(
        station.contact_a.point,
        Vector3::new(5.0, 1.0, 0.0),
        EXACT
    ));
    assert!(close(
        station.contact_b.point,
        Vector3::new(0.0, 1.0, 5.0),
        EXACT
    ));
    assert!(!station.contact_a.feature.is_rim() && !station.contact_b.feature.is_rim());
    assert!(station.steps > 0, "the nudge leaves Newton work to do");
}

#[test]
fn in_a_sixty_degree_wedge_the_contacts_stand_the_cotangent_setback_out() {
    // A wall rising at 60 degrees from the floor: the ball's centre is on the 30 degree bisector,
    // 2r out, and each contact r cot 30 = r sqrt 3 from the edge.
    let mut positions = Vec::new();
    let floor = rectangle(
        &mut positions,
        Vector3::new(0.0, -10.0, 0.0),
        Vector3::new(30.0, 0.0, 0.0),
        Vector3::new(0.0, 20.0, 0.0),
        5,
        4,
    );
    let rising = Vector3::new(0.5, 0.0, 3.0_f64.sqrt() / 2.0);
    let wall = rectangle(
        &mut positions,
        Vector3::new(0.0, -10.0, 0.0),
        Vector3::new(0.0, 20.0, 0.0),
        rising * 30.0,
        4,
        5,
    );
    let sides = sides(&positions, &floor, &wall);
    let wall_normal = Vector3::new(3.0_f64.sqrt() / 2.0, 0.0, -0.5);
    let station = solve(
        &sides,
        Vector3::ZERO,
        Vector3::UNIT_Y,
        (Vector3::UNIT_Z, wall_normal),
        false,
        Vector3::new(-0.8, 0.0, 0.6),
    );
    let setback = RADIUS * 3.0_f64.sqrt();
    assert!(
        close(station.centre, Vector3::new(setback, 0.0, RADIUS), EXACT),
        "{station:?}"
    );
    assert!(close(
        station.contact_a.point,
        Vector3::new(setback, 0.0, 0.0),
        EXACT
    ));
    assert!(close(station.contact_b.point, rising * setback, EXACT));
}

#[test]
fn on_a_convex_edge_the_ball_sits_inside_the_solid() {
    // A box edge: top face z = 0 over x < 0, side face x = 0 below z = 0, the solid between them.
    let mut positions = Vec::new();
    let top = rectangle(
        &mut positions,
        Vector3::new(-30.0, -10.0, 0.0),
        Vector3::new(30.0, 0.0, 0.0),
        Vector3::new(0.0, 20.0, 0.0),
        5,
        4,
    );
    let side = rectangle(
        &mut positions,
        Vector3::new(0.0, -10.0, -30.0),
        Vector3::new(0.0, 20.0, 0.0),
        Vector3::new(0.0, 0.0, 30.0),
        4,
        5,
    );
    let sides = sides(&positions, &top, &side);
    let station = solve(
        &sides,
        Vector3::ZERO,
        Vector3::UNIT_Y,
        (Vector3::UNIT_Z, Vector3::UNIT_X),
        true,
        Vector3::new(0.9, 0.0, 0.4),
    );
    assert!(
        close(station.centre, Vector3::new(-5.0, 0.0, -5.0), EXACT),
        "{station:?}"
    );
    assert!(close(
        station.contact_a.point,
        Vector3::new(-5.0, 0.0, 0.0),
        EXACT
    ));
    assert!(close(
        station.contact_b.point,
        Vector3::new(0.0, 0.0, -5.0),
        EXACT
    ));
}

#[test]
fn a_cylinder_standing_on_a_plate_takes_the_ball_a_radius_off_its_facet() {
    // Facets centred on angle 0, so the chain point is mid-facet and the facet is the plane the
    // ball touches: the faceted answer is exact.
    let (cylinder, sides_around) = (10.0, 720);
    let mut positions = Vec::new();
    let plate = rectangle(
        &mut positions,
        Vector3::new(-40.0, -40.0, 0.0),
        Vector3::new(80.0, 0.0, 0.0),
        Vector3::new(0.0, 80.0, 0.0),
        8,
        8,
    );
    let wall = cylinder_wall(&mut positions, cylinder, 30.0, sides_around, -0.5, 6);
    let sides = sides(&positions, &plate, &wall);
    let facet = cylinder * (std::f64::consts::PI / sides_around as f64).cos();
    let station = solve(
        &sides,
        Vector3::new(facet, 0.0, 0.0),
        Vector3::UNIT_Y,
        (Vector3::UNIT_Z, Vector3::UNIT_X),
        false,
        Vector3::new(1.2, 0.0, -0.9),
    );
    assert!(
        close(
            station.centre,
            Vector3::new(facet + RADIUS, 0.0, RADIUS),
            EXACT
        ),
        "{station:?}"
    );
    assert!(close(
        station.contact_a.point,
        Vector3::new(facet + RADIUS, 0.0, 0.0),
        EXACT
    ));
    assert!(close(
        station.contact_b.point,
        Vector3::new(facet, 0.0, RADIUS),
        EXACT
    ));
}

#[test]
fn where_the_floor_runs_out_the_ball_rests_on_its_rim() {
    // The floor is 3 wide and the ball's reach 5: the ball touches the wall a radius out and rests
    // on the floor's far edge, so its centre stands sqrt(r^2 - (r - w)^2) up.
    // Read at y = 1, between the floor grid's vertices, so the contact is on the rim's edge.
    let width = 3.0;
    let sides = floor_and_wall(width);
    let station = solve(
        &sides,
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::UNIT_Y,
        (Vector3::UNIT_Z, Vector3::UNIT_X),
        false,
        Vector3::ZERO,
    );
    let up = ((RADIUS * RADIUS) - ((RADIUS - width) * (RADIUS - width))).sqrt();
    assert!(
        close(station.centre, Vector3::new(RADIUS, 1.0, up), EXACT),
        "{station:?}"
    );
    assert!(close(
        station.contact_a.point,
        Vector3::new(width, 1.0, 0.0),
        EXACT
    ));
    assert!(
        matches!(
            station.contact_a.feature,
            ContactFeature::Edge { rim: true, .. }
        ),
        "the floor's contact is its rim: {:?}",
        station.contact_a.feature
    );
    assert!(close(
        station.contact_b.point,
        Vector3::new(0.0, 1.0, up),
        EXACT
    ));
}

#[test]
fn a_sphere_sunk_in_a_plate_takes_the_ball_where_the_smooth_sphere_would() {
    // Sphere radius 20 about (0, 0, -5): the ball rests on the plate (centre at z = r) and on the
    // sphere (centre R + r from the sphere's), so it stands at rho = sqrt((R + r)^2 - (r + 5)^2).
    let (big, sunk) = (20.0, 5.0);
    let mut positions = Vec::new();
    let plate = rectangle(
        &mut positions,
        Vector3::new(-40.0, -40.0, 0.0),
        Vector3::new(80.0, 0.0, 0.0),
        Vector3::new(0.0, 80.0, 0.0),
        8,
        8,
    );
    let ball = sphere(&mut positions, Vector3::new(0.0, 0.0, -sunk), big, 360, 180);
    let sides = sides(&positions, &plate, &ball);
    let rim = ((big * big) - (sunk * sunk)).sqrt();
    let sphere_normal = Vector3::new(rim, 0.0, sunk) / big;
    let station = solve(
        &sides,
        Vector3::new(rim, 0.0, 0.0),
        Vector3::UNIT_Y,
        (Vector3::UNIT_Z, sphere_normal),
        false,
        Vector3::ZERO,
    );
    let rho = (((big + RADIUS) * (big + RADIUS)) - ((RADIUS + sunk) * (RADIUS + sunk))).sqrt();
    // The centre is held by the two distances, so it is off the smooth answer by about the facets'
    // chord, 20 (1 - cos 0.5 degrees) = 7.6e-4. The contact is the centre's projection onto a flat
    // facet, which slides across the facet as far as r sin(half a facet's angle) from where the
    // smooth sphere's would be.
    let chord = 3e-3;
    let half_facet = (0.5_f64).to_radians();
    let slide = (RADIUS * half_facet.sin()) + (big * (1.0 - half_facet.cos()));
    assert!(
        close(station.centre, Vector3::new(rho, 0.0, RADIUS), chord),
        "{station:?}"
    );
    assert!(close(
        station.contact_a.point,
        Vector3::new(rho, 0.0, 0.0),
        chord
    ));
    let on_sphere = Vector3::new(0.0, 0.0, -sunk)
        + ((Vector3::new(rho, 0.0, RADIUS) - Vector3::new(0.0, 0.0, -sunk)).get_normal() * big);
    assert!(
        close(station.contact_b.point, on_sphere, slide),
        "sphere contact {:?} against {:?}: {}",
        station.contact_b.point,
        on_sphere,
        (station.contact_b.point - on_sphere).length()
    );
}

#[test]
fn a_contact_whose_surface_lies_wholly_past_its_halves_cannot_be_followed() {
    // A contact on the floor 27.5 out from the wall, followed within the half x <= 10: every
    // triangle round it lies past that plane, though the floor still has surface inside it. The
    // contact cannot be followed there without leaping, and the station says so rather than
    // calling the floor empty.
    let (a, b) = floor_and_wall(30.0);
    let contact = a.closest(Vector3::new(27.5, 1.0, 1.0)).unwrap();
    let half = HalfSpace {
        origin: Vector3::new(10.0, 0.0, 0.0),
        normal: -Vector3::UNIT_X,
    };
    assert!(a
        .closest_in(Vector3::new(5.0, 0.0, 5.0), Some(half))
        .is_some());
    let halves = [half];
    let station = ball_station_reading(
        (
            &a,
            SideReading::TrackedFrom {
                contact,
                halves: &halves,
            },
        ),
        (&b, SideReading::Nearest(&[])),
        Vector3::ZERO,
        Vector3::UNIT_Y,
        Vector3::new(RADIUS, 0.0, RADIUS),
        RADIUS,
    );
    assert_eq!(
        station,
        Err(BallStationError::ContactNotFollowed { side_b: false })
    );
}
