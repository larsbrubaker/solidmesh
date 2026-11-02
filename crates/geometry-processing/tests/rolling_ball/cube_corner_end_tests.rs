//! The ball at a corner end of a chain that turns round a cube's face: the rim of two adjacent
//! faces of a 2 cube picked as three open chains (the edge the faces share, and the rest of each
//! face's rim), meeting at the shared edge's two corners, at 0.9 of the pick's room (1). With no
//! bend threshold the front rim's chain is walked whole round its two turns, so one of its sides is
//! three faces (right, bottom, left), and the ball rolled from the bottom-left corner up the left
//! edge to the top corner follows that side's contact from the bottom face. The corner's end
//! station is the ball held in the end face's plane a radius from the left and front faces, whose
//! setbacks are the angle's, r tan(45 degrees); the right face, the same side's far wall, stands
//! 1.1 from that ball and must never take its contact. Each chain's setbacks, both sides, are held
//! to the angle's at every chain point. The rule that fixes it (`contact_tracking.rs`): where the
//! followed contact's face ends across a crease, a ball touching one side at two places rolls on
//! from the touch ahead of it; where the face carries on it is followed from its contact alone.
//! Both are held directly on that three-face ball's side.

use geometry_processing::bevel::level_line::band_gap_setback_fields::PassSetbacks;
use geometry_processing::bevel::level_line::rolling_ball_setbacks::{for_the_pass, SideSetbacks};
use geometry_processing::bevel::level_line::BandProfile;
use geometry_processing::bevel::rolling_ball::rolling_ball_routing::{
    last_route, reset_to_the_default, set_last_route, set_use_rolling_ball, ROLLING_BALL_ROUTE,
};
use geometry_processing::bevel::rolling_ball::{RollingBallSide, TouchedFrom};
use vector_math::Vector3;

use crate::band_route_tests::{creases, Triangles};

/// How far a setback per unit of size may stand from the angle's on flat faces: the end station
/// is held in its end face's plane to 1e-9 of the radius, and every solve to 1e-11.
const AGREE: f64 = 1e-9;

/// A cube of half-size 1, wound outward, each face split along one diagonal.
fn a_cube() -> (Vec<Vector3>, Triangles) {
    let positions = vec![
        Vector3::new(-1.0, -1.0, -1.0),
        Vector3::new(1.0, -1.0, -1.0),
        Vector3::new(1.0, 1.0, -1.0),
        Vector3::new(-1.0, 1.0, -1.0),
        Vector3::new(-1.0, -1.0, 1.0),
        Vector3::new(1.0, -1.0, 1.0),
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(-1.0, 1.0, 1.0),
    ];
    let triangles = vec![
        (0, 2, 1),
        (0, 3, 2),
        (4, 5, 6),
        (4, 6, 7),
        (0, 1, 5),
        (0, 5, 4),
        (1, 2, 6),
        (1, 6, 5),
        (2, 3, 7),
        (2, 7, 6),
        (3, 0, 4),
        (3, 4, 7),
    ];
    (positions, triangles)
}

/// The top-front edge 4-5, the rest of the top rim and the rest of the front rim.
fn two_faces() -> Vec<Vec<i32>> {
    vec![vec![4, 5], vec![5, 6, 7, 4], vec![5, 1, 0, 4]]
}

/// The ball's setbacks for the two faces' rims at `size`, with no bend threshold (as a build's
/// default options give: no chain is walked in legs), after checking the pass took the ball.
fn the_balls(size: f64) -> Vec<SideSetbacks> {
    let (positions, triangles) = a_cube();
    let chains = two_faces();
    let closed = vec![false; chains.len()];
    let crease = creases(&positions, &triangles);
    let band = PassSetbacks::of(
        &positions,
        &triangles,
        &chains,
        &closed,
        f64::INFINITY,
        BandProfile::Fillet,
    )
    .expect("the band's setbacks");
    set_use_rolling_ball(true);
    set_last_route(None);
    let setbacks = for_the_pass(
        &positions,
        &triangles,
        &chains,
        &closed,
        Some(&*crease),
        None,
        size,
        BandProfile::Fillet,
        &band,
        false,
    );
    let route = last_route();
    reset_to_the_default();
    assert_eq!(route.as_deref(), Some(ROLLING_BALL_ROUTE), "at {size}");
    setbacks.expect("the ball takes the two faces' rims")
}

/// At 0.9 of the room, every setback of every chain, both sides, is the angle's (1 per unit of
/// size on a cube), the front rim's top-left corner included, whose end station is centred a
/// radius in from the left and front faces in the top face's plane.
#[test]
fn a_corner_end_rolled_up_from_a_turn_keeps_the_angles_setbacks_near_the_room() {
    for size in [0.1, 0.5, 0.9] {
        let balls = the_balls(size);
        for (k, ball) in balls.iter().enumerate() {
            for (side, setbacks) in [&ball.left, &ball.right].into_iter().enumerate() {
                for (i, got) in setbacks.iter().enumerate() {
                    assert!(
                        (got - 1.0).abs() <= AGREE,
                        "at {size}: chain {k} side {side} point {i}: {got} for 1"
                    );
                }
            }
        }

        // The front rim's last point is the top-left corner (vertex 4): its ball is centred a
        // radius in from the left (x = -1) and front (y = -1) faces, in the top face's plane.
        let corner = balls[2].centres[3];
        let want = Vector3::new(1.0, 1.0, 0.0);
        assert!(
            (corner - want).length() <= AGREE,
            "at {size}: the corner's centre {corner:?} per unit, for {want:?}"
        );
    }
}

/// The three-face ball at the front rim's bottom-left corner (centre 0.9 in from the bottom, left
/// and front faces) touches its side of right, bottom and left faces on the bottom (its contact,
/// the lower triangle of two equally near) and on the left. Rolling on up the left edge, the left
/// face's touch is ahead and is a start for the next contact; rolling back along the bottom edge
/// it is behind, the face the ball leaves, and is none.
#[test]
fn a_ball_touching_one_side_twice_rolls_on_from_the_touch_ahead() {
    let (positions, triangles) = a_cube();
    // The bottom, right and left faces, in the cube's triangle order.
    let side_triangles: Vec<[i32; 3]> = [0, 1, 6, 7, 10, 11]
        .into_iter()
        .map(|t| {
            let (a, b, c) = triangles[t];
            [a, b, c]
        })
        .collect();
    let side = RollingBallSide::new(&positions, &side_triangles);
    let centre = Vector3::new(-0.1, -0.1, -0.1);
    let contact = side.closest(centre).expect("the side holds triangles");
    assert!((contact.point - Vector3::new(-0.1, -0.1, -1.0)).length() <= AGREE);

    // The left face's lower triangle (3, 0, 4) holds the touch at (-1, -0.1, -0.1).
    let up_the_left_edge = side.touched_elsewhere(centre, &contact, Vector3::new(0.0, 0.0, 2.0));
    assert_eq!(up_the_left_edge, vec![4]);
    let back_along_the_bottom =
        side.touched_elsewhere(centre, &contact, Vector3::new(2.0, 0.0, 0.0));
    assert_eq!(back_along_the_bottom, Vec::<usize>::new());
}

/// Rolled from the three-face ball at the bottom-left corner straight to the top corner's end
/// station, the bottom contact's descent leaves the bottom face across the side's crease and
/// settles on the right face, 1.1 off: the bottom face ends there, so the left face's touch ahead
/// is a start too, and the contact is the left face's, 0.9 off. Rolled only part way up, where the
/// bottom still stands nearest, the bottom face carries the contact on and the touch is never read:
/// the answer is the followed contact's own.
#[test]
fn the_touch_ahead_is_read_only_where_the_contacts_face_ends() {
    let (positions, triangles) = a_cube();
    let side_triangles: Vec<[i32; 3]> = [0, 1, 6, 7, 10, 11]
        .into_iter()
        .map(|t| {
            let (a, b, c) = triangles[t];
            [a, b, c]
        })
        .collect();
    let side = RollingBallSide::new(&positions, &side_triangles);
    let centre = Vector3::new(-0.1, -0.1, -0.1);
    let contact = side.closest(centre).expect("the side holds triangles");
    let touched = Some(TouchedFrom {
        centre,
        ahead: Vector3::new(0.0, 0.0, 2.0),
    });

    let top = Vector3::new(-0.1, -0.1, 1.0);
    let followed = side.closest_tracked(top, &contact, &[]).expect("followed");
    assert!((followed.point - Vector3::new(1.0, -0.1, 1.0)).length() <= AGREE);
    let rolled = side
        .closest_tracked_from_any(top, &contact, touched, &[])
        .expect("rolled");
    assert!(
        (rolled.point - Vector3::new(-1.0, -0.1, 1.0)).length() <= AGREE,
        "{:?}",
        rolled.point
    );

    let part_way = Vector3::new(-0.1, -0.1, -0.5);
    let followed = side
        .closest_tracked(part_way, &contact, &[])
        .expect("followed");
    let rolled = side
        .closest_tracked_from_any(part_way, &contact, touched, &[])
        .expect("rolled");
    assert_eq!(rolled, followed);
}
