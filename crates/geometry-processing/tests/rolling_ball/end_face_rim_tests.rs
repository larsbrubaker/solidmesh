//! A side whose end-face contact rests on the side's own rim: a wall exactly as tall as the
//! fillet, picked along its top edge between two square end faces. The ball rests on the wall's
//! bottom rim all along, and at each end face its contact is the wall's bottom corner there, the
//! side's own boundary where it meets the end face, not a hole in the side as extended past the
//! face. The ball takes the pass, the wall's rail is its bottom rim, and the round fills closed.

use geometry_processing::bevel::level_line::gap_fill::DEFAULT_FILLET_SEGMENTS;
use geometry_processing::bevel::level_line::BandGapRefusal;
use vector_math::Vector3;

use super::band_route_tests::{a_box, built, Triangles};
use super::rounded_on_past_the_end_tests::filled;
use geometry_processing::bevel::rolling_ball::rolling_ball_routing::{
    ROLLING_BALL_REFUSED, ROLLING_BALL_ROUTE,
};

/// The fillet's radius, and the front wall's height.
const R: f64 = 1.5;
/// The picked edge's length.
const LENGTH: f64 = 10.0;

/// A wall exactly the radius tall: the box's front face (y = -5) runs from its top edge, the pick
/// (corners 4 to 5, along x), down R to its bottom rim; its top is 10 deep, so only the wall is
/// consumed. Each end face (x = -5 and x = 5) is square to the edge, and the ball's contact on the
/// wall there is the box's bottom front corner.
#[test]
fn an_end_face_contact_on_the_sides_own_rim_takes_the_ball_and_fills_closed() {
    let part = a_box(Vector3::new(-5.0, -5.0, 0.0), Vector3::new(5.0, 5.0, R));
    let (ball, route) = built(&part, &[vec![4, 5]], R, true);
    assert_eq!(
        ball.refusal,
        BandGapRefusal::None,
        "{:?}",
        ball.rolling_ball_refusal
    );
    assert_eq!(route.as_deref(), Some(ROLLING_BALL_ROUTE));
    // A square corner: both rails stand the radius from the edge, the wall's on its bottom rim.
    for setback in &ball.chains[0].setbacks {
        assert!((setback - R).abs() < 1e-8, "{setback}");
    }

    let round = filled(&ball).unwrap_or_else(|why| panic!("the fill: {why}"));
    assert!(round.closed);
    assert!(!round.crosses_itself);
    // By hand: the round removes the square of the radius less the rows' section along the edge,
    // n chords of the quarter turn about the ball's centre, each a triangle of area
    // R^2 sin(pi / 2n) / 2, and its end caps lie flat in the end faces. The rows are spaced by the
    // arc's parameter, not evenly by angle, so the chords' section is near this, not exactly it.
    let box_volume = 10.0 * 10.0 * R;
    let n = f64::from(DEFAULT_FILLET_SEGMENTS);
    let section = (n / 2.0) * (std::f64::consts::FRAC_PI_2 / n).sin();
    let by_hand = (1.0 - section) * R * R * LENGTH;
    let removed = box_volume - round.volume;
    assert!(
        (removed - by_hand).abs() < 1e-3 * by_hand,
        "{removed} against {by_hand}"
    );
    // On flat faces the ball's setbacks are the angle's, so the round is the level line's own.
    let (level_line, _) = built(&part, &[vec![4, 5]], R, false);
    let line = filled(&level_line).unwrap_or_else(|why| panic!("the level line's fill: {why}"));
    assert!(
        (round.volume - line.volume).abs() < 1e-9 * box_volume,
        "{} against the level line's {}",
        round.volume,
        line.volume
    );
}

/// A cube of edge 2, its top front edge asked for 2.2: wider than either face. The ball rests on
/// both faces' far rims, its centre outside the cube, touching neither face square to it; its
/// contacts at the end faces lie on the faces' far corners. A ball that only rests on a rim
/// there, not tangent to the side, gives no round of the size asked (a 2.0 round for a 2.2
/// request), so the size is refused, never built smaller: by its size, naming the faces' width,
/// the room the ball's own walk at a smaller size reads (`rolling_ball_size_room`), where the ball
/// builds. The route still says why the ball refused 2.2.
#[test]
fn a_cube_edge_over_its_faces_width_is_refused_by_size_naming_the_faces_width() {
    let part = a_box(Vector3::new(-1.0, -1.0, -1.0), Vector3::new(1.0, 1.0, 1.0));
    let (ball, route) = built(&part, &[vec![4, 5]], 2.2, true);
    assert_eq!(ball.refusal, BandGapRefusal::Size, "{route:?}");
    assert!(ball.mesh.is_none(), "{route:?}");
    assert!((ball.room - 2.0).abs() < 1e-8, "{}", ball.room);
    let why = route.unwrap_or_default();
    assert!(why.starts_with(ROLLING_BALL_REFUSED), "{why}");
    assert!(
        why.contains("end face rests on the side's rim without touching the side square"),
        "{why}"
    );
    let (at_the_room, at_route) = built(&part, &[vec![4, 5]], ball.room, true);
    assert_eq!(at_the_room.refusal, BandGapRefusal::None, "{at_route:?}");
    assert_eq!(at_route.as_deref(), Some(ROLLING_BALL_ROUTE));
}

/// A bar 3 tall and 10 deep along x, its end at x = 5 chamfered at 45 degrees at the bottom (the
/// front face's outline (-5, 0), (3.5, 0), (5, 1.5), (5, 3), (-5, 3) in x and z, extruded from
/// y = -1 to 9), its top front edge picked at 2.5. Along the edge the ball is tangent to the
/// front face 0.5 up, but at the end face the front face is only 1.5 tall, so a ball there would
/// rest on the chamfer's rim, the side's own rim, which the end rule alone would accept only if the
/// ball touched the front face square. Here it is refused before that, as the walk reaches the
/// end: the ball's round would cut into the chamfer (`ReachesPastThePart`). That reach check is
/// what refuses an end whose side curves back or is cut short (`side_extension.rs`'s header). The
/// size is refused, never built smaller: by its size, naming the room the ball's own walk at a
/// smaller size reads (`rolling_ball_size_room`), where the ball builds; the route still says why
/// the ball refused 2.5.
#[test]
fn an_end_whose_side_is_chamfered_short_of_the_ball_is_refused_by_size() {
    let outline = [(-5.0, 0.0), (3.5, 0.0), (5.0, 1.5), (5.0, 3.0), (-5.0, 3.0)];
    let mut positions = Vec::new();
    for y in [-1.0, 9.0] {
        for &(x, z) in &outline {
            positions.push(Vector3::new(x, y, z));
        }
    }
    let n = outline.len() as i32;
    let mut triangles: Triangles = Vec::new();
    for i in 1..n - 1 {
        triangles.push((0, i, i + 1));
        triangles.push((n, n + i, n + i + 1));
    }
    for i in 0..n {
        let j = (i + 1) % n;
        triangles.push((i, j, n + j));
        triangles.push((i, n + j, n + i));
    }
    // Wind every triangle away from the bar's centre (it is convex).
    let centre = Vector3::new(0.0, 4.0, 1.5);
    let triangles: Triangles = triangles
        .into_iter()
        .map(|(a, b, c)| {
            let (pa, pb, pc) = (
                positions[a as usize],
                positions[b as usize],
                positions[c as usize],
            );
            if (pb - pa).cross(pc - pa).dot(pa - centre) < 0.0 {
                (a, c, b)
            } else {
                (a, b, c)
            }
        })
        .collect();
    // The top front edge, (-5, -1, 3) to (5, -1, 3).
    let chain = vec![4, 3];
    assert_eq!(positions[4], Vector3::new(-5.0, -1.0, 3.0));
    assert_eq!(positions[3], Vector3::new(5.0, -1.0, 3.0));
    let part = (positions, triangles);
    let (ball, route) = built(&part, std::slice::from_ref(&chain), 2.5, true);
    assert_eq!(ball.refusal, BandGapRefusal::Size, "{route:?}");
    assert!(ball.mesh.is_none(), "{route:?}");
    assert!(ball.room > 0.0 && ball.room < 2.5, "{}", ball.room);
    let why = route.unwrap_or_default();
    assert!(why.starts_with(ROLLING_BALL_REFUSED), "{why}");
    assert!(why.contains("ReachesPastThePart"), "{why}");
    let (at_the_room, at_route) = built(&part, &[chain], ball.room, true);
    eprintln!(
        "chamfered end: room {} named, {:?} there",
        ball.room, at_the_room.refusal
    );
    assert_eq!(at_the_room.refusal, BandGapRefusal::None, "{at_route:?}");
    assert_eq!(at_route.as_deref(), Some(ROLLING_BALL_ROUTE));
}
