//! Revolve's volume on profiles of every shape a sketch hands it: a square, a square with a notch
//! cut into its outer edge (a concave outline), a square and a circle overlapping as two contours
//! (filled even-odd, as
//! extrude fills them, so the overlap is left out), a square with a hole wholly inside, and a profile
//! touching the axis. Each solid is checked closed and outward and its volume against Pappus for
//! the faceted solid: a sweep of `steps` strips through `angle` encloses
//! `steps * sin(angle / steps) * ∫∫ x dA` of the profile as snapped onto the micron grid, since
//! each strip's cross-section at any height is a fan of triangles of area `r² sin(Δ) / 2`.

use solidmesh_math::math_helper::TAU;
use solidmesh_math::Vector2;
use solidmesh_mesh::processors::outline_to_mesh::{contours_to_polygons, revolve_contours};

use super::solid_checks::assert_closed_outward;

/// Segments of the test circle: a multiple of four, so it has corners exactly at its top and
/// bottom, where it meets the square's edge.
const CIRCLE_SEGMENTS: usize = 32;

/// How far a revolve's volume may stray from Pappus, as a fraction of it: the mesh holds f32
/// positions, about 1e-7 of a coordinate, and the clean-up merges points under 1.4 microns apart.
const VOLUME_FRACTION: f64 = 1e-5;

fn contour(points: &[(f64, f64)]) -> Vec<Vector2> {
    points.iter().map(|&(x, y)| Vector2::new(x, y)).collect()
}

/// The square 20 to 40 mm from the axis, 20 tall, counter clockwise.
fn square() -> Vec<Vector2> {
    contour(&[(20.0, 0.0), (40.0, 0.0), (40.0, 20.0), (20.0, 20.0)])
}

/// The corner of the 8 mm circle centred on the square's outer edge at `index` (counter
/// clockwise from the +x side).
fn circle_point(index: usize) -> Vector2 {
    let angle = TAU * index as f64 / CIRCLE_SEGMENTS as f64;
    Vector2::new(40.0 + 4.0 * angle.cos(), 10.0 + 4.0 * angle.sin())
}

/// The circle as a counter clockwise contour of its own.
fn circle() -> Vec<Vector2> {
    (0..CIRCLE_SEGMENTS).map(circle_point).collect()
}

/// The square with the circle subtracted: its outer edge runs up to the circle's bottom, round the
/// circle's left half inside the square, and on up from the circle's top.
fn notched_square() -> Vec<Vector2> {
    let quarter = CIRCLE_SEGMENTS / 4;
    let mut points = contour(&[(20.0, 0.0), (40.0, 0.0), (40.0, 6.0)]);
    // from the bottom (three quarters round) back clockwise through the left to the top
    points.extend((quarter..=3 * quarter).rev().map(circle_point).skip(1));
    points.pop();
    points.extend(contour(&[(40.0, 14.0), (40.0, 20.0), (20.0, 20.0)]));
    points
}

/// The circle's right half, the part outside the square, counter clockwise from its bottom
/// through its right to its top and closed down the square's edge.
fn circle_outside_square() -> Vec<Vector2> {
    let quarter = CIRCLE_SEGMENTS / 4;
    (3 * quarter..CIRCLE_SEGMENTS)
        .chain(0..=quarter)
        .map(circle_point)
        .collect()
}

/// `∫∫ x dA` over the contours as snapped onto the micron grid, in mm³, each contour signed by
/// its winding (a clockwise hole subtracts).
fn first_moment(contours: &[Vec<Vector2>]) -> f64 {
    let mut sum = 0.0;
    for path in contours_to_polygons(contours) {
        for i in 0..path.len() {
            let (a, b) = (path[i], path[(i + 1) % path.len()]);
            let (ax, ay) = (a.x as f64 / 1000.0, a.y as f64 / 1000.0);
            let (bx, by) = (b.x as f64 / 1000.0, b.y as f64 / 1000.0);
            sum += (ax + bx) * (ax * by - bx * ay);
        }
    }
    sum / 6.0
}

/// Pappus for the faceted sweep of `steps` strips through `angle`, of a profile with first
/// moment `moment`.
fn faceted_pappus(moment: f64, steps: i32, angle: f64) -> f64 {
    f64::from(steps) * (angle / f64::from(steps)).sin() * moment
}

/// Revolves `contours` through `angle` in `steps` strips and checks the solid is closed and
/// outward with the volume of `filled` (the outline the contours fill).
fn assert_revolves_to(contours: &[Vec<Vector2>], filled: &[Vec<Vector2>], steps: i32, angle: f64) {
    let mesh = revolve_contours(contours, steps, 0.0, angle, false).expect("the profile revolves");
    assert_closed_outward(&mesh, "revolve");
    let expected = faceted_pappus(first_moment(filled), steps, angle);
    let volume = mesh.get_volume();
    assert!(
        (volume - expected).abs() <= expected * VOLUME_FRACTION,
        "volume {volume}, expected {expected}"
    );
}

#[test]
fn a_square_revolves_to_its_volume() {
    assert_revolves_to(&[square()], &[square()], 40, TAU);
}

#[test]
fn a_notched_square_revolves_to_its_volume() {
    assert_revolves_to(&[notched_square()], &[notched_square()], 40, TAU);
}

#[test]
fn a_notched_square_revolves_part_way_to_its_volume() {
    assert_revolves_to(&[notched_square()], &[notched_square()], 20, TAU / 2.0);
}

#[test]
fn a_notched_square_wound_clockwise_revolves_to_its_volume() {
    let mut clockwise = notched_square();
    clockwise.reverse();
    assert_revolves_to(&[clockwise], &[notched_square()], 40, TAU);
}

#[test]
fn overlapping_contours_revolve_even_odd() {
    // even-odd fills what one contour covers and not what both do: the notched square and the
    // circle's half outside it
    let filled = [notched_square(), circle_outside_square()];
    assert_revolves_to(&[square(), circle()], &filled, 40, TAU);
}

#[test]
fn a_square_with_a_hole_inside_revolves_to_its_volume() {
    let hole = contour(&[(25.0, 5.0), (25.0, 15.0), (35.0, 15.0), (35.0, 5.0)]);
    let contours = [square(), hole];
    assert_revolves_to(&contours, &contours, 40, TAU);
}

#[test]
fn a_profile_touching_the_axis_revolves_to_its_volume() {
    let touching = [contour(&[
        (0.0, 0.0),
        (10.0, 0.0),
        (10.0, 20.0),
        (0.0, 20.0),
    ])];
    assert_revolves_to(&touching, &touching, 40, TAU);
    assert_revolves_to(&touching, &touching, 10, TAU / 4.0);
}
