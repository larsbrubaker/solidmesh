//! A growing neighbour that is one facet of a faceted wall: the seam between two facets whose
//! creases with the picked face carry on as one curve does not bound the room, while a seam where
//! the crease turns a corner still does. The rib of `strip_clip_rib_meshes` with its left flank
//! bent at a ridge P R out of the flank's far end P: the flank's crease runs T P, then carries on
//! P P2, 5.6 degrees off its line, the two flank facets folding 5.7 degrees at P R. At the back the
//! crease turns square (P2 Q) and at the right the flank's crease T Q meets the back face's crease
//! turning back, so the far creases P2 R and Q R still bound.

use geometry_processing::bevel::level_line::band_gap_reading;
use geometry_processing::bevel::level_line::{BandProfile, RoomReading, RoomReadingKind};
use vector_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;

const T: Vector3 = Vector3::new(10.0, 3.0, 10.0);
const P: Vector3 = Vector3::new(9.0, 8.0, 10.0);
const P2: Vector3 = Vector3::new(8.5, 13.0, 10.0);
const Q: Vector3 = Vector3::new(11.5, 13.0, 10.0);
const R: Vector3 = Vector3::new(10.0, 10.0, 20.0);

/// Box corners bottom 0-3 (20 below the top, so the front face's own bottom edge, 20 from the
/// chain, bounds nothing the rib does), top 4-7; the rib's tip T 8, P 9, P2 10, Q 11 and its apex
/// R 12.
fn positions() -> Vec<Vector3> {
    positions_with(P)
}

/// [`positions`] with the flank's bend at `p`.
fn positions_with(p: Vector3) -> Vec<Vector3> {
    vec![
        Vector3::new(0.0, 0.0, -10.0),
        Vector3::new(20.0, 0.0, -10.0),
        Vector3::new(20.0, 20.0, -10.0),
        Vector3::new(0.0, 20.0, -10.0),
        Vector3::new(0.0, 0.0, 10.0),
        Vector3::new(20.0, 0.0, 10.0),
        Vector3::new(20.0, 20.0, 10.0),
        Vector3::new(0.0, 20.0, 10.0),
        T,
        p,
        P2,
        Q,
        R,
    ]
}

/// The box's sides and bottom, the top round the rib's base T Q P2 P (counter-clockwise from
/// above), and the rib: the two left flank facets P T R and P2 P R, the back Q P2 R and the right
/// flank T Q R.
fn indices() -> Vec<(i32, i32, i32)> {
    vec![
        (0, 2, 1),
        (0, 3, 2),
        (0, 1, 5),
        (0, 5, 4),
        (1, 2, 6),
        (1, 6, 5),
        (2, 3, 7),
        (2, 7, 6),
        (3, 0, 4),
        (3, 4, 7),
        (4, 5, 8),
        (5, 11, 8),
        (5, 6, 11),
        (6, 10, 11),
        (6, 7, 10),
        (7, 9, 10),
        (7, 4, 9),
        (4, 8, 9),
        (9, 8, 12),
        (10, 9, 12),
        (11, 10, 12),
        (8, 11, 12),
    ]
}

/// The box's top front edge.
const CHAIN: [i32; 2] = [4, 5];

fn read() -> RoomReading {
    read_on(&positions(), &indices())
}

fn read_on(positions: &[Vector3], indices: &[(i32, i32, i32)]) -> RoomReading {
    let is_feature_edge = feature_edges_over(positions, indices, 20.0);
    band_gap_reading::read_room(
        positions,
        indices,
        &CHAIN,
        false,
        Some(is_feature_edge),
        BandProfile::Chamfer,
        1e-9,
        None,
        0.0,
        f64::INFINITY,
    )
    .expect("the reading")
}

fn distance_to_segment(point: Vector3, a: Vector3, b: Vector3) -> f64 {
    let along = b - a;
    let t = ((point - a).dot(along) / along.length_squared()).clamp(0.0, 1.0);
    (point - (a + (along * t))).length()
}

/// The seam P R between the two left flank facets is no far crease: the band runs on past P,
/// both facets growing their caps, and the room is set where the crease turns the back corner.
#[test]
fn the_seam_between_two_facets_of_a_growing_wall_does_not_bound() {
    let reading = read();
    let said = format!("room {:?}, set by {:?}", reading.room, reading.room_set_by);

    assert_eq!(reading.room_set_by.kind, RoomReadingKind::Foreign, "{said}");
    assert!(reading.room_set_by.is_a_growing_crease(), "{said}");
    let at = reading.room_set_by.crease_at.expect("CreaseAt");
    assert!(
        distance_to_segment(at, P, R) > 1e-6,
        "the facet seam P R still bounds: {said}"
    );
    let on_a_corner_crease = distance_to_segment(at, P2, R).min(distance_to_segment(at, Q, R));
    assert!(
        on_a_corner_crease < 1e-9,
        "CreaseAt {at:?} should stand on P2 R or Q R: {said}"
    );
    // P, the seam's foot, stands 8 from the chain; the room is past it, where P2 R and Q R come
    // nearest the chain: (13 - 3t, 10t) off it at t = 39/109, less the reading's margin of two
    // snaps.
    let nearest = (169.0_f64 - (39.0 * 39.0 / 109.0)).sqrt();
    assert!((reading.room - nearest).abs() <= 1e-8, "{said}");
}

/// The flank bent the other way, its bend P' = (9.5, 8, 10) a reflex corner of the rib's base: the
/// seam P' R between the two flank facets is concave, so it still bounds, at P', 8 from the chain.
#[test]
fn a_concave_seam_still_bounds() {
    let bent = Vector3::new(9.5, 8.0, 10.0);
    let reading = read_on(&positions_with(bent), &indices());
    let said = format!("room {:?}, set by {:?}", reading.room, reading.room_set_by);
    assert!(reading.room_set_by.is_a_growing_crease(), "{said}");
    let at = reading.room_set_by.crease_at.expect("CreaseAt");
    assert!(distance_to_segment(at, bent, R) < 1e-9, "{said}");
    assert!((reading.room - 8.0).abs() <= 1e-8, "{said}");
}

/// A 12-gon boss of radius 4 and height 5 on the box's top, a vertex pointing at the chain: each
/// facet turns exactly 30 degrees off the last, which `SEPARATE_END_FACE_DEGREES` reads as one
/// wall, so no seam bounds and the room is where the boss's top rim comes nearest the chain, at
/// (10, 6, 15): sqrt(6^2 + 5^2) less the reading's margin. Read as corners, the seams would set it
/// at the boss's foot, 6.
#[test]
fn a_twelve_gon_walls_seams_carry_on() {
    let (positions, indices) = boss_on_a_box(12);
    let reading = read_on(&positions, &indices);
    let said = format!("room {:?}, set by {:?}", reading.room, reading.room_set_by);
    assert!(reading.room_set_by.is_a_growing_crease(), "{said}");
    let at = reading.room_set_by.crease_at.expect("CreaseAt");
    assert!((at.z - 15.0).abs() < 1e-9, "{said}");
    assert!((reading.room - 61.0_f64.sqrt()).abs() <= 1e-6, "{said}");
}

/// The box of [`positions`] with an `n`-gon boss of radius 4 and height 5 standing on its top
/// about (10, 10), its first vertex at (10, 6) pointing at the chain. The top is the ring between
/// the box's top corners and the boss's base, each boss edge fanned to the top corner nearest it.
fn boss_on_a_box(n: usize) -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    let mut positions: Vec<Vector3> = positions().into_iter().take(8).collect();
    let base = positions.len() as i32;
    let angle = |i: usize| (-90.0 + 360.0 * i as f64 / n as f64).to_radians();
    for i in 0..n {
        positions.push(Vector3::new(
            10.0 + 4.0 * angle(i).cos(),
            10.0 + 4.0 * angle(i).sin(),
            10.0,
        ));
    }
    for i in 0..n {
        positions.push(Vector3::new(
            10.0 + 4.0 * angle(i).cos(),
            10.0 + 4.0 * angle(i).sin(),
            15.0,
        ));
    }
    let b = |i: usize| base + (i % n) as i32;
    let t = |i: usize| base + (n + i % n) as i32;
    let mut indices: Vec<(i32, i32, i32)> = indices().into_iter().take(10).collect();
    // The top corner each boss edge fans to, by the quarter its middle stands in about the boss:
    // (20, 20) for 0 to 90 degrees, (0, 20), (0, 0), then (20, 0) for 270 to 360.
    let corner_at = |degrees: f64| {
        let d = degrees.rem_euclid(360.0);
        if d < 90.0 {
            6
        } else if d < 180.0 {
            7
        } else if d < 270.0 {
            4
        } else {
            5
        }
    };
    let owner = |i: usize| corner_at(-90.0 + 360.0 * (i as f64 + 0.5) / n as f64);
    for i in 0..n {
        indices.push((b(i + 1), b(i), owner(i)));
        if owner(i) != owner(i + 1) {
            indices.push((owner(i), owner(i + 1), b(i + 1)));
        }
        indices.push((b(i), b(i + 1), t(i + 1)));
        indices.push((b(i), t(i + 1), t(i)));
    }
    for i in 1..n - 1 {
        indices.push((t(0), t(i), t(i + 1)));
    }
    (positions, indices)
}
