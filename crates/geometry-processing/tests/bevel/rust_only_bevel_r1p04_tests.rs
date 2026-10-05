//! Rust-only tests for the bevel-core package R1.P04's methods that no ported C# test class covers
//! yet: the strip's rail readings (`SurfaceStrip`), the strip station (`SurfaceStripStation`) and
//! the chamfer mitre's rail-end pairing (`SurfaceCornerMitre.RailEndsPairUp`). Each pins the C#
//! rule on a hand-built case.

use geometry_processing::bevel::surface::{
    surface_corner_mitre, SurfaceStrip, SurfaceStripStation,
};
use vector_math::Vector3;

/// A strip's rails run along its edge from their own feet; a point's parameter is read from the
/// strip's origin; a plane the rails run along is no crossing.
#[test]
fn rust_only_a_strip_reads_its_rails() {
    let strip = SurfaceStrip {
        origin: Vector3::new(0.0, 0.0, 0.0),
        along: Vector3::new(1.0, 0.0, 0.0),
        rail_points: vec![Vector3::new(0.0, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0)],
        rail_step_degrees: 0.0,
        outward_reference: Vector3::new(0.0, 1.0, 1.0).get_normal(),
    };

    assert_eq!(strip.rail_count(), 2);
    assert_eq!(strip.along(), Vector3::new(1.0, 0.0, 0.0));
    assert_eq!(strip.point_on(1, 3.0), Vector3::new(3.0, 0.0, 1.0));
    assert_eq!(strip.param_of(Vector3::new(4.5, 7.0, -2.0)), 4.5);
    assert_eq!(
        strip.try_cross(0, Vector3::new(2.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)),
        Some(2.0)
    );
    assert_eq!(
        strip.try_cross(0, Vector3::new(2.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)),
        None
    );
}

/// A clean station has an empty between for each pair of rails, and none for a lone rail.
#[test]
fn rust_only_a_clean_station_has_nothing_between() {
    let station = SurfaceStripStation::clean(vec![1, 2, 3], vec![0.0, 0.5, 1.0]);
    assert_eq!(station.between, vec![Vec::<i32>::new(); 2]);
    assert!(SurfaceStripStation::clean(vec![7], vec![0.0])
        .between
        .is_empty());
}

/// Three strips' ends chain into one rim only when every end is shared exactly twice and no strip
/// is pinched shut.
#[test]
fn rust_only_rail_ends_pair_up_two_at_a_time() {
    assert!(surface_corner_mitre::rail_ends_pair_up(&[1, 2, 2, 3, 3, 1]));
    assert!(!surface_corner_mitre::rail_ends_pair_up(&[
        1, 1, 2, 3, 3, 2
    ]));
    assert!(!surface_corner_mitre::rail_ends_pair_up(&[
        1, 2, 2, 3, 3, 4
    ]));
}
