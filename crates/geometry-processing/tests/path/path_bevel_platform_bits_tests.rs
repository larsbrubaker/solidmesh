//! A path bevel's output pinned to the last bit. The 2D bevel and the topology readings work out
//! their angles and arc points with the pure-Rust `libm` crate rather than std's `sin`, `cos`,
//! `tan`, `acos` and `atan2`, which call each platform's own maths library and differ in the last
//! bits between Windows, Linux, macOS and the browser. With `libm` the same design gives the same
//! curves everywhere, so a value recorded on one platform holds on every other, and these pins
//! fail on any platform where that stops being true (CI runs them on all of them).

use solidmesh::path::path_bevel::bevel;
use solidmesh::path::{Contour, PathBevelSizeMode};
use solidmesh::topology::FeatureEdge;
use solidmesh_math::Vector2;

use super::flattened_curve_path::FlattenedCurvePath;

/// FNV-1a over the bits of every coordinate, contour by contour: any change in any last bit
/// moves it.
fn digest(contours: &[Contour]) -> u64 {
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    for contour in contours {
        for point in &contour.points {
            for value in [point.x, point.y] {
                for byte in value.to_bits().to_le_bytes() {
                    hash ^= u64::from(byte);
                    hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
                }
            }
        }
    }
    hash
}

/// Every point, shortest round trip, so a moved pin shows which coordinate moved.
fn written(contours: &[Contour]) -> String {
    contours
        .iter()
        .flat_map(|contour| contour.points.iter())
        .map(|point| format!("{:?} {:?}", point.x, point.y))
        .collect::<Vec<_>>()
        .join(" ")
}

/// A closed outline with straight sides and one side a cubic: corners between lines, corners
/// where a line meets the curve, and the curve's own chords for the fillet centres to ride.
fn curved_outline() -> Contour {
    let mut path = FlattenedCurvePath::starting_at(0.0, 0.0);
    path.line_to(20.0, 0.0);
    path.curve4_vector(
        Vector2::new(28.0, 6.0),
        Vector2::new(14.0, 14.0),
        Vector2::new(20.0, 20.0),
    );
    path.line_to(0.0, 20.0);
    Contour {
        points: path.contour().points,
        is_closed: true,
    }
}

fn triangle() -> Contour {
    Contour {
        points: vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(10.0, 0.0),
            Vector2::new(3.0, 7.0),
        ],
        is_closed: true,
    }
}

#[test]
fn a_triangle_rounds_to_the_same_points_on_every_platform() {
    let rounded = bevel(&[triangle()], 1.0, 3, 10.0, None, PathBevelSizeMode::Radius);

    assert_eq!(written(&rounded), TRIANGLE_ROUNDED);
}

#[test]
fn a_curved_outline_rounds_to_the_same_bits_on_every_platform() {
    let outline = curved_outline();
    let rounded = bevel(
        std::slice::from_ref(&outline),
        3.0,
        8,
        10.0,
        None,
        PathBevelSizeMode::Radius,
    );
    let chamfered = bevel(
        &[outline],
        2.0,
        5,
        10.0,
        None,
        PathBevelSizeMode::TangentDistance,
    );

    assert_eq!(
        (rounded[0].points.len(), digest(&rounded)),
        CURVED_OUTLINE_ROUNDED
    );
    assert_eq!(
        (chamfered[0].points.len(), digest(&chamfered)),
        CURVED_OUTLINE_SET_BACK
    );
}

#[test]
fn the_default_flat_angle_is_the_same_on_every_platform() {
    assert_eq!(
        FeatureEdge::default_flat_angle_degrees().to_bits(),
        DEFAULT_FLAT_ANGLE_BITS
    );
}

/// The triangle rounded with radius 1 and three chords a corner.
const TRIANGLE_ROUNDED: &str = "0.5973939851053575 1.3939192985791673 0.5485413193887677 0.7490409175733768 0.9045582484430493 0.2091273546181336 1.5165390151234153 0.0 7.585786437626904 0.0 8.292893218813452 0.2928932188134521 8.585786437626904 0.9999999999999993 8.292893218813452 1.7071067811865472 4.04442002306101 5.95557997693899 3.4718678097562035 6.239379381151523 2.8443691435452982 6.118534179777383 2.4181682118564045 5.64239249433161";

/// The curved outline rounded with radius 3, eight chords a corner: (points, digest).
const CURVED_OUTLINE_ROUNDED: (usize, u64) = (236, 17019323593562225393);

/// The curved outline set back 2 along each edge, five chords a corner: (points, digest).
const CURVED_OUTLINE_SET_BACK: (usize, u64) = (155, 3537107927878198669);

/// `acos(0.99996)` in degrees, about 0.512.
const DEFAULT_FLAT_ANGLE_BITS: u64 = 4602791145608232345;
