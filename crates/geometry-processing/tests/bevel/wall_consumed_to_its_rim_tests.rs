//! A fillet on the top edge of a wall exactly as tall as the fillet - the ball's tangent line on
//! the wall is the wall's own bottom rim, so the wall is consumed down to that rim. It must build,
//! and just above it must not.

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::open_gap_fill::ends_site;
use geometry_processing::bevel::level_line::{
    band_gap_reading, gap_fill, BandGapRefusal, BandGapResult, BandProfile, GapFillRefusal,
    GapFillResult,
};
use vector_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::{cube_solid, signed_volume, signed_volume_of};
use crate::sweep_mesh_measures::{is_closed_and_consistent, kept, shell_count};

const FOLD_DEGREES: f64 = 45.0;

/// A 20 x 20 x 2 plate, the cube scaled in x and y; its top front edge [7, 6] is the
/// pick.
fn plate() -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    let (cube, indices) = cube_solid(1.0);
    (
        cube.iter()
            .map(|p| Vector3::new(p.x * 10.0, p.y * 10.0, p.z))
            .collect(),
        indices,
    )
}

/// The top front edge at `size`, and its fill where the band built.
fn build(size: f64) -> (BandGapResult, Option<GapFillResult>) {
    let (positions, indices) = plate();
    let gap = band_gap_entry_points::build(
        &positions,
        &indices,
        &[7, 6],
        false,
        Some(feature_edges_over(&positions, &indices, FOLD_DEGREES)),
        size,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
    )
    .expect("the chain builds or refuses by name");
    let fill = if gap.built() {
        Some(
            gap_fill::fill(&gap, BandProfile::Fillet, gap_fill::DEFAULT_FILLET_SEGMENTS)
                .expect("the fill fills or refuses by name"),
        )
    } else {
        None
    };
    (gap, fill)
}

/// A fillet as tall as the wall builds down to the rim.
#[test]
fn a_fillet_as_tall_as_the_wall_builds_down_to_the_rim() {
    let (positions, indices) = plate();
    let before = signed_volume_of(&positions, &indices);
    let (gap, fill) = build(2.0);

    assert_eq!(
        gap.refusal,
        BandGapRefusal::None,
        "the ball's tangent line on the wall is the wall's bottom rim, which the band ends on exactly"
    );
    let fill = fill.expect("the band built");
    assert_eq!(
        fill.refusal,
        GapFillRefusal::None,
        "the rim is the wall's run; site: {:?}",
        ends_site()
    );

    let mesh = fill.mesh.as_ref().expect("a filled mesh").borrow();
    let kept = kept(&mesh);
    assert!(is_closed_and_consistent(&mesh.vertices, &kept));
    assert_eq!(shell_count(mesh.vertices.len() as i32, &kept), 1);

    // Nothing of the wall (y = 10) stands above its bottom rim (z = -1).
    let wall_left = kept.iter().any(|t| {
        let corners = [t.0, t.1, t.2];
        corners
            .iter()
            .all(|&v| (mesh.position(v).y - 10.0).abs() < 1e-9)
            && corners.iter().any(|&v| mesh.position(v).z > -1.0 + 1e-9)
    });
    assert!(!wall_left);

    // The profile the fill lays is four facets through `gap_fill::arc_point`'s stations, and the
    // fillet runs straight between two flat end faces, so the solid taken is exactly that polygon
    // along 20 mm. Read in the (y, z) section.
    const FACETS: i32 = 4;
    let corner = Vector3::new(0.0, 10.0, 1.0);
    let on_top = Vector3::new(0.0, 8.0, 1.0);
    let on_rim = Vector3::new(0.0, 10.0, -1.0);
    let mut section = vec![corner];
    section.extend(
        (0..=FACETS).map(|k| gap_fill::arc_point(on_top, corner, on_rim, k as f64 / FACETS as f64)),
    );
    let mut area = 0.0;
    for i in 0..section.len() {
        let p = section[i];
        let q = section[(i + 1) % section.len()];
        area += (p.y * q.z) - (q.y * p.z);
    }

    let r = 2.0;
    let removed = before - signed_volume(&mesh);
    assert!(removed > 20.0 * ((r * r) - (std::f64::consts::PI * r * r / 4.0)));
    assert!((removed - (20.0 * f64::abs(area) / 2.0)).abs() <= 1e-9);
}

/// Under the wall builds and above is refused.
#[test]
fn under_the_wall_builds_and_above_is_refused() {
    let (under, under_fill) = build(1.5);
    assert_eq!(under.refusal, BandGapRefusal::None);
    assert_eq!(
        under_fill.expect("the band built").refusal,
        GapFillRefusal::None
    );

    let (above, _) = build(2.2);
    assert_eq!(above.refusal, BandGapRefusal::Size);
}

/// Every size between the room and the rim builds.
#[test]
fn every_size_between_the_room_and_the_rim_builds() {
    let margin = band_gap_reading::room_margin(1.0, 1e-9);
    for fraction in [0.25, 0.5, 0.99] {
        let (gap, fill) = build(2.0 - margin + (margin * fraction));
        assert_eq!(
            gap.refusal,
            BandGapRefusal::None,
            "{fraction} of the margin above the room"
        );
        let fill = fill.expect("the band built");
        assert_eq!(
            fill.refusal,
            GapFillRefusal::None,
            "{fraction} of the margin above the room"
        );
        let mesh = fill.mesh.as_ref().expect("a filled mesh").borrow();
        let kept = kept(&mesh);
        assert!(is_closed_and_consistent(&mesh.vertices, &kept));
    }
}

/// A closed rim at the wall's height reads the same fresh and handed its room.
#[test]
fn a_closed_rim_at_the_walls_height_is_the_same_fresh_and_handed_its_room() {
    let (positions, indices) = plate();
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    let build_rim = |known_room: Option<f64>| {
        band_gap_entry_points::build(
            &positions,
            &indices,
            &[4, 5, 6, 7],
            true,
            Some(feature.clone()),
            2.0,
            BandProfile::Fillet,
            &BandGapBuildOptions {
                known_room,
                ..BandGapBuildOptions::default()
            },
        )
        .expect("the rim builds or refuses by name")
    };
    let fresh = build_rim(None);
    let handed = build_rim(Some(fresh.room));

    assert_eq!(fresh.refusal, BandGapRefusal::Size);
    assert_eq!(handed.refusal, fresh.refusal);
    assert_eq!(handed.room, fresh.room);
}
