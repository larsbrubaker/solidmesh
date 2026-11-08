//! The fill's own promise, held to on every input this project can build - what comes back is
//! either a CLOSED, consistently wound mesh or a named refusal, never an open solid. The two loops
//! of a gap run opposite ways round the chain, so exactly one rail is reversed; the pair is chosen
//! opposite by construction, and the fill counts its own edges before it returns. Parameter cases
//! run as one loop per test.

use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::seam_edges::FeatureEdgeTest;
use solidmesh::bevel::level_line::{gap_fill, BandGapRefusal, BandProfile, GapFillRefusal};
use solidmesh_math::Vector3;

use crate::hole_fill_corner_tests::{fixture_feature, fixture_profile};
use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_fixture::LevelLineFixture;
use crate::level_line_test_geometry::{cube_solid, cylinder_solid, open_edges};

const FOLD_DEGREES: f64 = 30.0;

/// A cylinder's top rim at a ladder of sizes, ending well
/// past the room.
#[test]
fn cylinder_rim_fills_closed_or_refuses() {
    for sides in [12, 24, 40] {
        let (positions, indices) = cylinder_solid(sides, 5.0, 6.0);
        let rim = top_rim(&positions);

        for size in [0.2, 0.5, 1.0, 2.0, 3.0, 4.0, 4.5, 4.9, 4.95, 4.99, 5.0, 5.2] {
            closed_or_refused(
                &positions,
                &indices,
                &rim,
                size,
                BandProfile::Fillet,
                &format!("{sides} sides at {size}"),
                None,
                4,
            );
        }
    }
}

/// A cube's top rim, the same ladder.
#[test]
fn cube_rim_fills_closed_or_refuses() {
    let (positions, indices) = cube_solid(5.0);
    let rim = top_rim(&positions);

    for size in [0.2, 0.5, 1.0, 2.0, 3.0, 4.0, 4.9, 4.95, 4.99, 5.0, 5.5] {
        closed_or_refused(
            &positions,
            &indices,
            &rim,
            size,
            BandProfile::Fillet,
            &format!("cube at {size}"),
            None,
            4,
        );
        closed_or_refused(
            &positions,
            &indices,
            &rim,
            size,
            BandProfile::Chamfer,
            &format!("cube chamfer at {size}"),
            None,
            4,
        );
    }
}

/// As the size approaches the slant height the level line
/// on the sloping faces collapses onto the APEX.
#[test]
fn pyramid_base_rim_fills_closed_or_refuses() {
    let (positions, indices) = pyramid(5.0, 6.0);
    let rim = [0, 1, 2, 3];

    for size in [0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 7.8, 7.81, 7.9, 8.0] {
        closed_or_refused(
            &positions,
            &indices,
            &rim,
            size,
            BandProfile::Fillet,
            &format!("pyramid at {size}"),
            None,
            4,
        );
        closed_or_refused(
            &positions,
            &indices,
            &rim,
            size,
            BandProfile::Chamfer,
            &format!("pyramid chamfer at {size}"),
            None,
            4,
        );
    }
}

/// A square pyramid, base rim first in index order and wound so the outside is out.
fn pyramid(half: f64, height: f64) -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    let positions = vec![
        Vector3::new(-half, -half, 0.0),
        Vector3::new(half, -half, 0.0),
        Vector3::new(half, half, 0.0),
        Vector3::new(-half, half, 0.0),
        Vector3::new(0.0, 0.0, height),
    ];
    let indices = vec![
        (0, 2, 1),
        (0, 3, 2),
        (0, 1, 4),
        (1, 2, 4),
        (2, 3, 4),
        (3, 0, 4),
    ];
    (positions, indices)
}

/// Every fixture this project carries, at its own size and at a
/// ladder around it.
#[test]
fn fixture_fills_closed_or_refuses() {
    for name in [
        "internal-fillet-face8-r1",
        "internal-fillet-face8-r2",
        "internal-fillet-face9-r1",
        "internal-fillet-face9-r2",
    ] {
        let fixture = Rc::new(LevelLineFixture::load(name));
        for scale in [0.25, 0.5, 1.0, 1.5] {
            closed_or_refused(
                &fixture.positions,
                &fixture.indices,
                &fixture.chains[0],
                fixture.size * scale,
                fixture_profile(&fixture),
                &format!("{name} at x{scale}"),
                Some(fixture_feature(&fixture)),
                if fixture.segments > 0 {
                    fixture.segments
                } else {
                    4
                },
            );
        }
    }
}

/// The one claim - a fill that returns has laid a closed, consistently wound
/// strip.
#[allow(clippy::too_many_arguments)]
fn closed_or_refused(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    size: f64,
    profile: BandProfile,
    what: &str,
    is_feature: Option<FeatureEdgeTest>,
    segments: i32,
) {
    let is_feature =
        is_feature.unwrap_or_else(|| feature_edges_over(positions, indices, FOLD_DEGREES));
    let gap = band_gap_entry_points::build(
        positions,
        indices,
        chain,
        true,
        Some(is_feature),
        size,
        profile,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick builds or refuses by name");
    if gap.refusal != BandGapRefusal::None {
        return;
    }

    let fill = gap_fill::fill(&gap, profile, segments).expect("the fill fills or refuses by name");
    if fill.refusal != GapFillRefusal::None {
        return;
    }

    let open = open_edges(&fill.mesh.as_ref().expect("a filled mesh").borrow());
    assert_eq!(
        open.len(),
        0,
        "{what} filled and must be closed, but {} edges are open",
        open.len()
    );
}

/// The rim of the top face - every vertex at the greatest z, wound round the centre,
/// which is the order the rim is picked in (a chain is a walk, not a set).
fn top_rim(positions: &[Vector3]) -> Vec<i32> {
    let top = positions
        .iter()
        .map(|p| p.z)
        .fold(f64::NEG_INFINITY, f64::max);
    let rim: Vec<i32> = (0..positions.len() as i32)
        .filter(|&i| (positions[i as usize].z - top).abs() < 1e-9)
        .collect();
    let count = rim.len() as f64;
    let centre = Vector3::new(
        rim.iter().map(|&i| positions[i as usize].x).sum::<f64>() / count,
        rim.iter().map(|&i| positions[i as usize].y).sum::<f64>() / count,
        top,
    );

    let angle =
        |i: i32| (positions[i as usize].y - centre.y).atan2(positions[i as usize].x - centre.x);
    let mut ordered = rim;
    // A stable sort, so ties keep their order.
    ordered.sort_by(|&a, &b| angle(a).partial_cmp(&angle(b)).expect("finite angles"));
    ordered
}
