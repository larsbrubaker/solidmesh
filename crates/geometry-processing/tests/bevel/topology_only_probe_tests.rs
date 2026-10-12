//! Ports `Tests/GeometryProcessingTests/LevelLine/TopologyOnlyProbeTests.cs`: the pinch probe's
//! licence - a cut made with the refinement's chord ACCURACY rule switched off leaves each side
//! exactly the loops a full build leaves, checked at five sizes on each of four picks, including
//! sizes either side of the hourglass neck's pinch. C# `[Arguments]` rows are loops over the
//! same-named test; its `[NotInParallel]` guards no shared state here (nextest gives every test a
//! process of its own).

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::seam_edges::FeatureEdgeTest;
use geometry_processing::bevel::level_line::{BandGapRefusal, BandGapResult, BandProfile};
use vector_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::hourglass_solid;
use crate::level_line_test_geometry::cube_solid;
use crate::tessellated_cylinder::tessellated_cylinder;
use crate::usable_room_tests::strip_solid;

const FOLD_DEGREES: f64 = 45.0;

/// C# `ATopologyOnlyCutLeavesTheLoopsAFullBuildLeaves`.
#[test]
fn a_topology_only_cut_leaves_the_loops_a_full_build_leaves() {
    for pick in [
        "hourglass-face",
        "cube-top-rim",
        "strip-face",
        "cylinder-rim",
    ] {
        let (positions, indices, chain, profile, sizes) = pick_named(pick);
        let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);

        for size in sizes {
            let full = cut(&positions, &indices, &chain, &feature, size, profile, false);
            let cheap = cut(&positions, &indices, &chain, &feature, size, profile, true);
            println!(
                "{pick} at {size}: full {}, topology-only {}",
                describe(&full),
                describe(&cheap)
            );

            assert_eq!(describe(&cheap), describe(&full));
        }
    }
}

/// C# `Cut`: one cut at one size, with the room handed in as infinite so no size is refused for
/// WIDTH.
fn cut(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    feature: &FeatureEdgeTest,
    size: f64,
    profile: BandProfile,
    topology_only: bool,
) -> BandGapResult {
    band_gap_entry_points::build_one_unvalidated(
        positions,
        indices,
        chain,
        true,
        Some(feature.clone()),
        size,
        profile,
        &BandGapBuildOptions {
            chord_tolerance: 0.0,
            snap: 1e-9,
            known_room: Some(f64::INFINITY),
            ..BandGapBuildOptions::default()
        },
        topology_only,
    )
    .expect("the cut builds or refuses by name")
}

/// C# `Describe`: the refusal, and per side how many loops it left and whether they closed.
fn describe(cut: &BandGapResult) -> String {
    // A pinch refusal carries the loops that were read to refuse; only a refusal that never got as
    // far as cutting is described by its name.
    if cut.refusal != BandGapRefusal::None && cut.removed_triangles.is_empty() {
        return format!("{:?}", cut.refusal);
    }

    [1, 2]
        .iter()
        .map(|&side| {
            let mine: Vec<_> = cut.loops.iter().filter(|l| l.side == side).collect();
            format!(
                "side {side}: {} loops, {} closed",
                mine.len(),
                mine.iter().filter(|l| l.is_closed).count()
            )
        })
        .collect::<Vec<_>>()
        .join("; ")
}

type Pick = (
    Vec<Vector3>,
    Vec<(i32, i32, i32)>,
    Vec<i32>,
    BandProfile,
    Vec<f64>,
);

/// C# `PickNamed`.
fn pick_named(name: &str) -> Pick {
    match name {
        "hourglass-face" => {
            // The neck is 2 wide, so the two sides of the band meet in it somewhere just under half
            // of that. The sizes step across that flip.
            let (positions, indices, rim) = hourglass_solid(10.0, 2.0, 4.0, 4.0);
            (
                positions,
                indices,
                rim,
                BandProfile::Chamfer,
                vec![0.2, 0.49, 0.497, 0.51, 0.8],
            )
        }
        "cube-top-rim" => {
            let (positions, indices) = cube_solid(1.0);
            (
                positions,
                indices,
                vec![4, 5, 6, 7],
                BandProfile::Chamfer,
                vec![0.05, 0.2, 0.5, 0.7, 0.95],
            )
        }
        "strip-face" => {
            // A 4 by 20 strip: the top face's own side runs out of room at half the width.
            let (positions, indices) = strip_solid();
            (
                positions,
                indices,
                vec![4, 5, 6, 7],
                BandProfile::Chamfer,
                vec![0.3, 1.0, 1.9, 2.1, 3.0],
            )
        }
        _ => {
            let (positions, indices, rim) = tessellated_cylinder(64, 6, 10.0, 20.0);
            (
                positions,
                indices,
                rim,
                BandProfile::Fillet,
                vec![0.5, 2.0, 5.0, 9.9, 10.5],
            )
        }
    }
}
