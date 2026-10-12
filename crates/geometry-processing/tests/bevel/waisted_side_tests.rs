//! Ports `Tests/GeometryProcessingTests/LevelLine/WaistedSideTests.cs`: an open member whose side
//! face is WAISTED - a slab 20 x 10 x 5 whose top face a V-notch from the back narrows to 2 deep at
//! x = 10. The front top edge's band at 3 would cut the top face's front in two - a neck, not a
//! crossing feature - so it must refuse by size, never build as two pieces. C# `[Arguments]` rows
//! are loops over the same-named test.

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::{BandGapRefusal, BandGapResult, BandProfile};
use vector_math::{Vector2, Vector3};

use crate::level_line_feature_edges::feature_edges_over;

const HEIGHT: f64 = 5.0;

/// C# `Plan`: top-face corners - the front edge is top 7 - 8, the right top edge 8 - 9, the right
/// vertical edge 1 - 8.
fn plan() -> [Vector2; 7] {
    [
        Vector2::new(0.0, 0.0),
        Vector2::new(20.0, 0.0),
        Vector2::new(20.0, 10.0),
        Vector2::new(12.0, 10.0),
        Vector2::new(10.0, 2.0),
        Vector2::new(8.0, 10.0),
        Vector2::new(0.0, 10.0),
    ]
}

/// C# `TheWaistRefusesBySize`.
#[test]
fn the_waist_refuses_by_size() {
    for one_member in [false, true] {
        let (positions, indices) = slab();

        // The corner pick: the front top edge with the right TOP edge (a cluster of two on the top
        // face), or with the right VERTICAL edge (the front edge alone on the top face).
        let chains = vec![if one_member {
            vec![1, 8, 7]
        } else {
            vec![7, 8, 9]
        }];

        let build = |size: f64| -> BandGapResult {
            band_gap_entry_points::build_pick(
                &positions,
                &indices,
                &chains,
                &[false],
                Some(feature_edges_over(&positions, &indices, 30.0)),
                size,
                BandProfile::Fillet,
                &BandGapBuildOptions::default(),
            )
            .expect("the pick builds or refuses by name")
        };

        let under = build(1.5);
        assert_eq!(
            under.refusal,
            BandGapRefusal::None,
            "under the waist: {:?} tri {} edge {:?} room {:?} set by {:?}",
            under.refusal,
            under.offending_triangle,
            under.offending_edge,
            under.room,
            under.room_set_by
        );

        let over = build(3.0);
        assert_eq!(
            over.refusal,
            BandGapRefusal::Size,
            "room {:?} set by {:?}",
            over.room,
            over.room_set_by
        );
        assert!(over.room <= 2.0 + 1e-6);
    }
}

/// C# `Slab`: bottom vertices 0..6, top 7..13, every triangle wound outward.
fn slab() -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    let plan = plan();
    let positions: Vec<Vector3> = plan
        .iter()
        .map(|p| Vector3::new(p.x, p.y, 0.0))
        .chain(plan.iter().map(|p| Vector3::new(p.x, p.y, HEIGHT)))
        .collect();
    let top = [(0, 1, 4), (1, 2, 3), (1, 3, 4), (0, 4, 5), (0, 5, 6)];
    let mut indices: Vec<(i32, i32, i32)> = Vec::new();
    indices.extend(top.iter().map(|t| (t.0 + 7, t.1 + 7, t.2 + 7)));
    indices.extend(top.iter().map(|t| (t.0, t.2, t.1)));
    let n = plan.len() as i32;
    for i in 0..n {
        let j = (i + 1) % n;
        indices.push((i, j, j + n));
        indices.push((i, j + n, i + n));
    }

    (positions, indices)
}
