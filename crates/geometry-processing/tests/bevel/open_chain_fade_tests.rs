//! Port of `LevelLine/OpenChainFadeTests.cs`: Stage 2's first rung - a bevel on PART of a crease
//! that carries on past both ends FADES OUT. The pick is a run of top-rim facet edges of a faceted
//! cylinder; the crease continues either side of it, so the two sides' level lines curl round each
//! end vertex and meet at one point on the continuing ridge, about a setback past the end. The two
//! open runs therefore chain into ONE cycle, and the fill zips them in open mode. C#
//! `[MethodDataSource]`/`[Arguments]` rows are loops over the same-named test.

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::gap_fill::DEFAULT_FILLET_SEGMENTS;
use geometry_processing::bevel::level_line::{
    gap_fill, BandGapRefusal, BandGapResult, BandProfile, GapFillRefusal, GapLoop,
};
use vector_math::{Matrix4X4, Vector3};

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::{distance_to_surface, signed_volume, winding_number};
use crate::open_chain_fixtures;
use crate::several_chains_fixtures;
use crate::sweep_mesh_measures;

/// C# `Ladder`: a size that builds means every smaller size builds - under a rim facet's 1.5643
/// length and well over it, 2 being the size in Lars's owner file, where the fade reaches past the
/// neighbouring rim vertex.
fn ladder() -> Vec<(i32, BandProfile, f64)> {
    let mut rows = Vec::new();
    for edges in [1, 3] {
        for profile in [BandProfile::Chamfer, BandProfile::Fillet] {
            for size in [0.1, 0.25, 0.5, 1.0, 2.0, 3.0] {
                rows.push((edges, profile, size));
            }
        }
    }
    rows
}

fn cut(edges: i32, profile: BandProfile, size: f64, by: Option<Matrix4X4>) -> BandGapResult {
    let (positions, indices) = match by {
        None => open_chain_fixtures::cylinder(),
        Some(by) => open_chain_fixtures::moved(by),
    };

    band_gap_entry_points::build(
        &positions,
        &indices,
        &open_chain_fixtures::top_rim_run(edges),
        false,
        Some(feature_edges_over(
            &positions,
            &indices,
            open_chain_fixtures::FOLD_DEGREES,
        )),
        size,
        profile,
        &BandGapBuildOptions::default(),
    )
    .expect("a valid pick")
}

fn loop_of(gap: &BandGapResult, side: i32) -> GapLoop {
    let on: Vec<&GapLoop> = gap.loops.iter().filter(|l| l.side == side).collect();
    assert_eq!(on.len(), 1, "one loop on side {side}");
    on[0].clone()
}

/// The cut's own claim, asserted before any fill: the band builds, it leaves one open run per
/// side, and the two runs share their END VERTICES - the same mesh vertices, not two readings of
/// one point, which is what lets the fill zip them as one cycle.
#[test]
fn the_two_open_runs_meet_at_the_same_mesh_vertices() {
    for (edges, profile, size) in ladder() {
        let gap = cut(edges, profile, size, None);

        assert!(
            gap.built(),
            "a partial rim pick at {size} must cut: {:?}",
            gap.refusal
        );
        assert_eq!(gap.loops.len(), 2);
        assert!(gap.loops.iter().all(|l| !l.is_closed));

        let left = loop_of(&gap, 1);
        let right = loop_of(&gap, 2);

        let ends = [
            (left.vertices[0], right.vertices[right.vertices.len() - 1]),
            (left.vertices[left.vertices.len() - 1], right.vertices[0]),
        ];

        for (a, b) in ends {
            assert_eq!(
                a, b,
                "{edges} {profile:?} {size}: the two rails meet at ONE vertex on the continuing ridge, so the runs are one cycle"
            );
        }
    }
}

/// The fade itself: the gap fills closed, one shell, wound as the input, no triangle of no area,
/// the volume down by less than the picked run plus its two fades can hold, and nothing the fill
/// invents standing outside the solid.
#[test]
fn a_partial_rim_pick_fades_out_and_fills_closed() {
    for (edges, profile, size) in ladder() {
        let (positions, indices) = open_chain_fixtures::cylinder();
        let before = sweep_mesh_measures::signed_volume(&positions, &indices);

        let gap = cut(edges, profile, size, None);
        assert!(gap.built(), "the cut must build: {:?}", gap.refusal);

        let fill = gap_fill::fill(&gap, profile, DEFAULT_FILLET_SEGMENTS).expect("the fill runs");
        assert!(
            fill.filled(),
            "{edges} {profile:?} {size}: an open chain on a carrying crease fades: {:?}",
            fill.refusal
        );

        let mesh = fill.mesh.as_ref().expect("a filled mesh").borrow();
        let kept = sweep_mesh_measures::kept(&mesh);
        let points: Vec<Vector3> = (0..mesh.vertex_count()).map(|v| mesh.position(v)).collect();

        assert!(sweep_mesh_measures::is_closed_and_consistent(
            &points, &kept
        ));
        assert_eq!(
            sweep_mesh_measures::shell_count(points.len() as i32, &kept),
            1
        );
        assert!(sweep_mesh_measures::smallest_area(&points, &kept) > 0.0);

        let after = sweep_mesh_measures::signed_volume(&points, &kept);
        assert!(after > 0.0, "the result is wound the way the input was");
        assert!(after < before, "a convex rim's bevel takes material away");

        // The band cannot reach further than the picked run plus one fade at each end, and what it
        // takes off there is inside a square of the size: a plain bound stated from the size and
        // the run.
        let run = edges as f64 * open_chain_fixtures::facet_length();
        assert!(before - after < size * size * (run + 2.0 * size));

        // The shape claim: nothing the fade invents stands outside the solid that went in. It is
        // read against the INPUT, and where the winding says "out" the point has to be ON the
        // surface: the fade's last rows collapse onto the continuing ridge.
        for &v in &fill.added_vertices {
            let point = mesh.position(v);
            if winding_number(&positions, &indices, point) <= 0.45 {
                assert!(
                    distance_to_surface(&positions, &indices, point)
                        < 1e-9 * open_chain_fixtures::RADIUS,
                    "a point the winding reads as outside may only be ON the surface"
                );
            }
        }
    }
}

/// A rail whose correspondence CROSSES is refused by name, not crushed monotone. Open mode
/// flattens a backward reading onto its envelope exactly as the closed fill does, and under the
/// same bound (three setbacks): a swing deeper than that is a rail that really does turn back on
/// itself, and flattening it would lay a self-intersecting strip which still closes by
/// construction.
#[test]
fn a_rail_that_turns_back_too_far_is_refused_by_name() {
    for (swing, fills) in [(0.2, true), (3.0, false)] {
        let gap = cut(3, BandProfile::Fillet, 0.5, None);
        let left = loop_of(&gap, 1);
        let mut moved = left.parameters.clone();

        // The rail's LAST interior reading is dragged back behind the one before it: by 0.2, which
        // is within what a concave turn explains at this setback (3 x 0.5), and by 3.0, which is
        // several times deeper. It is floored just INSIDE the chain, because a reading at an end is
        // a clamped one and the fill reads those from the angle about the end vertex instead.
        let length = gap.chain().expect("the chain").length;
        let interior = (0..moved.len())
            .filter(|&i| moved[i] > 1e-6 && moved[i] < length - 1e-6)
            .max()
            .expect("an interior reading");
        moved[interior] = f64::max(1e-3, moved[interior - 1] - swing);

        let swung = BandGapResult {
            refusal: BandGapRefusal::None,
            mesh: gap.mesh.clone(),
            chains: gap.chains.clone(),
            loops: vec![
                GapLoop::new(
                    left.side,
                    false,
                    left.vertices.clone(),
                    moved,
                    0,
                    false,
                    None,
                ),
                loop_of(&gap, 2),
            ],
            ..BandGapResult::default()
        };

        let fill = gap_fill::fill(&swung, BandProfile::Fillet, DEFAULT_FILLET_SEGMENTS)
            .expect("the fill runs");

        assert_eq!(
            fill.filled(),
            fills,
            "the swing of {swing} was read as {:?}",
            fill.refusal
        );
        if !fills {
            assert_eq!(fill.refusal, GapFillRefusal::RailIsNotMonotone);
            assert_eq!(fill.offending_vertex, left.vertices[interior]);
        }
    }
}

/// Ends that COINCIDE are not enough: the fade is only right where the two runs meet ON the crease
/// that carries on, and the routing asks the SIDE TAGGING for that rather than trusting the
/// coincidence. Here both runs are made to end at a vertex in the middle of the cap - one surface,
/// no ridge between the two sides - and the fill refuses by name instead of fading into nothing.
#[test]
fn ends_that_meet_off_the_ridge_are_refused_by_name() {
    let gap = cut(3, BandProfile::Fillet, 0.5, None);
    let left = loop_of(&gap, 1);
    let right = loop_of(&gap, 2);

    // The cap's own centre vertex: it stands on cap triangles only, so no edge through it has one
    // side's surface on one face and the other's on the other.
    let centre = open_chain_fixtures::SIDES * 2 + 1;
    let mut left_ends = left.vertices.clone();
    let mut right_ends = right.vertices.clone();
    left_ends[0] = centre;
    let last = right_ends.len() - 1;
    right_ends[last] = centre;

    let off_ridge = BandGapResult {
        refusal: BandGapRefusal::None,
        mesh: gap.mesh.clone(),
        chains: gap.chains.clone(),
        loops: vec![
            GapLoop::new(
                left.side,
                false,
                left_ends,
                left.parameters.clone(),
                0,
                false,
                None,
            ),
            GapLoop::new(
                right.side,
                false,
                right_ends,
                right.parameters.clone(),
                0,
                false,
                None,
            ),
        ],
        ..BandGapResult::default()
    };

    let fill = gap_fill::fill(&off_ridge, BandProfile::Fillet, DEFAULT_FILLET_SEGMENTS)
        .expect("the fill runs");

    assert!(!fill.filled());
    assert_eq!(fill.refusal, GapFillRefusal::OpenChainEnds);
}

/// The fade is rigid: turned and moved, the same pick takes the same volume.
#[test]
fn the_fade_is_unchanged_by_a_rigid_motion() {
    for (edges, profile, size) in ladder() {
        let still = gap_fill::fill(
            &cut(edges, profile, size, None),
            profile,
            DEFAULT_FILLET_SEGMENTS,
        )
        .expect("fills");
        let travelled = gap_fill::fill(
            &cut(
                edges,
                profile,
                size,
                Some(several_chains_fixtures::motion()),
            ),
            profile,
            DEFAULT_FILLET_SEGMENTS,
        )
        .expect("fills");

        assert!(still.filled());
        assert!(travelled.filled());

        let (positions, indices) = open_chain_fixtures::cylinder();
        let before = sweep_mesh_measures::signed_volume(&positions, &indices);
        let took = before - signed_volume(&still.mesh.as_ref().expect("a mesh").borrow());
        let took_moved = before - signed_volume(&travelled.mesh.as_ref().expect("a mesh").borrow());

        // On a faceted cylinder the level line crosses the same mesh edges wherever the part
        // stands, so this pick is rigid to the last bits; the worst rung moves by 1.01e-9 of what it
        // takes (k=1, fillet, size 0.1), and every one lays the very same strip.
        assert_eq!(travelled.added_triangles.len(), still.added_triangles.len());
        assert!(
            (took - took_moved).abs() < 1e-8 * took.abs(),
            "{edges} {profile:?} {size}: a rigid motion may not change what the fade takes"
        );
    }
}
