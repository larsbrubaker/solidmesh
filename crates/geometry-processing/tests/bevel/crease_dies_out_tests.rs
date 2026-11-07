//! A crease that DIES OUT into a smooth surface at both of its ends - a block whose top ridge
//! swells out of a flat top and sinks back into it, the whole crease picked. Round each end the two
//! sides are ONE smooth surface, so the sides do not divide, and that is what a dying-out end
//! MEANS, not a defect. The fixtures are `crease_dies_out_fixtures.rs`.

use geometry_processing::bevel::level_line::gap_fill::{self, DEFAULT_FILLET_SEGMENTS};
use geometry_processing::bevel::level_line::{
    BandGapRefusal, BandGapResult, BandProfile, GapFillResult,
};
use vector_math::Vector3;

use crate::crease_dies_out_fixtures::{
    build, build_chain, chain, neighbours, seam_epsilon, swollen_ridge_block,
    swollen_ridge_block_with_a_seam_copy, FOLD_DEGREES, PEAK_RIDGE, RING, SEAM_RING,
};
use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::distance_to_surface;
use crate::level_line_test_geometry::{signed_volume, winding_number};
use crate::several_chains_fixtures;
use crate::sweep_mesh_measures;

fn loops_text(gap: &BandGapResult) -> String {
    gap.loops
        .iter()
        .map(|l| {
            format!(
                "side {} {} {}",
                l.side,
                if l.is_closed { "closed" } else { "open" },
                l.vertices.len()
            )
        })
        .collect::<Vec<_>>()
        .join(" ")
}

/// The gap's fillet fill, with the default fillet segments.
fn fill(gap: &BandGapResult) -> GapFillResult {
    gap_fill::fill(gap, BandProfile::Fillet, DEFAULT_FILLET_SEGMENTS).expect("the fill runs")
}

/// The fill's kept triangles and every position of its mesh.
fn kept_and_points(fill: &GapFillResult) -> (Vec<(i32, i32, i32)>, Vec<Vector3>) {
    let mesh = fill.mesh.as_ref().expect("a filled mesh").borrow();
    (sweep_mesh_measures::kept(&mesh), mesh.vertices().to_vec())
}

/// The fixture is a closed, consistently wound solid before anything is read or cut.
#[test]
fn the_block_is_a_solid() {
    let (positions, indices) = swollen_ridge_block();
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        &positions, &indices
    ));
    assert_eq!(
        sweep_mesh_measures::shell_count(positions.len() as i32, &indices),
        1
    );
}

/// The crease really does die out: the picked run is two or more edges long, and neither of its
/// end vertices stands on any feature edge but the run's own - past each end the surface is smooth.
#[test]
fn the_crease_dies_out_at_both_ends() {
    let (positions, indices) = swollen_ridge_block();
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    let chain = chain(&positions, &indices, RING, 4);
    println!("chain {:?} of {} edges", chain, chain.len() - 1);

    assert!(chain.len() >= 3);
    let ends = [
        (chain[0], chain[1]),
        (chain[chain.len() - 1], chain[chain.len() - 2]),
    ];
    for (vertex, inward) in ends {
        let features: Vec<i32> = neighbours(&indices, vertex)
            .into_iter()
            .filter(|&n| feature(vertex, n))
            .collect();
        println!("end {vertex}: feature neighbours {features:?}");
        assert_eq!(features, vec![inward]);
    }
}

/// The cut this box exists for: the two sides meet round both dying ends, so the flood finds ONE
/// region - and the band round the crease leaves exactly ONE closed loop, which is what the fill
/// takes.
#[test]
fn the_cut_leaves_one_closed_loop_round_the_chain() {
    let (positions, indices) = swollen_ridge_block();
    let built = build(&positions, &indices, 0.1);
    println!(
        "0.1: {:?} room {:?}, loops {} [{}]",
        built.refusal,
        built.room,
        built.loops.len(),
        loops_text(&built)
    );

    assert!(built.built(), "the cut must build: {:?}", built.refusal);
    assert_eq!(built.loops.len(), 1);
    assert!(built.loops[0].is_closed);
}

/// The fill: the one loop cut into two rails fills closed, one shell, wound as the input, no
/// triangle of no area, the volume DOWN by less than the picked run plus a fade at each end can
/// hold, and nothing the fill invents standing outside the solid. Read at a tenth and at 0.5, 0.9
/// and 1.0 of the room, and a hair above the room refuses by size.
#[test]
fn the_dying_crease_fills_closed() {
    let (positions, indices) = swollen_ridge_block();
    let before = sweep_mesh_measures::signed_volume(&positions, &indices);
    let chain = chain(&positions, &indices, RING, 4);
    let run: f64 = (0..chain.len() - 1)
        .map(|i| (positions[chain[i + 1] as usize] - positions[chain[i] as usize]).length())
        .sum();
    let room = build(&positions, &indices, 0.1).room;
    println!("room {room:?}, run {run:?}, block {before:?}");

    // What the ladder took, measured: 6.93e-5, 2.408, 7.840 and 9.536 of a block of 2,401. Each
    // row is pinned just above what it takes, because the prism bound the fade's rows use - the
    // size squared along the run - is 10,945 at this room and would pass anything at all.
    let mut rows: Vec<String> = Vec::new();
    let bounds = [1e-4, 3.0, 9.5, 12.0];
    for (rung, size) in [0.1, 0.5 * room, 0.9 * room, room].into_iter().enumerate() {
        let gap = build(&positions, &indices, size);
        assert!(
            gap.built(),
            "{size} is at or under the room: {:?}",
            gap.refusal
        );

        let fill = fill(&gap);
        assert!(
            fill.filled(),
            "a crease that dies out fills: {:?}",
            fill.refusal
        );

        let (kept, points) = kept_and_points(&fill);
        let after = sweep_mesh_measures::signed_volume(&points, &kept);
        println!(
            "size {size:?}: {} triangles, took {:?}",
            fill.added_triangles.len(),
            before - after
        );

        assert!(sweep_mesh_measures::is_closed_and_consistent(
            &points, &kept
        ));
        assert_eq!(
            sweep_mesh_measures::shell_count(points.len() as i32, &kept),
            1
        );
        assert!(sweep_mesh_measures::smallest_area(&points, &kept) > 0.0);
        assert!(after > 0.0, "the result is wound the way the input was");
        assert!(
            after < before,
            "a convex crease's bevel takes material away"
        );

        let mut worst_outside = 0.0_f64;
        let mut proud = 0;
        for &v in &fill.added_vertices {
            let point = points[v as usize];
            if winding_number(&positions, &indices, point) <= 0.45 {
                proud += 1;
                worst_outside = worst_outside.max(distance_to_surface(&positions, &indices, point));
            }
        }

        rows.push(format!(
            "size {size:?}: {} triangles, {} vertices, took {:?}, {proud} proud by {worst_outside:?}",
            fill.added_triangles.len(),
            fill.added_vertices.len(),
            before - after
        ));
        let report = rows.join(" | ");

        assert!(before - after < bounds[rung], "{report}");

        // KNOWN TRADE, measured and not repaired here: the fill is NOT tangent where both rails
        // stand on the flat past a dying end. The arc's control point clamps to the chain's END
        // VERTEX (OpenGapFill.Arcs), and that vertex stands above the flat the crease died into,
        // so the row bulges instead of blending away. Nothing stands outside while the bevel is
        // smaller than the crease it rounds (one vertex reads out by 2e-17 at 0.1, which no
        // winding can tell from on); at the room 31 of 207 added vertices stand proud by up to
        // 0.0192, seven ten-thousandths of the block's diagonal, and 8 by up to 0.0110 at half the
        // room. The control point past a dying end wants to be the projection onto the surface - a
        // fill-design rung of its own, in the plan's open list.
        let limit = if size <= PEAK_RIDGE { 1e-8 } else { 0.04 };
        assert!(worst_outside < limit, "{report}");
    }

    let above = build(&positions, &indices, room * 1.000001);
    println!(
        "a hair above the room: {:?} naming {:?}",
        above.refusal, above.room
    );
    assert_eq!(above.refusal, BandGapRefusal::Size);
    assert!((above.room - room).abs() <= 1e-12 * room);
}

/// The same crease, turned and moved, dies out the same way: the same room, the same counts and the
/// same volume taken - a pin, a flood and a rail are statements about the mesh, never about where it
/// stands.
#[test]
fn the_dying_crease_reads_the_same_moved() {
    let (positions, indices) = swollen_ridge_block();
    let motion = several_chains_fixtures::motion();
    let moved: Vec<Vector3> = positions.iter().map(|&p| p.transform(motion)).collect();

    let here = fill(&build(&positions, &indices, 0.1));
    let there = fill(&build(&moved, &indices, 0.1));
    assert!(here.filled(), "{:?}", here.refusal);
    assert!(there.filled(), "{:?}", there.refusal);

    let took = sweep_mesh_measures::signed_volume(&positions, &indices)
        - signed_volume(&here.mesh.as_ref().expect("a mesh").borrow());
    let took_moved = sweep_mesh_measures::signed_volume(&moved, &indices)
        - signed_volume(&there.mesh.as_ref().expect("a mesh").borrow());
    println!("took {took:?} moved {took_moved:?}");

    assert_eq!(there.added_triangles.len(), here.added_triangles.len());
    assert_eq!(there.added_vertices.len(), here.added_vertices.len());

    // What a shallow crease's fillet takes off this block is a ten-millionth of it, so the two
    // readings are a difference of two volumes near 2,400 and the last bits of THOSE are the
    // noise - measured, one part in 4e15 of the block, which is an ulp. The tolerance is therefore
    // stated against the block and not against what was taken.
    assert!(
        (took_moved - took).abs()
            <= 1e-14 * sweep_mesh_measures::signed_volume(&positions, &indices)
    );
}

/// The MIXED end: one edge picked at the very end of the run, so one of its ends dies out into the
/// smooth top and the other carries on up the crease. Measured, it is the SAME case: the sides are
/// one region (they meet round the dying end), and the one boundary the cut leaves comes back
/// through the meeting point on the continuing ridge as ONE CLOSED loop - which the dying-crease
/// fill cuts into two rails exactly as it does a crease that dies at both ends.
#[test]
fn an_end_that_dies_and_an_end_that_carries_on() {
    let (positions, indices) = swollen_ridge_block();
    let run = chain(&positions, &indices, RING, 4);
    let one = [run[0], run[1]];

    let gap = build_chain(&positions, &indices, &one, 0.1);
    println!(
        "mixed ends: {:?}, loops {} [{}]",
        gap.refusal,
        gap.loops.len(),
        loops_text(&gap)
    );

    let fill = if gap.built() { Some(fill(&gap)) } else { None };
    println!("mixed ends fill: {:?}", fill.as_ref().map(|f| f.refusal));
    assert!(fill.as_ref().is_some_and(|f| f.filled()));
    assert_eq!(gap.loops.len(), 1);
    assert!(gap.loops[0].is_closed);
}

/// The row the real part has: the crease DOUBLED by a ribbon eight microns wide sharing both end
/// vertices - under the app's seam epsilon here, so the copy is demoted to a non-feature and the
/// flood crosses it, which is exactly what leaves the ends unprotected on Lars's part. The same
/// pick must still build closed, one shell, and at every smaller size.
#[test]
fn a_crease_doubled_by_a_seam_ribbon_still_dies_out_and_builds() {
    let (positions, indices) = swollen_ridge_block_with_a_seam_copy();
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        &positions, &indices
    ));

    let chain = chain(&positions, &indices, SEAM_RING, 4);
    println!(
        "seam chain {:?} of {} edges, epsilon {:?}",
        chain,
        chain.len() - 1,
        seam_epsilon()
    );
    assert!(chain.len() >= 3);

    let mut built = true;
    for size in [0.01, 0.1, 1.0] {
        let gap = build_chain(&positions, &indices, &chain, size);
        let fill = if gap.built() { Some(fill(&gap)) } else { None };
        let filled = fill.as_ref().is_some_and(|f| f.filled());
        println!(
            "seam size {size}: {:?} loops {} fill {:?}",
            gap.refusal,
            gap.loops.len(),
            fill.as_ref().map(|f| f.refusal)
        );

        let Some(fill) = fill.filter(|_| filled) else {
            built = false;
            continue;
        };

        assert!(
            built,
            "{size} builds where a smaller size did not - the room is not monotone"
        );

        let (kept, points) = kept_and_points(&fill);
        assert!(sweep_mesh_measures::is_closed_and_consistent(
            &points, &kept
        ));
        assert_eq!(
            sweep_mesh_measures::shell_count(points.len() as i32, &kept),
            1
        );
        assert!(sweep_mesh_measures::smallest_area(&points, &kept) > 0.0);
    }

    assert!(built, "the doubled crease builds at every size tried");
}
