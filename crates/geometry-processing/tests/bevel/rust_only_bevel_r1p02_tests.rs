//! Rust-only tests for the bevel-core package R1.P02's methods that no other test covers on its
//! own: the row zipper (`SurfaceRowZip`), the end cut's run readings (`SurfaceEndCut`), the rolled
//! section's concave cap (`SurfaceEndFollowsTheWalkedChords.TryCapWithTheSection`), the shared side
//! reading and the per-face sewing readings (`SurfaceSplitSewing`), the stretch readings
//! (`SurfaceSplitsOnASegment`), the rim sewing of a section given up
//! (`SurfaceEndGivesUpTheSection.SewTheRim`) and the bite's side readings
//! (`SurfaceBiteSharedSides`, `SurfaceConvexBite.WithTheVerticesStandingOnIt`) and the corner fans
//! (`SurfaceMixedCornerDualWall`, `SurfaceTwoCreaseJunctionWall.TryFillTheNotch`,
//! `SurfaceMixedCornerRunOutStrip.Draw`), the patch refinement (`BevelPatchRefinement`), the corner
//! rows (`SurfaceCornerPatchRows`) and the plan's constructor and accessors (`SurfaceBevelPlan`).
//! Each pins the rule on a hand-built case.

use std::collections::HashMap;

use geometry_processing::bevel::surface::{
    surface_convex_bite, surface_corner_patch_rows, surface_end_cut,
    surface_end_follows_the_walked_chords, surface_end_gives_up_the_section,
    surface_mixed_corner_dual_wall, surface_mixed_corner_run_out_strip, surface_row_zip,
    surface_split_sewing, surface_two_crease_junction_wall, SurfaceBiteSharedSides,
    SurfaceMixedCornerDualWallCrossing, SurfaceSplitsOnASegment,
    SurfaceTwoCreaseJunctionWallCrossing, SurfaceVertexPool,
};
use geometry_processing::bevel::surface::{SurfaceBevelPlan, SurfaceEdgeRoles, SurfacePlanRecipe};
use geometry_processing::bevel::BevelSweptChain;
use geometry_processing::bevel::{bevel_patch_refinement, BevelAdvisoryCode};
use indexmap::{IndexMap, IndexSet};
use vector_math::Vector3;

/// Two rows of equal count alternate, the upper winning each tie, and a row of 2 chords hands over
/// to a row of 3 with every point used.
#[test]
fn rust_only_the_zipper_alternates_and_uses_every_point() {
    let mut band = Vec::new();
    surface_row_zip::sew(&mut band, &[0, 1, 2], &[10, 11, 12]);
    assert_eq!(band, vec![(0, 10, 1), (1, 10, 11), (1, 11, 2), (2, 11, 12)]);

    let mut band = Vec::new();
    surface_row_zip::sew(&mut band, &[0, 1, 2], &[10, 11, 12, 13]);
    // (i+1)*3 <= (j+1)*2: upper steps at i=0 only once j has reached 1.
    assert_eq!(
        band,
        vec![
            (0, 10, 11),
            (0, 11, 1),
            (1, 11, 12),
            (1, 12, 2),
            (2, 12, 13)
        ]
    );

    let mut band = Vec::new();
    surface_row_zip::sew(&mut band, &[], &[1, 2]);
    assert!(band.is_empty());
}

/// A triangle is wound away from the ball's centre on a convex corner and towards it in a pocket.
#[test]
fn rust_only_emit_outward_faces_out_of_the_material() {
    let mut pool = SurfaceVertexPool::new(&[]);
    let a = pool.add(Vector3::new(1.0, 0.0, 0.0));
    let b = pool.add(Vector3::new(0.0, 1.0, 0.0));
    let c = pool.add(Vector3::new(0.0, 0.0, 1.0));

    for (outside, sign) in [(false, 1.0), (true, -1.0)] {
        let mut faces = Vec::new();
        surface_row_zip::emit_outward(&pool, &mut faces, a, c, b, Vector3::ZERO, outside);
        assert_eq!(faces.len(), 1);
        let (v0, v1, v2) = faces[0];
        let from = pool.position_of(v0);
        let normal = (pool.position_of(v1) - from).cross(pool.position_of(v2) - from);
        assert!(normal.dot(Vector3::new(1.0, 1.0, 1.0)) * sign > 0.0);
    }
}

/// The vertices standing along a run come back in order along it, its ends and anything off it
/// left out; the side reading says where along the side a point stands.
#[test]
fn rust_only_standing_between_reads_the_run_in_order() {
    let mut pool = SurfaceVertexPool::new(&[]);
    let from = pool.add(Vector3::new(0.0, 0.0, 0.0));
    let to = pool.add(Vector3::new(10.0, 0.0, 0.0));
    let late = pool.add(Vector3::new(7.0, 0.0, 0.0));
    let off = pool.add(Vector3::new(5.0, 1.0, 0.0));
    let early = pool.add(Vector3::new(2.0, 0.0, 0.0));

    let faces = vec![(from, late, off), (off, early, to)];
    let used = surface_end_cut::vertices_of(&faces);
    assert_eq!(
        used.iter().copied().collect::<Vec<_>>(),
        vec![from, late, off, early, to]
    );

    assert_eq!(
        surface_end_cut::standing_between(&pool, &used, from, to),
        vec![early, late]
    );

    let (on, where_) = surface_split_sewing::stands_on_the_side(
        &pool,
        Vector3::ZERO,
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(7.0, 0.0, 0.0),
    );
    assert!(on);
    assert!((where_ - 0.7).abs() < 1e-12);
    assert!(
        !surface_split_sewing::stands_on_the_side(
            &pool,
            Vector3::ZERO,
            Vector3::new(10.0, 0.0, 0.0),
            Vector3::new(10.0, 0.0, 0.0)
        )
        .0
    );
}

/// The rolled section's cap covers the quarter square between the corner, the flat foot, the
/// profile and the walked rim, and a section with no area is refused with nothing left behind.
#[test]
fn rust_only_the_concave_cap_covers_the_section_or_refuses() {
    let mut pool = SurfaceVertexPool::new(&[]);
    // The profile runs from the wall foot (index 0) to the flat foot.
    let profile = vec![
        pool.add(Vector3::new(0.0, 4.0, 0.0)),
        pool.add(Vector3::new(3.0, 3.0, 0.0)),
        pool.add(Vector3::new(4.0, 0.0, 0.0)),
    ];
    let walked = vec![pool.add(Vector3::new(0.0, 2.0, 0.0))];
    let normal = Vector3::new(0.0, 0.0, 1.0);
    let mut faces = Vec::new();

    let along = surface_end_follows_the_walked_chords::try_cap_with_the_section(
        &mut pool,
        &mut faces,
        &profile,
        0,
        Vector3::ZERO,
        normal,
        &walked,
    )
    .expect("a quarter section is a ring the ears can finish");

    assert_eq!(along, vec![Vec::<i32>::new(); 2]);
    let area: f64 = faces
        .iter()
        .map(|&(a, b, c)| {
            let from = pool.position_of(a);
            0.5 * (pool.position_of(b) - from)
                .cross(pool.position_of(c) - from)
                .dot(normal)
        })
        .sum();
    // Corner, flat foot (4,0), (3,3), wall foot (0,4), walked (0,2): shoelace area 12.
    assert!((area - 12.0).abs() < 1e-9);

    let mut pool = SurfaceVertexPool::new(&[]);
    let flat = vec![
        pool.add(Vector3::new(1.0, 0.0, 0.0)),
        pool.add(Vector3::new(2.0, 0.0, 0.0)),
    ];
    let mut faces = vec![(0, 1, 0)];
    let refused = surface_end_follows_the_walked_chords::try_cap_with_the_section(
        &mut pool,
        &mut faces,
        &flat,
        1,
        Vector3::ZERO,
        normal,
        &[],
    )
    .expect_err("a section on one line covers nothing");
    assert_eq!(
        refused.code(),
        BevelAdvisoryCode::SurfaceStripEndNotSupported
    );
    assert_eq!(faces, vec![(0, 1, 0)]);
}

/// A reading over a pool, as `SurfaceSplitsOnASegment`'s constructor (another package's) sets it up.
fn reading<'a>(pool: &'a SurfaceVertexPool, candidates: Vec<i32>) -> SurfaceSplitsOnASegment<'a> {
    SurfaceSplitsOnASegment {
        pool,
        candidates,
        known: HashMap::new(),
        settled: true,
        the_stretch_that_did_not_settle: None,
    }
}

/// A stretch is read in order either way along it, and a face's ring takes in what stands on its
/// sides, leaving out its own corners.
#[test]
fn rust_only_a_stretch_reads_the_same_both_ways_and_sews_into_a_face() {
    let mut pool = SurfaceVertexPool::new(&[]);
    let a = pool.add(Vector3::new(0.0, 0.0, 0.0));
    let b = pool.add(Vector3::new(10.0, 0.0, 0.0));
    let c = pool.add(Vector3::new(0.0, 10.0, 0.0));
    let p = pool.add(Vector3::new(3.0, 0.0, 0.0));
    let q = pool.add(Vector3::new(6.0, 0.0, 0.0));
    let r = pool.add(Vector3::new(5.0, 5.0, 0.0));

    let mut standing_on = reading(&pool, vec![q, p, r, a]);

    assert_eq!(standing_on.between(a, b), vec![p, q]);
    assert_eq!(standing_on.between(b, a), vec![q, p]);
    assert!(standing_on.settled);

    let ring = surface_split_sewing::ring_with_the_splits_in(&[a, b, c], &mut standing_on);
    assert_eq!(ring, vec![a, p, q, b, r, c]);

    // A needle [a, p, b] along a-b: the stretch names p, so the neighbours carry its middle; a
    // needle whose middle is not a candidate is not carried.
    assert!(surface_split_sewing::the_neighbours_carry_its_middle(
        &pool,
        &mut standing_on,
        &[a, p, b]
    ));
    let mut nothing_named = reading(&pool, vec![]);
    assert!(!surface_split_sewing::the_neighbours_carry_its_middle(
        &pool,
        &mut nothing_named,
        &[a, p, b]
    ));
}

/// Three points closer than the ruler each stand on the stretch between the other two: the
/// refinement stops, says it did not settle, and names the stretch.
#[test]
fn rust_only_a_crowd_finer_than_the_ruler_does_not_settle() {
    let mut pool = SurfaceVertexPool::new(&[]);
    let a = pool.add(Vector3::new(0.0, 0.0, 0.0));
    let b = pool.add(Vector3::new(10.0, 0.0, 0.0));
    let near = pool.near_enough();
    let first = pool.add(Vector3::new(5.0, 0.0, 0.0));
    let second = pool.add(Vector3::new(5.0 + near * 3.0, near * 0.4, 0.0));
    let third = pool.add(Vector3::new(5.0 + near * 6.0, 0.0, 0.0));

    let mut standing_on = reading(&pool, vec![first, second, third]);
    let along = standing_on.between(a, b);

    // Whatever the walk got to, no vertex is visited twice.
    let unique: IndexSet<i32> = along.iter().copied().collect();
    assert_eq!(unique.len(), along.len());
    if !standing_on.settled {
        assert!(standing_on.the_stretch_that_did_not_settle.is_some());
    }
}

/// A face the subtraction did not rebuild is split where a new vertex stands on its edge, and the
/// original is dropped.
#[test]
fn rust_only_sew_the_rim_splits_an_untouched_neighbour() {
    let mut pool = SurfaceVertexPool::new(&[]);
    let a = pool.add(Vector3::new(0.0, 0.0, 0.0));
    let b = pool.add(Vector3::new(10.0, 0.0, 0.0));
    let c = pool.add(Vector3::new(0.0, 10.0, 0.0));
    let first_new = pool.count();
    let m = pool.add(Vector3::new(5.0, 0.0, 0.0));

    let mut rebuilt = vec![(a, b, c)];
    surface_end_gives_up_the_section::sew_the_rim(&mut pool, &mut rebuilt, &[0], first_new);

    assert_eq!(rebuilt.len(), 2);
    assert!(rebuilt.iter().all(|&(x, y, z)| x != y && y != z && z != x));
    assert!(rebuilt.iter().any(|&(x, y, z)| [x, y, z].contains(&m)));

    // Nothing new: nothing changes.
    let mut untouched = vec![(a, b, c)];
    let count = pool.count();
    surface_end_gives_up_the_section::sew_the_rim(&mut pool, &mut untouched, &[0], count);
    assert_eq!(untouched, vec![(a, b, c)]);
}

/// The piece across a side says which of its corners stand on it; a corner of the neighbour that
/// it does not name is left out even though it measures as on the side.
#[test]
fn rust_only_a_bites_shared_side_is_read_off_the_neighbour() {
    let mut pool = SurfaceVertexPool::new(&[]);
    let a = pool.add(Vector3::new(0.0, 0.0, 0.0));
    let b = pool.add(Vector3::new(10.0, 0.0, 0.0));
    let up = pool.add(Vector3::new(5.0, 5.0, 0.0));
    let down = pool.add(Vector3::new(5.0, -5.0, 0.0));
    let mid = pool.add(Vector3::new(4.0, 0.0, 0.0));
    let stray = pool.add(Vector3::new(7.0, 0.0, 0.0));

    // Piece 0 walks a -> b -> up; piece 1 walks b -> mid -> a -> down, carrying mid on the side.
    let pieces = vec![vec![a, b, up], vec![b, mid, a, down]];
    let mut carried_by: HashMap<i32, Vec<i32>> = HashMap::new();
    for (index, piece) in pieces.iter().enumerate() {
        for &vertex in piece {
            let carriers = carried_by.entry(vertex).or_default();
            if carriers.last() != Some(&(index as i32)) {
                carriers.push(index as i32);
            }
        }
    }
    let shared = SurfaceBiteSharedSides { pieces, carried_by };

    let (chain, across) = shared
        .try_the_chain_across(&pool, 0, a, b)
        .expect("piece 1 carries both ends");
    assert_eq!(chain, vec![mid]);
    assert_eq!(across, &[b, mid, a, down][..]);
    assert!(shared.try_the_chain_across(&pool, 0, up, a).is_none());

    let carried: IndexSet<i32> = [stray, mid, down].into_iter().collect();
    let grown = surface_convex_bite::with_the_vertices_standing_on_it(
        &pool,
        vec![a, b, up],
        &carried,
        None,
        -1,
    );
    assert_eq!(grown, vec![a, mid, stray, b, up]);

    // `stray` is named by nobody, so it is still taken in; `mid` is named by the chain.
    let grown = surface_convex_bite::with_the_vertices_standing_on_it(
        &pool,
        vec![a, b, up],
        &carried,
        Some(&shared),
        0,
    );
    assert_eq!(grown, vec![a, mid, stray, b, up]);
}

/// The corner fans sweep one way round the corner and face out of the wall, and refuse a rail that
/// doubles back; the strip's triangle skips a repeated vertex.
#[test]
fn rust_only_the_corner_fans_sweep_one_way_or_refuse() {
    let mut pool = SurfaceVertexPool::new(&[]);
    let out = Vector3::new(0.0, 0.0, 1.0);
    let rail: Vec<i32> = (0..4)
        .map(|step| {
            let angle = step as f64 * 0.4;
            pool.add(Vector3::new(angle.cos() * 5.0, angle.sin() * 5.0, 0.0))
        })
        .collect();
    let grid: Vec<Vec<i32>> = rail.iter().map(|&vertex| vec![vertex]).collect();
    let bead = pool.add(Vector3::new(6.0, -1.0, 0.0));
    let edge = pool.add(Vector3::new(-1.0, 6.0, 0.0));

    let crossing = |before: i32, after: i32| SurfaceMixedCornerDualWallCrossing {
        before,
        after,
        at: Vector3::ZERO,
    };
    let mut faces = Vec::new();
    surface_mixed_corner_dual_wall::try_fill_the_root(
        &mut pool,
        &mut faces,
        Vector3::ZERO,
        out,
        &grid,
        crossing(-1, 0),
        bead,
        crossing(3, 4),
        edge,
    )
    .expect("the rail sweeps one way round the corner");
    assert_eq!(faces.len(), 5);
    for &(a, b, c) in &faces {
        let from = pool.position_of(a);
        let normal = (pool.position_of(b) - from).cross(pool.position_of(c) - from);
        assert!(normal.dot(out) > 0.0);
    }

    let notch_crossing = |before: i32, after: i32| SurfaceTwoCreaseJunctionWallCrossing {
        before,
        after,
        at: Vector3::ZERO,
    };
    let mut notch = Vec::new();
    surface_two_crease_junction_wall::try_fill_the_notch(
        &mut pool,
        &mut notch,
        Vector3::ZERO,
        out,
        &grid,
        notch_crossing(-1, 0),
        bead,
        notch_crossing(2, 3),
        edge,
    )
    .expect("the same rail fills the notch");
    assert_eq!(notch, faces);

    let apex = pool.add(Vector3::ZERO);
    let doubled = surface_mixed_corner_dual_wall::try_fan(
        &pool,
        &mut Vec::new(),
        apex,
        Vector3::ZERO,
        out,
        &[rail[0], rail[2], rail[1]],
    )
    .expect_err("a rail that doubles back is refused");
    assert_eq!(doubled.code(), BevelAdvisoryCode::SurfaceCornerNotSupported);

    let mut drawn = Vec::new();
    surface_mixed_corner_run_out_strip::draw(&pool, &mut drawn, apex, rail[0], rail[0], out);
    assert!(drawn.is_empty());
    surface_mixed_corner_run_out_strip::draw(&pool, &mut drawn, apex, rail[1], rail[0], out);
    assert_eq!(drawn, vec![(apex, rail[0], rail[1])]);
}

/// One level of refinement splits every interior edge once on the sphere and leaves the seam's
/// edges whole: four children for a free triangle, three for one carrying a seam edge.
#[test]
fn rust_only_the_patch_refinement_keeps_the_seam() {
    let mut directions = vec![
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
    ];
    let free = bevel_patch_refinement::refine(&mut directions.clone(), &[(0, 1, 2)], None);
    assert_eq!(free.len(), 4);

    let seam = vec![vec![2, 0]];
    let refined = bevel_patch_refinement::refine(&mut directions, &[(0, 1, 2)], Some(&seam));
    assert_eq!(refined.len(), 3);
    assert_eq!(directions.len(), 5);
    for direction in &directions {
        assert!((direction.length() - 1.0).abs() < 1e-12);
    }
    // The seam edge 2-0 is still an edge of exactly one child.
    let carrying = refined
        .iter()
        .filter(|&&(a, b, c)| {
            [(a, b), (b, c), (c, a)]
                .iter()
                .any(|&(x, y)| (x, y) == (2, 0) || (x, y) == (0, 2))
        })
        .count();
    assert_eq!(carrying, 1);
}

/// A square octant corner of n chords a side is `4n^2 - 3n` faces over `2n^2 + 1` points, every
/// one on the ball and facing out of it; the pocket corner is the same faces wound the other way.
#[test]
fn rust_only_the_corner_rows_lid_the_octant() {
    let chords = 4;
    for material_outside_the_ball in [false, true] {
        let mut pool = SurfaceVertexPool::new(&[]);
        let axes = [
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
        ];
        let size = 5.0;
        let sides: Vec<Vec<i32>> = (0..3)
            .map(|side| {
                (0..=chords)
                    .map(|step| {
                        let angle = step as f64 / chords as f64 * std::f64::consts::FRAC_PI_2;
                        let at = axes[side] * angle.cos() + axes[(side + 1) % 3] * angle.sin();
                        pool.add(at * size)
                    })
                    .collect()
            })
            .collect();
        let mut faces = Vec::new();

        assert!(surface_corner_patch_rows::try_emit(
            &mut pool,
            &mut faces,
            &sides,
            Vector3::ZERO,
            size,
            material_outside_the_ball
        ));

        assert_eq!(faces.len() as i32, 4 * chords * chords - 3 * chords);
        assert_eq!(pool.count(), 2 * chords * chords + 1);
        for &(a, b, c) in &faces {
            let from = pool.position_of(a);
            let normal = (pool.position_of(b) - from).cross(pool.position_of(c) - from);
            let middle = (from + pool.position_of(b) + pool.position_of(c)) / 3.0;
            let sign = if material_outside_the_ball { -1.0 } else { 1.0 };
            assert!(normal.dot(middle) * sign > 0.0);
        }
        for vertex in 0..pool.count() {
            assert!((pool.position_of(vertex).length() - size).abs() < 1e-5);
        }
    }

    // A rim off the ball is refused.
    let mut pool = SurfaceVertexPool::new(&[]);
    let off: Vec<Vec<i32>> = (0..3)
        .map(|side| {
            vec![
                pool.add(Vector3::new(side as f64, 0.0, 0.0)),
                pool.add(Vector3::new(0.0, 9.0, 0.0)),
            ]
        })
        .collect();
    assert!(!surface_corner_patch_rows::try_emit(
        &mut pool,
        &mut Vec::new(),
        &off,
        Vector3::ZERO,
        5.0,
        false
    ));
}

/// A plan built straight from its parts reads them back: no mixed corners, the roles it was given,
/// no regions, and the part's ruler not yet read.
#[test]
fn rust_only_a_plan_reads_back_what_it_was_built_from() {
    let plan = SurfaceBevelPlan::new(
        Vec::new(),
        true,
        2.5,
        true,
        None,
        HashMap::new(),
        false,
        false,
        None,
        None,
        IndexMap::new(),
        SurfaceEdgeRoles::default(),
        SurfacePlanRecipe {
            chain: BevelSweptChain::default(),
            regions: None,
            curve_regions: None,
            round_profile: true,
        },
    );

    assert_eq!(plan.mixed_corner_count(), 0);
    assert!(plan.mixed_corner_keys().is_empty());
    assert_eq!(*plan.roles(), SurfaceEdgeRoles::default());
    assert!(plan.regions().is_none());
    assert!(plan.junction_arms.borrow().is_empty());
    assert_eq!(plan.near_enough_on_the_part.get(), None);
    assert!(plan.is_closed && plan.round_profile && plan.size == 2.5);
}
