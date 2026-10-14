//! The band's removal, held to the requirement - the part that
//! is left plus the band that came out is still the solid that went in, the only open edges are the
//! gap's own, the gap's boundary stands at its setback with the kept surface on its left, and a
//! band that would reach a foreign feature refuses by size and names the room truthfully.

use std::cell::Ref;
use std::time::Instant;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::band_gap_reading::room_margin;
use geometry_processing::bevel::level_line::chain_setback;
use geometry_processing::bevel::level_line::seam_edges::FeatureEdgeTest;
use geometry_processing::bevel::level_line::{
    BandGapRefusal, BandGapResult, BandProfile, ChainField, LevelLineMesh,
};
use std::rc::Rc;
use vector_math::{Matrix4X4, Vector3};

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::box_with_square_hole;
use crate::level_line_test_geometry::{
    cube_solid, cylinder_solid, open_edges, signed_volume_of, surface_area,
    volume_of_every_triangle_even_removed,
};

/// A fold threshold that reads a 90 degree corner as a feature and a 16-sided cylinder as smooth.
const FOLD_DEGREES: f64 = 45.0;

/// The band gap of one chain at `size` and `profile`, every other argument at its default.
fn build(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    closed: bool,
    feature: FeatureEdgeTest,
    size: f64,
    profile: BandProfile,
) -> BandGapResult {
    band_gap_entry_points::build(
        positions,
        indices,
        chain,
        closed,
        Some(feature),
        size,
        profile,
        &BandGapBuildOptions::default(),
    )
    .expect("a valid pick")
}

fn mesh(result: &BandGapResult) -> Ref<'_, LevelLineMesh> {
    result.mesh.as_ref().expect("the working mesh").borrow()
}

fn chain(result: &BandGapResult) -> Rc<ChainField> {
    result.chain().expect("the chain")
}

/// The room margin of a pick whose largest unit setback is `unit`, at the default 1e-9 snap.
fn margin(unit: f64) -> f64 {
    room_margin(unit, 1e-9)
}

/// The convention, stated and tested: reading the chain in its own direction, the triangle whose
/// winding traverses that edge the same way is the LEFT side, tag 1. Walking the cube's top rim
/// 4-5-6-7, that is the top face; the four walls are the right side.
#[test]
fn left_of_the_chain_is_the_triangle_whose_winding_runs_with_it() {
    let result = cube_top_rim(0.25, BandProfile::Chamfer);

    assert!(result.built());
    let lefts: Vec<_> = result.loops.iter().filter(|l| l.side == 1).collect();
    let rights: Vec<_> = result.loops.iter().filter(|l| l.side == 2).collect();
    assert_eq!(lefts.len(), 1);
    assert_eq!(rights.len(), 1);
    let m = mesh(&result);

    assert!(
        lefts[0]
            .vertices
            .iter()
            .all(|&v| (m.position(v).z - 1.0).abs() < 1e-9),
        "the left region is the top face, so its boundary is inset within that face"
    );
    assert!(
        rights[0]
            .vertices
            .iter()
            .all(|&v| (m.position(v).z - 0.75).abs() < 1e-9),
        "the right region is the four walls, so its boundary is the setback down them"
    );
}

/// The central claim of the removal. Nothing is moved and nothing is invented: the kept mesh and
/// the band that left still measure exactly the solid that went in.
#[test]
fn the_kept_mesh_plus_the_band_is_still_the_solid_that_went_in() {
    let (positions, indices) = cube_solid(1.0);
    let before = signed_volume_of(&positions, &indices);
    let before_area = 4.0 * 6.0;

    let result = cube_top_rim(0.3, BandProfile::Fillet);

    assert!(result.built());
    assert!(!result.removed_triangles.is_empty());
    let m = mesh(&result);
    assert!((volume_of_every_triangle_even_removed(&m) - before).abs() <= 1e-9);
    assert!((surface_area(&m) - before_area).abs() <= 1e-9);
    assert_eq!(m.removed_count(), result.removed_triangles.len() as i32);
}

/// After the removal the mesh's only open edges are the gap's: the chain's own edges left with the
/// band, so every open edge is a loop edge, and each is open exactly once.
#[test]
fn the_only_open_edges_left_are_the_gaps_own() {
    let result = cube_top_rim(0.3, BandProfile::Fillet);

    assert!(result.built());
    let mut open: Vec<(i32, i32)> = open_edges(&mesh(&result))
        .into_iter()
        .map(|e| key(e.0, e.1))
        .collect();
    let mut loop_edges = loop_edges(&result);

    let mut distinct = loop_edges.clone();
    distinct.sort();
    distinct.dedup();
    assert_eq!(
        loop_edges.len(),
        distinct.len(),
        "each loop edge is open exactly once"
    );
    open.sort();
    loop_edges.sort();
    assert_eq!(open, loop_edges);
}

/// A closed chain leaves a closed loop on each side, standing at the setback from the chain,
/// walked with the kept surface on its left and carrying the chain parameter of every one of its
/// vertices.
#[test]
fn a_closed_chain_leaves_one_closed_loop_per_side_at_its_setback() {
    let result = cube_top_rim(0.3, BandProfile::Fillet);

    assert!(result.built());
    assert_eq!(result.loops.len(), 2);
    assert!(result.loops.iter().all(|l| l.is_closed));
    let mut sides: Vec<i32> = result.loops.iter().map(|l| l.side).collect();
    sides.sort();
    assert_eq!(sides, vec![1, 2]);

    let m = mesh(&result);
    let field = chain(&result);
    for gap_loop in &result.loops {
        for &vertex in &gap_loop.vertices {
            let sample = field.evaluate(m.position(vertex)).expect("no budget");
            assert!(
                sample.value.abs() <= 1e-9,
                "a loop vertex stands exactly at its setback from the chain"
            );
        }

        for i in 0..gap_loop.vertices.len() {
            let parameter = field
                .evaluate(m.position(gap_loop.vertices[i]))
                .expect("no budget")
                .parameter;
            assert!((gap_loop.parameters[i] - parameter).abs() <= 1e-12);
        }
    }
}

/// The kept surface is on the left of the walk. Each step runs along its own kept triangle's
/// winding, so the outward normal of that triangle, the step and the step's outward offset agree in
/// hand.
#[test]
fn every_step_of_a_loop_runs_along_its_kept_triangles_winding() {
    let result = cube_top_rim(0.3, BandProfile::Fillet);
    let m = mesh(&result);

    for gap_loop in &result.loops {
        let count = gap_loop.vertices.len();
        for i in 0..count {
            if i + 1 == count && !gap_loop.is_closed {
                break;
            }

            let from = gap_loop.vertices[i];
            let to = gap_loop.vertices[(i + 1) % count];
            let standing = m.triangles_on(from, to);
            assert_eq!(standing.len(), 1, "a gap edge keeps one triangle");
            let t = m.triangles()[standing[0] as usize];
            let runs_with = (t.a == from && t.b == to)
                || (t.b == from && t.c == to)
                || (t.c == from && t.a == to);
            assert!(runs_with);
        }
    }
}

/// A chamfer's setback is the size itself, so the two loops of a square-cornered rim stand exactly
/// the size back from the chain - the field's own statement, read on the mesh that came out.
#[test]
fn a_chamfers_loops_stand_the_size_back_from_the_chain() {
    let result = cube_top_rim(0.4, BandProfile::Chamfer);
    let m = mesh(&result);
    let field = chain(&result);

    for gap_loop in &result.loops {
        for &vertex in &gap_loop.vertices {
            let distance = field
                .evaluate(m.position(vertex))
                .expect("no budget")
                .distance;
            assert!((distance - 0.4).abs() <= 1e-9);
        }
    }
}

/// An open chain ending on end faces: the band runs off both ends, which is not a collision, and
/// cuts each END region too - an open run per side (tags 1, 2) and per end (tags 3, 4), every
/// vertex on the level line.
#[test]
fn an_open_chain_leaves_an_open_run_per_side_and_per_end() {
    let (positions, indices) = cube_solid(1.0);
    let before = signed_volume_of(&positions, &indices);

    let result = build(
        &positions,
        &indices,
        &[4, 5],
        false,
        feature_edges_over(&positions, &indices, FOLD_DEGREES),
        0.3,
        BandProfile::Fillet,
    );

    assert!(
        result.built(),
        "running off the chain's own ends is not a collision"
    );
    let m = mesh(&result);
    assert!((volume_of_every_triangle_even_removed(&m) - before).abs() <= 1e-9);
    assert_eq!(result.loops.len(), 4);
    assert!(result.loops.iter().all(|l| !l.is_closed));
    let mut sides: Vec<i32> = result.loops.iter().map(|l| l.side).collect();
    sides.sort();
    assert_eq!(sides, vec![1, 2, 3, 4]);

    let field = chain(&result);
    for gap_loop in &result.loops {
        for &vertex in &gap_loop.vertices {
            let value = field.evaluate(m.position(vertex)).expect("no budget").value;
            assert!(value.abs() <= 1e-9);
        }
    }
}

/// A tessellated cylinder's top rim: the same closed solid, two closed loops, and the parameter
/// running one way round each of them - non-decreasing but for the single wrap where the walk
/// passes the chain's own start.
#[test]
fn a_cylinders_top_rim_leaves_loops_whose_parameter_runs_one_way() {
    let (positions, indices) = cylinder_solid(16, 1.0, 2.0);
    let before = signed_volume_of(&positions, &indices);
    let rim: Vec<i32> = (16..32).collect();

    let result = build(
        &positions,
        &indices,
        &rim,
        true,
        feature_edges_over(&positions, &indices, FOLD_DEGREES),
        0.2,
        BandProfile::Fillet,
    );

    assert!(result.built());
    assert!((volume_of_every_triangle_even_removed(&mesh(&result)) - before).abs() <= 1e-9);
    assert_eq!(result.loops.len(), 2);
    assert!(result.loops.iter().all(|l| l.is_closed));

    let mut senses = Vec::new();
    for gap_loop in &result.loops {
        // Kept on the left puts the two sides' loops in opposite senses about the chain, so one
        // climbs in parameter and the other falls; either way it turns back exactly once, at the
        // chain's start.
        let mut climbs = 0;
        let mut falls = 0;
        let count = gap_loop.parameters.len();
        for i in 0..count {
            let here = gap_loop.parameters[i];
            let next = gap_loop.parameters[(i + 1) % count];
            if next > here + 1e-9 {
                climbs += 1;
            } else if next < here - 1e-9 {
                falls += 1;
            }
        }

        assert_eq!(
            i32::min(climbs, falls),
            1,
            "the parameter runs one way round the loop and turns back once, where the walk passes the chain's start"
        );
        senses.push((climbs - falls).signum());
    }

    assert_eq!(
        senses[0], -senses[1],
        "kept on the left walks the two sides of the chain in opposite senses"
    );
}

/// The room, on a mouth near the edge of its face: the band from the hole's rim reaches the block's
/// outer rim at 1.5, so any smaller size builds and any larger refuses - and both name the same
/// room, because the room is read off the region's own boundary and not off the size asked for.
#[test]
fn the_room_is_the_same_number_whether_the_build_fits_or_refuses() {
    let (positions, indices, rim) = box_with_square_hole(2.0, 0.5, 1.0);
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);

    let fits = build(
        &positions,
        &indices,
        &rim,
        true,
        feature.clone(),
        1.0,
        BandProfile::Chamfer,
    );
    let refused = build(
        &positions,
        &indices,
        &rim,
        true,
        feature,
        2.0,
        BandProfile::Chamfer,
    );

    assert!(fits.built());
    assert!(!refused.built());
    assert_eq!(refused.refusal, BandGapRefusal::Size);
    assert!(
        (fits.room - (1.5 - margin(chain_setback::for_chamfer(1.0)))).abs() <= 1e-6,
        "the hole's rim stands 1.5 from the block's outer rim on the top face, less the margin the kept strip needs to have width"
    );
    assert!((refused.room - fits.room).abs() <= 1e-9);
}

/// An open chain drawn across the middle of a face with NOTHING counted as a fold has no crease to
/// bevel at all, and is refused by that name at every size. It is not
/// `ChainDoesNotDivideItsSides`, which since the dying crease landed is a closed chain's refusal
/// alone: an open chain's sides MAY be one region. It is the band's own statement - the cut took
/// no triangle off and left no boundary, so there was nothing there to round.
#[test]
fn a_chain_with_no_crease_to_bevel_is_refused_by_name() {
    let (positions, indices) = cube_solid(1.0);
    let none = || -> FeatureEdgeTest { Rc::new(|_, _| false) };

    let result = build(
        &positions,
        &indices,
        &[4, 6],
        false,
        none(),
        0.2,
        BandProfile::Fillet,
    );

    assert!(!result.built());
    assert_eq!(result.refusal, BandGapRefusal::ChainHasNoCreaseToBevel);

    // At every size, and the same name: a pick with nothing to round is not a pick that is too big.
    for size in [2.0, 50.0] {
        let bigger = build(
            &positions,
            &indices,
            &[4, 6],
            false,
            none(),
            size,
            BandProfile::Fillet,
        );
        assert_eq!(bigger.refusal, BandGapRefusal::ChainHasNoCreaseToBevel);
    }
}

/// The gap is a property of the part, not of where the part stands: moved and un-moved it is the
/// same gap to the last bit the arithmetic allows. Run twice from the same input it is identical.
#[test]
fn a_moved_part_gives_the_same_gap_and_the_same_input_the_same_answer() {
    let (positions, indices) = cube_solid(1.0);
    let motion = Matrix4X4::create_rotation(Vector3::new(0.3, -0.7, 0.2))
        * Matrix4X4::create_translation(5.0, -3.0, 11.0);
    let moved: Vec<Vector3> = positions.iter().map(|&p| p.transform(motion)).collect();

    let still = cube_top_rim(0.3, BandProfile::Fillet);
    let again = cube_top_rim(0.3, BandProfile::Fillet);
    let travelled = build(
        &moved,
        &indices,
        &[4, 5, 6, 7],
        true,
        feature_edges_over(&moved, &indices, FOLD_DEGREES),
        0.3,
        BandProfile::Fillet,
    );

    assert!(travelled.built());
    let travelled_mesh = mesh(&travelled);
    let still_mesh = mesh(&still);
    assert_eq!(travelled_mesh.triangle_count(), still_mesh.triangle_count());
    assert_eq!(travelled.removed_triangles, still.removed_triangles);
    assert_eq!(
        again.removed_triangles, still.removed_triangles,
        "the same input gives the same answer"
    );

    let back = motion.inverted();
    for v in 0..still_mesh.vertex_count() {
        let unmoved = travelled_mesh.position(v).transform(back);
        assert!((unmoved - still_mesh.position(v)).length() < 1e-9);
    }

    for side in [1, 2] {
        let here = single_side(&still, side);
        let there = single_side(&travelled, side);
        assert_eq!(there, here);
    }
}

/// The same part, its triangles written with their corners rotated - the same solid, the same
/// windings, the same vertex indices, and only the order the mesh's edges are first named in is
/// different. The cut must be the SAME cut, to the bit: the same vertices in the same places at
/// the same indices, the same band removed, the same boundary loops.
///
/// This is the requirement the module's edges are kept in ascending (low, high) order for. The
/// working mesh holds its edges in a dictionary whose key order follows the order they were first
/// attached in - and that order changes mid-build as triangles detach and re-attach. A build that
/// rooted and split in it would split into different vertex indices for reasons the caller cannot
/// see. Rotating the corners is the smallest change to the input that moves that order and nothing
/// else.
#[test]
fn rotating_every_triangles_corners_gives_the_same_cut_to_the_bit() {
    let (positions, indices) = cylinder_solid(16, 1.0, 2.0);
    let rim: Vec<i32> = (16..32).collect();
    let rotated: Vec<(i32, i32, i32)> = indices.iter().map(|&(a, b, c)| (b, c, a)).collect();

    let straight = build(
        &positions,
        &indices,
        &rim,
        true,
        feature_edges_over(&positions, &indices, FOLD_DEGREES),
        0.2,
        BandProfile::Fillet,
    );
    let turned = build(
        &positions,
        &rotated,
        &rim,
        true,
        feature_edges_over(&positions, &rotated, FOLD_DEGREES),
        0.2,
        BandProfile::Fillet,
    );

    assert!(straight.built());
    assert!(turned.built());
    assert_eq!(turned.room, straight.room);
    let straight_mesh = mesh(&straight);
    let turned_mesh = mesh(&turned);
    assert_eq!(turned_mesh.vertex_count(), straight_mesh.vertex_count());
    assert_eq!(turned_mesh.triangle_count(), straight_mesh.triangle_count());

    for v in 0..straight_mesh.vertex_count() {
        assert_eq!(
            turned_mesh.position(v),
            straight_mesh.position(v),
            "vertex {v} is the same vertex of the same cut"
        );
    }

    assert_eq!(
        turned.removed_triangles, straight.removed_triangles,
        "the same band comes out"
    );

    for side in [1, 2] {
        assert_eq!(single_side(&turned, side), single_side(&straight, side));
    }
}

/// What the removal costs on the fixtures above - a row recorded, not a budget asserted.
#[test]
fn the_cost_of_a_removal_is_recorded() {
    let (box_positions, box_indices, rim) = box_with_square_hole(2.0, 0.5, 1.0);
    let (cylinder_positions, cylinder_indices) = cylinder_solid(16, 1.0, 2.0);

    let rows = [
        time("cube top rim, fillet 0.3", || {
            cube_top_rim(0.3, BandProfile::Fillet)
        }),
        time("box mouth rim, chamfer 1.0", || {
            build(
                &box_positions,
                &box_indices,
                &rim,
                true,
                feature_edges_over(&box_positions, &box_indices, FOLD_DEGREES),
                1.0,
                BandProfile::Chamfer,
            )
        }),
        time("cylinder top rim, fillet 0.2", || {
            build(
                &cylinder_positions,
                &cylinder_indices,
                &(16..32).collect::<Vec<i32>>(),
                true,
                feature_edges_over(&cylinder_positions, &cylinder_indices, FOLD_DEGREES),
                0.2,
                BandProfile::Fillet,
            )
        }),
    ];

    for row in &rows {
        println!("{row}");
    }

    assert_eq!(rows.len(), 3);
}

/// A side can run out of ITSELF with nothing foreign in the way: the cube's 2 by 2 top face is
/// swallowed whole once the setback reaches its inradius, and there is then no boundary loop to
/// fill. The room is that size, and it is the same number whether the size asked for was above or
/// below it.
#[test]
fn a_top_face_refuses_above_its_own_room_and_builds_below_it() {
    let below = cube_top_rim(0.9, BandProfile::Chamfer);
    let above = cube_top_rim(1.1, BandProfile::Chamfer);

    assert!(
        below.built(),
        "0.9 leaves a 0.2 square of the top face standing"
    );
    assert!(!above.built(), "1.1 would take the whole top face");
    assert_eq!(above.refusal, BandGapRefusal::Size);

    assert!(
        (above.room - (1.0 - margin(chain_setback::for_chamfer(1.0)))).abs() <= 1e-3,
        "a chamfer's setback is its size, so the room is the face's inradius less the strip's margin"
    );
    assert!(
        (below.room - above.room).abs() <= 1e-9,
        "the room is a fact about the part and the pick, not about the size that was asked"
    );
}

fn time(what: &str, build: impl FnOnce() -> BandGapResult) -> String {
    let watch = Instant::now();
    let result = build();
    let elapsed = watch.elapsed();
    let triangles = result
        .mesh
        .as_ref()
        .map_or(0, |m| m.borrow().triangle_count());
    format!(
        "{what}: {:.1} ms, {triangles} triangles, {} removed",
        elapsed.as_secs_f64() * 1000.0,
        result.removed_triangles.len()
    )
}

fn cube_top_rim(size: f64, profile: BandProfile) -> BandGapResult {
    let (positions, indices) = cube_solid(1.0);
    build(
        &positions,
        &indices,
        &[4, 5, 6, 7],
        true,
        feature_edges_over(&positions, &indices, FOLD_DEGREES),
        size,
        profile,
    )
}

/// The vertices of the one loop on `side`; fails unless there is exactly one.
fn single_side(result: &BandGapResult, side: i32) -> Vec<i32> {
    let found: Vec<_> = result.loops.iter().filter(|l| l.side == side).collect();
    assert_eq!(found.len(), 1);
    found[0].vertices.clone()
}

fn loop_edges(result: &BandGapResult) -> Vec<(i32, i32)> {
    let mut edges = Vec::new();
    for gap_loop in &result.loops {
        let count = gap_loop.vertices.len();
        for i in 0..count {
            if i + 1 == count && !gap_loop.is_closed {
                break;
            }

            edges.push(key(
                gap_loop.vertices[i],
                gap_loop.vertices[(i + 1) % count],
            ));
        }
    }

    edges
}

fn key(a: i32, b: i32) -> (i32, i32) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}
