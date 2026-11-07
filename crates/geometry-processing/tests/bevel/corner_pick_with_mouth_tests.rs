//! A pick with corners that ALSO holds a closed chain touching none of them - the owner file "Outer
//! faces should still do the bevel" in miniature: a box (4 x 4 x 2) with a square hole through it,
//! its top rim and front rim picked as three open chains meeting at two corners, plus the hole's
//! top mouth. The mouth's two loops are each one owner all the way round, so the corner fill hands
//! them to the per-chain zip. Each test's cases run as a loop inside it.

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::corner_fill;
use geometry_processing::bevel::level_line::gap_fill::{self, DEFAULT_FILLET_SEGMENTS};
use geometry_processing::bevel::level_line::{
    BandGapResult, BandProfile, GapFillRefusal, GapFillResult,
};
use vector_math::{Vector2, Vector3};

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::{box_with_square_hole, SolidWithChains};
use crate::level_line_test_geometry::distance_to_surface;
use crate::level_line_test_geometry::{
    cube_solid, signed_volume, signed_volume_of, winding_number,
};
use crate::level_line_test_solids::prism_on_outline;
use crate::several_chains_fixtures::FOLD_DEGREES;
use crate::sweep_mesh_measures;

/// The top face's front edge, the rest of the top rim, the rest of the front rim, and the hole's
/// mouth.
fn pick() -> Vec<Vec<i32>> {
    vec![
        vec![0, 1],
        vec![1, 2, 3, 0],
        vec![1, 9, 8, 0],
        vec![4, 5, 6, 7],
    ]
}

const CLOSED: [bool; 4] = [false, false, false, true];

/// The band gap of the pick at `size` and `profile`, every other argument at its default.
fn build(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    size: f64,
    profile: BandProfile,
) -> BandGapResult {
    band_gap_entry_points::build_pick(
        positions,
        indices,
        chains,
        closed,
        Some(feature_edges_over(positions, indices, FOLD_DEGREES)),
        size,
        profile,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick is well formed")
}

/// The gap's fill at `profile`, with the default fillet segments.
fn fill(gap: &BandGapResult, profile: BandProfile) -> GapFillResult {
    gap_fill::fill(gap, profile, DEFAULT_FILLET_SEGMENTS).expect("the fill runs")
}

#[test]
fn corners_and_a_hole_mouth_chamfer_closed_as_one_shell() {
    for size in [0.1, 0.3] {
        fills_closed_as_one_shell(size, BandProfile::Chamfer);
    }
}

#[test]
fn corners_and_a_hole_mouth_fillet_closed_as_one_shell() {
    for size in [0.1, 0.3] {
        fills_closed_as_one_shell(size, BandProfile::Fillet);
    }
}

fn fills_closed_as_one_shell(size: f64, profile: BandProfile) {
    let (positions, indices, _) = box_with_square_hole(2.0, 0.5, 1.0);
    let before = signed_volume_of(&positions, &indices);
    let gap = build(&positions, &indices, &pick(), &CLOSED, size, profile);
    println!(
        "{profile:?} {size}: band {:?}, room {}",
        gap.refusal, gap.room
    );
    assert!(gap.built());
    assert!(corner_fill::takes(Some(&gap), profile));
    assert_every_rail_ends_at_a_tie(&gap);

    let filled = fill(&gap, profile);
    println!(
        "{profile:?} {size}: fill {:?}, chain {}, vertex {}, edge {:?}",
        filled.refusal, filled.offending_chain, filled.offending_vertex, filled.open_edge
    );
    assert_eq!(filled.refusal, GapFillRefusal::None);

    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        mesh.vertices(),
        &kept
    ));
    assert_eq!(
        sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
        1
    );

    // Every picked edge is convex, so the fill only takes material: less volume, and no invented
    // point outside the part that went in.
    let volume = signed_volume(&mesh);
    println!(
        "{profile:?} {size}: volume {volume:?} of {before:?}, mouth strip {} triangles",
        filled.triangles_of_chain[3]
    );
    assert!(volume < before);
    assert!(volume > 0.0);
    assert!(filled.triangles_of_chain[3] > 0);
    for &v in &filled.added_vertices {
        let turns = winding_number(&positions, &indices, mesh.position(v));
        assert!(turns > 0.45, "added vertex {v} at {:?}", mesh.position(v));
    }
}

/// The same box's pick as a fillet removes less than its chamfer: the arc stands outside the flat.
#[test]
fn the_fillet_removes_less_than_the_chamfer() {
    for size in [0.1, 0.3] {
        let (positions, indices, _) = box_with_square_hole(2.0, 0.5, 1.0);
        let volume = |profile: BandProfile| {
            let filled = fill(
                &build(&positions, &indices, &pick(), &CLOSED, size, profile),
                profile,
            );
            let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
            signed_volume(&mesh)
        };
        assert!(volume(BandProfile::Fillet) > volume(BandProfile::Chamfer));
    }
}

/// The two adjacent cube faces' pick: its rails end at ties too (its edges pass through them).
#[test]
fn every_rail_of_the_cubes_corner_groups_ends_at_a_tie() {
    let (positions, indices) = cube_solid(1.0);
    let two_faces = vec![vec![4, 5], vec![5, 6, 7, 4], vec![5, 1, 0, 4]];
    let gap = build(
        &positions,
        &indices,
        &two_faces,
        &[false, false, false],
        0.2,
        BandProfile::Chamfer,
    );
    assert!(gap.built());
    assert_every_rail_ends_at_a_tie(&gap);
}

/// Wherever a corner group's loop changes owner, one of the two vertices either side of the change
/// reads the two chains' fields equal (`corner_fill::fields_tie`): the rail ENDS at the mitre point
/// rather than short of it.
fn assert_every_rail_ends_at_a_tie(gap: &BandGapResult) {
    let mesh = gap.mesh.as_ref().expect("a built mesh").borrow();
    for gap_loop in &gap.loops {
        let owners = gap_loop
            .vertex_owners
            .as_ref()
            .expect("a built loop reads its owners");
        let n = gap_loop.vertices.len();
        for i in 0..n {
            let previous = (i + n - 1) % n;
            let was = owners[previous];
            let now = owners[i];
            if was == now {
                continue;
            }

            let a = &gap.chains[was as usize];
            let b = &gap.chains[now as usize];
            let here = mesh.position(gap_loop.vertices[i]);
            let there = mesh.position(gap_loop.vertices[previous]);
            let ties = corner_fill::fields_tie(a, b, here).expect("the fields read")
                || corner_fill::fields_tie(a, b, there).expect("the fields read");
            assert!(
                ties,
                "chain {was} hands over to {now} between {there:?} and {here:?} with no tie"
            );
        }
    }
}

/// The L block in miniature (design rung 3): an L prism's top rim and the rim of the inner wall
/// that runs from the L's inner corner, so the corner at the top of that inner vertical edge is
/// MIXED - the top rim's two edges convex, the vertical edge concave, the top face's angle there
/// 270 degrees.
#[test]
fn an_l_blocks_mixed_corner_fills_closed_as_one_shell() {
    for (size, profile) in [(0.2, BandProfile::Chamfer), (0.2, BandProfile::Fillet)] {
        let (positions, indices, pick) = l_block();
        let before = signed_volume_of(&positions, &indices);
        let gap = build(
            &positions,
            &indices,
            &pick,
            &[false, false, false],
            size,
            profile,
        );
        println!(
            "L {profile:?} {size}: band {:?}, room {}",
            gap.refusal, gap.room
        );
        assert!(gap.built());
        assert_every_rail_ends_at_a_tie(&gap);

        let filled = fill(&gap, profile);
        println!(
            "L {profile:?} {size}: fill {:?}, chain {}, vertex {}",
            filled.refusal, filled.offending_chain, filled.offending_vertex
        );
        assert_eq!(filled.refusal, GapFillRefusal::None);

        let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
        let kept = sweep_mesh_measures::kept(&mesh);
        let volume = signed_volume(&mesh);
        let outside = filled
            .added_vertices
            .iter()
            .filter(|&&v| winding_number(&positions, &indices, mesh.position(v)) < 0.45)
            .count();
        println!(
            "L {profile:?} {size}: volume {volume:?} of {before:?}, {outside} of {} added vertices outside",
            filled.added_vertices.len()
        );
        assert!(sweep_mesh_measures::is_closed_and_consistent(
            mesh.vertices(),
            &kept
        ));
        assert_eq!(
            sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
            1
        );

        // The convex rim takes more than the one concave edge adds.
        assert!(volume < before);
    }
}

/// The L block's pick takes material everywhere but beside its one concave edge (the inner
/// vertical edge at (2, 2)): no added vertex stands outside the part further than 1.5 sizes from
/// that edge. Its convex corner at (2, 6, 4) once carried a fillet row 0.19 proud - the tie there
/// was never inserted (see CornerTiePoints.TryMitre's remarks) and the cut chorded from
/// (1.8, 5.7937, 4) to (1, 5.8, 4).
#[test]
fn an_l_blocks_chamfer_adds_material_only_beside_its_concave_edge() {
    adds_material_only_beside_the_concave_edge(0.2, BandProfile::Chamfer);
}

/// The fillet of `an_l_blocks_chamfer_adds_material_only_beside_its_concave_edge`.
#[test]
fn an_l_blocks_fillet_adds_material_only_beside_its_concave_edge() {
    adds_material_only_beside_the_concave_edge(0.2, BandProfile::Fillet);
}

fn adds_material_only_beside_the_concave_edge(size: f64, profile: BandProfile) {
    let (positions, indices, pick) = l_block();
    let gap = build(
        &positions,
        &indices,
        &pick,
        &[false, false, false],
        size,
        profile,
    );
    assert!(gap.built());
    let filled = fill(&gap, profile);
    assert_eq!(filled.refusal, GapFillRefusal::None);

    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    let concave_top = Vector3::new(2.0, 2.0, 4.0);
    let concave_foot = Vector3::new(2.0, 2.0, 0.0);
    let from_the_concave_edge = |p: Vector3| {
        let axis = concave_foot - concave_top;
        let t = ((p - concave_top).dot(axis) / axis.length_squared()).clamp(0.0, 1.0);
        (p - (concave_top + axis * t)).length()
    };

    let stray: Vec<Vector3> = filled
        .added_vertices
        .iter()
        .map(|&v| mesh.position(v))
        .filter(|&p| {
            winding_number(&positions, &indices, p) < 0.45
                && distance_to_surface(&positions, &indices, p) > 1e-9
                && from_the_concave_edge(p) > size * 1.5
        })
        .collect();
    for &p in stray.iter().take(30) {
        println!(
            "    stray {p:?}, {:.3} from the concave edge, {:.4} outside",
            from_the_concave_edge(p),
            distance_to_surface(&positions, &indices, p)
        );
    }

    assert_eq!(stray.len(), 0);
}

/// Beside the L block's concave edge material is added on purpose, but no more than the profile
/// adds there: at a concave edge whose faces open by the angle 2h a fillet of setback s stands at
/// most s sin h (1 - sin h) / cos h off the nearer face (0.293 s at 90 degrees), a chamfer
/// s sin h cos h (0.5 s). The mixed corner's cap once put its centre 0.158 out at size 0.2 -
/// floating in the notch, beyond the arc.
#[test]
fn an_l_blocks_added_material_stays_within_the_concave_profile() {
    for (size, profile) in [(0.2, BandProfile::Chamfer), (0.2, BandProfile::Fillet)] {
        let (positions, indices, pick) = l_block();
        let gap = build(
            &positions,
            &indices,
            &pick,
            &[false, false, false],
            size,
            profile,
        );
        let filled = fill(&gap, profile);
        assert_eq!(filled.refusal, GapFillRefusal::None);

        let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
        let bound = concave_bound(profile, size, std::f64::consts::PI / 4.0);
        // The first of the largest, or a zero reading when nothing stands outside.
        let mut worst: (Vector3, f64) = (Vector3::ZERO, 0.0);
        let mut first = true;
        for &v in &filled.added_vertices {
            let p = mesh.position(v);
            if winding_number(&positions, &indices, p) >= 0.45 {
                continue;
            }

            let out = distance_to_surface(&positions, &indices, p);
            if first || out > worst.1 {
                worst = (p, out);
                first = false;
            }
        }

        println!(
            "L {profile:?} {size}: worst outside {:.5} at {:?}, bound {bound:.5}",
            worst.1, worst.0
        );
        assert!(worst.1 <= bound * (1.0 + 1e-6));
    }
}

/// How far off the nearer face a profile of setback `size` stands at a concave
/// edge whose faces open by 2 `half_opening`: the arc's (or the chord's) point on the bisector.
pub fn concave_bound(profile: BandProfile, size: f64, half_opening: f64) -> f64 {
    let sin = half_opening.sin();
    let cos = half_opening.cos();
    if profile == BandProfile::Fillet {
        size * sin * (1.0 - sin) / cos
    } else {
        size * sin * cos
    }
}

/// The L prism (6 x 6, arms 2 wide, 4 tall) and its pick: the top face's inner edge (2, 2)-(2, 6),
/// the rest of the top rim, and the inner wall's rim down, across the bottom and up the concave
/// edge.
fn l_block() -> SolidWithChains {
    let outline = [
        Vector2::new(0.0, 0.0),
        Vector2::new(6.0, 0.0),
        Vector2::new(6.0, 2.0),
        Vector2::new(2.0, 2.0),
        Vector2::new(2.0, 6.0),
        Vector2::new(0.0, 6.0),
    ];
    let (positions, indices, _) = prism_on_outline(&outline, Vector2::new(1.0, 1.0), 4.0);
    let pick = vec![vec![3, 4], vec![4, 5, 0, 1, 2, 3], vec![4, 10, 9, 3]];
    (positions, indices, pick)
}
