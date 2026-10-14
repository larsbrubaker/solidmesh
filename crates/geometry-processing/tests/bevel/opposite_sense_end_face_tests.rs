//! Flow rung 1 - a CONCAVE chain whose end face meets the chain's sides at CONVEX creases (an L
//! block's inner vertical edge, ending on the L-shaped top and bottom). The end face is an
//! OPPOSITE-SENSE neighbour: it grows flat over the bead's section, is not cut, does not bound the
//! room, and each end cap lies in its plane. Parameter rows are a loop.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::gap_fill;
use geometry_processing::bevel::level_line::seam_edges::FeatureEdgeTest;
use geometry_processing::bevel::level_line::{
    BandGapRefusal, BandGapResult, BandProfile, GapFillRefusal, GapFillResult,
};
use vector_math::{Matrix4X4, Vector2, Vector3};

use crate::end_face_builds::build_chains;
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_fixture::LevelLineFixture;
use crate::level_line_test_geometry::{signed_volume, signed_volume_of};
use crate::sweep_mesh_measures;

const ARM: f64 = 4.0;

const WALL: f64 = 16.0;

const HEIGHT: f64 = 10.0;

const SEGMENTS: i32 = 8;

/// The chord bound is the polygon through n EQUAL steps of angle; the level line's arc points are
/// at equal steps of the rational quadratic's parameter, which cut a little more off the arc (0.6%
/// of the gap between the chords and the exact bead). 1% allows that and nothing a real defect
/// could hide in.
const ARC_SPACING_ALLOWANCE: f64 = 0.01;

// The L in plan, counter-clockwise: arms 4 wide, inner walls 16 long, the inner corner (plan
// index 3) at (4, 4).
fn plan() -> [Vector2; 6] {
    [
        Vector2::new(0.0, 0.0),
        Vector2::new(ARM + WALL, 0.0),
        Vector2::new(ARM + WALL, ARM),
        Vector2::new(ARM, ARM),
        Vector2::new(ARM, ARM + WALL),
        Vector2::new(0.0, ARM + WALL),
    ]
}

fn chain() -> Vec<i32> {
    vec![3, 3 + plan().len() as i32]
}

fn indices() -> Vec<(i32, i32, i32)> {
    let n = plan().len() as i32;
    let mut indices = Vec::new();
    for i in 1..n - 1 {
        // The fan from the outer corner (0, 0) covers the L: every fan triangle is
        // counter-clockwise.
        indices.push((n, n + i, n + i + 1));
        indices.push((0, i + 1, i));
    }

    for i in 0..n {
        let j = (i + 1) % n;
        indices.push((i, j, n + j));
        indices.push((i, n + j, n + i));
    }

    indices
}

fn prism_on(plan: &[Vector2]) -> Vec<Vector3> {
    plan.iter()
        .map(|p| Vector3::new(p.x, p.y, 0.0))
        .chain(plan.iter().map(|p| Vector3::new(p.x, p.y, HEIGHT)))
        .collect()
}

fn positions(motion: Option<Matrix4X4>) -> Vec<Vector3> {
    let positions = prism_on(&plan());
    match motion {
        Some(motion) => positions.into_iter().map(|p| p.transform(motion)).collect(),
        None => positions,
    }
}

fn build_on(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    feature: FeatureEdgeTest,
    size: f64,
) -> BandGapResult {
    band_gap_entry_points::build(
        positions,
        indices,
        chain,
        false,
        Some(feature),
        size,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
    )
    .expect("a valid pick")
}

fn fill(size: f64, motion: Option<Matrix4X4>) -> GapFillResult {
    let positions = positions(motion);
    let indices = indices();
    let feature = feature_edges_over(&positions, &indices, 20.0);
    let gap = build_on(&positions, &indices, &chain(), feature, size);
    assert!(
        gap.built(),
        "the L's inner edge refused at {size}: {:?}, room {}",
        gap.refusal,
        gap.room
    );
    gap_fill::fill(&gap, BandProfile::Fillet, SEGMENTS).expect("the fill runs")
}

fn chord_and_exact(size: f64) -> (f64, f64) {
    let n = SEGMENTS as f64;
    let chord = HEIGHT * size * size * (1.0 - (n / 2.0 * (std::f64::consts::PI / (2.0 * n)).sin()));
    let exact = HEIGHT * size * size * (1.0 - (std::f64::consts::PI / 4.0));
    (chord, exact)
}

/// The L's inner vertical edge at a fillet of 8: above the arm width (4), below the inner walls
/// (16). The bead added lies between the exact fillet and the chord polygon, below the chamfer.
#[test]
fn an_l_blocks_inner_edge_fillets_past_its_arm_width_with_the_end_faces_growing_flat() {
    const SIZE: f64 = 8.0;
    let filled = fill(SIZE, None);
    assert_eq!(
        filled.refusal,
        GapFillRefusal::None,
        "{} {:?}",
        filled.offending_vertex,
        filled.open_edge
    );

    let mesh = filled.mesh.as_ref().expect("the filled mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        mesh.vertices(),
        &kept
    ));
    assert_eq!(
        sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
        1
    );

    let added = signed_volume(&mesh) - signed_volume_of(&positions(None), &indices());
    let r = SIZE;
    let (chord, exact) = chord_and_exact(r);
    let chamfer = HEIGHT * r * r / 2.0;
    println!(
        "L inner edge at {SIZE}: added {added}, exact {exact}, chord {chord}, chamfer {chamfer}"
    );
    assert!(added > exact, "{added}");
    assert!(
        added <= chord + (ARC_SPACING_ALLOWANCE * (chord - exact)),
        "{added}"
    );
    assert!(added < chamfer, "{added}");

    // Every added triangle facing along z is a cap, and lies ON its end face z = 0 or z = Height,
    // wound out.
    let mut residual: f64 = 0.0;
    let mut caps = 0;
    for &t in &filled.added_triangles {
        let face = mesh.triangles()[t as usize];
        let corners: Vec<Vector3> = [face.a, face.b, face.c]
            .iter()
            .map(|&v| mesh.position(v))
            .collect();
        let normal = (corners[1] - corners[0])
            .cross(corners[2] - corners[0])
            .get_normal();
        if (normal.z.abs() - 1.0).abs() > 1e-6 {
            continue;
        }

        caps += 1;
        let end = if corners[0].z < HEIGHT / 2.0 {
            0.0
        } else {
            HEIGHT
        };
        residual = residual.max(
            corners
                .iter()
                .map(|c| (c.z - end).abs())
                .fold(f64::NEG_INFINITY, f64::max),
        );
        assert!(
            normal.z * (if end == 0.0 { -1.0 } else { 1.0 }) > 0.0,
            "cap {t} at z = {end} faces {normal:?}"
        );
    }

    println!("caps {caps}, residual {residual}");
    assert!(caps > 0);
    assert!(residual <= 1e-9, "{residual}");

    let motion = Matrix4X4::create_rotation(Vector3::new(0.3, -0.7, 1.1))
        * Matrix4X4::create_translation_vector(Vector3::new(11.5, -3.25, 7.125));
    let moved = fill(SIZE, Some(motion));
    assert_eq!(
        moved.refusal,
        GapFillRefusal::None,
        "moved: {} {:?}",
        moved.offending_vertex,
        moved.open_edge
    );
    let moved_added = signed_volume(&moved.mesh.as_ref().expect("the moved mesh").borrow())
        - signed_volume_of(&positions(Some(motion)), &indices());
    assert!(
        (moved_added - added).abs() / added <= 1e-9,
        "moved {moved_added} against {added}"
    );
}

/// A growing end face does not bound the room, but its crease stretch does: a size past the inner
/// wall's 16 refuses by size, naming 16 as the room, and 0.9 of that room builds.
#[test]
fn past_the_inner_wall_the_l_refuses_by_size_and_builds_within_its_room() {
    let positions = positions(None);
    let indices = indices();
    let feature = feature_edges_over(&positions, &indices, 20.0);
    let past = build_on(&positions, &indices, &chain(), feature, 20.0);
    println!("L inner edge at 20: {:?}, room {}", past.refusal, past.room);
    assert_eq!(past.refusal, BandGapRefusal::Size);
    assert!((past.room - WALL).abs() <= 1e-6, "{}", past.room);

    let filled = fill(0.9 * past.room, None);
    assert_eq!(
        filled.refusal,
        GapFillRefusal::None,
        "{} {:?}",
        filled.offending_vertex,
        filled.open_edge
    );
    let mesh = filled.mesh.as_ref().expect("the filled mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        mesh.vertices(),
        &kept
    ));
}

/// The bead is the same shape at every size: on an L of 10 mm arms a fillet of n chords adds
/// between the exact bead and the chord polygon per unit length, whatever r is.
#[test]
fn the_bead_on_the_ladders_l_is_between_the_exact_fillet_and_its_chords_at_every_size() {
    for size in [1.0, 3.0, 8.0] {
        const ARM10: f64 = 10.0;
        let plan = [
            Vector2::new(0.0, 0.0),
            Vector2::new(2.0 * ARM10, 0.0),
            Vector2::new(2.0 * ARM10, ARM10),
            Vector2::new(ARM10, ARM10),
            Vector2::new(ARM10, 2.0 * ARM10),
            Vector2::new(0.0, 2.0 * ARM10),
        ];
        let positions = prism_on(&plan);
        let indices = indices();
        let feature = feature_edges_over(&positions, &indices, 20.0);
        let gap = build_on(&positions, &indices, &chain(), feature, size);
        assert!(gap.built(), "{:?}, room {}", gap.refusal, gap.room);
        let filled = gap_fill::fill(&gap, BandProfile::Fillet, SEGMENTS).expect("the fill runs");
        assert_eq!(filled.refusal, GapFillRefusal::None);

        let added = signed_volume(&filled.mesh.as_ref().expect("the mesh").borrow())
            - signed_volume_of(&positions, &indices);
        let (chord, exact) = chord_and_exact(size);
        println!("ladder L at {size}: added {added}, exact {exact}, chord {chord}");
        assert!(added >= exact * (1.0 - 1e-9), "at {size}: {added}");
        assert!(
            added <= chord + (ARC_SPACING_ALLOWANCE * (chord - exact)),
            "at {size}: {added}"
        );
    }
}

/// Bottom Center Edge: a concave crease of a knife-edge nose in its chevron notch, both ends cut on
/// OBLIQUE end faces; with no bend threshold the fill refuses by name.
#[test]
fn a_concave_crease_cut_on_oblique_end_faces_is_refused_by_name() {
    let fixture = Rc::new(LevelLineFixture::load("bottom-center-edge-op0"));
    let gap = build_chains(
        &fixture,
        &[0],
        fixture.size,
        &BandGapBuildOptions {
            chord_tolerance: 0.0,
            snap: 1e-9,
            ..BandGapBuildOptions::default()
        },
    );
    assert!(gap.built(), "{:?}, room {}", gap.refusal, gap.room);

    let filled = gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).expect("the fill");
    assert_eq!(filled.refusal, GapFillRefusal::OpenChainEnds);
}

/// A faceted boss standing on a block's front face: the concave crease where the barrel meets the
/// front face ends on the flat top and bottom at convex creases, so the end faces grow; the rest of
/// the rim must not name the room.
#[test]
fn a_boss_on_a_block_fillets_its_foot_crease_past_one_facet_of_its_rim() {
    const FACETS: i32 = 24;
    const BOSS: f64 = 5.0;
    let mut plan = vec![
        Vector2::new(-20.0, 20.0),
        Vector2::new(-20.0, 0.0),
        Vector2::new(-BOSS, 0.0),
    ];
    for i in 1..FACETS {
        let angle = std::f64::consts::PI + (std::f64::consts::PI * i as f64 / FACETS as f64);
        plan.push(Vector2::new(BOSS * angle.cos(), BOSS * angle.sin()));
    }

    plan.push(Vector2::new(BOSS, 0.0));
    plan.push(Vector2::new(20.0, 0.0));
    plan.push(Vector2::new(20.0, 20.0));

    // The caps: the half disc fanned from the boss's centre, the block fanned from its far corner
    // through that centre, so no cap triangle is a sliver.
    let n = plan.len() as i32;
    let centre = 2 * n;
    let first_arc = 2;
    let last_arc = n - 3;
    let mut cap_bottom = Vec::new();
    for i in first_arc..last_arc {
        cap_bottom.push((centre, i, i + 1));
    }

    cap_bottom.push((0, 1, 2));
    cap_bottom.push((0, 2, centre));
    cap_bottom.push((0, centre, last_arc));
    cap_bottom.push((0, last_arc, n - 2));
    cap_bottom.push((0, n - 2, n - 1));

    let up = |v: i32| if v == centre { centre + 1 } else { v + n };
    let mut indices = Vec::new();
    for &(a, b, c) in &cap_bottom {
        // Bottom faces down (clockwise from above), the top up.
        indices.push((a, c, b));
        indices.push((up(a), up(b), up(c)));
    }

    for i in 0..n {
        let j = (i + 1) % n;
        indices.push((i, j, n + j));
        indices.push((i, n + j, n + i));
    }

    let mut positions = prism_on(&plan);
    positions.push(Vector3::new(0.0, 0.0, 0.0));
    positions.push(Vector3::new(0.0, 0.0, HEIGHT));
    let chain = [2, 2 + n];
    let feature = feature_edges_over(&positions, &indices, 10.0);
    let source = signed_volume_of(&positions, &indices);
    assert!(source > 0.0);

    // The end face is square to the crease, so the kept edge on it runs S -> V -> T through the end
    // vertex itself; the end zip reads V at the angle of the run vertex before it.
    let room = build_on(&positions, &indices, &chain, feature.clone(), 3.0).room;
    // Every size either builds right or refuses by name - never a closed part of the wrong shape.
    // At 1.1 of the room it must refuse, and name a room no larger than the size asked.
    for fraction in [0.1, 0.5, 0.9, 1.0, 1.1] {
        let size = room * fraction;
        let gap = build_on(&positions, &indices, &chain, feature.clone(), size);
        let filled = if gap.refusal == BandGapRefusal::None {
            Some(gap_fill::fill(&gap, BandProfile::Fillet, SEGMENTS).expect("the fill runs"))
        } else {
            None
        };
        let refusal = match &filled {
            None => format!("{:?}", gap.refusal),
            Some(filled) => format!("{:?}", filled.refusal),
        };
        println!(
            "boss foot at {fraction} ({size}): {refusal}, room {} set by {:?}",
            gap.room, gap.room_set_by
        );
        if fraction <= 0.5 {
            assert_eq!(
                refusal, "None",
                "room {} set by {:?}",
                gap.room, gap.room_set_by
            );
        }

        if fraction > 1.0 {
            assert_ne!(refusal, "None", "past the room the pick refuses by name");
            assert!(gap.room <= size, "the refusal names the room");
        }

        if refusal != "None" {
            continue;
        }

        let filled = filled.expect("a fill");
        let mesh = filled.mesh.as_ref().expect("the filled mesh").borrow();
        let kept = sweep_mesh_measures::kept(&mesh);
        assert!(sweep_mesh_measures::is_closed_and_consistent(
            mesh.vertices(),
            &kept
        ));
        assert_eq!(
            sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
            1
        );
        assert!(sweep_mesh_measures::smallest_area(mesh.vertices(), &kept) > 0.0);
        let after = signed_volume(&mesh);
        let added = after - source;
        println!("boss foot at {fraction} ({size}): added {added}");
        assert!(after > 0.0, "the result is wound the way the input was");
        assert!(added > 0.0, "a concave crease's fillet adds material");
    }
}

/// The owner fixture: Simple Internal Fillet at its saved 8 (analytic 275, 8-chord ~281).
#[test]
fn simple_internal_fillet_builds_at_its_saved_size() {
    let fixture = Rc::new(LevelLineFixture::load("simple-internal-fillet-op0"));
    let gap = build_chains(
        &fixture,
        &[0],
        fixture.size,
        &BandGapBuildOptions {
            chord_tolerance: 0.0,
            snap: 1e-9,
            ..BandGapBuildOptions::default()
        },
    );
    assert_eq!(gap.refusal, BandGapRefusal::None, "room {}", gap.room);

    let filled = gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).expect("the fill");
    assert_eq!(
        filled.refusal,
        GapFillRefusal::None,
        "{} {:?}",
        filled.offending_vertex,
        filled.open_edge
    );
    let mesh = filled.mesh.as_ref().expect("the filled mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        mesh.vertices(),
        &kept
    ));
    assert_eq!(
        sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
        1
    );
    let added = signed_volume(&mesh) - signed_volume_of(&fixture.positions, &fixture.indices);
    println!("Simple Internal Fillet at {}: added {added}", fixture.size);
    assert!(added > 200.0, "{added}");
    assert!(added < 400.0, "{added}");
}
