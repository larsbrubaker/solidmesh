//! Port of `LevelLine/ObliqueEndFaceTests.cs`: flow rung 3 - an open chain whose end faces are
//! OBLIQUE to it. A triangular prism's top edge (a right angle, sides at 45 degrees), sheared so
//! both end caps lean 30 degrees off square. The end ray runs the rails straight past the chain's
//! end to the end plane, and the end plane clips the strip. C# `[Arguments]` rows are loops.
//!
//! C#'s `ASheardPrismsTopEdgeFilletsStraightToItsObliqueEndFaces` and its `...NearTheRoom` rows are
//! `[Skip]` in C# ("flow rung 3 open") and are not ported.

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::{gap_fill, virtual_rail_past_the_end};
use geometry_processing::bevel::level_line::{
    BandGapRefusal, BandGapResult, BandProfile, GapFillRefusal, GapFillResult,
};
use vector_math::{Matrix4X4, Vector2, Vector3};

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::{signed_volume, signed_volume_of};
use crate::sweep_mesh_measures;

const HALF: f64 = 5.0;

const LENGTH: f64 = 20.0;

const SEGMENTS: i32 = 8;

/// The operation's bend threshold, as the app passes it: what says a crease carries on past an end.
const BEND_DEGREES: f64 = 20.0;

fn lean() -> f64 {
    (std::f64::consts::PI / 6.0).tan()
}

// The section, counter-clockwise seen from +x: the base corners and the apex (index 2).
fn section() -> [Vector2; 3] {
    [
        Vector2::new(-HALF, 0.0),
        Vector2::new(HALF, 0.0),
        Vector2::new(0.0, HALF),
    ]
}

pub(crate) fn chain() -> Vec<i32> {
    vec![2, 5]
}

pub(crate) fn indices() -> Vec<(i32, i32, i32)> {
    let mut indices = vec![(0, 2, 1), (3, 4, 5)];
    for i in 0..3 {
        let j = (i + 1) % 3;
        indices.push((i, j, 3 + j));
        indices.push((i, 3 + j, 3 + i));
    }

    indices
}

pub(crate) fn positions(motion: Option<Matrix4X4>) -> Vec<Vector3> {
    let positions: Vec<Vector3> = section()
        .iter()
        .map(|p| Vector3::new(lean() * p.y, p.x, p.y))
        .chain(
            section()
                .iter()
                .map(|p| Vector3::new(LENGTH + (lean() * p.y), p.x, p.y)),
        )
        .collect();
    match motion {
        Some(motion) => positions.into_iter().map(|p| p.transform(motion)).collect(),
        None => positions,
    }
}

fn build_on(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    size: f64,
) -> BandGapResult {
    band_gap_entry_points::build(
        positions,
        indices,
        chain,
        false,
        Some(feature_edges_over(positions, indices, 20.0)),
        size,
        BandProfile::Fillet,
        &BandGapBuildOptions {
            sharp_turn_degrees: BEND_DEGREES,
            ..BandGapBuildOptions::default()
        },
    )
    .expect("a valid pick")
}

fn build(size: f64) -> BandGapResult {
    build_on(&positions(None), &indices(), &chain(), size)
}

pub(crate) fn room_of() -> f64 {
    let gap = build(100.0);
    if gap.refusal == BandGapRefusal::Size {
        gap.room
    } else {
        -1.0
    }
}

fn fill(size: f64, motion: Option<Matrix4X4>) -> GapFillResult {
    let gap = build_on(&positions(motion), &indices(), &chain(), size);
    assert!(
        gap.built(),
        "the prism's top edge refused at {size}: {:?}, room {}",
        gap.refusal,
        gap.room
    );
    gap_fill::fill(&gap, BandProfile::Fillet, SEGMENTS).expect("the fill runs")
}

fn motion() -> Matrix4X4 {
    Matrix4X4::create_rotation(Vector3::new(0.3, -0.7, 1.1))
        * Matrix4X4::create_translation_vector(Vector3::new(11.5, -3.25, 7.125))
}

fn volume_of(filled: &GapFillResult) -> f64 {
    signed_volume(&filled.mesh.as_ref().expect("the filled mesh").borrow())
}

/// Unchanged under a rigid motion: the end cap's zip once broke a tie by 1e-12 radians, and a
/// translation alone laid the other diagonal.
#[test]
fn a_sheard_prisms_top_edge_is_unchanged_under_a_rigid_motion() {
    for fraction in [0.1, 0.5, 1.0] {
        let size = fraction * room_of();
        let volume = volume_of(&fill(size, None));
        let moved = fill(size, Some(motion()));
        assert_eq!(
            moved.refusal,
            GapFillRefusal::None,
            "moved: {} {:?}",
            moved.offending_vertex,
            moved.open_edge
        );
        let moved_volume = volume_of(&moved);
        assert!(
            (moved_volume - volume).abs() / volume <= 1e-9,
            "at {fraction}: moved {moved_volume} against {volume}"
        );
    }
}

/// Every blend triangle not lying in an end plane stands on the round to within the round's chord
/// bound r (1 - cos(pi / 2n)): the strip runs whole up to the end plane.
#[test]
fn a_sheard_prisms_blend_stands_on_the_round_up_to_the_end_planes() {
    for fraction in [0.1, 0.5, 1.0] {
        let size = fraction * room_of();
        let filled = fill(size, None);
        assert_eq!(filled.refusal, GapFillRefusal::None);
        let mesh = filled.mesh.as_ref().expect("the filled mesh").borrow();
        let axis_z = HALF - (size * 2f64.sqrt());
        let chord_bound = size * (1.0 - (std::f64::consts::PI / (2.0 * SEGMENTS as f64)).cos());
        let mut worst = 0.0;
        let mut worst_triangle = String::new();
        for &t in &filled.added_triangles {
            let face = mesh.triangles()[t as usize];
            let corners: Vec<Vector3> = [face.a, face.b, face.c]
                .iter()
                .map(|&v| mesh.position(v))
                .collect();
            let from_end: Vec<f64> = corners
                .iter()
                .map(|c| c.x - (lean() * c.z))
                .map(|q| f64::min(q.abs(), (q - LENGTH).abs()))
                .collect();
            if from_end.iter().all(|&e| e <= 1e-9) {
                continue;
            }

            for c in &corners {
                let off = (((c.y * c.y) + ((c.z - axis_z) * (c.z - axis_z))).sqrt() - size).abs();
                if off > worst {
                    worst = off;
                    worst_triangle = (0..3)
                        .map(|k| {
                            format!(
                                "[{:.4},{:.4},{:.4} end {:.3e}]",
                                corners[k].x, corners[k].y, corners[k].z, from_end[k]
                            )
                        })
                        .collect::<Vec<_>>()
                        .join(" ");
                }
            }
        }

        println!("at {size}: off the round {worst}, chord bound {chord_bound}");
        assert!(
            worst <= chord_bound,
            "off the round {worst} against the chord bound {chord_bound}, in {worst_triangle}"
        );
    }
}

#[test]
fn a_sheard_prisms_top_edge_refuses_by_size_past_its_room() {
    let room = room_of();
    let past = build(1.1 * room);
    assert_eq!(past.refusal, BandGapRefusal::Size);
    assert!(
        (past.room - room).abs() <= 1e-9,
        "{} against {room}",
        past.room
    );
}

/// With the end faces left whole the room is the side face's own width across the rail, 5 sqrt 2;
/// at 0.9 and 1.0 of it the band builds, and every minted rail point past the far end stays in the
/// side's plane short of the base (z above 0).
#[test]
fn a_sheard_prisms_room_is_the_side_faces_width_and_builds() {
    for fraction in [0.9, 1.0] {
        let room = room_of();
        assert!((room - (HALF * 2f64.sqrt())).abs() <= 1e-6, "room {room}");
        let gap = build(fraction * room);
        assert!(gap.built(), "{:?}, room {}", gap.refusal, gap.room);
        let mut minted = Vec::new();
        virtual_rail_past_the_end::extended(
            &gap,
            &gap.loops,
            BandProfile::Fillet,
            &mut minted,
            false,
        )
        .expect("the rails carry");
        let mesh = gap.mesh.as_ref().expect("the mesh").borrow();
        let lowest = minted
            .iter()
            .map(|&v| mesh.position(v).z)
            .fold(None, |low: Option<f64>, z| {
                Some(low.map_or(z, |l| l.min(z)))
            })
            .unwrap_or(f64::NAN);
        println!(
            "at {fraction} of {room}: {} minted, lowest z {lowest}",
            minted.len()
        );
        for &v in &minted {
            assert!(mesh.position(v).z >= -1e-9, "{:?}", mesh.position(v));
        }
    }
}

/// The OPPOSITE sense: an L block's inner (concave) vertical edge whose top leans 30 degrees, so
/// the top meets the inner walls at convex creases and is oblique to the chain. The top GROWS over
/// the bead's section - a cap in the top's own plane - and the bottom grows as rung 1 builds it.
#[test]
fn an_l_blocks_inner_edge_under_a_leaning_top_grows_the_top_in_its_plane() {
    const ARM: f64 = 4.0;
    const WALL: f64 = 16.0;
    const HEIGHT: f64 = 10.0;
    let plan = [
        Vector2::new(0.0, 0.0),
        Vector2::new(ARM + WALL, 0.0),
        Vector2::new(ARM + WALL, ARM),
        Vector2::new(ARM, ARM),
        Vector2::new(ARM, ARM + WALL),
        Vector2::new(0.0, ARM + WALL),
    ];
    let n = plan.len() as i32;
    let lean = lean();
    let positions: Vec<Vector3> = plan
        .iter()
        .map(|p| Vector3::new(p.x, p.y, 0.0))
        .chain(
            plan.iter()
                .map(|p| Vector3::new(p.x, p.y, HEIGHT + (lean * p.x))),
        )
        .collect();
    let mut indices = Vec::new();
    for i in 1..n - 1 {
        indices.push((n, n + i, n + i + 1));
        indices.push((0, i + 1, i));
    }

    for i in 0..n {
        let j = (i + 1) % n;
        indices.push((i, j, n + j));
        indices.push((i, n + j, n + i));
    }

    let chain = [3, 3 + n];
    const SIZE: f64 = 3.0;
    let gap = build_on(&positions, &indices, &chain, SIZE);
    assert!(gap.built(), "{:?}, room {}", gap.refusal, gap.room);
    let filled = gap_fill::fill(&gap, BandProfile::Fillet, SEGMENTS).expect("the fill runs");
    assert_eq!(
        filled.refusal,
        GapFillRefusal::None,
        "at {} {:?}",
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
    let added = signed_volume(&mesh) - signed_volume_of(&positions, &indices);
    assert!(added > 0.0);

    // Every added triangle facing along the top's normal lies in the top's plane.
    let corners_of = |t: i32| -> Vec<Vector3> {
        let face = mesh.triangles()[t as usize];
        [face.a, face.b, face.c]
            .iter()
            .map(|&v| mesh.position(v))
            .collect()
    };
    let top_normal = Vector3::new(-lean, 0.0, 1.0).get_normal();
    let mut residual: f64 = 0.0;
    let mut caps = 0;
    for &t in &filled.added_triangles {
        let corners = corners_of(t);
        let normal = (corners[1] - corners[0])
            .cross(corners[2] - corners[0])
            .get_normal();
        if normal.dot(top_normal) < 1.0 - 1e-6 {
            continue;
        }

        caps += 1;
        residual = residual.max(
            corners
                .iter()
                .map(|c| (c.z - HEIGHT - (lean * c.x)).abs() / (1.0 + (lean * lean)).sqrt())
                .fold(f64::NEG_INFINITY, f64::max),
        );
    }

    println!("leaning L at {SIZE}: added {added}, top caps {caps}, residual {residual}");
    assert!(caps > 0);
    assert!(residual <= 1e-9, "{residual}");

    let moved: Vec<Vector3> = positions.iter().map(|p| p.transform(motion())).collect();
    let moved_gap = build_on(&moved, &indices, &chain, SIZE);
    assert!(moved_gap.built(), "moved: {:?}", moved_gap.refusal);
    let moved_fill =
        gap_fill::fill(&moved_gap, BandProfile::Fillet, SEGMENTS).expect("the fill runs");
    assert_eq!(moved_fill.refusal, GapFillRefusal::None);
    let moved_added = volume_of(&moved_fill) - signed_volume_of(&moved, &indices);
    assert!(
        (moved_added - added).abs() / added <= 1e-9,
        "moved {moved_added} against {added}"
    );

    // The stricter reading, as the prism's: every blend triangle not lying in the top or the
    // bottom stands on the round (axis vertical through (arm + r, arm + r)) within its chord bound.
    let chord_bound = SIZE * (1.0 - (std::f64::consts::PI / (2.0 * SEGMENTS as f64)).cos());
    let mut off_round: f64 = 0.0;
    for &t in &filled.added_triangles {
        let corners = corners_of(t);
        let in_top = corners
            .iter()
            .all(|c| (c.z - HEIGHT - (lean * c.x)).abs() <= 1e-9);
        let in_bottom = corners.iter().all(|c| c.z.abs() <= 1e-9);
        if in_top || in_bottom {
            continue;
        }

        off_round = off_round.max(
            corners
                .iter()
                .map(|c| {
                    (((c.x - ARM - SIZE).powi(2) + (c.y - ARM - SIZE).powi(2)).sqrt() - SIZE).abs()
                })
                .fold(f64::NEG_INFINITY, f64::max),
        );
    }

    println!("leaning L at {SIZE}: off the round {off_round}, chord bound {chord_bound}");
    assert!(
        off_round <= chord_bound,
        "off the round {off_round} against the chord bound {chord_bound}"
    );
}

/// At the far end the end face leans out over the end: the rail is carried on past P in the side's
/// plane to where the row's last point leaves the end plane - one minted point per side, at the
/// setback from the chain's extended line, both at the same parameter.
#[test]
fn a_sheard_prisms_rails_run_on_past_the_far_end_until_the_row_leaves_the_end_plane() {
    for fraction in [0.1, 0.5, 1.0] {
        let size = fraction * room_of();
        let gap = build(size);
        assert!(gap.built(), "{:?}, room {}", gap.refusal, gap.room);
        let mut minted = Vec::new();
        let loops = virtual_rail_past_the_end::extended(
            &gap,
            &gap.loops,
            BandProfile::Fillet,
            &mut minted,
            false,
        )
        .expect("the rails carry");
        assert_eq!(minted.len(), 2);

        // The chain runs along x at (y, z) = (0, Half); the far end plane is x - Lean z = Length.
        let mesh = gap.mesh.as_ref().expect("the mesh").borrow();
        let r = size;
        let axis_z = HALF - (r * 2f64.sqrt());
        for &v in &minted {
            let q = mesh.position(v);
            let from_line = ((q.y * q.y) + ((q.z - HALF) * (q.z - HALF))).sqrt();
            assert!(
                (from_line - r).abs() <= 1e-9,
                "minted {q:?} stands {from_line} off the line"
            );
            assert!(
                q.x - (lean() * q.z) > LENGTH,
                "minted {q:?} is past the end plane"
            );
        }

        let a = mesh.position(minted[0]);
        let b = mesh.position(minted[1]);
        assert!((a.x - b.x).abs() <= 1e-12);

        // The row at the minted points is the arc of radius r about (x, 0, axisZ); its top is its
        // last point to leave the end plane: it stands ON the plane.
        let top_off = a.x - (lean() * (axis_z + r)) - LENGTH;
        println!(
            "at {size}: minted at x {}, top off the end plane {top_off}",
            a.x
        );
        assert!(top_off.abs() <= 1e-9, "{top_off}");

        // Each run's vertex beside the minted one is P, on the rail: no crease stretch above P is
        // left.
        for gap_loop in loops.iter().filter(|l| l.side == 1 || l.side == 2) {
            let m = *minted
                .iter()
                .find(|v| gap_loop.vertices.contains(v))
                .expect("a minted vertex on the run");
            let i = gap_loop
                .vertices
                .iter()
                .position(|&v| v == m)
                .expect("on the run");
            let neighbour = mesh.position(gap_loop.vertices[if i == 0 { 1 } else { i - 1 }]);
            let from_line = ((neighbour.y * neighbour.y)
                + ((neighbour.z - HALF) * (neighbour.z - HALF)))
                .sqrt();
            assert!(
                (from_line - r).abs() <= 1e-8,
                "P at {neighbour:?} stands {from_line} off the line"
            );
        }
    }
}
