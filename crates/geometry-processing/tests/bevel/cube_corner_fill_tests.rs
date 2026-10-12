//! Port of `LevelLine/CubeCornerFillTests.cs`: the first corner case of the level line, the fill
//! half (the strips and the corner hole, see `CornerFill`) - two adjacent faces of a cube (edge 2)
//! picked as three open chains meeting at two corners, each chain zipped between its two rail
//! pieces and each corner's hole closed. C# `[Arguments]` cases run as one loop per test.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::band_gap_reading::read_room_of_pick;
use geometry_processing::bevel::level_line::corner_cap_hole_fill_switch::use_hole_fill;
use geometry_processing::bevel::level_line::{
    corner_fill, gap_fill, BandGapRefusal, BandGapResult, BandProfile, GapFillRefusal,
    GapFillResult, LevelLineMesh, WorkBudget,
};
use vector_math::{Matrix4X4, Vector3};

use crate::hole_fill_corner_tests::{
    build, every_edge, room_of, sign, smallest_kept_area, two_faces,
};
use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::{cube_solid, signed_volume};
use crate::several_chains_fixtures::{self, FOLD_DEGREES};
use crate::sweep_mesh_measures;

const ALL_OPEN: [bool; 3] = [false, false, false];

/// The two faces' three chains, 2 + 6 + 6 long - what the prism-sum bound multiplies.
const PICKED_LENGTH: f64 = 14.0;

const CUBE_VOLUME: f64 = 8.0;

// Fixture A's chains have no sharp joint (no turn threshold), so its chamfer reads every rail
// vertex at its own parameter and fills exactly as before.
const FIXTURE_A_DIGEST: &str = "013765d38bb29bf9";

const EVERY_EDGE_ROOM: f64 = 1.0;

/// C#'s motion: turned and carried off the origin.
fn motion() -> Matrix4X4 {
    Matrix4X4::create_rotation(Vector3::new(0.3, -0.7, 1.1))
        * Matrix4X4::create_translation(11.5, -3.25, 7.125)
}

/// The largest coordinate magnitude, which a fill that only removes keeps within the cube's 1.
fn largest_coordinate(p: Vector3) -> f64 {
    p.x.abs().max(p.y.abs().max(p.z.abs()))
}

/// C# `TwoAdjacentFacesChamferToAClosedSolidInsideTheCube`.
#[test]
fn two_adjacent_faces_chamfer_to_a_closed_solid_inside_the_cube() {
    for size in [0.2, 0.1, 0.5, 0.9] {
        let filled = fill_cube(size, None, BandProfile::Chamfer);
        assert_eq!(filled.refusal, GapFillRefusal::None);

        let mesh = filled.mesh.as_ref().unwrap().borrow();
        let kept = sweep_mesh_measures::kept(&mesh);
        assert!(sweep_mesh_measures::is_closed_and_consistent(
            mesh.vertices(),
            &kept
        ));
        assert_eq!(
            sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
            1
        );

        let smallest_area = smallest_kept_area(&mesh, &kept);
        assert!(smallest_area > 1e-12);

        // A chamfer of setback s on a right-angled edge takes a prism of s^2 / 2 per unit length;
        // round a corner the prisms of its chains overlap, so the sum over-counts, and each of the
        // two corners' caps is allowed at most a cube of s on top.
        let volume = signed_volume(&mesh);
        let removed = CUBE_VOLUME - volume;
        let bound = (PICKED_LENGTH * size * size / 2.0) + (2.0 * size * size * size);
        println!(
            "size {size}: volume {volume}, removed {removed}, bound {bound}, smallest area {smallest_area}, {} fill triangles",
            filled.added_triangles.len()
        );
        assert!(removed > 0.0);
        assert!(removed <= bound);

        // The cube is convex, so a fill that only removes material leaves every new vertex inside
        // or on it.
        for &v in &filled.added_vertices {
            assert!(largest_coordinate(mesh.position(v)) <= 1.0 + 1e-9);
        }

        let moved = fill_cube(size, Some(motion()), BandProfile::Chamfer);
        assert_eq!(moved.refusal, GapFillRefusal::None);
        let moved_volume = signed_volume(&moved.mesh.as_ref().unwrap().borrow());
        println!(
            "size {size}: moved volume {moved_volume}, relative difference {}",
            (moved_volume - volume).abs() / volume
        );
        assert!((moved_volume - volume).abs() / volume <= 1e-9);
    }
}

/// C# `APickWithoutCornersFillsBitIdentically`: a pick whose chains touch nowhere fills through the
/// per-chain zip exactly as before - pinned by the digest of fixture A's chamfer.
#[test]
fn a_pick_without_corners_fills_bit_identically() {
    let (positions, indices, chains) = several_chains_fixtures::named("A");
    let closed = vec![true; chains.len()];
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    let room = room_of(
        &positions,
        &indices,
        &chains,
        &closed,
        &feature,
        BandProfile::Chamfer,
    );
    let options = BandGapBuildOptions {
        budget: Some(Rc::new(
            WorkBudget::new(50 * WorkBudget::default_limit(indices.len() as i32))
                .expect("a positive limit"),
        )),
        ..BandGapBuildOptions::default()
    };
    let gap = band_gap_entry_points::build_pick(
        &positions,
        &indices,
        &chains,
        &closed,
        Some(feature),
        room / 2.0,
        BandProfile::Chamfer,
        &options,
    )
    .expect("the pick builds or refuses by name");
    let filled = gap_fill::fill(
        &gap,
        BandProfile::Chamfer,
        gap_fill::DEFAULT_FILLET_SEGMENTS,
    )
    .expect("the fill");
    assert_eq!(filled.refusal, GapFillRefusal::None);

    let digest = digest(&filled.mesh.as_ref().unwrap().borrow());
    println!("fixture A chamfer digest {digest}");
    assert_eq!(digest, FIXTURE_A_DIGEST);
}

/// C# `EveryEdgeOfTheCubeChamfersClosed`: every edge of the cube as its own chain, twelve chains
/// meeting three at a time at eight corners.
#[test]
fn every_edge_of_the_cube_chamfers_closed() {
    for fraction_of_room in [0.2, 0.1, 0.5, 0.9] {
        let size = fraction_of_room * EVERY_EDGE_ROOM;
        let filled = fill_every_edge(size, None);
        println!(
            "every edge at {size}: fill {:?}, chain {}, vertex {}",
            filled.refusal, filled.offending_chain, filled.offending_vertex
        );
        assert_eq!(filled.refusal, GapFillRefusal::None);

        let mesh = filled.mesh.as_ref().unwrap().borrow();
        let kept = sweep_mesh_measures::kept(&mesh);
        assert!(sweep_mesh_measures::is_closed_and_consistent(
            mesh.vertices(),
            &kept
        ));
        assert_eq!(
            sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
            1
        );

        // Twelve prisms of s^2 / 2 per unit length over edges of length 2, overlapping at the
        // corners, so their sum bounds what the chamfer removes.
        let volume = signed_volume(&mesh);
        let removed = CUBE_VOLUME - volume;
        let bound = 12.0 * 2.0 * size * size / 2.0;
        println!("every edge at {size}: volume {volume}, removed {removed}, bound {bound}");
        assert!(volume < CUBE_VOLUME);
        assert!(volume > CUBE_VOLUME - bound);

        for &v in &filled.added_vertices {
            assert!(largest_coordinate(mesh.position(v)) <= 1.0 + 1e-9);
        }

        let moved = fill_every_edge(size, Some(motion()));
        assert_eq!(moved.refusal, GapFillRefusal::None);
        let moved_volume = signed_volume(&moved.mesh.as_ref().unwrap().borrow());
        println!(
            "every edge at {size}: moved volume {moved_volume}, relative difference {}",
            (moved_volume - volume).abs() / volume
        );
        assert!((moved_volume - volume).abs() / volume <= 1e-9);
    }
}

/// C# `EveryEdgeOfTheCubeReadsRoomOne`: a face collapses when the insets of its four edges meet at
/// its centre, size 1 on a cube of edge 2; past it the band refuses by size.
#[test]
fn every_edge_of_the_cube_reads_room_one() {
    let chains = every_edge();
    let open = vec![false; chains.len()];
    let (positions, indices) = cube_solid(1.0);
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    let reading = read_room_of_pick(
        &positions,
        &indices,
        &chains,
        &open,
        Some(Rc::clone(&feature)),
        BandProfile::Chamfer,
        1e-9,
        None,
        f64::NAN,
        0.0,
        f64::INFINITY,
    )
    .expect("a valid pick");
    println!("every edge: room {}", reading.room);
    assert!((reading.room - EVERY_EDGE_ROOM).abs() <= 1e-6);

    let built = build(
        &positions,
        &indices,
        &chains,
        &open,
        &feature,
        1.1 * reading.room,
        BandProfile::Chamfer,
    );
    println!(
        "every edge at {}: build {:?}, room {}",
        1.1 * reading.room,
        built.refusal,
        built.room
    );
    assert_eq!(built.refusal, BandGapRefusal::Size);
}

fn fill_every_edge(size: f64, motion: Option<Matrix4X4>) -> GapFillResult {
    let (mut positions, indices) = cube_solid(1.0);
    if let Some(motion) = motion {
        positions = positions.iter().map(|p| p.transform(motion)).collect();
    }

    let chains = every_edge();
    let open = vec![false; chains.len()];
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    let built = build(
        &positions,
        &indices,
        &chains,
        &open,
        &feature,
        size,
        BandProfile::Chamfer,
    );
    assert!(
        built.built(),
        "every edge refused at {size}: {:?}, room {}",
        built.refusal,
        built.room
    );

    gap_fill::fill(
        &built,
        BandProfile::Chamfer,
        gap_fill::DEFAULT_FILLET_SEGMENTS,
    )
    .expect("the fill")
}

/// C# `TwoAdjacentFacesFilletToAClosedSolidInsideTheCube`: the same two faces FILLETED; at a cube
/// corner with every edge rounded at one radius the true blend is a sphere octant about the
/// corner's inset point, and a fillet removes less than the chamfer of the same size.
#[test]
fn two_adjacent_faces_fillet_to_a_closed_solid_inside_the_cube() {
    const SEGMENTS: i32 = 4;
    for size in [0.1, 0.2, 0.5, 0.9] {
        let filled = fill_cube(size, None, BandProfile::Fillet);
        println!(
            "size {size}: fillet fill {:?}, chain {}, vertex {}, edge {:?}",
            filled.refusal, filled.offending_chain, filled.offending_vertex, filled.open_edge
        );
        assert_eq!(filled.refusal, GapFillRefusal::None);

        let mesh = filled.mesh.as_ref().unwrap().borrow();
        let kept = sweep_mesh_measures::kept(&mesh);
        assert!(sweep_mesh_measures::is_closed_and_consistent(
            mesh.vertices(),
            &kept
        ));
        assert_eq!(
            sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
            1
        );

        let smallest_area = smallest_kept_area(&mesh, &kept);
        assert!(smallest_area > 1e-12);

        let volume = signed_volume(&mesh);
        let chamfer_volume = signed_volume(
            &fill_cube(size, None, BandProfile::Chamfer)
                .mesh
                .unwrap()
                .borrow(),
        );
        println!(
            "size {size}: fillet volume {volume}, chamfer volume {chamfer_volume}, smallest area {smallest_area}, {} fill triangles",
            filled.added_triangles.len()
        );
        assert!(volume < CUBE_VOLUME);
        assert!(volume > chamfer_volume);

        for &v in &filled.added_vertices {
            assert!(largest_coordinate(mesh.position(v)) <= 1.0 + 1e-9);
        }

        // Each corner of this pick has all three of its edges rounded at radius `size`: the rolling
        // ball there is centred at the corner pulled in by `size` along every axis.
        let sag = size * (1.0 - (std::f64::consts::PI / (4.0 * SEGMENTS as f64)).cos());
        let mut deviation: f64 = 0.0;
        let mut cap_vertices = 0;
        for corner in
            corner_fill::corner_points(&fill_cube_gap(size, None, BandProfile::Fillet).chains)
        {
            let outward = Vector3::new(sign(corner.x), sign(corner.y), sign(corner.z));
            let inset = corner - (outward * size);
            for &v in &filled.added_vertices {
                let off = mesh.position(v) - inset;
                if off.x * outward.x < -1e-9
                    || off.y * outward.y < -1e-9
                    || off.z * outward.z < -1e-9
                {
                    continue;
                }

                cap_vertices += 1;
                deviation = deviation.max((off.length() - size).abs());
            }
        }

        println!(
            "size {size}: {cap_vertices} corner vertices, max radial deviation {deviation} = {:.3}% of the radius, chord sag {sag}, hole filler {}",
            100.0 * deviation / size,
            if use_hole_fill() { "on" } else { "off" }
        );
        assert!(cap_vertices > 0);

        // The rings lay the exact sphere, so they are held to chord sag; the hole filler is fair,
        // not an exact rolling ball, and is held to the same 5% of the radius
        // HoleFillCornerTests.TheCubeCornerStaysNearTheRollingBall holds it to.
        if use_hole_fill() {
            assert!(deviation / size <= 0.05);
        } else {
            assert!(deviation <= sag);
        }

        let moved = fill_cube(size, Some(motion()), BandProfile::Fillet);
        assert_eq!(moved.refusal, GapFillRefusal::None);
        let moved_volume = signed_volume(&moved.mesh.as_ref().unwrap().borrow());
        println!(
            "size {size}: moved fillet volume {moved_volume}, relative difference {}",
            (moved_volume - volume).abs() / volume
        );
        assert!((moved_volume - volume).abs() / volume <= 1e-9);
    }
}

fn fill_cube(size: f64, motion: Option<Matrix4X4>, profile: BandProfile) -> GapFillResult {
    gap_fill::fill(
        &fill_cube_gap(size, motion, profile),
        profile,
        gap_fill::DEFAULT_FILLET_SEGMENTS,
    )
    .expect("the fill")
}

fn fill_cube_gap(size: f64, motion: Option<Matrix4X4>, profile: BandProfile) -> BandGapResult {
    let (mut positions, indices) = cube_solid(1.0);
    if let Some(motion) = motion {
        positions = positions.iter().map(|p| p.transform(motion)).collect();
    }

    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    let built = build(
        &positions,
        &indices,
        &two_faces(),
        &ALL_OPEN,
        &feature,
        size,
        profile,
    );
    assert!(
        built.built(),
        "the two faces refused at {size}: {:?}",
        built.refusal
    );

    built
}

/// C# `Digest`: every live triangle's corner positions, bit for bit, in triangle order, folded by
/// FNV-1a.
fn digest(mesh: &LevelLineMesh) -> String {
    let mut h: u64 = 1469598103934665603;
    let mut mix = |d: f64| {
        h ^= d.to_bits();
        h = h.wrapping_mul(1099511628211);
    };

    for t in 0..mesh.triangle_count() {
        if mesh.is_removed(t) {
            continue;
        }

        let tri = mesh.triangles()[t as usize];
        for v in [tri.a, tri.b, tri.c] {
            let p = mesh.position(v);
            mix(p.x);
            mix(p.y);
            mix(p.z);
        }
    }

    format!("{h:016x}")
}
