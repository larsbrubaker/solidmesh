//! Flow rung 2, the strip clipped on a neighbour's plane.
//! A 20 mm box's top front edge (convex, 90 degrees) is filleted past a wedge rib standing on the
//! top face: the rib's tip T (10, 3, 10) is 3 from the edge, its two flank faces meet the top
//! along CONCAVE creases T-P and T-Q (opposite sense to the chain), and its ridge T-R rises steeply
//! away from the edge. Under the flow design the rows are laid whole on the top's plane (a virtual
//! rail past the reached crease), the flanks' planes clip the strip, and since the rib is opposite
//! sense it GROWS: each flank runs on down past its old crease and stops on the fillet, a cap in
//! the flank's plane, the ridge extended to where it pierces the strip. The rib's geometry is in
//! `strip_clip_rib_meshes.rs`.

use std::io::Write;
use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::{gap_fill, strip_clip};
use solidmesh::bevel::level_line::{
    BandGapRefusal, BandGapResult, BandProfile, GapFillRefusal, GapFillResult, LevelLineMesh,
};
use solidmesh_math::{Matrix4X4, Vector3};

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_fixture::LevelLineFixture;
use crate::level_line_test_geometry::{signed_volume, signed_volume_of};
use crate::strip_clip_rib_meshes::{
    flat_rib, flat_rib_positions, plain, positions, ribbed, CHAIN, RIB_FACES,
};
use crate::sweep_mesh_measures;

const SIZE: f64 = 5.0;

const SEGMENTS: i32 = 16;

fn mesh(filled: &GapFillResult) -> std::cell::Ref<'_, LevelLineMesh> {
    filled.mesh.as_ref().expect("the filled mesh").borrow()
}

fn open_edge_at(filled: &GapFillResult) -> String {
    if filled.open_edge.0 < 0 {
        String::new()
    } else {
        let m = mesh(filled);
        format!(
            "{:?} {:?}",
            m.position(filled.open_edge.0),
            m.position(filled.open_edge.1)
        )
    }
}

/// The rib grows onto the fillet at 5, above today's room of 3.
#[test]
fn the_rib_grows_down_onto_the_fillet() {
    let filled = fill(&ribbed(), SIZE, None);
    assert_eq!(
        filled.refusal,
        GapFillRefusal::None,
        "{} {:?} {}, clip {:?}",
        filled.offending_vertex,
        filled.open_edge,
        open_edge_at(&filled),
        strip_clip::last_outcome()
    );
    let mesh = mesh(&filled);
    let kept = sweep_mesh_measures::kept(&mesh);
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        mesh.vertices(),
        &kept
    ));
    assert_eq!(
        sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
        1
    );

    // Wound as the input: the signed volume stays positive and below the input's.
    let positions = positions(None);
    let before = signed_volume_of(&positions, &ribbed());
    let after = signed_volume(&mesh);
    assert!(after > 0.0);
    assert!(after < before);

    // The rib only gains: less is removed than the same fillet takes from the plain box - the plain
    // box SAMPLED AS THE RIB CASE IS (FlatRib: the rib flattened into the top, so the top rail is
    // laid at the same chain parameters). Like with like, because the strip's volume depends on
    // where its rails are sampled: the plain box with its own top (rail 0, 5, 20 against the
    // front's 0, 10, 20) removes 100.376 where both of these remove about 101.1, and its section
    // areas differ from theirs by up to 0.2 everywhere along the edge (measured with
    // RemovedSectionAreas). Sampled alike, the two builds are identical away from the rib, section
    // for section, and the rib case removes less by the lens.
    let flat = fill_at(&flat_rib_positions(), &flat_rib(), SIZE);
    assert_eq!(flat.refusal, GapFillRefusal::None);
    let flat_mesh = self::mesh(&flat);
    let flat_removed =
        signed_volume_of(&flat_rib_positions(), &flat_rib()) - signed_volume(&flat_mesh);
    println!(
        "rib at {SIZE}: removed {}, plain box sampled alike removed {flat_removed}",
        before - after
    );
    assert!(before - after < flat_removed);
    let input_flat = LevelLineMesh::from_triangles(&flat_rib_positions(), &flat_rib(), None)
        .expect("the flat rib");
    let input_rib = LevelLineMesh::from_triangles(&positions, &ribbed(), None).expect("the rib");
    for x in [3.0, 8.9, 11.1, 15.2] {
        let rib_removes = section(&input_rib, x) - section(&mesh, x);
        let flat_removes = section(&input_flat, x) - section(&flat_mesh, x);
        assert!(
            (rib_removes - flat_removes).abs() <= 1e-9,
            "section x = {x}, away from the rib"
        );
    }

    // And each flank's area in its own plane grows by its cap: none of the rib is trimmed.
    for flank in RIB_FACES.iter().take(2) {
        let origin = positions[flank.0 as usize];
        let cross =
            (positions[flank.1 as usize] - origin).cross(positions[flank.2 as usize] - origin);
        let normal = cross.get_normal();
        let mut grown = 0.0;
        for (t, face) in mesh.triangles().iter().enumerate() {
            if mesh.is_removed(t as i32) {
                continue;
            }

            let corners = [face.a, face.b, face.c].map(|v| mesh.position(v));
            let c = (corners[1] - corners[0]).cross(corners[2] - corners[0]);
            if c.get_normal().dot(normal) >= 1.0 - 1e-6
                && corners
                    .iter()
                    .all(|&p| (p - origin).dot(normal).abs() <= 1e-9)
            {
                grown += c.length() / 2.0;
            }
        }

        println!(
            "flank {flank:?}: area {} grown to {grown}",
            cross.length() / 2.0
        );
        assert!(grown > cross.length() / 2.0);
    }

    // Every added triangle in a flank's plane lies on it to 1e-9, and each flank has a cap.
    for flank in RIB_FACES.iter().take(2) {
        let origin = positions[flank.0 as usize];
        let normal = (positions[flank.1 as usize] - origin)
            .cross(positions[flank.2 as usize] - origin)
            .get_normal();
        let mut caps = 0;
        let mut residual: f64 = 0.0;
        for &t in &filled.added_triangles {
            let face = mesh.triangles()[t as usize];
            let corners = [face.a, face.b, face.c].map(|v| mesh.position(v));
            let n = (corners[1] - corners[0])
                .cross(corners[2] - corners[0])
                .get_normal();
            if n.dot(normal) < 1.0 - 1e-6 {
                continue;
            }

            caps += 1;
            residual = f64::max(
                residual,
                corners
                    .iter()
                    .map(|&c| (c - origin).dot(normal).abs())
                    .fold(f64::NEG_INFINITY, f64::max),
            );
        }

        println!("flank {flank:?}: caps {caps}, residual {residual}");
        assert!(caps > 0);
        assert!(residual <= 1e-9);
    }

    // The ridge runs on past T to where it pierces the strip: a vertex on the ridge line, nearer
    // the edge than T, within size/50 of where the line meets the true cylinder (the chords lie
    // inside it).
    let tie = tie_on_the_cylinder(SIZE);
    let ridge = (positions[8] - positions[11]).get_normal();
    let on_ridge: Vec<Vector3> = (0..mesh.vertex_count())
        .map(|v| mesh.position(v))
        .filter(|&p| {
            (p - positions[8] - (ridge * (p - positions[8]).dot(ridge))).length() <= 1e-9
                && p.y < 3.0 - 1e-6
        })
        .collect();
    assert!(!on_ridge.is_empty());
    let nearest = on_ridge
        .iter()
        .map(|&p| (p - tie).length())
        .fold(f64::INFINITY, f64::min);
    println!("tie on the cylinder {tie:?}, nearest extended ridge vertex {nearest} away");
    assert!(nearest <= SIZE / 50.0);

    let motion = Matrix4X4::create_rotation(Vector3::new(0.3, -0.7, 1.1))
        * Matrix4X4::create_translation(11.5, -3.25, 7.125);
    let moved = fill(&ribbed(), SIZE, Some(motion));
    assert_eq!(
        moved.refusal,
        GapFillRefusal::None,
        "moved: {} {:?}",
        moved.offending_vertex,
        moved.open_edge
    );
    let moved_removed = signed_volume_of(&self::positions(Some(motion)), &ribbed())
        - signed_volume(&self::mesh(&moved));
    assert!(((moved_removed - (before - after)) / (before - after)).abs() <= 1e-9);

    // Far from the origin the neighbour's planes must still read as the same planes (the merge is
    // relative): 1000 units off, the build is the same to what signed volume at that offset can
    // resolve.
    let far = Matrix4X4::create_rotation(Vector3::new(0.3, -0.7, 1.1))
        * Matrix4X4::create_translation(1000.0, -1000.0, 1000.0);
    let far_built = fill(&ribbed(), SIZE, Some(far));
    assert_eq!(
        far_built.refusal,
        GapFillRefusal::None,
        "1000 off: {} {:?} {:?}",
        far_built.offending_vertex,
        far_built.open_edge,
        strip_clip::last_outcome()
    );
    let far_mesh = self::mesh(&far_built);
    let far_kept = sweep_mesh_measures::kept(&far_mesh);
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        far_mesh.vertices(),
        &far_kept
    ));
    let far_removed =
        signed_volume_of(&self::positions(Some(far)), &ribbed()) - signed_volume(&far_mesh);
    assert!(((far_removed - (before - after)) / (before - after)).abs() <= 1e-6);
}

/// A growing rib never bounds the room: the room sits above the asked 5, and 0.1, 0.5, 0.9 and 1.0
/// of it all build closed.
#[test]
fn the_rib_does_not_bound_the_room() {
    let positions = positions(None);
    let past = build(&positions, &ribbed(), 20.0);
    println!(
        "rib at 20: {:?}, room {}, set by {:?}",
        past.refusal, past.room, past.room_set_by
    );
    assert_eq!(past.refusal, BandGapRefusal::Size);
    assert!(past.room > SIZE);

    // The extended ridge (the plane x = 10: z = 2y + 4) stays |3r - 6| / sqrt 5 from the round's
    // axis (y = r, z = 10 - r). The round is built of 16 chords, none nearer the axis than
    // r cos(pi / 32), so the ridge pierces them wherever it meets them while that distance is at
    // most r cos(pi / 32), which ends at r = 6 / (3 - sqrt 5 cos(pi / 32)) = 7.745 (the true
    // round's 7.854 is past the chords).
    let pierce_ends = 6.0 / (3.0 - (5f64.sqrt() * (std::f64::consts::PI / 32.0).cos()));
    assert!(
        (past.room - pierce_ends).abs() <= 1e-6,
        "room {} against {pierce_ends}",
        past.room
    );

    // Past the room the pick refuses by size and names the same room.
    let beyond = build(&positions, &ribbed(), 1.1 * past.room);
    assert_eq!(beyond.refusal, BandGapRefusal::Size);
    assert!((beyond.room - past.room).abs() <= 1e-9);

    for fraction in [0.1, 0.5, 0.9, 1.0] {
        let size = fraction * past.room;
        let filled = fill(&ribbed(), size, None);
        assert_eq!(
            filled.refusal,
            GapFillRefusal::None,
            "{fraction} of the room: {} {:?} {}, clip {:?}",
            filled.offending_vertex,
            filled.open_edge,
            open_edge_at(&filled),
            strip_clip::last_outcome()
        );
        let mesh = mesh(&filled);
        let kept = sweep_mesh_measures::kept(&mesh);
        assert!(
            sweep_mesh_measures::is_closed_and_consistent(mesh.vertices(), &kept),
            "{fraction} of the room"
        );

        // Once the band reaches the rib (its tip is 3 from the edge), the tie - the extended
        // ridge's crossing - lies ON the profile as built: 16 chords of the round, so between
        // r cos(pi/32) and r from its axis.
        if size <= 3.0 {
            continue;
        }

        let ties: Vec<Vector3> = (0..mesh.vertex_count())
            .map(|v| mesh.position(v))
            .filter(|p| {
                (p.x - 10.0).abs() <= 1e-9
                    && (p.z - ((2.0 * p.y) + 4.0)).abs() <= 1e-9
                    && p.y < 3.0 - 1e-9
            })
            .collect();
        assert!(
            !ties.is_empty(),
            "{fraction} of the room: no tie on the extended ridge"
        );
        for tie in ties {
            let from_axis = (((tie.y - size) * (tie.y - size))
                + ((tie.z - (10.0 - size)) * (tie.z - (10.0 - size))))
                .sqrt();
            assert!(
                from_axis <= size + 1e-9,
                "{fraction} of the room: tie {tie:?} is {from_axis} from the axis"
            );
            assert!(
                from_axis >= (size * (std::f64::consts::PI / 32.0).cos()) - 1e-9,
                "{fraction} of the room: tie {tie:?} is {from_axis} from the axis"
            );
        }
    }
}

/// The owner fixture: Encroaching Side at its saved 3.4 builds, and the lens's far tip - where the
/// nose's ridge pierces the round - is within size/50 of the control's (15.475, 5, 10) in the box
/// frame, which is the fixture's frame moved by (20, 118.5, 0) (the nose's ridge end (15, 5, 10) is
/// the fixture's v9).
#[test]
fn encroaching_side_builds_with_the_lens_tip_on_the_control() {
    let fixture = Rc::new(LevelLineFixture::load("encroaching-side-r34"));
    let profile = if fixture.chamfer {
        BandProfile::Chamfer
    } else {
        BandProfile::Fillet
    };
    let feature = {
        let fixture = Rc::clone(&fixture);
        Rc::new(move |a, b| fixture.is_feature_edge(a, b))
    };
    let options = BandGapBuildOptions {
        chord_tolerance: 0.0,
        snap: 1e-9,
        fillet_segments: fixture.segments,
        ..BandGapBuildOptions::default()
    };
    let gap = band_gap_entry_points::build_pick(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        &fixture.closed,
        Some(feature),
        fixture.size,
        profile,
        &options,
    )
    .expect("a valid pick");
    assert_eq!(
        gap.refusal,
        BandGapRefusal::None,
        "room {}, set by {:?}",
        gap.room,
        gap.room_set_by
    );

    let filled = gap_fill::fill(&gap, profile, fixture.segments).expect("the fill runs");
    assert_eq!(
        filled.refusal,
        GapFillRefusal::None,
        "{} {:?} {}, clip {:?}",
        filled.offending_vertex,
        filled.open_edge,
        open_edge_at(&filled),
        strip_clip::last_outcome()
    );
    let mesh = mesh(&filled);
    let kept = sweep_mesh_measures::kept(&mesh);
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        mesh.vertices(),
        &kept
    ));
    assert_eq!(
        sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
        1
    );

    // The control's cylinder: axis one radius in from the face x = 20 and one radius off the slope
    // x + y = 20.
    let radius = fixture.size;
    let axis_x = 20.0 - radius;
    let axis_y = 20.0 + (radius * 2f64.sqrt()) - axis_x;
    let lens_x = axis_x - ((radius * radius) - ((axis_y - 5.0) * (axis_y - 5.0))).sqrt();
    let lens_tip = Vector3::new(lens_x - 20.0, 5.0 - 118.5, 10.0);
    let all: Vec<Vector3> = (0..mesh.vertex_count()).map(|v| mesh.position(v)).collect();
    let nearest = all
        .iter()
        .map(|&p| (p - lens_tip).length())
        .fold(f64::INFINITY, f64::min);
    // The first of the least, so a tie names the same point every run.
    let mut nearest_at = all[0];
    for &p in &all {
        if (p - lens_tip).length() < (nearest_at - lens_tip).length() {
            nearest_at = p;
        }
    }
    let mut on_ridge: Vec<f64> = all
        .iter()
        .filter(|p| (p.y + 113.5).abs() < 1e-6 && (p.z - 10.0).abs() < 1e-6)
        .map(|p| p.x)
        .collect();
    on_ridge.sort_by(f64::total_cmp);
    println!(
        "Encroaching Side at {}: lens tip {lens_tip:?}, nearest vertex {nearest_at:?}, {nearest} away, clip {:?}, ridge-line vertices x {on_ridge:?}",
        fixture.size,
        strip_clip::last_outcome()
    );
    assert!(nearest <= fixture.size / 50.0);
}

/// A reading, not a verdict (returns at once unless `RIB_SECTION_LOG` names a file): the area each
/// build removes from the section x = c, for the rib and the plain box, against the 16-chord
/// polygon's r^2 (1 - 8 sin(pi/32)).
#[test]
fn removed_section_areas() {
    let Some(log_path) = std::env::var("RIB_SECTION_LOG")
        .ok()
        .filter(|p| !p.is_empty())
    else {
        return;
    };

    let mut log = std::fs::File::create(log_path).expect("the log file");
    let rib = fill(&ribbed(), SIZE, None);
    let plain_built = fill(&plain(), SIZE, None);
    let input_rib =
        LevelLineMesh::from_triangles(&positions(None), &ribbed(), None).expect("the rib");
    let input_plain =
        LevelLineMesh::from_triangles(&positions(None), &plain(), None).expect("the plain box");
    let flat = fill_at(&flat_rib_positions(), &flat_rib(), SIZE);
    let input_flat = LevelLineMesh::from_triangles(&flat_rib_positions(), &flat_rib(), None)
        .expect("the flat rib");
    let (rib_mesh, plain_mesh, flat_mesh) = (mesh(&rib), mesh(&plain_built), mesh(&flat));
    writeln!(
        log,
        "volumes removed: rib {}, plain {}, flat rib {}",
        signed_volume_of(&positions(None), &ribbed()) - signed_volume(&rib_mesh),
        signed_volume_of(&positions(None), &plain()) - signed_volume(&plain_mesh),
        signed_volume_of(&flat_rib_positions(), &flat_rib()) - signed_volume(&flat_mesh)
    )
    .expect("the log");
    writeln!(
        log,
        "chord polygon removes {} per unit length",
        SIZE * SIZE * (1.0 - (8.0 * (std::f64::consts::PI / 32.0).sin()))
    )
    .expect("the log");
    for x in [
        0.01, 0.5, 1.0, 2.0, 3.0, 4.0, 5.3, 6.0, 7.0, 8.0, 8.9, 9.3, 9.5, 9.7, 9.8, 9.9, 9.95,
        10.05, 10.1, 10.2, 10.3, 10.5, 10.7, 11.1, 12.0, 13.0, 14.0, 15.2, 16.0, 18.0, 19.5, 19.99,
    ] {
        writeln!(
            log,
            "x {x}: rib removes {:.6}, plain removes {:.6}, flat rib removes {:.6}",
            section(&input_rib, x) - section(&rib_mesh, x),
            section(&input_plain, x) - section(&plain_mesh, x),
            section(&input_flat, x) - section(&flat_mesh, x)
        )
        .expect("the log");
    }
}

/// The area of a closed mesh's section by the plane x = c, by Green's theorem over the cut
/// segments.
fn section(mesh: &LevelLineMesh, c: f64) -> f64 {
    let mut area = 0.0;
    for t in 0..mesh.triangle_count() {
        if mesh.is_removed(t) {
            continue;
        }

        let face = mesh.triangles()[t as usize];
        let corners = [face.a, face.b, face.c].map(|v| mesh.position(v));
        let mut cut = Vec::new();
        for i in 0..3 {
            let (p, q) = (corners[i], corners[(i + 1) % 3]);
            if (p.x - c) * (q.x - c) < 0.0 {
                cut.push(p + ((q - p) * ((c - p.x) / (q.x - p.x))));
            }
        }

        if cut.len() != 2 {
            continue;
        }

        // Counter-clockwise about +x round an outward normal: the tangent is x cross normal.
        let normal = (corners[1] - corners[0]).cross(corners[2] - corners[0]);
        let tangent = Vector3::UNIT_X.cross(normal);
        let (a, b) = if (cut[1] - cut[0]).dot(tangent) >= 0.0 {
            (cut[0], cut[1])
        } else {
            (cut[1], cut[0])
        };
        area += ((a.y * b.z) - (a.z * b.y)) / 2.0;
    }

    area
}

/// Where the ridge line T-R, run on past T, first meets the fillet's cylinder (axis y = r,
/// z = 10 - r).
fn tie_on_the_cylinder(r: f64) -> Vector3 {
    // Along T + s (0, -5, -10): (3 - r - 5s)^2 + (r - 10s)^2 = r^2, the smaller root.
    let a = 125.0;
    let b = (-10.0 * (3.0 - r)) - (20.0 * r);
    let c = (3.0 - r) * (3.0 - r);
    let s = (-b - ((b * b) - (4.0 * a * c)).sqrt()) / (2.0 * a);
    Vector3::new(10.0, 3.0 - (5.0 * s), 10.0 - (10.0 * s))
}

/// The rib chain's open band at `size`: a fillet of [`SEGMENTS`], over feature edges past 20
/// degrees.
fn build(positions: &[Vector3], indices: &[(i32, i32, i32)], size: f64) -> BandGapResult {
    let options = BandGapBuildOptions {
        fillet_segments: SEGMENTS,
        ..BandGapBuildOptions::default()
    };
    band_gap_entry_points::build(
        positions,
        indices,
        &CHAIN,
        false,
        Some(feature_edges_over(positions, indices, 20.0)),
        size,
        BandProfile::Fillet,
        &options,
    )
    .expect("a valid pick")
}

/// The fill on the rib's positions, moved by `motion` where given.
fn fill(indices: &[(i32, i32, i32)], size: f64, motion: Option<Matrix4X4>) -> GapFillResult {
    fill_at(&positions(motion), indices, size)
}

/// The band built and filled: a refused cut is a test failure.
fn fill_at(positions: &[Vector3], indices: &[(i32, i32, i32)], size: f64) -> GapFillResult {
    let gap = build(positions, indices, size);

    assert!(
        gap.built(),
        "refused at {size}: {:?}, room {}, set by {:?}",
        gap.refusal,
        gap.room,
        gap.room_set_by
    );

    gap_fill::fill(&gap, BandProfile::Fillet, SEGMENTS).expect("the fill runs")
}
