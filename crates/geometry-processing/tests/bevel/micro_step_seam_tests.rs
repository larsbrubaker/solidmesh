//! Port of `R/Tests/GeometryProcessingTests/LevelLine/MicroStepSeamTests.cs`: a picked crease
//! doubled by a step a few microns wide - a seam, not a wall - reads the part's room and builds at
//! the asked size when the seam epsilon is given (`SeamEdges`), while a step outside the epsilon
//! is still a wall; and the read-drop-re-read loop ([`PickThatFits`]) honours the same epsilon.

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::band_gap_reading::read_room;
use geometry_processing::bevel::level_line::gap_fill::{self, DEFAULT_FILLET_SEGMENTS};
use geometry_processing::bevel::level_line::{
    BandGapResult, BandProfile, PickThatFits, RoomReading,
};
use vector_math::{Vector2, Vector3};

use crate::level_line_feature_edges::feature_edges_over;
use crate::several_chains_fixtures;
use crate::sweep_mesh_measures;

const STEP_SIZE: f64 = 0.003;

const FOLD_DEGREES: f64 = 10.0;

const FILLET: f64 = 0.1;

const SEAM_EPSILON: f64 = 5e-4 * 17.320508075688775;

const RING: i32 = 6;

const SLICES: i32 = 8;

fn chain() -> Vec<i32> {
    vec![(SLICES / 2) * RING, (SLICES / 2 + 1) * RING]
}

fn read(positions: &[Vector3], indices: &[(i32, i32, i32)]) -> RoomReading {
    read_room(
        positions,
        indices,
        &chain(),
        false,
        Some(feature_edges_over(positions, indices, FOLD_DEGREES)),
        BandProfile::Fillet,
        1e-9,
        None,
        SEAM_EPSILON,
        f64::INFINITY,
    )
    .expect("the chain reads")
}

fn build(positions: &[Vector3], indices: &[(i32, i32, i32)], size: f64) -> BandGapResult {
    band_gap_entry_points::build(
        positions,
        indices,
        &chain(),
        false,
        Some(feature_edges_over(positions, indices, FOLD_DEGREES)),
        size,
        BandProfile::Fillet,
        &BandGapBuildOptions {
            seam_epsilon: SEAM_EPSILON,
            ..BandGapBuildOptions::default()
        },
    )
    .expect("the cut runs")
}

#[test]
fn a_doubled_crease_reads_the_parts_room_and_not_the_micron_beside() {
    let (positions, indices) = stepped_block(STEP_SIZE);
    let reading = read(&positions, &indices);

    println!(
        "micro-step room {} set by {:?}",
        reading.room, reading.room_set_by.kind
    );
    assert!(
        reading.room > FILLET,
        "the step is {STEP_SIZE} of seam, not a wall: read {} ({:?})",
        reading.room,
        reading.room_set_by.kind
    );
}

#[test]
fn a_doubled_crease_builds_at_the_asked_size() {
    let (positions, indices) = stepped_block(STEP_SIZE);
    let gap = build(&positions, &indices, FILLET);

    assert!(
        gap.built(),
        "refused {:?} naming room {}",
        gap.refusal,
        gap.room
    );

    let fill =
        gap_fill::fill(&gap, BandProfile::Fillet, DEFAULT_FILLET_SEGMENTS).expect("the fill runs");
    assert!(fill.filled(), "{:?}", fill.refusal);

    // The fixture itself first, so a winding fault in the test's own prism cannot read as the
    // cut's.
    assert!(
        sweep_mesh_measures::is_closed_and_consistent(&positions, &indices),
        "the fixture is a closed, consistently wound solid before anything is cut"
    );

    let mesh = fill.mesh.as_ref().expect("the filled mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    let points: Vec<Vector3> = (0..mesh.vertex_count()).map(|v| mesh.position(v)).collect();
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        &points, &kept
    ));
    assert_eq!(
        sweep_mesh_measures::shell_count(points.len() as i32, &kept),
        1
    );
}

#[test]
fn a_step_whose_outer_edge_is_outside_the_epsilon_is_still_a_wall() {
    let step = 0.008;
    let (positions, indices) = stepped_block(step);
    let reading = read(&positions, &indices);

    println!(
        "eight-micron seam room {} set by {:?}",
        reading.room, reading.room_set_by.kind
    );
    assert!(
        reading.room < FILLET,
        "the tread's end is {} from the crease, outside the epsilon {SEAM_EPSILON}",
        step * 2f64.sqrt()
    );

    // The room is the tread end's own distance from the picked crease - the step's diagonal - and
    // so is a statement about the part, never the flat zero a foreign edge standing ON the chain
    // gives.
    assert!(reading.room > 0.0);
    assert!(
        (reading.room - step * 2f64.sqrt()).abs() <= 1e-7,
        "{}",
        reading.room
    );
}

#[test]
fn a_half_fillet_ribbon_is_still_a_wall() {
    let wall = 0.05;
    let (positions, indices) = stepped_block(wall);
    let reading = read(&positions, &indices);

    println!(
        "half-fillet ribbon room {} set by {:?}",
        reading.room, reading.room_set_by.kind
    );
    assert!(
        reading.room < FILLET,
        "a wall a twentieth of the part thick bounds a fillet twice its width"
    );
}

#[test]
fn the_three_micron_seam_reads_the_same_moved() {
    let (positions, indices) = stepped_block(STEP_SIZE);
    let here = read(&positions, &indices);

    let motion = several_chains_fixtures::motion();
    let moved: Vec<Vector3> = positions.iter().map(|p| p.transform(motion)).collect();
    let there = read(&moved, &indices);

    println!(
        "here {} ({:?}) moved {} ({:?})",
        here.room, here.room_set_by.kind, there.room, there.room_set_by.kind
    );
    assert_eq!(there.room_set_by.kind, here.room_set_by.kind);
    assert!((there.room - here.room).abs() <= 1e-9 * here.room.max(1.0));

    let built = build(&moved, &indices, FILLET);

    assert!(
        built.built(),
        "refused {:?} naming room {}",
        built.refusal,
        built.room
    );
}

#[test]
fn the_micro_step_is_monotone_in_the_size() {
    let (positions, indices) = stepped_block(STEP_SIZE);
    let room = read(&positions, &indices).room;
    let mut built = true;
    for size in [0.001, 0.01, 0.1, 1.0] {
        let builds = build(&positions, &indices, size).built();

        println!("size {size} builds {builds} (room {room})");
        if !builds {
            built = false;
        } else {
            assert!(
                built,
                "{size} builds where a smaller size did not - the room is not monotone"
            );
        }
    }
}

#[test]
fn the_settling_condemns_nobody_over_a_seam() {
    let (positions, indices) = stepped_block(STEP_SIZE);
    let runs = vec![
        vec![3 * RING, 4 * RING],
        vec![(3 * RING) + 5, (4 * RING) + 5],
    ];

    let closed = [false, false];
    let features = feature_edges_over(&positions, &indices, FOLD_DEGREES);

    let blind = PickThatFits::read(
        &positions,
        &indices,
        &runs,
        &closed,
        Some(features.clone()),
        BandProfile::Fillet,
        FILLET,
        1e-9,
        None,
        0.0,
        f64::INFINITY,
    )
    .expect("the pick reads");
    println!(
        "blind: dropped {}, room {:?}",
        blind.dropped.len(),
        blind.last.as_ref().map(|r| r.room)
    );
    assert!(
        !blind.dropped.is_empty(),
        "read at no epsilon the ribbon is a wall and the settling must condemn the doubled runs"
    );

    let seeing = PickThatFits::read(
        &positions,
        &indices,
        &runs,
        &closed,
        Some(features),
        BandProfile::Fillet,
        FILLET,
        1e-9,
        None,
        SEAM_EPSILON,
        f64::INFINITY,
    )
    .expect("the pick reads");
    println!(
        "seeing: dropped {}, room {:?}",
        seeing.dropped.len(),
        seeing.last.as_ref().map(|r| r.room)
    );
    assert_eq!(
        seeing.dropped.len(),
        0,
        "nothing is too big for the part: {:?}",
        seeing.dropped
    );
    assert!(seeing.fits);
    assert!(seeing.last.as_ref().expect("a last reading").room > FILLET);
}

/// C# `SteppedBlock(step)`: a 10 mm block along x whose top-front crease is doubled by a step
/// `step` wide, as a prism of `SLICES` slices with both ends fanned from the section's first point.
fn stepped_block(step: f64) -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    // The cross-section in (y, z), counter-clockwise seen down -x, from the top-front crease round.
    let section = [
        Vector2::new(-5.0, 5.0),               // 0 the picked crease
        Vector2::new(-5.0, 5.0 - step),        // 1 the riser's foot - the second feature edge
        Vector2::new(-5.0 - step, 5.0 - step), // 2 the tread's end
        Vector2::new(-5.0 - step, -5.0),       // 3 the wall's foot
        Vector2::new(5.0, -5.0),               // 4
        Vector2::new(5.0, 5.0),                // 5
    ];

    let mut positions = Vec::new();
    for slice in 0..=SLICES {
        let x = -5.0 + 10.0 * slice as f64 / SLICES as f64;
        for point in &section {
            positions.push(Vector3::new(x, point.x, point.y));
        }
    }

    let mut indices = Vec::new();
    let ring = section.len() as i32;
    for slice in 0..SLICES {
        let a = slice * ring;
        let b = (slice + 1) * ring;
        for i in 0..ring {
            let next = (i + 1) % ring;
            indices.push((a + i, b + i, b + next));
            indices.push((a + i, b + next, a + next));
        }
    }

    // The two end caps, fanned from the section's first point.
    let last = SLICES * ring;
    for i in 1..ring - 1 {
        indices.push((0, i, i + 1));
        indices.push((last, last + i + 1, last + i));
    }

    (positions, indices)
}
