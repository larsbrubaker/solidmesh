//! What it takes for an open chain's two sides to meet ROUND its END vertex, measured. A crease
//! that simply RAMPS out of a flat top does NOT do it: the ramp's own fan edges at the end vertex
//! are features, and they are the wall that keeps the two sides two. The red shape (pick c564 of
//! "Issue with even a single edge.mcx") also wants the crease DOUBLED over its length; these rows
//! record the ramp alone - its reading divides the sides, is rigid, and is monotone in the size.

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::band_gap_reading;
use solidmesh::bevel::level_line::{BandGapResult, BandProfile, RoomReading};
use solidmesh_math::{Vector2, Vector3};

use crate::level_line_feature_edges::feature_edges_over;
use crate::several_chains_fixtures;
use crate::sweep_mesh_measures;

/// The size a user asks for, and the size this block has room for many times over.
const FILLET: f64 = 0.1;

const FOLD_DEGREES: f64 = 10.0;

/// The cross-section's point count - the stride between one slice's vertices and the next's.
const RING: i32 = 5;

/// How many slices the prism is cut into along its length.
const SLICES: i32 = 8;

/// The slice the ridge starts rising at: the picked edge runs from here to the next.
const RIDGE_STARTS: i32 = 4;

/// The ridge's height where it is only a hint - a fold of a quarter of a degree over a half-width
/// of five, well under [`FOLD_DEGREES`], so the top before the crease is one face. It is not zero
/// because the end cap is fanned from the section's first point and a ridge of no height would fan
/// a triangle of no area.
const FLAT_RIDGE: f64 = 0.01;

/// The ridge's height where it is a crease: a fold of 33 degrees, a feature by any threshold.
const CREASE_RIDGE: f64 = 1.5;

/// The picked crease: the ridge's FIRST edge, an open chain of one edge whose near end stands in
/// the flat top and whose far end carries on up the ridge.
fn chain() -> Vec<i32> {
    vec![(RIDGE_STARTS * RING) + 4, ((RIDGE_STARTS + 1) * RING) + 4]
}

/// The app's seam epsilon for this fixture, by the app's own rule - 5e-4 of the bounding-box
/// diagonal (`BevelWorkingMesh.SeamEpsilonFor`), about 0.0088 here.
fn seam_epsilon() -> f64 {
    let (positions, _) = ramped_crease_block();
    let mut low = positions[0];
    let mut high = positions[0];
    for &point in &positions {
        low = Vector3::component_min(low, point);
        high = Vector3::component_max(high, point);
    }

    5e-4 * (high - low).length()
}

/// The fixture is a closed, consistently wound solid before anything is read or cut.
#[test]
fn the_block_is_a_solid() {
    let (positions, indices) = ramped_crease_block();
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        &positions, &indices
    ));
    assert_eq!(
        sweep_mesh_measures::shell_count(positions.len() as i32, &indices),
        1
    );
}

/// The measurement this file exists for: a crease ramping out of a flat top divides its sides, at
/// the app's epsilon and at none, and its room is a statement about size.
#[test]
fn a_crease_ramping_from_a_flat_top_still_divides_its_sides() {
    let (positions, indices) = ramped_crease_block();
    for epsilon in [0.0, seam_epsilon()] {
        let reading = read(&positions, &indices, Some(epsilon));

        assert!(reading.chain_divides_sides, "epsilon {epsilon:?}");
        assert!(reading.is_a_size_statement(), "epsilon {epsilon:?}");
        assert!(
            reading.room > FILLET,
            "the nearest wall is the block's own: read {}",
            reading.room
        );
    }
}

/// The reading says the same thing wherever the part stands: the same crease, turned and moved,
/// divides its sides the same way and reads the same room.
#[test]
fn the_ramped_crease_reads_the_same_moved() {
    let (positions, indices) = ramped_crease_block();
    let here = read(&positions, &indices, None);

    let moved: Vec<Vector3> = positions
        .iter()
        .map(|p| p.transform(several_chains_fixtures::motion()))
        .collect();
    let there = read(&moved, &indices, None);

    assert_eq!(there.chain_divides_sides, here.chain_divides_sides);
    assert_eq!(there.room_set_by.kind, here.room_set_by.kind);
    assert!(
        (there.room - here.room).abs() <= 1e-9 * f64::max(1.0, here.room),
        "here {:?} moved {:?}",
        here.room,
        there.room
    );
}

/// A pick that builds at a size builds at every smaller size - the promise no rule about the ends
/// may break, since a pin is a statement about the mesh and never about the size asked for.
#[test]
fn the_ramped_crease_is_monotone_in_the_size() {
    let (positions, indices) = ramped_crease_block();
    let mut built = true;
    for size in [0.001, 0.01, 0.1, 1.0] {
        let builds = build(&positions, &indices, size).built();
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

fn read(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    seam_epsilon_or_the_apps: Option<f64>,
) -> RoomReading {
    band_gap_reading::read_room(
        positions,
        indices,
        &chain(),
        false,
        Some(feature_edges_over(positions, indices, FOLD_DEGREES)),
        BandProfile::Fillet,
        1e-9,
        None,
        seam_epsilon_or_the_apps.unwrap_or_else(seam_epsilon),
        f64::INFINITY,
    )
    .expect("the reading")
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
            seam_epsilon: seam_epsilon(),
            ..BandGapBuildOptions::default()
        },
    )
    .expect("a valid pick")
}

/// A block whose top carries a ridge that RAMPS: from the far cap back to [`RIDGE_STARTS`] the
/// ridge is a crease of 33 degrees, and from there to the near cap it is [`FLAT_RIDGE`] high - a
/// quarter of a degree, under the threshold - so the top before the crease is one face.
fn ramped_crease_block() -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    let mut positions = Vec::new();
    for slice in 0..=SLICES {
        let x = -5.0 + (10.0 * slice as f64 / SLICES as f64);
        let ridge = if slice <= RIDGE_STARTS {
            FLAT_RIDGE
        } else {
            CREASE_RIDGE
        };

        // The cross-section in (y, z), counter-clockwise seen down -x, from the top-front corner
        // round the front and the bottom and back over the ridge.
        let section = [
            Vector2::new(-5.0, 5.0),
            Vector2::new(-5.0, -5.0),
            Vector2::new(5.0, -5.0),
            Vector2::new(5.0, 5.0),
            Vector2::new(0.0, 5.0 + ridge),
        ];

        for point in section {
            positions.push(Vector3::new(x, point.x, point.y));
        }
    }

    let mut indices = Vec::new();
    for slice in 0..SLICES {
        let a = slice * RING;
        let b = (slice + 1) * RING;
        for i in 0..RING {
            let next = (i + 1) % RING;
            indices.push((a + i, b + i, b + next));
            indices.push((a + i, b + next, a + next));
        }
    }

    // The two end caps, fanned from the section's first point.
    let last = SLICES * RING;
    for i in 1..RING - 1 {
        indices.push((0, i, i + 1));
        indices.push((last, last + i + 1, last + i));
    }

    (positions, indices)
}
