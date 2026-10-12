//! Ports `Tests/GeometryProcessingTests/LevelLine/TrebledEndSeamTests.cs`: the END half of the
//! seam rule, as real manifold geometry - a picked crease whose end vertex the file carries as
//! THREE copies microns apart, stitched by triangles of no area. The room must read the part, not
//! the microns at its end, while a real feature joined by ordinary edges microns away is still a
//! wall. The block and its rewrites are `trebled_end_geometry.rs`.

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::{
    band_gap_reading, gap_fill, BandGapRefusal, BandGapResult, BandProfile, RoomReading,
};
use vector_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;
use crate::several_chains_fixtures::motion;
use crate::sweep_mesh_measures::{is_closed_and_consistent, kept, shell_count};
use crate::trebled_end_geometry::{plain_block, split_vertex_at_spokes, tent_inside, Solid};

const HALF: f64 = 5.0;

const RING: i32 = 4;

const SLICES: i32 = 8;

/// C# `Diagonal`: the block is 10 across, so its diagonal is this.
const DIAGONAL: f64 = 17.320508075688775;

/// C# `SeamEpsilon`: the app's number for this block, 5e-4 of the diagonal.
const SEAM_EPSILON: f64 = 5e-4 * DIAGONAL;

/// C# `Split`: how far apart the copies stand - 2.5e-4 of the diagonal.
const SPLIT: f64 = 2.5e-4 * DIAGONAL;

const FILLET: f64 = 0.1;

const FOLD_DEGREES: f64 = 10.0;

/// C# `EndSlice`: the slice the picked crease run ends at, mid-block.
const END_SLICE: i32 = SLICES / 2;

/// C# `Chain`: one edge of the top-front crease, an open chain.
fn chain() -> Vec<i32> {
    vec![(END_SLICE - 1) * RING, END_SLICE * RING]
}

/// C# `ContinuingCrease`: the crease's continuation past the chain's end.
const CONTINUING_CREASE: i32 = (END_SLICE + 1) * RING;

/// C# `TentSize`: the tent's base, microns across - and so its distance from the chain's end.
const TENT_SIZE: f64 = 0.002;

/// C# `ATrebledEndReadsThePartsRoomAndNotTheMicronsAtItsEnd`.
#[test]
fn a_trebled_end_reads_the_parts_room_and_not_the_microns_at_its_end() {
    let (positions, indices) = trebled_end_block();
    let blind = read(&positions, &indices, 0.0);
    let seeing = read(&positions, &indices, SEAM_EPSILON);

    let (clean_positions, clean_indices) = plain_block(HALF, SLICES);
    let clean = read(&clean_positions, &clean_indices, SEAM_EPSILON);

    println!(
        "trebled end: blind {:?} ({:?}), seeing {:?} ({:?}), clean block {:?}",
        blind.room, blind.room_set_by.kind, seeing.room, seeing.room_set_by.kind, clean.room
    );

    assert!(
        seeing.room >= clean.room - 1e-9,
        "the split is not material: an unsplit block has room {}",
        clean.room
    );
    assert!(seeing.room > FILLET);
    assert!(
        (blind.room - seeing.room).abs() <= 1e-9,
        "the stitching at an end is not a wall with or without the epsilon"
    );
}

/// C# `ATrebledEndBuildsAtTheAskedSize`.
#[test]
fn a_trebled_end_builds_at_the_asked_size() {
    let (positions, indices) = trebled_end_block();

    // The fixture itself first, so a winding fault in the test's own stitching cannot read as the
    // cut's.
    assert!(
        is_closed_and_consistent(&positions, &indices),
        "the trebled block is a closed, consistently wound solid before anything is cut"
    );

    let gap = build(&positions, &indices, FILLET);
    assert!(
        gap.built(),
        "refused {:?} naming room {}",
        gap.refusal,
        gap.room
    );

    let fill = gap_fill::fill(&gap, BandProfile::Fillet, gap_fill::DEFAULT_FILLET_SEGMENTS)
        .expect("the fill fills or refuses by name");
    assert!(fill.filled(), "{:?}", fill.refusal);

    let mesh = fill.mesh.as_ref().expect("a filled mesh").borrow();
    let kept = kept(&mesh);
    let points: Vec<Vector3> = (0..mesh.vertices.len() as i32)
        .map(|v| mesh.position(v))
        .collect();
    assert!(is_closed_and_consistent(&points, &kept));
    assert_eq!(shell_count(points.len() as i32, &kept), 1);
}

/// C# `TheTrebledEndIsMonotoneInTheSize`.
#[test]
fn the_trebled_end_is_monotone_in_the_size() {
    let (positions, indices) = trebled_end_block();
    let mut built = true;
    for size in [0.001, 0.01, 0.1, 1.0] {
        let builds = build(&positions, &indices, size).built();
        println!("size {size} builds {builds}");
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

/// C# `TheTrebledEndReadsTheSameMoved`.
#[test]
fn the_trebled_end_reads_the_same_moved() {
    let (positions, indices) = trebled_end_block();
    let here = read(&positions, &indices, SEAM_EPSILON);

    let moved: Vec<Vector3> = positions.iter().map(|&p| p.transform(motion())).collect();
    let there = read(&moved, &indices, SEAM_EPSILON);

    println!(
        "here {:?} ({:?}) moved {:?} ({:?})",
        here.room, here.room_set_by.kind, there.room, there.room_set_by.kind
    );
    assert_eq!(there.room_set_by.kind, here.room_set_by.kind);
    assert!((there.room - here.room).abs() <= 1e-9 * here.room.max(1.0));
}

/// C# `AShortFeatureEdgeNearTheEndButNotJoinedToItIsStillAWall`.
#[test]
fn a_short_feature_edge_near_the_end_but_not_joined_to_it_is_still_a_wall() {
    let (positions, indices) = trebled_end_block_with_a_tent_beside_it();
    assert!(
        is_closed_and_consistent(&positions, &indices),
        "the tent leaves the block closed and consistently wound"
    );

    let reading = read(&positions, &indices, SEAM_EPSILON);
    println!(
        "tent beside the end: room {:?} ({:?})",
        reading.room, reading.room_set_by.kind
    );

    assert!(reading.room > 0.0);
    assert!(
        reading.room < TENT_SIZE * 4.0,
        "the tent's base edges are the nearest wall and the room is their distance"
    );

    let refused = build(&positions, &indices, FILLET);
    assert!(
        !refused.built(),
        "a fillet a hundred times the tent's distance must not cut into it in silence"
    );
    assert_eq!(refused.refusal, BandGapRefusal::Size);

    // And a size inside the room still builds, so the wall bounds the pick without condemning it.
    let small = build(&positions, &indices, reading.room / 4.0);
    assert!(
        small.built(),
        "refused {:?} naming room {}",
        small.refusal,
        small.room
    );
}

/// C# `Read`.
fn read(positions: &[Vector3], indices: &[(i32, i32, i32)], seam_epsilon: f64) -> RoomReading {
    band_gap_reading::read_room(
        positions,
        indices,
        &chain(),
        false,
        Some(feature_edges_over(positions, indices, FOLD_DEGREES)),
        BandProfile::Fillet,
        1e-9,
        None,
        seam_epsilon,
        f64::INFINITY,
    )
    .expect("the chain reads")
}

/// C# `Build`.
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
    .expect("the chain builds or refuses by name")
}

/// C# `TrebledEndBlock`: the block with its picked crease's END vertex split into three copies.
fn trebled_end_block() -> Solid {
    let (positions, indices) = plain_block(HALF, SLICES);
    split_vertex_at_spokes(
        &positions,
        &indices,
        END_SLICE * RING,
        &[
            CONTINUING_CREASE,
            (END_SLICE * RING) + 1,
            ((END_SLICE - 1) * RING) + 3,
        ],
        chain()[0],
        &[
            Vector3::new(SPLIT, 0.0, 0.0),
            Vector3::new(0.0, SPLIT / 2.0, 0.0),
        ],
    )
}

/// C# `TrebledEndBlockWithATentBesideIt`.
fn trebled_end_block_with_a_tent_beside_it() -> Solid {
    let (positions, indices) = trebled_end_block();
    let end = positions[(END_SLICE * RING) as usize];

    // The top-face triangle at the end vertex itself - the one that names the continuing crease
    // and this slice's back corner but NOT the next slice's.
    let top = indices
        .iter()
        .position(|&t| {
            names(t, CONTINUING_CREASE)
                && names(t, (END_SLICE * RING) + 3)
                && !names(t, ((END_SLICE + 1) * RING) + 3)
        })
        .expect("C#'s First");

    tent_inside(
        &positions,
        &indices,
        top,
        &[
            end + Vector3::new(TENT_SIZE, TENT_SIZE, 0.0),
            end + Vector3::new(TENT_SIZE * 2.5, TENT_SIZE * 1.1, 0.0),
            end + Vector3::new(TENT_SIZE * 1.75, TENT_SIZE * 2.5, 0.0),
        ],
        TENT_SIZE * 1.5,
    )
}

/// C# `Names`.
fn names(triangle: (i32, i32, i32), vertex: i32) -> bool {
    triangle.0 == vertex || triangle.1 == vertex || triangle.2 == vertex
}
