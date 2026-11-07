//! The stopped end's room reading (`stopped_end_room.rs`) read directly on hand-made fields: a
//! side whose plane cut lies far from the end vertex, and a chain whose far end's clip cuts the
//! plane short of where its raw band reaches.

use geometry_processing::bevel::level_line::stopped_end_room::{stopped_end_room, StoppedEnd};
use geometry_processing::bevel::level_line::{ChainField, ChainFieldEndClip};
use vector_math::Vector3;

/// A chain field of unit setback at size `s`.
fn field(points: &[Vector3], s: f64) -> ChainField {
    ChainField::from_polyline_sharp(points, false, &vec![s; points.len()], 30.0)
        .expect("a two-point chain with a setback per point")
}

/// The end at the origin of a chain running in along +x, its plane facing +x.
fn end_at_origin() -> StoppedEnd {
    StoppedEnd {
        chain: 0,
        end_vertex: 1,
        vertex: Vector3::new(0.0, 0.0, 0.0),
        normal: Vector3::new(1.0, 0.0, 0.0),
    }
}

#[test]
fn a_plane_cut_far_from_the_end_reads_where_the_band_first_reaches_it() {
    // The side's one triangle meets the plane x = 0 only from y = 5 to y = 6, so the chain's band
    // (unit setback round the chain ending at the origin) reaches the cut at size 5. The other
    // chain, at y = 9, has covered the whole cut by size 4: from size 5 on the chain's stretch is
    // covered, so the room is 5 - not 0, which is what reading "covered" at every size gave.
    let positions = vec![
        Vector3::new(-1.0, 5.0, 0.0),
        Vector3::new(1.0, 5.0, 0.0),
        Vector3::new(0.0, 6.0, 0.0),
    ];
    let indices = vec![(0, 1, 2)];
    let tags = vec![1];
    let own = [Vector3::new(-1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0)];
    let other = [Vector3::new(0.0, 9.0, 0.0), Vector3::new(0.5, 9.0, 0.0)];
    let field_at = |k: usize, s: f64| field(if k == 0 { &own } else { &other }, s);
    let read = stopped_end_room(
        &positions,
        &indices,
        &tags,
        1,
        &end_at_origin(),
        &[1],
        &field_at,
        1.0,
        10.0,
    )
    .expect("no budget runs out")
    .expect("the other chain covers the cut");
    assert!(
        read.room <= 5.0 && read.room > 5.0 - 1e-9,
        "the room is where the band first reaches the cut: {}",
        read.room
    );
    assert_eq!(read.covered_by, 1, "the other chain covers it");
}

#[test]
fn the_chains_far_end_clip_bounds_its_stretch_of_the_plane() {
    // The chain runs from (-1, 0, 0) to the origin; its far end is clipped on the plane through
    // (-1, 0, 0) facing (-1, 1, 0), which cuts the end plane x = 0 above y = 1. The side's
    // triangle meets x = 0 from the origin to y = 3. The other chain, a short segment at x = 2,
    // covers the plane up to y = 0.1 + sqrt(s^2 - 4) at size s: the chain's stretch, below y = 1
    // by its far clip, is covered by size sqrt(0.81 + 4). Read on the raw band alone, the stretch
    // would run on to y = s and stay uncovered at every size up to 3.
    let positions = vec![
        Vector3::new(-1.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 3.0, 0.0),
    ];
    let indices = vec![(0, 1, 2)];
    let tags = vec![1];
    let own = [Vector3::new(-1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0)];
    let other = [Vector3::new(2.0, -0.1, 0.0), Vector3::new(2.0, 0.1, 0.0)];
    let far_normal = Vector3::new(-1.0, 1.0, 0.0).get_normal();
    let field_at = |k: usize, s: f64| {
        if k == 0 {
            let mut clipped = field(&own, s);
            clipped.end_clips = Some(vec![
                ChainFieldEndClip {
                    vertex: own[0],
                    normal: far_normal,
                    parameter: 0.0,
                    tilt: 0.0,
                },
                ChainFieldEndClip {
                    vertex: own[1],
                    normal: Vector3::new(1.0, 0.0, 0.0),
                    parameter: 1.0,
                    tilt: 0.0,
                },
            ]);
            clipped
        } else {
            field(&other, s)
        }
    };
    let read = stopped_end_room(
        &positions,
        &indices,
        &tags,
        1,
        &end_at_origin(),
        &[1],
        &field_at,
        1.0,
        3.0,
    )
    .expect("no budget runs out")
    .expect("the other chain covers the stretch");
    let covered = (0.81f64 + 4.0).sqrt();
    assert!(
        read.room <= covered && read.room > 2.0,
        "the room stops where the stretch inside the far clip is covered, {covered}: {}",
        read.room
    );
}
