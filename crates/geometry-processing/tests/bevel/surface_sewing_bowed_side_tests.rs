//! A side with several of a cut's vertices standing along it is no longer the straight line
//! its two corners draw, so the ones still to be taken in are read against the side AS DRAWN; and
//! two faces meeting along one stretch come out of the sewing with the same sequence along it.

use std::collections::HashSet;

use solidmesh::bevel::surface::{surface_split_sewing, SurfaceVertexPool};
use solidmesh_math::{Vector3, Vector3Float};

/// A hundred across, so the ruler this pool measures with is a thousandth.
fn a_pool_a_hundred_across() -> SurfaceVertexPool {
    SurfaceVertexPool::new(&[
        Vector3Float::new(0.0, 0.0, 0.0),
        Vector3Float::new(57.735, 57.735, 57.735),
    ])
}

#[test]
fn a_split_standing_on_the_side_as_the_other_splits_drew_it_is_taken_in() {
    let mut pool = a_pool_a_hundred_across();
    assert!(
        (pool.near_enough() - 1e-3).abs() <= 1e-5,
        "the distances below are chosen against a thousandth"
    );

    let left = pool.add(Vector3::new(0.0, 0.0, 0.0));
    let right = pool.add(Vector3::new(10.0, 0.0, 0.0));
    let up = pool.add(Vector3::new(0.0, 10.0, 0.0));

    let mut faces = vec![(left, right, up)];

    let split_from = pool.vertices().len() as i32;

    let near = pool.add(Vector3::new(3.0, 0.0009, 0.0)); // 0.9 of the ruler off the chord
    let bowed = pool.add(Vector3::new(5.0, 0.0012, 0.0)); // 1.2 of it, and 0.3 off the line of the other two
    let far = pool.add(Vector3::new(7.0, 0.0009, 0.0)); // 0.9 again

    surface_split_sewing::sew_in_the_splits_since_unchecked(&mut pool, &mut faces, split_from);

    let carried: HashSet<i32> = faces.iter().flat_map(|f| [f.0, f.1, f.2]).collect();

    assert!(
        carried.contains(&near) && carried.contains(&far),
        "the two within a ruler of the chord are taken in as they always were"
    );
    assert!(
        carried.contains(&bowed),
        "and so is the one between them, which stands on the side as those two drew it"
    );
    assert_eq!(
        faces.len(),
        4,
        "a side cut in four is four triangles off the corner across from it"
    );
}

#[test]
fn two_faces_on_one_stretch_get_the_same_sequence_along_it() {
    let mut pool = a_pool_a_hundred_across();

    let near = pool.add(Vector3::new(0.0, 0.0, 0.0)); // the stretch's near end, a corner of both
    let far = pool.add(Vector3::new(0.006, 0.0, 9.0)); // the first face's far corner, up the line
    let past = pool.add(Vector3::new(0.002, 0.0, 3.0)); // the second face's corner, just past the row
    let one_way = pool.add(Vector3::new(-0.5, 0.0, 9.0)); // the first face's third corner
    let the_other = pool.add(Vector3::new(0.5, 0.0, 1.5)); // the second face's third corner

    let split_from = pool.vertices().len() as i32;

    // The row: four stations on the line through the stretch.
    let row = [
        pool.add(Vector3::new(0.0, 0.0, 0.5)),
        pool.add(Vector3::new(0.0, 0.0, 1.0)),
        pool.add(Vector3::new(0.0, 0.0, 1.5)),
        pool.add(Vector3::new(0.0, 0.0, 2.0)),
    ];

    // And a split further up the first face's side, beyond the row.
    pool.add(Vector3::new(0.006 * 4.0 / 9.0, 0.0, 4.0));

    let mut faces = vec![(near, far, one_way), (past, near, the_other)];

    surface_split_sewing::sew_in_the_splits_since_unchecked(&mut pool, &mut faces, split_from);

    let along_the_stretch = [near, row[0], row[1], row[2], row[3]];
    for step in 0..along_the_stretch.len() - 1 {
        let (first, second) = (along_the_stretch[step], along_the_stretch[step + 1]);
        assert_eq!(
            uses_of(&faces, first, second),
            2,
            "both faces have to walk the step from {first} to {second} - one each way"
        );
    }
}

/// How many of the triangles use the edge between two vertices, either way round.
fn uses_of(faces: &[(i32, i32, i32)], first: i32, second: i32) -> i32 {
    let mut uses = 0;
    for &(v0, v1, v2) in faces {
        for (from, to) in [(v0, v1), (v1, v2), (v2, v0)] {
            if (from == first && to == second) || (from == second && to == first) {
                uses += 1;
            }
        }
    }
    uses
}
