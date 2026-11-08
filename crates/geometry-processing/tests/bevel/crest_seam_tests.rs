//! What the band build leaves on the narrow box the crest reference proves its answer on
//! (x ±5, y ±2, z ±5) when its two top edges are rounded together at 3, past the room where their
//! bands meet: the gap the crest fill (`gap_fill_crest`) starts from.

use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{build_pick, BandGapBuildOptions};
use solidmesh::bevel::level_line::{BandGapRefusal, BandProfile, RoomReadingKind};
use solidmesh_math::Vector3;

fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

/// The narrow box as the kernel reads it: corners 0-3 at z = -5, 4-7 at z = 5, two triangles a
/// face, wound outward.
fn narrow_box() -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    let positions = vec![
        v(-5.0, -2.0, -5.0),
        v(5.0, -2.0, -5.0),
        v(5.0, 2.0, -5.0),
        v(-5.0, 2.0, -5.0),
        v(-5.0, -2.0, 5.0),
        v(5.0, -2.0, 5.0),
        v(5.0, 2.0, 5.0),
        v(-5.0, 2.0, 5.0),
    ];
    let indices = vec![
        (0, 2, 1),
        (0, 3, 2),
        (4, 5, 6),
        (4, 6, 7),
        (0, 1, 5),
        (0, 5, 4),
        (1, 2, 6),
        (1, 6, 5),
        (2, 3, 7),
        (2, 7, 6),
        (3, 0, 4),
        (3, 4, 7),
    ];
    (positions, indices)
}

/// What the band build leaves on the box's two top edges at 3: refused at step 5 by the room
/// one chain's band meets the other's at, and - handed a room past 3 so it goes on - the loops
/// the fill would be given.
#[test]
fn the_band_build_leaves_one_gap_of_four_open_loops_on_the_box_s_top_edges_at_3() {
    let (positions, indices) = narrow_box();
    let diagonals = [(0, 2), (4, 6), (0, 5), (1, 6), (2, 7), (3, 4)];
    let feature = Rc::new(move |a: i32, b: i32| !diagonals.contains(&(a.min(b), a.max(b))));
    let chains = vec![vec![6, 7], vec![4, 5]];
    let build = |known_room: Option<f64>| {
        build_pick(
            &positions,
            &indices,
            &chains,
            &[false, false],
            Some(feature.clone()),
            3.0,
            BandProfile::Fillet,
            &BandGapBuildOptions {
                known_room,
                fillet_segments: 8,
                ..BandGapBuildOptions::default()
            },
        )
        .expect("the build runs")
    };

    // Refused at step 5 by the room where one band's front meets the other's: half the face,
    // less the margin. The mesh is cut by then but nothing is removed yet.
    let refused = build(None);
    assert_eq!(refused.refusal, BandGapRefusal::Size);
    assert_eq!(refused.room_set_by.kind, RoomReadingKind::ChainToChain);
    assert!((refused.room - 2.0).abs() < 1e-8, "{}", refused.room);
    assert!(refused.removed_triangles.is_empty());

    // Handed a room past 3, the build still refuses, because the top face (both chains' side 1,
    // one group) keeps no loop of its own: the band swallows it whole. What is left is ONE gap
    // bounded by four open loops: each chain's far rail at z = 2 on its side face, owned by it
    // alone, and on each end face the band's cut from one far rail's end to the other's, owned by
    // both chains (mixed: the owner changes where the cut crosses y = 0).
    let handed = build(Some(10.0));
    assert_eq!(handed.refusal, BandGapRefusal::Size);
    let mesh = handed.mesh.as_ref().expect("the cut mesh").borrow();
    let rows: Vec<(i32, i32, bool, Vector3, Vector3)> = handed
        .loops
        .iter()
        .map(|gap| {
            let first = mesh.position(gap.vertices[0]);
            let last = mesh.position(*gap.vertices.last().expect("a loop has vertices"));
            (
                gap.group_tag,
                gap.owners.len() as i32,
                gap.is_closed,
                first,
                last,
            )
        })
        .collect();
    assert_eq!(
        rows,
        [
            (2, 1, false, v(-5.0, 2.0, 2.0), v(5.0, 2.0, 2.0)),
            (3, 1, false, v(5.0, -2.0, 2.0), v(-5.0, -2.0, 2.0)),
            (4, 2, false, v(5.0, 2.0, 2.0), v(5.0, -2.0, 2.0)),
            (5, 2, false, v(-5.0, -2.0, 2.0), v(-5.0, 2.0, 2.0)),
        ]
    );
    let top_face_loops: Vec<i32> = handed
        .side_loops
        .iter()
        .filter(|row| row.group_tag == 1)
        .map(|row| row.loops)
        .collect();
    assert_eq!(top_face_loops, [0, 0]);

    // The end face's cut is the band's level line there - points 3 from the nearer edge end - so
    // it peaks near z = 5 - sqrt(5) at y = 0, well below the crest at 2 + sqrt(8): the end holes
    // the fill closes run from this cut up to the two rounds' end rows and their crest vertex.
    let end_cut_top = handed.loops[2]
        .vertices
        .iter()
        .map(|&i| mesh.position(i).z)
        .fold(f64::MIN, f64::max);
    assert!(end_cut_top < 5.0 - 5.0_f64.sqrt(), "{end_cut_top}");
}
