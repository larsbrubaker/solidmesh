//! Flow rung 2, the unclipped strip - on the rib synthetic (`strip_clip_rib_tests.rs`) with the
//! rib's opposite-sense creases bounding the room only at their far ends (`GrowingCreaseRuns`), the
//! top side's rail runs on in the top's plane at the setback where the band has passed the rib's
//! creases.

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::end_face_fill_try;
use solidmesh::bevel::level_line::fill_closure;
use solidmesh::bevel::level_line::virtual_rail;
use solidmesh::bevel::level_line::{
    BandGapRefusal, BandGapResult, BandProfile, GapFillRefusal, GapLoop, LevelLineMesh,
};
use solidmesh_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::open_edges;
use crate::strip_clip_rib_meshes;

fn build(positions: &[Vector3], indices: &[(i32, i32, i32)]) -> BandGapResult {
    band_gap_entry_points::build(
        positions,
        indices,
        &strip_clip_rib_meshes::CHAIN,
        false,
        Some(feature_edges_over(positions, indices, 20.0)),
        5.0,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
    )
    .expect("the rib builds or refuses by name")
}

fn least(l: &GapLoop) -> f64 {
    l.parameters.iter().copied().fold(f64::INFINITY, f64::min)
}

/// The tops rail runs on virtually past the rib.
#[test]
fn the_tops_rail_runs_on_virtually_past_the_rib() {
    let positions = strip_clip_rib_meshes::positions(None);
    let indices = strip_clip_rib_meshes::ribbed();
    let gap = build(&positions, &indices);

    let mut runs: Vec<GapLoop> = gap.loops.iter().filter(|l| l.side == 1).cloned().collect();
    // A stable sort, so loops with equal keys keep their order.
    runs.sort_by(|a, b| least(a).total_cmp(&least(b)));
    let mut minted = Vec::new();
    let joined = if runs.len() == 2 {
        let mesh = gap.mesh.as_ref().expect("a built gap's mesh");
        virtual_rail::join(
            &mut mesh.borrow_mut(),
            &gap.chains[0],
            &runs[0],
            &runs[1],
            &mut minted,
        )
        .expect("the join spends no budget it lacks")
    } else {
        None
    };

    assert_eq!(gap.refusal, BandGapRefusal::None);
    assert_eq!(runs.len(), 2);
    let joined = joined.expect("the top's two runs join");
    assert!(!minted.is_empty());

    // Each virtual point stands on the top's plane z = 10, at the setback (5) from the chain
    // y = 0, z = 10, past the rib's tip (x in the stretch between the two runs' ends).
    {
        let mesh = gap.mesh.as_ref().expect("a mesh").borrow();
        for &v in &minted {
            let p = mesh.position(v);
            assert!((p.z - 10.0).abs() <= 1e-9, "{p:?}");
            assert!((p.y - 5.0).abs() <= 1e-9, "{p:?}");
            assert!(p.x > 9.6 - 1e-9, "{p:?}");
            assert!(p.x < 10.4 + 1e-9, "{p:?}");
        }
    }

    // The joined rail is one run, monotone in the chain parameter.
    let parameters = &joined.parameters;
    for i in 1..parameters.len() {
        assert!(parameters[i] > parameters[i - 1]);
    }

    assert_eq!(
        joined.vertices.len(),
        runs[0].vertices.len() + minted.len() + runs[1].vertices.len()
    );

    // The rows are laid WHOLE across the stretch, then clipped on the rib's planes and capped
    // (StripClip): the fill closes with no open edge left anywhere.
    let mut sides_two = gap.loops.iter().filter(|l| l.side == 2);
    let side_two = sides_two.next().expect("one side-2 loop").clone();
    assert!(sides_two.next().is_none(), "exactly one side-2 loop");
    let rows = end_face_fill_try::try_end_faces(
        &gap,
        joined,
        side_two,
        BandProfile::Fillet,
        16,
        None,
        None,
    )
    .expect("the end faces fill or refuse by name")
    .expect("the runs chain into one cycle");
    assert_eq!(rows.refusal, GapFillRefusal::None);
    assert_eq!(
        open_edges(&gap.mesh.as_ref().expect("a mesh").borrow()).len(),
        0
    );

    // Nothing moves where no opposite-sense crease is reached: the plain box never leaves two runs.
    let plain_indices = strip_clip_rib_meshes::plain();
    let plain = build(&positions, &plain_indices);
    assert_eq!(plain.loops.iter().filter(|l| l.side == 1).count(), 1);
}

/// The closure check sees an unclosed hole.
#[test]
fn a_fill_answering_an_edge_nothing_kept_stands_on_is_refused() {
    let positions = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    ];
    let mesh = LevelLineMesh::from_triangles(&positions, &[(0, 1, 2)], None).expect("a mesh");
    let open = fill_closure::first_open_edge_of(&mesh, &[(1, 0), (2, 1), (0, 2)], &[0]);
    assert!(open.0 >= 0);
}
