//! A square crease end's band corner: c27 of "15mm Bevel Top Edge" / "Center Flows Through" ends
//! square on the slanted face y - x = -98.5 at (-10, -108.5, 20), on the concave crease that face
//! makes with the side below. The end vertex is the band's corner on that plane and reads on the
//! line, as a stopped end's does. Built alone, the slanted face is c27's own end region and is
//! left whole for the end-face fill, which closes it flat. Built with the face's own rim (f4) it is
//! that member's side, and the band cuts into it where c27's own value crosses 0, so the crease's
//! stretch from the corner to the end vertex lies inside the band, not on a rail.

use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::BandGapBuildOptions;
use solidmesh::bevel::level_line::{gap_fill, GapFillRefusal};
use solidmesh_math::Vector3;

use crate::end_face_builds::{as_saved, build_chains, default_budget, FlatEnds};
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;
use crate::round_mouth_fillet_crossings::crossing_pairs;
use crate::sweep_mesh_measures;

/// c27's end vertex.
const END: Vector3 = Vector3 {
    x: -10.0,
    y: -108.5,
    z: 20.0,
};

/// Builds `chains` of the fixture pick-wide at its size and fills it: closed and one shell.
/// Answers the loops' vertex positions and the filled mesh's crossing pairs.
fn builds_closed(name: &str, chains: Option<&[usize]>) -> (Vec<Vec<Vector3>>, i32) {
    let _flat_ends = FlatEnds::set(true);
    let fixture = Rc::new(LevelLineFixture::load(name));
    let all: Vec<usize> = (0..fixture.chains.len()).collect();
    let gap = build_chains(
        &fixture,
        chains.unwrap_or(&all),
        fixture.size,
        &BandGapBuildOptions {
            budget: Some(default_budget(&fixture, false)),
            ..as_saved(&fixture, fixture.known_room)
        },
    );
    assert!(gap.built(), "{name}: band {:?}", gap.refusal);
    let loops: Vec<Vec<Vector3>> = {
        let mesh = gap.mesh.as_ref().expect("a built band").borrow();
        gap.loops
            .iter()
            .map(|l| l.vertices.iter().map(|&v| mesh.position(v)).collect())
            .collect()
    };

    let filled = gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).expect("the fill");
    assert_eq!(filled.refusal, GapFillRefusal::None, "{name}");
    let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        mesh.vertices(),
        &kept
    ));
    assert_eq!(
        sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
        1
    );
    (loops, crossing_pairs(&mesh))
}

/// Whether some loop runs between the end vertex and a point of the concave crease below it
/// (x = -10, y = -108.5): the crease's stretch in the end face's plane laid as a rail.
fn a_loop_runs_down_the_crease(loops: &[Vec<Vector3>]) -> bool {
    let on_the_crease =
        |p: Vector3| p != END && (p.x - END.x).abs() < 1e-5 && (p.y - END.y).abs() < 1e-5;
    loops.iter().any(|l| {
        l.windows(2).any(|pair| {
            (pair[0] == END && on_the_crease(pair[1])) || (pair[1] == END && on_the_crease(pair[0]))
        })
    })
}

#[test]
fn the_square_end_alone_closes_its_own_end_region_flat() {
    let (_, crossings) = builds_closed("center-flows-through-c9-c27-r005", Some(&[1]));
    assert_eq!(crossings, 0);
}

#[test]
fn the_square_end_beside_its_end_faces_rim_cuts_into_that_side() {
    let (loops, _) = builds_closed("fifteen-mm-f4-c27-r005", None);
    assert!(
        !a_loop_runs_down_the_crease(&loops),
        "the crease's stretch to the end vertex is a rail"
    );
}
