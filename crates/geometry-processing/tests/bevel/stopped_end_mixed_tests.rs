//! A round STOPPED at one end and cut into its end face at the other: "Top Face Should stil
//! bevel.mcx"'s chain at (1, -44, 0.06) as the app hands it, the arch's first two facet edges
//! along the flat wall, on the bottom face at one end and stopped where the arch's next facet
//! carries the crease on. With stopped ends on, the stopped end is cut flat on its plane by the
//! end-face fill (`end_face_fill_try.rs`), its section closed in the plane, rather than tapered
//! into the end vertex as a nose, whose rows folded over each other.

use std::rc::Rc;

use solidmesh::bevel::level_line::end_face_section_row::where_the_traces_meet;
use solidmesh::bevel::level_line::open_gap_fill::ends_site;
use solidmesh_math::Vector3;

use solidmesh::bevel::level_line::{
    gap_fill, turn_stopped_ends_on, BandGapResult, GapFillRefusal, LevelLineMesh,
};

use crate::end_face_builds::{as_saved, build_chains};
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;
use crate::level_line_test_geometry::{signed_volume, signed_volume_of};
use crate::round_mouth_fillet_crossings::crossing_pairs;
use crate::sweep_mesh_measures;

/// The stopped end beside an end face fills closed, one shell, uncrossed, and only takes material.
#[test]
fn a_round_stopped_at_one_end_and_on_its_face_at_the_other_fills_uncrossed() {
    let _stopped = turn_stopped_ends_on();
    let fixture = Rc::new(LevelLineFixture::load("top-face-stopped-mixed-end"));
    let chains = [0];
    // The room the app reads first and hands the build.
    let room = build_chains(&fixture, &chains, fixture.size, &as_saved(&fixture, None)).room;
    for size in [0.05, 0.1, 0.25, 1.8, 3.2, 3.59] {
        let gap = build_chains(&fixture, &chains, size, &as_saved(&fixture, Some(room)));
        assert!(gap.built(), "{:?} at {size} of room {room}", gap.refusal);

        let filled =
            gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).expect("the fill runs");
        assert_eq!(
            filled.refusal,
            GapFillRefusal::None,
            "{:?} at {size}",
            ends_site()
        );

        let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
        let kept = sweep_mesh_measures::kept(&mesh);
        assert!(
            sweep_mesh_measures::is_closed_and_consistent(mesh.vertices(), &kept),
            "closed at {size}"
        );
        assert_eq!(
            sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
            1
        );
        assert_eq!(
            crossing_pairs(&mesh),
            0,
            "the fill crosses itself at {size}"
        );

        // The flat end shows the round's whole section: on a side that is one plane across the
        // band, the section of a round of radius `size` between faces turned `turn` apart, read
        // in the leaning end plane, laid with chords (which add about 2.3% at 8 segments). From
        // 3.2 the band on the arch's side crosses its folds, whose trace bends away from the
        // round, adding up to about 12% more flat end; never less than the section.
        let cap = in_plane_area(&gap, &mesh, &kept);
        let section = section_area(&fixture, &gap, size);
        let most = if size <= 1.8 { 1.03 } else { 1.15 };
        assert!(
            cap >= section && cap <= section * most,
            "the flat end {cap} against the section {section} at {size}"
        );

        let before = signed_volume_of(&fixture.positions, &fixture.indices);
        let volume = signed_volume(&mesh);
        assert!(
            volume < before && volume > 0.0,
            "{volume} against {before} at {size}"
        );
    }
}

/// The area the fill lays in the stopped end's plane: every kept triangle whose three corners
/// stand on that plane.
fn in_plane_area(gap: &BandGapResult, mesh: &LevelLineMesh, kept: &[(i32, i32, i32)]) -> f64 {
    let chain = &gap.chains[0];
    let bound = chain.end_ray_bounds.1.expect("the far end is stopped");
    assert!(bound.stopped, "the far end is stopped");
    let normal = bound.plane_normal.get_normal();
    let points = chain.points();
    let end = points[points.len() - 1];
    let on = |i: i32| ((mesh.position(i) - end).dot(normal)).abs() <= 1e-9;
    kept.iter()
        .filter(|&&(a, b, c)| on(a) && on(b) && on(c))
        .map(|&(a, b, c)| {
            let (a, b, c) = (mesh.position(a), mesh.position(b), mesh.position(c));
            (b - a).cross(c - a).length() / 2.0
        })
        .sum()
}

/// The section of a round of radius `size` on the chain's last edge, its two faces taken as
/// planes, in the stopped end's plane: the round's square section R^2 (tan(turn / 2) - turn / 2)
/// over the cosine between the chain and the plane's normal.
fn section_area(fixture: &LevelLineFixture, gap: &BandGapResult, size: f64) -> f64 {
    let chain = &fixture.chains[0];
    let (a, b) = (chain[chain.len() - 2], chain[chain.len() - 1]);
    let normals: Vec<Vector3> = fixture
        .indices
        .iter()
        .filter(|t| [t.0, t.1, t.2].contains(&a) && [t.0, t.1, t.2].contains(&b))
        .map(|t| {
            let p = |i: i32| fixture.positions[i as usize];
            (p(t.1) - p(t.0)).cross(p(t.2) - p(t.0)).get_normal()
        })
        .collect();
    assert_eq!(normals.len(), 2, "the chain's last edge has two faces");
    let half_turn = normals[0].dot(normals[1]).clamp(-1.0, 1.0).acos() / 2.0;
    let along = (fixture.positions[b as usize] - fixture.positions[a as usize]).get_normal();
    let bound = gap.chains[0]
        .end_ray_bounds
        .1
        .expect("the far end is stopped");
    let lean = along.dot(bound.plane_normal.get_normal()).abs();
    size * size * (half_turn.tan() - half_turn) / lean
}

/// A folded side's trace bends, and the row's control is where the two traces at the rail corners
/// meet, between the end vertex and the rail corners.
#[test]
fn a_folded_sides_row_is_drawn_about_where_the_traces_meet() {
    let v = Vector3::new(0.0, 0.0, 0.0);
    let p = (Vector3::new(0.0, -3.0, 0.0), Vector3::new(0.0, 1.0, 0.0));
    let q = (
        Vector3::new(-3.0, 0.0, 0.0),
        Vector3::new(1.0, -0.3, 0.0).get_normal(),
    );
    let meeting = where_the_traces_meet(v, p, q, 1e-9).expect("the traces meet near the end");
    assert!(
        (meeting - Vector3::new(0.0, -0.9, 0.0)).length() < 1e-12,
        "{meeting:?}"
    );
}

/// Two nearly parallel traces (a fold just past a rail corner) meet far past the rail corners;
/// drawn about that crossing the row would bow far out of the section, so the end vertex is kept.
#[test]
fn nearly_parallel_traces_keep_the_end_vertex() {
    let v = Vector3::new(0.0, 0.0, 0.0);
    let p = (Vector3::new(0.0, -3.0, 0.0), Vector3::new(0.0, 1.0, 0.0));
    let q = (
        Vector3::new(-3.0, 0.2, 0.0),
        Vector3::new(1e-3, 1.0, 0.0).get_normal(),
    );
    assert_eq!(where_the_traces_meet(v, p, q, 1e-9), None);
}
