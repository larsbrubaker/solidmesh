//! The face regions carried past a build (`side_regions_after_a_build::carried`) read on meshes
//! made by hand, and the regions a pick built a chain at a time is handed where the ball would not
//! read them: a re-tiled needle on a gently curved face keeps its face's region however far its
//! centroid stands off its parents, the fill's own triangles are the round, and a list of the
//! wrong length with the switch off builds exactly as no list.

use geometry_processing::bevel::level_line::band_gap_in_turn::build_open_chains_in_turn_with_side_regions;
use geometry_processing::bevel::level_line::band_gap_rounds_first::kept;
use geometry_processing::bevel::level_line::gap_fill::DEFAULT_FILLET_SEGMENTS;
use geometry_processing::bevel::level_line::side_regions_after_a_build::carried;
use geometry_processing::bevel::level_line::{BandProfile, GapFillResult, LevelLineMesh};
use geometry_processing::bevel::rolling_ball::rolling_ball_routing::{
    reset_to_the_default, set_use_rolling_ball,
};
use vector_math::Vector3;

use super::band_route_tests::{a_box, creases};

/// A gently curved face, a quad twisted 0.02 out of its plane, as two triangles (region 4), and
/// one far triangle of another face (region 9). After a build its quad is re-tiled across the
/// other diagonal, as a needle repair or a strip re-tiling does from the vertices already there,
/// and a fill triangle stands 0.5 off it.
#[test]
fn a_retiled_needle_keeps_its_faces_region_and_the_fill_is_the_round() {
    let before_at = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 0.02),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(5.0, 0.0, 0.0),
        Vector3::new(6.0, 0.0, 0.0),
        Vector3::new(5.0, 1.0, 0.0),
    ];
    let before = vec![(0, 1, 2), (0, 2, 3), (4, 5, 6)];
    let labels = [4, 4, 9];

    let mut after_at = before_at.clone();
    after_at.push(Vector3::new(0.5, 0.5, 0.5));
    // The re-tiling: by hand its centroids stand about 0.0047 off the parents' planes, far past
    // any rounding of a position.
    let after = vec![(0, 1, 3), (1, 2, 3), (4, 5, 6), (0, 1, 7)];
    let mesh = LevelLineMesh::from_triangles(&after_at, &after, None).expect("a mesh");
    let regions = carried((&before_at, &before), &labels, &mesh, &[3], None)
        .expect("nothing cancels")
        .expect("the regions carry");
    assert_eq!(regions, [4, 4, 9, 10]);

    // A list that labels another mesh carries nothing.
    assert_eq!(
        carried((&before_at, &before), &labels[..2], &mesh, &[3], None).expect("no cancel"),
        None
    );
}

/// The cube's top front and bottom back edges built a chain at a time with the switch off: a
/// list one region short (which would name no sides) is never read, and the result is the same
/// mesh, vertex for vertex and triangle for triangle, as a build handed none.
#[test]
fn side_regions_with_the_switch_off_build_exactly_as_none() {
    let (positions, triangles) = a_box(Vector3::new(-1.0, -1.0, -1.0), Vector3::new(1.0, 1.0, 1.0));
    let short: Vec<u32> = (0..triangles.len() as u32 - 1).map(|t| t / 2).collect();
    let build = |side_regions: Option<&[u32]>| -> GapFillResult {
        set_use_rolling_ball(false);
        let result = build_open_chains_in_turn_with_side_regions(
            &positions,
            &triangles,
            &[vec![4, 5], vec![3, 2]],
            Some(creases(&positions, &triangles)),
            0.25,
            BandProfile::Fillet,
            DEFAULT_FILLET_SEGMENTS,
            None,
            1e-9,
            0.0,
            f64::INFINITY,
            None,
            None,
            side_regions,
        )
        .expect("the pick's arguments are good");
        reset_to_the_default();
        result
    };
    let (none, with_short) = (build(None), build(Some(&short)));
    assert_eq!(none.refusal, with_short.refusal);
    let mesh_of = |r: &GapFillResult| {
        let mesh = r.mesh.as_ref().expect("a mesh").borrow();
        (mesh.vertices.clone(), kept(&mesh))
    };
    assert_eq!(mesh_of(&none), mesh_of(&with_short));
}
