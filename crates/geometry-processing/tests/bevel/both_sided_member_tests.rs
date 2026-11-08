//! A face rim with an open edge starting at one of its corners - the box's top rim 4-5-6-7 and the
//! vertical edge 5-1. The edge's two sides are the front and right faces, separate regions, but
//! both are the rim's outer side, so they fall in one side group: the edge is a BOTH-SIDED member
//! of that group, not a chain that fails to divide its sides. Each test's cases, and their
//! combinations, run as loops inside it.

use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::band_side_groups::group_sides;
use solidmesh::bevel::level_line::band_side_tagging;
use solidmesh::bevel::level_line::chain_on_the_mesh;
use solidmesh::bevel::level_line::triangle_edges;
use solidmesh::bevel::level_line::{
    corner_fill, gap_fill, BandGapRefusal, BandGapResult, BandProfile, GapFillRefusal,
    LevelLineMesh, WorkBudget,
};
use solidmesh_math::Vector3;

use crate::gap_fill_tests::shell_count;
use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::{
    cube_solid, open_edges, signed_volume, signed_volume_of, triangle_area,
};
use crate::several_chains_fixtures::FOLD_DEGREES;

fn rim_and_edge() -> Vec<Vec<i32>> {
    vec![vec![4, 5, 6, 7], vec![5, 1]]
}

const CLOSED: [bool; 2] = [true, false];

fn chain_edges() -> Vec<Vec<(i32, i32)>> {
    rim_and_edge()
        .iter()
        .enumerate()
        .map(|(k, c)| chain_on_the_mesh::chain_edges(c, CLOSED[k]))
        .collect()
}

/// The band gap on the box at `size` and `profile`: no chord tolerance, a 1e-9 snap, the room
/// measured.
fn build(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    size: f64,
    profile: BandProfile,
    options: &BandGapBuildOptions,
) -> BandGapResult {
    band_gap_entry_points::build_pick(
        positions,
        indices,
        chains,
        closed,
        Some(feature_edges_over(positions, indices, FOLD_DEGREES)),
        size,
        profile,
        options,
    )
    .expect("the pick builds or refuses by name")
}

/// An edge from a rim corner is both-sided in the rim's outer group.
#[test]
fn an_edge_from_a_rim_corner_is_both_sided_in_the_rims_outer_group() {
    let (positions, indices) = cube_solid(1.0);
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);

    let (_, groups, undivided) =
        group_sides(&indices, &chain_edges(), &*feature, true, None, None, None)
            .expect("the sides group");

    assert_eq!(
        undivided, -1,
        "no path on the box joins the front face to the right face"
    );
    let with_edge: Vec<_> = groups.iter().filter(|g| g.members.contains(&1)).collect();
    assert_eq!(with_edge.len(), 1);
    let outer = with_edge[0];
    let mut members = outer.members.clone();
    members.sort();
    assert_eq!(members, vec![0, 1]);
    assert_eq!(outer.both_sided, vec![1]);
    assert!(groups
        .iter()
        .filter(|g| g.tag != outer.tag)
        .all(|g| g.both_sided.is_empty()));
}

/// Both-sided members are
/// read (the default), so the band is cut: the rim's inner side leaves its one closed loop, the
/// outer group ONE open loop running up the edge's one rail, round the rim and down its other rail
/// (owned by the edge, then the rim, then the edge), and the edge's free end its end region's one
/// run. With the switch off the pick refuses by the old name.
#[test]
fn the_band_around_a_rim_corner_edge_leaves_one_loop_down_one_rail_and_up_the_other() {
    let (positions, indices) = cube_solid(1.0);

    // Turning both-sided members off travels with the build's budget, so it reaches no other
    // build.
    let off =
        WorkBudget::new(WorkBudget::default_limit(indices.len() as i32)).expect("a positive limit");
    off.both_sided_members_off.set(true);
    let refused = build(
        &positions,
        &indices,
        &rim_and_edge(),
        &CLOSED,
        0.2,
        BandProfile::Chamfer,
        &BandGapBuildOptions {
            budget: Some(Rc::new(off)),
            ..BandGapBuildOptions::default()
        },
    );
    assert_eq!(refused.refusal, BandGapRefusal::ChainDoesNotDivideItsSides);
    assert!(band_side_tagging::both_sided_members_build());

    let built = build(
        &positions,
        &indices,
        &rim_and_edge(),
        &CLOSED,
        0.2,
        BandProfile::Chamfer,
        &BandGapBuildOptions::default(),
    );
    assert_eq!(built.refusal, BandGapRefusal::None);

    // The edge's free end at vertex 1 stands on end faces, and their END REGION leaves a run of
    // its own round that end (BandGroup.EndOf); the claim here is about the side loops.
    let side_loops: Vec<_> = built
        .loops
        .iter()
        .filter(|l| !built.end_region_tags.contains(&l.group_tag))
        .collect();
    assert_eq!(side_loops.len(), 2);
    assert_eq!(built.loops.len() - side_loops.len(), 1);

    let inner: Vec<_> = side_loops.iter().filter(|l| l.is_closed).collect();
    assert_eq!(inner.len(), 1);
    assert!(inner[0]
        .vertex_owners
        .as_ref()
        .expect("the walk read the owners")
        .iter()
        .all(|&o| o == 0));

    let outer: Vec<_> = side_loops.iter().filter(|l| !l.is_closed).collect();
    assert_eq!(outer.len(), 1);
    let owners = outer[0]
        .vertex_owners
        .as_ref()
        .expect("the walk read the owners");
    let runs: Vec<i32> = owners
        .iter()
        .enumerate()
        .filter(|&(i, &o)| i == 0 || owners[i - 1] != o)
        .map(|(_, &o)| o)
        .collect();
    assert_eq!(runs, vec![1, 0, 1]);
}

/// With the rim picked starting at the corner,
/// the corner fill takes the pick - the corner is an end on the rim's interior - and the fillet
/// fills.
#[test]
fn the_fillet_fills_when_the_rim_starts_at_the_corner() {
    let (positions, indices) = cube_solid(1.0);
    let from_the_corner = vec![vec![5, 6, 7, 4], vec![5, 1]];

    let built = build(
        &positions,
        &indices,
        &from_the_corner,
        &CLOSED,
        0.2,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
    );
    assert!(corner_fill::takes(Some(&built), BandProfile::Fillet));
    let fill = gap_fill::fill(&built, BandProfile::Fillet, 4).expect("the fill runs");
    assert_eq!(fill.refusal, GapFillRefusal::None);
}

/// The rim picked from ANY of its vertices fills
/// the same - the band rotates a closed chain to start at a corner before its fields are built.
#[test]
fn the_fillet_fills_whichever_vertex_the_rim_starts_at() {
    for rotation in 0..4 {
        let (positions, indices) = cube_solid(1.0);
        let rim = [4, 5, 6, 7];
        let rotated_rim: Vec<i32> = rim[rotation..]
            .iter()
            .chain(rim[..rotation].iter())
            .copied()
            .collect();
        let rotated = vec![rotated_rim, vec![5, 1]];

        let built = build(
            &positions,
            &indices,
            &rotated,
            &CLOSED,
            0.2,
            BandProfile::Fillet,
            &BandGapBuildOptions::default(),
        );
        assert_eq!(built.refusal, BandGapRefusal::None, "rotation {rotation}");
        let fill = gap_fill::fill(&built, BandProfile::Fillet, 4).expect("the fill runs");
        assert_eq!(fill.refusal, GapFillRefusal::None, "rotation {rotation}");
    }
}

/// The box rim + corner edge filled at a spread
/// of the pick's room, fillet and chamfer: the answer is closed, one shell, has no zero-area
/// triangle, and every edge here is convex so the volume only goes down.
#[test]
fn the_rim_and_corner_edge_fill_closed_across_the_room() {
    for profile in [BandProfile::Fillet, BandProfile::Chamfer] {
        for fraction in [0.1, 0.5, 0.9, 1.0] {
            let row = format!("{profile:?} at {fraction}");
            let (positions, indices) = cube_solid(1.0);
            let before = signed_volume_of(&positions, &indices);
            let defaults = BandGapBuildOptions::default();

            let room = build(
                &positions,
                &indices,
                &rim_and_edge(),
                &CLOSED,
                0.01,
                profile,
                &defaults,
            )
            .room;
            assert!(room > 0.0, "{row}");
            let built = build(
                &positions,
                &indices,
                &rim_and_edge(),
                &CLOSED,
                room * fraction,
                profile,
                &defaults,
            );
            assert_eq!(built.refusal, BandGapRefusal::None, "{row}");
            let fill = gap_fill::fill(&built, profile, 4).expect("the fill runs");
            assert_eq!(fill.refusal, GapFillRefusal::None, "{row}");
            let mesh = fill.mesh.as_ref().expect("a filled mesh").borrow();

            assert!(open_edges(&mesh).is_empty(), "{row}");
            assert_eq!(shell_count(&mesh), 1, "{row}");

            // At the full room the rim's own inner loop closes to a point at the top face's
            // centre and leaves a speck there (the rim alone does the same, pinned here); the
            // corner edge may add none.
            let specks = specks(&mesh);
            if fraction < 1.0 {
                assert_eq!(specks, 0, "{row}");
            } else {
                let rim_alone = build(
                    &positions,
                    &indices,
                    &[rim_and_edge()[0].clone()],
                    &[true],
                    room * fraction,
                    profile,
                    &defaults,
                );
                let alone = gap_fill::fill(&rim_alone, profile, 4).expect("the fill runs");
                let alone_mesh = alone.mesh.as_ref().expect("a filled mesh").borrow();
                assert_eq!(specks, self::specks(&alone_mesh), "{row}");
            }

            let after = signed_volume(&mesh);
            assert!(after > 0.0, "{row}");
            assert!(after < before, "{row}: {after} against {before}");
        }
    }
}

/// Live triangles of (near) zero area.
fn specks(mesh: &LevelLineMesh) -> usize {
    (0..mesh.triangle_count())
        .filter(|&t| {
            !mesh.is_removed(t) && triangle_area(mesh, mesh.triangles()[t as usize]) <= 1e-12
        })
        .count()
}

/// Which side of the edge a point is on,
/// read from the geometry, agrees with the side `chain_on_the_mesh::sides_of` names.
#[test]
fn a_both_sided_members_loop_side_is_read_from_the_geometry() {
    let (positions, indices) = cube_solid(1.0);
    let standing = triangle_edges::standing_map(&indices);
    let edge = chain_edges()[1].clone();
    let (left, right) = chain_on_the_mesh::sides_of(&indices, &standing, edge[0].0, edge[0].1)
        .expect("the edge stands on two triangles");

    // The front face is y = -1, the right face x = +1; a point a little into each, halfway down
    // the edge.
    let on_front = Vector3::new(0.9, -1.0, 0.0);
    let on_right = Vector3::new(1.0, -0.9, 0.0);
    let is_front = |triangle: i32| {
        let t = indices[triangle as usize];
        [t.0, t.1, t.2]
            .iter()
            .all(|&v| positions[v as usize].y == -1.0)
    };
    let front_side = if is_front(left) { 1 } else { 2 };
    assert!(is_front(left) || is_front(right));

    assert_eq!(
        band_side_tagging::side_of(&positions, &indices, &standing, &edge, on_front)
            .expect("the edge stands on two triangles"),
        front_side
    );
    assert_eq!(
        band_side_tagging::side_of(&positions, &indices, &standing, &edge, on_right)
            .expect("the edge stands on two triangles"),
        3 - front_side
    );
}
