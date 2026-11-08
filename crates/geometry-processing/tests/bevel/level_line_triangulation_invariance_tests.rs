//! The level line must build the same part whatever the diagonals across the source's flat faces
//! are (docs/modeling/bevel_plan.md, "Triangulation-independent level line"). Each saved pick is
//! built on its source as stored and again with every flippable coplanar diagonal flipped
//! (`coplanar_diagonal_flip`) - what a source rebuilt by the kernel's batch boolean route hands
//! over - and the two builds must agree. Parameter rows are loops over the same-named test.

use std::collections::HashSet;
use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::band_gap_in_turn::{
    build_open_chains_in_turn, takes_open_chains_in_turn,
};
use solidmesh::bevel::level_line::open_gap_fill::ends_site;
use solidmesh::bevel::level_line::{gap_fill, GapFillRefusal, GapFillResult, WorkBudget};

use crate::coplanar_diagonal_flip;
use crate::end_face_builds::feature_of;
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;
use crate::sweep_mesh_measures;

#[test]
fn flipping_the_flat_faces_diagonals_builds_the_same_part() {
    for name in [
        "fifteen-mm-f4-c27-r005",
        "chain-and-face-face-group-saved",
        "fifteen-mm-x20-rim-c27-r005-batch",
        "outer-faces-r04",
        "outer-faces-r04-batch",
        "outer-faces-r04-x305-v215",
    ] {
        flipping_builds_the_same_part(name);
    }
}

fn flipping_builds_the_same_part(name: &str) {
    let fixture = Rc::new(LevelLineFixture::load(name));
    let mut chain_edges: HashSet<(i32, i32)> = HashSet::new();
    for (k, chain) in fixture.chains.iter().enumerate() {
        let count = if fixture.closed[k] {
            chain.len()
        } else {
            chain.len() - 1
        };
        for i in 0..count {
            chain_edges.insert(key(chain[i], chain[(i + 1) % chain.len()]));
        }
    }

    let (flipped_indices, flips) = coplanar_diagonal_flip::flip_all(
        &fixture.positions,
        &fixture.indices,
        |a, b| fixture.is_feature_edge(a, b) || chain_edges.contains(&key(a, b)),
        1e-9,
    );
    assert!(flips > 0, "{name}: the source has no diagonal to flip");
    let flipped_volume = sweep_mesh_measures::signed_volume(&fixture.positions, &flipped_indices);
    let stored_volume = sweep_mesh_measures::signed_volume(&fixture.positions, &fixture.indices);
    assert!(
        (flipped_volume - stored_volume).abs() <= 1e-9,
        "{name}: flipping a coplanar diagonal does not change the part"
    );

    let stored = build(&fixture, &fixture.indices);
    let flipped = build(&fixture, &flipped_indices);

    for (which, build) in [
        ("stored".to_string(), &stored),
        (format!("flipped ({flips} diagonals)"), &flipped),
    ] {
        assert!(build.said.is_none(), "{name} {which}: {:?}", build.said);
        assert!(
            build.closed,
            "{name} {which}: not closed (route {})",
            build.route
        );
        assert!(
            !build.crosses,
            "{name} {which}: crosses itself (route {})",
            build.route
        );
    }

    assert_eq!(
        flipped.route, stored.route,
        "{name}: the flip changed the route"
    );

    // The two builds are the same part, so their volumes agree to the part's own float noise.
    let part_volume = stored_volume.abs();
    assert!(
        (flipped.volume - stored.volume).abs() <= 1e-6 * part_volume,
        "{name}: stored {:?}, flipped {:?}, part {part_volume:?}",
        stored.volume,
        flipped.volume
    );
}

/// What one build left: the mesh and the readings the two builds are compared on.
#[derive(Clone)]
struct Built {
    route: String,
    said: Option<String>,
    closed: bool,
    crosses: bool,
    volume: f64,
}

/// The pick built through the kernel calls the app's level-line route makes, in its order
/// (BevelSurfacePartBuilder.LevelLine): the pick-wide cut and fill, and where that refuses, does
/// not close or crosses itself, the chains in turn. The route names which of the two answered.
fn build(fixture: &Rc<LevelLineFixture>, indices: &[(i32, i32, i32)]) -> Built {
    let profile = profile_of(fixture);
    let options = BandGapBuildOptions {
        chord_tolerance: 0.0,
        snap: 1e-9,
        known_room: fixture.known_room,
        budget: Some(Rc::new(
            WorkBudget::new(WorkBudget::default_limit(indices.len() as i32)).expect("a limit"),
        )),
        seam_epsilon: fixture.seam_epsilon,
        sharp_turn_degrees: fixture.sharp_turn_degrees,
        fillet_segments: fixture.segments,
    };
    let gap = band_gap_entry_points::build_pick(
        &fixture.positions,
        indices,
        &fixture.chains,
        &fixture.closed,
        Some(feature_of(fixture)),
        fixture.size,
        profile,
        &options,
    )
    .expect("a valid pick");

    let mut pick_wide_said = format!("pick-wide: band {:?}", gap.refusal);
    if gap.built() {
        let in_turn = takes_open_chains_in_turn(Some(&gap), &fixture.closed);
        let filled = if in_turn {
            in_turn_fill(fixture, indices, Some(gap.room))
        } else {
            gap_fill::fill(&gap, profile, fixture.segments).expect("the fill runs")
        };
        let route = if in_turn { "in turn" } else { "pick-wide" };
        let answer = read(route, &filled, fixture, indices);
        if answer.said.is_none() && answer.closed && !answer.crosses {
            return answer;
        }

        pick_wide_said = format!(
            "{route}: {}",
            answer.said.clone().unwrap_or_else(|| if answer.closed {
                "crosses itself".to_string()
            } else {
                "not closed".to_string()
            })
        );
    }

    if fixture.chains.len() < 2 {
        return Built {
            route: "refused".to_string(),
            said: Some(pick_wide_said),
            closed: false,
            crosses: false,
            volume: 0.0,
        };
    }

    let in_turn = read(
        "in turn",
        &in_turn_fill(fixture, indices, None),
        fixture,
        indices,
    );
    let said = in_turn
        .said
        .as_ref()
        .map(|said| format!("{pick_wide_said}; {said}"));
    Built { said, ..in_turn }
}

fn in_turn_fill(
    fixture: &Rc<LevelLineFixture>,
    indices: &[(i32, i32, i32)],
    room: Option<f64>,
) -> GapFillResult {
    build_open_chains_in_turn(
        &fixture.positions,
        indices,
        &fixture.chains,
        Some(feature_of(fixture)),
        fixture.size,
        profile_of(fixture),
        fixture.segments,
        room,
        1e-9,
        fixture.seam_epsilon,
        fixture.sharp_turn_degrees,
        None,
        Some(&fixture.closed),
    )
    .expect("the build runs")
}

fn read(
    route: &str,
    filled: &GapFillResult,
    fixture: &LevelLineFixture,
    indices: &[(i32, i32, i32)],
) -> Built {
    let Some(mesh) = filled
        .mesh
        .as_ref()
        .filter(|_| filled.refusal == GapFillRefusal::None)
    else {
        let at = match (&filled.mesh, filled.offending_vertex >= 0) {
            (Some(mesh), true) => format!("{:?}", mesh.borrow().position(filled.offending_vertex)),
            _ => "-".to_string(),
        };
        return Built {
            route: route.to_string(),
            said: Some(format!(
                "{route}: chain {} {:?} at {at} ({:?})",
                filled.offending_chain,
                filled.refusal,
                ends_site()
            )),
            closed: false,
            crosses: false,
            volume: 0.0,
        };
    };

    let mesh = mesh.borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    let closed = sweep_mesh_measures::is_closed_and_consistent(mesh.vertices(), &kept);
    let crosses = sweep_mesh_measures::intersects_itself_where_the_source_did_not(
        mesh.vertices(),
        &kept,
        &fixture.positions,
        indices,
    );
    Built {
        route: route.to_string(),
        said: None,
        closed,
        crosses,
        volume: sweep_mesh_measures::signed_volume(mesh.vertices(), &kept),
    }
}

fn key(a: i32, b: i32) -> (i32, i32) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}
