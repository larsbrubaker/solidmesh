//! `RunOutField` read through `BandGroupFields` on the Top Center Face nose - with run-outs on, the
//! band cut carries the ridge round down the unpicked lower crease to where the control's blend
//! dies on it (4.5846 along), instead of stopping 1.95 along where the side flood stops. Parameter
//! rows are loops over the same-named test.

use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use indexmap::IndexMap;
use solidmesh::bevel::level_line::band_gap_entry_points::BandGapBuildOptions;
use solidmesh::bevel::level_line::run_out_field::RunOutField;
use solidmesh::bevel::level_line::{
    triangle_edges, turn_run_outs_on, BandGapResult, ChainDistanceField, CreaseRunOut, GapLoop,
    LevelLineWork, ScalarField,
};
use solidmesh_math::Vector3;

use crate::end_face_builds::{as_saved, build_chains, default_budget};
use crate::level_line_fixture::LevelLineFixture;

/// The control's P* less the regression tests' chord tolerance.
const REACH_FLOOR: f64 = 4.5846 - 0.05;

/// The lower crease's far end in the fixture: both ridge ends run down to it.
const LOWER_CREASE_END: usize = 5;

#[test]
fn the_nose_band_runs_out_along_the_lower_crease_from_both_ridge_ends() {
    let fixture = Rc::new(LevelLineFixture::load("top-center-face-nose-r1"));
    let gap = {
        // Run-outs are turned on for the build only: the gate rides on the
        // build's budget, so the budget is put in force first and the scope turns it on there.
        let budget = default_budget(&fixture, false);
        let _work = LevelLineWork::begin(Some(Rc::clone(&budget)));
        let _run_outs = turn_run_outs_on();
        let options = BandGapBuildOptions {
            budget: Some(budget),
            ..as_saved(&fixture, fixture.known_room)
        };
        let all: Vec<usize> = (0..fixture.chains.len()).collect();
        build_chains(&fixture, &all, fixture.size, &options)
    };

    assert!(gap.built(), "band {:?}", gap.refusal);

    {
        let mesh = gap.mesh.as_ref().expect("the working mesh").borrow();
        for end in [1usize, 9] {
            let start = fixture.positions[end];
            let down = (fixture.positions[LOWER_CREASE_END] - start).get_normal();
            let crease_length = (fixture.positions[LOWER_CREASE_END] - start).length();
            let reach = gap
                .loops
                .iter()
                .flat_map(|l| l.vertices.iter())
                .map(|&v| mesh.position(v) - start)
                .filter(|&from| (from - (down * from.dot(down))).length() < 1e-4)
                .map(|from| from.dot(down))
                .filter(|&along| along < crease_length - 1e-6)
                .fold(None, |best: Option<f64>, along| {
                    Some(best.map_or(along, |b| b.max(along)))
                })
                .unwrap_or(0.0);
            assert!(
                reach >= REACH_FLOOR,
                "from ridge end v{end} the cut reaches {reach:.4} along the unpicked lower crease"
            );
        }
    }

    // A side's loop may come as several open runs, split where its owner changes at a pick corner
    // (the nose's side 2 does without any run-out); the runs must still join end to end into ONE
    // loop.
    let mut sides: IndexMap<i32, Vec<&GapLoop>> = IndexMap::new();
    for l in &gap.loops {
        sides.entry(l.side).or_default().push(l);
    }
    for (side, runs) in &sides {
        let described: Vec<String> = runs
            .iter()
            .map(|l| {
                format!(
                    "{} vertices{}",
                    l.vertices.len(),
                    if l.is_closed { " closed" } else { " open" }
                )
            })
            .collect();
        assert_eq!(
            joined_loops(runs),
            1,
            "side {side}: {}",
            described.join(" | ")
        );
    }

    let (chi, pick_chi, boundary_loops, over_shared) = band_euler(&fixture, &gap);
    assert_eq!(chi, pick_chi, "band V - E + F {chi}, pick V - E {pick_chi}");
    assert_eq!(boundary_loops, 2 - pick_chi);
    assert_eq!(over_shared, 0);
}

/// The wedge's zero set is the taper: on the crease it is inside up to P* and outside past it.
#[test]
fn the_field_tapers_to_nothing_at_the_run_outs_end() {
    let fixture = LevelLineFixture::load("top-center-face-nose-r1");
    for run_out in CreaseRunOut::find(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        &fixture.closed,
        &|a, b| fixture.is_feature_edge(a, b),
        fixture.size,
        16,
    ) {
        let field = RunOutField::of(&run_out);
        let on = |along: f64| run_out.corner_point + (run_out.crease_direction * along);
        let value_at = |p: Vector3| field.value_at(p).expect("no budget");
        assert!(field.slope > 0.0);
        assert!(value_at(on(run_out.reach)).abs() <= 1e-9);
        assert!(value_at(on(run_out.reach * 0.5)) < 0.0);
        assert!((value_at(on(run_out.reach + 0.1)) - 0.1).abs() <= 1e-9);
        assert!(value_at(on(-0.1)) > 0.0);

        // The least along a segment crossing the crease is found where the segment crosses it.
        let across = run_out
            .crease_direction
            .cross(Vector3::new(0.0, 0.0, 1.0))
            .get_normal();

        // Never wider than the widest wall contact (0.66 here): the bare taper stood 2.97 wide at
        // the corner and carved the faces round it past the neighbouring chain's rail
        // (FillLeavesThePart).
        let widest = run_out
            .wall_contact
            .iter()
            .map(|&p| run_out.distance_to_crease_line(p))
            .fold(f64::NEG_INFINITY, f64::max);
        assert!((field.width - widest).abs() <= 1e-12);
        assert!((value_at(on(0.5) + (across * (widest + 0.1))) - 0.1).abs() <= 1e-9);
        let least = field
            .min_along_segment(on(2.0) - across, on(2.0) + (across * 0.5))
            .expect("no budget");
        assert!((least.value - value_at(on(2.0))).abs() <= 1e-9);
    }
}

/// The wedge follows the contact's concave profile: a hull-vertex sample of it reads on the zero
/// set, and a step further off the crease reads outside.
#[test]
fn the_wedge_follows_the_contact_profile() {
    // Concave: rising to 0.6 at s = 2, then falling to P* at 4.
    let profile = [(0.5, 0.3), (1.0, 0.45), (2.0, 0.6), (3.0, 0.4)];
    let field =
        RunOutField::from_profile(Vector3::ZERO, Vector3::new(1.0, 0.0, 0.0), 4.0, &profile);
    for (s, off) in profile {
        assert!(
            field.read(Vector3::new(s, off, 0.0)) <= 1e-9,
            "sample ({s}, {off})"
        );
        assert!(
            field.read(Vector3::new(s, off + 0.01, 0.0)) > 0.0,
            "just off ({s}, {off})"
        );
    }
}

/// No contact sample short of P*: a bare taper, not an index past the hull's start.
#[test]
fn a_profile_with_no_sample_before_p_star_is_a_bare_cone() {
    let field = RunOutField::from_profile(
        Vector3::ZERO,
        Vector3::new(1.0, 0.0, 0.0),
        2.0,
        &[(2.0, 0.0), (2.5, 0.1)],
    );
    assert!(field.read(Vector3::new(1.0, 0.0, 0.0)) <= 0.0);
    assert!(field.read(Vector3::new(1.0, 0.5, 0.0)).is_finite());
}

/// Two samples at one s make no vertical hull edge: every reading stays finite.
#[test]
fn two_samples_at_one_distance_along_read_finite() {
    let field = RunOutField::from_profile(
        Vector3::ZERO,
        Vector3::new(1.0, 0.0, 0.0),
        4.0,
        &[(0.0, 0.2), (0.0, 0.5), (2.0, 0.6)],
    );
    for p in [
        Vector3::new(0.0, 0.3, 0.0),
        Vector3::new(1.0, 0.5, 0.0),
        Vector3::new(3.0, 0.1, 0.0),
        Vector3::new(-1.0, 2.0, 0.0),
    ] {
        let value = field.read(p);
        assert!(value.is_finite(), "at {p:?} the field reads {value}");
    }

    assert!(field.read(Vector3::new(0.0, 0.5, 0.0)) <= 1e-9);
}

/// The cheap "no" never turns away a point the field reads at or below the margin - probed just
/// outside the wide rim at the corner cap, where the cone's flare passes any ball round the solid
/// alone.
#[test]
fn nothing_reaches_below_never_denies_a_reading_at_the_margin() {
    for slope in [1.0, 0.3, 0.1] {
        let field = RunOutField::cone(
            Vector3::ZERO,
            Vector3::new(1.0, 0.0, 0.0),
            1.0,
            slope,
            f64::INFINITY,
        );
        let outward = Vector3::new(slope, 1.0, 0.0).get_normal();
        for along in [-0.02, 0.0, 0.1, 0.5, 0.9, 1.0] {
            for gap in [0.01, 0.05, 0.2, 1.0] {
                let p = Vector3::new(along, slope * (1.0 - along), 0.0) + (outward * gap);
                let value = field.read(p);
                assert!(
                    !field.nothing_reaches_below(p, p, value + 1e-9),
                    "slope {slope}, rim along {along}, {gap} out: the field reads {value:.4}"
                );
            }
        }
    }
}

/// Union-find's find with path halving over `parent`.
fn find(parent: &mut HashMap<i32, i32>, mut v: i32) -> i32 {
    while parent[&v] != v {
        let grand = parent[&parent[&v]];
        parent.insert(v, grand);
        v = grand;
    }

    v
}

/// How many connected loops a side's runs make, joined where they share a vertex.
fn joined_loops(runs: &[&GapLoop]) -> usize {
    let mut parent: HashMap<i32, i32> = HashMap::new();
    for run in runs {
        for &v in &run.vertices {
            let first = run.vertices[0];
            parent.entry(v).or_insert(v);
            parent.entry(first).or_insert(first);
            let a = find(&mut parent, v);
            let b = find(&mut parent, first);
            parent.insert(a, b);
        }
    }

    let keys: Vec<i32> = parent.keys().copied().collect();
    let roots: HashSet<i32> = keys.into_iter().map(|k| find(&mut parent, k)).collect();
    roots.len()
}

/// The deleted band's V - E + F, the pick graph's V - E, the band's boundary loop count and its
/// edges shared by more than two band triangles - invariant 1 of `BevelInvariantsMeasureTests`,
/// for a pick of one connected component.
fn band_euler(fixture: &LevelLineFixture, gap: &BandGapResult) -> (i32, i32, i32, i32) {
    let pick_vertices: HashSet<i32> = fixture.chains.iter().flatten().copied().collect();
    let mut pick_edges = HashSet::new();
    for (k, chain) in fixture.chains.iter().enumerate() {
        let count = if fixture.closed[k] {
            chain.len()
        } else {
            chain.len() - 1
        };
        for i in 0..count {
            pick_edges.insert(triangle_edges::key(chain[i], chain[(i + 1) % chain.len()]));
        }
    }

    let mesh = gap.mesh.as_ref().expect("the working mesh").borrow();
    let mut edge_use: HashMap<(i32, i32), i32> = HashMap::new();
    let mut vertices = HashSet::new();
    for &t in &gap.removed_triangles {
        let face = mesh.triangles()[t as usize];
        for c in 0..3 {
            vertices.insert(face.corner(c));
            let key = triangle_edges::key(face.corner(c), face.corner((c + 1) % 3));
            *edge_use.entry(key).or_insert(0) += 1;
        }
    }

    // Boundary edges joined at shared vertices: each connected run is one loop of a band with no
    // pinch.
    let mut parent: HashMap<i32, i32> = HashMap::new();
    for (&(a, b), _) in edge_use.iter().filter(|(_, &n)| n == 1) {
        parent.entry(a).or_insert(a);
        parent.entry(b).or_insert(b);
        let ra = find(&mut parent, a);
        let rb = find(&mut parent, b);
        parent.insert(ra, rb);
    }

    let keys: Vec<i32> = parent.keys().copied().collect();
    let loops: HashSet<i32> = keys.into_iter().map(|k| find(&mut parent, k)).collect();
    let chi = vertices.len() as i32 - edge_use.len() as i32 + gap.removed_triangles.len() as i32;
    (
        chi,
        pick_vertices.len() as i32 - pick_edges.len() as i32,
        loops.len() as i32,
        edge_use.values().filter(|&&n| n > 2).count() as i32,
    )
}
