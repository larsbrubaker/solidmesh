//! Separate rim picks tests: a run of a tessellated rim picked ONE EDGE AT A TIME
//! against the same edges picked as one selection, and what the surface construction leaves at the
//! two ends of a run that stops part way round.
//!
//! Two questions. The first is self-consistency: N separate groups go through
//! `surface_shared_edge_network::link_together` and the chain join, and what comes out has to be
//! the run one whole-run pick would have made, down to the vertex; picked all the way round, the
//! same joint has to come back as one CLOSED loop. The second is what an OPEN run does at its two
//! free ends: the surface construction stops SQUARE in the plane through the picked vertex and says
//! so (`SurfaceRunStopsSquareAtBothEnds`). What is pinned is read off closed forms - the band is
//! the size asked for at every station, it never crosses the end plane, and the unpicked neighbour
//! is untouched. The fixture is a 24 sided cylinder; a test over several cases loops. The fixture
//! helpers are `pub` for `surface_cone_rim_tests.rs` and the routed-accuracy readings.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::surface::{
    surface_mesh_closure, surface_run_stops_short, SurfaceBevelPlan,
};
use geometry_processing::bevel::BevelAdvisoryCode;
use geometry_processing::topology::{
    bevel_edge_group_resolver, pick_interpretations, BevelEdgeGroup, BevelEdgeGroupResolution,
    BevelSelectionKind, FaceGroup, FeatureEdge, FeatureEdgeSet,
};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{Vector2, Vector3};

use crate::mesh_measurements::shell_count;
use crate::mesh_surface_distance;
use crate::surface_test_build::{try_bevel, try_bevel_resolutions, TryBevelOptions};
use crate::topology_test_meshes::create_cylinder;

/// The circumradius of the cylinder's polygon, which is where its rim vertices sit.
pub const CIRCUMRADIUS: f64 = 9.0;

pub const SIDES: i32 = 24;

pub const HEIGHT: f64 = 3.0;

pub const SIZE: f64 = 0.5;

/// The cylinder is centred on the origin, so the top rim is half its height up.
pub const CAP_Z: f64 = HEIGHT / 2.0;

/// How far each wall FACET's plane stands from the axis - the polygon's apothem.
pub fn apothem() -> f64 {
    CIRCUMRADIUS * (std::f64::consts::PI / SIDES as f64).cos()
}

/// Single-edge picks joined back into the run one whole-run pick would have made, to the vertex.
/// The comparison is on the VERTEX SETS and not on the indices: the two builds walk their chains
/// from different seeds.
#[test]
fn separate_picks_land_the_same_surface_as_one_group_of_the_same_edges() {
    for (round, edge_count) in [
        (true, 2),
        (false, 2),
        (true, 3),
        (false, 3),
        (true, 24),
        (false, 24),
    ] {
        let part = create_cylinder(CIRCUMRADIUS, HEIGHT, SIDES);
        let features = Arc::new(FeatureEdgeSet::create(&part));
        let run: Vec<FeatureEdge> = the_top_rim(&features)
            .into_iter()
            .take(edge_count)
            .collect();

        let (separately, apart) = built(&part, &each_on_its_own(&features, &run), round);
        let (together, _) = built_resolutions(&part, &[one_selection_of(&features, &run)], round);

        assert_eq!(
            separately.vertices.len(),
            together.vertices.len(),
            "the join has to publish the same run either way it was picked, so neither build may \
             carry a seam vertex the other does not"
        );

        for (from, to) in [(&separately, &together), (&together, &separately)] {
            let (worst, at) = farthest_vertex_from(from, to);

            assert!(
                worst < 1e-6,
                "{edge_count} separate picks and one selection of the same {edge_count} edges are \
                 the same run, and {at:?} is where they disagree"
            );
        }

        let (closed, open_at) = surface_mesh_closure::is_closed(&separately);
        assert!(
            closed,
            "a bevel of a closed solid stays closed, and this one is open at {open_at:?}"
        );
        assert!(separately.is_manifold());
        assert_eq!(shell_count(&separately), 1);

        assert_eq!(
            apart.len(),
            1,
            "the pieces are linked and joined into ONE run, not left as one plan each"
        );
    }
}

/// All 24 picked separately come back as the CLOSED rim, with no end to stop square at and so
/// nothing said about one.
#[test]
fn the_whole_rim_picked_facet_by_facet_comes_back_closed() {
    for round in [true, false] {
        let part = create_cylinder(CIRCUMRADIUS, HEIGHT, SIDES);
        let features = Arc::new(FeatureEdgeSet::create(&part));
        let rim = the_top_rim(&features);

        let (built, plans) = built(&part, &each_on_its_own(&features, &rim), round);

        assert!(
            plans[0].is_closed,
            "a whole rim picked a facet at a time is still a whole rim, and the join has to close \
             the wrap pair rather than leave the first and last ends standing at one vertex"
        );
        assert_eq!(plans[0].edges.len(), SIDES as usize);
        assert!(!plans[0].start_runs_out);
        assert!(!plans[0].end_runs_out);

        assert!(
            surface_run_stops_short::read(Some(&plans), SIDES, None).is_none(),
            "nothing stopped short of anything - there is no free end in a closed loop to warn \
             about, and a warning here would be one the user could not act on"
        );

        let (closed, open_at) = surface_mesh_closure::is_closed(&built);
        assert!(closed, "open at {open_at:?}");
        assert_eq!(shell_count(&built), 1);
    }
}

/// The free end of a run that stops part way round: the band ends in the plane square to the
/// picked edge, the facet beyond it is untouched right up to the shared corner, and the group is
/// told. The boolean construction tapers onto the next facet instead; what is asserted here is the
/// surface construction's own answer read off closed forms.
#[test]
fn a_free_end_stops_square_and_leaves_the_facet_beyond_it_alone() {
    for (round, edge_count) in [(true, 2), (false, 2), (true, 3), (false, 3)] {
        let part = create_cylinder(CIRCUMRADIUS, HEIGHT, SIDES);
        let features = Arc::new(FeatureEdgeSet::create(&part));
        let rim = the_top_rim(&features);
        let run: Vec<FeatureEdge> = rim.iter().take(edge_count).copied().collect();

        let (built, plans) = built(&part, &each_on_its_own(&features, &run), round);

        let said = surface_run_stops_short::read(Some(&plans), edge_count as i32, None)
            .expect("the run stops short, and says so");
        assert_eq!(said.len(), 1);
        assert_eq!(
            said[0].code(),
            BevelAdvisoryCode::SurfaceRunStopsSquareAtBothEnds,
            "both ends of the run stand on a rim that carries on past them, and one run's two \
             square ends are one sentence"
        );

        let ends = free_ends(&run);
        assert_eq!(ends.len(), 2);

        for &end in &ends {
            let onward = away_from_the_run(&rim, &run, end);

            // Nothing was taken off the unpicked facet: the part's surface still contains the
            // corner the pick stopped at and every station out along the neighbour. The boolean's
            // taper reads 0.2 rounded and 0.29 chamfered at the first of these.
            for along in [0.0, 0.1, 0.25, 0.6] {
                assert!(
                    mesh_surface_distance::from(&built, end + (onward * along)) < 1e-6,
                    "{along} mm out along the facet nobody picked is untouched material, so the \
                     built surface still runs through it"
                );
            }
        }

        // And the band on the picked side never crosses either end plane. Measured against BOTH
        // ends at once: every band vertex is inside the run, so it is on the run's side of both.
        for vertex in the_bands_own_vertices(&part, &built, &run) {
            for &end in &ends {
                let inward = into_the_run(&run, end);

                assert!(
                    (vertex - end).dot(inward) > -1e-6,
                    "the run stops SQUARE at {end:?}, so no part of the band may stand beyond that \
                     plane - {vertex:?} does"
                );
            }
        }
    }
}

/// The band is the profile the size asked for, everywhere: every vertex the bevel adds sits on its
/// facet's 45 degree chamfer plane or on the circle of the radius, and the part has retreated from
/// the middle of every picked edge by exactly what that profile takes off a right angle. The
/// tolerance is 2e-6 because mesh vertices are single precision.
#[test]
fn every_vertex_the_band_adds_is_on_the_profile_the_size_asks_for() {
    for (round, edge_count) in [
        (true, 2),
        (false, 2),
        (true, 3),
        (false, 3),
        (true, 24),
        (false, 24),
    ] {
        let part = create_cylinder(CIRCUMRADIUS, HEIGHT, SIDES);
        let features = Arc::new(FeatureEdgeSet::create(&part));
        let run: Vec<FeatureEdge> = the_top_rim(&features)
            .into_iter()
            .take(edge_count)
            .collect();

        let (built, _) = built(&part, &each_on_its_own(&features, &run), round);

        let facets: Vec<Vector3> = run.iter().map(facet_normal_under).collect();
        let added = the_bands_own_vertices(&part, &built, &run);

        assert!(
            added.len() > edge_count,
            "a band of this many edges cannot be built out of fewer corners than it has edges"
        );

        for &vertex in &added {
            let miss = facets
                .iter()
                .map(|&normal| {
                    if round {
                        (distance_to_the_balls_centre_line(vertex, normal) - SIZE).abs()
                    } else {
                        (sideways(vertex, normal) + vertex.z - (apothem() + CAP_Z - SIZE)).abs()
                    }
                })
                .fold(f64::INFINITY, f64::min);

            assert!(
                miss < 2e-6,
                "{vertex:?} is on the band, so it is on some picked facet's {} - it misses by \
                 {miss:.9}",
                if round {
                    "rolling ball circle"
                } else {
                    "45 degree plane"
                }
            );
        }

        let expected = if round {
            SIZE * (2f64.sqrt() - 1.0)
        } else {
            SIZE / 2f64.sqrt()
        };

        for edge in &run {
            let retreat = mesh_surface_distance::from(&built, (edge.start + edge.end) / 2.0);

            assert!(
                (retreat - expected).abs() <= 1e-6,
                "the middle of every picked edge carries the full section, the free ends included \
                 - a band that ran out towards an end would read less here ({retreat} against \
                 {expected})"
            );
        }
    }
}

/// The cylinder's top rim, walked round from the negative x axis.
pub fn the_top_rim(features: &FeatureEdgeSet) -> Vec<FeatureEdge> {
    let mut rim: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| on_the_top_rim(edge.start) && on_the_top_rim(edge.end))
        .copied()
        .collect();

    // A stable sort, so edges at equal angles keep their mesh order.
    rim.sort_by(|a, b| {
        angle_of(a)
            .partial_cmp(&angle_of(b))
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    rim
}

fn angle_of(edge: &FeatureEdge) -> f64 {
    (edge.start.y + edge.end.y).atan2(edge.start.x + edge.end.x)
}

pub fn on_the_top_rim(point: Vector3) -> bool {
    (point.z - CAP_Z).abs() < 1e-6
        && (Vector2::new(point.x, point.y).length() - CIRCUMRADIUS).abs() < 1e-6
}

/// One single-edge Edge pick per edge - what picking them one at a time leaves stored.
pub fn each_on_its_own(features: &FeatureEdgeSet, run: &[FeatureEdge]) -> Vec<BevelEdgeGroup> {
    run.iter()
        .map(|edge| {
            bevel_edge_group_resolver::create_from_edge(
                features,
                pick_interpretations::index_of_mesh_edge(features, edge.mesh_edge_index),
                BevelSelectionKind::Edge,
                FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
            )
            .expect("every rim edge is a feature edge")
        })
        .collect()
}

/// The same edges as ONE selection - a resolution carrying all of them, which is the shape a
/// single pick that read the whole run hands the kernel (a succeeded `BevelEdgeGroupResolution`).
pub fn one_selection_of(
    features: &Arc<FeatureEdgeSet>,
    run: &[FeatureEdge],
) -> BevelEdgeGroupResolution {
    BevelEdgeGroupResolution {
        group: each_on_its_own(features, run).swap_remove(0),
        features: features.clone(),
        failure_reason: None,
        seed_edge_index: -1,
        edges: run.to_vec(),
        chain: None,
        face_group: None,
    }
}

fn options(round: bool) -> TryBevelOptions {
    TryBevelOptions {
        round_profile: round,
        ..TryBevelOptions::default()
    }
}

/// The build from groups, which has to succeed, and its plans.
pub fn built(
    part: &Mesh,
    groups: &[BevelEdgeGroup],
    round: bool,
) -> (Mesh, Vec<Rc<SurfaceBevelPlan>>) {
    let outcome = try_bevel(
        part,
        groups,
        SIZE,
        if round { 8 } else { 1 },
        &CancelToken::new(),
        options(round),
    );

    finish(outcome)
}

/// The build from resolutions, which has to succeed, and its plans.
pub fn built_resolutions(
    part: &Mesh,
    resolutions: &[BevelEdgeGroupResolution],
    round: bool,
) -> (Mesh, Vec<Rc<SurfaceBevelPlan>>) {
    let outcome = try_bevel_resolutions(
        part,
        resolutions,
        SIZE,
        if round { 8 } else { 1 },
        &CancelToken::new(),
        options(round),
    );

    finish(outcome)
}

fn finish(
    outcome: crate::surface_test_build::TryBevelOutcome,
) -> (Mesh, Vec<Rc<SurfaceBevelPlan>>) {
    let code = outcome.reason().map(|reason| reason.code());

    match outcome.result {
        Ok(built) => (built.mesh, outcome.plans_built),
        Err(_) => panic!("the build refused: {code:?}"),
    }
}

/// The vertex of `from` that is farthest from any vertex of the other.
pub fn farthest_vertex_from(from: &Mesh, to: &Mesh) -> (f64, Vector3) {
    let theirs: Vec<Vector3> = to
        .vertices
        .iter()
        .map(|&vertex| Vector3::from_vector3_float(vertex))
        .collect();
    let mut worst = 0.0;
    let mut at = Vector3::ZERO;

    for mine in from
        .vertices
        .iter()
        .map(|&vertex| Vector3::from_vector3_float(vertex))
    {
        let nearest = theirs
            .iter()
            .map(|&other| (other - mine).length())
            .fold(f64::INFINITY, f64::min);

        if nearest > worst {
            worst = nearest;
            at = mine;
        }
    }

    (worst, at)
}

/// The two ends of the run - the rim vertices only one picked edge stands on.
pub fn free_ends(run: &[FeatureEdge]) -> Vec<Vector3> {
    // The groups keep the order their first point was met; rounding to six places is banker's
    // rounding (ties to even).
    let key = |point: Vector3| {
        (
            (point.x * 1e6).round_ties_even() as i64,
            (point.y * 1e6).round_ties_even() as i64,
            (point.z * 1e6).round_ties_even() as i64,
        )
    };
    let mut groups: Vec<((i64, i64, i64), Vector3, usize)> = Vec::new();

    for point in run.iter().flat_map(|edge| [edge.start, edge.end]) {
        match groups.iter_mut().find(|group| group.0 == key(point)) {
            Some(group) => group.2 += 1,
            None => groups.push((key(point), point, 1)),
        }
    }

    groups
        .into_iter()
        .filter(|group| group.2 == 1)
        .map(|group| group.1)
        .collect()
}

/// The unit direction out of `end` along the rim edge NOBODY picked.
pub fn away_from_the_run(rim: &[FeatureEdge], run: &[FeatureEdge], end: Vector3) -> Vector3 {
    let onward = rim
        .iter()
        .find(|edge| !run.contains(edge) && stands(edge, end))
        .expect("the rim carries on past the run");

    (the_far_end_of(onward, end) - end).get_normal()
}

/// And the unit direction out of `end` INTO the picked run.
pub fn into_the_run(run: &[FeatureEdge], end: Vector3) -> Vector3 {
    let inward = run
        .iter()
        .find(|edge| stands(edge, end))
        .expect("the end stands on the run");

    (the_far_end_of(inward, end) - end).get_normal()
}

pub fn stands(edge: &FeatureEdge, point: Vector3) -> bool {
    (edge.start - point).length() < 1e-6 || (edge.end - point).length() < 1e-6
}

pub fn the_far_end_of(edge: &FeatureEdge, point: Vector3) -> Vector3 {
    if (edge.start - point).length() < 1e-6 {
        edge.end
    } else {
        edge.start
    }
}

/// The vertices the BAND is made of - inside the band's footprint on the picked facets, and not
/// one of the part's own vertices. Both exclusions matter: the cap's triangulator plants points
/// inside the hole that are new but not band vertices, and the rim vertex a run STOPS at survives
/// untouched.
pub fn the_bands_own_vertices(part: &Mesh, built: &Mesh, run: &[FeatureEdge]) -> Vec<Vector3> {
    let facets: Vec<Vector3> = run.iter().map(facet_normal_under).collect();
    let the_parts: Vec<Vector3> = part
        .vertices
        .iter()
        .map(|&vertex| Vector3::from_vector3_float(vertex))
        .collect();

    // Distinct by exact equality, first occurrence kept, in order.
    let mut distinct: Vec<Vector3> = Vec::new();
    for vertex in built
        .vertices
        .iter()
        .map(|&vertex| Vector3::from_vector3_float(vertex))
    {
        if !distinct.contains(&vertex) {
            distinct.push(vertex);
        }
    }

    distinct
        .into_iter()
        .filter(|&vertex| {
            vertex.z > CAP_Z - SIZE - 1e-6
                && vertex.z < CAP_Z + 1e-6
                && facets
                    .iter()
                    .map(|&normal| sideways(vertex, normal))
                    .fold(f64::NEG_INFINITY, f64::max)
                    > apothem() - SIZE - 1e-6
                && !the_parts.iter().any(|&own| (own - vertex).length() < 1e-6)
        })
        .collect()
}

/// The outward horizontal normal of the wall facet this rim edge caps.
pub fn facet_normal_under(edge: &FeatureEdge) -> Vector3 {
    let middle = (edge.start + edge.end) / 2.0;

    Vector3::new(middle.x, middle.y, 0.0).get_normal()
}

/// How far a point stands out along a facet's own normal - its distance to the axis plane.
pub fn sideways(point: Vector3, facet_normal: Vector3) -> f64 {
    (point.x * facet_normal.x) + (point.y * facet_normal.y)
}

/// The distance from a point to the line the rolling ball's centre runs down over that facet -
/// `SIZE` in from the facet's plane and `SIZE` down from the cap.
pub fn distance_to_the_balls_centre_line(point: Vector3, facet_normal: Vector3) -> f64 {
    let across = sideways(point, facet_normal) - (apothem() - SIZE);
    let down = point.z - (CAP_Z - SIZE);

    ((across * across) + (down * down)).sqrt()
}
