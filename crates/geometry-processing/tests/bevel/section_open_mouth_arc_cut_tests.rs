//! Port of `SectionOpenMouthArcCutTests.cs`: THE OPEN ARC'S SMOOTH SIDE IS CUT. The 40-edge top
//! mouth arc of [`crate::inner_face_test_mesh`] (read through [`crate::open_mouth_arc_walk`]) runs
//! between two `SectionKind::End` sections, so its boundary is a PATH and not a cycle: it starts at
//! one end's wall-side landing and stops at the other's, and the band is closed off by the creases
//! those ends slid down.
//!
//! The cut needed no new geometry for that. The crease an end slides down is a FEATURE edge, so its
//! chords are already ring edges of the bore wall's facets; the landing lies ON one of them and is
//! inserted into the ring exactly as a fold crossing is, and the crease vertices between the
//! chain's end vertex and the landing fall on the band's side of the split by the same direction
//! reading as everything else. What the open case changed is the wrapping alone.
//!
//! The landing is generally NOT on the chord `SectionEnd` recorded: the mouth's crease starts with
//! a chord 0.1214 long and the slide at radius 2 runs 0.2346, so it is two chords down. Which chord
//! is WALKED for, chord by chord from the end vertex.

use std::collections::{HashMap, HashSet};

use indexmap::IndexMap;
use polygon_mesh::Mesh;
use vector_math::Vector3;

use geometry_processing::bevel::sections::section_builder_run::try_build_run;
use geometry_processing::bevel::sections::section_fan_walk::edges_by_vertex_pair;
use geometry_processing::bevel::sections::section_smooth_seam::side_is_smooth;
use geometry_processing::bevel::sections::section_smooth_side::{
    try_cut_the_smooth_side, SmoothSideCut,
};
use geometry_processing::bevel::sections::section_smooth_side_boundary::SmoothSideFailure;
use geometry_processing::bevel::sections::{SectionPool, SectionRun};
use geometry_processing::topology::FeatureEdgeSet;

use crate::inner_face_test_mesh;
use crate::open_mouth_arc_walk::the_top_mouth_arc;
use crate::section_smooth_side_tests::{ring_area, twice_the_area};

/// The BORE WALL is the arc's LEFT side - the tessellated side the whole cut is about. Its right is
/// the cube's flat top face, which is NOT cut here: the join from the last interior station to the
/// end section's top-side landing is refused by `SectionSurfaceWalk` (the landing lies exactly on
/// the cube's top/-X edge, which is the far boundary of the strip the join is laid in), and that is
/// the walk's box and not the cut's.
const THE_BORE_WALL_IS_THE_LEFT: bool = true;

/// The radius the whole cut lands at. At 2 the bore wall's own interior joins run out of
/// `SectionSurfaceWalk`'s strip on the raked facets, which is the same walk box; the OPEN boundary
/// machinery is exercised at both, and [`an_ends_landing_is_walked_down_its_crease`] takes the
/// larger one because only there does the slide leave the crease's first chord.
const RADIUS: f64 = 0.5;

#[test]
fn the_open_arcs_bore_wall_side_cuts() {
    let part = inner_face_test_mesh::the_part();
    let graph = part.get_mesh_edge_graph();
    let features = the_features(&part, &graph);
    let edge_at_pair = edges_by_vertex_pair(&graph);
    let feature_edges = the_feature_edges(&features);

    let mut pool = SectionPool::new(&part.vertices);
    let run = the_run(&part, &graph, &features, &mut pool, RADIUS);

    let is_a_feature = |edge: i32| feature_edges.contains(&edge);
    let (cut, done) = try_cut_the_smooth_side(
        &part,
        &graph,
        &edge_at_pair,
        &mut pool,
        &run,
        THE_BORE_WALL_IS_THE_LEFT,
        Some(&is_a_feature),
    );

    match &done {
        Ok(()) => println!(
            "{} facets, {} boundary points, {} triangles",
            cut.facets.len(),
            cut.boundary.len(),
            cut.facets.iter().map(|f| f.triangles.len()).sum::<usize>()
        ),
        Err(failure) => println!("REFUSED {failure:?}"),
    }

    match done {
        Ok(()) => {}
        Err(SmoothSideFailure::Refused(refusal, at)) => panic!(
            "an open run between two end sections cuts its smooth side: {refusal:?} at {at:?}"
        ),
        Err(SmoothSideFailure::NotANumber(nan)) => panic!("the cut minted a NaN point: {nan}"),
    }

    the_cut_holds(&part, &pool, &run, &cut);
}

/// ONLY THE FOLDED SIDE IS SMOOTH. The arc's left is the bore's tessellated wall, which folds at
/// every facet edge; its right is the cube's flat TOP FACE, which the boolean left as long thin
/// triangles meeting at COPLANAR diagonals. Reading a side as smooth because its walk ended in a
/// face that does not carry the chain vertex called that flat face smooth, sent it to the cut, and
/// the cut's joins ran off `SectionSurfaceWalk`'s strip. Nothing folded there, so nothing is cut
/// there: the face rebuild's substitution answers it as it does every other flat face.
#[test]
fn the_flat_top_face_side_is_not_smooth() {
    let part = inner_face_test_mesh::the_part();
    let graph = part.get_mesh_edge_graph();
    let features = the_features(&part, &graph);

    let mut pool = SectionPool::new(&part.vertices);
    let run = the_run(&part, &graph, &features, &mut pool, RADIUS);

    let mut said = String::new();
    let mut folded_on_the_flat_side = 0;

    for (i, section) in run.sections.iter().enumerate() {
        said.push_str(&format!(
            "station {i}: bore wall fold {} face {}, top face fold {} face {}\n",
            section.left_walk_crossed_a_fold,
            section.left_walk_face,
            section.right_walk_crossed_a_fold,
            section.right_walk_face
        ));

        if section.right_walk_crossed_a_fold {
            folded_on_the_flat_side += 1;
        }
    }

    println!("{said}");

    assert_eq!(
        folded_on_the_flat_side, 0,
        "no station's reach folds on the cube's flat top face: {said}"
    );

    let edge_at_pair = edges_by_vertex_pair(&graph);
    let mut regions = IndexMap::new();

    assert!(
        side_is_smooth(
            &part,
            &graph,
            &edge_at_pair,
            &run,
            THE_BORE_WALL_IS_THE_LEFT,
            &mut regions
        ),
        "the bore's tessellated wall folds: {said}"
    );
    assert!(
        !side_is_smooth(
            &part,
            &graph,
            &edge_at_pair,
            &run,
            !THE_BORE_WALL_IS_THE_LEFT,
            &mut regions
        ),
        "the cube's top face is flat, however it is tessellated: {said}"
    );
}

/// AN END'S LANDING IS FOUND BY WALKING ITS CREASE. At radius 2 the wall-side slide runs 0.2346 and
/// the crease's first chord - the one `SectionEnd` recorded - is only 0.1214 long, so the landing is
/// a chord or more further down and the boundary's place must be that chord, not the recorded one.
/// (The cut itself then refuses at radius 2 on an INTERIOR join, which is `SectionSurfaceWalk`'s
/// strip and not this; the places are all laid before any join is tried, so they are read off the
/// cut either way.)
#[test]
fn an_ends_landing_is_walked_down_its_crease() {
    let part = inner_face_test_mesh::the_part();
    let graph = part.get_mesh_edge_graph();
    let features = the_features(&part, &graph);
    let edge_at_pair = edges_by_vertex_pair(&graph);
    let feature_edges = the_feature_edges(&features);

    let mut pool = SectionPool::new(&part.vertices);
    let run = the_run(&part, &graph, &features, &mut pool, 2.0);

    let is_a_feature = |edge: i32| feature_edges.contains(&edge);
    let (cut, _) = try_cut_the_smooth_side(
        &part,
        &graph,
        &edge_at_pair,
        &mut pool,
        &run,
        THE_BORE_WALL_IS_THE_LEFT,
        Some(&is_a_feature),
    );

    assert_eq!(
        cut.station_points.len(),
        run.sections.len(),
        "every station's place is laid before any join is tried"
    );

    let mut walked_on = 0;
    let mut said = String::new();

    for index in [0, run.sections.len() - 1] {
        let section = &run.sections[index];
        let place = &cut.places[&cut.station_boundary[index]];

        said.push_str(&format!(
            "end {index}: recorded chord {}, landed on {} at {:.4}\n",
            section.left_slide_edge, place.edge, place.parameter
        ));

        assert!(
            place.edge >= 0,
            "an end's landing is ON a chord of the crease it slid down"
        );
        assert!((0.0..=1.0).contains(&place.parameter));

        if place.edge != section.left_slide_edge {
            walked_on += 1;
        }
    }

    println!("{said}");

    assert_eq!(
        walked_on, 2,
        "at radius 2 BOTH ends slide past the crease's recorded first chord: {said}"
    );
}

/// Every claim the cut makes, on one side: area conserved per facet, kept rings simple, every fold
/// edge shared by exactly its two facets (no T junction), the boundary a PATH, and the band within a
/// few percent of setback times arc length.
fn the_cut_holds(part: &Mesh, pool: &SectionPool, run: &SectionRun, cut: &SmoothSideCut) {
    assert!(!cut.boundary_is_closed, "an open run's boundary is a path");

    // The band is closed by the two ends' creases, so the boundary's OWN last point is the second
    // end section's and is joined to nothing - there is one join fewer than there are stations.
    assert_eq!(cut.station_points.len(), run.sections.len());
    assert_eq!(cut.crossings_after.len(), run.sections.len() - 1);
    assert_eq!(cut.boundary[0], cut.station_boundary[0]);
    assert_eq!(
        cut.boundary[cut.boundary.len() - 1],
        cut.station_boundary[cut.station_boundary.len() - 1]
    );

    let mut worst_area_slip = 0.0f64;
    let mut band_area = 0.0;
    let mut rings_that_repeat_a_point = 0;
    let mut on_a_kept_edge: HashMap<(i32, i32), i32> = HashMap::new();
    let mut on_a_band_edge: HashMap<(i32, i32), i32> = HashMap::new();

    for facet in &cut.facets {
        let whole: f64 = facet
            .faces
            .iter()
            .map(|&face| twice_the_area(part, face, facet.normal))
            .sum::<f64>()
            / 2.0;
        let kept: f64 = facet
            .kept_rings
            .iter()
            .map(|ring| ring_area(pool, ring, facet.normal))
            .sum();
        let band: f64 = facet
            .band_rings
            .iter()
            .map(|ring| ring_area(pool, ring, facet.normal))
            .sum();

        worst_area_slip = worst_area_slip.max((kept + band - whole).abs() / 1e-12f64.max(whole));
        band_area += band;

        for (rings, counted) in [
            (&facet.kept_rings, &mut on_a_kept_edge),
            (&facet.band_rings, &mut on_a_band_edge),
        ] {
            for ring in rings {
                let distinct: HashSet<i32> = ring.iter().copied().collect();

                if distinct.len() != ring.len() {
                    rings_that_repeat_a_point += 1;
                }

                for i in 0..ring.len() {
                    let edge = (ring[i], ring[(i + 1) % ring.len()]);

                    *counted.entry(edge).or_insert(0) += 1;
                }
            }
        }
    }

    // A kept ring's directed edge is drawn once; the same edge drawn the OTHER way in another facet
    // is the two of them meeting across a fold, which is exactly what makes no T junction.
    let drawn_twice = on_a_kept_edge.values().filter(|&&times| times > 1).count();

    let arc_length: f64 = run
        .edges
        .iter()
        .map(|edge| (vertex(part, edge.to_vertex()) - vertex(part, edge.from_vertex())).length())
        .sum();

    // The END stations' setbacks are the ends' own, and the band tapers to nothing outside them, so
    // the strip wanted is the setback averaged over the INTERIOR stations times the arc it rides.
    let interior = &run.sections[1..run.sections.len() - 1];
    let setback =
        interior.iter().map(|section| section.setback).sum::<f64>() / interior.len() as f64;
    let wanted = setback * arc_length;

    // The chain's own END VERTICES go with the BAND: the boundary leaves the facet's outline on the
    // crease PAST them, so they are inside the arc the split hands to the band.
    let end_vertices = [
        run.sections[0].chain_vertex,
        run.sections[run.sections.len() - 1].chain_vertex,
    ];
    let in_the_band = end_vertices
        .iter()
        .filter(|vertex| {
            cut.facets
                .iter()
                .any(|facet| facet.band_rings.iter().any(|ring| ring.contains(vertex)))
        })
        .count();
    let standing_in_the_kept = end_vertices
        .iter()
        .filter(|vertex| {
            cut.facets
                .iter()
                .any(|facet| facet.kept_rings.iter().any(|ring| ring.contains(vertex)))
        })
        .count();

    println!(
        "{} facets, band {band_area:.4} against setback {setback:.4} x arc {arc_length:.4} = \
         {wanted:.4} ({:+.2}%), worst area slip {worst_area_slip:.3e}, {rings_that_repeat_a_point} \
         rings repeating a point, {drawn_twice} kept edges drawn twice, \
         {in_the_band}/{standing_in_the_kept} end vertices in band/kept rings",
        cut.facets.len(),
        ((band_area / wanted) - 1.0) * 100.0
    );

    assert_eq!(
        in_the_band, 2,
        "each end of the chain is inside its band ring, between the chain and the crease landing"
    );

    assert_eq!(
        standing_in_the_kept, 0,
        "no end of the chain is left standing in what the part keeps"
    );

    // The fixture's points are stored SINGLE precision, so a facet's own faces are not exactly
    // coplanar and their areas projected on one normal do not sum to the ring's to 1e-9. What is
    // asserted is that the split lost nothing beyond that float lattice.
    assert!(
        worst_area_slip < 1e-3,
        "a topological split hands back the facet it was given, to the fixture's own float noise: \
         {worst_area_slip}"
    );

    assert_eq!(
        rings_that_repeat_a_point, 0,
        "every ring the split produced is simple"
    );

    assert_eq!(
        drawn_twice, 0,
        "no directed edge of a kept ring is drawn twice, so the facets meet without a T junction"
    );

    let ratio = band_area / wanted;
    assert!(
        (0.93..=1.07).contains(&ratio),
        "the band is the arc swept one setback over the wall, give or take its two tapered ends: \
         {ratio}"
    );
}

fn the_features(part: &Mesh, graph: &polygon_mesh::MeshEdgeGraph) -> FeatureEdgeSet {
    FeatureEdgeSet::create_with_graph(part, graph, inner_face_test_mesh::BEND_THRESHOLD_DEGREES)
}

fn the_feature_edges(features: &FeatureEdgeSet) -> HashSet<i32> {
    features
        .edges()
        .iter()
        .map(|feature| feature.mesh_edge_index)
        .collect()
}

/// The open run along the top mouth arc at `radius`, as every test here builds it.
fn the_run(
    part: &Mesh,
    graph: &polygon_mesh::MeshEdgeGraph,
    features: &FeatureEdgeSet,
    pool: &mut SectionPool,
    radius: f64,
) -> SectionRun {
    let arc = the_top_mouth_arc(part, features);

    try_build_run(
        &arc,
        false,
        part,
        graph,
        pool,
        radius,
        inner_face_test_mesh::SEGMENTS,
        true,
        Some(features),
    )
    .unwrap_or_else(|why_not| panic!("the open run builds: {why_not:?}"))
}

fn vertex(part: &Mesh, index: i32) -> Vector3 {
    Vector3::from_vector3_float(part.vertices[index as usize])
}
