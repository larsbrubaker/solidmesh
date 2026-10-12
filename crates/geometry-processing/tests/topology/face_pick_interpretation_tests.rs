//! Ports `GeometryProcessingTests/FacePickInterpretationTests.cs`: face regions as readings of a
//! pick - "bevel the top of this box" said by pointing at the box. A pick offers the rim of each
//! of the two faces its edge divides, and a click in the middle of a face offers that face's rim
//! and nothing else. The region is flooded at the pick's FLAT tolerance, not at the thirty degree
//! curve tolerance a chain walks with.

use std::collections::HashSet;
use std::sync::Arc;

use geometry_processing::bevel::bevel_group_tools::{self, BevelGroupToolsOptions};
use geometry_processing::bevel::{BevelAdvisoryArg, BevelAdvisoryCode};
use geometry_processing::topology::{
    bevel_edge_group_resolver, pick_interpretations, BevelSelectionKind, FaceGroup,
    FaceRegionCache, FeatureEdge, FeatureEdgeChain, FeatureEdgeSet, PickInterpretation,
    PickInterpretationKind,
};
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::Mesh;
use vector_math::Vector3;

use crate::split_crease_test_meshes;
use crate::topology_test_meshes::create_cylinder;

const CUBE_SIZE: f64 = 20.0;
const CYLINDER_SIDES: i32 = 64;

fn assert_within(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {expected} within {tolerance}, got {actual}"
    );
}

/// A cube edge means three things: that edge, the rim of the face above it and the rim of the
/// face below it. The two rims are separate readings because they are separate selections.
#[test]
fn a_cube_edge_offers_itself_and_the_rim_of_each_face_it_divides() {
    let cube = Arc::new(platonic_solids::create_cube(
        CUBE_SIZE, CUBE_SIZE, CUBE_SIZE,
    ));
    let readings = readings_at(&cube, Vector3::new(0.0, -CUBE_SIZE / 2.0, CUBE_SIZE / 2.0));

    // IsEquivalentTo: the same kinds, in any order
    let of_kind = |kind| readings.iter().filter(|r| r.kind() == kind).count();
    assert_eq!(readings.len(), 3);
    assert_eq!(of_kind(PickInterpretationKind::CollinearRun), 1);
    assert_eq!(of_kind(PickInterpretationKind::FaceGroup), 2);

    assert_eq!(readings[0].count(), 1);
    assert_eq!(
        readings[1].count(),
        4,
        "a cube face is bounded by four edges"
    );
    assert_eq!(readings[2].count(), 4);

    // the two rims share the picked edge and nothing else: they are the two faces it divides
    let second: HashSet<i32> = readings[2]
        .edges()
        .iter()
        .map(|e| e.mesh_edge_index)
        .collect();
    let shared: HashSet<i32> = readings[1]
        .edges()
        .iter()
        .map(|e| e.mesh_edge_index)
        .filter(|i| second.contains(i))
        .collect();

    assert_eq!(
        shared,
        HashSet::from([readings[0].edges()[0].mesh_edge_index])
    );

    // and both are face group selections, which is what makes them re-resolve as regions
    assert_eq!(
        readings[1].group().selection_kind,
        BevelSelectionKind::FaceGroup
    );
    assert_eq!(
        readings[2].group().selection_kind,
        BevelSelectionKind::FaceGroup
    );
}

/// A face rim takes a whole crease with it: where a boolean cut a rim edge in two the rim holds
/// both halves, so a face selection and an edge selection of the same crease agree.
#[test]
fn a_face_rim_holds_the_whole_of_a_split_crease() {
    let mesh = Arc::new(split_crease_test_meshes::create_split_crease_bar());
    let readings = readings_on_edge_at(&mesh, 10.0);

    let run = &readings[0];

    assert_eq!(run.kind(), PickInterpretationKind::CollinearRun);
    assert_eq!(
        run.count(),
        3,
        "the bar's crease is cut into three by its cells"
    );

    let rims: Vec<&PickInterpretation> = readings
        .iter()
        .filter(|r| r.kind() == PickInterpretationKind::FaceGroup)
        .collect();

    assert_eq!(rims.len(), 2);

    for rim in rims {
        let rim_edges: HashSet<i32> = rim.edges().iter().map(|e| e.mesh_edge_index).collect();

        for crease_edge in run.edges() {
            assert!(
                rim_edges.contains(&crease_edge.mesh_edge_index),
                "a rim that held half a crease would select half an edge of the shape"
            );
        }
    }
}

/// A click in the middle of a face means that face's rim, and nothing else.
#[test]
fn a_click_in_the_middle_of_a_face_means_that_faces_rim_and_nothing_else() {
    let cube = Arc::new(platonic_solids::create_cube(
        CUBE_SIZE, CUBE_SIZE, CUBE_SIZE,
    ));
    let features = Arc::new(FeatureEdgeSet::create(&cube));
    let top = index_of_face_with_normal(&cube, Vector3::new(0.0, 0.0, 1.0));

    let (readings, region) = pick_interpretations::build_for_face(
        &cube,
        None,
        &features,
        top,
        FeatureEdgeChain::DEFAULT_TANGENT_ANGLE_DEGREES,
        None,
    )
    .unwrap();

    assert_eq!(readings.len(), 1);
    assert_eq!(readings[0].kind(), PickInterpretationKind::FaceGroup);
    assert_eq!(readings[0].count(), 4);

    // every edge of it lies on the top face's plane, which says the right side was flooded
    for edge in readings[0].edges() {
        assert_within(edge.start.z, CUBE_SIZE / 2.0, 1e-6);
        assert_within(edge.end.z, CUBE_SIZE / 2.0, 1e-6);
    }

    // and the shading the viewport draws for that click is the region the reading came from
    assert_eq!(
        region.unwrap().faces().len(),
        2,
        "the cube's top is two triangles, and the flood crosses the diagonal between them"
    );
}

/// A saved face group finds its region again on a mesh triangulated differently: the anchor
/// names a rim edge and a side, and the region is flooded fresh.
#[test]
fn a_saved_face_group_resolves_to_the_same_rim_after_a_coplanar_retriangulation() {
    let bar = Arc::new(split_crease_test_meshes::create_unsplit_crease_bar());
    let features = Arc::new(FeatureEdgeSet::create(&bar));
    let top = index_of_face_with_normal(&bar, Vector3::new(0.0, 0.0, 1.0));

    let group = pick_interpretations::build_for_face(
        &bar,
        None,
        &features,
        top,
        FeatureEdgeChain::DEFAULT_TANGENT_ANGLE_DEGREES,
        None,
    )
    .unwrap()
    .0[0]
        .group()
        .clone();

    let on_the_bar_it_was_made_on = bevel_edge_group_resolver::resolve(&group, &bar);

    assert!(on_the_bar_it_was_made_on.is_resolved());
    assert_eq!(on_the_bar_it_was_made_on.edges.len(), 4);
    assert_within(perimeter(&on_the_bar_it_was_made_on.edges), 80.0, 1e-6);

    // the same solid, its top and front cut into three quads each
    let retriangulated = split_crease_test_meshes::create_split_crease_bar();

    let afterwards = bevel_edge_group_resolver::resolve(&group, &retriangulated);

    assert!(afterwards.is_resolved());
    assert_eq!(
        afterwards.edges.len(),
        8,
        "the two long sides of the rim are three mesh edges each on this triangulation"
    );

    // a different count of edges around the same rim is the same selection
    assert_within(perimeter(&afterwards.edges), 80.0, 1e-6);

    for edge in &afterwards.edges {
        assert_within(edge.start.z, 10.0, 1e-6);
        assert_within(edge.end.z, 10.0, 1e-6);
    }
}

/// Against real boolean output: a cube with a post punched through it has a top face with a HOLE
/// in it, and the rim of that face is both boundaries - 52 mm of edge, not 40.
#[test]
fn a_face_rim_takes_the_hole_in_it_as_well_as_its_outside() {
    let punched = Arc::new(split_crease_test_meshes::create_cube_with_a_post_punched_through());
    let features = Arc::new(FeatureEdgeSet::create(&punched));
    let top = index_of_face_with_normal(&punched, Vector3::new(0.0, 0.0, 1.0));

    let (readings, region) = pick_interpretations::build_for_face(
        &punched,
        None,
        &features,
        top,
        FeatureEdgeChain::DEFAULT_TANGENT_ANGLE_DEGREES,
        None,
    )
    .unwrap();

    assert_eq!(readings.len(), 1);

    // 40 mm round the outside of the top and 12 mm round the hole
    assert_within(perimeter(readings[0].edges()), 52.0, 1e-6);

    for edge in readings[0].edges() {
        assert_within(edge.start.z, 10.0, 1e-6);
        assert_within(edge.end.z, 10.0, 1e-6);
    }

    // nothing about the region is open or non-manifold, so the rim is the whole of its boundary
    assert_eq!(region.unwrap().open_boundary_mesh_edges().len(), 0);

    // and it stores as what it showed
    let resolved = bevel_edge_group_resolver::resolve(readings[0].group(), &punched);

    assert!(resolved.is_resolved());
    assert_within(perimeter(&resolved.edges), 52.0, 1e-6);
}

/// A region whose boundary is not all bevelable still offers its rim, and the flood says how much
/// it had to leave out.
#[test]
fn a_face_bounded_partly_by_an_open_edge_offers_the_rest_of_its_rim() {
    let open = Arc::new(split_crease_test_meshes::create_bar_with_a_wall_missing());
    let features = Arc::new(FeatureEdgeSet::create(&open));
    let top = index_of_face_with_normal(&open, Vector3::new(0.0, 0.0, 1.0));

    let (readings, region) = pick_interpretations::build_for_face(
        &open,
        None,
        &features,
        top,
        FeatureEdgeChain::DEFAULT_TANGENT_ANGLE_DEGREES,
        None,
    )
    .unwrap();

    assert_eq!(
        readings.len(),
        1,
        "three sides of the top have a second face to bevel against, and a partial rim is still a rim"
    );

    assert!(
        !region.unwrap().open_boundary_mesh_edges().is_empty(),
        "the missing wall leaves the top's fourth side with one face on it"
    );

    // the rim is the three closed sides: 10 + 30 + 10, and none of it along y = 0
    assert_within(perimeter(readings[0].edges()), 50.0, 1e-6);

    for edge in readings[0].edges() {
        assert!(
            edge.start.y + edge.end.y > 0.0,
            "an edge along the open side has nothing to bevel against and cannot be in the rim"
        );
    }
}

/// A cylinder rim edge offers the collinear run, the tangent chain round the whole rim, and the
/// wall facet it was clicked on - with the cap's rim collapsed into the chain, since it is the same
/// 64 edges.
#[test]
fn a_cylinder_rim_edge_offers_the_chain_and_the_wall_facet_with_the_caps_rim_collapsed_into_the_chain(
) {
    let cylinder = Arc::new(create_cylinder(10.0, 10.0, CYLINDER_SIDES));
    let readings = readings_on_edge_at(&cylinder, 5.0);

    assert_eq!(readings[0].kind(), PickInterpretationKind::CollinearRun);
    assert_eq!(readings[0].count(), 1);

    assert_eq!(readings[1].kind(), PickInterpretationKind::TangentChain);
    assert_eq!(
        readings[1].count(),
        CYLINDER_SIDES as usize,
        "read at ten degrees the wall is one surface and the rim walks the whole way round"
    );

    assert_eq!(readings[2].kind(), PickInterpretationKind::FaceGroup);
    assert_eq!(
        readings[2].count(),
        4,
        "at the pick's own tolerance the wall is facets, so the face clicked on is one quad"
    );

    assert_eq!(
        readings.len(),
        3,
        "the cap's rim is the same 64 edges as the chain, so it is that reading told twice"
    );
}

/// A group on a partly open face is warned about rather than refused: a partial rim is a warning
/// about the part, naming how many boundary edges had nothing to bevel against.
#[test]
fn a_group_on_a_partly_open_face_is_warned_about_rather_than_refused() {
    let open = Arc::new(split_crease_test_meshes::create_bar_with_a_wall_missing());
    let features = Arc::new(FeatureEdgeSet::create(&open));
    let top = index_of_face_with_normal(&open, Vector3::new(0.0, 0.0, 1.0));

    let (readings, region) = pick_interpretations::build_for_face(
        &open,
        None,
        &features,
        top,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
        None,
    )
    .unwrap();
    let region = region.expect("the face's region");

    let resolution = bevel_edge_group_resolver::resolve(readings[0].group(), &open);

    assert!(
        resolution.is_resolved(),
        "the three closed sides of the face are a real selection"
    );

    let built =
        bevel_group_tools::try_build(&resolution, true, 1.0, 4, BevelGroupToolsOptions::default())
            .unwrap_or_else(|refusal| {
                panic!(
                    "a partial rim is a warning about the part, not a refusal of the group: {:?}",
                    refusal.reason
                )
            });

    let warnings = built.warnings.expect("the partial rim is said");
    let open_edges = region.open_boundary_mesh_edges().len() as i32;

    assert!(
        warnings.iter().any(
            |warning| warning.code() == BevelAdvisoryCode::FaceRegionRimIsPartial
                && warning.args().first() == Some(&BevelAdvisoryArg::Int(open_edges))
        ),
        "the warning has to name how many edges of the boundary had nothing to bevel against"
    );
}

/// One flood per face, however many times it is asked for.
#[test]
fn asking_for_one_region_twice_floods_once() {
    let cube = Arc::new(platonic_solids::create_cube(
        CUBE_SIZE, CUBE_SIZE, CUBE_SIZE,
    ));
    let features = Arc::new(FeatureEdgeSet::create(&cube));
    let graph = cube.get_mesh_edge_graph();
    let regions = FaceRegionCache::new(
        cube.clone(),
        Some(graph.clone()),
        features.flat_angle_degrees(),
    );

    let top = index_of_face_with_normal(&cube, Vector3::new(0.0, 0.0, 1.0));

    let first = regions.region(top).unwrap();
    let second = regions.region(top).unwrap();

    assert_eq!(regions.flood_count(), 1);
    assert!(Arc::ptr_eq(&first, &second));

    // and a reading built through the cache pays nothing for the region it already has
    pick_interpretations::build_for_face(
        &cube,
        Some(&graph),
        &features,
        top,
        FeatureEdgeChain::DEFAULT_TANGENT_ANGLE_DEGREES,
        Some(&regions),
    )
    .unwrap();

    assert_eq!(
        regions.flood_count(),
        1,
        "the reading and the highlight are the same flood, and it had already been run"
    );
}

fn perimeter(edges: &[FeatureEdge]) -> f64 {
    edges
        .iter()
        .map(|edge| (edge.end - edge.start).length())
        .sum()
}

/// The readings of a pick on the feature edge whose midpoint is where a test says, built the way
/// the viewport builds them: the finest rung as the pick set, the whole ladder behind it.
fn readings_at(mesh: &Arc<Mesh>, midpoint: Vector3) -> Vec<PickInterpretation> {
    let features = Arc::new(FeatureEdgeSet::create(mesh));

    let seed = features
        .edges()
        .iter()
        .position(|e| (((e.start + e.end) / 2.0) - midpoint).length() < 1e-3)
        .unwrap_or_else(|| panic!("no feature edge has its midpoint at {midpoint:?}"));

    readings(mesh, &features, seed as i32)
}

/// The same, naming the edge by the height it runs at - for a rim, where x and y vary.
fn readings_on_edge_at(mesh: &Arc<Mesh>, height: f64) -> Vec<PickInterpretation> {
    let features = Arc::new(FeatureEdgeSet::create(mesh));

    let seed = features
        .edges()
        .iter()
        .position(|e| (e.start.z - height).abs() < 1e-6 && (e.end.z - height).abs() < 1e-6)
        .unwrap_or_else(|| panic!("no feature edge runs at z {height}"));

    readings(mesh, &features, seed as i32)
}

fn readings(
    mesh: &Arc<Mesh>,
    pick_features: &Arc<FeatureEdgeSet>,
    seed_edge_index: i32,
) -> Vec<PickInterpretation> {
    let graph = mesh.get_mesh_edge_graph();

    let ladder: Vec<Arc<FeatureEdgeSet>> =
        pick_interpretations::default_flat_angle_ladder_degrees()
            .iter()
            .map(|&degrees| Arc::new(FeatureEdgeSet::create_with_graph(mesh, &graph, degrees)))
            .collect();

    pick_interpretations::build(
        mesh,
        Some(&graph),
        pick_features,
        seed_edge_index,
        FeatureEdgeChain::DEFAULT_TANGENT_ANGLE_DEGREES,
        &ladder,
        None,
    )
    .unwrap()
}

fn index_of_face_with_normal(mesh: &Mesh, normal: Vector3) -> i32 {
    for (i, face) in mesh.faces.iter().enumerate() {
        let position0 = Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]);
        let position1 = Vector3::from_vector3_float(mesh.vertices[face.v1 as usize]);
        let position2 = Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]);

        if (position1 - position0)
            .cross(position2 - position0)
            .get_normal()
            .dot(normal)
            > 0.999
        {
            return i as i32;
        }
    }

    panic!("no face of this mesh faces {normal:?}");
}
