//! One part of the pierced face tests: THE SECTION THE JUNCTION IS DRAWN AS
//! (`section_junction_build`) on the very geometry [`crate::section_pierced_face_tests`] measures -
//! the junction predicate, the corner deferring to it, one mitre ellipse used twice with both wall
//! ends on it, the patch of wall across the fold, the ledger closing by identity, the reading under
//! the rigid motions and the refusals that mint nothing. The class's helpers are
//! [`crate::section_pierced_face_readings`]; the junction's place in a whole RUN (the builder, the
//! rebuild, the smooth side's cut) is [`crate::section_pierced_face_junction_run_tests`], split
//! from this file only for length.

use std::collections::HashMap;
use std::time::Instant;

use solidmesh::bevel::sections::section_corner::{self, CornerRefusal};
use solidmesh::bevel::sections::section_fan_walk;
use solidmesh::bevel::sections::section_junction_build::{self, JunctionRefusal};
use solidmesh::bevel::sections::{JunctionPiece, SectionKind, SectionPool};
use solidmesh::bevel::BevelAdvisoryCode;
use solidmesh::topology::{ChainEdge, FeatureEdge};
use solidmesh_math::math_helper::{self, degrees_to_radians};
use solidmesh_math::Vector3;
use solidmesh_mesh::{Mesh, MeshEdgeGraph};

use crate::inner_face_test_mesh;
use crate::rigid_motion_meshes::{move_mesh, the_motions};
use crate::section_pierced_face_readings::{
    distance_to_line, the_axis_at, the_crease_at, the_fold_at, the_junction_at,
    the_operations_feature_reading, the_plans, the_turn_at, wedge, RADIUS,
};

/// THE READING THAT DISPATCHES IT: the junction's own predicate reads TRUE at the two mouth
/// junctions and FALSE at every other station of the same chain - the cube's four square corners,
/// which the mitre owns, and every smooth station of the mouth arc.
///
/// The router's turn clause asks `SectionCorner.LooksLikeACorner` of the 86.81 degree turn, the
/// wedge screen says no, and `SectionBuilder.TrySection` would refuse `SurfaceCornerNotSupported`
/// before `SectionCorner.TryBuild` is called at all. This predicate is what both of those have to
/// ask instead.
#[test]
fn the_junction_station_is_recognised() {
    let part = inner_face_test_mesh::the_part();
    let graph = part.get_mesh_edge_graph();
    let (plans, route, _) = the_plans(&part, None);

    assert_eq!(
        route.as_deref(),
        Some("sections"),
        "the turn clause no longer refuses the pass at the first junction - duty four lifts the \
         wedge screen where this very predicate reads true"
    );

    let edge_at_pair = section_fan_walk::edges_by_vertex_pair(&graph);
    let is_a_feature_edge = the_operations_feature_reading(&part, &graph);

    for plan in &plans {
        let count = plan.edges.len();

        for i in 0..count {
            let arriving = plan.edges[(i + count - 1) % count].edge;
            let leaving = plan.edges[i].edge;
            let vertex = leaving.from_vertex();

            let is_a_junction = section_junction_build::is_a_junction_station(
                &part,
                &graph,
                &arriving,
                &leaving,
                vertex,
                Some(&*is_a_feature_edge),
                Some(&edge_at_pair),
            );

            assert_eq!(
                is_a_junction,
                vertex == 19 || vertex == 59,
                "vertex {vertex} turns {} degrees and the corner rule reads it {}",
                section_corner::turn_degrees(&arriving, &leaving),
                section_corner::looks_like_a_corner(&arriving, &leaving)
            );
        }
    }

    // And the same reading taken without the caller's lookup table, which is how a pass that has
    // not walked the fan yet will ask it.
    for vertex in [19, 59] {
        let (arriving, leaving) = the_turn_at(&plans, vertex);

        assert!(
            section_junction_build::is_a_junction_station(
                &part,
                &graph,
                &arriving,
                &leaving,
                vertex,
                Some(&*is_a_feature_edge),
                None,
            ),
            "the predicate builds its own lookup where it is handed none"
        );
    }
}

/// THE CORNER DEFERS IT. Asked to draw the junction, `SectionCorner.TryBuild` says no and names
/// NOTHING - the "handled elsewhere" answer - so the station can be dispatched to the junction
/// rather than refused. A corner is still drawn.
#[test]
fn the_corner_defers_the_junction_rather_than_refusing_it() {
    let part = inner_face_test_mesh::the_part();
    let graph = part.get_mesh_edge_graph();
    let (plans, _, _) = the_plans(&part, None);
    let edge_at_pair = section_fan_walk::edges_by_vertex_pair(&graph);
    let is_a_feature_edge = the_operations_feature_reading(&part, &graph);

    for vertex in [19, 59] {
        let (arriving, leaving) = the_turn_at(&plans, vertex);
        let mut pool = SectionPool::new(&part.vertices);

        let built = section_corner::try_build(
            &arriving,
            &leaving,
            &part,
            &graph,
            &edge_at_pair,
            &mut pool,
            RADIUS,
            8,
            Some(&*is_a_feature_edge),
        );

        // Refused as a junction, with no section and no failure: the junction is handled elsewhere.
        match built {
            Err(CornerRefusal::AJunction) => {}
            Ok(_) => panic!("the mitre cannot draw two wall ends"),
            Err(CornerRefusal::Refused(failure)) => panic!(
                "the junction at {vertex} is handled elsewhere, not refused - instead {:?}",
                failure.code()
            ),
            Err(CornerRefusal::NotANumber(nan)) => panic!("refused on a NaN: {nan:?}"),
        }
    }

    let (square_arriving, square_leaving) = the_turn_at(&plans, 1);
    let mut square_pool = SectionPool::new(&part.vertices);

    let square = section_corner::try_build(
        &square_arriving,
        &square_leaving,
        &part,
        &graph,
        &edge_at_pair,
        &mut square_pool,
        RADIUS,
        8,
        Some(&*is_a_feature_edge),
    );

    assert!(
        square.is_ok(),
        "the cube's own square corner is a mitre - instead {:?}",
        square.err()
    );
}

/// THE JUNCTION SECTION ITSELF: one row of samples along the one mitre ellipse, every sample on
/// BOTH canals, the flat end on the offset-meet, and the two wall ends at their own walls'
/// tangencies - the corner the mitre rule could not draw, drawn.
#[test]
fn the_junctions_samples_stand_on_both_canals() {
    let part = inner_face_test_mesh::the_part();
    let (plans, _, _) = the_plans(&part, None);

    for radius in [RADIUS, 2.0] {
        for vertex in [19, 59] {
            let (arriving, leaving) = the_turn_at(&plans, vertex);
            let station = leaving.from();
            let (section, pool) = the_junction_at(&part, &plans, vertex, radius, 8);

            let arriving_axis = the_axis_at(&arriving, station, radius);
            let leaving_axis = the_axis_at(&leaving, station, radius);
            let arriving_along = arriving.direction().get_normal();
            let leaving_along = leaving.direction().get_normal();

            for &sample in &section.samples {
                let at = pool.position_of(sample);

                assert!(
                    (distance_to_line(at, arriving_axis, arriving_along) - radius).abs() <= 1e-9,
                    "every sample stands on the ARRIVING canal"
                );

                assert!(
                    (distance_to_line(at, leaving_axis, leaving_along) - radius).abs() <= 1e-9,
                    "and on the LEAVING one - it is one ellipse, and both strips end on it"
                );
            }

            // The FLAT end: the offset-meet, standing in the shared top face on both offset lines.
            let meet = pool.position_of(section.samples[0]);

            assert!(
                (meet.z - 20.0).abs() <= 1e-9,
                "the shared side is the top face"
            );

            assert!(
                (distance_to_line(meet, station, arriving_along)
                    - radius * (degrees_to_radians(wedge(&arriving)) / 2.0).tan())
                .abs()
                    <= 1e-6,
                "on the arriving edge's own offset line"
            );

            assert!(
                (distance_to_line(meet, station, leaving_along)
                    - radius * (degrees_to_radians(wedge(&leaving)) / 2.0).tan())
                .abs()
                    <= 1e-6,
                "and on the leaving edge's, which is what an offset-meet is"
            );

            // The two WALL ends: each its own canal's tangency with its own wall, so it stands a
            // radius off that axis ALONG that wall's normal and IN that wall's plane.
            is_the_tangency_of(
                &pool,
                section.samples[section.arriving_wall_end as usize],
                &arriving,
                station,
                radius,
            );
            is_the_tangency_of(
                &pool,
                section.samples[section.leaving_wall_end as usize],
                &leaving,
                station,
                radius,
            );

            assert_ne!(
                section.arriving_wall_end, section.leaving_wall_end,
                "two different points of the one ellipse is what makes it a junction"
            );

            assert_eq!(
                section.far_wall_end(),
                section.samples.len() as i32 - 1,
                "the row is drawn out to the farther of the two"
            );

            // The two strips' own sections are stretches of that ONE row, shared by index.
            assert_eq!(section.arriving_section.kind, SectionKind::Junction);
            assert_eq!(
                section.arriving_section.profile.len() as i32,
                section.arriving_wall_end + 1
            );
            assert_eq!(
                section.leaving_section.profile.len() as i32,
                section.leaving_wall_end + 1
            );

            for point in section
                .arriving_section
                .profile
                .iter()
                .chain(section.leaving_section.profile.iter())
            {
                assert!(
                    section.samples.contains(point),
                    "neither strip's section mints a point of its own"
                );
            }
        }
    }
}

/// THE READING'S OWN NUMBERS at vertex 19 and `r = 0.5`: the offset-meet, P1 on the -X wall, P2 on
/// the bore facet 0.0278 beyond it, and F1 on the fold - the four points the file header names.
#[test]
fn the_junctions_points_are_the_ones_the_reading_names() {
    let part = inner_face_test_mesh::the_part();
    let (plans, _, _) = the_plans(&part, None);
    let (section, pool) = the_junction_at(&part, &plans, 19, RADIUS, 8);

    let meet = pool.position_of(section.samples[0]);
    let p1 = pool.position_of(section.samples[section.arriving_wall_end as usize]);
    let p2 = pool.position_of(section.samples[section.leaving_wall_end as usize]);
    let f1 = pool.position_of(section.fold_point);

    assert!(
        (meet - Vector3::new(-18.5, -44.5015, 20.0)).length() < 1e-3,
        "the flat end is the offset-meet"
    );

    assert!(
        (p1 - Vector3::new(-19.0, -44.0286, 19.5)).length() < 1e-3,
        "the arriving strip ends on the -X wall's tangency"
    );

    assert!(
        (p2 - Vector3::new(-18.9992, -44.0293, 19.4723)).length() < 1e-3,
        "and the leaving strip on the bore facet's"
    );

    assert!(
        ((p1 - p2).length() - 0.027766).abs() <= 1e-5,
        "the stretch of ellipse the patch bridges"
    );

    assert!(
        (f1 - Vector3::new(-19.0, -43.9722, 19.5)).length() < 1e-3,
        "F1 is where the arriving canal's contact with that wall ends, on the fold"
    );
}

/// THE PATCH: one triangle standing on the fold between the two wall ends, its three vertices
/// points the section already holds, and the whole of it INSIDE the part.
#[test]
fn the_patch_stands_on_the_fold_inside_the_part() {
    let part = inner_face_test_mesh::the_part();
    let (plans, _, _) = the_plans(&part, None);

    for radius in [RADIUS, 2.0] {
        for vertex in [19, 59] {
            let (arriving, leaving) = the_turn_at(&plans, vertex);
            let station = leaving.from();
            let fold = the_fold_at(&part, &arriving, &leaving);
            let (section, pool) = the_junction_at(&part, &plans, vertex, radius, 8);

            let patches = section.patch_triangles();

            assert_eq!(
                patches.len(),
                1,
                "the far wall end stands less than one sample's step past the near one"
            );

            let patch = patches[0];
            let f1 = pool.position_of(section.fold_point);

            assert!(
                distance_to_line(f1, station, fold) < 1e-9,
                "F1 stands ON the fold"
            );

            assert!(
                (f1 - station).dot(fold) > 0.0,
                "and down it, away from the vertex, on the part's own crease"
            );

            let mut drawn = patch.to_vec();
            let mut expected = vec![
                section.fold_point,
                section.samples[section.far_wall_end() as usize],
                section.samples[section.near_wall_end() as usize],
            ];
            drawn.sort();
            expected.sort();

            assert_eq!(
                drawn, expected,
                "its three vertices are F1 and the two wall ends, all of them already drawn"
            );

            // INSIDE THE PART: the two planes that bound the part AT THE JUNCTION are the fold's own
            // two faces - the cube's -X wall and the bore facet that abuts it - and the patch stands
            // under both. Both pass through the station, because the fold leaves the station.
            let centroid = (pool.position_of(patch[0])
                + pool.position_of(patch[1])
                + pool.position_of(patch[2]))
                / 3.0;
            let crease = the_crease_at(&part, &arriving, &leaving);

            assert!(
                (centroid - station).dot(crease.normal0.get_normal()) < 0.0,
                "the patch lies inside one of the two faces the fold joins"
            );

            assert!(
                (centroid - station).dot(crease.normal1.get_normal()) < 0.0,
                "and inside the other - it does not poke out of the part at the fold"
            );
        }
    }
}

/// THE NEIGHBOURHOOD CLOSES BY IDENTITY: every directed edge the junction's ledger names is used
/// exactly twice, once each way - the two strips' rungs cancelling on the stretch they share, the
/// patch cancelling the far strip's stretch past the near wall end, and the two rebuilt rims
/// cancelling the patch's other two sides.
#[test]
fn the_junctions_neighbourhood_closes_by_identity() {
    let part = inner_face_test_mesh::the_part();
    let (plans, _, _) = the_plans(&part, None);

    for vertex in [19, 59] {
        let (section, _) = the_junction_at(&part, &plans, vertex, RADIUS, 8);
        let edges = section.directed_edges();
        let mut counted: HashMap<(i32, i32), i32> = HashMap::new();

        for (from, to, _) in &edges {
            *counted.entry((*from, *to)).or_insert(0) += 1;
        }

        for (key, value) in &counted {
            assert_eq!(
                *value, 1,
                "no directed edge is laid twice the same way ({key:?})"
            );

            assert!(
                counted.contains_key(&(key.1, key.0)),
                "and every one has its opposite ({key:?})"
            );
        }

        // And the pieces are the ones the wiring is told to lay.
        let pieces = |piece: JunctionPiece| edges.iter().filter(|edge| edge.2 == piece).count();

        assert_eq!(pieces(JunctionPiece::Patch), 3, "one patch triangle");

        assert_eq!(
            pieces(JunctionPiece::NearWallRim),
            1,
            "the wall that holds the fold lays one rim segment"
        );

        assert_eq!(
            pieces(JunctionPiece::FarWallRim),
            1,
            "and the tessellated facet the other"
        );
    }
}

/// THE JUNCTION IS THE PART'S, NOT THE AXES': stood somewhere else, it is the same section moved -
/// the same sample count and the same points through the motion.
#[test]
fn the_junction_is_the_same_under_the_rigid_motions() {
    let part = inner_face_test_mesh::the_part();
    let (plans, _, _) = the_plans(&part, None);
    let (stood, pool) = the_junction_at(&part, &plans, 19, RADIUS, 8);

    for (name, motion) in the_motions() {
        let mut moved = inner_face_test_mesh::the_part();

        move_mesh(&mut moved, motion);

        let up = Vector3::UNIT_Z.transform_normal(motion).get_normal();
        let (moved_plans, _, _) = the_plans(&moved, Some(up));
        let (there, moved_pool) = the_junction_at(&moved, &moved_plans, 19, RADIUS, 8);

        assert_eq!(
            there.samples.len(),
            stood.samples.len(),
            "{name} moves the part, not the construction"
        );

        for k in 0..stood.samples.len() {
            let expected = pool
                .position_of(stood.samples[k])
                .transform_position(motion);

            // A moved mesh is stored back on the float lattice, so the reading moves by a float's
            // worth of the part's size rather than by nothing at all.
            assert!(
                (moved_pool.position_of(there.samples[k]) - expected).length() < 1e-4,
                "sample {k} under {name}"
            );
        }

        let fold_there = pool
            .position_of(stood.fold_point)
            .transform_position(motion);

        assert!(
            (moved_pool.position_of(there.fold_point) - fold_there).length() < 1e-4,
            "and F1 under {name}"
        );
    }
}

/// A LEAVING WALL THAT DOES NOT STAND ON THE ARRIVING CANAL'S SWEEP SIDE is refused by name, and in
/// bounded time. The sweep runs 0 at the shared flat side and 1 at the arriving edge's own wall; a
/// leaving wall whose own fold turns the OTHER way off the flat side reads at a NEGATIVE parameter,
/// and the row of samples the junction draws between them has a step of zero or less - the
/// carry-on loop marches away from its end and never reaches it, growing a list until the drag runs
/// out of memory. The refusal must come before any sampling; this test's own completion is the
/// proof.
#[test]
fn a_junction_whose_leaving_wall_turns_the_other_way_is_refused_rather_than_sampled_forever() {
    let part = the_wall_turned_the_other_way();
    let graph = part.get_mesh_edge_graph();

    // The two edges of the shared flat top face that meet at the origin, each running so that the
    // top face is on the same side of both - which is what makes them one chain through the vertex.
    let arriving = the_chain_edge(&part, &graph, Vector3::new(0.0, -10.0, 0.0), Vector3::ZERO);
    let leaving = the_chain_edge(&part, &graph, Vector3::ZERO, Vector3::new(10.0, 0.0, 0.0));
    let mut pool = SectionPool::new(&part.vertices);
    let watch = Instant::now();

    let built = section_junction_build::try_build(
        &arriving,
        &leaving,
        Vector3::UNIT_X,
        &mut pool,
        1.0,
        8,
        None,
    );

    let elapsed = watch.elapsed();

    let failure = match built {
        Ok(_) => panic!(
            "the leaving wall stands behind the shared flat side, so there is no row to draw"
        ),
        Err(JunctionRefusal::Refused(failure)) => failure,
        Err(JunctionRefusal::NotANumber(nan)) => panic!("refused on a NaN instead: {nan:?}"),
    };

    assert_eq!(failure.code(), BevelAdvisoryCode::SurfaceCornerNotSupported);

    assert!(
        elapsed.as_secs_f64() < 5.0,
        "and it is refused before any sampling - the reading itself is the whole cost"
    );

    assert_eq!(
        pool.count(),
        part.vertices.len() as i32,
        "a refused junction mints nothing"
    );
}

/// A REFUSAL MINTS NOTHING IN THE SHARED POOL. The pool never removes, so a sample minted before a
/// refusal that can still fire is an orphan every later pass reads as a point of the bevel. Refused
/// here on the LAST reading the junction takes - a fold the near canal's tangency line never meets
/// - which is the one that used to fire after the whole row was already in the pool.
#[test]
fn a_refused_junction_leaves_no_orphans_in_the_pool() {
    let part = inner_face_test_mesh::the_part();
    let (plans, _, _) = the_plans(&part, None);
    let (arriving, leaving) = the_turn_at(&plans, 19);
    let mut pool = SectionPool::new(&part.vertices);
    let before = pool.count();

    // The fold as the ARRIVING edge's own direction: the near canal's tangency line runs along one
    // of the two edges, so this fold either lies along it - no one meet - or crosses it skew.
    let built = section_junction_build::try_build(
        &arriving,
        &leaving,
        arriving.direction().get_normal(),
        &mut pool,
        RADIUS,
        8,
        None,
    );

    let failure = match built {
        Ok(_) => panic!("that fold is no fold of the junction's own wall"),
        Err(JunctionRefusal::Refused(failure)) => failure,
        Err(JunctionRefusal::NotANumber(nan)) => panic!("refused on a NaN instead: {nan:?}"),
    };

    assert_eq!(failure.code(), BevelAdvisoryCode::SurfaceCornerNotSupported);

    assert_eq!(
        pool.count(),
        before,
        "and the samples it had already drawn are not left in the pool behind it"
    );
}

/// A SHARED FLAT TOP FACE whose two boundary edges at the origin carry walls that turn OPPOSITE
/// ways off it: the x = 0 wall hangs straight down at ninety degrees, and the y = 0 face dips twenty
/// degrees back UNDER the top face rather than away from it. Slivery on purpose - it is the input
/// the junction has to refuse, not a part anyone would bevel.
fn the_wall_turned_the_other_way() -> Mesh {
    let mut mesh = Mesh::new();
    let dip = math_helper::degrees_to_radians(20.0);
    let under = Vector3::new(0.0, -5.0, -5.0 * dip.tan());
    let far_under = Vector3::new(10.0, -5.0, -5.0 * dip.tan());

    // The top face, z = 0, over x in [0, 10] and y in [-10, 0].
    mesh.create_face(&[
        Vector3::ZERO,
        Vector3::new(0.0, -10.0, 0.0),
        Vector3::new(10.0, -10.0, 0.0),
    ]);
    mesh.create_face(&[
        Vector3::ZERO,
        Vector3::new(10.0, -10.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
    ]);

    // The x = 0 wall, hanging down - the ARRIVING edge's own wall, at ninety degrees to the top.
    mesh.create_face(&[
        Vector3::ZERO,
        Vector3::new(0.0, 0.0, -5.0),
        Vector3::new(0.0, -10.0, -5.0),
    ]);
    mesh.create_face(&[
        Vector3::ZERO,
        Vector3::new(0.0, -10.0, -5.0),
        Vector3::new(0.0, -10.0, 0.0),
    ]);

    // The y = 0 face, dipping back under the top face - the LEAVING edge's wall, turned the other
    // way.
    mesh.create_face(&[Vector3::ZERO, under, far_under]);
    mesh.create_face(&[Vector3::ZERO, far_under, Vector3::new(10.0, 0.0, 0.0)]);

    mesh.clean_and_merge();

    mesh
}

/// The chain edge running from one position to another, classified off the mesh itself.
fn the_chain_edge(mesh: &Mesh, graph: &MeshEdgeGraph, from: Vector3, to: Vector3) -> ChainEdge {
    for i in 0..graph.edge_count() {
        let Some(edge) =
            FeatureEdge::try_classify(mesh, graph, i, FeatureEdge::default_flat_angle_degrees())
        else {
            continue;
        };

        if (edge.start - from).length() < 1e-6 && (edge.end - to).length() < 1e-6 {
            return ChainEdge::new(i, edge, false);
        }

        if (edge.end - from).length() < 1e-6 && (edge.start - to).length() < 1e-6 {
            return ChainEdge::new(i, edge, true);
        }
    }

    panic!("no feature edge from {from} to {to}");
}

/// Whether a sample is the tangency of one edge's canal with its OWN wall.
fn is_the_tangency_of(
    pool: &SectionPool,
    sample: i32,
    edge: &ChainEdge,
    station: Vector3,
    radius: f64,
) {
    let at = pool.position_of(sample);
    let axis = the_axis_at(edge, station, radius);

    // The SHARED side on this pick is the top face, so the wall is whichever normal is not up.
    let flat_is_left = (edge.left_normal().get_normal().z - 1.0).abs() < 1e-6;
    let wall = if flat_is_left {
        edge.right_normal()
    } else {
        edge.left_normal()
    }
    .get_normal();

    assert!(
        ((at - axis).dot(wall) - radius).abs() <= 1e-9,
        "the wall end stands a radius off its own axis along its own wall's normal"
    );

    assert!(
        (at - station).dot(wall).abs() < 1e-9,
        "which puts it IN that wall's plane - a tangency, not a point in the air"
    );
}
