//! THE CORNER SECTION on a box's top rim - four ninety degree
//! corners, each one mitre both its strips read, symmetric whichever edge's canal draws it, ending
//! at the offset-meet in the shared flat face and a setback down the fold; the strips between still
//! the rolling ball's quarter cylinder; the whole rim assembled closed, one shell, removing what the
//! legacy construction removes, wherever the box stands. The rim is reached through the FACE GROUP
//! pick's own plan, built through the shared `surface_test_build`.

use std::rc::Rc;

use geometry_processing::bevel::sections::{
    section_assembly, section_builder_run, section_corner, section_fan_walk, section_routing,
    SectionKind, SectionPool, SectionRun,
};
use geometry_processing::bevel::surface::surface_mesh_closure;
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelSelectionKind, ChainEdge, FaceGroup, FeatureEdgeSet,
};
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::{CancelToken, Mesh, MeshEdgeGraph};
use vector_math::{Matrix4X4, Vector3};

use crate::mesh_measurements::{shell_count, signed_volume, unclassifiable_edges};
use crate::mesh_surface_distance::farthest_from;
use crate::rigid_motion_meshes::{most_faces_on_an_edge, move_mesh, signature_of, the_motions};
use crate::surface_test_build::{pinned_to_the_sections, try_bevel, TryBevelOptions};

const SIDE: f64 = 10.0;

const RADIUS: f64 = 1.0;

const SEGMENTS: i32 = 8;

/// What [`the_rim_run`] hands back: the box, the run of sections the top face pick's own plan draws
/// on it, and the mesh the construction this replaces builds from that same plan.
struct RimRun {
    part: Mesh,
    graph: MeshEdgeGraph,
    pool: SectionPool,
    run: SectionRun,
    legacy: Mesh,
}

/// The four corners are corner sections, and each is one object both its strips read.
#[test]
fn the_box_top_rim_draws_four_corner_sections_and_no_smooth_one() {
    let rim = the_rim_run(Matrix4X4::IDENTITY);
    let run = &rim.run;

    assert_eq!(run.count(), 4);

    for i in 0..run.count() {
        assert_eq!(
            run.sections[i as usize].kind,
            SectionKind::Corner,
            "vertex {} turns ninety degrees",
            run.sections[i as usize].chain_vertex
        );

        assert!(
            Rc::ptr_eq(
                &run.end_of(i).expect("a closed run"),
                &run.start_of((i + 1) % run.count())
            ),
            "the two strips meeting at a corner read ONE section, not two that agree"
        );
    }
}

/// The same corner walked the other way round - the leaving edge reversed arriving, the arriving
/// edge reversed leaving - draws the same points in the opposite order. That is the symmetry claim:
/// the mitre is the bisector plane's cut of EITHER canal.
#[test]
fn either_edges_canal_cuts_the_bisector_plane_in_the_same_curve() {
    let rim = the_rim_run(Matrix4X4::IDENTITY);
    let run = &rim.run;
    let count = run.count() as usize;

    let edge_at_pair = section_fan_walk::edges_by_vertex_pair(&rim.graph);

    for i in 0..count {
        let arriving = run.edges[(i + count - 1) % count];
        let leaving = run.edges[i];

        let mut backwards = SectionPool::new(&rim.part.vertices);

        let other = section_corner::try_build(
            &leaving.reversed(),
            &arriving.reversed(),
            &rim.part,
            &rim.graph,
            &edge_at_pair,
            &mut backwards,
            RADIUS,
            SEGMENTS,
            None,
        )
        .unwrap_or_else(|failure| {
            panic!("the corner draws from either edge - instead {failure:?}")
        });

        let forwards = &run.sections[i];

        for k in 0..forwards.profile.len() {
            let here = rim.pool.position_of(forwards.profile[k]);
            let there = backwards.position_of(other.profile[other.profile.len() - 1 - k]);

            assert!(
                (here - there).length() < 1e-9,
                "sample {k} at vertex {} reads {here:?} one way and {there:?} the other",
                forwards.chain_vertex
            );
        }
    }
}

/// The profile's ends ARE the two boundaries the rule names: the offset-meet in the shared flat
/// face, and the setback down the fold.
#[test]
fn the_corners_ends_are_the_offset_meet_and_the_point_down_the_fold() {
    let rim = the_rim_run(Matrix4X4::IDENTITY);
    let run = &rim.run;
    let pool = &rim.pool;
    let count = run.count() as usize;

    for i in 0..count {
        let section = &run.sections[i];
        let arriving = run.edges[(i + count - 1) % count];
        let leaving = run.edges[i];
        let station = leaving.from();

        // The top face is the shared flat side of a box's rim, and the setback at ninety degrees is
        // the radius itself: s = r tan(45).
        let setback = RADIUS;

        assert!((section.setback - setback).abs() < 1e-9);

        let flat_on_the_left = arriving
            .left_normal()
            .get_normal()
            .dot(leaving.left_normal().get_normal())
            > 0.999;
        let flat = if flat_on_the_left {
            arriving.left_normal()
        } else {
            arriving.right_normal()
        }
        .get_normal();

        let g0 = away_in_the_flat_face(&arriving, flat, flat_on_the_left);
        let g1 = away_in_the_flat_face(&leaving, flat, flat_on_the_left);

        let meet = station + (setback * (g0 + g1) / (1.0 + g0.dot(g1)));
        let flat_end = pool.position_of(if flat_on_the_left {
            section.left_boundary
        } else {
            section.right_boundary
        });

        assert!(
            (flat_end - meet).length() < 1e-9,
            "the flat end stands at {flat_end:?}, and the offset-meet is {meet:?}"
        );

        // s sqrt(2) from the vertex along the diagonal, which is the ninety degree reading of it.
        assert!(((flat_end - station).length() - (setback * 2f64.sqrt())).abs() < 1e-9);

        // The box's walls are vertical and its top horizontal, so both edges lean one on the fold
        // and the wall end is the setback straight down.
        let wall_end = pool.position_of(if flat_on_the_left {
            section.right_boundary
        } else {
            section.left_boundary
        });

        assert!(
            (wall_end - (station - (setback * Vector3::UNIT_Z))).length() < 1e-9,
            "the wall end stands at {wall_end:?}, a setback down the fold from {station:?}"
        );
    }
}

/// The strip between two corners is still the rolling ball's quarter cylinder about the edge: every
/// sample of it stands a radius from the edge's own centre line.
#[test]
fn every_strip_stands_a_radius_from_its_own_edges_centre_line() {
    let rim = the_rim_run(Matrix4X4::IDENTITY);
    let run = &rim.run;

    for i in 0..run.count() {
        let edge = run.edges[i as usize];
        let cosine = edge
            .left_normal()
            .get_normal()
            .dot(edge.right_normal().get_normal());
        let axis = edge.from()
            - (RADIUS * (edge.left_normal().get_normal() + edge.right_normal().get_normal())
                / (1.0 + cosine));
        let along = edge.direction().get_normal();

        for section in [run.start_of(i), run.end_of(i).expect("a closed run")] {
            for &sample in &section.profile {
                let at = rim.pool.position_of(sample);
                let offset = at - axis;
                let reach = (offset - (offset.dot(along) * along)).length();

                assert!(
                    (reach - RADIUS).abs() < 1e-9,
                    "a strip sample stands {reach} from edge {i}'s centre line, not {RADIUS}"
                );
            }
        }
    }
}

/// The whole rim assembled: closed, manifold, one shell, and the same material removed as the
/// construction it replaces takes off the same pick.
#[test]
fn the_assembled_rim_is_one_closed_solid_and_removes_what_the_legacy_build_removes() {
    let mut rim = the_rim_run(Matrix4X4::IDENTITY);

    let built = section_assembly::try_assemble(&rim.part, &rim.graph, &rim.run, &mut rim.pool)
        .unwrap_or_else(|failure| panic!("the box's rim assembles - instead {failure:?}"));

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
    assert_eq!(open, 0, "open at {open_at:?}");

    assert_eq!(most_faces_on_an_edge(&built), 2);
    assert_eq!(shell_count(&built), 1);
    assert!(signed_volume(&built) > 0.0);

    let (unreadable, unreadable_at) = unclassifiable_edges(Some(&built));
    assert_eq!(
        unreadable, 0,
        "a second pass reads {unreadable_at} as a torn face"
    );

    let by_sections = signed_volume(&built);
    let by_legacy = signed_volume(&rim.legacy);
    let apart = (by_sections - by_legacy).abs() / by_legacy.abs();

    assert!(
        apart < 1e-3,
        "the sections enclose {by_sections} and the legacy build {by_legacy} - {apart} apart"
    );

    let on_the_legacy = farthest_from(&built, &rim.legacy);
    let on_the_sections = farthest_from(&rim.legacy, &built);

    assert!(on_the_legacy < 3e-3 * RADIUS, "farthest {on_the_legacy}");
    assert!(
        on_the_sections < 3e-3 * RADIUS,
        "farthest {on_the_sections}"
    );
}

/// Where the box stands changes nothing about what is assembled.
#[test]
fn where_the_box_stands_changes_none_of_the_assembly() {
    let signature = signature_of(Some(&assemble(Matrix4X4::IDENTITY)));

    for (name, motion) in the_motions() {
        let moved = assemble(motion);

        let (open, open_at) = surface_mesh_closure::open_edge_count(&moved);
        assert_eq!(
            open, 0,
            "the build moved by {name} is closed - open at {open_at:?}"
        );

        let (unreadable, unreadable_at) = unclassifiable_edges(Some(&moved));
        assert_eq!(
            unreadable, 0,
            "the build moved by {name} is readable - a second pass reads {unreadable_at} as a \
             torn face"
        );

        assert_eq!(
            signature_of(Some(&moved)),
            signature,
            "the build moved by {name} has the topology it had at the origin"
        );
    }
}

/// With the flag on, a face rim's corners no longer send the pass to the legacy path.
#[test]
fn the_rim_now_routes_to_the_sections() {
    let _sections = pinned_to_the_sections();

    the_rim_run(Matrix4X4::IDENTITY);

    assert_eq!(
        section_routing::last_route().as_deref(),
        Some("sections"),
        "the turn clause lets a corner through, and the build draws its mitre"
    );
}

/// The direction in the SHARED flat face, square to one chain edge and
/// pointing away from it - the rule's own `g = (c n - nOther) / sqrt(1 - c^2)`, written out here so
/// the offset-meet is read against the formula and not against the builder's arithmetic.
fn away_in_the_flat_face(edge: &ChainEdge, flat: Vector3, flat_on_the_left: bool) -> Vector3 {
    let wall = if flat_on_the_left {
        edge.right_normal()
    } else {
        edge.left_normal()
    }
    .get_normal();
    let cosine = flat.dot(wall);
    let across = (1.0 - (cosine * cosine)).max(0.0).sqrt();

    ((cosine * flat) - wall) / across
}

fn assemble(motion: Matrix4X4) -> Mesh {
    let mut rim = the_rim_run(motion);

    section_assembly::try_assemble(&rim.part, &rim.graph, &rim.run, &mut rim.pool).unwrap_or_else(
        |failure| panic!("the box assembles wherever it stands - instead {failure:?}"),
    )
}

/// The box, the run of sections the top face PICK's own plan draws on it, and the
/// mesh the construction this replaces builds from that same plan.
///
/// The rim is reached through the FACE GROUP pick and not through a bare chain walk: three feature
/// edges meet at every corner of a box, so a tangent walk stops at the first of them whatever
/// tolerance it is given. The face group reaches all four by the face they share.
fn the_rim_run(motion: Matrix4X4) -> RimRun {
    let mut part = platonic_solids::create_cube(SIDE, SIDE, SIDE);

    // Named at the origin and then the WHOLE part moved, so a moved build walks the same edges in
    // the same order and any difference downstream is the build's own.
    let seed = the_top_rim(&FeatureEdgeSet::create_with_graph(
        &part,
        &part.get_mesh_edge_graph(),
        10.0,
    ));

    move_mesh(&mut part, motion);

    let graph = part.get_mesh_edge_graph();

    let group = bevel_edge_group_resolver::create_from_edge(
        &FeatureEdgeSet::create_with_graph(&part, &graph, 10.0),
        seed,
        BevelSelectionKind::FaceGroup,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("the top rim is a crease of the box");

    let outcome = try_bevel(
        &part,
        &[group],
        RADIUS,
        SEGMENTS,
        &CancelToken::new(),
        TryBevelOptions::default(),
    );

    let legacy = match outcome.result {
        Ok(built) => built.mesh,
        Err(reason) => {
            panic!("the legacy construction fillets this rim - instead {reason:?}")
        }
    };

    let mut pool = SectionPool::new(&part.vertices);

    let run = section_builder_run::try_build_plan(
        Some(Rc::clone(&outcome.plans_built[0])),
        &part,
        &graph,
        &mut pool,
        RADIUS,
        SEGMENTS,
        None,
    )
    .unwrap_or_else(|failure| panic!("the box's top rim sections - instead {failure:?}"));

    RimRun {
        part,
        graph,
        pool,
        run,
        legacy,
    }
}

/// An edge of the box's top rim - both ends on the top face.
fn the_top_rim(features: &FeatureEdgeSet) -> i32 {
    for (i, edge) in features.edges().iter().enumerate() {
        if edge.start.z > (SIDE / 2.0) - 1e-6 && edge.end.z > (SIDE / 2.0) - 1e-6 {
            return i as i32;
        }
    }

    panic!("the box has no top rim");
}
