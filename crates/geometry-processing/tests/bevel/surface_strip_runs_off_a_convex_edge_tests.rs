//! Rung 10 (b), a band that takes the WHOLE of the face it lands on and runs OFF its convex rim
//! onto the plane beyond. The nose fixture's top edge at radius 15 is the shape: the top face is
//! 14.142 across off that edge, the tangent rail stands at 15 and so never touches the face over
//! the edge's own span, and the strip is cut instead along the ellipse the x = 0 plane draws on it.
//! Plus the run-off reading's refusals on two bench solids (a tent and a chorded arc beyond the
//! rim).

use std::collections::HashSet;
use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::surface_band_runs_off_the_face_beyond::{
    self as run_off, BandSpan, TheFaceBeyond,
};
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, surface_room, surface_room_measure,
    SurfaceBevelPlan, SurfaceBuildOptions, SurfaceRoomCut,
};
use geometry_processing::bevel::{BevelAdvisory, BevelAdvisoryCode};
use geometry_processing::topology::{FaceGroup, FaceRegionCache, FeatureEdgeSet};
use polygon_mesh::csg::manifold_import;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{Vector2, Vector3};

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::surface_walkback_checks::regions_of;
use crate::{lobed_prism_meshes, nose_in_notch_test_meshes};

/// A mesh keeps its vertices as floats, so a distance of 20 is a distance of 20 to about this.
const NEAR: f64 = 1e-5;

/// The top face's width off the chain edge, which is also the edge's own length.
const ACROSS_THE_TOP: f64 = 14.142135623730951;

/// The room reading turns over exactly where the last of the face goes. Under 14.142 the tangent
/// line still crosses the top face inside the edge's span, so the x = 0 rim is the wall it runs out
/// at; at 15 the band takes the whole triangle and runs off that rim onto the x = 0 plane, which is
/// not a size that refuses.
#[test]
fn taking_the_whole_top_face_is_run_off_rather_than_running_out_of_room() {
    let part = nose_in_notch_test_meshes::create();

    let still_lands = top_cut_of(&top_edge_plan_of(&part, 14.0));

    assert!(
        (still_lands.room - ACROSS_THE_TOP).abs() <= NEAR,
        "at 14 the tangent line is still on the top face, so its x = 0 rim is a wall - {}",
        still_lands.room
    );

    let runs_off = top_cut_of(&top_edge_plan_of(&part, 15.0));

    assert!(
        runs_off.room.is_infinite() && runs_off.room > 0.0,
        "at 15 nothing of the top face survives the band over the edge's own span, so there is no \
         far side to run out at - the bevel runs off the x = 0 rim. Measured {}",
        runs_off.room
    );

    let fit = surface_room::try_fit(
        &[top_edge_plan_of(&part, 15.0)],
        15.0,
        &CancelToken::new(),
        None,
    )
    .expect("not cancelled");
    assert!(
        fit.fits,
        "and so 15 fits the part rather than naming 14.142 - {:?}",
        fit.refusal
    );
    assert!((fit.largest_fitting_size - 15.0).abs() <= NEAR);
}

/// The predicate itself, both ways round, and what it hands the trim: the plane beyond is the
/// x = 0 face's own, pointing out of the part.
#[test]
fn the_run_off_names_the_plane_beyond_the_rim_it_runs_off_async() {
    let part = nose_in_notch_test_meshes::create();
    let plan = top_edge_plan_of(&part, 15.0);
    let edge = &plan.edges[0];
    let along = (edge.to() - edge.from()).get_normal();
    let length = (edge.to() - edge.from()).length();

    let beveled = HashSet::from([edge.edge.edge.mesh_edge_index]);
    let corners = HashSet::from([edge.edge.edge.start_vertex, edge.edge.edge.end_vertex]);

    let read = |region: &FaceGroup, inward: Vector3, setback: f64| {
        run_off::try_read(
            Some(&part),
            Some(region),
            &BandSpan {
                from: edge.from(),
                along,
                length,
                inward,
                setback,
            },
            &beveled,
            &corners,
        )
    };

    let beyond = taken_whole(
        read(
            &edge.left_region,
            edge.left_inward_at_the_edge.get_normal(),
            edge.left_setback,
        ),
        "the top face is wholly taken at 15",
    );

    assert!(
        (beyond.plane_normal - Vector3::new(-1.0, 0.0, 0.0)).length() < NEAR,
        "what is beyond the top face's x = 0 rim is the x = 0 face, facing out - {}",
        beyond.plane_normal
    );
    assert!(
        beyond.plane_point.x.abs() < NEAR,
        "and the plane stands at x = 0 - {}",
        beyond.plane_point
    );

    assert!(
        beyond.trims_the_start,
        "walked from the knife tip the rails are in the air at the START - rail 0 crosses x = 0 at \
         15, past the edge's own 14.142, and every rail below it crosses earlier"
    );

    assert!(
        !matches!(
            read(
                &edge.left_region,
                edge.left_inward_at_the_edge.get_normal(),
                14.0
            ),
            Ok(Some(_))
        ),
        "at 14 the face is still there to land on"
    );

    assert!(
        !matches!(
            read(
                &edge.right_region,
                edge.right_inward_at_the_edge.get_normal(),
                edge.right_setback
            ),
            Ok(Some(_))
        ),
        "and the slope below is 20 deep, which 15 does not take the whole of"
    );
}

/// The same shape walked the OTHER WAY: the chain runs from the crease corner to the knife tip, so
/// the tangent line crosses the x = 0 plane just BEFORE the span rather than just after it, and the
/// rails are cut back from the far end. Nothing about the shape changed, so the reading may not.
#[test]
fn the_same_run_off_reads_the_same_walked_the_other_way() {
    let part = nose_in_notch_test_meshes::create();
    let plan = top_edge_plan_of(&part, 15.0);
    let edge = &plan.edges[0];
    let length = (edge.to() - edge.from()).length();
    let backwards = -(edge.to() - edge.from()).get_normal();

    let beveled = HashSet::from([edge.edge.edge.mesh_edge_index]);
    let corners = HashSet::from([edge.edge.edge.start_vertex, edge.edge.edge.end_vertex]);

    let beyond = taken_whole(
        run_off::try_read(
            Some(&part),
            Some(&edge.left_region),
            &BandSpan {
                from: edge.to(),
                along: backwards,
                length,
                inward: edge.left_inward_at_the_edge.get_normal(),
                setback: edge.left_setback,
            },
            &beveled,
            &corners,
        ),
        "the top face is wholly taken whichever way the chain was linked",
    );

    assert!(
        (beyond.plane_normal - Vector3::new(-1.0, 0.0, 0.0)).length() < NEAR,
        "and it is the same plane beyond - {}",
        beyond.plane_normal
    );

    assert!(
        !beyond.trims_the_start,
        "walked back to the tip the rails are in the air at the END instead: the tangent line \
         crosses x = 0 at -0.858, before the span, and is outside it the whole way after"
    );
}

/// A band on the lobed prism's top face takes the whole of it at 25 and runs off - and is NOT this
/// construction, because it runs off PARALLEL: there is no staircase to cut it back along, and the
/// honest answer is the size the face has room for.
#[test]
fn a_band_running_off_a_wall_parallel_is_still_a_size_that_does_not_fit() {
    let prism = Arc::new(lobed_prism_meshes::create());
    let features = FeatureEdgeSet::create(&prism);
    let regions = Arc::new(FaceRegionCache::new(
        prism.clone(),
        Some(prism.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    ));

    // The lobe's furthest facet, at the top: its top rim edge, which the top face lands beside.
    let on_the_lobe: Vec<_> = features
        .edges()
        .iter()
        .filter(|edge| {
            (edge.start - Vector3::new(20.0, -8.0, lobed_prism_meshes::HEIGHT)).length() < 1e-6
                && (edge.end - Vector3::new(21.564, -7.877, lobed_prism_meshes::HEIGHT)).length()
                    < 1e-6
        })
        .copied()
        .collect();
    assert_eq!(on_the_lobe.len(), 1);

    let mut chains = bevel_chain_linker::link(&on_the_lobe);
    assert_eq!(chains.len(), 1);
    let chain = chains.remove(0);

    let plan =
        SurfaceBevelPlan::try_create(&chain, &regions, 25.0, true).unwrap_or_else(|reason| {
            panic!("the edge itself plans - it is the room that has something to say - {reason}")
        });

    let fit =
        surface_room::try_fit(&[plan], 25.0, &CancelToken::new(), None).expect("not cancelled");
    assert!(
        !fit.fits,
        "25 takes the whole of the top face off that edge"
    );

    let refusal = fit.refusal.expect("a refusal names the size");
    assert_eq!(
        refusal.code(),
        BevelAdvisoryCode::SizeExceedsFaceWidth,
        "and the honest answer is the width the face has, because the run-off there is parallel \
         and nothing trims it - {refusal}"
    );
    assert!(
        (fit.largest_fitting_size - lobed_prism_meshes::DEPTH).abs() <= 1e-6,
        "which is the prism's own depth - {}",
        fit.largest_fitting_size
    );
}

/// The whole solid, at the chording the kernel's own tests build at. The exact solid is 5563.424
/// (the 521.079 the exact round takes off, and the 28.947 the nose's extended facets carry back)
/// and the chorded sequence closes on it from below.
#[test]
fn the_whole_top_edge_rounds_at_fifteen() {
    for (segments_per_quarter, volume, shared_z) in
        [(8, 5555.302, 19.915508), (16, 5561.387, 19.957835)]
    {
        let part = Arc::new(nose_in_notch_test_meshes::create());
        let plan = top_edge_plan_of(&part, 15.0);

        let fit =
            surface_room::try_fit(std::slice::from_ref(&plan), 15.0, &CancelToken::new(), None)
                .expect("not cancelled");
        assert!(
            fit.fits,
            "the band runs off the top face's x = 0 rim rather than running out of room - {:?}",
            fit.refusal
        );

        let (regions, _) = regions_of(&part);
        let built = surface_bevel_builder::try_build(
            &part,
            &part.get_mesh_edge_graph(),
            &regions,
            &[plan],
            segments_per_quarter,
            &CancelToken::new(),
            SurfaceBuildOptions {
                crossings: Some(&fit.crossings),
                ..SurfaceBuildOptions::default()
            },
        )
        .unwrap_or_else(|reason| panic!("and so it builds - {reason:?}"))
        .mesh;

        let faces: Vec<(i32, i32, i32)> = built
            .faces
            .iter()
            .map(|face| (face.v0, face.v1, face.v2))
            .collect();

        let (folded, folded_at) = surface_mesh_closure::folded_edge_count(&faces);
        assert_eq!(folded, 0, "nothing may double back - instead {folded_at:?}");
        let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
        assert_eq!(
            open, 0,
            "the trim shares every vertex it makes, so the surface closes - open at {open_at}"
        );
        assert!(built.is_manifold(), "and closes manifold");
        assert_eq!(shell_count(&built), 1, "in one piece");

        assert!(
            manifold_import::import_strict(&built).is_ok(),
            "the boolean kernel has to take it"
        );

        let measured = signed_volume(&built);
        assert!(
            (measured - volume).abs() <= 0.5,
            "chorded at {segments_per_quarter} segments to the quarter, closing on the exact \
             5563.424 from below - {measured}, expected {volume}"
        );

        // The ellipse the x = 0 plane cuts the profile's cylinder in, at its two ends. The low one
        // is the strip's own tangent foot and is exact; the high one is where the LAST chord of the
        // profile is clipped at x = 0, so it chords with everything else.
        assert!(
            carries(&built, Vector3::new(0.0, 0.0, 5.0)) > 0,
            "the arc starts at the tangent foot on the slope"
        );

        let shared = Vector3::new(0.0, 20.0, shared_z);

        assert_eq!(
            vertices_at(&built, shared),
            1,
            "the arc's far end is ONE vertex, minted once by the trim and handed to the strip, to \
             the x = 0 section and to the end face - at z {shared_z} for {segments_per_quarter} \
             segments, closing on the analytic 19.975449. Two of them a rounding apart is the \
             crack this rung is about"
        );

        assert!(
            carries(&built, shared) >= 6,
            "and all three of those rims run through it, so it is no loose end"
        );
    }
}

/// The band takes the whole of its landing face and runs off onto TWO FLATS meeting at a ridge,
/// which is refused as a CORNER and not as a curve: the far side is a tent bending 65.9 degrees at
/// its apex, more than the 30 degree tolerance a surface is grown at.
#[test]
fn running_off_onto_two_flats_at_a_corner_is_named_a_corner() {
    let bench = run_off_bench(&[
        Vector2::new(25.0, 1.5),
        Vector2::new(10.0, 12.0),
        Vector2::new(-5.0, 3.0),
    ]);

    let failure = the_bench_run_off(&bench).expect_err(
        "what is beyond the rim is two planes, which this construction does not trim against - \
         refused by NAME rather than falling through to a size the wall rule would answer with",
    );

    assert_eq!(
        failure.code(),
        BevelAdvisoryCode::SurfaceStripRunsOffAtACorner,
        "the two walls beyond are FLAT and meet at a corner - nothing here curves - {failure}"
    );
}

/// The same bench with its far side drawn as a chorded ARC instead of a tent - thirteen facets of
/// one r = 17 circle - keeps the curved name: the surface beyond the rim floods through those
/// facets and what it floods to is not one plane.
#[test]
fn running_off_onto_a_chorded_curve_keeps_the_curved_name() {
    let bench = run_off_bench(&the_chorded_arc());

    let failure = the_bench_run_off(&bench).expect_err(
        "what is beyond the rim is a curved wall, which this construction does not trim against",
    );

    assert_eq!(
        failure.code(),
        BevelAdvisoryCode::SurfaceStripRunsOffACurvedFace,
        "the facets beyond are one tangent-continuous surface and it is not flat - {failure}"
    );
}

/// How tall the bench prisms are, so their top face stands at this z.
const BENCH_HEIGHT: f64 = 10.0;

/// How deep the band is driven on the bench, more than the 12 its top face is anywhere deep.
const BENCH_SETBACK: f64 = 30.0;

/// The span the reading is made over, an inner stretch of the bench's own y = 0 top edge.
const BENCH_SPAN: f64 = 20.0;

/// The bench arc's radius, and the y its centre stands at.
const ARC_RADIUS: f64 = 17.0;

const ARC_CENTRE_Y: f64 = -5.0;

/// A run-off reading that took the face whole, or the reason it did not.
fn taken_whole(read: Result<Option<TheFaceBeyond>, BevelAdvisory>, because: &str) -> TheFaceBeyond {
    match read {
        Ok(Some(beyond)) => beyond,
        Ok(None) => panic!("{because} - read as no run-off"),
        Err(why) => panic!("{because} - {why}"),
    }
}

/// A bench solid for the run-off reading: the prism of an outline whose `y = 0` side carries the
/// chain edge, walked from `x = -5` to `x = 25`, with `far_boundary` walked back right to left.
fn run_off_bench(far_boundary: &[Vector2]) -> Mesh {
    let mut path = agg_path::vertex_storage::VertexStorage::new();

    path.move_to(-5.0, 0.0);
    path.line_to(25.0, 0.0);

    for corner in far_boundary {
        path.line_to(corner.x, corner.y);
    }

    path.close_polygon();

    polygon_mesh::processors::vertex_source_to_mesh::extrude(&path, BENCH_HEIGHT)
        .expect("the bench outline extrudes")
}

/// The bench's far side as a chorded arc of one circle, apex (10, 12) and ends (25, 3) and (-5, 3).
fn the_chorded_arc() -> Vec<Vector2> {
    (0..=12)
        .map(|step| {
            let x = 25.0 - (step as f64 * 2.5);

            Vector2::new(
                x,
                ARC_CENTRE_Y + ((ARC_RADIUS * ARC_RADIUS) - ((x - 10.0) * (x - 10.0))).sqrt(),
            )
        })
        .collect()
}

/// The run-off reading on a bench solid, over the inner span of its `y = 0` top edge, at a setback
/// that takes the whole of the top face.
fn the_bench_run_off(bench: &Mesh) -> Result<Option<TheFaceBeyond>, BevelAdvisory> {
    let bench_arc = Arc::new(bench.clone());
    let features = FeatureEdgeSet::create(bench);
    let regions = FaceRegionCache::new(
        bench_arc,
        Some(bench.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    );
    let start = Vector3::new(-5.0, 0.0, BENCH_HEIGHT);
    let end = Vector3::new(25.0, 0.0, BENCH_HEIGHT);

    let chain: Vec<_> = features
        .edges()
        .iter()
        .filter(|candidate| {
            ((candidate.start - start).length() < 1e-6 && (candidate.end - end).length() < 1e-6)
                || ((candidate.end - start).length() < 1e-6
                    && (candidate.start - end).length() < 1e-6)
        })
        .copied()
        .collect();
    assert_eq!(chain.len(), 1);

    let edge = chain[0];
    let top = if (edge.normal0 - Vector3::UNIT_Z).length() < 1e-6 {
        edge.face0
    } else {
        edge.face1
    };
    let region = regions.region(top);

    run_off::try_read(
        Some(bench),
        region.as_deref(),
        &BandSpan {
            from: Vector3::new(0.0, 0.0, BENCH_HEIGHT),
            along: Vector3::UNIT_X,
            length: BENCH_SPAN,
            inward: Vector3::UNIT_Y,
            setback: BENCH_SETBACK,
        },
        &HashSet::from([edge.mesh_edge_index]),
        &HashSet::from([edge.start_vertex, edge.end_vertex]),
    )
}

/// How many distinct VERTICES of the built mesh stand at one point, within a rounding.
fn vertices_at(built: &Mesh, at: Vector3) -> usize {
    built
        .vertices
        .iter()
        .filter(|&&vertex| (Vector3::from_vector3_float(vertex) - at).length() < 1e-3)
        .count()
}

/// How many face corners of the built mesh stand at one point, within a rounding.
fn carries(built: &Mesh, at: Vector3) -> usize {
    built
        .faces
        .iter()
        .flat_map(|face| [face.v0, face.v1, face.v2])
        .filter(|&vertex| {
            (Vector3::from_vector3_float(built.vertices[vertex as usize]) - at).length() < 1e-3
        })
        .count()
}

/// The band on the TOP face, which is the side this rung is about.
fn top_cut_of(plan: &Rc<SurfaceBevelPlan>) -> SurfaceRoomCut {
    let cuts: Vec<SurfaceRoomCut> =
        surface_room_measure::measure(std::slice::from_ref(plan), None, None)
            .into_iter()
            .filter(|cut| !cut.corner_footprint() && cut.side == 0)
            .collect();
    assert_eq!(cuts.len(), 1, "one band on the top face");

    cuts.into_iter().next().expect("one cut")
}

/// The fixture's own top edge, planned as a round at one size.
fn top_edge_plan_of(part: &Mesh, size: f64) -> Rc<SurfaceBevelPlan> {
    let features = FeatureEdgeSet::create(part);
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(part.clone()),
        Some(part.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    ));
    let from = Vector3::new(0.0, 0.0, nose_in_notch_test_meshes::SIZE);
    let to = Vector3::new(10.0, 10.0, nose_in_notch_test_meshes::SIZE);

    let top: Vec<_> = features
        .edges()
        .iter()
        .filter(|candidate| {
            ((candidate.start - from).length() < 1e-6 && (candidate.end - to).length() < 1e-6)
                || ((candidate.end - from).length() < 1e-6
                    && (candidate.start - to).length() < 1e-6)
        })
        .copied()
        .collect();
    assert_eq!(top.len(), 1);

    let mut chains = bevel_chain_linker::link(&top);
    assert_eq!(chains.len(), 1);
    let chain = chains.remove(0);

    SurfaceBevelPlan::try_create(&chain, &regions, size, true).unwrap_or_else(|reason| {
        panic!("the fixture has to plan before its room can be measured: {reason}")
    })
}
