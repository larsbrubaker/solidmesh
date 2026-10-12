//! Port of `SurfaceRoomStandingFeatureTests.cs`: a feature HANGING over the face a band lands on is
//! not the far side of that face (`surface_room_standing_feature::stands_on`). The room across a
//! region stops at a wall - a convex rim it runs off, or a wall standing square on it - and runs on
//! underneath anything leaning out over it. The nose in the chevron notch
//! (`nose_in_notch_test_meshes`) leans out over both slopes; the rib on the plate
//! (`topology_test_meshes::create_rib_on_plate`) stands square on it. The chorded fixture is the
//! same nose-in-notch solid with its upper right crease cut in two, as the boolean behind "Center
//! Flows Through.mcx" leaves it.

use std::collections::HashSet;
use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    surface_face_walkback, surface_mesh_closure, surface_room, surface_room_measure,
    surface_room_standing_feature, surface_shared_edge_network, SurfaceBevelPlan,
    SurfaceWalkbackResult,
};
use geometry_processing::bevel::{BevelAdvisory, BevelAdvisoryCode};
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelEdgeGroup, FaceGroup, FaceRegionCache, FeatureEdge,
    FeatureEdgeSet,
};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::mesh_measurements;
use crate::nose_in_notch_test_meshes;
use crate::surface_test_chains;
use crate::surface_walkback_checks::regions_of;
use crate::topology_test_meshes;

/// A mesh keeps its vertices as floats, so a distance of 20 is a distance of 20 to about this.
const NEAR: f64 = 1e-5;

/// The size "Center Flows Through.mcx" carries, which is what its two pieces are read at.
const SIZE: f64 = 6.0;

/// The 15 mm fixture's own edge - the left slope's top edge from the knife tip at (0, 0, 20) to the
/// crease corner at (10, 10, 20). The slope below it is the whole 20 mm deep: the nose reaches the
/// centre crease 6.667 down, but it HANGS there, and the crease wall it interrupts goes on behind
/// it to the floor.
#[test]
fn a_nose_hanging_over_a_slope_is_not_the_far_side_of_it() {
    let part = Arc::new(nose_in_notch_test_meshes::create());
    let plan = top_edge_plan_of(&part, 3.0);

    let mut rooms: Vec<f64> =
        surface_room_measure::measure(std::slice::from_ref(&plan), None, None)
            .iter()
            .filter(|cut| !cut.corner_footprint())
            .map(|cut| cut.room)
            .collect();
    rooms.sort_by(f64::total_cmp);

    assert_eq!(
        rooms.len(),
        2,
        "one edge, two regions: the top face and the slope under it"
    );

    assert!(
        (rooms[0] - 10.0 * 2f64.sqrt()).abs() <= NEAR,
        "the TOP face runs out at its own convex x = 0 edge, 14.142 across - measured {}",
        rooms[0]
    );

    assert!(
        (rooms[1] - 20.0).abs() <= NEAR,
        "the SLOPE is the whole 20 mm down to the floor. The nose's two creases hang over it and \
         bound nothing, and the stub of centre crease they interrupt at z = 13.333 is not the end \
         of that wall either - reading either as one said the slope was 6.667 deep, which is where \
         the nose reaches the crease and not where the face stops. Measured {}",
        rooms[1]
    );

    let fit = surface_room::try_fit(&[plan], 15.0, &CancelToken::new(), None).expect("no cancel");
    assert!(
        !fit.fits,
        "the fixture's saved 15 still does not fit - the top face is only 14.142 across"
    );
    assert!(
        (fit.largest_fitting_size - 10.0 * 2f64.sqrt()).abs() <= NEAR,
        "what fits is the narrower of the two, which is the top face's 14.142 - {:?}",
        fit.refusal
    );
}

/// The other answer, unchanged: a rib standing SQUARE on a plate is a wall. Nothing hangs over the
/// plate's top, so a band that reached the rib's foot would take the plate out from under the rib's
/// end wall - and the room across the plate over the rib's own stretch of the rim is the 5 mm to
/// that foot, whatever the 20 mm plate says.
///
/// The stretches either SIDE of the rib have the whole 20 mm, and say so here. The chain still fits
/// at 5 and no more, because the stretch the rib really does stand on is one of its three edges and
/// the fit is the narrowest of them.
#[test]
fn a_rib_standing_square_on_a_plate_still_bounds_the_room() {
    let plate = Arc::new(topology_test_meshes::create_rib_on_plate());
    let (regions, features) = regions_of(&plate);
    let regions = Arc::new(regions);

    // The plate's whole front top rim at z = 5, in three edges: the rib's grid puts vertices at
    // x = 8 and x = 12 along it.
    let rim: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| {
            (edge.start.z - 5.0).abs() < 1e-6
                && (edge.end.z - 5.0).abs() < 1e-6
                && edge.start.y.abs() < 1e-6
                && edge.end.y.abs() < 1e-6
        })
        .copied()
        .collect();

    let chains = bevel_chain_linker::link(&rim);
    assert_eq!(chains.len(), 1);

    let plan = SurfaceBevelPlan::try_create(&chains[0], &regions, 8.0, false)
        .unwrap_or_else(|reason| panic!("the edges themselves are sound - {reason}"));

    for cut in surface_room_measure::measure(std::slice::from_ref(&plan), None, None)
        .iter()
        .filter(|cut| !cut.corner_footprint())
    {
        // Which stretch of the rim this cut is on, and which way it goes: down the front face, or
        // back across the top.
        let across_the_top = cut.inward.y > 0.5;
        let under_the_rib = (((cut.from + cut.to) / 2.0).x - 10.0).abs() < 2.0;
        let expected = if across_the_top && !under_the_rib {
            20.0
        } else {
            5.0
        };

        assert!(
            (cut.room - expected).abs() <= NEAR,
            "the plate's front face is 5 mm thick, its top runs 5 mm to the rib's free-end foot \
             over the rib's own stretch of the rim and the whole 20 mm either side of it - this \
             one, {} from {} to {}, measured {}",
            if across_the_top {
                "across the top"
            } else {
                "down the front"
            },
            cut.from,
            cut.to,
            cut.room
        );
    }

    let fit = surface_room::try_fit(&[plan], 8.0, &CancelToken::new(), None).expect("no cancel");
    assert!(
        !fit.fits,
        "8 mm would reach the rib's foot at y = 5 and leave its end wall on nothing"
    );
    assert!((fit.largest_fitting_size - 5.0).abs() <= NEAR);
    assert_eq!(
        fit.refusal.map(|refusal| refusal.code()),
        Some(BevelAdvisoryCode::SizeExceedsFaceWidth)
    );
}

/// The predicate itself, on the fixture both readings come from: the nose's creases stand on the
/// slope, the crease between the two slopes and the cube's own convex rim do not.
#[test]
fn only_a_rim_leaning_out_over_the_region_reads_as_standing_on_it() {
    let part = Arc::new(nose_in_notch_test_meshes::create());
    let (regions, features) = regions_of(&part);
    let top = features.edges()[top_edge_of(&features)];
    let mut slope = regions.region(top.face0).expect("a region");

    if part.faces[slope.seed_face() as usize].normal.z.abs() > 0.5 {
        slope = regions.region(top.face1).expect("a region");
    }

    // Nothing is being beveled in this reading - the predicate is asked of the rim alone.
    let nothing_picked = HashSet::new();

    let mut standing: Vec<FeatureEdge> = slope
        .boundary_edges()
        .iter()
        .filter(|rim| {
            surface_room_standing_feature::stands_on(&slope, rim, &nothing_picked, &nothing_picked)
        })
        .copied()
        .collect();
    standing.sort_by(|a, b| a.start.z.total_cmp(&b.start.z));

    assert_eq!(
        standing.len(),
        2,
        "the nose's two creases on this slope hang over it, and nothing else on its rim does - the \
         centre crease's two stubs are square to it and every other rim edge is convex"
    );

    for rim in &standing {
        assert!(
            (rim.start.x - rim.start.y).abs() < 1e-6 && (rim.end.x - rim.end.y).abs() < 1e-6,
            "both of them run in the slope's own plane y = x - {} to {}",
            rim.start,
            rim.end
        );
    }
}

/// The one predicate answers the room and the walkback alike, exclusions and all: a crease that
/// MEETS the chain at a corner is not a feature the island will close, so it is not let off in the
/// room either. The nose's upper right crease at the ridge end is that case whenever the LOWER
/// right one is the thing being beaded (`SurfaceBeadPastAKnifeEdgeTipTests`).
#[test]
fn a_feature_meeting_the_chain_goes_on_bounding_the_room() {
    let part = Arc::new(nose_in_notch_test_meshes::create());
    let (regions, features) = regions_of(&part);

    let lower = features.edges()[crease_between(
        &features,
        nose_in_notch_test_meshes::right_ridge_end(),
        nose_in_notch_test_meshes::lower_crease_end(),
    )];
    let upper = features.edges()[crease_between(
        &features,
        nose_in_notch_test_meshes::right_ridge_end(),
        nose_in_notch_test_meshes::upper_crease_end(),
    )];

    // The right slope, which both creases stand on.
    let mut slope = regions.region(lower.face0).expect("a region");

    if part.faces[slope.seed_face() as usize].normal.z.abs() > 0.5 {
        slope = regions.region(lower.face1).expect("a region");
    }

    let nothing_picked = HashSet::new();

    assert!(
        surface_room_standing_feature::stands_on(&slope, &upper, &nothing_picked, &nothing_picked),
        "the nose hangs over the slope, so with nothing picked the upper crease is a feature"
    );

    let picked = HashSet::from([lower.mesh_edge_index]);
    let corners = HashSet::from([lower.start_vertex, lower.end_vertex]);

    assert!(
        !surface_room_standing_feature::stands_on(&slope, &upper, &picked, &corners),
        "the two creases meet at the ridge end, so a bead along the lower one has the upper one at \
         its own corner - the island will not close that, and the room must not grant the setback \
         for a root nothing closes"
    );
}

/// Where the boolean behind "Center Flows Through.mcx" chords the upper right crease: its t = 0.6,
/// a mesh vertex that is no corner of the shape at all.
fn the_chord() -> Vector3 {
    Vector3::new(12.0, 8.0, 12.0)
}

/// The chorded fixture is the same solid: one more vertex and two more facets, and not a
/// millimetre of difference anywhere. Worth pinning on its own, because everything the rung below
/// measures is only interesting while the two meshes are the same shape told two ways.
#[test]
fn the_crease_chorded_in_two_is_the_same_solid() {
    let chorded = nose_in_notch_test_meshes::create_with_upper_right_crease_split_at(the_chord());

    assert_eq!(
        chorded.vertices.len(),
        15,
        "the fixture's 14 vertices and the one the chord adds"
    );

    assert_eq!(
        chorded.faces.len(),
        26,
        "the plain fixture's 24, with the upper nose face and the slope's fan triangle across the \
         same crease each cut in two"
    );

    let (closed, open_at) = surface_mesh_closure::is_closed(&chorded);
    assert!(
        closed,
        "both sides of the chorded crease are cut at the same point, so nothing may be left open - \
         and something is at {open_at}"
    );

    assert!(
        chorded.is_manifold(),
        "a vertex on a crease that only one of its two faces knew about would leave the edges \
         either side of it with one face each"
    );

    let volume = mesh_measurements::signed_volume(&chorded);
    assert!(
        (volume - nose_in_notch_test_meshes::VOLUME).abs() <= 1e-3,
        "the chord lies ON the crease, so the solid measures the fixture's own {:.3} - measured \
         {volume}",
        nose_in_notch_test_meshes::VOLUME
    );
}

/// A crease is ONE crease however the mesh chords it. The boolean behind "Center Flows
/// Through.mcx" cuts the upper right crease in two at [`the_chord`], and only the far piece touches
/// the corner the crease dies in - so read edge by edge, the near piece is a feature standing on
/// the slope, the whole upper nose face is withheld from the walkback, and the upper strip's lower
/// end then finds no face to end on at all (SurfaceStripEndNotSupported). Read along the crease's
/// own LINE, both pieces meet the chain at that corner and nothing is withheld, which is what the
/// same part built clean does.
#[test]
fn a_crease_chorded_in_two_still_meets_the_chain_at_its_corner() {
    let part =
        Arc::new(nose_in_notch_test_meshes::create_with_upper_right_crease_split_at(the_chord()));
    let features = FeatureEdgeSet::create(&part);
    let (groups, _) = nose_in_notch_test_meshes::centre_flows_through_groups(&features);

    let (walked, plans) = try_walkback(&part, &groups, SIZE).unwrap_or_else(|reason| {
        panic!("the two pieces of centre crease plan and walk back on this part - instead {reason}")
    });

    // Named only when there is something to name: the creases are read off the walked mesh, and on
    // the passing path there is nothing to read.
    let withheld = if walked.standing_features.is_empty() {
        String::new()
    } else {
        creases_of(&walked)
    };

    assert_eq!(
        walked.standing_features.len(),
        0,
        "nothing stands on either slope here that the chain does not itself meet: every one of the \
         nose's four creases runs into a chain end, chorded or not. What is withheld instead: \
         {withheld}"
    );

    // And the predicate itself, on the piece that does not touch the corner.
    let (slope, near_piece) = the_right_slope_of(&part, &features);

    let (beveled_mesh_edges, chain_vertices) = surface_face_walkback::the_chains_own(&plans);

    assert!(
        !surface_room_standing_feature::stands_on(
            &slope,
            &near_piece,
            &beveled_mesh_edges,
            &chain_vertices
        ),
        "the piece {} to {} is the ridge end's half of one crease that dies on the chain at (10, \
         10, 13.333) - the corner's business, and not an island's",
        near_piece.start,
        near_piece.end
    );
}

/// The mirror, on the same chorded part: a run that reaches no chain vertex goes on standing. With
/// only the piece of crease BELOW the nose picked, the chain ends at (10, 10, 0) and (10, 10, 6.667)
/// and the upper right crease meets neither - so both of its chords are features hanging over the
/// slope, as the whole crease is when it is one edge.
#[test]
fn a_chorded_crease_that_reaches_no_chain_vertex_goes_on_standing() {
    let part =
        Arc::new(nose_in_notch_test_meshes::create_with_upper_right_crease_split_at(the_chord()));
    let features = FeatureEdgeSet::create(&part);
    let (groups, names) = nose_in_notch_test_meshes::centre_flows_through_groups(&features);

    assert_eq!(names[0], "below the nose");

    let (_, plans) = try_walkback(&part, &groups[..1], SIZE).unwrap_or_else(|reason| {
        panic!("the lower piece alone plans and walks back - instead {reason}")
    });

    let (slope, near_piece) = the_right_slope_of(&part, &features);

    let (beveled_mesh_edges, chain_vertices) = surface_face_walkback::the_chains_own(&plans);

    assert!(
        surface_room_standing_feature::stands_on(
            &slope,
            &near_piece,
            &beveled_mesh_edges,
            &chain_vertices
        ),
        "the run {} to (10, 10, 13.333) ends at the upper crease end, which this chain never \
         reaches - so reading along the line lets nothing off that reading the one edge would not",
        near_piece.start
    );
}

/// The right slope, and the chord of the upper right crease that touches no corner of the shape.
fn the_right_slope_of(
    part: &Arc<Mesh>,
    features: &FeatureEdgeSet,
) -> (Arc<FaceGroup>, FeatureEdge) {
    let regions = FaceRegionCache::new(
        part.clone(),
        Some(part.get_mesh_edge_graph()),
        features.flat_angle_degrees(),
    );
    let chord = features.edges()[crease_between(
        features,
        nose_in_notch_test_meshes::right_ridge_end(),
        the_chord(),
    )];
    let mut slope = regions.region(chord.face0).expect("a region");

    if part.faces[slope.seed_face() as usize].normal.z.abs() > 0.5 {
        // That one is the nose's upper face, which leans 0.832 out of the horizontal; the slope is
        // the other side of the crease and stands in the vertical plane y = 20 - x.
        slope = regions.region(chord.face1).expect("a region");
    }

    let near_piece: Vec<FeatureEdge> = slope
        .boundary_edges()
        .iter()
        .filter(|rim| rim.mesh_edge_index == chord.mesh_edge_index)
        .copied()
        .collect();
    assert_eq!(near_piece.len(), 1);

    (slope, near_piece[0])
}

/// Which creases a walkback held back, for a failure to say so by.
fn creases_of(walked: &SurfaceWalkbackResult) -> String {
    walked
        .standing_features
        .iter()
        .flat_map(|feature| feature.faces.iter())
        .map(|face| {
            let (from, to) = face.crease();
            format!(
                "{} to {}",
                walked.mesh.vertices[from as usize].as_vector3(),
                walked.mesh.vertices[to as usize].as_vector3()
            )
        })
        .collect::<Vec<_>>()
        .join(", ")
}

/// One part's groups planned and walked back, by the surface builder's own steps and in its order,
/// stopping at the walkback - which is where what stands on the part is read.
fn try_walkback(
    part: &Arc<Mesh>,
    groups: &[BevelEdgeGroup],
    size: f64,
) -> Result<(SurfaceWalkbackResult, Vec<Rc<SurfaceBevelPlan>>), BevelAdvisory> {
    let (resolutions, _) = bevel_edge_group_resolver::resolve_all(groups, part);
    let graph = part.get_mesh_edge_graph();
    let regions = Arc::new(FaceRegionCache::new(
        part.clone(),
        Some(graph.clone()),
        resolutions[0].features.flat_angle_degrees(),
    ));
    let curve_regions = Arc::new(FaceRegionCache::new(
        part.clone(),
        Some(graph.clone()),
        resolutions[0].group.curve_tolerance_degrees,
    ));

    let chains = surface_test_chains::join_and_open(&surface_shared_edge_network::link_together(
        &resolutions,
    ));

    let mut plans = Vec::new();

    for chain in &chains {
        plans.push(SurfaceBevelPlan::try_create_with_curve_regions(
            chain,
            &regions,
            Some(&curve_regions),
            size,
            true,
        )?);
    }

    let walked = surface_face_walkback::try_apply(part, &graph, &regions, &plans)?;

    Ok((walked, plans))
}

/// Which feature edge runs between two named points.
fn crease_between(features: &FeatureEdgeSet, from: Vector3, to: Vector3) -> usize {
    let found: Vec<usize> = (0..features.count())
        .filter(|&index| {
            let edge = &features.edges()[index];

            ((edge.start - from).length() < 1e-6 && (edge.end - to).length() < 1e-6)
                || ((edge.end - from).length() < 1e-6 && (edge.start - to).length() < 1e-6)
        })
        .collect();
    assert_eq!(found.len(), 1, "one crease from {from} to {to}");

    found[0]
}

/// The fixture's top-left edge, planned on both the regions it lands on.
fn top_edge_plan_of(part: &Arc<Mesh>, size: f64) -> Rc<SurfaceBevelPlan> {
    let (regions, features) = regions_of(part);
    let regions = Arc::new(regions);
    let chains = bevel_chain_linker::link(&[features.edges()[top_edge_of(&features)]]);
    assert_eq!(chains.len(), 1);

    SurfaceBevelPlan::try_create(&chains[0], &regions, size, true).unwrap_or_else(|reason| {
        panic!("the fixture has to plan before its room can be measured: {reason}")
    })
}

/// Which feature edge is the left slope's top one, from the knife tip to the crease corner.
fn top_edge_of(features: &FeatureEdgeSet) -> usize {
    crease_between(
        features,
        Vector3::new(0.0, 0.0, nose_in_notch_test_meshes::SIZE),
        Vector3::new(10.0, 10.0, nose_in_notch_test_meshes::SIZE),
    )
}
