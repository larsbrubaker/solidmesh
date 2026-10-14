//! The junction at the RIGHT ridge end of `nose_in_notch_test_meshes` - one cut and TWO creases
//! (`SurfaceJunctionKind::TwoCreases`, `surface_mixed_junction`) - measured against the Onshape
//! control of "Center Right and Left Side" at r = 2. ONE canal, whose ball is the beads' void ball,
//! swept from the upper bead's square end ring round to the lower bead's; each arm is an ordinary
//! mixed corner of the ridge against one crease, and the canal is that corner's dual
//! (`SurfaceMixedCornerDualFrame`).
//!
//! Also holds the class's shared helpers ([`the_junction`], [`the_plans_of_the_right_ridge_end`],
//! [`the_vertex_at`], [`RADIUS`], [`the_corner`]), which `surface_two_crease_junction_rings_tests`
//! reads.

#![allow(dead_code)]

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    SurfaceBevelPlan, SurfaceEdgePlan, SurfaceJunctionKind, SurfaceMixedCorner,
    SurfaceMixedCornerDualFrame, SurfaceMixedJunction, SurfaceVertexPool,
};
use geometry_processing::bevel::{BevelAdvisoryArg, BevelAdvisoryCode};
use geometry_processing::topology::{EdgeConvexity, FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::Mesh;
use vector_math::{math_helper, Vector3};

use crate::nose_in_notch_test_meshes;
use crate::surface_test_chains;
use crate::surface_walkback_checks::regions_of;

/// The control's own fillet radius.
pub const RADIUS: f64 = 2.0;

/// How near two numbers that should be one have to stand - the canal is read, not solved.
const EXACT: f64 = 1e-9;

/// The corner: where the ridge runs out onto the right slope.
pub fn the_corner() -> Vector3 {
    nose_in_notch_test_meshes::right_ridge_end()
}

/// The corner READS: three chain ends at one vertex, TWO creases among them, and the one cut both
/// arms are drawn about.
#[test]
fn the_right_ridge_end_reads_as_a_junction_of_two_creases() {
    let (junction, _, _) = the_junction();

    assert_eq!(junction.kind, SurfaceJunctionKind::TwoCreases);
    assert!((junction.corner - the_corner()).length() < EXACT);

    // No cap: the cut's two faces are each SHARED with a crease, so each arm's cap is the other
    // arm's shared face and no region is the corner's own.
    assert_eq!(junction.cap_region_key, -1);

    // Both arms are the SAME cut - the ridge, its chain's one edge, arriving at the corner with
    // nothing swallowed - read against a different crease each.
    for arm in [&junction.first, &junction.second] {
        assert_eq!(arm.plan, 0);
        assert_eq!(arm.cut_index, 0);
        assert!(!arm.corner_at_the_start);
        assert_eq!(arm.swallowed.len(), 0);
        assert!(!arm.mixed.hands_over_at_a_mitre);
        assert_eq!(arm.mixed.bead.edge.edge.convexity, EdgeConvexity::Concave);
        assert!(arm.dual.is_some());
    }

    // A bead per arm, each its own chain's one edge - the upper crease arrives at the corner and
    // the lower one leaves it, which is what the walkback has to know to bound the right end.
    assert_eq!(junction.first.bead_end.plan, 1);
    assert!(!junction.first.bead_end.at_the_start);
    assert_eq!(junction.second.bead_end.plan, 2);
    assert!(junction.second.bead_end.at_the_start);

    // Two different shared faces - the nose's two - and ONE wall, the slope they both stand
    // against.
    assert_ne!(
        junction.first.mixed.shared_region_key,
        junction.second.mixed.shared_region_key
    );
    assert!(
        (junction.first.mixed.out_of_the_wall - junction.second.mixed.out_of_the_wall).length()
            < EXACT
    );
    assert!(
        (junction.first.mixed.out_of_the_wall - nose_in_notch_test_meshes::right_slope_normal())
            .length()
            < EXACT
    );
}

/// The canal runs from the UPPER bead's square end ring round to the LOWER bead's: each arm's first
/// ball is centred ON its own bead's axis, at a station that bead still has, and the ball one arm
/// hands over at is the ball the other starts from.
#[test]
fn the_canal_runs_from_one_beads_square_ring_to_the_others() {
    let (junction, _, _) = the_junction();
    let upper = &junction.first;
    let lower = &junction.second;
    let upper_dual = upper.dual.as_ref().expect("a dual");
    let lower_dual = lower.dual.as_ref().expect("a dual");

    let turn = math_helper::radians_to_degrees(upper_dual.turn_radians);
    assert!(
        (turn - 112.6199).abs() <= 1e-4,
        "the canal turns the CUT strip's own sweep - instead {turn:.4}"
    );
    assert!(
        (upper_dual.turn_radians - lower_dual.turn_radians).abs() < EXACT,
        "and both arms turn the same, because it is one canal"
    );

    let start = upper_dual.spine.centre_at(0.0);
    assert!(
        (start - Vector3::new(10.7848, 6.3868, 13.3282)).length() < 1e-4,
        "turn zero is the upper bead's own ball - instead {start}"
    );
    let end = upper_dual.spine.centre_at(upper_dual.turn_radians);
    assert!(
        (end - Vector3::new(10.7848, 6.3868, 6.6718)).length() < 1e-4,
        "and the end of the turn is the lower bead's - instead {end}"
    );

    // The two arms read ONE canal, at both of its ends.
    assert!((end - lower_dual.spine.centre_at(0.0)).length() < EXACT);
    assert!((lower_dual.spine.centre_at(lower_dual.turn_radians) - start).length() < EXACT);

    // And each end really is its bead's SQUARE ring: the centre stands on the bead's own axis, at
    // 5.0022 of the crease's 7.8174. That is what leaves the bead's band ending square.
    for arm in [upper, lower] {
        let centre = arm.dual.as_ref().expect("a dual").spine.centre_at(0.0);
        let axis = arm.mixed.axis();

        assert!(
            (centre - axis.closest_point_to(centre)).length() < EXACT,
            "the first ball stands on the bead's own axis - instead {centre}"
        );

        let station = (centre - axis.closest_point_to(the_corner())).dot(
            SurfaceMixedCorner::away_from_the_corner(&arm.mixed.bead, the_corner()),
        );

        assert!(
            (station - 5.0022).abs() <= 1e-4,
            "at the crease station the control measured - instead {station:.4}"
        );
        assert!(((arm.mixed.bead.to() - arm.mixed.bead.from()).length() - 7.8174).abs() <= 1e-4);
    }
}

/// The canal's spine is the ELLIPSE the plane a radius outside the wall cuts the cut's own `2r`
/// cylinder in: centred where that plane meets the cut line, with semi-axes `2r` square to the cut
/// and `2r` over the cosine of the wall's lean along it.
#[test]
fn the_spine_is_the_ellipse_the_wall_cuts_the_cuts_cylinder_in() {
    let (junction, _, _) = the_junction();
    let spine = &junction.first.dual.as_ref().expect("a dual").spine;
    let pi = std::f64::consts::PI;

    // Read off the curve by difference rather than from the frame it was built out of, so that this
    // measures the spine rather than restating its construction.
    let centre = (spine.centre_at(0.0) + spine.centre_at(pi)) / 2.0;
    let cosine_arm = (spine.centre_at(0.0) - spine.centre_at(pi)) / 2.0;
    let sine_arm = (spine.centre_at(pi / 2.0) - spine.centre_at(3.0 * pi / 2.0)) / 2.0;

    assert!(
        (centre - Vector3::new(8.5660, 8.6056, 10.0)).length() < 1e-4,
        "the ellipse is centred where the offset wall meets the cut line - instead {centre}"
    );

    let mut longest = 0.0f64;
    let mut shortest = f64::MAX;

    for step in 0..=3600 {
        let at = step as f64 * pi / 1800.0;
        let arm = ((cosine_arm * at.cos()) + (sine_arm * at.sin())).length();

        longest = longest.max(arm);
        shortest = shortest.min(arm);
    }

    assert!(
        (shortest - 2.0 * RADIUS).abs() <= 1e-4,
        "square to the cut the ellipse is the cylinder's own 2r - instead {shortest:.4}"
    );
    assert!(
        (longest - 5.6569).abs() <= 1e-4,
        "and along it, 2r over the 45 degree cosine of the wall's lean - instead {longest:.4}"
    );
}

/// The CUT's end ring is planar and LEANS 63.43 degrees across the cut - not the square section a
/// one-crease junction's cuts stop on - and reaches out to x = 12.5660 at mid turn.
#[test]
fn the_cuts_end_ring_is_planar_and_leans_across_the_cut() {
    let (junction, _, _) = the_junction();
    let dual = junction.first.dual.as_ref().expect("a dual");
    let plane = dual.cut_ring_plane();

    assert!(
        (plane.inward + Vector3::new(1.0, 2.0, 0.0).get_normal()).length() < EXACT,
        "the ring plane's normal is (1, 2, 0) over root five, facing back down the cut - instead {}",
        plane.inward
    );
    assert!(
        (plane.point - Vector3::new(10.7848, 7.4962, 11.6641)).length() < 1e-4,
        "through the point the two cylinders touch at on the upper nose face - instead {}",
        plane.point
    );

    let lean = math_helper::radians_to_degrees(
        plane
            .inward
            .dot(junction.first.mixed.cut_center_line().direction)
            .abs()
            .acos(),
    );

    assert!(
        (lean - 63.4349).abs() <= 1e-4,
        "which leans that far off the cut's square section - instead {lean:.4}"
    );

    // Every station's contact on the cut's tube lies in it, which is what lets the strip be cut at
    // ONE plane rather than station by station.
    let mut worst = 0.0f64;

    for step in 0..=64 {
        let at = step as f64 * dual.turn_radians / 64.0;

        worst = worst.max(
            (dual.cut_contact_at(at) - plane.point)
                .dot(plane.inward)
                .abs(),
        );
    }

    assert!(
        worst < EXACT,
        "the whole contact curve is planar - instead one station stands {worst:e} out of it"
    );

    let crossing = the_contact_where_it_crosses(
        dual,
        |turn| dual.cut_contact_at(turn),
        nose_in_notch_test_meshes::RIDGE_Z,
    );
    assert!(
        (crossing - Vector3::new(12.5660, 6.6056, 10.0)).length() < 1e-4,
        "and at mid turn the ring reaches x = 12.5660, which is the canal's own tube contact rail"
    );
}

/// The ball's rail on the WALL crosses BOTH creases - the upper at 45.8836 degrees and the lower at
/// 66.7363 - and runs across the nose's ROOT between them, which is the stretch of wall the control
/// has no vertex of the ridge tip on at all.
#[test]
fn the_wall_rail_crosses_both_creases_and_runs_across_the_noses_root() {
    let (junction, _, _) = the_junction();
    let dual = junction.first.dual.as_ref().expect("a dual");

    let at_the_upper = the_turn_where_the_wall_rail_crosses(dual, &junction.first.mixed.bead);
    let at_the_lower = the_turn_where_the_wall_rail_crosses(dual, &junction.second.mixed.bead);

    // Quoted against the CONTROL's own model at a thousandth of a degree, not against this reading:
    // the part's vertices are stored as floats, and the frame each arm is read in comes off them,
    // so the crossings land 0.0002 degrees from the exactly computed ones.
    let upper_degrees = math_helper::radians_to_degrees(at_the_upper);
    assert!(
        (upper_degrees - 45.8836).abs() <= 1e-3,
        "instead the upper crossing is at {upper_degrees:.4}"
    );
    let lower_degrees = math_helper::radians_to_degrees(at_the_lower);
    assert!(
        (lower_degrees - 66.7363).abs() <= 1e-3,
        "instead the lower crossing is at {lower_degrees:.4}"
    );

    assert!(
        (dual.wall_rail_at(at_the_upper) - Vector3::new(13.9142, 6.0858, 10.7239)).length() < 1e-4,
        "instead it crosses the upper crease at {}",
        dual.wall_rail_at(at_the_upper)
    );
    assert!(
        (dual.wall_rail_at(at_the_lower) - Vector3::new(13.9142, 6.0858, 9.2761)).length() < 1e-4,
        "instead it crosses the lower crease at {}",
        dual.wall_rail_at(at_the_lower)
    );

    let crossing = the_contact_where_it_crosses(
        dual,
        |turn| dual.wall_rail_at(turn),
        nose_in_notch_test_meshes::RIDGE_Z,
    );
    assert!(
        (crossing - Vector3::new(13.9802, 6.0198, 10.0)).length() < 1e-4,
        "and between them it stands 1.02 short of the tip, out on the nose's own root"
    );
}

/// The rows the patch will be laid on NARROW from each bead's own 66.91 degree arc to 45 degrees at
/// mid turn and open again - a V, not a shrink, because BOTH ends of this canal are bead rings.
#[test]
fn the_sweep_narrows_to_forty_five_at_mid_turn_and_opens_again() {
    let (junction, _, _) = the_junction();
    let dual = junction.first.dual.as_ref().expect("a dual");

    assert!(
        (swept_at(dual, 0.0) - 66.9065).abs() <= 1e-4,
        "instead the sweep at turn zero is {:.4}",
        swept_at(dual, 0.0)
    );
    assert!(
        (swept_at(dual, dual.turn_radians) - 66.9065).abs() <= 1e-4,
        "instead the sweep at the end of the turn is {:.4}",
        swept_at(dual, dual.turn_radians)
    );

    let mut narrowest = 180.0;
    let mut narrowest_at = 0.0;

    for step in 0..=3600 {
        let at = step as f64 * dual.turn_radians / 3600.0;

        if swept_at(dual, at) < narrowest {
            narrowest = swept_at(dual, at);
            narrowest_at = at;
        }
    }

    assert!(
        (narrowest - 45.000f64).abs() <= 1e-3,
        "instead the narrowest row sweeps {narrowest:.4}"
    );
    let narrowest_degrees = math_helper::radians_to_degrees(narrowest_at);
    assert!(
        (narrowest_degrees - 56.3099).abs() <= 1e-2,
        "where the rail crosses the ridge's own plane - instead at {narrowest_degrees:.4}"
    );
}

/// The whole canal is MIRROR symmetric in the ridge's own plane, which is what says the two arms
/// read one surface rather than two that nearly meet in the middle.
#[test]
fn the_canal_is_mirror_symmetric_in_the_ridges_own_plane() {
    let (junction, _, _) = the_junction();
    let dual = junction.first.dual.as_ref().expect("a dual");

    let mut worst = 0.0f64;

    for step in 0..=256 {
        let at = step as f64 * dual.turn_radians / 256.0;
        let back = dual.turn_radians - at;

        worst =
            worst.max((mirrored(dual.spine.centre_at(at)) - dual.spine.centre_at(back)).length());
        worst = worst.max((mirrored(dual.wall_rail_at(at)) - dual.wall_rail_at(back)).length());
        worst = worst.max((mirrored(dual.cut_contact_at(at)) - dual.cut_contact_at(back)).length());
    }

    // Not to the bit: the mesh stores its vertices as floats, so the two nose faces' own normals -
    // which is where each arm's frame comes from - are mirrors of each other only to about 3e-7.
    assert!(
        worst < 1e-6,
        "instead one station stands {worst:e} off its own mirror"
    );
}

/// A canal larger than the creases it ends on is refused BY NAME rather than built out past them:
/// at r = 3.2 the upper bead's end ring would stand at station 8.00 of a 7.82 mm crease.
///
/// The station climbs as 2.5011 r here, so the size that walks it off the end is 3.126 - and
/// r = 3.1 still reads, which is what says the refusal is the geometry's and not a guard set too
/// tight.
#[test]
fn a_canal_longer_than_its_creases_is_refused_by_name() {
    let mesh = Arc::new(nose_in_notch_test_meshes::create());
    let (regions, features) = regions_of(&mesh);
    let regions = Arc::new(regions);
    let near_enough = SurfaceVertexPool::near_enough_for(&mesh.vertices);

    assert!(
        SurfaceMixedJunction::try_read_all(
            &the_plans_of_the_right_ridge_end(&features, &regions, 3.1, None),
            near_enough
        )
        .is_ok(),
        "3.1 still ends on the crease"
    );

    let refused = SurfaceMixedJunction::try_read_all(
        &the_plans_of_the_right_ridge_end(&features, &regions, 3.2, None),
        near_enough,
    )
    .expect_err("3.2 runs off the crease");

    assert_eq!(
        refused.code(),
        BevelAdvisoryCode::SurfaceJunctionCanalRunsOffTheBead,
        "instead {refused}"
    );
    assert_named_at_the_corner(&refused.args()[0]);
}

/// THREE creases at one vertex is not this corner either - there is no cut for the ball to be
/// pressed against - and it keeps the name two creases were refused under before either was read.
#[test]
fn three_creases_at_one_vertex_keep_the_older_name() {
    let mesh = Arc::new(nose_in_notch_test_meshes::create());
    let (regions, features) = regions_of(&mesh);
    let regions = Arc::new(regions);
    let corner = nose_in_notch_test_meshes::upper_crease_end();

    let plans = the_plans_of(
        &regions,
        1.0,
        &[
            the_edge_from(
                &features,
                nose_in_notch_test_meshes::left_ridge_end(),
                corner,
            ),
            the_edge_from(
                &features,
                nose_in_notch_test_meshes::right_ridge_end(),
                corner,
            ),
            the_edge_from(
                &features,
                corner,
                nose_in_notch_test_meshes::notch_ceiling(),
            ),
        ],
    );

    let vertex = the_vertex_at(&plans, corner);

    let refused = SurfaceMixedJunction::try_read(
        &plans,
        vertex,
        corner,
        SurfaceVertexPool::near_enough_for(&mesh.vertices),
    )
    .expect_err("three creases are no junction of this kind");

    assert_eq!(
        refused.code(),
        BevelAdvisoryCode::SurfaceJunctionOfTwoCreases,
        "instead {refused}"
    );
}

/// The refusal's first argument is the corner itself.
pub fn assert_named_at_the_corner(arg: &BevelAdvisoryArg) {
    match arg {
        BevelAdvisoryArg::Vector3(at) => assert!(
            (*at - the_corner()).length() < 1e-6,
            "named at the corner - instead {at}"
        ),
        other => panic!("the first argument is not a point: {other:?}"),
    }
}

/// The junction at the right ridge end, read from a pass over the ridge and its two creases: the
/// junction, the mesh and the plans.
pub fn the_junction() -> (SurfaceMixedJunction, Arc<Mesh>, Vec<Rc<SurfaceBevelPlan>>) {
    let mesh = Arc::new(nose_in_notch_test_meshes::create());
    let (regions, features) = regions_of(&mesh);
    let regions = Arc::new(regions);

    let plans = the_plans_of_the_right_ridge_end(&features, &regions, RADIUS, None);

    let junctions = SurfaceMixedJunction::try_read_all(
        &plans,
        SurfaceVertexPool::near_enough_for(&mesh.vertices),
    )
    .unwrap_or_else(|why| panic!("the junction would not read: {why}"));
    assert_eq!(junctions.len(), 1);

    let junction = junctions.into_iter().next().expect("one junction");

    (junction, mesh, plans)
}

/// The pass the control was filleted from: the ridge and the two RIGHT creases, joined and opened
/// exactly as the builder does. `the_upper_crease_chorded_at` is where the upper crease was
/// chorded, when the mesh was made that way
/// (`nose_in_notch_test_meshes::create_with_upper_right_crease_split_at`): the crease is then TWO
/// edges and both have to be picked, or the chain the pass reads is only the half beside the
/// corner.
pub fn the_plans_of_the_right_ridge_end(
    features: &FeatureEdgeSet,
    regions: &Arc<FaceRegionCache>,
    radius: f64,
    the_upper_crease_chorded_at: Option<Vector3>,
) -> Vec<Rc<SurfaceBevelPlan>> {
    let mut edges = vec![the_edge_from(
        features,
        nose_in_notch_test_meshes::left_ridge_end(),
        nose_in_notch_test_meshes::right_ridge_end(),
    )];

    if let Some(chord) = the_upper_crease_chorded_at {
        edges.push(the_edge_from(
            features,
            nose_in_notch_test_meshes::right_ridge_end(),
            chord,
        ));
        edges.push(the_edge_from(
            features,
            chord,
            nose_in_notch_test_meshes::upper_crease_end(),
        ));
    } else {
        edges.push(the_edge_from(
            features,
            nose_in_notch_test_meshes::right_ridge_end(),
            nose_in_notch_test_meshes::upper_crease_end(),
        ));
    }

    edges.push(the_edge_from(
        features,
        nose_in_notch_test_meshes::right_ridge_end(),
        nose_in_notch_test_meshes::lower_crease_end(),
    ));

    the_plans_of(regions, radius, &edges)
}

/// The mesh's own index for the vertex standing at `corner`, off the plans themselves.
pub fn the_vertex_at(plans: &[Rc<SurfaceBevelPlan>], corner: Vector3) -> i32 {
    for plan in plans {
        for edge in &plan.edges {
            if (edge.from() - corner).length() < 1e-6 {
                return edge.edge.from_vertex();
            }

            if (edge.to() - corner).length() < 1e-6 {
                return edge.edge.to_vertex();
            }
        }
    }

    panic!("no planned edge stands at {corner}");
}

pub fn the_plans_of(
    regions: &Arc<FaceRegionCache>,
    radius: f64,
    edges: &[FeatureEdge],
) -> Vec<Rc<SurfaceBevelPlan>> {
    let linked: Vec<_> = edges
        .iter()
        .map(|edge| {
            let chains = bevel_chain_linker::link(std::slice::from_ref(edge));
            assert_eq!(chains.len(), 1);
            chains.into_iter().next().expect("one chain")
        })
        .collect();
    let chains = surface_test_chains::join_and_open(&linked);

    chains
        .iter()
        .map(|chain| {
            SurfaceBevelPlan::try_create(chain, regions, radius, true)
                .unwrap_or_else(|why| panic!("the plan refused the chain: {why}"))
        })
        .collect()
}

pub fn the_edge_from(features: &FeatureEdgeSet, from: Vector3, to: Vector3) -> FeatureEdge {
    let found: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| {
            ((edge.start - from).length() < 1e-6 && (edge.end - to).length() < 1e-6)
                || ((edge.end - from).length() < 1e-6 && (edge.start - to).length() < 1e-6)
        })
        .copied()
        .collect();
    assert_eq!(found.len(), 1, "one edge from {from} to {to}");

    found[0]
}

/// How far the ring at `turn` sweeps, from the wall's contact to the cut's.
fn swept_at(dual: &SurfaceMixedCornerDualFrame, turn: f64) -> f64 {
    let centre = dual.spine.centre_at(turn);
    let wall = (dual.wall_rail_at(turn) - centre).get_normal();
    let cut = (dual.cut_contact_at(turn) - centre).get_normal();

    math_helper::radians_to_degrees(wall.cross(cut).length().atan2(wall.dot(cut)))
}

/// The turn at which the ball's wall rail crosses `crease`'s own line.
fn the_turn_where_the_wall_rail_crosses(
    dual: &SurfaceMixedCornerDualFrame,
    crease: &SurfaceEdgePlan,
) -> f64 {
    let along = SurfaceMixedCorner::away_from_the_corner(crease, the_corner());
    let across = dual.out_of_the_wall.cross(along).get_normal();

    the_turn_where(dual, |turn| {
        (dual.wall_rail_at(turn) - the_corner()).dot(across)
    })
}

/// Where a rail of the canal crosses the plane z = `height`.
fn the_contact_where_it_crosses(
    dual: &SurfaceMixedCornerDualFrame,
    rail: impl Fn(f64) -> Vector3,
    height: f64,
) -> Vector3 {
    rail(the_turn_where(dual, |turn| rail(turn).z - height))
}

/// The one turn of the canal at which `of` changes sign, by halving.
fn the_turn_where(dual: &SurfaceMixedCornerDualFrame, of: impl Fn(f64) -> f64) -> f64 {
    let mut low = 0.0;
    let mut high = dual.turn_radians;

    assert!(
        of(low) * of(high) <= 0.0,
        "the canal does not cross it at all"
    );

    for _ in 0..200 {
        let middle = (low + high) / 2.0;

        if of(middle) * of(low) > 0.0 {
            low = middle;
        } else {
            high = middle;
        }
    }

    (low + high) / 2.0
}

/// A point reflected in the ridge's own plane, z = 10.
fn mirrored(point: Vector3) -> Vector3 {
    Vector3::new(
        point.x,
        point.y,
        (2.0 * nose_in_notch_test_meshes::RIDGE_Z) - point.z,
    )
}
