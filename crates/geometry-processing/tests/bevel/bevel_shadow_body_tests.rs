//! The plan's step 25 - an obstruction that WIDENS behind the face it stands on keeps its
//! shoulders, not just the column under its footprint. Measured on the nose-in-notch part at both
//! the radii the fixtures use - 3.4, where the round only nicks the nose ("Encroaching Side.mcx"),
//! and 15, where it uncovers the whole of it ("15mm Bevel Top Edge.mcx"). Every test runs the cut
//! BOTH ways - prisms alone and prisms plus body - so what the body is worth is measured rather
//! than asserted, and held to the nose's own two exact planes rather than to a control mesh.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_cap_clip::{self, Candidate};
use geometry_processing::bevel::bevel_face_fit::DEFAULT_SURFACE_TOLERANCE_DEGREES;
use geometry_processing::bevel::bevel_group_tools::{self, BevelGroupToolsOptions};
use geometry_processing::bevel::bevel_shadow_clip::{self, Obstruction};
use geometry_processing::bevel::BevelFaceFit;
use geometry_processing::topology::bevel_edge_group_resolver;
use geometry_processing::topology::{
    BevelEdgeGroupResolution, BevelSelectionKind, FaceGroup, FeatureEdgeSet,
};
use polygon_mesh::csg::{BooleanError, CsgModes};
use polygon_mesh::Mesh;
use vector_math::{Matrix4X4, Vector3};

use crate::bevel_obstruction_test_meshes::{
    create_l_boss_near_a_top_edge, create_rib_near_a_top_edge,
};
use crate::bevel_volume_probes::{probe_cube, volume_inside};
use crate::kernel_cut_helpers::{boolean, cut, edge_index};
use crate::mesh_measurements::{shell_count, signed_volume};
use crate::nose_in_notch_test_meshes as nose;

/// How fast the nose's two faces open away from the ridge, in z per unit y.
const NOSE_SLOPE: f64 = 2.0 / 3.0;

/// The ridge's y, which the wedge opens back from.
const RIDGE_Y: f64 = nose::RIDGE_Y;

/// And its z, exactly mid part.
const RIDGE_Z: f64 = nose::RIDGE_Z;

/// How big a cube is used to ask whether a point is inside the solid.
const PROBE_SIDE: f64 = 0.05;

/// What a probe reads where the point is well inside; half of it is the threshold either way.
const FULL_PROBE: f64 = PROBE_SIDE * PROBE_SIDE * PROBE_SIDE;

/// How far a vertex may be off a plane the shape puts it exactly on.
const ON_IT: f64 = 1e-4;

/// What the cut still takes out of the nose at radius 15 with the body handed back. Measured at
/// 0.003, which is two tessellations of the one surface rather than a shoulder; the bound is left
/// loose because what this class pins here is the DIFFERENCE the body makes and not a chording.
const BURIED_RESIDUAL_AT_15: f64 = 0.45;

/// What the round at 3.4 takes out of the nose with the footprint prisms alone: the nose's exposed
/// material ends where its creases run out onto the slope at x = 15 and the prism ends with it,
/// while the round stands proud of that plane out to [`the_ridge_meets_the_round_at`]. Measured at
/// 0.085 - the "odd trim at the notch tip" the fixture's owner reports.
const HAIR_THE_PRISMS_ALONE_TAKE: f64 = 0.05;

/// And what is left of it once the body follows the nose's buried continuation out to the round.
/// Measured at 0.0003, which is a boolean's own rounding.
const HAIR_LEFT_BY_THE_CONTINUATION: f64 = 0.01;

/// The encroaching fixture's radius, on the right knife tip's vertical edge.
const ENCROACHING_RADIUS: f64 = 3.4;

/// Where the L boss's own grid line splits the selected top edge in two. An edge pick takes the
/// collinear run with it, so picking either half selects the whole edge.
const TOP_EDGE_SPLITS_AT: f64 = 10.0;

/// The round's axis in x, one radius in from the flat face at x = 20.
fn the_axis_x() -> f64 {
    nose::SIZE - ENCROACHING_RADIUS
}

/// And in y, one radius off the slope plane - which for the 45 degree tip is 8.208.
fn the_axis_y() -> f64 {
    nose::SIZE + (ENCROACHING_RADIUS * 2.0_f64.sqrt()) - the_axis_x()
}

/// Where the round's cylinder crosses the nose's ridge, which is where the control mesh's own trim
/// curve comes to its point: `AxisX - sqrt(R^2 - (AxisY - RidgeY)^2)`. 15.475 at 3.4.
fn the_ridge_meets_the_round_at() -> f64 {
    the_axis_x()
        - ((ENCROACHING_RADIUS * ENCROACHING_RADIUS)
            - ((the_axis_y() - RIDGE_Y) * (the_axis_y() - RIDGE_Y)))
            .sqrt()
}

/// A point of the nose's wedge on the FAR side of the round, between the cylinder and the tip. It
/// is buried material like the hair the continuation is for, and the cut takes it: the
/// continuation stops at the FIRST crossing of the cylinder, so it never reaches here.
fn past_the_round_on_the_tip_side() -> Vector3 {
    Vector3::new(19.3, 5.4, 10.1)
}

#[test]
fn the_nose_keeps_its_shoulders_where_the_oversize_round_uncovers_it() {
    let without_the_body = the_oversize_round(false);
    let with_the_body = the_oversize_round(true);

    assert_eq!(
        shell_count(&with_the_body),
        1,
        "handing material back may not leave the nose - or anything else - floating"
    );

    // The shoulders: nose material behind the slope plane, wider than the footprint the prism has.
    // Asserted CUT without the body first, because a probe that was kept either way would pass on a
    // fixture where this class did nothing at all.
    for shoulder in [
        Vector3::new(6.5, 7.0, 11.233),
        Vector3::new(7.1444, 8.5, 12.233),
    ] {
        assert!(
            inside(&without_the_body, shoulder) < FULL_PROBE / 2.0,
            "({:.3}, {:.3}, {:.3}) is nose material outside the column under the nose's footprint, \
             so the prism alone does not protect it - if this is already kept, the probe is in the \
             wrong place and the test below proves nothing",
            shoulder.x,
            shoulder.y,
            shoulder.z
        );
        assert!(
            inside(&with_the_body, shoulder) > FULL_PROBE / 2.0,
            "it is inside the nose's own wedge, which is a feature standing on the face the round \
             lands on and is not the round's to take"
        );
    }

    let (before, where_before) = worst_into_the_nose(&without_the_body, 3.0, 11.0);
    let (after, where_after) = worst_into_the_nose(&with_the_body, 3.0, 11.0);

    assert!(
        after < before,
        "the body is what the shoulders are kept by: the prism alone leaves {before:.3} at \
         {where_before:?}"
    );
    assert!(
        after <= BURIED_RESIDUAL_AT_15,
        "what is left is the buried 1.2 mm at the nose's own end, and it measures {after:.3} at \
         {where_after:?} - anything bigger is a shoulder still being cut, which is what this class \
         is for"
    );

    // The cut still has to HAPPEN. A region that protected the whole band would leave the part
    // whole.
    assert!(
        signed_volume(&with_the_body) < nose::VOLUME,
        "the one group is convex, so the round can only take material off"
    );
    assert!(
        signed_volume(&with_the_body) > signed_volume(&without_the_body),
        "a clip only ever hands material BACK, so the fuller region cannot cut more than the prism \
         did - a result that came out SMALLER is the signature of a protected region the boolean \
         read inside out"
    );

    // And the run-outs the oversize round is really about are where BevelOversizeRunOutTests left
    // them, which is what says the body did not spread into the cut.
    let vertices: Vec<Vector3> = with_the_body
        .vertices
        .iter()
        .map(|v| v.as_vector3())
        .collect();

    assert!(
        vertices
            .iter()
            .filter(|vertex| (vertex.z - 20.0).abs() < ON_IT)
            .all(|vertex| vertex.x + vertex.y >= 20.0 - 1e-3),
        "the round still eats the whole top face on its own side of x + y = 20"
    );
    assert!(
        vertices
            .iter()
            .any(|&vertex| (vertex - Vector3::new(0.0, 0.0, 5.0)).length() < 0.05),
        "and the tip's vertical edge is still cut down to z = 5"
    );
}

/// The other radius, and the continuation the body follows past the obstruction's own last
/// triangle. At 3.4 the round only nicks the nose, and what it took was all PAST the nose's
/// visible end at x = 15 - the buried 0.475 mm the round's own cylinder stands proud of the slope
/// over. The far side is asserted too: the nose's planes go on to the tip, and a continuation with
/// nothing to stop it would protect [`past_the_round_on_the_tip_side`] and leave a fin.
#[test]
fn the_nose_keeps_its_buried_continuation_out_to_the_round_that_uncovers_it() {
    let without_the_body = the_encroaching_round(false);
    let with_the_body = the_encroaching_round(true);

    assert_eq!(shell_count(&with_the_body), 1);

    let (before, where_before) = worst_into_the_nose(&without_the_body, 13.5, 16.5);
    let (after, where_after) = worst_into_the_nose(&with_the_body, 13.5, 16.5);

    assert!(
        before > HAIR_THE_PRISMS_ALONE_TAKE,
        "the prisms alone cut {before:.4} into the nose at {where_before:?} - if they no longer \
         do, the fault this measures is being answered somewhere else and the reading below proves \
         nothing"
    );
    assert!(
        after <= HAIR_LEFT_BY_THE_CONTINUATION,
        "the body carries the nose's own two planes past its last triangle at x = 15 out to the \
         cylinder, so nothing should be left cutting into the wedge - {after:.4} at \
         {where_after:?} is"
    );

    // And the ridge itself, which is what the fixture's owner sees: it has to arrive at the
    // cylinder rather than being sawn off where its creases died on the slope.
    let reaches = ridge_reaches(&with_the_body);

    assert!(
        (reaches - the_ridge_meets_the_round_at()).abs() <= 0.02,
        "the ridge runs to {reaches:.4} and the round crosses it at {:.4} - short of that is \
         the trim cut back into the nose, and past it is protection reaching where the round has \
         already taken the surface over",
        the_ridge_meets_the_round_at()
    );
    assert!(
        ridge_reaches(&without_the_body) < nose::right_ridge_end().x + 0.01,
        "with the prisms alone it stops at the nose's own last triangle, which is what makes the \
         reading above the continuation's doing and not the cutter's"
    );

    // The far side: buried nose material between the cylinder and the tip, which the cut takes and
    // the continuation may not reach.
    for result in [&without_the_body, &with_the_body] {
        let past = past_the_round_on_the_tip_side();
        assert!(
            inside(result, past) < FULL_PROBE / 2.0,
            "{past:?} is inside the nose's wedge on the FAR side of the round, where the cut takes \
             everything down to the cylinder - a continuation that reached here would leave a fin \
             of nose standing off the finished round"
        );
    }

    assert!(
        signed_volume(&with_the_body) > signed_volume(&without_the_body),
        "a clip only ever hands material BACK, so protecting the continuation cannot cut more than \
         the prisms did"
    );
}

/// The general case the fixture tests do not cover: a rib, whose sides are parallel, so its body
/// adds nothing the column did not already have - and takes nothing away either.
#[test]
fn a_ribs_column_is_the_same_with_the_body_as_without() {
    let part = create_rib_near_a_top_edge(1.0);
    let features = FeatureEdgeSet::create(&part);
    let edge = edge_index(
        &features,
        Vector3::new(0.0, 0.0, 20.0),
        Vector3::new(0.0, 20.0, 20.0),
        ON_IT,
    );
    let resolution = resolve(&features, edge, &part);
    let obstructions = obstructions(&part, &resolution, 5.0).1;

    let graph = part.get_mesh_edge_graph();
    let prisms = bevel_shadow_clip::build(&part, &graph, &obstructions, false, 1.0)
        .expect("the clipper takes the outline")
        .0;
    let with_body = bevel_shadow_clip::build(&part, &graph, &obstructions, true, 1.0)
        .expect("the clipper takes the outline")
        .0;

    assert_eq!(prisms.len(), 1, "one rib casts one footprint");
    assert!(
        with_body.len() >= prisms.len(),
        "the body is handed back BESIDE the prisms, never instead of them"
    );

    for i in 0..prisms.len() {
        assert_eq!(with_body[i].vertices.len(), prisms[i].vertices.len());

        for vertex in 0..prisms[i].vertices.len() {
            assert_eq!(
                (with_body[i].vertices[vertex] - prisms[i].vertices[vertex]).length(),
                0.0,
                "column {i} is the same column vertex for vertex whether the body is built or not \
                 - every part that builds today builds from these, and step 25 adds to them rather \
                 than changing them"
            );
        }
    }
}

/// The case that bounds what the body may do: an obstruction whose own footprint is NOT convex.
/// The notch of an L is material no feature stands on, and it has to be cut exactly as the
/// footprint prism alone would cut it - an intersection of half spaces is convex and would fill
/// the notch.
#[test]
fn the_notch_of_an_l_shaped_boss_is_cut_just_as_the_footprint_alone_would_cut_it() {
    let part = create_l_boss_near_a_top_edge();
    let (obstructions, cutter, selected) = the_top_edge_round(&part);

    assert_eq!(
        selected, 2,
        "the boss's grid line splits the top edge at y = 10, and an edge pick has to take the \
         collinear other half with it - a run that stopped there would never reach the notch at y \
         in [10, 20] and the measurements below would mean nothing"
    );
    assert!(
        !obstructions.is_empty(),
        "the boss stands on the top face inside the setback, so there is a clip to run at all - \
         without one this fixture would pass on a technicality"
    );

    let without_the_body = cut_with(&part, &cutter, &obstructions, false);
    let with_the_body = cut_with(&part, &cutter, &obstructions, true);

    // In the notch: x in [2, 4], y in [10, 20], under the cylinder's reach. Nothing stands here.
    for in_the_notch in [Vector3::new(3.0, 15.0, 19.8), Vector3::new(2.5, 18.0, 19.9)] {
        assert!(
            inside(&without_the_body, in_the_notch) < FULL_PROBE / 2.0,
            "{in_the_notch:?} is in the notch of the L, which the footprint prism does not protect"
        );
        assert!(
            inside(&with_the_body, in_the_notch) < FULL_PROBE / 2.0,
            "and the body may not protect it either: an intersection of the boss's face half spaces \
             is convex and would fill the notch, which would leave the fillet uncut in a place no \
             feature stands"
        );
    }

    // And the boss's own column is still kept, so the stand-down is not a silent loss of the clip.
    assert!(
        inside(&with_the_body, Vector3::new(1.5, 15.0, 19.4)) > FULL_PROBE / 2.0,
        "the L's long arm stands on this column at every depth, and it is the prism's to keep"
    );

    let (with, without) = (
        signed_volume(&with_the_body),
        signed_volume(&without_the_body),
    );
    assert!(
        (with - without).abs() <= 1e-6,
        "a non-convex obstruction is left to its footprint, so this fixture measures the same solid \
         with the body as without it - {with} against {without}"
    );
}

/// How far along x the nose's RIDGE survives: the furthest vertex sitting on the ridge line
/// (y = RIDGE_Y, z = RIDGE_Z), which is where the trim curve comes to its point on both the
/// control mesh and a rebuild that kept the nose.
fn ridge_reaches(mesh: &Mesh) -> f64 {
    let mut furthest = 0.0_f64;

    for vertex in mesh.vertices.iter().map(|v| v.as_vector3()) {
        if (vertex.y - RIDGE_Y).abs() < 0.02 && (vertex.z - RIDGE_Z).abs() < 0.02 {
            furthest = furthest.max(vertex.x);
        }
    }

    furthest
}

/// How far the worst of a result's vertices reaches INTO the nose's own wedge, over a span of x:
/// the distance from the nose's two exact planes of any vertex that is inside the wedge, which for
/// a surface that was trimmed against the nose is zero and for one the round cut through is how
/// deep it cut. Vertices outside the wedge are not the nose's and are not asked about.
fn worst_into_the_nose(mesh: &Mesh, from_x: f64, to_x: f64) -> (f64, Vector3) {
    let upper = nose::upper_nose_normal();
    let lower = nose::lower_nose_normal();
    let ridge = Vector3::new(0.0, RIDGE_Y, RIDGE_Z);
    let mut worst = 0.0;
    let mut at = Vector3::ZERO;

    for vertex in mesh.vertices.iter().map(|v| v.as_vector3()) {
        if vertex.x < from_x
            || vertex.x > to_x
            || vertex.y < RIDGE_Y - 0.01
            || (vertex.z - RIDGE_Z).abs() > (NOSE_SLOPE * (vertex.y - RIDGE_Y)) + 0.05
        {
            continue;
        }

        let distance = (upper.dot(vertex) - upper.dot(ridge))
            .abs()
            .min((lower.dot(vertex) - lower.dot(ridge)).abs());

        if distance > worst {
            worst = distance;
            at = vertex;
        }
    }

    (worst, at)
}

fn resolve(features: &FeatureEdgeSet, edge: i32, part: &Mesh) -> BevelEdgeGroupResolution {
    let group = bevel_edge_group_resolver::create_from_edge(
        features,
        edge,
        BevelSelectionKind::Edge,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("an edge of the set");

    bevel_edge_group_resolver::resolve(&group, part)
}

/// `BevelFaceFit.Create(...).TryCheckGroup(0, ...)`: whether a wall the run ends on trimmed it, and
/// what stands on the faces it lands on.
fn obstructions(
    part: &Mesh,
    resolution: &BevelEdgeGroupResolution,
    radius: f64,
) -> (bool, Vec<Obstruction>) {
    let check = BevelFaceFit::create(
        Arc::new(part.clone()),
        Rc::new(vec![resolution.clone()]),
        true,
        radius,
        DEFAULT_SURFACE_TOLERANCE_DEGREES,
        None,
    )
    .try_check_group_fully(0);

    (check.trimmed_by_a_wall_it_ends_on, check.standing_on_a_face)
}

/// The 5 mm round on the top edge at x = 0 of a 20 mm cube - `bevel_shadow_clip_tests`' own
/// selection - and what `BevelFaceFit` found standing in its way.
fn the_top_edge_round(part: &Mesh) -> (Vec<Obstruction>, Mesh, usize) {
    const RADIUS: f64 = 5.0;

    let features = FeatureEdgeSet::create(part);
    let edge = edge_index(
        &features,
        Vector3::new(0.0, 0.0, 20.0),
        Vector3::new(0.0, TOP_EDGE_SPLITS_AT, 20.0),
        ON_IT,
    );
    let resolution = resolve(&features, edge, part);
    let obstructions = obstructions(part, &resolution, RADIUS).1;
    let tools = bevel_group_tools::try_build(
        &resolution,
        true,
        RADIUS,
        8,
        BevelGroupToolsOptions::default(),
    )
    .unwrap_or_else(|refusal| panic!("{:?}", refusal.reason))
    .tools;

    (obstructions, tools[0].mesh.clone(), resolution.edges.len())
}

/// "15mm Bevel Top Edge.mcx"'s cut, by the rebuild's own steps (fit, tools, the cap clip, the
/// shadow, the subtract): the left slope's top edge at radius 15, whose cap the sweep cannot carry,
/// cut down far enough to uncover the buried nose. `bevel_oversize_run_out_tests::round` is this
/// with the bodies, so it reads it from here. The clip goes through `bevel_cap_clip::apply` rather
/// than being re-done, and a withdrawal fails loudly.
pub fn the_oversize_round(include_bodies: bool) -> Mesh {
    const RADIUS: f64 = 15.0;

    let part = nose::create();
    let features = FeatureEdgeSet::create(&part);
    let edge = edge_index(
        &features,
        Vector3::new(0.0, 0.0, 20.0),
        Vector3::new(10.0, 10.0, 20.0),
        ON_IT,
    );
    let resolution = resolve(&features, edge, &part);
    let (trimmed_by_a_wall, obstructions) = obstructions(&part, &resolution, RADIUS);

    let tools = bevel_group_tools::try_build(
        &resolution,
        true,
        RADIUS,
        16,
        BevelGroupToolsOptions {
            may_clip_caps: trimmed_by_a_wall,
            ..BevelGroupToolsOptions::default()
        },
    )
    .unwrap_or_else(|refusal| panic!("{:?}", refusal.reason))
    .tools;

    let mut cutters = vec![tools[0].mesh.clone()];

    if let Some(cap_clips) = &tools[0].cap_clips {
        let combine =
            |operands: &[(Mesh, Matrix4X4)], mode: CsgModes| -> Result<Mesh, BooleanError> {
                let meshes: Vec<&Mesh> = operands.iter().map(|(mesh, _)| mesh).collect();
                Ok(boolean(&meshes, mode))
            };
        let withdrawals = bevel_cap_clip::apply(
            &mut cutters,
            &[Candidate::new(0, 0, RADIUS, cap_clips.clone())],
            &combine,
        )
        .expect("not cancelled");

        assert!(
            withdrawals.is_empty(),
            "the cap clip withdrew the cutter, so there is no round to measure: {:?}",
            withdrawals
                .first()
                .map(|withdrawal| withdrawal.reason.code())
        );
    }

    cut_with(&part, &cutters[0], &obstructions, include_bodies)
}

/// "Encroaching Side.mcx"'s cut: the right knife tip's vertical edge at radius 3.4.
fn the_encroaching_round(include_bodies: bool) -> Mesh {
    let part = nose::create();
    let features = FeatureEdgeSet::create(&part);
    let edge = edge_index(
        &features,
        Vector3::new(20.0, 0.0, 0.0),
        Vector3::new(20.0, 0.0, 20.0),
        ON_IT,
    );
    let resolution = resolve(&features, edge, &part);
    let obstructions = obstructions(&part, &resolution, ENCROACHING_RADIUS).1;
    let tools = bevel_group_tools::try_build(
        &resolution,
        true,
        ENCROACHING_RADIUS,
        8,
        BevelGroupToolsOptions::default(),
    )
    .unwrap_or_else(|refusal| panic!("{:?}", refusal.reason))
    .tools;

    cut_with(&part, &tools[0].mesh, &obstructions, include_bodies)
}

/// The part with the shadow-clipped cutter taken out of it, which is what a rebuild publishes.
fn cut_with(
    part: &Mesh,
    cutter: &Mesh,
    obstructions: &[Obstruction],
    include_bodies: bool,
) -> Mesh {
    let shadows = bevel_shadow_clip::build(
        part,
        &part.get_mesh_edge_graph(),
        obstructions,
        include_bodies,
        1.0,
    )
    .expect("the clipper takes the outline")
    .0;

    cut(part, cutter, &shadows)
}

fn inside(solid: &Mesh, at: Vector3) -> f64 {
    volume_inside(solid, &probe_cube(at, PROBE_SIDE))
}
