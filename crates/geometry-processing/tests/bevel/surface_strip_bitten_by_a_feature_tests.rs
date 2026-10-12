//! Port of `SurfaceStripBittenByAFeatureTests.cs`: the READING of the bite a nose standing part way
//! along a band takes out of the middle of its strip ("Encroaching Side.mcx" at its own 3.4) -
//! `SurfaceStripBittenByAFeature::try_read` over the walkback (`surface_face_walkback`) - against
//! the closed form of the wedge, z = 10 +/- 2(y - 5)/3, and the refusal of a side with no seam
//! (`SurfaceStripBiteRows::try_the_seams`).
//!
//! The pick is the chevron's right knife tip, the vertical edge at (20, 0) of
//! `nose_in_notch_test_meshes`; the arc dips below the ridge at its middle and rises back above
//! it at both ends, so the wedge passes through two runs of rails and only the one at the
//! feature's own seam is the bite.
//!
//! Waiting on `SurfaceTestBuild.TryBevel` (slice S28): `TheTipBandBuildsWithTheBiteInIt`,
//! `TheLeftTipBiteIsTheRightsMirror`, `TheBallCornerAndTheRunOutBuildWithTheLeftBite`,
//! `TheTwoTipBandsEachBiteTheNoseAndItsFacesRunOutBothWays`, `TheMitreBuildsWithTheRightBite` and
//! `AllFiveGroupsOfTheCenterFeatureBuildTogether`.

use std::collections::HashSet;
use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::surface::surface_mitre_joint_step::SideSeams;
use geometry_processing::bevel::surface::{
    surface_face_walkback, SurfaceEdgePlan, SurfaceStandingFeatureWithheld,
    SurfaceStandingFeatureWithheldFace, SurfaceStrip, SurfaceStripBiteRows,
    SurfaceStripBittenByAFeature, SurfaceStripStation, SurfaceStripTrimmedByTheFaceBeyond,
    SurfaceVertexPool,
};
use geometry_processing::bevel::BevelAdvisoryCode;
use geometry_processing::topology::FeatureEdgeSet;
use polygon_mesh::Mesh;
use vector_math::Vector3;

use crate::nose_in_notch_test_meshes::{self as nose, RIDGE_Y, RIDGE_Z, SIZE};
use crate::surface_standing_feature_stretch_tests::{plan_of, top_edge_of};
use crate::surface_walkback_checks::regions_of;

/// The fixture's own radius: reach 3.4/tan 22.5 = 8.208, against the nose foot's 7.071.
const RADIUS: f64 = 3.4;

const SEGMENTS: i32 = 8;

/// A place named on the mesh is three floats, so an exact vertex arrives about 1e-7 off it.
const AS_NEAR_AS_A_FLOAT_HOLDS: f64 = 1e-6;

/// The knife tip's vertical edge, floor end - the corner (20, 0) of the cube.
fn the_tip_edge_foot() -> Vector3 {
    Vector3::new(SIZE, 0.0, 0.0)
}

/// The same edge's ceiling end.
fn the_tip_edge_top() -> Vector3 {
    Vector3::new(SIZE, 0.0, SIZE)
}

/// One band, taken to the stage the bite is read at: the plan, the walkback that held the nose
/// aside, the pool over the walked mesh, the strip's rails and the tangent runs on the feature's
/// own side.
struct Band {
    pool: SurfaceVertexPool,
    strip: SurfaceStrip,
    edge: SurfaceEdgePlan,
    feature: Rc<SurfaceStandingFeatureWithheld>,
    runs: Vec<Vec<i32>>,
}

/// THE CLOSED FORM. Every rail the bite covers is entered and left at z = 10 +/- 2(y - 5)/3, where
/// y is the rail's own distance back from the ridge's line - the wedge is that region and nothing
/// else, so this is the whole of what a correct reading can say about the pairs.
#[test]
fn every_covered_rail_is_entered_and_left_where_the_wedge_says_it_is() {
    let mut band = the_tip_band();

    let _ = the_bite(&mut band);

    let bite = the_bite(&mut band);

    for step in 0..bite.rail_of.len() {
        let rail = bite.rail_of[step];
        let y = band.strip.point_on(rail, 0.0).y;
        let half = 2.0 * (y - RIDGE_Y) / 3.0;

        assert!(
            y > RIDGE_Y,
            "a rail the wedge passes through stands behind the ridge - rail {rail} is at y {y}"
        );

        // The two crossings as POINTS, which is the only way to say this without depending on
        // which way round the edge happens to run: the pair brackets the ridge's own height,
        // symmetrically.
        let first = band.strip.point_on(rail, bite.near_param_of[step]);
        let second = band.strip.point_on(rail, bite.far_param_of[step]);

        let lower = first.z.min(second.z);
        let upper = first.z.max(second.z);

        assert!(
            (lower - (RIDGE_Z - half)).abs() < AS_NEAR_AS_A_FLOAT_HOLDS,
            "rail {rail} at y {y} is entered at z {} - instead {lower}",
            RIDGE_Z - half
        );

        assert!(
            (upper - (RIDGE_Z + half)).abs() < AS_NEAR_AS_A_FLOAT_HOLDS,
            "rail {rail} at y {y} is left again at z {} - instead {upper}",
            RIDGE_Z + half
        );
    }
}

/// THE RUN SELECTION. Rails 10, 11 and 12 - the slope's own foot inward - are the bite; rails 0
/// to 6, which the wedge passes through just as truly, are the far half of it crossing the round
/// again on the other side of the axis and are no part of the feature.
#[test]
fn the_run_is_the_one_at_the_features_own_seam_and_not_the_wedges_far_half() {
    let mut band = the_tip_band();
    let bite = the_bite(&mut band);

    assert_eq!(
        band.strip.rail_count(),
        13,
        "8 segments over a 135 degree turn is 12 columns, and this test's rail numbers say so"
    );

    let left = band.feature.reaches.borrow()[0].1;

    assert_eq!(bite.at_the_left_seam, left);

    assert_eq!(
        bite.seam_rail(),
        if left { 0 } else { 12 },
        "the bite opens onto the seam of the very face the nose stands on - the right slope"
    );

    assert_eq!(
        bite.rail_of,
        vec![12, 11, 10],
        "the slope's foot inward to the tip is the bite - instead {:?}",
        bite.rail_of
    );

    // And the far half, which is there to be read and is deliberately not read: the middle three
    // rails dip below the ridge and the seven beyond them rise back above it.
    let mut covered = Vec::new();
    let mut found: Vec<(Rc<SurfaceStandingFeatureWithheldFace>, f64)> = Vec::new();

    for rail in 0..band.strip.rail_count() {
        band.feature.crossings_on_the_wedge(
            &band.strip,
            rail,
            SurfaceVertexPool::near_enough_for(band.pool.vertices()),
            &mut found,
        );

        if found.len() == 2 {
            covered.push(rail);
        }
    }

    assert_eq!(
        covered,
        vec![0, 1, 2, 3, 4, 5, 6, 10, 11, 12],
        "the wedge crosses the round twice - instead {covered:?}"
    );
}

/// THE TIP. The bite closes where the nose's ridge crosses the chord between rails 9 and 10,
/// which is read on the CHORD the strip really has and not on the ideal cylinder.
#[test]
fn the_bite_closes_on_one_tip_read_on_the_chord() {
    let mut band = the_tip_band();
    let bite = the_bite(&mut band);

    let last = band.strip.point_on(10, 0.0);
    let beyond = band.strip.point_on(9, 0.0);

    // Where y = 5 stands on that chord, which is one division: the ridge runs along x at y = 5
    // and z = 10, so the chord's own crossing of y = 5 is the point, and z = 10 is where along
    // the rails.
    let how_far = (RIDGE_Y - last.y) / (beyond.y - last.y);
    let on_the_chord = last + ((beyond - last) * how_far);

    let at = band.pool.position_of(bite.tip.vertex);

    assert!(
        (at - Vector3::new(on_the_chord.x, RIDGE_Y, RIDGE_Z)).length() < AS_NEAR_AS_A_FLOAT_HOLDS,
        "the tip is the ridge on the chord between rails 9 and 10 - instead {at:?}"
    );

    assert!(
        (at.x - 15.5202).abs() < 1e-3,
        "which is x 15.5202 on this part at this chording - instead {}",
        at.x
    );

    // And the ideal cylinder would put it 0.045 further in, at AxisX - sqrt(R^2 - (AxisY - 5)^2)
    // = 15.4743. That difference is the sagitta of one 16.875 degree step at radius 3.4 and is the
    // whole point of reading on the chords: a tip read off the cylinder would stand off the mesh
    // by exactly this much, which is a crack.
    let axis_y = 20.0 + (RADIUS * 2.0_f64.sqrt()) - (SIZE - RADIUS);
    let ideal =
        (SIZE - RADIUS) - ((RADIUS * RADIUS) - ((axis_y - RIDGE_Y) * (axis_y - RIDGE_Y))).sqrt();

    assert!(
        at.x - ideal > 0.0,
        "the chord stands inside the cylinder, so the chorded tip is short of the ideal {ideal}"
    );

    assert!(
        at.x - ideal < 0.05,
        "and short of it by the chording only - instead {}",
        at.x - ideal
    );

    assert_eq!(
        bite.tip.below, 9,
        "the tip stands between rails 9 and 10, whose lower is 9"
    );
}

/// THE CORNERS AT THE SEAM. The hole reaches the slope's tangent line at the two points the nose's
/// creases cross it, z = 10 +/- 0.53612 - and the reading takes the WALKBACK's own vertices there
/// rather than minting a second pair beside them off the planes.
#[test]
fn the_holes_corners_at_the_seam_are_the_walkbacks_own_vertices() {
    let mut band = the_tip_band();
    let bite = the_bite(&mut band);

    // The seam rail stands at y = AxisY - R/sqrt(2) = 5.80416, so the closed form puts the corners
    // at z = 10 +/- 2(0.80416)/3 = 10 +/- 0.53611.
    let seam_rail = band.strip.point_on(bite.seam_rail(), 0.0);
    let half = 2.0 * (seam_rail.y - RIDGE_Y) / 3.0;

    let near = band.pool.position_of(bite.near_seam_vertex);
    let far = band.pool.position_of(bite.far_seam_vertex);

    let lower = if near.z < far.z { near } else { far };
    let upper = if near.z < far.z { far } else { near };

    assert!(
        (lower.z - (RIDGE_Z - half)).abs() < AS_NEAR_AS_A_FLOAT_HOLDS,
        "the lower corner is at z {} - instead {lower:?}",
        RIDGE_Z - half
    );

    assert!(
        (upper.z - (RIDGE_Z + half)).abs() < AS_NEAR_AS_A_FLOAT_HOLDS,
        "the upper corner is at z {} - instead {upper:?}",
        RIDGE_Z + half
    );

    // Both of them are the walkback's own, which is the claim that matters: the landing face is
    // already sewn to these two vertices, and a second pair a rounding away would be a crack.
    let walked: HashSet<i32> = band.runs.iter().flatten().copied().collect();

    assert!(walked.contains(&bite.near_seam_vertex));
    assert!(walked.contains(&bite.far_seam_vertex));

    // And the seam the bite leaves behind is the two runs joined, in order along the edge, with
    // nothing added and nothing dropped.
    assert_eq!(bite.seam.len(), band.runs[0].len() + band.runs[1].len());

    for i in 1..bite.seam.len() {
        assert!(
            bite.seam[i].1 > bite.seam[i - 1].1,
            "the joined seam runs one way along the edge - {i} does not"
        );
    }
}

/// AND AN END CASE IS NOT A BITE. The same part's top left edge at 15 reaches the same nose UNDER
/// the band's end, where the notch reader owns it: this reading says "no bite" rather than
/// refusing, so that the caller goes on to the one that builds it.
#[test]
fn a_feature_under_the_bands_end_is_not_a_bite() {
    let mut band = the_band_on(
        nose::create(),
        |features| {
            top_edge_of(
                features,
                Vector3::new(0.0, 0.0, 20.0),
                Vector3::new(10.0, 10.0, 20.0),
            )
        },
        15.0,
        SEGMENTS,
    );

    let ran_off: [Option<&SurfaceStripTrimmedByTheFaceBeyond>; 2] = [None, None];
    let read = SurfaceStripBittenByAFeature::try_read(
        &mut band.pool,
        &band.strip,
        &band.edge,
        &band.feature,
        0,
        &ran_off,
        Some(&band.runs),
    );

    let bite = read.unwrap_or_else(|failure| {
        panic!("a feature under the band's END is not this reading's to refuse - instead {failure}")
    });

    assert!(
        bite.is_none(),
        "what it takes out is a notch open at that end, which SurfaceStripNotchedByAFeature reads"
    );
}

/// AND A REFUSAL SAYS SOMETHING. A side with no run of tangent line on it at all is named rather
/// than handed back as a bare false: the pass formats whatever comes back
/// (`BevelSurfacePartBuilder`), and a refusal with no advisory behind it throws there instead of
/// telling anyone what was wrong.
///
/// Driven at the slice itself rather than through a part, because no fixture reaches it - a strip
/// whose seam was never picked is refused long before a bite is read. The station is built by hand
/// for the same reason: all the slice asks of one is the vertex and parameter on the rail it cuts
/// at.
#[test]
fn a_side_with_no_seam_at_all_is_named() {
    let mut band = the_tip_band();
    let bite = the_bite(&mut band);

    let count = band.strip.rail_count() as usize;
    let rail = vec![bite.near_seam_vertex; count];
    let param = vec![bite.near_seam_param; count];

    // C#'s `new List<(int Vertex, double Param)>[1, 2]`: one chain's two sides, neither filled.
    let seams: Vec<SideSeams> = vec![[None, None]];

    let read = SurfaceStripBiteRows::try_the_seams(
        &band.pool,
        &seams,
        0,
        &bite,
        band.strip.rail_count(),
        &SurfaceStripStation::clean(rail, param),
        true,
    );

    // There is nothing on that side to stitch either section to, and a refusal with nothing to
    // say throws where the pass formats it.
    let failure = read.expect_err("there is nothing on that side to stitch either section to");

    assert_eq!(
        failure.code(),
        BevelAdvisoryCode::SurfaceBiteDoesNotRead,
        "and what it says is that the bite would not read - instead {failure}"
    );
}

/// The bite off the tip band, or the refusal that stopped it read out as one.
fn the_bite(band: &mut Band) -> SurfaceStripBittenByAFeature {
    let ran_off: [Option<&SurfaceStripTrimmedByTheFaceBeyond>; 2] = [None, None];

    let bite = SurfaceStripBittenByAFeature::try_read(
        &mut band.pool,
        &band.strip,
        &band.edge,
        &band.feature,
        0,
        &ran_off,
        Some(&band.runs),
    )
    .unwrap_or_else(|failure| panic!("the bite refused to read: {failure}"));

    bite.expect("the nose mid strip was read as an end case")
}

/// The 3.4 band on the chevron's right knife tip, walked back, with the nose held aside.
fn the_tip_band() -> Band {
    the_band_on(nose::create(), the_tip_edge, RADIUS, SEGMENTS)
}

fn the_tip_edge(features: &FeatureEdgeSet) -> usize {
    let found: Vec<usize> = (0..features.count())
        .filter(|&index| {
            let edge = &features.edges()[index];

            ((edge.start - the_tip_edge_foot()).length() < 1e-6
                && (edge.end - the_tip_edge_top()).length() < 1e-6)
                || ((edge.end - the_tip_edge_foot()).length() < 1e-6
                    && (edge.start - the_tip_edge_top()).length() < 1e-6)
        })
        .collect();

    assert_eq!(found.len(), 1, "one tip edge");

    found[0]
}

/// C# `TheBandOn`. The builder's own steps in the builder's own order, less the ones nothing here
/// needs - a single-edge chain has no junction to read and no room to fit, and both of those stand
/// between the plan and the walkback in `SurfaceBevelBuilder` without feeding it.
fn the_band_on(
    part: Mesh,
    pick: impl Fn(&FeatureEdgeSet) -> usize,
    size: f64,
    segments: i32,
) -> Band {
    let part = Arc::new(part);
    let (regions, features) = regions_of(&part);
    let regions = Arc::new(regions);
    let plan = plan_of(&regions, &features, pick(&features), size, true);

    let walked =
        surface_face_walkback::try_apply_one(&part, &part.get_mesh_edge_graph(), &regions, &plan)
            .unwrap_or_else(|why| panic!("the walkback refused the band: {why}"));

    let strip = SurfaceStrip::try_create(&plan.edges[0], segments)
        .unwrap_or_else(|why| panic!("the strip refused to rule: {why}"));

    let held: Vec<&Rc<SurfaceStandingFeatureWithheld>> = walked
        .standing_features
        .iter()
        .filter(|held| held.reaches.borrow()[0].0 == 0)
        .collect();
    assert_eq!(held.len(), 1, "one feature held aside for edge 0");
    let feature = held[0].clone();

    let left = feature.reaches.borrow()[0].1;
    let runs = walked.tangent_runs[&(0, left)].clone();

    Band {
        pool: SurfaceVertexPool::new(&walked.mesh.vertices),
        strip,
        edge: plan.edges[0].clone(),
        feature,
        runs,
    }
}
