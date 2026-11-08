//! A chain end's notch reaches out of its corner in a
//! DIRECTION, not every way at once (`SurfaceRoomNotchSides`). The nose-in-notch part's ridge
//! picked together with the cube's knife edge at the origin is the pair that proves it: the ridge's
//! end notch on the left slope and the knife's band on the same slope are 2.24 mm clear of each
//! other, and the omnidirectional reading had them 0.757 mm overlapped.

use std::rc::Rc;
use std::sync::Arc;

use solidmesh::bevel::surface::{
    surface_room, surface_room_end_cut, surface_room_measure, SurfaceBevelPlan, SurfaceRoomCut,
    SurfaceRoomFootprint,
};
use solidmesh::bevel::BevelSweptChain;
use solidmesh::topology::{ChainEdge, FaceRegionCache, FeatureEdgeSet};
use solidmesh_math::Vector3;
use solidmesh_mesh::CancelToken;

use crate::nose_in_notch_test_meshes;
use crate::surface_walkback_checks::regions_of;

/// What the pair is asked for, and what the omnidirectional reading refused at 1.8065.
const SIZE: f64 = 2.0;

/// The ridge's end, where the nose bites into the left slope.
fn the_ridge_end() -> Vector3 {
    nose_in_notch_test_meshes::left_ridge_end()
}

/// The two sides of the ridge end's section, at [`SIZE`]: where each tangent line meets the slope
/// `y = x`, measured from the corner.
fn the_upper_side() -> Vector3 {
    Vector3::new(2.4962, 2.4962, 1.6641)
}

fn the_lower_side() -> Vector3 {
    Vector3::new(2.4962, 2.4962, -1.6641)
}

/// 1 + sqrt 2 as a literal, so the expected value does not depend on how `sqrt` rounds.
#[allow(clippy::excessive_precision)]
const ONE_PLUS_ROOT_TWO: f64 = 2.414_213_562_373_095_1;

/// The section leans back along the creases rather than at the knife's band, so the pair does not
/// charge and the size the user asked for fits.
#[test]
fn the_ridge_ends_notch_leans_away_from_the_knife_edges_band_and_the_size_fits() {
    let part = nose_in_notch_test_meshes::create();
    let (regions, features) = regions_of(&Arc::new(part.clone()));
    let regions = Arc::new(regions);
    let plans = the_pair(&regions, &features, SIZE);

    let cuts = surface_room_measure::measure(&plans, None, None);

    let notches: Vec<&SurfaceRoomCut> = cuts
        .iter()
        .filter(|cut| {
            cut.shape == SurfaceRoomFootprint::EndNotch
                && (cut.from - the_ridge_end()).length() < 1e-6
        })
        .collect();
    assert_eq!(notches.len(), 1, "one notch stands at the ridge's end");
    let notch = notches[0];

    let (Some(side_a), Some(side_b)) = (notch.section_side_a, notch.section_side_b) else {
        panic!("the left slope is one plane, so both tangent lines cross it and the section has two sides");
    };

    let upper = side_a * SIZE;
    let lower = side_b * SIZE;

    assert!(
        (upper - the_upper_side()).length() < 1e-3,
        "the upper tangent line meets the slope at (7.4962, 7.4962, 11.6641) - instead the side is {upper:?}"
    );
    assert!(
        (lower - the_lower_side()).length() < 1e-3,
        "the lower one meets it at (7.4962, 7.4962, 8.3359) - instead the side is {lower:?}"
    );

    let bands: Vec<&SurfaceRoomCut> = cuts
        .iter()
        .filter(|cut| cut.shape == SurfaceRoomFootprint::Band && cut.region_key == notch.region_key)
        .collect();
    assert_eq!(
        bands.len(),
        1,
        "the knife edge lays one band on the left slope"
    );
    let knifes_band = bands[0];

    let leaning = (-side_a.dot(knifes_band.inward)).max(-side_b.dot(knifes_band.inward));

    assert!(
        leaning < 0.0,
        "both sides lean back along the creases, away from the knife's band - instead {leaning:.5}"
    );

    let fit =
        surface_room::try_fit(&plans, SIZE, &CancelToken::new(), None).expect("not cancelled");

    assert!(
        fit.fits,
        "nothing of the section stands in front of the band - instead {:?} at {:.5}",
        fit.refusal.as_ref().map(|refusal| refusal.code()),
        fit.largest_fitting_size
    );
    assert!((fit.largest_fitting_size - SIZE).abs() <= 1e-9);
}

/// The control: the same knife edge's notch on the FLOOR, whose section really does lean at the
/// floor's own rims, is charged for exactly the part of it that leans that way - in full at the rim
/// it points straight at, and less at the one it leans across.
#[test]
fn a_notch_leaning_at_a_band_is_still_charged_for_what_leans_that_way() {
    let part = nose_in_notch_test_meshes::create();
    let (regions, features) = regions_of(&Arc::new(part.clone()));
    let regions = Arc::new(regions);
    let knife = match SurfaceBevelPlan::try_create(&the_knife_edge(&features), &regions, SIZE, true)
    {
        Ok(plan) => plan,
        Err(why) => panic!("{:?}", why.code()),
    };

    let on_the_floor = knife.edges[0].from().z < knife.edges[0].to().z;

    let notch = surface_room_end_cut::try_cut(Some(&knife), on_the_floor).expect(
        "the knife edge finishes on the floor, which gives up the section its profile cuts",
    );

    for (name, from, along, inward, answer) in [
        (
            "y = 20",
            Vector3::new(0.0, 20.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, -1.0, 0.0),
            20.0 / (1.0 + ONE_PLUS_ROOT_TWO),
        ),
        (
            "x = 20",
            Vector3::new(20.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(-1.0, 0.0, 0.0),
            20.0 / (1.0 + 1.707_106_781_186_547_5),
        ),
    ] {
        let rim = SurfaceRoomCut::new(
            notch.region_key,
            from,
            from + (along * 20.0),
            along,
            inward,
            20.0,
            // A round on a square corner sets back its own radius.
            1.0,
            f64::INFINITY,
        );

        let (largest, worst_at, worst_is_a_collision) = surface_room::largest_fitting(
            &[rim, notch],
            f64::INFINITY,
            &CancelToken::new(),
            f64::INFINITY,
            None,
            None,
        )
        .expect("not cancelled");

        assert!(
            worst_is_a_collision,
            "the section leans at the {name} rim, so the pair answers rather than either one's own room"
        );
        assert!(
            (worst_at - Vector3::ZERO).length() < 1e-9,
            "the collision is reported at the knife's corner - instead {worst_at:?}"
        );
        assert!(
            (largest - answer).abs() <= 1e-4,
            "the {name} rim and the section meet at {answer:.5} - instead {largest:.5}"
        );
    }
}

/// The ridge and the knife edge, planned at one size: the pair the fixture is picked as.
fn the_pair(
    regions: &Arc<FaceRegionCache>,
    features: &FeatureEdgeSet,
    size: f64,
) -> Vec<Rc<SurfaceBevelPlan>> {
    [the_ridge(features), the_knife_edge(features)]
        .iter()
        .map(
            |chain| match SurfaceBevelPlan::try_create(chain, regions, size, true) {
                Ok(plan) => plan,
                Err(why) => panic!("the fixture's chain would not plan: {:?}", why.code()),
            },
        )
        .collect()
}

/// The nose's exposed ridge, convex, from (5, 5, 10) to (15, 5, 10).
fn the_ridge(features: &FeatureEdgeSet) -> BevelSweptChain {
    edge_between(
        features,
        nose_in_notch_test_meshes::left_ridge_end(),
        nose_in_notch_test_meshes::right_ridge_end(),
    )
}

/// The cube's own vertical edge at the origin, where the face `x = 0` meets the left slope `y = x`
/// at 45 degrees - the knife edge the notch's section is measured against.
fn the_knife_edge(features: &FeatureEdgeSet) -> BevelSweptChain {
    edge_between(
        features,
        Vector3::ZERO,
        Vector3::new(0.0, 0.0, nose_in_notch_test_meshes::SIZE),
    )
}

/// The fixture's one feature edge between two named vertices, as a chain of its own.
fn edge_between(features: &FeatureEdgeSet, from: Vector3, to: Vector3) -> BevelSweptChain {
    for (i, edge) in features.edges().iter().enumerate() {
        if (edge.start - from).length() < 1e-6 && (edge.end - to).length() < 1e-6 {
            return BevelSweptChain::new(vec![ChainEdge::new(i as i32, *edge, false)], false);
        }

        if (edge.end - from).length() < 1e-6 && (edge.start - to).length() < 1e-6 {
            return BevelSweptChain::new(vec![ChainEdge::new(i as i32, *edge, true)], false);
        }
    }

    panic!("the fixture has no edge from {from:?} to {to:?}");
}
