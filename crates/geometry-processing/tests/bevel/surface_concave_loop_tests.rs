//! A CLOSED concave loop - the foot of a square boss standing
//! on a plate - walked back, mitred and sewn. Rung 6 of the polygon-space bevel plan at the
//! kernel's own level, where the loop can be handed over as the four edges it is rather than named
//! through a selection kind.
//!
//! **What a concave loop asks that a convex one does not.** The band comes off each face on the
//! side AWAY from the material the bevel adds: outward across the plate, up the boss's walls. So the
//! plate's band is the OUTWARD offset of the foot, and where the loop turns a corner the two bands
//! meet on the same `d1 + d2` bisector plane a convex rim's do. The outward offset of a square of
//! side `a` by `w` is a square of side `a + 2w`, so the band's area is `4 a w + 4 w^2`; integrated
//! over the depth of the section that is `S L + (sum tan(gamma / 2)) M`, with `S = d^2 / 2` and
//! `M = d^3 / 3` for a chamfer. The parameter rows are a loop.

use std::rc::Rc;
use std::sync::Arc;

use solidmesh::bevel::surface::{
    surface_bevel_builder, surface_face_walkback, surface_mesh_closure, SurfaceBevelPlan,
};
use solidmesh::bevel::{bevel_chain_linker, BevelSweptChain};
use solidmesh::topology::{EdgeConvexity, FeatureEdge, FeatureEdgeSet};
use solidmesh_mesh::CancelToken;

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::surface_walkback_checks::{area_on, regions_of};
use crate::topology_test_meshes::create_square_boss_on_plate;

/// Areas and volumes are summed over float vertices, so they are read to a ten thousandth.
const MEASURED: f64 = 1e-4;

/// The boss is 8 mm square, so its foot is 32 mm round.
const FOOT_PERIMETER: f64 = 32.0;

/// A 20 by 20 by 4 plate with an 8 by 8 by 8 boss standing on it.
const SOURCE_VOLUME: f64 = (20.0 * 20.0 * 4.0) + (8.0 * 8.0 * 8.0);

/// The whole foot of a square boss, chamfered. Four concave edges closing on themselves, four
/// mitres and no end anywhere - and what goes on is the swept section plus one mitre's worth at
/// every corner.
#[test]
fn the_foot_of_a_square_boss_is_beaded_all_round_and_its_mitres_add() {
    for setback in [1.0, 2.0] {
        let boss = Arc::new(create_square_boss_on_plate());
        let (regions, features) = regions_of(&boss);
        let regions = Arc::new(regions);
        let chain = the_foot_loop(&features);

        assert!(
            chain.is_closed,
            "the foot of a boss closes on itself, which is what makes this a loop and not a run"
        );

        let plan: Rc<SurfaceBevelPlan> =
            SurfaceBevelPlan::try_create(&chain, &regions, setback, false)
                .unwrap_or_else(|reason| panic!("{reason}"));

        let built = surface_bevel_builder::try_build_one(
            &boss,
            &boss.get_mesh_edge_graph(),
            &regions,
            plan,
            1,
            &CancelToken::new(),
        )
        .unwrap_or_else(|reason| panic!("{setback}: {reason:?}"));

        assert_eq!(
            surface_mesh_closure::open_edge_count(&built).0,
            0,
            "a bead all the way round a loop leaves a closed solid, with no cap anywhere to get wrong"
        );
        assert!(built.is_manifold());
        assert_eq!(shell_count(&built), 1);

        // S L + (sum of tan(gamma / 2)) M, at four right angles where the tangent is 1
        let section = setback * setback / 2.0;
        let moment = setback * setback * setback / 3.0;
        let expected = SOURCE_VOLUME + (section * FOOT_PERIMETER) + (4.0 * moment);
        let volume = signed_volume(&built);

        assert!(
            (volume - expected).abs() <= MEASURED,
            "{setback}: the mitred outward band of a square is (a + 2w)^2 - a^2 at every depth, \
             which is the perimeter term and one corner term per corner - instead {volume} against \
             {expected}"
        );
    }
}

/// The guard that has to know better. A boss's foot is an inner loop of the plate's top rim, and
/// the band this bevel takes off DOES take the material under it away - which is exactly what a
/// bead's walkback is, and not the undermining
/// `surface_walkback_guard_tests::a_band_that_reaches_a_feature_standing_on_the_face_is_refused`
/// refuses. The difference is whether the loop reached is the chain being bevelled; here it is.
#[test]
fn the_loop_being_bevelled_is_not_read_as_a_feature_standing_in_its_own_band() {
    let boss = Arc::new(create_square_boss_on_plate());
    let (regions, features) = regions_of(&boss);
    let regions = Arc::new(regions);

    let plan = SurfaceBevelPlan::try_create(&the_foot_loop(&features), &regions, 3.0, false)
        .unwrap_or_else(|reason| panic!("{reason}"));

    let walked =
        surface_face_walkback::try_apply_one(&boss, &boss.get_mesh_edge_graph(), &regions, &plan)
            .unwrap_or_else(|reason| {
                panic!(
                    "the loop the band is walked back from is the chain itself - instead {reason}"
                )
            });

    // The plate's top keeps everything but the 3 mm annulus round the foot, and the foot's own
    // corners are gone with it - they are on the chain, which is the point.
    let area = area_on(&walked.mesh, |vertex| {
        (f64::from(vertex.z) - 4.0).abs() < 1e-6
    });

    assert!(
        (area - ((20.0 * 20.0) - (14.0 * 14.0))).abs() <= MEASURED,
        "a 20 mm top less the 14 mm square the 3 mm band round the 8 mm foot opens up - instead \
         {area}"
    );
}

/// The four concave edges where the boss meets the plate, linked into the one loop they make.
fn the_foot_loop(features: &FeatureEdgeSet) -> BevelSweptChain {
    let foot: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| edge.convexity == EdgeConvexity::Concave)
        .copied()
        .collect();

    assert_eq!(foot.len(), 4, "the boss's foot is four edges");

    let mut chains = bevel_chain_linker::link(&foot);
    assert_eq!(chains.len(), 1);

    chains.remove(0)
}
