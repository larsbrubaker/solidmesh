//! A mixed corner's RUN-OUT meeting an END's notch of the SAME chain, on one cap - the lobed prism
//! picked as one chain that starts at the far corner's vertical edge, climbs it, crosses the flat
//! wall's top rim, comes back down the seam and runs away round the lobe's bottom rim
//! (`lobed_prism_meshes`).
//!
//! A notch is never charged against its own plan's bands, because the section it takes is bounded
//! by the two tangent points standing ON those bands; the skirt carried its plan's index too, so
//! the exemption swallowed it. The skirt is no band of an edge, and telling it apart is the whole
//! of the fix: it is a footprint of its own (`SurfaceRoomFootprint::RunOutSkirt`).

use std::rc::Rc;
use std::sync::Arc;

use solidmesh::bevel::bevel_chain_linker;
use solidmesh::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, surface_room, surface_room_measure,
    SurfaceBevelPlan, SurfaceRoomCrossing, SurfaceRoomCut, SurfaceRoomFootprint,
};
use solidmesh::topology::{FaceGroup, FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use solidmesh_math::Vector3;
use solidmesh_mesh::{CancelToken, Mesh};

use crate::lobed_prism_meshes;
use crate::mesh_measurements::shell_count;
use crate::surface_test_chains::join_and_open;

/// How far the cap runs between the seam's foot and the far corner, which is the gap the skirt and
/// the notch share: the lobe crosses the flat side at 10.228 and the corner stands at the origin.
const ACROSS_THE_CAP: f64 = 10.228;

/// The size the two of them meet at: the gap over what the two of them take per unit of size,
/// which at this size is 2.4645 of skirt and 1 of notch.
const THEY_MEET_AT: f64 = 2.952;

/// Where the picked chain's free convex end stands, which is the corner its notch is taken out of.
fn the_end_corner() -> Vector3 {
    Vector3::ZERO
}

/// Where the seam's foot stands, which is the corner the skirt runs out from.
fn the_seam_foot() -> Vector3 {
    Vector3::new(lobed_prism_meshes::left_crossing().x, 0.0, 0.0)
}

/// The chain lays BOTH footprints on the bottom cap, and they are one plan's: the skirt travelling
/// out of the seam's foot towards the corner, and the notch seated at the corner.
#[test]
fn the_skirt_and_the_notch_are_one_plans_two_footprints_on_one_cap() {
    let prism = lobed_prism_meshes::create();
    let plans = the_curling_chain(&prism, &regions_of(&prism), 2.96);
    let cuts = surface_room_measure::measure(&plans, None, None);

    let skirt = the_skirt_across_the_bottom_cap(&prism, &cuts);
    let notch = single(&cuts, |cut| cut.shape == SurfaceRoomFootprint::EndNotch);

    assert_eq!(
        skirt.shape,
        SurfaceRoomFootprint::RunOutSkirt,
        "a run-out belongs to no edge, so it is not one of the bands a notch is let off against"
    );

    assert_eq!(
        skirt.plan_index, notch.plan_index,
        "one pick, one chain, one plan - which is the very thing the exemption keyed on"
    );

    assert_eq!(
        notch.region_key, skirt.region_key,
        "both stand on the bottom cap, so there is a gap between them to be read at all"
    );

    assert!(
        (notch.from - the_end_corner()).length() < 1e-3,
        "the notch is seated at the chain's free end - instead {:?}",
        notch.from
    );

    assert!(
        (skirt.from - the_seam_foot()).dot(skirt.inward).abs() < 1e-3,
        "and the skirt travels out of the seam's foot, which is where its reach is measured from - \
         instead {:?}",
        skirt.from
    );

    let gap = (notch.from - skirt.from).dot(skirt.inward);

    assert!(
        (gap - ACROSS_THE_CAP).abs() <= 1e-3,
        "the cap runs {ACROSS_THE_CAP} from the seam to the corner - instead {gap}"
    );
}

/// So the fit ACCEPTS the size whose skirt reaches its own chain's notch, and reports the pair as a
/// crossing to sew: the notch is the far END of the very band this chain lays on the wall the same
/// corner's run-out bites.
#[test]
fn the_skirt_and_its_own_chains_notch_cross_rather_than_refusing_the_size() {
    let prism = lobed_prism_meshes::create();
    let plans = the_curling_chain(&prism, &regions_of(&prism), 2.96);

    let fit = surface_room::try_fit(&plans, 2.96, &CancelToken::new(), None)
        .expect("nothing cancels the fit");
    assert!(
        fit.fits,
        "the skirt bites the wall its own chain's first edge stands on, which is a pair to sew - \
         {:?}, which allowed {}",
        fit.refusal.as_ref().map(|reason| reason.code()),
        fit.largest_fitting_size
    );

    let cuts = surface_room_measure::measure(&plans, None, None);
    let skirt = the_skirt_across_the_bottom_cap(&prism, &cuts);
    let notch = single(&cuts, |cut| cut.shape == SurfaceRoomFootprint::EndNotch);
    let gap = (notch.from - skirt.from).dot(skirt.inward);
    let meet = gap / (skirt.setback_per_size + notch.setback_per_size);

    assert!(
        (meet - THEY_MEET_AT).abs() <= 5e-3,
        "the two really do reach into each other at 2.96 - they meet at 10.228 / (2.4645 + 1) = \
         {THEY_MEET_AT}, and this reads {meet}"
    );

    assert!(
        SurfaceRoomCrossing::it_is_sewn(Some(&fit.crossings), &skirt, &notch),
        "and the pair is sewn as the far end of the crossing the same skirt makes on the wall"
    );

    assert!(
        !SurfaceRoomCrossing::it_is_sewn(Some(&[]), &skirt, &notch),
        "with no crossing of that band under it, a skirt over a notch is the refusal it always was"
    );

    assert!(
        fit.crossings
            .iter()
            .any(|sewn| sewn.also_takes_the_cap_notch),
        "the crossing itself says that the band's own notch stands under the same skirt"
    );
}

/// And why the crossing is NAMED rather than left alone: at 2.95 the chain closes into one manifold
/// shell with nothing to sew, and at 2.96 a build that ignores the crossing really does leave the
/// shell open along the curve the two surfaces cross in.
#[test]
fn under_the_crossing_the_chain_closes_and_over_it_the_shell_would_be_open() {
    for (size, below) in [(2.95, true), (2.96, false)] {
        let prism = lobed_prism_meshes::create();
        let graph = prism.get_mesh_edge_graph();
        let regions = Arc::new(FaceRegionCache::new(
            Arc::new(prism.clone()),
            Some(graph.clone()),
            lobed_prism_meshes::THRESHOLD_DEGREES,
        ));
        let plans = the_curling_chain(&prism, &regions, size);

        let fit = surface_room::try_fit(&plans, size, &CancelToken::new(), None)
            .expect("nothing cancels the fit");
        assert!(
            fit.fits,
            "the size fits either way now - {:?}",
            fit.refusal.as_ref().map(|reason| reason.code())
        );

        assert_eq!(
            !fit.crossings.is_empty(),
            !below,
            "below the meeting size there is nothing to sew at all - instead {}",
            fit.crossings.len()
        );

        // Asked WITHOUT the crossing (a plain `try_build`), the build has nothing to
        // refuse either way.
        let built = surface_bevel_builder::try_build(
            &prism,
            &graph,
            &regions,
            &plans,
            8,
            &CancelToken::new(),
            crate::surface_test_build::build_options(),
        )
        .unwrap_or_else(|reason| {
            panic!("asked WITHOUT the crossing the build has nothing to refuse either way - {reason:?}")
        })
        .mesh;

        let (open, open_at) = surface_mesh_closure::open_edge_count(&built);

        if below {
            assert_eq!(open, 0, "open at {open_at:?}");
            assert!(built.is_manifold());
            assert_eq!(shell_count(&built), 1);
        } else {
            assert!(
                open > 0,
                "which is the open shell the named refusal stands in place of"
            );
        }
    }
}

/// The one cut that matches; a test failure if there is not exactly one.
fn single(cuts: &[SurfaceRoomCut], matches: impl Fn(&SurfaceRoomCut) -> bool) -> SurfaceRoomCut {
    let found: Vec<&SurfaceRoomCut> = cuts.iter().filter(|cut| matches(cut)).collect();

    assert_eq!(found.len(), 1, "exactly one cut matches");

    *found[0]
}

/// The skirt the seam's mixed corner lays on the bottom cap: the footprint no edge owns there.
fn the_skirt_across_the_bottom_cap(prism: &Mesh, cuts: &[SurfaceRoomCut]) -> SurfaceRoomCut {
    let cap = SurfaceBevelPlan::key_of(&the_bottom_cap(prism));

    single(cuts, |cut| {
        cut.region_key == cap && !cut.corner_footprint() && cut.room == f64::INFINITY
    })
}

fn regions_of(prism: &Mesh) -> Arc<FaceRegionCache> {
    Arc::new(FaceRegionCache::new(
        Arc::new(prism.clone()),
        Some(prism.get_mesh_edge_graph()),
        lobed_prism_meshes::THRESHOLD_DEGREES,
    ))
}

/// The chain this fixture is about, planned at one size: the far corner's vertical edge, the flat
/// wall's top rim, the seam, and the whole of the lobe's bottom rim - one open chain with a free
/// convex end at the origin and a mixed corner at the seam's foot.
fn the_curling_chain(
    prism: &Mesh,
    regions: &Arc<FaceRegionCache>,
    size: f64,
) -> Vec<Rc<SurfaceBevelPlan>> {
    join_and_open(&bevel_chain_linker::link(&the_picked_edges(prism)))
        .iter()
        .map(|chain| {
            SurfaceBevelPlan::try_create(chain, regions, size, true).unwrap_or_else(|reason| {
                panic!("the pick has to plan at {size}: {:?}", reason.code())
            })
        })
        .collect()
}

/// The four pieces of rim the pick is made of, read off the mesh's own feature edges by where they
/// stand: up the far corner, across the top of the flat wall, down the seam, and away round the
/// lobe.
fn the_picked_edges(prism: &Mesh) -> Vec<FeatureEdge> {
    let foot = lobed_prism_meshes::left_crossing().x;

    // The mesh keeps its vertices as floats, so the seam's own x is only good to about a
    // thousandth of a millimetre of the outline's.
    const TOLERANCE: f64 = 1e-3;

    let features = FeatureEdgeSet::create_with_graph(
        prism,
        &prism.get_mesh_edge_graph(),
        lobed_prism_meshes::THRESHOLD_DEGREES,
    );

    features
        .edges()
        .iter()
        .filter(|edge| {
            let start = edge.start;
            let end = edge.end;
            let vertical = edge.direction.dot(Vector3::UNIT_Z).abs() > 0.999;
            let on_the_flat_side = start.y.abs() < TOLERANCE && end.y.abs() < TOLERANCE;

            let the_far_corners_edge = vertical && start.length() < TOLERANCE;
            let the_seam =
                vertical && (start.x - foot).abs() < TOLERANCE && start.y.abs() < TOLERANCE;
            let the_walls_top_rim = on_the_flat_side
                && (start.z - lobed_prism_meshes::HEIGHT).abs() < TOLERANCE
                && (end.z - lobed_prism_meshes::HEIGHT).abs() < TOLERANCE
                && start.x.max(end.x) < foot + TOLERANCE;
            let the_lobes_bottom_rim = start.z.abs() < TOLERANCE
                && end.z.abs() < TOLERANCE
                && start.y.min(end.y) < -TOLERANCE;

            the_far_corners_edge || the_seam || the_walls_top_rim || the_lobes_bottom_rim
        })
        .copied()
        .collect()
}

/// The cap both footprints are laid on: the prism's bottom.
fn the_bottom_cap(prism: &Mesh) -> FaceGroup {
    let seed = (0..prism.faces.len())
        .find(|&face| {
            (Vector3::from_vector3_float(prism.faces[face].normal) + Vector3::UNIT_Z).length()
                < 1e-6
        })
        .expect("the prism has a bottom cap");

    FaceGroup::create_with_tolerance(prism, seed as i32, lobed_prism_meshes::THRESHOLD_DEGREES)
        .expect("the cap is a region")
}
