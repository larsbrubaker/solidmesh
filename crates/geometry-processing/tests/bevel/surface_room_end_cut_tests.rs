//! The room a chain's END takes on the face beyond its corner,
//! negotiated against another chain's run-out - the lobed prism's barrel picked together with the
//! single vertical edge at the flat side's far corner, 10.228 mm away across the bottom cap
//! (`lobed_prism_meshes`).
//!
//! An END is a cut too: the face beyond the corner gives up the section the strip's end profile
//! takes out of it (`SurfaceEndCut`), a notch reaching the profile's own reach from the corner. The
//! pair rule could not see it even in principle - the colliding chain has NO cut on the run-out's
//! region - so the region key short circuits the pair before the geometry is read. The parameter
//! rows are a loop.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, surface_room, surface_room_end_cut,
    surface_room_measure, SurfaceBevelPlan, SurfaceBuildOptions, SurfaceRoomCrossing,
    SurfaceRoomCut, SurfaceRoomFootprint,
};
use geometry_processing::topology::{FaceGroup, FaceRegionCache, FeatureEdge};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::lobed_prism_meshes;
use crate::mesh_measurements::shell_count;
use crate::surface_test_chains::join_and_open;

/// What "Every Chamfer Works.mcx" is filleted at, and what the shell opened at.
const RADIUS: f64 = 3.756;

/// How far the cap runs between the seam and the far corner, which is the gap the skirt and the
/// notch share: the lobe crosses the flat side at 10.228 and the corner stands at the origin.
const ACROSS_THE_CAP: f64 = 10.228;

/// Where the picked vertical edge ends, which is the corner its notch is taken out of.
fn the_end_corner() -> Vector3 {
    Vector3::ZERO
}

/// The end reads as a CORNER footprint on the face beyond it - the bottom cap - seated at the
/// corner itself and reaching the profile's own reach out of it, which for a round on a square
/// corner is the radius. Its room is infinite.
#[test]
fn the_ends_notch_is_a_corner_footprint_on_the_face_beyond_it() {
    let prism = lobed_prism_meshes::create();
    let regions = regions_of(&prism);
    let plans = the_far_corners_edge(&prism, &regions, RADIUS);
    assert_eq!(plans.len(), 1);
    let plan = &plans[0];

    let bottom = plan.edges[0].from().z < plan.edges[0].to().z;

    let notch = surface_room_end_cut::try_cut(Some(plan), bottom).expect(
        "the crease really finishes at the cap, so the cap gives up the section the profile cuts",
    );

    assert_eq!(
        notch.region_key,
        SurfaceBevelPlan::key_of(&the_bottom_cap(&prism)),
        "the face beyond the corner is the one standing across BOTH of the edge's faces there"
    );

    assert!(
        (notch.from - the_end_corner()).length() < 1e-6,
        "the corner is the whole of the footprint's base - instead {:?}",
        notch.from
    );

    assert!(
        notch.corner_footprint(),
        "it spreads from a point rather than along an edge, so the pair rule reads it in the other \
         cut's frame"
    );

    assert!(
        (notch.setback_per_size * RADIUS - RADIUS).abs() <= 1e-9,
        "a round on a square corner reaches its own radius - instead {}",
        notch.setback_per_size * RADIUS
    );

    assert!(
        notch.room == f64::INFINITY,
        "a notch is allowed to reach the end face's own rim"
    );
}

/// With the far corner's edge picked too the two footprints land on one another - and that is a
/// CROSSING and not a refusal: the band that edge lays on the WALL runs under the run-out's own bite
/// of that wall, and the notch on the cap is the far END of the very same removal.
#[test]
fn the_skirt_and_the_ends_band_cross_on_the_wall_rather_than_refusing_the_size() {
    let plans = the_barrel_and_the_far_corners_edge_alone(RADIUS);

    let fit = surface_room::try_fit(&plans, RADIUS, &CancelToken::new(), None)
        .expect("nothing cancels the fit");
    assert!(
        fit.fits,
        "the skirt bites the same wall that edge's band stands on, which is a pair to sew - {:?}, \
         which allowed {}",
        fit.refusal.as_ref().map(|reason| reason.code()),
        fit.largest_fitting_size
    );
    let crossings = &fit.crossings;

    let cuts = surface_room_measure::measure(&plans, None, None);
    let skirt = the_skirt_across_the_bottom_cap(&cuts);
    let notch = the_notch_at_the_far_corner(&cuts);

    let gap = (notch.from - skirt.from).dot(skirt.inward);

    assert!(
        (gap - ACROSS_THE_CAP).abs() <= 1e-3,
        "the cap still runs {ACROSS_THE_CAP} from the seam to the far corner - instead {gap}"
    );

    // One crossing for each of the two mixed corners the barrel's rim turns on the flat wall's own
    // side - the bottom cap's and the top cap's - because the vertical edge's band runs the whole
    // height of that wall and both run-outs bite it.
    let on_the_wall: Vec<SurfaceRoomCrossing> = crossings
        .iter()
        .filter(|sewn| sewn.region_key != skirt.region_key)
        .copied()
        .collect();

    assert_eq!(
        on_the_wall.len(),
        2,
        "the band stands on the wall from cap to cap, and a run-out bites that wall at each end of \
         it - instead {}",
        crossings.len()
    );

    for crossing in &on_the_wall {
        assert_eq!(crossing.skirt.shape, SurfaceRoomFootprint::RunOutSkirt);
        assert_eq!(crossing.band.shape, SurfaceRoomFootprint::Band);

        assert_ne!(
            crossing.band.plan_index, crossing.skirt.plan_index,
            "the band belongs to the picked vertical edge's own chain"
        );

        assert!(
            crossing.also_takes_the_cap_notch,
            "and the notch on the cap is the far end of that same band's removal, carried by the \
             same crossing"
        );

        assert!(
            (crossing.gap - ACROSS_THE_CAP).abs() <= 1e-3,
            "the wall runs {ACROSS_THE_CAP} from the seam to the corner too - instead {}",
            crossing.gap
        );

        assert!(
            (crossing.overlap - 2.734).abs() <= 1e-3,
            "3.756 * (2.4509 + 1) - 10.228 is how far the two reach into each other - instead {}",
            crossing.overlap
        );
    }

    // And a notch whose band this skirt does NOT otherwise cross is the refusal it always was.
    // Asked with a real list that names ANOTHER band, so that what is tested is the keying.
    assert!(SurfaceRoomCrossing::it_is_sewn(
        Some(crossings),
        &skirt,
        &notch
    ));

    let of_another_band: Vec<SurfaceRoomCrossing> = on_the_wall
        .iter()
        .map(|sewn| SurfaceRoomCrossing {
            band: off_another_edge(&sewn.band),
            ..*sewn
        })
        .collect();

    assert!(
        !SurfaceRoomCrossing::it_is_sewn(Some(&of_another_band), &skirt, &notch),
        "with no crossing of THAT band under it, the skirt over the notch is face it may not have"
    );

    // And nor does a crossing of the same band claim the notch at its FAR end.
    assert!(
        !SurfaceRoomCrossing::it_is_sewn(
            Some(crossings),
            &skirt,
            &at_the_far_end_of(&notch, &on_the_wall[0].band)
        ),
        "a band has an end at each end of its edge, and a chain of one edge keys both to that edge"
    );
}

/// The same band read as the NEXT edge of its chain, which is a band this skirt crosses nowhere.
fn off_another_edge(band: &SurfaceRoomCut) -> SurfaceRoomCut {
    SurfaceRoomCut {
        shape: band.shape,
        plan_index: band.plan_index,
        edge_index: band.edge_index + 1,
        side: band.side,
        ..SurfaceRoomCut::new(
            band.region_key,
            band.from,
            band.to,
            band.along,
            band.inward,
            band.length,
            band.setback_per_size,
            band.room,
        )
    }
}

/// The same notch, seated at the OTHER end of the band's own edge.
fn at_the_far_end_of(notch: &SurfaceRoomCut, band: &SurfaceRoomCut) -> SurfaceRoomCut {
    let far = if (notch.from - band.from).length_squared() < (notch.from - band.to).length_squared()
    {
        band.to
    } else {
        band.from
    };

    SurfaceRoomCut {
        shape: notch.shape,
        plan_index: notch.plan_index,
        edge_index: notch.edge_index,
        side: notch.side,
        ..SurfaceRoomCut::new(
            notch.region_key,
            far,
            far,
            notch.along,
            notch.inward,
            notch.length,
            notch.setback_per_size,
            notch.room,
        )
    }
}

/// And the build SEWS it: the run-out's columns end on the band from the turn the two first meet,
/// the band's strip is cut back to the same curve, and the shell closes over both.
#[test]
fn the_crossing_on_the_wall_is_sewn_and_the_shell_closes() {
    let prism = lobed_prism_meshes::create();
    let graph = prism.get_mesh_edge_graph();
    let regions = regions_of(&prism);
    let plans = the_barrel_and_the_far_corners_edge(&prism, &regions, RADIUS);

    let fit = surface_room::try_fit(&plans, RADIUS, &CancelToken::new(), None)
        .expect("nothing cancels the fit");
    assert!(fit.fits);

    let built = surface_bevel_builder::try_build(
        &prism,
        &graph,
        &regions,
        &plans,
        8,
        &CancelToken::new(),
        SurfaceBuildOptions {
            crossings: Some(&fit.crossings),
            ..crate::surface_test_build::build_options()
        },
    )
    .unwrap_or_else(|reason| panic!("the pair is sewn rather than refused - {reason:?}"))
    .mesh;

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
    assert_eq!(open, 0, "open at {open_at:?}");

    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);
}

/// And why the crossing is NAMED rather than left alone: 2.95 and 2.96 straddle the size the two
/// footprints first reach into each other at, `10.228 / (2.4509 + 1)`. Built without the crossing
/// the shell over that size is open.
#[test]
fn under_the_crossing_the_pair_closes_and_over_it_the_shell_would_be_open() {
    for (size, below) in [(2.95, true), (2.96, false)] {
        let prism = lobed_prism_meshes::create();
        let graph = prism.get_mesh_edge_graph();
        let regions = regions_of(&prism);
        let plans = the_barrel_and_the_far_corners_edge(&prism, &regions, size);

        let fit = surface_room::try_fit(&plans, size, &CancelToken::new(), None)
            .expect("nothing cancels the fit");
        assert!(
            fit.fits,
            "the size fits either way now - {:?}",
            fit.refusal.as_ref().map(|reason| reason.code())
        );

        assert_eq!(
            fit.crossings.len(),
            if below { 0 } else { 2 },
            "below the meeting size there is nothing to sew at all - instead {}",
            fit.crossings.len()
        );

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

/// And the edge picked ALONE still fits at the size it always did: its notch has nothing to
/// negotiate with, and a footprint with infinite room constrains nothing.
#[test]
fn the_far_corners_edge_picked_alone_is_unaffected() {
    let prism = lobed_prism_meshes::create();
    let regions = regions_of(&prism);

    let fit = surface_room::try_fit(
        &the_far_corners_edge(&prism, &regions, RADIUS),
        RADIUS,
        &CancelToken::new(),
        None,
    )
    .expect("nothing cancels the fit");

    assert!(
        fit.fits,
        "nothing else has taken the cap, so 3.756 fits as it always did - {:?}, which allowed {}",
        fit.refusal.as_ref().map(|reason| reason.code()),
        fit.largest_fitting_size
    );
}

/// The one cut that matches; fails on none or several.
pub(crate) fn single(
    cuts: &[SurfaceRoomCut],
    matches: impl Fn(&SurfaceRoomCut) -> bool,
) -> SurfaceRoomCut {
    let found: Vec<&SurfaceRoomCut> = cuts.iter().filter(|cut| matches(cut)).collect();

    assert_eq!(found.len(), 1, "exactly one cut matches");

    *found[0]
}

/// The bottom cap's run-out cut: the one on the cap reaching back along the flat side.
fn the_skirt_across_the_bottom_cap(cuts: &[SurfaceRoomCut]) -> SurfaceRoomCut {
    // And told from the run-out's own WALL bite, which travels the same way out of the same corner:
    // that one is laid DOWN the wall, so its width runs along z where a cut on the cap has none.
    single(cuts, |cut| {
        cut.inward.dot(-Vector3::UNIT_X) > 0.999
            && cut.from.z.abs() < 1e-6
            && cut.along.z.abs() < 1e-9
            && !cut.corner_footprint()
            && cut.room == f64::INFINITY
    })
}

/// The notch the picked edge's end takes out of the bottom cap, seated at its corner.
fn the_notch_at_the_far_corner(cuts: &[SurfaceRoomCut]) -> SurfaceRoomCut {
    single(cuts, |cut| {
        (cut.from - the_end_corner()).length() < 1e-6 && (cut.to - the_end_corner()).length() < 1e-6
    })
}

pub(crate) fn regions_of(prism: &Mesh) -> Arc<FaceRegionCache> {
    Arc::new(FaceRegionCache::new(
        Arc::new(prism.clone()),
        Some(prism.get_mesh_edge_graph()),
        lobed_prism_meshes::THRESHOLD_DEGREES,
    ))
}

/// The barrel's rim and the far corner's single vertical edge, planned together, on a prism of
/// their own.
fn the_barrel_and_the_far_corners_edge_alone(size: f64) -> Vec<Rc<SurfaceBevelPlan>> {
    let prism = lobed_prism_meshes::create();

    the_barrel_and_the_far_corners_edge(&prism, &regions_of(&prism), size)
}

fn the_barrel_and_the_far_corners_edge(
    prism: &Mesh,
    regions: &Arc<FaceRegionCache>,
    size: f64,
) -> Vec<Rc<SurfaceBevelPlan>> {
    let mut picked: Vec<FeatureEdge> = the_barrel(prism).boundary_edges().to_vec();

    picked.push(the_vertical_edge_at_the_far_corner(prism));

    plans_for(regions, size, &picked)
}

/// The far corner's single vertical edge, picked alone.
fn the_far_corners_edge(
    prism: &Mesh,
    regions: &Arc<FaceRegionCache>,
    size: f64,
) -> Vec<Rc<SurfaceBevelPlan>> {
    plans_for(regions, size, &[the_vertical_edge_at_the_far_corner(prism)])
}

/// The vertical edge where the flat side meets the side the cap runs across to - the rectangle's
/// own corner, standing 10.228 from the foot of the seam.
fn the_vertical_edge_at_the_far_corner(prism: &Mesh) -> FeatureEdge {
    let side = the_far_side(prism);
    let found: Vec<&FeatureEdge> = side
        .boundary_edges()
        .iter()
        .filter(|edge| {
            edge.direction.dot(Vector3::UNIT_Z).abs() > 0.999
                && (edge.start - the_end_corner()).length() < 1e-6
        })
        .collect();

    assert_eq!(found.len(), 1, "one vertical edge stands at the far corner");

    *found[0]
}

/// Each picked edge joined into chains and planned at one size.
pub(crate) fn plans_for(
    regions: &Arc<FaceRegionCache>,
    size: f64,
    picked: &[FeatureEdge],
) -> Vec<Rc<SurfaceBevelPlan>> {
    join_and_open(&bevel_chain_linker::link(picked))
        .iter()
        .map(|chain| {
            SurfaceBevelPlan::try_create(chain, regions, size, true).unwrap_or_else(|reason| {
                panic!("the pick has to plan at {size}: {:?}", reason.code())
            })
        })
        .collect()
}

pub(crate) fn normal_of(prism: &Mesh, face: usize) -> Vector3 {
    Vector3::from_vector3_float(prism.faces[face].normal)
}

/// The lobe picked whole - one smooth region of 9 degree facets, which is the cylinder's barrel.
pub(crate) fn the_barrel(prism: &Mesh) -> FaceGroup {
    region_at(prism, |face| {
        normal_of(prism, face).dot(-Vector3::UNIT_Y) > 0.99
            && normal_of(prism, face).z.abs() < 1e-6
            && middle_of(prism, face).y < -1e-3
    })
}

/// The flat side the cap runs across to, which is the cube's own side rim in the fixture.
pub(crate) fn the_far_side(prism: &Mesh) -> FaceGroup {
    region_at(prism, |face| {
        (normal_of(prism, face) + Vector3::UNIT_X).length() < 1e-6
    })
}

/// The cap the skirt is laid on: the prism's bottom.
pub(crate) fn the_bottom_cap(prism: &Mesh) -> FaceGroup {
    region_at(prism, |face| {
        (normal_of(prism, face) + Vector3::UNIT_Z).length() < 1e-6
    })
}

/// The region flooded from the first face answering `seeds`.
pub(crate) fn region_at(prism: &Mesh, seeds: impl Fn(usize) -> bool) -> FaceGroup {
    let seed = (0..prism.faces.len())
        .find(|&face| seeds(face))
        .expect("a face answers the seed");

    FaceGroup::create_with_tolerance(prism, seed as i32, lobed_prism_meshes::THRESHOLD_DEGREES)
        .expect("the seed floods a region")
}

/// The middle of one face, which is what tells the lobe's facets from the flat side's.
pub(crate) fn middle_of(mesh: &Mesh, face: usize) -> Vector3 {
    let face = &mesh.faces[face];

    (Vector3::from_vector3_float(mesh.vertices[face.v0 as usize])
        + Vector3::from_vector3_float(mesh.vertices[face.v1 as usize])
        + Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]))
        / 3.0
}
