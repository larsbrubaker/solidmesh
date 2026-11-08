//! The blend's sector on the cap face is bounded by the
//! plane the cut's band STOPS on, whatever the corner hands over on (`surface_mixed_corner_sector`) -
//! and not only where the ring was read as a mitre.
//!
//! A corner handing over on its own tangency leans too. On the lobe wall's rim at `r = 5` the ring's
//! own vertex on the cap stands 1.0151 degrees round from the trace of that plane, which is a ribbon
//! of cap between the two lines that the bite KEEPS and nothing sews to: 0.895 square millimetres
//! here. The sizes it was measured on are the wall sweep of "Round Side Does not bevel attached
//! edges" (`BevelRadiusSweepTests`).

use std::rc::Rc;
use std::sync::Arc;

use solidmesh::bevel::bevel_chain_linker;
use solidmesh::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, surface_mixed_corner_sector, SurfaceBevelPlan,
    SurfaceMixedCorner, SurfaceVertexPool,
};
use solidmesh::topology::FaceRegionCache;
use solidmesh_math::Vector3;
use solidmesh_mesh::{CancelToken, Mesh};

use crate::lobed_prism_meshes;
use crate::mesh_measurements::shell_count;
use crate::surface_mixed_corner_fixture as fixture;

/// The size whose corners lean without being read as mitres - the plain case of the bug.
const LEANING: f64 = 5.0;

/// How far round from the trace the ring's own cap vertex stands there, in degrees.
const LEANS_BY: f64 = 1.0151;

/// At `r = 5` the corner does NOT hand over at a mitre, and its ring leans all the same: the apex
/// stands off the axis, along the closing radius, and the sector's first side runs along the trace
/// of the plane the cut's band stops on.
#[test]
fn the_first_side_is_the_trace_even_where_the_handover_is_not_a_mitre() {
    let (prism, _, _, corner) = the_walls_rim(LEANING);

    assert!(
        !corner.hands_over_at_a_mitre,
        "this is the reading the sector used to skip, which is the whole of the bug"
    );

    let ring_on_the_cap = the_rings_own_vertex_on_the_cap(&corner);
    let last_on_the_cap = corner.outer_rail_point(corner.turn_radians());

    let apex = surface_mixed_corner_sector::try_apex(&corner, ring_on_the_cap, last_on_the_cap)
        .unwrap_or_else(|refused| panic!("the apex has to read: {:?}", refused.code()));

    // On the last station's own radius, so the side the runout's sector shares is the line it was.
    let along_the_radius = (last_on_the_cap - corner.outer_rail_centre).get_normal();
    let off_the_axis = apex - corner.outer_rail_centre;

    assert!(
        (off_the_axis - (along_the_radius * off_the_axis.dot(along_the_radius))).length() < 1e-9,
        "the apex stays on that radius - instead {apex:?}"
    );

    assert!(
        (off_the_axis.dot(along_the_radius) - -0.2077).abs() <= 1e-3,
        "and stands that far back along it, on the far side of the axis - instead {apex:?}"
    );

    // And the first side IS the trace: square to the inward normal of the plane the band stops on.
    let along_the_first_side = (ring_on_the_cap - apex).get_normal();

    assert!(
        along_the_first_side.dot(corner.cut_stops_at.inward).abs() < 1e-9,
        "instead {along_the_first_side:?} against {:?}",
        corner.cut_stops_at.inward
    );

    // The ribbon the old sector left standing, read two ways: as the lean itself, and as the wedge
    // of cap between the trace and the radius the sector was bounded by.
    let lean = (ring_on_the_cap - corner.outer_rail_centre)
        .get_normal()
        .dot(along_the_first_side)
        .clamp(-1.0, 1.0)
        .acos()
        .to_degrees();

    assert!((lean - LEANS_BY).abs() <= 0.001, "instead {lean}");

    let wedge = off_the_axis
        .cross(ring_on_the_cap - corner.outer_rail_centre)
        .length()
        / 2.0;

    assert!(
        (wedge - 0.8950).abs() <= 1e-3,
        "instead {wedge}, against a ruler of {}",
        SurfaceVertexPool::near_enough_for(&prism.vertices)
    );
}

/// And the rim CLOSES at that size: one shell, manifold, nothing open. Bounded by the radius it came
/// back with 52 edges open, every one of them on a face the cap bite made.
#[test]
fn the_rim_closes_at_the_size_the_ribbon_used_to_stand_at() {
    let (prism, regions, plan, _) = the_walls_rim(LEANING);

    let built = surface_bevel_builder::try_build_one(
        &prism,
        &prism.get_mesh_edge_graph(),
        &regions,
        plan,
        8,
        &CancelToken::new(),
    )
    .unwrap_or_else(|reason| panic!("the rim has to build at {LEANING}: {reason:?}"));

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
    assert_eq!(open, 0, "open at {open_at:?}");

    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);
}

/// And the ORDINARY corner is unmoved: where the handover plane is spanned by the axis its trace
/// runs through the axis, so the crossing is the axis itself - the same Vector3, bit for bit, and not
/// a hair off it - and every number the L prism's build is pinned at stands where it stood.
#[test]
fn an_ordinary_corners_apex_is_the_axis_itself_and_not_arithmetic_near_it() {
    let (plan, _, _) = fixture::the_plan();
    let corner = fixture::the_corner(&plan);

    assert!(
        !corner.hands_over_at_a_mitre,
        "the L prism's corner is the square one this pin is about"
    );

    let apex = surface_mixed_corner_sector::try_apex(
        &corner,
        corner.outer_rail_point(0.0),
        corner.outer_rail_point(corner.turn_radians()),
    )
    .unwrap_or_else(|refused| panic!("the apex has to read: {:?}", refused.code()));

    assert!(
        apex == corner.outer_rail_centre,
        "exactly the axis, so the bite's half planes are the ones they were - instead {apex:?} \
         against {:?}",
        corner.outer_rail_centre
    );
}

/// The ring's own vertex on the cap face: the cut's tangent line there, met with the plane the ring
/// is cut at - the same two readings the patch's first column is built between.
fn the_rings_own_vertex_on_the_cap(corner: &SurfaceMixedCorner) -> Vector3 {
    let on_the_cap = if corner.cut.left_region_key() == corner.shared_region_key {
        corner.cut.right_tangent_line
    } else {
        corner.cut.left_tangent_line
    };

    let across = on_the_cap.direction.dot(corner.cut_stops_at.inward);

    on_the_cap.point
        + (on_the_cap.direction
            * ((corner.cut_stops_at.point - on_the_cap.point).dot(corner.cut_stops_at.inward)
                / across))
}

/// The LOBE WALL as a face pick - the region a hover highlights when it lands on the barrel -
/// planned whole at `radius`, with the first of its mixed corners read off the plan. The four
/// corners of this rim are the same reading four times over, so the first one is all of them.
fn the_walls_rim(
    radius: f64,
) -> (
    Mesh,
    Arc<FaceRegionCache>,
    Rc<SurfaceBevelPlan>,
    Rc<SurfaceMixedCorner>,
) {
    let prism = lobed_prism_meshes::create();
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(prism.clone()),
        Some(prism.get_mesh_edge_graph()),
        lobed_prism_meshes::THRESHOLD_DEGREES,
    ));

    let region = regions
        .region(a_facet_of_the_lobes_wall(&prism))
        .expect("the wall is a region");
    let mut chains = bevel_chain_linker::link(region.boundary_edges());
    assert_eq!(chains.len(), 1);
    let chain = chains.remove(0);

    let plan = SurfaceBevelPlan::try_create(&chain, &regions, radius, true)
        .unwrap_or_else(|why| panic!("the wall's rim has to plan at {radius}: {:?}", why.code()));

    for key in plan.mixed_corner_keys() {
        if let Some(corner) = plan.try_mixed_corner_after(key) {
            return (prism, regions, plan, corner);
        }
    }

    panic!("the wall's rim turns over at its seams at {radius}");
}

/// Any one facet of the lobe's own wall - a side face standing well outside the flat side.
fn a_facet_of_the_lobes_wall(prism: &Mesh) -> i32 {
    for (face, facet) in prism.faces.iter().enumerate() {
        let middle = (Vector3::from_vector3_float(prism.vertices[facet.v0 as usize])
            + Vector3::from_vector3_float(prism.vertices[facet.v1 as usize])
            + Vector3::from_vector3_float(prism.vertices[facet.v2 as usize]))
            / 3.0;

        if middle.y < -lobed_prism_meshes::RADIUS / 2.0
            && middle.z > 1.0
            && middle.z < lobed_prism_meshes::HEIGHT - 1.0
        {
            return face as i32;
        }
    }

    panic!("the prism has lost the lobe it pokes through its side");
}
