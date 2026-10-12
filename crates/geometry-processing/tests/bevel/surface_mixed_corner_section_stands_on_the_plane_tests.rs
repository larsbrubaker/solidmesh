//! Port of `SurfaceMixedCornerSectionStandsOnThePlaneTests.cs`: the SECTION the face across gives up
//! where the wall runs out has to stand ON that face's plane - not a ruler outside the part, which is
//! where the crossing turn used to put it.
//!
//! A row of the runout is carried round to the turn its own circle crosses that plane at
//! (`surface_mixed_corner_wall_ends::where_it_crosses`), solved in the circle's own frame. That
//! frame once leaned half a hundredth of a radian off the tangent, the turn came out short, and the
//! row landed up to a ruler past the plane - the speckle of radii the short wall would not close at.
//! Read here as the surface's own statement rather than as a closed mesh. C# `[Arguments]` rows are
//! a loop.

use std::f64::consts::PI;
use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, surface_mixed_corner_wall_ends, SurfaceBevelPlan,
    SurfaceMixedCorner,
};
use geometry_processing::topology::FaceRegionCache;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::lobed_prism_meshes;
use crate::mesh_measurements::shell_count;

/// A radius in the speckle: at 4.3 the section stood 6.5 ten thousandths past the plane and the
/// corner refused, while 4.1 either side of it closed.
const IN_THE_SPECKLE: f64 = 4.3;

/// Every row of the corner's own profile that the trim carries round to the plane really lands ON
/// it - to the double's own precision and not to the part's ruler, since the answer is one arc
/// cosine rather than a search.
#[test]
fn every_row_carried_round_to_the_plane_stands_on_it() {
    for radius in [3.756, IN_THE_SPECKLE] {
        let (_, regions, _, corner) = the_walls_rim(radius);

        let across = surface_mixed_corner_wall_ends::try_read(Some(&regions), &corner)
            .unwrap_or_else(|why| panic!("the wall has to run out here: {:?}", why.code()))
            .expect("the wall has to run out here");

        let mut carried = 0;
        let mut worst = 0.0f64;

        for row in the_profile_at_the_last_turn(&corner) {
            let leaves = surface_mixed_corner_wall_ends::where_it_crosses(&corner, row, across);

            if leaves == f64::INFINITY {
                // A row the wall swallowed before the plane could cut it, which the runout drops.
                continue;
            }

            let off = across
                .signed_distance(corner.turned_about(row, leaves))
                .abs();

            worst = worst.max(off);
            carried += 1;
        }

        assert!(
            carried > 1,
            "the rows the plane cuts are what the section is made of (radius {radius})"
        );

        assert!(
            worst < 1e-9,
            "the worst row landed {worst} off the plane it was carried to (radius {radius})"
        );
    }
}

/// And the corner that would not close now does: at 4.3 the short wall's rim builds closed,
/// manifold and in one shell.
#[test]
fn the_speckled_radius_builds_closed() {
    let (prism, regions, plan, _) = the_walls_rim(IN_THE_SPECKLE);

    let built = surface_bevel_builder::try_build_one(
        &prism,
        &prism.get_mesh_edge_graph(),
        &regions,
        plan,
        8,
        &CancelToken::new(),
    )
    .unwrap_or_else(|reason| panic!("the rim has to build at {IN_THE_SPECKLE}: {reason:?}"));

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
    assert_eq!(open, 0, "open at {open_at:?}");
    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);
}

/// The corner's own profile at the turn the bead's rim ends at - the rows the runout carries round.
/// The centre line of the torus stands a radius under the cap at the outer rail radius out of the
/// axis, so the profile is that circle's own section: the rail where it meets the cap, round to the
/// shared face.
fn the_profile_at_the_last_turn(corner: &SurfaceMixedCorner) -> Vec<Vector3> {
    let size = corner.outer_rail_radius() / 2.0;
    let rail = corner.outer_rail_point(corner.turn_radians());
    let out_of_the_axis = corner.out_of_the_axis(corner.turn_radians());
    let centre_line = rail - (corner.out_of_the_cap_face * size);

    (0..=8)
        .map(|row| {
            let round = (PI / 2.0) * (1.0 - (f64::from(row) / 8.0));

            centre_line
                + (out_of_the_axis * (size * round.cos()))
                + (corner.out_of_the_cap_face * (size * round.sin()))
        })
        .collect()
}

/// The short-walled prism's barrel picked as a face, its rim planned whole at `radius`, and the
/// mixed corner whose runout reaches past the end of the wall.
fn the_walls_rim(
    radius: f64,
) -> (
    Mesh,
    Arc<FaceRegionCache>,
    Rc<SurfaceBevelPlan>,
    Rc<SurfaceMixedCorner>,
) {
    let prism = lobed_prism_meshes::create_with_a_short_wall();
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

    let plan = SurfaceBevelPlan::try_create(&chains.remove(0), &regions, radius, true)
        .unwrap_or_else(|why| panic!("the rim has to plan: {why}"));

    for edge in 0..plan.edges.len() as i32 {
        if let Some(mixed) = plan.try_mixed_corner_after(edge) {
            if mixed.corner.z > lobed_prism_meshes::HEIGHT - 1.0
                && mixed.corner.x > lobed_prism_meshes::centre().x
            {
                return (prism, regions, plan, mixed);
            }
        }
    }

    panic!("the rim turns over at the top of its right-hand seam");
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
