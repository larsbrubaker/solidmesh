//! The L prism's JUNCTION, BUILT - the mesh the two
//! cuts, the bead and the torus between them publish, measured against the closed forms the
//! reading derives (`surface_mixed_three_edge_corner_tests`, whose plan and strip helpers these
//! share). The reading's tests are pure; these run the whole pass at three samplings and measure
//! what it leaves.

use std::collections::HashMap;
use std::rc::Rc;
use std::sync::Arc;

use solidmesh::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, surface_mixed_junction_patch, SurfaceBevelPlan,
    SurfaceMixedJunction, SurfaceStrip, SurfaceVertexPool,
};
use solidmesh::topology::FaceRegionCache;
use solidmesh_math::{math_helper, Vector3};
use solidmesh_mesh::{CancelToken, Face, Mesh};

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::surface_mixed_corner_fixture as fixture;
use crate::surface_mixed_three_edge_corner_tests::{plan_of, strip_of};
use crate::surface_walkback_checks::area_on;
use crate::topology_test_meshes::create_l_shaped_grid_prism;

const RADIUS: f64 = 1.0;

/// How near a published vertex has to come to a surface to be read as standing on it: these
/// positions are read back out of float storage AFTER a turn, a crossing and a weld, and the
/// reading is a distance to a circle rather than to a plane.
const ON_THE_ROUND: f64 = 1e-5;

/// The L's top reflex corner, where the crease and both top rims meet.
fn the_corner() -> Vector3 {
    Vector3::new(10.0, 10.0, 10.0)
}

fn assert_within(actual: f64, expected: f64, tolerance: f64, because: &str) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} is not {expected} within {tolerance}: {because}"
    );
}

/// The junction BUILDS: a closed, manifold, single shell whose volume is the junction's own
/// closed form.
fn the_junction_closes_at_every_sampling(segments: i32) {
    let (built, junction, cut_strip) = the_junction_built(segments);

    assert!(
        built.is_manifold(),
        "a bevel of a closed solid stays a closed solid"
    );
    let (closed, open_at) = surface_mesh_closure::is_closed(&built);
    assert!(
        closed,
        "nothing may be left open, and something is at {open_at:?}"
    );
    assert_eq!(shell_count(&built), 1);

    assert_within(
        signed_volume(&built).abs(),
        fixture::the_junctions_volume(&junction.first.mixed, &cut_strip),
        3000.0 * 1e-9,
        "the L less two cuts, plus the bead, plus the corner square, less what the blend sweeps",
    );

    assert_within(
        area_on(&built, |v| (v.x - 10.0).abs() < 1e-5),
        81.0,
        1e-6,
        "the step wall keeps its 9 by 9 square and nothing of the blend lands on it",
    );
    assert_within(
        area_on(&built, |v| (v.y - 10.0).abs() < 1e-5),
        81.0,
        1e-6,
        "and so does the void wall",
    );
    assert_within(
        area_on(&built, |v| (v.z - 10.0).abs() < 1e-5),
        fixture::the_junctions_cap_area(segments),
        1e-6,
        "the cap loses its two bands and the sector the blend's outer rail sweeps",
    );
}

#[test]
fn the_junction_closes_at_every_sampling_4() {
    the_junction_closes_at_every_sampling(4);
}

#[test]
fn the_junction_closes_at_every_sampling_8() {
    the_junction_closes_at_every_sampling(8);
}

#[test]
fn the_junction_closes_at_every_sampling_16() {
    the_junction_closes_at_every_sampling(16);
}

/// What it leaves at the corner is the four points the two tangencies and the two ends of the
/// outer rail name - and NOT the four the corner used to have, every one now under the blend.
#[test]
fn the_corners_own_four_points_stand_and_the_old_four_are_gone() {
    let (built, _, _) = the_junction_built(8);

    for at in [
        Vector3::new(10.0, 11.0, 9.0), // P, where the first cut touches the bead on the step wall
        Vector3::new(11.0, 10.0, 9.0), // Q, where the second touches it on the void wall
        Vector3::new(9.0, 11.0, 10.0), // the outer rail at turn zero
        Vector3::new(11.0, 9.0, 10.0), // and at the far end of the turn
    ] {
        assert!(standing(&built, at), "{at:?} is a corner of the blend");
    }

    for at in [
        Vector3::new(10.0, 10.0, 10.0), // V itself
        Vector3::new(10.0, 10.0, 9.0),  // where the bead's stop plane met the crease
        Vector3::new(11.0, 10.0, 10.0), // the void wall's own top corner
        Vector3::new(10.0, 11.0, 10.0), // and the step wall's
    ] {
        assert!(!standing(&built, at), "{at:?} is under the blend");
    }
}

/// The patch is the grid and nothing else: `(n + 1)` squared vertices on the blend's own torus.
/// Counted as VERTICES, because the outer rail's crossings of the cap's grid lines split rim
/// triangles without adding a vertex to the grid.
fn the_patch_is_the_grid_and_nothing_more(segments: i32) {
    let (built, _, _) = the_junction_built(segments);
    let mut on_the_patch: Vec<Vector3> = Vec::new();

    for face in built.faces.iter() {
        for vertex in [face.v0, face.v1, face.v2] {
            let at = Vector3::from_vector3_float(built.vertices[vertex as usize]);

            if on_the_torus(at) && !on_the_patch.iter().any(|&had| (had - at).length() < 1e-6) {
                on_the_patch.push(at);
            }
        }
    }

    assert_eq!(
        on_the_patch.len() as i32,
        (segments + 1) * (segments + 1),
        "the grid is the cut's ring at every station of the turn and nothing past it"
    );
}

#[test]
fn the_patch_is_the_grid_and_nothing_more_4() {
    the_patch_is_the_grid_and_nothing_more(4);
}

#[test]
fn the_patch_is_the_grid_and_nothing_more_8() {
    the_patch_is_the_grid_and_nothing_more(8);
}

/// And it is SMOOTH across its own rows and its four seams alike: no two neighbouring facets of
/// the rounded surface turn by more than one sampling step (and a fiftieth, second order in the
/// step: a chord and not a kink).
fn nothing_rounded_turns_by_more_than_one_step(segments: i32) {
    let (built, _, _) = the_junction_built(segments);
    let sharpest = the_sharpest_turn_between_rounded_facets(&built);

    assert!(
        sharpest < (90.0 / segments as f64) * 1.02,
        "one step of the sampling is {} degrees - instead {sharpest}",
        90.0 / segments as f64
    );
}

#[test]
fn nothing_rounded_turns_by_more_than_one_step_4() {
    nothing_rounded_turns_by_more_than_one_step(4);
}

#[test]
fn nothing_rounded_turns_by_more_than_one_step_8() {
    nothing_rounded_turns_by_more_than_one_step(8);
}

/// The grid's LAST column is the closing ring by reuse, and the reuse is measured: a rail of that
/// ring moved out of the bead's axis by a few rulers is refused.
#[test]
fn a_closing_rail_moved_out_of_the_axis_is_refused() {
    let (junction, _, _, _) = the_junction_read();
    let corner = &junction.first.mixed;
    let slack = 1e-3;
    let sweep = corner.turn_radians() / 3.0;

    // A rail of the leading ring, and the same rail carried the row's sweep - which is exactly what
    // the closing ring is reused as.
    let leading = corner.outer_rail_point(0.0);
    let turned = corner.turned_about(leading, sweep);

    let (stands, turned_to) = surface_mixed_junction_patch::the_closing_rail_stands(
        corner, leading, turned, sweep, slack,
    );
    assert!(
        stands,
        "the ring the turn itself makes is the one ring that cannot be refused"
    );
    assert!(
        (turned_to - turned).length() < 1e-9,
        "and a refusal is named where the turn puts the rail"
    );

    let out_of_the_axis = (turned - corner.axis().closest_point_to(turned)).get_normal();

    assert!(
        surface_mixed_junction_patch::the_closing_rail_stands(
            corner,
            leading,
            turned + (out_of_the_axis * slack * 0.5),
            sweep,
            slack
        )
        .0,
        "half the slack is what the mesh's own chords already carry"
    );
    assert!(
        !surface_mixed_junction_patch::the_closing_rail_stands(
            corner,
            leading,
            turned + (out_of_the_axis * slack * 4.0),
            sweep,
            slack
        )
        .0,
        "four times it is a ring that is not this turn's"
    );
    assert!(
        !surface_mixed_junction_patch::the_closing_rail_stands(
            corner,
            leading,
            turned + (corner.axis().direction * slack * 4.0),
            sweep,
            slack
        )
        .0,
        "and neither is one standing four slacks along the axis"
    );
}

/// Whether the published mesh has a vertex at `at`, on any face.
fn standing(built: &Mesh, at: Vector3) -> bool {
    built.faces.iter().any(|face| {
        [face.v0, face.v1, face.v2].iter().any(|&vertex| {
            (Vector3::from_vector3_float(built.vertices[vertex as usize]) - at).length() < 1e-5
        })
    })
}

/// Whether a point stands on the blend's own torus: a radius from the circle of radius `2r` about
/// the bead's axis in the plane the ball centres run in.
fn on_the_torus(at: Vector3) -> bool {
    let out_of_the_axis =
        (((at.x - 11.0) * (at.x - 11.0)) + ((at.y - 11.0) * (at.y - 11.0))).sqrt();
    let along_the_axis = at.z - (10.0 - RADIUS);

    ((((out_of_the_axis - (2.0 * RADIUS)) * (out_of_the_axis - (2.0 * RADIUS)))
        + (along_the_axis * along_the_axis))
        .sqrt()
        - RADIUS)
        .abs()
        < ON_THE_ROUND
}

/// Whether a face lies in one of the part's own three flat planes at the corner.
fn is_flat(built: &Mesh, face: &Face) -> bool {
    let normal = normal_of(built, face);

    normal.x.abs() > 0.99999 || normal.y.abs() > 0.99999 || normal.z.abs() > 0.99999
}

fn normal_of(built: &Mesh, face: &Face) -> Vector3 {
    let at = |vertex: i32| Vector3::from_vector3_float(built.vertices[vertex as usize]);
    let first = at(face.v0);

    (at(face.v1) - first)
        .cross(at(face.v2) - first)
        .get_normal()
}

/// The widest angle between two ROUNDED facets sharing an edge, in degrees - across the patch,
/// along each strip, and over the four seams between them alike. Rounded means every corner of
/// the facet stands on one of the four surfaces the corner is made of, and the facet is not one
/// of the part's own flat faces.
fn the_sharpest_turn_between_rounded_facets(built: &Mesh) -> f64 {
    let mut by_edge: HashMap<(i32, i32), Vec<Vector3>> = HashMap::new();

    for face in built.faces.iter() {
        if is_flat(built, face) || !is_rounded(built, face) {
            continue;
        }

        let normal = normal_of(built, face);

        for (first, second) in [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)] {
            let key = if first < second {
                (first, second)
            } else {
                (second, first)
            };

            by_edge.entry(key).or_default().push(normal);
        }
    }

    // Only the largest turn is read, so the map's order cannot reach the answer.
    let mut sharpest = 0.0_f64;

    for sharing in by_edge.values() {
        for one in 0..sharing.len() {
            for other in (one + 1)..sharing.len() {
                sharpest = sharpest.max(math_helper::radians_to_degrees(
                    sharing[one].dot(sharing[other]).clamp(-1.0, 1.0).acos(),
                ));
            }
        }
    }

    sharpest
}

/// Whether every corner of a face stands on one of the corner's four rounded surfaces.
fn is_rounded(built: &Mesh, face: &Face) -> bool {
    [face.v0, face.v1, face.v2].iter().all(|&vertex| {
        let at = Vector3::from_vector3_float(built.vertices[vertex as usize]);

        let on_a_cylinder =
            ((((at.x - 9.0) * (at.x - 9.0)) + ((at.z - 9.0) * (at.z - 9.0))).sqrt() - RADIUS).abs()
                < ON_THE_ROUND
                || ((((at.y - 9.0) * (at.y - 9.0)) + ((at.z - 9.0) * (at.z - 9.0))).sqrt()
                    - RADIUS)
                    .abs()
                    < ON_THE_ROUND
                || ((((at.x - 11.0) * (at.x - 11.0)) + ((at.y - 11.0) * (at.y - 11.0))).sqrt()
                    - RADIUS)
                    .abs()
                    < ON_THE_ROUND;

        on_a_cylinder || on_the_torus(at)
    })
}

/// The L with all three edges at the corner filleted, the junction the build read, and the cut's
/// strip at that sampling.
fn the_junction_built(segments: i32) -> (Mesh, SurfaceMixedJunction, SurfaceStrip) {
    let (junction, mesh, regions, plans) = the_junction_read();
    let cut_strip = strip_of(&junction.first.mixed.cut, segments);

    let built = surface_bevel_builder::try_build(
        &mesh,
        &mesh.get_mesh_edge_graph(),
        &regions,
        &plans,
        segments,
        &CancelToken::new(),
        solidmesh::bevel::surface::SurfaceBuildOptions {
            crossings: None,
            ran_off_over_a_stretch: None,
            lay_the_oblique_rows_anyway: false,
            planless_loops: &[],
            features: None,
            use_sections: crate::surface_test_build::use_sections(),
        },
    )
    .unwrap_or_else(|reason| {
        panic!("the build refused the junction at {segments} segments: {reason:?}")
    });

    (built.mesh, junction, cut_strip)
}

/// The junction as the pass reads it at the L's reflex corner, with nothing built yet.
fn the_junction_read() -> (
    SurfaceMixedJunction,
    Mesh,
    Arc<FaceRegionCache>,
    Vec<Rc<SurfaceBevelPlan>>,
) {
    let mesh = create_l_shaped_grid_prism();
    let (regions, features) = fixture::regions_of(&mesh);

    let plans = vec![
        plan_of(
            &regions,
            &features,
            fixture::the_convex_edge(&features),
            true,
        ),
        plan_of(
            &regions,
            &features,
            fixture::the_second_convex_edge(&features),
            true,
        ),
        plan_of(
            &regions,
            &features,
            fixture::the_concave_edge(&features),
            true,
        ),
    ];

    let junction = SurfaceMixedJunction::try_read(
        &plans,
        plans[2].edges[0].edge.from_vertex(),
        the_corner(),
        SurfaceVertexPool::near_enough_for(&mesh.vertices),
    )
    .unwrap_or_else(|reason| panic!("the junction would not read: {reason:?}"));

    (junction, mesh, regions, plans)
}
