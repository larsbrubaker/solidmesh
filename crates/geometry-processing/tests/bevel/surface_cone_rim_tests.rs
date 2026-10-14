//! A curved region that is NOT a cylinder. A truncated cone's
//! wall facets are each tilted about their own chord, so the per-facet tangent planes do not
//! coincide the way a cylinder's vertical ones do, and a construction that read a curved wall as
//! one plane would pass every cylinder test and fail here.
//!
//! **Why the creases still close.** Each facet is walked back to a line in its own plane at the
//! setback from its own rim chord; a cone of revolution is symmetric across every crease, so the
//! crease makes the same angle with both chords and the two tangent points are one point.
//!
//! **The closed form.** Slice the removed solid by horizontal planes: the mitre plane at a rim
//! vertex is the axial one through it, so the slice at depth `v` is the ring between two concentric
//! regular polygons - `n tan(pi / n) [(a + k v)^2 - (a - U)^2]` - and `U` is linear in `v` along
//! each chord of the tessellated profile, which makes Simpson's rule exact. The fixture helpers are
//! `pub` for `section_routed_accuracy_reading_tests.rs`. The parameter rows are a loop.

use std::sync::Arc;

use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, SurfaceBevelPlan, SurfaceBuildOptions, SurfaceLine,
};
use geometry_processing::bevel::{bevel_arc_chording, bevel_chain_linker};
use geometry_processing::topology::{FaceGroup, FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{math_helper, Vector2, Vector3};

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::topology_test_meshes::create_frustum;

pub const BOTTOM_RADIUS: f64 = 12.0;

pub const TOP_RADIUS: f64 = 8.0;

pub const HEIGHT: f64 = 10.0;

pub const SIDES: i32 = 32;

pub const MEASURED: f64 = 1e-4;

/// The whole top rim of a frustum, filleted. Closed, exactly the analytic solid, and a blend every
/// vertex of which stands a radius from the line the rolling ball's centre runs along FOR ITS OWN
/// EDGE - a different line for each of the 32 edges, tilted with its own facet.
#[test]
fn a_cones_top_rim_is_bevelled_to_its_own_tilted_facets() {
    for (round_profile, segments) in [(true, 8), (false, 1)] {
        let size = 1.5;

        let built = build(size, round_profile, segments);

        assert!(
            built.is_manifold(),
            "a bevel of a closed solid stays a closed solid"
        );
        assert_eq!(shell_count(&built), 1);
        let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
        assert_eq!(
            open, 0,
            "every edge is shared by two faces, and the first that is not is at {open_at:?}"
        );

        let profile = profile(size, round_profile, segments);
        let removed = frustum_volume() - signed_volume(&built);
        let expected = removed_volume(&profile);

        assert!(
            (removed - expected).abs() <= MEASURED,
            "the removed solid is the ring between the widening wall and the shrinking blend, \
             integrated down the profile: {removed} against {expected}"
        );

        if round_profile {
            // Only a round has rows BETWEEN its two tangent rings; a chamfer's blend is the one
            // chord between them, and where those two rings stand is what the volume already says.
            assert_every_blend_vertex_stands_a_radius_from_its_own_centre_line(
                &built, size, &profile,
            );
        }
    }
}

/// That the blend is a rolling ball's surface facet by facet. Each rim edge has its OWN centre
/// line, and every vertex of that edge's strip stands a radius from it.
pub fn assert_every_blend_vertex_stands_a_radius_from_its_own_centre_line(
    built: &Mesh,
    size: f64,
    profile: &[Vector2],
) {
    let top = HEIGHT / 2.0;
    let deepest = profile[profile.len() - 1].y;

    // The blend's own vertices: below the cap's tangent ring and above the wall's, and not on
    // either of the two planes those rings stand in.
    let blend: Vec<Vector3> = built
        .vertices
        .iter()
        .map(|&vertex| Vector3::from_vector3_float(vertex))
        .filter(|at| at.z < top - 1e-6 && at.z > top - deepest + 1e-6)
        .collect();

    assert!(
        !blend.is_empty(),
        "a chorded fillet has rows between its two tangent rings"
    );

    let lines = centre_lines(size);

    for at in blend {
        // How far the BEST line misses the radius by, not how near the nearest line is. The lines
        // are infinite, and on a cone they lean; which line a vertex belongs to is what the strip
        // knows and a measurement of the mesh does not, so the claim is that one is exactly right.
        let missed_by = lines
            .iter()
            .map(|line| (line.distance_to(at) - size).abs())
            .fold(f64::INFINITY, f64::min);

        assert!(
            missed_by.abs() <= 1e-5,
            "{at:?} is on the blend, so it stands a radius from the centre line of the edge it \
             belongs to"
        );
    }
}

/// The 32 lines the rolling ball's centre runs along, one per rim edge: parallel to that edge's own
/// chord, a radius in from its own facet's plane and a radius down from the cap. Read off the
/// SOURCE mesh's own rim.
pub fn centre_lines(size: f64) -> Vec<SurfaceLine> {
    let slope = wall_slope();

    // How far in from the rim chord the ball's centre stands, horizontally.
    let inward = (size * (1.0 + (slope * slope)).sqrt()) - (size * slope);

    rim_edges(&create_frustum(BOTTOM_RADIUS, TOP_RADIUS, HEIGHT, SIDES))
        .into_iter()
        .map(|edge| {
            let middle = (edge.start + edge.end) / 2.0;
            let outward = Vector3::new(middle.x, middle.y, 0.0).get_normal();

            SurfaceLine::new(
                middle - (outward * inward) - Vector3::new(0.0, 0.0, size),
                (edge.end - edge.start).get_normal(),
            )
        })
        .collect()
}

/// How fast the wall moves outward as the cone is walked down, measured square to a chord.
pub fn wall_slope() -> f64 {
    (BOTTOM_RADIUS - TOP_RADIUS) * (std::f64::consts::PI / SIDES as f64).cos() / HEIGHT
}

/// The removed section's outline in the frame square to one rim chord: X in from the chord line,
/// horizontally, and Y down from the cap. From the tangent point on the cap round to the tangent
/// point on the wall, chorded as the strip chords it (`bevel_arc_chording::segments_for`).
pub fn profile(size: f64, round_profile: bool, segments_per_quarter: i32) -> Vec<Vector2> {
    let slope = wall_slope();

    // The wall's own inward normal and its own downhill direction, in this frame.
    let wall_normal = Vector2::new(1.0, slope) / (1.0 + (slope * slope)).sqrt();
    let down_the_wall = Vector2::new(-slope, 1.0) / (1.0 + (slope * slope)).sqrt();

    if !round_profile {
        return vec![Vector2::new(size, 0.0), down_the_wall * size];
    }

    let centre = Vector2::new(
        (size * (1.0 + (slope * slope)).sqrt()) - (size * slope),
        size,
    );
    let on_the_cap = Vector2::new(centre.x, 0.0);
    let on_the_wall = centre - (wall_normal * size);

    let from = on_the_cap - centre;
    let sweep = from
        .get_normal()
        .dot((on_the_wall - centre).get_normal())
        .clamp(-1.0, 1.0)
        .acos();
    let chords = bevel_arc_chording::segments_for(
        math_helper::radians_to_degrees(sweep),
        segments_per_quarter,
    )
    .max(1);
    let mut chorded = Vec::with_capacity(chords as usize + 1);

    for i in 0..=chords {
        let turn = -sweep * i as f64 / chords as f64;

        chorded.push(
            centre
                + Vector2::new(
                    (from.x * turn.cos()) - (from.y * turn.sin()),
                    (from.x * turn.sin()) + (from.y * turn.cos()),
                ),
        );
    }

    chorded
}

/// The removed solid, as the integral down the profile of the ring between the wall and the blend.
/// Simpson's rule per chord, exact because the integrand is a quadratic in depth.
pub fn removed_volume(profile: &[Vector2]) -> f64 {
    let apothem = TOP_RADIUS * (std::f64::consts::PI / SIDES as f64).cos();
    let slope = wall_slope();
    let corner = SIDES as f64 * (std::f64::consts::PI / SIDES as f64).tan();

    let ring = |depth: f64, across: f64| {
        let outer = apothem + (slope * depth);
        let inner = apothem - across;

        corner * ((outer * outer) - (inner * inner))
    };

    let mut summed = 0.0;

    for i in 0..profile.len().saturating_sub(1) {
        let from = profile[i];
        let to = profile[i + 1];
        let middle = (from + to) / 2.0;

        summed += (to.y - from.y) / 6.0
            * (ring(from.y, from.x) + (4.0 * ring(middle.y, middle.x)) + ring(to.y, to.x));
    }

    summed
}

/// The frustum's own volume, as the prism formula on its two polygons.
pub fn frustum_volume() -> f64 {
    let shape = SIDES as f64 / 2.0 * (2.0 * std::f64::consts::PI / SIDES as f64).sin();

    HEIGHT / 3.0
        * shape
        * ((BOTTOM_RADIUS * BOTTOM_RADIUS)
            + (BOTTOM_RADIUS * TOP_RADIUS)
            + (TOP_RADIUS * TOP_RADIUS))
}

/// The frustum's whole top rim through the plan and the surface builder.
pub fn build(size: f64, round_profile: bool, segments_per_quarter: i32) -> Mesh {
    let cone = create_frustum(BOTTOM_RADIUS, TOP_RADIUS, HEIGHT, SIDES);
    let features = FeatureEdgeSet::create(&cone);
    let graph = cone.get_mesh_edge_graph();
    let shared = Arc::new(cone.clone());
    let regions = Arc::new(FaceRegionCache::new(
        shared.clone(),
        Some(graph.clone()),
        features.flat_angle_degrees(),
    ));
    let curve_regions = Arc::new(FaceRegionCache::new(
        shared,
        Some(graph.clone()),
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    ));

    let chains = bevel_chain_linker::link(&rim_edges(&cone));
    assert_eq!(chains.len(), 1, "the rim is one chain");
    let chain = &chains[0];

    assert!(
        chain.is_closed && chain.edges.len() == SIDES as usize,
        "the rim is {} edges and closed is {}",
        chain.edges.len(),
        chain.is_closed
    );

    let plan = SurfaceBevelPlan::try_create_with_curve_regions(
        chain,
        &regions,
        Some(&curve_regions),
        size,
        round_profile,
    )
    .unwrap_or_else(|reason| panic!("the plan refused the rim: {reason}"));

    surface_bevel_builder::try_build(
        &cone,
        &graph,
        &regions,
        &[plan],
        segments_per_quarter,
        &CancelToken::new(),
        SurfaceBuildOptions::default(),
    )
    .unwrap_or_else(|reason| panic!("the build refused the rim: {reason:?}"))
    .mesh
}

/// The feature edges of the frustum's top rim, which is a closed loop of them.
pub fn rim_edges(cone: &Mesh) -> Vec<FeatureEdge> {
    let features = FeatureEdgeSet::create(cone);
    let top = HEIGHT / 2.0;

    features
        .edges()
        .iter()
        .filter(|edge| (edge.start.z - top).abs() < 1e-6 && (edge.end.z - top).abs() < 1e-6)
        .copied()
        .collect()
}
