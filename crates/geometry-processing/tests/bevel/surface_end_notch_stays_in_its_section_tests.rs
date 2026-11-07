//! The end notch cut into a cap that CARRIES
//! ON past the corner's own rim - a square prism with a tab stuck on one side, filleted on the
//! vertical edge at the far corner of that side.
//!
//! The notch is made by clipping the end cap's triangles by the half plane of each chord of the
//! profile, keeping the side away from the corner, and the argument that what those clips remove
//! between them is exactly the section holds only while the cap stays inside the corner's own two
//! rims. The chord at the tangent point leans a half segment off the rim line, so past the tangent
//! point its half plane dives out through the rim - and anything of the cap lying out there is
//! taken off as well. The tab is exactly that: cap on the far side of the `y = 0` rim. This is the
//! shape "Every Chamfer Works.mcx" fails on, reduced to a mesh that needs no boolean to build.

use std::f64::consts::PI;
use std::sync::Arc;

use crate::path_solids::extrude;
use crate::path_solids::Outline;
use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, SurfaceBevelPlan,
};
use geometry_processing::topology::FeatureEdgeSet;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{Vector2, Vector3};

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::surface_strip_tests::{chord_excess_area, wedge_area};
use crate::surface_walkback_checks::regions_of;

const RADIUS: f64 = 3.0;

const SEGMENTS: i32 = 8;

/// The prism's own volume: a 20 square and a 6 by 5 tab, 10 tall.
const PRISM_VOLUME: f64 = ((20.0 * 20.0) + (6.0 * 5.0)) * 10.0;

/// How near the closed form below the measured mesh has to come, in mm^3.
const MEASURED: f64 = 1e-4;

/// How tall both prisms are.
const HEIGHT: f64 = 10.0;

/// The fillet the coarse cap below is cut with - small enough that the lip it reaches over is
/// well outside its own reach.
const COARSE_RADIUS: f64 = 1.0;

/// The coarse prism's own volume: the hexagon A B C D G H, 10 tall.
const COARSE_PRISM_VOLUME: f64 = 413.0 * HEIGHT;

#[test]
fn a_fillet_whose_end_cap_carries_on_past_the_rim_leaves_that_part_of_it_alone() {
    let prism = tabbed_prism();
    let built = filleted_at_the_corner(&prism, RADIUS);

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
    assert_eq!(
        open, 0,
        "a notch that stays in its own section leaves the cap closed, and this one opened at \
         {open_at:?}"
    );
    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);

    // The whole of the answer in one number: the fillet takes its own section along its own 10 mm
    // of edge and nothing else. A notch running on past the rim would bite a slice out of the tab
    // as well, at both ends of the edge.
    let section = wedge_area(RADIUS, PI / 2.0) + chord_excess_area(RADIUS, PI / 2.0, SEGMENTS);

    let expected = PRISM_VOLUME - (10.0 * section);
    let measured = signed_volume(&built);
    assert!(
        (measured - expected).abs() <= MEASURED,
        "{measured}, expected {expected}"
    );

    // And said where it matters: the tab is untouched, so nothing of the built mesh stands inside
    // it that was not a corner of it to begin with.
    let into_the_tab: Vec<Vector3> = built
        .vertices
        .iter()
        .map(|&vertex| Vector3::from_vector3_float(vertex))
        .filter(|at| at.y < -1e-9 && (at.y + 5.0).abs() > 1e-9)
        .collect();

    assert!(
        into_the_tab.is_empty(),
        "the notch is 3 mm from the corner at (0, 0) and the tab starts 10 mm away: {into_the_tab:?}"
    );
}

/// The same trouble on a COARSE cap, where one triangle of it reaches from beside the corner all
/// the way out past the lip and owns a piece of rim out there.
///
/// Two things have to hold at once here. The triangle `H C D` stands within the corner's own box -
/// `H` is on the `x = 0` rim - so the notch does not turn it away, and it carries material on BOTH
/// sides of the `y = 0` rim line: without the wedge, the chords take the part below that line off
/// it, which is a hole in the middle of the cap. And with the wedge, the side's own line crosses
/// that triangle's rim edge `C D` at `(17.33, 0)` - 17 mm from a notch 2 mm across - while the wall
/// on the other side of that edge is carried over whole. The vertex left standing there is a T
/// junction unless the notch sews its own splits back into whatever face carries one.
#[test]
fn a_coarse_cap_reaching_past_the_lip_is_neither_eaten_into_nor_left_with_a_t_junction() {
    let prism = coarse_lipped_prism();
    let built = filleted_at_the_corner(&prism, COARSE_RADIUS);

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
    assert_eq!(
        open, 0,
        "neither the notch nor its own splits may leave the cap open, and this one opened at \
         {open_at:?}"
    );
    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);

    let section =
        wedge_area(COARSE_RADIUS, PI / 2.0) + chord_excess_area(COARSE_RADIUS, PI / 2.0, SEGMENTS);

    let expected = COARSE_PRISM_VOLUME - (10.0 * section);
    let measured = signed_volume(&built);
    assert!(
        (measured - expected).abs() <= MEASURED,
        "the fillet takes its own section along its own 10 mm and nothing out over the lip - \
         {measured}, expected {expected}"
    );
}

/// The corner edge at (0, 0), planned and built at `radius`.
fn filleted_at_the_corner(prism: &Mesh, radius: f64) -> Mesh {
    let (regions, features) = regions_of(&Arc::new(prism.clone()));
    let regions = Arc::new(regions);

    let chains = bevel_chain_linker::link(&[features.edges()[the_corner_edge(&features)]]);
    assert_eq!(chains.len(), 1);

    let plan = SurfaceBevelPlan::try_create(&chains[0], &regions, radius, true)
        .unwrap_or_else(|reason| panic!("{:?}", reason.code()));

    surface_bevel_builder::try_build_one(
        prism,
        &prism.get_mesh_edge_graph(),
        &regions,
        plan,
        SEGMENTS,
        &CancelToken::new(),
    )
    .unwrap_or_else(|reason| panic!("{reason:?}"))
}

/// A prism on a hexagon that dips below the `y = 0` rim line past `x = 10` and climbs back over
/// it at `(17.33, 0)`, with its caps triangulated by hand: `A B H`, `B C H`, `H C D` and `H D G`.
/// It is the cap TRIANGULATION that this fixture is for, so it is built face by face rather than
/// extruded - Clipper would choose its own.
fn coarse_lipped_prism() -> Mesh {
    let outline = [
        Vector2::new(0.0, 0.0),   // A, the corner the fillet is on
        Vector2::new(10.0, 0.0),  // B, where the cap dips below its own rim line
        Vector2::new(14.0, -5.0), // C
        Vector2::new(20.0, 4.0),  // D, back over the line at (17.33, 0)
        Vector2::new(20.0, 20.0), // G
        Vector2::new(0.0, 20.0),  // H
    ];

    let cap = [(0, 1, 5), (1, 2, 5), (5, 2, 3), (5, 3, 4)];

    let mut mesh = Mesh::new();

    for (first, second, third) in cap {
        mesh.create_face(&[
            Vector3::from_vector2(outline[first], HEIGHT),
            Vector3::from_vector2(outline[second], HEIGHT),
            Vector3::from_vector2(outline[third], HEIGHT),
        ]);

        mesh.create_face(&[
            Vector3::from_vector2(outline[first], 0.0),
            Vector3::from_vector2(outline[third], 0.0),
            Vector3::from_vector2(outline[second], 0.0),
        ]);
    }

    for i in 0..outline.len() {
        let from = outline[i];
        let to = outline[(i + 1) % outline.len()];

        mesh.create_face(&[
            Vector3::from_vector2(from, 0.0),
            Vector3::from_vector2(to, 0.0),
            Vector3::from_vector2(to, HEIGHT),
            Vector3::from_vector2(from, HEIGHT),
        ]);
    }

    // create_face adds fresh vertices every call, so what is built above is triangle soup.
    mesh.clean_and_merge();

    mesh
}

/// A 20 square prism with a 6 by 5 tab on its `y = 0` side, 10 tall. The tab is what makes the
/// cap carry on past the rim the corner at `(0, 0)` stands on.
fn tabbed_prism() -> Mesh {
    let mut path = Outline::new();

    path.move_to(0.0, 0.0);
    path.line_to(10.0, 0.0);
    path.line_to(10.0, -5.0);
    path.line_to(16.0, -5.0);
    path.line_to(16.0, 0.0);
    path.line_to(20.0, 0.0);
    path.line_to(20.0, 20.0);
    path.line_to(0.0, 20.0);
    path.close_polygon();

    extrude(&path, 10.0).expect("the outline extrudes")
}

/// The convex vertical edge at `(0, 0)` - the corner the tab's rim line runs out from.
fn the_corner_edge(features: &FeatureEdgeSet) -> usize {
    let found: Vec<usize> = (0..features.count())
        .filter(|&index| {
            let edge = features.edges()[index];

            edge.start.x.abs() < 1e-9
                && edge.end.x.abs() < 1e-9
                && edge.start.y.abs() < 1e-9
                && edge.end.y.abs() < 1e-9
        })
        .collect();

    assert_eq!(found.len(), 1, "one edge stands at the corner");
    found[0]
}
