//! The routes a request carries (`bevel::part::BevelRouting`): a library caller who leaves them at
//! their default gets MatterCAD's constructions, and the request is the whole of the choice - a
//! cube edge asked with the defaults is built by the level line, and the same request with the
//! level line taken out of its routing is built by the older construction instead.

use std::sync::Arc;

use geometry_processing::bevel::level_line::level_line_routing::{
    first_route, set_first_route, LEVEL_LINE_ROUTE,
};
use geometry_processing::bevel::part::{bevel_part, BevelPartRequest, BevelRouting};
use geometry_processing::bevel::sections::section_routing;
use geometry_processing::topology::bevel_edge_group_resolver::create_groups_for_all_edges;
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::CancelToken;

/// A 1 mm fillet of a 10 mm cube's first straight crease, with the default routes.
fn a_cube_edge() -> BevelPartRequest {
    let cube = create_cube(10.0, 10.0, 10.0);
    let groups = create_groups_for_all_edges(&cube, 20.0, None).expect("a cube has edges");
    BevelPartRequest::new(
        Some(Arc::new(cube)),
        vec![groups[0].clone()],
        1.0,
        4,
        true,
        20.0,
    )
}

/// What building `request` came to: the first level-line route it wrote, the last route the
/// legacy surface construction under it wrote, and whether its one row built.
struct Routed {
    level_line: Option<String>,
    surface: Option<String>,
    built: bool,
}

fn build(request: BevelPartRequest) -> Routed {
    set_first_route(None);
    section_routing::set_last_route(None);
    let outcome = bevel_part(request, &CancelToken::new(), None).expect("the build runs");
    Routed {
        level_line: first_route(),
        surface: section_routing::last_route(),
        built: outcome.group_results[0]
            .as_ref()
            .is_some_and(|row| !row.failed()),
    }
}

/// The default routes are the ones MatterCAD builds by: the level line, without the sections,
/// with corner holes closed by the hole filler and END FACE ends closed flat.
#[test]
fn the_default_routes_are_mattercads() {
    let mattercad = BevelRouting {
        level_line: true,
        sections: false,
        corner_hole_fill: true,
        flat_ends: true,
    };
    assert_eq!(BevelRouting::default(), mattercad);
    assert_eq!(BevelRouting::MATTERCAD, mattercad);
    assert_eq!(a_cube_edge().routing, mattercad);
}

/// A request made with `BevelPartRequest::new` builds a cube edge by the level line, with nothing
/// else asked of the caller.
#[test]
fn a_request_with_the_default_routes_takes_the_level_line() {
    let routed = build(a_cube_edge());
    assert!(routed.built, "the cube edge builds");
    assert_eq!(routed.level_line.as_deref(), Some(LEVEL_LINE_ROUTE));
    assert_eq!(
        routed.surface, None,
        "the older construction is never asked"
    );
}

/// The same request with the level line out of its routing is never offered to the level line:
/// the older surface construction builds it.
#[test]
fn a_request_without_the_level_line_takes_the_older_construction() {
    let request = BevelPartRequest {
        routing: BevelRouting {
            level_line: false,
            ..BevelRouting::default()
        },
        ..a_cube_edge()
    };
    let routed = build(request);
    assert!(routed.built, "the cube edge builds");
    assert_eq!(routed.level_line, None, "the level line is never asked");
    assert_eq!(routed.surface.as_deref(), Some("legacy: the flag is off"));
}
