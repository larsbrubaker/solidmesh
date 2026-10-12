//! Port of `SurfaceMixedCornerWhereTheWallRunsOutTests.cs`: the mixed corner whose runout RUNS OFF
//! THE END of the wall it dies into - the lobed prism with its right-hand side brought in so only
//! 8 mm of flat wall stands past the seam where the runout asks for 9.7
//! (`lobed_prism_meshes::create_with_a_short_wall`).
//!
//! Why 9.7: the ball's touch point runs `2r` out of the bead's axis and the axis stands a radius
//! off the wall, so a flat wall is cleared sixty degrees past the bead whatever the wedge, at
//! `2r sin 60` along the wall from the bead's own foot. "Every Chamfer Works.mcx" has exactly this.
//! The blend is TRIMMED by the plane of the face across the wall's own edge, and that face gives
//! up the section (`surface_mixed_corner_wall_ends`). The rim reading is the one
//! `surface_mixed_corner_section_stands_on_the_plane_tests.rs` shares. C# `[Arguments]` rows are a
//! loop.

use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, surface_mixed_corner_wall_ends, SurfaceBandEnd,
    SurfaceBevelPlan, SurfaceMixedCorner, SurfaceMixedCornerWall,
};
use geometry_processing::topology::FaceRegionCache;
use polygon_mesh::{CancelToken, Mesh};
use std::rc::Rc;
use std::sync::Arc;
use vector_math::math_helper::radians_to_degrees;
use vector_math::Vector3;

use crate::lobed_prism_meshes;
use crate::mesh_measurements::{shell_count, signed_volume};
use crate::surface_mixed_corner_section_stands_on_the_plane_tests::{
    a_facet_of_the_lobes_wall, the_walls_rim,
};

/// The fixture's own fillet radius, which is what "Every Chamfer Works.mcx" is filleted at.
const RADIUS: f64 = 3.756;

/// How near a published vertex has to stand to be read as ON a face of the part: the mesh keeps
/// its vertices as floats, which at this part's own size is a couple of millionths.
const ON_THE_FACE: f64 = 1e-4;

type Read = (
    Mesh,
    Arc<FaceRegionCache>,
    Rc<SurfaceBevelPlan>,
    Rc<SurfaceMixedCorner>,
);

/// Where the rectangle's right-hand side stands, which is the plane the blend is trimmed by. To a
/// THOUSANDTH, because the outline goes through Clipper's own integer grid on its way to a prism
/// and comes back on it: the side is asked for at 37.772292 and the part carries it at 37.772.
fn the_far_side() -> f64 {
    let at = lobed_prism_meshes::right_crossing().x + lobed_prism_meshes::SHORT_WALL_LENGTH;
    // C# Math.Round(x, 3): banker's rounding at the third place.
    (at * 1000.0).round_ties_even() / 1000.0
}

fn assert_within(actual: f64, expected: f64, tolerance: f64, because: &str) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} is not {expected} within {tolerance}: {because}"
    );
}

/// The wall really does run out under the blend: the rail meets it 1.2 mm past the end of the part.
#[test]
fn the_runout_asks_for_more_wall_than_the_part_has() {
    let (_, _, _, corner) = the_walls_rim(RADIUS);

    assert_within(
        corner.corner.x,
        lobed_prism_meshes::right_crossing().x,
        1e-3,
        &format!(
            "the corner stands at the seam - instead {:?}",
            corner.corner
        ),
    );

    let wall = SurfaceMixedCornerWall::try_read(&corner)
        .unwrap_or_else(|refused| panic!("the wall has to read: {:?}", refused.code()));
    let meets = wall
        .try_where_the_rail_meets_the_wall()
        .expect("the wall has to read");

    let dies = corner.outer_rail_point(meets);

    assert_within(
        radians_to_degrees(meets - corner.turn_radians()),
        60.0,
        1e-6,
        "a flat wall is cleared sixty degrees past the bead, whatever the wedge",
    );

    assert_within(
        dies.x - the_far_side(),
        1.2054,
        1e-3,
        &format!("which is that far past the end of the wall - instead {dies:?}"),
    );
}

/// And the rim BUILDS: closed, manifold, one shell, and taking less off the finer it is chorded.
///
/// The eight and sixteen station readings went up by five thousandths of a cubic millimetre when
/// the crossing turn stopped being solved in a leaning frame
/// (`surface_mixed_corner_section_stands_on_the_plane_tests.rs`).
#[test]
fn the_rim_closes_with_the_blend_trimmed_by_the_face_across() {
    for (segments, takes_off, coarser) in [
        (4, 182.10557, f64::MAX),
        (8, 175.32241, 182.10557),
        (16, 173.56762, 175.32241),
    ] {
        let (built, prism) = the_rim_built(segments);

        let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
        assert_eq!(open, 0, "open at {open_at:?} ({segments} segments)");
        assert!(built.is_manifold(), "{segments} segments");
        assert_eq!(shell_count(&built), 1, "{segments} segments");

        let took = signed_volume(&prism) - signed_volume(&built);

        assert_within(took, takes_off, 1e-3, &format!("instead {took}"));
        assert!(
            took < coarser,
            "at {segments} segments it took {took}, and half the stations took {coarser}"
        );
    }
}

/// What the face across gives up is the blend's own SECTION: every vertex the part carries in that
/// face's plane over the blend's own turn stands on the ideal torus about the bead's axis, exactly
/// - they are rows of the surface carried round to their own crossings, not points stepped to.
#[test]
fn the_section_in_the_face_across_stands_on_the_ideal_torus() {
    let read = the_walls_rim(RADIUS);
    let corner = &read.3;
    let (built, _) = the_rim_built(8);

    let mut on_the_section = Vec::new();
    let mut off_the_torus = Vec::new();

    for stored in &built.vertices {
        let at = Vector3::from_vector3_float(*stored);

        if (at.x - the_far_side()).abs() > ON_THE_FACE
            || at.z < lobed_prism_meshes::HEIGHT - RADIUS
            || at.y < ON_THE_FACE
            || at.y > 2.0 * RADIUS
        {
            // Not in the face across, or past the ball's own reach of 2r out of the part's corner,
            // where nothing of the blend can stand. Everything left is asked to be ON the torus or
            // on a radius out of the part's corner - nothing is dropped for being far off it.
            continue;
        }

        let out_of_the_axis = (at - corner.axis().closest_point_to(at)).length();
        let under_the_cap = (corner.outer_rail_centre - at).dot(corner.out_of_the_cap_face);
        let from_the_centre_line = ((out_of_the_axis - corner.outer_rail_radius()).powi(2)
            + (under_the_cap - RADIUS).powi(2))
        .sqrt();

        let off = (from_the_centre_line - RADIUS).abs();

        if off < ON_THE_FACE {
            on_the_section.push(at);
        } else {
            off_the_torus.push(at);
        }
    }

    // Nothing standing in that corner of the face across is off the torus except on a RADIUS out
    // of the part's own corner, which is where the bite's own straight sides run and where the
    // crossings they leave stand. A vertex that is neither fails, which is what this is here to
    // catch.
    let at_the_corner = the_parts_own_corner(&read);

    for at in &off_the_torus {
        let on_a_radius = on_the_section.iter().any(|vertex| {
            (*at - at_the_corner)
                .get_normal()
                .cross((*vertex - at_the_corner).get_normal())
                .length()
                < 1e-4
        });

        assert!(
            on_a_radius,
            "{at:?} stands off the torus and off the radii out of {at_the_corner:?}"
        );
    }

    assert!(
        on_the_section.len() >= 2,
        "the section has to be there to measure - instead {} vertices",
        on_the_section.len()
    );

    // And the foot it ends on, where the trim curve crossed that plane - on the wall, so the
    // section closes against the part's own edge rather than in mid air.
    let foot = built.vertices.iter().any(|stored| {
        (f64::from(stored.x) - the_far_side()).abs() < ON_THE_FACE
            && f64::from(stored.y).abs() < ON_THE_FACE
            && (f64::from(stored.z) - 19.86).abs() < 1e-2
    });

    assert!(
        foot,
        "the section's own foot stands on the wall at z = 19.86"
    );
}

/// And the part's own far edge is left alone where the blend does not reach it: the vertical edge
/// the wall and the face across meet in carries the two blends' feet and the two band stops, and
/// nothing else. Its own two end corners ARE eaten, standing as they do inside the ball's reach.
#[test]
fn the_far_edge_stays_sharp_between_the_two_blends() {
    let (built, _) = the_rim_built(8);

    let mut on_the_edge = Vec::new();

    for stored in &built.vertices {
        let at = Vector3::from_vector3_float(*stored);

        if (at.x - the_far_side()).abs() < ON_THE_FACE && at.y.abs() < ON_THE_FACE {
            on_the_edge.push(at.z);
        }
    }

    on_the_edge.sort_by(f64::total_cmp);

    // The two blends' feet, and the two places the beads' own bands stop - and nothing between.
    let read = on_the_edge
        .iter()
        .map(|at| format!("{at:.3}"))
        .collect::<Vec<_>>()
        .join(", ");

    assert_eq!(
        read, "0.140, 3.756, 16.244, 19.860",
        "instead {on_the_edge:?}"
    );
}

/// A row that is ALREADY past the plane where the runout starts never leaves the material at a turn
/// of its own: the first turn its circle meets that plane at is where it comes back IN.
#[test]
fn a_row_already_past_the_plane_has_no_turn_it_leaves_at() {
    let read = the_walls_rim(RADIUS);
    let corner = &read.3;
    let across = the_plane_the_wall_ends_on(&read);

    let rail = corner.outer_rail_point(corner.turn_radians());
    let leaves = surface_mixed_corner_wall_ends::where_it_crosses(corner, rail, across);

    assert!(
        leaves != f64::INFINITY,
        "the outer rail's own row does leave the material, which is what the trim is"
    );

    // The same row carried a twentieth of a radian past its own crossing, which is a point
    // standing where the part has nothing.
    let past_it = corner.turned_about(rail, leaves + 0.05);

    assert!(
        across.signed_distance(past_it) < 0.0,
        "it has to really be past the plane - instead {}",
        across.signed_distance(past_it)
    );

    assert_eq!(
        surface_mixed_corner_wall_ends::where_it_crosses(corner, past_it, across),
        f64::INFINITY,
        "a row out there has no turn it leaves at, and coming back in is not one"
    );
}

/// And a row standing ON that plane where the runout starts crosses at NOUGHT: the turn it crosses
/// at is a turn like any other, not the absence of one.
#[test]
fn a_row_on_the_plane_at_the_seed_crosses_there() {
    let read = the_walls_rim(RADIUS);
    let across = the_plane_the_wall_ends_on(&read);

    let turn = surface_mixed_corner_wall_ends::try_turn_onto(&read.3, across.point, across, 0.0);

    let turn = turn.expect(
        "the point the travel line crosses the rim at stands on the plane, so it crosses at nought",
    );

    assert_within(turn, 0.0, 1e-9, &format!("instead {turn}"));
}

/// What the trim is taken in has to BE one plane: the flat side the wall runs out into is, and the
/// lobe's own barrel - a region of the same part the runout could as easily have met - is not.
#[test]
fn only_a_region_that_is_all_one_plane_can_give_up_the_section() {
    let read = the_walls_rim(RADIUS);
    let across = the_plane_the_wall_ends_on(&read);
    let (prism, regions) = (&read.0, &read.1);

    assert_within(
        across.point.x,
        the_far_side(),
        1e-3,
        &format!(
            "the wall runs out into the rectangle's far side - instead {:?}",
            across.point
        ),
    );

    let far_side = regions.region(a_facet_of_the_far_side(prism));
    assert!(
        surface_mixed_corner_wall_ends::it_is_all_one_plane(
            Some(regions),
            far_side.as_deref(),
            Vector3::UNIT_X
        ),
        "the face across is flat, and is the whole of what stands there"
    );

    let barrel = regions.region(a_facet_of_the_lobes_wall(prism));
    assert!(
        !surface_mixed_corner_wall_ends::it_is_all_one_plane(
            Some(regions),
            barrel.as_deref(),
            Vector3::UNIT_X
        ),
        "the barrel bends away past its first facet, and a plane through that facet is not it"
    );
}

/// The part's own corner the section is fanned out of - where the cap, the wall and the face across
/// meet - read exactly as the bite reads it.
fn the_parts_own_corner(read: &Read) -> Vector3 {
    let across = the_plane_the_wall_ends_on(read);
    let corner = &read.3;

    surface_mixed_corner_wall_ends::try_where_the_three_meet(
        corner.outer_rail_centre,
        corner.out_of_the_cap_face,
        corner.corner,
        corner.out_of_the_wall,
        across.point,
        -across.inward,
    )
    .expect("the three planes have to meet")
}

/// The plane the wall runs out on, read as the build reads it.
fn the_plane_the_wall_ends_on(read: &Read) -> SurfaceBandEnd {
    surface_mixed_corner_wall_ends::try_read(Some(&read.1), &read.3)
        .unwrap_or_else(|why| panic!("the wall has to run out here: {:?}", why.code()))
        .expect("the wall has to run out here")
}

/// Any one facet of the rectangle's far side - the flat face the wall runs out into.
fn a_facet_of_the_far_side(prism: &Mesh) -> i32 {
    for (face, facet) in prism.faces.iter().enumerate() {
        let middle = (Vector3::from_vector3_float(prism.vertices[facet.v0 as usize])
            + Vector3::from_vector3_float(prism.vertices[facet.v1 as usize])
            + Vector3::from_vector3_float(prism.vertices[facet.v2 as usize]))
            / 3.0;

        if (middle.x - the_far_side()).abs() < 1e-3
            && middle.z > 1.0
            && middle.z < lobed_prism_meshes::HEIGHT - 1.0
        {
            return face as i32;
        }
    }

    panic!("the prism has lost its own far side");
}

/// The wall region's rim, planned whole and built at `segments` stations, with the prism it was
/// built on.
fn the_rim_built(segments: i32) -> (Mesh, Mesh) {
    let (prism, regions, plan, _) = the_walls_rim(RADIUS);

    let built = surface_bevel_builder::try_build_one(
        &prism,
        &prism.get_mesh_edge_graph(),
        &regions,
        plan,
        segments,
        &CancelToken::new(),
    )
    .unwrap_or_else(|reason| panic!("the rim has to build: {reason:?}"));

    (built, prism)
}
