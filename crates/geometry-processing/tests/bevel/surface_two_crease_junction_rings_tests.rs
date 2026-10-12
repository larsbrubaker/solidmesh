//! Port of `SurfaceTwoCreaseJunctionRingsTests.cs`: the three end rings the two-crease junction of
//! `surface_two_crease_junction_reading_tests` is sewn to, and the grid between them
//! (`SurfaceTwoCreaseJunctionRings`). The CUT's ring is cut on the plane the canal's ball traces on
//! the cut's own tube; each BEAD's ring is square; and the grid is TRANSPOSED - the stations run
//! along the cut's ring and every row runs from the wall's contact to the cut's tube. Nothing here
//! is emitted: these are readings, driven directly.

use std::sync::Arc;

use geometry_processing::bevel::surface::{
    surface_face_walkback, SurfaceEdgePlan, SurfaceMixedCorner, SurfaceMixedCornerDualFrame,
    SurfaceMixedJunction, SurfaceStrip, SurfaceTwoCreaseJunctionRings, SurfaceVertexPool,
};
use geometry_processing::bevel::BevelAdvisoryCode;
use polygon_mesh::Mesh;
use vector_math::{math_helper, Vector3};

use crate::nose_in_notch_test_meshes;
use crate::surface_two_crease_junction_reading_tests::{
    assert_named_at_the_corner, the_corner, the_junction, the_plans_of_the_right_ridge_end,
};
use crate::surface_walkback_checks::regions_of;

/// The sampling the control was filleted at.
const SEGMENTS: i32 = 16;

const RADIUS: f64 = crate::surface_two_crease_junction_reading_tests::RADIUS;

/// Where the ball touches the nose's UPPER face - one end of the cut's ring.
fn the_upper_tangency() -> Vector3 {
    Vector3::new(10.784822, 7.496151, 11.664101)
}

/// Its mirror on the lower face, which is the other end.
fn the_lower_tangency() -> Vector3 {
    Vector3::new(10.784822, 7.496151, 8.335899)
}

/// The CUT's ring is cut on the CANAL's own plane and not on a square section: every rail of it
/// stands on the cut's tube and in that one leaning plane, and it reaches out to x = 12.5660 at
/// mid turn where a square section would have stood at x = 10.7848 the whole way.
#[test]
fn the_cuts_ring_is_cut_on_the_canals_own_leaning_plane() {
    let (rings, junction, _) = the_rings();
    let plane = rings.canal.cut_ring_plane();
    let axis = junction.first.mixed.cut_center_line();

    assert_eq!(
        rings.station_count(),
        21,
        "the cut's strip carries 21 rails over its 112.62 degree wedge at {SEGMENTS} segments a \
         quarter - instead {}",
        rings.station_count()
    );

    let mut worst_off_the_tube = 0.0f64;
    let mut worst_out_of_the_plane = 0.0f64;
    let mut furthest_out = 0.0f64;

    for &rail in &rings.cut_ring {
        worst_off_the_tube =
            worst_off_the_tube.max(((rail - axis.closest_point_to(rail)).length() - RADIUS).abs());
        worst_out_of_the_plane =
            worst_out_of_the_plane.max((rail - plane.point).dot(plane.inward).abs());
        furthest_out = furthest_out.max(rail.x);
    }

    assert!(
        worst_off_the_tube < 1e-6,
        "every rail stands a radius from the cut's own centre line - instead one is \
         {worst_off_the_tube:e} off it"
    );
    assert!(
        worst_out_of_the_plane < 1e-9,
        "and in the one plane the whole contact curve lies in - instead one is \
         {worst_out_of_the_plane:e} out of it"
    );

    // Both ends ON the two tangencies, which is why neither of the cut's faces owes it a band end.
    let last = rings.station_count() - 1;
    assert!(
        (rings.cut_ring[0] - the_upper_tangency()).length() < 1e-5,
        "the ring starts at the upper tangency - instead {}",
        rings.cut_ring[0]
    );
    assert!(
        (rings.cut_ring[last] - the_lower_tangency()).length() < 1e-5,
        "and ends at the lower one - instead {}",
        rings.cut_ring[last]
    );

    assert!(
        (furthest_out - 12.5660).abs() <= 1e-4,
        "and reaches x = 12.5660 at mid turn, where the square section it replaced stood at \
         10.7848 - instead {furthest_out:.4}"
    );

    // Mid turn is where the ring crosses the ridge's own plane, which is the canal's tube contact
    // rail. C#'s OrderBy(...).First() keeps the first of equal keys, as min_by does.
    let at_the_ridge = rings
        .cut_ring
        .iter()
        .copied()
        .min_by(|a, b| {
            (a.z - nose_in_notch_test_meshes::RIDGE_Z)
                .abs()
                .total_cmp(&(b.z - nose_in_notch_test_meshes::RIDGE_Z).abs())
        })
        .expect("a ring");

    assert!(
        (at_the_ridge - Vector3::new(12.5660, 6.6056, 10.0)).length() < 1e-4,
        "instead the rail at the ridge's plane is {at_the_ridge}"
    );
}

/// Each rail of that ring IS a station of the grid: the rail lies on the cut's tube, so the angle
/// it makes round the cut's centre line is that station's own turn, and the canal puts the tube's
/// contact back on the rail rather than near it.
#[test]
fn the_cuts_ring_rails_are_the_grids_own_stations() {
    let (rings, _, _) = the_rings();
    let last = rings.station_count() - 1;

    let first_degrees = math_helper::radians_to_degrees(rings.stations[0]);
    assert!(
        first_degrees.abs() <= 1e-9,
        "station zero is the upper bead's own end of the turn - instead {first_degrees:e}"
    );
    let last_degrees = math_helper::radians_to_degrees(rings.stations[last]);
    assert!(
        (last_degrees - 112.6199).abs() <= 1e-4,
        "and the last is the lower bead's - instead {last_degrees:.4}"
    );

    let mut narrowest = f64::MAX;
    let mut widest = 0.0f64;
    let mut worst_back = 0.0f64;

    for station in 0..rings.station_count() {
        if station > 0 {
            let step = math_helper::radians_to_degrees(
                rings.stations[station] - rings.stations[station - 1],
            );

            narrowest = narrowest.min(step);
            widest = widest.max(step);
        }

        // REUSE, not re-mint: the canal's own tube contact at that station is the rail itself.
        worst_back = worst_back.max(
            (rings.canal.cut_contact_at(rings.stations[station]) - rings.cut_ring[station])
                .length(),
        );
    }

    assert!(
        worst_back < 1e-9,
        "the canal puts the tube's contact back on the rail it was read from - instead one is \
         {worst_back:e} off it"
    );

    // The strip's rails are uniform in angle, so the stations are too - which is what lets the rows
    // be laid at the rails' own turns rather than at a sampling of this class's choosing.
    assert!(
        (narrowest - 5.631).abs() <= 1e-3,
        "instead the narrowest station step is {narrowest:.4}"
    );
    assert!(
        widest - narrowest < 1e-9,
        "and they are all one step - instead they range over {:e} degrees",
        widest - narrowest
    );
}

/// Each BEAD's ring is its own SQUARE section, centred on the canal's first ball at that end of the
/// turn - which is what leaves the bead's band stopping square, and what makes the two ends of this
/// patch the beads' own rings rather than curves near them.
#[test]
fn each_beads_ring_is_its_own_square_section_on_the_canals_first_ball() {
    let (rings, junction, _) = the_rings();
    let corner = the_corner();

    assert_eq!(
        rings.rails_per_row(),
        13,
        "each bead's strip carries 13 rails over its 66.91 degree wedge at {SEGMENTS} segments a \
         quarter - instead {}",
        rings.rails_per_row()
    );

    for (arm, ring, turn) in [
        (&junction.first, &rings.first_bead_ring, 0.0),
        (&junction.second, &rings.last_bead_ring, 0.0),
    ] {
        let dual = arm.dual.as_ref().expect("a dual");
        let centre = dual.spine.centre_at(turn);
        let along = SurfaceMixedCorner::away_from_the_corner(&arm.mixed.bead, corner);

        let mut worst_off_the_ball = 0.0f64;
        let mut worst_out_of_square = 0.0f64;

        for &rail in ring {
            worst_off_the_ball = worst_off_the_ball.max(((rail - centre).length() - RADIUS).abs());
            worst_out_of_square = worst_out_of_square.max((rail - centre).dot(along).abs());
        }

        assert!(
            worst_off_the_ball < 1e-9,
            "every rail stands a radius from the canal's first ball - instead one is \
             {worst_off_the_ball:e} off it"
        );
        assert!(
            worst_out_of_square < 1e-9,
            "and SQUARE across the bead, which is where that ball is tangent to it - instead one \
             is {worst_out_of_square:e} out"
        );

        // Read the way every row of the grid runs: the wall's contact first, the cut's tube last.
        assert!(
            (ring[0] - dual.wall_rail_at(turn)).length() < 1e-9,
            "rail zero rides the wall - instead {} against {}",
            ring[0],
            dual.wall_rail_at(turn)
        );
        assert!(
            (ring[ring.len() - 1] - dual.cut_contact_at(turn)).length() < 1e-9,
            "and the last rail stands on the cut's tube - instead {}",
            ring[ring.len() - 1]
        );
    }

    // The two of them are the two tangencies, which is the pair the cut's own ring ends on as well.
    let last = rings.rails_per_row() - 1;
    assert!((rings.first_bead_ring[last] - the_upper_tangency()).length() < 1e-5);
    assert!((rings.last_bead_ring[last] - the_lower_tangency()).length() < 1e-5);
}

/// All three rings end on the TWO vertices the walkback already pooled - one tangency per nose face
/// - so the patch is sewn to the faces rather than to points a weld away from them.
#[test]
fn the_rings_end_on_the_tangencies_the_walkback_already_pooled() {
    let (rings, _, mesh) = the_rings();
    let pooled = the_walkbacks_tangencies(&mesh);

    assert_eq!(
        pooled.len(),
        2,
        "the walkback leaves each nose face with ONE corner at the junction - instead {}",
        pooled.len()
    );

    let last_station = rings.station_count() - 1;
    let last_rail = rings.rails_per_row() - 1;

    for (name, at) in [
        ("the cut ring's first rail", rings.cut_ring[0]),
        ("the cut ring's last rail", rings.cut_ring[last_station]),
        (
            "the first bead ring's tube contact",
            rings.first_bead_ring[last_rail],
        ),
        (
            "the last bead ring's tube contact",
            rings.last_bead_ring[last_rail],
        ),
    ] {
        assert!(
            pooled.iter().any(|vertex| (*vertex - at).length() < 1e-5),
            "{name} lands on a tangency the walkback pooled - instead {at} stands off both of {}",
            pooled
                .iter()
                .map(|vertex| vertex.to_string())
                .collect::<Vec<_>>()
                .join(" and ")
        );
    }
}

/// Two creases sampled at DIFFERENT numbers of steps have no rows to pair, and are refused by name
/// rather than gridded against whichever is shorter.
#[test]
fn beads_sampled_differently_are_refused_by_name() {
    let (junction, _, _) = the_junction();

    let cut = the_strip_of(&junction.first.mixed.cut, SEGMENTS);
    let first = the_strip_of(&junction.first.mixed.bead, SEGMENTS);
    let coarser = the_strip_of(&junction.second.mixed.bead, SEGMENTS / 2);

    assert_ne!(
        coarser.rail_count(),
        first.rail_count(),
        "the coarser sampling really does give the second bead a different ring"
    );

    let refused = SurfaceTwoCreaseJunctionRings::try_read_strips(&junction, &cut, &first, &coarser)
        .err()
        .expect("beads sampled differently are refused");

    assert_eq!(
        refused.code(),
        BevelAdvisoryCode::SurfaceJunctionBeadsSampledDifferently,
        "instead {refused}"
    );
    assert_named_at_the_corner(&refused.args()[0]);
}

/// Every point of the grid is a point of the CANAL: a radius from the ball's centre at its own
/// station, and in the plane square to the spine there - the two conditions that say the patch IS
/// the rolling ball's surface rather than a lofting between its two ends.
///
/// The spine's tangent is taken ANALYTICALLY ([`the_spines_tangent_at`]) rather than by
/// difference: a central difference at 1e-6 leaves its own 3e-10 of second-order error, which
/// would be the whole of what a 1e-12 test measured.
#[test]
fn every_grid_point_stands_on_the_ball_at_its_own_station() {
    let (rings, _, _) = the_rings();

    let mut worst_off_the_ball = 0.0f64;
    let mut worst_out_of_the_plane = 0.0f64;

    for station in 0..rings.station_count() {
        let turn = rings.stations[station];
        let centre = rings.canal.spine.centre_at(turn);
        let along = the_spines_tangent_at(&rings.canal, turn);

        for rail in 0..rings.rails_per_row() {
            let from = rings.at(rail, station) - centre;

            worst_off_the_ball = worst_off_the_ball.max((from.length() - RADIUS).abs());
            worst_out_of_the_plane = worst_out_of_the_plane.max(from.dot(along).abs());
        }
    }

    assert!(
        worst_off_the_ball < 1e-12,
        "every grid point is a point of the ball at its station - instead one stands \
         {worst_off_the_ball:e} off it"
    );
    assert!(
        worst_out_of_the_plane < 1e-12,
        "and in the plane square to the spine, which is where a constant radius canal's ring lies \
         - instead one stands {worst_out_of_the_plane:e} out of it"
    );
}

/// The grid's four sides ARE the four curves it is sewn to, by reuse and not by nearness: row zero
/// on the wall's plane, the last row on the cut's tube and on the cut ring's own rails, column zero
/// the first bead's ring, and the closing column the second bead's - within the slack that bead's
/// own chording already has.
#[test]
fn the_grids_four_sides_are_the_curves_it_is_sewn_to() {
    let (rings, junction, mesh) = the_rings();
    let wall = junction.first.mixed.out_of_the_wall;
    let on_the_wall = the_corner();
    let axis = junction.first.mixed.cut_center_line();

    let mut worst_off_the_wall = 0.0f64;
    let mut worst_off_the_tube = 0.0f64;
    let mut worst_off_the_cut_ring = 0.0f64;

    for station in 0..rings.station_count() {
        worst_off_the_wall =
            worst_off_the_wall.max((rings.at(0, station) - on_the_wall).dot(wall).abs());

        let last = rings.at(rings.rails_per_row() - 1, station);

        worst_off_the_tube =
            worst_off_the_tube.max(((last - axis.closest_point_to(last)).length() - RADIUS).abs());
        worst_off_the_cut_ring =
            worst_off_the_cut_ring.max((last - rings.cut_ring[station]).length());
    }

    assert!(
        worst_off_the_wall < 1e-12,
        "row zero rides the wall both creases stand against - instead one point is \
         {worst_off_the_wall:e} off its plane"
    );
    assert!(
        worst_off_the_tube < 1e-12,
        "and the last row stands on the cut's own tube - instead one point is \
         {worst_off_the_tube:e} off it"
    );
    assert!(
        worst_off_the_cut_ring < 1e-12,
        "which is the cut's RING, rail for rail, so the patch reuses it - instead one is \
         {worst_off_the_cut_ring:e} away"
    );

    let mut worst_off_column_zero = 0.0f64;

    for rail in 0..rings.rails_per_row() {
        worst_off_column_zero =
            worst_off_column_zero.max((rings.at(rail, 0) - rings.first_bead_ring[rail]).length());
    }

    assert!(
        worst_off_column_zero < 1e-12,
        "column zero IS the first bead's own end ring - instead one rail is \
         {worst_off_column_zero:e} away"
    );

    // And the closing column, which is the one side the grid cannot simply be given: it is the
    // SECOND bead's ring, and it is checked rather than assumed.
    let slack = SurfaceTwoCreaseJunctionRings::closing_slack(
        &junction,
        &SurfaceVertexPool::new(&mesh.vertices),
    );

    let (stands, stands_off, by) = rings.the_closing_column_stands(slack);
    assert!(
        stands,
        "instead the closing column stands {by:e} off the canal at {stands_off}, against a slack \
         of {slack:e}"
    );

    // Both faces are flat here, so the slack really is the part's own ruler and nothing wider.
    let ruler = SurfaceVertexPool::new(&mesh.vertices).near_enough();
    assert!(
        (slack - ruler).abs() <= 1e-12,
        "nothing was rolled along a chorded face, so there is no chord sagitta - instead {slack:e}"
    );
    assert!(
        by < 1e-12,
        "and the column stands on the canal to machine precision, not merely inside the slack - \
         instead {by:e}"
    );
}

/// The rings and grid of the two-crease junction, read from the fixture's own strips, with the
/// junction and the mesh (C#'s two `out` parameters).
pub fn the_rings() -> (
    SurfaceTwoCreaseJunctionRings,
    SurfaceMixedJunction,
    Arc<Mesh>,
) {
    let (junction, mesh, _) = the_junction();

    let rings = SurfaceTwoCreaseJunctionRings::try_read_strips(
        &junction,
        &the_strip_of(&junction.first.mixed.cut, SEGMENTS),
        &the_strip_of(&junction.first.mixed.bead, SEGMENTS),
        &the_strip_of(&junction.second.mixed.bead, SEGMENTS),
    )
    .unwrap_or_else(|why| panic!("the rings refused: {why}"));

    (rings, junction, mesh)
}

fn the_strip_of(edge: &SurfaceEdgePlan, segments: i32) -> SurfaceStrip {
    SurfaceStrip::try_create(edge, segments)
        .unwrap_or_else(|why| panic!("the strip refused: {why}"))
}

/// The tangencies the walkback left on the nose's two faces - each the ONE vertex the two bands on
/// that face meet at, pooled once, which the rings have to land on.
fn the_walkbacks_tangencies(mesh: &Arc<Mesh>) -> Vec<Vector3> {
    let (regions, features) = regions_of(mesh);
    let regions = Arc::new(regions);
    let plans = the_plans_of_the_right_ridge_end(&features, &regions, RADIUS, None);

    let walked =
        surface_face_walkback::try_apply(mesh, &mesh.get_mesh_edge_graph(), &regions, &plans)
            .unwrap_or_else(|why| panic!("the walkback refused: {why}"));

    let mut found: Vec<i32> = Vec::new();

    for runs in walked.tangent_runs.values() {
        for run in runs {
            for vertex in [run[0], run[run.len() - 1]] {
                let at = walked.mesh.vertices[vertex as usize].as_vector3();

                if ((at - the_upper_tangency()).length() < 1e-5
                    || (at - the_lower_tangency()).length() < 1e-5)
                    && !found.contains(&vertex)
                {
                    found.push(vertex);
                }
            }
        }
    }

    found
        .iter()
        .map(|&vertex| walked.mesh.vertices[vertex as usize].as_vector3())
        .collect()
}

/// Which way the canal's spine runs at `turn`, from its own algebra: the centre is
/// `railCentre + a t(phi) + 2r e(phi)` and the slide `t` is what the plane condition fixes, so both
/// derivatives are closed forms of what the dual already publishes.
fn the_spines_tangent_at(canal: &SurfaceMixedCornerDualFrame, turn: f64) -> Vector3 {
    let out_of_the_axis = (canal.spine.towards_the_shared_face() * -turn.sin())
        + (canal.spine.towards_the_wall_square() * turn.cos());

    let slide = -2.0 * RADIUS * canal.out_of_the_wall.dot(out_of_the_axis)
        / canal.along_the_cut_out_of_the_wall;

    ((canal.axis.direction * slide) + (out_of_the_axis * (2.0 * RADIUS))).get_normal()
}
