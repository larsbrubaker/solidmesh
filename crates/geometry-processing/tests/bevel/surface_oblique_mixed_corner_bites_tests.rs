//! Port of `SurfaceObliqueMixedCornerBitesTests`: what the two faces nobody picked give up at the
//! OBLIQUE mixed corner of the nose-in-notch part, and the one fact about that fixture that says
//! how far the ladder can climb on it - the CAP FACE RUNS OUT under the blend, a third of the way
//! through the turn. Both tests are measurements of the fixture, not of a wish.

use geometry_processing::bevel::surface::surface_mixed_corner_patch::try_cut_the_ring;
use geometry_processing::bevel::surface::surface_mixed_corner_sector::try_apex;
use geometry_processing::bevel::surface::{
    SurfaceMixedCorner, SurfaceMixedCornerRows, SurfaceStrip,
};
use geometry_processing::topology::{EdgeConvexity, FeatureEdgeSet};
use polygon_mesh::Mesh;
use vector_math::Vector3;

use crate::nose_in_notch_test_meshes;
use crate::surface_oblique_mixed_corner_reading_tests::the_oblique_corner;

const SEGMENTS: i32 = 8;

/// The cap's bite is the convex bite on the ELLIPSE ARC, on the patch's own pooled outer rail, with
/// its apex where the cut's stopping plane crosses the closing radius.
#[test]
fn the_caps_bite_stands_on_the_patches_own_outer_rail() {
    let corner = the_oblique_corner();
    let rail = the_outer_rail_of(&corner);

    // The rim IS where the ball touches the cap at each station, which is what the patch's rows
    // lay as their first rail.
    assert!(
        rail[0].equals_within(Vector3::new(7.1076, 6.2481, 9.1679), 5e-5),
        "instead {:?}",
        rail[0]
    );
    let last = rail[rail.len() - 1];
    assert!(
        last.equals_within(Vector3::new(4.2929, 5.1524, 9.8984), 5e-5),
        "instead {last:?}"
    );

    let apex = try_apex(&corner, rail[0], last)
        .unwrap_or_else(|why| panic!("the sector's apex reads - instead {why:?}"));

    assert!(
        apex.equals_within(Vector3::new(7.1076, 2.7210, 11.5193), 5e-5),
        "the apex is where the cut's own stopping plane crosses the closing radius, well off the axis at (5.5128, 4.0986, 10.6009) - instead {apex:?}"
    );

    // Convex, every corner of it turning the same way in the cap's plane.
    let mut polygon = vec![apex];
    polygon.extend(rail.iter().copied());

    for at in 0..polygon.len() {
        let a = polygon[at];
        let b = polygon[(at + 1) % polygon.len()];
        let c = polygon[(at + 2) % polygon.len()];

        assert!(
            (b - a).cross(c - b).dot(corner.out_of_the_cap_face) < 0.0,
            "the sector turns the same way at every corner - at {at} it turns back"
        );
    }
}

/// The fixture's cap face RUNS OUT under the blend: the ball's contact with it crosses the lower
/// concave crease at turn 21.02 degrees and is buried under the left slope's own face from there
/// on.
#[test]
fn the_cap_face_runs_out_under_the_blend_at_the_lower_crease() {
    let mesh = nose_in_notch_test_meshes::create();
    let features = FeatureEdgeSet::create(&mesh);
    let left = nose_in_notch_test_meshes::left_ridge_end();
    let lower_end = Vector3::new(10.0, 10.0, 20.0 / 3.0);

    let lower: Vec<usize> = (0..features.count())
        .filter(|&i| {
            let edge = &features.edges()[i];
            ((edge.start - left).length() < 1e-6 || (edge.end - left).length() < 1e-6)
                && ((edge.start - lower_end).length() < 1e-6
                    || (edge.end - lower_end).length() < 1e-6)
        })
        .collect();
    assert_eq!(lower.len(), 1);

    assert_eq!(
        features.edges()[lower[0]].convexity,
        EdgeConvexity::Concave,
        "the cap face and the wall meet in a valley, so past that crease the wall's material stands over the cap's own plane"
    );

    let corner = the_oblique_corner();
    let slope = nose_in_notch_test_meshes::left_slope_normal();

    // Where the cap contact crosses it: the signed distance to the wall's plane, which is zero on
    // the crease and negative under the wall's material.
    let under_the_wall = |turn: f64| (corner.spine.t_contact_at(turn) - left).dot(slope);

    let mut below = 0.0;
    let mut above = corner.turn_radians();

    for _ in 0..60 {
        let middle = (below + above) / 2.0;

        if under_the_wall(middle) > 0.0 {
            below = middle;
        } else {
            above = middle;
        }
    }

    let crosses = ((below + above) / 2.0).to_degrees();

    assert!(
        (crosses - 21.0228).abs() <= 1e-3,
        "the cap contact leaves the cap face there - instead {crosses:.4}"
    );
    let at_the_end = under_the_wall(corner.turn_radians());
    assert!(
        (at_the_end - -0.6078).abs() <= 1e-4,
        "and stands that far under the wall by the end of the turn - instead {at_the_end:.4}"
    );

    // No face of the part carries it past the crossing, which is what "runs out" means.
    let stations = the_station_count_of(&corner);

    for station in 0..stations {
        let turn = station as f64 * corner.turn_radians() / (stations - 1) as f64;
        let on = stands_on_a_face_of(&mesh, corner.spine.t_contact_at(turn));

        assert_eq!(
            on,
            turn.to_degrees() < crosses,
            "station {station} at {:.2} degrees stands {} the part",
            turn.to_degrees(),
            if on { "on" } else { "off" }
        );
    }

    // And every row past it crosses the wall's plane, so the blend's footprint on the wall is
    // bounded by a curve cut in that plane and not by any rail of the patch.
    let (rows, ring) = the_rows_of(&corner);
    let mut crossing = 0;

    for station in 0..stations {
        let turn = station as f64 * corner.turn_radians() / (stations - 1) as f64;
        let mut deepest = f64::MAX;
        let mut highest = f64::MIN;

        for rail in 0..ring.len() {
            let under = (rows.at(rail, turn) - left).dot(slope);

            deepest = deepest.min(under);
            highest = highest.max(under);
        }

        if deepest < -1e-9 && highest > 1e-9 {
            crossing += 1;
        }
    }

    assert_eq!(
        crossing, 4,
        "four of the seven stations run from inside the wall to outside it - instead {crossing}"
    );

    // And the fifth, the last of the turn, has come all the way down: its whole row stands inside
    // the wall's plane and touches it at the bead contact, which is the tangency the turn ends on.
    let mut last_rows_highest = f64::MIN;

    for rail in 0..ring.len() {
        last_rows_highest =
            last_rows_highest.max((rows.at(rail, corner.turn_radians()) - left).dot(slope));
    }

    assert!(
        last_rows_highest.abs() <= 1e-9,
        "the last row touches the wall rather than crossing it - instead its highest point stands {last_rows_highest:e} off"
    );
}

/// The patch's outer rail - where the ball touches the cap at each station of the turn.
fn the_outer_rail_of(corner: &SurfaceMixedCorner) -> Vec<Vector3> {
    let stations = the_station_count_of(corner);

    (0..stations)
        .map(|station| {
            corner
                .spine
                .t_contact_at(station as f64 * corner.turn_radians() / (stations - 1) as f64)
        })
        .collect()
}

/// How many stations the patch's grid has - one per rail of the bead's own end ring.
pub fn the_station_count_of(corner: &SurfaceMixedCorner) -> i32 {
    let bead = SurfaceStrip::try_create(&corner.bead, SEGMENTS)
        .unwrap_or_else(|why| panic!("the bead's strip refused: {why:?}"));

    bead.rail_count()
}

/// The patch's rows, and the cut's end ring they are read off (from the cap face).
pub fn the_rows_of(corner: &SurfaceMixedCorner) -> (SurfaceMixedCornerRows, Vec<Vector3>) {
    let cut = SurfaceStrip::try_create(&corner.cut, SEGMENTS)
        .unwrap_or_else(|why| panic!("the cut's strip refused: {why:?}"));

    let (_, ring) = try_cut_the_ring(corner, &cut, true, None)
        .unwrap_or_else(|why| panic!("the cut's end ring refused: {why:?}"));

    let start = corner.spine.t_contact_at(0.0);
    let from_the_cap_face: Vec<Vector3> =
        if (ring[0] - start).length() < (ring[ring.len() - 1] - start).length() {
            ring
        } else {
            ring.into_iter().rev().collect()
        };

    let rows = SurfaceMixedCornerRows::try_read_corner(corner, &from_the_cap_face)
        .unwrap_or_else(|why| panic!("the rows refused the ring: {why:?}"));

    (rows, from_the_cap_face)
}

/// Whether `at` stands on a face of `mesh` - in its plane and inside its three sides.
fn stands_on_a_face_of(mesh: &Mesh, at: Vector3) -> bool {
    for face in 0..mesh.faces.len() {
        let f = mesh.faces[face];
        let a = Vector3::from_vector3_float(mesh.vertices[f.v0 as usize]);
        let b = Vector3::from_vector3_float(mesh.vertices[f.v1 as usize]);
        let c = Vector3::from_vector3_float(mesh.vertices[f.v2 as usize]);
        let normal = (b - a).cross(c - a);

        // C#'s `!(x > 0)`, which skips a NaN too.
        if normal.length().partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater) {
            continue;
        }

        let normal = normal.get_normal();

        if (at - a).dot(normal).abs() > 1e-6 {
            continue;
        }

        let first = (b - a).cross(at - a).dot(normal);
        let second = (c - b).cross(at - b).dot(normal);
        let third = (a - c).cross(at - c).dot(normal);

        if (first >= -1e-9 && second >= -1e-9 && third >= -1e-9)
            || (first <= 1e-9 && second <= 1e-9 && third <= 1e-9)
        {
            return true;
        }
    }

    false
}
