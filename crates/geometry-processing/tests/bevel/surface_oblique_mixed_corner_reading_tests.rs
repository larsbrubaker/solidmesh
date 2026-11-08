//! The mixed corner of the nose-in-notch
//! part's left ridge end, which is OBLIQUE in both of the ways the torus reading used to refuse -
//! the handover plane leans 39.76 degrees off square to the cut, and the bead's axis stands 44.80
//! degrees off square to the cap. It reads, and this is what it reads.
//!
//! The upper nose face is the one the two edges SHARE; the lower nose face is the CAP; the left
//! slope is the WALL. A leaning cap cuts the ball's `2r` cylinder about the bead's axis in an
//! ELLIPSE, semi-axes `2r` and `2r / 0.7096`, which still starts on the cut's own centre line with
//! its tangent along the cut. The canal read here is the square corner's own surface exactly and a
//! sub-band of this one (`surface_oblique_mixed_corner_wall_spine_tests` measures the dual), and
//! the square corner is the same reading, pinned to the bit by `the_square_corner_is_unchanged`.
//! `the_oblique_corner` is the one copy of the corner every oblique class reads.

use std::fmt::Write;
use std::rc::Rc;

use sha2::{Digest, Sha256};
use solidmesh::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, surface_mixed_corner_which_canal, SurfaceLine,
    SurfaceMixedCorner, SurfaceMixedCornerSpine,
};
use solidmesh_math::{math_helper, Vector3, Vector3Float};
use solidmesh_mesh::{CancelToken, Mesh};

use crate::mesh_measurements::shell_count;
use crate::nose_in_notch_mixed_corners;
use crate::nose_in_notch_test_meshes;
use crate::surface_mixed_corner_fixture as fixture;
use crate::surface_mixed_corner_reading_tests::the_mixed_corner;
use crate::surface_oblique_round_corner_tests::the_face_list_of;
use crate::topology_test_meshes::create_l_shaped_grid_prism;

const RADIUS: f64 = 1.0;

/// Four decimals, which is what the design's own measurements are quoted to.
const AS_MEASURED: f64 = 5e-5;

const EXACT: f64 = 1e-9;

/// How far off a FACE of the fixture a derived point may stand: the face's plane is written
/// through a corner that has been through float storage while the plan's tangent lines were
/// derived in double - measured, the two disagree by 3.3e-8 on this 20 mm part.
const ON_THE_FACE: f64 = 1e-6;

/// The L prism's mixed-corner reading, hashed (`the_reading_numbers_of`), taken in a tree with
/// none of the spine in it. A nanometre and not the last bit, deliberately: what must not move is
/// the geometry.
const THE_SQUARE_READING_AS_IT_WAS: &str =
    "7a8c97be95afd6c9178a8ddb2eb9278da89ac6e8090bd85da1010bfbe55b37d6";

/// The same corner's published face list at 2, 8 and 16 segments, hashed as
/// `surface_oblique_round_corner_tests::the_face_list_of` hashes the octant's.
const THE_SQUARE_BUILD_AS_IT_WAS: [(i32, &str); 3] = [
    (
        2,
        "0b76c9045f421febf1403b4002dc7b2435fa9fe4ef581f2b0f526fe33858d399/108",
    ),
    (
        8,
        "d4513e0395973c770b63903de8295bfc9d1199b5cc1c00dff1240b598a0ea248/430",
    ),
    (
        16,
        "f5e407d2b73fad4b04b3e4d504cbbfa79a71576ef791f396c201ee2eb78d3fa8/1154",
    ),
];

/// The corner: where the nose's ridge runs out onto the left slope.
fn the_corner() -> Vector3 {
    nose_in_notch_test_meshes::left_ridge_end()
}

/// The way the concave crease runs away from that corner, which a band end has to face.
fn upper_crease_end_from_the_corner() -> Vector3 {
    nose_in_notch_test_meshes::upper_crease_end() - the_corner()
}

fn assert_within(actual: f64, expected: f64, tolerance: f64, because: &str) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} is not {expected} within {tolerance}: {because}"
    );
}

/// The corner reads at all, and the ball at turn zero is the two centre lines' own common
/// perpendicular foot: exactly `2r` from the bead's axis and a radius inside BOTH nose faces.
#[test]
fn the_ball_at_turn_zero_is_the_common_perpendicular_foot() {
    let corner = the_oblique_corner();
    let at = corner.spine.centre_at(0.0);

    assert!(
        at.equals_within(Vector3::new(7.1076, 6.8028, 10.0), AS_MEASURED),
        "the common perpendicular's foot on the cut's centre line - instead {at:?}"
    );

    assert_within(
        corner.axis().distance_to(at),
        2.0 * RADIUS,
        EXACT,
        "the ball touches the bead's void cylinder, so its centre stands 2r out of the axis",
    );

    assert_within(
        (at - the_corner()).dot(nose_in_notch_test_meshes::upper_nose_normal()),
        -RADIUS,
        ON_THE_FACE,
        "and a radius inside the face the two edges share",
    );
    assert_within(
        (at - the_corner()).dot(nose_in_notch_test_meshes::lower_nose_normal()),
        -RADIUS,
        ON_THE_FACE,
        "and a radius inside the cap",
    );

    assert!(
        corner.cut_center_line().distance_to(at) < 1e-6,
        "which is a point of the cut's own centre line - instead {at:?}"
    );

    // The touch point the shared face is left with, a radius up out of that centre.
    assert!(
        corner
            .tangent_point
            .equals_within(Vector3::new(7.1076, 6.2481, 10.8321), AS_MEASURED),
        "instead {:?}",
        corner.tangent_point
    );
}

/// The centre path is the ELLIPSE the plane a radius inside the cap cuts the cylinder of radius
/// `2r` about the axis in: semi-axes `2r` and `2r / 0.7096 = 2.8186`. Measured over the WHOLE
/// turn about the axis, because the arc the blend uses reaches neither extreme.
#[test]
fn the_centre_path_is_the_ellipse_the_plane_cuts_the_cylinder_in() {
    let corner = the_oblique_corner();
    let spine = &corner.spine;

    // Opposite stations of any ellipse straddle its centre, which is how it is found without a
    // second construction of the curve.
    let centre = (spine.centre_at(0.0) + spine.centre_at(std::f64::consts::PI)) / 2.0;

    let mut longest = 0.0_f64;
    let mut shortest = f64::MAX;

    for station in 0..3600 {
        let at = spine.centre_at(station as f64 * std::f64::consts::TAU / 3600.0);
        let out = (at - centre).length();

        longest = longest.max(out);
        shortest = shortest.min(out);

        assert_within(
            corner.axis().distance_to(at),
            2.0 * RADIUS,
            1e-9,
            &format!("every centre of it stands 2r from the axis - {at:?} does not"),
        );
        assert_within(
            (at - the_corner()).dot(nose_in_notch_test_meshes::lower_nose_normal()),
            -RADIUS,
            ON_THE_FACE,
            &format!("and a radius inside the cap - {at:?} does not"),
        );
    }

    assert_within(
        longest,
        2.0 * RADIUS / 0.709575,
        1e-4,
        "the long semi-axis is 2r over how square the axis stands to the cap",
    );
    assert_within(
        shortest,
        2.0 * RADIUS,
        1e-4,
        "and the short one is the cylinder's own 2r",
    );
}

/// And it leaves turn zero ALONG the cut, which makes the handover the cut's own SQUARE end ring.
#[test]
fn it_leaves_turn_zero_along_the_cut_so_the_handover_stays_square_to_it() {
    let corner = the_oblique_corner();
    let spine = &corner.spine;

    let step = 1e-6;
    let tangent = (spine.centre_at(step) - spine.centre_at(-step)).get_normal();

    let leaning = math_helper::radians_to_degrees(
        tangent
            .dot(corner.cut_center_line().direction)
            .abs()
            .clamp(-1.0, 1.0)
            .acos(),
    );

    assert_within(
        leaning,
        0.0,
        1e-3,
        "the spine's tangent is the cut's own direction",
    );

    assert!(
        tangent.equals_within(Vector3::new(-1.0, 0.0, 0.0), 1e-6),
        "running back down the cut, away from the corner's far side - instead {tangent:?}"
    );
}

/// The turn ends where the BEAD's own arc does - 180 degrees less the void wedge the bead sits in.
#[test]
fn the_turn_ends_where_the_beads_arc_does() {
    let corner = the_oblique_corner();
    let turn = math_helper::radians_to_degrees(corner.turn_radians());

    assert_within(turn, 66.9065, 1e-3, "the turn");
    assert_within(
        turn,
        180.0 - corner.bead.wedge_angle_degrees,
        1e-9,
        &format!(
            "the void wedge the bead sits in is {}",
            corner.bead.wedge_angle_degrees
        ),
    );

    // And the far end of the turn IS the way out towards the wall - the left slope here.
    let out = corner.out_of_the_axis(corner.turn_radians());
    assert!(
        out.equals_within(-nose_in_notch_test_meshes::left_slope_normal(), 1e-6),
        "instead {out:?}"
    );
}

/// The bead's end ring is its own SQUARE section through the point the two cylinders touch at -
/// NOT the ring the cap-riding ball traces on the bead, which leans 63.27 degrees off it.
#[test]
fn the_beads_end_ring_is_its_own_square_section() {
    let corner = the_oblique_corner();
    let spine = &corner.spine;

    assert!(
        corner
            .bead_stops_at
            .point
            .equals_within(corner.tangent_point, EXACT),
        "the bead stops on the point the two cylinders touch at - instead {:?}",
        corner.bead_stops_at.point
    );
    assert_within(
        corner
            .bead_stops_at
            .inward
            .dot(corner.axis().direction)
            .abs(),
        1.0,
        EXACT,
        "and it stops SQUARE to its own axis",
    );
    assert!(
        corner
            .bead_stops_at
            .inward
            .dot(upper_crease_end_from_the_corner())
            > 0.0,
        "facing the way the crease runs away from the corner - instead {:?}",
        corner.bead_stops_at.inward
    );

    // And the cap-riding ball's own ring on the bead, which is what that is NOT: planar, leaning
    // 63.27 degrees, and spanning 2.19 mm of the bead's length.
    let plane = spine.tube_ring_plane();

    assert!(
        plane
            .inward
            .equals_within(Vector3::new(-0.2876, 0.4155, 0.8629), AS_MEASURED),
        "instead {:?}",
        plane.inward
    );

    let leaning = math_helper::radians_to_degrees(
        plane
            .inward
            .dot(corner.axis().direction)
            .clamp(-1.0, 1.0)
            .acos(),
    );

    assert_within(
        leaning,
        63.2743,
        1e-3,
        "the bead's square section faces along the axis",
    );

    // It runs from the tangency to the bead's last rail on the wall, and every station between is
    // in the one plane - so the ring is a ring and not a curve needing a surface of its own.
    assert!(
        spine
            .bead_contact_at(0.0)
            .equals_within(corner.tangent_point, 1e-6),
        "instead {:?}",
        spine.bead_contact_at(0.0)
    );
    let last = spine.bead_contact_at(corner.turn_radians());
    assert!(
        last.equals_within(Vector3::new(5.0, 5.0, 10.7304), AS_MEASURED),
        "instead {last:?}"
    );

    let mut along = Vec::new();

    for station in 0..=64 {
        let turn = station as f64 * corner.turn_radians() / 64.0;
        let at = spine.bead_contact_at(turn);

        assert!(
            plane.signed_distance(at).abs() < 1e-9,
            "the bead's contact curve is planar, and {at:?} stands off it"
        );
        assert_within(
            corner.axis().distance_to(at),
            RADIUS,
            EXACT,
            &format!("and lies on the bead's own cylinder - {at:?} does not"),
        );
        assert_within(
            (at - spine.centre_at(turn)).length(),
            RADIUS,
            EXACT,
            &format!("and on the ball at its own station - {at:?} does not"),
        );

        along
            .push((at - corner.axis().closest_point_to(the_corner())).dot(corner.axis().direction));
    }

    let max = along.iter().copied().fold(f64::MIN, f64::max);
    let min = along.iter().copied().fold(f64::MAX, f64::min);

    assert_within(
        max - min,
        2.190,
        1e-3,
        "that staircase spans that much of the bead's length",
    );
}

/// The ball touches the CAP on a curve of its own, and the last station of it is on the WALL as
/// well - the corner where the blend gives way to the run-out.
#[test]
fn the_cap_contact_runs_from_the_handover_to_the_wall() {
    let corner = the_oblique_corner();
    let spine = &corner.spine;

    let first = spine.t_contact_at(0.0);
    assert!(
        first.equals_within(Vector3::new(7.1076, 6.2481, 9.1679), AS_MEASURED),
        "instead {first:?}"
    );
    let last = spine.t_contact_at(corner.turn_radians());
    assert!(
        last.equals_within(Vector3::new(4.2929, 5.1524, 9.8984), AS_MEASURED),
        "instead {last:?}"
    );

    for station in 0..=32 {
        let turn = station as f64 * corner.turn_radians() / 32.0;
        let at = spine.t_contact_at(turn);

        assert_within(
            (at - the_corner()).dot(nose_in_notch_test_meshes::lower_nose_normal()),
            0.0,
            ON_THE_FACE,
            &format!("every one of them stands ON the cap - {at:?} does not"),
        );
        assert_within(
            (at - spine.centre_at(turn)).length(),
            RADIUS,
            EXACT,
            &format!("and on the ball at its own station - {at:?} does not"),
        );
    }
}

/// The same corner at the OTHER end of the ridge, mirrored about `x = 10`: the turn's far end is
/// the way out of the axis towards THAT wall.
#[test]
fn the_mirrored_corner_at_the_other_ridge_end_reads_the_same_way_round() {
    let corner = the_oblique_corner_at(nose_in_notch_test_meshes::right_ridge_end());
    let towards_the_wall = -nose_in_notch_test_meshes::right_slope_normal();

    let out = corner.out_of_the_axis(corner.turn_radians());
    assert!(
        out.equals_within(towards_the_wall, 1e-6),
        "the turn ends on the right slope's own foot - instead {out:?}"
    );

    assert_within(
        math_helper::radians_to_degrees(corner.turn_radians()),
        66.9065,
        1e-3,
        "and sweeps what the left one sweeps",
    );

    // The mirror of (7.1076, 6.8028, 10) about x = 10, and of the tangency with it.
    let first = corner.spine.centre_at(0.0);
    assert!(
        first.equals_within(Vector3::new(12.8924, 6.8028, 10.0), AS_MEASURED),
        "instead {first:?}"
    );
    let bead = corner.spine.bead_contact_at(corner.turn_radians());
    assert!(
        bead.equals_within(Vector3::new(15.0, 5.0, 10.7304), AS_MEASURED),
        "instead {bead:?}"
    );
}

/// The one refusal the spine keeps of its own, and the sliver of lean it lives in: the bead's ring
/// plane turns to run ALONG the bead only as the cap turns to contain the axis. Driven straight at
/// the spine, because no corner of a real part reaches this sliver.
fn the_beads_ring_plane_is_refused_where_it_turns_to_run_along_the_bead(
    how_square_the_cap_is: f64,
    reads: bool,
) {
    let axis = SurfaceLine::new(Vector3::ZERO, Vector3::new(0.0, 0.0, 1.0));

    // A cap almost containing the axis: its normal stands that cosine off the axis and the rest of
    // the way along turn zero, which is what drives the slide's arms up.
    let out_of_the_cap_face = Vector3::new(
        (1.0 - (how_square_the_cap_is * how_square_the_cap_is)).sqrt(),
        0.0,
        how_square_the_cap_is,
    );

    let read = SurfaceMixedCornerSpine::try_create(
        axis,
        Vector3::ZERO,
        out_of_the_cap_face,
        how_square_the_cap_is,
        false,
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        RADIUS,
        std::f64::consts::PI / 2.0,
        Vector3::new(0.0, 0.0, 1.0),
        1e-6,
    );

    assert_eq!(
        read.is_some(),
        reads,
        "the ring plane stands {:.2e} off the bead's rails, against a tolerance of 1e-6",
        how_square_the_cap_is / 2.0
    );

    if let Some(spine) = read {
        assert!(
            spine.tube_ring_plane().inward.dot(axis.direction).abs() > 1e-6,
            "and a spine that reads carries a plane that really does cross them"
        );
    }
}

#[test]
fn the_beads_ring_plane_is_refused_where_it_turns_to_run_along_the_bead_1_5e_6_false() {
    the_beads_ring_plane_is_refused_where_it_turns_to_run_along_the_bead(1.5e-6, false);
}

#[test]
fn the_beads_ring_plane_is_refused_where_it_turns_to_run_along_the_bead_3_0e_6_true() {
    the_beads_ring_plane_is_refused_where_it_turns_to_run_along_the_bead(3.0e-6, true);
}

/// A corner leaning as little as two degrees READS, is ROUTED to the cap-riding canal, and BUILDS:
/// closed, manifold and one solid, because every piece of that path is laid on the elliptical spine
/// the reading names.
#[test]
fn a_corner_leaning_two_degrees_reads_and_builds_on_its_spine() {
    let mesh = the_l_prism_with_its_cap_leaning(2.0);
    let (regions, features) = fixture::regions_of(&mesh);
    let plan = fixture::the_plan_on(&regions, &features);

    let corner = plan
        .try_mixed_corner_after(0)
        .expect("the reading takes a leaning cap now - that is the whole of the rung");

    let leaning = corner.spine.cap_lean_degrees();

    // A ten-thousandth of a degree, because the tilted cap's z goes through float storage and
    // comes back a millionth of a degree steeper than it was laid.
    assert_within(leaning, 2.0, 1e-4, "the cap was tilted two degrees");

    assert!(
        surface_mixed_corner_which_canal::try_how_far_the_duals_crease_runs(&corner).is_none(),
        "the dual's crease runs the other way out of this corner, into the notch's void, where the L carries no part at all"
    );
    let cap_riding_reach =
        surface_mixed_corner_which_canal::try_how_far_the_cap_riding_crease_runs(&corner).expect(
            "while the cap-riding run-out's own crease is the rim the square corner's run-out already dies on",
        );
    assert!(
        cap_riding_reach > 10.0 - 1e-4,
        "ten millimetres of it, stretched by the tilt of the cap it lies in - instead {cap_riding_reach}"
    );

    assert!(
        !corner.takes_the_dual_canal(),
        "so the selector routes this corner to the CAP-RIDING path - by crease reach, and never by how far the cap leans"
    );

    let band_end = corner.band_end_for(&corner.cut);

    assert!(
        band_end.point.equals_within(corner.cut_stops_at.point, 1e-9)
            && band_end.inward.equals_within(corner.cut_stops_at.inward, 1e-9),
        "and the cut's band stops on the plane THAT path's ring really stands on - the square handover - instead {:?} / {:?}",
        band_end.point,
        band_end.inward
    );

    assert!(
        !corner.hands_over_at_a_mitre,
        "and it hands over on its own tangency, which is the handover the path lays on the spine"
    );

    let result = surface_bevel_builder::try_build_one(
        &mesh,
        &mesh.get_mesh_edge_graph(),
        &regions,
        plan.clone(),
        8,
        &CancelToken::new(),
    )
    .unwrap_or_else(|reason| {
        panic!("so the product builds this corner now, with no gate lifted for it - instead {reason:?}")
    });

    let (open, open_at) = surface_mesh_closure::open_edge_count(&result);
    assert_eq!(open, 0, "closed - instead open at {open_at:?}");

    let faces: Vec<(i32, i32, i32)> = result
        .faces
        .iter()
        .map(|face| (face.v0, face.v1, face.v2))
        .collect();

    let (folded, folded_at) = surface_mesh_closure::folded_edge_count(&faces);
    assert_eq!(
        folded, 0,
        "manifold - instead folded between {} and {}",
        folded_at.0, folded_at.1
    );
    assert_eq!(shell_count(&result), 1, "and one solid");
}

/// The square corner is untouched: the L prism's reading and its published mesh are the ones
/// they were - at every station and every face, to the nanometre these hashes round at.
#[test]
fn the_square_corner_is_unchanged() {
    assert_eq!(
        the_reading_numbers_of(&the_mixed_corner()),
        THE_SQUARE_READING_AS_IT_WAS,
        "the spine is the torus where the axis stands square, and any other reading of it is a change to the case the whole ladder was measured on"
    );

    for (segments, hash) in THE_SQUARE_BUILD_AS_IT_WAS {
        let (built, _) = fixture::the_build(segments);

        assert_eq!(
            the_face_list_of(&built),
            hash,
            "and the corner it builds at {segments} segments is the one it built"
        );
    }
}

/// Every frame vector, both stopping planes, and the ball's centre, cap
/// contact and rigid turn at thirty-three stations, rounded to a nanometre and hashed.
pub fn the_reading_numbers_of(corner: &SurfaceMixedCorner) -> String {
    let mut said = String::new();

    // A zero is a zero whichever sign it carries: -0.0 and 0.0 print differently, so an arithmetic
    // rearrangement that could not move the geometry could still move this text.
    fn zeroed(d: f64) -> f64 {
        if d == 0.0 {
            0.0
        } else {
            d
        }
    }

    let at = |said: &mut String, v: Vector3| {
        let _ = write!(
            said,
            "{:.9},{:.9},{:.9};",
            zeroed(v.x),
            zeroed(v.y),
            zeroed(v.z)
        );
    };
    let num = |said: &mut String, d: f64| {
        let _ = write!(said, "{:.9};", zeroed(d));
    };

    at(&mut said, corner.corner);
    num(&mut said, corner.size);
    at(&mut said, corner.tangent_point);
    at(&mut said, corner.cut_stops_at.point);
    at(&mut said, corner.cut_stops_at.inward);
    at(&mut said, corner.bead_stops_at.point);
    at(&mut said, corner.bead_stops_at.inward);
    at(&mut said, corner.band_end_for(&corner.bead).point);
    at(&mut said, corner.band_end_for(&corner.bead).inward);
    num(&mut said, corner.turn_radians());
    at(&mut said, corner.outer_rail_centre);
    num(&mut said, corner.outer_rail_radius());
    at(&mut said, corner.towards_the_shared_face());
    at(&mut said, corner.towards_the_wall_square());
    at(&mut said, corner.out_of_the_cap_face);
    at(&mut said, corner.out_of_the_wall);
    at(&mut said, corner.cap_corner());
    at(&mut said, corner.shared_face_foot());

    for station in 0..=32 {
        let turn = station as f64 * corner.turn_radians() / 32.0;

        at(&mut said, corner.out_of_the_axis(turn));
        at(&mut said, corner.outer_rail_point(turn));
        at(&mut said, corner.ball_centre_at(turn));
        at(&mut said, corner.turned_about(corner.tangent_point, turn));
        at(&mut said, corner.turned_about(corner.corner, turn));
    }

    Sha256::digest(said.as_bytes())
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect()
}

/// The corner the nose's ridge and its upper left crease make.
pub fn the_oblique_corner() -> Rc<SurfaceMixedCorner> {
    the_oblique_corner_at(the_corner())
}

/// The same at either end of the ridge: the left one the design's numbers were measured at, and
/// the right one they are mirrored about `x = 10` at. Read through `nose_in_notch_mixed_corners`,
/// so every class reads the one corner.
pub fn the_oblique_corner_at(ridge_end: Vector3) -> Rc<SurfaceMixedCorner> {
    nose_in_notch_mixed_corners::at_the_ridge_end(ridge_end, RADIUS)
}

/// The L prism with its cap tilted about the plane `x = 10` - the face the two picked edges SHARE
/// - so the corner vertex and both of those edges stay where they are and only the cap leans.
fn the_l_prism_with_its_cap_leaning(degrees: f64) -> Mesh {
    let mut mesh = create_l_shaped_grid_prism();
    let slope = math_helper::degrees_to_radians(degrees).tan();

    for vertex in 0..mesh.vertices.len() {
        let at = mesh.vertices[vertex];

        if ((at.z - 10.0) as f64).abs() < 1e-6 {
            mesh.vertices[vertex] = Vector3Float::new(
                at.x as f64,
                at.y as f64,
                ((10.0 + (slope * (at.x - 10.0) as f64)) as f32) as f64,
            );
        }
    }

    mesh
}
