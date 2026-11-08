//! The closest point on or behind every face a corner patch's vertex crosses
//! (`corner_cap_hold::closest_within_the_planes`): the point itself where it crosses none, its
//! projection onto the one face it crosses, and where two faces hold it, the point on the line
//! they meet in - not one face's projection after the other's; and the held patch's two shape
//! re-checks (`corner_cap_hold::the_held_patch_fails`), each pinned where it alone fires.

use solidmesh::bevel::level_line::corner_cap_hold::{
    closest_within_the_planes, the_held_patch_fails, HeldPatchFault,
};
use solidmesh::bevel::level_line::{hole_fill, HoleFillFailure, HoleFillResult, HoleFillTriangle};
use solidmesh_math::Vector3;

/// The quarter-space x <= 0, y <= 0, as two faces with outward normals.
fn corner() -> Vec<(Vector3, Vector3)> {
    vec![
        (Vector3::ZERO, Vector3::new(1.0, 0.0, 0.0)),
        (Vector3::ZERO, Vector3::new(0.0, 1.0, 0.0)),
    ]
}

#[test]
fn a_point_crossing_no_face_stays() {
    let p = Vector3::new(-1.0, -2.0, 3.0);
    assert_eq!(closest_within_the_planes(p, &corner()), Some(p));
}

#[test]
fn a_point_crossing_one_face_goes_onto_it() {
    let p = Vector3::new(0.5, -2.0, 3.0);
    assert_eq!(
        closest_within_the_planes(p, &corner()),
        Some(Vector3::new(0.0, -2.0, 3.0))
    );
}

/// Crossing both faces, the closest point is on the line x = y = 0.
#[test]
fn a_point_crossing_two_faces_goes_where_they_meet() {
    let p = Vector3::new(0.5, 0.25, 3.0);
    assert_eq!(
        closest_within_the_planes(p, &corner()),
        Some(Vector3::new(0.0, 0.0, 3.0))
    );
}

/// Two faces meeting at 60 degrees, the point past only the first: projecting onto the first
/// carries it through the second, so the answer is on their meeting line, standing on or behind
/// both.
#[test]
fn a_projection_carried_through_another_face_lands_where_they_meet() {
    let tilted = Vector3::new(-0.5, 3.0_f64.sqrt() / 2.0, 0.0);
    let planes = vec![
        (Vector3::ZERO, Vector3::new(1.0, 0.0, 0.0)),
        (Vector3::ZERO, tilted),
    ];
    let p = Vector3::new(1.0, 0.3, 0.0);
    let held = closest_within_the_planes(p, &planes).expect("the faces meet");
    for &(origin, outward) in &planes {
        assert!(
            outward.dot(held - origin) <= 1e-12,
            "{held:?} through {outward:?}"
        );
    }
    assert!(held.x.abs() <= 1e-12 && held.y.abs() <= 1e-12, "{held:?}");
}

/// The closest point to `p` within `planes`, by brute force over a fine grid around the answer
/// would be slow; instead the answer is held to the two things that define it: it stands within
/// every plane, and moving it toward `p` along the line to `p` leaves some plane.
fn holds(p: Vector3, planes: &[(Vector3, Vector3)], held: Vector3) {
    for &(origin, outward) in planes {
        assert!(
            outward.dot(held - origin) <= 1e-12,
            "{held:?} through {outward:?}"
        );
    }
    let nearer = held + (p - held) * 1e-6;
    assert!(
        planes
            .iter()
            .any(|&(origin, outward)| outward.dot(nearer - origin) > 0.0),
        "{held:?} is not the closest: a point nearer {p:?} still holds"
    );
}

/// Four faces, from a seeded search, where growing and shrinking an active set refused a vertex
/// the faces hold (their region runs on below): every subset of up to three is tried.
#[test]
fn four_faces_hold_a_point_the_active_set_refused() {
    let planes = vec![
        (
            Vector3::new(
                0.11602105871322231,
                0.07202912244502091,
                -0.09049453848907274,
            ),
            Vector3::new(0.7171545086435882, 0.675677319781236, 0.17073245229135248),
        ),
        (
            Vector3::new(
                0.1908880917806017,
                0.09516641718983694,
                -0.06097788646612119,
            ),
            Vector3::new(-0.6649367094458045, 0.7092321571698665, 0.234198462137486),
        ),
        (
            Vector3::new(
                -0.17296797142403084,
                -0.15525050509264965,
                0.0581585887619259,
            ),
            Vector3::new(0.6832971742801075, -0.6438745514334692, 0.3442826362702151),
        ),
        (
            Vector3::new(0.08842314457434389, 0.1560550187449135, 0.12082377634183757),
            Vector3::new(
                -0.8678064108124585,
                0.09687195515298083,
                0.48736829775605806,
            ),
        ),
    ];
    let p = Vector3::new(-0.0708730559579287, 0.5417703060086714, 2.137142891587279);
    let held = closest_within_the_planes(p, &planes).expect("the faces hold it");
    holds(p, &planes, held);
}

/// Three faces of a corner, the point past all three: it goes to their corner.
#[test]
fn a_point_past_three_faces_goes_to_their_corner() {
    let planes = vec![
        (Vector3::ZERO, Vector3::new(1.0, 0.0, 0.0)),
        (Vector3::ZERO, Vector3::new(0.0, 1.0, 0.0)),
        (Vector3::ZERO, Vector3::new(0.0, 0.0, 1.0)),
    ];
    let p = Vector3::new(0.5, 0.25, 0.75);
    assert_eq!(closest_within_the_planes(p, &planes), Some(Vector3::ZERO));
}

/// One face listed twice reads as one: the point goes onto it, not refused as two parallel faces.
#[test]
fn a_face_listed_twice_holds_like_one() {
    let face = (Vector3::ZERO, Vector3::new(1.0, 0.0, 0.0));
    let p = Vector3::new(0.5, 1.0, 2.0);
    assert_eq!(
        closest_within_the_planes(p, &[face, face]),
        Some(Vector3::new(0.0, 1.0, 2.0))
    );
}

/// Two faces a hair off parallel, the point past both: the nearer-in face holds it on its own,
/// and their near-parallel meeting is never needed.
#[test]
fn near_parallel_faces_hold_on_the_inner_one() {
    let tilt = 1e-14;
    let planes = vec![
        (Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)),
        (
            Vector3::new(-0.1, 0.0, 0.0),
            Vector3::new(1.0, tilt, 0.0).get_normal(),
        ),
    ];
    let p = Vector3::new(0.5, 1.0, 2.0);
    let held = closest_within_the_planes(p, &planes).expect("the inner face holds it");
    holds(p, &planes, held);
    assert!((held.x + 0.1).abs() <= 1e-12, "{held:?}");
}

/// A unit square's loop fanned to one interior vertex at its middle, as the filler left it, and the
/// same patch with that vertex held at `held`.
fn fan_held_at(held: Vector3) -> (Vec<Vector3>, HoleFillResult) {
    let before = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.5, 0.5, 0.0),
    ];
    let triangles = vec![
        HoleFillTriangle::new(0, 1, 4),
        HoleFillTriangle::new(1, 2, 4),
        HoleFillTriangle::new(2, 3, 4),
        HoleFillTriangle::new(3, 0, 4),
    ];
    let mut after = before.clone();
    after[4] = held;
    (
        before,
        HoleFillResult::new(HoleFillFailure::None, after, triangles),
    )
}

/// Held out past the loop's first edge, a triangle turns over, though every triangle keeps its
/// area: only the turn check refuses it.
#[test]
fn a_hold_that_turns_a_triangle_over_is_refused_by_the_turn_check() {
    let (before, held) = fan_held_at(Vector3::new(0.5, -0.2, 0.0));
    let tiny = hole_fill::tiny_area(&held.vertices[..4]);
    assert_eq!(
        hole_fill::validate(&held.vertices, &held.triangles, 4, tiny),
        HoleFillFailure::None
    );
    assert_eq!(
        the_held_patch_fails(&before, &held, 4),
        Some(HeldPatchFault::TurnedOver)
    );
}

/// Held a hair inside the first edge, every triangle keeps its turn but one is too thin to count:
/// only the filler's own validation refuses it.
#[test]
fn a_hold_that_flattens_a_triangle_is_refused_by_validation() {
    let (before, held) = fan_held_at(Vector3::new(0.5, 1e-12, 0.0));
    assert_eq!(
        the_held_patch_fails(&before, &held, 4),
        Some(HeldPatchFault::Invalid(HoleFillFailure::ZeroAreaTriangle))
    );
}

/// Held where it was, the patch passes both.
#[test]
fn a_hold_that_moves_nothing_passes_both_checks() {
    let (before, held) = fan_held_at(Vector3::new(0.5, 0.5, 0.0));
    assert_eq!(the_held_patch_fails(&before, &held, 4), None);
}
