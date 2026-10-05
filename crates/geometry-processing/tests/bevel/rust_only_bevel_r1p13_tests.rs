//! Rust-only checks of the bevel core's R1.P13 package pieces no ported C# test reaches on its own:
//! the end-face sense's ray bounds and convex end planes, the crease plane, the end-plane trim's
//! reach and section, the fillet arc's helpers, the arc point, the corner cap's switches, the band
//! gap's small readings, the band room's rounding and pair bounds and the corner cap's geometry.

use std::collections::HashMap;

use geometry_processing::bevel::level_line::{
    band_gap, band_gap_rounds_first, band_room, band_room_pair_room, corner_cap,
    corner_cap_flat_ends, corner_cap_hole_fill, corner_cap_hole_fill_switch, end_face_crease_plane,
    end_face_sense, end_plane_trim, fillet_arc, gap_fill, level_line_routing, triangle_edges,
    BandProfile, CornerCapLastRow, HoleFillFailure, HoleFillResult, HoleFillTriangle,
    LevelLineTriangle, RoomSource,
};
use vector_math::Vector3;

fn near(a: Vector3, b: Vector3, tolerance: f64) -> bool {
    (a - b).length() <= tolerance
}

/// A concave chain 0-1 along x (floor z = 0 meets wall y = 0) ending at vertex 1 on one end
/// triangle (1, Q, P) that leans back over the chain: its normal is -(1, 0.5, 0.5).
fn concave_chain_onto_a_leaning_end() -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    let positions = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.5, 1.0, 0.0),
        Vector3::new(0.5, 0.0, 1.0),
    ];
    (positions, vec![(0, 1, 2), (1, 0, 3), (1, 3, 2)])
}

/// The unit cube's top-front edge along x ending on the x = 1 face: convex, square end.
fn convex_cube_edge() -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    let positions = vec![
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, 0.0, 1.0),
        Vector3::new(1.0, 0.5, 1.0),
        Vector3::new(1.0, 0.0, 0.0),
    ];
    (positions, vec![(0, 1, 2), (1, 0, 3), (1, 3, 2)])
}

#[test]
fn rust_only_a_concave_chain_onto_a_leaning_end_bounds_its_ray_at_the_plane() {
    let (positions, indices) = concave_chain_onto_a_leaning_end();
    assert!(!end_face_sense::is_convex(
        &positions, indices[0], indices[1], 1
    ));
    let (first, last) =
        end_face_sense::end_ray_bounds(&positions, &indices, &[0, 1], (false, true));
    assert!(first.is_none());
    let bound = last.expect("the leaning end is read");
    // Both rails stand 0.5 / sqrt(1.5) out of the plane, over a slant of 1 / sqrt(1.5).
    assert!((bound.reach_per_setback - 0.5).abs() <= 1e-12);
    let n = Vector3::new(1.0, 0.5, 0.5).get_normal();
    assert!(near(bound.plane_normal, n, 1e-12));
    assert!(bound.same_sense);
    assert_eq!(bound.crease_room, f64::INFINITY);
    // A concave end has no convex end plane.
    let faces = end_face_sense::convex_end_faces(&positions, &indices, &[0, 1], (true, true));
    assert_eq!(faces, (None, None));
}

#[test]
fn rust_only_a_convex_square_end_has_a_plane_but_no_ray() {
    let (positions, indices) = convex_cube_edge();
    assert!(end_face_sense::is_convex(
        &positions, indices[0], indices[1], 1
    ));
    let (_, last) = end_face_sense::end_ray_bounds(&positions, &indices, &[0, 1], (false, true));
    assert!(last.is_none());
    let (_, face) = end_face_sense::convex_end_faces(&positions, &indices, &[0, 1], (false, true));
    let (point, normal) = face.expect("the x = 1 face is read");
    assert_eq!(point, Vector3::new(1.0, 0.0, 1.0));
    assert!(near(normal, Vector3::new(1.0, 0.0, 0.0), 1e-12));
    assert_eq!(
        end_face_sense::oblique_ends(&positions, &indices, &[0, 1], false, f64::INFINITY),
        (false, false)
    );
    assert_eq!(
        end_face_sense::oblique_ends(&positions, &indices, &[0, 1], true, 10.0),
        (false, false)
    );
}

#[test]
fn rust_only_a_leaning_end_off_a_concave_chain_takes_a_ray() {
    let (positions, indices) = concave_chain_onto_a_leaning_end();
    // Vertex 0 has no end face; vertex 1's leans back across a concave chain, which takes the ray
    // with flat ends on. Flag off, it must lean back over both rails, and it leans across them.
    corner_cap_flat_ends::set_use_flat_ends_override(Some(true));
    let flat = end_face_sense::oblique_ends(&positions, &indices, &[0, 1], false, 10.0);
    // With no bend threshold every crease carries on, so no end takes a ray.
    let unbounded =
        end_face_sense::oblique_ends(&positions, &indices, &[0, 1], false, f64::INFINITY);
    corner_cap_flat_ends::set_use_flat_ends_override(Some(false));
    let not_flat = end_face_sense::oblique_ends(&positions, &indices, &[0, 1], false, 10.0);
    corner_cap_flat_ends::set_use_flat_ends_override(None);
    assert_eq!(flat, (false, true));
    assert_eq!(unbounded, (false, false));
    assert_eq!(not_flat, (false, false));
}

#[test]
fn rust_only_leans_off_square_past_fifteen_degrees() {
    assert!(!end_face_sense::leans_off_square(-1.0));
    assert!(!end_face_sense::leans_off_square(
        -(10.0f64.to_radians().cos())
    ));
    assert!(end_face_sense::leans_off_square(
        -(20.0f64.to_radians().cos())
    ));
}

#[test]
fn rust_only_a_side_runs_past_the_end_plane_only_where_its_face_carries_on() {
    let positions = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(2.0, 0.0, 0.0),
        Vector3::new(0.0, 2.0, 0.0),
        Vector3::new(-2.0, 0.0, 0.0),
    ];
    let outward = Vector3::new(1.0, 0.0, 0.0);
    let rails = [Vector3::new(0.0, 1.0, 0.0)];
    let normals = [Vector3::new(0.0, 0.0, 1.0)];
    let on = end_face_sense::side_runs_past_the_end_plane(
        &positions,
        &[(0, 1, 2)],
        0,
        outward,
        outward,
        &rails,
        &normals,
    );
    let off = end_face_sense::side_runs_past_the_end_plane(
        &positions,
        &[(0, 2, 3)],
        0,
        outward,
        outward,
        &rails,
        &normals,
    );
    assert!(on);
    assert!(!off);
}

#[test]
fn rust_only_the_crease_plane_runs_through_both_creases_or_falls_back() {
    let (positions, indices) = concave_chain_onto_a_leaning_end();
    let fallback = Vector3::new(-1.0, 0.0, 0.0);
    let plane =
        end_face_crease_plane::through(&positions, 1, &indices[..2], &indices[2..], fallback);
    assert!(near(
        plane,
        -Vector3::new(1.0, 0.5, 0.5).get_normal(),
        1e-12
    ));
    // One crease only: the fallback.
    let one = end_face_crease_plane::through(&positions, 1, &indices[..1], &indices[2..], fallback);
    assert_eq!(one, fallback);
}

#[test]
fn rust_only_farthest_out_reads_the_arc_top_only_where_it_bulges_out() {
    let end = Vector3::new(0.0, 0.0, 0.0);
    let left = Vector3::new(0.0, 1.0, 0.0);
    let right = Vector3::new(0.0, 0.0, 1.0);
    let n = Vector3::new(0.0, -1.0, -1.0).get_normal();
    // The arc about (0, 1, 1) of radius 1 comes nearest the end at its middle, which stands
    // furthest out along n.
    let fillet = end_plane_trim::farthest_out(left, right, end, n, BandProfile::Fillet);
    assert!((fillet - (1.0 - std::f64::consts::SQRT_2)).abs() <= 1e-12);
    let chamfer = end_plane_trim::farthest_out(left, right, end, n, BandProfile::Chamfer);
    assert!((chamfer - -std::f64::consts::FRAC_1_SQRT_2).abs() <= 1e-12);
}

#[test]
fn rust_only_the_end_plane_section_keeps_its_ends_and_lies_on_the_plane() {
    let end = Vector3::new(0.0, 0.0, 0.0);
    let outward = Vector3::new(1.0, 0.0, 0.0);
    let normal = Vector3::new(1.0, 0.0, 0.0);
    let a = Vector3::new(0.0, 1.0, 0.0);
    let b = Vector3::new(0.0, 0.0, 1.0);
    let fillet = end_plane_trim::section(a, b, end, outward, normal, BandProfile::Fillet, 4);
    assert_eq!(fillet.len(), 5);
    assert_eq!(fillet[0], a);
    assert_eq!(fillet[4], b);
    let centre = Vector3::new(0.0, 1.0, 1.0);
    for p in &fillet {
        assert!(p.x.abs() <= 1e-12);
        assert!(((*p - centre).length() - 1.0).abs() <= 1e-12);
    }
    let chamfer = end_plane_trim::section(a, b, end, outward, normal, BandProfile::Chamfer, 4);
    assert_eq!(chamfer, vec![a, b]);
}

#[test]
fn rust_only_fillet_arc_helpers() {
    assert_eq!(fillet_arc::smooth_step(-1.0), 0.0);
    assert_eq!(fillet_arc::smooth_step(0.5), 0.5);
    assert_eq!(fillet_arc::smooth_step(2.0), 1.0);
    assert_eq!(
        fillet_arc::seam_tolerance(Vector3::new(3.0, 4.0, 0.0), 1.0),
        6e-6
    );
    let joints = [(1.0, 0.1), (2.0, 0.1), (2.0, 0.2), (5.0, 0.1)];
    assert_eq!(fillet_arc::first_joint_at_or_past(&joints, 0.0), 0);
    assert_eq!(fillet_arc::first_joint_at_or_past(&joints, 2.0), 1);
    assert_eq!(fillet_arc::first_joint_at_or_past(&joints, 2.5), 3);
    assert_eq!(fillet_arc::first_joint_at_or_past(&joints, 6.0), 4);
    let x = Vector3::new(1.0, 0.0, 0.0);
    let y = Vector3::new(0.0, 1.0, 0.0);
    assert!(near(fillet_arc::turn(x, y, x), y, 1e-15));
    assert!(near(
        fillet_arc::turn(x, y, Vector3::new(0.0, 0.0, 2.0)),
        Vector3::new(0.0, 0.0, 2.0),
        1e-15
    ));
    assert_eq!(fillet_arc::turn(x, x, y), y);
}

#[test]
fn rust_only_carried_with_one_direction_is_the_plain_arc() {
    let x = Vector3::new(1.0, 0.0, 0.0);
    let origin = Vector3::new(0.0, 0.0, 0.0);
    let from = Vector3::new(0.0, 1.0, 0.0);
    let to = Vector3::new(0.0, 0.0, 1.0);
    let arc = fillet_arc::carried(from, to, (origin, x), (origin, x), (origin, x), 4);
    assert_eq!(arc[0], from);
    assert_eq!(arc[4], to);
    for (s, p) in arc.iter().enumerate().take(4).skip(1) {
        let expected = gap_fill::arc_point(from, origin, to, s as f64 / 4.0);
        assert!(near(*p, expected, 1e-15));
    }
}

#[test]
fn rust_only_arc_point_is_the_quarter_circle_and_backwards_reverses() {
    let p = Vector3::new(1.0, 0.0, 0.0);
    let c = Vector3::new(1.0, 1.0, 0.0);
    let q = Vector3::new(0.0, 1.0, 0.0);
    let mid = gap_fill::arc_point(p, c, q, 0.5);
    assert!((mid.length() - 1.0).abs() <= 1e-15);
    assert_eq!(gap_fill::arc_point(p, c, q, 0.0), p);
    assert_eq!(gap_fill::backwards(&[1, 2, 3]), vec![3, 2, 1]);
    // Forward the only backward step is the wrap 3.0 -> 0.5 (i = 3); read reversed every step but
    // the wrap goes backward, and the first of the largest (3.0 -> 2.0, i = 0) is taken.
    let parameters = [0.5, 1.0, 2.0, 3.0];
    assert_eq!(gap_fill::largest_backward_step(&parameters, false), 3);
    assert_eq!(gap_fill::largest_backward_step(&parameters, true), 0);
    assert_eq!(gap_fill::largest_backward_step(&[1.0, 1.0], false), -1);
}

#[test]
fn rust_only_settings_read_as_off_only_when_they_say_so() {
    assert!(level_line_routing::reads_as_off(Some("0")));
    assert!(level_line_routing::reads_as_off(Some("FALSE")));
    assert!(!level_line_routing::reads_as_off(Some("no")));
    assert!(!level_line_routing::reads_as_off(None));
    assert!(level_line_routing::the_running_app_wants_it_on(None));
    assert!(!level_line_routing::the_running_app_wants_it_on(Some(
        "false"
    )));
}

#[test]
fn rust_only_the_corner_cap_switches_take_a_per_thread_override() {
    corner_cap_flat_ends::set_use_flat_ends_override(Some(false));
    assert!(!corner_cap_flat_ends::use_flat_ends());
    corner_cap_flat_ends::set_use_flat_ends_override(Some(true));
    assert!(corner_cap_flat_ends::use_flat_ends());
    corner_cap_flat_ends::set_use_flat_ends_override(None);
    assert_eq!(corner_cap_flat_ends::use_flat_ends_override(), None);
    corner_cap_hole_fill_switch::set_use_hole_fill_override(Some(false));
    assert!(!corner_cap_hole_fill_switch::use_hole_fill());
    corner_cap_hole_fill_switch::set_use_hole_fill_override(None);
}

#[test]
fn rust_only_band_gap_tolerances_and_planes() {
    assert_eq!(band_gap::chord_tolerance_of(2.0, 0.0), 0.1);
    assert_eq!(band_gap::chord_tolerance_of(2.0, 0.3), 0.3);
    assert_eq!(band_gap::needle_floor_of(1e-6, 1.0), 1e-3);
    assert_eq!(band_gap::needle_floor_of(1.0, 1.0), 4.0);
    let at = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(3.0, 0.0, 0.0),
        Vector3::new(0.0, 3.0, 0.0),
        Vector3::new(9.0, 9.0, 0.0),
        Vector3::new(9.0, 9.0, 1.0),
    ];
    let p = band_gap_rounds_first::plane_of(&at, (0, 1, 2));
    assert_eq!(
        p,
        (
            Vector3::new(0.0, 0.0, 1.0),
            0.0,
            Vector3::new(1.0, 1.0, 0.0)
        )
    );
    assert!(band_gap_rounds_first::same_plane(
        p,
        band_gap_rounds_first::plane_of(&at, (1, 3, 2))
    ));
    assert!(!band_gap_rounds_first::same_plane(
        p,
        band_gap_rounds_first::plane_of(&at, (1, 4, 2))
    ));
    assert!(!band_gap_rounds_first::same_plane(
        p,
        band_gap_rounds_first::plane_of(&at, (0, 2, 1))
    ));
    assert_eq!(triangle_edges::corners((4, 5, 6)), [(4, 5), (5, 6), (6, 4)]);
    assert_eq!(
        triangle_edges::normal(&at, (0, 2, 1)),
        Vector3::new(0.0, 0.0, -1.0)
    );
}

/// A straight run 0 - 1 - 2 - 3 along x with a spur 1 - 4 off it.
fn run_with_a_spur() -> (Vec<Vector3>, HashMap<i32, Vec<i32>>) {
    let at = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(2.0, 0.0, 0.0),
        Vector3::new(3.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
    ];
    let neighbours = HashMap::from([
        (0, vec![1]),
        (1, vec![0, 2, 4]),
        (2, vec![1, 3]),
        (3, vec![2]),
        (4, vec![1]),
    ]);
    (at, neighbours)
}

#[test]
fn rust_only_the_surviving_piece_ends_where_the_crease_leaves_the_line() {
    let (at, neighbours) = run_with_a_spur();
    // The chain 0 -> 3 has lost vertex 1 from its edge list; the walk lays it back.
    let all_crease = |_: i32, _: i32| true;
    assert_eq!(
        band_gap_rounds_first::surviving_piece(&[0, 3], &at, &all_crease, &neighbours),
        None
    );
    let to_two = |a: i32, b: i32| !(a == 2 && b == 3);
    assert_eq!(
        band_gap_rounds_first::surviving_piece(&[0, 3], &at, &to_two, &neighbours),
        Some(vec![0, 1, 2])
    );
    let none = |_: i32, _: i32| false;
    assert_eq!(
        band_gap_rounds_first::surviving_piece(&[0, 3], &at, &none, &neighbours),
        None
    );
}

#[test]
fn rust_only_the_crease_on_the_end_face_walks_one_way_or_refuses() {
    let (_, neighbours) = run_with_a_spur();
    let all_crease = |_: i32, _: i32| true;
    // From 1, having come from 0, both 2 and 4 are crease: a fork.
    assert_eq!(
        band_gap_rounds_first::crease_on_the_end_face(1, 0, 3, &all_crease, &neighbours),
        None
    );
    let not_the_spur = |a: i32, b: i32| a != 4 && b != 4;
    assert_eq!(
        band_gap_rounds_first::crease_on_the_end_face(1, 0, 3, &not_the_spur, &neighbours),
        Some(vec![2, 3])
    );
}

#[test]
fn rust_only_the_room_rounds_down_onto_twelve_digits() {
    assert_eq!(band_room::steady_down(2.0000000000000004), 2.0);
    assert_eq!(band_room::steady_down(-1.67e-9), 0.0);
    assert_eq!(band_room::steady_down(f64::INFINITY), f64::INFINITY);
    assert!(band_room::steady_down(f64::NAN).is_nan());
    assert_eq!(band_room::on_grid(123.456, 2), 120.0);
    assert_eq!(band_room::on_grid(-3.0, 2), -3.0);
    let face = (Vector3::new(1.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
    assert_eq!(band_room::present((None, Some(face))), vec![face]);
    assert_eq!(band_room::present((Some(face), Some(face))).len(), 2);
}

#[test]
fn rust_only_the_foreign_source_names_a_growing_crease_only_where_it_alone_sets_the_room() {
    let at = Vector3::new(1.0, 2.0, 3.0);
    let crease_at = || at;
    assert_eq!(
        band_room::the_foreign_source(1.0, 2.0, f64::INFINITY, &crease_at),
        RoomSource::nothing()
    );
    assert_eq!(
        band_room::the_foreign_source(2.0, 1.0, 1.0, &crease_at),
        RoomSource::growing_crease(0, at)
    );
    assert_eq!(
        band_room::the_foreign_source(1.0, 1.0, 1.0, &crease_at),
        RoomSource::foreign(0)
    );
}

#[test]
fn rust_only_pair_room_bounds() {
    assert!(band_room_pair_room::is_uniform(&[0.5, 0.5]));
    assert!(!band_room_pair_room::is_uniform(&[]));
    let at = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(2.0, 0.0, 0.0),
        Vector3::new(2.0, 2.0, 0.0),
    ];
    let p = Vector3::new(1.0, 1.0, 0.0);
    assert_eq!(
        band_room_pair_room::distance_to_polyline(&at, &[0, 1, 2], false, p),
        1.0
    );
    let closed = band_room_pair_room::distance_to_polyline(&at, &[0, 1, 2], true, p);
    assert!(closed.abs() <= 1e-15);
    assert_eq!(band_room_pair_room::index_in(&[4, 7, 9], 9), 2);
    assert_eq!(band_room_pair_room::index_in(&[4, 7, 9], 5), -1);
    assert_eq!(band_room_pair_room::sphere_gap(at[0], 0.5, at[1], 0.5), 1.0);
    assert_eq!(band_room_pair_room::sphere_gap(at[0], 1.5, at[1], 0.5), 0.0);
    assert!(band_room_pair_room::pair_cannot_lower_the_room(
        0.5,
        2.0,
        &[1.0],
        0.0
    ));
    assert!(!band_room_pair_room::pair_cannot_lower_the_room(
        2.0,
        2.0,
        &[1.0],
        0.0
    ));
    assert!(!band_room_pair_room::pair_cannot_lower_the_room(
        0.5,
        2.0,
        &[0.0, 1.0],
        0.0
    ));
    assert!(!band_room_pair_room::pair_cannot_lower_the_room(
        f64::INFINITY,
        2.0,
        &[1.0],
        0.0
    ));
}

#[test]
fn rust_only_corner_cap_turns_between_directions() {
    let x = Vector3::new(2.0, 0.0, 0.0);
    let y = Vector3::new(0.0, 4.0, 0.0);
    let right = corner_cap::angle(x, y);
    assert!((right - std::f64::consts::FRAC_PI_2).abs() <= 1e-15);
    let half = corner_cap::toward(x, y, right, 0.5);
    let expected = Vector3::new(1.0, 1.0, 0.0).get_normal() * 3.0;
    assert!(near(half, expected, 1e-14));
    let straight = corner_cap::toward(x, x * 2.0, 0.0, 0.5);
    assert!(near(straight, Vector3::new(3.0, 0.0, 0.0), 1e-15));
}

fn ball_row(concave: bool) -> CornerCapLastRow {
    CornerCapLastRow {
        first: Vector3::new(1.0, 0.0, 0.0),
        last: Vector3::new(0.0, 1.0, 0.0),
        plane_normal: Vector3::new(0.0, 0.0, 1.0),
        chain_point: Vector3::new(1.0, 1.0, 0.0),
        concave,
        arc: vec![0, 1],
        axis: Vector3::ZERO,
        centre: Vector3::ZERO,
        radius: 1.0,
    }
}

#[test]
fn rust_only_a_concave_rows_material_centre_is_mirrored_through_its_chain_point() {
    assert_eq!(ball_row(false).material_centre(), Vector3::ZERO);
    assert_eq!(
        ball_row(true).material_centre(),
        Vector3::new(2.0, 2.0, 0.0)
    );
}

#[test]
fn rust_only_cross_tangents_are_laid_into_the_crease_planes() {
    let z = Vector3::new(0.0, 0.0, 1.0);
    let tangent = Vector3::new(1.0, 0.0, 1.0).get_normal();
    assert_eq!(
        corner_cap_hole_fill::out_of_the_planes(tangent, &[z, z]),
        Vector3::new(1.0, 0.0, 0.0)
    );
    let x = Vector3::new(1.0, 0.0, 0.0);
    let slant = Vector3::new(1.0, 1.0, 0.0).get_normal();
    let along_z = corner_cap_hole_fill::out_of_the_planes(Vector3::new(1.0, 1.0, 1.0), &[x, slant]);
    assert!(near(along_z, z, 1e-15));
    assert_eq!(
        corner_cap_hole_fill::out_of_the_planes(x, &[x]),
        Vector3::ZERO
    );
    assert_eq!(corner_cap_hole_fill::faded(tangent, &[z], 0.0), tangent);
    assert!(near(
        corner_cap_hole_fill::faded(tangent, &[z], 1.0),
        x,
        1e-15
    ));
    // The crease end takes the whole lay, the far end none.
    let first = corner_cap_hole_fill::inside_the_creases(tangent, 0, 3, &[z], &[]);
    let last = corner_cap_hole_fill::inside_the_creases(tangent, 2, 3, &[z], &[]);
    assert!(near(first, x, 1e-15));
    assert_eq!(last, tangent);
    // With creases at both ends the middle keeps its own.
    let middle = corner_cap_hole_fill::inside_the_creases(tangent, 1, 3, &[z], &[z]);
    assert_eq!(middle, tangent);
    assert!(corner_cap_hole_fill::leaves_through_convex_crease(x, x, x));
    assert!(corner_cap_hole_fill::leaves_through_convex_crease(-x, x, x));
    assert!(!corner_cap_hole_fill::leaves_through_convex_crease(x, z, x));
    assert!(!corner_cap_hole_fill::leaves_through_convex_crease(x, x, z));
    let face = LevelLineTriangle {
        a: 1,
        b: 2,
        c: 3,
        tag: 0,
    };
    assert!(corner_cap_hole_fill::runs_forward(face, 3, 1));
    assert!(!corner_cap_hole_fill::runs_forward(face, 1, 3));
}

#[test]
fn rust_only_a_patch_on_one_ball_is_placed_on_it_unless_it_folds() {
    let points = [
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
    ];
    let mut vertices = points.to_vec();
    vertices.push(Vector3::new(0.5, 0.5, 0.5));
    let fan = vec![
        HoleFillTriangle::new(0, 1, 3),
        HoleFillTriangle::new(1, 2, 3),
        HoleFillTriangle::new(2, 0, 3),
    ];
    let filled = HoleFillResult::new(HoleFillFailure::default(), vertices.clone(), fan.clone());
    let placed = corner_cap_hole_fill::on_the_ball(filled, &points, &[ball_row(false)]);
    assert!(near(
        placed.vertices[3],
        Vector3::new(1.0, 1.0, 1.0).get_normal(),
        1e-15
    ));
    assert_eq!(placed.triangles, fan);

    let mut folded = fan.clone();
    folded[0] = HoleFillTriangle::new(1, 0, 3);
    assert!(!corner_cap_hole_fill::covers_the_ball_once(
        Vector3::ZERO,
        &vertices,
        3,
        &folded
    ));
    let kept = HoleFillResult::new(HoleFillFailure::default(), vertices.clone(), folded);
    let as_filled = corner_cap_hole_fill::on_the_ball(kept.clone(), &points, &[ball_row(false)]);
    assert_eq!(as_filled, kept);
    // Rows of mixed sense are no one ball.
    let mixed = HoleFillResult::new(HoleFillFailure::default(), vertices, fan);
    let unchanged = corner_cap_hole_fill::on_the_ball(
        mixed.clone(),
        &points,
        &[ball_row(false), ball_row(true)],
    );
    assert_eq!(unchanged, mixed);
}
