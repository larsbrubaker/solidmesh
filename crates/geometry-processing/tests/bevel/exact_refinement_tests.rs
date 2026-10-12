//! Ports `Tests/GeometryProcessingTests/LevelLine/ExactRefinementTests.cs`: the refinement by
//! asking the chain field where its line runs - the exact least queries against brute-force
//! sampling, and whole cuts on a cube and on a big face. C# `[Arguments]` rows are loops over the
//! same-named test.

use std::rc::Rc;
use std::time::Instant;

use geometry_processing::bevel::level_line::{ChainField, LevelLineMesh, ScalarField};
use vector_math::dotnet_random::DotNetRandom;
use vector_math::{cs_math, Matrix4X4, Vector3};

use crate::level_line_test_geometry::{
    cube, is_watertight, run_cut, signed_volume, subdivided_plane, surface_area,
    two_triangle_square,
};

const SNAP: f64 = 1e-9;

#[test]
fn the_least_value_over_a_segment_and_over_a_triangle_agrees_with_brute_force_sampling() {
    for one_setback_all_along in [true, false] {
        let mut random = DotNetRandom::new(20260918);
        for _ in 0..12 {
            let field = random_chain(&mut random, one_setback_all_along);
            let a = random_point(&mut random);
            let b = random_point(&mut random);
            let c = random_point(&mut random);

            let on_segment = field.min_along_segment(a, b).expect("no budget");
            let mut sampled_segment = f64::INFINITY;
            const STEPS: i32 = 2000;
            for i in 0..=STEPS {
                sampled_segment = cs_math::min_f64(
                    sampled_segment,
                    field
                        .value_at(Vector3::lerp(a, b, i as f64 / STEPS as f64))
                        .expect("no budget"),
                );
            }

            let over_triangle = field.min_over_triangle(a, b, c).expect("no budget");
            let mut sampled_triangle = f64::INFINITY;
            const ROWS: i32 = 80;
            for i in 0..=ROWS {
                for j in 0..=(ROWS - i) {
                    let u = i as f64 / ROWS as f64;
                    let v = j as f64 / ROWS as f64;
                    sampled_triangle = cs_math::min_f64(
                        sampled_triangle,
                        field
                            .value_at(a + ((b - a) * u) + ((c - a) * v))
                            .expect("no budget"),
                    );
                }
            }

            assert!(
                on_segment.value <= sampled_segment + 1e-12,
                "the minimum is a bound on the field: a sample can never beat it"
            );
            assert!(over_triangle.value <= sampled_triangle + 1e-12);
            assert!(
                over_triangle.value <= on_segment.value + 1e-12,
                "the triangle contains the segment"
            );

            if !one_setback_all_along {
                continue;
            }

            assert!(
                on_segment.value > sampled_segment - 1e-2,
                "with one setback the minimum is the field's own, which the samples come close to"
            );
            assert!(over_triangle.value > sampled_triangle - 0.05);
            assert!(
                (field.value_at(on_segment.point).expect("no budget") - on_segment.value).abs()
                    <= 1e-9,
                "the point handed back is where the minimum stands"
            );
            assert!(
                (field.value_at(over_triangle.point).expect("no budget") - over_triangle.value)
                    .abs()
                    <= 1e-9
            );
        }
    }
}

#[test]
fn a_straight_line_across_a_twelve_triangle_cube_costs_only_its_crossings() {
    let mesh = cube(1.0);
    let volume_before = signed_volume(&mesh);
    let area_before = surface_area(&mesh);

    let field = straight_chain(
        Vector3::new(-10.0, 3.0, -1.0),
        Vector3::new(10.0, 3.0, -1.0),
        2.5,
    );

    let (reading, cut, refined) = run_cut(mesh, field, SNAP, 0.05);
    let mesh = reading.mesh.borrow();

    println!(
        "straight line across a cube: 12 triangles in, {} out",
        mesh.triangle_count()
    );

    assert!(refined);
    assert!(
        mesh.triangle_count() < 100,
        "the line is asked of the field, not hunted by halving the cube's faces"
    );
    assert!(is_watertight(&mesh));
    assert!((signed_volume(&mesh) - volume_before).abs() <= 1e-9);
    assert!((surface_area(&mesh) - area_before).abs() <= 1e-9);
    assert_eq!(cut.lines.len(), 1);
    assert!(cut.lines[0].closed);
}

#[test]
fn a_chain_along_one_cube_edge_cuts_both_faces_with_straight_lines() {
    const SETBACK: f64 = 0.3;
    let field = straight_chain(
        Vector3::new(-4.0, -1.0, -1.0),
        Vector3::new(4.0, -1.0, -1.0),
        SETBACK,
    );

    let (reading, cut, refined) = run_cut(cube(1.0), field, SNAP, 0.02);
    let mesh = reading.mesh.borrow();

    assert!(refined);
    assert!(is_watertight(&mesh));
    assert_eq!(cut.lines.len(), 1);
    assert!(
        cut.lines[0].closed,
        "the line runs round the cube and closes"
    );

    let on_line: Vec<Vector3> = cut.lines[0]
        .vertices
        .iter()
        .map(|&v| mesh.position(v))
        .collect();
    let on_bottom: Vec<Vector3> = on_line
        .iter()
        .copied()
        .filter(|p| (p.z + 1.0).abs() < 1e-9)
        .collect();
    let on_side: Vec<Vector3> = on_line
        .iter()
        .copied()
        .filter(|p| (p.y + 1.0).abs() < 1e-9)
        .collect();

    assert!(on_bottom.len() >= 2);
    assert!(on_side.len() >= 2);
    for p in on_bottom {
        assert!(
            (p.y - (-1.0 + SETBACK)).abs() <= 1e-9,
            "on the bottom face the level line is the straight line one setback in from the edge"
        );
    }

    for p in on_side {
        assert!((p.z - (-1.0 + SETBACK)).abs() <= 1e-9);
    }
}

#[test]
fn a_hole_rim_on_a_big_face_gives_a_rounded_square_offset_to_the_chord_tolerance() {
    for (face_size, chord_tolerance) in [(20.0, 0.02), (200.0, 0.02)] {
        let centre = Vector3::new(face_size / 2.0, face_size / 2.0, 0.0);
        let field = square_rim(centre, 1.0, 1.0);

        let clock = Instant::now();
        let (reading, cut, refined) =
            run_cut(two_triangle_square(face_size), field, SNAP, chord_tolerance);
        let elapsed = clock.elapsed();
        let mesh = reading.mesh.borrow();

        println!(
            "hole rim on a {face_size} face at chord tolerance {chord_tolerance}: {} line vertices, {} triangles, {:.1} ms",
            cut.lines[0].vertices.len(),
            mesh.triangle_count(),
            elapsed.as_secs_f64() * 1000.0
        );

        assert!(refined);
        assert_eq!(cut.lines.len(), 1);
        assert!(cut.lines[0].closed, "a rim's offset is a loop");

        let lp: Vec<Vector3> = cut.lines[0]
            .vertices
            .iter()
            .map(|&v| mesh.position(v))
            .collect();
        for i in 0..lp.len() {
            let next = lp[(i + 1) % lp.len()];
            assert!(
                reading.value_at_point(lp[i]).expect("no budget").abs() <= chord_tolerance,
                "a line vertex is a root of the field"
            );
            assert!(
                reading
                    .value_at_point(0.5 * (lp[i] + next))
                    .expect("no budget")
                    .abs()
                    <= chord_tolerance,
                "the chord between two line vertices is what the chord tolerance bounds"
            );
        }
    }
}

#[test]
fn the_rims_cost_follows_the_chord_tolerance_and_not_the_face_size() {
    let coarse = rim_line_vertices(20.0, 0.04);
    let fine = rim_line_vertices(20.0, 0.01);
    let coarse_on_a_big_face = rim_line_vertices(200.0, 0.04);

    println!(
        "rim line vertices: coarse {coarse}, fine {fine}, coarse on a ten times bigger face {coarse_on_a_big_face}"
    );

    let growth = fine as f64 / coarse as f64;
    assert!(
        growth > 1.3,
        "four times the accuracy asks for about twice the points"
    );
    assert!(
        growth < 3.0,
        "and no more than that - the cost is the sagitta's law"
    );
    assert!(
        (coarse_on_a_big_face - coarse).abs() <= 2,
        "the line is the rim's offset; the face it is drawn on can be any size at all"
    );
}

#[test]
fn the_cut_is_the_same_after_a_rigid_motion_of_the_part() {
    let motion = Matrix4X4::create_rotation(Vector3::new(0.4, -0.7, 0.2))
        * Matrix4X4::create_translation(3.0, -5.0, 11.0);

    let still_field = straight_chain(
        Vector3::new(-4.0, -1.0, -1.0),
        Vector3::new(4.0, -1.0, -1.0),
        0.3,
    );
    let (still, still_cut, _) = run_cut(cube(1.0), still_field, SNAP, 0.02);

    let moved_field: Rc<dyn ScalarField> = Rc::new(
        ChainField::from_polyline(
            &[
                Vector3::new(-4.0, -1.0, -1.0).transform(motion),
                Vector3::new(4.0, -1.0, -1.0).transform(motion),
            ],
            false,
            &[0.3, 0.3],
        )
        .expect("a chain"),
    );
    let (moved, moved_cut, _) = run_cut(moved_cube(motion), moved_field, SNAP, 0.02);
    let still = still.mesh.borrow();
    let moved = moved.mesh.borrow();

    assert_eq!(moved.triangle_count(), still.triangle_count());
    assert_eq!(moved.vertex_count(), still.vertex_count());
    assert_eq!(moved_cut.lines.len(), still_cut.lines.len());
    assert_eq!(
        moved_cut.inside_triangles.len(),
        still_cut.inside_triangles.len()
    );

    let back = motion.inverted();
    for v in 0..still.vertex_count() {
        let unmoved = moved.position(v).transform(back);
        assert!((unmoved - still.position(v)).length() < 1e-9);
    }
}

#[test]
fn two_runs_of_the_same_cut_agree_exactly() {
    let (first, first_cut, _) = run_cut(
        two_triangle_square(20.0),
        square_rim(Vector3::new(10.0, 10.0, 0.0), 1.0, 1.0),
        SNAP,
        0.02,
    );
    let (second, second_cut, _) = run_cut(
        two_triangle_square(20.0),
        square_rim(Vector3::new(10.0, 10.0, 0.0), 1.0, 1.0),
        SNAP,
        0.02,
    );
    let first = first.mesh.borrow();
    let second = second.mesh.borrow();

    assert_eq!(second.vertex_count(), first.vertex_count());
    for v in 0..first.vertex_count() {
        assert!(second.position(v) == first.position(v));
    }

    // C#'s `IsEquivalentTo`: the same lines, in any order.
    let names = |lines: &[geometry_processing::bevel::level_line::LevelLineChain]| {
        let mut names: Vec<String> = lines
            .iter()
            .map(|l| {
                l.vertices
                    .iter()
                    .map(|v| v.to_string())
                    .collect::<Vec<_>>()
                    .join(",")
            })
            .collect();
        names.sort();
        names
    };
    assert_eq!(names(&second_cut.lines), names(&first_cut.lines));
}

#[test]
fn a_cut_on_a_twenty_thousand_triangle_plane_is_timed() {
    let mesh = subdivided_plane(100, 100.0);
    assert_eq!(mesh.triangle_count(), 20000);
    let field = square_rim(Vector3::new(50.0, 50.0, 0.0), 20.0, 25.0);

    let clock = Instant::now();
    let (reading, cut, refined) = run_cut(mesh, field, 1e-7, 0.5);
    let elapsed = clock.elapsed();

    println!(
        "chain field cut over {} triangles took {:.1} ms and left {} line(s)",
        reading.mesh.borrow().triangle_count(),
        elapsed.as_secs_f64() * 1000.0,
        cut.lines.len()
    );

    assert!(refined);
    assert_eq!(cut.lines.len(), 1);
    assert!(
        elapsed.as_secs_f64() < 60.0,
        "a loose ceiling only, so a hundredfold regression fails the suite rather than hanging it"
    );
}

fn rim_line_vertices(face_size: f64, chord_tolerance: f64) -> i32 {
    let field = square_rim(
        Vector3::new(face_size / 2.0, face_size / 2.0, 0.0),
        1.0,
        1.0,
    );
    let (_, cut, _) = run_cut(two_triangle_square(face_size), field, SNAP, chord_tolerance);
    cut.lines[0].vertices.len() as i32
}

fn straight_chain(from: Vector3, to: Vector3, setback: f64) -> Rc<dyn ScalarField> {
    Rc::new(ChainField::from_polyline(&[from, to], false, &[setback, setback]).expect("a chain"))
}

fn square_rim(centre: Vector3, half: f64, setback: f64) -> Rc<dyn ScalarField> {
    let points = [
        centre + Vector3::new(-half, -half, 0.0),
        centre + Vector3::new(half, -half, 0.0),
        centre + Vector3::new(half, half, 0.0),
        centre + Vector3::new(-half, half, 0.0),
    ];
    Rc::new(ChainField::from_polyline(&points, true, &[setback; 4]).expect("a chain"))
}

fn random_chain(random: &mut DotNetRandom, one_setback_all_along: bool) -> ChainField {
    let count = 2 + random.next(3);
    let only = 0.2 + (random.next_double() * 0.8);
    let mut points = Vec::new();
    let mut setbacks = Vec::new();
    for _ in 0..count {
        points.push(random_point(random));
        setbacks.push(if one_setback_all_along {
            only
        } else {
            0.2 + (random.next_double() * 0.8)
        });
    }

    // C# evaluates `count > 2 && random.Next(2) == 0` with short-circuit: no draw for two points.
    let closed = count > 2 && random.next(2) == 0;
    ChainField::from_polyline(&points, closed, &setbacks).expect("a chain")
}

fn random_point(random: &mut DotNetRandom) -> Vector3 {
    let x = (random.next_double() * 4.0) - 2.0;
    let y = (random.next_double() * 4.0) - 2.0;
    let z = (random.next_double() * 4.0) - 2.0;
    Vector3::new(x, y, z)
}

fn moved_cube(motion: Matrix4X4) -> LevelLineMesh {
    let reference = cube(1.0);
    let positions: Vec<Vector3> = (0..reference.vertex_count())
        .map(|v| reference.position(v).transform(motion))
        .collect();
    let indices: Vec<(i32, i32, i32)> = reference
        .triangles()
        .iter()
        .map(|t| (t.a, t.b, t.c))
        .collect();
    LevelLineMesh::from_triangles(&positions, &indices, None).expect("a mesh")
}
