//! Ports `Tests/GeometryProcessingTests/LevelLine/BandCutTests.cs`: the band cut - conforming
//! splits, the refinement, the roots and the partition - on hand-built solids and planes with
//! analytic fields. C# `[Arguments]` rows are loops over the same-named test.

use std::cell::RefCell;
use std::rc::Rc;
use std::time::Instant;

use geometry_processing::bevel::level_line::band_refinement::{refine, RefineOptions};
use geometry_processing::bevel::level_line::{
    LevelLineChain, LevelLineMesh, ScalarField, VertexFieldReading, VertexSide,
};
use vector_math::{cs_math, Matrix4X4, Vector3};

use crate::level_line_test_geometry::{
    cube, cylinder, is_watertight, line_points, run_cut, shortest_side, signed_volume,
    smallest_triangle_area, subdivided_plane, surface_area, two_triangle_square, BallField,
    CapsuleField, MovedField, PlaneField,
};

const SNAP: f64 = 1e-9;

#[test]
fn splitting_an_edge_splits_every_triangle_standing_on_it() {
    let mut mesh = two_triangle_square(1.0);
    let before = surface_area(&mesh);

    let mid = mesh
        .split_edge(0, 2, Vector3::new(0.5, 0.5, 0.0), true)
        .expect("a split");

    assert_eq!(
        mesh.triangle_count(),
        4,
        "both triangles on the diagonal are split, not just the one asked about"
    );
    assert!(
        !mesh.has_edge(0, 2),
        "the split edge is gone, replaced by its two halves"
    );
    assert_eq!(mesh.triangles_on(0, mid).len(), 2);
    assert_eq!(mesh.triangles_on(mid, 2).len(), 2);
    assert!(
        (surface_area(&mesh) - before).abs() <= 1e-12,
        "a split moves no material"
    );
}

#[test]
fn children_of_a_split_inherit_their_face_tag() {
    let mut mesh = LevelLineMesh::from_triangles(
        &[
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(1.0, 1.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ],
        &[(0, 1, 2), (0, 2, 3)],
        Some(&[7, 9]),
    )
    .expect("a mesh");

    mesh.split_edge(0, 2, Vector3::new(0.5, 0.5, 0.0), true)
        .expect("a split");

    assert_eq!(mesh.triangles().iter().filter(|t| t.tag == 7).count(), 2);
    assert_eq!(mesh.triangles().iter().filter(|t| t.tag == 9).count(), 2);
}

#[test]
fn the_cut_leaves_the_same_closed_solid() {
    for (shape, offset, chord_tolerance) in [
        ("cube", 0.3, 0.1),
        ("cube", -0.55, 0.1),
        ("cylinder", 0.2, 0.1),
    ] {
        let mesh = if shape == "cube" {
            cube(1.0)
        } else {
            cylinder(24, 1.0, 2.0)
        };
        let volume_before = signed_volume(&mesh);
        let area_before = surface_area(&mesh);
        let field = Rc::new(PlaneField::new(Vector3::new(1.0, 0.0, 0.0), offset));

        let (reading, cut, refined) = run_cut(mesh, field, SNAP, chord_tolerance);
        let mesh = reading.mesh.borrow();

        assert!(
            refined,
            "the refinement must settle, never give up, on a shape this plain"
        );
        assert!(
            is_watertight(&mesh),
            "a conforming split leaves every edge with exactly two triangles, so the solid is still closed"
        );
        assert!((signed_volume(&mesh) - volume_before).abs() <= 1e-9);
        assert!((surface_area(&mesh) - area_before).abs() <= 1e-9);
        assert!(
            smallest_triangle_area(&mesh) > 0.0,
            "no triangle may be left degenerate"
        );
        assert!(
            shortest_side(&mesh) > SNAP,
            "a root within the snap of an end puts that END on the line instead of cutting a sliver"
        );
        assert_eq!(
            cut.inside_triangles.len() + cut.outside_triangles.len(),
            mesh.triangle_count() as usize,
            "every triangle lands on exactly one side"
        );
        assert_eq!(cut.lines.len(), 1);
        assert!(
            cut.lines[0].closed,
            "a plane through a closed solid leaves a closed loop"
        );
    }
}

#[test]
fn the_line_and_its_chords_stand_on_the_zero_of_the_field() {
    for chord_tolerance in [0.1, 0.03] {
        let field = Rc::new(BallField::new(Vector3::new(0.0, 0.0, -1.0), 0.9));

        let (reading, cut, refined) = run_cut(cylinder(16, 1.0, 2.0), field, SNAP, chord_tolerance);

        assert!(refined);
        assert!(!cut.lines.is_empty());
        let worst = line_points(&reading.mesh.borrow(), &cut)
            .into_iter()
            .map(|p| reading.value_at_point(p).expect("no budget").abs())
            .fold(f64::NEG_INFINITY, cs_math::max_f64);
        assert!(
            worst <= chord_tolerance,
            "a line vertex is a root of the field, and a chord midpoint is what the chord tolerance bounds"
        );
    }
}

#[test]
fn a_line_through_mesh_vertices_uses_them_and_cuts_no_sliver() {
    let mesh = subdivided_plane(4, 1.0);
    let triangles_before = mesh.triangle_count();
    let vertices_before = mesh.vertex_count();
    let field = Rc::new(PlaneField::new(Vector3::new(1.0, 0.0, 0.0), 0.5));

    let (reading, cut, refined) = run_cut(mesh, field, SNAP, 0.1);

    assert!(refined);
    assert_eq!(
        reading.mesh.borrow().vertex_count(),
        vertices_before,
        "the line already has vertices to run through, so it makes none"
    );
    assert_eq!(reading.mesh.borrow().triangle_count(), triangles_before);
    assert_eq!(cut.lines.len(), 1);
    assert_eq!(
        cut.lines[0].vertices.len(),
        5,
        "the line is exactly the column of five grid vertices at x = 0.5"
    );
    for &v in &cut.lines[0].vertices {
        assert_eq!(reading.side_of(v).ok(), Some(VertexSide::On));
        assert!((reading.mesh.borrow().position(v).x - 0.5).abs() <= 1e-12);
    }
}

#[test]
fn a_small_loop_hidden_inside_one_triangle_pair_is_found_and_closed() {
    let field = Rc::new(BallField::new(Vector3::new(0.5, 0.5, 0.0), 0.2));

    let (reading, cut, refined) = run_cut(two_triangle_square(1.0), field, SNAP, 0.02);
    let mesh = reading.mesh.borrow();

    assert!(refined);
    assert_eq!(cut.lines.len(), 1);
    assert!(cut.lines[0].closed, "a circle has no ends");
    let radii: Vec<f64> = cut.lines[0]
        .vertices
        .iter()
        .map(|&v| (mesh.position(v) - Vector3::new(0.5, 0.5, 0.0)).length())
        .collect();
    assert!(
        radii
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, cs_math::max_f64)
            <= 0.2 + 1e-9
    );
    assert!(radii.iter().copied().fold(f64::INFINITY, cs_math::min_f64) >= 0.2 - 1e-9);
    assert!(
        !cut.inside_triangles.is_empty(),
        "the disc inside the circle is the band the fill will later delete"
    );
    let worst = line_points(&mesh, &cut)
        .into_iter()
        .map(|p| reading.value_at_point(p).expect("no budget").abs())
        .fold(f64::NEG_INFINITY, cs_math::max_f64);
    assert!(worst <= 0.02);
}

#[test]
fn a_field_that_touches_nothing_leaves_the_mesh_alone_and_draws_no_line() {
    let mesh = cube(1.0);
    let triangles_before = mesh.triangle_count();
    let field = Rc::new(PlaneField::new(Vector3::new(1.0, 0.0, 0.0), -12.0));

    let (reading, cut, refined) = run_cut(mesh, field, SNAP, 0.05);

    assert!(refined);
    assert_eq!(
        reading.mesh.borrow().triangle_count(),
        triangles_before,
        "nothing is near the line, so no triangle needs opening up"
    );
    assert_eq!(cut.lines.len(), 0);
    assert_eq!(cut.inside_triangles.len(), 0);
}

#[test]
fn the_cut_is_the_same_after_a_rigid_motion_of_the_part() {
    let motion = Matrix4X4::create_rotation(Vector3::new(0.4, -0.7, 0.2))
        * Matrix4X4::create_translation(3.0, -5.0, 11.0);
    let field: Rc<dyn ScalarField> = Rc::new(CapsuleField::new(
        Vector3::new(-2.0, 0.0, 0.4),
        Vector3::new(2.0, 0.0, 0.4),
        0.8,
    ));

    let (still, still_cut, _) = run_cut(cube(1.0), field.clone(), SNAP, 0.1);

    let (moved, moved_cut, _) = run_cut(
        moved_cube(motion),
        Rc::new(MovedField::new(field, motion)),
        SNAP,
        0.1,
    );
    let still = still.mesh.borrow();
    let moved = moved.mesh.borrow();

    assert_eq!(moved.triangle_count(), still.triangle_count());
    assert_eq!(moved_cut.lines.len(), still_cut.lines.len());
    assert_eq!(
        moved_cut.inside_triangles.len(),
        still_cut.inside_triangles.len()
    );

    let back = motion.inverted();
    for v in 0..still.vertex_count() {
        let unmoved = moved.position(v).transform(back);
        assert!(
            (unmoved - still.position(v)).length() < 1e-9,
            "every vertex the cut made lands in the same place on the part"
        );
    }
}

#[test]
fn the_mesh_names_its_edges_in_one_order_however_it_was_built() {
    let mut mesh = cylinder(24, 1.0, 2.0);
    assert!(is_ascending(&mesh.edges()), "as built");

    for edge in mesh.edges().into_iter().take(8) {
        let middle = 0.5 * (mesh.position(edge.0) + mesh.position(edge.1));
        mesh.split_edge(edge.0, edge.1, middle, true)
            .expect("a split");
    }

    mesh.remove_triangle(0);

    let after = mesh.edges();
    assert!(is_ascending(&after), "after splitting and removing");
    let mut sorted = after.clone();
    sorted.sort();
    assert_eq!(after, sorted);
}

fn is_ascending(edges: &[(i32, i32)]) -> bool {
    for i in 1..edges.len() {
        if edges[i].0 < edges[i - 1].0
            || (edges[i].0 == edges[i - 1].0 && edges[i].1 <= edges[i - 1].1)
        {
            return false;
        }
    }

    true
}

#[test]
fn two_runs_of_the_same_cut_agree_exactly() {
    let field = || Rc::new(BallField::new(Vector3::new(0.0, 0.0, 0.4), 1.1));
    let (first, first_cut, _) = run_cut(cylinder(24, 1.0, 2.0), field(), SNAP, 0.05);
    let (second, second_cut, _) = run_cut(cylinder(24, 1.0, 2.0), field(), SNAP, 0.05);
    let first = first.mesh.borrow();
    let second = second.mesh.borrow();

    assert_eq!(second.vertex_count(), first.vertex_count());
    assert_eq!(second.triangle_count(), first.triangle_count());
    for v in 0..first.vertex_count() {
        assert!(
            second.position(v) == first.position(v),
            "the same input has to give the same bits, not merely a near answer"
        );
    }

    // C#'s `IsEquivalentTo`: the same lines, in any order.
    assert_eq!(names(&second_cut.lines), names(&first_cut.lines));
}

fn names(lines: &[LevelLineChain]) -> Vec<String> {
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
}

#[test]
fn refinement_refuses_rather_than_looping_when_the_cap_is_reached() {
    let reading = VertexFieldReading::new(
        Rc::new(RefCell::new(two_triangle_square(1.0))),
        Rc::new(BallField::new(Vector3::new(0.5, 0.5, 0.0), 0.2)),
        SNAP,
    )
    .expect("a snap");

    let (settled, offending) = refine(
        &reading,
        &[0, 1],
        1e-4,
        RefineOptions {
            lipschitz: 1.0,
            max_depth: 3,
            topology_only: false,
        },
    )
    .expect("the refinement");

    assert!(
        !settled,
        "three halvings cannot resolve a curve at a ten thousandth"
    );
    assert!(offending >= 0, "a refusal names the triangle that beat it");
}

#[test]
fn a_cut_on_a_twenty_thousand_triangle_plane_is_timed() {
    let mesh = subdivided_plane(100, 100.0);
    assert_eq!(mesh.triangle_count(), 20000);
    let field = Rc::new(BallField::new(Vector3::new(50.0, 50.0, 0.0), 30.0));

    let clock = Instant::now();
    let (reading, cut, refined) = run_cut(mesh, field, 1e-7, 0.5);
    let elapsed = clock.elapsed();

    println!(
        "level line cut over {} triangles took {:.1} ms and left {} line(s)",
        reading.mesh.borrow().triangle_count(),
        elapsed.as_secs_f64() * 1000.0,
        cut.lines.len()
    );

    assert!(refined);
    assert_eq!(cut.lines.len(), 1);
    assert!(cut.lines[0].closed);
    assert!(
        elapsed.as_secs_f64() < 30.0,
        "a loose ceiling only, so a hundredfold regression fails the suite rather than hanging it"
    );
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
