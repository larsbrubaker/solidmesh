//! Face colours through nested booleans: a result that already wears two colours, fed into
//! further booleans, keeps each colour on exactly the faces that came from it. The shapes are a
//! case: a pink shell with a floor, green bosses standing on that floor, then holes cut and boxes
//! added on top of the mixed-colour result. A colour may never show up on a plane where no face
//! of that colour ever was.

use polygon_mesh::csg::{boolean_processing, BooleanOptions, CsgModes};
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Color;
use vector_math::{Matrix4X4, Vector3};

const PINK: Color = Color::rgb(0xFE, 0x8D, 0x86);
const GREEN: Color = Color::rgb(0xB0, 0xFE, 0x86);

/// The inner floor of the shell, which bosses stand on.
const FLOOR_Z: f64 = 2.0;

/// A closed cylinder along Z, built with shared vertex indices so it is manifold by construction.
fn cylinder(radius: f64, bottom: f64, top: f64, segments: usize) -> Mesh {
    let mut vertices = vec![Vector3::new(0.0, 0.0, bottom), Vector3::new(0.0, 0.0, top)];
    for i in 0..segments {
        let angle = std::f64::consts::TAU * i as f64 / segments as f64;
        let (y, x) = angle.sin_cos();
        vertices.push(Vector3::new(x * radius, y * radius, bottom));
        vertices.push(Vector3::new(x * radius, y * radius, top));
    }
    let mut indices = Vec::new();
    for i in 0..segments {
        let j = (i + 1) % segments;
        let (b0, t0, b1, t1) = (2 + 2 * i, 3 + 2 * i, 2 + 2 * j, 3 + 2 * j);
        indices.extend([0, b1, b0]);
        indices.extend([1, t0, t1]);
        indices.extend([b0, b1, t1]);
        indices.extend([b0, t1, t0]);
    }
    let indices: Vec<i32> = indices.into_iter().map(|i| i as i32).collect();
    Mesh::from_vector3_list_and_indices(&vertices, &indices)
}

/// A green cylinder placed in the scene, remembered so the test can say where its faces are.
struct PlacedCylinder {
    mesh: Mesh,
    x: f64,
    y: f64,
    radius: f64,
    bottom: f64,
    top: f64,
}

fn placed(x: f64, y: f64, radius: f64, bottom: f64, top: f64) -> PlacedCylinder {
    let mut mesh = cylinder(radius, bottom, top, 40);
    mesh.translate(Vector3::new(x, y, 0.0));
    PlacedCylinder {
        mesh,
        x,
        y,
        radius,
        bottom,
        top,
    }
}

fn box_at(size: Vector3, center: Vector3) -> Mesh {
    let mut mesh = create_cube(size.x, size.y, size.z);
    mesh.translate(center);
    mesh
}

fn run(operation: CsgModes, items: &[(&Mesh, Color)]) -> Mesh {
    let options = BooleanOptions {
        mesh_colors: Some(items.iter().map(|(_, color)| *color).collect()),
        ..BooleanOptions::default()
    };
    let operands: Vec<(&Mesh, Matrix4X4)> = items
        .iter()
        .map(|(mesh, _)| (*mesh, Matrix4X4::IDENTITY))
        .collect();
    boolean_processing::do_array(&operands, operation, &CancelToken::new(), &options)
        .expect("the boolean runs")
}

fn face_normal(mesh: &Mesh, face: usize) -> Vector3 {
    let [a, b, c] = corners(mesh, face);
    (b - a).cross(c - a).get_normal()
}

fn corners(mesh: &Mesh, face: usize) -> [Vector3; 3] {
    let face = mesh.faces[face];
    [face.v0, face.v1, face.v2].map(|v| Vector3::from_vector3_float(mesh.vertices[v as usize]))
}

/// Whether a face lies on a plane of one of the cylinders: on its side wall (horizontal normal,
/// every corner on its radius) or on its top or bottom cap's plane.
fn on_a_cylinder_face(mesh: &Mesh, face: usize, cylinders: &[PlacedCylinder]) -> bool {
    let normal = face_normal(mesh, face);
    let points = corners(mesh, face);
    cylinders.iter().any(|cylinder| {
        if normal.z.abs() < 1e-3 {
            points.iter().all(|p| {
                let r = ((p.x - cylinder.x).powi(2) + (p.y - cylinder.y).powi(2)).sqrt();
                (r - cylinder.radius).abs() < 1e-3
            })
        } else {
            points
                .iter()
                .all(|p| (p.z - cylinder.top).abs() < 1e-4 || (p.z - cylinder.bottom).abs() < 1e-4)
        }
    })
}

fn assert_floor_is_pink_and_green_is_on_cylinders(result: &Mesh, cylinders: &[PlacedCylinder]) {
    let colors = result
        .face_colors
        .as_deref()
        .expect("the result has face colours");
    assert_eq!(colors.len(), result.faces.len());
    let mut floor_faces = 0;
    let mut green_faces = 0;
    for (face, color) in colors.iter().enumerate() {
        let normal = face_normal(result, face);
        let on_floor = normal.z > 1.0 - 1e-6
            && corners(result, face)
                .iter()
                .all(|p| (p.z - FLOOR_Z).abs() < 1e-4);
        if on_floor {
            floor_faces += 1;
            assert_eq!(
                *color, PINK,
                "face {face} on the inner floor is the shell's colour"
            );
        }
        if *color == GREEN {
            green_faces += 1;
            assert!(
                on_a_cylinder_face(result, face, cylinders),
                "green face {face} lies on a plane of a green cylinder: {:?}",
                corners(result, face)
            );
        }
        assert!(
            *color == PINK || *color == GREEN,
            "face {face} wears a colour from nowhere: {color:?}"
        );
    }
    assert!(floor_faces > 0, "the inner floor is still there");
    assert!(green_faces > 0, "the bosses are still there");
}

#[test]
fn nested_booleans_paint_the_floor_with_the_shell_colour() {
    // A 60 x 60 x 20 shell with a 2 mm floor and 2 mm walls.
    let outer = box_at(Vector3::new(60.0, 60.0, 20.0), Vector3::new(0.0, 0.0, 10.0));
    let inner = box_at(
        Vector3::new(56.0, 56.0, 20.0),
        Vector3::new(0.0, 0.0, FLOOR_Z + 10.0),
    );
    let shell = run(CsgModes::Subtract, &[(&outer, PINK), (&inner, PINK)]);

    // Bosses standing on the floor: two sunk into it, two resting exactly on it.
    let bosses = [
        placed(-20.0, -20.0, 3.0, 1.0, 8.0),
        placed(20.0, 20.0, 3.0, 1.0, 8.0),
        placed(20.0, -20.0, 3.0, FLOOR_Z, 8.0),
        placed(-20.0, 20.0, 3.0, FLOOR_Z, 8.0),
    ];
    let mut level1: Vec<(&Mesh, Color)> = vec![(&shell, PINK)];
    level1.extend(bosses.iter().map(|boss| (&boss.mesh, GREEN)));
    let case_solid = run(CsgModes::Union, &level1);

    // Pilot holes down the bosses, a hole through the floor, and a notch out of one wall.
    let holes = [
        placed(-20.0, -20.0, 1.25, 4.0, 30.0),
        placed(20.0, 20.0, 1.25, 4.0, 30.0),
        placed(5.0, -7.0, 2.5, -5.0, 30.0),
    ];
    let notch = box_at(Vector3::new(10.0, 6.0, 6.0), Vector3::new(0.0, 29.0, 15.0));
    let mut level2: Vec<(&Mesh, Color)> = vec![(&case_solid, PINK)];
    level2.extend(holes.iter().map(|hole| (&hole.mesh, GREEN)));
    level2.push((&notch, PINK));
    let cut = run(CsgModes::Subtract, &level2);

    // A corner fill standing on the floor, and a green re-pilot sleeve.
    let fill = box_at(
        Vector3::new(6.0, 6.0, 6.0),
        Vector3::new(25.0, -5.0, FLOOR_Z + 3.0),
    );
    let sleeve = placed(-8.0, 10.0, 2.0, 1.0, 6.0);
    let finished = run(
        CsgModes::Union,
        &[(&cut, PINK), (&fill, PINK), (&sleeve.mesh, GREEN)],
    );

    let mut cylinders: Vec<PlacedCylinder> = bosses.into_iter().collect();
    cylinders.extend(holes);
    cylinders.push(sleeve);
    assert_floor_is_pink_and_green_is_on_cylinders(&finished, &cylinders);
}

#[test]
fn a_two_colour_plane_cut_across_its_seam_keeps_each_side() {
    // A slab whose left half is pink and right half green, as one mesh with per-face colours:
    // the union of two touching boxes, each in its own colour.
    let left = box_at(Vector3::new(20.0, 20.0, 4.0), Vector3::new(-10.0, 0.0, 2.0));
    let right = box_at(Vector3::new(20.0, 20.0, 4.0), Vector3::new(10.0, 0.0, 2.0));
    let slab = run(CsgModes::Union, &[(&left, PINK), (&right, GREEN)]);
    assert!(slab.face_colors.is_some());

    // A hole straddling the seam retriangulates the flat top on both sides of it.
    let hole = cylinder(4.0, -1.0, 5.0, 48);
    let cut = run(CsgModes::Subtract, &[(&slab, PINK), (&hole, PINK)]);
    let colors = cut
        .face_colors
        .as_deref()
        .expect("the cut has face colours");
    let mut seen = [false; 2];
    for (face, color) in colors.iter().enumerate() {
        let normal = face_normal(&cut, face);
        if normal.z.abs() < 1.0 - 1e-6 {
            continue;
        }
        let points = corners(&cut, face);
        let centroid_x = (points[0].x + points[1].x + points[2].x) / 3.0;
        let expected = if centroid_x < 0.0 { PINK } else { GREEN };
        seen[usize::from(centroid_x >= 0.0)] = true;
        assert_eq!(
            *color, expected,
            "flat face {face} at x {centroid_x} keeps its side's colour"
        );
    }
    assert_eq!(
        seen,
        [true, true],
        "both halves of the slab are still there"
    );
}
