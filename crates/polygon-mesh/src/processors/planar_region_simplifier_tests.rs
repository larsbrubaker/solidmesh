//! Planar region simplifier tests (unit tests, for the crate-private work counter and the
//! nearest-triangle query): the distance-field Dilate / Erode surface of a mechanical part is
//! mostly grid-sized triangles on flat walls; merging them must shed most triangles and leave
//! the surface, its closure and its volume exactly as they were.
//!
//! The `rust_only_*` tests check the same properties on hand-built meshes.

use std::collections::HashMap;

use solidmesh_math::{Vector2, Vector3Float};

use super::{simplify, simplify_counting_work};
use crate::g3::dmesh3::tri_distance_sqr;
use crate::g3::DMeshAABBTree3;
use crate::processors::outline_to_mesh::extrude_contours;
use crate::sdf::MeshSdfGrid;
use crate::solids::platonic_solids::{create_cube, create_icosahedron};
use crate::{CancelToken, Mesh};

/// An outline drawn as contours in mm: each move-to starts a contour, and a contour is closed
/// back to its first point.
#[derive(Default)]
struct Outline {
    contours: Vec<Vec<Vector2>>,
}

impl Outline {
    fn new() -> Self {
        Self::default()
    }

    fn move_to(&mut self, x: f64, y: f64) {
        self.move_to_vector(Vector2::new(x, y));
    }

    fn line_to(&mut self, x: f64, y: f64) {
        self.line_to_vector(Vector2::new(x, y));
    }

    fn move_to_vector(&mut self, p: Vector2) {
        self.contours.push(vec![p]);
    }

    fn line_to_vector(&mut self, p: Vector2) {
        match self.contours.last_mut() {
            Some(contour) => contour.push(p),
            None => self.contours.push(vec![p]),
        }
    }

    /// Ends the contour (it closes back to its first point on its own): a line-to after it starts
    /// the next one. The empty contour this leaves is skipped by the conversion to polygons.
    fn close_polygon(&mut self) {
        self.contours.push(Vec::new());
    }
}

fn block_with_hole_and_notch(iso: f64) {
    let mut outline = Outline::new();
    outline.move_to(0.0, 0.0);
    outline.line_to(12.0, 0.0);
    outline.line_to(12.0, 5.0);
    outline.line_to(18.0, 5.0);
    outline.line_to(18.0, 0.0);
    outline.line_to(30.0, 0.0);
    outline.line_to(30.0, 20.0);
    outline.line_to(0.0, 20.0);
    outline.close_polygon();
    add_circle(&mut outline, Vector2::new(8.0, 11.0), 4.0, 48);
    assert_simplifies(
        &format!("block {iso}"),
        extrude_part(&outline, 10.0),
        iso,
        4,
    );
}

#[test]
fn block_with_hole_and_notch_1() {
    block_with_hole_and_notch(1.0);
}

#[test]
fn block_with_hole_and_notch_minus_1() {
    block_with_hole_and_notch(-1.0);
}

#[test]
fn gear_like_extrusion() {
    let mut outline = Outline::new();
    const TEETH: i32 = 12;
    for i in 0..TEETH * 4 {
        let angle = i as f64 * std::f64::consts::PI * 2.0 / (TEETH * 4) as f64;
        let radius = if (i % 4) < 2 { 15.0 } else { 12.0 };
        let p = Vector2::new(angle.cos() * radius, angle.sin() * radius);
        if i == 0 {
            outline.move_to_vector(p);
        } else {
            outline.line_to_vector(p);
        }
    }

    outline.close_polygon();
    add_circle(&mut outline, Vector2::new(0.0, 0.0), 4.0, 32);
    assert_simplifies("gear", extrude_part(&outline, 6.0), 1.0, 2);
}

#[test]
fn l_bracket() {
    let mut outline = Outline::new();
    outline.move_to(0.0, 0.0);
    outline.line_to(40.0, 0.0);
    outline.line_to(40.0, 5.0);
    outline.line_to(5.0, 5.0);
    outline.line_to(5.0, 30.0);
    outline.line_to(0.0, 30.0);
    outline.close_polygon();
    assert_simplifies("L bracket", extrude_part(&outline, 20.0), 1.0, 4);
}

#[test]
fn round_part_is_left_almost_as_it_was() {
    // A near-point dilated far is a ball: no flat wall to merge.
    let sdf = MeshSdfGrid::new(create_cube(0.01, 0.01, 0.01), 5.0, 64).expect("valid grid");
    let source = sdf.extract(5.0, &CancelToken::new()).expect("extracts");
    let simplified = simplify(&source);
    println!(
        "sphere: {} -> {}",
        source.faces.len(),
        simplified.faces.len()
    );
    assert_same_solid_on_surface(&source, &simplified);
    assert!(simplified.faces.len() > source.faces.len() * 9 / 10);
}

#[test]
fn work_grows_linearly_with_the_mesh() {
    let mut outline = Outline::new();
    outline.move_to(0.0, 0.0);
    outline.line_to(30.0, 0.0);
    outline.line_to(30.0, 20.0);
    outline.line_to(0.0, 20.0);
    outline.close_polygon();
    add_circle(&mut outline, Vector2::new(8.0, 11.0), 4.0, 48);
    let single = MeshSdfGrid::new(extrude_part(&outline, 10.0), 1.0, 64)
        .expect("valid grid")
        .extract(1.0, &CancelToken::new())
        .expect("extracts");

    // Two copies with their own vertices: every region, boundary and edge twice over. They
    // share their place, so the bounds and the float rounding are the same as the single's.
    let doubled = doubled_in_place(&single);

    let (simplified_single, single_work) = simplify_counting_work(&single, 1e-4, 1e-7);
    let (simplified_doubled, doubled_work) = simplify_counting_work(&doubled, 1e-4, 1e-7);
    println!("work {single_work} -> {doubled_work}");
    assert_eq!(
        simplified_doubled.faces.len(),
        2 * simplified_single.faces.len()
    );
    assert!(doubled_work as f64 <= 2.5 * single_work as f64);
}

fn extrude_part(outline: &Outline, height: f64) -> Mesh {
    extrude_contours(&outline.contours, height).expect("the outline extrudes")
}

/// Two copies of `single`, each with its own vertices, in the same place.
fn doubled_in_place(single: &Mesh) -> Mesh {
    let mut doubled = Mesh::new();
    doubled.vertices.extend_from_slice(&single.vertices);
    doubled.vertices.extend_from_slice(&single.vertices);
    let offset = single.vertices.len() as i32;
    for face in single.faces.iter() {
        doubled
            .faces
            .add(face.v0, face.v1, face.v2, &doubled.vertices);
    }
    for face in single.faces.iter() {
        doubled.faces.add(
            face.v0 + offset,
            face.v1 + offset,
            face.v2 + offset,
            &doubled.vertices,
        );
    }
    doubled
}

fn add_circle(outline: &mut Outline, center: Vector2, radius: f64, sides: i32) {
    for i in 0..sides {
        let angle = i as f64 * std::f64::consts::PI * 2.0 / sides as f64;
        let p = center + Vector2::new(angle.cos() * radius, angle.sin() * radius);
        if i == 0 {
            outline.move_to_vector(p);
        } else {
            outline.line_to_vector(p);
        }
    }

    outline.close_polygon();
}

fn assert_simplifies(name: &str, part: Mesh, iso: f64, minimum_reduction: usize) {
    let source = MeshSdfGrid::new(part, iso.abs(), MeshSdfGrid::DEFAULT_RESOLUTION)
        .expect("valid grid")
        .extract(iso, &CancelToken::new())
        .expect("extracts");
    let simplified = simplify(&source);
    println!(
        "{name}: {} -> {}",
        source.faces.len(),
        simplified.faces.len()
    );
    assert_same_solid_on_surface(&source, &simplified);
    assert!(simplified.faces.len() * minimum_reduction < source.faces.len());
}

/// Closed, manifold, same volume, and every source vertex (kept or
/// dropped) still lies on the simplified surface.
fn assert_same_solid_on_surface(source: &Mesh, simplified: &Mesh) {
    assert!(simplified.is_manifold());
    assert_eq!(closed_edge_errors(simplified), 0);
    let volume = source.get_volume();
    assert!((simplified.get_volume() - volume).abs() <= 1e-6 * volume.abs());

    // The kept vertices are the source's own; every dropped one must still lie on the surface.
    let target = simplified.to_dmesh3();
    let tree = DMeshAABBTree3::new(&target);
    let mut worst = 0.0f64;
    for vertex in &source.vertices {
        let point = [vertex.x as f64, vertex.y as f64, vertex.z as f64];
        let triangle = tree.find_nearest_triangle(point, f64::MAX);
        let corners = target.get_tri_vertices(triangle as usize);
        worst = worst.max(tri_distance_sqr(&corners, point).sqrt());
    }

    println!("  worst dropped-vertex distance {worst:.2e}");
    assert!(worst <= 1e-5);
}

/// A welded `size` cube whose every side is tiled with `n` x `n` grid squares, two triangles each,
/// wound outwards: the shape of a distance-field wall.
fn tiled_box(size: f64, n: i32) -> Mesh {
    let mut mesh = Mesh::new();
    let mut index: HashMap<[i32; 3], i32> = HashMap::new();
    let mut vertex = |mesh: &mut Mesh, grid: [i32; 3]| -> i32 {
        *index.entry(grid).or_insert_with(|| {
            let step = size / n as f64;
            mesh.vertices.push(Vector3Float::new(
                grid[0] as f64 * step,
                grid[1] as f64 * step,
                grid[2] as f64 * step,
            ));
            mesh.vertices.len() as i32 - 1
        })
    };
    for axis in 0..3 {
        let (a_axis, b_axis) = ((axis + 1) % 3, (axis + 2) % 3);
        for side in [0, n] {
            let outward = if side == 0 { -1.0 } else { 1.0 };
            for a in 0..n {
                for b in 0..n {
                    let corner = |da: i32, db: i32| {
                        let mut grid = [0; 3];
                        grid[axis] = side;
                        grid[a_axis] = a + da;
                        grid[b_axis] = b + db;
                        grid
                    };
                    let quad = [corner(0, 0), corner(1, 0), corner(1, 1), corner(0, 1)];
                    let ids: Vec<i32> = quad.iter().map(|&g| vertex(&mut mesh, g)).collect();
                    // (a, b, axis) is right-handed, so 0-1-2 winds about +axis.
                    let (t0, t1) = if outward > 0.0 {
                        ([ids[0], ids[1], ids[2]], [ids[0], ids[2], ids[3]])
                    } else {
                        ([ids[0], ids[2], ids[1]], [ids[0], ids[3], ids[2]])
                    };
                    for t in [t0, t1] {
                        mesh.faces.add(t[0], t[1], t[2], &mesh.vertices);
                    }
                }
            }
        }
    }
    mesh
}

/// Half-edges without exactly one opposite partner.
fn closed_edge_errors(mesh: &Mesh) -> usize {
    let mut directed: HashMap<(i32, i32), i32> = HashMap::new();
    for face in mesh.faces.iter() {
        for edge in [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)] {
            *directed.entry(edge).or_insert(0) += 1;
        }
    }
    directed
        .iter()
        .filter(|(&(a, b), &count)| count != 1 || directed.get(&(b, a)).copied() != Some(1))
        .count()
}

/// The same solid (without the nearest-triangle distance check of
/// `assert_same_solid_on_surface`): closed, manifold, same volume, and every kept vertex is
/// one of the source's own.
fn assert_same_solid(source: &Mesh, simplified: &Mesh) {
    assert!(simplified.is_manifold());
    assert_eq!(closed_edge_errors(simplified), 0);
    let volume = source.get_volume();
    assert!((simplified.get_volume() - volume).abs() <= 1e-6 * volume.abs());
    for v in &simplified.vertices {
        assert!(source.vertices.contains(v));
    }
}

#[test]
fn rust_only_tiled_box_collapses_to_twelve_triangles() {
    let source = tiled_box(10.0, 6);
    assert_eq!(source.faces.len(), 6 * 6 * 6 * 2);
    let simplified = simplify(&source);
    assert_same_solid(&source, &simplified);
    // Each side keeps only its four corners, which three sides share.
    assert_eq!(simplified.faces.len(), 12);
    assert_eq!(simplified.vertices.len(), 8);
}

#[test]
fn rust_only_two_triangle_sides_keep_their_corners() {
    // Each side of the plain cube is two coplanar triangles: a region that merges, but its four
    // corners are all shared by three sides, so the ear clipper gives back two triangles.
    let source = create_cube(10.0, 10.0, 10.0);
    let simplified = simplify(&source);
    assert_same_solid(&source, &simplified);
    assert_eq!(simplified.faces.len(), source.faces.len());
    assert_eq!(simplified.vertices.len(), source.vertices.len());
}

#[test]
fn rust_only_lone_faces_are_left_as_they_were() {
    let source = create_icosahedron(10.0);
    let simplified = simplify(&source);
    assert_same_solid(&source, &simplified);
    assert_eq!(simplified.faces.len(), source.faces.len());
    for (a, b) in source.faces.iter().zip(simplified.faces.iter()) {
        assert_eq!(
            [
                source.vertices[a.v0 as usize],
                source.vertices[a.v1 as usize],
                source.vertices[a.v2 as usize]
            ],
            [
                simplified.vertices[b.v0 as usize],
                simplified.vertices[b.v1 as usize],
                simplified.vertices[b.v2 as usize]
            ]
        );
    }
}

#[test]
fn rust_only_a_raised_vertex_keeps_its_side_from_merging_over_it() {
    // Lift one interior vertex of the top side well off its plane: the faces around it can not
    // join the side's region, so the bump survives and the solid's volume is unchanged.
    let mut source = tiled_box(10.0, 4);
    let top_center = source
        .vertices
        .iter()
        .position(|v| *v == Vector3Float::new(5.0, 5.0, 10.0))
        .expect("the top side has a center vertex");
    source.vertices[top_center] = Vector3Float::new(5.0, 5.0, 11.0);
    let simplified = simplify(&source);
    assert_same_solid(&source, &simplified);
    assert!(simplified
        .vertices
        .contains(&Vector3Float::new(5.0, 5.0, 11.0)));
    assert!(simplified.faces.len() < source.faces.len() / 2);
}

#[test]
fn rust_only_work_grows_linearly_with_the_mesh() {
    // The linear-work check on a tiled box: two copies with their own vertices, in
    // the same place, so the bounds and the rounding are the same as the single's.
    let single = tiled_box(10.0, 8);
    let doubled = doubled_in_place(&single);

    let (simplified_single, single_work) = simplify_counting_work(&single, 1e-4, 1e-7);
    let (simplified_doubled, doubled_work) = simplify_counting_work(&doubled, 1e-4, 1e-7);
    assert_eq!(
        simplified_doubled.faces.len(),
        2 * simplified_single.faces.len()
    );
    assert!(doubled_work as f64 <= 2.5 * single_work as f64);
}
