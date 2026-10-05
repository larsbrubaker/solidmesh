//! Rust-only tests for `planar_region_simplifier` (unit tests, for the crate-private work
//! counter). The ported `PlanarRegionSimplifierTests` build their parts with `MeshSdfGrid` and
//! `VertexStorage.Extrude` (Phase 2e) and are listed as pending in `docs/PORTING_PLAN.md`; these
//! check the same properties (closed, same volume, fewer faces, linear work) on hand-built meshes.

use std::collections::HashMap;

use vector_math::Vector3Float;

use super::{simplify, simplify_counting_work};
use crate::solids::platonic_solids::{create_cube, create_icosahedron};
use crate::Mesh;

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

/// Half-edges without exactly one opposite partner (C# `ClosedEdgeErrors`).
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

/// C# `AssertSameSolid`, less the nearest-triangle distance (no g3 here): closed, manifold, same
/// volume, and every kept vertex is one of the source's own.
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
    // C# `WorkGrowsLinearlyWithTheMesh` on a tiled box: two copies with their own vertices, in
    // the same place, so the bounds and the rounding are the same as the single's.
    let single = tiled_box(10.0, 8);
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

    let (simplified_single, single_work) = simplify_counting_work(&single, 1e-4, 1e-7);
    let (simplified_doubled, doubled_work) = simplify_counting_work(&doubled, 1e-4, 1e-7);
    assert_eq!(
        simplified_doubled.faces.len(),
        2 * simplified_single.faces.len()
    );
    assert!(doubled_work as f64 <= 2.5 * single_work as f64);
}
