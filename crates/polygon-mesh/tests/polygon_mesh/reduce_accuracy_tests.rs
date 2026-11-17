//! How closely Reduce (`Mesh::reduced_to_triangle_count`) keeps a closed shape's surface, on
//! shapes made here: a finely tessellated torus, a rounded box, a shape meshed from a signed
//! distance function by dual contouring, and a thin blade whose rim is easy to cut away. Each is
//! reduced to a small fraction of its triangles, with and without Maintain Surface, and must
//! stay a closed manifold within a bound on its two-way surface deviation
//! (`mesh_distance::surface_deviation`), as a fraction of its largest side. Reducing twice gives
//! the same mesh to the bit.

use solidmesh_math::Vector3;
use solidmesh_mesh::dual_contouring::{build_octree, generate_mesh_from_octree};
use solidmesh_mesh::mesh_distance::surface_deviation;
use solidmesh_mesh::solids::RoundedCornerBox;
use solidmesh_mesh::Mesh;

use crate::reduce_test_support::{assert_closed_manifold, geometry_hash};

/// A closed surface of revolution-like grid: `rings` x `segments` quads of `point(u, v)` with
/// both directions wrapping, split into triangles wound outward.
fn wrapped_grid(rings: usize, segments: usize, point: impl Fn(f64, f64) -> [f64; 3]) -> Mesh {
    let mut v = Vec::new();
    for i in 0..rings {
        for j in 0..segments {
            let p = point(i as f64 / rings as f64, j as f64 / segments as f64);
            v.extend_from_slice(&p);
        }
    }
    let id = |i: usize, j: usize| ((i % rings) * segments + (j % segments)) as i32;
    let mut f = Vec::new();
    for i in 0..rings {
        for j in 0..segments {
            f.extend_from_slice(&[id(i, j), id(i + 1, j), id(i + 1, j + 1)]);
            f.extend_from_slice(&[id(i, j), id(i + 1, j + 1), id(i, j + 1)]);
        }
    }
    Mesh::from_arrays(&v, &f)
}

fn torus() -> Mesh {
    let tau = std::f64::consts::TAU;
    wrapped_grid(160, 64, |u, v| {
        let (big, small) = (1.0, 0.3);
        let ring = big + small * libm::cos(tau * v);
        [
            ring * libm::cos(tau * u),
            ring * libm::sin(tau * u),
            small * libm::sin(tau * v),
        ]
    })
}

/// A blade: an ellipsoid 2 long, 0.3 wide and 0.02 thick, closed at its tips by a latitude
/// grid whose poles are single vertices.
fn blade() -> Mesh {
    let (stacks, slices) = (120usize, 48usize);
    let tau = std::f64::consts::TAU;
    let pi = std::f64::consts::PI;
    let mut v = vec![1.0, 0.0, 0.0];
    for i in 1..stacks {
        let theta = pi * i as f64 / stacks as f64;
        for j in 0..slices {
            let phi = tau * j as f64 / slices as f64;
            v.extend_from_slice(&[
                libm::cos(theta),
                0.15 * libm::sin(theta) * libm::cos(phi),
                0.01 * libm::sin(theta) * libm::sin(phi),
            ]);
        }
    }
    v.extend_from_slice(&[-1.0, 0.0, 0.0]);
    let last = (v.len() / 3 - 1) as i32;
    let ring = |i: usize, j: usize| (1 + (i - 1) * slices + (j % slices)) as i32;
    let mut f = Vec::new();
    for j in 0..slices {
        f.extend_from_slice(&[0, ring(1, j), ring(1, j + 1)]);
        f.extend_from_slice(&[last, ring(stacks - 1, j + 1), ring(stacks - 1, j)]);
    }
    for i in 1..stacks - 1 {
        for j in 0..slices {
            f.extend_from_slice(&[ring(i, j), ring(i + 1, j), ring(i + 1, j + 1)]);
            f.extend_from_slice(&[ring(i, j), ring(i + 1, j + 1), ring(i, j + 1)]);
        }
    }
    Mesh::from_arrays(&v, &f)
}

/// A sphere fused to a cylinder through it, meshed by dual contouring.
fn sdf_shape() -> Mesh {
    let f = |p: Vector3| {
        let sphere = (p.x * p.x + p.y * p.y + p.z * p.z).sqrt() - 0.8;
        let radial = (p.x * p.x + p.y * p.y).sqrt() - 0.3;
        let along = p.z.abs() - 1.2;
        let cylinder = radial.max(along);
        sphere.min(cylinder)
    };
    let octree = build_octree(
        &f,
        Vector3::new(-1.5, -1.5, -1.5),
        Vector3::new(3.0, 3.0, 3.0),
        6,
        0.0,
    );
    generate_mesh_from_octree(&octree)
}

fn rounded_box() -> Mesh {
    RoundedCornerBox::create(16, Vector3::new(4.0, 3.0, 2.0), 0.6)
}

/// Reduces `shape` to `fraction` of its triangles both ways and checks each result: closed,
/// manifold, within `max_bound` and `p99_bound` of the original (fractions of its largest
/// side), and the same mesh when reduced again. The bounds are a quarter above what Reduce
/// measures on each shape, the worse of the two modes.
fn check(shape: &Mesh, what: &str, fraction: f64, max_bound: f64, p99_bound: f64) {
    assert_closed_manifold(shape, &format!("{what} as made"));
    let target = (shape.faces.len() as f64 * fraction) as i32;
    for maintain in [false, true] {
        let label = format!("{what}, maintain_surface {maintain}, target {target}");
        let reduced = shape.reduced_to_triangle_count(target, maintain);
        assert_closed_manifold(&reduced, &label);
        assert!(
            reduced.faces.len() as i32 <= target,
            "{label}: {} faces",
            reduced.faces.len()
        );
        let d = surface_deviation(shape, &reduced, 100_000, 0.99);
        eprintln!(
            "{label}: max {:.5} p99 {:.5}",
            d.max_fraction(),
            d.percentile_fraction()
        );
        assert!(
            d.max_fraction() <= max_bound,
            "{label}: max {} over {max_bound}",
            d.max_fraction()
        );
        assert!(
            d.percentile_fraction() <= p99_bound,
            "{label}: p99 {} over {p99_bound}",
            d.percentile_fraction()
        );
        let again = shape.reduced_to_triangle_count(target, maintain);
        assert_eq!(
            geometry_hash(&reduced),
            geometry_hash(&again),
            "{label}: not repeatable"
        );
    }
}

#[test]
fn a_torus_reduced_to_a_twentieth_keeps_its_surface() {
    check(&torus(), "torus", 0.05, 0.0056, 0.0044);
}

#[test]
fn a_rounded_box_reduced_to_a_tenth_keeps_its_surface() {
    check(&rounded_box(), "rounded box", 0.1, 0.0024, 0.0015);
}

#[test]
fn a_dual_contoured_shape_reduced_to_a_tenth_keeps_its_surface() {
    check(&sdf_shape(), "sdf shape", 0.1, 0.0088, 0.003);
}

#[test]
fn a_thin_blade_reduced_to_a_twentieth_keeps_its_rim() {
    check(&blade(), "blade", 0.05, 0.0023, 0.00033);
}
