//! Measures the flat GPU tree's reference traversal against the existing `f64` BVH, one thread,
//! on a scene of about 100k triangles: five bumpy spheres over a plate, camera-like coherent rays
//! and random incoherent rays, closest-hit and any-hit. Each figure is the best of three runs.
//!
//! Usage: `cargo run --release --example flat_tree_bench -p solidmesh-mesh`.

use std::sync::Arc;
use std::time::Instant;

use solidmesh_math::{IntersectionType, Ray, Vector3};
use solidmesh_mesh::ray_tracer::flat_tree::{
    FlatRay, FlatTree, WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
};
use solidmesh_mesh::ray_tracer::{
    BoundingVolumeHierarchy, BvhCreationOptions, TraceableRef, TriangleShape,
};

const RUNS: usize = 3;
const CAMERA_SIZE: usize = 1024;
const RANDOM_RAYS: usize = 1_000_000;

fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

fn scene() -> Vec<[Vector3; 3]> {
    let mut triangles = Vec::with_capacity(103_000);
    for (index, x) in [-30.0, -15.0, 0.0, 15.0, 30.0].iter().enumerate() {
        let center = v(*x, index as f64 * 2.5, 0.0);
        let (rows, columns) = (50usize, 100usize);
        let point = |column: usize, row: usize| {
            let u = column as f64 / columns as f64 * std::f64::consts::TAU;
            let w = row as f64 / rows as f64 * std::f64::consts::PI;
            let bump = 1.0 + 0.08 * (7.0 * u + 1.0).sin() * (5.0 * w).sin();
            center
                + v(
                    bump * w.sin() * u.cos(),
                    bump * w.cos(),
                    bump * w.sin() * u.sin(),
                ) * 6.0
        };
        for row in 0..rows {
            for column in 0..columns {
                let (a, b, c, d) = (
                    point(column, row),
                    point(column + 1, row),
                    point(column + 1, row + 1),
                    point(column, row + 1),
                );
                triangles.push([a, c, b]);
                triangles.push([a, d, c]);
            }
        }
    }
    let at = |i: usize, j: usize| v(-40.0 + i as f64 * 0.5, -40.0 + j as f64 * 0.5, -6.0);
    for j in 0..160 {
        for i in 0..160 {
            let (a, b, c, d) = (at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j + 1));
            triangles.push([a, b, c]);
            triangles.push([a, c, d]);
        }
    }
    triangles
}

fn camera_rays() -> Vec<Ray> {
    let eye = v(10.0, -60.0, 30.0);
    let center = v(0.0, 0.0, 0.0);
    let forward = (center - eye).get_normal();
    let right = forward.cross(v(0.0, 0.0, 1.0)).get_normal();
    let up = right.cross(forward).get_normal();
    let mut rays = Vec::with_capacity(CAMERA_SIZE * CAMERA_SIZE);
    for row in 0..CAMERA_SIZE {
        for column in 0..CAMERA_SIZE {
            let across = (column as f64 + 0.5) / CAMERA_SIZE as f64 - 0.5;
            let lift = (row as f64 + 0.5) / CAMERA_SIZE as f64 - 0.5;
            let direction = (forward + right * across + up * lift).get_normal();
            rays.push(Ray::with_limits(
                eye,
                direction,
                0.0,
                f64::INFINITY,
                IntersectionType::Both,
            ));
        }
    }
    rays
}

fn random_rays() -> Vec<Ray> {
    let mut state: u64 = 0x1234_5678_9abc_def0;
    let mut next = || {
        state = state.wrapping_mul(0x5851_f42d_4c95_7f2d).wrapping_add(1);
        ((state >> 33) as f64) / ((1u64 << 31) as f64)
    };
    let mut rays = Vec::with_capacity(RANDOM_RAYS);
    for _ in 0..RANDOM_RAYS {
        let origin = v(
            next() * 120.0 - 60.0,
            next() * 120.0 - 60.0,
            next() * 60.0 + 5.0,
        );
        let target = v(
            next() * 80.0 - 40.0,
            next() * 80.0 - 40.0,
            next() * 12.0 - 6.0,
        );
        rays.push(Ray::with_limits(
            origin,
            (target - origin).get_normal(),
            0.0,
            f64::INFINITY,
            IntersectionType::Both,
        ));
    }
    rays
}

fn best_seconds(mut trace: impl FnMut() -> u64) -> f64 {
    let mut best = f64::MAX;
    for _ in 0..RUNS {
        let start = Instant::now();
        std::hint::black_box(trace());
        best = best.min(start.elapsed().as_secs_f64());
    }
    best
}

fn report(name: &str, seconds: f64, rays: usize) {
    println!(
        "  {name:<42} {:>7.2} M rays/s ({:>7.1} ns/ray)",
        rays as f64 / seconds / 1e6,
        seconds * 1e9 / rays as f64
    );
}

fn main() {
    let triangles = scene();
    let build_start = Instant::now();
    let tree = FlatTree::build(&triangles, WEBGPU_DEFAULT_MAX_STORAGE_BINDING).expect("built");
    let flat_build = build_start.elapsed().as_secs_f64();
    let sizes = tree.sizes();
    println!(
        "{} triangles: flat build {:.0} ms, {} nodes ({} bytes), {} triangle bytes, depth {}",
        triangles.len(),
        flat_build * 1e3,
        sizes.node_count,
        sizes.node_bytes,
        sizes.triangle_bytes,
        sizes.depth
    );

    let items: Vec<TraceableRef> = triangles
        .iter()
        .enumerate()
        .map(|(index, c)| {
            Arc::new(TriangleShape::new(c[0], c[1], c[2], index as i32)) as TraceableRef
        })
        .collect();
    let build_start = Instant::now();
    let bvh = BoundingVolumeHierarchy::create_new_hierarchy(items, BvhCreationOptions::default())
        .expect("a tree");
    println!(
        "binary BVH build {:.0} ms",
        build_start.elapsed().as_secs_f64() * 1e3
    );

    for (name, rays) in [("camera", camera_rays()), ("random", random_rays())] {
        println!("{name}: {} rays", rays.len());
        let bvh_closest = best_seconds(|| {
            let mut hits = 0u64;
            for ray in &rays {
                let mut traced = ray.clone();
                if bvh.get_closest_intersection(&mut traced).is_some() {
                    hits += 1;
                }
            }
            hits
        });
        report("existing f64 BVH, closest hit", bvh_closest, rays.len());

        let flat_rays: Vec<FlatRay> = rays.iter().map(FlatRay::from_ray).collect();
        let flat_closest = best_seconds(|| {
            let mut hits = 0u64;
            for ray in &flat_rays {
                if tree.trace_closest(ray).is_some() {
                    hits += 1;
                }
            }
            hits
        });
        report("flat tree, closest hit", flat_closest, rays.len());

        let bvh_any = best_seconds(|| {
            let mut hits = 0u64;
            for ray in &rays {
                let mut traced = ray.clone();
                traced.is_shadow_ray = true;
                if bvh.get_closest_intersection(&mut traced).is_some() {
                    hits += 1;
                }
            }
            hits
        });
        report("existing f64 BVH, any hit (shadow)", bvh_any, rays.len());

        let flat_any = best_seconds(|| {
            let mut hits = 0u64;
            for ray in &flat_rays {
                if tree.trace_any(ray) {
                    hits += 1;
                }
            }
            hits
        });
        report("flat tree, any hit", flat_any, rays.len());
    }
}
