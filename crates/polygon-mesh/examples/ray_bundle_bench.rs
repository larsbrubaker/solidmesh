//! Measures ray bundles against lone rays at 1024x1024: perspective primary rays and a shadow
//! pass toward a point light, on bumpy spheres of about 1k, 10k, 100k and 1M triangles framed
//! so the triangles range from many pixels each down to under one, for packet sizes 1 to 32.
//! Lone means the real `get_closest_intersection` path, not a bundle of one. The machine is
//! shared, so each timed point is the minimum of three runs.
//!
//! Each scene then gets an attribution table: the measured nanoseconds per ray-box test, per
//! interval test and per ray-triangle test through the real `MinimalTriangle` leaf (timed alone
//! over rays and boxes taken from a real traversal), times the deterministic counters, for lone
//! rays and the best packet size, with whatever remains charged to bookkeeping and dispatch. The
//! isolated timings run on a different memory-access pattern from a real trace, so the
//! remainder also holds the difference; the counts themselves are exact.
//!
//! Usage: `cargo run --release --example ray_bundle_bench -p solidmesh-mesh`.

use std::sync::Arc;
use std::time::Instant;

use solidmesh_math::{AxisAlignedBoundingBox, IntersectionType, Ray, Vector3, Vector3Float};
use solidmesh_mesh::ray_tracer::{
    BoundingVolumeHierarchy, BvhCreationOptions, IntersectInfo, MinimalTriangle, PacketBounds,
    RayBundle, RayBundleStats, Traceable, TraceableRef, VertexFunc,
};

const IMAGE: usize = 1024;
const RUNS: usize = 3;
const PACKET_SIZES: [usize; 6] = [1, 2, 4, 8, 16, 32];

fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

fn bumpy_sphere_triangles(wanted: usize) -> Arc<Vec<[Vector3Float; 3]>> {
    // columns x rows x 2 is about `wanted`, with columns twice rows.
    let rows = ((wanted as f64 / 4.0).sqrt() as usize).max(4);
    let columns = rows * 2;
    let point = |column: usize, row: usize| {
        let u = column as f64 / columns as f64 * std::f64::consts::TAU;
        let w = row as f64 / rows as f64 * std::f64::consts::PI;
        let radius = 1.0 + 0.08 * (7.0 * u + 1.0).sin() * (5.0 * w).sin();
        Vector3Float::from_vector3(v(
            radius * w.sin() * u.cos(),
            radius * w.cos(),
            radius * w.sin() * u.sin(),
        ))
    };
    let mut triangles = Vec::with_capacity(columns * rows * 2);
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
    Arc::new(triangles)
}

fn build(triangles: &Arc<Vec<[Vector3Float; 3]>>) -> TraceableRef {
    let source = triangles.clone();
    let corners: VertexFunc = Arc::new(move |face, corner| source[face as usize][corner as usize]);
    let items: Vec<TraceableRef> = (0..triangles.len())
        .map(|face| Arc::new(MinimalTriangle::new(corners.clone(), face as i32)) as TraceableRef)
        .collect();
    BoundingVolumeHierarchy::create_new_hierarchy(items, BvhCreationOptions::LocalOrderClustering)
        .expect("a mesh has triangles")
}

/// The primary ray of each pixel, row-major, from a camera framing the unit sphere.
fn primary_rays() -> Vec<Ray> {
    let eye = v(0.4, 0.3, 3.6);
    let mut rays = Vec::with_capacity(IMAGE * IMAGE);
    for y in 0..IMAGE {
        for x in 0..IMAGE {
            let u = (x as f64 + 0.5) / IMAGE as f64 * 2.0 - 1.0;
            let w = (y as f64 + 0.5) / IMAGE as f64 * 2.0 - 1.0;
            let target = v(u * 1.3, w * 1.3, 0.0);
            rays.push(Ray::new(eye, (target - eye).get_normal()));
        }
    }
    rays
}

/// A shadow ray toward the light from each primary hit, keeping its pixel.
fn shadow_rays(primary: &[Ray], root: &TraceableRef) -> Vec<(u32, Ray)> {
    let light = v(4.0, 5.0, 6.0);
    let mut rays = Vec::with_capacity(primary.len());
    for (pixel, ray) in primary.iter().enumerate() {
        let mut lone = ray.clone();
        if let Some(hit) = hit_of(&root.get_closest_intersection(&mut lone)) {
            let start = hit.hit_position + hit.normal_at_hit * 0.001;
            let to_light = light - start;
            let distance = to_light.length();
            let mut shadow = Ray::with_limits(
                start,
                to_light / distance,
                0.0,
                distance,
                IntersectionType::FrontFace,
            );
            shadow.is_shadow_ray = true;
            rays.push((pixel as u32, shadow));
        }
    }
    rays
}

fn hit_of<'a>(info: &Option<IntersectInfo<'a>>) -> Option<IntersectInfo<'a>> {
    info.filter(|hit| hit.hit_type != IntersectionType::None)
}

/// Rays of the image cut into tiles of `tile` x `tile` pixels: the same rays in every run.
fn tiled(rays: &[(u32, Ray)], tile: usize) -> Vec<Vec<Ray>> {
    let across = IMAGE.div_ceil(tile);
    let mut tiles = vec![Vec::new(); across * across];
    for (pixel, ray) in rays {
        let (x, y) = ((*pixel as usize) % IMAGE, (*pixel as usize) / IMAGE);
        tiles[y / tile * across + x / tile].push(ray.clone());
    }
    tiles.retain(|one: &Vec<Ray>| !one.is_empty());
    tiles
}

fn lone_seconds(root: &TraceableRef, rays: &[(u32, Ray)]) -> f64 {
    let mut best = f64::MAX;
    for _ in 0..RUNS {
        let start = Instant::now();
        let mut hits = 0u32;
        for (_, ray) in rays {
            let mut ray = ray.clone();
            if root.get_closest_intersection(&mut ray).is_some() {
                hits = hits.wrapping_add(1);
            }
        }
        std::hint::black_box(hits);
        best = best.min(start.elapsed().as_secs_f64());
    }
    best
}

/// Timed bundle trace of the tiles, returning the best time and the counters.
fn bundle_seconds(root: &TraceableRef, tiles: &[Vec<Ray>]) -> (f64, RayBundleStats) {
    let mut best = f64::MAX;
    let mut stats = RayBundleStats::default();
    for run in 0..RUNS {
        let mut bundle = RayBundle::new(Vec::new());
        let start = Instant::now();
        for tile in tiles {
            bundle.reset(tile.iter().cloned());
            bundle.trace(root.as_ref()).expect("a valid bundle");
            if run == 0 {
                stats.add(bundle.stats());
            }
        }
        best = best.min(start.elapsed().as_secs_f64());
    }
    (best, stats)
}

/// The cost of one test of each kind, measured alone.
struct TestCosts {
    box_ns: f64,
    interval_ns: f64,
    triangle_ns: f64,
}

/// The box and leaf tests a real lone traversal makes for a sample of rays.
struct TraversalSample<'a> {
    boxes: Vec<(u32, AxisAlignedBoundingBox)>,
    leaves: Vec<(u32, f64, &'a dyn Traceable)>,
}

/// A lone traversal, recording what it tests: the same child order and narrowing as the node.
fn sample_walk<'a>(
    node: &'a dyn Traceable,
    ray: &mut Ray,
    ray_id: u32,
    sample: &mut TraversalSample<'a>,
) -> Option<IntersectInfo<'a>> {
    let Some(children) = node.children() else {
        sample
            .leaves
            .push((ray_id, ray.max_distance_to_consider, node));
        return node.get_closest_intersection(ray);
    };
    let bounds = node.get_axis_aligned_bounding_box();
    sample.boxes.push((ray_id, bounds));
    if !ray.intersection(&bounds) {
        return None;
    }
    let axis = if bounds.x_size() > bounds.y_size() {
        0
    } else {
        1
    };
    let axis = if bounds.size()[axis] > bounds.z_size() {
        axis
    } else {
        2
    };
    let order = if ray.direction_normal[axis] < 0.0 {
        [1, 0]
    } else {
        [0, 1]
    };
    let mut best = None;
    for slot in order {
        let hit = sample_walk(children[slot], ray, ray_id, sample);
        if let Some(hit) = hit_of(&hit) {
            if ray.is_shadow_ray {
                return Some(hit);
            }
            ray.max_distance_to_consider = hit.distance_to_hit;
            best = Some(hit);
        }
    }
    best
}

/// Times one test of each kind on a sample of about 20,000 rays from this pass.
fn measure_test_costs(root: &TraceableRef, rays: &[(u32, Ray)]) -> TestCosts {
    let stride = (rays.len() / 20_000).max(1);
    let mut sample_rays: Vec<Ray> = rays
        .iter()
        .step_by(stride)
        .map(|(_, r)| r.clone())
        .collect();
    let mut sample = TraversalSample {
        boxes: Vec::new(),
        leaves: Vec::new(),
    };
    for (index, ray) in sample_rays.iter_mut().enumerate() {
        let mut walked = ray.clone();
        sample_walk(root.as_ref(), &mut walked, index as u32, &mut sample);
    }
    let tile_bounds: Vec<PacketBounds> = tiled(rays, 16)
        .iter()
        .filter(|tile| tile.len() >= 2)
        .map(|tile| {
            let indices: Vec<u32> = (0..tile.len() as u32).collect();
            PacketBounds::new(tile, &indices)
        })
        .collect();

    let box_ns = best_ns_each(sample.boxes.len(), || {
        for (id, bounds) in &sample.boxes {
            std::hint::black_box(sample_rays[*id as usize].intersection(bounds));
        }
    });
    let interval_ns = best_ns_each(sample.boxes.len(), || {
        for (index, (_, bounds)) in sample.boxes.iter().enumerate() {
            std::hint::black_box(tile_bounds[index % tile_bounds.len()].misses(bounds));
        }
    });
    let triangle_ns = best_ns_each(sample.leaves.len(), || {
        for &(id, window, leaf) in &sample.leaves {
            let ray = &mut sample_rays[id as usize];
            ray.max_distance_to_consider = window;
            std::hint::black_box(leaf.get_closest_intersection(ray).is_some());
        }
    });
    TestCosts {
        box_ns,
        interval_ns,
        triangle_ns,
    }
}

/// The best time per item over five runs of `work`.
fn best_ns_each(items: usize, mut work: impl FnMut()) -> f64 {
    let mut best = f64::MAX;
    for _ in 0..5 {
        let start = Instant::now();
        work();
        best = best.min(start.elapsed().as_secs_f64());
    }
    best * 1e9 / items as f64
}

/// One row of the attribution table: count x measured cost for each kind of test, and what
/// remains of the pass.
fn print_attribution(label: &str, seconds: f64, costs: &TestCosts, stats: &RayBundleStats) -> f64 {
    let total_ms = seconds * 1e3;
    let ms = |count: u64, ns: f64| count as f64 * ns / 1e6;
    let box_ms = ms(stats.box_tests, costs.box_ns);
    let interval_ms = ms(stats.interval_tests, costs.interval_ns);
    let triangle_ms = ms(stats.leaf_tests, costs.triangle_ns);
    let rest_ms = total_ms - box_ms - interval_ms - triangle_ms;
    let share = |value: f64| value * 100.0 / total_ms;
    println!(
        "  {label:<14} {total_ms:>9.1}  box {:>9} ({box_ms:>8.1} ms {:>5.1}%)  interval {:>8} ({interval_ms:>6.1} ms {:>4.1}%)  triangle {:>9} ({triangle_ms:>8.1} ms {:>5.1}%)  rest {rest_ms:>8.1} ms {:>5.1}%",
        stats.box_tests,
        share(box_ms),
        stats.interval_tests,
        share(interval_ms),
        stats.leaf_tests,
        share(triangle_ms),
        share(rest_ms),
    );
    total_ms - box_ms - interval_ms
}

fn report(name: &str, root: &TraceableRef, rays: &[(u32, Ray)], triangles: usize) {
    let lone = lone_seconds(root, rays);
    println!(
        "{name}: {triangles} triangles, {} rays, lone rays {:.1} ms ({:.0} ns/ray)",
        rays.len(),
        lone * 1e3,
        lone * 1e9 / rays.len() as f64
    );
    println!("  packet  time ms   vs lone   box+interval  ray-tri   early%  interval%  resort%  avg rays in resort");
    let mut baseline = 0u64;
    let mut lone_stats = RayBundleStats::default();
    let mut best: Option<(usize, f64, RayBundleStats)> = None;
    for &tile in &PACKET_SIZES {
        let (seconds, stats) = bundle_seconds(root, &tiled(rays, tile));
        if tile == 1 {
            baseline = stats.box_tests + stats.interval_tests;
            lone_stats = stats;
        } else if best.is_none_or(|(_, best_seconds, _)| seconds < best_seconds) {
            best = Some((tile, seconds, stats));
        }
        let steps = stats.early_hits + stats.interval_misses + stats.last_resorts;
        let percent = |count: u64| count as f64 * 100.0 / steps as f64;
        println!(
            "  {tile:>6} {:>8.1} {:>8.2}x {:>13}  {:>7}  {:>6.1} {:>9.1} {:>8.1} {:>10.1}  (box-test reduction vs size-1: {:.2}x)",
            seconds * 1e3,
            lone / seconds,
            stats.box_tests + stats.interval_tests,
            stats.leaf_tests,
            percent(stats.early_hits),
            percent(stats.interval_misses),
            percent(stats.last_resorts),
            stats.last_resort_rays as f64 / stats.last_resorts.max(1) as f64,
            baseline as f64 / (stats.box_tests + stats.interval_tests) as f64,
        );
    }

    let costs = measure_test_costs(root, rays);
    println!(
        "  attribution: {:.2} ns per ray-box test, {:.2} ns per interval test, {:.2} ns per ray-triangle test",
        costs.box_ns, costs.interval_ns, costs.triangle_ns
    );
    print_attribution("lone rays", lone, &costs, &lone_stats);
    if let Some((tile, seconds, stats)) = best {
        let without_boxes_ms =
            print_attribution(&format!("{tile}x{tile} packets"), seconds, &costs, &stats);
        let triangle_ms = stats.leaf_tests as f64 * costs.triangle_ns / 1e6;
        println!(
            "  ceiling with no box or interval tests at all: {:.2}x (keeping the rest), {:.2}x (triangles alone)",
            lone * 1e3 / without_boxes_ms,
            lone * 1e3 / triangle_ms
        );
    }
}

fn main() {
    let primary = primary_rays();
    let primary_pixels: Vec<(u32, Ray)> = primary
        .iter()
        .cloned()
        .enumerate()
        .map(|(pixel, ray)| (pixel as u32, ray))
        .collect();
    for wanted in [1_000usize, 10_000, 100_000, 1_000_000] {
        let triangles = bumpy_sphere_triangles(wanted);
        let build_start = Instant::now();
        let root = build(&triangles);
        println!(
            "built {} triangles in {:.0} ms",
            triangles.len(),
            build_start.elapsed().as_secs_f64() * 1e3
        );
        report("primary", &root, &primary_pixels, triangles.len());
        let shadows = shadow_rays(&primary, &root);
        report("shadow", &root, &shadows, triangles.len());
    }
}
