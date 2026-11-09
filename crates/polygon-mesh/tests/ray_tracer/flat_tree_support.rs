//! Shared fixtures for the flat tree tests: the meshes (sphere, flat grids, tied centroids, a long
//! thin part, a 100k-triangle scene), the deterministic ray sets, and the comparison that traces
//! each ray on the flat tree and on the `f64` BVH and accounts for every disagreement.
//!
//! The two tracers cannot agree on every ray to the last bit: the BVH's slab test is `f64` and
//! its edge test has a slack proportional to distance, while the flat traversal is `f32` end to
//! end. So a disagreement is only tolerated where the ray truly is on the edge of being a hit or
//! a tie: [`GRAZING_STEPS`] says how close to a triangle's boundary a ray may be and still
//! disagree, and [`distances_match`] says when two hits on different triangles are the same hit.

use std::sync::Arc;

use solidmesh_math::{IntersectionType, Ray, Vector3};
use solidmesh_mesh::ray_tracer::flat_tree::{
    FlatHit, FlatRay, FlatTree, WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
};
use solidmesh_mesh::ray_tracer::{
    BoundingVolumeHierarchy, BvhCreationOptions, Traceable, TraceableRef, TriangleShape,
};

/// How many `f32` epsilons, of the scale `|origin| + t + extent`, a ray may pass from a
/// triangle's boundary and still disagree with the `f64` tracer. The flat ray's origin and
/// direction are the `f64` ray narrowed (half a step of the coordinates and of `t` times the
/// direction); the sheared corners and edge functions add a few more steps of the corners' size;
/// and the BVH's own edge test accepts anything within 4 `f32` epsilons of `t` outside an edge.
/// Those sum to about 8 epsilons of the scale, doubled for margin. The same figure, divided by the
/// ray's incidence, bounds how far two hits on the same surface can differ in distance.
const GRAZING_STEPS: f64 = 16.0;

/// Rays closer to parallel than this are tolerated as grazing wherever they land: the two
/// tracers' plane-parallel cutoffs and `det` noise differ near zero incidence.
const NEAR_PARALLEL_COS: f64 = 256.0 * f32::EPSILON as f64;

pub(crate) fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

pub(crate) fn bumpy_sphere(
    center: Vector3,
    radius: f64,
    rows: usize,
    columns: usize,
    triangles: &mut Vec<[Vector3; 3]>,
) {
    let point = |column: usize, row: usize| {
        let u = column as f64 / columns as f64 * std::f64::consts::TAU;
        let w = row as f64 / rows as f64 * std::f64::consts::PI;
        let bump = 1.0 + 0.08 * (7.0 * u + 1.0).sin() * (5.0 * w).sin();
        center
            + v(
                radius * bump * w.sin() * u.cos(),
                radius * bump * w.cos(),
                radius * bump * w.sin() * u.sin(),
            )
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

pub(crate) fn flat_grid(n: usize, step: f64, origin: Vector3, triangles: &mut Vec<[Vector3; 3]>) {
    for j in 0..n {
        for i in 0..n {
            let at = |i: usize, j: usize| origin + v(i as f64 * step, j as f64 * step, 0.0);
            let (a, b, c, d) = (at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j + 1));
            triangles.push([a, b, c]);
            triangles.push([a, c, d]);
        }
    }
}

pub(crate) fn tied_centroids(count: usize) -> Vec<[Vector3; 3]> {
    (0..count)
        .map(|_| [v(0.0, 0.0, 0.0), v(9.0, 0.0, 0.0), v(0.0, 9.0, 0.0)])
        .collect()
}

/// A thin tube, long along x: a shape whose boxes are long and whose triangles are slivers.
pub(crate) fn long_thin_tube(segments: usize, sides: usize) -> Vec<[Vector3; 3]> {
    let radius = 0.05;
    let length = 2000.0;
    let point = |segment: usize, side: usize| {
        let angle = side as f64 / sides as f64 * std::f64::consts::TAU;
        v(
            segment as f64 / segments as f64 * length,
            radius * angle.cos(),
            radius * angle.sin(),
        )
    };
    let mut triangles = Vec::with_capacity(segments * sides * 2);
    for segment in 0..segments {
        for side in 0..sides {
            let (a, b, c, d) = (
                point(segment, side),
                point(segment + 1, side),
                point(segment + 1, side + 1),
                point(segment, side + 1),
            );
            triangles.push([a, d, c]);
            triangles.push([a, c, b]);
        }
    }
    triangles
}

/// About 100k triangles: five spheres over a plate.
pub(crate) fn scene_100k() -> Vec<[Vector3; 3]> {
    let mut triangles = Vec::with_capacity(103_000);
    for (offset, x) in [-30.0, -15.0, 0.0, 15.0, 30.0].iter().enumerate() {
        bumpy_sphere(
            v(*x, offset as f64 * 2.5, 0.0),
            6.0,
            50,
            100,
            &mut triangles,
        );
    }
    flat_grid(160, 0.5, v(-40.0, -40.0, -6.0), &mut triangles);
    triangles
}

fn bvh_of(triangles: &[[Vector3; 3]]) -> TraceableRef {
    let items: Vec<TraceableRef> = triangles
        .iter()
        .enumerate()
        .map(|(index, corners)| {
            Arc::new(TriangleShape::new(
                corners[0],
                corners[1],
                corners[2],
                index as i32,
            )) as TraceableRef
        })
        .collect();
    BoundingVolumeHierarchy::create_new_hierarchy(items, BvhCreationOptions::default())
        .expect("there are triangles")
}

/// The scene's box and the biggest absolute coordinate.
fn bounds_of(triangles: &[[Vector3; 3]]) -> (Vector3, Vector3, f64) {
    let (mut min, mut max) = (
        v(f64::MAX, f64::MAX, f64::MAX),
        v(f64::MIN, f64::MIN, f64::MIN),
    );
    let mut extent = 0.0f64;
    for corner in triangles.iter().flatten() {
        for axis in 0..3 {
            min[axis] = min[axis].min(corner[axis]);
            max[axis] = max[axis].max(corner[axis]);
            extent = extent.max(corner[axis].abs());
        }
    }
    (min, max, extent)
}

struct Pcg(u64);

impl Pcg {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(0x5851_f42d_4c95_7f2d).wrapping_add(1);
        ((self.0 >> 33) as f64) / ((1u64 << 31) as f64)
    }
}

/// A perspective camera's rays over the scene's box, `size` across, from outside it.
pub(crate) fn camera_rays(
    triangles: &[[Vector3; 3]],
    size: usize,
    kind: IntersectionType,
) -> Vec<Ray> {
    let (min, max, _) = bounds_of(triangles);
    let center = (min + max) * 0.5;
    let radius = (max - min).length().max(1.0);
    let eye = center + v(0.31, 0.47, 0.83) * (radius * 1.6);
    let mut rays = Vec::with_capacity(size * size);
    for row in 0..size {
        for column in 0..size {
            let across = (column as f64 + 0.5) / size as f64 - 0.5;
            let up = (row as f64 + 0.5) / size as f64 - 0.5;
            let target = center + v(across * radius, up * radius, 0.0);
            let direction = (target - eye).get_normal();
            rays.push(Ray::with_limits(eye, direction, 0.0, f64::INFINITY, kind));
        }
    }
    rays
}

/// Incoherent rays from outside the box to random points inside it, some with narrowed windows.
pub(crate) fn random_rays(
    triangles: &[[Vector3; 3]],
    count: usize,
    seed: u64,
    kind: IntersectionType,
) -> Vec<Ray> {
    let (min, max, _) = bounds_of(triangles);
    let center = (min + max) * 0.5;
    let radius = (max - min).length().max(1.0);
    let mut random = Pcg(seed);
    let mut rays = Vec::with_capacity(count);
    for at in 0..count {
        let origin_dir = v(
            random.next() - 0.5,
            random.next() - 0.5,
            random.next() - 0.5,
        )
        .get_normal();
        let origin = center + origin_dir * (radius * 1.8);
        let target = v(
            min.x + random.next() * (max.x - min.x),
            min.y + random.next() * (max.y - min.y),
            min.z + random.next() * (max.z - min.z),
        );
        let direction = (target - origin).get_normal();
        let (near, far) = if at % 5 == 0 {
            (random.next() * radius, (1.0 + random.next()) * radius * 2.0)
        } else {
            (0.0, f64::INFINITY)
        };
        rays.push(Ray::with_limits(origin, direction, near, far, kind));
    }
    rays
}

/// Incoherent rays from outside the box aimed at randomly chosen triangles' centers.
pub(crate) fn rays_at_triangles(
    triangles: &[[Vector3; 3]],
    count: usize,
    seed: u64,
    kind: IntersectionType,
) -> Vec<Ray> {
    let (min, max, _) = bounds_of(triangles);
    let center = (min + max) * 0.5;
    let radius = (max - min).length().max(1.0);
    let mut random = Pcg(seed);
    let mut rays = Vec::with_capacity(count);
    for _ in 0..count {
        let origin_dir = v(
            random.next() - 0.5,
            random.next() - 0.5,
            random.next() - 0.5,
        )
        .get_normal();
        let origin = center + origin_dir * (radius * 1.8);
        let pick = (random.next() * triangles.len() as f64) as usize % triangles.len();
        let corners = &triangles[pick];
        let target = (corners[0] + corners[1] + corners[2]) / 3.0;
        let direction = (target - origin).get_normal();
        rays.push(Ray::with_limits(
            origin,
            direction,
            0.0,
            f64::INFINITY,
            kind,
        ));
    }
    rays
}

/// Rays straight down at grid corners, edge midpoints and cell centers of an integer grid:
/// directions with zero components, origins exactly on the tree's slab planes.
pub(crate) fn straight_down_rays(n: usize) -> Vec<Ray> {
    let mut rays = Vec::new();
    for j in 0..n {
        for i in 0..n {
            for &(du, dw) in &[(0.0, 0.0), (0.5, 0.0), (0.0, 0.5), (0.5, 0.5), (0.25, 0.75)] {
                let origin = v(i as f64 + du, j as f64 + dw, 5.0);
                rays.push(Ray::with_limits(
                    origin,
                    v(0.0, 0.0, -1.0),
                    0.0,
                    f64::INFINITY,
                    IntersectionType::Both,
                ));
            }
        }
    }
    rays
}

/// Unit normal of a source triangle in `f64`.
fn normal_of(corners: &[Vector3; 3]) -> Vector3 {
    (corners[1] - corners[0])
        .cross(corners[2] - corners[0])
        .get_normal()
}

/// The distance from `point` to the triangle's boundary (its nearest edge).
fn distance_to_boundary(corners: &[Vector3; 3], point: Vector3) -> f64 {
    (0..3)
        .map(|edge| {
            let a = corners[edge];
            let b = corners[(edge + 1) % 3];
            let along = b - a;
            let length = along.length_squared();
            let t = if length > 0.0 {
                ((point - a).dot(along) / length).clamp(0.0, 1.0)
            } else {
                0.0
            };
            (a + along * t - point).length()
        })
        .fold(f64::INFINITY, f64::min)
}

#[derive(Clone, Copy)]
struct Hit {
    source: u32,
    t: f64,
}

/// How a comparison of one ray set came out.
#[derive(Debug, Default)]
pub(crate) struct Comparison {
    pub(crate) rays: usize,
    pub(crate) both_miss: usize,
    pub(crate) same_triangle: usize,
    pub(crate) tied_triangle: usize,
    pub(crate) grazing: usize,
    pub(crate) reference_only: usize,
    pub(crate) flat_only: usize,
}

pub(crate) fn compare(triangles: &[[Vector3; 3]], rays: &[Ray], what: &str) -> Comparison {
    let comparison = compare_rays(triangles, rays, what);
    assert!(
        comparison.grazing * 50 <= comparison.rays,
        "{what}: too many grazing disagreements: {comparison:?}"
    );
    comparison
}

/// Traces every ray both ways and accounts for each one, failing on any disagreement the module
/// header does not excuse.
pub(crate) fn compare_rays(triangles: &[[Vector3; 3]], rays: &[Ray], what: &str) -> Comparison {
    let tree = FlatTree::build(triangles, WEBGPU_DEFAULT_MAX_STORAGE_BINDING).expect("built");
    let bvh = bvh_of(triangles);
    let (_, _, extent) = bounds_of(triangles);
    let mut comparison = Comparison {
        rays: rays.len(),
        ..Comparison::default()
    };

    for ray in rays {
        let reference = hit_of_bvh(bvh.as_ref(), ray);
        let flat_ray = FlatRay::from_ray(ray);
        let flat = tree.trace_closest(&flat_ray);
        // Any-hit agrees with closest-hit on the same tracer, so the closest comparison covers it.
        assert_eq!(
            tree.trace_any(&flat_ray),
            flat.is_some(),
            "{what}: flat any-hit and closest-hit disagree"
        );
        let mut shadow = ray.clone();
        shadow.is_shadow_ray = true;
        assert_eq!(
            hit_of_bvh(bvh.as_ref(), &shadow).is_some(),
            reference.is_some(),
            "{what}: reference shadow ray and closest-hit disagree"
        );

        let flat = flat.map(|hit: FlatHit| Hit {
            source: hit.source_index,
            t: f64::from(hit.t),
        });
        match (reference, flat) {
            (None, None) => comparison.both_miss += 1,
            (Some(a), Some(b)) if a.source == b.source => {
                assert!(
                    distances_match(ray, triangles, a, b, extent),
                    "{what}: same triangle {} but distances {} and {} on ray {ray:?}",
                    a.source,
                    a.t,
                    b.t
                );
                comparison.same_triangle += 1;
            }
            (Some(a), Some(b)) => {
                if distances_match(ray, triangles, a, b, extent) {
                    comparison.tied_triangle += 1;
                } else {
                    let nearer = if a.t <= b.t { a } else { b };
                    assert_grazing(ray, triangles, nearer, extent, what);
                    comparison.grazing += 1;
                }
            }
            (Some(hit), None) => {
                assert_grazing(ray, triangles, hit, extent, what);
                comparison.grazing += 1;
                comparison.reference_only += 1;
            }
            (None, Some(hit)) => {
                assert_grazing(ray, triangles, hit, extent, what);
                comparison.grazing += 1;
                comparison.flat_only += 1;
            }
        }
    }
    comparison
}

fn hit_of_bvh(bvh: &dyn Traceable, ray: &Ray) -> Option<Hit> {
    let mut traced = ray.clone();
    let info = bvh.get_closest_intersection(&mut traced)?;
    if info.hit_type == IntersectionType::None {
        return None;
    }
    let leaf = info.closest_hit_as::<TriangleShape>()?;
    Some(Hit {
        source: leaf.index as u32,
        t: info.distance_to_hit,
    })
}

fn scale_of(ray: &Ray, t: f64, extent: f64) -> f64 {
    let origin = ray
        .origin
        .x
        .abs()
        .max(ray.origin.y.abs())
        .max(ray.origin.z.abs());
    origin + t.abs() + extent
}

fn cos_of(ray: &Ray, corners: &[Vector3; 3]) -> f64 {
    normal_of(corners).dot(ray.direction_normal).abs()
}

fn distances_match(ray: &Ray, triangles: &[[Vector3; 3]], a: Hit, b: Hit, extent: f64) -> bool {
    let cos = cos_of(ray, &triangles[a.source as usize])
        .min(cos_of(ray, &triangles[b.source as usize]))
        .max(1e-6);
    let tolerance = GRAZING_STEPS * f64::from(f32::EPSILON) * scale_of(ray, a.t.max(b.t), extent);
    (a.t - b.t).abs() <= tolerance / cos
}

fn assert_grazing(ray: &Ray, triangles: &[[Vector3; 3]], hit: Hit, extent: f64, what: &str) {
    let corners = &triangles[hit.source as usize];
    let point = ray.origin + ray.direction_normal * hit.t;
    let distance = distance_to_boundary(corners, point);
    let allowed = GRAZING_STEPS * f64::from(f32::EPSILON) * scale_of(ray, hit.t, extent);
    assert!(
        distance <= allowed || cos_of(ray, corners) <= NEAR_PARALLEL_COS,
        "{what}: a disagreement away from any edge: triangle {} is {distance} from the ray \
         (allowed {allowed}) at t {} on ray {ray:?}",
        hit.source,
        hit.t,
    );
}
