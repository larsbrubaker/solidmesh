//! Ray bundles against lone rays: every ray of every bundle must get the result, bit for bit,
//! that tracing it alone gets, across meshes, cameras, bundle sizes and containers, including
//! the rays that make two surfaces tie.

use std::sync::Arc;

use solidmesh_math::{IntersectionType, Matrix4X4, Plane, Ray, Vector3};
use solidmesh_mesh::ray_tracer::{
    BoundingVolumeHierarchy, BvhCreationOptions, IntersectInfo, PlaneShape, RayBundle, RayTracer,
    Scene, Traceable, TraceableRef, Transform, TriangleShape, UnboundCollection,
};

pub(super) fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

pub(super) type Triangle = [Vector3; 3];

pub(super) fn bvh(triangles: &[Triangle]) -> TraceableRef {
    let items: Vec<TraceableRef> = triangles
        .iter()
        .enumerate()
        .map(|(i, t)| Arc::new(TriangleShape::new(t[0], t[1], t[2], i as i32)) as TraceableRef)
        .collect();
    BoundingVolumeHierarchy::create_new_hierarchy(items, BvhCreationOptions::LocalOrderClustering)
        .expect("a mesh has triangles")
}

/// A grid of `columns` x `rows` quads of `f(u, v)`, u and v running 0 to 1 and wrapping.
pub(super) fn grid_mesh(
    columns: usize,
    rows: usize,
    f: impl Fn(f64, f64) -> Vector3,
) -> Vec<Triangle> {
    let mut triangles = Vec::new();
    let p = |c: usize, r: usize| f(c as f64 / columns as f64, r as f64 / rows as f64);
    for r in 0..rows {
        for c in 0..columns {
            let (a, b, c2, d) = (p(c, r), p(c + 1, r), p(c + 1, r + 1), p(c, r + 1));
            triangles.push([a, b, c2]);
            triangles.push([a, c2, d]);
        }
    }
    triangles
}

pub(super) fn sphere(columns: usize, rows: usize) -> Vec<Triangle> {
    grid_mesh(columns, rows, |u, w| {
        let (theta, phi) = (u * std::f64::consts::TAU, w * std::f64::consts::PI);
        v(
            2.0 * phi.sin() * theta.cos(),
            2.0 * phi.sin() * theta.sin(),
            2.0 * phi.cos(),
        )
    })
}

pub(super) fn torus(columns: usize, rows: usize) -> Vec<Triangle> {
    grid_mesh(columns, rows, |u, w| {
        let (a, b) = (u * std::f64::consts::TAU, w * std::f64::consts::TAU);
        let radius = 2.0 + 0.7 * b.cos();
        v(radius * a.cos(), radius * a.sin(), 0.7 * b.sin())
    })
}

/// A bumpy sphere, lumpy enough that tree boxes overlap and neighbouring rays disagree.
pub(super) fn irregular(columns: usize, rows: usize) -> Vec<Triangle> {
    grid_mesh(columns, rows, |u, w| {
        let (theta, phi) = (u * std::f64::consts::TAU, w * std::f64::consts::PI);
        let radius = 1.5 + 0.5 * (7.0 * theta + 2.0).sin() * (5.0 * phi).cos();
        v(
            radius * phi.sin() * theta.cos(),
            radius * phi.cos() + 0.1 * (3.0 * theta).sin(),
            radius * phi.sin() * theta.sin(),
        )
    })
}

pub(super) fn cube() -> Vec<Triangle> {
    let corner = |x: f64, y: f64, z: f64| v(x, y, z);
    let (a, b, c, d) = (
        corner(-1.0, -1.0, -1.0),
        corner(1.0, -1.0, -1.0),
        corner(1.0, 1.0, -1.0),
        corner(-1.0, 1.0, -1.0),
    );
    let (e, f, g, h) = (
        corner(-1.0, -1.0, 1.0),
        corner(1.0, -1.0, 1.0),
        corner(1.0, 1.0, 1.0),
        corner(-1.0, 1.0, 1.0),
    );
    vec![
        [a, c, b],
        [a, d, c],
        [e, f, g],
        [e, g, h],
        [a, b, f],
        [a, f, e],
        [d, g, c],
        [d, h, g],
        [a, e, h],
        [a, h, d],
        [b, c, g],
        [b, g, f],
    ]
}

/// Rays from `eye` through a `size` x `size` grid on a plane facing it.
pub(super) fn perspective_grid(eye: Vector3, size: usize, half_width: f64) -> Vec<Ray> {
    let mut rays = Vec::with_capacity(size * size);
    for y in 0..size {
        for x in 0..size {
            let px = -half_width + 2.0 * half_width * (x as f64 + 0.5) / size as f64;
            let py = -half_width + 2.0 * half_width * (y as f64 + 0.5) / size as f64;
            let target = v(px, py, 0.0);
            rays.push(Ray::new(eye, (target - eye).get_normal()));
        }
    }
    rays
}

/// Parallel rays through the same grid, going `direction`.
pub(super) fn orthographic_grid(direction: Vector3, size: usize, half_width: f64) -> Vec<Ray> {
    let mut rays = Vec::with_capacity(size * size);
    for y in 0..size {
        for x in 0..size {
            let px = -half_width + 2.0 * half_width * (x as f64 + 0.5) / size as f64;
            let py = -half_width + 2.0 * half_width * (y as f64 + 0.5) / size as f64;
            rays.push(Ray::new(v(px, py, 20.0), direction));
        }
    }
    rays
}

/// A deterministic jitter in [-0.5, 0.5), for rays no two of which line up.
pub(super) struct Noise(pub(super) u64);

impl Noise {
    pub(super) fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 33) as f64 / (1u64 << 31) as f64) - 0.5
    }
}

pub(super) fn jittered(mut rays: Vec<Ray>, amount: f64) -> Vec<Ray> {
    let mut noise = Noise(0x9E3779B97F4A7C15);
    for ray in &mut rays {
        let origin = ray.origin
            + v(
                noise.next() * amount,
                noise.next() * amount,
                noise.next() * amount,
            );
        let direction = ray.direction_normal
            + v(
                noise.next() * amount * 0.05,
                noise.next() * amount * 0.05,
                noise.next() * amount * 0.05,
            );
        *ray = Ray::new(origin, direction.get_normal());
    }
    rays
}

/// Compares a result with the lone trace of its ray, field by field and bit by bit.
pub(super) fn assert_same_hit(
    label: &str,
    index: usize,
    bundle_result: Option<&IntersectInfo<'_>>,
    lone_result: Option<IntersectInfo<'_>>,
) {
    let (ours, lone) = match (bundle_result, lone_result) {
        (None, None) => return,
        (Some(ours), Some(lone)) => (ours, lone),
        (ours, lone) => panic!(
            "{label} ray {index}: bundle {:?}, lone {:?}",
            ours.map(|o| o.distance_to_hit),
            lone.map(|o| o.distance_to_hit)
        ),
    };
    assert_eq!(
        ours.hit_type, lone.hit_type,
        "{label} ray {index}: hit type"
    );
    let same_object = match (ours.closest_hit_object, lone.closest_hit_object) {
        (None, None) => true,
        (Some(a), Some(b)) => {
            std::ptr::addr_eq(a as *const dyn Traceable, b as *const dyn Traceable)
        }
        _ => false,
    };
    assert!(same_object, "{label} ray {index}: a different object");
    assert_eq!(
        ours.distance_to_hit.to_bits(),
        lone.distance_to_hit.to_bits(),
        "{label} ray {index}: distance"
    );
    for axis in 0..3 {
        assert_eq!(
            ours.hit_position[axis].to_bits(),
            lone.hit_position[axis].to_bits(),
            "{label} ray {index}: position"
        );
        assert_eq!(
            ours.normal_at_hit[axis].to_bits(),
            lone.normal_at_hit[axis].to_bits(),
            "{label} ray {index}: normal"
        );
    }
}

/// Traces `rays` in bundles of each size, and checks each against its lone trace.
pub(super) fn assert_bundles_match_lone(
    label: &str,
    shape: &TraceableRef,
    rays: &[Ray],
    sizes: &[usize],
) {
    for &size in sizes {
        let size = size.min(rays.len()).max(1);
        for chunk_index in 0..rays.len().div_ceil(size) {
            let start = chunk_index * size;
            let chunk: Vec<Ray> = rays[start..(start + size).min(rays.len())].to_vec();
            let mut bundle = RayBundle::new(chunk.clone());
            bundle.trace(shape.as_ref()).expect("a valid bundle");
            for (offset, ray) in chunk.iter().enumerate() {
                let mut lone = ray.clone();
                let lone_result = shape.get_closest_intersection(&mut lone);
                let label = format!("{label}, bundles of {size}");
                assert_same_hit(
                    &label,
                    start + offset,
                    bundle.results()[offset].as_ref(),
                    lone_result,
                );
                assert_eq!(
                    bundle.rays()[offset].max_distance_to_consider.to_bits(),
                    lone.max_distance_to_consider.to_bits(),
                    "{label} ray {}: final window",
                    start + offset
                );
            }
        }
    }
}

pub(super) const ALL_SIZES: &[usize] = &[1, 2, 3, 7, 256, 1024, usize::MAX];

fn every_mesh() -> Vec<(&'static str, TraceableRef)> {
    vec![
        ("sphere", bvh(&sphere(24, 16))),
        ("cube", bvh(&cube())),
        ("torus", bvh(&torus(32, 16))),
        ("irregular", bvh(&irregular(40, 24))),
    ]
}

#[test]
fn perspective_grids_match_lone_rays() {
    for (name, mesh) in every_mesh() {
        for eye in [v(0.3, 0.2, 9.0), v(-6.0, 3.0, 2.0), v(0.0, 0.0, 12.0)] {
            let rays = perspective_grid(eye, 32, 4.0);
            assert_bundles_match_lone(
                &format!("{name} perspective {eye}"),
                &mesh,
                &rays,
                ALL_SIZES,
            );
        }
    }
}

#[test]
fn orthographic_grids_match_lone_rays() {
    for (name, mesh) in every_mesh() {
        for direction in [
            v(0.0, 0.0, -1.0),
            v(0.1, -0.1, -1.0).get_normal(),
            v(-0.05, 0.3, -1.0).get_normal(),
        ] {
            let rays = orthographic_grid(direction, 32, 3.0);
            assert_bundles_match_lone(&format!("{name} ortho"), &mesh, &rays, &[1, 7, 16, 256]);
        }
    }
}

#[test]
fn jittered_rays_with_no_shared_origin_match_lone_rays() {
    for (name, mesh) in every_mesh() {
        let rays = jittered(perspective_grid(v(1.0, 1.0, 10.0), 32, 4.0), 0.4);
        assert_bundles_match_lone(&format!("{name} jittered"), &mesh, &rays, ALL_SIZES);
    }
}

#[test]
fn shadow_rays_with_finite_windows_match_lone_rays() {
    let light = v(5.0, 9.0, 6.0);
    for (name, mesh) in every_mesh() {
        // Rays from a grid of points toward the light, each stopping short of it by a different
        // amount, some passing clear of the mesh and some through it.
        let mut rays = Vec::new();
        let mut noise = Noise(7);
        for y in 0..32 {
            for x in 0..32 {
                let start = v(
                    -3.0 + 6.0 * (x as f64) / 32.0,
                    -3.0 + 6.0 * (y as f64) / 32.0,
                    -4.0 + noise.next(),
                );
                let distance = (light - start).length();
                let mut ray = Ray::with_limits(
                    start,
                    (light - start).get_normal(),
                    0.0,
                    distance * (0.5 + 0.5 * noise.next().abs()),
                    IntersectionType::FrontFace,
                );
                ray.is_shadow_ray = true;
                rays.push(ray);
            }
        }
        assert_bundles_match_lone(&format!("{name} shadow"), &mesh, &rays, ALL_SIZES);
    }
}

#[test]
fn rays_starting_inside_a_part_match_lone_rays() {
    for (name, mesh) in every_mesh() {
        let mut rays = Vec::new();
        for y in 0..24 {
            for x in 0..24 {
                let (a, b) = (x as f64 / 24.0, y as f64 / 24.0);
                let direction = v((a - 0.5) * 2.0, (b - 0.5) * 2.0, (a + b - 1.0) + 0.17);
                let mut ray = Ray::new(v(0.05, -0.1, 0.2), direction.get_normal());
                ray.intersection_type = if (x + y) % 2 == 0 {
                    IntersectionType::Both
                } else {
                    IntersectionType::BackFace
                };
                rays.push(ray);
            }
        }
        assert_bundles_match_lone(&format!("{name} inside"), &mesh, &rays, ALL_SIZES);
    }
}

/// Rays along an axis, with zero (and negative zero) direction components and origins exactly
/// on box planes: where the slab test meets `0 * inf`.
#[test]
fn axis_aligned_rays_on_slab_planes_match_lone_rays() {
    let mesh = bvh(&cube());
    let mut rays = Vec::new();
    for &direction in &[
        v(0.0, 0.0, -1.0),
        v(-0.0, 0.0, -1.0),
        v(0.0, -0.0, 1.0),
        v(1.0, 0.0, 0.0),
        v(0.0, -1.0, -0.0),
    ] {
        for x in -3..=3 {
            for y in -3..=3 {
                let (a, b) = (x as f64 / 3.0, y as f64 / 3.0);
                let origin = if direction.z != 0.0 {
                    v(a, b, -direction.z * 5.0)
                } else if direction.x != 0.0 {
                    v(-direction.x * 5.0, a, b)
                } else {
                    v(a, -direction.y * 5.0, b)
                };
                rays.push(Ray::new(origin, direction));
                // The same rays exactly on the unit cube's planes.
                let on_plane = if direction.z != 0.0 {
                    v(a.round(), b.round(), -direction.z * 5.0)
                } else if direction.x != 0.0 {
                    v(-direction.x * 5.0, a.round(), b.round())
                } else {
                    v(a.round(), -direction.y * 5.0, b.round())
                };
                rays.push(Ray::new(on_plane, direction));
            }
        }
    }
    assert_bundles_match_lone("axis aligned", &mesh, &rays, &[1, 2, 3, 7, 16, 256, 1024]);
}

/// Rays aimed exactly at edges two triangles share, at vertices, and at duplicated coplanar
/// triangles: where equal distances decide the winner and child order matters.
#[test]
fn rays_at_shared_edges_vertices_and_coincident_faces_match_lone_rays() {
    let triangles = torus(16, 8);
    let mut duplicated = triangles.clone();
    duplicated.extend(triangles.iter().copied());
    duplicated.extend(triangles.iter().map(|t| [t[0], t[2], t[1]]));
    let duplicate_mesh = bvh(&duplicated);
    let mesh = bvh(&triangles);
    let eye = v(0.6, 0.1, 9.0);
    let mut rays = Vec::new();
    for t in &triangles {
        for point in [
            (t[0] + t[1]) / 2.0,
            (t[1] + t[2]) / 2.0,
            (t[2] + t[0]) / 2.0,
            t[0],
            t[1],
            t[2],
        ] {
            rays.push(Ray::new(eye, (point - eye).get_normal()));
        }
    }
    for (name, shape) in [("edges", &mesh), ("coincident faces", &duplicate_mesh)] {
        assert_bundles_match_lone(name, shape, &rays, ALL_SIZES);
    }

    // Rays on the far side of the view axis, so rays of one bundle disagree about child order
    // at the nodes they share, and the winner of a tie is decided by the side each comes from.
    let mut straddling = Vec::new();
    for t in &duplicated {
        let point = (t[0] + t[1]) / 2.0;
        for eye in [
            v(-7.0, 0.3, 4.0),
            v(7.0, 0.3, 4.0),
            v(0.3, -7.0, 4.0),
            v(0.3, 7.0, 4.0),
        ] {
            straddling.push(Ray::new(eye, (point - eye).get_normal()));
        }
    }
    assert_bundles_match_lone(
        "mixed-order ties",
        &duplicate_mesh,
        &straddling,
        &[16, 256, 1024],
    );
}

#[test]
fn views_straddling_every_octant_match_lone_rays() {
    for (name, mesh) in every_mesh() {
        // A camera inside the mesh looking everywhere: one bundle holds rays in all octants.
        let mut rays = Vec::new();
        for z in 0..16 {
            for y in 0..16 {
                for x in 0..16 {
                    let direction = v(x as f64 - 7.5, y as f64 - 7.5, z as f64 - 7.5);
                    rays.push(Ray::new(v(0.2, 0.1, 0.3), direction.get_normal()));
                }
            }
        }
        assert_bundles_match_lone(
            &format!("{name} octants"),
            &mesh,
            &rays,
            &[7, 16, 256, 1024, 4096],
        );
    }
}

#[test]
fn nested_transforms_match_lone_rays() {
    let mesh = bvh(&sphere(24, 12));
    let translate = Matrix4X4::create_translation(3.0, -1.0, 2.0);
    let rotate = Matrix4X4::create_rotation_y(0.7) * Matrix4X4::create_rotation_x(-1.1);
    let scale = Matrix4X4::create_scale(1.5, 0.5, 2.0);
    let mirror = Matrix4X4::create_scale(-1.0, 1.0, 1.0);
    let inner: TraceableRef = Arc::new(Transform::with_matrix(mesh.clone(), rotate));
    let scaled: TraceableRef = Arc::new(Transform::with_matrix(inner, scale));
    let mirrored: TraceableRef = Arc::new(Transform::with_matrix(scaled, mirror));
    let placed: TraceableRef = Arc::new(Transform::with_matrix(mirrored, translate));
    for eye in [v(3.0, -1.0, 12.0), v(10.0, 4.0, 5.0)] {
        let rays = perspective_grid(eye, 32, 6.0);
        assert_bundles_match_lone("nested transforms", &placed, &rays, ALL_SIZES);
    }

    // A rotating transform can turn a world-space bundle in one octant into local rays of mixed
    // sign: its child sees the rays it really gets, on a mesh with ties.
    let ties = bvh(&cube());
    let quarter: TraceableRef = Arc::new(Transform::with_matrix(
        ties,
        Matrix4X4::create_rotation_y(0.9),
    ));
    let rays = orthographic_grid(v(0.0, 0.0, -1.0), 24, 2.0);
    assert_bundles_match_lone("rotated cube", &quarter, &rays, &[16, 256, 576]);
}

#[test]
fn unbound_collection_and_a_plane_match_lone_rays() {
    let a = bvh(&sphere(16, 12));
    let b: TraceableRef = Arc::new(Transform::with_matrix(
        bvh(&cube()),
        Matrix4X4::create_translation(2.5, 0.0, 1.0),
    ));
    let plane: TraceableRef = Arc::new(PlaneShape::new(Plane::new(v(0.0, 0.0, 1.0), -3.0)));
    let unbound: TraceableRef = Arc::new(UnboundCollection::new(vec![
        a.clone(),
        b.clone(),
        plane.clone(),
    ]));
    let rays = perspective_grid(v(1.0, 0.0, 15.0), 32, 7.0);
    assert_bundles_match_lone("unbound collection", &unbound, &rays, ALL_SIZES);

    // A plane under a BVH widens a ray's window past a mesh behind it: a lone ray then tests the
    // sibling it would have skipped, and the bundle must follow.
    let with_plane = BoundingVolumeHierarchy::new(a, plane);
    let shape: TraceableRef = Arc::new(BoundingVolumeHierarchy::new(Arc::new(with_plane), b));
    assert_bundles_match_lone("plane in a tree", &shape, &rays, ALL_SIZES);
}

#[test]
fn a_scene_traced_as_a_bundle_matches_primary_rays_one_at_a_time() {
    let mut scene = Scene::new();
    scene.shapes.push(bvh(&sphere(16, 12)));
    scene.shapes.push(Arc::new(PlaneShape::new(Plane::new(
        v(0.0, 0.0, 1.0),
        -5.0,
    ))));
    let rays = perspective_grid(v(0.5, 0.5, 14.0), 16, 8.0);
    let tracer = RayTracer::new();
    let mut bundle = RayBundle::new(rays.clone());
    tracer
        .trace_ray_bundle(&mut bundle, &scene)
        .expect("a valid bundle");
    for (index, ray) in rays.iter().enumerate() {
        let mut lone = ray.clone();
        let lone_result = tracer.trace_primary_ray(&mut lone, &scene);
        assert_same_hit(
            "scene",
            index,
            bundle.results()[index].as_ref(),
            Some(lone_result),
        );
        assert_eq!(
            bundle.rays()[index].max_distance_to_consider.to_bits(),
            lone.max_distance_to_consider.to_bits(),
            "scene ray {index}: final window"
        );
    }
}

/// The rays of a row-major `width` x `width` image cut into square tiles of `tile` x `tile`.
pub(super) fn tiles(rays: &[Ray], width: usize, tile: usize) -> Vec<Vec<Ray>> {
    let mut result = Vec::new();
    for tile_y in (0..width).step_by(tile) {
        for tile_x in (0..width).step_by(tile) {
            let mut one = Vec::with_capacity(tile * tile);
            for y in tile_y..tile_y + tile {
                for x in tile_x..tile_x + tile {
                    one.push(rays[y * width + x].clone());
                }
            }
            result.push(one);
        }
    }
    result
}

/// Coherent 16x16 tiles of a closed mesh against the same rays one at a time: the reduction
/// that makes bundles worth having. The bound carries a small margin over what was measured.
#[test]
fn a_coherent_bundle_needs_far_fewer_box_tests() {
    let mesh = bvh(&sphere(256, 128));
    let rays = perspective_grid(v(0.5, 0.25, 10.0), 256, 3.0);
    let mut lone_box_tests = 0;
    for ray in &rays {
        let mut bundle = RayBundle::new(vec![ray.clone()]);
        bundle.trace(mesh.as_ref()).expect("a valid bundle");
        lone_box_tests += bundle.stats().box_tests;
    }
    let mut packet_tests = 0;
    for tile in tiles(&rays, 256, 16) {
        let mut bundle = RayBundle::new(tile);
        bundle.trace(mesh.as_ref()).expect("a valid bundle");
        packet_tests += bundle.stats().box_tests + bundle.stats().interval_tests;
    }
    let reduction = lone_box_tests as f64 / packet_tests as f64;
    assert!(
        reduction >= 1.5,
        "packets cut box tests by only {reduction:.2}x ({lone_box_tests} against {packet_tests})"
    );
}
