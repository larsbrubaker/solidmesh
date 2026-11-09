//! The GPU Ray Tracing page's scene and ray sets: the Ray Tracing page's part on its floor, as a
//! flat tree (`solidmesh::mesh::ray_tracer::flat_tree`), and three byte-identical ray sets for the
//! GPU (`demo/src/gpu-bench.ts`, in WebGPU on the main thread) and for this reference to trace:
//!
//! - camera rays: one per pixel of a fixed picture, coherent, closest hit;
//! - shadow rays: from the first hits toward one point light, any-hit, skipping surfaces that
//!   face away from the light, since a renderer never traces those;
//! - bounce rays: one random cosine-weighted direction per first hit, incoherent, closest hit.
//!
//! The rays come from this reference's own first hits, never the GPU's, so a GPU bug cannot change
//! what the two are asked to trace. `gpu_bench_compare` turns the GPU's hit bytes into an
//! agreement count and the picture.
//!
//! The clock is a parameter: `std::time::Instant` is not available on the web target and the
//! reference runs in native tests.

use js_sys::{Object, Reflect, Uint8Array};
use solidmesh::math::Vector3;
use solidmesh::mesh::ray_tracer::flat_tree::{
    FlatHit, FlatRay, FlatTree, FACE_BOTH, RAY_STRIDE, TRACE_WGSL,
};
use wasm_bindgen::prelude::*;

use crate::ray_trace_scene::{part_mesh, Camera, PlacedOnFloor};

/// The picture's size, fixed so the page's layout can never feed back into the work.
pub const IMAGE_WIDTH: u32 = 480;
pub const IMAGE_HEIGHT: u32 = 360;

/// The view: the same kind of three-quarter orbit the Ray Tracing page opens with.
const YAW: f64 = 0.9;
const PITCH: f64 = 0.55;

/// How far a secondary ray's origin leaves the surface, as a fraction of the part's radius, plus
/// a term for `f32` rounding growing with the hit's distance.
const OFFSET_RADII: f32 = 1e-4;
const OFFSET_PER_DISTANCE: f32 = 4e-6;

/// The seed of the bounce rays' directions.
const BOUNCE_SEED: u64 = 0x1234_5678_9abc_def1;

/// One of the three ray sets, by the index the page sends.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RayKind {
    Camera,
    Shadow,
    Bounce,
}

impl RayKind {
    pub const ALL: [RayKind; 3] = [RayKind::Camera, RayKind::Shadow, RayKind::Bounce];

    pub fn index(self) -> usize {
        match self {
            RayKind::Camera => 0,
            RayKind::Shadow => 1,
            RayKind::Bounce => 2,
        }
    }

    /// The kind an index names, or an error naming the index.
    pub fn from_index(index: u32) -> Result<RayKind, String> {
        RayKind::ALL
            .get(index as usize)
            .copied()
            .ok_or_else(|| format!("There is no ray set {index} on this page."))
    }

    /// True for the any-hit rays: a shadow ray stops at the first thing it finds.
    pub fn is_any_hit(self) -> bool {
        self == RayKind::Shadow
    }
}

/// What the prepared scene came to, for the page's readout.
#[derive(Debug, Clone, PartialEq)]
pub struct BenchInfo {
    pub triangles: u64,
    pub node_count: u64,
    pub node_bytes: u64,
    pub triangle_bytes: u64,
    pub depth: u32,
    pub note: String,
    pub mesh_ms: f64,
    pub tree_ms: f64,
}

/// A scene ready to trace.
pub struct Bench {
    pub(crate) tree: FlatTree,
    pub(crate) rays: [Vec<FlatRay>; 3],
    /// For each picture pixel in raster order, its shadow ray's index (`NO_RAY` when the pixel
    /// has none: it faced away from the light, or found nothing).
    pub(crate) shadow_of_pixel: Vec<u32>,
    pub(crate) light: [f32; 3],
    pub info: BenchInfo,
}

/// A pixel with no shadow ray.
pub(crate) const NO_RAY: u32 = u32::MAX;

fn add(a: [f32; 3], b: [f32; 3], k: f32) -> [f32; 3] {
    [a[0] + b[0] * k, a[1] + b[1] * k, a[2] + b[2] * k]
}

fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub(crate) fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

pub(crate) fn normalized(a: [f32; 3]) -> [f32; 3] {
    let length = dot(a, a).sqrt();
    if length > 0.0 {
        [a[0] / length, a[1] / length, a[2] / length]
    } else {
        [0.0, 0.0, 1.0]
    }
}

/// The unit normal of a triangle of the tree, facing the side the ray came from.
pub(crate) fn facing_normal(tree: &FlatTree, hit: &FlatHit, direction: [f32; 3]) -> [f32; 3] {
    let triangle = &tree.triangles()[hit.triangle as usize];
    let normal = normalized(cross(
        sub(triangle.v1, triangle.v0),
        sub(triangle.v2, triangle.v0),
    ));
    if dot(normal, direction) > 0.0 {
        [-normal[0], -normal[1], -normal[2]]
    } else {
        normal
    }
}

/// The next deterministic sample in `0..1`.
fn next_sample(state: &mut u64) -> f32 {
    *state = state.wrapping_mul(0x5851_f42d_4c95_7f2d).wrapping_add(1);
    ((*state >> 33) as f32) / ((1u64 << 31) as f32)
}

/// A unit vector, near-uniform on the sphere (by rejection in a cube, which is plenty for ray
/// directions).
fn random_unit(state: &mut u64) -> [f32; 3] {
    loop {
        let v = [
            next_sample(state) * 2.0 - 1.0,
            next_sample(state) * 2.0 - 1.0,
            next_sample(state) * 2.0 - 1.0,
        ];
        let length_squared = dot(v, v);
        if length_squared > 1e-4 && length_squared <= 1.0 {
            return normalized(v);
        }
    }
}

impl Bench {
    /// Builds part `part` at detail `detail` on its floor, the flat tree over them and the three
    /// ray sets. `max_storage_binding` is the GPU's real limit; the tree refuses a scene that
    /// exceeds it, and that refusal is the error text the page shows.
    pub fn new(
        part: u32,
        detail: u32,
        max_storage_binding: u64,
        now: &mut dyn FnMut() -> f64,
    ) -> Result<Bench, String> {
        let start = now();
        let (mesh, note) = part_mesh(part, detail)?;
        let placed = PlacedOnFloor::around(&mesh);
        let meshed = now();

        let mut triangles = Vec::with_capacity(mesh.faces.len() + placed.floor.faces.len());
        for source in [&mesh, &placed.floor] {
            for face in source.faces.iter() {
                triangles.push([
                    source.vertices[face.v0 as usize].as_vector3(),
                    source.vertices[face.v1 as usize].as_vector3(),
                    source.vertices[face.v2 as usize].as_vector3(),
                ]);
            }
        }
        let tree = FlatTree::build(&triangles, max_storage_binding).map_err(|e| e.to_string())?;
        let built = now();

        let camera = Camera::around(
            placed.center,
            placed.radius,
            YAW,
            PITCH,
            1.0,
            IMAGE_WIDTH,
            IMAGE_HEIGHT,
        )?;
        let light =
            placed.center + Vector3::new(0.55, -0.45, 0.7).get_normal() * placed.radius * 8.0;
        let light = [light.x as f32, light.y as f32, light.z as f32];
        let radius = placed.radius as f32;

        let mut camera_rays = Vec::with_capacity((IMAGE_WIDTH * IMAGE_HEIGHT) as usize);
        for py in 0..IMAGE_HEIGHT {
            for px in 0..IMAGE_WIDTH {
                let mut ray =
                    FlatRay::from_ray(&camera.ray(f64::from(px) + 0.5, f64::from(py) + 0.5));
                ray.face_mask = FACE_BOTH;
                camera_rays.push(ray);
            }
        }

        let mut shadow_rays = Vec::new();
        let mut bounce_rays = Vec::new();
        let mut shadow_of_pixel = vec![NO_RAY; camera_rays.len()];
        let mut state = BOUNCE_SEED;
        for (pixel, ray) in camera_rays.iter().enumerate() {
            let Some(hit) = tree.trace_closest(ray) else {
                continue;
            };
            let normal = facing_normal(&tree, &hit, ray.direction);
            let point = add(ray.origin, ray.direction, hit.t);
            let origin = add(
                point,
                normal,
                radius * OFFSET_RADII + hit.t * OFFSET_PER_DISTANCE,
            );

            let to_light = sub(light, origin);
            let distance = dot(to_light, to_light).sqrt();
            let direction = normalized(to_light);
            if dot(direction, normal) > 0.0 {
                shadow_of_pixel[pixel] = shadow_rays.len() as u32;
                shadow_rays.push(FlatRay {
                    origin,
                    direction,
                    t_min: 0.0,
                    t_max: distance,
                    face_mask: FACE_BOTH,
                });
            }

            // Lambertian: the normal plus a random unit vector, resampled when they nearly cancel.
            let bounce = loop {
                let sample = random_unit(&mut state);
                let summed = add(normal, sample, 1.0);
                if dot(summed, summed) > 1e-6 {
                    break normalized(summed);
                }
            };
            bounce_rays.push(FlatRay {
                origin,
                direction: bounce,
                t_min: 0.0,
                t_max: f32::MAX,
                face_mask: FACE_BOTH,
            });
        }

        let sizes = tree.sizes();
        Ok(Bench {
            rays: [camera_rays, shadow_rays, bounce_rays],
            shadow_of_pixel,
            light,
            info: BenchInfo {
                triangles: sizes.triangle_count,
                node_count: sizes.node_count,
                node_bytes: sizes.node_bytes,
                triangle_bytes: sizes.triangle_bytes,
                depth: sizes.depth,
                note,
                mesh_ms: meshed - start,
                tree_ms: built - meshed,
            },
            tree,
        })
    }

    /// The rays of `kind`.
    pub fn rays(&self, kind: RayKind) -> &[FlatRay] {
        &self.rays[kind.index()]
    }

    /// The rays of `kind` as a buffer for the shader, one [`RAY_STRIDE`]-byte record per ray.
    pub fn ray_bytes(&self, kind: RayKind) -> Vec<u8> {
        let rays = self.rays(kind);
        let mut bytes = Vec::with_capacity(rays.len() * RAY_STRIDE);
        for ray in rays {
            bytes.extend_from_slice(&ray.to_le_bytes());
        }
        bytes
    }

    /// The tree.
    pub fn tree(&self) -> &FlatTree {
        &self.tree
    }

    /// One thread's rate on the reference traversal: traces every ray of `kind`, and keeps going
    /// over them until `budget_ms` has passed, so a tiny ray set still measures for a while.
    /// Answers the rays traced and the time they took.
    pub fn reference_run(
        &self,
        kind: RayKind,
        budget_ms: f64,
        now: &mut dyn FnMut() -> f64,
    ) -> (u64, f64) {
        let start = now();
        let rays = self.rays(kind);
        let mut traced: u64 = 0;
        let mut found: u64 = 0;
        loop {
            for ray in rays {
                let hit = if kind.is_any_hit() {
                    self.tree.trace_any(ray)
                } else {
                    self.tree.trace_closest(ray).is_some()
                };
                found += u64::from(hit);
            }
            traced += rays.len() as u64;
            let elapsed = now() - start;
            if elapsed >= budget_ms || rays.is_empty() {
                // Keeps the hit count observable, so the traces are not optimized away.
                std::hint::black_box(found);
                return (traced, elapsed);
            }
        }
    }
}

thread_local! {
    /// The scene the GPU Ray Tracing page last prepared.
    static BENCH: std::cell::RefCell<Option<Bench>> = const { std::cell::RefCell::new(None) };
}

fn with_bench<R>(f: impl FnOnce(&Bench) -> Result<R, JsError>) -> Result<R, JsError> {
    BENCH.with(|bench| {
        let bench = bench.borrow();
        let bench = bench
            .as_ref()
            .ok_or_else(|| JsError::new("The scene is not ready: run the benchmark again."))?;
        f(bench)
    })
}

fn fields(pairs: &[(&str, JsValue)]) -> Result<Object, JsError> {
    let object = Object::new();
    for (name, value) in pairs {
        Reflect::set(&object, &(*name).into(), value)
            .map_err(|_| JsError::new("The results could not be handed to the page."))?;
    }
    Ok(object)
}

fn kind_named(index: u32) -> Result<RayKind, JsError> {
    RayKind::from_index(index).map_err(|e| JsError::new(&e))
}

/// GPU Ray Tracing page: builds the scene on this worker and answers its buffers as `Uint8Array`s
/// the page uploads, the shader's text, and the sizes. `max_binding` is the adapter's largest
/// storage binding in bytes (a double, since a limit can exceed `u32`); `as` saturates.
#[wasm_bindgen]
pub fn gpu_bench_prepare(part: u32, detail: u32, max_binding: f64) -> Result<Object, JsError> {
    BENCH.with(|bench| *bench.borrow_mut() = None);
    let bench = Bench::new(part, detail, max_binding as u64, &mut js_sys::Date::now)
        .map_err(|e| JsError::new(&e))?;
    let info = &bench.info;
    let result = fields(&[
        ("shader", TRACE_WGSL.into()),
        (
            "nodes",
            Uint8Array::from(bench.tree.node_bytes().as_slice()).into(),
        ),
        (
            "triangles",
            Uint8Array::from(bench.tree.triangle_bytes().as_slice()).into(),
        ),
        (
            "rays0",
            Uint8Array::from(bench.ray_bytes(RayKind::Camera).as_slice()).into(),
        ),
        (
            "rays1",
            Uint8Array::from(bench.ray_bytes(RayKind::Shadow).as_slice()).into(),
        ),
        (
            "rays2",
            Uint8Array::from(bench.ray_bytes(RayKind::Bounce).as_slice()).into(),
        ),
        ("triangleCount", (info.triangles as f64).into()),
        ("nodeCount", (info.node_count as f64).into()),
        ("nodeBytes", (info.node_bytes as f64).into()),
        ("triangleBytes", (info.triangle_bytes as f64).into()),
        ("depth", info.depth.into()),
        ("note", info.note.clone().into()),
        ("meshMs", info.mesh_ms.into()),
        ("treeMs", info.tree_ms.into()),
        ("width", IMAGE_WIDTH.into()),
        ("height", IMAGE_HEIGHT.into()),
    ]);
    BENCH.with(|slot| *slot.borrow_mut() = Some(bench));
    result
}

/// GPU Ray Tracing page: traces every ray of set `kind` on the reference traversal (this worker,
/// one thread), for at least `budget_ms`; answers the rays traced and the milliseconds taken.
#[wasm_bindgen]
pub fn gpu_bench_reference(kind: u32, budget_ms: f64) -> Result<Object, JsError> {
    let kind = kind_named(kind)?;
    with_bench(|bench| {
        let (rays, ms) = bench.reference_run(kind, budget_ms, &mut js_sys::Date::now);
        fields(&[("rays", (rays as f64).into()), ("ms", ms.into())])
    })
}

/// GPU Ray Tracing page: compares the GPU's hit records of set `kind` with the reference and
/// answers the counts (`gpu_bench_compare`).
#[wasm_bindgen]
pub fn gpu_bench_compare(kind: u32, hits: &[u8]) -> Result<Object, JsError> {
    let kind = kind_named(kind)?;
    with_bench(|bench| {
        let agreement = bench.compare(kind, hits).map_err(|e| JsError::new(&e))?;
        fields(&[
            ("compared", (agreement.compared as f64).into()),
            ("exact", (agreement.exact as f64).into()),
            ("ties", (agreement.ties as f64).into()),
            ("disagreed", (agreement.disagreed as f64).into()),
            ("examples", agreement.examples.join("\n").into()),
        ])
    })
}

/// GPU Ray Tracing page: the picture drawn from the GPU's camera and shadow hits, as RGBA8 in
/// raster order of [`IMAGE_WIDTH`] by [`IMAGE_HEIGHT`].
#[wasm_bindgen]
pub fn gpu_bench_picture(camera_hits: &[u8], shadow_hits: &[u8]) -> Result<Object, JsError> {
    with_bench(|bench| {
        let pixels = bench
            .picture(camera_hits, shadow_hits)
            .map_err(|e| JsError::new(&e))?;
        fields(&[
            ("pixels", Uint8Array::from(pixels.as_slice()).into()),
            ("width", IMAGE_WIDTH.into()),
            ("height", IMAGE_HEIGHT.into()),
        ])
    })
}
