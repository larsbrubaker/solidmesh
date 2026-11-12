//! Clear glass resting on a matte floor: a ray inside the glass heading down meets the glass's own
//! surface first and then the floor across the contact, whatever the order of the parts and
//! whichever face a trace returns, and the GPU route's rule (`leaves_onto_floor_f32`, the Rust
//! mirror of the path loop's test, on the hit the `f32` reference traversal finds) decides the
//! same, ray for ray, with its constants the shader's; a box resting on the floor renders as one
//! lifted a hundredth of a millimetre off it; no path reaches the ground below the floor through
//! the glass; and a 3 by 3 grid of touching boxes renders as the slab they make.

use naga::Literal;
use solidmesh_math::{IntersectionType, Matrix4X4, Ray, Vector3};
use solidmesh_mesh::ray_tracer::flat_tree::{
    leaves_onto_floor_f32, FlatRay, GpuCompositing, GpuFloor, GpuRenderScene, GpuSceneLimits,
    GpuView, FACE_BOTH, FLOOR_OFFSET_STEPS, FLOOR_REST_ANTI_PARALLEL, FLOOR_REST_ROUNDING_STEPS,
    PATH_WGSL, WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
};
use solidmesh_mesh::ray_tracer::{
    Environment, LightSetup, PathTraceSettings, PathTracer, PlacedPart, PlacedParts, RenderCamera,
    RenderScene, RenderWork, ScatteringSurface,
};

use crate::flat_tree_shader_tests::literal;
use crate::glass_tray_support::{box_meshes, part_of};
use crate::path_tracer_support::{agreement_tolerance, flat_patch};
use crate::scattering_support::Stats;

type Boxes = Vec<([f64; 3], [f64; 3])>;

pub(crate) fn one_box(lift: f64) -> Boxes {
    vec![([20.0; 3], [0.0, 0.0, 10.0 + lift])]
}

fn slab() -> Boxes {
    vec![([60.0, 60.0, 20.0], [0.0, 0.0, 10.0])]
}

fn grid() -> Boxes {
    (0..9)
        .map(|index| {
            let (column, row) = ((index / 3) as f64, (index % 3) as f64);
            ([20.0; 3], [20.0 * (column - 1.0), 20.0 * (row - 1.0), 10.0])
        })
        .collect()
}

/// Clear glass `boxes` and a matte floor of half side `floor_half` at z = 0, the floor the first
/// part when `floor_first` and the last otherwise. Returns the scene and the floor's index.
fn on_floor(
    boxes: &[([f64; 3], [f64; 3])],
    floor_half: f64,
    floor_first: bool,
    lights: LightSetup,
) -> (RenderScene, usize) {
    let mut parts: Vec<PlacedPart> = box_meshes(boxes)
        .iter()
        .map(|mesh| part_of(mesh, Matrix4X4::IDENTITY))
        .collect();
    let mut surfaces = vec![ScatteringSurface::glass(); parts.len()];
    let floor = part_of(&flat_patch(floor_half, 0.0), Matrix4X4::IDENTITY);
    let matte = ScatteringSurface::matte_plastic([0.3; 3]);
    let floor_index = if floor_first {
        parts.insert(0, floor);
        surfaces.insert(0, matte);
        0
    } else {
        parts.push(floor);
        surfaces.push(matte);
        parts.len() - 1
    };
    let scene = RenderScene::new(PlacedParts::new(parts), surfaces, lights).unwrap();
    (scene, floor_index)
}

/// The 3,600 downward rays from inside the glass the tie was measured with, over a footprint of
/// half side `half`.
pub(crate) fn downward_rays(half: f64) -> Vec<Ray> {
    let n = 60;
    let mut rays = Vec::new();
    for i in 0..n {
        for j in 0..n {
            let start = Vector3::new(
                -half + 2.0 * half * (f64::from(i) + 0.5) / f64::from(n),
                -half + 2.0 * half * (f64::from(j) + 0.5) / f64::from(n),
                3.0 + 0.37 * f64::from((i * 7 + j * 3) % 11),
            );
            let direction = Vector3::new(
                0.3 * (f64::from(i) * 0.7).sin(),
                0.3 * (f64::from(j) * 1.3).cos(),
                -1.0,
            )
            .get_normal();
            rays.push(Ray::with_limits(
                start,
                direction,
                0.0,
                f64::INFINITY,
                IntersectionType::Both,
            ));
        }
    }
    rays
}

/// The glass `boxes` alone packed for the GPU route, over its analytic floor at z = 0 of half side
/// `floor_half`.
fn gpu_pack(boxes: &[([f64; 3], [f64; 3])], floor_half: f64) -> GpuRenderScene {
    gpu_pack_with(boxes, &[], &vec![1.0; boxes.len()], floor_half)
}

/// The glass `boxes` then the matte `matte` boxes packed for the GPU route at `opacities` (in that
/// order), over its analytic floor at z = 0 of half side `floor_half`.
pub(crate) fn gpu_pack_with(
    boxes: &[([f64; 3], [f64; 3])],
    matte: &[([f64; 3], [f64; 3])],
    opacities: &[f64],
    floor_half: f64,
) -> GpuRenderScene {
    let parts: Vec<PlacedPart> = box_meshes(boxes)
        .iter()
        .chain(box_meshes(matte).iter())
        .map(|mesh| part_of(mesh, Matrix4X4::IDENTITY))
        .collect();
    let mut surfaces = vec![ScatteringSurface::glass(); boxes.len()];
    surfaces.resize(parts.len(), ScatteringSurface::matte_plastic([0.6; 3]));
    let scene = RenderScene::new(PlacedParts::new(parts), surfaces, furnace())
        .unwrap()
        .with_part_opacities(opacities.to_vec())
        .unwrap();
    let camera = RenderCamera::orthographic(
        Vector3::new(0.0, -80.0, 60.0),
        Vector3::new(0.0, 0.0, 10.0),
        Vector3::UNIT_Z,
        40.0,
    )
    .unwrap();
    let view = GpuView {
        camera,
        image: (4, 4),
        trace: (4, 4),
        lights: furnace(),
        settings: PathTraceSettings::default(),
        compositing: GpuCompositing::Floor(GpuFloor {
            center: (0.0, 0.0),
            half_extent: floor_half,
            z: 0.0,
            show_through: 0.0,
            surface: ScatteringSurface::matte_plastic([0.3; 3]),
        }),
    };
    let limits = GpuSceneLimits {
        max_storage_binding: WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
        max_vertex_step: 1e-3,
    };
    GpuRenderScene::pack_view(&scene, &view, limits).unwrap()
}

/// The part the GPU route's path loop takes as a ray's first vertex when that vertex is glass
/// resting on the floor, `None` otherwise: the hit the `f32` reference traversal finds, tested
/// as the path loop does (`leaves_onto_floor_f32`), in the shader's order of operations.
pub(crate) fn gpu_resting_glass(pack: &GpuRenderScene, ray: &Ray) -> Option<u32> {
    let packed = Ray::with_limits(
        ray.origin - pack.scene_origin(),
        ray.direction_normal,
        0.0,
        f64::INFINITY,
        IntersectionType::Both,
    );
    let mut flat = FlatRay::from_ray(&packed);
    flat.face_mask = FACE_BOTH;
    let hit = pack.tree().trace_closest(&flat)?;
    let triangle = pack.tree().triangles()[hit.triangle as usize];
    let face = pack.faces()[hit.source_index as usize];
    let [u, v, w] = hit.barycentric;
    let position: [f32; 3] = std::array::from_fn(|axis| {
        triangle.v0[axis] * u + triangle.v1[axis] * v + triangle.v2[axis] * w
    });
    leaves_onto_floor_f32(
        flat.direction,
        position,
        &face,
        &pack.parts()[face.part as usize],
        pack.view().floor_plane,
    )
    .then_some(face.part)
}

#[test]
fn the_floor_rest_constants_are_the_shaders() {
    let module = naga::front::wgsl::parse_str(PATH_WGSL).expect("the path loop parses as WGSL");
    assert_eq!(
        literal(&module, "PATH_FLOOR_REST_ROUNDING_STEPS"),
        Literal::F32(FLOOR_REST_ROUNDING_STEPS)
    );
    assert_eq!(
        literal(&module, "PATH_FLOOR_REST_ANTI_PARALLEL"),
        Literal::F32(FLOOR_REST_ANTI_PARALLEL)
    );
    assert_eq!(
        literal(&module, "PATH_FLOOR_OFFSET_STEPS"),
        Literal::F32(FLOOR_OFFSET_STEPS)
    );
}

pub(crate) fn furnace() -> LightSetup {
    LightSetup::environment_only(Environment::constant([1.0; 3]).unwrap())
}

#[test]
fn every_downward_ray_inside_glass_on_the_floor_leaves_the_glass_then_meets_the_floor() {
    // (name, boxes, half side the rays start over, footprint's half side, a box's side).
    for (name, boxes, half, footprint, cell) in [
        ("box 20", one_box(0.0), 9.5, 10.0, 20.0),
        ("slab 60x60x20", slab(), 29.5, 30.0, 60.0),
        ("grid 3x3 of 20", grid(), 29.5, 30.0, 20.0),
    ] {
        let cell_of = |at: Vector3| {
            (
                ((at.x + footprint) / cell).floor(),
                ((at.y + footprint) / cell).floor(),
            )
        };
        let pack = gpu_pack(&boxes, 40_000.0);
        for floor_first in [false, true] {
            let (scene, floor) = on_floor(&boxes, 40_000.0, floor_first, furnace());
            let mut through_the_bottom = 0;
            for (at, ray) in downward_rays(half).into_iter().enumerate() {
                // Only a ray reaching z = 0 inside the footprint leaves through the bottom; the
                // rest leave through a side. In the grid, a ray reaching the bottom of another box
                // first crosses a seam between two boxes, which is the seam rule's to decide.
                let t = -ray.origin.z / ray.direction_normal.z;
                let reaches = ray.origin + ray.direction_normal * t;
                if reaches.x.abs() >= footprint
                    || reaches.y.abs() >= footprint
                    || cell_of(reaches) != cell_of(ray.origin)
                {
                    continue;
                }
                through_the_bottom += 1;
                let vertex = scene
                    .path_vertex_at(ray.clone())
                    .unwrap()
                    .unwrap_or_else(|| panic!("{name}: ray {at} met nothing"));
                assert!(
                    vertex.part != floor && vertex.back_face && vertex.across == Some(floor),
                    "{name} (floor first: {floor_first}): ray {at} took {vertex:?}, not the \
                     glass's exit with the floor across it"
                );
                // The packed parts are the glass alone, in the same order.
                let glass_index = if floor_first {
                    vertex.part - 1
                } else {
                    vertex.part
                };
                assert_eq!(
                    gpu_resting_glass(&pack, &ray),
                    Some(glass_index as u32),
                    "{name}: ray {at}: the GPU rule does not leave glass {glass_index} onto the floor"
                );
            }
            println!("{name} (floor first: {floor_first}): {through_the_bottom} rays");
            assert!(
                through_the_bottom > 2000,
                "{name}: only {through_the_bottom} rays checked"
            );
        }
    }
}

/// An environment white above the horizon and blue below it: on a clear glass and grey floor
/// scene every path's red and blue are equal unless it reached the ground below the floor.
pub(crate) fn white_sky_blue_ground() -> LightSetup {
    LightSetup::environment_only(
        Environment::gradient([1.0; 3], [1.0; 3], [0.0, 0.0, 1.0]).unwrap(),
    )
}

/// The red channel's statistics over a 24 by 24 view of `scene` at 16 samples a pixel, and how
/// many samples came back bluer than red (reached the ground below the floor).
pub(crate) fn red_and_ground_leaks(scene: &RenderScene, extent: f64) -> (Stats, usize) {
    let camera = RenderCamera::orthographic(
        Vector3::new(0.0, -80.0, 60.0),
        Vector3::new(0.0, 0.0, 10.0),
        Vector3::UNIT_Z,
        extent,
    )
    .unwrap();
    let tracer = PathTracer::new(scene, camera, 24, 24, PathTraceSettings::default()).unwrap();
    let mut work = RenderWork::default();
    let mut red = Stats::default();
    let mut leaks = 0;
    for y in 0..24 {
        for x in 0..24 {
            for sample in 0..16 {
                let [r, _, b] = tracer.sample_pixel(x, y, sample, &mut work).unwrap();
                red.add(r);
                if b > r + 1e-12 {
                    leaks += 1;
                }
            }
        }
    }
    (red, leaks)
}

#[test]
fn a_glass_box_resting_on_the_floor_renders_as_one_lifted_off_it() {
    // A floor small enough that its rounding step stays far below the 0.01 lift.
    let (resting, _) = on_floor(&one_box(0.0), 200.0, false, white_sky_blue_ground());
    let (lifted, _) = on_floor(&one_box(0.01), 200.0, false, white_sky_blue_ground());
    let (resting_red, resting_leaks) = red_and_ground_leaks(&resting, 40.0);
    let (lifted_red, lifted_leaks) = red_and_ground_leaks(&lifted, 40.0);
    let tolerance = agreement_tolerance(&resting_red, &lifted_red);
    println!(
        "resting {} lifted {} (tolerance {tolerance}); ground leaks resting {resting_leaks} \
         lifted {lifted_leaks}",
        resting_red.mean(),
        lifted_red.mean()
    );
    assert_eq!(
        resting_leaks, 0,
        "paths reached the ground under a resting box"
    );
    assert_eq!(
        lifted_leaks, 0,
        "paths reached the ground under a lifted box"
    );
    assert!(
        (resting_red.mean() - lifted_red.mean()).abs() <= tolerance,
        "resting {} against lifted {} (tolerance {tolerance})",
        resting_red.mean(),
        lifted_red.mean()
    );
}

#[test]
fn a_grid_of_touching_glass_boxes_on_the_floor_renders_as_the_slab_they_make() {
    let (slab_scene, _) = on_floor(&slab(), 200.0, false, white_sky_blue_ground());
    let (grid_scene, _) = on_floor(&grid(), 200.0, false, white_sky_blue_ground());
    let (slab_red, slab_leaks) = red_and_ground_leaks(&slab_scene, 90.0);
    let (grid_red, grid_leaks) = red_and_ground_leaks(&grid_scene, 90.0);
    let tolerance = agreement_tolerance(&slab_red, &grid_red);
    println!(
        "slab {} grid {} (tolerance {tolerance}); ground leaks slab {slab_leaks} grid {grid_leaks}",
        slab_red.mean(),
        grid_red.mean()
    );
    assert_eq!(slab_leaks, 0, "paths reached the ground under the slab");
    assert_eq!(grid_leaks, 0, "paths reached the ground under the grid");
    assert!(
        (slab_red.mean() - grid_red.mean()).abs() <= tolerance,
        "slab {} against grid {} (tolerance {tolerance})",
        slab_red.mean(),
        grid_red.mean()
    );
}
