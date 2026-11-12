//! Measures, for rays inside clear glass resting on the floor and heading down, which of the
//! glass's bottom face and the floor each route meets first: the CPU, whose floor is a large quad
//! part added after the parts, and the GPU route's rule (the packed tree traced by the `f32`
//! reference traversal `trace.wgsl` mirrors, against the analytic floor plane `catcher.wgsl`
//! intersects, the floor first only when strictly nearer). One 20 box, a 60 by 60 by 20 slab, and
//! nine 20 boxes in a 3 by 3 grid on the slab's footprint. Ignored by default, since it reports
//! rather than checks. Measured: the CPU meets the floor first for 48 to 53% of these rays, the
//! GPU rule for 22 to 25%, because nearly half the rays find the two at the same `f32` distance
//! and a tie goes to the glass there. This is the raw trace only: the path loop on both routes no
//! longer takes its vertex from it at glass resting on the floor (`glass_rest`, and
//! `path_rests_on_floor` on the GPU), but decides the glass face first whichever the trace meets,
//! so these shares are what that rule replaces, not what a render does.

use solidmesh_math::{IntersectionType, Matrix4X4, Ray, Vector3};
use solidmesh_mesh::ray_tracer::flat_tree::{
    FlatRay, GpuRenderScene, GpuSceneLimits, FACE_BOTH, WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
};
use solidmesh_mesh::ray_tracer::{
    PathTraceSettings, PlacedPart, PlacedParts, RenderCamera, RenderScene, ScatteringSurface,
};

use crate::glass_tray_support::{box_meshes, part_of};
use crate::path_tracer_glass_contact_tests::furnace;
use crate::path_tracer_support::flat_patch;

/// (floor first, glass first, of those at the same distance) on the CPU and by the GPU rule.
fn measure(boxes: &[([f64; 3], [f64; 3])], half: f64) -> ([usize; 2], [usize; 3]) {
    let meshes = box_meshes(boxes);
    let placed = || -> Vec<PlacedPart> {
        meshes
            .iter()
            .map(|mesh| part_of(mesh, Matrix4X4::IDENTITY))
            .collect()
    };
    let glass = placed();
    let count = glass.len();
    let mut with_floor = placed();
    with_floor.push(part_of(&flat_patch(40_000.0, 0.0), Matrix4X4::IDENTITY));
    let mut surfaces = vec![ScatteringSurface::glass(); count];
    let cpu = RenderScene::new(
        PlacedParts::new(with_floor),
        {
            let mut all = surfaces.clone();
            all.push(ScatteringSurface::matte_plastic([0.3; 3]));
            all
        },
        furnace(),
    )
    .unwrap();
    surfaces.truncate(count);
    let gpu_scene = RenderScene::new(PlacedParts::new(glass), surfaces, furnace()).unwrap();
    let camera = RenderCamera::orthographic(
        Vector3::new(0.0, -80.0, 60.0),
        Vector3::new(0.0, 0.0, 10.0),
        Vector3::UNIT_Z,
        40.0,
    )
    .unwrap();
    let limits = GpuSceneLimits {
        max_storage_binding: WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
        max_vertex_step: 1e-3,
    };
    let pack = GpuRenderScene::pack(
        &gpu_scene,
        &camera,
        4,
        4,
        &PathTraceSettings::default(),
        limits,
    )
    .unwrap();
    let origin = pack.scene_origin();
    let plane_z = (0.0 - origin.z) as f32;
    let mut cpu_counts = [0usize; 2];
    let mut gpu_counts = [0usize; 3];
    let n = 60;
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
            let mut ray =
                Ray::with_limits(start, direction, 0.0, f64::INFINITY, IntersectionType::Both);
            match cpu.parts().trace(&mut ray).map(|hit| hit.part.index()) {
                Some(index) if index == count => cpu_counts[0] += 1,
                Some(_) => cpu_counts[1] += 1,
                None => {}
            }
            let packed = Ray::with_limits(
                start - origin,
                direction,
                0.0,
                f64::INFINITY,
                IntersectionType::Both,
            );
            let mut flat = FlatRay::from_ray(&packed);
            flat.face_mask = FACE_BOTH;
            let Some(hit) = pack.tree().trace_closest(&flat) else {
                continue;
            };
            let floor_t = (plane_z - flat.origin[2]) / flat.direction[2];
            if floor_t < hit.t {
                gpu_counts[0] += 1;
            } else {
                gpu_counts[1] += 1;
                if floor_t == hit.t {
                    gpu_counts[2] += 1;
                }
            }
        }
    }
    (cpu_counts, gpu_counts)
}

#[test]
#[ignore = "reports which of a glass bottom and the floor each route meets first"]
fn measure_which_a_ray_inside_glass_on_the_floor_meets_first() {
    let grid: Vec<_> = (0..9)
        .map(|index| {
            let (column, row) = ((index / 3) as f64, (index % 3) as f64);
            ([20.0; 3], [20.0 * (column - 1.0), 20.0 * (row - 1.0), 10.0])
        })
        .collect();
    for (name, boxes, half) in [
        ("box 20", vec![([20.0; 3], [0.0, 0.0, 10.0])], 9.5),
        (
            "slab 60x60x20",
            vec![([60.0, 60.0, 20.0], [0.0, 0.0, 10.0])],
            29.5,
        ),
        ("grid 3x3 of 20", grid, 29.5),
    ] {
        let (cpu, gpu) = measure(&boxes, half);
        println!(
            "{name}: CPU floor first {} glass first {}; GPU rule floor first {} glass first {} \
             (equal distances {})",
            cpu[0], cpu[1], gpu[0], gpu[1], gpu[2]
        );
    }
}
