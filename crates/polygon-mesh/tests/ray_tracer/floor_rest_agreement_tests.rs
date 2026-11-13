//! The GPU path loop's "resting on the floor" mark on a ray's first vertex (`resting` in
//! `path.wgsl`, read by the shadow ray and the bounce across the contact) agrees ray for ray with
//! the CPU's contact partner at the same hit (the `across` of `RenderScene::line_vertex_at` with
//! every part found there, the path loop's own decision at a first hit): the
//! scenes and line sets of `glass_on_floor_tests` (a clear box, a slab and a grid of touching
//! boxes on the floor, lines from inside the glass and the same lines from above it) and of
//! `see_through_on_floor_tests` (a half-there glass box and a half-there matte box on the wide
//! floor). The GPU side is the path loop's steps at a traced first hit, in its order, on the hit
//! the `f32` reference traversal finds, through the Rust mirrors of its tests
//! (`leaves_onto_floor_f32`, `see_through_floor_first_f32`, `there_glass_leaves_onto_floor_f32`);
//! a half-there part is taken as found there on both sides. `path_vertex_at` is not the CPU side
//! here: it takes the face the trace returns, so where the trace returns the floor before a
//! half-there glass box's bottom it reports the floor with nothing across, while the path loop
//! (and `line_vertex_at`) meets the glass's face first and leaves it onto the floor.

use solidmesh_math::{IntersectionType, Matrix4X4, Ray};
use solidmesh_mesh::ray_tracer::flat_tree::{
    leaves_onto_floor_f32, see_through_floor_first_f32, there_glass_leaves_onto_floor_f32, FlatRay,
    GpuRenderScene, FACE_BOTH,
};
use solidmesh_mesh::ray_tracer::{PlacedParts, RenderScene, ScatteringSurface};

use crate::glass_on_floor_tests::{
    downward_rays, furnace, gpu_pack_with, grid, on_floor, one_box, slab, white_sky_blue_ground,
    Boxes,
};
use crate::glass_tray_support::{box_meshes, part_of};
use crate::see_through_on_floor_tests::{box_on_floor, through_the_bottom, wide_floor};

/// The path loop's `floor_t` (`path_floor_distance`) for a packed ray of `t_min` 0 and no end:
/// where the ray crosses the floor's plane over its square, else -1. The lines here are steep,
/// far from the shader's smallest direction.
fn floor_distance(flat: &FlatRay, plane: [f32; 4]) -> f32 {
    let t = (plane[2] - flat.origin[2]) / flat.direction[2];
    if t.is_nan() || t < 0.0 {
        return -1.0;
    }
    let at = [
        flat.origin[0] + flat.direction[0] * t,
        flat.origin[1] + flat.direction[1] * t,
    ];
    if (at[0] - plane[0]).abs() > plane[3] || (at[1] - plane[1]).abs() > plane[3] {
        return -1.0;
    }
    t
}

/// The GPU loop's `resting` at the first traced hit of `ray` in `pack`: glass at opacity 1 left
/// through its face on the floor, else, the floor not met first, half-there glass found there and
/// left through that face. `false` where the ray meets no part.
fn gpu_resting(pack: &GpuRenderScene, ray: &Ray) -> bool {
    let packed = Ray::with_limits(
        ray.origin - pack.scene_origin(),
        ray.direction_normal,
        0.0,
        f64::INFINITY,
        IntersectionType::Both,
    );
    let mut flat = FlatRay::from_ray(&packed);
    flat.face_mask = FACE_BOTH;
    let Some(hit) = pack.tree().trace_closest(&flat) else {
        return false;
    };
    let triangle = pack.tree().triangles()[hit.triangle as usize];
    let face = pack.faces()[hit.source_index as usize];
    let [u, v, w] = hit.barycentric;
    let position: [f32; 3] = std::array::from_fn(|axis| {
        triangle.v0[axis] * u + triangle.v1[axis] * v + triangle.v2[axis] * w
    });
    let part = &pack.parts()[face.part as usize];
    let plane = pack.view().floor_plane;
    if leaves_onto_floor_f32(flat.direction, position, &face, part, plane) {
        return true;
    }
    let floor_t = floor_distance(&flat, plane);
    let nearer = floor_t >= 0.0 && floor_t < hit.t;
    let floor_first = see_through_floor_first_f32(
        flat.direction,
        position,
        &face,
        part,
        plane,
        (floor_t, nearer),
    );
    !floor_first && there_glass_leaves_onto_floor_f32(flat.direction, position, &face, part, plane)
}

/// `ray` started back along itself at height `z`, so it meets the scene from above.
fn from_height(ray: &Ray, z: f64) -> Ray {
    let back = (z - ray.origin.z) / ray.direction_normal.z;
    Ray::with_limits(
        ray.origin + ray.direction_normal * back,
        ray.direction_normal,
        0.0,
        f64::INFINITY,
        IntersectionType::Both,
    )
}

/// A clear glass box of side 20 on the wide floor at `opacity`, the floor first when
/// `floor_first`.
fn glass_box_on_wide_floor(opacity: f64, floor_first: bool) -> RenderScene {
    let boxed = part_of(&box_meshes(&one_box(0.0))[0], Matrix4X4::IDENTITY);
    let floor = part_of(&wide_floor(), Matrix4X4::IDENTITY);
    let glass = ScatteringSurface::glass();
    let grey = ScatteringSurface::matte_plastic([0.3; 3]);
    let (parts, surfaces, opacities) = if floor_first {
        (vec![floor, boxed], vec![grey, glass], vec![1.0, opacity])
    } else {
        (vec![boxed, floor], vec![glass, grey], vec![opacity, 1.0])
    };
    RenderScene::new(PlacedParts::new(parts), surfaces, white_sky_blue_ground())
        .unwrap()
        .with_part_opacities(opacities)
        .unwrap()
}

/// How the CPU and the GPU mark one case's lines: how many agree resting, agree not resting, and
/// the lines where they differ, by index.
struct Tally {
    resting: usize,
    not_resting: usize,
    differing: Vec<(usize, bool, bool)>,
}

fn tally(scene: &RenderScene, pack: &GpuRenderScene, rays: &[Ray]) -> Tally {
    let mut count = Tally {
        resting: 0,
        not_resting: 0,
        differing: Vec::new(),
    };
    for (at, ray) in rays.iter().enumerate() {
        let cpu = scene
            .line_vertex_at(ray.clone(), &[])
            .unwrap()
            .is_some_and(|line| line.vertex.across.is_some());
        let gpu = gpu_resting(pack, ray);
        match (cpu, gpu) {
            (true, true) => count.resting += 1,
            (false, false) => count.not_resting += 1,
            _ => count.differing.push((at, cpu, gpu)),
        }
    }
    count
}

#[test]
fn the_gpu_resting_mark_is_the_cpus_contact_partner_ray_for_ray() {
    let mut cases: Vec<(String, RenderScene, GpuRenderScene, Vec<Ray>)> = Vec::new();
    let glass: [(&str, Boxes, f64); 3] = [
        ("box 20", one_box(0.0), 9.5),
        ("slab 60x60x20", slab(), 29.5),
        ("grid 3x3 of 20", grid(), 29.5),
    ];
    for (name, boxes, half) in glass {
        let inside = downward_rays(half);
        let above: Vec<Ray> = inside.iter().map(|ray| from_height(ray, 40.0)).collect();
        let lines: Vec<Ray> = inside.into_iter().chain(above).collect();
        for floor_first in [false, true] {
            let (scene, _) = on_floor(&boxes, 40_000.0, floor_first, furnace());
            let pack = gpu_pack_with(&boxes, &[], &vec![1.0; boxes.len()], 40_000.0);
            let case = format!("glass {name} (floor first: {floor_first})");
            cases.push((case, scene, pack, lines.clone()));
        }
    }
    let bottom: Vec<Ray> = through_the_bottom()
        .into_iter()
        .flat_map(|(inside, above)| [inside, above])
        .chain(downward_rays(9.5))
        .collect();
    for floor_first in [false, true] {
        cases.push((
            format!("half-there glass box (floor first: {floor_first})"),
            glass_box_on_wide_floor(0.5, floor_first),
            gpu_pack_with(&one_box(0.0), &[], &[0.5], 50.0),
            bottom.clone(),
        ));
        let (matte, _, _) = box_on_floor(0.0, 0.5, floor_first, white_sky_blue_ground());
        cases.push((
            format!("half-there matte box (floor first: {floor_first})"),
            matte,
            gpu_pack_with(&[], &one_box(0.0), &[0.5], 50.0),
            bottom.clone(),
        ));
    }

    let mut failures = Vec::new();
    for (case, scene, pack, rays) in &cases {
        let count = tally(scene, pack, rays);
        println!(
            "{case}: {} lines, resting on both {}, on neither {}, differing {}",
            rays.len(),
            count.resting,
            count.not_resting,
            count.differing.len()
        );
        // Most lines from inside glass leave through its bottom, and a matte part never rests,
        // so neither kind of case compares two empty sets.
        let rests = !case.contains("matte");
        if rests != (count.resting > 3000) || (!rests && count.resting > 0) {
            failures.push(format!("{case}: {} lines rest on both", count.resting));
        }
        for &(at, cpu, gpu) in &count.differing {
            let ray = &rays[at];
            let t = -ray.origin.z / ray.direction_normal.z;
            let lands = ray.origin + ray.direction_normal * t;
            failures.push(format!(
                "{case}: line {at} from {:?} reaching z = 0 at ({}, {}): CPU resting {cpu}, GPU \
                 {gpu}",
                ray.origin, lands.x, lands.y
            ));
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}
