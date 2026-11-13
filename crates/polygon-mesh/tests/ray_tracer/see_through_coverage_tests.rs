//! The coverage of a camera line ended at the floor, on the GPU route (its Rust mirror
//! `coverage_along_f32`, on the hits the `f32` reference traversal finds) and on the CPU
//! (`RenderScene::coverage_along`), line for line: a see-through part, matte or glass at opacity
//! 0.5, resting on the floor, on an opaque box, on a clear glass box and on another see-through
//! part, and the same part lifted a hundredth off what it rests on, each in both part orders. A
//! line passing the part's base where it lies on another part's top goes on into that part; the
//! shader's walk is pinned to the mirror's steps.

use solidmesh_math::{IntersectionType, Matrix4X4, Ray, Vector3};
use solidmesh_mesh::ray_tracer::flat_tree::{
    catcher_distance_f32, coverage_along_f32, FlatRay, GpuCompositing, GpuFloor, GpuRenderScene,
    GpuSceneLimits, GpuView, PATH_WGSL, WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
};
use solidmesh_mesh::ray_tracer::{
    PathTraceSettings, PlacedPart, PlacedParts, RenderCamera, RenderScene, ScatteringSurface,
};
use solidmesh_mesh::Mesh;

use crate::glass_on_floor_tests::{furnace, white_sky_blue_ground};
use crate::glass_tray_support::{box_meshes, part_of};
use crate::scattering_support::Stats;
use crate::see_through_glass_on_floor_tests::paired_against_lifted;
use crate::see_through_on_floor_tests::{function_text, wide_floor};

/// What the see-through part rests on.
#[derive(Debug, Clone, Copy)]
pub(crate) enum Support {
    Floor,
    OpaqueBox,
    ClearGlass,
    SeeThrough,
}

/// Every support a see-through part is tested resting on.
pub(crate) const SUPPORTS: [Support; 4] = [
    Support::Floor,
    Support::OpaqueBox,
    Support::ClearGlass,
    Support::SeeThrough,
];

/// The see-through part (side 40 by 40 by 20, at opacity 0.5, glass when `glass`) lifted `lift`
/// over `support` (a box of the same footprint from z = 0 to 20, or the floor itself), the support
/// first when `support_first`; the scene, and its packing over a floor at z = 0.
pub(crate) fn stack(
    support: Support,
    glass: bool,
    lift: f64,
    support_first: bool,
) -> (RenderScene, GpuRenderScene) {
    let size = [40.0, 40.0, 20.0];
    let base = if matches!(support, Support::Floor) {
        0.0
    } else {
        20.0
    };
    let part = (
        box_meshes(&[(size, [0.0, 0.0, base + 10.0 + lift])])[0].clone(),
        if glass {
            ScatteringSurface::glass()
        } else {
            ScatteringSurface::matte_plastic([0.6; 3])
        },
        0.5,
    );
    let under = box_meshes(&[(size, [0.0, 0.0, 10.0])])[0].clone();
    let mut parts = vec![part];
    match support {
        Support::Floor => {}
        Support::OpaqueBox => parts.push((under, ScatteringSurface::matte_plastic([0.3; 3]), 1.0)),
        Support::ClearGlass => parts.push((under, ScatteringSurface::glass(), 1.0)),
        Support::SeeThrough => {
            parts.push((under, ScatteringSurface::matte_plastic([0.3; 3]), 0.5));
        }
    }
    if support_first {
        parts.reverse();
    }
    scene_and_pack(&parts)
}

/// The scene of `parts` (each a mesh, its surface and its opacity, in place) and its packing over
/// a floor at z = 0.
pub(crate) fn scene_and_pack(
    parts: &[(Mesh, ScatteringSurface, f64)],
) -> (RenderScene, GpuRenderScene) {
    let placed: Vec<PlacedPart> = parts
        .iter()
        .map(|(mesh, _, _)| part_of(mesh, Matrix4X4::IDENTITY))
        .collect();
    let scene = RenderScene::new(
        PlacedParts::new(placed),
        parts.iter().map(|(_, surface, _)| *surface).collect(),
        furnace(),
    )
    .unwrap()
    .with_part_opacities(parts.iter().map(|&(_, _, opacity)| opacity).collect())
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
            half_extent: 200.0,
            z: 0.0,
            show_through: 0.0,
            surface: ScatteringSurface::matte_plastic([0.3; 3]),
        }),
    };
    let limits = GpuSceneLimits {
        max_storage_binding: WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
        max_vertex_step: 1e-3,
    };
    let pack = GpuRenderScene::pack_view(&scene, &view, limits).unwrap();
    (scene, pack)
}

/// Camera lines from above the stack, down at up to about 17 degrees off vertical, over the
/// part's footprint and a little past it.
pub(crate) fn camera_lines() -> Vec<Ray> {
    let n = 48;
    let mut lines = Vec::new();
    for i in 0..n {
        for j in 0..n {
            let start = Vector3::new(
                -24.0 + 48.0 * (f64::from(i) + 0.5) / f64::from(n),
                -24.0 + 48.0 * (f64::from(j) + 0.5) / f64::from(n),
                60.0 + 0.37 * f64::from((i * 7 + j * 3) % 11),
            );
            let direction = Vector3::new(
                0.3 * (f64::from(i) * 0.7).sin(),
                0.3 * (f64::from(j) * 1.3).cos(),
                -1.0,
            )
            .get_normal();
            lines.push(Ray::with_limits(
                start,
                direction,
                0.0,
                f64::INFINITY,
                IntersectionType::Both,
            ));
        }
    }
    lines
}

/// The coverage of `line` ended at the floor's plane, on the CPU and on the GPU route's mirror.
pub(crate) fn both_coverages(scene: &RenderScene, pack: &GpuRenderScene, line: &Ray) -> (f64, f64) {
    let direction = line.direction_normal;
    let t = (0.0 - line.origin.z) / direction.z;
    let ended = Ray::with_limits(line.origin, direction, 0.0, t, IntersectionType::Both);
    let cpu = scene.coverage_along(ended).unwrap();
    let packed = Ray::with_limits(
        line.origin - pack.scene_origin(),
        direction,
        0.0,
        f64::INFINITY,
        IntersectionType::Both,
    );
    let mut flat = FlatRay::from_ray(&packed);
    let floor = pack.view().floor_plane;
    flat.t_max = catcher_distance_f32(flat.origin, flat.direction, floor, false)
        .expect("every camera line here reaches the floor");
    (cpu, f64::from(coverage_along_f32(pack, &flat)))
}

#[test]
fn a_see_through_part_resting_on_anything_is_covered_alike_on_the_card_and_the_cpu_and_lifted() {
    // Every case is measured before any is judged, so a failure shows them all.
    let mut failures = Vec::new();
    let lines = camera_lines();
    for support in SUPPORTS {
        for glass in [false, true] {
            for support_first in [false, true] {
                let (resting, resting_pack) = stack(support, glass, 0.0, support_first);
                let (lifted, lifted_pack) = stack(support, glass, 0.01, support_first);
                // Lines where the card and the CPU differ, resting and lifted, and where resting
                // and lifted differ, on the CPU and on the card.
                let mut differing = [0; 4];
                let mut example = None;
                let mut sums = [0.0; 4];
                for line in &lines {
                    let (cpu, card) = both_coverages(&resting, &resting_pack, line);
                    let (cpu_lifted, card_lifted) = both_coverages(&lifted, &lifted_pack, line);
                    let values = [cpu, card, cpu_lifted, card_lifted];
                    for (sum, value) in sums.iter_mut().zip(values) {
                        *sum += value;
                    }
                    let pairs = [
                        (cpu, card),
                        (cpu_lifted, card_lifted),
                        (cpu, cpu_lifted),
                        (card, card_lifted),
                    ];
                    for (count, (a, b)) in differing.iter_mut().zip(pairs) {
                        if a != b {
                            *count += 1;
                            example.get_or_insert((line.origin, values));
                        }
                    }
                }
                let case = format!("{support:?}, glass {glass}, support first {support_first}");
                let means = sums.map(|sum| sum / lines.len() as f64);
                println!(
 "{case}: mean coverage [cpu, card, cpu lifted, card lifted] {means:?}; lines differing [card-cpu, lifted card-cpu, cpu resting-lifted, card resting-lifted] {differing:?} of {}",
 lines.len()
 );
                if differing != [0; 4] {
                    failures.push(format!("{case}: {differing:?} lines differ, {example:?}"));
                }
            }
        }
    }
    assert!(failures.is_empty(), "{failures:#?}");
}

/// `path_coverage_along`, comments dropped and whitespace collapsed, step for step as
/// `coverage_along_f32` takes them.
const COVERAGE_WALK: &str = concat!(
    "fn path_coverage_along(origin_ray: Ray) -> f32 { line_restart(); var ray = origin_ray; ",
    "let direction = origin_ray.direction; let bounded = origin_ray.t_max < PATH_FAR; ",
    "let end = origin_ray.origin + direction * origin_ray.t_max; ",
    "let see_through = (path_view.flags & PATH_FLAG_SEE_THROUGH) != 0u; ",
    "var left = origin_ray.t_max; var uncovered = 1.0f; var past = false; var odd = 0u; ",
    // Each stretch to its next face; the first face past the end starts the walk past it.
    "loop { let hit = trace_closest(ray); if hit.triangle == NO_HIT { break; } ",
    "let point = path_surface_point(hit); past = past || (see_through && hit.t > left + point.ray_offset); ",
    // Past the end: every crossing counts toward the cap and flips a part not passed before.
    "if past { if line_crossings >= MAX_CROSSINGS_PER_LINE { return 1.0f; } ",
    "line_crossings += 1u; if !line_has_passed(point.part) { ",
    "odd = line_flip_past(point.part, odd); } } ",
    // Before the end: a fully there part covers, a see-through one counts once.
    "else { let opacity = path_part_opacity(point.part); if opacity >= 1.0f { return 1.0f; } ",
    "if line_crossings >= MAX_CROSSINGS_PER_LINE { return 1.0f; } line_crossings += 1u; ",
    "if !line_has_passed(point.part) { line_passed[line_passed_count] = point.part; ",
    "line_passed_count += 1u; uncovered *= 1.0f - opacity; } } ",
    // The next stretch, and how far it has left to the end until the walk is past it.
    "ray = path_ray_from(path_next_origin(point, direction), direction); ",
    "if bounded && !past { left = dot(end - ray.origin, direction); if left <= 0.0f { ",
    "if !see_through { return 1.0f - uncovered; } past = true; } } } ",
    // The parts the end lies inside.
    "for (var index = 0u; index < odd; index += 1u) { ",
    "let opacity = path_part_opacity(line_passed[line_passed_count + index]); ",
    "if opacity >= 1.0f { return 1.0f; } uncovered *= 1.0f - opacity; } ",
    "return 1.0f - uncovered;",
);

/// `line_flip_past`, as `coverage_line.rs`'s `flip_past` takes its steps.
const FLIP_PAST: &str = concat!(
    "fn line_flip_past(part: u32, odd: u32) -> u32 { ",
    "for (var index = 0u; index < odd; index += 1u) { ",
    "if line_passed[line_passed_count + index] == part { ",
    "line_passed[line_passed_count + index] = line_passed[line_passed_count + odd - 1u]; ",
    "return odd - 1u; } } line_passed[line_passed_count + odd] = part; return odd + 1u;",
);

#[test]
fn the_shader_coverage_line_walks_as_its_mirror_does() {
    let walk = function_text(PATH_WGSL, "path_coverage_along");
    assert_eq!(
        walk, COVERAGE_WALK,
        "the coverage line is not its mirror's walk"
    );
    let flip = function_text(PATH_WGSL, "line_flip_past");
    assert_eq!(flip, FLIP_PAST, "the past-the-end list is not its mirror's");
}

/// The see-through part (side 40 by 40 by 20, at opacity 0.5, glass when `glass`) lifted `lift`
/// over `support` standing on the wide grey floor (`wide_floor`, an opaque part), under a white
/// sky over a blue ground, for the CPU path loop.
fn picture_stack(support: Support, glass: bool, lift: f64) -> RenderScene {
    let size = [40.0, 40.0, 20.0];
    let base = if matches!(support, Support::Floor) {
        0.0
    } else {
        20.0
    };
    let grey = ScatteringSurface::matte_plastic([0.3; 3]);
    let mut parts = vec![(wide_floor(), grey, 1.0)];
    let under = box_meshes(&[(size, [0.0, 0.0, 10.0])])[0].clone();
    match support {
        Support::Floor => {}
        Support::OpaqueBox => parts.push((under, grey, 1.0)),
        Support::ClearGlass => parts.push((under, ScatteringSurface::glass(), 1.0)),
        Support::SeeThrough => parts.push((under, grey, 0.5)),
    }
    let surface = if glass {
        ScatteringSurface::glass()
    } else {
        ScatteringSurface::matte_plastic([0.6; 3])
    };
    let mesh = box_meshes(&[(size, [0.0, 0.0, base + 10.0 + lift])])[0].clone();
    parts.push((mesh, surface, 0.5));
    let placed: Vec<PlacedPart> = parts
        .iter()
        .map(|(mesh, _, _)| part_of(mesh, Matrix4X4::IDENTITY))
        .collect();
    RenderScene::new(
        PlacedParts::new(placed),
        parts.iter().map(|(_, surface, _)| *surface).collect(),
        white_sky_blue_ground(),
    )
    .unwrap()
    .with_part_opacities(parts.iter().map(|&(_, _, opacity)| opacity).collect())
    .unwrap()
}

#[test]
fn a_see_through_part_resting_on_what_its_rest_rules_cover_renders_as_one_lifted_off_it() {
    // Every case is measured before any is judged, so a failure shows them all. See-through glass
    // on each support; a see-through matte part on the floor and on an opaque box, the supports
    // the CPU's `see_through_rest` covers.
    let mut failures = Vec::new();
    let cases = SUPPORTS
        .iter()
        .map(|&support| (support, true))
        .chain([(Support::Floor, false), (Support::OpaqueBox, false)]);
    for (support, glass) in cases {
        let paired = paired_against_lifted(
            &picture_stack(support, glass, 0.0),
            &picture_stack(support, glass, 0.01),
        );
        let case = format!("{support:?}, glass {glass}");
        let means = paired.difference.each_ref().map(Stats::mean);
        println!(
            "{case}: ground leaks resting {} lifted {}; resting minus lifted {means:?}",
            paired.leaks.0, paired.leaks.1
        );
        if paired.leaks != (0, 0) {
            failures.push(format!("{case}: leaks {:?}", paired.leaks));
        }
        for (channel, difference, bound) in paired.disagreements() {
            failures.push(format!(
                "{case}, channel {channel}: resting minus lifted {difference} (bound {bound})"
            ));
        }
    }
    assert!(failures.is_empty(), "{failures:#?}");
}
