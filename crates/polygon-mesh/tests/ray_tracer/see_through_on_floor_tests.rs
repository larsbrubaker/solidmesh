//! A see-through part resting on a floor: a line that finds the part not there and leaves it
//! through the face lying on the floor lands on the floor at that point, on the CPU route for any
//! opaque part under it and on the GPU route's analytic floor (its Rust mirror, on the hit the
//! `f32` reference traversal finds), whichever of the two faces a trace returns and whichever
//! order the parts are in, with the part's presence decided once per crossing either way. So a
//! half-there box on the floor renders as one lifted a hundredth off it (by the paired difference
//! of `see_through_glass_on_floor_tests`), no path reaches the ground under the floor, and the
//! coverage along a camera line ends where its limit says.

use solidmesh_math::{IntersectionType, Matrix4X4, Ray, Vector3};
use solidmesh_mesh::ray_tracer::{
    Environment, LightSetup, PathTraceSettings, PathTracer, PlacedPart, PlacedParts, RenderCamera,
    RenderScene, RenderWork, ScatteringSurface,
};
use solidmesh_mesh::Mesh;

use solidmesh_mesh::ray_tracer::flat_tree::{
    see_through_floor_first_f32, see_through_on_floor_f32, FlatRay, GpuRenderScene, FACE_BOTH,
    PATH_WGSL,
};

use crate::glass_on_floor_tests::{downward_rays, gpu_pack_with, one_box, white_sky_blue_ground};
use crate::glass_tray_support::{box_meshes, part_of};
use crate::path_tracer_support::{agreement_tolerance, flat_patch};
use crate::scattering_support::Stats;
use crate::see_through_glass_on_floor_tests::paired_against_lifted;

/// How far the floor reaches: far enough that a path leaving a box's side just below the horizon
/// still lands on it, so a sample reaches the blue ground only by going under the floor.
const FLOOR_REACH: f64 = 1.0e6;

/// A floor at z = 0 out to `FLOOR_REACH`, with small triangles under the parts: a square of half
/// side 50 in a ring of eight triangles, so the contact tolerance follows the square's corners and
/// a modelled lift of 0.01 stays a real gap.
pub(crate) fn wide_floor() -> Mesh {
    let (inner, outer) = (50.0, FLOOR_REACH);
    let vertices = [
        -inner, -inner, 0.0, inner, -inner, 0.0, inner, inner, 0.0, -inner, inner, 0.0, //
        -outer, -outer, 0.0, outer, -outer, 0.0, outer, outer, 0.0, -outer, outer, 0.0,
    ];
    let faces = [
        0, 1, 2, 0, 2, 3, // the square
        4, 5, 1, 4, 1, 0, // the ring, side by side, all facing up
        5, 6, 2, 5, 2, 1, //
        6, 7, 3, 6, 3, 2, //
        7, 4, 0, 7, 0, 3,
    ];
    Mesh::from_arrays(&vertices, &faces)
}

/// A matte red box of side 20 lifted `lift` over the wide grey floor, the floor first when
/// `floor_first`, the box at `opacity`. Returns the scene and the box's and floor's indices.
pub(crate) fn box_on_floor(
    lift: f64,
    opacity: f64,
    floor_first: bool,
    lights: LightSetup,
) -> (RenderScene, usize, usize) {
    let boxed = part_of(&box_meshes(&one_box(lift))[0], Matrix4X4::IDENTITY);
    let floor = part_of(&wide_floor(), Matrix4X4::IDENTITY);
    let red = ScatteringSurface::matte_plastic([0.8, 0.3, 0.3]);
    let grey = ScatteringSurface::matte_plastic([0.3; 3]);
    let (parts, surfaces, opacities, at_box, at_floor): (Vec<PlacedPart>, _, _, _, _) =
        if floor_first {
            (
                vec![floor, boxed],
                vec![grey, red],
                vec![1.0, opacity],
                1,
                0,
            )
        } else {
            (
                vec![boxed, floor],
                vec![red, grey],
                vec![opacity, 1.0],
                0,
                1,
            )
        };
    let scene = RenderScene::new(PlacedParts::new(parts), surfaces, lights)
        .unwrap()
        .with_part_opacities(opacities)
        .unwrap();
    (scene, at_box, at_floor)
}

/// Each channel's statistics over a 24 by 24 view of `scene` from steeply above, so most lines
/// through the box leave it through its bottom, at 64 samples a pixel, and how many
/// samples came back bluer than red: under a white sky over a blue ground, with only red and grey
/// surfaces, only a path that reached the ground below the floor does.
pub(crate) fn channels_and_ground_leaks(scene: &RenderScene) -> ([Stats; 3], usize) {
    let camera = RenderCamera::orthographic(
        Vector3::new(0.0, -8.0, 80.0),
        Vector3::new(0.0, 0.0, 10.0),
        Vector3::UNIT_Y,
        30.0,
    )
    .unwrap();
    let tracer = PathTracer::new(scene, camera, 24, 24, PathTraceSettings::default()).unwrap();
    let mut work = RenderWork::default();
    let mut channels = [Stats::default(), Stats::default(), Stats::default()];
    let mut leaks = 0;
    for y in 0..24 {
        for x in 0..24 {
            for sample in 0..64 {
                let rgb = tracer.sample_pixel(x, y, sample, &mut work).unwrap();
                for (stats, value) in channels.iter_mut().zip(rgb) {
                    stats.add(value);
                }
                if rgb[2] > rgb[0] + 1e-12 {
                    leaks += 1;
                }
            }
        }
    }
    (channels, leaks)
}

#[test]
fn a_see_through_box_resting_on_the_floor_renders_as_one_lifted_off_it() {
    // Every case is measured before any is judged, so a failure shows them all.
    let mut failures = Vec::new();
    for opacity in [0.25, 0.5, 0.75] {
        for floor_first in [false, true] {
            let (resting, _, _) = box_on_floor(0.0, opacity, floor_first, white_sky_blue_ground());
            let (lifted, _, _) = box_on_floor(0.01, opacity, floor_first, white_sky_blue_ground());
            let paired = paired_against_lifted(&resting, &lifted);
            let case = format!("opacity {opacity} (floor first: {floor_first})");
            println!(
                "{case}: resting minus lifted {:?}; ground leaks resting {} lifted {}",
                paired.difference.each_ref().map(Stats::mean),
                paired.leaks.0,
                paired.leaks.1
            );
            if paired.leaks != (0, 0) {
                failures.push(format!("{case}: paths reached the ground under the floor"));
            }
            for (channel, difference, bound) in paired.disagreements() {
                failures.push(format!(
                    "{case}, channel {channel}: resting minus lifted {difference} (bound {bound})"
                ));
            }
        }
    }
    assert!(failures.is_empty(), "{failures:#?}");
}

/// A half-there box over nothing but an opaque box below the floor's plane, the floor being the
/// caller's shadow catcher and not a part: the coverage along a camera line that ends at the
/// floor's plane.
#[test]
fn coverage_along_a_line_to_the_floor_counts_a_resting_part_once_and_nothing_below() {
    for lift in [0.0, 0.01] {
        let boxes = [
            ([20.0; 3], [0.0, 0.0, 10.0 + lift]),
            ([40.0, 40.0, 10.0], [0.0, 0.0, -20.0]),
        ];
        let parts = box_meshes(&boxes)
            .iter()
            .map(|mesh| part_of(mesh, Matrix4X4::IDENTITY))
            .collect();
        let scene = RenderScene::new(
            PlacedParts::new(parts),
            vec![ScatteringSurface::matte_plastic([0.5; 3]); 2],
            LightSetup::environment_only(Environment::constant([1.0; 3]).unwrap()),
        )
        .unwrap()
        .with_part_opacities(vec![0.5, 1.0])
        .unwrap();
        let mut checked = 0;
        for ray in downward_rays(9.0) {
            // From above the box, down to the floor's plane inside the box's footprint.
            let origin = ray.origin + Vector3::new(0.0, 0.0, 40.0);
            let t = -origin.z / ray.direction_normal.z;
            let end = origin + ray.direction_normal * t;
            let entry_t = (20.0 + lift - origin.z) / ray.direction_normal.z;
            let entry = origin + ray.direction_normal * entry_t;
            let inside = |at: Vector3| at.x.abs() < 9.9 && at.y.abs() < 9.9;
            if !inside(end) || !inside(entry) {
                continue;
            }
            checked += 1;
            let line =
                Ray::with_limits(origin, ray.direction_normal, 0.0, t, IntersectionType::Both);
            let coverage = scene.coverage_along(line).unwrap();
            assert_eq!(
                coverage, 0.5,
                "lift {lift}: the line to the floor's plane is covered {coverage}"
            );
        }
        assert!(checked > 1000, "lift {lift}: only {checked} lines");
    }
}

/// The rays from inside the box that reach z = 0 inside its footprint, and the same lines started
/// above the box, so they cross its top first.
pub(crate) fn through_the_bottom() -> Vec<(Ray, Ray)> {
    downward_rays(9.5)
        .into_iter()
        .filter(|ray| {
            let t = -ray.origin.z / ray.direction_normal.z;
            let reaches = ray.origin + ray.direction_normal * t;
            reaches.x.abs() < 10.0 && reaches.y.abs() < 10.0
        })
        .map(|ray| {
            let back = (40.0 - ray.origin.z) / ray.direction_normal.z;
            let mut above = ray.clone();
            above.origin = ray.origin + ray.direction_normal * back;
            (ray, above)
        })
        .collect()
}

#[test]
fn every_line_leaving_a_see_through_box_through_its_bottom_lands_on_the_floor() {
    for (name, floor) in [
        ("wide floor", wide_floor()),
        ("floor 100 across", flat_patch(50.0, 0.0)),
        ("floor 64 km across", flat_patch(32_000.0, 0.0)),
    ] {
        for floor_first in [false, true] {
            let boxed = part_of(&box_meshes(&one_box(0.0))[0], Matrix4X4::IDENTITY);
            let floor = part_of(&floor, Matrix4X4::IDENTITY);
            let (parts, at_box, at_floor) = if floor_first {
                (vec![floor, boxed], 1, 0)
            } else {
                (vec![boxed, floor], 0, 1)
            };
            let mut opacities = vec![1.0; 2];
            opacities[at_box] = 0.5;
            let scene = RenderScene::new(
                PlacedParts::new(parts),
                vec![ScatteringSurface::matte_plastic([0.5; 3]); 2],
                white_sky_blue_ground(),
            )
            .unwrap()
            .with_part_opacities(opacities)
            .unwrap();
            let case = format!("{name} (floor first: {floor_first})");
            let (mut floor_returned, mut bottom_returned) = (0, 0);
            let rays = through_the_bottom();
            assert!(rays.len() > 2000, "{case}: only {} rays", rays.len());
            for (at, (inside, above)) in rays.iter().enumerate() {
                let mut traced = inside.clone();
                match scene.parts().trace(&mut traced).map(|hit| hit.part.index()) {
                    Some(part) if part == at_floor => floor_returned += 1,
                    Some(_) => bottom_returned += 1,
                    None => panic!("{case}: ray {at} met nothing"),
                }
                let t = -inside.origin.z / inside.direction_normal.z;
                let lands = inside.origin + inside.direction_normal * t;
                for (from, ray) in [("inside", inside), ("above", above)] {
                    let gone = scene.line_vertex_at(ray.clone(), &[at_box]).unwrap();
                    let gone =
                        gone.unwrap_or_else(|| panic!("{case}: {from} ray {at} met nothing"));
                    assert_eq!(
                        (gone.vertex.part, gone.vertex.back_face, gone.vertex.across),
                        (at_floor, false, Some(at_box)),
                        "{case}: {from} ray {at} past the box took {gone:?}, not the floor"
                    );
                    // On the floor's plane exactly, where the line crosses it to within the `f32`
                    // trace's rounding over the line's length.
                    assert!(
                        gone.position.z == 0.0 && (gone.position - lands).length() < 1e-4,
                        "{case}: {from} ray {at} landed at {:?}, not {lands:?}",
                        gone.position
                    );
                    assert_eq!(gone.decided, vec![at_box], "{case}: {from} ray {at}");
                }
                // Found there, the line ends on the box's bottom from inside, decided once.
                let there = scene.line_vertex_at(inside.clone(), &[]).unwrap().unwrap();
                assert_eq!(
                    (
                        there.vertex.part,
                        there.vertex.back_face,
                        there.vertex.across
                    ),
                    (at_box, true, None),
                    "{case}: inside ray {at} with the box there took {there:?}"
                );
                assert_eq!(there.decided, vec![at_box], "{case}: inside ray {at}");
            }
            println!(
                "{case}: {} rays, the trace returned the floor first {floor_returned} times and \
                 the box's bottom {bottom_returned}",
                rays.len()
            );
            assert!(
                floor_returned > 0 && bottom_returned > 0,
                "{case}: only one tie outcome was exercised"
            );
        }
    }
}

/// The GPU route's decision at a line's first hit in `pack`, the Rust mirror of the path loop on
/// the hit the `f32` reference traversal finds: `Some(part)` where the hit is a see-through
/// part's face on the floor, with whether the floor is nearer in `f32` (which of the two the
/// loop's trace would take first) and whether the loop meets the floor first
/// (`see_through_floor_first_f32`).
fn gpu_see_through_on_floor(pack: &GpuRenderScene, ray: &Ray) -> Option<(u32, bool, bool)> {
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
    let plane = pack.view().floor_plane;
    let part = &pack.parts()[face.part as usize];
    // The floor's distance as `path_floor_distance` finds it for these lines, all over the square.
    let floor_t = (plane[2] - flat.origin[2]) / flat.direction[2];
    let nearer = floor_t >= 0.0 && floor_t < hit.t;
    let first = see_through_floor_first_f32(
        flat.direction,
        position,
        &face,
        part,
        plane,
        (floor_t, nearer),
    );
    see_through_on_floor_f32(position, &face, part, plane).then_some((face.part, nearer, first))
}

/// `inside` turned around and started 5 under the floor, below where it crosses it.
fn from_under_the_floor(inside: &Ray) -> Ray {
    let t = -inside.origin.z / inside.direction_normal.z;
    let lands = inside.origin + inside.direction_normal * t;
    Ray::with_limits(
        lands + inside.direction_normal * 5.0,
        -inside.direction_normal,
        0.0,
        f64::INFINITY,
        IntersectionType::Both,
    )
}

#[test]
fn the_gpu_rule_lands_every_line_leaving_a_see_through_box_on_its_floor() {
    for floor_half in [50.0, 32_000.0] {
        let pack = gpu_pack_with(&[], &one_box(0.0), &[0.5], floor_half);
        let (mut floor_nearer, mut bottom_nearer) = (0, 0);
        for (at, (inside, _)) in through_the_bottom().iter().enumerate() {
            let decided = gpu_see_through_on_floor(&pack, inside);
            let Some((part, floor_is_nearer, floor_met_first)) = decided else {
                panic!("floor {floor_half}: ray {at} is not taken onto the floor");
            };
            // Leaving the box, the line meets its bottom first.
            assert_eq!(
                (part, floor_met_first),
                (0, false),
                "floor {floor_half}: ray {at}"
            );
            if floor_is_nearer {
                floor_nearer += 1;
            } else {
                bottom_nearer += 1;
            }
            // The same line coming up from under the floor meets the floor first.
            assert_eq!(
                gpu_see_through_on_floor(&pack, &from_under_the_floor(inside))
                    .map(|(part, _, floor_met_first)| (part, floor_met_first)),
                Some((0, true)),
                "floor {floor_half}: ray {at} from below"
            );
        }
        println!(
            "floor half side {floor_half}: the floor is nearer in f32 for {floor_nearer} lines, \
             the box's bottom for {bottom_nearer}"
        );
        assert!(floor_nearer + bottom_nearer > 2000);
    }
    // A fully there box on the floor is not taken by the see-through rule.
    let opaque = gpu_pack_with(&[], &one_box(0.0), &[1.0], 50.0);
    for (at, (inside, _)) in through_the_bottom().iter().enumerate() {
        assert_eq!(gpu_see_through_on_floor(&opaque, inside), None, "ray {at}");
    }
}

#[test]
fn a_line_from_under_the_floor_meets_the_floor_before_a_see_through_part_on_it() {
    for floor_first in [false, true] {
        let (scene, at_box, at_floor) =
            box_on_floor(0.0, 0.5, floor_first, white_sky_blue_ground());
        for (at, (inside, _)) in through_the_bottom().iter().enumerate() {
            let up = from_under_the_floor(inside);
            for absent in [vec![], vec![at_box]] {
                let vertex = scene.line_vertex_at(up.clone(), &absent).unwrap().unwrap();
                assert_eq!(
                    (
                        vertex.vertex.part,
                        vertex.vertex.back_face,
                        vertex.decided.len()
                    ),
                    (at_floor, true, 0),
                    "floor first {floor_first}: ray {at} from under the floor took {vertex:?}"
                );
            }
        }
    }
}

#[test]
fn a_see_through_box_and_a_glass_box_resting_side_by_side_each_keep_their_rule() {
    let boxes = [
        ([20.0; 3], [-15.0, 0.0, 10.0]),
        ([20.0; 3], [15.0, 0.0, 10.0]),
    ];
    let scene_of = |lift: f64| {
        let lifted: Vec<_> = boxes
            .iter()
            .map(|&(size, center)| (size, [center[0], center[1], center[2] + lift]))
            .collect();
        let mut parts: Vec<PlacedPart> = box_meshes(&lifted)
            .iter()
            .map(|mesh| part_of(mesh, Matrix4X4::IDENTITY))
            .collect();
        parts.push(part_of(&wide_floor(), Matrix4X4::IDENTITY));
        let surfaces = vec![
            ScatteringSurface::matte_plastic([0.8, 0.3, 0.3]),
            ScatteringSurface::glass(),
            ScatteringSurface::matte_plastic([0.3; 3]),
        ];
        RenderScene::new(PlacedParts::new(parts), surfaces, white_sky_blue_ground())
            .unwrap()
            .with_part_opacities(vec![0.5, 1.0, 1.0])
            .unwrap()
    };
    let scene = scene_of(0.0);
    let shifted = |ray: &Ray, x: f64| {
        Ray::with_limits(
            ray.origin + Vector3::new(x, 0.0, 0.0),
            ray.direction_normal,
            0.0,
            f64::INFINITY,
            IntersectionType::Both,
        )
    };
    for (at, (inside, _)) in through_the_bottom().iter().enumerate() {
        let past = scene
            .line_vertex_at(shifted(inside, -15.0), &[0])
            .unwrap()
            .unwrap();
        assert_eq!(
            (past.vertex.part, past.vertex.across, past.decided),
            (2, Some(0), vec![0]),
            "ray {at} past the see-through box"
        );
        let glass = scene
            .path_vertex_at(shifted(inside, 15.0))
            .unwrap()
            .unwrap();
        assert_eq!(
            (glass.part, glass.back_face, glass.across),
            (1, true, Some(2)),
            "ray {at}: not the glass's exit with the floor across it"
        );
    }
    let (resting, resting_leaks) = channels_and_ground_leaks(&scene);
    let (lifted, lifted_leaks) = channels_and_ground_leaks(&scene_of(0.01));
    assert_eq!(
        (resting_leaks, lifted_leaks),
        (0, 0),
        "paths reached the ground"
    );
    for (channel, (a, b)) in resting.iter().zip(&lifted).enumerate() {
        let tolerance = agreement_tolerance(a, b);
        println!(
            "side by side, channel {channel}: resting {} lifted {} (tolerance {tolerance})",
            a.mean(),
            b.mean()
        );
        assert!(
            (a.mean() - b.mean()).abs() <= tolerance,
            "channel {channel}"
        );
    }
}

/// The text of `fn name(` through its closing brace at the start of a line, its comments dropped
/// and its whitespace collapsed, its line endings made `\n` first: a checkout may hold the shader
/// with `\r\n`.
pub(crate) fn function_text(source: &str, name: &str) -> String {
    let source = source.replace("\r\n", "\n");
    let start = source
        .find(&format!("fn {name}("))
        .unwrap_or_else(|| panic!("no fn {name}"));
    let end = source[start..]
        .find("\n}\n")
        .unwrap_or_else(|| panic!("fn {name} never ends"));
    let body = &source[start..start + end];
    body.lines()
        .map(|line| line.split("//").next().unwrap_or(""))
        .flat_map(str::split_whitespace)
        .collect::<Vec<_>>()
        .join(" ")
}

#[test]
fn the_shader_loop_carries_the_see_through_rest_rule() {
    let rule = function_text(PATH_WGSL, "path_see_through_on_floor");
    assert!(
        rule.contains("return path_part_opacity(point.part) < 1.0f && path_rests_on_floor(point);"),
        "the shader's rule is not the mirror's: {rule}"
    );
    // `see_through_floor_first_f32`, step for step.
    let first = function_text(PATH_WGSL, "path_see_through_floor_first");
    assert!(
        first.contains(
            "if !path_see_through_on_floor(over) { return floor_first; } let leaving = \
             dot(direction, over.geometric) > 0.0f; return !leaving && (floor_first || floor_t \
             >= 0.0f);"
        ),
        "the shader's tie rule is not the mirror's: {first}"
    );
    let body = function_text(PATH_WGSL, "path_radiance");
    for step in [
        // Each face met from its own side first, only in a scene with see-through parts: a scene
        // with none skips it.
        "if hit.triangle != NO_HIT && !resting && (path_view.flags & PATH_FLAG_SEE_THROUGH) != \
         0u { let over = path_surface_point(hit); let taken = path_see_through_floor_first(over, \
         ray.direction, floor_t, floor_first); if taken && !floor_first { point = \
         path_floor_point(ray, floor_t); } else if !taken && floor_first { point = over; } \
         floor_first = taken; }",
        // A line passing the part out through the face lands on the floor, untraced.
        "if !present { if dot(ray.direction, point.geometric) > 0.0f && \
         path_see_through_on_floor(point) { ray = path_ray_from(point.position, ray.direction); \
         rest_hit = hit; across = PATH_ACROSS_FLOOR; continue; }",
        // The part across the contact from the floor is decided as at a traced face.
        "resting = true; if (path_view.flags & PATH_FLAG_SEE_THROUGH) != 0u { meets = \
         path_part_opacity(point.part) < 1.0f; resting = !meets; }",
        "rest_hit = hit; } resting = resting || meets; }",
    ] {
        assert!(body.contains(step), "the path loop no longer has `{step}`");
    }
    // The coverage line's walk is pinned to its mirror by `see_through_coverage_tests`.
}
