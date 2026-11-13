//! Glass resting on the floor with part opacity in the scene: the resting rule is decided per
//! contact, on both routes. A half-there part elsewhere in the scene leaves the glass's decisions
//! as they were and no path reaches the ground below the floor; glass that is itself half there
//! and found there leaves through its bottom by its exit event with the floor across, as fully
//! there glass does, while glass on a floor that is half there keeps the contact out of it (the
//! path decides at the face a trace returns), to the bit; a glass box lifted a modelled 0.05 off a
//! floor of half side 200 is a real gap on both routes, while over one of half side 40,000 the CPU
//! takes it as resting (the contact tolerance follows the floor triangle's size, a documented
//! limit); and the shader's loop carries the rule and its step across the contact, which nothing
//! else would notice being removed.

use solidmesh_math::{Matrix4X4, Ray, Vector3};
use solidmesh_mesh::ray_tracer::flat_tree::PATH_WGSL;
use solidmesh_mesh::ray_tracer::{PlacedPart, PlacedParts, RenderScene, ScatteringSurface};

use crate::glass_on_floor_tests::{
    downward_rays, furnace, gpu_pack_with, gpu_resting_glass, one_box, red_and_ground_leaks,
    white_sky_blue_ground,
};
use crate::glass_tray_support::{box_meshes, part_of};
use crate::path_tracer_golden_tests::render_digest;
use crate::path_tracer_support::{agreement_tolerance, flat_patch};

/// A matte 20 box far outside every view these tests take, the part made half there.
const FAR_BOX: ([f64; 3], [f64; 3]) = ([20.0; 3], [150.0, 0.0, 10.0]);

/// The 20 glass box lifted `lift` over a matte floor of half side 200 at z = 0, with `FAR_BOX`
/// after them when `far_box`; the parts at `opacities` in that order (glass, floor, far box).
fn glass_floor_scene(
    lift: f64,
    far_box: bool,
    opacities: &[f64],
    lights: solidmesh_mesh::ray_tracer::LightSetup,
) -> RenderScene {
    glass_over_floor(200.0, lift, far_box, opacities, lights)
}

/// `glass_floor_scene` over a floor of half side `floor_half`.
fn glass_over_floor(
    floor_half: f64,
    lift: f64,
    far_box: bool,
    opacities: &[f64],
    lights: solidmesh_mesh::ray_tracer::LightSetup,
) -> RenderScene {
    let mut parts: Vec<PlacedPart> = box_meshes(&one_box(lift))
        .iter()
        .map(|mesh| part_of(mesh, Matrix4X4::IDENTITY))
        .collect();
    let mut surfaces = vec![ScatteringSurface::glass()];
    parts.push(part_of(&flat_patch(floor_half, 0.0), Matrix4X4::IDENTITY));
    surfaces.push(ScatteringSurface::matte_plastic([0.3; 3]));
    if far_box {
        parts.push(part_of(&box_meshes(&[FAR_BOX])[0], Matrix4X4::IDENTITY));
        surfaces.push(ScatteringSurface::matte_plastic([0.6; 3]));
    }
    RenderScene::new(PlacedParts::new(parts), surfaces, lights)
        .unwrap()
        .with_part_opacities(opacities.to_vec())
        .unwrap()
}

/// The rays from inside the glass that reach z = 0 inside its footprint.
fn bottom_rays() -> Vec<Ray> {
    downward_rays(9.5)
        .into_iter()
        .filter(|ray| {
            let t = -ray.origin.z / ray.direction_normal.z;
            let reaches = ray.origin + ray.direction_normal * t;
            reaches.x.abs() < 10.0 && reaches.y.abs() < 10.0
        })
        .collect()
}

#[test]
fn a_half_there_part_elsewhere_leaves_glass_on_the_floor_resting() {
    let plain = glass_floor_scene(0.0, false, &[1.0, 1.0], furnace());
    let with_far = glass_floor_scene(0.0, true, &[1.0, 1.0, 0.5], furnace());
    let rays = bottom_rays();
    assert!(rays.len() > 2000, "only {} rays", rays.len());
    for (at, ray) in rays.iter().enumerate() {
        let before = plain.path_vertex_at(ray.clone()).unwrap();
        let after = with_far.path_vertex_at(ray.clone()).unwrap();
        assert_eq!(
            before, after,
            "ray {at}: the far half-there box moved the decision"
        );
        assert_eq!(
            after.and_then(|vertex| vertex.across),
            Some(1),
            "ray {at}: {after:?} is not the glass's exit with the floor across it"
        );
    }
    // The GPU rule, with the far box packed half there.
    let gpu_plain = gpu_pack_with(&one_box(0.0), &[], &[1.0], 200.0);
    let gpu_far = gpu_pack_with(&one_box(0.0), &[FAR_BOX], &[1.0, 0.5], 200.0);
    for (at, ray) in rays.iter().enumerate() {
        assert_eq!(gpu_resting_glass(&gpu_plain, ray), Some(0), "ray {at}");
        assert_eq!(gpu_resting_glass(&gpu_far, ray), Some(0), "ray {at}");
    }
    // The path loop: no path gets past the glass's bottom to the ground under the floor, and the
    // box looks as it does with nothing half there in the scene.
    let resting = glass_floor_scene(0.0, true, &[1.0, 1.0, 0.5], white_sky_blue_ground());
    let lifted = glass_floor_scene(0.01, false, &[1.0, 1.0], white_sky_blue_ground());
    let (resting_red, resting_leaks) = red_and_ground_leaks(&resting, 40.0);
    let (lifted_red, _) = red_and_ground_leaks(&lifted, 40.0);
    let tolerance = agreement_tolerance(&resting_red, &lifted_red);
    println!(
        "resting beside a half-there box {} lifted {} (tolerance {tolerance}); ground leaks \
         {resting_leaks}",
        resting_red.mean(),
        lifted_red.mean()
    );
    assert_eq!(resting_leaks, 0, "paths reached the ground under the box");
    assert!(
        (resting_red.mean() - lifted_red.mean()).abs() <= tolerance,
        "resting {} against lifted {} (tolerance {tolerance})",
        resting_red.mean(),
        lifted_red.mean()
    );
}

/// What a half-there glass box on the floor, and a glass box on a half-there floor, render as.
/// The second was pinned before the rule was decided per contact and uses no contact. The first
/// moved from 0.3.0's 0x6125_995c_abcc_e5a7 when lines through the glass came to meet its bottom
/// first on both tie outcomes, found there to leave it by its exit event with the floor across,
/// and found not there to land on the floor (`see_through_rest`, `glass_rest`): in 0.3.0 a line
/// refracted out of the bottom was traced from below the floor and reached the ground under it.
const HALF_THERE_GLASS_ON_FLOOR_DIGEST: u64 = 0x24d8_dc07_8ad6_1290;
const GLASS_ON_HALF_THERE_FLOOR_DIGEST: u64 = 0x771d_004f_60e3_2798;

#[test]
fn half_there_glass_rests_and_glass_on_a_half_there_floor_does_not() {
    let cases = [
        (
            "half-there glass",
            [0.5, 1.0],
            HALF_THERE_GLASS_ON_FLOOR_DIGEST,
        ),
        (
            "half-there floor",
            [1.0, 0.5],
            GLASS_ON_HALF_THERE_FLOOR_DIGEST,
        ),
    ];
    // The renders first: they are what a user sees, and what the vertices below explain.
    let digests: Vec<u64> = cases
        .iter()
        .map(|(_, opacities, _)| {
            render_digest(&glass_floor_scene(0.0, false, opacities, furnace()))
        })
        .collect();
    for ((name, _, digest), got) in cases.iter().zip(&digests) {
        assert_eq!(got, digest, "{name}: got {got:#018x}");
    }
    // Half-there glass found there: its own exit event, the floor across, decided once.
    let glass = glass_floor_scene(0.0, false, &[0.5, 1.0], furnace());
    for (at, ray) in bottom_rays().iter().enumerate() {
        let there = glass.line_vertex_at(ray.clone(), &[]).unwrap().unwrap();
        assert_eq!(
            (
                there.vertex.part,
                there.vertex.back_face,
                there.vertex.across
            ),
            (0, true, Some(1)),
            "half-there glass: ray {at} took {there:?}"
        );
        assert_eq!(there.decided, vec![0], "half-there glass: ray {at}");
    }
    // Glass on a half-there floor: the face the trace returned, no contact.
    let floor = glass_floor_scene(0.0, false, &[1.0, 0.5], furnace());
    for (at, ray) in bottom_rays().iter().enumerate() {
        let vertex = floor.path_vertex_at(ray.clone()).unwrap().unwrap();
        let mut traced = ray.clone();
        let hit = floor.parts().trace(&mut traced).unwrap();
        assert_eq!(
            (vertex.part, vertex.face, vertex.across),
            (
                hit.part.index(),
                hit.surface_point().unwrap().face_index,
                None
            ),
            "half-there floor: ray {at} was not decided at the face the trace returned"
        );
    }
    // The GPU route's rule for glass at opacity 1 leaves half-there glass to the see-through rule.
    let gpu = gpu_pack_with(&one_box(0.0), &[], &[0.5], 200.0);
    for (at, ray) in bottom_rays().iter().enumerate() {
        assert_eq!(
            gpu_resting_glass(&gpu, ray),
            None,
            "half-there glass: ray {at}"
        );
    }
}

/// `bottom_rays` moved up with the glass lifted `lift`.
fn lifted_bottom_rays(lift: f64) -> Vec<Ray> {
    bottom_rays()
        .into_iter()
        .map(|ray| {
            let mut lifted = ray.clone();
            lifted.origin = ray.origin + Vector3::new(0.0, 0.0, lift);
            lifted
        })
        .collect()
}

#[test]
fn glass_a_modelled_gap_off_a_floor_of_half_side_200_is_not_resting_on_it() {
    let scene = glass_floor_scene(0.05, false, &[1.0, 1.0], furnace());
    let gpu = gpu_pack_with(&one_box(0.05), &[], &[1.0], 200.0);
    let rays = lifted_bottom_rays(0.05);
    let mut on_glass = 0;
    for (at, ray) in rays.iter().enumerate() {
        let vertex = scene.path_vertex_at(ray.clone()).unwrap().unwrap();
        assert_eq!(
            vertex.across, None,
            "ray {at}: {vertex:?} crosses a 0.05 gap"
        );
        if vertex.part == 0 {
            on_glass += 1;
        }
        assert_eq!(
            gpu_resting_glass(&gpu, ray),
            None,
            "ray {at}: the GPU rule rests"
        );
    }
    assert!(
        on_glass > 2000,
        "only {on_glass} rays met the glass's bottom"
    );
}

/// The CPU's contact tolerance follows the floor triangle's size (`glass_contacts`' header): about
/// 0.15 over a floor quad of half side 40,000, so glass lifted 0.05 there is taken as resting,
/// while the GPU's analytic floor keeps its tolerance tight and refuses it; a lift of 0.5 is a gap
/// on both. This pins a documented limit, not a goal: a change to the tolerance moves it on
/// purpose, and says so.
#[test]
fn over_a_floor_of_half_side_40000_the_cpu_takes_a_small_lift_as_resting() {
    const FLOOR_HALF: f64 = 40_000.0;
    for (lift, cpu_rests) in [(0.05, true), (0.5, false)] {
        let scene = glass_over_floor(FLOOR_HALF, lift, false, &[1.0, 1.0], furnace());
        let gpu = gpu_pack_with(&one_box(lift), &[], &[1.0], FLOOR_HALF);
        let mut on_glass = 0;
        for (at, ray) in lifted_bottom_rays(lift).iter().enumerate() {
            let vertex = scene.path_vertex_at(ray.clone()).unwrap().unwrap();
            if cpu_rests {
                assert_eq!(
                    (vertex.part, vertex.across),
                    (0, Some(1)),
                    "lift {lift}, ray {at}: {vertex:?} is not the glass's exit onto the floor"
                );
            } else {
                assert_eq!(
                    vertex.across, None,
                    "lift {lift}, ray {at}: {vertex:?} crosses the gap"
                );
            }
            if vertex.part == 0 {
                on_glass += 1;
            }
            assert_eq!(
                gpu_resting_glass(&gpu, ray),
                None,
                "lift {lift}, ray {at}: the GPU rule rests"
            );
        }
        assert!(
            on_glass > 2000,
            "lift {lift}: only {on_glass} rays met the glass's bottom"
        );
    }
}

/// The text of `fn name(` through its closing brace at the start of a line. The source's line
/// endings are made `\n` first: a checkout may hold the shader with `\r\n` endings, and what the
/// shader says doesn't depend on which.
fn function_body(source: &str, name: &str) -> String {
    let source = source.replace("\r\n", "\n");
    let start = source
        .find(&format!("fn {name}("))
        .unwrap_or_else(|| panic!("no fn {name}"));
    let end = source[start..]
        .find("\n}\n")
        .unwrap_or_else(|| panic!("fn {name} never ends"));
    source[start..start + end].to_string()
}

#[test]
fn the_shader_loop_decides_glass_resting_on_the_floor_and_steps_across() {
    let body = function_body(PATH_WGSL, "path_radiance");
    let flat = body.split_whitespace().collect::<Vec<_>>().join(" ");
    assert_eq!(
        body.matches("path_rests_on_floor(").count(),
        1,
        "the path loop must test glass resting on the floor exactly once"
    );
    // The rule is the Rust mirror's (`leaves_onto_floor_f32`): leaving through the face, the glass
    // fully there, the face on the floor, and glass; never the scene-wide see-through flag, in
    // that test or the `if` around it. The whole block is pinned, so the vertex it takes and the
    // contact it records for the step across can't be dropped either.
    assert!(
        flat.contains(
            "if hit.triangle != NO_HIT { let glass = path_surface_point(hit); if \
             dot(ray.direction, glass.geometric) > 0.0f && path_part_opacity(glass.part) >= \
             1.0f && path_rests_on_floor(glass) && has_glass(path_part_surface(glass.part)) { \
             floor_first = false; point = glass; resting = true; rest_hit = hit;"
        ),
        "the shader's resting block is not the mirror's"
    );
    // A resting vertex is decided by the rule alone: neither the line's presence decision nor
    // the seam's runs at it.
    for gate in [
        "if !floor_first && !resting && (path_view.flags & PATH_FLAG_SEE_THROUGH) != 0u {",
        "if !floor_first && !resting && (path_view.flags & PATH_FLAG_GLASS_CONTACTS) != 0u {",
    ] {
        assert!(flat.contains(gate), "the path loop no longer has `{gate}`");
    }
    // The scene-wide see-through flag only skips the steps for see-through parts in a scene that
    // has none: the presence gate, the tie rule on the floor and the part across from the floor.
    // Any other use would gate some other step on part opacity anywhere in the scene.
    let gates = [
        "if !floor_first && !resting && (path_view.flags & PATH_FLAG_SEE_THROUGH) != 0u {",
        "if hit.triangle != NO_HIT && !resting && (path_view.flags & PATH_FLAG_SEE_THROUGH) != 0u {",
        "resting = true; if (path_view.flags & PATH_FLAG_SEE_THROUGH) != 0u { meets = \
         path_part_opacity(point.part) < 1.0f; resting = !meets; }",
    ];
    for gate in gates {
        assert_eq!(
            flat.matches(gate).count(),
            1,
            "the path loop no longer has `{gate}`"
        );
    }
    assert_eq!(
        body.matches("PATH_FLAG_SEE_THROUGH").count(),
        gates.len(),
        "the path loop reads the scene-wide see-through flag other than at its three gates"
    );
    for consumed in [
        "if across == PATH_ACROSS_FLOOR { hit = rest_hit; floor_first = true; point = \
         path_floor_point_under(ray.origin); resting = true;",
        "} else if across == PATH_ACROSS_GLASS { hit = rest_hit; floor_first = false; point = \
         path_surface_point(rest_hit); resting = true;",
        "if resting && dot(wi_world, point.geometric) > 0.0f {",
        "across = select(PATH_ACROSS_FLOOR, PATH_ACROSS_GLASS, floor_first); line_restart(); \
         continue;",
    ] {
        assert!(
            flat.contains(consumed),
            "the path loop no longer has `{consumed}`"
        );
    }
}
