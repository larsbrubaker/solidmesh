//! A see-through glass part resting on a floor: found there at the face lying on the floor, it
//! leaves through that face by its own exit event onto the floor at the same point, and found not
//! there a line past it lands on the floor, so at every opacity no path reaches the ground under
//! the floor, and the part renders as one lifted a modelled gap off it, by a paired difference of
//! the two renders (they share every random stream); the GPU route's mirror takes the same lines
//! out the same way, ray for ray, and its shader carries the rule. Also the comparison every
//! resting render here is judged by, [`paired_against_lifted`]: its renders go through the path
//! loop itself (`sample_pixel`), so they hold the loop's own calls of the tie rule and the
//! landing, which the ray-by-ray hook (`line_vertex_at`) walks separately.

use solidmesh_math::{IntersectionType, Matrix4X4, Ray, Vector3};
use solidmesh_mesh::ray_tracer::flat_tree::{
    there_glass_leaves_onto_floor_f32, FlatRay, GpuRenderScene, FACE_BOTH, PATH_WGSL,
};
use solidmesh_mesh::ray_tracer::{
    PathTraceSettings, PathTracer, PlacedPart, PlacedParts, RenderCamera, RenderScene, RenderWork,
    ScatteringSurface,
};

use crate::glass_on_floor_tests::{
    gpu_pack_with, one_box, red_and_ground_leaks, white_sky_blue_ground,
};
use crate::glass_tray_support::{box_meshes, part_of};
use crate::scattering_support::Stats;
use crate::see_through_on_floor_tests::{function_text, through_the_bottom, wide_floor};

/// How a resting render compares with the lifted one, sample by sample.
pub(crate) struct Paired {
    /// Each channel's per-sample difference, resting minus lifted.
    pub(crate) difference: [Stats; 3],
    /// How many samples of each render came back bluer than red: under a white sky over a blue
    /// ground, with only colourless glass and red or grey surfaces, only a path that reached the
    /// ground below the floor does.
    pub(crate) leaks: (usize, usize),
}

impl Paired {
    /// The channels whose mean difference lies outside five standard errors of the per-sample
    /// differences, with the difference and the bound. The two renders draw the same random
    /// numbers for every sample until a path meets the modelled gap differently, so the spread of
    /// the differences is that of the paths the gap changes (it is zero where none does), and a
    /// systematic difference such as half the lines past a part going under the floor stands far
    /// outside it.
    pub(crate) fn disagreements(&self) -> Vec<(usize, f64, f64)> {
        self.difference
            .iter()
            .enumerate()
            .filter_map(|(channel, stats)| {
                let bound = stats.tolerance() + 1e-12;
                (stats.mean().abs() > bound).then_some((channel, stats.mean(), bound))
            })
            .collect()
    }
}

/// Renders `resting` and `lifted` over a 24 by 24 view from steeply above, so most lines through
/// a part on the floor leave it through its bottom, at 64 samples a pixel, sample for sample.
pub(crate) fn paired_against_lifted(resting: &RenderScene, lifted: &RenderScene) -> Paired {
    paired_against_lifted_with(resting, lifted, PathTraceSettings::default())
}

/// [`paired_against_lifted`] rendered with `settings`.
pub(crate) fn paired_against_lifted_with(
    resting: &RenderScene,
    lifted: &RenderScene,
    settings: PathTraceSettings,
) -> Paired {
    let camera = RenderCamera::orthographic(
        Vector3::new(0.0, -8.0, 80.0),
        Vector3::new(0.0, 0.0, 10.0),
        Vector3::UNIT_Y,
        30.0,
    )
    .unwrap();
    let first = PathTracer::new(resting, camera, 24, 24, settings).unwrap();
    let second = PathTracer::new(lifted, camera, 24, 24, settings).unwrap();
    let mut work = RenderWork::default();
    let mut difference = [Stats::default(), Stats::default(), Stats::default()];
    let mut leaks = (0, 0);
    let leaked = |rgb: [f64; 3]| usize::from(rgb[2] > rgb[0] + 1e-12);
    for y in 0..24 {
        for x in 0..24 {
            for sample in 0..64 {
                let a = first.sample_pixel(x, y, sample, &mut work).unwrap();
                let b = second.sample_pixel(x, y, sample, &mut work).unwrap();
                for (stats, (a, b)) in difference.iter_mut().zip(a.iter().zip(&b)) {
                    stats.add(a - b);
                }
                leaks.0 += leaked(a);
                leaks.1 += leaked(b);
            }
        }
    }
    Paired { difference, leaks }
}

/// A clear glass box of side 20 lifted `lift` over the wide matte grey floor (`wide_floor`, so a
/// sample reaches the blue ground only by going under it), the floor first when `floor_first`,
/// the glass at `opacity`.
fn glass_box_on_floor(lift: f64, opacity: f64, floor_first: bool) -> RenderScene {
    let glass = part_of(&box_meshes(&one_box(lift))[0], Matrix4X4::IDENTITY);
    let floor = part_of(&wide_floor(), Matrix4X4::IDENTITY);
    let clear = ScatteringSurface::glass();
    let grey = ScatteringSurface::matte_plastic([0.3; 3]);
    let (parts, surfaces, opacities): (Vec<PlacedPart>, _, _) = if floor_first {
        (vec![floor, glass], vec![grey, clear], vec![1.0, opacity])
    } else {
        (vec![glass, floor], vec![clear, grey], vec![opacity, 1.0])
    };
    RenderScene::new(PlacedParts::new(parts), surfaces, white_sky_blue_ground())
        .unwrap()
        .with_part_opacities(opacities)
        .unwrap()
}

#[test]
fn half_there_glass_resting_on_a_floor_sends_no_path_under_it() {
    // Every case is measured before any is judged, so a failure shows them all.
    let mut failures = Vec::new();
    for opacity in [0.25, 0.5, 0.75, 0.9, 0.99] {
        for floor_first in [false, true] {
            let paired = paired_against_lifted(
                &glass_box_on_floor(0.0, opacity, floor_first),
                &glass_box_on_floor(0.01, opacity, floor_first),
            );
            let case = format!("opacity {opacity} (floor first: {floor_first})");
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
    }
    assert!(failures.is_empty(), "{failures:#?}");
}

/// The GPU route's decision at a line's first hit in `pack` when the line finds the part there,
/// the Rust mirror of the path loop on the hit the `f32` reference traversal finds: the part hit,
/// and whether the loop takes it as see-through glass leaving onto the floor.
fn gpu_there_glass_leaves(pack: &GpuRenderScene, ray: &Ray) -> Option<(u32, bool)> {
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
    let part = &pack.parts()[face.part as usize];
    let plane = pack.view().floor_plane;
    Some((
        face.part,
        there_glass_leaves_onto_floor_f32(flat.direction, position, &face, part, plane),
    ))
}

#[test]
fn both_routes_take_half_there_glass_found_there_out_onto_the_floor_ray_for_ray() {
    for floor_first in [false, true] {
        let scene = glass_box_on_floor(0.0, 0.5, floor_first);
        let (at_glass, at_floor) = if floor_first { (1, 0) } else { (0, 1) };
        for (at, (inside, _)) in through_the_bottom().iter().enumerate() {
            let there = scene.line_vertex_at(inside.clone(), &[]).unwrap().unwrap();
            assert_eq!(
                (
                    there.vertex.part,
                    there.vertex.back_face,
                    there.vertex.across
                ),
                (at_glass, true, Some(at_floor)),
                "floor first {floor_first}: ray {at} with the glass there took {there:?}"
            );
            assert_eq!(there.decided, vec![at_glass], "ray {at}");
            let gone = scene
                .line_vertex_at(inside.clone(), &[at_glass])
                .unwrap()
                .unwrap();
            assert_eq!(
                (gone.vertex.part, gone.vertex.across),
                (at_floor, Some(at_glass)),
                "floor first {floor_first}: ray {at} past the glass took {gone:?}"
            );
        }
    }
    for floor_half in [50.0, 32_000.0] {
        let pack = gpu_pack_with(&one_box(0.0), &[], &[0.5], floor_half);
        for (at, (inside, _)) in through_the_bottom().iter().enumerate() {
            assert_eq!(
                gpu_there_glass_leaves(&pack, inside),
                Some((0, true)),
                "floor {floor_half}: ray {at}"
            );
        }
        let opaque = gpu_pack_with(&one_box(0.0), &[], &[1.0], floor_half);
        for (at, (inside, _)) in through_the_bottom().iter().enumerate() {
            assert_eq!(
                gpu_there_glass_leaves(&opaque, inside),
                Some((0, false)),
                "floor {floor_half}: ray {at} through glass at opacity 1"
            );
        }
    }
}

#[test]
fn the_shader_loop_takes_see_through_glass_found_there_out_onto_the_floor() {
    let rule = function_text(PATH_WGSL, "path_there_glass_leaves_onto_floor");
    assert!(
        rule.contains(
            "return dot(direction, point.geometric) > 0.0f && path_see_through_on_floor(point) && \
             has_glass(path_part_surface(point.part));"
        ),
        "the shader's rule is not the mirror's: {rule}"
    );
    let body = function_text(PATH_WGSL, "path_radiance");
    let step = "present = verdict != LINE_PASSES; if present && \
                path_there_glass_leaves_onto_floor(point, ray.direction) { resting = true; \
                rest_hit = hit; } resting = resting || meets; }";
    assert!(body.contains(step), "the path loop no longer has `{step}`");
}

/// Half-there glass resting on the wide floor with a fully there glass box of the same footprint
/// under the floor, its top in the floor's plane: three coplanar faces, two of them glass.
fn glass_on_floor_over_glass(opacity: f64) -> RenderScene {
    let parts = vec![
        part_of(&box_meshes(&one_box(0.0))[0], Matrix4X4::IDENTITY),
        part_of(&wide_floor(), Matrix4X4::IDENTITY),
        part_of(
            &box_meshes(&[([20.0; 3], [0.0, 0.0, -10.0])])[0],
            Matrix4X4::IDENTITY,
        ),
    ];
    let surfaces = vec![
        ScatteringSurface::glass(),
        ScatteringSurface::matte_plastic([0.3; 3]),
        ScatteringSurface::glass(),
    ];
    RenderScene::new(PlacedParts::new(parts), surfaces, white_sky_blue_ground())
        .unwrap()
        .with_part_opacities(vec![opacity, 1.0, 1.0])
        .unwrap()
}

/// The mean red of `red_and_ground_leaks`'s view of `glass_on_floor_over_glass` at 0.5, pinned to
/// its bits: a path that looked for a glass-on-glass seam at a hit on the see-through rest contact
/// (the lower glass's top lies there too) would decide the seam by which face the trace returned,
/// and moves it (measured: 0.32882, against 0.32559). Paths do reach the ground here: the lower
/// glass's top is on no contact, so a trace that returns it in the floor's plane enters it.
#[test]
fn half_there_glass_on_a_floor_over_glass_looks_for_no_seam_on_the_contact() {
    let (red, leaks) = red_and_ground_leaks(&glass_on_floor_over_glass(0.5), 40.0);
    println!("mean red {:?}, ground leaks {leaks}", red.mean());
    assert_eq!(
        red.mean().to_bits(),
        HALF_THERE_GLASS_OVER_GLASS_RED.to_bits()
    );
}

const HALF_THERE_GLASS_OVER_GLASS_RED: f64 = 0.325_591_366_369_197_94;
