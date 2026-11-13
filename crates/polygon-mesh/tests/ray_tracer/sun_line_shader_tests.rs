//! The GPU route's line toward the sun (`path_sun_line` in `sun.wgsl`, its Rust mirror
//! `sun_line_f32` on the hits the `f32` reference traversal finds) against the CPU's
//! (`RenderScene::sun_line_at`, the CPU's own `sun_reach_from`), line for line: whether it reached
//! the sun and how many faces of glass it crossed agree exactly, and what the glass passed to
//! `f32` rounding. The scenes cover each step of the walk and of its start: clear, tinted and
//! frosted panes, two stacked boxes of one glass (a seam passed straight), a 1.7 glass on a 1.5
//! one (an interface, and a line starting at it), glass resting on the floor (a line from the
//! floor under it starting across the contact), a see-through matte box and a see-through glass
//! box on the floor (the part across decided by the line's own draw, both ways), an opaque part
//! above glass, the crossing cap, and a receiver inside glass (its first face and the bent
//! direction). `sun_line_text_tests` pins the shader's text to the mirror's steps.

use solidmesh_math::{IntersectionType, Matrix4X4, Ray, Vector3};
use solidmesh_mesh::ray_tracer::flat_tree::{
    floor_point_under_f32, sun_bent_f32, sun_line_f32, surface_point_f32, FlatRay, FloorContactF32,
    GpuCompositing, GpuFloor, GpuRenderScene, GpuSceneLimits, GpuView, LineRecordF32, SeamAtF32,
    SunLineF32, SunLineScene, SunVertexF32, FACE_BOTH, MAX_GLASS_CROSSINGS_F32, PATH_WGSL,
    SCENE_FLAG_SUN_STRAIGHT, WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
};
use solidmesh_mesh::ray_tracer::{
    PathTraceSettings, PixelRandom, PlacedPart, PlacedParts, RenderCamera, RenderScene,
    ScatteringSurface, SunLineAt, SunThroughGlass,
};
use solidmesh_mesh::Mesh;

use crate::flat_tree_shader_tests::literal;
use crate::glass_on_floor_tests::furnace;
use crate::glass_tray_support::{box_meshes, part_of};
use crate::see_through_on_floor_tests::wide_floor;

/// One box of the scene: its size, center, surface and opacity.
type Piece = ([f64; 3], [f64; 3], ScatteringSurface, f64);

fn glass_of(color: [f64; 3], roughness: f64, ior: f64) -> ScatteringSurface {
    ScatteringSurface::new(color, 0.0, roughness, 1.0, ior)
}

fn matte() -> ScatteringSurface {
    ScatteringSurface::matte_plastic([0.6; 3])
}

/// A clear glass box far off the lines tested, so a scene whose own parts have no glass still
/// has glass a line passes.
fn glass_elsewhere() -> Piece {
    (
        [4.0; 3],
        [300.0, 300.0, 50.0],
        ScatteringSurface::glass(),
        1.0,
    )
}

/// The CPU scene of `pieces` (with the wide matte floor last when `floor`) and the GPU pack of
/// `pieces` alone, over its analytic floor at z = 0 when `floor`.
fn pair(pieces: &[Piece], floor: bool) -> (RenderScene, GpuRenderScene) {
    let meshes: Vec<Mesh> = pieces
        .iter()
        .map(|&(size, center, _, _)| box_meshes(&[(size, center)])[0].clone())
        .collect();
    let scene_of = |with_floor: bool| {
        let mut placed: Vec<PlacedPart> = meshes
            .iter()
            .map(|mesh| part_of(mesh, Matrix4X4::IDENTITY))
            .collect();
        let mut surfaces: Vec<ScatteringSurface> = pieces.iter().map(|piece| piece.2).collect();
        let mut opacities: Vec<f64> = pieces.iter().map(|piece| piece.3).collect();
        if with_floor {
            placed.push(part_of(&wide_floor(), Matrix4X4::IDENTITY));
            surfaces.push(ScatteringSurface::matte_plastic([0.3; 3]));
            opacities.push(1.0);
        }
        RenderScene::new(PlacedParts::new(placed), surfaces, furnace())
            .unwrap()
            .with_part_opacities(opacities)
            .unwrap()
    };
    let gpu_scene = scene_of(false);
    let camera = RenderCamera::orthographic(
        Vector3::new(0.0, -80.0, 60.0),
        Vector3::new(0.0, 0.0, 10.0),
        Vector3::UNIT_Z,
        40.0,
    )
    .unwrap();
    let compositing = if floor {
        GpuCompositing::Floor(GpuFloor {
            center: (0.0, 0.0),
            half_extent: 200.0,
            z: 0.0,
            show_through: 0.0,
            surface: ScatteringSurface::matte_plastic([0.3; 3]),
        })
    } else {
        GpuCompositing::Plain
    };
    let view = GpuView {
        camera,
        image: (4, 4),
        trace: (4, 4),
        lights: furnace(),
        settings: PathTraceSettings::default(),
        compositing,
    };
    let limits = GpuSceneLimits {
        max_storage_binding: WEBGPU_DEFAULT_MAX_STORAGE_BINDING,
        max_vertex_step: 1e-3,
    };
    let pack = GpuRenderScene::pack_view(&gpu_scene, &view, limits).unwrap();
    (scene_of(floor), pack)
}

/// Directions toward the sun: straight up and tilted 25 and 50 degrees two ways.
fn sun_directions() -> Vec<Vector3> {
    let mut directions = vec![Vector3::UNIT_Z];
    for (degrees, azimuth) in [(25.0_f64, 0.3_f64), (50.0, 2.1), (25.0, 4.0), (50.0, 5.5)] {
        let (tilt, turn) = (degrees.to_radians(), azimuth);
        directions.push(Vector3::new(
            tilt.sin() * turn.cos(),
            tilt.sin() * turn.sin(),
            tilt.cos(),
        ));
    }
    directions
}

fn narrow(v: Vector3) -> [f32; 3] {
    [v.x as f32, v.y as f32, v.z as f32]
}

fn packed(pack: &GpuRenderScene, position: Vector3) -> [f32; 3] {
    narrow(position - pack.scene_origin())
}

fn down_from(origin: Vector3) -> Ray {
    Ray::with_limits(
        origin,
        -Vector3::UNIT_Z,
        0.0,
        f64::INFINITY,
        IntersectionType::Both,
    )
}

/// The packed scene's nearest hit from `origin` along `direction` (world coordinates).
fn gpu_hit(
    pack: &GpuRenderScene,
    origin: Vector3,
    direction: Vector3,
) -> solidmesh_mesh::ray_tracer::flat_tree::FlatHit {
    let ray = FlatRay {
        origin: packed(pack, origin),
        direction: narrow(direction),
        t_min: 0.0,
        t_max: 3.0e38,
        face_mask: FACE_BOTH,
    };
    pack.tree()
        .trace_closest(&ray)
        .expect("the line meets a face")
}

/// The vertex on the floor at `(x, y)` with nothing resting there.
fn gpu_floor_vertex(pack: &GpuRenderScene, x: f64, y: f64) -> SunVertexF32 {
    let plane = pack.view().floor_plane;
    SunVertexF32 {
        point: floor_point_under_f32(packed(pack, Vector3::new(x, y, 0.0)), plane),
        seam: None,
        contact: None,
    }
}

/// The vertex on the floor under the part whose bottom face the line down from `inside` meets,
/// reached across the contact (`PATH_ACROSS_FLOOR`).
fn gpu_floor_under(pack: &GpuRenderScene, inside: Vector3) -> SunVertexF32 {
    let rest_hit = gpu_hit(pack, inside, -Vector3::UNIT_Z);
    let face = surface_point_f32(pack, &rest_hit);
    SunVertexF32 {
        point: floor_point_under_f32(face.position, pack.view().floor_plane),
        seam: None,
        contact: Some(FloorContactF32::FloorUnder { rest_hit }),
    }
}

/// The vertex the line from `origin` along `direction` first meets, on no seam or contact.
fn gpu_traced_vertex(pack: &GpuRenderScene, origin: Vector3, direction: Vector3) -> SunVertexF32 {
    SunVertexF32 {
        point: surface_point_f32(pack, &gpu_hit(pack, origin, direction)),
        seam: None,
        contact: None,
    }
}

/// The CPU's line and the mirror's from the same vertex toward `direction`, each drawing from its
/// own copy of `random`.
fn both(
    scene: &RenderScene,
    pack: &GpuRenderScene,
    (cpu_ray, absent, across): (&Ray, &[usize], bool),
    gpu_vertex: &SunVertexF32,
    direction: Vector3,
    stop_at_any: bool,
    random: &PixelRandom,
) -> (SunLineAt, SunLineF32) {
    let cpu = scene
        .sun_line_at(
            cpu_ray.clone(),
            absent,
            across,
            direction,
            stop_at_any,
            &mut random.clone(),
        )
        .unwrap()
        .expect("the CPU's line has a vertex");
    let walk = SunLineScene::new(pack, true, true);
    let mut draws = random.clone();
    let mut there = |opacity: f32| draws.next_unit() < f64::from(opacity);
    let gpu = sun_line_f32(
        &walk,
        gpu_vertex,
        narrow(direction),
        !stop_at_any,
        &mut LineRecordF32::default(),
        &mut there,
    );
    (cpu, gpu)
}

/// How far the mirror's transmittance may be from the CPU's: each crossing rounds the shader's
/// Fresnel, the tint's square root and the product a step or so of `f32` (2^-24, 6e-8, of a value
/// at most 1), and reads Fresnel at the `f32` direction's cosine to the `f32` normal, whose error
/// near these angles moves Fresnel by less; the errors add across crossings. 2e-7 a crossing is
/// about three such steps; the worst measured here is 3.6e-8 a crossing.
fn transmittance_bound(crossed: u32) -> f64 {
    2e-7 * f64::from(crossed.max(1))
}

/// Holds the two lines to agree: reached and crossed exactly, transmittance within the bound.
/// Answers the line's crossings and transmittance when it reached the sun.
fn agree(cpu: &SunLineAt, gpu: &SunLineF32, what: &str) -> Option<(u32, [f64; 3])> {
    match cpu.reach {
        None => {
            assert!(
                !gpu.reached,
                "{what}: the CPU's line is blocked, the GPU's reached: {gpu:?}"
            );
            None
        }
        Some((transmittance, crossed)) => {
            assert!(
                gpu.reached,
                "{what}: the CPU's line reached the sun, the GPU's is blocked"
            );
            assert_eq!(gpu.crossed, crossed, "{what}: faces of glass crossed");
            for channel in 0..3 {
                let error = (f64::from(gpu.transmittance[channel]) - transmittance[channel]).abs();
                assert!(
                    error <= transmittance_bound(crossed),
                    "{what}: transmittance {:?} against the CPU's {transmittance:?}",
                    gpu.transmittance
                );
            }
            Some((crossed, transmittance))
        }
    }
}

/// Fresnel at normal incidence from air into glass of index `ior`.
fn normal_fresnel(ior: f64) -> f64 {
    ((ior - 1.0) / (ior + 1.0)).powi(2)
}

fn random() -> PixelRandom {
    PixelRandom::for_pixel(7, 0, 0, 0)
}

#[test]
fn a_line_from_the_floor_through_a_pane_passes_what_the_cpu_line_passes() {
    // Clear, tinted and frosted panes 2 thick lifted 10 over the floor: the line crosses two
    // faces and carries `(1 - F)^2` times the base color (one crossing's tint squared).
    let tint = [0.8, 0.5, 0.3];
    let panes = [
        ("clear", glass_of([1.0; 3], 0.0, 1.5), [1.0; 3]),
        ("tinted", glass_of(tint, 0.0, 1.5), tint),
        ("frosted", glass_of([1.0; 3], 0.3, 1.5), [1.0; 3]),
    ];
    for (name, surface, color) in panes {
        let (scene, pack) = pair(&[([60.0, 60.0, 2.0], [0.0, 0.0, 11.0], surface, 1.0)], true);
        for (x, y) in [(0.0, 0.0), (3.7, -5.1)] {
            let vertex = gpu_floor_vertex(&pack, x, y);
            let cpu_ray = down_from(Vector3::new(x, y, 5.0));
            for direction in sun_directions() {
                let (cpu, gpu) = both(
                    &scene,
                    &pack,
                    (&cpu_ray, &[], false),
                    &vertex,
                    direction,
                    false,
                    &random(),
                );
                let (crossed, transmittance) =
                    agree(&cpu, &gpu, name).expect("the pane passes the sun");
                assert_eq!(crossed, 2, "{name}");
                if direction.z == 1.0 {
                    let pass = (1.0 - normal_fresnel(1.5)).powi(2);
                    for channel in 0..3 {
                        assert!((transmittance[channel] - pass * color[channel]).abs() < 1e-12);
                    }
                }
                // A line that stops at any surface stops at the pane.
                let (cpu, gpu) = both(
                    &scene,
                    &pack,
                    (&cpu_ray, &[], false),
                    &vertex,
                    direction,
                    true,
                    &random(),
                );
                assert_eq!(agree(&cpu, &gpu, name), None);
            }
        }
    }
}

#[test]
fn a_line_through_stacked_glass_passes_a_seam_of_one_glass_and_crosses_an_interface() {
    // Two boxes of side 20 stacked 5 over the floor: of one glass the seam is passed straight
    // (two faces crossed in all); of 1.7 on 1.5 it is one more interface (three).
    let lower = ([20.0; 3], [0.0, 0.0, 15.0]);
    let upper = ([20.0; 3], [0.0, 0.0, 35.0]);
    for (name, top, faces) in [
        ("one glass", glass_of([1.0; 3], 0.0, 1.5), 2),
        ("1.7 on 1.5", glass_of([1.0; 3], 0.0, 1.7), 3),
    ] {
        let pieces = [
            (lower.0, lower.1, glass_of([1.0; 3], 0.0, 1.5), 1.0),
            (upper.0, upper.1, top, 1.0),
        ];
        let (scene, pack) = pair(&pieces, true);
        for (x, y) in [(0.0, 0.0), (2.3, -1.9)] {
            let vertex = gpu_floor_vertex(&pack, x, y);
            let cpu_ray = down_from(Vector3::new(x, y, 3.0));
            for direction in [Vector3::UNIT_Z, Vector3::new(0.1, 0.05, 1.0).get_normal()] {
                let (cpu, gpu) = both(
                    &scene,
                    &pack,
                    (&cpu_ray, &[], false),
                    &vertex,
                    direction,
                    false,
                    &random(),
                );
                let (crossed, _) = agree(&cpu, &gpu, name).expect("the stack passes the sun");
                assert_eq!(crossed, faces, "{name}");
            }
        }
    }
}

#[test]
fn a_line_from_a_vertex_on_an_interface_starts_past_both_faces() {
    // The vertex is the interface between 1.5 glass below and 1.7 above, met from inside the
    // lower box; its line up starts past the seam, inside the upper box, and crosses its top.
    let pieces = [
        (
            [20.0; 3],
            [0.0, 0.0, 15.0],
            glass_of([1.0; 3], 0.0, 1.5),
            1.0,
        ),
        (
            [20.0; 3],
            [0.0, 0.0, 35.0],
            glass_of([1.0; 3], 0.0, 1.7),
            1.0,
        ),
    ];
    let (scene, pack) = pair(&pieces, true);
    let inside = Vector3::new(1.3, -2.2, 15.0);
    let hit = gpu_hit(&pack, inside, Vector3::UNIT_Z);
    let point = surface_point_f32(&pack, &hit);
    let walk = SunLineScene::new(&pack, true, true);
    let seam = match walk.seam_at(
        &point,
        hit.source_index,
        [0.0, 0.0, 1.0],
        true,
        &mut LineRecordF32::default(),
        &mut |_| true,
    ) {
        SeamAtF32::Interface(seam) => seam,
        other => panic!("the hit is on the interface: {other:?}"),
    };
    let vertex = SunVertexF32 {
        point,
        seam: Some(seam),
        contact: None,
    };
    let cpu_ray = Ray::with_limits(
        inside,
        Vector3::UNIT_Z,
        0.0,
        f64::INFINITY,
        IntersectionType::Both,
    );
    for direction in sun_directions() {
        for stop_at_any in [false, true] {
            let (cpu, gpu) = both(
                &scene,
                &pack,
                (&cpu_ray, &[], false),
                &vertex,
                direction,
                stop_at_any,
                &random(),
            );
            let reach = agree(&cpu, &gpu, "from the interface");
            if stop_at_any {
                assert_eq!(reach, None, "the upper box's top stops the line");
            } else {
                assert_eq!(reach.map(|(crossed, _)| crossed), Some(1));
            }
        }
    }
}

#[test]
fn a_line_from_the_floor_under_resting_glass_crosses_the_glass_across_the_contact_first() {
    // A clear box of side 20 resting on the floor: from the floor under it the line meets the
    // box's bottom across the contact (one crossing) and goes on to cross its top (two). From the
    // box's bottom face itself the line goes up inside and crosses the top alone. A line that
    // stops at any surface is stopped by the glass across.
    let (scene, pack) = pair(
        &[([20.0; 3], [0.0, 0.0, 10.0], ScatteringSurface::glass(), 1.0)],
        true,
    );
    for (x, y) in [(0.0, 0.0), (6.1, -4.4)] {
        let inside = Vector3::new(x, y, 5.0);
        let under = gpu_floor_under(&pack, inside);
        let face = SunVertexF32 {
            contact: Some(FloorContactF32::OverFloor),
            ..gpu_traced_vertex(&pack, inside, -Vector3::UNIT_Z)
        };
        for direction in sun_directions() {
            for (across, vertex, faces) in [(true, &under, 2), (false, &face, 1)] {
                let (cpu, gpu) = both(
                    &scene,
                    &pack,
                    (&down_from(inside), &[], across),
                    vertex,
                    direction,
                    false,
                    &random(),
                );
                let reach = agree(&cpu, &gpu, "resting glass");
                assert_eq!(reach.map(|(crossed, _)| crossed), Some(faces));
                let (cpu, gpu) = both(
                    &scene,
                    &pack,
                    (&down_from(inside), &[], across),
                    vertex,
                    direction,
                    true,
                    &random(),
                );
                assert_eq!(agree(&cpu, &gpu, "resting glass, stop at any"), None);
            }
        }
    }
}

#[test]
fn a_line_from_the_floor_under_a_half_there_part_decides_it_by_its_own_draw() {
    // A box of side 20 at opacity 0.5 resting on the floor, matte or glass, the floor vertex
    // under it reached past it: the line decides the box across the contact with its own draw,
    // and each seed's draws give the CPU and the mirror the same outcome. Matte found there stops
    // the line; found not there the line goes on with the box passed and reaches the sun. Glass
    // found there is crossed and decided again at its top by the fresh line after the crossing.
    for (name, surface) in [("matte", matte()), ("glass", ScatteringSurface::glass())] {
        let (scene, pack) = pair(
            &[
                ([20.0; 3], [0.0, 0.0, 10.0], surface, 0.5),
                glass_elsewhere(),
            ],
            true,
        );
        let inside = Vector3::new(2.5, 3.5, 10.0);
        let vertex = gpu_floor_under(&pack, inside);
        let cpu_ray = down_from(Vector3::new(2.5, 3.5, 40.0));
        let mut outcomes = Vec::new();
        for seed in 0..48 {
            let random = PixelRandom::for_pixel(seed, 0, 0, 0);
            for (direction, stop_at_any) in [(Vector3::UNIT_Z, false), (Vector3::UNIT_Z, true)] {
                let (cpu, gpu) = both(
                    &scene,
                    &pack,
                    (&cpu_ray, &[0], false),
                    &vertex,
                    direction,
                    stop_at_any,
                    &random,
                );
                let reach = agree(&cpu, &gpu, name).map(|(crossed, _)| crossed);
                if !outcomes.contains(&(stop_at_any, reach)) {
                    outcomes.push((stop_at_any, reach));
                }
            }
        }
        outcomes.sort_unstable();
        let expected: Vec<(bool, Option<u32>)> = if name == "matte" {
            vec![
                (false, None),
                (false, Some(0)),
                (true, None),
                (true, Some(0)),
            ]
        } else {
            vec![
                (false, Some(0)),
                (false, Some(1)),
                (false, Some(2)),
                (true, None),
                (true, Some(0)),
            ]
        };
        assert_eq!(outcomes, expected, "{name}: the outcomes the draws gave");
    }
}

#[test]
fn an_opaque_part_above_glass_blocks_the_line() {
    let (scene, pack) = pair(
        &[
            (
                [60.0, 60.0, 2.0],
                [0.0, 0.0, 11.0],
                ScatteringSurface::glass(),
                1.0,
            ),
            ([200.0, 200.0, 2.0], [0.0, 0.0, 30.0], matte(), 1.0),
        ],
        true,
    );
    let vertex = gpu_floor_vertex(&pack, 1.0, 2.0);
    for direction in sun_directions() {
        for stop_at_any in [false, true] {
            let (cpu, gpu) = both(
                &scene,
                &pack,
                (&down_from(Vector3::new(1.0, 2.0, 5.0)), &[], false),
                &vertex,
                direction,
                stop_at_any,
                &random(),
            );
            assert_eq!(agree(&cpu, &gpu, "opaque above glass"), None);
        }
    }
}

#[test]
fn a_line_through_more_faces_of_glass_than_the_cap_is_blocked() {
    // Panes 1 thick, 3 apart: eight of them are the cap's sixteen faces, nine are one too many.
    assert_eq!(MAX_GLASS_CROSSINGS_F32, 16);
    let module = naga::front::wgsl::parse_str(PATH_WGSL).expect("the path loop parses");
    assert_eq!(
        literal(&module, "PATH_MAX_GLASS_CROSSINGS"),
        naga::Literal::U32(MAX_GLASS_CROSSINGS_F32)
    );
    for (count, reached) in [(8, Some(16)), (9, None)] {
        let pieces: Vec<Piece> = (0..count)
            .map(|pane| {
                let z = 10.0 + 3.0 * f64::from(pane);
                (
                    [40.0, 40.0, 1.0],
                    [0.0, 0.0, z],
                    ScatteringSurface::glass(),
                    1.0,
                )
            })
            .collect();
        let (scene, pack) = pair(&pieces, true);
        let vertex = gpu_floor_vertex(&pack, 0.5, -0.5);
        let (cpu, gpu) = both(
            &scene,
            &pack,
            (&down_from(Vector3::new(0.5, -0.5, 5.0)), &[], false),
            &vertex,
            Vector3::UNIT_Z,
            false,
            &random(),
        );
        let reach = agree(&cpu, &gpu, "the cap").map(|(crossed, _)| crossed);
        assert_eq!(reach, reached, "{count} panes");
        // From the first pane's bottom face, inside it: one face fewer, so nine panes are
        // seventeen faces, the first past the cap.
        let below = Vector3::new(0.5, -0.5, 5.0);
        let up = Ray::with_limits(
            below,
            Vector3::UNIT_Z,
            0.0,
            f64::INFINITY,
            IntersectionType::Both,
        );
        let (cpu, gpu) = both(
            &scene,
            &pack,
            (&up, &[], false),
            &gpu_traced_vertex(&pack, below, Vector3::UNIT_Z),
            Vector3::UNIT_Z,
            false,
            &random(),
        );
        let reach = agree(&cpu, &gpu, "the cap, from inside").map(|(crossed, _)| crossed);
        let inside = if count == 8 { Some(15) } else { None };
        assert_eq!(reach, inside, "{count} panes from inside the first");
    }
}

#[test]
fn a_receiver_inside_glass_reads_the_first_face_and_its_bent_direction_as_the_cpu_does() {
    // A matte box inside a glass block of 1.5, lifted off the floor: from the matte box's top the
    // line's first trace meets the block's top from inside, its first face.
    let (scene, pack) = pair(
        &[
            ([40.0; 3], [0.0, 0.0, 30.0], ScatteringSurface::glass(), 1.0),
            ([6.0; 3], [0.0, 0.0, 30.0], matte(), 1.0),
        ],
        true,
    );
    let above = Vector3::new(0.7, -1.1, 45.0);
    let vertex = gpu_traced_vertex(&pack, above, -Vector3::UNIT_Z);
    for direction in sun_directions() {
        let (cpu, gpu) = both(
            &scene,
            &pack,
            (&down_from(above), &[], false),
            &vertex,
            direction,
            false,
            &random(),
        );
        assert_eq!(
            agree(&cpu, &gpu, "inside glass").map(|(crossed, _)| crossed),
            Some(1)
        );
        let (outward, ior) = cpu.first_face.expect("the CPU's line has a first face");
        let (gpu_outward, gpu_ior) = gpu.first_face.expect("the GPU's line has a first face");
        assert_eq!(f64::from(gpu_ior), ior);
        for (a, b) in gpu_outward.iter().zip([outward.x, outward.y, outward.z]) {
            assert!((f64::from(*a) - b).abs() <= 1e-7, "outward normal");
        }
        let (inside, cos_out, cos_in) = cpu.bent.expect("the line leaves the face outward");
        let (gpu_inside, gpu_cos_out, gpu_cos_in) =
            sun_bent_f32(gpu_outward, gpu_ior, narrow(direction)).expect("bent on the GPU");
        assert!((f64::from(gpu_cos_out) - cos_out).abs() <= 1e-6);
        assert!((f64::from(gpu_cos_in) - cos_in).abs() <= 1e-6);
        for (a, b) in gpu_inside.iter().zip([inside.x, inside.y, inside.z]) {
            assert!((f64::from(*a) - b).abs() <= 1e-6, "bent direction");
        }
    }
}

#[test]
fn the_pack_sets_the_straight_sun_flag_exactly_where_the_cpu_goes_straight() {
    let module = naga::front::wgsl::parse_str(PATH_WGSL).expect("the path loop parses");
    assert_eq!(
        literal(&module, "PATH_FLAG_SUN_STRAIGHT"),
        naga::Literal::U32(SCENE_FLAG_SUN_STRAIGHT)
    );
    let piece = |surface: ScatteringSurface| ([20.0; 3], [0.0, 0.0, 10.0], surface, 1.0);
    // The CPU goes straight under Straight in a scene where some part has a glass share: clear,
    // frosted, half glass, half there; not a matte part, nor a metal whose transmission no
    // dielectric share carries.
    for (pieces, has_glass) in [
        (vec![piece(ScatteringSurface::glass())], true),
        (vec![piece(glass_of([0.9, 0.8, 0.7], 0.3, 1.5))], true),
        (
            vec![piece(ScatteringSurface::new([0.8; 3], 0.0, 0.2, 0.5, 1.5))],
            true,
        ),
        (
            vec![
                piece(matte()),
                ([4.0; 3], [40.0, 0.0, 2.0], ScatteringSurface::glass(), 0.5),
            ],
            true,
        ),
        (vec![piece(matte())], false),
        (
            vec![piece(ScatteringSurface::new([0.8; 3], 1.0, 0.2, 1.0, 1.5))],
            false,
        ),
    ] {
        let (scene, mut pack) = pair(&pieces, false);
        let set = |pack: &GpuRenderScene| pack.view().flags & SCENE_FLAG_SUN_STRAIGHT != 0;
        // The default is Straight, as on the CPU.
        assert_eq!(set(&pack), has_glass, "{pieces:?}");
        for (setting, expected) in [
            (SunThroughGlass::Exact, false),
            (SunThroughGlass::Straight, has_glass),
        ] {
            let mut view = plain_view();
            view.settings.sun_through_glass = setting;
            pack.set_view(&scene, &view).unwrap();
            assert_eq!(set(&pack), expected, "{setting:?} {pieces:?}");
        }
    }
}

/// The view `pair` packs with no floor.
fn plain_view() -> GpuView {
    GpuView {
        camera: RenderCamera::orthographic(
            Vector3::new(0.0, -80.0, 60.0),
            Vector3::new(0.0, 0.0, 10.0),
            Vector3::UNIT_Z,
            40.0,
        )
        .unwrap(),
        image: (4, 4),
        trace: (4, 4),
        lights: furnace(),
        settings: PathTraceSettings::default(),
        compositing: GpuCompositing::Plain,
    }
}

#[test]
fn without_the_flag_the_line_is_the_plain_shadow_ray() {
    // No glass in the scene, so the CPU walks Exact's line: a half-there matte box lifted 10 over
    // the floor and an opaque one beside it. The mirror without `PATH_FLAG_SUN_STRAIGHT` makes
    // the same decisions from the same draws, blocked or not.
    let (scene, pack) = pair(
        &[
            ([20.0; 3], [0.0, 0.0, 20.0], matte(), 0.5),
            ([20.0; 3], [40.0, 0.0, 20.0], matte(), 1.0),
        ],
        true,
    );
    let walk = SunLineScene::new(&pack, true, false);
    let mut outcomes = Vec::new();
    for seed in 0..32 {
        let random = PixelRandom::for_pixel(seed, 0, 0, 0);
        for (x, direction) in [
            (1.0, Vector3::UNIT_Z),
            (38.0, Vector3::UNIT_Z),
            (-15.0, Vector3::new(0.4, 0.0, 1.0).get_normal()),
        ] {
            let cpu = scene
                .sun_line_at(
                    down_from(Vector3::new(x, 0.5, 5.0)),
                    &[],
                    false,
                    direction,
                    false,
                    &mut random.clone(),
                )
                .unwrap()
                .expect("the floor is the vertex");
            let mut draws = random.clone();
            let mut there = |opacity: f32| draws.next_unit() < f64::from(opacity);
            let gpu = sun_line_f32(
                &walk,
                &gpu_floor_vertex(&pack, x, 0.5),
                narrow(direction),
                true,
                &mut LineRecordF32::default(),
                &mut there,
            );
            let reach = agree(&cpu, &gpu, "no glass");
            if !outcomes.contains(&(x, reach.is_some())) {
                outcomes.push((x, reach.is_some()));
            }
        }
    }
    outcomes.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(
        outcomes,
        vec![
            (-15.0, false),
            (-15.0, true),
            (1.0, false),
            (1.0, true),
            (38.0, false)
        ]
    );
}
