// The demo pages' scenes, built natively with each page's starting controls: every page must
// produce a non-empty mesh, and every page that promises a closed solid must deliver one. These
// are the same builders the browser calls through `solidmesh_wasm`'s bindings, so a kernel change
// that breaks a page fails here before the site is deployed.

use solidmesh_wasm::booleans::{boolean_scene, BooleanParams};
use solidmesh_wasm::implicit::implicit_scene;
use solidmesh_wasm::morph::{hull_scene, minkowski_scene};
use solidmesh_wasm::outline::{contours_from_flat, extrude_scene, revolve_scene};
use solidmesh_wasm::repair::{repair_scene, RepairParams};
use solidmesh_wasm::showcase::showcase_part;
use solidmesh_wasm::Scene;

fn assert_solid(scene: &Scene, what: &str) {
    assert!(!scene.mesh.faces.is_empty(), "{what}: empty mesh");
    assert!(
        scene.mesh.is_closed_consistently_wound(),
        "{what}: not closed ({} open edges); {}",
        scene.mesh.get_non_manifold_edges().len(),
        scene.note
    );
    assert!(scene.mesh.get_volume() > 0.0, "{what}: no volume");
}

fn booleans(op: u32, messy: u32, nonzero: bool, repair: bool) -> Scene {
    boolean_scene(&BooleanParams {
        shape_a: 0,
        shape_b: 1,
        op,
        offset: [8.0, 6.0, 5.0],
        rotate_deg: 20.0,
        messy,
        nonzero,
        repair,
    })
    .expect("boolean scene")
}

#[test]
fn every_boolean_of_clean_operands_is_a_closed_solid() {
    for op in 0..3 {
        assert_solid(&booleans(op, 0, false, false), &format!("op {op}"));
    }
}

#[test]
fn messy_operands_still_give_a_closed_solid() {
    assert_solid(&booleans(0, 3, true, false), "nonzero union");
    assert_solid(&booleans(1, 3, false, true), "repaired difference");
}

#[test]
fn outlines_extrude_bevel_and_revolve_to_solids() {
    // A square with a square hole, in mm.
    let coords = [
        0.0, 0.0, 20.0, 0.0, 20.0, 20.0, 0.0, 20.0, 6.0, 6.0, 6.0, 14.0, 14.0, 14.0, 14.0, 6.0,
    ];
    let contours = contours_from_flat(&coords, &[4, 4]);
    assert_solid(
        &extrude_scene(&contours, 10.0, 0.0, 0.0, 4, true).expect("extrude"),
        "extrude",
    );
    assert_solid(
        &extrude_scene(&contours, 10.0, 2.0, 1.0, 4, true).expect("bevel"),
        "bevelled extrude",
    );
    let offset: Vec<f64> = coords.chunks(2).flat_map(|p| [p[0] + 5.0, p[1]]).collect();
    let profile = contours_from_flat(&offset, &[4, 4]);
    assert_solid(
        &revolve_scene(&profile, 48, 0.0, 360.0).expect("revolve"),
        "revolve",
    );
    assert_solid(
        &revolve_scene(&profile, 24, 0.0, 270.0).expect("partial revolve"),
        "partial revolve",
    );
}

#[test]
fn implicit_scenes_mesh_with_both_meshers() {
    for scene in 0..3 {
        for mesher in 0..2 {
            let result = implicit_scene(scene, 3.0, 48, mesher).expect("implicit scene");
            assert!(
                !result.mesh.faces.is_empty(),
                "scene {scene} mesher {mesher}: empty"
            );
        }
    }
    assert_solid(
        &implicit_scene(0, 3.0, 48, 0).expect("blob"),
        "marching cubes blob",
    );
}

#[test]
fn minkowski_and_hull_give_solids() {
    assert_solid(
        &minkowski_scene(1, 0, 4.0, false, 10).expect("sum"),
        "Minkowski sum",
    );
    assert_solid(
        &minkowski_scene(2, 1, 4.0, true, 10).expect("difference"),
        "Minkowski difference",
    );
    for source in 0..3 {
        assert_solid(
            &hull_scene(source, 40, 7).expect("hull"),
            &format!("hull {source}"),
        );
    }
}

#[test]
fn repair_closes_the_broken_mesh() {
    let params = |mode| RepairParams {
        shape: 0,
        holes: 3,
        flips: 12,
        soup: true,
        seed: 1,
        mode,
        fill_holes: true,
        face_orientation: true,
        weld_edges: true,
    };
    let broken = repair_scene(&params(0)).expect("broken");
    assert!(!broken.mesh.is_closed_consistently_wound(), "not broken");
    assert_solid(&repair_scene(&params(1)).expect("auto"), "auto repair");
    assert_solid(
        &repair_scene(&params(2)).expect("advanced"),
        "advanced repair",
    );
}

#[test]
fn the_showcase_part_is_one_closed_solid() {
    assert_solid(
        &showcase_part(6, 26.0, 3.0, 48).expect("showcase"),
        "showcase",
    );
    assert_solid(
        &showcase_part(0, 20.0, 0.0, 24).expect("plain"),
        "plain showcase",
    );
}

// The Ray Tracing page: the production scene, camera and renderer the wasm calls, run natively
// on known solids (a part, a cube, a sphere), reading back real pixels and hits.
mod ray_trace {
    use solidmesh::math::Vector3;
    use solidmesh::mesh::Mesh;
    use solidmesh_wasm::ray_trace::{Accumulation, TraceRequest};
    use solidmesh_wasm::ray_trace_scene::{Camera, Part, Surface, TraceScene};
    use solidmesh_wasm::ray_trace_shade::{
        depth_range, depth_value, sun_direction, CustomSliders, Light, Look, Material, Mode,
    };
    use solidmesh_wasm::shapes::Shape;

    const WIDTH: u32 = 120;
    const HEIGHT: u32 = 90;

    fn scene_of(mesh: &Mesh) -> TraceScene {
        TraceScene::new(mesh, "test".to_string()).expect("the scene builds")
    }

    fn look(material: Material, color: [u8; 3], light: Light) -> Look {
        Look {
            material,
            color,
            custom: CustomSliders {
                roughness: 0.3,
                metallic: 0.0,
                transmission: 0.0,
            },
            light,
            sun_degrees: 5.0,
        }
    }

    const TEAL: [u8; 3] = [64, 180, 160];
    const WHITE: [u8; 3] = [250, 250, 250];

    fn matte() -> Look {
        look(Material::MattePlastic, TEAL, Light::Studio)
    }

    fn request(mode: Mode, seed: u32) -> TraceRequest {
        TraceRequest {
            yaw: 0.9,
            pitch: 0.5,
            zoom: 1.0,
            mode,
            look: matte(),
            seed,
            width: WIDTH,
            height: HEIGHT,
            preview: false,
        }
    }

    /// `samples` whole samples of `request`, the same way a preview frame is rendered.
    fn render(scene: &TraceScene, request: TraceRequest, samples: u32) -> Accumulation {
        let mut image = Accumulation::new();
        for _ in 0..samples {
            image
                .pass(scene, request, 0.0, &mut || 0.0)
                .expect("the pass runs");
        }
        image
    }

    fn pixel(image: &Accumulation, x: u32, y: u32) -> [u8; 3] {
        let at = ((y * image.width() + x) * 4) as usize;
        let px = image.pixels();
        [px[at], px[at + 1], px[at + 2]]
    }

    fn brightness(color: [u8; 3]) -> u32 {
        color.iter().map(|&c| u32::from(c)).sum()
    }

    fn camera_of(scene: &TraceScene, r: &TraceRequest) -> Camera {
        Camera::new(scene, r.yaw, r.pitch, r.zoom, r.width, r.height).expect("the camera builds")
    }

    fn torus_scene() -> TraceScene {
        scene_of(&Shape::Torus.build(60.0, 64).expect("the torus builds"))
    }

    fn sphere_scene() -> TraceScene {
        scene_of(&Shape::Sphere.build(40.0, 96).expect("the sphere builds"))
    }

    /// The camera orbits the part's centre and looks at it, so a ray at the centre of the image
    /// must hit a solid part rather than run off into the background, and the shaded pixel must
    /// be the part's color rather than the blue-white sky or the neutral floor.
    #[test]
    fn a_ray_at_the_part_hits_it_and_the_centre_pixel_is_not_background() {
        let scene = sphere_scene();
        let req = request(Mode::PathTraced, 1);
        let camera = camera_of(&scene, &req);
        let centre = (f64::from(WIDTH) / 2.0, f64::from(HEIGHT) / 2.0);

        let (surface, distance) = scene.surface_at(&camera, centre.0, centre.1);

        assert_eq!(surface, Surface::Part);
        assert!(distance.is_finite() && distance > 0.0);
        let image = render(&scene, req, 16);
        let [r, g, _] = pixel(&image, WIDTH / 2, HEIGHT / 2);
        assert!(
            f64::from(g) > f64::from(r) * 1.3,
            "the centre pixel shows the part: red {r}, green {g}"
        );
    }

    /// Far from the part, a corner of the image sees the floor or the sky, and never the part.
    #[test]
    fn a_corner_past_the_part_sees_the_ground_or_the_sky() {
        let scene = scene_of(&Shape::Sphere.build(10.0, 48).expect("the sphere builds"));
        let mut req = request(Mode::PathTraced, 1);
        req.zoom = 0.4;
        let camera = camera_of(&scene, &req);
        for (px, py) in [
            (1.0, 1.0),
            (f64::from(WIDTH) - 1.0, 1.0),
            (1.0, 89.0),
            (119.0, 89.0),
        ] {
            let (surface, _) = scene.surface_at(&camera, px, py);
            assert_ne!(surface, Surface::Part, "corner ({px}, {py})");
        }
    }

    /// A point on the floor under the part, away from the sun, is in the part's shadow and comes
    /// out darker than an open patch of floor on the sunny side.
    #[test]
    fn the_floor_in_the_parts_shadow_is_darker_than_in_the_open() {
        let scene = torus_scene();
        let req = request(Mode::PathTraced, 1);
        let camera = camera_of(&scene, &req);
        let sun = sun_direction();
        let drop = (scene.center.z - scene.ground_z) / sun.z;
        let shadow_spot = Vector3::new(
            scene.center.x - sun.x * drop + scene.radius * 0.71,
            scene.center.y - sun.y * drop,
            scene.ground_z,
        );
        let open_spot =
            shadow_spot + Vector3::new(sun.x, sun.y, 0.0).get_normal() * scene.radius * 2.0;
        let (sx, sy) = camera.project(shadow_spot).expect("the shadow is in view");
        let (ox, oy) = camera
            .project(open_spot)
            .expect("the open floor is in view");
        for (px, py) in [(sx, sy), (ox, oy)] {
            assert_eq!(scene.surface_at(&camera, px, py).0, Surface::Ground);
        }

        let image = render(&scene, req, 48);

        let shadowed = brightness(pixel(&image, sx as u32, sy as u32));
        let open = brightness(pixel(&image, ox as u32, oy as u32));
        assert!(shadowed < open, "shadow {shadowed} vs open {open}");
    }

    /// A seed always gives the same image, bit for bit, whatever the time budget does: a
    /// different seed does not, and slicing a sample across many passes changes nothing.
    #[test]
    fn the_same_seed_gives_identical_buffers_however_the_passes_are_sliced() {
        let scene = torus_scene();
        let mut req = request(Mode::PathTraced, 7);
        req.look = look(Material::GlossyPlastic, TEAL, Light::Studio);

        let whole = render(&scene, req, 3);
        assert_eq!(whole.surface(), render(&scene, req, 3).surface());

        let mut sliced = Accumulation::new();
        let mut now = 0.0;
        let mut clock = || {
            now += 1000.0;
            now
        };
        while sliced.samples() < 3 {
            sliced
                .pass(&scene, req, 1.0, &mut clock)
                .expect("the pass runs");
        }
        assert_eq!(whole.surface(), sliced.surface(), "sliced differently");
        assert_eq!(whole.pixels(), sliced.pixels(), "sliced differently");

        let mut other = req;
        other.seed = 8;
        assert_ne!(whole.pixels(), render(&scene, other, 3).pixels());
    }

    /// Changing the material on a prepared part gives a different image, starting its samples
    /// over, and needs nothing rebuilt: the part's trees and tables are held in the scene behind
    /// a shared reference, and the second image comes from the very same scene.
    #[test]
    fn a_new_material_refines_from_scratch_on_the_same_prepared_part() {
        let scene = torus_scene();
        let mut req = request(Mode::PathTraced, 2);
        let triangles = scene.triangles;
        let mut image = Accumulation::new();
        for _ in 0..4 {
            image
                .pass(&scene, req, 0.0, &mut || 0.0)
                .expect("the pass runs");
        }
        assert_eq!(image.samples(), 4);
        let matte_image = image.pixels().to_vec();

        req.look = look(Material::Rubber, [200, 60, 40], Light::Overcast);
        for _ in 0..4 {
            image
                .pass(&scene, req, 0.0, &mut || 0.0)
                .expect("the pass runs");
        }

        assert_eq!(image.samples(), 4, "refining starts again from scratch");
        assert_eq!(scene.triangles, triangles);
        assert_ne!(
            matte_image,
            image.pixels(),
            "the material reaches the image"
        );
    }

    /// A polished white metal is a mirror of the sky, so the part of the ball facing the camera
    /// shows the sky seen straight back along the view, within what the small roughness blurs;
    /// white matte plastic in the same place is lit by the warm sun and does not.
    #[test]
    fn polished_metal_mirrors_the_sky_where_matte_plastic_does_not() {
        let scene = sphere_scene();
        let mut req = request(Mode::PathTraced, 3);
        // From the sunny side, so the matte plastic below is lit by the sun.
        req.yaw = -2.2;
        req.look = look(Material::PolishedMetal, WHITE, Light::Studio);
        let camera = camera_of(&scene, &req);
        let (cx, cy) = (WIDTH / 2, HEIGHT / 2);
        let toward_eye = (camera.eye - scene.center).get_normal();

        let metal = render(&scene, req, 24);
        let linear = metal.linear_at(cx, cy);

        let lights = req.look.lights().expect("the lights build");
        let sky = lights.environment.radiance(toward_eye);
        for channel in 0..3 {
            assert!(
                (linear[channel] - sky[channel]).abs() < sky[channel] * 0.15,
                "channel {channel}: mirror shows {:?}, the sky there is {:?}",
                linear,
                sky
            );
        }
        // The sky's blue over red is what a mirror shows; the sun warms a white matte surface.
        assert!(
            linear[2] > linear[0] * 1.2,
            "blue sky reflected: {linear:?}"
        );

        req.look = look(Material::MattePlastic, WHITE, Light::Studio);
        let plastic = render(&scene, req, 24).linear_at(cx, cy);
        // The sun lights the matte surface far above the sky alone, and its blue no longer leads
        // the way the mirrored sky's does.
        assert!(
            plastic[0] > linear[0] * 1.5 && plastic[2] / plastic[0] < (linear[2] / linear[0]) * 0.9,
            "matte plastic is lit by the sun, not a mirror of the sky: {plastic:?} vs {linear:?}"
        );
    }

    /// Clear glass shows the floor through the part: a glass cube seen from above has the
    /// floor's light and dark squares in its top face, each where it lies under the cube; the
    /// same pixels of a matte cube are one flat color.
    #[test]
    fn glass_shows_the_floor_through_the_part() {
        let scene = scene_of(&Shape::Cube.build(40.0, 8).expect("the cube builds"));
        let mut req = request(Mode::PathTraced, 4);
        req.width = 140;
        req.height = 140;
        req.pitch = 1.5;
        req.zoom = 1.6;
        let camera = camera_of(&scene, &req);
        let square = scene.floor_square();

        // Pixels on the part whose straight line down to the floor stays well inside one square.
        let mut light_pixels = Vec::new();
        let mut dark_pixels = Vec::new();
        for y in 0..req.height {
            for x in 0..req.width {
                let (px, py) = (f64::from(x) + 0.5, f64::from(y) + 0.5);
                let ray = camera.ray(px, py);
                if scene.surface_at(&camera, px, py).0 != Surface::Part {
                    continue;
                }
                let to_floor = (scene.ground_z - ray.origin.z) / ray.direction_normal.z;
                let floor = ray.origin + ray.direction_normal * to_floor;
                let margin = |value: f64| {
                    let across = (value / square).rem_euclid(1.0);
                    across.min(1.0 - across) * square
                };
                if margin(floor.x) < square * 0.2 || margin(floor.y) < square * 0.2 {
                    continue;
                }
                if scene.floor_is_light(floor.x, floor.y) {
                    light_pixels.push((x, y));
                } else {
                    dark_pixels.push((x, y));
                }
            }
        }
        assert!(
            light_pixels.len() > 40 && dark_pixels.len() > 40,
            "the cube covers both kinds of square: {} light, {} dark",
            light_pixels.len(),
            dark_pixels.len()
        );

        let mean = |image: &Accumulation, pixels: &[(u32, u32)]| {
            let sum: f64 = pixels
                .iter()
                .map(|&(x, y)| {
                    let c = image.linear_at(x, y);
                    c[0] + c[1] + c[2]
                })
                .sum();
            sum / pixels.len() as f64
        };

        req.look = look(Material::Glass, WHITE, Light::Overcast);
        let glass = render(&scene, req, 16);
        let (light, dark) = (mean(&glass, &light_pixels), mean(&glass, &dark_pixels));
        assert!(
            light > dark * 1.12,
            "the floor's squares show through the glass: light {light}, dark {dark}"
        );

        req.look = look(Material::MattePlastic, WHITE, Light::Overcast);
        let matte = render(&scene, req, 16);
        let (light, dark) = (mean(&matte, &light_pixels), mean(&matte, &dark_pixels));
        assert!(
            light < dark * 1.03 && dark < light * 1.03,
            "the matte top is one color: light {light}, dark {dark}"
        );
    }

    /// The moving image's paths are short, so the same view costs fewer rays while dragging.
    #[test]
    fn the_preview_fires_fewer_rays_than_refinement() {
        let scene = torus_scene();
        let mut req = request(Mode::PathTraced, 5);
        req.look = look(Material::Glass, WHITE, Light::Studio);
        let refine = Accumulation::new()
            .pass(&scene, req, 0.0, &mut || 0.0)
            .unwrap();
        req.preview = true;
        let preview = Accumulation::new()
            .pass(&scene, req, 0.0, &mut || 0.0)
            .unwrap();
        assert!(
            preview.rays < refine.rays,
            "preview {} vs refinement {}",
            preview.rays,
            refine.rays
        );
    }

    /// Normals mode shows the actual face normal: straight up on a cube's top is (0, 0, 1).
    #[test]
    fn normals_mode_shows_a_cubes_top_as_straight_up() {
        let scene = scene_of(&Shape::Cube.build(40.0, 8).expect("the cube builds"));
        let mut req = request(Mode::Normals, 1);
        req.pitch = 1.1;
        let camera = camera_of(&scene, &req);
        let top = Vector3::new(scene.center.x, scene.center.y, scene.center.z + 20.0);
        let (px, py) = camera.project(top).expect("the top is in view");
        assert_eq!(scene.surface_at(&camera, px, py).0, Surface::Part);

        let image = render(&scene, req, 1);

        // (n + 1) / 2 for (0, 0, 1), rounded to bytes.
        assert_eq!(pixel(&image, px as u32, py as u32), [128, 128, 255]);
    }

    /// The normal view is the smooth shading normal the path tracer uses, and on a coarse ball
    /// that is measurably not the flat normal of the facet the ray hit.
    #[test]
    fn normals_mode_shows_the_smooth_normal_on_a_coarse_ball() {
        let scene = scene_of(&Shape::Sphere.build(40.0, 12).expect("the sphere builds"));
        let req = request(Mode::Normals, 1);
        let camera = camera_of(&scene, &req);
        let image = render(&scene, req, 1);
        let mut different_from_flat = 0;
        for y in 20..70 {
            for x in 30..90 {
                let (px, py) = (f64::from(x) + 0.5, f64::from(y) + 0.5);
                let mut ray = camera.ray(px, py);
                let Some(hit) = scene.hit(&mut ray) else {
                    continue;
                };
                if hit.part.index() != 0 {
                    continue;
                }
                let point = hit.surface_point().expect("the surface point");
                let shown = pixel(&image, x, y);
                for (channel, value) in [
                    point.shading_normal.x,
                    point.shading_normal.y,
                    point.shading_normal.z,
                ]
                .into_iter()
                .enumerate()
                {
                    let expected = ((value + 1.0) * 0.5 * 255.0).round() as i32;
                    assert!(
                        (i32::from(shown[channel]) - expected).abs() <= 2,
                        "pixel ({x}, {y}) channel {channel}: shows {} expected {expected}",
                        shown[channel]
                    );
                }
                if (point.shading_normal - point.geometric_normal).length() > 0.03 {
                    different_from_flat += 1;
                }
            }
        }
        assert!(
            different_from_flat > 100,
            "only {different_from_flat} pixels differ from the flat normal"
        );
    }

    /// The page's starting view of `scene` at `width` x `height`.
    fn starting_camera(scene: &TraceScene, width: u32, height: u32) -> Camera {
        Camera::new(scene, 0.85, 0.42, 1.0, width, height).expect("the camera builds")
    }

    /// The angle in degrees between the rays through two pixel positions.
    fn degrees_between(camera: &Camera, a: (f64, f64), b: (f64, f64)) -> f64 {
        let (a, b) = (
            camera.ray_direction(a.0, a.1),
            camera.ray_direction(b.0, b.1),
        );
        a.dot(b).clamp(-1.0, 1.0).acos().to_degrees()
    }

    /// A phone holds the canvas taller than wide: the whole part still has to fit, so no pixel
    /// on the image's border may land on it, while the middle of the image does.
    #[test]
    fn a_portrait_image_keeps_the_whole_part_inside_the_frame() {
        let mesh = Part::Boolean.build(64).expect("the part builds");
        let scene = scene_of(&mesh);
        let (width, height) = (300u32, 600u32);
        let camera = starting_camera(&scene, width, height);
        let at = |x: u32, y: u32| {
            scene
                .surface_at(&camera, f64::from(x) + 0.5, f64::from(y) + 0.5)
                .0
        };

        for x in 0..width {
            assert_ne!(at(x, 0), Surface::Part, "top edge, x {x}");
            assert_ne!(at(x, height - 1), Surface::Part, "bottom edge, x {x}");
        }
        for y in 0..height {
            assert_ne!(at(0, y), Surface::Part, "left edge, y {y}");
            assert_ne!(at(width - 1, y), Surface::Part, "right edge, y {y}");
        }
        assert_eq!(
            at(width / 2, height / 2),
            Surface::Part,
            "the part is in view"
        );
    }

    /// The 38 degree field of view spans the shorter side: half of it from the centre to the
    /// top in a landscape image, and to the side in a portrait one.
    #[test]
    fn the_field_of_view_spans_the_images_shorter_side() {
        let scene = torus_scene();
        let half_view = 19.0;

        let landscape = starting_camera(&scene, 400, 200);
        let up = degrees_between(&landscape, (200.0, 100.0), (200.0, 0.0));
        assert!((up - half_view).abs() < 1e-9, "landscape top at {up}");

        let portrait = starting_camera(&scene, 200, 400);
        let side = degrees_between(&portrait, (100.0, 200.0), (0.0, 200.0));
        assert!((side - half_view).abs() < 1e-9, "portrait side at {side}");
    }

    /// Landscape framing depends only on the height: widening an image shows more at the sides
    /// and moves nothing, down to the last bit of each ray.
    #[test]
    fn widening_a_landscape_image_leaves_its_rays_exactly_where_they_were() {
        let scene = torus_scene();
        let square = starting_camera(&scene, 300, 300);
        let wide = starting_camera(&scene, 1200, 300);

        // Straight up the middle, where the width cannot enter.
        for py in [0.0, 37.5, 150.0, 299.0] {
            assert_eq!(
                square.ray_direction(150.0, py),
                wide.ray_direction(600.0, py),
                "column through the middle, y {py}"
            );
        }
        // A quarter of the way out in the 4:1 image is the square image's right edge.
        assert_eq!(
            square.ray_direction(300.0, 150.0),
            wide.ray_direction(750.0, 150.0),
            "the same angle to the right"
        );
    }

    /// Generating a ray and projecting a point are inverses at any shape of image, which is what
    /// makes a projected point land on the pixel whose ray reaches it.
    #[test]
    fn projecting_a_point_on_a_pixels_ray_returns_that_pixel() {
        let scene = torus_scene();
        for (width, height) in [(120u32, 90u32), (300, 600), (256, 256)] {
            let camera = starting_camera(&scene, width, height);
            for (px, py) in [(0.5, 0.5), (17.25, 80.5), (100.75, 3.5), (60.0, 45.0)] {
                let point = camera.eye + camera.ray_direction(px, py) * 87.5;
                let (x, y) = camera.project(point).expect("in front of the camera");
                assert!(
                    (x - px).abs() < 1e-9 && (y - py).abs() < 1e-9,
                    "{width} x {height}: ({px}, {py}) came back as ({x}, {y})"
                );
            }
        }
    }

    /// Depth mode reports the actual hit distance: the centre of a sphere's face is
    /// the eye's distance to the centre, less the radius.
    #[test]
    fn depth_mode_reads_the_hit_distance_on_a_sphere() {
        let radius = 20.0;
        let scene = scene_of(
            &Shape::Sphere
                .build(radius * 2.0, 96)
                .expect("the sphere builds"),
        );
        let req = request(Mode::Depth, 1);
        let camera = camera_of(&scene, &req);
        let centre = (f64::from(WIDTH) / 2.0, f64::from(HEIGHT) / 2.0);
        let eye_distance = (scene.center - camera.eye).length();

        let (surface, hit) = scene.surface_at(&camera, centre.0, centre.1);
        assert_eq!(surface, Surface::Part);
        assert!(
            (hit - (eye_distance - radius)).abs() < radius * 0.02,
            "hit at {hit}, expected about {}",
            eye_distance - radius
        );

        let image = render(&scene, req, 1);

        let (near, far) = depth_range(eye_distance, scene.radius);
        let expected = (depth_value(hit, near, far) * 255.0).round() as i32;
        let shown = i32::from(pixel(&image, WIDTH / 2, HEIGHT / 2)[0]);
        assert!(
            (shown - expected).abs() <= 2,
            "shown {shown}, expected {expected}"
        );
    }
}

// The Fillet & Chamfer page: a pick made the way the page makes it (a ray at the part, through
// `PickSession::pick`) and built the way the page builds it (`fillet_part`, every pick at one
// radius in one `bevel_part` call).
mod fillet {
    use solidmesh::math::Vector3;
    use solidmesh::mesh::CancelToken;
    use solidmesh_wasm::fillet::{fillet_part, FilletBuild};
    use solidmesh_wasm::fillet_parts::part;
    use solidmesh_wasm::fillet_pick::{Pick, PickMode, PickSession};

    /// The mm a pixel spans at 1 mm from the eye in the page's 40 degree view, about 700 px tall.
    const PIXEL_SIZE: f64 = 0.001;

    /// What a click at `target`, seen from `target + from`, picks in `mode`.
    fn pick_at(session: &PickSession, target: Vector3, from: Vector3, mode: PickMode) -> Pick {
        let origin = target + from;
        session
            .pick(origin, target - origin, mode, PIXEL_SIZE, true)
            .expect("the pick runs")
            .expect("the click lands on a pick")
    }

    /// The L-bracket's concave edge inside the L, picked as a chain from above the base.
    fn inside_edge(session: &PickSession) -> Pick {
        pick_at(
            session,
            Vector3::new(-16.9, 0.0, 8.0),
            Vector3::new(20.0, 0.0, 20.0),
            PickMode::Chain,
        )
    }

    fn build(index: u32, picks: Vec<Pick>, size: f64) -> FilletBuild {
        let source = part(index).expect("the part builds");
        let groups: Vec<_> = picks.into_iter().map(|pick| pick.group).collect();
        fillet_part(&source, &groups, size, true, 6, &CancelToken::new()).expect("the build runs")
    }

    fn assert_built(build: &FilletBuild, what: &str) {
        assert!(
            build.scene.mesh.is_closed_consistently_wound(),
            "{what}: not closed; {}",
            build.scene.note
        );
        for row in &build.rows {
            assert!(row.built && row.code.is_none(), "{what}: {row:?}");
        }
    }

    /// The L-bracket's concave edge inside the L, picked as a chain from above the base, rounds.
    #[test]
    fn the_l_brackets_inside_edge_is_picked_and_filleted() {
        let session = PickSession::new(&part(0).expect("the bracket builds"));
        let pick = inside_edge(&session);
        assert_eq!(1, pick.segments.len(), "{}", pick.label);
        let source_volume = part(0).expect("the bracket builds").get_volume();

        let build = build(0, vec![pick], 4.0);

        assert_built(&build, "inside edge");
        // a concave round adds material
        assert!(build.scene.mesh.get_volume() > source_volume);
    }

    /// The plate's pocket rim, rounded corners and all, is one chain loop and rounds whole.
    #[test]
    fn the_plates_pocket_rim_is_picked_as_one_loop_and_filleted() {
        let session = PickSession::new(&part(1).expect("the plate builds"));
        let pick = pick_at(
            &session,
            Vector3::new(18.1, 0.0, 10.0),
            Vector3::new(15.0, 0.0, 15.0),
            PickMode::Chain,
        );
        assert_eq!(36, pick.segments.len(), "{}", pick.label);

        let build = build(1, vec![pick], 1.5);

        assert_built(&build, "pocket rim");
    }

    /// The L-bracket's three starting picks - the edge inside the L, the top of the upright and
    /// the base's far edge - all round at the page's starting radius, 3 mm, in the one call.
    #[test]
    fn several_picks_at_one_radius_build_in_one_call() {
        let session = PickSession::new(&part(0).expect("the bracket builds"));
        let from = Vector3::new(20.0, 0.0, 20.0);
        let picks = vec![
            inside_edge(&session),
            pick_at(
                &session,
                Vector3::new(-17.1, 0.0, 40.0),
                from,
                PickMode::Chain,
            ),
            pick_at(
                &session,
                Vector3::new(24.9, 0.0, 8.0),
                from,
                PickMode::Chain,
            ),
        ];
        let source_faces = part(0).expect("the bracket builds").faces.len();

        let build = build(0, picks, 3.0);

        assert_built(&build, "three picks");
        assert_eq!(3, build.rows.len());
        assert!(build
            .scene
            .note
            .contains("3 of 3 picks built, in one bevel_part call"));
        assert!(build.scene.mesh.faces.len() > source_faces);
    }

    /// A radius wider than the face the round lands on is refused with its code and the size
    /// that does fit, and the part comes back unchanged and closed.
    #[test]
    fn an_oversized_radius_is_refused_with_the_size_that_fits() {
        let session = PickSession::new(&part(0).expect("the bracket builds"));

        let build = build(0, vec![inside_edge(&session)], 50.0);

        let row = &build.rows[0];
        assert!(!row.built, "{row:?}");
        assert_eq!(Some("SizeExceedsFaceWidth"), row.code.as_deref(), "{row:?}");
        let fits = row.fits.expect("a size that fits");
        assert!(fits > 0.0 && fits < 50.0, "{row:?}");
        assert!(row.text.contains("most that fits"), "{}", row.text);
        assert!(build.scene.mesh.is_closed_consistently_wound());
    }
}
