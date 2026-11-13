//! What `SunThroughGlass::Straight` keeps and leaves out, each against `SunThroughGlass::Exact`,
//! the plain path tracer, or a closed form: a polished floor under a pane keeps its glint and is
//! no noisier for it; frosted glass passes the sun as smooth glass does, with no step at the
//! smooth threshold; light bouncing inside a pane leaves no specks on the floor under it; the
//! sun's reflection off a glass wall is left off the floor; the camera looking at the sun through
//! a pane sees what Exact shows, bit for bit; and two different glasses touching pass the sun
//! through their one interface.

use std::f64::consts::PI;
use std::sync::Arc;

use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::ray_tracer::{
    LightSetup, MeshSurface, PathTraceSettings, PathTracer, PlacedPart, PlacedParts, RenderCamera,
    RenderScene, RenderWork, ScatteringSurface, Sun, SunThroughGlass,
};
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::Mesh;

use crate::path_tracer_support::{agreement_tolerance, estimate, flat_patch, SHARP_CREASE};
use crate::scattering_support::Stats;
use crate::surface_hit_support::tree_of;

pub(crate) const IOR: f64 = 1.5;
pub(crate) const ALBEDO: f64 = 0.3;

pub(crate) fn matte() -> ScatteringSurface {
    ScatteringSurface::new([ALBEDO; 3], 0.0, 1.0, 0.0, 1.0)
}

pub(crate) fn glass(roughness: f64, ior: f64) -> ScatteringSurface {
    ScatteringSurface::new([1.0; 3], 0.0, roughness, 1.0, ior)
}

fn part(mesh: &Mesh, matrix: Matrix4X4) -> PlacedPart {
    PlacedPart::new(
        tree_of(mesh),
        Arc::new(MeshSurface::new(mesh, SHARP_CREASE).unwrap()),
        matrix,
    )
    .unwrap()
}

pub(crate) fn scene(parts: Vec<(Mesh, Matrix4X4, ScatteringSurface)>, sun: Sun) -> RenderScene {
    let (placed, surfaces): (Vec<_>, Vec<_>) = parts
        .into_iter()
        .map(|(mesh, matrix, surface)| (part(&mesh, matrix), surface))
        .unzip();
    RenderScene::new(
        PlacedParts::new(placed),
        surfaces,
        LightSetup::sun_only(sun),
    )
    .unwrap()
}

/// The Fresnel reflectance of unpolarized light meeting glass of index `ior` from air at `cos_i`.
pub(crate) fn fresnel(cos_i: f64, ior: f64) -> f64 {
    let sin2_t = (1.0 - cos_i * cos_i) / (ior * ior);
    let cos_t = (1.0 - sin2_t).sqrt();
    let perpendicular = (cos_i - ior * cos_t) / (cos_i + ior * cos_t);
    let parallel = (ior * cos_i - cos_t) / (ior * cos_i + cos_t);
    0.5 * (perpendicular * perpendicular + parallel * parallel)
}

/// A sun `from_zenith` radians off straight up toward `toward_x` (+1 or -1) of `radius`, its
/// radiance giving the unshaded floor an irradiance of about pi.
pub(crate) fn sun(from_zenith: f64, toward_x: f64, radius: f64) -> Sun {
    let direction = Vector3::new(toward_x * from_zenith.sin(), 0.0, from_zenith.cos());
    let radiance = 1.0 / (radius.sin() * radius.sin());
    Sun::new(direction, radius, [radiance; 3]).unwrap()
}

/// The floor at z = 0 and a pane of `pane` from z = 1 to z = 2, far wider than the patch seen.
pub(crate) fn pane_scene(
    floor: ScatteringSurface,
    pane: ScatteringSurface,
    sun: Sun,
) -> RenderScene {
    scene(
        vec![
            (flat_patch(1000.0, 0.0), Matrix4X4::IDENTITY, floor),
            (
                create_cube(2000.0, 2000.0, 1.0),
                Matrix4X4::create_translation(0.0, 0.0, 1.5),
                pane,
            ),
        ],
        sun,
    )
}

/// Between the floor and the pane at `x`, looking straight down.
pub(crate) fn floor_camera(x: f64) -> RenderCamera {
    RenderCamera::orthographic(
        Vector3::new(x, 0.0, 0.5),
        Vector3::new(x, 0.0, 0.0),
        Vector3::UNIT_Y,
        2.0,
    )
    .unwrap()
}

pub(crate) fn with(
    through: SunThroughGlass,
    change: impl FnOnce(&mut PathTraceSettings),
) -> PathTraceSettings {
    let mut settings = PathTraceSettings {
        sun_through_glass: through,
        ..PathTraceSettings::default()
    };
    change(&mut settings);
    settings
}

pub(crate) fn straight() -> PathTraceSettings {
    with(SunThroughGlass::Straight, |_| {})
}

pub(crate) fn exact() -> PathTraceSettings {
    with(SunThroughGlass::Exact, |_| {})
}

#[test]
fn a_polished_floor_under_a_pane_keeps_its_glint_without_sparkle() {
    // The view's mirror direction off the floor is straight up, into the sun: the glint the
    // polished metal shows, seen through nothing, lit through the pane.
    let sun = sun(0.0, 1.0, 2.5_f64.to_radians());
    let scene = pane_scene(
        ScatteringSurface::polished_metal([0.9; 3]),
        glass(0.0, IOR),
        sun,
    );
    let straight = estimate(&scene, floor_camera(0.0), (4, 4), straight(), 1024).0[0];
    let exact = estimate(&scene, floor_camera(0.0), (4, 4), exact(), 1024).0[0];
    println!(
        "polished floor: straight {} +- {}, exact {} +- {}",
        straight.mean(),
        straight.standard_error(),
        exact.mean(),
        exact.standard_error()
    );
    // The two differ only by what Straight leaves out: the pane's inside reflections, 0.16% of
    // the light through it with the sun overhead.
    let allowed = agreement_tolerance(&straight, &exact) + 0.005 * exact.mean();
    assert!(
        (straight.mean() - exact.mean()).abs() <= allowed,
        "the glint under the pane is {} going straight through and {} exactly (allowed {allowed})",
        straight.mean(),
        exact.mean()
    );
    // Both find the glint by the bounce that lands in the sun's disc; the shadow ray alone would
    // find it rarely and brightly.
    assert!(
        straight.standard_error() <= 1.5 * exact.standard_error(),
        "the glint under the pane is {} +- {} going straight through, noisier than exactly's {} \
         +- {}",
        straight.mean(),
        straight.standard_error(),
        exact.mean(),
        exact.standard_error()
    );
}

#[test]
fn frosted_glass_passes_the_sun_as_smooth_glass_does() {
    let sun = sun(30.0_f64.to_radians(), 1.0, 2.5_f64.to_radians());
    // The smooth threshold is a roughness of about 0.0316; each step either side of it.
    let roughnesses = [0.0, 0.02, 0.031, 0.032, 0.05, 0.1, 0.3];
    // The floor's own sun, through the pane: the same for every roughness, to the bit.
    let direct = PathTraceSettings {
        max_bounces: 1,
        russian_roulette_after: None,
        ..straight()
    };
    let smooth = pane_scene(matte(), glass(0.0, IOR), sun);
    let reference = estimate(&smooth, floor_camera(0.0), (4, 4), direct, 64).0[0].mean();
    assert!(
        reference > 0.0,
        "no sun reaches the floor under a clear pane"
    );
    for roughness in roughnesses {
        let frosted = pane_scene(matte(), glass(roughness, IOR), sun);
        let value = estimate(&frosted, floor_camera(0.0), (4, 4), direct, 64).0[0].mean();
        println!("frosted {roughness}: direct {value}, smooth {reference}");
        assert_eq!(
            value, reference,
            "the sun through a pane at roughness {roughness} differs from a smooth one's"
        );
    }
    // Every bounce: the floor changes only as the pane's reflections of it change, smoothly.
    let mut previous: Option<(f64, Stats)> = None;
    for roughness in roughnesses {
        let frosted = pane_scene(matte(), glass(roughness, IOR), sun);
        let stats = estimate(&frosted, floor_camera(0.0), (4, 4), straight(), 512).0[0];
        println!(
            "frosted {roughness}: {} +- {}",
            stats.mean(),
            stats.standard_error()
        );
        if let Some((last, last_stats)) = previous {
            let allowed = agreement_tolerance(&stats, &last_stats) + 0.005 * stats.mean();
            assert!(
                (stats.mean() - last_stats.mean()).abs() <= allowed,
                "the floor under a pane steps from {} at roughness {last} to {} at {roughness} \
                 (allowed {allowed})",
                last_stats.mean(),
                stats.mean()
            );
        }
        previous = Some((roughness, stats));
    }
}

/// Every sample's luminance of a `size` image of `scene`, `samples` a pixel.
pub(crate) fn sample_values(
    scene: &RenderScene,
    camera: RenderCamera,
    settings: PathTraceSettings,
    samples: u32,
) -> Vec<f64> {
    let tracer = PathTracer::new(scene, camera, 4, 4, settings).unwrap();
    let mut work = RenderWork::default();
    let mut values = Vec::new();
    for y in 0..4 {
        for x in 0..4 {
            for sample in 0..samples {
                let c = tracer.sample_pixel(x, y, sample, &mut work).unwrap();
                values.push(0.2126 * c[0] + 0.7152 * c[1] + 0.0722 * c[2]);
            }
        }
    }
    values
}

#[test]
fn light_bouncing_inside_a_pane_leaves_no_specks_on_the_floor() {
    let scene = pane_scene(
        matte(),
        glass(0.0, IOR),
        sun(60.0_f64.to_radians(), 1.0, 10.0_f64.to_radians()),
    );
    // The floor in the open would read about 0.15; a speck is the sun itself, some 60 times that,
    // carried by a path that went through the pane into the sun's disc, after reflecting inside
    // the pane or not. A large sun makes the reflected ones common enough to count.
    let threshold = 0.5;
    let count = |settings| {
        sample_values(&scene, floor_camera(0.0), settings, 16384)
            .into_iter()
            .filter(|&value| value > threshold)
            .count()
    };
    // Roulette off: a survivor's reweighting makes the odd sample a few times the floor's,
    // which is not what is counted here.
    let no_roulette = |through| with(through, |s| s.russian_roulette_after = None);
    let (straight, exact) = (
        count(no_roulette(SunThroughGlass::Straight)),
        count(no_roulette(SunThroughGlass::Exact)),
    );
    println!("samples over {threshold}: straight {straight}, exact {exact}");
    assert!(exact > 0, "the exact render shows no specks to leave out");
    assert_eq!(straight, 0, "the floor under a pane has {straight} specks");
}

/// The irradiance a floor facing up gets from a disc of `radius` around `axis` of `radiance`,
/// each direction passed by `passed(direction)`. Midpoint rule in the disc's polar coordinates.
pub(crate) fn disc_irradiance(
    axis: Vector3,
    radius: f64,
    radiance: f64,
    passed: impl Fn(Vector3) -> f64,
) -> f64 {
    let tangent = Vector3::new(axis.z, 0.0, -axis.x);
    let (rings, turns) = (200, 200);
    let mut sum = 0.0;
    for ring in 0..rings {
        let alpha = (f64::from(ring) + 0.5) * radius / f64::from(rings);
        for turn in 0..turns {
            let phi = (f64::from(turn) + 0.5) * 2.0 * PI / f64::from(turns);
            let direction = axis * alpha.cos()
                + (tangent * phi.cos() + Vector3::UNIT_Y * phi.sin()) * alpha.sin();
            sum += passed(direction) * direction.z * alpha.sin();
        }
    }
    radiance * sum * (radius / f64::from(rings)) * (2.0 * PI / f64::from(turns))
}

#[test]
fn the_suns_reflection_off_a_glass_wall_is_left_off_the_floor() {
    // A glass wall 40 high from x = 4.5 to 5.5, the sun low behind the camera's patch at
    // x = 2 to 4: the wall mirrors it onto the patch, from 30 degrees off the wall's normal.
    let (from_zenith, radius) = (60.0_f64.to_radians(), 10.0_f64.to_radians());
    let sun = sun(from_zenith, -1.0, radius);
    let wall = scene(
        vec![
            (flat_patch(1000.0, 0.0), Matrix4X4::IDENTITY, matte()),
            (
                create_cube(1.0, 2000.0, 40.0),
                Matrix4X4::create_translation(5.0, 0.0, 20.0),
                glass(0.0, IOR),
            ),
        ],
        sun,
    );
    let camera = floor_camera(3.0);
    let direct_only = |through| {
        with(through, |s| {
            s.max_bounces = 2;
            s.russian_roulette_after = None;
        })
    };
    let straight = estimate(
        &wall,
        camera,
        (4, 4),
        direct_only(SunThroughGlass::Straight),
        256,
    )
    .0[0];
    let exact = estimate(
        &wall,
        camera,
        (4, 4),
        direct_only(SunThroughGlass::Exact),
        16384,
    )
    .0[0];
    let radiance = 1.0 / (radius.sin() * radius.sin());
    let axis = Vector3::new(-from_zenith.sin(), 0.0, from_zenith.cos());
    let open = ALBEDO / PI * disc_irradiance(axis, radius, radiance, |_| 1.0);
    // The mirrored sun, through the wall's front face only: one reflection off it, what two
    // bounces reach.
    let mirrored = Vector3::new(-axis.x, 0.0, axis.z);
    let reflected = ALBEDO / PI
        * disc_irradiance(mirrored, radius, radiance, |direction| {
            fresnel(direction.x.abs(), IOR)
        });
    println!(
        "wall: straight {} +- {}, exact {} +- {}, open {open}, reflected {reflected}",
        straight.mean(),
        straight.standard_error(),
        exact.mean(),
        exact.standard_error()
    );
    // Absent, not merely averaging out: the floor going straight through is as smooth as the
    // open floor's shadow ray makes it.
    assert!(
        straight.standard_error() < 0.005 * open,
        "the floor by a glass wall is {} +- {} going straight through: the wall's reflection of \
         the sun is on it as specks",
        straight.mean(),
        straight.standard_error()
    );
    let allowed = 5.0 * straight.standard_error() + 1e-3 * open;
    assert!(
        (straight.mean() - open).abs() <= allowed,
        "the floor by a glass wall is {} going straight through, where the open floor is {open}: \
         the wall's reflection of the sun is on it (allowed {allowed})",
        straight.mean()
    );
    let allowed = 5.0 * exact.standard_error() + 1e-3 * open;
    assert!(
        (exact.mean() - (open + reflected)).abs() <= allowed,
        "the floor by a glass wall is {} exactly, where the open floor and the wall's reflection \
         are {} (allowed {allowed})",
        exact.mean(),
        open + reflected
    );
}

#[test]
fn the_camera_sees_the_sun_through_a_pane_as_exact_shows_it() {
    let sun = sun(0.0, 1.0, 2.5_f64.to_radians());
    let pane = scene(
        vec![(
            create_cube(2000.0, 2000.0, 1.0),
            Matrix4X4::create_translation(0.0, 0.0, 1.5),
            glass(0.0, IOR),
        )],
        sun,
    );
    // Below the pane looking straight up into the sun.
    let camera =
        RenderCamera::orthographic(Vector3::ZERO, Vector3::UNIT_Z, Vector3::UNIT_Y, 2.0).unwrap();
    let straight = estimate(&pane, camera, (4, 4), straight(), 64).0;
    let exact = estimate(&pane, camera, (4, 4), exact(), 64).0;
    println!(
        "sun through a pane: straight {}, exact {}",
        straight[0].mean(),
        exact[0].mean()
    );
    assert!(
        straight[0].mean() > 0.0,
        "the camera sees no sun through the pane"
    );
    for channel in 0..3 {
        assert_eq!(straight[channel].mean(), exact[channel].mean());
        assert_eq!(
            straight[channel].standard_error(),
            exact[channel].standard_error()
        );
    }
}

#[test]
fn two_different_glasses_touching_pass_the_sun_through_their_one_interface() {
    let radius = 2.5_f64.to_radians();
    let sun = sun(0.0, 1.0, radius);
    let (lower, upper) = (1.5, 2.0);
    // The two slabs lie on each other at z = 2: one interface of the relative index there, no
    // air between them.
    let touching = scene(
        vec![
            (flat_patch(1000.0, 0.0), Matrix4X4::IDENTITY, matte()),
            (
                create_cube(2000.0, 2000.0, 1.0),
                Matrix4X4::create_translation(0.0, 0.0, 1.5),
                glass(0.0, lower),
            ),
            (
                create_cube(2000.0, 2000.0, 1.0),
                Matrix4X4::create_translation(0.0, 0.0, 2.5),
                glass(0.0, upper),
            ),
        ],
        sun,
    );
    let direct = PathTraceSettings {
        max_bounces: 1,
        russian_roulette_after: None,
        ..straight()
    };
    let stats = estimate(&touching, floor_camera(0.0), (4, 4), direct, 64).0[0];
    let radiance = 1.0 / (radius.sin() * radius.sin());
    let expected = ALBEDO / PI
        * disc_irradiance(Vector3::UNIT_Z, radius, radiance, |direction| {
            let cos = direction.z;
            (1.0 - fresnel(cos, lower))
                * (1.0 - fresnel(cos, upper / lower))
                * (1.0 - fresnel(cos, upper))
        });
    let air_gap = ALBEDO / PI
        * disc_irradiance(Vector3::UNIT_Z, radius, radiance, |direction| {
            let cos = direction.z;
            (1.0 - fresnel(cos, lower)).powi(2) * (1.0 - fresnel(cos, upper)).powi(2)
        });
    println!(
        "two glasses: {} +- {}, expected {expected} (with an air gap {air_gap})",
        stats.mean(),
        stats.standard_error()
    );
    let allowed = 5.0 * stats.standard_error() + 1e-4 * expected;
    assert!(
        (stats.mean() - expected).abs() <= allowed,
        "the floor under two touching glasses is {} +- {}, expected {expected} (allowed {allowed})",
        stats.mean(),
        stats.standard_error()
    );
}
