//! Where the white specks over glass come from: a shallow open glass tray with interior walls on
//! a matte floor under a sun and a sky gradient, rendered at 160 by 120 and 128 samples a pixel
//! through `PathTracer::sample_pixel`, once per variant (no sun, each way of reaching the sun,
//! roulette off, bounce limits, rough glass, a matte or metal tray, the tray lifted off the floor,
//! and white furnaces that would catch a path gaining energy), with the tray built both as
//! touching boxes, each its own part, and as one unioned solid. Each variant prints one row: how
//! many samples are bright and how much of the energy they carry, how many pixels stand out of
//! their neighbourhood, how bright the floor seen through the glass is, and the brightest single
//! sample. Ignored by default, since it reports rather than checks; run it in a release build.

use std::sync::Arc;

use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::ray_tracer::{
    Environment, LightSetup, LightStrategy, MeshSurface, PathTraceSettings, PathTracer, PlacedPart,
    PlacedParts, RenderCamera, RenderScene, RenderWork, ScatteringSurface,
};

use crate::glass_tray_support::{rig_lights, tray_meshes};
use crate::path_tracer_support::{flat_patch, SHARP_CREASE};
use crate::surface_hit_support::tree_of;

const WIDTH: u32 = 160;
const HEIGHT: u32 = 120;
const SAMPLES: u32 = 128;
const THREADS: usize = 6;

/// The floor: Lambertian (an index of 1 has no sheen), albedo 0.3.
fn floor_surface() -> ScatteringSurface {
    ScatteringSurface::new([0.3; 3], 0.0, 1.0, 0.0, 1.0)
}

/// Part 0 is the floor; every other part is a piece of the tray (or the whole tray, as one
/// solid), lifted by `lift`.
fn tray_scene(variant: &Variant) -> RenderScene {
    let floor = flat_patch(2000.0, 0.0);
    let mut parts = vec![PlacedPart::new(
        tree_of(&floor),
        Arc::new(MeshSurface::new(&floor, SHARP_CREASE).unwrap()),
        Matrix4X4::IDENTITY,
    )
    .unwrap()];
    let mut surfaces = vec![floor_surface()];
    for mesh in tray_meshes(variant.one_solid) {
        parts.push(
            PlacedPart::new(
                tree_of(&mesh),
                Arc::new(MeshSurface::new(&mesh, SHARP_CREASE).unwrap()),
                Matrix4X4::create_translation(0.0, 0.0, variant.lift),
            )
            .unwrap(),
        );
        surfaces.push(variant.tray);
    }
    RenderScene::new(PlacedParts::new(parts), surfaces, variant.lights).unwrap()
}

fn camera() -> RenderCamera {
    RenderCamera::perspective(
        Vector3::new(60.0, -140.0, 150.0),
        Vector3::new(0.0, 0.0, 4.0),
        Vector3::UNIT_Z,
        40.0,
    )
    .unwrap()
}

fn luminance(c: [f64; 3]) -> f64 {
    0.2126 * c[0] + 0.7152 * c[1] + 0.0722 * c[2]
}

/// What a pixel's center ray first meets.
#[derive(Clone, Copy, PartialEq)]
enum Seen {
    Sky,
    Floor,
    /// The tray, and whether at the top of its base inside the rim: the floor seen through glass.
    Tray {
        base_inside: bool,
    },
}

fn seen_at(scene: &RenderScene, x: u32, y: u32, lift: f64) -> Seen {
    let mut ray = camera().ray_through(f64::from(x) + 0.5, f64::from(y) + 0.5, WIDTH, HEIGHT);
    let Some(hit) = scene.parts().trace(&mut ray) else {
        return Seen::Sky;
    };
    if hit.part.index() == 0 {
        return Seen::Floor;
    }
    let p = hit.surface_point().unwrap().position;
    let base_inside = (p.z - (2.0 + lift)).abs() < 1e-6 && p.x.abs() < 48.0 && p.y.abs() < 48.0;
    Seen::Tray { base_inside }
}

/// One variant's samples, summed.
#[derive(Default, Clone)]
struct Tally {
    samples: u64,
    above_10: u64,
    above_50: u64,
    energy: f64,
    energy_above_10: f64,
    energy_above_50: f64,
    non_finite: u64,
    /// Samples with any channel over 1.001, and the largest channel seen.
    above_one: u64,
    largest_channel: f64,
    /// The brightest sample's luminance, pixel and index.
    brightest: (f64, u32, u32, u32),
}

impl Tally {
    fn add(&mut self, other: &Tally) {
        self.samples += other.samples;
        self.above_10 += other.above_10;
        self.above_50 += other.above_50;
        self.energy += other.energy;
        self.energy_above_10 += other.energy_above_10;
        self.energy_above_50 += other.energy_above_50;
        self.non_finite += other.non_finite;
        self.above_one += other.above_one;
        self.largest_channel = self.largest_channel.max(other.largest_channel);
        if other.brightest.0 > self.brightest.0 {
            self.brightest = other.brightest;
        }
    }
}

/// Every pixel's mean luminance (row major) and the tally, rendered on `THREADS` threads by row.
fn render(scene: &RenderScene, settings: PathTraceSettings) -> (Vec<f64>, Tally) {
    let tracer = PathTracer::new(scene, camera(), WIDTH, HEIGHT, settings).unwrap();
    let tracer = &tracer;
    let rows: Vec<(Vec<f64>, Tally)> = std::thread::scope(|scope| {
        let handles: Vec<_> = (0..THREADS)
            .map(|thread| {
                scope.spawn(move || {
                    let mut out = Vec::new();
                    let mut work = RenderWork::default();
                    for y in (0..HEIGHT).filter(|y| *y as usize % THREADS == thread) {
                        let mut means = Vec::with_capacity(WIDTH as usize);
                        let mut tally = Tally::default();
                        for x in 0..WIDTH {
                            let mut sum = 0.0;
                            for sample in 0..SAMPLES {
                                let c = tracer.sample_pixel(x, y, sample, &mut work).unwrap();
                                tally.samples += 1;
                                if !c.iter().all(|v| v.is_finite()) {
                                    tally.non_finite += 1;
                                    continue;
                                }
                                let lum = luminance(c);
                                let top = c[0].max(c[1]).max(c[2]);
                                sum += lum;
                                tally.energy += lum;
                                if lum > 10.0 {
                                    tally.above_10 += 1;
                                    tally.energy_above_10 += lum;
                                }
                                if lum > 50.0 {
                                    tally.above_50 += 1;
                                    tally.energy_above_50 += lum;
                                }
                                if top > 1.001 {
                                    tally.above_one += 1;
                                }
                                tally.largest_channel = tally.largest_channel.max(top);
                                if lum > tally.brightest.0 {
                                    tally.brightest = (lum, x, y, sample);
                                }
                            }
                            means.push(sum / f64::from(SAMPLES));
                        }
                        out.push((y, means, tally));
                    }
                    out
                })
            })
            .collect();
        let mut all: Vec<(u32, Vec<f64>, Tally)> = handles
            .into_iter()
            .flat_map(|handle| handle.join().unwrap())
            .collect();
        all.sort_by_key(|(y, _, _)| *y);
        all.into_iter().map(|(_, m, t)| (m, t)).collect()
    });
    let mut image = Vec::with_capacity((WIDTH * HEIGHT) as usize);
    let mut tally = Tally::default();
    for (means, row_tally) in rows {
        image.extend(means);
        tally.add(&row_tally);
    }
    (image, tally)
}

/// Whether pixel (`x`, `y`)'s mean is over twice the median of its 5 by 5 neighbourhood (the
/// part of it inside the image).
fn is_speck(image: &[f64], x: u32, y: u32) -> bool {
    let mut around = Vec::with_capacity(25);
    for ny in y.saturating_sub(2)..(y + 3).min(HEIGHT) {
        for nx in x.saturating_sub(2)..(x + 3).min(WIDTH) {
            around.push(image[(ny * WIDTH + nx) as usize]);
        }
    }
    around.sort_by(f64::total_cmp);
    let median = around[around.len() / 2];
    image[(y * WIDTH + x) as usize] > 2.0 * median
}

struct Variant {
    label: &'static str,
    tray: ScatteringSurface,
    lift: f64,
    /// The tray as one unioned solid rather than touching boxes.
    one_solid: bool,
    lights: LightSetup,
    settings: PathTraceSettings,
}

fn report(variant: &Variant) {
    let scene = tray_scene(variant);
    let (image, tally) = render(&scene, variant.settings);
    let (mut specks, mut glass, mut glass_specks) = (0_u32, 0_u32, 0_u32);
    let (mut base_sum, mut base_count, mut glass_sum) = (0.0, 0_u32, 0.0);
    for y in 0..HEIGHT {
        for x in 0..WIDTH {
            let speck = is_speck(&image, x, y);
            specks += u32::from(speck);
            if let Seen::Tray { base_inside } = seen_at(&scene, x, y, variant.lift) {
                let mean = image[(y * WIDTH + x) as usize];
                glass += 1;
                glass_sum += mean;
                glass_specks += u32::from(speck);
                if base_inside {
                    base_sum += mean;
                    base_count += 1;
                }
            }
        }
    }
    let samples = tally.samples as f64;
    let energy = tally.energy.max(f64::MIN_POSITIVE);
    println!(
        "{:<28} | {:>7.4}% {:>7.4}% | {:>6.2}% {:>6.2}% | {:>6.2}% {:>6.2}% | {:>8.4} {:>8.4} | {:>3} | {:>9.3} @ ({},{}) s{} | >1.001: {} max {:.6}",
        variant.label,
        100.0 * tally.above_10 as f64 / samples,
        100.0 * tally.above_50 as f64 / samples,
        100.0 * tally.energy_above_10 / energy,
        100.0 * tally.energy_above_50 / energy,
        100.0 * f64::from(specks) / f64::from(WIDTH * HEIGHT),
        100.0 * f64::from(glass_specks) / f64::from(glass.max(1)),
        base_sum / f64::from(base_count.max(1)),
        glass_sum / f64::from(glass.max(1)),
        tally.non_finite,
        tally.brightest.0,
        tally.brightest.1,
        tally.brightest.2,
        tally.brightest.3,
        tally.above_one,
        tally.largest_channel,
    );
}

fn with(settings: impl FnOnce(&mut PathTraceSettings)) -> PathTraceSettings {
    let mut s = PathTraceSettings::default();
    settings(&mut s);
    s
}

#[test]
#[ignore = "reports where glass specks come from; run in a release build"]
fn measure_speckles_under_a_glass_tray() {
    let glass = ScatteringSurface::glass();
    let rig = rig_lights();
    let sky_only = LightSetup::environment_only(rig.environment);
    let furnace = LightSetup::environment_only(Environment::constant([1.0; 3]).unwrap());
    let base = |label, settings| Variant {
        label,
        tray: glass,
        lift: 0.0,
        one_solid: false,
        lights: rig,
        settings,
    };
    let no_roulette = with(|s| s.russian_roulette_after = None);
    let mut variants = vec![
        base("baseline", PathTraceSettings::default()),
        Variant {
            lights: sky_only,
            ..base("no sun", PathTraceSettings::default())
        },
        base(
            "next event only",
            with(|s| s.strategy = LightStrategy::NextEventOnly),
        ),
        base(
            "surface sampling only",
            with(|s| s.strategy = LightStrategy::SurfaceSamplingOnly),
        ),
        base("roulette off", no_roulette),
    ];
    for (label, bounces) in [
        ("max_bounces 1", 1),
        ("max_bounces 2", 2),
        ("max_bounces 3", 3),
        ("max_bounces 4", 4),
        ("max_bounces 8", 8),
    ] {
        variants.push(base(label, with(|s| s.max_bounces = bounces)));
    }
    variants.extend([
        Variant {
            tray: ScatteringSurface::new([1.0; 3], 0.0, 0.3, 1.0, 1.5),
            ..base("glass roughness 0.3", PathTraceSettings::default())
        },
        Variant {
            tray: ScatteringSurface::matte_plastic([0.8; 3]),
            ..base("tray matte plastic", PathTraceSettings::default())
        },
        Variant {
            lift: 0.01,
            ..base("tray lifted 0.01", PathTraceSettings::default())
        },
        Variant {
            tray: ScatteringSurface::polished_metal([0.9; 3]),
            ..base("tray polished metal r0.05", PathTraceSettings::default())
        },
        Variant {
            lights: furnace,
            ..base("furnace, roulette off", no_roulette)
        },
        Variant {
            lights: furnace,
            lift: 0.01,
            ..base("furnace lifted, roulette off", no_roulette)
        },
        Variant {
            lights: furnace,
            ..base("furnace, roulette default", PathTraceSettings::default())
        },
    ]);
    let solid = |variant: Variant| Variant {
        one_solid: true,
        ..variant
    };
    variants.extend([
        solid(base("one solid: baseline", PathTraceSettings::default())),
        solid(Variant {
            lights: sky_only,
            ..base("one solid: no sun", PathTraceSettings::default())
        }),
        solid(base("one solid: roulette off", no_roulette)),
        solid(Variant {
            lift: 0.01,
            ..base("one solid: lifted 0.01", PathTraceSettings::default())
        }),
        solid(Variant {
            tray: ScatteringSurface::new([1.0; 3], 0.0, 0.3, 1.0, 1.5),
            ..base("one solid: glass r0.3", PathTraceSettings::default())
        }),
        solid(Variant {
            lights: furnace,
            ..base("one solid: furnace, no RR", no_roulette)
        }),
        solid(Variant {
            lights: furnace,
            lift: 0.01,
            ..base("one solid: furnace lift, no RR", no_roulette)
        }),
    ]);
    println!(
        "variant                      | samples lum>10 lum>50 | energy >10 >50 | speck px all glass | under-glass floor, all glass mean | non-finite | brightest sample | any channel over 1.001"
    );
    for variant in &variants {
        report(variant);
    }
}
