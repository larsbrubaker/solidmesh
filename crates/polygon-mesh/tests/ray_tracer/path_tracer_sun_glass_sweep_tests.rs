//! Sweeps of `SunThroughGlass::Straight` across a setting a user moves (roughness, transmission)
//! where a light path could be left as specks or stepped: a partly-glass pane over a floor, and a
//! glossy part embedded in a glass block. Each sweep is sample-budgeted to run in the default test
//! profile in a few seconds; a measure prints what a known limit costs.

use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::ray_tracer::{
    PathTraceSettings, RenderCamera, RenderScene, ScatteringSurface, SunThroughGlass,
};
use solidmesh_mesh::solids::platonic_solids::create_cube;

use crate::path_tracer_sun_glass_rules_tests::{
    exact, floor_camera, glass, matte, pane_scene, sample_values, scene, straight, sun, with, IOR,
};
use crate::path_tracer_support::{agreement_tolerance, estimate, fine_sphere};
use crate::scattering_support::Stats;

/// A sample brighter than this on a floor that reads about 0.035 to 0.1 is a speck: the sun
/// itself carried by a path that found it through or off glass.
const SPECK: f64 = 0.5;

/// A matte part inside the glass block, sun and view straight down and at 30 degrees, as the
/// unbent line read it: mean and standard error (`a_matte_part_inside_glass_keeps_its_light`).
const MATTE_UNBENT_0: f64 = 21.416616638733384;
const MATTE_ERROR_0: f64 = 0.5685798642128329;
const MATTE_UNBENT_30: f64 = 22.28808688609476;
const MATTE_ERROR_30: f64 = 0.580552121587151;

fn stats_of(values: &[f64]) -> Stats {
    let mut stats = Stats::default();
    for &value in values {
        stats.add(value);
    }
    stats
}

/// Every roughness either side of the smooth threshold (about 0.0316) and well past it.
const ROUGHNESSES: [f64; 6] = [0.0, 0.031, 0.032, 0.05, 0.1, 0.3];

/// A pane of `transmission` glass and the rest plastic over a matte floor, under a large low sun
/// so light bouncing inside the pane is common, at each of `ROUGHNESSES`: no sample of the floor
/// is a speck where the plastic share is smooth or broad, and the floor's mean does not step
/// across the threshold.
fn assert_partly_glass_pane_is_speck_free(transmission: f64) {
    let sun = sun(60.0_f64.to_radians(), 1.0, 10.0_f64.to_radians());
    // Roulette off, so a survivor's reweighting is not counted; eight bounces reach the family
    // that went back and forth inside the pane (floor, refraction in, two reflections off the
    // plastic share, refraction out, the sun).
    let settings = with(SunThroughGlass::Straight, |s| {
        s.russian_roulette_after = None;
        s.max_bounces = 8;
    });
    let mut rows = Vec::new();
    for roughness in ROUGHNESSES {
        let pane = ScatteringSurface::new([0.8; 3], 0.0, roughness, transmission, IOR);
        let scene = pane_scene(matte(), pane, sun);
        let values = sample_values(&scene, floor_camera(0.0), settings, 4096);
        let stats = stats_of(&values);
        let specks = values.iter().filter(|&&value| value > SPECK).count();
        let largest = values.iter().copied().fold(0.0, f64::max);
        println!(
            "transmission {transmission} roughness {roughness}: {} +- {}, {specks} over \
             {SPECK}, largest {largest}",
            stats.mean(),
            stats.standard_error()
        );
        rows.push((roughness, stats, specks));
    }
    // Light that went back and forth inside the pane off its smooth plastic share leaves no
    // specks, nor does a broad share's; the sharp sampled share's own glints through the pane
    // stay (`glass_chain`, known limit 4), so those rows are printed, not pinned.
    for (roughness, _, specks) in &rows {
        if *roughness < 0.032 || *roughness >= 0.3 {
            assert_eq!(
                *specks, 0,
                "the floor under a pane of transmission {transmission} at roughness {roughness} \
                 has {specks} specks"
            );
        }
    }
    let (smooth, frosted) = (&rows[1].1, &rows[2].1);
    let allowed =
        3.0 * (smooth.standard_error().powi(2) + frosted.standard_error().powi(2)).sqrt() + 1e-9;
    assert!(
        (smooth.mean() - frosted.mean()).abs() <= allowed,
        "the floor under a pane of transmission {transmission} steps from {} at roughness 0.031 \
         to {} at 0.032 (allowed {allowed})",
        smooth.mean(),
        frosted.mean()
    );
}

#[test]
fn a_quarter_glass_pane_leaves_no_specks_from_light_bouncing_inside_it() {
    assert_partly_glass_pane_is_speck_free(0.25);
}

#[test]
fn a_half_glass_pane_leaves_no_specks_from_light_bouncing_inside_it() {
    assert_partly_glass_pane_is_speck_free(0.5);
}

#[test]
fn a_three_quarter_glass_pane_leaves_no_specks_from_light_bouncing_inside_it() {
    assert_partly_glass_pane_is_speck_free(0.75);
}

/// A cube of `part`, 1 by 1 by 0.5, embedded in the middle of a clear glass block 6 by 6 by 3
/// standing on nothing (or a glass ball of radius 3 around it, with `ball`), under a 2.5 degree
/// sun `angle_degrees` off straight up, and a camera above the glass in the direction that
/// mirrors the sun off the cube's top.
fn embedded_part(
    part: ScatteringSurface,
    angle_degrees: f64,
    ball: bool,
) -> (RenderScene, RenderCamera) {
    let glass_mesh = if ball {
        fine_sphere(3.0)
    } else {
        create_cube(6.0, 6.0, 3.0)
    };
    let scene = scene(
        vec![
            (
                glass_mesh,
                Matrix4X4::create_translation(0.0, 0.0, 1.5),
                glass(0.0, IOR),
            ),
            (
                create_cube(1.0, 1.0, 0.5),
                Matrix4X4::create_translation(0.0, 0.0, 1.5),
                part,
            ),
        ],
        sun(angle_degrees.to_radians(), 1.0, 2.5_f64.to_radians()),
    );
    let angle = angle_degrees.to_radians();
    let view = Vector3::new(-angle.sin(), 0.0, angle.cos());
    let target = Vector3::new(0.0, 0.0, 1.75);
    let eye = target + view * (5.0 / view.z);
    let camera = RenderCamera::orthographic(eye, target, Vector3::UNIT_Y, 0.4).unwrap();
    (scene, camera)
}

fn metal(roughness: f64) -> ScatteringSurface {
    ScatteringSurface::new([0.9; 3], 1.0, roughness, 0.0, IOR)
}

fn matte_part() -> ScatteringSurface {
    ScatteringSurface::new([0.9; 3], 0.0, 1.0, 0.0, IOR)
}

/// The mean of an embedded part under `settings`, over 16 pixels of `samples` each.
fn embedded_mean(
    part: ScatteringSurface,
    angle: f64,
    ball: bool,
    settings: PathTraceSettings,
    samples: u32,
) -> Stats {
    let (scene, camera) = embedded_part(part, angle, ball);
    estimate(&scene, camera, (4, 4), settings, samples).0[1]
}

/// How many standard errors of the pair `a` and `b` are apart.
fn z_of(a: &Stats, b: &Stats) -> f64 {
    (a.mean() - b.mean()) / (a.standard_error().powi(2) + b.standard_error().powi(2)).sqrt()
}

/// A metal cube inside a flat-faced glass block, at normal incidence and at 30 degrees: the
/// straight line from inside glass reads the part's lobe along the direction bent at the block's
/// face, over the disc as seen inside (`glass_chain`, "A receiver inside glass"), so the glint
/// reads Exact's light at every roughness and does not step at the smooth threshold.
#[test]
fn a_glossy_part_inside_glass_reads_exacts_glint_at_every_roughness() {
    for angle in [0.0, 30.0] {
        let mut rows = Vec::new();
        for roughness in [0.031, 0.032, 0.05, 0.08, 0.12, 0.2, 0.3] {
            let s = embedded_mean(metal(roughness), angle, false, straight(), 2048);
            let e = embedded_mean(metal(roughness), angle, false, exact(), 4096);
            println!(
                "sun and view {angle} roughness {roughness}: straight {} +- {}, exact {} +- {} \
                 (z {:+.1})",
                s.mean(),
                s.standard_error(),
                e.mean(),
                e.standard_error(),
                z_of(&s, &e)
            );
            rows.push((roughness, s, e));
        }
        for (roughness, s, e) in &rows[1..] {
            let allowed = agreement_tolerance(s, e);
            assert!(
                (s.mean() - e.mean()).abs() <= allowed,
                "a metal part at roughness {roughness} inside glass, sun and view {angle}: \
                 straight {} against exact {} (allowed {allowed})",
                s.mean(),
                e.mean()
            );
        }
        let ((_, s0, e0), (_, s1, e1)) = (&rows[0], &rows[1]);
        let step = (s1.mean() - s0.mean()) - (e1.mean() - e0.mean());
        let allowed = 3.0
            * [s0, s1, e0, e1]
                .iter()
                .map(|stats| stats.standard_error().powi(2))
                .sum::<f64>()
                .sqrt();
        assert!(
            step.abs() <= allowed,
            "a metal part inside glass, sun and view {angle}, steps {step} more than Exact from \
             roughness 0.031 to 0.032 (allowed {allowed})"
        );
    }
}

/// A matte part inside glass keeps the light the unbent line gives it: the bent line narrows the
/// disc by `n^2 cos_in / cos_out` and brightens it by `n^2`, which a broad lobe does not tell
/// apart, so its irradiance is what it was. The pinned means are the unbent line's, measured with
/// these settings and seeds; the allowance is five standard errors of the pair.
#[test]
fn a_matte_part_inside_glass_keeps_its_light() {
    for (angle, unbent, unbent_error) in [
        (0.0, MATTE_UNBENT_0, MATTE_ERROR_0),
        (30.0, MATTE_UNBENT_30, MATTE_ERROR_30),
    ] {
        let s = embedded_mean(matte_part(), angle, false, straight(), 2048);
        println!(
            "matte part inside glass, sun and view {angle}: {} +- {}",
            s.mean(),
            s.standard_error()
        );
        let allowed = 5.0 * (s.standard_error().powi(2) + unbent_error * unbent_error).sqrt();
        assert!(
            (s.mean() - unbent).abs() <= allowed,
            "a matte part inside glass, sun and view {angle}, reads {} where the unbent line gave \
             {unbent} (allowed {allowed})",
            s.mean()
        );
    }
}

/// The embedded parts against Exact, in the block and in a glass ball, whose curved first face
/// the bent line reads as the plane touching it where the line crosses.
#[test]
#[ignore = "a measure: run with --ignored --nocapture"]
fn measure_a_part_inside_glass_against_exact() {
    for ball in [false, true] {
        for angle in [0.0, 30.0] {
            for roughness in [0.031, 0.032, 0.05, 0.08, 0.12, 0.2, 0.3, 0.5] {
                let s = embedded_mean(metal(roughness), angle, ball, straight(), 2048);
                let e = embedded_mean(metal(roughness), angle, ball, exact(), 4096);
                println!(
                    "ball {ball} sun and view {angle} metal {roughness}: straight {:.3} +- {:.3}, \
                     exact {:.3} +- {:.3}, {:+.2}% (z {:+.1})",
                    s.mean(),
                    s.standard_error(),
                    e.mean(),
                    e.standard_error(),
                    100.0 * (s.mean() / e.mean() - 1.0),
                    z_of(&s, &e)
                );
            }
            let s = embedded_mean(matte_part(), angle, ball, straight(), 2048);
            println!(
                "ball {ball} sun and view {angle} matte: straight {} +- {}",
                s.mean(),
                s.standard_error()
            );
        }
    }
}
