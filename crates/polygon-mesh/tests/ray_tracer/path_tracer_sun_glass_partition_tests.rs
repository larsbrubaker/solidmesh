//! The rule `SunThroughGlass::Straight` is built on, against `SunThroughGlass::Exact`: the sun
//! through glass at a receiver is the straight line and exact transport mixed by the receiver's
//! own power heuristic weights, which sum to one. Glass that does not bend (index-matched) makes
//! the two routes the same light, so Straight must read Exact's mean on every floor and by every
//! light strategy, or a light path is dropped or counted twice. A partly-glass pane steps nowhere
//! as its roughness crosses the smooth threshold; a diffuse floor under glass has no sample
//! brighter than its own direct light; and the camera seeing glass in a mirror, or a floor through
//! glass the path leaves by no face, sees Exact's bits.

use std::f64::consts::PI;

use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::ray_tracer::{
    LightStrategy, PathTraceSettings, RenderCamera, RenderScene, ScatteringSurface, Sun,
    SunThroughGlass,
};
use solidmesh_mesh::solids::platonic_solids::create_cube;

use crate::path_tracer_sun_glass_rules_tests::{
    exact, floor_camera, glass, matte, pane_scene, sample_values, scene as rules_scene, straight,
    sun, with, ALBEDO, IOR,
};
use crate::path_tracer_support::{agreement_tolerance, estimate, flat_patch};
use crate::scattering_support::Stats;

/// An index the scattering model treats as index-matched, so glass of it refracts as a smooth
/// face by Snell's law (bending by under 0.05%) and reflects about 6e-8 of the light at normal
/// incidence: the straight line and the exact path through it carry the same sun. Only within
/// about two degrees of grazing does it reflect much; that returns about 1e-4 of a matte floor's
/// light to it (measured: 0.30003 under it against 0.30000 in the open), a quarter of the open
/// floor check's window.
const MATCHED: f64 = 1.0005;

/// A sample brighter than this on a floor that reads about 0.15 is a speck: the sun itself, some
/// 60 times the floor, carried by a path that found it through or off glass.
const SPECK: f64 = 0.5;

/// The mean and standard error of `values`, and how many are specks.
fn summary(values: &[f64]) -> (Stats, usize) {
    let mut stats = Stats::default();
    for &value in values {
        stats.add(value);
    }
    (stats, values.iter().filter(|&&value| value > SPECK).count())
}

fn mirror() -> ScatteringSurface {
    ScatteringSurface::new([0.9; 3], 1.0, 0.0, 0.0, 1.5)
}

/// `settings` with `strategy`.
fn by(strategy: LightStrategy, settings: PathTraceSettings) -> PathTraceSettings {
    PathTraceSettings {
        strategy,
        ..settings
    }
}

/// Exact, and Straight by each light strategy, for `camera`'s view of `scene`; every pair agrees
/// within its noise with no further slack.
fn assert_reads_exact(what: &str, scene: &RenderScene, camera: RenderCamera, samples: u32) {
    let runs = [
        ("exact", exact(), 2 * samples),
        ("straight", straight(), samples),
        (
            "straight, next event only",
            by(LightStrategy::NextEventOnly, straight()),
            samples,
        ),
        (
            "straight, surface sampling only",
            by(LightStrategy::SurfaceSamplingOnly, straight()),
            2 * samples,
        ),
    ];
    let estimates: Vec<(&str, Stats)> = runs
        .into_iter()
        .map(|(label, settings, samples)| {
            (
                label,
                estimate(scene, camera, (4, 4), settings, samples).0[0],
            )
        })
        .collect();
    for (label, stats) in &estimates {
        println!(
            "{what}, {label}: {} +- {}",
            stats.mean(),
            stats.standard_error()
        );
    }
    assert!(estimates[0].1.mean() > 0.0, "{what}: no light at all");
    for (i, (a_label, a)) in estimates.iter().enumerate() {
        for (b_label, b) in &estimates[i + 1..] {
            let allowed = agreement_tolerance(a, b);
            assert!(
                (a.mean() - b.mean()).abs() <= allowed,
                "{what}: {a_label} reads {} and {b_label} {} (allowed {allowed}): a light path \
                 is dropped or counted twice",
                a.mean(),
                b.mean()
            );
        }
    }
}

#[test]
fn glass_that_does_not_bend_reads_exacts_mean_on_every_floor_and_by_every_strategy() {
    // Overhead, so the view's mirror direction off the floor is the sun's middle.
    let sun = sun(0.0, 1.0, 10.0_f64.to_radians());
    let floors = [
        ("matte", matte()),
        (
            "glossy 0.3",
            ScatteringSurface::new([0.9; 3], 1.0, 0.3, 0.0, 1.5),
        ),
        ("polished 0.05", ScatteringSurface::polished_metal([0.9; 3])),
        ("mirror", mirror()),
    ];
    for (label, floor) in floors {
        let scene = pane_scene(floor, glass(0.0, MATCHED), sun);
        assert_reads_exact(
            &format!("{label} floor under index-matched glass"),
            &scene,
            floor_camera(0.0),
            2048,
        );
        // The pane is all but not there (`MATCHED`): the floor reads what it reads in the open,
        // where its own estimators find it with little noise.
        let open = rules_scene(
            vec![(flat_patch(1000.0, 0.0), Matrix4X4::IDENTITY, floor)],
            sun,
        );
        let open = estimate(&open, floor_camera(0.0), (4, 4), straight(), 2048).0[0];
        let under = estimate(&scene, floor_camera(0.0), (4, 4), straight(), 2048).0[0];
        println!(
            "{label} floor in the open: {} +- {}, under index-matched glass {} +- {}",
            open.mean(),
            open.standard_error(),
            under.mean(),
            under.standard_error()
        );
        let allowed = agreement_tolerance(&open, &under);
        assert!(
            (open.mean() - under.mean()).abs() <= allowed,
            "the {label} floor reads {} in the open and {} under index-matched glass (allowed \
             {allowed})",
            open.mean(),
            under.mean()
        );
    }
    // Half glass and half frosted plastic, which at this index is all but diffuse.
    let partly = ScatteringSurface::new([0.8; 3], 0.0, 0.3, 0.5, MATCHED);
    let scene = pane_scene(matte(), partly, sun);
    assert_reads_exact(
        "matte floor under an index-matched partly-glass pane",
        &scene,
        floor_camera(0.0),
        2048,
    );
}

#[test]
fn a_partly_glass_pane_steps_nowhere_across_the_smooth_threshold() {
    // Half glass and half plastic at every roughness either side of the threshold (about 0.0316),
    // over a matte floor, under a large sun low enough that light inside the pane is common.
    let sun = sun(60.0_f64.to_radians(), 1.0, 10.0_f64.to_radians());
    let settings = with(SunThroughGlass::Straight, |s| {
        s.russian_roulette_after = None
    });
    let mut rows = Vec::new();
    for roughness in [0.0, 0.031, 0.032, 0.05, 0.1, 0.3] {
        let pane = ScatteringSurface::new([0.8; 3], 0.0, roughness, 0.5, IOR);
        let scene = pane_scene(matte(), pane, sun);
        let values = sample_values(&scene, floor_camera(0.0), settings, 4096);
        let (stats, specks) = summary(&values);
        let largest = values.iter().copied().fold(0.0, f64::max);
        println!(
            "partly-glass pane at roughness {roughness}: {} +- {}, {specks} over {SPECK}, \
             largest {largest}",
            stats.mean(),
            stats.standard_error()
        );
        rows.push((roughness, stats, specks));
    }
    let (_, smooth, smooth_specks) = rows[1];
    let (_, frosted, frosted_specks) = rows[2];
    let allowed =
        3.0 * (smooth.standard_error().powi(2) + frosted.standard_error().powi(2)).sqrt() + 1e-9;
    assert!(
        (smooth.mean() - frosted.mean()).abs() <= allowed,
        "the floor under a partly-glass pane steps from {} at roughness 0.031 to {} at 0.032 \
         (allowed {allowed})",
        smooth.mean(),
        frosted.mean()
    );
    // Glints of the plastic share inside the pane are kept at every roughness, 0 included: as
    // many either side of the threshold, within their counts' own noise.
    for (a, b, what) in [
        (rows[0].2, frosted_specks, "0 and 0.032"),
        (smooth_specks, frosted_specks, "0.031 and 0.032"),
    ] {
        let allowed = 3.0 * ((a + b) as f64).sqrt() + 3.0;
        assert!(
            (a as f64 - b as f64).abs() <= allowed,
            "the floor under a partly-glass pane has {a} and {b} specks at roughness {what} \
             (allowed {allowed})"
        );
    }
}

#[test]
fn a_diffuse_floor_under_glass_has_no_sample_brighter_than_its_direct_light() {
    // A sample that reaches the sun through glass from a diffuse floor carries at most about the
    // sun times the floor's power heuristic weight against the sun, so no sample exceeds the
    // floor's own direct light through nothing, plus what the pane returns of it.
    let (from_zenith, radius) = (60.0_f64.to_radians(), 10.0_f64.to_radians());
    let sun: Sun = sun(from_zenith, 1.0, radius);
    let settings = with(SunThroughGlass::Straight, |s| {
        s.russian_roulette_after = None
    });
    let radiance = 1.0 / (radius.sin() * radius.sin());
    // The brightest direct sample: the disc's nearest edge to straight up, nothing in the way.
    let brightest = ALBEDO / PI * radiance * sun.solid_angle() * (from_zenith - radius).cos();
    let bound = 1.25 * brightest;
    for roughness in [0.0, 0.032, 0.1, 0.3] {
        let scene = pane_scene(matte(), glass(roughness, IOR), sun);
        let values = sample_values(&scene, floor_camera(0.0), settings, 4096);
        let largest = values.iter().copied().fold(0.0, f64::max);
        println!("pane at roughness {roughness}: largest {largest}, bound {bound}");
        assert!(
            largest <= bound,
            "a sample of the floor under a pane at roughness {roughness} is {largest}, over \
             {bound}: glass-first light reached it at more than its weight"
        );
    }
}

#[test]
fn the_camera_sees_a_floor_through_glass_it_leaves_by_no_face_as_exact_shows_it() {
    // Looking down at 30 degrees through glass at a metal floor, the sun where a ray bent once
    // into the glass mirrors off the floor, clear of the glass: the camera's ray refracts in and
    // meets no face on the way out (the far face is found not there, or there is none), so the
    // floor lies in open air while the path still counts itself inside glass. Its clear line to
    // the sun is Exact's pair, which must sum to one.
    let view_angle = 30.0_f64.to_radians();
    let view = Vector3::new(-view_angle.sin(), 0.0, view_angle.cos());
    let center = view * 3.0;
    let eye = view * (6.0 / view.z);
    let camera = RenderCamera::orthographic(eye, Vector3::ZERO, Vector3::UNIT_Y, 0.2).unwrap();
    let sun = sun(19.4712206_f64.to_radians(), 1.0, 2.5_f64.to_radians());
    let at_center = Matrix4X4::create_translation(center.x, center.y, center.z);
    for roughness in [0.05, 0.1, 0.2] {
        let floor = ScatteringSurface::new([0.9; 3], 1.0, roughness, 0.0, 1.5);
        let open_floor = (flat_patch(1000.0, 0.0), Matrix4X4::IDENTITY, floor);
        let half_there = rules_scene(
            vec![
                open_floor.clone(),
                (create_cube(2.0, 2.0, 2.0), at_center, glass(0.0, IOR)),
            ],
            sun,
        )
        .with_part_opacities(vec![1.0, 0.5])
        .unwrap();
        let sheet = rules_scene(
            vec![
                open_floor,
                (flat_patch(0.6, 0.0), at_center, glass(0.0, IOR)),
            ],
            sun,
        );
        for (what, scene) in [("a half-there cube", half_there), ("a sheet", sheet)] {
            let straight = sample_values(&scene, camera, straight(), 512);
            let exact = sample_values(&scene, camera, exact(), 512);
            // The two settings draw the same random numbers, so a sample that differs differs
            // only by what Straight does with glass; their paired difference has its own noise.
            let mut difference = Stats::default();
            for (s, e) in straight.iter().zip(&exact) {
                difference.add(s - e);
            }
            let differing = straight.iter().zip(&exact).filter(|(s, e)| s != e).count();
            let (exact, _) = summary(&exact);
            println!(
                "floor at roughness {roughness} through {what}: exact {} +- {}, straight - exact \
                 {} +- {}, {differing} samples differ",
                exact.mean(),
                exact.standard_error(),
                difference.mean(),
                difference.standard_error()
            );
            assert!(exact.mean() > 0.0, "{what}: the floor shows no sun");
            // Below roughness 0.2 no light leaves the floor back through the glass to the sun, so
            // every sample is Exact's; at 0.2 a few do, and Straight splits them by its own weights.
            if roughness < 0.2 {
                assert_eq!(
                    differing, 0,
                    "the floor at roughness {roughness} seen through {what}"
                );
            }
            let allowed = 5.0 * difference.standard_error() + 1e-12;
            assert!(
                difference.mean().abs() <= allowed,
                "the floor at roughness {roughness} seen through {what} reads {} less than Exact \
                 (allowed {allowed}): its clear line to the sun is not counted in full",
                -difference.mean()
            );
        }
    }
}

#[test]
fn the_camera_sees_glass_in_a_mirror_as_exact_shows_it() {
    // Between a mirror floor and a pane, looking down: the mirror shows the sun through the pane.
    let scene = pane_scene(
        mirror(),
        glass(0.0, IOR),
        sun(0.0, 1.0, 2.5_f64.to_radians()),
    );
    let straight = estimate(&scene, floor_camera(0.0), (4, 4), straight(), 256).0;
    let exact = estimate(&scene, floor_camera(0.0), (4, 4), exact(), 256).0;
    println!(
        "glass in a mirror: straight {} +- {}, exact {} +- {}",
        straight[0].mean(),
        straight[0].standard_error(),
        exact[0].mean(),
        exact[0].standard_error()
    );
    assert!(straight[0].mean() > 0.0, "the mirror shows no sun");
    for channel in 0..3 {
        assert_eq!(straight[channel].mean(), exact[channel].mean());
        assert_eq!(
            straight[channel].standard_error(),
            exact[channel].standard_error()
        );
    }
}
