//! Sunlight through smooth glass onto a matte floor, against its closed form: a parallel-sided
//! pane passes the sun straight through both faces, `(1 - F)^2` (the light that went back and
//! forth inside the pane is left out, `SunThroughGlass::Straight`), the floor returns it
//! diffusely, and the pane sends part of that back down with its whole reflection series. The
//! render must reach that value, apart from the series' `(1 - F) / (1 + F)` it leaves out, and
//! reach it with little noise, which needs the sun found by a shadow ray that passes through the
//! pane rather than only by the rare bounce that happens to land in the sun's disc.

use std::f64::consts::PI;
use std::sync::Arc;

use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::ray_tracer::{
    LightSetup, MeshSurface, PathTraceSettings, PlacedPart, PlacedParts, RenderCamera, RenderScene,
    ScatteringSurface, Sun,
};
use solidmesh_mesh::solids::platonic_solids::create_cube;

use crate::path_tracer_support::{estimate, flat_patch, SHARP_CREASE};
use crate::surface_hit_support::tree_of;

const IOR: f64 = 1.5;
const ALBEDO: f64 = 0.3;

/// The Fresnel reflectance of unpolarized light meeting glass of `IOR` from air at `cos_i`.
fn fresnel(cos_i: f64) -> f64 {
    let sin2_t = (1.0 - cos_i * cos_i) / (IOR * IOR);
    let cos_t = (1.0 - sin2_t).sqrt();
    let perpendicular = (cos_i - IOR * cos_t) / (cos_i + IOR * cos_t);
    let parallel = (IOR * cos_i - cos_t) / (IOR * cos_i + cos_t);
    0.5 * (perpendicular * perpendicular + parallel * parallel)
}

/// A pane's reflectance and transmittance at `cos_i`, every internal round trip summed: the
/// inside faces reflect `F` too, so `R = 2F / (1 + F)` and `T = (1 - F) / (1 + F)`.
fn pane(cos_i: f64) -> (f64, f64) {
    let f = fresnel(cos_i);
    (2.0 * f / (1.0 + f), (1.0 - f) / (1.0 + f))
}

/// `integrand(theta)` integrated over [0, `upper`] by the midpoint rule.
fn integrate(upper: f64, integrand: impl Fn(f64) -> f64) -> f64 {
    let steps = 20_000;
    let step = upper / f64::from(steps);
    (0..steps)
        .map(|i| integrand((f64::from(i) + 0.5) * step) * step)
        .sum()
}

#[test]
fn sunlight_through_a_glass_pane_lights_the_floor_smoothly() {
    let radius = 2.5_f64.to_radians();
    // Radiance making the unshaded floor's irradiance pi, so its radiance is its albedo.
    let sun_radiance = 1.0 / (radius.sin() * radius.sin());
    let sun = Sun::new(Vector3::UNIT_Z, radius, [sun_radiance; 3]).unwrap();

    // The floor at z = 0, the pane from z = 1 to z = 2, both far wider than the patch seen.
    let floor = flat_patch(1000.0, 0.0);
    let glass = create_cube(2000.0, 2000.0, 1.0);
    let part = |mesh: &solidmesh_mesh::Mesh, matrix| {
        PlacedPart::new(
            tree_of(mesh),
            Arc::new(MeshSurface::new(mesh, SHARP_CREASE).unwrap()),
            matrix,
        )
        .unwrap()
    };
    let scene = RenderScene::new(
        PlacedParts::new(vec![
            part(&floor, Matrix4X4::IDENTITY),
            part(&glass, Matrix4X4::create_translation(0.0, 0.0, 1.5)),
        ]),
        vec![
            ScatteringSurface::new([ALBEDO; 3], 0.0, 1.0, 0.0, 1.0),
            ScatteringSurface::new([1.0; 3], 0.0, 0.0, 1.0, IOR),
        ],
        LightSetup::sun_only(sun),
    )
    .unwrap();
    // Between the floor and the pane, looking straight down, so the view crosses no glass.
    let camera = RenderCamera::orthographic(
        Vector3::new(0.0, 0.0, 0.5),
        Vector3::ZERO,
        Vector3::UNIT_Y,
        2.0,
    )
    .unwrap();

    // The irradiance through the pane: the sun's disc, cosine weighted, each direction passed
    // by both faces' transmission (a parallel pane leaves directions unchanged), and what the
    // whole series would pass, which Straight leaves out.
    let disc = |passed: &dyn Fn(f64) -> f64| {
        sun_radiance
            * integrate(radius, |theta| {
                passed(theta.cos()) * theta.cos() * theta.sin() * 2.0 * PI
            })
    };
    let direct = disc(&|cos| (1.0 - fresnel(cos)).powi(2));
    let series = disc(&|cos| pane(cos).1);
    // The share of the floor's diffuse light the pane sends back down.
    let returned = integrate(PI / 2.0, |theta| {
        pane(theta.cos()).0 * 2.0 * theta.cos() * theta.sin()
    });
    let expected = ALBEDO / PI * direct / (1.0 - ALBEDO * returned);

    let samples = 1024;
    let (stats, work) = estimate(
        &scene,
        camera,
        (4, 4),
        PathTraceSettings::default(),
        samples,
    );
    let (mean, error) = (stats[0].mean(), stats[0].standard_error());
    let count = 16 * samples;
    println!(
        "pane: {mean} +- {error} against {expected} (direct {direct}, returned {returned}); \
         {} shadow rays for {count} samples",
        work.first_hit_shadow_rays + work.later_shadow_rays
    );
    // (a) The right light: five standard errors, plus 0.5% for the pane's finite width.
    let allowed = 5.0 * error + 0.005 * expected;
    assert!(
        (mean - expected).abs() <= allowed,
        "the floor under the pane is {mean} +- {error}, the closed form {expected} \
         (allowed {allowed})"
    );
    // (c) Only the straight line: the floor's own sun, one scattering event, sits at
    // `(1 - F)^2` to well under the 0.16% the series would add.
    let direct_only = PathTraceSettings {
        max_bounces: 1,
        russian_roulette_after: None,
        ..PathTraceSettings::default()
    };
    let (first, _) = estimate(&scene, camera, (4, 4), direct_only, 256);
    let (first_mean, first_error) = (first[0].mean(), first[0].standard_error());
    let straight = ALBEDO / PI * direct;
    let gap = ALBEDO / PI * series - straight;
    let allowed_first = 5.0 * first_error + 1e-5 * straight;
    println!(
        "pane, one event: {first_mean} +- {first_error} against {straight} (the series {})",
        straight + gap
    );
    assert!(
        allowed_first < 0.5 * gap,
        "the check can't tell the series from the straight line: allowed {allowed_first}, gap {gap}"
    );
    assert!(
        (first_mean - straight).abs() <= allowed_first,
        "the floor's own sun under the pane is {first_mean} +- {first_error}, the straight line's \
         {straight} (allowed {allowed_first})"
    );
    // (b) Reached smoothly: with a shadow ray through the pane every sample sees the sun, so
    // the pixel mean's standard error at these samples is well under 1% of the mean.
    assert!(
        error < 0.01 * mean,
        "the floor under the pane is {mean} +- {error} over {count} samples: {:.1}% noise, \
         where under 1% is wanted",
        100.0 * error / mean
    );
}
