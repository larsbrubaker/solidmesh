//! The sun's straight line from a floor vertex a see-through part rests on: a part less than
//! fully there resting on the floor, in sun, with clear glass elsewhere so `SunThroughGlass`'s
//! straight line is in use, renders as the same part lifted a modelled gap off the floor, by a
//! paired difference of the two renders (`paired_against_lifted_with`), under Straight and under
//! Exact. The views cover every step that moves a path across the contact without a trace: a
//! line landing on the floor past the part (`onto_rest`), see-through glass found there leaving
//! through its resting face onto the floor, a hit on the floor inside the footprint taken as the
//! part's face (`met_first`), and the floor's bounce up meeting the part across the contact. So
//! each would part the renders if the sun line from such a vertex, or whether the path travels
//! inside glass after it, differed from the lifted part's.

use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::ray_tracer::{
    Environment, LightSetup, PathTraceSettings, PlacedPart, PlacedParts, RenderScene,
    ScatteringSurface, Sun, SunThroughGlass,
};

use crate::glass_on_floor_tests::one_box;
use crate::glass_tray_support::{box_meshes, part_of};
use crate::scattering_support::Stats;
use crate::see_through_glass_on_floor_tests::paired_against_lifted_with;
use crate::see_through_on_floor_tests::wide_floor;

/// A white sky over a blue ground, and a sun 30 degrees off straight up toward +x, 2.5 degrees
/// across, ten times the sky's radiance over its disc.
fn sky_and_sun() -> LightSetup {
    let radius = 2.5_f64.to_radians();
    let direction = Vector3::new(30_f64.to_radians().sin(), 0.0, 30_f64.to_radians().cos());
    let radiance = 10.0 / (radius.sin() * radius.sin());
    LightSetup {
        environment: Environment::gradient([1.0; 3], [1.0; 3], [0.0, 0.0, 1.0]).unwrap(),
        sun: Some(Sun::new(direction, radius, [radiance; 3]).unwrap()),
    }
}

/// The wide matte floor, a box of side 20 of `surface` at `opacity` lifted `lift` off it, and a
/// fully there clear glass box of side 10 standing 5 above the floor out of the view's middle.
fn scene(surface: ScatteringSurface, opacity: f64, lift: f64) -> RenderScene {
    let part = part_of(&box_meshes(&one_box(lift))[0], Matrix4X4::IDENTITY);
    let elsewhere = part_of(
        &box_meshes(&[([10.0; 3], [40.0, 0.0, 10.0])])[0],
        Matrix4X4::IDENTITY,
    );
    let floor = part_of(&wide_floor(), Matrix4X4::IDENTITY);
    let parts: Vec<PlacedPart> = vec![part, elsewhere, floor];
    let surfaces = vec![
        surface,
        ScatteringSurface::glass(),
        ScatteringSurface::matte_plastic([0.3; 3]),
    ];
    RenderScene::new(PlacedParts::new(parts), surfaces, sky_and_sun())
        .unwrap()
        .with_part_opacities(vec![opacity, 1.0, 1.0])
        .unwrap()
}

fn settings(sun_through_glass: SunThroughGlass) -> PathTraceSettings {
    PathTraceSettings {
        sun_through_glass,
        ..PathTraceSettings::default()
    }
}

/// Every case of `surface` at a quarter, half and three quarters there under `mode`, measured
/// before any is judged, as failures.
fn resting_against_lifted(
    what: &str,
    surface: ScatteringSurface,
    mode: SunThroughGlass,
) -> Vec<String> {
    let mut failures = Vec::new();
    for opacity in [0.25, 0.5, 0.75] {
        let paired = paired_against_lifted_with(
            &scene(surface, opacity, 0.0),
            &scene(surface, opacity, 0.01),
            settings(mode),
        );
        let case = format!("{what} at opacity {opacity} under {mode:?}");
        let means = paired.difference.each_ref().map(Stats::mean);
        println!("{case}: resting minus lifted {means:?}");
        for (channel, difference, bound) in paired.disagreements() {
            failures.push(format!(
                "{case}, channel {channel}: resting minus lifted {difference} (bound {bound})"
            ));
        }
    }
    failures
}

#[test]
fn a_half_there_matte_box_on_the_floor_in_sun_renders_as_one_lifted() {
    let red = ScatteringSurface::matte_plastic([0.8, 0.3, 0.2]);
    let mut failures = resting_against_lifted("matte", red, SunThroughGlass::Straight);
    failures.extend(resting_against_lifted("matte", red, SunThroughGlass::Exact));
    assert!(failures.is_empty(), "{failures:#?}");
}

#[test]
fn half_there_glass_on_the_floor_in_sun_renders_as_glass_lifted() {
    let clear = ScatteringSurface::glass();
    let mut failures = resting_against_lifted("glass", clear, SunThroughGlass::Straight);
    failures.extend(resting_against_lifted(
        "glass",
        clear,
        SunThroughGlass::Exact,
    ));
    assert!(failures.is_empty(), "{failures:#?}");
}
