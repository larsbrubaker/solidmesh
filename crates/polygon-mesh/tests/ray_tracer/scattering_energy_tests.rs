//! The scattering model's energy, measured: nothing gains energy, a white surface in a white
//! furnace returns all of it, `sample`'s mean weight is the integral of `f * cos`, and the
//! precomputed albedo table matches the production single-scatter lobe.
//!
//! Tolerances are five standard errors of each estimate plus the named margin for the quadrature
//! or the table's interpolation: the fixed sample counts make the runs repeatable.

use std::f64::consts::PI;

use crate::scattering_support::{direction, integral_of_f_cos, sphere_grid, Rng, Stats};
use solidmesh_math::Vector3;
use solidmesh_mesh::ray_tracer::{
    albedo_column_cos, albedo_row_alpha, directional_albedo, directional_albedo_by_quadrature,
    mean_albedo_of_row, stored_albedo_cell, stored_mean_albedo, AlbedoTable, ScatteringSurface,
    ALBEDO_TABLE_SIZE, GENERATION_SUBDIVISIONS,
};

const WHITE: [f64; 3] = [1.0, 1.0, 1.0];
const ROUGHNESSES: [f64; 6] = [0.0, 0.1, 0.3, 0.6, 0.8, 1.0];
const COSINES: [f64; 6] = [0.02, 0.1, 0.3, 0.5, 0.9, 1.0];
/// Indices of refraction, deliberately including the range from 1 to 1.33 where the dielectric's
/// grazing reflectance scales with `F0`.
const IORS: [f64; 6] = [1.0, 1.1, 1.2, 1.33, 1.5, 2.5];

/// The margin the table's bilinear interpolation of a steep function can cost at the grid's worst
/// spot, above the sampling error: a white surface in a white furnace comes out this far from 1.
const FURNACE_TABLE_MARGIN: f64 = 6e-3;

#[test]
fn no_surface_gains_energy() {
    let mut rng = Rng(0xfeed_0001);
    let samples = 16_384;
    let mut surfaces = Vec::new();
    for &roughness in &ROUGHNESSES {
        surfaces.push(ScatteringSurface::new(WHITE, 1.0, roughness, 0.0, 1.5));
        for &ior in &IORS {
            for &metallic in &[0.0, 0.4] {
                surfaces.push(ScatteringSurface::new(WHITE, metallic, roughness, 0.0, ior));
            }
            for &transmission in &[0.5, 1.0] {
                surfaces.push(ScatteringSurface::new(
                    WHITE,
                    0.0,
                    roughness,
                    transmission,
                    ior,
                ));
            }
        }
    }
    let mut worst: (f64, String) = (0.0, String::new());
    for surface in &surfaces {
        for &cos_theta in COSINES.iter().chain(&[-0.3, -0.9]) {
            if cos_theta < 0.0 && surface.transmission() == 0.0 {
                continue;
            }
            let wo = direction(cos_theta, 1.0);
            let stats = integral_of_f_cos(surface, wo, samples, &mut rng, true);
            for (channel, stat) in stats.iter().enumerate() {
                let bound = 1.0 + stat.tolerance() + FURNACE_TABLE_MARGIN;
                assert!(
                    stat.mean() <= bound,
                    "{surface:?} at cos {cos_theta}, channel {channel}: returned {} > {bound}",
                    stat.mean()
                );
                if stat.mean() > worst.0 {
                    worst = (stat.mean(), format!("{surface:?} at cos {cos_theta}"));
                }
            }
        }
    }
    println!("worst integral of f*cos: {:.5} for {}", worst.0, worst.1);
}

/// The worst `|mean - 1|` so far: `(deviation, description)`.
type Worst = (f64, String);

/// Asserts `mean` is `1` to within tolerance, in both directions, tracks the worst.
fn assert_furnace_closes(stats: &[Stats; 3], what: &str, worst: &mut Worst) {
    for stat in stats {
        let deviation = (stat.mean() - 1.0).abs();
        let bound = stat.tolerance() + FURNACE_TABLE_MARGIN;
        assert!(deviation <= bound, "{what}: returned {}", stat.mean());
        if deviation > worst.0 {
            *worst = (deviation, what.to_string());
        }
    }
}

#[test]
fn a_white_plastic_furnace_returns_all_of_its_light() {
    let mut rng = Rng(0xfeed_0005);
    let samples = 32_768;
    let mut worst: Worst = (0.0, String::new());
    for &ior in &IORS {
        for &roughness in &ROUGHNESSES {
            let plastic = ScatteringSurface::new(WHITE, 0.0, roughness, 0.0, ior);
            for &cos_theta in &COSINES {
                let wo = direction(cos_theta, 0.2);
                let stats = integral_of_f_cos(&plastic, wo, samples, &mut rng, false);
                let what = format!("ior {ior} roughness {roughness} cos {cos_theta}");
                assert_furnace_closes(&stats, &what, &mut worst);
            }
        }
    }
    println!(
        "worst white plastic furnace deviation: {:.5} at {}",
        worst.0, worst.1
    );
    let matte = ScatteringSurface::matte_plastic(WHITE);
    for &cos_theta in &[0.1, 0.3, 0.5] {
        let wo = direction(cos_theta, 0.2);
        let stats = integral_of_f_cos(&matte, wo, samples, &mut rng, false);
        println!(
            "matte plastic furnace cos {cos_theta}: {:.4}",
            stats[0].mean()
        );
    }
}

#[test]
fn a_white_metal_furnace_returns_all_of_its_light() {
    let mut rng = Rng(0xfeed_0006);
    let samples = 64_000;
    let mut worst: Worst = (0.0, String::new());
    for step in 0..=10 {
        let roughness = step as f64 / 10.0;
        let metal = ScatteringSurface::new(WHITE, 1.0, roughness, 0.0, 1.5);
        for &cos_theta in &[1.0, 0.5, 0.1, 0.02] {
            let wo = direction(cos_theta, 0.2);
            let stats = integral_of_f_cos(&metal, wo, samples, &mut rng, false);
            let what = format!("roughness {roughness} cos {cos_theta}");
            println!("metal furnace {what}: {:.4}", stats[0].mean());
            assert_furnace_closes(&stats, &what, &mut worst);
        }
    }
    println!(
        "worst white metal furnace deviation: {:.5} at {}",
        worst.0, worst.1
    );
}

#[test]
fn a_colored_metal_does_not_exceed_its_color_at_normal_incidence() {
    let mut rng = Rng(0xfeed_0007);
    let samples = 64_000;
    let wo = Vector3::new(0.0, 0.0, 1.0);
    let colors: [[f64; 3]; 3] = [[1.0, 0.78, 0.34], [0.95, 0.64, 0.54], [0.56, 0.57, 0.58]];
    for base in colors {
        for &roughness in &[0.0, 0.2, 0.4, 0.6, 0.8, 1.0] {
            let metal = ScatteringSurface::new(base, 1.0, roughness, 0.0, 1.5);
            let stats = integral_of_f_cos(&metal, wo, samples, &mut rng, false);
            for (channel, stat) in stats.iter().enumerate() {
                println!(
                    "metal {base:?} roughness {roughness}: channel {channel} returns {:.4}",
                    stat.mean()
                );
                assert!(
                    stat.mean() <= base[channel] + stat.tolerance() + 1e-3,
                    "{base:?} roughness {roughness}: channel {channel} returned {} over {}",
                    stat.mean(),
                    base[channel]
                );
            }
        }
    }
    // A channel with almost no color is lifted by the facets that reflect at grazing, which is
    // real Fresnel behavior; it still stays dark.
    let saturated = ScatteringSurface::new([0.9, 0.01, 0.001], 1.0, 1.0, 0.0, 1.5);
    let stats = integral_of_f_cos(&saturated, wo, samples, &mut rng, false);
    println!(
        "saturated metal at roughness 1: {:.4} {:.4} {:.4}",
        stats[0].mean(),
        stats[1].mean(),
        stats[2].mean()
    );
    assert!(stats[1].mean() <= 0.05, "{}", stats[1].mean());
    assert!(stats[2].mean() <= 0.05, "{}", stats[2].mean());
}

#[test]
fn low_roughness_glass_returns_all_of_its_light() {
    let mut rng = Rng(0xfeed_0009);
    let samples = 200_000;
    for &ior in &[1.002, 1.05, 1.33, 1.5] {
        for &cos_theta in &[0.9, -0.9] {
            let glass = ScatteringSurface::new(WHITE, 0.0, 0.1, 1.0, ior);
            let wo = direction(cos_theta, 0.4);
            let stats = integral_of_f_cos(&glass, wo, samples, &mut rng, true);
            for stat in &stats {
                println!("glass ior {ior} cos {cos_theta}: {:.5}", stat.mean());
                assert!(
                    (stat.mean() - 1.0).abs() <= stat.tolerance() + 2e-3,
                    "ior {ior} cos {cos_theta}: returned {}",
                    stat.mean()
                );
            }
        }
    }
}

#[test]
fn rough_glass_only_loses_its_single_scatter_energy() {
    let mut rng = Rng(0xfeed_000a);
    let samples = 200_000;
    let mut report = Vec::new();
    for &roughness in &[0.2, 0.4, 0.6, 0.8, 1.0] {
        let glass = ScatteringSurface::new(WHITE, 0.0, roughness, 1.0, 1.5);
        let wo = direction(0.9, 0.0);
        let stats = integral_of_f_cos(&glass, wo, samples, &mut rng, true);
        report.push(format!("rough glass r {roughness}: {:.4}", stats[0].mean()));
        assert!(stats[0].mean() <= 1.0 + stats[0].tolerance() + 1e-3);
        // Uncompensated, but it should not lose wildly more than the reflective lobe does.
        assert!(
            stats[0].mean() >= 0.45,
            "roughness {roughness}: {}",
            stats[0].mean()
        );
    }
    println!("{}", report.join("\n"));
}

#[test]
fn the_mean_weight_of_sample_is_the_integral_of_f_cos() {
    let mut rng = Rng(0xfeed_0003);
    let samples = 200_000;
    let (sphere, cell) = sphere_grid(720, 360);
    let surfaces = [
        ScatteringSurface::new(WHITE, 1.0, 0.5, 0.0, 1.5),
        ScatteringSurface::new([0.9, 0.5, 0.2], 1.0, 0.8, 0.0, 1.5),
        ScatteringSurface::new([0.7, 0.4, 0.2], 0.0, 0.5, 0.0, 1.5),
        ScatteringSurface::new(WHITE, 0.0, 0.8, 0.0, 2.0),
        ScatteringSurface::new(WHITE, 0.5, 0.7, 0.0, 1.33),
        ScatteringSurface::new(WHITE, 0.0, 0.6, 1.0, 1.5),
        ScatteringSurface::new(WHITE, 0.0, 0.8, 0.4, 1.33),
    ];
    for surface in surfaces {
        for &cos_theta in &[0.15, 0.6, 0.95, -0.5] {
            if cos_theta < 0.0 && surface.transmission() == 0.0 {
                continue;
            }
            let wo = direction(cos_theta, 0.3);
            let stats = integral_of_f_cos(&surface, wo, samples, &mut rng, false);
            for (channel, stat) in stats.iter().enumerate() {
                let mut integral = 0.0;
                for &wi in &sphere {
                    integral += surface.evaluate(wo, wi)[channel] * wi.z.abs() * cell;
                }
                let tolerance = stat.tolerance() + 4e-3;
                assert!(
                    (stat.mean() - integral).abs() <= tolerance,
                    "{surface:?} cos {cos_theta} channel {channel}: mean weight {} against \
                     integral {integral}",
                    stat.mean()
                );
            }
        }
    }
}

#[test]
fn pdf_integrates_to_the_mass_of_the_continuous_lobes() {
    let mut rng = Rng(0xfeed_000b);
    let draws = 200_000;
    let (sphere, cell) = sphere_grid(720, 360);
    let surfaces = [
        // Nothing but the cosine lobe.
        (
            ScatteringSurface::new([0.5, 0.5, 0.5], 0.0, 0.5, 0.0, 1.0),
            0.0,
        ),
        (
            ScatteringSurface::new([0.5, 0.5, 0.5], 0.3, 0.0, 0.0, 1.0),
            1e-9,
        ),
        // Smooth plastic: a delta lobe, and a cosine lobe that integrates to what the delta leaves.
        (ScatteringSurface::matte_plastic(WHITE), 0.0),
        (ScatteringSurface::new(WHITE, 0.0, 0.0, 0.0, 1.5), 0.0),
        // A rough metal: the cosine multiple-scatter lobe beside the GGX lobe.
        (ScatteringSurface::new(WHITE, 1.0, 0.8, 0.0, 1.5), 0.0),
    ];
    for (surface, _refusal) in surfaces {
        for &cos_theta in &[0.2, 0.6, 1.0] {
            let wo = direction(cos_theta, 0.9);
            let mut integral = 0.0;
            for &wi in &sphere {
                integral += surface.pdf(wo, wi) * cell;
            }
            let mut continuous = Stats::default();
            for _ in 0..draws {
                let value = match surface.sample(wo, rng.u3()) {
                    Some(event) if !event.is_specular => 1.0,
                    _ => 0.0,
                };
                continuous.add(value);
            }
            let tolerance = continuous.tolerance() + 3e-3;
            assert!(
                (integral - continuous.mean()).abs() <= tolerance,
                "{surface:?} cos {cos_theta}: pdf integrates to {integral}, drawn {}",
                continuous.mean()
            );
        }
    }
}

/// With roughness 1 the GGX density is constant, and at normal incidence the albedo of a white
/// metal reduces to `1/2 * integral of 2c/(1+c) dc`, which is `1 - ln 2`. Integrates the
/// single-scatter term directly, so this checks the formulas with neither the sampler nor the
/// albedo table involved.
#[test]
fn single_scatter_at_full_roughness_has_its_closed_form() {
    let metal = ScatteringSurface::new(WHITE, 1.0, 1.0, 0.0, 1.5);
    let wo = Vector3::new(0.0, 0.0, 1.0);
    let (bands, slices) = (2000, 64);
    let mut integral = 0.0;
    for band in 0..bands {
        let cos_i = (band as f64 + 0.5) / bands as f64;
        for slice in 0..slices {
            let wi = direction(cos_i, 2.0 * PI * (slice as f64 + 0.5) / slices as f64);
            integral += metal.evaluate_single_scatter(wo, wi)[0] * cos_i;
        }
    }
    integral *= 2.0 * PI / (bands * slices) as f64;
    let expected = 1.0 - std::f64::consts::LN_2;
    assert!(
        (integral - expected).abs() < 1e-4,
        "{integral} against {expected}"
    );
}

/// Stored cells against the generator, exactly, on a sample of cells at the generator's own
/// quadrature: the table must be exactly what the checked-in code computes. The cheapest full
/// pass of every cell at a coarse quadrature is below.
#[test]
fn the_albedo_table_is_what_the_generator_computes() {
    for &row in &[0, 5, 15, 31] {
        for &column in &[0, 11, 31] {
            let cell = directional_albedo_by_quadrature(
                albedo_column_cos(row, column),
                albedo_row_alpha(row),
                GENERATION_SUBDIVISIONS,
            );
            let stored = stored_albedo_cell(row, column);
            for (computed, stored) in cell.iter().zip(stored) {
                assert!(
                    (computed - stored).abs() < 1e-13,
                    "row {row} column {column}: table is stale, {stored} against {computed}; \
                     rerun the generate_albedo_table example"
                );
            }
        }
    }
}

/// Every cell of the table against the production lobe at a coarser quadrature: a stale table
/// anywhere shows up here. 64 by 64 points cost about 3e-3 of accuracy against the horizon's
/// discontinuity at the worst cell.
#[test]
fn every_albedo_table_cell_matches_the_production_lobe() {
    for row in 0..ALBEDO_TABLE_SIZE {
        for column in 0..ALBEDO_TABLE_SIZE {
            let cell = directional_albedo_by_quadrature(
                albedo_column_cos(row, column),
                albedo_row_alpha(row),
                64,
            );
            let stored = stored_albedo_cell(row, column);
            for (computed, stored) in cell.iter().zip(stored) {
                assert!(
                    (computed - stored).abs() < 8e-3,
                    "row {row} column {column}: {stored} against {computed}"
                );
            }
        }
    }
}

#[test]
fn the_mean_albedo_table_is_the_mean_of_the_directional_table() {
    let mut table: AlbedoTable = [[[0.0; 2]; ALBEDO_TABLE_SIZE]; ALBEDO_TABLE_SIZE];
    for (row, cells) in table.iter_mut().enumerate() {
        for (column, cell) in cells.iter_mut().enumerate() {
            *cell = stored_albedo_cell(row, column);
        }
    }
    for row in 0..ALBEDO_TABLE_SIZE {
        let computed = mean_albedo_of_row(&table, row);
        let stored = stored_mean_albedo(row);
        assert!(
            (computed[0] - stored[0]).abs() < 1e-13 && (computed[1] - stored[1]).abs() < 1e-13,
            "row {row}: {stored:?} against {computed:?}"
        );
    }
}

/// The table against `evaluate_single_scatter` integrated on a hemisphere grid: white metal
/// (`F0 = F90 = 1`) is `A + B` and black metal (`F0 = 0`, `F90 = 1`) is `B`. A route to the same
/// number through the BSDF rather than the visible-normal quadrature the table was built from.
#[test]
fn the_albedo_table_matches_the_single_scatter_lobe_on_a_hemisphere() {
    let (bands, slices) = (600, 720);
    for &roughness in &[0.5, 0.8, 1.0] {
        let alpha = roughness * roughness;
        let white = ScatteringSurface::new(WHITE, 1.0, roughness, 0.0, 1.5);
        let black = ScatteringSurface::new([0.0; 3], 1.0, roughness, 0.0, 1.5);
        for &cos_theta in &[0.3, 0.6, 1.0] {
            let wo = direction(cos_theta, 0.0);
            let mut white_integral = 0.0;
            let mut black_integral = 0.0;
            for band in 0..bands {
                let cos_i = (band as f64 + 0.5) / bands as f64;
                for slice in 0..slices {
                    let wi = direction(cos_i, 2.0 * PI * (slice as f64 + 0.5) / slices as f64);
                    white_integral += white.evaluate_single_scatter(wo, wi)[0] * cos_i;
                    black_integral += black.evaluate_single_scatter(wo, wi)[0] * cos_i;
                }
            }
            let scale = 2.0 * PI / (bands * slices) as f64;
            let [a, b] = directional_albedo(cos_theta, alpha);
            let what = format!("roughness {roughness} cos {cos_theta}");
            assert!(
                (white_integral * scale - (a + b)).abs() < 6e-3,
                "{what}: white {} against {}",
                white_integral * scale,
                a + b
            );
            assert!(
                (black_integral * scale - b).abs() < 6e-3,
                "{what}: black {} against {b}",
                black_integral * scale
            );
        }
    }
}
