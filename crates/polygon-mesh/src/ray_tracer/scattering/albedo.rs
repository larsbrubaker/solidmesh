//! The precomputed directional albedo of a GGX reflection, and how it is computed. How much a
//! rough mirror reflects along a direction has no closed form, but it does not depend on the
//! color: with Schlick Fresnel `F0 + (F90 - F0)(1 - c)^5` the albedo is `F0 * A + F90 * B`, where
//! `A` and `B` are functions of the cosine and the roughness alone. `albedo_table` holds them on
//! a grid, `albedo` interpolates them bilinearly; the model uses them to keep energy where the
//! single-scatter lobe drops it. The table is generated, never edited: the
//! `generate_albedo_table` example writes it, and the scattering tests recompute it through the
//! same single-scatter lobe so it cannot go stale. Part of `scattering`.

use solidmesh_math::Vector3;

use super::albedo_table::{ALBEDO, MEAN_ALBEDO};
use super::microfacet::{
    reflect, sample_visible_normal, smith_masking, smith_masking_shadowing, MIN_ALPHA,
};

/// How many samples the albedo table has along each of its two axes.
pub const ALBEDO_TABLE_SIZE: usize = 32;

/// The table's own type: `[row by roughness][column by cosine] -> [A, B]`.
pub type AlbedoTable = [[[f64; 2]; ALBEDO_TABLE_SIZE]; ALBEDO_TABLE_SIZE];

/// The lowest cosine the table is computed at: its first column stands for this, because the
/// quadrature's formulas divide by the cosine.
const MIN_TABLE_COS: f64 = 1e-6;

/// The quadrature points per direction used to generate the table: enough that the integrand's
/// discontinuity where a reflected ray crosses the horizon costs under a thousandth.
pub const GENERATION_SUBDIVISIONS: usize = 400;

/// How many steps along the cosine go into each row's mean.
const MEAN_STEPS: usize = 4096;

/// Where a cosine falls on the table's column axis, in [0, 1], at GGX width `alpha`. A narrow lobe
/// has its masking change over `cos` of about `alpha` near the horizon, while Schlick's term rises
/// steeply over `sqrt(cos)` at any width: the axis averages one coordinate suited to each, so both
/// interpolate gently.
fn column_coordinate(cos: f64, alpha: f64) -> f64 {
    let cos = cos.clamp(0.0, 1.0);
    0.5 * cos.sqrt() + 0.5 * (cos * (1.0 + alpha) / (cos + alpha))
}

/// The cosine of column `column` of row `row`, the inverse of the axis by bisection (only the
/// generator and the tests that check its output use it; the model only maps forward).
pub fn albedo_column_cos(row: usize, column: usize) -> f64 {
    let target = column as f64 / (ALBEDO_TABLE_SIZE - 1) as f64;
    let alpha = albedo_row_alpha(row);
    let (mut low, mut high) = (0.0_f64, 1.0_f64);
    for _ in 0..80 {
        let middle = 0.5 * (low + high);
        if column_coordinate(middle, alpha) < target {
            low = middle;
        } else {
            high = middle;
        }
    }
    (0.5 * (low + high)).max(MIN_TABLE_COS)
}

/// Where a GGX width falls on the table's row axis, in [0, 1]. Rows are spaced evenly in
/// `alpha^(1/4)` (the square root of perceptual roughness) from the narrowest lobe the model
/// allows to 1: the albedo near the horizon changes fastest at the narrow end.
fn row_coordinate(alpha: f64) -> f64 {
    let lowest = MIN_ALPHA.sqrt().sqrt();
    ((alpha.max(MIN_ALPHA).sqrt().sqrt() - lowest) / (1.0 - lowest)).clamp(0.0, 1.0)
}

/// The GGX width of a table row: the inverse of `row_coordinate`.
pub fn albedo_row_alpha(row: usize) -> f64 {
    let lowest = MIN_ALPHA.sqrt().sqrt();
    let quarter_root = lowest + (1.0 - lowest) * row as f64 / (ALBEDO_TABLE_SIZE - 1) as f64;
    let root = quarter_root * quarter_root;
    (root * root).clamp(MIN_ALPHA, 1.0)
}

/// `[A, B]` by midpoint quadrature over the visible normals, using the production lobe's formulas
/// for the sampling and masking, the same integral the table holds at `subdivisions` squared
/// points. Direct on the visible normals because a narrow lobe has a peak no hemisphere grid can
/// see; there `f cos / pdf` is `G2/G1` times the Fresnel term, zero where the reflection is below
/// the horizon.
pub fn directional_albedo_by_quadrature(cos: f64, alpha: f64, subdivisions: usize) -> [f64; 2] {
    let sin = (1.0 - cos * cos).max(0.0).sqrt();
    let wo = Vector3::new(sin, 0.0, cos);
    let g1 = smith_masking(cos, alpha);
    let mut a = 0.0;
    let mut b = 0.0;
    for i in 0..subdivisions {
        for j in 0..subdivisions {
            let u = (i as f64 + 0.5) / subdivisions as f64;
            let v = (j as f64 + 0.5) / subdivisions as f64;
            let Some(h) = sample_visible_normal(wo, alpha, u, v) else {
                continue;
            };
            let wi = reflect(wo, h);
            if wi.z <= 0.0 {
                continue;
            }
            let weight = smith_masking_shadowing(cos, wi.z, alpha) / g1;
            let grazing = (1.0 - wo.dot(h).clamp(0.0, 1.0)).powi(5);
            a += weight * (1.0 - grazing);
            b += weight * grazing;
        }
    }
    let count = (subdivisions * subdivisions) as f64;
    [a / count, b / count]
}

/// `[A, B]` from `table` at this cosine and GGX width, bilinear in the column coordinate and
/// perceptual roughness. Pure indexing and arithmetic.
pub fn directional_albedo_from(table: &AlbedoTable, cos: f64, alpha: f64) -> [f64; 2] {
    let last = (ALBEDO_TABLE_SIZE - 1) as f64;
    let row = row_coordinate(alpha) * last;
    let column = column_coordinate(cos, alpha) * last;
    // Both are in [0, last], so the casts neither overflow nor saturate.
    let row_low = (row as usize).min(ALBEDO_TABLE_SIZE - 2);
    let column_low = (column as usize).min(ALBEDO_TABLE_SIZE - 2);
    let row_t = row - row_low as f64;
    let column_t = column - column_low as f64;
    let mut result = [0.0; 2];
    for (channel, value) in result.iter_mut().enumerate() {
        let near = table[row_low][column_low][channel] * (1.0 - column_t)
            + table[row_low][column_low + 1][channel] * column_t;
        let far = table[row_low + 1][column_low][channel] * (1.0 - column_t)
            + table[row_low + 1][column_low + 1][channel] * column_t;
        *value = near * (1.0 - row_t) + far * row_t;
    }
    result
}

/// `[A, B]` at this cosine and GGX width.
pub fn directional_albedo(cos: f64, alpha: f64) -> [f64; 2] {
    directional_albedo_from(&ALBEDO, cos, alpha)
}

/// The cosine-weighted mean of `[A, B]` over the hemisphere at this GGX width, interpolated in
/// roughness.
pub(super) fn mean_albedo(alpha: f64) -> [f64; 2] {
    let last = (ALBEDO_TABLE_SIZE - 1) as f64;
    let row = row_coordinate(alpha) * last;
    // In [0, last], so the cast neither overflows nor saturates.
    let row_low = (row as usize).min(ALBEDO_TABLE_SIZE - 2);
    let row_t = row - row_low as f64;
    let mut result = [0.0; 2];
    for (channel, value) in result.iter_mut().enumerate() {
        *value = MEAN_ALBEDO[row_low][channel] * (1.0 - row_t)
            + MEAN_ALBEDO[row_low + 1][channel] * row_t;
    }
    result
}

/// `2 * integral of cos * [A, B]` over the cosine, of `table` interpolated, at `row`'s width: the
/// exact mean of the interpolated function, so a furnace closes on what the model actually looks
/// up. This is what `MEAN_ALBEDO` holds.
pub fn mean_albedo_of_row(table: &AlbedoTable, row: usize) -> [f64; 2] {
    let alpha = albedo_row_alpha(row);
    let mut sum = [0.0; 2];
    for step in 0..MEAN_STEPS {
        let cos = (step as f64 + 0.5) / MEAN_STEPS as f64;
        let [a, b] = directional_albedo_from(table, cos, alpha);
        sum[0] += 2.0 * cos * a;
        sum[1] += 2.0 * cos * b;
    }
    [sum[0] / MEAN_STEPS as f64, sum[1] / MEAN_STEPS as f64]
}

/// The stored `[A, B]` of a table cell, for the tests that keep the table honest.
pub fn stored_albedo_cell(row: usize, column: usize) -> [f64; 2] {
    ALBEDO[row][column]
}

/// The stored mean of a table row, for the tests that keep the table honest.
pub fn stored_mean_albedo(row: usize) -> [f64; 2] {
    MEAN_ALBEDO[row]
}
