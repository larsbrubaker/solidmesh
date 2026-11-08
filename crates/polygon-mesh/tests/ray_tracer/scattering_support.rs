//! Shared helpers for the scattering test files: a seeded generator, a running standard error,
//! direction and sphere grids, and the path-tracing estimate of the integral of `f * cos`.

use std::f64::consts::PI;

use solidmesh_math::Vector3;
use solidmesh_mesh::ray_tracer::ScatteringSurface;

/// A seeded generator so runs are repeatable.
pub(crate) struct Rng(pub(crate) u64);

impl Rng {
    pub(crate) fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }

    pub(crate) fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    pub(crate) fn u3(&mut self) -> [f64; 3] {
        [self.unit(), self.unit(), self.unit()]
    }
}

/// Running mean and standard error of an estimate.
#[derive(Default, Clone, Copy)]
pub(crate) struct Stats {
    count: u64,
    sum: f64,
    sum_squares: f64,
}

impl Stats {
    pub(crate) fn add(&mut self, value: f64) {
        self.count += 1;
        self.sum += value;
        self.sum_squares += value * value;
    }

    pub(crate) fn mean(&self) -> f64 {
        self.sum / self.count as f64
    }

    pub(crate) fn standard_error(&self) -> f64 {
        let n = self.count as f64;
        let mean = self.mean();
        ((self.sum_squares / n - mean * mean).max(0.0) / n).sqrt()
    }

    /// Five standard errors of the mean.
    pub(crate) fn tolerance(&self) -> f64 {
        5.0 * self.standard_error()
    }
}

/// The unit direction with this cosine to +z at azimuth `phi`.
pub(crate) fn direction(cos_theta: f64, phi: f64) -> Vector3 {
    let sin_theta = (1.0 - cos_theta * cos_theta).max(0.0).sqrt();
    Vector3::new(sin_theta * phi.cos(), sin_theta * phi.sin(), cos_theta)
}

/// Spherical directions over `slices` azimuths and `bands` cosines from -1 to 1 by midpoints, each
/// given with its solid angle (equal for all: cosine bands have equal area).
pub(crate) fn sphere_grid(bands: usize, slices: usize) -> (Vec<Vector3>, f64) {
    let mut directions = Vec::with_capacity(bands * slices);
    for band in 0..bands {
        let cos_theta = -1.0 + 2.0 * (band as f64 + 0.5) / bands as f64;
        for slice in 0..slices {
            directions.push(direction(
                cos_theta,
                2.0 * PI * (slice as f64 + 0.5) / slices as f64,
            ));
        }
    }
    (directions, 4.0 * PI / (bands * slices) as f64)
}

/// What the integral of `f * cos` over the sphere comes to, per channel, from `n` of `sample`'s
/// draws: the path-tracing estimator, with refused draws counting zero. Refraction into a denser
/// medium carries the radiance factor `eta_wo^2 / eta_wi^2`, which is not an energy loss or gain
/// but a change of units for the radiance; `undo_radiance` removes it to measure energy.
pub(crate) fn integral_of_f_cos(
    surface: &ScatteringSurface,
    wo: Vector3,
    samples: usize,
    rng: &mut Rng,
    undo_radiance: bool,
) -> [Stats; 3] {
    let mut stats = [Stats::default(); 3];
    for _ in 0..samples {
        let event = surface.sample(wo, rng.u3());
        for (channel, stat) in stats.iter_mut().enumerate() {
            let mut value = 0.0;
            if let Some(event) = event {
                value = event.weight[channel];
                if undo_radiance && event.direction.z * wo.z < 0.0 {
                    let ratio = if wo.z > 0.0 {
                        surface.ior()
                    } else {
                        1.0 / surface.ior()
                    };
                    value *= ratio * ratio;
                }
            }
            stat.add(value);
        }
    }
    stats
}
