//! The light a path tracer's rays can reach: [`Environment`], the radiance arriving from every
//! direction no part blocks, and [`Sun`], a disc of finite angular size, with [`LightSetup`]
//! holding one of each. An environment is a constant radiance (a white furnace is the simplest
//! closed-form check a renderer has) or a smooth gradient from ground through horizon to zenith.
//!
//! World up is +Z. Radiance is linear RGB. The sun's disc adds on top of the environment.

use std::f64::consts::{PI, TAU};

use solidmesh_math::Vector3;

use crate::ray_tracer::{LocalFrame, RayTracerError};

/// `value` if it is a finite, non-negative radiance in every channel.
fn checked_radiance(value: [f64; 3], what: &'static str) -> Result<[f64; 3], RayTracerError> {
    if value
        .iter()
        .all(|channel| channel.is_finite() && *channel >= 0.0)
    {
        Ok(value)
    } else {
        Err(RayTracerError::InvalidArgument(what))
    }
}

/// `3t^2 - 2t^3`, flat at both ends, so the gradient has no visible band where it starts.
fn smoothstep(t: f64) -> f64 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

fn lerp(a: [f64; 3], b: [f64; 3], blend: f64) -> [f64; 3] {
    [
        a[0] + (b[0] - a[0]) * blend,
        a[1] + (b[1] - a[1]) * blend,
        a[2] + (b[2] - a[2]) * blend,
    ]
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum EnvironmentKind {
    Constant([f64; 3]),
    Gradient {
        zenith: [f64; 3],
        horizon: [f64; 3],
        ground: [f64; 3],
    },
}

/// The radiance arriving from any direction no part blocks.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Environment {
    kind: EnvironmentKind,
}

impl Environment {
    /// No light at all from the environment.
    pub fn black() -> Self {
        Environment {
            kind: EnvironmentKind::Constant([0.0; 3]),
        }
    }

    /// The same `radiance` from every direction. A channel that is negative or not finite is
    /// [`RayTracerError::InvalidArgument`].
    pub fn constant(radiance: [f64; 3]) -> Result<Self, RayTracerError> {
        Ok(Environment {
            kind: EnvironmentKind::Constant(checked_radiance(
                radiance,
                "an environment's radiance must be finite and not negative",
            )?),
        })
    }

    /// A gradient: `zenith` straight up (+Z), `horizon` level, `ground` straight down, easing
    /// smoothly between them. Invalid radiance is refused as in [`Environment::constant`].
    pub fn gradient(
        zenith: [f64; 3],
        horizon: [f64; 3],
        ground: [f64; 3],
    ) -> Result<Self, RayTracerError> {
        let check = "a gradient's radiances must be finite and not negative";
        Ok(Environment {
            kind: EnvironmentKind::Gradient {
                zenith: checked_radiance(zenith, check)?,
                horizon: checked_radiance(horizon, check)?,
                ground: checked_radiance(ground, check)?,
            },
        })
    }

    /// The radiance arriving along the unit `direction` (pointing away from the scene).
    pub fn radiance(&self, direction: Vector3) -> [f64; 3] {
        match self.kind {
            EnvironmentKind::Constant(radiance) => radiance,
            EnvironmentKind::Gradient {
                zenith,
                horizon,
                ground,
            } => {
                if direction.z >= 0.0 {
                    lerp(horizon, zenith, smoothstep(direction.z))
                } else {
                    lerp(horizon, ground, smoothstep(-direction.z))
                }
            }
        }
    }
}

/// The smallest solid angle a sun may have, steradians. A sun has a density of one over this for
/// sampling it, and the integrator weighs densities against each other by their squares: this
/// keeps that square finite, with room to add a surface's density to it.
const MIN_SUN_SOLID_ANGLE: f64 = 1e-150;

/// The sun: a disc of uniform radiance a finite angle wide, sampled directly.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sun {
    direction: Vector3,
    frame: LocalFrame,
    angular_radius: f64,
    radiance: [f64; 3],
    /// `1 - cos(angular_radius)` computed as `2 * sin^2(radius / 2)`, which stays accurate for a
    /// disc so small that `cos` of it rounds to 1.
    versine: f64,
    solid_angle: f64,
}

impl Sun {
    /// A sun toward `direction` (need not be unit length), `angular_radius` radians from the
    /// disc's center to its edge, in (0, pi/2], of solid angle at least [`MIN_SUN_SOLID_ANGLE`] so
    /// the weighting of its density stays finite, and of `radiance`. Anything else is
    /// [`RayTracerError::InvalidArgument`], since a sun with no size has no finite radiance.
    pub fn new(
        direction: Vector3,
        angular_radius: f64,
        radiance: [f64; 3],
    ) -> Result<Self, RayTracerError> {
        let frame = LocalFrame::from_normal(direction).ok_or(RayTracerError::InvalidArgument(
            "the sun's direction has no length",
        ))?;
        if !(angular_radius > 0.0 && angular_radius <= PI / 2.0) {
            return Err(RayTracerError::InvalidArgument(
                "the sun's angular radius must be above 0 and at most pi/2 radians",
            ));
        }
        let radiance = checked_radiance(
            radiance,
            "the sun's radiance must be finite and not negative",
        )?;
        let half_sine = libm::sin(angular_radius / 2.0);
        let versine = 2.0 * half_sine * half_sine;
        let solid_angle = TAU * versine;
        if solid_angle < MIN_SUN_SOLID_ANGLE {
            return Err(RayTracerError::InvalidArgument(
                "the sun is too small: its solid angle must be at least 1e-150 steradians (an angular radius of about 5.7e-76 radians)",
            ));
        }
        Ok(Sun {
            direction: frame.normal,
            frame,
            angular_radius,
            radiance,
            versine,
            solid_angle,
        })
    }

    /// The unit direction toward the sun's center.
    pub fn direction(&self) -> Vector3 {
        self.direction
    }

    /// The angle from the disc's center to its edge, radians.
    pub fn angular_radius(&self) -> f64 {
        self.angular_radius
    }

    /// The disc's radiance, linear RGB.
    pub fn radiance(&self) -> [f64; 3] {
        self.radiance
    }

    /// The solid angle of the disc, steradians.
    pub fn solid_angle(&self) -> f64 {
        self.solid_angle
    }

    /// The density of [`Sun::sample`], per steradian.
    pub fn pdf(&self) -> f64 {
        1.0 / self.solid_angle
    }

    /// Whether the unit `direction` lands on the disc.
    pub fn contains(&self, direction: Vector3) -> bool {
        // The squared chord between two unit vectors is `2 - 2cos(angle)`, which unlike the dot
        // product does not lose precision for a very small disc.
        (direction - self.direction).length_squared() <= 2.0 * self.versine
    }

    /// A direction drawn uniformly from the disc, from two random numbers in `[0, 1)`.
    pub fn sample(&self, u: [f64; 2]) -> Vector3 {
        let height = u[0] * self.versine;
        // `sin(t)` from `1 - cos(t)`, again accurate when the disc is tiny.
        let sin_angle = (height * (2.0 - height)).max(0.0).sqrt();
        let azimuth = TAU * u[1];
        self.frame.to_world(Vector3::new(
            sin_angle * libm::cos(azimuth),
            sin_angle * libm::sin(azimuth),
            1.0 - height,
        ))
    }
}

/// All the light in a scene.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LightSetup {
    pub environment: Environment,
    pub sun: Option<Sun>,
}

impl LightSetup {
    /// An environment and no sun.
    pub fn environment_only(environment: Environment) -> Self {
        LightSetup {
            environment,
            sun: None,
        }
    }

    /// A sun with no environment.
    pub fn sun_only(sun: Sun) -> Self {
        LightSetup {
            environment: Environment::black(),
            sun: Some(sun),
        }
    }
}
