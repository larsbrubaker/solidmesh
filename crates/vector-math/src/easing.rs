//! Easing curves (functions from Tween.js, MIT): the ease types MatterCAD animates with, their
//! In/Out/InOut variants, and a bisection inverse.
//!
//! Each family is a module (`easing::quadratic::in_`, since `in` is a Rust keyword). The
//! constants are single-precision values (`0.1`, `1.70158`, `1 / 2.75` as f32) widened to f64
//! (`f32_const`), so the curves return the same doubles the animations and saved settings
//! were tuned with. `EaseType`/`EaseOption` serialize as their names, the form saved files hold.

use serde::{Deserialize, Serialize};

/// The curve family.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum EaseType {
    #[default]
    Linear,
    Quadratic,
    Cubic,
    Quartic,
    Quintic,
    Sinusoidal,
}

/// Which end of the curve eases.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum EaseOption {
    #[default]
    In,
    Out,
    InOut,
}

/// A single-precision constant as the double it widens to.
const fn f32_const(value: f32) -> f64 {
    value as f64
}

/// Iteratively solves for the input that gives `output` (0 to 1) from a monotonic `function`;
/// stops once within `error_tolerance` (usually `.000000001`) or after 128 halvings.
/// Outputs outside [0, 1] answer 0.
pub fn calculate_inverse_fn(
    function: impl Fn(f64) -> f64,
    output: f64,
    error_tolerance: f64,
) -> f64 {
    if (0.0..=1.0).contains(&output) {
        let mut start = 0.0;
        let mut end = 1.0;
        let mut mid = 0.5;
        let mut iterations = 0;
        let mut error = 1.0;
        while error > error_tolerance && iterations < 128 {
            iterations += 1;
            let delta = end - start;
            mid = start + delta / 2.0;
            let test_output = function(mid);
            error = (output - test_output).abs();
            if error <= error_tolerance {
                return mid;
            }

            if test_output > output {
                end = mid;
            } else {
                start = mid;
            }
        }

        return mid;
    }

    0.0
}

/// The curve for a type and option; types without a curve of their own are linear.
pub fn get_easing_function(ease_type: EaseType, ease_option: EaseOption) -> fn(f64) -> f64 {
    use EaseOption::*;
    match (ease_type, ease_option) {
        (EaseType::Quadratic, In) => quadratic::in_,
        (EaseType::Quadratic, Out) => quadratic::out,
        (EaseType::Quadratic, InOut) => quadratic::in_out,
        (EaseType::Cubic, In) => cubic::in_,
        (EaseType::Cubic, Out) => cubic::out,
        (EaseType::Cubic, InOut) => cubic::in_out,
        (EaseType::Quartic, In) => quartic::in_,
        (EaseType::Quartic, Out) => quartic::out,
        (EaseType::Quartic, InOut) => quartic::in_out,
        (EaseType::Quintic, In) => quintic::in_,
        (EaseType::Quintic, Out) => quintic::out,
        (EaseType::Quintic, InOut) => quintic::in_out,
        (EaseType::Sinusoidal, In) => sinusoidal::in_,
        (EaseType::Sinusoidal, Out) => sinusoidal::out,
        (EaseType::Sinusoidal, InOut) => sinusoidal::in_out,
        (EaseType::Linear, _) => linear,
    }
}

/// The curve's value at `k`.
pub fn calculate(ease_type: EaseType, ease_option: EaseOption, k: f64) -> f64 {
    get_easing_function(ease_type, ease_option)(k)
}

/// Given the curve's output `k`, the approximate input that produces it (the usual
/// `error_tolerance` is [`DEFAULT_INVERSE_TOLERANCE`]).
pub fn calculate_inverse(
    ease_type: EaseType,
    ease_option: EaseOption,
    k: f64,
    error_tolerance: f64,
) -> f64 {
    if ease_type == EaseType::Linear {
        return k;
    }

    calculate_inverse_fn(
        get_easing_function(ease_type, ease_option),
        k,
        error_tolerance,
    )
}

/// The default tolerance of [`calculate_inverse`].
pub const DEFAULT_INVERSE_TOLERANCE: f64 = 0.000000001;

pub fn linear(k: f64) -> f64 {
    k
}

pub mod quadratic {
    pub fn in_(k: f64) -> f64 {
        k * k
    }

    pub fn out(k: f64) -> f64 {
        k * (2.0 - k)
    }

    pub fn in_out(k: f64) -> f64 {
        let k = k * 2.0;
        if k < 1.0 {
            return 0.5 * k * k;
        }

        let k = k - 1.0;
        -0.5 * (k * (k - 2.0) - 1.0)
    }
}

pub mod cubic {
    pub fn in_(k: f64) -> f64 {
        k * k * k
    }

    pub fn out(k: f64) -> f64 {
        let k = k - 1.0;
        1.0 + (k * k * k)
    }

    pub fn in_out(k: f64) -> f64 {
        let k = k * 2.0;
        if k < 1.0 {
            return 0.5 * k * k * k;
        }

        let k = k - 2.0;
        0.5 * (k * k * k + 2.0)
    }
}

pub mod quartic {
    pub fn in_(k: f64) -> f64 {
        k * k * k * k
    }

    pub fn out(k: f64) -> f64 {
        let k = k - 1.0;
        1.0 - (k * k * k * k)
    }

    pub fn in_out(k: f64) -> f64 {
        let k = k * 2.0;
        if k < 1.0 {
            return 0.5 * k * k * k * k;
        }

        let k = k - 2.0;
        -0.5 * (k * k * k * k - 2.0)
    }
}

pub mod quintic {
    pub fn in_(k: f64) -> f64 {
        k * k * k * k * k
    }

    pub fn out(k: f64) -> f64 {
        let k = k - 1.0;
        1.0 + (k * k * k * k * k)
    }

    pub fn in_out(k: f64) -> f64 {
        let k = k * 2.0;
        if k < 1.0 {
            return 0.5 * k * k * k * k * k;
        }

        let k = k - 2.0;
        0.5 * (k * k * k * k * k + 2.0)
    }
}

pub mod sinusoidal {
    use std::f64::consts::PI;

    pub fn in_(k: f64) -> f64 {
        1.0 - (k * PI / 2.0).cos()
    }

    pub fn out(k: f64) -> f64 {
        (k * PI / 2.0).sin()
    }

    pub fn in_out(k: f64) -> f64 {
        0.5 * (1.0 - (PI * k).cos())
    }
}

pub mod exponential {
    pub fn in_(k: f64) -> f64 {
        if k == 0.0 {
            0.0
        } else {
            1024f64.powf(k - 1.0)
        }
    }

    pub fn out(k: f64) -> f64 {
        if k == 1.0 {
            1.0
        } else {
            1.0 - 2f64.powf(-10.0 * k)
        }
    }

    pub fn in_out(k: f64) -> f64 {
        if k == 0.0 {
            return 0.0;
        }

        if k == 1.0 {
            return 1.0;
        }

        let k = k * 2.0;
        if k < 1.0 {
            return 0.5 * 1024f64.powf(k - 1.0);
        }

        0.5 * (-(2f64.powf(-10.0 * (k - 1.0))) + 2.0)
    }
}

pub mod circular {
    pub fn in_(k: f64) -> f64 {
        1.0 - (1.0 - k * k).sqrt()
    }

    pub fn out(k: f64) -> f64 {
        let k = k - 1.0;
        (1.0 - (k * k)).sqrt()
    }

    pub fn in_out(k: f64) -> f64 {
        let k = k * 2.0;
        if k < 1.0 {
            return -0.5 * ((1.0 - k * k).sqrt() - 1.0);
        }

        let k = k - 2.0;
        0.5 * ((1.0 - k * k).sqrt() + 1.0)
    }
}

pub mod elastic {
    use super::f32_const;
    use std::f64::consts::PI;

    const POINT_1: f64 = f32_const(0.1);
    const POINT_4: f64 = f32_const(0.4);

    pub fn in_(k: f64) -> f64 {
        if k == 0.0 {
            return 0.0;
        }

        if k == 1.0 {
            return 1.0;
        }

        let k = k - 1.0;
        -(2f64.powf(10.0 * k)) * ((k - POINT_1) * (2.0 * PI) / POINT_4).sin()
    }

    pub fn out(k: f64) -> f64 {
        if k == 0.0 {
            return 0.0;
        }

        if k == 1.0 {
            return 1.0;
        }

        2f64.powf(-10.0 * k) * ((k - POINT_1) * (2.0 * PI) / POINT_4).sin() + 1.0
    }

    pub fn in_out(k: f64) -> f64 {
        let k = k * 2.0;
        if k < 1.0 {
            let k = k - 1.0;
            return -0.5 * 2f64.powf(10.0 * k) * ((k - POINT_1) * (2.0 * PI) / POINT_4).sin();
        }

        let k = k - 1.0;
        2f64.powf(-10.0 * k) * ((k - POINT_1) * (2.0 * PI) / POINT_4).sin() * 0.5 + 1.0
    }
}

pub mod back {
    use super::f32_const;

    const S: f64 = f32_const(1.70158);
    const S2: f64 = f32_const(2.5949095);

    pub fn in_(k: f64) -> f64 {
        k * k * ((S + 1.0) * k - S)
    }

    pub fn out(k: f64) -> f64 {
        let k = k - 1.0;
        k * k * ((S + 1.0) * k + S) + 1.0
    }

    pub fn in_out(k: f64) -> f64 {
        let k = k * 2.0;
        if k < 1.0 {
            return 0.5 * (k * k * ((S2 + 1.0) * k - S2));
        }

        let k = k - 2.0;
        0.5 * (k * k * ((S2 + 1.0) * k + S2) + 2.0)
    }
}

pub mod bounce {
    use super::f32_const;

    // Single-precision quotients, widened: the bounce segment edges the curve was tuned with.
    const B1: f64 = f32_const(1.0 / 2.75);
    const B2: f64 = f32_const(2.0 / 2.75);
    const B25: f64 = f32_const(2.5 / 2.75);
    const O15: f64 = f32_const(1.5 / 2.75);
    const O225: f64 = f32_const(2.25 / 2.75);
    const O2625: f64 = f32_const(2.625 / 2.75);

    pub fn in_(k: f64) -> f64 {
        1.0 - out(1.0 - k)
    }

    pub fn out(k: f64) -> f64 {
        if k < B1 {
            7.5625 * k * k
        } else if k < B2 {
            let k = k - O15;
            7.5625 * k * k + 0.75
        } else if k < B25 {
            let k = k - O225;
            7.5625 * k * k + 0.9375
        } else {
            let k = k - O2625;
            7.5625 * k * k + 0.984375
        }
    }

    pub fn in_out(k: f64) -> f64 {
        if k < 0.5 {
            return in_(k * 2.0) * 0.5;
        }

        out(k * 2.0 - 1.0) * 0.5 + 0.5
    }
}
