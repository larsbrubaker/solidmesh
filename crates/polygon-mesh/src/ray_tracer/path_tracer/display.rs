//! Turning linear HDR radiance into display bytes: exposure, a neutral tone map, sRGB encoding.
//! The tone map is the same curve on every channel, so it doesn't tint anything, and it reaches 1
//! only at infinity, so a brighter input is never a darker output. `f64` throughout; the byte
//! rounds half to even so the conversion is exact to state.

/// The alpha byte of a displayed pixel: a render has no transparency.
pub(super) const DISPLAY_OPAQUE: u8 = 255;

/// Maps a linear `value` in `[0, inf)` to `[0, 1)`: `x / (1 + x)`. Not finite or not above 0
/// gives 0, except positive infinity, which gives 1.
pub fn tone_map(value: f64) -> f64 {
    if value == f64::INFINITY {
        1.0
    } else if value > 0.0 {
        value / (1.0 + value)
    } else {
        0.0
    }
}

/// Encodes a linear `value` in `[0, 1]` with the sRGB transfer function; out of range clamps.
pub fn linear_to_srgb(value: f64) -> f64 {
    let value = if value.is_finite() {
        value.clamp(0.0, 1.0)
    } else {
        0.0
    };
    if value >= 1.0 {
        // The formula rounds to just below 1, and a white pixel must be exactly white.
        return 1.0;
    }
    if value <= 0.003_130_8 {
        12.92 * value
    } else {
        1.055 * libm::pow(value, 1.0 / 2.4) - 0.055
    }
}

/// Decodes an 8-bit sRGB `code` to linear: how a mesh's stored face colors are read.
pub(super) fn srgb_to_linear(code: u8) -> f64 {
    let value = f64::from(code) / 255.0;
    if value <= 0.040_45 {
        value / 12.92
    } else {
        libm::pow((value + 0.055) / 1.055, 2.4)
    }
}

/// One channel's display byte: `linear` scaled by `exposure`, tone mapped, sRGB encoded and
/// rounded to the nearest byte.
pub fn display_byte(linear: f64, exposure: f64) -> u8 {
    let encoded = linear_to_srgb(tone_map(linear * exposure));
    // `encoded * 255` is in [0, 255] by the clamp above, so the cast cannot saturate.
    (encoded * 255.0).round_ties_even() as u8
}
