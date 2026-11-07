//! The colour interface and its extension methods: the [`ColorType`] trait, with
//! `ColorExtensionMethods` (`WithContrast`, `AdjustSaturation`, `Luminance0To1`, `OverlayOn`,
//! `Contrast`, `WithLightness`, `AdjustLightness`) as its provided methods. `WithAlpha` lives on
//! [`Color`] because only the 8-bit colour has it.

use super::color8::Color;
use super::color_f::ColorF;

/// The 0..1 channel view both colour types share. Setters store the way each
/// type does (`Color` truncates to a byte, `ColorF` keeps the float).
pub trait ColorType: Copy {
    fn red0_to_1(&self) -> f32;
    fn green0_to_1(&self) -> f32;
    fn blue0_to_1(&self) -> f32;
    fn alpha0_to_1(&self) -> f32;
    fn set_red0_to_1(&mut self, value: f32);
    fn set_green0_to_1(&mut self, value: f32);
    fn set_blue0_to_1(&mut self, value: f32);
    fn set_alpha0_to_1(&mut self, value: f32);
    fn to_color(&self) -> Color;
    fn to_color_f(&self) -> ColorF;
    /// Stores a `Color` as this type.
    fn from_color(color: Color) -> Self;

    /// WCAG relative luminance.
    fn luminance0_to_1(&self) -> f64 {
        let channel = |c: f32| {
            let c = c as f64;
            if c <= 0.03928 {
                c / 12.92
            } else {
                ((c + 0.055) / 1.055).powf(2.4)
            }
        };
        0.2126 * channel(self.red0_to_1())
            + 0.7152 * channel(self.green0_to_1())
            + 0.0722 * channel(self.blue0_to_1())
    }

    /// This colour composited over `color`.
    fn overlay_on<C: ColorType>(&self, color: &C) -> Self {
        let mut overlaid = *self;
        let alpha = self.alpha0_to_1();
        if alpha >= 1.0 {
            return overlaid;
        }
        let under =
            |own: f32, theirs: f32| own * alpha + theirs * color.alpha0_to_1() * (1.0 - alpha);
        overlaid.set_red0_to_1(under(overlaid.red0_to_1(), color.red0_to_1()));
        overlaid.set_green0_to_1(under(overlaid.green0_to_1(), color.green0_to_1()));
        overlaid.set_blue0_to_1(under(overlaid.blue0_to_1(), color.blue0_to_1()));
        overlaid.set_alpha0_to_1(alpha + color.alpha0_to_1() * (1.0 - alpha));
        overlaid
    }

    /// The WCAG contrast ratio, 1 to 21, of `color` shown on this colour, rounded to one
    /// decimal (two when this colour is semi-transparent). A semi-transparent `color` is
    /// composited on this colour first.
    fn contrast<C: ColorType>(&self, color: &C) -> f64 {
        // Formula: http://www.w3.org/TR/2008/REC-WCAG20-20081211/#contrast-ratiodef
        let alpha = self.alpha0_to_1();
        if alpha >= 1.0 {
            let shown = color.overlay_on(self);
            let l1 = self.luminance0_to_1() + 0.05;
            let l2 = shown.luminance0_to_1() + 0.05;
            let mut ratio = l1 / l2;
            if l2 > l1 {
                ratio = 1.0 / ratio;
            }
            return round_digits(ratio, 1);
        }

        // A semi-transparent background shows differently over dark and light content, so
        // measure it composited on black and on white and take the midpoint of the range.
        let on_black = self.overlay_on(&Color::BLACK);
        let on_white = self.overlay_on(&Color::WHITE);
        let contrast_on_black = on_black.contrast(color);
        let contrast_on_white = on_white.contrast(color);

        let max = contrast_on_black.max(contrast_on_white);
        let mut min = 1.0;
        if on_black.luminance0_to_1() > color.luminance0_to_1() {
            min = contrast_on_black;
        } else if on_white.luminance0_to_1() < color.luminance0_to_1() {
            min = contrast_on_white;
        }
        round_digits((min + max) / 2.0, 2)
    }

    /// Lighten or darken by 5%
    /// steps (at most 30) until the contrast with `fixed_color` is reached.
    fn with_contrast<C: ColorType>(&self, fixed_color: &C, minimum_required_contrast: f64) -> Self {
        let mut color_to_adjust = *self;
        let mut contrast = color_to_adjust.contrast(fixed_color);
        let mut tries = 0;
        while contrast < minimum_required_contrast && {
            let go = tries < 30;
            tries += 1;
            go
        } {
            let factor = if fixed_color.luminance0_to_1() < 0.5 {
                1.05
            } else {
                0.95
            };
            color_to_adjust = Self::from_color(color_to_adjust.adjust_lightness(factor).to_color());
            contrast = color_to_adjust.contrast(fixed_color);
        }
        color_to_adjust
    }

    fn adjust_saturation(&self, saturation_multiplier: f64) -> ColorF {
        let (h, s, l) = self.to_color_f().get_hsl();
        ColorF::from_hsl(h, s * saturation_multiplier, l, 1.0)
    }

    fn with_lightness(&self, lightness: f64) -> ColorF {
        let (h, s, _) = self.to_color_f().get_hsl();
        ColorF::from_hsl(h, s, lightness, 1.0)
    }

    fn adjust_lightness(&self, lightness_multiplier: f64) -> ColorF {
        let (h, s, l) = self.to_color_f().get_hsl();
        ColorF::from_hsl(h, s, l * lightness_multiplier, 1.0)
    }
}

/// Rounds to `digits` decimals: scale, round half to even, unscale.
fn round_digits(value: f64, digits: i32) -> f64 {
    let power10 = 10f64.powi(digits);
    (value * power10).round_ties_even() / power10
}

/// A 0..1 channel as a byte: `value * 255` truncated, then clamped to 0..=255.
fn to_byte(value: f32) -> u8 {
    ((value * 255.0) as i32).clamp(0, 255) as u8
}

impl ColorType for Color {
    fn red0_to_1(&self) -> f32 {
        self.red as f32 / 255.0
    }
    fn green0_to_1(&self) -> f32 {
        self.green as f32 / 255.0
    }
    fn blue0_to_1(&self) -> f32 {
        self.blue as f32 / 255.0
    }
    fn alpha0_to_1(&self) -> f32 {
        self.alpha as f32 / 255.0
    }
    fn set_red0_to_1(&mut self, value: f32) {
        self.red = to_byte(value);
    }
    fn set_green0_to_1(&mut self, value: f32) {
        self.green = to_byte(value);
    }
    fn set_blue0_to_1(&mut self, value: f32) {
        self.blue = to_byte(value);
    }
    fn set_alpha0_to_1(&mut self, value: f32) {
        self.alpha = to_byte(value);
    }
    fn to_color(&self) -> Color {
        *self
    }
    fn to_color_f(&self) -> ColorF {
        Color::to_color_f(*self)
    }
    fn from_color(color: Color) -> Self {
        color
    }
}

impl ColorType for ColorF {
    fn red0_to_1(&self) -> f32 {
        self.red
    }
    fn green0_to_1(&self) -> f32 {
        self.green
    }
    fn blue0_to_1(&self) -> f32 {
        self.blue
    }
    fn alpha0_to_1(&self) -> f32 {
        self.alpha
    }
    fn set_red0_to_1(&mut self, value: f32) {
        self.red = value;
    }
    fn set_green0_to_1(&mut self, value: f32) {
        self.green = value;
    }
    fn set_blue0_to_1(&mut self, value: f32) {
        self.blue = value;
    }
    fn set_alpha0_to_1(&mut self, value: f32) {
        self.alpha = value;
    }
    fn to_color(&self) -> Color {
        ColorF::to_color(*self)
    }
    fn to_color_f(&self) -> ColorF {
        *self
    }
    fn from_color(color: Color) -> Self {
        color.to_color_f()
    }
}
