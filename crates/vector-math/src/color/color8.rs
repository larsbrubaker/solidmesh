//! The 8-bit RGBA colour (the `Color` struct, its named statics, HTML parsing and formatting, the `+`/`-`/`*` operators, `Blend`, `gradient`,
//! `add`, `WithAlpha` from `ColorExtensionMethods`). The LAB naming helpers live in
//! `color_names`, the float colour in [`super::color_f`], the `IColorType` extension
//! methods in [`super::color_type`], and the JSON shape in `color_serde`.

use std::fmt;
use std::ops::{Add, Mul, Sub};

use super::color_f::ColorF;

/// Rounds half up: `v + 0.5` converted to `u32` (saturating, NaN is 0), then reinterpreted as
/// `i32`, which wraps.
pub(crate) fn uround(v: f64) -> i32 {
    (v + 0.5) as u32 as i32
}

/// 8-bit RGBA. `Default` is transparent black.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct Color {
    pub red: u8,
    pub green: u8,
    pub blue: u8,
    pub alpha: u8,
}

impl Color {
    pub const COVER_SHIFT: i32 = 8;
    pub const COVER_SIZE: i32 = 1 << Self::COVER_SHIFT;
    pub const COVER_MASK: i32 = Self::COVER_SIZE - 1;
    pub const BASE_SHIFT: i32 = 8;
    pub const BASE_SCALE: i32 = 1 << Self::BASE_SHIFT;
    pub const BASE_MASK: i32 = Self::BASE_SCALE - 1;

    pub const DARK_BLUE: Color = Color::rgb(0, 0, 0xA0);
    pub const DARK_GREEN: Color = Color::rgb(0, 100, 0);
    pub const DARK_MAGENTA: Color = Color::rgb(139, 0, 139);
    pub const DARK_RED: Color = Color::rgb(139, 0, 0);
    pub const DARK_YELLOW: Color = Color::rgb(139, 139, 0);
    pub const GREEN: Color = Color::rgb(0, 255, 0);
    pub const LIGHT_GREEN: Color = Color::rgb(144, 238, 144);
    pub const MAGENTA: Color = Color::rgb(255, 0, 255);
    pub const RED: Color = Color::rgb(255, 0, 0);
    pub const YELLOW: Color = Color::rgb(255, 255, 0);
    pub const BLACK: Color = Color::rgb(0, 0, 0);
    pub const BLUE: Color = Color::rgb(0, 0, 255);
    pub const CRIMSON: Color = Color::rgb(0xDC, 0x14, 0x3C);
    pub const CYAN: Color = Color::rgb(0, 255, 255);
    pub const DARK_CYAN: Color = Color::rgb(0, 139, 139);
    pub const DARK_GRAY: Color = Color::rgb(85, 85, 85);
    pub const FIRE_ENGINE_RED: Color = Color::rgb(0xF6, 0x28, 0x17);
    pub const GRAY: Color = Color::rgb(125, 125, 125);
    pub const INDIGO: Color = Color::rgb(75, 0, 130);
    pub const LIGHT_BLUE: Color = Color::rgb(0xAD, 0xD8, 0xE6);
    /// The UI's light gray (225), not the named-colour table's `LightGray` entry.
    pub const LIGHT_GRAY: Color = Color::rgb(225, 225, 225);
    pub const ORANGE: Color = Color::rgb(255, 127, 0);
    pub const PURPLE: Color = Color::rgb(128, 0, 128);
    pub const PINK: Color = Color::rgb(255, 192, 203);
    pub const TRANSPARENT: Color = Color::rgba(0, 0, 0, 0);
    pub const VIOLET: Color = Color::rgb(143, 0, 255);
    pub const WHITE: Color = Color::rgb(255, 255, 255);
    pub const YELLOW_GREEN: Color = Color::rgb(154, 205, 50);

    /// Byte channels, as stored.
    pub const fn rgba(red: u8, green: u8, blue: u8, alpha: u8) -> Self {
        Self {
            red,
            green,
            blue,
            alpha,
        }
    }

    /// Opaque byte channels.
    pub const fn rgb(red: u8, green: u8, blue: u8) -> Self {
        Self::rgba(red, green, blue, 255)
    }

    /// Each channel clamped to 0..=255.
    pub fn new(r: i32, g: i32, b: i32, a: i32) -> Self {
        let c = |v: i32| v.clamp(0, 255) as u8;
        Self::rgba(c(r), c(g), c(b), c(a))
    }

    /// Opaque, clamped.
    pub fn new_rgb(r: i32, g: i32, b: i32) -> Self {
        Self::new(r, g, b, Self::BASE_MASK)
    }

    /// 0..1 channels, `uround`ed and
    /// truncated to a byte (out-of-range values wrap).
    pub fn from_f64(r: f64, g: f64, b: f64, a: f64) -> Self {
        let c = |v: f64| uround(v * Self::BASE_MASK as f64) as u8;
        Self::rgba(c(r), c(g), c(b), c(a))
    }

    /// Opaque.
    pub fn from_f64_rgb(r: f64, g: f64, b: f64) -> Self {
        let c = |v: f64| uround(v * Self::BASE_MASK as f64) as u8;
        Self::rgba(c(r), c(g), c(b), 255)
    }

    pub fn from_color_f_alpha(c: ColorF, a: f64) -> Self {
        let mut color = Self::from_color_f(c);
        color.alpha = uround(a * Self::BASE_MASK as f64) as u8;
        color
    }

    pub fn from_color_f(c: ColorF) -> Self {
        let ch = |v: f32| uround(v as f64 * Self::BASE_MASK as f64) as u8;
        Self::rgba(ch(c.red), ch(c.green), ch(c.blue), ch(c.alpha))
    }

    /// `0xAARRGGBB`.
    pub fn from_argb(four_byte_color: u32) -> Self {
        Self::rgba(
            ((four_byte_color >> 16) & 0xFF) as u8,
            ((four_byte_color >> 8) & 0xFF) as u8,
            (four_byte_color & 0xFF) as u8,
            ((four_byte_color >> 24) & 0xFF) as u8,
        )
    }

    /// The `Html` setter applied to transparent black, so anything it
    /// does not recognize leaves the colour transparent.
    pub fn from_html_or_transparent(html: &str) -> Self {
        Self::from_html(html).unwrap_or(Self::TRANSPARENT)
    }

    /// Parses `#RGB`, `#RGBA`, `#RRGGBB` or `#RRGGBBAA` in any case,
    /// surrounding whitespace ignored, or one of the nine names the setter knows (with or
    /// without a leading `#`). `None` for anything else, so the caller keeps its colour.
    pub fn from_html(html: &str) -> Option<Self> {
        let lowered = html.trim().to_lowercase();
        if lowered.chars().count() > 3
            && lowered.starts_with('#')
            && lowered[1..].chars().all(|c| "0123456789abcdef".contains(c))
        {
            let hex = &lowered[1..];
            let pair = |s: &str| u8::from_str_radix(s, 16).ok();
            let single = |s: &str| pair(&format!("{s}{s}"));
            return match lowered.len() {
                4 | 5 => Some(Self::rgba(
                    single(&hex[0..1])?,
                    single(&hex[1..2])?,
                    single(&hex[2..3])?,
                    if lowered.len() == 5 {
                        single(&hex[3..4])?
                    } else {
                        255
                    },
                )),
                7 | 9 => Some(Self::rgba(
                    pair(&hex[0..2])?,
                    pair(&hex[2..4])?,
                    pair(&hex[4..6])?,
                    if lowered.len() == 9 {
                        pair(&hex[6..8])?
                    } else {
                        255
                    },
                )),
                // don't know what it is, do nothing
                _ => None,
            };
        }
        let named = lowered.trim();
        let named = named.strip_prefix('#').unwrap_or(named);
        match named {
            "blue" => Some(Self::BLUE),
            "green" => Some(Self::GREEN),
            "red" => Some(Self::RED),
            "black" => Some(Self::BLACK),
            "gray" | "grey" => Some(Self::GRAY),
            "orange" => Some(Self::ORANGE),
            "purple" => Some(Self::PURPLE),
            "yellow" => Some(Self::YELLOW),
            "white" => Some(Self::WHITE),
            _ => None,
        }
    }

    /// `#RRGGBBAA`, upper-case hex.
    pub fn html(&self) -> String {
        format!(
            "#{:02X}{:02X}{:02X}{:02X}",
            self.red, self.green, self.blue, self.alpha
        )
    }

    /// `#RRGGBB` when opaque, else `#RRGGBBAA`.
    pub fn get_as_html_string(&self) -> String {
        if self.alpha == 255 {
            format!("#{:02X}{:02X}{:02X}", self.red, self.green, self.blue)
        } else {
            self.html()
        }
    }

    /// FNV-1a over red, green, blue, alpha.
    pub fn get_long_hash_code(&self, hash: u64) -> u64 {
        crate::long_hash::compute_hash(&[self.red, self.green, self.blue, self.alpha], hash)
    }

    pub fn to_color_f(self) -> ColorF {
        let m = Self::BASE_MASK as f32;
        ColorF::new(
            self.red as f32 / m,
            self.green as f32 / m,
            self.blue as f32 / m,
            self.alpha as f32 / m,
        )
    }

    pub fn from_hsl(
        hue0_to_1: f64,
        saturation0_to_1: f64,
        lightness0_to_1: f64,
        alpha: f64,
    ) -> Self {
        ColorF::from_hsl(hue0_to_1, saturation0_to_1, lightness0_to_1, alpha).to_color()
    }

    /// Fixed-point interpolation toward `c`.
    pub fn gradient(self, c: Color, k: f64) -> Color {
        let ik = uround(k * Self::BASE_SCALE as f64);
        let lerp = |a: u8, b: u8| -> u8 {
            let a = a as i32;
            (a + (((b as i32 - a).wrapping_mul(ik)) >> Self::BASE_SHIFT)) as u8
        };
        Color::rgba(
            lerp(self.red, c.red),
            lerp(self.green, c.green),
            lerp(self.blue, c.blue),
            lerp(self.alpha, c.alpha),
        )
    }

    /// Accumulate `c` scaled by `cover`, saturating.
    pub fn add_cover(&mut self, c: Color, cover: i32) {
        let mask = Self::BASE_MASK;
        let sat = |v: i32| (if v > mask { mask } else { v }) as u8;
        if cover == Self::COVER_MASK {
            if c.alpha as i32 == mask {
                *self = c;
            } else {
                self.red = sat(self.red as i32 + c.red as i32);
                self.green = sat(self.green as i32 + c.green as i32);
                self.blue = sat(self.blue as i32 + c.blue as i32);
                self.alpha = sat(self.alpha as i32 + c.alpha as i32);
            }
        } else {
            let part = |v: u8| (v as i32 * cover + Self::COVER_MASK / 2) >> Self::COVER_SHIFT;
            self.red = sat(self.red as i32 + part(c.red));
            self.green = sat(self.green as i32 + part(c.green));
            self.blue = sat(self.blue as i32 + part(c.blue));
            self.alpha = sat(self.alpha as i32 + part(c.alpha));
        }
    }

    pub fn no_color() -> Color {
        Color::TRANSPARENT
    }

    /// `0xRRGGBB`, opaque.
    pub fn rgb8_packed(v: i32) -> Color {
        Color::new_rgb((v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF)
    }

    /// `this * (1 - weight) + other * weight`.
    pub fn blend(self, other: Color, weight: f64) -> Color {
        self * (1.0 - weight) + other * weight
    }

    /// Interpolate hue, saturation and lightness.
    pub fn blend_hsl(self, color2: Color, ratio_of_2: f64) -> Color {
        let (a_h, a_s, a_l) = ColorF::from_color(self).get_hsl();
        let (b_h, b_s, b_l) = ColorF::from_color(color2).get_hsl();
        ColorF::from_hsl(
            a_h * (1.0 - ratio_of_2) + b_h * ratio_of_2,
            a_s * (1.0 - ratio_of_2) + b_s * ratio_of_2,
            a_l * (1.0 - ratio_of_2) + b_l * ratio_of_2,
            1.0,
        )
        .to_color()
    }

    /// `new Color(color, alpha)`, clamped to 0..=255.
    pub fn with_alpha(self, alpha: i32) -> Color {
        Color {
            alpha: alpha.clamp(0, 255) as u8,
            ..self
        }
    }

    /// `255 * alpha` rounded half to even, saturating into `i32`, then clamped to 0..=255.
    pub fn with_alpha_f(self, alpha: f64) -> Color {
        self.with_alpha((255.0 * alpha).round_ties_even() as i32)
    }
}

impl fmt::Display for Color {
    /// `GetAsHTMLString()`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.get_as_html_string())
    }
}

impl Add for Color {
    type Output = Color;

    /// Per-channel sum, saturating at 255.
    fn add(self, b: Color) -> Color {
        Color::rgba(
            self.red.saturating_add(b.red),
            self.green.saturating_add(b.green),
            self.blue.saturating_add(b.blue),
            self.alpha.saturating_add(b.alpha),
        )
    }
}

impl Sub for Color {
    type Output = Color;

    /// Per-channel difference floored at 0; alpha is always 255.
    fn sub(self, b: Color) -> Color {
        Color::rgba(
            self.red.saturating_sub(b.red),
            self.green.saturating_sub(b.green),
            self.blue.saturating_sub(b.blue),
            255,
        )
    }
}

impl Mul<f64> for Color {
    type Output = Color;

    /// Every channel (alpha too) scaled in `f32`, then back
    /// through `new Color(ColorF)`.
    fn mul(self, weight: f64) -> Color {
        let b = weight as f32;
        let temp = ColorF::new(
            self.red as f32 / 255.0 * b,
            self.green as f32 / 255.0 * b,
            self.blue as f32 / 255.0 * b,
            self.alpha as f32 / 255.0 * b,
        );
        Color::from_color_f(temp)
    }
}

impl From<ColorF> for Color {
    fn from(c: ColorF) -> Self {
        Color::from_color_f(c)
    }
}
