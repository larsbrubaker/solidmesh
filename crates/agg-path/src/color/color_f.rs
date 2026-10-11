//! The float RGBA colour. Ports agg-sharp `agg/Primitives/ColorF.cs`: constructors, HSL
//! conversion, the arithmetic operators, premultiply/demultiply, wavelength colours, tweening
//! and blending. [`super::color8::Color`] is the 8-bit colour it converts to and from.

use std::hash::{Hash, Hasher};
use std::ops::{Add, Div, Mul, Sub};

use super::color8::{uround, Color};

/// .NET `Math.Max(float, float)`: NaN if either argument is NaN (Rust's `f32::max` ignores it).
fn net_max(a: f32, b: f32) -> f32 {
    if a.is_nan() || b.is_nan() {
        f32::NAN
    } else if a > b {
        a
    } else {
        b
    }
}

/// .NET `Math.Min(float, float)`, NaN-propagating.
fn net_min(a: f32, b: f32) -> f32 {
    if a.is_nan() || b.is_nan() {
        f32::NAN
    } else if a < b {
        a
    } else {
        b
    }
}

/// agg-sharp `ColorF`: `f32` RGBA, nominally 0..1 but unclamped.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ColorF {
    pub red: f32,
    pub green: f32,
    pub blue: f32,
    pub alpha: f32,
}

/// C# `GetHashCode` (`new { blue, green, red, alpha }.GetHashCode()`): equal colours hash
/// equal. -0 is folded into 0, which `==` treats as the same value (as .NET's float hash does).
impl Hash for ColorF {
    fn hash<H: Hasher>(&self, state: &mut H) {
        for channel in [self.blue, self.green, self.red, self.alpha] {
            (channel + 0.0).to_bits().hash(state);
        }
    }
}

impl ColorF {
    const BASE_MASK: i32 = 255;

    pub const WHITE: ColorF = ColorF::new(1.0, 1.0, 1.0, 1.0);
    pub const BLACK: ColorF = ColorF::new(0.0, 0.0, 0.0, 1.0);
    pub const RED: ColorF = ColorF::new(1.0, 0.0, 0.0, 1.0);
    pub const GREEN: ColorF = ColorF::new(0.0, 1.0, 0.0, 1.0);
    pub const BLUE: ColorF = ColorF::new(0.0, 0.0, 1.0, 1.0);
    pub const CYAN: ColorF = ColorF::new(0.0, 1.0, 1.0, 1.0);
    pub const MAGENTA: ColorF = ColorF::new(1.0, 0.0, 1.0, 1.0);
    pub const YELLOW: ColorF = ColorF::new(1.0, 1.0, 0.0, 1.0);

    /// C# `new ColorF(float r, float g, float b, float a)`.
    pub const fn new(red: f32, green: f32, blue: f32, alpha: f32) -> Self {
        Self {
            red,
            green,
            blue,
            alpha,
        }
    }

    /// C# `new ColorF(double r, double g, double b, double a)`: each narrowed to `f32`.
    pub fn from_f64(r: f64, g: f64, b: f64, a: f64) -> Self {
        Self::new(r as f32, g as f32, b as f32, a as f32)
    }

    /// C# `new ColorF(double r, double g, double b)`: opaque.
    pub fn from_f64_rgb(r: f64, g: f64, b: f64) -> Self {
        Self::from_f64(r, g, b, 1.0)
    }

    /// C# `new ColorF(ColorF c, float a)`.
    pub fn with_alpha_of(c: ColorF, a: f32) -> Self {
        Self::new(c.red, c.green, c.blue, a)
    }

    /// C# `new ColorF(Color color)`: `channel / 255f`.
    pub fn from_color(color: Color) -> Self {
        color.to_color_f()
    }

    /// C# `new ColorF(float wavelen, float gamma)`.
    pub fn from_wavelength(wl: f32, gamma: f32) -> ColorF {
        let mut t = ColorF::new(0.0, 0.0, 0.0, 1.0);
        let wl64 = wl as f64;
        if (380.0..=440.0).contains(&wl64) {
            t.red = (-(wl64 - 440.0) / (440.0 - 380.0)) as f32;
            t.blue = 1.0;
        } else if (440.0..=490.0).contains(&wl64) {
            t.green = ((wl64 - 440.0) / (490.0 - 440.0)) as f32;
            t.blue = 1.0;
        } else if (490.0..=510.0).contains(&wl64) {
            t.green = 1.0;
            t.blue = (-(wl64 - 510.0) / (510.0 - 490.0)) as f32;
        } else if (510.0..=580.0).contains(&wl64) {
            t.red = ((wl64 - 510.0) / (580.0 - 510.0)) as f32;
            t.green = 1.0;
        } else if (580.0..=645.0).contains(&wl64) {
            t.red = 1.0;
            t.green = (-(wl64 - 645.0) / (645.0 - 580.0)) as f32;
        } else if (645.0..=780.0).contains(&wl64) {
            t.red = 1.0;
        }

        let mut s = 1.0f32;
        if wl64 > 700.0 {
            s = (0.3 + 0.7 * (780.0 - wl64) / (780.0 - 700.0)) as f32;
        } else if wl64 < 420.0 {
            s = (0.3 + 0.7 * (wl64 - 380.0) / (420.0 - 380.0)) as f32;
        }

        t.red = ((t.red * s) as f64).powf(gamma as f64) as f32;
        t.green = ((t.green * s) as f64).powf(gamma as f64) as f32;
        t.blue = ((t.blue * s) as f64).powf(gamma as f64) as f32;
        t
    }

    /// C# `Red0To255` getter (and the other three): note the clamp is to 0..255, not 0..1,
    /// as in C#; `ToColor`'s constructor clamps the result.
    fn channel_0_to_255(v: f32) -> i32 {
        uround((net_max(0.0, net_min(255.0, v)) * Self::BASE_MASK as f32) as f64)
    }

    pub fn red0_to_255(&self) -> i32 {
        Self::channel_0_to_255(self.red)
    }

    pub fn green0_to_255(&self) -> i32 {
        Self::channel_0_to_255(self.green)
    }

    pub fn blue0_to_255(&self) -> i32 {
        Self::channel_0_to_255(self.blue)
    }

    pub fn alpha0_to_255(&self) -> i32 {
        Self::channel_0_to_255(self.alpha)
    }

    /// C# `FromHSL(h, s, l, alpha = 1)`: all inputs 0..1.
    pub fn from_hsl(
        hue0_to_1: f64,
        saturation0_to_1: f64,
        lightness0_to_1: f64,
        alpha: f64,
    ) -> ColorF {
        let alpha = if alpha > 1.0 { 1.0 } else { alpha };
        // default to gray
        let (mut r, mut g, mut b) = (lightness0_to_1, lightness0_to_1, lightness0_to_1);
        let mut v = lightness0_to_1 + saturation0_to_1 - lightness0_to_1 * saturation0_to_1;
        if lightness0_to_1 <= 0.5 {
            v = lightness0_to_1 * (1.0 + saturation0_to_1);
        }

        if v > 0.0 {
            let m = lightness0_to_1 + lightness0_to_1 - v;
            let sv = (v - m) / v;
            let hue = hue0_to_1 * 6.0;
            // C# (int) truncates toward zero (saturating on .NET 9+, as `as` does).
            let sextant = hue as i32;
            let fract = hue - sextant as f64;
            let vsf = v * sv * fract;
            let mid1 = m + vsf;
            let mid2 = v - vsf;
            match sextant {
                0 | 6 => (r, g, b) = (v, mid1, m),
                1 => (r, g, b) = (mid2, v, m),
                2 => (r, g, b) = (m, v, mid1),
                3 => (r, g, b) = (m, mid2, v),
                4 => (r, g, b) = (mid1, m, v),
                5 => (r, g, b) = (v, m, mid2),
                _ => {}
            }
        }

        ColorF::from_f64(r, g, b, alpha)
    }

    /// C# `GetHSL(out h, out s, out l)`, returned as `(hue, saturation, lightness)`.
    pub fn get_hsl(&self) -> (f64, f64, f64) {
        let max_rgb = net_max(self.red, net_max(self.green, self.blue)) as f64;
        let min_rgb = net_min(self.red, net_min(self.green, self.blue)) as f64;
        let delta_max_to_min = max_rgb - min_rgb;

        let mut hue = 0.0; // default to black
        let lightness = (min_rgb + max_rgb) / 2.0;
        if lightness <= 0.0 {
            return (hue, 0.0, lightness);
        }
        let mut saturation = delta_max_to_min;
        if saturation > 0.0 {
            saturation /= if lightness <= 0.5 {
                max_rgb + min_rgb
            } else {
                2.0 - max_rgb - min_rgb
            };
        } else {
            return (hue, saturation, lightness);
        }
        let (red, green, blue) = (self.red as f64, self.green as f64, self.blue as f64);
        let r2 = (max_rgb - red) / delta_max_to_min;
        let g2 = (max_rgb - green) / delta_max_to_min;
        let b2 = (max_rgb - blue) / delta_max_to_min;
        if red == max_rgb {
            hue = if green == min_rgb { 5.0 + b2 } else { 1.0 - g2 };
        } else if green == max_rgb {
            hue = if blue == min_rgb { 1.0 + r2 } else { 3.0 - b2 };
        } else {
            hue = if red == min_rgb { 3.0 + g2 } else { 5.0 - r2 };
        }
        hue /= 6.0;
        (hue, saturation, lightness)
    }

    /// C# `ToColor()`.
    pub fn to_color(self) -> Color {
        Color::new(
            self.red0_to_255(),
            self.green0_to_255(),
            self.blue0_to_255(),
            self.alpha0_to_255(),
        )
    }

    /// C# `clear()`.
    pub fn clear(&mut self) {
        *self = ColorF::default();
    }

    /// C# `transparent()`: zero alpha in place, returning the result.
    pub fn transparent(&mut self) -> ColorF {
        self.alpha = 0.0;
        *self
    }

    /// C# `opacity(float)`: set alpha clamped to 0..1.
    pub fn set_opacity(&mut self, a: f32) -> ColorF {
        self.alpha = a.clamp(0.0, 1.0);
        *self
    }

    /// C# `opacity()`.
    pub fn opacity(&self) -> f32 {
        self.alpha
    }

    /// C# `premultiply()`.
    pub fn premultiply(&mut self) -> ColorF {
        self.red *= self.alpha;
        self.green *= self.alpha;
        self.blue *= self.alpha;
        *self
    }

    /// C# `premultiply(float a)`.
    pub fn premultiply_to(&mut self, a: f32) -> ColorF {
        if self.alpha <= 0.0 || a <= 0.0 {
            *self = ColorF::default();
            return *self;
        }
        let a = a / self.alpha;
        self.red *= a;
        self.green *= a;
        self.blue *= a;
        self.alpha = a;
        *self
    }

    /// C# `ComponentMax(a, b)`.
    pub fn component_max(a: ColorF, b: ColorF) -> ColorF {
        let mut result = a;
        if result.red < b.red {
            result.red = b.red;
        }
        if result.green < b.green {
            result.green = b.green;
        }
        if result.blue < b.blue {
            result.blue = b.blue;
        }
        if result.alpha < b.alpha {
            result.alpha = b.alpha;
        }
        result
    }

    /// C# `demultiply()`.
    pub fn demultiply(&mut self) -> ColorF {
        if self.alpha == 0.0 {
            self.red = 0.0;
            self.green = 0.0;
            self.blue = 0.0;
            return *self;
        }
        let a = 1.0f32 / self.alpha;
        self.red *= a;
        self.green *= a;
        self.blue *= a;
        *self
    }

    /// C# `gradient(Color c_8, double k)`: float interpolation toward `c_8`, as a `Color`.
    pub fn gradient(&self, c_8: Color, k: f64) -> Color {
        let c = c_8.to_color_f();
        let lerp = |a: f32, b: f32| (a as f64 + (b - a) as f64 * k) as f32;
        ColorF::new(
            lerp(self.red, c.red),
            lerp(self.green, c.green),
            lerp(self.blue, c.blue),
            lerp(self.alpha, c.alpha),
        )
        .to_color()
    }

    /// C# `no_color()`.
    pub fn no_color() -> ColorF {
        ColorF::default()
    }

    /// C# `rgba_pre(float r, float g, float b, float a)`.
    pub fn rgba_pre(r: f32, g: f32, b: f32, a: f32) -> ColorF {
        ColorF::new(r, g, b, a).premultiply()
    }

    /// C# `GetTweenColor(Color1, Color2, RatioOf2)`: the result is opaque unless a ratio is
    /// out of 0..1, when it is a copy of an end colour.
    pub fn get_tween_color(color1: ColorF, color2: ColorF, ratio_of_2: f64) -> ColorF {
        if ratio_of_2 <= 0.0 {
            return color1;
        }
        if ratio_of_2 >= 1.0 {
            return color2;
        }
        // figure out how much of each color we should be.
        let ratio_of_1 = 1.0 - ratio_of_2;
        ColorF::from_f64_rgb(
            color1.red as f64 * ratio_of_1 + color2.red as f64 * ratio_of_2,
            color1.green as f64 * ratio_of_1 + color2.green as f64 * ratio_of_2,
            color1.blue as f64 * ratio_of_1 + color2.blue as f64 * ratio_of_2,
        )
    }

    /// C# `Blend(other, weight)`: `this * (1 - weight) + other * weight`.
    pub fn blend(self, other: ColorF, weight: f64) -> ColorF {
        self * (1.0 - weight) + other * weight
    }

    /// C# `SumOfDistances(other)`: the RGB L1 distance.
    pub fn sum_of_distances(&self, other: ColorF) -> f64 {
        (self.red - other.red).abs() as f64
            + (self.green - other.green).abs() as f64
            + (self.blue - other.blue).abs() as f64
    }

    /// C# `Clamp0To1()`.
    pub fn clamp_0_to_1(&mut self) {
        // NaN stays NaN, as in C#.
        let c = |v: &mut f32| *v = v.clamp(0.0, 1.0);
        c(&mut self.red);
        c(&mut self.green);
        c(&mut self.blue);
        c(&mut self.alpha);
    }
}

impl From<Color> for ColorF {
    fn from(c: Color) -> Self {
        ColorF::from_color(c)
    }
}

impl Add for ColorF {
    type Output = ColorF;
    fn add(self, b: ColorF) -> ColorF {
        ColorF::new(
            self.red + b.red,
            self.green + b.green,
            self.blue + b.blue,
            self.alpha + b.alpha,
        )
    }
}

impl Sub for ColorF {
    type Output = ColorF;
    fn sub(self, b: ColorF) -> ColorF {
        ColorF::new(
            self.red - b.red,
            self.green - b.green,
            self.blue - b.blue,
            self.alpha - b.alpha,
        )
    }
}

impl Mul for ColorF {
    type Output = ColorF;
    fn mul(self, b: ColorF) -> ColorF {
        ColorF::new(
            self.red * b.red,
            self.green * b.green,
            self.blue * b.blue,
            self.alpha * b.alpha,
        )
    }
}

impl Div for ColorF {
    type Output = ColorF;
    fn div(self, b: ColorF) -> ColorF {
        ColorF::new(
            self.red / b.red,
            self.green / b.green,
            self.blue / b.blue,
            self.alpha / b.alpha,
        )
    }
}

impl Mul<f32> for ColorF {
    type Output = ColorF;
    fn mul(self, b: f32) -> ColorF {
        ColorF::new(self.red * b, self.green * b, self.blue * b, self.alpha * b)
    }
}

/// C# `operator *(ColorF, double)`: the factor is narrowed to `f32` first.
impl Mul<f64> for ColorF {
    type Output = ColorF;
    fn mul(self, b: f64) -> ColorF {
        self * (b as f32)
    }
}

impl Div<f32> for ColorF {
    type Output = ColorF;
    fn div(self, b: f32) -> ColorF {
        ColorF::new(self.red / b, self.green / b, self.blue / b, self.alpha / b)
    }
}

/// C# `operator /(ColorF, double)`: the divisor is narrowed to `f32` first.
impl Div<f64> for ColorF {
    type Output = ColorF;
    fn div(self, b: f64) -> ColorF {
        self / (b as f32)
    }
}
