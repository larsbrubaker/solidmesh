//! Colour naming and CIE LAB conversion (`ColorToName`, `CalculateColorDistance`, `FromLab`,
//! `RgbToLab`, `XyzToLab`) with the named-colour table, whose order decides ties.

use super::color8::Color;

/// The named colours, in a fixed order: the first of equally close names wins.
const DETAILED_COLORS: &[(&str, Color)] = &[
    ("Red", Color::rgb(255, 0, 0)),
    ("Green", Color::rgb(0, 255, 0)),
    ("Blue", Color::rgb(0, 0, 255)),
    ("Aqua", Color::rgb(0, 255, 255)),
    ("Black", Color::rgb(0, 0, 0)),
    ("Fuchsia", Color::rgb(255, 0, 255)),
    ("Gray", Color::rgb(128, 128, 128)),
    ("Lime", Color::rgb(0, 255, 0)),
    ("Maroon", Color::rgb(128, 0, 0)),
    ("Navy", Color::rgb(0, 0, 128)),
    ("Olive", Color::rgb(128, 128, 0)),
    ("Purple", Color::rgb(128, 0, 128)),
    ("Silver", Color::rgb(192, 192, 192)),
    ("Teal", Color::rgb(0, 128, 128)),
    ("White", Color::rgb(255, 255, 255)),
    ("Yellow", Color::rgb(255, 255, 0)),
    ("Orange", Color::rgb(255, 165, 0)),
    ("Pink", Color::rgb(255, 192, 203)),
    ("Cyan", Color::rgb(0, 255, 255)),
    ("Magenta", Color::rgb(255, 0, 255)),
    ("Gold", Color::rgb(255, 215, 0)),
    ("Chocolate", Color::rgb(210, 105, 30)),
    ("Coral", Color::rgb(255, 127, 80)),
    ("DarkBlue", Color::rgb(0, 0, 139)),
    ("DarkCyan", Color::rgb(0, 139, 139)),
    ("DarkGray", Color::rgb(169, 169, 169)),
    ("DarkGreen", Color::rgb(0, 100, 0)),
    ("DarkMagenta", Color::rgb(139, 0, 139)),
    ("DarkOliveGreen", Color::rgb(85, 107, 47)),
    ("DarkOrange", Color::rgb(255, 140, 0)),
    ("DarkOrchid", Color::rgb(153, 50, 204)),
    ("DarkRed", Color::rgb(139, 0, 0)),
    ("DarkSalmon", Color::rgb(233, 150, 122)),
    ("DarkSeaGreen", Color::rgb(143, 188, 139)),
    ("DarkSlateBlue", Color::rgb(72, 61, 139)),
    ("DarkSlateGray", Color::rgb(47, 79, 79)),
    ("DarkTurquoise", Color::rgb(0, 206, 209)),
    ("DarkViolet", Color::rgb(148, 0, 211)),
    ("DeepPink", Color::rgb(255, 20, 147)),
    ("DeepSkyBlue", Color::rgb(0, 191, 255)),
    ("DimGray", Color::rgb(105, 105, 105)),
    ("DodgerBlue", Color::rgb(30, 144, 255)),
    ("Firebrick", Color::rgb(178, 34, 34)),
    ("ForestGreen", Color::rgb(34, 139, 34)),
    ("Gainsboro", Color::rgb(220, 220, 220)),
    ("HotPink", Color::rgb(255, 105, 180)),
    ("IndianRed", Color::rgb(205, 92, 92)),
    ("Indigo", Color::rgb(75, 0, 130)),
    ("Khaki", Color::rgb(240, 230, 140)),
    ("Lavender", Color::rgb(230, 230, 250)),
    ("LawnGreen", Color::rgb(124, 252, 0)),
    ("LightBlue", Color::rgb(173, 216, 230)),
    ("LightCoral", Color::rgb(240, 128, 128)),
    ("LightCyan", Color::rgb(224, 255, 255)),
    ("LightGoldenrodYellow", Color::rgb(250, 250, 210)),
    ("LightGray", Color::rgb(211, 211, 211)),
    ("LightGreen", Color::rgb(144, 238, 144)),
    ("LightPink", Color::rgb(255, 182, 193)),
    ("LightSalmon", Color::rgb(255, 160, 122)),
    ("LightSeaGreen", Color::rgb(32, 178, 170)),
    ("LightSkyBlue", Color::rgb(135, 206, 250)),
    ("LightSlateGray", Color::rgb(119, 136, 153)),
    ("LightSteelBlue", Color::rgb(176, 196, 222)),
    ("LightYellow", Color::rgb(255, 255, 224)),
    ("Beige", Color::rgb(0xF5, 0xF5, 0xDC)),
    ("Brown", Color::rgb(0x8B, 0x45, 0x13)),
    ("LimeGreen", Color::rgb(50, 205, 50)),
];

/// The Euclidean distance between two LAB colours (squares taken with `powf(2.0)`).
fn lab_distance(lab1: &[f64; 3], lab2: &[f64; 3]) -> f64 {
    ((lab1[0] - lab2[0]).powf(2.0) + (lab1[1] - lab2[1]).powf(2.0) + (lab1[2] - lab2[2]).powf(2.0))
        .sqrt()
}

fn lightness_description(lightness: f64) -> &'static str {
    if lightness >= 90.0 {
        "Very Light "
    } else if lightness >= 70.0 {
        "Light "
    } else if lightness <= 20.0 {
        "Very Dark "
    } else if lightness <= 40.0 {
        "Dark "
    } else {
        ""
    }
}

fn saturation_description(saturation: f64) -> &'static str {
    if saturation < 5.0 {
        "Grayish "
    } else if saturation < 30.0 {
        "Muted "
    } else if saturation > 60.0 {
        "Vivid "
    } else {
        ""
    }
}

fn rgb_to_xyz(color: Color) -> [f64; 3] {
    let lin = |c: u8| {
        let v = c as f64 / 255.0;
        let v = if v > 0.04045 {
            ((v + 0.055) / 1.055).powf(2.4)
        } else {
            v / 12.92
        };
        v * 100.0
    };
    let (r, g, b) = (lin(color.red), lin(color.green), lin(color.blue));
    [
        r * 0.4124 + g * 0.3576 + b * 0.1805,
        r * 0.2126 + g * 0.7152 + b * 0.0722,
        r * 0.0193 + g * 0.1192 + b * 0.9505,
    ]
}

/// To sRGB 0..255, rounded half to even.
fn xyz_to_rgb(xyz: [f64; 3]) -> [f64; 3] {
    let x = xyz[0] / 100.0;
    let y = xyz[1] / 100.0;
    let z = xyz[2] / 100.0;
    let inv_r = 3.2406254773200533 * x - 1.5372079722103187 * y - 0.4986285986982479 * z;
    let inv_g = -0.9689307147293197 * x + 1.8757560608852415 * y + 0.041517523842953964 * z;
    let inv_b = 0.055710120445510616 * x + -0.2040210505984867 * y + 1.0569959422543882 * z;
    // Linear rgb must be gamma corrected to normalized srgb. Gamma correction is linear for
    // values <= 0.0031308 to avoid infinite log slope near zero.
    let compand = |c: f64| {
        if c <= 0.0031308 {
            12.92 * c
        } else {
            1.055 * c.powf(1.0 / 2.4) - 0.055
        }
    };
    // Add zero to prevent signed zeros (force 0 rather than -0)
    [
        (compand(inv_r) * 255.0).round_ties_even() + 0.0,
        (compand(inv_g) * 255.0).round_ties_even() + 0.0,
        (compand(inv_b) * 255.0).round_ties_even() + 0.0,
    ]
}

/// LAB to XYZ with the CIE constants `eps` = 216/24389 and `kap` = 24389/27, so dark colours
/// (L* below 8) take the linear branch.
fn lab_to_xyz(lab: [f64; 3]) -> [f64; 3] {
    // d65 standard illuminant in XYZ
    let d65 = [95.05, 100.0, 108.9];
    let (l, a, b) = (lab[0], lab[1], lab[2]);
    let eps = 216.0 / 24389.0;
    let kap = 24389.0 / 27.0;
    let f_y = (l + 16.0) / 116.0;
    let f_z = f_y - b / 200.0;
    let f_x = a / 500.0 + f_y;
    let x_r = if f_x.powf(3.0) > eps {
        f_x.powf(3.0)
    } else {
        (116.0 * f_x - 16.0) / kap
    };
    let y_r = if l > kap * eps {
        ((l + 16.0) / 116.0).powf(3.0)
    } else {
        l / kap
    };
    let z_r = if f_z.powf(3.0) > eps {
        f_z.powf(3.0)
    } else {
        (116.0 * f_z - 16.0) / kap
    };
    // Add zero to prevent signed zeros (force 0 rather than -0)
    [x_r * d65[0] + 0.0, y_r * d65[1] + 0.0, z_r * d65[2] + 0.0]
}

impl Color {
    /// The closest named colour, prefixed with lightness and saturation
    /// words when it is not a near match.
    pub fn color_to_name(&self) -> String {
        let lab = Self::rgb_to_lab(*self);
        let (name, closest) = find_closest_named_color(&lab);
        let distance = lab_distance(&lab, &Self::rgb_to_lab(closest));

        // Threshold for considering it a direct match
        if distance < 10.0 {
            return name.to_string();
        }

        let lightness_desc = lightness_description(lab[0]);
        let saturation = (lab[1] * lab[1] + lab[2] * lab[2]).sqrt();
        let saturation_desc = saturation_description(saturation);
        format!("{lightness_desc}{saturation_desc}{name}")
    }

    /// Euclidean distance in LAB.
    pub fn calculate_color_distance(&self, color2: Color) -> f64 {
        let lab1 = Self::rgb_to_lab(*self);
        let lab2 = Self::rgb_to_lab(color2);
        let l_diff = lab1[0] - lab2[0];
        let a_diff = lab1[1] - lab2[1];
        let b_diff = lab1[2] - lab2[2];
        (l_diff * l_diff + a_diff * a_diff + b_diff * b_diff).sqrt()
    }

    pub fn from_lab(l: f64, a: f64, b: f64) -> Color {
        let rgb = xyz_to_rgb(lab_to_xyz([l, a, b]));
        // Truncate the rounded doubles to integers, then the constructor clamps them.
        Color::new_rgb(rgb[0] as i32, rgb[1] as i32, rgb[2] as i32)
    }

    pub fn rgb_to_lab(color: Color) -> [f64; 3] {
        let xyz = rgb_to_xyz(color);
        Self::xyz_to_lab(xyz[0], xyz[1], xyz[2])
    }

    pub fn xyz_to_lab(x: f64, y: f64, z: f64) -> [f64; 3] {
        let f = |v: f64| {
            if v > 0.008856 {
                v.powf(1.0 / 3.0)
            } else {
                (7.787 * v) + (16.0 / 116.0)
            }
        };
        let var_x = f(x / 95.047);
        let var_y = f(y / 100.000);
        let var_z = f(z / 108.883);
        [
            (116.0 * var_y) - 16.0,
            500.0 * (var_x - var_y),
            200.0 * (var_y - var_z),
        ]
    }
}

/// `MinBy` keeps the first of equal distances.
fn find_closest_named_color(lab: &[f64; 3]) -> (&'static str, Color) {
    let mut best = DETAILED_COLORS[0];
    let mut best_distance = lab_distance(lab, &Color::rgb_to_lab(best.1));
    for &(name, color) in &DETAILED_COLORS[1..] {
        let distance = lab_distance(lab, &Color::rgb_to_lab(color));
        if distance < best_distance {
            best = (name, color);
            best_distance = distance;
        }
    }
    best
}
