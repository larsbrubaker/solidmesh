//! Rust-only checks of the colours: every HTML form, the JSON string and its round trip,
//! blending, HSL, naming, LAB, contrast and hashing, complementing the ported
//! `ColorJsonConverterTests`.

use solidmesh_math::color::{Color, ColorF, ColorType};
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

fn hash_of<T: Hash>(value: &T) -> u64 {
    let mut hasher = DefaultHasher::new();
    value.hash(&mut hasher);
    hasher.finish()
}

#[test]
fn rust_only_parses_every_html_form() {
    assert_eq!(
        Color::from_html("#333"),
        Some(Color::rgba(0x33, 0x33, 0x33, 255))
    );
    assert_eq!(
        Color::from_html("#bbb3"),
        Some(Color::rgba(0xbb, 0xbb, 0xbb, 0x33))
    );
    assert_eq!(
        Color::from_html(" #FF7F00 "),
        Some(Color::rgba(255, 127, 0, 255))
    );
    assert_eq!(
        Color::from_html("#00000028"),
        Some(Color::rgba(0, 0, 0, 0x28))
    );
    assert_eq!(Color::from_html("#12345"), None);
    assert_eq!(Color::from_html("Grey"), Some(Color::GRAY));
    assert_eq!(
        Color::from_html_or_transparent("nonsense"),
        Color::TRANSPARENT
    );
    assert_eq!(Color::rgba(1, 2, 255, 16).html(), "#0102FF10");
}

#[test]
fn rust_only_writes_the_type_converter_string() {
    assert_eq!(serde_json::to_string(&Color::RED).unwrap(), "\"#FF0000\"");
    assert_eq!(
        serde_json::to_string(&Color::rgba(0, 0, 0, 0x28)).unwrap(),
        "\"#00000028\""
    );
}

#[test]
fn rust_only_blend_and_with_alpha() {
    let red = Color::from_html("#DC322F").unwrap();
    assert_eq!(red.blend(Color::WHITE, 0.0), red);
    assert_eq!(red.blend(Color::WHITE, 1.0), Color::WHITE);
    assert_eq!(red.with_alpha(300).alpha, 255);
    assert_eq!(red.with_alpha_f(0.5).alpha, 128);
}

#[test]
fn rust_only_hsl_round_trips() {
    for color in [Color::RED, Color::CRIMSON, Color::LIGHT_BLUE, Color::GRAY] {
        let (h, s, l) = color.to_color_f().get_hsl();
        assert_eq!(Color::from_hsl(h, s, l, 1.0), color, "{color}");
    }
}

#[test]
fn rust_only_names_and_contrast() {
    assert_eq!(Color::RED.color_to_name(), "Red");
    assert_eq!(Color::BLACK.contrast(&Color::WHITE), 21.0);
    let adjusted = Color::GRAY.with_contrast(&Color::WHITE, 4.5);
    assert!(adjusted.contrast(&Color::WHITE) >= 4.5);
}

#[test]
fn dark_colors_round_trip_through_lab() {
    // Below L* = 8 the LAB to XYZ conversion is linear; the cube-root branch there gives
    // colours several steps too bright.
    for value in [1, 5, 10, 20] {
        let color = Color::new_rgb(value, value, value);
        let lab = Color::rgb_to_lab(color);
        assert!(lab[0] < 8.0, "{value}: L* {}", lab[0]);
        let back = Color::from_lab(lab[0], lab[1], lab[2]);
        for (got, want) in [
            (back.red, color.red),
            (back.green, color.green),
            (back.blue, color.blue),
        ] {
            assert!(got.abs_diff(want) <= 1, "{value}: {back:?}");
        }
    }
}

#[test]
fn semi_transparent_background_contrast_measures_against_black_and_white() {
    // Half-transparent mid gray shows as dark gray on black and light gray on white; black
    // text is barely readable on the first and readable on the second, so the ratio lies
    // between the two opaque ratios rather than at the on-black one.
    let background = Color::new(128, 128, 128, 128);
    let on_black = background.overlay_on(&Color::BLACK);
    let on_white = background.overlay_on(&Color::WHITE);
    let low = on_black.contrast(&Color::BLACK);
    let high = on_white.contrast(&Color::BLACK);
    assert!(high > low + 5.0, "{low} {high}");
    let ratio = background.contrast(&Color::BLACK);
    assert!(ratio > low + 1.0 && ratio < high, "{low} {ratio} {high}");
}

#[test]
fn semi_transparent_text_is_overlaid_on_each_background_it_is_measured_against() {
    // The same half-transparent white text over a candidate background: the contrast with_contrast
    // reaches must be the contrast of the text composited on the colour it returns.
    let text = Color::new(255, 255, 255, 128);
    let adjusted = Color::new_rgb(200, 200, 200).with_contrast(&text, 1.6);
    let composited = text.overlay_on(&adjusted);
    assert!(adjusted.contrast(&composited) >= 1.6, "{adjusted:?}");
}

/// Saved files hold a colour as its JSON string, so writing one and reading it back must give
/// the same colour and the same text, opaque and translucent alike.
#[test]
fn rust_only_json_round_trips_exactly() {
    for (color, text) in [
        (Color::RED, "\"#FF0000\""),
        (Color::rgba(1, 2, 255, 16), "\"#0102FF10\""),
        (Color::rgba(0, 0, 0, 0), "\"#00000000\""),
        (Color::rgba(18, 52, 86, 255), "\"#123456\""),
    ] {
        let written = serde_json::to_string(&color).expect("a colour serializes");
        assert_eq!(written, text);
        let read: Color = serde_json::from_str(&written).expect("its own text reads back");
        assert_eq!(read, color);
        assert_eq!(
            serde_json::to_string(&read).expect("serializes again"),
            text
        );
    }
}

#[test]
fn test_get_hash_code() {
    {
        let a = Color::new_rgb(10, 11, 12);
        let b = Color::new_rgb(10, 11, 12);
        assert!(hash_of(&a) == hash_of(&b));
    }
    {
        let a = ColorF::new(10.0, 11.0, 12.0, 1.0);
        let b = ColorF::new(10.0, 11.0, 12.0, 1.0);
        assert!(hash_of(&a) == hash_of(&b));
    }
}
