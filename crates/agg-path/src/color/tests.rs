//! Rust-only checks of the colour port (moved from mattercad-app's theme colour, plus the HSL,
//! naming and serialization paths), complementing the ported `ColorJsonConverterTests`.

use super::*;

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
