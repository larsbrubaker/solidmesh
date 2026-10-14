// Color's JSON reader takes every shape a Color has ever been written in (the "#RRGGBB" string, a
// named color, the object with an Html property) and the degenerate values real customer files
// carry (an empty string, a null, an array). The After property pins reader positioning: if the
// reader leaves the stream partway inside the value it just read, the next property does not
// survive.

use agg_path::Color;
use serde::Deserialize;

/// No converter attribute here on purpose: Color's own Deserialize is the path customer files
/// take.
#[derive(Deserialize)]
#[serde(rename_all = "PascalCase")]
struct ColorHolder {
    #[serde(default)]
    color: Color,
    #[serde(default)]
    after: i32,
}

fn read(json: &str) -> ColorHolder {
    serde_json::from_str(json).expect("holder parses")
}

#[test]
fn empty_string_reads_as_transparent() {
    let holder = read("{\"Color\": \"\", \"After\": 7}");
    assert_eq!(holder.color, Color::TRANSPARENT);
    assert_eq!(holder.after, 7);
}

#[test]
fn html_string_reads_as_that_color() {
    let holder = read("{\"Color\": \"#FF0000\"}");
    assert_eq!(holder.color, Color::RED);
}

#[test]
fn named_color_string_reads_as_that_color() {
    let holder = read("{\"Color\": \"red\"}");
    assert_eq!(holder.color, Color::RED);
}

#[test]
fn object_with_html_property_reads_as_that_color() {
    let holder = read("{\"Color\": {\"Html\": \"#00FF00\"}, \"After\": 7}");
    assert_eq!(holder.color, Color::from_html_or_transparent("#00FF00"));
    assert_eq!(holder.after, 7);
}

#[test]
fn null_reads_as_transparent() {
    let holder = read("{\"Color\": null}");
    assert_eq!(holder.color, Color::TRANSPARENT);
}

#[test]
fn array_reads_as_transparent_and_is_skipped() {
    let holder = read("{\"Color\": [1, 2, 3], \"After\": 7}");
    assert_eq!(holder.color, Color::TRANSPARENT);
    assert_eq!(holder.after, 7);
}
