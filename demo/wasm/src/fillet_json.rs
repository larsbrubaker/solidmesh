//! The Fillet & Chamfer page's answers as JSON text, which the worker parses and posts to the
//! page: a prepared part's feature edges and corners ([`prepared`]), one pick ([`pick`]) and a
//! build's rows ([`rows`]). Written by hand because the shapes are small and fixed; numbers are
//! written in Rust's shortest round-trip form, so a pick's group comes back to the build bit for
//! bit.

use std::fmt::Write;

use vector_math::Vector3;

use crate::fillet::PickRow;
use crate::fillet_pick::{group_to_numbers, Pick, PickSession};

/// A number as JSON: non-finite values (which JSON cannot hold) as null.
fn number(out: &mut String, value: f64) {
    if value.is_finite() {
        let _ = write!(out, "{value:?}");
    } else {
        out.push_str("null");
    }
}

/// A string as JSON, escaped.
fn string(out: &mut String, value: &str) {
    out.push('"');
    for c in value.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            c if (c as u32) < 0x20 => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            c => out.push(c),
        }
    }
    out.push('"');
}

/// A flat array of numbers.
fn numbers(out: &mut String, values: impl IntoIterator<Item = f64>) {
    out.push('[');
    for (i, value) in values.into_iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        number(out, value);
    }
    out.push(']');
}

/// Segments as one flat array, six numbers each.
fn segments(out: &mut String, segments: &[[Vector3; 2]]) {
    numbers(
        out,
        segments
            .iter()
            .flat_map(|[a, b]| [a.x, a.y, a.z, b.x, b.y, b.z]),
    );
}

/// `{"features": [...], "corners": [x, y, z, allowsFillet, ...]}` for a prepared part.
pub fn prepared(session: &PickSession) -> String {
    let mut out = String::from("{\"features\":");
    segments(&mut out, &session.feature_segments());
    out.push_str(",\"corners\":");
    numbers(
        &mut out,
        session
            .corners()
            .iter()
            .flat_map(|(p, allows_fillet)| [p.x, p.y, p.z, if *allows_fillet { 1.0 } else { 0.0 }]),
    );
    out.push('}');
    out
}

/// `{"hit": false}`, or the pick: its group as numbers, its segments, its corner and its label.
pub fn pick(pick: Option<&Pick>) -> String {
    let Some(pick) = pick else {
        return "{\"hit\":false}".to_string();
    };
    let mut out = String::from("{\"hit\":true,\"group\":");
    numbers(&mut out, group_to_numbers(&pick.group));
    out.push_str(",\"segments\":");
    segments(&mut out, &pick.segments);
    out.push_str(",\"point\":");
    match pick.point {
        Some(p) => numbers(&mut out, [p.x, p.y, p.z]),
        None => out.push_str("null"),
    }
    out.push_str(",\"label\":");
    string(&mut out, &pick.label);
    out.push('}');
    out
}

/// `{"rows": [{"built", "edges", "text", "fits", "code"}, ...]}`.
pub fn rows(rows: &[PickRow]) -> String {
    let mut out = String::from("{\"rows\":[");
    for (i, row) in rows.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        let _ = write!(
            out,
            "{{\"built\":{},\"edges\":{},\"text\":",
            row.built, row.edge_count
        );
        string(&mut out, &row.text);
        out.push_str(",\"fits\":");
        match row.fits {
            Some(fits) => number(&mut out, fits),
            None => out.push_str("null"),
        }
        out.push_str(",\"code\":");
        match &row.code {
            Some(code) => string(&mut out, code),
            None => out.push_str("null"),
        }
        out.push('}');
    }
    out.push_str("]}");
    out
}
