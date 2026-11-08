//! Platform maths gate: the 2D path bevel (`src/path`) and the topology readings (`src/topology`)
//! take their transcendental functions from the pure-Rust `libm` crate, never from std. std's
//! `f64::sin`, `cos`, `tan`, `acos`, `atan2`, `exp`, `ln`, `powf` and the rest call each
//! platform's own maths library, whose last bits differ between Windows, Linux, macOS and the
//! browser, so a design's curves and corner readings would differ by platform; `libm` is the same
//! everywhere. (`sqrt`, `abs` and the four operations are correctly rounded and stay std's.)
//!
//! Clippy cannot hold this line: its `disallowed-methods` list is one lint for the whole
//! workspace, also the one refusing `mul_add`, so listing std's trig there and allowing it in the
//! modules still using it (the 3D bevel in `src/bevel`, vector-math) would switch the `mul_add`
//! refusal off in those modules too. This test reads the two module trees instead, with comments
//! dropped, and refuses a std method call (`.sin(`) or path (`f64::sin`) of any of these names.

use std::fs;
use std::path::{Path, PathBuf};

/// std's `f64`/`f32` methods whose results depend on the platform's maths library.
const PLATFORM_DEPENDENT: [&str; 26] = [
    "sin", "cos", "tan", "sin_cos", "asin", "acos", "atan", "atan2", "sinh", "cosh", "tanh",
    "asinh", "acosh", "atanh", "exp", "exp2", "exp_m1", "ln", "ln_1p", "log", "log2", "log10",
    "powf", "powi", "hypot", "cbrt",
];

/// The module trees held to `libm`, relative to the crate.
const LIBM_ONLY: [&str; 2] = ["src/path", "src/topology"];

fn rust_files(dir: &Path, found: &mut Vec<PathBuf>) {
    let entries = fs::read_dir(dir).unwrap_or_else(|error| panic!("reading {dir:?}: {error}"));
    for entry in entries {
        let path = entry.expect("a directory entry").path();
        if path.is_dir() {
            rust_files(&path, found);
        } else if path.extension().is_some_and(|extension| extension == "rs") {
            found.push(path);
        }
    }
}

/// The line with its `//` comment (doc comments included) dropped. A `//` inside a string literal
/// would cut the line short too; these modules have none that hide a call.
fn code_of(line: &str) -> &str {
    line.split_once("//").map_or(line, |(code, _)| code)
}

fn is_identifier_char(c: char) -> bool {
    c.is_ascii_alphanumeric() || c == '_'
}

/// Every platform-dependent std call on the line: `.name(` as a method, `f64::name` or
/// `f32::name` as a path.
fn std_calls(code: &str) -> Vec<String> {
    let mut calls = Vec::new();
    for name in PLATFORM_DEPENDENT {
        if code.contains(&format!(".{name}(")) {
            calls.push(format!(".{name}("));
        }
        for float in ["f64", "f32"] {
            let path = format!("{float}::{name}");
            let bare = code.match_indices(&path).any(|(at, _)| {
                !code[at + path.len()..]
                    .chars()
                    .next()
                    .is_some_and(is_identifier_char)
            });
            if bare {
                calls.push(path);
            }
        }
    }
    calls
}

#[test]
fn the_path_bevel_and_topology_take_their_trig_from_libm() {
    let crate_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut files = Vec::new();
    for tree in LIBM_ONLY {
        rust_files(&crate_dir.join(tree), &mut files);
    }
    assert!(
        files.len() > 10,
        "expected the path and topology modules, found {files:?}"
    );

    let mut refused = Vec::new();
    for file in &files {
        let text = fs::read_to_string(file).unwrap_or_else(|error| panic!("{file:?}: {error}"));
        for (index, line) in text.lines().enumerate() {
            for call in std_calls(code_of(line)) {
                let relative = file.strip_prefix(crate_dir).unwrap_or(file);
                refused.push(format!("{}:{}: {call}", relative.display(), index + 1));
            }
        }
    }

    assert!(
        refused.is_empty(),
        "std's transcendental functions give different last bits on each platform, so the path \
         bevel and the topology readings call the `libm` crate's (`libm::atan2(y, x)` for \
         `y.atan2(x)`) to give a design the same curves everywhere:\n{}",
        refused.join("\n")
    );
}

#[test]
fn the_gate_sees_a_std_call_and_passes_a_libm_one() {
    assert_eq!(std_calls("let a = (x / 2.0).tan();"), vec![".tan("]);
    assert_eq!(std_calls("let a = y.atan2(x);"), vec![".atan2("]);
    assert_eq!(std_calls("map(f64::cos)"), vec!["f64::cos"]);
    assert!(std_calls("let a = libm::atan2(y, x) + libm::cos(t);").is_empty());
    assert!(std_calls("f64::consts::PI; f64::cosine_of(t)").is_empty());
    assert!(std_calls(code_of("let a = 1.0; // was x.sin()")).is_empty());
}
