// Browser clock gate: library code never reads std's clock. On
// wasm32-unknown-unknown `std::time::Instant::now()` and `std::time::SystemTime::now()` panic
// ("time not implemented on this platform"), and a panic takes the whole page down, so every
// clock read goes through `web_time`, which is std's own clock natively and the browser's on the
// web. Every library crate builds for wasm32-unknown-unknown (CLAUDE.md, "Never"), so this holds
// for all of them. The gate reads the sources rather than leaning on clippy's
// `disallowed-methods`: natively `web_time::Instant` *is* `std::time::Instant` (a re-export), so a
// native lint cannot tell the safe spelling from the panicking one, and this gate runs with every
// native test run.
//
// Checked: every `.rs` file under `crates/<crate>/src/` of every crate folder, so a crate that
// moves in is checked at once, except the native-only crates (`NATIVE_ONLY_CRATES`: these gates)
// and test-only files (`tests.rs`, `*_tests.rs`), which never reach the browser. Integration
// tests (`crates/<crate>/tests/`) are not under `src/` and run natively only. A file fails when,
// outside `//` comments, it
//
//   (a) names `std::time::Instant` at all (std's `Instant` has no use but its clock), in a path
//       or a `use std::time::{...}` list;
//   (b) calls `std::time::SystemTime::now`; or
//   (c) imports `SystemTime` from `std::time` and calls a bare `SystemTime::now`.
//
// std's `SystemTime` as a type stays allowed: it is what the file system reports (`modified()`,
// `accessed()`), and code comparing such a time with the clock does so through durations since
// 1970, which are the same type on both builds.

use std::fs;
use std::path::{Path, PathBuf};

/// Crate folders that never build for the browser: the gates, which run only as native tests.
const NATIVE_ONLY_CRATES: [&str; 1] = ["workspace-gates"];

fn crates_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("crates/workspace-gates sits in crates/")
        .to_path_buf()
}

fn collect_rs(dir: &Path, out: &mut Vec<PathBuf>) {
    let entries = fs::read_dir(dir).unwrap_or_else(|e| panic!("read {}: {e}", dir.display()));
    for entry in entries {
        let path = entry.expect("directory entry").path();
        if path.is_dir() {
            collect_rs(&path, out);
        } else if path.extension().is_some_and(|e| e == "rs") {
            out.push(path);
        }
    }
}

/// True for a file that only builds under `cfg(test)`.
fn is_test_file(path: &Path) -> bool {
    let name = path.file_name().and_then(|n| n.to_str()).unwrap_or("");
    name == "tests.rs" || name.ends_with("_tests.rs")
}

/// `text` with every `//` comment removed, line by line. A `//` inside a string (a URL) cuts the
/// rest of that line too, which can only hide code, never invent a hit.
fn without_comments(text: &str) -> String {
    text.lines()
        .map(|line| line.split("//").next().unwrap_or(""))
        .collect::<Vec<_>>()
        .join("\n")
}

/// The names a `use std::time::...;` statement brings in (`Instant`, `SystemTime`, ...), for every
/// such statement in `code`, braced lists over several lines included.
fn std_time_imports(code: &str) -> Vec<String> {
    let mut names = Vec::new();
    let mut rest = code;
    while let Some(at) = rest.find("use std::time::") {
        let after = &rest[at + "use std::time::".len()..];
        let end = after.find(';').unwrap_or(after.len());
        let list = &after[..end];
        names.extend(
            list.split(|c: char| !(c.is_alphanumeric() || c == '_'))
                .filter(|w| !w.is_empty())
                .map(str::to_string),
        );
        rest = &after[end..];
    }
    names
}

/// True when `code` calls `name::now` bare: not as the tail of a longer path (`web_time::`,
/// `std::time::`) or of another identifier.
fn calls_bare_now(code: &str, name: &str) -> bool {
    let needle = format!("{name}::now");
    code.match_indices(&needle).any(|(at, _)| {
        let before = &code[..at];
        !(before.ends_with("::") || before.ends_with(|c: char| c.is_alphanumeric() || c == '_'))
    })
}

/// Every reason `source` (a file the browser build compiles) reads std's clock.
fn std_clock_reads(source: &str) -> Vec<&'static str> {
    let code = without_comments(source);
    let imports = std_time_imports(&code);
    let mut reasons = Vec::new();
    if code.contains("std::time::Instant") || imports.iter().any(|n| n == "Instant") {
        reasons.push("uses std::time::Instant (use web_time::Instant)");
    }
    if code.contains("std::time::SystemTime::now") {
        reasons.push("calls std::time::SystemTime::now (use web_time::SystemTime::now)");
    }
    if imports.iter().any(|n| n == "SystemTime") && calls_bare_now(&code, "SystemTime") {
        reasons.push("calls SystemTime::now on std's SystemTime (use web_time::SystemTime::now)");
    }
    reasons
}

#[test]
fn web_built_code_never_reads_the_std_clock() {
    let root = crates_dir();
    let mut failures = Vec::new();
    let mut crates: Vec<PathBuf> = fs::read_dir(&root)
        .expect("read crates/")
        .map(|e| e.expect("crate entry").path())
        .filter(|p| p.join("src").is_dir())
        .collect();
    crates.sort();
    // Guards against a walk that silently checks nothing (wrong root): the gates' own folder is
    // found, and every library crate found has at least one file checked.
    let names: Vec<&str> = crates
        .iter()
        .filter_map(|k| k.file_name().and_then(|n| n.to_str()))
        .collect();
    assert!(
        names.contains(&"workspace-gates"),
        "the walk of {} did not find crates/workspace-gates",
        root.display()
    );
    let mut unchecked = Vec::new();
    for krate in &crates {
        let name = krate.file_name().and_then(|n| n.to_str()).unwrap_or("");
        if NATIVE_ONLY_CRATES.contains(&name) {
            continue;
        }
        let mut files = Vec::new();
        collect_rs(&krate.join("src"), &mut files);
        files.sort();
        let mut checked = 0;
        for file in files.into_iter().filter(|f| !is_test_file(f)) {
            let text = fs::read_to_string(&file)
                .unwrap_or_else(|e| panic!("read {}: {e}", file.display()));
            checked += 1;
            for reason in std_clock_reads(&text) {
                let relative = file.strip_prefix(&root).unwrap_or(&file);
                failures.push(format!("crates/{}: {reason}", relative.display()));
            }
        }
        if checked == 0 {
            unchecked.push(name.to_string());
        }
    }
    assert!(
        unchecked.is_empty(),
        "no library source was checked in {unchecked:?}"
    );
    assert!(
        failures.is_empty(),
        "std's clock panics in the browser build; read the clock through web_time:\n  {}",
        failures.join("\n  ")
    );
}

#[test]
fn a_std_instant_import_is_caught() {
    let source = "use std::time::{Duration, Instant};\nfn f() { let t = Instant::now(); }";
    assert_eq!(std_clock_reads(source).len(), 1);
    let multi_line = "use std::time::{\n    Duration,\n    Instant,\n};\n";
    assert_eq!(std_clock_reads(multi_line).len(), 1);
    assert_eq!(
        std_clock_reads("let t = std::time::Instant::now();").len(),
        1
    );
}

#[test]
fn a_std_system_time_clock_read_is_caught() {
    assert_eq!(
        std_clock_reads("let t = std::time::SystemTime::now();").len(),
        1
    );
    let imported = "use std::time::{Duration, SystemTime};\nlet t = SystemTime::now();";
    assert_eq!(std_clock_reads(imported).len(), 1);
}

#[test]
fn web_time_clock_reads_and_file_times_pass() {
    let source = "use web_time::{Duration, Instant};\n\
                  use std::time::SystemTime;\n\
                  fn f(modified: SystemTime) { let t = Instant::now();\n\
                  let n = web_time::SystemTime::now(); }";
    assert!(std_clock_reads(source).is_empty());
    // A comment naming std's clock is not a clock read.
    assert!(std_clock_reads("// std::time::Instant::now() panics in the browser").is_empty());
}
