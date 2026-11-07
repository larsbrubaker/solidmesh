// Test reads stay inside their crate (CLAUDE.md, "Working"). Each library crate is published on
// its own, and the package crates.io gets holds only that crate's folder: a test that reads a
// fixture from the repository root, from another crate or from a sibling checkout passes here and
// fails for anyone testing the published crate, and it hides a dependency between crates that the
// manifests do not state. So a crate's test code reads files only below its own
// `crates/<crate>/` folder; a fixture two crates share is copied into both, or the test moves to
// the crate that owns the fixture.
//
// The scanner is `support/source_reads.rs`: it finds every read outside the crate in each crate's
// test code (under a `tests` folder, `tests.rs`, `*_tests.rs`) and every `include_str!`,
// `include_bytes!` or `#[path]` that leaves the crate anywhere. It walks every crate folder under
// `crates/`, so a crate that moves in is checked at once. workspace-gates itself is skipped: its
// tests read the whole repository by design.

#[path = "support/repo_walk.rs"]
#[allow(dead_code)]
mod repo_walk;
#[path = "support/source_reads.rs"]
mod source_reads;

use std::collections::BTreeSet;
use std::fs;

use source_reads::{crate_dirs, crate_sources, reads_in, top_level, Read, Target};

/// Every read outside its own crate, as "file:line reads target".
fn reads_outside_their_crate() -> Vec<String> {
    let root = repo_walk::workspace_root();
    let crates = crate_dirs(&root);
    let top = top_level(&root);
    let mut offenses = Vec::new();
    for own in crates.iter().filter(|c| *c != "workspace-gates") {
        for file in crate_sources(&root, own) {
            let Ok(text) = fs::read_to_string(root.join(&file)) else {
                continue;
            };
            for Read { file, line, target } in reads_in(&file, &text, own, &crates, &top) {
                offenses.push(format!("{file}:{line} reads {target:?}"));
            }
        }
    }
    offenses
}

#[test]
fn every_test_reads_only_inside_its_own_crate() {
    let offenses = reads_outside_their_crate();
    assert!(
        offenses.is_empty(),
        "these files read outside their own crate, which a published crate's tests cannot do (its \
         package holds only its own folder). Move or copy the file they read into the crate, \
         e.g. crates/<crate>/tests/fixtures/, and read it from CARGO_MANIFEST_DIR:\n{}",
        offenses.join("\n")
    );
}

#[test]
fn the_scan_walks_the_crates() {
    // Guards against a scan that silently checks nothing (wrong root).
    let root = repo_walk::workspace_root();
    let crates = crate_dirs(&root);
    assert!(
        crates.contains("workspace-gates"),
        "crates found: {crates:?}"
    );
    assert!(crate_sources(&root, "workspace-gates")
        .iter()
        .any(|f| f == "crates/workspace-gates/tests/scope_reads.rs"));
}

#[test]
fn the_scanner_places_each_kind_of_read() {
    let crates: BTreeSet<String> = ["a", "b", "own"].iter().map(|s| s.to_string()).collect();
    let top: BTreeSet<String> = ["crates", "test-data", "docs"]
        .iter()
        .map(|s| s.to_string())
        .collect();
    let file = "crates/own/tests/area/x_tests.rs";
    let targets = |text: &str| -> Vec<Target> {
        reads_in(file, text, "own", &crates, &top)
            .into_iter()
            .map(|r| r.target)
            .collect()
    };
    assert_eq!(
        targets(r#"let p = "../b/tests/fixtures";"#),
        vec![Target::Crate("b".into())]
    );
    assert_eq!(
        targets(r#"root.join("crates/a/src")"#),
        vec![Target::Crate("a".into())]
    );
    assert_eq!(targets(r#"root.join("crates/")"#), vec![Target::Workspace]);
    assert_eq!(
        targets(r#"include_str!("../../../a/src/lib.rs")"#),
        vec![Target::Crate("a".into())]
    );
    assert_eq!(
        targets(r#"#[path = "../../../b/tests/x.rs"]"#),
        vec![Target::Crate("b".into())]
    );
    assert_eq!(
        targets(r#"include_str!("../../../../test-data/x")"#),
        vec![Target::Root("test-data".into())]
    );
    assert_eq!(
        targets("let f = concat!(\n    env!(\"CARGO_MANIFEST_DIR\"),\n    \"/../../test-data/x.svgd\"\n);"),
        vec![Target::Root("test-data".into())]
    );
    assert_eq!(
        targets("let r = Path::new(env!(\"CARGO_MANIFEST_DIR\"))\n    .join(\"..\");\nlet f = r.join(\"a/src/lib.rs\");"),
        vec![Target::Crate("a".into())]
    );
    assert_eq!(
        targets("Path::new(env!(\"CARGO_MANIFEST_DIR\")).join(\"../..\").join(\"docs/x.md\")"),
        vec![Target::Root("docs".into())]
    );
    assert_eq!(
        targets("Path::new(env!(\"CARGO_MANIFEST_DIR\")).join(\"..\")"),
        vec![Target::Workspace]
    );
    // Not reads: dots that are no climb from the crate, made-up crates, a crate's own files.
    assert!(targets(r#"text.replace("..", ",")"#).is_empty());
    assert!(targets(r#"report("crates/x/src/lib.rs:10")"#).is_empty());
    assert!(targets(r#"include_str!("../fixtures/a.txt")"#).is_empty());
    assert!(targets(r#"include_str!("../../Cargo.toml")"#).is_empty());
    // A folder this repository does not have (a fixture tree that never moved in) is still a
    // read when it is climbed to from the manifest folder; without the manifest folder a climb
    // is a path under test.
    assert_eq!(
        targets("let f = concat!(\n    env!(\"CARGO_MANIFEST_DIR\"),\n    \"/../../gone/x\"\n);"),
        vec![Target::Root("gone".into())]
    );
    assert!(targets(r#"normalize("../../gone/x")"#).is_empty());
    assert!(
        targets(r#"format!("{}/tests/fixtures/a.stl", env!("CARGO_MANIFEST_DIR"))"#).is_empty()
    );
    assert!(targets(r#"// root.join("crates/a")"#).is_empty());
}
