// The scanner behind the `scope_reads` gate (its neighbour `../scope_reads.rs` holds the rule):
// finds where a crate's Rust source reads files outside its own directory. Reads by text, with no
// Rust or TOML parser, so it is deliberately eager: a read it cannot place is reported as reading
// the whole workspace.
//
// A source line is a read when it holds (comment lines skipped):
//   - `include_str!`/`include_bytes!` of a relative path that leaves the crate
//   - a string literal naming `crates/` (a crate's folder, or every crate's)
//   - a string literal climbing upward (`"../x"`, `"/../x"`, `".."`), as joined onto
//     `CARGO_MANIFEST_DIR`
//   - `CARGO_MANIFEST_DIR` with `.parent()` (the workspace's folders by another route)
// Each read is placed: a crate folder by name, a top-level path of the repo by its first
// component, or the whole workspace when the line names neither.

use std::collections::BTreeSet;
use std::fs;
use std::path::{Component, Path, PathBuf};

/// What a read reaches.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Target {
    /// Another workspace crate, by folder name under `crates/`.
    Crate(String),
    /// A top-level path of the repository (its first component), outside `crates/`.
    Root(String),
    /// Somewhere the scanner cannot place: every crate, or the workspace root.
    Workspace,
}

/// One read: the file (repo-relative, `/`-separated), its line number and what it reaches.
#[derive(Clone, Debug)]
pub struct Read {
    pub file: String,
    pub line: usize,
    pub target: Target,
}

/// The workspace crates by folder name (every `crates/<dir>/Cargo.toml`).
pub fn crate_dirs(root: &Path) -> BTreeSet<String> {
    let mut dirs = BTreeSet::new();
    if let Ok(entries) = fs::read_dir(root.join("crates")) {
        for entry in entries.flatten() {
            if entry.path().join("Cargo.toml").is_file() {
                dirs.insert(entry.file_name().to_string_lossy().into_owned());
            }
        }
    }
    dirs
}

/// Every `.rs` file under `crates/<dir>`, build output left out, repo-relative.
pub fn crate_sources(root: &Path, dir: &str) -> Vec<String> {
    fn walk(path: &Path, found: &mut Vec<PathBuf>) {
        let Ok(entries) = fs::read_dir(path) else {
            return;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                if entry.file_name() != "target" {
                    walk(&path, found);
                }
            } else if path.extension().is_some_and(|e| e == "rs") {
                found.push(path);
            }
        }
    }
    let mut found = Vec::new();
    walk(&root.join("crates").join(dir), &mut found);
    let mut files: Vec<String> = found
        .iter()
        .filter_map(|p| p.strip_prefix(root).ok())
        .map(|p| p.to_string_lossy().replace('\\', "/"))
        .collect();
    files.sort();
    files
}

/// The string literals on a line, roughly: the text between each pair of unescaped quotes.
fn literals(line: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut current: Option<String> = None;
    let mut escaped = false;
    for c in line.chars() {
        match current.as_mut() {
            None if c == '"' => current = Some(String::new()),
            None => {}
            Some(text) if escaped => {
                text.push(c);
                escaped = false;
            }
            Some(_) if c == '\\' => escaped = true,
            Some(_) if c == '"' => out.push(current.take().unwrap_or_default()),
            Some(text) => text.push(c),
        }
    }
    out
}

/// Places a repo-relative path (already normalised) as read from crate <own>.
fn place(path: &str, own: &str, crates: &BTreeSet<String>) -> Option<Target> {
    let mut parts = path.split('/').filter(|p| !p.is_empty());
    match parts.next() {
        None => Some(Target::Workspace),
        Some("crates") => match parts.next() {
            Some(dir) if dir == own => None,
            Some(dir) if crates.contains(dir) => Some(Target::Crate(dir.to_string())),
            // `crates/`, `crates/*`, `crates/{name}`: every crate, or one picked at run time.
            None => Some(Target::Workspace),
            Some(dir) if dir.contains(['*', '{']) => Some(Target::Workspace),
            // A made-up crate (`crates/x/src/lib.rs` in a report under test) is no read here.
            Some(_) => None,
        },
        Some(first) => Some(Target::Root(first.to_string())),
    }
}

/// Resolves <relative> against the repo-relative folder <base>; None when it climbs above the root.
fn resolve(base: &str, relative: &str) -> Option<String> {
    let mut parts: Vec<String> = base
        .split('/')
        .filter(|p| !p.is_empty())
        .map(String::from)
        .collect();
    for component in Path::new(relative).components() {
        match component {
            Component::ParentDir => {
                parts.pop()?;
            }
            Component::Normal(name) => parts.push(name.to_string_lossy().into_owned()),
            _ => {}
        }
    }
    Some(parts.join("/"))
}

/// The repository's top-level names (`static-data`, `Cargo.lock`, ...).
pub fn top_level(root: &Path) -> BTreeSet<String> {
    fs::read_dir(root)
        .map(|entries| {
            entries
                .flatten()
                .map(|e| e.file_name().to_string_lossy().into_owned())
                .collect()
        })
        .unwrap_or_default()
}

/// Whether a repo-relative source file is test code: under a `tests` folder, or named `tests.rs`
/// or `*_tests.rs`.
pub fn is_test_source(file: &str) -> bool {
    file.split('/').any(|part| part == "tests")
        || file.ends_with("/tests.rs")
        || file.ends_with("_tests.rs")
}

/// Where the literals on <lines> point when one starts with a crate's folder name or a top-level
/// name of the repository: the second half of an upward climb written over several lines.
fn named_on(lines: &[&str], crates: &BTreeSet<String>, top: &BTreeSet<String>) -> BTreeSet<Target> {
    let mut found = BTreeSet::new();
    for line in lines {
        for literal in literals(line) {
            let first = literal.split('/').next().unwrap_or("");
            if crates.contains(first) {
                found.insert(Target::Crate(first.to_string()));
            } else if !first.is_empty() && first != "crates" && top.contains(first) {
                found.insert(Target::Root(first.to_string()));
            }
        }
    }
    found
}

/// The reads in one source file of crate <own> (its text given). Only includes count outside test
/// code (<tests> false): production code reading the repository at run time is not a test's input.
pub fn reads_in(
    file: &str,
    text: &str,
    own: &str,
    crates: &BTreeSet<String>,
    top: &BTreeSet<String>,
) -> Vec<Read> {
    let folder = file.rsplit_once('/').map_or("", |(f, _)| f);
    let manifest_dir = format!("crates/{own}");
    let tests = is_test_source(file);
    let all: Vec<&str> = text.lines().collect();
    let mut reads = Vec::new();
    for (index, line) in all.iter().enumerate() {
        let trimmed = line.trim_start();
        if trimmed.starts_with("//") || trimmed.starts_with("*") {
            continue;
        }
        let mut targets = BTreeSet::new();
        // Paths relative to the source file itself: an include, and a `#[path]` module.
        let included = line.contains("include_str!")
            || line.contains("include_bytes!")
            || trimmed.starts_with("#[path");
        for literal in literals(line) {
            if included && !literal.contains("CARGO_MANIFEST_DIR") {
                if let Some(path) = resolve(folder, &literal) {
                    if !path.starts_with(&format!("{manifest_dir}/")) {
                        targets.extend(place(&path, own, crates));
                    }
                } else {
                    targets.insert(Target::Workspace);
                }
            } else if !tests {
                continue;
            } else if literal == "crates" {
                // `root.join("crates")`: placed by what is joined next (below).
                targets.insert(Target::Workspace);
            } else if literal.contains("crates/") {
                let after = &literal[literal.find("crates/").unwrap_or(0)..];
                targets.extend(place(after, own, crates));
            } else if literal.starts_with("..") || literal.starts_with("/..") {
                let climb = literal.trim_start_matches('/');
                let named = climb.split('/').find(|p| !p.is_empty() && *p != "..");
                // Joined onto the crate's manifest folder (named on this line or the three
                // before it), the climb certainly leaves the crate.
                let from = index.saturating_sub(3);
                let from_manifest = all[from..=index]
                    .iter()
                    .any(|l| l.contains("CARGO_MANIFEST_DIR"));
                match named {
                    // Only dots: placed by what is joined next (below).
                    None if from_manifest => {
                        targets.insert(Target::Workspace);
                    }
                    None => {}
                    // A climb to a crate folder or a top-level name of this repository.
                    Some(name) if crates.contains(name) => {
                        if name != own {
                            targets.insert(Target::Crate(name.to_string()));
                        }
                    }
                    Some(name) if top.contains(name) && name != "crates" => {
                        targets.insert(Target::Root(name.to_string()));
                    }
                    // Any other name climbed to from the manifest folder: a path outside the
                    // repository's current tree (a fixture folder that never moved in, a sibling
                    // checkout), which a published crate's tests cannot reach either.
                    Some(name) if from_manifest => {
                        targets.insert(Target::Root(name.to_string()));
                    }
                    // Not joined onto the manifest folder: a path under test, not a read.
                    Some(_) => {}
                }
            }
        }
        if tests && line.contains("CARGO_MANIFEST_DIR") && line.contains(".parent()") {
            targets.insert(Target::Workspace);
        }
        // An upward climb that lands on `crates/` or the root reads whatever is joined next: a
        // crate or top-level name on the same line or the five after it places it.
        if targets.contains(&Target::Workspace) {
            let named = named_on(&all[index..all.len().min(index + 6)], crates, top);
            if !named.is_empty() {
                targets.remove(&Target::Workspace);
                targets.extend(
                    named
                        .into_iter()
                        .filter(|t| *t != Target::Crate(own.to_string())),
                );
            }
        }
        for target in targets {
            reads.push(Read {
                file: file.to_string(),
                line: index + 1,
                target,
            });
        }
    }
    reads
}
