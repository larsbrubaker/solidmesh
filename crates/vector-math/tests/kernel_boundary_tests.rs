//! Kernel boundary tests: the math library under the mesh kernel depends on no crate of this
//! workspace at all, read off the crate's own `Cargo.toml`, every dependency table included
//! (`[dependencies]`, `[dev-dependencies]`, `[build-dependencies]` and their
//! `[target.'cfg(..)'.*]` forms), so it and its tests build on their own.
//!
//! Forbidden are agg (`agg-*`) and every MatterCAD app crate (`mattercad-*`), and of this
//! workspace's own crates (path dependencies one level up) none is allowed.

const FORBIDDEN_PREFIXES: [&str; 2] = ["agg-", "mattercad-"];

const ALLOWED_WORKSPACE_CRATES: [&str; 0] = [];

/// One dependency: the table it is in, its name, and its `path` if it has one.
struct Dependency {
    table: String,
    name: String,
    path: Option<String>,
}

/// Every dependency in every dependency table of the manifest.
fn dependencies() -> Vec<Dependency> {
    let manifest = include_str!("../Cargo.toml");
    let mut table = String::new();
    let mut in_dependencies = false;
    // A `[dependencies.name]` table: the dependency its keys belong to.
    let mut dotted: Option<usize> = None;
    let mut found: Vec<Dependency> = Vec::new();

    for line in manifest.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with('[') {
            table = trimmed.trim_matches(|c| c == '[' || c == ']').to_string();
            in_dependencies = table.ends_with("dependencies");
            dotted = None;
            let dotted_name = ["dependencies.", "dev-dependencies.", "build-dependencies."]
                .iter()
                .find_map(|prefix| table.split_once(prefix).map(|(_, name)| name.to_string()));
            if let Some(name) = dotted_name {
                found.push(Dependency {
                    table: table.clone(),
                    name,
                    path: None,
                });
                dotted = Some(found.len() - 1);
            }
            continue;
        }
        if trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }
        let Some((key, value)) = trimmed.split_once('=') else {
            continue;
        };
        let key = key.trim();
        if let Some(index) = dotted {
            if key == "path" {
                found[index].path = Some(value.trim().trim_matches('"').to_string());
            }
        } else if in_dependencies {
            found.push(Dependency {
                table: table.clone(),
                name: key.to_string(),
                path: path_in(value),
            });
        }
    }

    found
}

/// The `path = "..."` of an inline dependency table, if it has one.
fn path_in(value: &str) -> Option<String> {
    let (_, after) = value.split_once("path")?;
    let (_, quoted) = after.split_once('"')?;
    let (path, _) = quoted.split_once('"')?;
    Some(path.to_string())
}

/// A path dependency on a crate of this workspace: one level up (`../crate`), where the
/// sibling checkouts are further out (`../../../name`).
fn is_workspace_crate(dependency: &Dependency) -> bool {
    dependency
        .path
        .as_deref()
        .and_then(|path| path.strip_prefix("../"))
        .is_some_and(|rest| !rest.starts_with(".."))
}

#[test]
fn the_crate_depends_on_no_path_model_ui_or_app_crate() {
    let offenders: Vec<String> = dependencies()
        .into_iter()
        .filter(|dependency| {
            FORBIDDEN_PREFIXES
                .iter()
                .any(|forbidden| dependency.name.starts_with(forbidden))
        })
        .map(|dependency| format!("{} in [{}]", dependency.name, dependency.table))
        .collect();

    assert!(
        offenders.is_empty(),
        "vector-math depends on {}; the mesh kernel must stay free of agg and MatterCAD app crates          (convert the app's paths and images in mattercad-geometry)",
        offenders.join(", ")
    );
}

#[test]
fn the_crate_uses_only_the_kernel_crates_beneath_it() {
    let offenders: Vec<String> = dependencies()
        .into_iter()
        .filter(|dependency| {
            is_workspace_crate(dependency)
                && !ALLOWED_WORKSPACE_CRATES.contains(&dependency.name.as_str())
        })
        .map(|dependency| format!("{} in [{}]", dependency.name, dependency.table))
        .collect();

    assert!(
        offenders.is_empty(),
        "vector-math depends on {}; of this workspace it may use only {:?}",
        offenders.join(", "),
        ALLOWED_WORKSPACE_CRATES
    );
}

#[test]
fn the_manifest_is_read() {
    // A dependency the crate certainly has, so the checks above are not passing on an empty read.
    assert!(dependencies()
        .iter()
        .any(|dependency| dependency.table == "dependencies" && dependency.name == "serde"));
}
