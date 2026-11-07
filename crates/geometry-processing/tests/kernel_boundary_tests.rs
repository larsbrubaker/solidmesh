//! Kernel boundary tests: the bevel kernel stays a library of its own, read off the crate's own
//! `Cargo.toml`, every dependency table included (`[dependencies]`, `[dev-dependencies]`,
//! `[build-dependencies]` and their `[target.'cfg(..)'.*]` forms), so its tests build without the
//! app too.
//!
//! Forbidden are agg-path, the UI and the rest of agg (`agg-*`) and every MatterCAD app crate
//! (`mattercad-*`): the kernel takes contours and integer polygons, and the app's paths are
//! converted in mattercad-geometry. Of this workspace's own crates (path dependencies one level up)
//! only `polygon-mesh`, `vector-math` and `clipper-library` are allowed; the first two are
//! required.

const FORBIDDEN_PREFIXES: [&str; 2] = ["agg-", "mattercad-"];

const ALLOWED_WORKSPACE_CRATES: [&str; 3] = ["polygon-mesh", "vector-math", "clipper-library"];

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
fn the_kernel_depends_on_no_path_model_ui_or_app_crate() {
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
        "geometry-processing depends on {}; the kernel must stay free of agg and MatterCAD app \
         crates (convert the app's paths in mattercad-geometry)",
        offenders.join(", ")
    );
}

#[test]
fn the_kernel_uses_only_the_other_kernel_crates_of_this_workspace() {
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
        "geometry-processing depends on {}; of this workspace it may use only {}",
        offenders.join(", "),
        ALLOWED_WORKSPACE_CRATES.join(", ")
    );
}

#[test]
fn the_kernel_is_built_on_the_mesh_and_vector_math() {
    let all = dependencies();
    // The dev tables are read too, so the checks above cover the tests' dependencies.
    assert!(all
        .iter()
        .any(|dependency| dependency.table == "dev-dependencies"));
    let names: Vec<String> = all
        .into_iter()
        .filter(|dependency| dependency.table == "dependencies")
        .map(|dependency| dependency.name)
        .collect();

    assert!(names.iter().any(|name| name == "polygon-mesh"));
    assert!(names.iter().any(|name| name == "vector-math"));
}
