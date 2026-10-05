//! Ports `KernelBoundaryTests`: the kernel stays free of scene, UI and app code. C# reads the
//! assembly's references; here the crate's own `Cargo.toml` `[dependencies]` table is the same
//! fact.
//!
//! The C# forbidden assemblies map to the Rust crates that carry them: `Gui` to `agg-gui*`,
//! `Localizations`/`MatterCAD.Common`/`MatterCADLib` to the app and settings crates,
//! `DataConverters3D` to `mattercad-model`. The required ones are `PolygonMesh`, `VectorMath`
//! and `Agg`, the path model the `Path/` folder uses (here `agg-path`).

const FORBIDDEN_PREFIXES: [&str; 6] = [
    "agg-gui",
    "mattercad-app",
    "mattercad-model",
    "mattercad-settings",
    "mattercad-renderer",
    "mattercad-static-data",
];

fn dependency_names() -> Vec<String> {
    let manifest = include_str!("../Cargo.toml");
    let mut in_dependencies = false;
    let mut names = Vec::new();

    for line in manifest.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with('[') {
            in_dependencies = trimmed == "[dependencies]";
            continue;
        }

        if !in_dependencies || trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }

        if let Some((name, _)) = trimmed.split_once('=') {
            names.push(name.trim().to_string());
        }
    }

    names
}

#[test]
fn kernel_references_no_scene_ui_or_app_assembly() {
    let offenders: Vec<String> = dependency_names()
        .into_iter()
        .filter(|name| {
            FORBIDDEN_PREFIXES
                .iter()
                .any(|forbidden| name.starts_with(forbidden))
        })
        .collect();

    assert!(
        offenders.is_empty(),
        "geometry-processing depends on {}; the kernel must stay free of scene, UI and app crates",
        offenders.join(", ")
    );
}

#[test]
fn kernel_is_built_on_agg_mesh_and_vector_math() {
    let names = dependency_names();

    assert!(names.iter().any(|name| name == "polygon-mesh"));
    assert!(names.iter().any(|name| name == "vector-math"));
    assert!(names.iter().any(|name| name == "agg-path"));
}
