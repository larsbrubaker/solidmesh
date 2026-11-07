//! Which models the Thingi10K sweep draws from the corpus, and where each one's mesh lives. The
//! dataset is read IN PLACE and nothing is copied into this repository.

use std::path::{Path, PathBuf};

use vector_math::seeded_random::SeededRandom;

/// One record of the Thingi10K metadata: which model, which repository shard, how many faces.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Thingi10KModel {
    pub id: i32,
    pub repo: i32,
    pub faces: i32,
}

/// The clean records, shuffled by the seed and cut to the count asked for. Clean is the metadata's
/// own word for it: closed, edge and vertex manifold, one component, positively wound and normal,
/// and no degenerate or duplicated faces - and an STL, because that is all this reader reads.
pub fn chosen(
    metadata: &Path,
    max_faces: i32,
    seed: i32,
    models: i32,
) -> Result<Vec<Thingi10KModel>, String> {
    let text = std::fs::read_to_string(metadata).map_err(|e| e.to_string())?;
    let document: serde_json::Value = serde_json::from_str(&text).map_err(|e| e.to_string())?;
    let records = document
        .as_array()
        .ok_or_else(|| "the metadata is not an array of records".to_string())?;

    let mut clean = Vec::new();
    for record in records {
        let flag = |name: &str| record.get(name).and_then(|v| v.as_bool()) == Some(true);

        if !flag("closed")
            || !flag("edge_manifold")
            || !flag("vertex_manifold")
            || !flag("single_component")
            || !flag("pwn")
        {
            continue;
        }

        if flag("degenerate_faces") || flag("duplicated_faces") {
            continue;
        }

        if record.get("format").and_then(|v| v.as_str()) != Some("stl") {
            continue;
        }

        let number = |name: &str| -> Result<i32, String> {
            record
                .get(name)
                .and_then(|v| v.as_i64())
                .map(|v| v as i32)
                .ok_or_else(|| format!("a record has no integer \"{name}\""))
        };
        let faces = number("faces")?;
        if faces > max_faces {
            continue;
        }

        clean.push(Thingi10KModel {
            id: number("id")?,
            repo: number("repo")?,
            faces,
        });
    }

    // A named handful, for going back to the rows a past sweep flagged without re-walking the draw.
    if let Ok(only) = std::env::var("THINGI_ONLY") {
        if !only.trim().is_empty() {
            let wanted: Vec<i32> = only
                .split([';', ','])
                .filter(|t| !t.is_empty())
                .map(|t| {
                    t.trim()
                        .parse::<i32>()
                        .map_err(|_| format!("THINGI_ONLY holds \"{t}\", which is not a model id"))
                })
                .collect::<Result<_, _>>()?;
            return Ok(clean
                .into_iter()
                .filter(|m| wanted.contains(&m.id))
                .collect());
        }
    }

    let mut random = SeededRandom::new(seed);
    for i in (1..clean.len()).rev() {
        let j = random.next(i as i32 + 1) as usize;
        clean.swap(i, j);
    }

    clean.truncate(models.max(0) as usize);
    Ok(clean)
}

pub fn mesh_path(root: &Path, model: &Thingi10KModel) -> PathBuf {
    root.join("meshes")
        .join(format!("Thingi10K-meshes-{}", model.repo))
        .join("meshes")
        .join(format!("{}.stl.zip", model.id))
}
