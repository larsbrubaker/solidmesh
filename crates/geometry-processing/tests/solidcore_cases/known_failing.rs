//! The known-failing list: one file per case, `known_failing/<fixture stem>/<case>.json`, holding
//! `reason` (what goes wrong, in solidmesh's terms) and `seen` (the case's issues as the runner
//! words them today, joined by `"; "`). A listed case must keep failing exactly that way: one
//! that passes, or fails differently, makes the run fail until its file is removed or rewritten,
//! so the list only ever shrinks by a fix someone saw. A reason starting `untriaged:` is one the
//! sync script (`scripts/sync_solidcore_cases.sh`) wrote for a new or changed case nobody has
//! looked at yet; the run allows it and counts it apart. Unported cases are held the same way in
//! `unported.json`.

use std::collections::BTreeMap;

use serde::Deserialize;

use crate::case_file::cases_folder;

/// One listed case.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct KnownFailing {
    pub reason: String,
    pub seen: String,
}

/// Every listed case by `<fixture stem>/<case>`, in name order.
pub fn read_known_failing() -> Result<BTreeMap<String, KnownFailing>, String> {
    let root = cases_folder().join("known_failing");
    let mut listed = BTreeMap::new();
    let Ok(stems) = std::fs::read_dir(&root) else {
        return Ok(listed);
    };
    for stem in stems {
        let stem = stem.map_err(|e| format!("{}: {e}", root.display()))?.path();
        let stem_name = stem
            .file_name()
            .and_then(|n| n.to_str())
            .ok_or_else(|| format!("{} has no name", stem.display()))?
            .to_string();
        for entry in std::fs::read_dir(&stem).map_err(|e| format!("{}: {e}", stem.display()))? {
            let path = entry
                .map_err(|e| format!("{}: {e}", stem.display()))?
                .path();
            let case_name = path
                .file_stem()
                .and_then(|n| n.to_str())
                .ok_or_else(|| format!("{} has no name", path.display()))?
                .to_string();
            let text =
                std::fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
            let entry: KnownFailing =
                serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
            listed.insert(format!("{stem_name}/{case_name}"), entry);
        }
    }
    Ok(listed)
}

/// The cases held as unported, by `<fixture stem>/<case>`, each with the op solidmesh lacks
/// (`unported.json`, one object). A case that becomes or stops being unported, or whose missing
/// op changes, moves the run like any other.
pub fn read_unported() -> Result<BTreeMap<String, String>, String> {
    let path = cases_folder().join("unported.json");
    let text = std::fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))
}

/// Whether an entry is still waiting for someone to say what goes wrong: the sync script writes
/// `untriaged: ...` for a case that is new or whose expectations changed and does not pass.
pub fn is_untriaged(entry: &KnownFailing) -> bool {
    entry.reason.starts_with("untriaged:")
}
