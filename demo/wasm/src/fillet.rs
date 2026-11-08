//! The Fillet & Chamfer page's build: a part's picks, all at one radius (or chamfer setback),
//! applied with one call to the library's entry point, `bevel::part::bevel_part`, with its default
//! routes (the constructions MatterCAD builds by), and every pick's row worded for the list.
//!
//! One size for every pick is how `bevel_part` takes a request and how MatterCAD's Fillet works
//! (one size per operation): every pick is rounded at once, on the part as it was picked, so no
//! pick depends on its place in the list or on what another pick's round left behind. A pick is
//! a `BevelEdgeGroup`, a geometric signature (the anchor edge's midpoint, direction and face
//! planes, or a corner's point) rather than mesh indices, which the kernel resolves on the part.

use std::sync::Arc;

use geometry_processing::bevel::part::{
    bevel_part, the_shown_size, BevelPartError, BevelPartOutcome, BevelPartRequest,
};
use geometry_processing::bevel::BevelGroupResult;
use geometry_processing::topology::BevelEdgeGroup;
use polygon_mesh::{CancelToken, Mesh};

use crate::fillet_pick::BEND_THRESHOLD_DEGREES;
use crate::fillet_words::{format_advisory, join_advisories, with_the_size_that_fits};
use crate::Scene;

/// What became of one pick, in list order.
#[derive(Debug, Clone, PartialEq)]
pub struct PickRow {
    /// Whether it was built.
    pub built: bool,
    /// How many mesh edges it resolved to (0 for a corner or a pick never found).
    pub edge_count: i32,
    /// Why it was not built, or what to know about how it was; empty when there is nothing to say.
    pub text: String,
    /// The largest size that would have built it, where a refused row has one.
    pub fits: Option<f64>,
    /// The kernel's code for the refusal, for a test or a log; `None` when built.
    pub code: Option<String>,
}

/// A whole build: the mesh and a row per pick.
pub struct FilletBuild {
    pub scene: Scene,
    pub rows: Vec<PickRow>,
}

/// Every pick of `picks` applied to `source` at `size` mm in one `bevel_part` call: rounds when
/// `round`, chamfers otherwise, `segments` to a round. A cancel through `cancel` answers `Err`.
pub fn fillet_part(
    source: &Arc<Mesh>,
    picks: &[BevelEdgeGroup],
    size: f64,
    round: bool,
    segments: i32,
    cancel: &CancelToken,
) -> Result<FilletBuild, String> {
    if picks.is_empty() {
        return Ok(FilletBuild {
            scene: Scene::new(
                source.copy(),
                "Pick an edge, a face or a corner to round it.".to_string(),
            ),
            rows: Vec::new(),
        });
    }
    let request = BevelPartRequest::new(
        Some(Arc::clone(source)),
        picks.to_vec(),
        size,
        segments,
        round,
        BEND_THRESHOLD_DEGREES,
    );
    let outcome = match bevel_part(request, cancel, None) {
        Ok(outcome) => outcome,
        Err(BevelPartError::Cancelled) => return Err("The build was cancelled.".to_string()),
        Err(_) => BevelPartOutcome::every_group_refused(picks),
    };

    let rows: Vec<PickRow> = (0..picks.len())
        .map(|i| row_of(outcome.group_results.get(i).cloned().flatten()))
        .collect();
    let built = rows.iter().filter(|r| r.built).count();
    let mut note = format!(
        "{built} of {} picks built, in one bevel_part call.",
        picks.len()
    );
    if let Some(failures) = outcome.failures.as_deref().and_then(join_advisories) {
        note.push(' ');
        note.push_str(&failures);
    }
    // how the kernel conditioned the source before building is not the user's to act on
    let said: Vec<_> = outcome
        .advisories
        .iter()
        .flatten()
        .filter(|a| !a.code().to_string().starts_with("SourceMesh"))
        .cloned()
        .collect();
    if let Some(advisories) = join_advisories(&said) {
        note.push(' ');
        note.push_str(&advisories);
    }
    let mesh = outcome.mesh.unwrap_or_else(|| source.copy());
    Ok(FilletBuild {
        scene: Scene::new(mesh, note),
        rows,
    })
}

/// A kernel row as the list shows it.
fn row_of(result: Option<BevelGroupResult>) -> PickRow {
    let Some(result) = result else {
        return PickRow {
            built: false,
            edge_count: 0,
            text: "The build said nothing about this pick.".to_string(),
            fits: None,
            code: None,
        };
    };
    let fits = result
        .largest_size_that_fits
        .filter(|size| *size > 0.0)
        .map(the_shown_size);
    match &result.failure {
        Some(failure) => {
            // the size refusals already quote the room; say it only for the others
            let quotes_room = format_advisory(failure).contains("most that fits");
            let words = format_advisory(failure);
            PickRow {
                built: false,
                edge_count: result.edge_count,
                text: if quotes_room {
                    words
                } else {
                    with_the_size_that_fits(&words, fits)
                },
                fits,
                code: Some(failure.code().to_string()),
            }
        }
        None => {
            let mut text = join_advisories(&result.warnings).unwrap_or_default();
            if let Some(loops) = result.loop_failures.as_ref().filter(|l| !l.is_empty()) {
                let first = format_advisory(&loops[0].advisory);
                let head = format!(
                    "{} of its {} loops could not be built: {first}",
                    loops.len(),
                    result.loop_count.max(loops.len() as i32)
                );
                text = if text.is_empty() {
                    head
                } else {
                    format!("{text} {head}")
                };
            }
            PickRow {
                built: true,
                edge_count: result.edge_count,
                text,
                fits: None,
                code: None,
            }
        }
    }
}
