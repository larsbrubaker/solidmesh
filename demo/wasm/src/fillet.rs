//! The Fillet & Chamfer page's build: a part's picks, each with its own radius, applied with the
//! library's one entry point, `bevel::part::bevel_part`, and every pick's row worded for the list.
//!
//! `bevel_part` takes one size for every group of a request, so picks of different sizes are
//! applied as successive calls, one per distinct size in the order the sizes first appear in the
//! list, each call on the previous call's mesh - the way a second radius is made in MatterCAD, by
//! a second fillet on the result of the first. A pick is stored as a `BevelEdgeGroup`, a
//! geometric signature (the anchor edge's midpoint, direction and face planes, or a corner's
//! point) rather than mesh indices, which is what lets a later call find it again on a mesh the
//! earlier calls rebuilt. Before a later call uses a pick it is checked on that mesh
//! ([`still_found`]): an edge pick must resolve there, and every edge it resolves to must lie near
//! the edges it picked on the original part (an earlier round can join a picked edge into a
//! longer smooth run, which would bevel more than was picked); a corner pick must still be a
//! corner exactly where it was. A pick that fails is refused with the demo's own note rather than
//! guessed at.

use std::sync::Arc;

use geometry_processing::bevel::part::{
    bevel_part, the_shown_size, BevelPartError, BevelPartOutcome, BevelPartRequest,
};
use geometry_processing::bevel::BevelGroupResult;
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelEdgeGroup, BevelSelectionKind,
};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::fillet_pick::BEND_THRESHOLD_DEGREES;
use crate::fillet_words::{format_advisory, join_advisories, with_the_size_that_fits};
use crate::Scene;

/// One pick of the list: what it selects and the radius (or chamfer setback) it asks for, in mm.
#[derive(Debug, Clone)]
pub struct FilletPick {
    pub group: BevelEdgeGroup,
    pub size: f64,
}

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
    /// Which call (0-based) built or refused it.
    pub pass: usize,
    /// The kernel's code for the refusal, for a test or a log; `None` when built (or refused by
    /// the demo's own check).
    pub code: Option<String>,
}

/// A whole build: the mesh, a row per pick, and how many calls it took.
pub struct FilletBuild {
    pub scene: Scene,
    pub rows: Vec<PickRow>,
    pub passes: usize,
}

/// The note a pick refused by [`still_found`] carries.
pub const NOT_FOUND_AGAIN: &str = "This pick could not be found again unchanged after the picks \
with an earlier radius were rounded (it may have been cut away, or joined into a longer smooth \
run), so it was not built. Give it the same radius as the picks it touches, or remove it.";

/// Every pick of `picks` applied to `source`: rounds when `round`, chamfers otherwise, `segments`
/// to a round. A cancel through `cancel` answers `Err`.
pub fn fillet_part(
    source: &Arc<Mesh>,
    picks: &[FilletPick],
    round: bool,
    segments: i32,
    cancel: &CancelToken,
) -> Result<FilletBuild, String> {
    let mut sizes: Vec<f64> = Vec::new();
    for pick in picks {
        if !sizes.contains(&pick.size) {
            sizes.push(pick.size);
        }
    }

    let mut rows: Vec<Option<PickRow>> = vec![None; picks.len()];
    let mut mesh = Arc::clone(source);
    let mut notes: Vec<String> = Vec::new();
    let mut earlier_sizes = 0.0;
    let mut originals: Vec<Option<Vec<[Vector3; 2]>>> = vec![None; picks.len()];

    for (pass, &size) in sizes.iter().enumerate() {
        let mut members = Vec::new();
        for (i, pick) in picks.iter().enumerate() {
            if pick.size != size {
                continue;
            }
            if pass > 0 {
                let picked = originals[i]
                    .get_or_insert_with(|| picked_segments(&pick.group, source))
                    .clone();
                if !still_found(&pick.group, &picked, &mesh, earlier_sizes) {
                    rows[i] = Some(PickRow {
                        built: false,
                        edge_count: 0,
                        text: NOT_FOUND_AGAIN.to_string(),
                        fits: None,
                        pass,
                        code: None,
                    });
                    continue;
                }
            }
            members.push(i);
        }
        if members.is_empty() {
            earlier_sizes += size;
            continue;
        }

        let groups: Vec<BevelEdgeGroup> = members.iter().map(|&i| picks[i].group.clone()).collect();
        let request = BevelPartRequest::new(
            Some(Arc::clone(&mesh)),
            groups.clone(),
            size,
            segments,
            round,
            BEND_THRESHOLD_DEGREES,
        );
        let outcome = match bevel_part(request, cancel, None) {
            Ok(outcome) => outcome,
            Err(BevelPartError::Cancelled) => return Err("The build was cancelled.".to_string()),
            Err(_) => BevelPartOutcome::every_group_refused(&groups),
        };

        for (slot, &i) in members.iter().enumerate() {
            rows[i] = Some(row_of(
                outcome.group_results.get(slot).cloned().flatten(),
                pass,
            ));
        }
        if let Some(failures) = outcome.failures.as_deref().and_then(join_advisories) {
            notes.push(failures);
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
            notes.push(advisories);
        }
        if let Some(built) = outcome.mesh {
            mesh = Arc::new(built);
        }
        earlier_sizes += size;
    }

    let rows: Vec<PickRow> = rows
        .into_iter()
        .map(|row| {
            row.unwrap_or(PickRow {
                built: false,
                edge_count: 0,
                text: "Nothing was said about this pick.".to_string(),
                fits: None,
                pass: 0,
                code: None,
            })
        })
        .collect();
    let built = rows.iter().filter(|r| r.built).count();
    let mut note = if picks.is_empty() {
        "Pick an edge, a face or a corner to round it.".to_string()
    } else {
        format!(
            "{built} of {} picks built, in {} bevel_part call{}.",
            picks.len(),
            sizes.len(),
            if sizes.len() == 1 { "" } else { "s" }
        )
    };
    for extra in notes {
        note.push(' ');
        note.push_str(&extra);
    }
    let mesh = Arc::try_unwrap(mesh).unwrap_or_else(|shared| shared.copy());
    Ok(FilletBuild {
        scene: Scene::new(mesh, note),
        rows,
        passes: sizes.len(),
    })
}

/// A kernel row as the list shows it.
fn row_of(result: Option<BevelGroupResult>, pass: usize) -> PickRow {
    let Some(result) = result else {
        return PickRow {
            built: false,
            edge_count: 0,
            text: "The build said nothing about this pick.".to_string(),
            fits: None,
            pass,
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
                pass,
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
                pass,
                code: None,
            }
        }
    }
}

/// The edges `group` selects on the part it was picked on, as segments (none for a corner).
pub fn picked_segments(group: &BevelEdgeGroup, mesh: &Mesh) -> Vec<[Vector3; 2]> {
    if group.selection_kind == BevelSelectionKind::Point {
        return Vec::new();
    }
    let resolution =
        bevel_edge_group_resolver::resolve(&group.read_at(BEND_THRESHOLD_DEGREES), mesh);
    resolution.edges.iter().map(|e| [e.start, e.end]).collect()
}

/// Whether `group`, picked on the original part where it selected `picked`, still names the same
/// edges on `mesh`, which earlier rounds of `earlier_sizes` mm in all have rebuilt: every edge it
/// resolves to lies within reach of a picked one (an earlier round moves an edge's ends by about
/// its size). A corner must still be a vertex of `mesh` where it was.
pub fn still_found(
    group: &BevelEdgeGroup,
    picked: &[[Vector3; 2]],
    mesh: &Mesh,
    earlier_sizes: f64,
) -> bool {
    if group.selection_kind == BevelSelectionKind::Point {
        let tolerance = 1e-6 * mesh.get_axis_aligned_bounding_box().size().length();
        return mesh.vertices.iter().any(|v| {
            (Vector3::new(f64::from(v.x), f64::from(v.y), f64::from(v.z)) - group.anchor_midpoint)
                .length()
                <= tolerance.max(1e-5)
        });
    }
    if picked.is_empty() {
        return false;
    }
    let resolution =
        bevel_edge_group_resolver::resolve(&group.read_at(BEND_THRESHOLD_DEGREES), mesh);
    if !resolution.is_resolved() || resolution.edges.is_empty() {
        return false;
    }
    let reach = 2.0 * earlier_sizes + 1e-4;
    resolution.edges.iter().all(|edge| {
        [edge.start, edge.end]
            .iter()
            .all(|&p| picked.iter().any(|s| distance_to_segment(p, s) <= reach))
    })
}

fn distance_to_segment(p: Vector3, segment: &[Vector3; 2]) -> f64 {
    let [a, b] = *segment;
    let ab = b - a;
    let length_squared = ab.dot(ab);
    let t = if length_squared > 0.0 {
        ((p - a).dot(ab) / length_squared).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (p - (a + ab * t)).length()
}
