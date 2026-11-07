//! One row's worth of work in the Thingi10K sweep: the pick built and filled at one size behind the
//! watchdog, and the answer it gave read against what that size fraction promised.

use std::collections::HashSet;
use std::path::{Path, PathBuf};
use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_working_mesh::seam_epsilon_for_positions;
use geometry_processing::bevel::level_line::band_gap::BandGapBuildError;
use geometry_processing::bevel::level_line::band_gap_entry_points::{
    build_pick, BandGapBuildOptions,
};
use geometry_processing::bevel::level_line::gap_fill::{self, GapFillError};
use geometry_processing::bevel::level_line::{
    BandGapRefusal, BandGapResult, BandProfile, GapFillRefusal, LevelLineWorkPhase,
    RoomReadingKind, RoomSource, WorkBudget,
};
use vector_math::Vector3;

use crate::sweep_mesh_measures::{
    is_closed_and_consistent, kept, key, shell_count, signed_volume, smallest_area,
};
use crate::sweep_report::dump;
use crate::sweep_watchdog::{self, Thrown};

/// What a build that found a PINCH is called - the expected answer of a pinching pick.
pub const PINCH_OUTCOME: &str = "RefusedBySize:Pinch";

/// A refusal over a pinch room but inside the ladder's slack - see [`PINCH_ROOM_SLACK`].
pub const PINCH_SLACK_OUTCOME: &str = "RefusedBySize:InPinchSlack";

/// An open chain whose end does not have a crease carrying on past it: the fill refuses
/// `OpenChainEnds` by name because cutting the profile into the END FACE is a later rung of
/// stage 2. Its own class, and not a failure - this is the stage saying where it is.
pub const OPEN_ENDS_OUTCOME: &str = "NotBuiltYet:EndsOnAnEndFace";

/// How far above the size the pinch ladder names the true limit may lie. BandGap brackets the
/// limit in [low, 2*low] and bisects three times, so the bracket closes to low/8 and the limit is
/// in [low, low * 9/8]. Sizes inside that band over a pinch room are not over the limit in any
/// sense the kernel promised.
pub const PINCH_ROOM_SLACK: f64 = 9.0 / 8.0;

/// What one guarded build of one pick at one size came back with.
#[derive(Debug, Clone)]
pub struct SweepAttempt {
    pub refusal: BandGapRefusal,
    pub fill_refusal: GapFillRefusal,
    pub room: f64,
    /// What was running when a budget refusal stopped it.
    pub phase: LevelLineWorkPhase,
    pub positions: Option<Vec<Vector3>>,
    pub indices: Option<Vec<(i32, i32, i32)>>,
    /// Set only when the watchdog answered for the call: Hung, or Threw:type.
    pub outcome: Option<String>,
    /// A thrown exception's message and where in the kernel it came from.
    pub detail: Option<String>,
    /// Work units spent over every loop built so far.
    pub work: i64,
    /// The dump of the call that ended the row, still on disk.
    pub dump_path: Option<PathBuf>,
    /// Triangles in that dump, which is what decides whether it is worth keeping.
    pub dump_triangles: usize,
    /// The dumps of the loops that came back well before the row ended, still on disk. A row whose
    /// final mesh is UNSOUND was laid by these calls, so they are the inputs worth keeping.
    pub earlier_dumps: Vec<(PathBuf, usize)>,
}

impl Default for SweepAttempt {
    fn default() -> Self {
        SweepAttempt {
            refusal: BandGapRefusal::None,
            fill_refusal: GapFillRefusal::None,
            room: 0.0,
            phase: LevelLineWorkPhase::None,
            positions: None,
            indices: None,
            outcome: None,
            detail: None,
            work: 0,
            dump_path: None,
            dump_triangles: 0,
            earlier_dumps: Vec::new(),
        }
    }
}

/// The mesh and the pick, shared with the build thread.
#[derive(Clone)]
pub struct PickInput {
    pub positions: Arc<Vec<Vector3>>,
    pub indices: Arc<Vec<(i32, i32, i32)>>,
    pub loops: Arc<Vec<Vec<i32>>>,
    pub closed: Arc<Vec<bool>>,
    pub features: Arc<HashSet<(i32, i32)>>,
}

impl PickInput {
    /// The feature-edge test the kernel takes, over the undirected edge set.
    pub fn feature_test(&self) -> Rc<dyn Fn(i32, i32) -> bool> {
        let features = Arc::clone(&self.features);
        Rc::new(move |a, b| features.contains(&key(a, b)))
    }
}

/// The error-kind name a build error is reported under in the sweep's rows.
pub fn build_error(error: BandGapBuildError) -> Thrown {
    let name = match &error {
        BandGapBuildError::SizeNotPositive(_) => "ArgumentOutOfRangeException",
        BandGapBuildError::Cancelled => "OperationCanceledException",
        BandGapBuildError::Defect(_) => "InvalidOperationException",
        _ => "ArgumentException",
    };
    Thrown::new(name, error.to_string())
}

/// The error-kind name a fill error is reported under in the sweep's rows.
pub fn fill_error(error: GapFillError) -> Thrown {
    match error {
        GapFillError::Cancelled => {
            Thrown::new("OperationCanceledException", "the fill was cancelled")
        }
        GapFillError::Mesh(error) => Thrown::new("ArgumentException", format!("{error:?}")),
        GapFillError::Argument(message) => Thrown::new("ArgumentException", message),
    }
}

/// What the app route does with a face rim's several loops: hand them to the kernel as ONE PICK -
/// one `BandGap::build_pick` and one `GapFill::fill` - so every loop is cut against a field that
/// knows about all of them.
///
/// The call is guarded on its OWN thread, and its exact input - the mesh, EVERY chain of the pick,
/// the feature edges, the size and the profile - is written to disk before that thread starts. A
/// call that comes back well keeps its dump until the ROW is classified, because an unsound final
/// mesh was laid by that call and by no other.
pub fn build_the_pick(
    input: &PickInput,
    size: f64,
    profile: BandProfile,
    failure_root: &Path,
    model: i32,
    pick: i32,
    fraction: f64,
) -> std::io::Result<SweepAttempt> {
    let dump_path = dump(
        failure_root,
        model,
        pick,
        &format!("{fraction:.1}"),
        &input.positions,
        &input.indices,
        &input.loops,
        &input.closed,
        &input.features,
        size,
        profile,
        None,
    )?;
    let triangles = input.indices.len();

    let thread_input = input.clone();
    let guarded = sweep_watchdog::run(
        geometry_processing::bevel::level_line::WorkBudget::default_limit(triangles as i32),
        move |budget| the_pick(&thread_input, size, profile, budget),
    );

    if let Some(outcome) = guarded.outcome {
        return Ok(SweepAttempt {
            outcome: Some(outcome),
            detail: guarded.detail,
            work: guarded.work,
            dump_path: Some(dump_path),
            dump_triangles: triangles,
            ..SweepAttempt::default()
        });
    }

    let step = guarded
        .value
        .expect("a guarded call that did not end has a value");
    if step.positions.is_none() {
        return Ok(SweepAttempt {
            refusal: step.refusal,
            fill_refusal: step.fill_refusal,
            room: step.room,
            phase: step.phase,
            detail: step.detail,
            work: guarded.work,
            dump_path: Some(dump_path),
            dump_triangles: triangles,
            ..SweepAttempt::default()
        });
    }

    Ok(SweepAttempt {
        positions: step.positions,
        indices: step.indices,
        work: guarded.work,
        earlier_dumps: vec![(dump_path, triangles)],
        ..SweepAttempt::default()
    })
}

/// The pick's build and fill, in one call each, on the mesh as it went in.
fn the_pick(
    input: &PickInput,
    size: f64,
    profile: BandProfile,
    budget: Rc<WorkBudget>,
) -> Result<SweepAttempt, Thrown> {
    // The app's own seam epsilon, so a sweep scores the construction the user gets and not one
    // that still reads every micron-wide ribbon as a wall.
    let options = BandGapBuildOptions {
        budget: Some(Rc::clone(&budget)),
        seam_epsilon: seam_epsilon_for_positions(&input.positions),
        ..BandGapBuildOptions::default()
    };
    let gap = build_pick(
        &input.positions,
        &input.indices,
        &input.loops,
        &input.closed,
        Some(input.feature_test()),
        size,
        profile,
        &options,
    )
    .map_err(build_error)?;
    if !gap.built() {
        return Ok(SweepAttempt {
            refusal: gap.refusal,
            room: gap.room,
            phase: gap.phase,
            detail: said_gap(&gap),
            ..SweepAttempt::default()
        });
    }

    let filled = gap_fill::fill_with_budget(&gap, profile, 6, Some(budget)).map_err(fill_error)?;
    if !filled.filled() {
        return Ok(SweepAttempt {
            fill_refusal: filled.refusal,
            room: gap.room,
            phase: filled.phase,
            ..SweepAttempt::default()
        });
    }

    let mesh = filled
        .mesh
        .as_ref()
        .expect("a fill that went in carries its mesh")
        .borrow();
    Ok(SweepAttempt {
        positions: Some(mesh.vertices.clone()),
        indices: Some(kept(&mesh)),
        ..SweepAttempt::default()
    })
}

/// Which chain of the pick a refusal is about, where it is about one.
fn said_gap(gap: &BandGapResult) -> Option<String> {
    match gap.refusal {
        BandGapRefusal::ChainsTouch => Some(format!(
            "chains {} and {} at vertex {}",
            gap.offending_chain, gap.other_chain, gap.offending_vertex
        )),
        BandGapRefusal::ChainTouchesItself => Some(format!(
            "chain {} at vertex {}",
            gap.offending_chain, gap.offending_vertex
        )),
        BandGapRefusal::ChainDoesNotDivideItsSides => {
            Some(format!("chain {}", gap.offending_chain))
        }
        BandGapRefusal::ChainEdgeNotOnTwoTriangles => Some(format!(
            "edge {}-{}",
            gap.offending_edge.0, gap.offending_edge.1
        )),
        _ => None,
    }
}

/// Which reading of the pick's room named it, short enough to stand on every row.
pub fn said(source: &RoomSource) -> String {
    match source.kind {
        RoomReadingKind::Foreign => format!("Foreign({})", source.chain),
        RoomReadingKind::SelfRoom => format!("Self({})", source.chain),
        RoomReadingKind::ChainToChain => {
            format!("ChainToChain({},{})", source.chain, source.other_chain)
        }
        RoomReadingKind::None => "Unbounded".to_string(),
    }
}

/// One build against what its size fraction promised: everything up to the room must build a
/// closed, consistently oriented, single-shell solid whose volume CHANGED and which is still wound
/// the way its input was, and a hair over the room must refuse by size naming that same room.
pub fn classify(
    attempt: &SweepAttempt,
    fraction: f64,
    room: f64,
    unbounded: bool,
    pinch_room: bool,
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
) -> String {
    // A room a WIDTH reading names is exact, so a hair over it must refuse naming it. A room the
    // PINCH ladder names is a LOWER BOUND by construction: the ladder brackets the true limit in
    // [low, high] with high exactly twice low and then bisects three times, so all it promises is
    // that the limit lies in [low, low * PINCH_ROOM_SLACK]. The sweep's 1.1 sits INSIDE that band,
    // so over a pinch room 1.1 is not "a hair over" anything - the build may legitimately stand
    // up, or refuse naming a size larger than the one it was asked under. Run 8 counted 30 such
    // rows as WrongRefusal; they were the sweep's expectation being wrong, not the kernel's answer.
    let must_refuse = fraction > (if pinch_room { PINCH_ROOM_SLACK } else { 1.0 }) && !unbounded;

    // The two refusals that are about the kernel running away rather than about the size are
    // their own classes at either end, build or fill: lumping them in with a wrong refusal hides
    // how many there are.
    if attempt.refusal == BandGapRefusal::WorkBudgetExceeded
        || attempt.fill_refusal == GapFillRefusal::WorkBudgetExceeded
    {
        return format!("WorkBudgetExceeded:{:?}", attempt.phase);
    }

    if attempt.refusal == BandGapRefusal::RefinementWouldNotSettle {
        return "RefinementWouldNotSettle".to_string();
    }

    // Its own class for the same reason the reading gives it one: it is a statement about the mesh
    // under the pick, not a wrong answer about size.
    if attempt.refusal == BandGapRefusal::ChainEdgeNotOnTwoTriangles {
        return "ChainEdgeNotOnTwoTriangles".to_string();
    }

    // What a pick of SEVERAL loops can say that a pick of one never could: two of its chains
    // touching, or a chain that does not divide its sides. Each is a statement about the PICK, so
    // each is its own class rather than a wrong answer about size.
    if attempt.refusal == BandGapRefusal::ChainsTouch {
        return "ChainsTouch".to_string();
    }

    if attempt.refusal == BandGapRefusal::ChainTouchesItself {
        return "ChainTouchesItself".to_string();
    }

    if attempt.refusal == BandGapRefusal::ChainDoesNotDivideItsSides {
        return "ChainDoesNotDivideItsSides".to_string();
    }

    if attempt.refusal == BandGapRefusal::Size {
        if !must_refuse {
            // A refusal UNDER the room naming a SMALLER room is the build reading a pinch - two
            // stretches of one chain closing across a neck, which no width reading looks at. That
            // is what a pinching pick is supposed to do, and the caller re-derives this pick's
            // sizes from the room named. A refusal naming the room it was asked under is still
            // wrong. Over a pinch room but inside the ladder's own slack: naming a size at or
            // above the room asked is the ladder having refined its own lower bound upward, which
            // is what a lower bound is for.
            if pinch_room && fraction > 1.0 && attempt.room >= room {
                return PINCH_SLACK_OUTCOME.to_string();
            }

            return if attempt.room > 0.0 && attempt.room < room {
                PINCH_OUTCOME.to_string()
            } else {
                "WrongRefusal:SizeUnderTheRoom".to_string()
            };
        }

        // A WIDTH room cannot land here any more: the build no longer reads a room of its own off
        // the refined mesh at the size asked, it reads the very number this caller read (see
        // `band_room::room_of_cut_mesh`). The check stays as a GUARD - a second reading creeping
        // back in is exactly what it would catch - and what can still reach it is a room named by a
        // PINCH ladder, which is not a width at all.
        let named = attempt.room;
        return if (named - room).abs() <= 1e-9 * room.abs().max(1.0) {
            "RefusedBySize".to_string()
        } else {
            "WrongRefusal:SizeNamesAnotherRoom".to_string()
        };
    }

    if attempt.refusal != BandGapRefusal::None {
        return format!("WrongRefusal:{:?}", attempt.refusal);
    }

    if attempt.fill_refusal == GapFillRefusal::OpenChainEnds {
        return OPEN_ENDS_OUTCOME.to_string();
    }

    if attempt.fill_refusal != GapFillRefusal::None {
        return format!("WrongRefusal:Fill{:?}", attempt.fill_refusal);
    }

    if must_refuse {
        return "WrongRefusal:NoneOverTheRoom".to_string();
    }

    let (Some(built_positions), Some(built_indices)) = (&attempt.positions, &attempt.indices)
    else {
        return "WrongRefusal:NoMeshAndNoRefusal".to_string();
    };

    if !is_closed_and_consistent(built_positions, built_indices) {
        return "Unsound:NotClosed".to_string();
    }

    if shell_count(built_positions.len() as i32, built_indices) != 1 {
        return "Unsound:ManyShells".to_string();
    }

    if smallest_area(built_positions, built_indices).partial_cmp(&0.0)
        != Some(std::cmp::Ordering::Greater)
    {
        return "Unsound:DegenerateTriangle".to_string();
    }

    let before = signed_volume(positions, indices);
    let after = signed_volume(built_positions, built_indices);

    // The requirement is that the result is wound the same way round as the input it came from,
    // NOT that its signed volume is positive: an inward-wound input (95430 reads -111811, closed
    // and consistent, so it passes the input gate) would make every one of its builds "not
    // positive", which was all twelve of the third sweep's rows. A fillet on a CONCAVE edge
    // legitimately ADDS material too, so neither the sign of the change nor a demand that volume
    // be lost belongs here - only that it changed, and that the solid did not turn itself inside
    // out.
    if sign(after) != sign(before) {
        return "Unsound:VolumeInverted".to_string();
    }

    if (after - before).abs() <= 1e-12 * before.abs().max(1.0) {
        return "Unsound:VolumeUnchanged".to_string();
    }

    "BuiltClosed".to_string()
}

/// The sign of a double: -1, 0 or 1 (0 for a NaN, which a sound mesh never reads).
fn sign(value: f64) -> i32 {
    if value > 0.0 {
        1
    } else if value < 0.0 {
        -1
    } else {
        0
    }
}
