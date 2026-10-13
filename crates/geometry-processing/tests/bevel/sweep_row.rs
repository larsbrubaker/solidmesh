//! One line of the Thingi10K sweep's report: what was asked of the kernel, and what it answered.
//! Ports `Tests/GeometryProcessingTests/LevelLine/SweepRow.cs`.

use crate::sweep_build;

/// C# `SweepRow`. The defaults are C#'s initializers (`Pick` -1, `Kind` "-").
#[derive(Debug, Clone)]
pub struct SweepRow {
    pub model: i32,
    pub pick: i32,
    pub kind: String,
    pub fraction: f64,
    pub size: f64,
    pub outcome: String,
    pub seconds: f64,
    /// Work units the kernel spent on the row, beside the seconds it took.
    pub work: i64,
    /// What a throw said, or which phase ran out of work; empty otherwise.
    pub detail: Option<String>,
    /// Which reading of the pick's room named it - see `RoomSource`.
    pub set_by: Option<String>,
    /// True when the pick's room could not be read as a finite number and the sizes came off the
    /// fallback ladder instead - so the row's size is not a fraction of any room, and nothing
    /// here is over one.
    pub room_not_finite: bool,
}

impl SweepRow {
    /// True when the row did what its size fraction promised - only these are not dumped. A chain
    /// whose ends sit on an end face is counted here too: not building it yet is this stage's own
    /// statement, not a failure of the row.
    pub fn as_expected(&self) -> bool {
        self.outcome == "BuiltClosed"
            || self.outcome == "RefusedBySize"
            || self.outcome == sweep_build::PINCH_OUTCOME
            || self.outcome == sweep_build::PINCH_SLACK_OUTCOME
            || self.outcome == sweep_build::OPEN_ENDS_OUTCOME
    }

    /// A row that counts but is not a build: a model or a pick that never got that far.
    pub fn aside(
        model: i32,
        outcome: &str,
        pick: i32,
        kind: &str,
        detail: Option<String>,
        work: i64,
    ) -> SweepRow {
        SweepRow {
            model,
            pick,
            kind: kind.to_string(),
            fraction: 0.0,
            size: 0.0,
            outcome: outcome.to_string(),
            seconds: 0.0,
            work,
            detail,
            set_by: None,
            room_not_finite: false,
        }
    }

    /// An aside about a whole model: no pick, no kind.
    pub fn model_aside(model: i32, outcome: &str) -> SweepRow {
        SweepRow::aside(model, outcome, -1, "-", None, 0)
    }

    #[allow(clippy::too_many_arguments)]
    pub fn of(
        model: i32,
        pick: i32,
        kind: &str,
        fraction: f64,
        size: f64,
        outcome: &str,
        seconds: f64,
        work: i64,
        detail: Option<String>,
        set_by: Option<String>,
        room_not_finite: bool,
    ) -> SweepRow {
        SweepRow {
            model,
            pick,
            kind: kind.to_string(),
            fraction,
            size,
            outcome: outcome.to_string(),
            seconds,
            work,
            detail,
            set_by,
            room_not_finite,
        }
    }
}

/// C#'s custom format `0.######`: up to six decimals, trailing zeros dropped.
fn hashes(value: f64) -> String {
    if !value.is_finite() {
        return format!("{value}");
    }

    let text = format!("{value:.6}");
    let text = text.trim_end_matches('0').trim_end_matches('.');
    if text == "-0" || text.is_empty() {
        "0".to_string()
    } else {
        text.to_string()
    }
}

impl std::fmt::Display for SweepRow {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut said = format!(
            "{} pick {} {} x{:.1} size {} {} {:.3}s {} work",
            self.model,
            self.pick,
            self.kind,
            self.fraction,
            hashes(self.size),
            self.outcome,
            self.seconds,
            self.work
        );
        if let Some(set_by) = self.set_by.as_deref().filter(|s| !s.is_empty()) {
            said.push_str(" room by ");
            said.push_str(set_by);
        }

        if self.room_not_finite {
            said.push_str(" room not finite, size off the fallback ladder");
        }

        match self.detail.as_deref().filter(|d| !d.is_empty()) {
            None => f.write_str(&said),
            Some(detail) => write!(f, "{said} | {detail}"),
        }
    }
}
