//! The Thingi10K sweep's rows on their way to disk and the totals read back off them, and the
//! failing rows' input dumps. Rows go out as they are made, so an early stop keeps everything
//! already learned.

use std::collections::HashSet;
use std::fmt::Write as _;
use std::io::Write;
use std::path::{Path, PathBuf};

use geometry_processing::bevel::level_line::BandProfile;
use indexmap::IndexMap;
use vector_math::Vector3;

use crate::sweep_pick::{PARTIAL_KIND, WHOLE_OPEN_KIND};
use crate::sweep_row::SweepRow;

/// One row to the list and one line to disk, together, so the two can never drift apart.
pub fn write(
    log: &mut impl Write,
    row: SweepRow,
    rows: &mut Vec<SweepRow>,
) -> std::io::Result<i32> {
    writeln!(log, "{row}")?;
    rows.push(row);
    Ok(1)
}

/// Groups in first-appearance order, so the totals list in the order the rows met them.
fn group_by<'a, K: std::hash::Hash + Eq>(
    rows: impl IntoIterator<Item = &'a SweepRow>,
    key: impl Fn(&SweepRow) -> K,
) -> IndexMap<K, Vec<&'a SweepRow>> {
    let mut groups: IndexMap<K, Vec<&SweepRow>> = IndexMap::new();
    for row in rows {
        groups.entry(key(row)).or_default().push(row);
    }

    groups
}

pub fn summary(rows: &[SweepRow], open_chains: i32, ended_early: &str) -> String {
    let mut said = String::new();
    let _ = writeln!(
        said,
        "TOTALS: {} rows, {} open-chain picks",
        rows.len(),
        open_chains
    );
    if !ended_early.is_empty() {
        let _ = writeln!(said, "{ended_early}");
    }

    said.push_str("by class:\n");
    let mut classes: Vec<_> = group_by(rows, |r| r.outcome.clone()).into_iter().collect();
    classes.sort_by_key(|class| std::cmp::Reverse(class.1.len()));
    for (class, group) in &classes {
        let _ = writeln!(said, "  {}: {}", class, group.len());
    }

    said.push_str("by pick kind:\n");
    let mut kinds: Vec<_> = group_by(rows, |r| r.kind.clone()).into_iter().collect();
    kinds.sort_by(|a, b| a.0.cmp(&b.0));
    for (kind, group) in &kinds {
        let good = group.iter().filter(|r| r.as_expected()).count();
        let _ = writeln!(said, "  {}: {} of {} as expected", kind, good, group.len());
    }

    said.push_str("by size fraction:\n");
    let mut fractions: Vec<f64> = Vec::new();
    for row in rows.iter().filter(|r| r.fraction > 0.0) {
        if !fractions.contains(&row.fraction) {
            fractions.push(row.fraction);
        }
    }

    fractions.sort_by(|a, b| a.total_cmp(b));
    for fraction in fractions {
        let group: Vec<_> = rows.iter().filter(|r| r.fraction == fraction).collect();
        let good = group.iter().filter(|r| r.as_expected()).count();
        let _ = writeln!(
            said,
            "  x{:.1}: {} of {} as expected",
            fraction,
            good,
            group.len()
        );
    }

    // The open picks get a table of their own, kind by class, because their two kinds say
    // different things: a partial pick is the FADE's case and is expected to build, a whole open
    // chain ends on an end face and is expected to say so by name until that rung lands.
    let open: Vec<&SweepRow> = rows
        .iter()
        .filter(|r| r.kind == PARTIAL_KIND || r.kind == WHOLE_OPEN_KIND)
        .collect();
    if !open.is_empty() {
        said.push_str("open picks, kind x class:\n");
        let mut cells: Vec<_> = group_by(open.iter().copied(), |r| {
            (r.kind.clone(), r.outcome.clone())
        })
        .into_iter()
        .collect();
        cells.sort_by(|a, b| a.0 .0.cmp(&b.0 .0).then(b.1.len().cmp(&a.1.len())));
        for ((kind, outcome), group) in &cells {
            let _ = writeln!(said, "  {} {}: {}", kind, outcome, group.len());
        }

        let mut by_kind: Vec<_> = group_by(open.iter().copied(), |r| r.kind.clone())
            .into_iter()
            .collect();
        by_kind.sort_by(|a, b| a.0.cmp(&b.0));
        for (kind, group) in &by_kind {
            let not_finite: Vec<_> = group.iter().filter(|r| r.room_not_finite).collect();
            let picks_not_finite: HashSet<_> =
                not_finite.iter().map(|r| (r.model, r.pick)).collect();
            let picks: HashSet<_> = group.iter().map(|r| (r.model, r.pick)).collect();
            let _ = writeln!(
                said,
                "  {}: {} of {} rows on the fallback ladder ({} picks of {} whose room was not finite)",
                kind,
                not_finite.len(),
                group.len(),
                picks_not_finite.len(),
                picks.len()
            );
        }
    }

    let built: Vec<&SweepRow> = rows.iter().filter(|r| r.outcome == "BuiltClosed").collect();
    if !built.is_empty() {
        let seconds: Vec<f64> = built.iter().map(|r| r.seconds).collect();
        let work: Vec<f64> = built.iter().map(|r| r.work as f64).collect();
        let _ = writeln!(
            said,
            "built rows: seconds median {:.3}, p95 {:.3}; work median {:.0}, p95 {:.0}",
            percentile(&seconds, 0.5),
            percentile(&seconds, 0.95),
            percentile(&work, 0.5),
            percentile(&work, 0.95)
        );
    }

    said.push_str("slowest 10:\n");
    let mut slowest: Vec<&SweepRow> = rows.iter().collect();
    slowest.sort_by(|a, b| b.seconds.total_cmp(&a.seconds));
    for row in slowest.into_iter().take(10) {
        let _ = writeln!(said, "  {row}");
    }

    said
}

/// The nearest-rank percentile of a set of measurements; the median at 0.5.
fn percentile(of: &[f64], fraction: f64) -> f64 {
    let mut sorted = of.to_vec();
    sorted.sort_by(|a, b| a.total_cmp(b));
    let rank = (fraction * sorted.len() as f64).ceil() as i64 - 1;
    sorted[rank.clamp(0, sorted.len() as i64 - 1) as usize]
}

// ---- what the sweep leaves behind for a row that did not do what it promised --------------------

/// Triangles a kept dump may have.
pub const KEPT_TRIANGLE_LIMIT: usize = 5000;

/// A hang is worth more, being the one class the report cannot carry.
pub const HUNG_TRIANGLE_LIMIT: usize = 20000;

/// Keeps a dump if it is small enough to be a fixture, and removes it otherwise.
pub fn keep(path: Option<&Path>, triangles: usize, limit: usize) {
    if let Some(path) = path {
        if triangles > limit {
            let _ = std::fs::remove_file(path);
        }
    }
}

/// A failing row as a kernel fixture in `LevelLineFixture`'s own format, written OUTSIDE the
/// repository. Carrying one into the tests' fixtures is a deliberate act: it makes the defect a
/// red requirement test, and that is where it gets fixed. The feature edges are written sorted:
/// they are held in a hash set, whose order would change run to run.
#[allow(clippy::too_many_arguments)]
pub fn dump(
    failure_root: &Path,
    model: i32,
    pick: i32,
    at: &str,
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    features: &HashSet<(i32, i32)>,
    size: f64,
    profile: BandProfile,
    known_room: Option<f64>,
) -> std::io::Result<PathBuf> {
    std::fs::create_dir_all(failure_root)?;
    let path = failure_root.join(format!("{model}-{pick}-{at}.txt"));

    let mut writer = std::io::BufWriter::new(std::fs::File::create(&path)?);
    writeln!(
        writer,
        "# thingi {} pick {} at {}, {} chains",
        model,
        pick,
        at,
        chains.len()
    )?;
    writeln!(writer, "size {size}")?;
    if let Some(room) = known_room {
        writeln!(writer, "room {room}")?;
    }

    writeln!(
        writer,
        "profile {}",
        if profile == BandProfile::Chamfer {
            "chamfer"
        } else {
            "fillet"
        }
    )?;
    writeln!(writer, "segments 6")?;
    for p in positions {
        writeln!(writer, "v {} {} {}", p.x, p.y, p.z)?;
    }

    for t in indices {
        writeln!(writer, "f {} {} {}", t.0, t.1, t.2)?;
    }

    // One line per chain, in the order the pick hands them to the kernel, and the line's own WORD
    // says whether it is closed: a dumped open row re-runs as open, and a file written before open
    // picks existed still reads as the closed pick it was.
    for (i, chain) in chains.iter().enumerate() {
        let word = if i < closed.len() && !closed[i] {
            "openchain"
        } else {
            "chain"
        };
        let vertices: Vec<String> = chain.iter().map(|v| v.to_string()).collect();
        writeln!(writer, "{} {}", word, vertices.join(" "))?;
    }

    let mut edges: Vec<_> = features.iter().collect();
    edges.sort();
    for (low, high) in edges {
        writeln!(writer, "e {low} {high}")?;
    }

    writer.flush()?;
    Ok(path)
}
