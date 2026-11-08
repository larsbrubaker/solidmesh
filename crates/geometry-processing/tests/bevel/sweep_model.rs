//! One model of the Thingi10K sweep: its picks, each read for its room and then built at the five
//! size fractions.

use std::collections::HashSet;
use std::io::Write;
use std::path::Path;
use std::sync::Arc;
use std::time::Instant;

use solidmesh::bevel::bevel_working_mesh::seam_epsilon_for_positions;
use solidmesh::bevel::level_line::band_gap_reading::{read_room_of_pick, ReadRoomError};
use solidmesh::bevel::level_line::{BandProfile, RoomReading, WorkBudget};
use solidmesh_math::seeded_random::SeededRandom;

use crate::stl_reader;
use crate::sweep_build::{self, PickInput};
use crate::sweep_mesh_measures::{
    diagonal, feature_edges, is_closed_and_consistent, key, pick_length, smallest_area,
};
use crate::sweep_pick::{self, SweepPick};
use crate::sweep_report::{self, dump, keep, HUNG_TRIANGLE_LIMIT, KEPT_TRIANGLE_LIMIT};
use crate::sweep_row::SweepRow;
use crate::sweep_watchdog::{self, Thrown};
use crate::thingi10k_corpus::{mesh_path, Thingi10KModel};

/// The fractions of the room every pick is built at, and what each of them must do.
pub const FRACTIONS: [f64; 5] = [0.1, 0.5, 0.9, 1.0, 1.1];

/// How many times one pick's room may be re-read off a pinch refusal before the sweep gives up.
const MOST_REDERIVATIONS: i32 = 3;

fn read_room_error(error: ReadRoomError) -> Thrown {
    match error {
        ReadRoomError::Cancelled => {
            Thrown::new("OperationCanceledException", "the reading was cancelled")
        }
        ReadRoomError::Arguments(bad) => Thrown::new("ArgumentException", bad.to_string()),
        ReadRoomError::ChainField(bad) => Thrown::new("ArgumentException", bad.to_string()),
        ReadRoomError::Mesh(bad) => Thrown::new("ArgumentException", format!("{bad:?}")),
    }
}

/// Everything one model's sweep writes to.
pub struct Sinks<'a, W: Write> {
    pub rows: &'a mut Vec<SweepRow>,
    pub log: &'a mut W,
    pub failure_root: &'a Path,
    /// How many row lines were written to the log (the self-check counts them).
    pub row_lines: &'a mut i32,
}

impl<W: Write> Sinks<'_, W> {
    fn write(&mut self, row: SweepRow) -> std::io::Result<()> {
        *self.row_lines += sweep_report::write(self.log, row, self.rows)?;
        Ok(())
    }
}

/// Sweeps one model and returns how many of its picks were open chains (stage 2, not built).
pub fn sweep_model<W: Write>(
    model: &Thingi10KModel,
    root: &Path,
    bend_degrees: f64,
    picks_per_model: i32,
    seed: i32,
    sinks: &mut Sinks<'_, W>,
) -> std::io::Result<i32> {
    let path = mesh_path(root, model);
    if !path.exists() {
        sinks.write(SweepRow::model_aside(model.id, "Missing"))?;
        return Ok(0);
    }

    let (positions, indices) = match stl_reader::read_zip(&path) {
        Ok(welded) => welded,
        Err(thrown) => {
            sinks.write(SweepRow::model_aside(
                model.id,
                &format!("Threw:{}", thrown.name),
            ))?;
            return Ok(0);
        }
    };

    // The metadata lies sometimes, so the input is measured here by the same test the OUTPUT is
    // held to: every directed edge once and its opposite once.
    if !is_closed_and_consistent(&positions, &indices) {
        sinks.write(SweepRow::model_aside(model.id, "InputNotClosed"))?;
        return Ok(0);
    }

    // A triangle of no area in the INPUT is carried through every build untouched - it is in
    // neither band - and would then be read off the result as the kernel's fault. 65394 holds
    // exactly two (a pair whose second and third corners sit at the identical point
    // [31.6784, -31.6784, 0]) and they accounted for all eleven Unsound:DegenerateTriangle rows of
    // the third sweep. No sound output can be asked of an unsound input, so the model is set
    // aside here, like a model that does not close.
    if smallest_area(&positions, &indices).partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater) {
        sinks.write(SweepRow::model_aside(model.id, "InputDegenerateTriangle"))?;
        return Ok(0);
    }

    let features = Arc::new(feature_edges(&positions, &indices, bend_degrees));
    let is_feature = {
        let features = Arc::clone(&features);
        move |a: i32, b: i32| features.contains(&key(a, b))
    };

    // Mixed by hand rather than by a salted hash: a hash SALTED PER PROCESS would draw a different
    // set of faces for the same model and seed on every run, so "model N pick K" would name a
    // different face each time and no row of any report could ever be found again.
    let mixed = seed.wrapping_mul(397) ^ model.id;
    let mut random = SeededRandom::new(mixed);

    // The open picks draw from a stream of their OWN so that adding them moved no closed pick: the
    // closed picks are drawn first, from the same numbers they always were, and the open ones are
    // appended after them, so "model N pick K" still names the same closed pick as in earlier
    // reports.
    let mut open_random = SeededRandom::new(mixed.wrapping_mul(31).wrapping_add(17));
    let picks = sweep_pick::picks(
        &positions,
        &indices,
        &is_feature,
        &mut random,
        picks_per_model,
        &mut open_random,
        2.max(picks_per_model / 2),
    );
    let model_diagonal = diagonal(&positions, 0..positions.len() as i32);
    let mut open_chains = 0;

    let positions = Arc::new(positions);
    let indices = Arc::new(indices);
    for (p, pick) in picks.iter().enumerate() {
        let p = p as i32;
        if pick.loops.is_empty() {
            open_chains += i32::from(pick.kind == "chain");
            sinks.write(SweepRow::aside(
                model.id,
                pick.aside.as_deref().unwrap_or(""),
                p,
                &pick.kind,
                None,
                0,
            ))?;
            continue;
        }

        sweep_pick(
            model,
            p,
            pick,
            &positions,
            &indices,
            &features,
            model_diagonal,
            sinks,
        )?;
    }

    Ok(open_chains)
}

#[allow(clippy::too_many_arguments)]
fn sweep_pick<W: Write>(
    model: &Thingi10KModel,
    p: i32,
    pick: &SweepPick,
    positions: &Arc<Vec<solidmesh_math::Vector3>>,
    indices: &Arc<Vec<(i32, i32, i32)>>,
    features: &Arc<HashSet<(i32, i32)>>,
    model_diagonal: f64,
    sinks: &mut Sinks<'_, W>,
) -> std::io::Result<()> {
    let profile = if p % 2 == 0 {
        BandProfile::Fillet
    } else {
        BandProfile::Chamfer
    };
    let pick_diagonal = diagonal(positions, pick.loops.iter().flatten().copied());
    if pick_diagonal.partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater) {
        return sinks.write(SweepRow::aside(
            model.id,
            "DegeneratePick",
            p,
            &pick.kind,
            None,
            0,
        ));
    }

    // The room is ASKED for rather than probed at a tiny size: it is a statement about the part and
    // the pick, not about a size, so no band need be built to read it. The WHOLE pick is read in
    // ONE call - its loops share one field, so the pick's room is ONE number rather than the least
    // of numbers read loop by loop, each on a mesh the loop before it had changed.
    let input = PickInput {
        positions: Arc::clone(positions),
        indices: Arc::clone(indices),
        loops: Arc::new(pick.loops.clone()),
        closed: Arc::new(pick.closed.clone()),
        features: Arc::clone(features),
    };
    let open_pick = pick.closed.iter().any(|c| !c);

    // The reading's own input goes to disk BEFORE the thread starts, so a reading that wedges (or
    // takes the host down with it) still leaves behind exactly what was asked of it.
    let room_dump = dump(
        sinks.failure_root,
        model.id,
        p,
        "room",
        positions,
        indices,
        &pick.loops,
        &pick.closed,
        features,
        0.0,
        profile,
        None,
    )?;
    let reading_input = input.clone();
    let probe = sweep_watchdog::run(
        WorkBudget::default_limit(indices.len() as i32),
        move |budget| -> Result<RoomReading, Thrown> {
            read_room_of_pick(
                &reading_input.positions,
                &reading_input.indices,
                &reading_input.loops,
                &reading_input.closed,
                Some(reading_input.feature_test()),
                profile,
                1e-9,
                Some(budget),
                f64::NAN,
                seam_epsilon_for_positions(&reading_input.positions),
                f64::INFINITY,
            )
            .map_err(read_room_error)
        },
    );

    if let Some(outcome) = probe.outcome {
        sinks.write(SweepRow::aside(
            model.id,
            &format!("Probe{outcome}"),
            p,
            &pick.kind,
            probe.detail,
            probe.work,
        ))?;
        let limit = if outcome == "Hung" {
            HUNG_TRIANGLE_LIMIT
        } else {
            KEPT_TRIANGLE_LIMIT
        };
        keep(Some(&room_dump), indices.len(), limit);
        return Ok(());
    }

    let reading = probe
        .value
        .expect("a guarded call that did not end has a value");

    // A reading that ran out of work says so by name and its room is NaN - which is why the room is
    // never read off a comparison here; Read is the question.
    let refused = |name: &str, detail: Option<String>| {
        SweepRow::aside(model.id, name, p, &pick.kind, detail, reading.work_spent)
    };
    if !reading.read {
        sinks.write(refused(
            "RoomBudgetExceeded",
            Some(format!("{:?}", reading.phase)),
        ))?;
        keep(Some(&room_dump), indices.len(), KEPT_TRIANGLE_LIMIT);
        return Ok(());
    }

    // A chain edge that does not stand on exactly two triangles is a statement about the MESH under
    // this pick, and it gets a class of its own rather than being lumped in with a pick that simply
    // has no room.
    let named = if !reading.chain_edge_stands_on_two_triangles {
        Some((
            "ChainEdgeNotOnTwoTriangles",
            format!("{}-{}", reading.offending_edge.0, reading.offending_edge.1),
        ))
    } else if !reading.chains_are_apart {
        // Two statements a pick of SEVERAL loops can make that a pick of one never could, each its
        // own class: no size builds either of them, so neither is a room of nothing to be counted
        // as one.
        Some((
            "ChainsTouch",
            format!(
                "chains {} and {} at vertex {}",
                reading.offending_chain, reading.other_chain, reading.offending_vertex
            ),
        ))
    } else if !reading.chain_is_simple {
        // A chain that comes back through a vertex it has already been through - a figure-eight
        // face rim, which the collected picks really do produce. Its own class: no size builds it.
        Some((
            "ChainTouchesItself",
            format!(
                "chain {} at vertex {}",
                reading.offending_chain, reading.offending_vertex
            ),
        ))
    } else if !reading.chain_divides_sides {
        Some((
            "ChainDoesNotDivideItsSides",
            format!("chain {}", reading.offending_chain),
        ))
    } else {
        None
    };
    if let Some((name, detail)) = named {
        sinks.write(refused(name, Some(detail)))?;
        keep(Some(&room_dump), indices.len(), KEPT_TRIANGLE_LIMIT);
        return Ok(());
    }

    let _ = std::fs::remove_file(&room_dump);

    let mut room = reading.room;
    let mut set_by = sweep_build::said(&reading.room_set_by);

    // An OPEN pick's room can read infinite or NaN - at a FADE end nothing yet limits the size, a
    // known hole in `band_gap_reading::read_room` - and infinity times a fraction is no size at
    // all. Such a pick's sizes come off a stated fallback ladder instead: the reference size is the
    // SMALLER of an eighth of the pick's own length (its edges added up) and a twentieth of the
    // mesh's bounding diagonal, whichever is the tighter statement about this part. The row says
    // so, and the "1.1 must refuse by size" expectation is off for it, because 1.1 of a fallback is
    // not over any room the kernel named.
    let mut room_not_finite = open_pick && !room.is_finite();
    let fallback = if room_not_finite {
        (pick_length(positions, &pick.loops, &pick.closed) / 8.0).min(model_diagonal * 0.05)
    } else {
        0.0
    };

    // A room of zero is the construction saying there is no size at all that builds here. That is a
    // finding about the PICK, not five rows of ArgumentOutOfRangeException from asking the kernel
    // to build at a size of nothing.
    let has_size = if room_not_finite {
        fallback > 0.0
    } else {
        room > 0.0
    };
    if !has_size {
        let name = if room_not_finite {
            "NoFallbackSize"
        } else {
            "NoRoom"
        };
        return sinks.write(SweepRow::aside(model.id, name, p, &pick.kind, None, 0));
    }

    let mut unbounded = room_not_finite || room.is_infinite();
    let mut reference = if room_not_finite {
        fallback
    } else if unbounded {
        model_diagonal * 0.05
    } else {
        room
    };
    let mut hung = false;

    // A pick whose side PINCHES has a true room BELOW the one the width reading names, and only a
    // build finds it - see `band_gap_entry_points::build`. When one does, the room it names is
    // taken as this pick's room and the fractions are walked again from the top against it, so the
    // pick still gets its
    // 0.1 to 1.0 builds and its 1.1 refusal against a room that is true. Bounded, because each
    // re-reading is strictly smaller than the last.
    let mut rederivations = 0;

    let mut f = 0;
    while f < FRACTIONS.len() {
        let fraction = FRACTIONS[f];

        if hung {
            sinks.write(SweepRow::of(
                model.id,
                p,
                &pick.kind,
                fraction,
                0.0,
                "SkippedAfterHang",
                0.0,
                0,
                None,
                Some(set_by.clone()),
                room_not_finite,
            ))?;
            f += 1;
            continue;
        }

        let size = reference * fraction;
        let watch = Instant::now();
        let attempt = sweep_build::build_the_pick(
            &input,
            size,
            profile,
            sinks.failure_root,
            model.id,
            p,
            fraction,
        )?;
        let seconds = watch.elapsed().as_secs_f64();

        let outcome = match &attempt.outcome {
            Some(outcome) => outcome.clone(),
            None => sweep_build::classify(
                &attempt,
                fraction,
                room,
                unbounded,
                set_by == "Pinch",
                positions,
                indices,
            ),
        };

        if outcome == sweep_build::PINCH_OUTCOME
            && rederivations < MOST_REDERIVATIONS
            && attempt.room > 0.0
            && attempt.room < room
        {
            sinks.write(SweepRow::of(
                model.id,
                p,
                &pick.kind,
                fraction,
                size,
                &outcome,
                seconds,
                attempt.work,
                attempt.detail.clone(),
                Some(set_by.clone()),
                room_not_finite,
            ))?;
            keep(attempt.dump_path.as_deref(), attempt.dump_triangles, 0);
            for (earlier, triangles) in &attempt.earlier_dumps {
                keep(Some(earlier), *triangles, 0);
            }

            room = attempt.room;
            set_by = "Pinch".to_string();

            // A build that read a pinch has named a room that IS finite, so the pick leaves the
            // fallback ladder behind and its fractions mean what they mean everywhere else.
            room_not_finite = false;
            unbounded = false;
            reference = room;
            rederivations += 1;
            f = 0;
            continue;
        }

        let row = SweepRow::of(
            model.id,
            p,
            &pick.kind,
            fraction,
            size,
            &outcome,
            seconds,
            attempt.work,
            attempt.detail.clone(),
            Some(set_by.clone()),
            room_not_finite,
        );
        let as_expected = row.as_expected();
        sinks.write(row)?;

        // The input of the call that ENDED the row stays on disk unless the row did what it
        // promised; a Hung row is worth a bigger mesh than an ordinary failure, because a hang is
        // the one class that cannot be re-read from the report alone.
        let limit = if as_expected {
            0
        } else if outcome == "Hung" {
            HUNG_TRIANGLE_LIMIT
        } else {
            KEPT_TRIANGLE_LIMIT
        };
        keep(attempt.dump_path.as_deref(), attempt.dump_triangles, limit);
        for (earlier, triangles) in &attempt.earlier_dumps {
            keep(Some(earlier), *triangles, limit);
        }

        if outcome == "Hung" {
            hung = true;
        }

        f += 1;
    }

    Ok(())
}
