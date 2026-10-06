//! The gap fill of ONE closed chain's band: its two rails paired to run opposite ways, merged into
//! spokes, the spokes drawn as the fillet's arcs and the cells between them laid as strips - and
//! taken back out if the shell does not close. Ports `GapFill.FillOneChain` from
//! `Bevel/LevelLine/GapFill.cs`; the senses, the pairing, the merge, the arcs and the strip are in
//! `gap_fill`, the rails in `zip_rail`.

use std::cell::RefCell;
use std::rc::Rc;

use super::gap_fill::{
    arcs, backwards, merge, monotone_slack, not_monotone, senses, strip, try_pair,
    PARAMETER_TIE_RELATIVE,
};
use super::{
    BandProfile, ChainField, GapFillRefusal, GapFillResult, GapLoop, LevelLineMeshError,
    LevelLineWork, ZipRail,
};

/// C# `GapFill.FillOneChain`: fills the band of one closed chain between its side-1 and side-2
/// loops, appending the laid triangles to `laid`. Refuses (with nothing left laid) where a rail is
/// not monotone, the rails run the same way, or the strip does not close the shell.
pub fn fill_one_chain(
    mesh: &Rc<RefCell<super::LevelLineMesh>>,
    chain: &ChainField,
    loops: &[GapLoop],
    profile: BandProfile,
    fillet_segments: i32,
    laid: &mut Vec<i32>,
) -> Result<GapFillResult, LevelLineMeshError> {
    let start = laid.len();
    let slack = monotone_slack(chain);

    // C#'s `Single`: exactly one loop per side is the caller's promise.
    let of_side = |side: i32| {
        let mut found = loops.iter().filter(|l| l.side == side);
        let one = found.next().expect("one loop on each side");
        assert!(found.next().is_none(), "one loop on each side");
        one
    };
    let left_senses = senses(of_side(1), chain, slack);
    let right_senses = senses(of_side(2), chain, slack);
    if left_senses.forward.is_none() && left_senses.backward.is_none() {
        return Ok(not_monotone(mesh.clone(), left_senses.offending));
    }

    if right_senses.forward.is_none() && right_senses.backward.is_none() {
        return Ok(not_monotone(mesh.clone(), right_senses.offending));
    }

    // The two loops run OPPOSITE ways round the chain: exactly one of the pair reverses its loop
    // to run in increasing parameter, so each spoke carries one triangle each way.
    let Some((left, right)) = try_pair(&left_senses, &right_senses) else {
        return Ok(GapFillResult {
            refusal: GapFillRefusal::RailsRunTheSameWay,
            mesh: Some(mesh.clone()),
            ..GapFillResult::default()
        });
    };
    let mut left: ZipRail = (*left).clone();
    let right: ZipRail = (*right).clone();

    let length = chain.length;
    let tie = PARAMETER_TIE_RELATIVE * length;

    // A genuine two-leg tie stands only on a sharp joint's bisector, so a chamfer of a chain with
    // no sharp joint reads each vertex at its own parameter; a fillet reads them all.
    let reads_ties = (profile != BandProfile::Chamfer && fillet_segments != 1)
        || !chain.sharp_joints().is_empty();
    let spans = |rail: &ZipRail| -> Result<Vec<(f64, f64)>, LevelLineMeshError> {
        if reads_ties {
            Ok(super::rail_stations::tie_spans(
                &mesh.borrow(),
                chain,
                &rail.vertices,
                &rail.parameters,
                tie,
                length,
            )?)
        } else {
            Ok(rail.parameters.iter().map(|&p| (p, p)).collect())
        }
    };

    // A rail that JUMPS across its wrap begins instead at its vertex nearest the other's start.
    let mut left_spans = spans(&left)?;
    let started = ZipRail::start_nearest(
        &mesh.borrow(),
        &left,
        &left_spans,
        right.parameters[0],
        length,
        chain.max_setback,
    );
    // C# compares by reference: `start_nearest` hands back the rail itself where it does not re-cut
    // it, and a re-cut rail differs from it.
    if started != left {
        left = started;
        left_spans = spans(&left)?;
    }

    // The merge reads every rail vertex at one canonical parameter.
    let right_spans = spans(&right)?;
    let left_read = ZipRail::canonical(&left, &left_spans);
    let right_read = ZipRail::canonical(&right, &right_spans);

    // Read the right rail's start in the left one's frame.
    let mut right_offset = 0.0;
    while right_read.parameters[0] + right_offset < left_read.parameters[0] - (length / 2.0) {
        // A chain of zero or NaN length would never bring these together; the budget stops it.
        LevelLineWork::charge(1)?;
        right_offset += length;
    }

    while right_read.parameters[0] + right_offset > left_read.parameters[0] + (length / 2.0) {
        LevelLineWork::charge(1)?;
        right_offset -= length;
    }

    let advances = merge(
        &mesh.borrow(),
        chain,
        &left_read,
        &right_read,
        length,
        right_offset,
    )?;
    let segments = if profile == BandProfile::Chamfer {
        1
    } else {
        fillet_segments
    };
    let mut added_vertices = Vec::new();
    let mut mesh_mut = mesh.borrow_mut();
    let the_arcs = arcs(
        &mut mesh_mut,
        chain,
        &advances,
        &left,
        &right,
        &left_spans,
        &right_spans,
        length,
        right_offset,
        segments,
        &mut added_vertices,
    );

    for (step, &(_, _, advances_left)) in advances.iter().enumerate() {
        let before = &the_arcs[step];
        let after = &the_arcs[step + 1];
        let (u, v) = if advances_left {
            // Both spokes end at the right vertex the cell fans about.
            if left.reversed {
                (before.clone(), after.clone())
            } else {
                (after.clone(), before.clone())
            }
        } else if right.reversed {
            // Turned about, so index 0 is the right rail's vertex.
            (backwards(before), backwards(after))
        } else {
            (backwards(after), backwards(before))
        };

        strip(&mut mesh_mut, &u, &v, laid, None)?;
    }

    let added_triangles = laid[start..].to_vec();
    let open = super::fill_closure::first_open_edge(&mesh_mut, loops, &added_triangles);
    if open.0 >= 0 {
        // The construction promises a closed shell; when it did not deliver one the strip is taken
        // back out and the fill says so - never a repair pass.
        for &triangle in &added_triangles {
            mesh_mut.remove_triangle(triangle);
        }

        laid.truncate(start);
        drop(mesh_mut);
        return Ok(GapFillResult {
            refusal: GapFillRefusal::FillWouldNotClose,
            open_edge: open,
            mesh: Some(mesh.clone()),
            ..GapFillResult::default()
        });
    }

    drop(mesh_mut);
    Ok(GapFillResult {
        refusal: GapFillRefusal::None,
        mesh: Some(mesh.clone()),
        added_triangles,
        added_vertices,
        ..GapFillResult::default()
    })
}
