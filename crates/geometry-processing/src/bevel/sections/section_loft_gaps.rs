//! The loft of a run whose split vertices were absorbed (`Bevel/Sections/SectionLoft.cs`:
//! `TryEmitGaps`, `BoundaryRowOf`, `TheGroupsTailPast`, `TheGroupsHeadBefore`,
//! `TryFanTheArrivingTakeover`, `RowEndAt`, `BoundaryOf`, `TryFanTheTakeover`, `LaysTheEdge`): one
//! ladder per GAP between consecutive unsplit stations, each gap's two boundary rows fanned over
//! the polyline that runs A's boundary, the gap's absorbed boundary points and the seam's fold
//! crossings between them, to B's boundary. Split out of `section_loft`, which holds the entry
//! point and the checks every triangle passes.
//!
//! **Why the split stations drop out of the ladder.** A split vertex stands on the straight chain
//! edge between its two unsplit neighbours with the same surfaces on either side the whole way, so
//! the strip A -> B is exactly the strip an unsplit mesh would get; the split station's own profile
//! row would only subdivide it, and that subdivision is what reverses a column beside a
//! fold-averaged neighbour on the tapered barrel's 39-station mouths. Its boundary point is still
//! laid - as a point OF the boundary row - so the face rebuild and the smooth cut, which substitute
//! every chain vertex's corners by that station's boundary point, keep meeting the loft by
//! identity.
//!
//! **Why no sorting.** The seam records its crossings per consecutive FULL-length station pair, and
//! the absorbed points ARE those intermediate stations' boundaries, so the merged polyline is a
//! concatenation in station order: A', crossings after A, S1', crossings after S1, ..., B'.

use super::section_loft::{
    cross_unless_degenerate, head_extras_of, not_built, outward_across, quad_at, split_by_parity,
    try_fan, try_fan_the_junction_extras, try_lay, Faces,
};
use super::section_smooth_seam::SectionSmoothSeam;
use super::section_smooth_side::SmoothSideCut;
use super::{BevelSection, SectionAbsorbedSplits, SectionPool, SectionRun};
use crate::bevel::BevelAdvisory;

/// C# `TryEmitGaps`. The caller takes back what this laid on a refusal.
pub(super) fn try_emit_gaps(
    run: &SectionRun,
    absorbed: &SectionAbsorbedSplits,
    pool: &SectionPool,
    faces: &mut Faces,
    seam: Option<&SectionSmoothSeam>,
    flipped: bool,
    steps: i32,
) -> Result<(), BevelAdvisory> {
    if steps < 2 {
        // With one segment the profile's single row is BOTH boundary rows, and a quad cannot be
        // fanned along two of its edges at once. Refuse rather than drop one side's absorbed
        // points.
        return Err(not_built(Some(
            run.sections[absorbed.stations[0] as usize].chain_vertex,
        )));
    }

    for g in 0..absorbed.gap_count() {
        let from = absorbed.stations[g as usize];
        let to = absorbed.stations[((g + 1) % absorbed.count()) as usize];
        let start = &run.sections[from as usize];

        // A JUNCTION station owns two sections, and sections[to] is only the LEAVING one. The gap
        // ARRIVES at `to`, so it ends on the arriving section, exactly as SectionRun::end_of hands
        // it to the unabsorbed path - reading sections[to] here pairs the gap's rungs one sample
        // out of step and leaves the junction rung's first interval laid by nothing at all.
        let end = match run.junctions.get(&to) {
            Some(arriving_at) => &arriving_at.arriving_section,
            None => &run.sections[to as usize],
        };

        let start_head = head_extras_of(run, from, start, steps);
        let end_head = head_extras_of(run, to, end, steps);
        let mut fanned_the_extras = false;

        // The last point of the SEAM side's polyline BEFORE any junction extras were walked into it
        // - the point the row's own chord out of the arriving station runs to, which is the
        // previous station's boundary wherever the gap absorbed a split and not the gap's start
        // section.
        let mut seam_row_last = -1;

        for k in 0..steps {
            let quad = quad_at(start, end, k, start_head, end_head);
            let outward = outward_across(start, end, pool, k, start_head, end_head);
            let is_left = k == 0;
            // The row carries the station's cut group tail only where the row actually STANDS on
            // that station's boundary point. At a junction whose extras stand between the row's own
            // end and the boundary the extras fan reaches it instead, and the tail goes with that
            // fan.
            let stands_on_the_boundary =
                row_end_at(start, start_head, steps, is_left) == boundary_of(start, is_left);

            let mut interior = if is_left || k == steps - 1 {
                Some(boundary_row_of(
                    run,
                    absorbed,
                    seam,
                    g,
                    from,
                    to,
                    is_left,
                    stands_on_the_boundary,
                ))
            } else {
                None
            };

            if let (Some(seam), Some(interior)) = (seam, interior.as_ref()) {
                if is_left == seam.is_left {
                    if let Some(&last) = interior.last() {
                        seam_row_last = last;
                    }
                }
            }

            // A junction row's extra samples stand between its WALL end and the near wall end, and
            // the wall end is the very point the face rebuild rebuilt its rim onto - so on the side
            // the extras sit, the boundary polyline has to run THROUGH them to reach it. Fanning
            // them separately instead leaves a four-sided hole between the two fans. The polyline
            // is walked back from the gap's end on the left, so they are added last, outermost
            // sample nearest the corner.
            if is_left && end_head > 0 {
                let row = interior.get_or_insert_with(Vec::new);

                for e in (0..end_head).rev() {
                    row.push(end.profile[e as usize]);
                }

                fanned_the_extras = true;
            }

            if let Some(interior) = interior.as_ref().filter(|row| !row.is_empty()) {
                try_fan(
                    quad,
                    interior,
                    is_left,
                    outward,
                    flipped,
                    start.chain_vertex,
                    pool,
                    faces,
                )?;

                continue;
            }

            let (first, second) = split_by_parity(quad, g + k);

            try_lay(first, pool, outward, flipped, start.chain_vertex, faces)?;
            try_lay(second, pool, outward, flipped, start.chain_vertex, faces)?;
        }

        // A gap ending or starting at a junction has one rung longer than the ladder, the same as
        // an unabsorbed run's strip does, and closes it the same way - unless the boundary polyline
        // above already ran through the extras, which it does wherever the two meet.
        if !fanned_the_extras {
            try_fan_the_junction_extras(
                start, end, start_head, end_head, pool, steps, flipped, faces,
            )?;
        }

        try_fan_the_takeover(start, end, start_head, steps, seam, from, pool, faces)?;

        try_fan_the_arriving_takeover(
            start,
            end,
            start_head,
            end_head,
            steps,
            seam,
            to,
            seam_row_last,
            fanned_the_extras,
            pool,
            faces,
        )?;
    }

    Ok(())
}

/// C# `BoundaryRowOf`: the points between gap `gap`'s two ends on one side, in CHAIN order: the
/// absorbed split boundaries and, where that side carries the seam, the fold crossings of each
/// full-length station pair the gap spans. Empty where the gap absorbs nothing and crosses nothing,
/// which is the plain quad row.
#[allow(clippy::too_many_arguments)]
fn boundary_row_of(
    run: &SectionRun,
    absorbed: &SectionAbsorbedSplits,
    seam: Option<&SectionSmoothSeam>,
    gap: i32,
    from: i32,
    to: i32,
    is_left: bool,
    stands_on_the_boundary: bool,
) -> Vec<i32> {
    let points = if is_left {
        absorbed.left_of(gap)
    } else {
        absorbed.right_of(gap)
    };
    let cut = seam
        .filter(|seam| seam.is_left == is_left)
        .map(|seam| &seam.cut);
    let stations = run.sections.len() as i32;
    let mut row = Vec::new();
    let mut next = 0usize;

    // The gap's own end stands at station `from`'s BOUNDARY point, but that station's group on the
    // cut can run PAST it - a junction whose rim takes over from a feature crease mints a takeover
    // point after the carried wall end - and the cut's boundary runs through those before it
    // reaches the next station. The row is the only thing that lays that stretch, so it carries the
    // group's tail first, in walk order: the polyline then runs boundary, takeover, crossings,
    // absorbed, and the little triangle between the wall end, the takeover and the next station
    // closes.
    if stands_on_the_boundary {
        row.extend_from_slice(the_groups_tail_past(
            cut,
            from,
            boundary_of(&run.sections[from as usize], is_left),
        ));
    }

    let mut at = from;

    loop {
        // By STATION, not by flattened boundary index - a junction hands three points and the two
        // readings part company after it. See SmoothSideCut::crossings_after_station.
        if let Some(crossings) = cut.and_then(|cut| cut.crossings_after_station(at)) {
            row.extend_from_slice(crossings);
        }

        if (at + 1) % stations == to {
            break;
        }

        row.push(points[next].pool_index);
        next += 1;

        at = (at + 1) % stations;
    }

    row
}

/// C# `TheGroupsTailPast`: station `station`'s cut group past `row_end`, in walk order - the
/// points the cut's boundary carries after the very point the loft's row stands on, which is empty
/// for every station whose group ends at its boundary point (that is, all of them but a junction
/// the rim takes over at).
fn the_groups_tail_past(cut: Option<&SmoothSideCut>, station: i32, row_end: i32) -> &[i32] {
    let Some(group) = cut.and_then(|cut| cut.station_points.get(station as usize)) else {
        return &[];
    };

    // By INDEX, never by position: the row's end IS one of the group's points where it is one at
    // all.
    match group.iter().rposition(|&point| point == row_end) {
        Some(at) if at != group.len() - 1 => &group[at + 1..],
        _ => &[],
    }
}

/// C# `TheGroupsHeadBefore`: station `station`'s cut group BEFORE `row_end`, in walk order - the
/// points the cut's boundary carries before it reaches the very point the loft's row stands on,
/// which is empty for every station whose group starts at the row's own point (that is, all of them
/// but a junction the gap ARRIVES at, where the rim's takeover is minted ahead of the wall ends).
fn the_groups_head_before(cut: &SmoothSideCut, station: i32, row_end: i32) -> &[i32] {
    let Some(group) = cut.station_points.get(station as usize) else {
        return &[];
    };

    // By INDEX, never by position, for the same reason the_groups_tail_past reads by index.
    match group.iter().position(|&point| point == row_end) {
        Some(at) if at > 0 => &group[..at],
        _ => &[],
    }
}

/// C# `TryFanTheArrivingTakeover`: the MIRROR of [`try_fan_the_takeover`] - the wedge between the
/// chord a gap's row laid into the station it ARRIVES at and the points that station's cut group
/// carries BEFORE that chord's end.
///
/// Where the gap ends at a junction the cut walks its group the other way round - the takeover is
/// the group's FIRST point - so the row's polyline cannot carry it the way [`boundary_row_of`]
/// carries the START station's tail: the row has already arrived. One triangle per head point
/// closes it, standing on the point the row's own chord ran to, which is the last point of the
/// row's polyline where the gap absorbed anything and the start section's own row end where it did
/// not.
#[allow(clippy::too_many_arguments)]
fn try_fan_the_arriving_takeover(
    start: &BevelSection,
    end: &BevelSection,
    start_head: i32,
    end_head: i32,
    steps: i32,
    seam: Option<&SectionSmoothSeam>,
    to: i32,
    seam_row_last: i32,
    fanned_the_extras: bool,
    pool: &SectionPool,
    faces: &mut Faces,
) -> Result<(), BevelAdvisory> {
    let Some(seam) = seam else {
        return Ok(());
    };

    let is_left = seam.is_left;

    // Where the arriving junction's EXTRAS were walked into the row, the polyline runs on through
    // them past the row's own quad corner, and the point it stands on where it turns back towards
    // the previous station is the OUTERMOST extra - profile 0, the carried wall end itself.
    let row_end = if fanned_the_extras {
        end.profile[0]
    } else {
        row_end_at(end, end_head, steps, is_left)
    };
    let head = the_groups_head_before(&seam.cut, to, row_end);

    if head.is_empty() {
        return Ok(());
    }

    // What the row's chord out of the arriving corner actually runs to: the polyline's last point
    // on the seam's side, and the start section's own row end where the gap's row carried nothing.
    let apex = if seam_row_last >= 0 {
        seam_row_last
    } else {
        row_end_at(start, start_head, steps, is_left)
    };

    // Wound by IDENTITY against that chord, for the reason try_fan_the_takeover gives: the wedge
    // lies flat on the part's own face past the wall end, where the sweep's outward reads the other
    // way.
    let along_the_row = lays_the_edge(faces, row_end, apex);
    let mut previous = row_end;

    for &point in head.iter().rev() {
        let triangle = if along_the_row {
            (apex, previous, point)
        } else {
            (apex, point, previous)
        };

        if cross_unless_degenerate(pool, triangle).is_none() {
            return Err(not_built(Some(end.chain_vertex)));
        }

        faces.push(triangle);
        previous = point;
    }

    Ok(())
}

/// C# `RowEndAt`: the pool index the gap's boundary row STANDS on at its start section - the corner
/// of the row's own quad that lies on the boundary, which is the profile's head on the LEFT and its
/// tail on the RIGHT. At a junction this is the near WALL end and not the station's boundary point,
/// the extras standing between the two.
fn row_end_at(start: &BevelSection, start_head: i32, steps: i32, is_left: bool) -> i32 {
    if is_left {
        start.profile[start_head as usize]
    } else {
        start.profile[(steps + start_head) as usize]
    }
}

/// C# `BoundaryOf`: the section's own boundary point on one side - the end of its profile that side
/// lies at.
fn boundary_of(section: &BevelSection, is_left: bool) -> i32 {
    if is_left {
        section.left_boundary
    } else {
        section.right_boundary
    }
}

/// C# `TryFanTheTakeover`: lays the wedge between a junction's boundary point, the points its cut
/// group runs on to and the other row's own boundary - the TAKEOVER, where the rim takes over from a
/// feature crease past the carried wall end and the cut's boundary carries a point the loft's rows
/// never reach.
///
/// Only where the junction's EXTRAS stand between the row and the boundary, since that is the case
/// in which the row cannot carry the tail itself: the extras are fanned onto the other row's
/// boundary point by `try_fan_the_junction_extras`, so the stretch from the junction's boundary to
/// the takeover and on to that same point is one triangle per tail point, wound the way that fan
/// is. (C#'s `endHead` and `flipped` parameters are not read; they are not taken here.)
#[allow(clippy::too_many_arguments)]
fn try_fan_the_takeover(
    start: &BevelSection,
    end: &BevelSection,
    start_head: i32,
    steps: i32,
    seam: Option<&SectionSmoothSeam>,
    from: i32,
    pool: &SectionPool,
    faces: &mut Faces,
) -> Result<(), BevelAdvisory> {
    let Some(seam) = seam else {
        return Ok(());
    };

    let is_left = seam.is_left;
    let boundary = boundary_of(start, is_left);

    if row_end_at(start, start_head, steps, is_left) == boundary {
        // The row stands on the boundary itself, so it carried the tail - see boundary_row_of.
        return Ok(());
    }

    let tail = the_groups_tail_past(Some(&seam.cut), from, boundary);

    if tail.is_empty() {
        return Ok(());
    }

    // The same apex the extras fan runs onto: where the extras stand at the start's HEAD the other
    // row's own head is the point beside them, and where they stand at its tail it is its tail.
    let apex_at = if start_head > 0 { 0 } else { end.segments() };
    let apex = end.profile[apex_at as usize];

    // The wedge's winding is settled by IDENTITY against the row that already laid the chord from
    // the apex to the junction's boundary, not by the sweep's outward: the wedge lies flat on the
    // PART's own face past the wall end - measured at the pierced face, its normal is that face's
    // and the sweep's outward there reads the other way - so an outward test would refuse the very
    // triangle the seam needs. Closing against a laid edge cannot be wrong for the same reason the
    // strips close by index.
    let along_the_row = lays_the_edge(faces, apex, boundary);
    let mut previous = boundary;

    for &point in tail {
        let triangle = if along_the_row {
            (previous, apex, point)
        } else {
            (apex, previous, point)
        };

        if cross_unless_degenerate(pool, triangle).is_none() {
            return Err(not_built(Some(start.chain_vertex)));
        }

        faces.push(triangle);
        previous = point;
    }

    Ok(())
}

/// C# `LaysTheEdge`: whether any face laid so far runs the directed edge `from -> to`.
fn lays_the_edge(faces: &Faces, from: i32, to: i32) -> bool {
    faces.iter().any(|&(v0, v1, v2)| {
        (v0 == from && v1 == to) || (v1 == from && v2 == to) || (v2 == from && v0 == to)
    })
}
