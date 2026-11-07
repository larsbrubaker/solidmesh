//! An edge's STRIP: the quad ladder between the profiles of the two sections its ends stand at,
//! laid as triangles into a face list. This file holds the entry point ([`try_emit`]), the plain
//! station-to-station ladder, the smooth side's fan over the boundary polyline and the winding and
//! degeneracy checks every triangle passes; the ladder of a run whose split vertices were absorbed
//! - gap to gap, with the takeover wedges at a junction - is `section_loft_gaps`.
//!
//! **There is nothing to join.** Strip `i` ends on `SectionRun::end_of(i)`'s profile and strip
//! `i + 1` starts on `start_of(i + 1)`'s, and on a closed run those are one object. So the seam
//! between two strips is one row of pool indices read twice: the quads on either side of it
//! already share their corner integers, and no weld, tolerance or find-near takes any part in
//! making them one place.
//!
//! **The diagonal, by index parity.** A quad is split on the diagonal named by `(i + k)` being even
//! or odd - a fact about where the quad sits in the ladder, not about its shape. Splitting on the
//! shorter diagonal, or on the one that reads flatter, would be a decision made from coordinates,
//! and a decision made from coordinates is one a rigid motion can change: that is exactly the class
//! of failure the robustness harness measures. Parity cannot be changed by moving the part.
//!
//! **The winding, decided ONCE per run.** A rolling ball's surface stands a radius from the ball's
//! centre, so on a CONVEX chain - the only kind `SectionBuilder` draws, since it puts the centre
//! inside the material at `V - r (nL + nR) / (1 + c)` - the outward normal at a profile point is the
//! way from that centre to the point. The loft reads that once, off the first quad of the run, and
//! then lays every other quad the same way round. Per-triangle orientation decisions are what
//! leaves one face inside out where an arithmetic was marginal; here a quad that disagrees with the
//! run's own winding is a REFUSAL, not a quietly flipped triangle.
//!
//! **A concave chain** would need the opposite sign: its ball rides in the air rather than in the
//! material, so its surface faces TOWARDS the centre. It is out of scope here because it is out of
//! scope one step earlier - the centre formula the sections are built with is the convex one - so a
//! concave chain reaching this far shows up as a quad whose normal disagrees with the run's winding
//! and is refused rather than lofted inside out.

use vector_math::{ieee_math, Vector3};

use super::section_loft_gaps;
use super::section_smooth_seam::SectionSmoothSeam;
use super::{section_split_blend, BevelSection, SectionKind, SectionPool, SectionRun};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// One rung's quad, `Q0 Q1 Q2 Q3`.
pub(super) type Quad = (i32, i32, i32, i32);

/// The face list the strips are appended to.
pub(super) type Faces = Vec<(i32, i32, i32)>;

/// How small a triangle's cross product may be, RELATIVE to its own
/// longest edge squared, before it is called degenerate. Relative because an absolute area would
/// mean one thing at the origin and another thirty times coarser at a hanger's stand-off, and then
/// where a part stood would decide whether its loft was refused.
pub(super) const SMALLEST_RELATIVE_CROSS: f64 = 1e-12;

/// The loft's one refusal, `SurfaceConstructionNotBuilt`, quoting the chain vertex where one is
/// known.
pub(super) fn not_built(chain_vertex: Option<i32>) -> BevelAdvisory {
    BevelAdvisory::new(
        BevelAdvisoryCode::SurfaceConstructionNotBuilt,
        chain_vertex
            .map(|vertex| vec![vertex.into()])
            .unwrap_or_default(),
    )
}

/// Lofts every strip of `run` into `faces`, appending and minting nothing - every
/// corner is a pool index a section already published. `seam` is the run's smooth side and its
/// cut, or `None` where it has neither.
///
/// With a seam, the run's SMOOTH side's last row is lofted to the boundary the cut actually ran
/// along. On a smooth side the part's edge between two stations is not the chord `B(i) -> B(i+1)`
/// at all: the boundary was laid along the surface and crossed folds on the way, so the part's edge
/// there is the POLYLINE `B(i), c1 .. ck, B(i+1)`. Lofting the chord would leave every crossing a T
/// junction against the facet rings the cut kept. So the last quad of that side becomes a FAN over
/// the polyline, using the cut's own pool indices - after which every directed edge of the polyline
/// is run once by the loft and once the other way by a kept ring, and the seam is closed by
/// identity.
///
/// **Which apex.** The fan is split at the crossing NEAREST THE MIDDLE of the polyline, the two
/// inner corners taking a half each, with one triangle across the diagonal between them - for the
/// reason the quad's own diagonal is chosen by parity. A polyline of one segment carries no
/// crossing and is lofted as the plain quad it is.
///
/// On a refusal nothing this call laid is left in `faces`. An open run whose section list stops
/// short of its last edge (`end_of`'s refusal) is refused the same way
/// as any other run that cannot loft.
pub fn try_emit(
    run: &SectionRun,
    pool: &SectionPool,
    faces: &mut Faces,
    seam: Option<&SectionSmoothSeam>,
) -> Result<(), BevelAdvisory> {
    if run.count() < 1 {
        return Err(not_built(None));
    }

    let steps = steps_of(run);

    for i in 0..run.count() {
        let start = run.start_of(i);
        let end = run
            .end_of(i)
            .map_err(|_| not_built(Some(start.chain_vertex)))?;

        if !rung_stands(&start, steps) || !rung_stands(&end, steps) {
            // Two sections drawn in a different number of segments have no ladder between them:
            // the rungs would have to be matched up by position, which is the thing this path
            // exists to do without.
            return Err(not_built(Some(start.chain_vertex)));
        }
    }

    // A run whose split vertices were absorbed lays its ladders GAP TO GAP: the split stations
    // contribute boundary points to their gap's two boundary rows and no profile row of their own.
    // An OPEN run's gap list simply leaves the wrap-around gap out; its first and last stations are
    // its end sections, which are never split and so are always gap ends.
    let absorbed = if section_split_blend::absorb_split_vertices() {
        run.absorbed.as_ref()
    } else {
        None
    };

    let (wind_start, wind_end) = match absorbed {
        None => (
            run.start_of(0),
            run.end_of(0)
                .map_err(|_| not_built(Some(run.start_of(0).chain_vertex)))?,
        ),
        Some(absorbed) => (
            run.sections[absorbed.stations[0] as usize].clone(),
            run.sections[absorbed.stations[(1 % absorbed.count()) as usize] as usize].clone(),
        ),
    };

    let flipped = try_winding_of_the_run(&wind_start, &wind_end, pool)?;

    let laid = faces.len();

    let outcome = match absorbed {
        Some(absorbed) => {
            section_loft_gaps::try_emit_gaps(run, absorbed, pool, faces, seam, flipped, steps)
        }
        None => try_emit_ladders(run, pool, faces, seam, flipped, steps),
    };

    if outcome.is_err() {
        faces.truncate(laid);
    }

    outcome
}

/// The station-to-station ladders of a run without absorbed splits ([`try_emit`]'s body past the
/// winding). The caller takes back what this laid on a refusal.
fn try_emit_ladders(
    run: &SectionRun,
    pool: &SectionPool,
    faces: &mut Faces,
    seam: Option<&SectionSmoothSeam>,
    flipped: bool,
    steps: i32,
) -> Result<(), BevelAdvisory> {
    for i in 0..run.count() {
        let start = run.start_of(i);
        let end = run
            .end_of(i)
            .map_err(|_| not_built(Some(start.chain_vertex)))?;
        let ahead = if run.is_closed {
            (i + 1) % run.count()
        } else {
            i + 1
        };
        let start_head = head_extras_of(run, i, &start, steps);
        let end_head = head_extras_of(run, ahead, &end, steps);

        for k in 0..steps {
            let quad = quad_at(&start, &end, k, start_head, end_head);
            let crossings = crossings_on(seam, i, k, steps);

            if let (Some(crossings), Some(seam)) = (crossings, seam) {
                if !crossings.is_empty() {
                    let outward = outward_across(&start, &end, pool, k, start_head, end_head);

                    try_fan(
                        quad,
                        crossings,
                        seam.is_left,
                        outward,
                        flipped,
                        start.chain_vertex,
                        pool,
                        faces,
                    )?;

                    continue;
                }
            }

            let (first, second) = split_by_parity(quad, i + k);

            let outward = outward_across(&start, &end, pool, k, start_head, end_head);

            try_lay(first, pool, outward, flipped, start.chain_vertex, faces)?;
            try_lay(second, pool, outward, flipped, start.chain_vertex, faces)?;
        }

        try_fan_the_junction_extras(
            &start, &end, start_head, end_head, pool, steps, flipped, faces,
        )?;
    }

    Ok(())
}

/// The quad's two triangles on the diagonal `(i + k)`'s parity names.
pub(super) fn split_by_parity(quad: Quad, parity: i32) -> ((i32, i32, i32), (i32, i32, i32)) {
    let (q0, q1, q2, q3) = quad;

    if parity % 2 == 0 {
        ((q0, q1, q2), (q0, q2, q3))
    } else {
        ((q0, q1, q3), (q1, q2, q3))
    }
}

/// How many intervals the run's ladders are drawn in - read off a rung that is NOT a
/// [`SectionKind::Junction`], because a junction rung stands longer than the ladder it closes and
/// so is the exception, never the measure. A run of nothing but junctions has only its own first
/// rung to go by.
fn steps_of(run: &SectionRun) -> i32 {
    for section in &run.sections {
        if section.kind != SectionKind::Junction {
            return section.segments();
        }
    }

    run.start_of(0).segments()
}

/// Whether one rung can stand at the end of a ladder drawn in `steps` intervals:
/// an ordinary section has to be drawn in exactly those, and a junction rung may stand LONGER - and
/// only longer - because the samples it carries past the near wall end are the arriving strip's
/// own, shared by identity, and the tail fan closes them without moving any of them.
fn rung_stands(section: &BevelSection, steps: i32) -> bool {
    if section.kind == SectionKind::Junction {
        section.segments() >= steps
    } else {
        section.segments() == steps
    }
}

/// Closes the ladder whose rung is a junction drawn in more samples
/// than the other: the samples past the near wall end are fanned onto the OTHER row's own WALL
/// end, so that the seam stays shut with nothing resampled and no point moved.
///
/// **Always the wall end, never the flat meet.** Both rows are anchored at their FLAT side - that
/// is the end the face rebuild's own boundary point stands at, so a ladder anchored anywhere else
/// hands the rebuild a rim that ends one sample short and leaves a hole against it. The extras
/// therefore sit at whichever END of the profile the WALL is, which
/// `JunctionSection::flat_on_the_left` decides: flat on the left puts them at the tail, flat on the
/// right at the head.
///
/// The winding follows the ladder's own loop, `start[k] -> start[k + 1] -> end[k + 1] -> end[k]`:
/// where the long row is the START the fan runs it FORWARD onto the apex, and where it is the END it
/// runs BACKWARD, which is the same way round that loop. Laying it the other way is refused by
/// [`try_lay`] as a triangle facing into the material, so the choice is not free.
#[allow(clippy::too_many_arguments)]
pub(super) fn try_fan_the_junction_extras(
    start: &BevelSection,
    end: &BevelSection,
    start_head: i32,
    end_head: i32,
    pool: &SectionPool,
    steps: i32,
    flipped: bool,
    faces: &mut Faces,
) -> Result<(), BevelAdvisory> {
    let long_is_start = start.segments() > steps;
    let (longer, other) = if long_is_start {
        (start, end)
    } else {
        (end, start)
    };

    if longer.segments() <= steps {
        return Ok(());
    }

    let head = if long_is_start { start_head } else { end_head };
    let apex_at = if head > 0 { 0 } else { other.segments() };
    let apex = other.profile[apex_at as usize];
    let from = if head > 0 { 0 } else { steps };
    let beyond = if head > 0 {
        head
    } else {
        longer.profile.len() as i32 - 1
    };

    for k in from..beyond {
        let (at, next) = (longer.profile[k as usize], longer.profile[(k + 1) as usize]);
        let triangle = if long_is_start {
            (at, next, apex)
        } else {
            (next, at, apex)
        };

        let outward = outward_at(longer, pool, k)
            + outward_at(longer, pool, k + 1)
            + outward_at(other, pool, apex_at);

        try_lay(triangle, pool, outward, flipped, longer.chain_vertex, faces)?;
    }

    Ok(())
}

/// How many of one station's profile samples stand at the HEAD of it rather
/// than the tail - the extras a junction rung carries past the near wall end, where that
/// junction's flat side is on the RIGHT and so its profile is written wall-end first. Zero
/// everywhere else, junction or not.
pub(super) fn head_extras_of(
    run: &SectionRun,
    station: i32,
    section: &BevelSection,
    steps: i32,
) -> i32 {
    if section.segments() <= steps {
        return 0;
    }

    match run.junctions.get(&station) {
        Some(junction) if !junction.flat_on_the_left => section.segments() - steps,
        _ => 0,
    }
}

/// The fold crossings the part's edge runs through between station `i` and the
/// next, where quad `k` is the one whose outer edge that is - which is quad 0 on the LEFT side and
/// the last quad on the RIGHT. `None` everywhere else.
///
/// Read by STATION through `SmoothSideCut::crossings_after_station`: the row between rungs `i` and
/// `i + 1` runs from station `i`'s LAST boundary point to the next station's first, which is not
/// the i'th join once a junction on this side has handed its three points.
pub fn crossings_on(
    seam: Option<&SectionSmoothSeam>,
    i: i32,
    k: i32,
    steps: i32,
) -> Option<&Vec<i32>> {
    let seam = seam?;

    if (if seam.is_left { 0 } else { steps - 1 }) != k {
        return None;
    }

    seam.cut.crossings_after_station(i)
}

/// Lays the last row of a smooth side as a fan over the boundary polyline - see
/// [`try_emit`].
#[allow(clippy::too_many_arguments)]
pub(super) fn try_fan(
    quad: Quad,
    crossings: &[i32],
    is_left: bool,
    outward: Vector3,
    flipped: bool,
    chain_vertex: i32,
    pool: &SectionPool,
    faces: &mut Faces,
) -> Result<(), BevelAdvisory> {
    let (q0, q1, q2, q3) = quad;

    // The quad's ring is Q0 Q1 Q2 Q3. On the LEFT side the boundary is its Q3 -> Q0 edge, which
    // runs AGAINST the chain, so the crossings are read backwards; on the RIGHT it is Q1 -> Q2,
    // with the chain. Either way the ring becomes firstApex, polyline..., secondApex.
    let mut polyline = Vec::with_capacity(crossings.len() + 2);
    let first_apex;
    let second_apex;

    if is_left {
        first_apex = q2;
        second_apex = q1;
        polyline.push(q3);
        polyline.extend(crossings.iter().rev());
        polyline.push(q0);
    } else {
        first_apex = q0;
        second_apex = q3;
        polyline.push(q1);
        polyline.extend_from_slice(crossings);
        polyline.push(q2);
    }

    // A corner-swallowed station's point on the swallowed side IS the gap end's own point, by
    // index, so the polyline can repeat it - and a zero length edge is no edge to fan over.
    // Collapse by INDEX.
    polyline.dedup();

    if polyline.len() < 2 {
        if polyline.is_empty() {
            return Ok(());
        }

        return try_lay(
            (second_apex, first_apex, polyline[0]),
            pool,
            outward,
            flipped,
            chain_vertex,
            faces,
        );
    }

    let segments = polyline.len() - 1;
    let split = split_nearest_the_middle(segments);

    for j in 0..split {
        try_lay(
            (first_apex, polyline[j], polyline[j + 1]),
            pool,
            outward,
            flipped,
            chain_vertex,
            faces,
        )?;
    }

    try_lay(
        (second_apex, first_apex, polyline[split]),
        pool,
        outward,
        flipped,
        chain_vertex,
        faces,
    )?;

    for j in split..segments {
        try_lay(
            (second_apex, polyline[j], polyline[j + 1]),
            pool,
            outward,
            flipped,
            chain_vertex,
            faces,
        )?;
    }

    Ok(())
}

/// The interior point of a polyline of `segments` segments that is
/// nearest its middle - the one the fan hands over from one apex to the other.
fn split_nearest_the_middle(segments: usize) -> usize {
    // Between 1 and segments - 1, with segments >= 1 here.
    1.max((segments - 1).min(segments / 2))
}

/// Which way round the whole run is laid (`true` for flipped), read from
/// its FIRST quad against the ball that swept it - once, so that every strip of the run agrees by
/// construction rather than by each triangle separately reaching the same conclusion. The first
/// quad is the first one the run actually lays, which on an absorbed run is the first GAP's rather
/// than the first station pair's.
fn try_winding_of_the_run(
    start: &BevelSection,
    end: &BevelSection,
    pool: &SectionPool,
) -> Result<bool, BevelAdvisory> {
    // Read at the rows' own first samples, offsets or none: the answer wanted is a SIGN, and the
    // first rung of a junction row stands on the same sweep as any other rung of it.
    let quad = quad_at(start, end, 0, 0, 0);

    // The quad's own normal, over all four corners, so that a quad whose diagonals read
    // differently still gives one answer.
    let normal = newell(pool, quad);
    let outward = outward_across(start, end, pool, 0, 0, 0);
    let reach = normal.length() * outward.length();

    if reach < 1e-24 {
        return Err(not_built(Some(start.chain_vertex)));
    }

    Ok(normal.dot(outward) < 0.0)
}

/// The way OUT of the material across one rung of the ladder - the sum of the
/// four corners' own outward directions, each the way from the centre that corner was swung about
/// to the corner.
pub(super) fn outward_across(
    start: &BevelSection,
    end: &BevelSection,
    pool: &SectionPool,
    k: i32,
    start_head: i32,
    end_head: i32,
) -> Vector3 {
    outward_at(start, pool, k + start_head)
        + outward_at(start, pool, k + 1 + start_head)
        + outward_at(end, pool, k + 1 + end_head)
        + outward_at(end, pool, k + end_head)
}

/// Where profile sample `k` stands relative to the ball's centre AT that sample -
/// the centre segment read at the sample's own parameter, which is what the section's own
/// construction guarantees the sample is a radius from.
fn outward_at(section: &BevelSection, pool: &SectionPool, k: i32) -> Vector3 {
    let along = if section.segments() < 1 {
        0.0
    } else {
        k as f64 / section.segments() as f64
    };
    let centre = section.centre_from + ((section.centre_to - section.centre_from) * along);

    pool.position_of(section.profile[k as usize]) - centre
}

/// Rung `k` of the ladder, each row read from its own FLAT-side anchor: a junction row
/// drawn with the flat side on the RIGHT carries its extra samples at the HEAD of its profile (see
/// [`head_extras_of`]), and `start_head` / `end_head` step past them so that rung k still means the
/// same parameter on both rows. Both are zero on every ladder without a junction.
pub(super) fn quad_at(
    start: &BevelSection,
    end: &BevelSection,
    k: i32,
    start_head: i32,
    end_head: i32,
) -> Quad {
    (
        start.profile[(k + start_head) as usize],
        start.profile[(k + 1 + start_head) as usize],
        end.profile[(k + 1 + end_head) as usize],
        end.profile[(k + end_head) as usize],
    )
}

/// The quad's normal by Newell's rule, which needs no choice of diagonal.
fn newell(pool: &SectionPool, quad: Quad) -> Vector3 {
    let corners = [
        pool.position_of(quad.0),
        pool.position_of(quad.1),
        pool.position_of(quad.2),
        pool.position_of(quad.3),
    ];

    let mut normal = Vector3::ZERO;

    for i in 0..corners.len() {
        let from = corners[i];
        let to = corners[(i + 1) % corners.len()];

        normal += Vector3::new(
            (from.y - to.y) * (from.z + to.z),
            (from.z - to.z) * (from.x + to.x),
            (from.x - to.x) * (from.y + to.y),
        );
    }

    normal
}

/// Whether the triangle `zero, one, two` (positions) is degenerate by
/// [`SMALLEST_RELATIVE_CROSS`], returning its cross product where it is not.
pub(super) fn cross_unless_degenerate(
    pool: &SectionPool,
    triangle: (i32, i32, i32),
) -> Option<Vector3> {
    let zero = pool.position_of(triangle.0);
    let one = pool.position_of(triangle.1) - zero;
    let two = pool.position_of(triangle.2) - zero;
    let cross = one.cross(two);
    let longest = ieee_math::max_f64(
        ieee_math::max_f64(one.length_squared(), two.length_squared()),
        (two - one).length_squared(),
    );

    if cross.length() <= SMALLEST_RELATIVE_CROSS * longest {
        None
    } else {
        Some(cross)
    }
}

/// Appends one triangle the run's way round, having checked that it has an area at all
/// and that it agrees with the run's winding.
pub(super) fn try_lay(
    triangle: (i32, i32, i32),
    pool: &SectionPool,
    outward: Vector3,
    flipped: bool,
    chain_vertex: i32,
    faces: &mut Faces,
) -> Result<(), BevelAdvisory> {
    let laid = if flipped {
        (triangle.0, triangle.2, triangle.1)
    } else {
        triangle
    };

    let Some(cross) = cross_unless_degenerate(pool, laid) else {
        return Err(not_built(Some(chain_vertex)));
    };

    if cross.dot(outward) <= 0.0 {
        // The run was wound from its first quad; a later quad facing the other way is a chain this
        // construction cannot loft (a concave stretch, or a section whose ends have crossed), and
        // flipping this one triangle to hide that is how a fold gets built instead of reported.
        return Err(not_built(Some(chain_vertex)));
    }

    faces.push(laid);

    Ok(())
}
