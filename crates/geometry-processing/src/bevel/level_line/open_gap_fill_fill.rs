//! The open fill itself. Its own file because `open_gap_fill.rs` holds the rest of the open fill
//! and would pass the 800-line limit. It reads both runs as rails
//! (`open_gap_fill::try_rail`), gives each rail the other's stations
//! (`open_gap_fill_stations::with_stations_of`), lays the rows (`open_gap_fill::arcs`) and the
//! cells between them, closes the ends (`corner_cap_zip_end::zip_end`,
//! `end_plane_trim_carry::carry`), clips the strip on the growing neighbour or the part's hull
//! (`strip_clip_neighbour`, `hull_clip`) and checks that the fill closes (`fill_closure`).

use std::cell::RefCell;
use std::rc::Rc;

use super::open_gap_fill::{arcs, backwards, is_straight_row, set_ends_site, try_rail};
use super::open_gap_fill_cut_section::{flat_run_out, lay_beside_a_cut_row};
use super::open_gap_fill_stations::{rows_where_the_round_needs_them, spokes, with_stations_of};
use super::open_gap_fill_sunk_arc::{area2, past_an_end, sunk_arc_point};
use super::{
    corner_cap_flat_ends, corner_cap_zip_end, corner_fill, end_plane_trim_carry, fill_closure,
    gap_fill, hull_clip, strip_clip, strip_clip_neighbour, BandProfile, ChainField, GapFillRefusal,
    GapFillResult, GapLoop, LevelLineMesh, LevelLineMeshError, LevelLineWork,
};

/// What the fill is asked beyond the two runs, with the usual values in [`Default`].
#[derive(Debug, Clone, Copy, Default)]
pub struct OpenFillEnds<'a> {
    /// The end runs on the end faces at end 0 and end 1; `None` for a fade.
    pub end_runs: Option<&'a [Vec<i32>]>,
    /// Rows laid on the end ray.
    pub rows_on_the_ray: bool,
    /// The ends are clipped on the growing neighbour rather than zipped.
    pub clip_ends: bool,
    /// Which ends are HULL ends (`None` reading as both false).
    pub hull_ends: Option<[bool; 2]>,
    /// Which ends are carried on to their section in the end plane.
    pub carried_ends: Option<[bool; 2]>,
    /// The fill is laid where the flow builds.
    pub flowed: bool,
}

/// Takes back every triangle laid from `start`, off the mesh and off the laid list: what a
/// refusal does before it hands the mesh back.
fn take_back(mesh: &mut LevelLineMesh, laid: &mut Vec<i32>, start: usize) {
    for &triangle in &laid[start..] {
        mesh.remove_triangle(triangle);
    }

    laid.truncate(start);
}

/// Lays the fade's strip between the two open runs. The runs are walked with
/// the kept surface on the left of each, so together they traverse the whole gap boundary one way
/// round; the fill traverses every one of those edges backwards, which is what orients each
/// triangle it lays.
#[allow(clippy::too_many_arguments)]
pub fn fill(
    mesh_handle: &Rc<RefCell<LevelLineMesh>>,
    chain: &ChainField,
    left_loop: &GapLoop,
    right_loop: &GapLoop,
    profile: BandProfile,
    fillet_segments: i32,
    laid: &mut Vec<i32>,
    ends: OpenFillEnds,
) -> Result<GapFillResult, LevelLineMeshError> {
    let refusal = |refusal: GapFillRefusal, offending_vertex: i32| GapFillResult {
        refusal,
        offending_vertex,
        mesh: Some(mesh_handle.clone()),
        ..GapFillResult::default()
    };
    let mut mesh_guard = mesh_handle.borrow_mut();
    let mesh = &mut *mesh_guard;
    let start = laid.len();
    let hull_ends = ends.hull_ends.unwrap_or([false, false]);

    // The right run is read backwards so both rails travel from the meeting point at end 0 to the
    // one at end 1; the runs share those two vertices, so the rails begin and end on the very
    // same points.
    let slack = gap_fill::monotone_slack(chain);
    let rails = try_rail(
        mesh,
        chain,
        &left_loop.vertices,
        &left_loop.parameters,
        false,
        slack,
    )
    .and_then(|left| {
        try_rail(
            mesh,
            chain,
            &right_loop.vertices,
            &right_loop.parameters,
            true,
            slack,
        )
        .map(|right| (left, right))
    });
    let (mut left, mut right) = match rails {
        Ok(rails) => rails,
        Err(offending) => {
            drop(mesh_guard);
            return Ok(refusal(GapFillRefusal::RailIsNotMonotone, offending));
        }
    };

    // Each rail carries a station wherever the OTHER rail has a vertex, so every spoke joins two
    // points at the same chain parameter and every row stands square to the chain. Without it a
    // spoke paired a rail vertex with the other rail's nearest vertex - a wall's diagonal crossed
    // at one height on one side and another on the other - and the skewed rows cut into a concave
    // bead by a share that depended on where the diagonals fell relative to the size (the
    // ladder's L: 0.204 r^2 per unit length at size 3 against the exact 0.2146).
    // Only where both ends are closed by end-face caps: a fade's rails curl round its meeting
    // points, and its zipper is left exactly as it was.
    let square = ends.end_runs.is_some();
    if square {
        left = with_stations_of(mesh, chain, &left, &right, chain.length)?;
        right = with_stations_of(mesh, chain, &right, &left, chain.length)?;
    }

    let mut spokes = spokes(&left, &right, chain.length, square)?;
    if square && chain.has_smooth_joints() {
        spokes = rows_where_the_round_needs_them(mesh, chain, &left, &right, &spokes);
    }

    let segments = if profile == BandProfile::Chamfer {
        1
    } else {
        fillet_segments
    };
    let mut added_vertices = Vec::new();
    let arcs = arcs(
        mesh,
        chain,
        &spokes,
        &left,
        &right,
        segments,
        &mut added_vertices,
        ends.rows_on_the_ray,
        ends.end_runs.is_none(),
    )?;
    let past_an_end: Vec<bool> = spokes
        .iter()
        .map(|&spoke| past_an_end(chain, &left, &right, spoke))
        .collect();
    let sunk = sunk_arc_point(mesh, left_loop, right_loop, &arcs, &past_an_end);
    if sunk >= 0 {
        drop(mesh_guard);
        return Ok(refusal(GapFillRefusal::ArcSinksIntoTheSolid, sunk));
    }

    // Rows cut by the face a run-out side ran onto start on that face (Arcs), not at A.
    let on_the_plane: Vec<bool> = spokes
        .iter()
        .enumerate()
        .map(|(k, spoke)| arcs[k][0] != left.vertices[spoke.0 as usize])
        .collect();
    let flat_run_out = flat_run_out(mesh, chain, &spokes, &arcs, &left, &on_the_plane);
    let mut cell = 0;
    while cell + 1 < spokes.len() {
        LevelLineWork::charge(1)?;
        let before = &arcs[cell];
        let after = &arcs[cell + 1];
        let (this, next) = (spokes[cell], spokes[cell + 1]);
        if (on_the_plane[cell] || on_the_plane[cell + 1]) && next.0 == this.0 {
            lay_beside_a_cut_row(
                mesh,
                before,
                after,
                on_the_plane[cell],
                on_the_plane[cell + 1],
                left.vertices[this.0 as usize],
                flat_run_out.is_some(),
                laid,
            )?;
            cell += 1;
            continue;
        }

        if let Some(region) = &flat_run_out {
            if cell as i32 > region.last_cut {
                // Two in-plane rows beyond the cut ones: the run-out's flat region below covers
                // the cell.
                cell += 1;
                continue;
            }
        }

        // The rail between this row and the next on each side: more than its two ends where a
        // rail vertex was given no row of its own (RowsWhereTheRoundNeedsThem).
        let lefts: Vec<i32> = left.vertices[this.0 as usize..=next.0 as usize].to_vec();
        let rights: Vec<i32> = right.vertices[this.1 as usize..=next.1 as usize].to_vec();
        if next.0 != this.0 && next.1 != this.1 {
            // Both rails advanced: a row of quads, laid as the left-advancing cell lays its first
            // rows. A rail vertex that was given no row of its own (RowsWhereTheRoundNeedsThem) is
            // taken into the cell's first or last triangle as a fan over the rail between the two
            // rows.
            let mut r = 0;
            while r + 1 < after.len() {
                if r == 0 && lefts.len() > 2 {
                    for k in 0..lefts.len() - 1 {
                        corner_fill::lay(mesh, lefts[k + 1], lefts[k], before[1], laid)?;
                    }
                } else {
                    corner_fill::lay(mesh, after[r], before[r], before[r + 1], laid)?;
                }

                if r + 2 == after.len() && rights.len() > 2 {
                    for k in 0..rights.len() - 1 {
                        corner_fill::lay(mesh, after[r], rights[k], rights[k + 1], laid)?;
                    }
                } else {
                    corner_fill::lay(mesh, after[r], before[r + 1], after[r + 1], laid)?;
                }

                r += 1;
            }
        } else if is_straight_row(before) && is_straight_row(after) {
            // Two straight rows onto one meeting point P (see Arcs): read as the strip reads them
            // below, both run [far end, P, ..., P], and the cell is the one flat triangle the
            // strip's base lays; the strip's own rows beyond it would be P three times over.
            let left_advanced = next.0 != this.0;
            let u = if left_advanced {
                after.clone()
            } else {
                backwards(before)
            };
            let v = if left_advanced {
                before.clone()
            } else {
                backwards(after)
            };
            corner_fill::lay(mesh, u[0], v[0], u[u.len() - 1], laid)?;
        } else if next.0 != this.0 {
            // The left rail advanced, so the cell's base is that rail's own edge, which the fill
            // has to traverse against the run's walk: from the NEW left vertex back to the old one.
            gap_fill::strip(mesh, after, before, laid, Some(&lefts))?;
        } else {
            // The right rail advanced. Its walk runs the other way round the gap, so here the fill
            // runs WITH the rail - and the arcs are read from their right ends, where the base
            // edge is.
            let reversed: Vec<i32> = rights.iter().rev().copied().collect();
            gap_fill::strip(
                mesh,
                &backwards(before),
                &backwards(after),
                laid,
                Some(&reversed),
            )?;
        }

        cell += 1;
    }

    if let Some(region) = &flat_run_out {
        for t in &region.triangles {
            corner_fill::lay(
                mesh,
                region.r#loop[t.a as usize],
                region.r#loop[t.b as usize],
                region.r#loop[t.c as usize],
                laid,
            )?;
        }
    }

    // Read off the rails, which carry the stations split into the runs' edges: the left rail runs
    // as its run was walked, the right rail against it.
    let mut kept: Vec<(i32, i32)> = Vec::new();
    if square {
        for i in 0..left.vertices.len().saturating_sub(1) {
            kept.push((left.vertices[i], left.vertices[i + 1]));
        }

        for i in 0..right.vertices.len().saturating_sub(1) {
            kept.push((right.vertices[i + 1], right.vertices[i]));
        }
    } else {
        for run in [left_loop, right_loop] {
            for i in 0..run.vertices.len().saturating_sub(1) {
                kept.push((run.vertices[i], run.vertices[i + 1]));
            }
        }
    }

    if let Some(end_runs) = ends.end_runs.filter(|_| !ends.clip_ends) {
        // Each END is a hole between the end run on the end face and the strip's last row; the
        // row at end 0 is laid right to left and the one at end 1 left to right, so each is
        // handed over in the direction the cap must traverse it. Laid BEFORE the self-check, which
        // then sees one closed fill.
        let points = chain.points();
        let end_rows = [
            (backwards(&arcs[0]), &end_runs[0], points[0]),
            (
                arcs[arcs.len() - 1].clone(),
                &end_runs[1],
                points[points.len() - 1],
            ),
        ];
        for (e, (row, run, vertex)) in end_rows.iter().enumerate() {
            // A HULL end is not zipped: its rows run whole past the part and the hull clips them
            // below.
            if hull_ends[e] {
                continue;
            }

            let (first, last) = (row[0], row[row.len() - 1]);

            // A row laid wholly in the cutting plane of a run-out side (CutSectionArc) that ends
            // on the end run's own vertices IS that run: there is no hole.
            if corner_cap_flat_ends::use_flat_ends()
                && run.len() == 2
                && first == run[0]
                && last == run[1]
                && row[1..row.len() - 1]
                    .iter()
                    .all(|&v| v == first || v == last)
            {
                kept.push((run[0], run[1]));
                continue;
            }

            // A FADE's nose (a mixed pick's end where the crease carries on - see
            // EndFaceFill.TryEndFaces): both rails end on the one vertex, the row there has
            // collapsed onto it, and there is no hole.
            if run.len() == 1 && row.iter().all(|&v| v == run[0]) {
                continue;
            }

            // A trimmed end with flat ends on is CARRIED on to its section in the end plane
            // (EndPlaneTrim.Carry).
            if ends.carried_ends.is_some_and(|carried| carried[e])
                && end_plane_trim_carry::carry(mesh, chain, row, run, e == 1, profile, laid)?
            {
                for i in 0..run.len().saturating_sub(1) {
                    kept.push((run[i], run[i + 1]));
                }

                continue;
            }

            let (capped, off) = corner_cap_zip_end::zip_end(
                mesh,
                row,
                run,
                *vertex,
                laid,
                &mut added_vertices,
                ends.flowed,
            )?;
            if capped != GapFillRefusal::None {
                take_back(mesh, laid, start);
                drop(mesh_guard);
                return Ok(refusal(capped, off));
            }

            for i in 0..run.len().saturating_sub(1) {
                kept.push((run[i], run[i + 1]));
            }
        }
    }

    // Rows laid across a virtual rail are clipped on the growing neighbour's planes and capped
    // (rung 2). A hull end's open boundary is the part's own edges round the mouth, not a growing
    // neighbour: the neighbour clip is kept off it (both kinds at once is not built yet), and the
    // hull clips and caps it.
    let hull = hull_ends[0] || hull_ends[1];
    if hull && (ends.clip_ends || !hull_clip::try_clip_and_cap(mesh, laid, start, &mut kept)?) {
        take_back(mesh, laid, start);
        set_ends_site(Some("hull clip".to_string()));
        drop(mesh_guard);
        return Ok(refusal(GapFillRefusal::OpenChainEnds, -1));
    }

    if !hull && !strip_clip_neighbour::try_clip_and_cap(mesh, laid, start, &mut kept)? {
        take_back(mesh, laid, start);
        set_ends_site(Some(format!(
            "strip clip: {}",
            strip_clip::last_outcome().unwrap_or_default()
        )));
        drop(mesh_guard);
        return Ok(refusal(GapFillRefusal::GrowingNeighbourClipRefused, -1));
    }

    let added_triangles: Vec<i32> = laid[start..].to_vec();
    let open = fill_closure::first_open_edge_of(mesh, &kept, &added_triangles);
    if open.0 >= 0 {
        take_back(mesh, laid, start);
        drop(mesh_guard);
        return Ok(GapFillResult {
            open_edge: open,
            ..refusal(GapFillRefusal::FillWouldNotClose, -1)
        });
    }

    // A fill that closes but lays a triangle of no area is not a part to hand back: the 15mm top
    // edge's oblique corner end at 0.1 of its room laid one (CurlingSideRunConcavityTests).
    // Refused by name.
    let flat = added_triangles
        .iter()
        .copied()
        .find(|&t| area2(mesh, mesh.triangles()[t as usize]) == 0.0);
    if let Some(flat) = flat {
        let flat_corner = mesh.triangles()[flat as usize].a;
        take_back(mesh, laid, start);
        drop(mesh_guard);
        return Ok(refusal(GapFillRefusal::FillLaysAFlatTriangle, flat_corner));
    }

    drop(mesh_guard);
    Ok(GapFillResult {
        refusal: GapFillRefusal::None,
        mesh: Some(mesh_handle.clone()),
        added_triangles,
        added_vertices,
        ..GapFillResult::default()
    })
}
