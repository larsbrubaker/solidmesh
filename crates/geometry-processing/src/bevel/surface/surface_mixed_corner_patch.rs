//! The patch that closes a mixed corner between the cut's end ring and the bead's
//! (`Bevel/Surface/SurfaceMixedCornerPatch.cs`, a static class: free functions here). The ring
//! readers, the shared face's chord sagitta, the grid's last column and the patch's closure check
//! ([`nothing_covers_nothing`]), and each strip's end ring ([`try_ring`], [`try_cut_the_ring`]);
//! the patch's own construction lands with a later package.

use vector_math::{cs_math, Vector3};

use super::surface_mitre_joint_step::SideSeams;
use super::surface_strip_ends;
use super::{SurfaceBandEnd, SurfaceMixedCorner, SurfaceStrip, SurfaceVertexPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// How far off its own circle the shared face's chording already stands - the sagitta of one of
/// its chords, `(chord / 2) tan(turn / 4)` - which is what a ring standing on those chords may be
/// off the ideal torus by without being wrong about anything the mesh knows.
///
/// Read off the section the bead's own roll walked, so it is that wall's chording and not a
/// constant. A roll that never happened has no section and no sagitta, and the part's own ruler
/// is what is left.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(x > 0)`, which refuses a NaN as well.
pub fn the_shared_faces_own_sagitta(
    corner: &super::SurfaceMixedCorner,
    pool: &super::SurfaceVertexPool,
) -> f64 {
    let walk = match corner.rolled_section() {
        Some(walk) if walk.facets.len() >= 2 => walk,
        _ => return pool.near_enough(),
    };

    let mut widest = 0.0;

    for facet in 1..walk.facets.len() {
        let turned = walk.facets[facet]
            .normal
            .dot(walk.facets[facet - 1].normal)
            .clamp(-1.0, 1.0)
            .acos();

        if !(turned > 0.0) {
            continue;
        }

        widest = cs_math::max_f64(
            widest,
            walk.facets[facet].crossed() / 2.0 * (turned / 4.0).tan(),
        );
    }

    cs_math::max_f64(pool.near_enough(), widest)
}

/// The cut's ring read from the cap face round to the shared face, whichever way its own rails
/// run. Rail 0 lies on the LEFT of the walk, so which end is which is the plan's own left and
/// right.
pub fn from_the_cap_face(corner: &super::SurfaceMixedCorner, ring: &[i32]) -> Vec<i32> {
    if corner.cut.left_region_key() == corner.shared_region_key {
        reversed(ring)
    } else {
        ring.to_vec()
    }
}

/// The bead's ring read from the handover - its rail on the shared face - round to the wall.
pub fn from_the_handover(corner: &super::SurfaceMixedCorner, ring: &[i32]) -> Vec<i32> {
    if corner.bead.left_region_key() == corner.shared_region_key {
        ring.to_vec()
    } else {
        reversed(ring)
    }
}

/// `ring` back to front, as a new array.
fn reversed(ring: &[i32]) -> Vec<i32> {
    ring.iter().rev().copied().collect()
}

/// The last column of `grid`: each row's last vertex.
pub fn last_column_of(grid: &[Vec<i32>]) -> Vec<i32> {
    grid.iter().map(|row| row[row.len() - 1]).collect()
}

/// Whether `faces` cover nothing twice: no face repeated (in any winding), no edge used by more
/// than two faces, and every edge two faces share walked once each way - two faces walking it
/// the same way are the same side of one surface twice over. Refused (as a result that is not
/// closed) at the first vertex that breaks it, in the order the faces name their edges.
pub fn nothing_covers_nothing(
    pool: &super::SurfaceVertexPool,
    faces: &[(i32, i32, i32)],
) -> Result<(), crate::bevel::BevelAdvisory> {
    use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};
    use indexmap::IndexMap;
    use std::collections::{HashMap, HashSet};

    let not_closed = |vertex: i32| {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceResultNotClosed,
            vec![pool.position_of(vertex).into()],
        )
    };

    // Iterated below, so in insertion order as C#'s Dictionary is; the other two are looked up
    // only.
    let mut uses: IndexMap<(i32, i32), i32> = IndexMap::with_capacity(faces.len() * 3);
    let mut directed: HashMap<(i32, i32), i32> = HashMap::with_capacity(faces.len() * 3);
    let mut standing: HashSet<(i32, i32, i32)> = HashSet::new();

    for &(v0, v1, v2) in faces {
        let mut corners = [v0, v1, v2];

        corners.sort_unstable();

        if !standing.insert((corners[0], corners[1], corners[2])) {
            return Err(not_closed(v0));
        }

        for side in [(v0, v1), (v1, v2), (v2, v0)] {
            let key = (side.0.min(side.1), side.0.max(side.1));

            let already = uses.get(&key).copied().unwrap_or(0);
            uses.insert(key, already + 1);

            *directed.entry(side).or_insert(0) += 1;

            if already + 1 > 2 {
                return Err(not_closed(key.0));
            }
        }
    }

    for (&edge, &count) in &uses {
        if count != 2 {
            continue;
        }

        if directed.get(&edge).copied().unwrap_or(0) != 1 {
            // Both faces walk it the same way, so they are the same side of one surface twice
            // over.
            return Err(not_closed(edge.0));
        }
    }

    Ok(())
}

/// C# `TryRing`: one strip's end ring at the corner, as vertices - every rail crossed with the
/// plane that strip stops on ([`try_cut_the_ring`]), with the two outer rails taken from the
/// walkback's own tangent runs (`seams[edge_index]`, the run's last vertex where the strip
/// `ends_here`, its first otherwise) rather than made again. As `(ring, meets_at)`.
///
/// The rail on the face the two edges share has to land on the tangent point - except where the
/// CUT hands over on a mitre ring, which stands short of the tangency by design and is checked
/// against its own seam alone. `cut_on` is as [`try_cut_the_ring`]'s.
///
/// A tangent run that is missing or empty, where C# would throw reading it, is refused as the
/// seam not meeting.
#[allow(clippy::too_many_arguments)]
pub fn try_ring(
    pool: &mut SurfaceVertexPool,
    corner: &SurfaceMixedCorner,
    strip: &SurfaceStrip,
    is_the_cut: bool,
    seams: &[SideSeams],
    edge_index: usize,
    ends_here: bool,
    cut_on: Option<SurfaceBandEnd>,
) -> Result<(Vec<i32>, Vec<f64>), BevelAdvisory> {
    let (meets_at, at) = try_cut_the_ring(corner, strip, is_the_cut, cut_on)?;
    let did_not_meet = |at: Vector3| {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
            vec![at.into()],
        )
    };

    let plan = if is_the_cut {
        &corner.cut
    } else {
        &corner.bead
    };
    let last = at.len() - 1;
    let shared_rail = if plan.left_region_key() == corner.shared_region_key {
        0
    } else {
        last
    };

    let mut ring = vec![0; at.len()];

    for rail in 0..at.len() {
        if rail != 0 && rail != last {
            ring[rail] = pool.add(at[rail]);
            continue;
        }

        // The rail on the face the two edges share has to land on the tangent point, which is the
        // corner the walkback left that face with; the other lands where its own band stopped.
        if rail == shared_rail
            && !(is_the_cut && corner.hands_over_at_a_mitre)
            && (at[rail] - corner.tangent_point).length() > surface_strip_ends::meets_the_seam(pool)
        {
            return Err(did_not_meet(at[rail]));
        }

        let run = seams
            .get(edge_index)
            .and_then(|sides| sides[if rail == 0 { 0 } else { 1 }].as_ref());
        let vertex = match run.and_then(|run| if ends_here { run.last() } else { run.first() }) {
            Some(&(vertex, _)) => vertex,
            None => return Err(did_not_meet(at[rail])),
        };

        if (pool.position_of(vertex) - at[rail]).length() > surface_strip_ends::meets_the_seam(pool)
        {
            return Err(did_not_meet(at[rail]));
        }

        ring[rail] = vertex;
    }

    Ok((ring, meets_at))
}

/// C# `TryCutTheRing`: one strip's end ring at the corner - where every rail of it crosses the
/// plane that strip stops on, the handover plane for the cut (`is_the_cut`), the plane the bead's
/// own ring stands on for the bead - as `(meets_at, at)`: where along its own edge each rail meets
/// the plane, and where in space. A rail running along the plane is refused by name.
///
/// Ordinarily the ring is cut at the SAME plane the band it ends was cut at. A TWO-crease junction
/// parts them: its one cut is bounded on neither face, and the ring it really ends on LEANS
/// (63.43 degrees across the cut on the nose fixture) - so that plane is the caller's to name, as
/// `cut_on`; `None` for every corner whose ring and band stop together.
pub fn try_cut_the_ring(
    corner: &SurfaceMixedCorner,
    strip: &SurfaceStrip,
    is_the_cut: bool,
    cut_on: Option<SurfaceBandEnd>,
) -> Result<(Vec<f64>, Vec<Vector3>), BevelAdvisory> {
    let stops_at = match cut_on {
        Some(plane) => plane,
        None if is_the_cut => corner.cut_stops_at,
        None => corner.bead_ring_stands_on(),
    };

    let rails = strip.rail_count().max(0) as usize;
    let mut meets = Vec::with_capacity(rails);
    let mut points = Vec::with_capacity(rails);

    for rail in 0..rails as i32 {
        let Some(param) = strip.try_cross(rail, stops_at.point, stops_at.inward) else {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCornerNotSupported,
                vec![corner.corner.into()],
            ));
        };

        meets.push(param);
        points.push(strip.point_on(rail, param));
    }

    Ok((meets, points))
}

/// C# `SurfaceMixedCornerPatch.Emit`: the patch's grid of pool vertices emitted as quads, each
/// facing the corner's summed outward direction.
pub fn emit(
    pool: &mut SurfaceVertexPool,
    faces: &mut Vec<(i32, i32, i32)>,
    corner: &SurfaceMixedCorner,
    grid: &[Vec<i32>],
) {
    let outward = corner.outward_sum();

    for p in 0..grid.len().saturating_sub(1) {
        for j in 0..grid[p].len().saturating_sub(1) {
            super::surface_fan::emit_facing(
                &[
                    grid[p][j],
                    grid[p + 1][j],
                    grid[p + 1][j + 1],
                    grid[p][j + 1],
                ],
                pool,
                faces,
                outward,
            );
        }
    }
}

/// C# `SurfaceMixedCornerPatch.TryTurnTheRingRound`: the cut's ring `profile` turned round the
/// corner's axis through `turns` stations - row `p` of the grid is rail `p` at each turn, the
/// first column the ring itself and the last rail (on the shared face) the turns already laid
/// there, which the turned rail has to meet. Hands back the grid and the rows read off the ring.
pub fn try_turn_the_ring_round(
    pool: &mut SurfaceVertexPool,
    corner: &SurfaceMixedCorner,
    profile: &[i32],
    turns: &[i32],
) -> Result<(Vec<Vec<i32>>, super::SurfaceMixedCornerRows), BevelAdvisory> {
    if turns.len() < 2 || profile.len() < 2 {
        // One rail is no ring and no turn is no patch.
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![corner.corner.into()],
        ));
    }

    let ring: Vec<Vector3> = profile.iter().map(|&p| pool.position_of(p)).collect();
    let rows = super::SurfaceMixedCornerRows::try_read_corner(corner, &ring)?;

    let step = corner.turn_radians() / (turns.len() - 1) as f64;
    let mut grid = Vec::with_capacity(profile.len());
    for p in 0..profile.len() {
        let mut row = Vec::with_capacity(turns.len());
        let on_the_shared_face = p == profile.len() - 1;
        for (j, &turn) in turns.iter().enumerate() {
            let at = rows.at(p, j as f64 * step);
            if j == 0 {
                row.push(profile[p]);
                continue;
            }

            if on_the_shared_face {
                if (pool.position_of(turns[j]) - at).length()
                    > surface_strip_ends::meets_the_seam(pool)
                {
                    return Err(BevelAdvisory::new(
                        BevelAdvisoryCode::SurfaceStripSeamDidNotMeet,
                        vec![at.into()],
                    ));
                }

                row.push(turn);
                continue;
            }

            row.push(pool.add(at));
        }
        grid.push(row);
    }

    Ok((grid, rows))
}
