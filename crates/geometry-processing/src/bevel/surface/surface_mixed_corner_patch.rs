//! The patch that closes a mixed corner between the cut's end ring and the bead's
//! (`Bevel/Surface/SurfaceMixedCornerPatch.cs`, a static class: free functions here). The ring
//! readers, the shared face's chord sagitta, the grid's last column and the patch's closure check
//! ([`nothing_covers_nothing`]) land here first; the patch's own construction (`TryRing`,
//! `TryCutTheRing`) lands with a later package.

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

        widest = f64::max(
            widest,
            walk.facets[facet].crossed() / 2.0 * (turned / 4.0).tan(),
        );
    }

    f64::max(pool.near_enough(), widest)
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
