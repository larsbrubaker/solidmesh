//! How the band flows over a chain's end face. Ports `Bevel/LevelLine/EndFaceFlow.cs` (a C# static
//! class): so far its constants, the facet normal ([`normal_of`]), the side-turn test
//! ([`turns_along_a_side`]), the end-face bend ([`bends`]) and whether the end region's flood
//! crosses a soft bend ([`crosses_a_soft_bend`]) and a flat patch of the end region
//! ([`flat_patch`]), and the flow over a narrow end face ([`flow_over_a_narrow_end_face`]) and
//! the round-run reading ([`rounds_from_the_end`]).

use vector_math::Vector3;

use super::triangle_edges::{self, StandingMap};
use std::cell::RefCell;
use std::collections::{HashSet, VecDeque};

use indexmap::IndexSet;

use super::band_side_tagging;
use super::{ChainField, LevelLineWork, WorkError};

/// How many FULL facets past the end vertex's own one, each between two soft joins, make a round.
pub const ROUND_RUN_JOINS: i32 = 3;

/// How far the round's facet widths and join turns may differ, largest to smallest: loose for a
/// round tessellated a little unevenly, tight against a flat end face ahead of a rounding.
pub const ROUND_RUN_SPREAD: f64 = 1.25;

/// How far two triangles across an edge may turn and still be one END FACE. Far above the noise a
/// flat face picks up in an exported file (float coordinates put its facets thousandths of a
/// degree apart) and under the facet turn of any rounding a part is tessellated with, so a planar
/// face, however it is triangulated, is one end region and the first facet of a rounding is not.
pub const END_FACE_BEND_DEGREES: f64 = 0.5;

/// The joins of one round turn about one axis (the cylinder's): within about 10 degrees, as a
/// cosine (C# `static readonly RoundRunAxisCosine`).
pub fn round_run_axis_cosine() -> f64 {
    (10.0 * std::f64::consts::PI / 180.0).cos()
}

/// [`END_FACE_BEND_DEGREES`] as a cosine (C# `static readonly EndFaceBendCosine`).
fn end_face_bend_cosine() -> f64 {
    (END_FACE_BEND_DEGREES * std::f64::consts::PI / 180.0).cos()
}

/// True when edge `a b` stands on nothing, or one of its triangles faces one of `side_normals`
/// within 45 degrees (either way round when `either_way`).
pub fn turns_along_a_side(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    a: i32,
    b: i32,
    side_normals: &[Vector3],
    either_way: bool,
) -> bool {
    let Some(on) = standing.get(&triangle_edges::key(a, b)) else {
        return true;
    };

    on.iter().any(|&triangle| {
        side_normals.iter().any(|&side| {
            let facing = normal_of(positions, indices[triangle as usize]).dot(side);
            (if either_way { facing.abs() } else { facing }) > 0.5f64.sqrt()
        })
    })
}

/// The unit normal of an input triangle.
pub fn normal_of(positions: &[Vector3], t: (i32, i32, i32)) -> Vector3 {
    (positions[t.1 as usize] - positions[t.0 as usize])
        .cross(positions[t.2 as usize] - positions[t.0 as usize])
        .get_normal()
}

/// True when the two triangles standing on the edge turn by more than
/// [`END_FACE_BEND_DEGREES`] - see the end floods - or the edge does not stand on exactly two.
pub fn bends(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    a: i32,
    b: i32,
) -> bool {
    let on = match standing.get(&triangle_edges::key(a, b)) {
        Some(on) if on.len() == 2 => on,
        _ => return true,
    };

    let normal_of = |triangle: i32| {
        let t = indices[triangle as usize];
        (positions[t.1 as usize] - positions[t.0 as usize])
            .cross(positions[t.2 as usize] - positions[t.0 as usize])
    };

    let first = normal_of(on[0]);
    let second = normal_of(on[1]);
    let lengths = first.length() * second.length();
    // A NaN length reads as no bend, as C#'s `!(lengths > 0)` does.
    if lengths.is_nan() || lengths <= 0.0 {
        return false;
    }

    first.dot(second) < end_face_bend_cosine() * lengths
}

/// C# `EndFaceFlow.CrossesASoftBend`: whether an end region's flood carries on across the soft bend
/// `a`-`b`. A round from the end on carries on while the band at this size reaches it (to within
/// `margin`, the model-scale join margin) - or, size-free, until it comes round to face away from or
/// with a side; past the end it carries on only while the band reaches it, or size-free until it
/// faces WITH a side. `end_face_reach` `None` is the size-free reading.
#[allow(clippy::too_many_arguments)]
pub fn crosses_a_soft_bend(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    a: i32,
    b: i32,
    past_the_end: bool,
    rounds_from_the_end: bool,
    end_face_reach: Option<&ChainField>,
    margin: f64,
    side_normals: &[Vector3],
) -> Result<bool, WorkError> {
    let (pa, pb) = (positions[a as usize], positions[b as usize]);
    if rounds_from_the_end {
        return Ok(match end_face_reach {
            Some(reach) => reach.min_along_segment(pa, pb)?.value < margin,
            None => !turns_along_a_side(positions, indices, standing, a, b, side_normals, true),
        });
    }

    Ok(past_the_end
        && match end_face_reach {
            Some(reach) => reach.min_along_segment(pa, pb)?.value < 0.0,
            None => !turns_along_a_side(positions, indices, standing, a, b, side_normals, false),
        })
}

/// A flat patch of triangles and the soft bends on its rim.
pub type FlatPatch = (IndexSet<i32>, Vec<(i32, i32)>);

/// C# `EndFaceFlow.FlatPatch`: the triangles reached from `seeds` without crossing a chain edge, a
/// fold or a bend (a bend at `end_vertex` is crossed, as the end flood crosses it), and the soft
/// bends on the patch's rim that do not lead back into `previous`. Both keep C#'s `HashSet`
/// insertion order.
#[allow(clippy::too_many_arguments)]
pub fn flat_patch(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    chain_keys: &HashSet<(i32, i32)>,
    is_feature_edge: &dyn Fn(i32, i32) -> bool,
    seeds: &[i32],
    previous: Option<&IndexSet<i32>>,
    end_vertex: i32,
) -> Result<FlatPatch, WorkError> {
    let mut patch: IndexSet<i32> = seeds.iter().copied().collect();
    let mut pending: VecDeque<i32> = patch.iter().copied().collect();
    let mut soft_bends: IndexSet<(i32, i32)> = IndexSet::new();
    while let Some(t) = pending.pop_front() {
        LevelLineWork::charge(1)?;
        for (a, b) in triangle_edges::corners(indices[t as usize]) {
            let key = triangle_edges::key(a, b);
            let Some(on) = standing.get(&key).filter(|on| on.len() == 2) else {
                continue;
            };
            if chain_keys.contains(&key) || is_feature_edge(a, b) {
                continue;
            }

            if a != end_vertex && b != end_vertex && bends(positions, indices, standing, a, b) {
                soft_bends.insert(key);
                continue;
            }

            for &next in on {
                if patch.insert(next) {
                    pending.push_back(next);
                }
            }
        }
    }

    soft_bends.retain(|key| {
        let on = &standing[key];
        !(on.iter().all(|t| patch.contains(t))
            || previous.is_some_and(|previous| on.iter().any(|t| previous.contains(t))))
    });
    Ok((patch, soft_bends.into_iter().collect()))
}

/// C# `EndFaceFlow.FlowOverANarrowEndFace`: the Flow decision at an END - an end face narrower than
/// the setback is not an end the band stops at. Where the band at this size (`end_reach`'s level
/// line) cuts across a far crease of the end region (a crease not at the end vertex), the region
/// carries on over that crease onto the next face, so the end cut lands there and the creases
/// running out of the narrow face's far side are part of that end, not foreign walls. The flow
/// crosses only creases the band reaches, so it stops where the setback does.
#[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
pub fn flow_over_a_narrow_end_face(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    chain_keys: &HashSet<(i32, i32)>,
    is_feature_edge: &dyn Fn(i32, i32) -> bool,
    end_vertex: i32,
    tag: i32,
    tags: &mut [i32],
    end_reach: &ChainField,
    can_enter: Option<&dyn Fn(i32) -> bool>,
) -> Result<(), WorkError> {
    let mut across = Vec::new();
    for (&key, on) in standing {
        if on.len() != 2 || key.0 == end_vertex || key.1 == end_vertex || chain_keys.contains(&key)
        {
            continue;
        }

        let inside = if tags[on[0] as usize] == tag {
            on[0]
        } else if tags[on[1] as usize] == tag {
            on[1]
        } else {
            -1
        };
        let outside = if inside == on[0] { on[1] } else { on[0] };
        if inside < 0 || tags[outside as usize] != 0 {
            continue;
        }

        // A CREASE, not a bend: a curved end face's facets bend one into the next, and the band
        // crossing one of those is still inside the end face.
        if is_feature_edge(key.0, key.1)
            && end_reach
                .min_along_segment(positions[key.0 as usize], positions[key.1 as usize])?
                .value
                < 0.0
        {
            across.push(outside);
        }
    }

    if across.is_empty() {
        return Ok(());
    }

    // The flood's stop test reads the band, which may charge the budget: the first error stops
    // every crossing and is handed back after the flood.
    let failed: RefCell<Option<WorkError>> = RefCell::new(None);
    let stops = |a: i32, b: i32| {
        if failed.borrow().is_some() {
            return true;
        }

        if !(is_feature_edge(a, b) || bends(positions, indices, standing, a, b)) {
            return false;
        }

        match end_reach.min_along_segment(positions[a as usize], positions[b as usize]) {
            Ok(least) => !(least.value < 0.0),
            Err(error) => {
                *failed.borrow_mut() = Some(error);
                true
            }
        }
    };
    band_side_tagging::flood(
        indices, standing, chain_keys, &stops, &across, tag, tags, can_enter,
    )?;
    match failed.into_inner() {
        Some(error) => Err(error),
        None => Ok(()),
    }
}

/// C# `EndFaceFlow.RoundsFromTheEnd`: true when the END FACE is a ROUND FROM THE END ON: from the
/// end vertex on, a run of soft bends bounding [`ROUND_RUN_JOINS`] full facets as wide as each
/// other, all turning by as much, about the same axis - and the part of a facet the end vertex
/// stands on no wider than they are, wherever in that facet the vertex sits. A D prism's flat edge
/// ending on the D's round is the case: stopped at the first join the next facet read as foreign,
/// a finer round giving a SMALLER room. Not a flat end face that rounds away later, nor a flat face
/// with a lean or two in it.
#[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
pub fn rounds_from_the_end(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    standing: &StandingMap,
    chain_keys: &HashSet<(i32, i32)>,
    is_feature_edge: &dyn Fn(i32, i32) -> bool,
    seeds: &[i32],
    end_vertex: i32,
) -> Result<bool, WorkError> {
    let mut entry_from = positions[end_vertex as usize];
    let mut entry_along = Vector3::ZERO;
    let (mut area, mut exits) = flat_patch(
        positions,
        indices,
        standing,
        chain_keys,
        is_feature_edge,
        seeds,
        None,
        end_vertex,
    )?;
    let mut widths = Vec::new();
    let mut turns = Vec::new();
    let mut axes: Vec<Vector3> = Vec::new();
    for join in 0..=ROUND_RUN_JOINS {
        if exits.is_empty() {
            return Ok(false);
        }

        // The next join is the exit nearest the entry (a point at the end vertex, a join line
        // after).
        let from_entry = |edge: (i32, i32)| {
            let from = positions[edge.0 as usize];
            let along = (positions[edge.1 as usize] - from).get_normal();
            if join == 0 {
                let to_end = entry_from - from;
                return (to_end - (along * to_end.dot(along))).length();
            }

            let offset = ((from + positions[edge.1 as usize]) / 2.0) - entry_from;
            (offset - (entry_along * offset.dot(entry_along))).length()
        };

        // C#'s OrderBy(..).First(): the first nearest, NaN ordering first.
        let mut exit = exits[0];
        for &candidate in &exits[1..] {
            let (d, best) = (from_entry(candidate), from_entry(exit));
            if !best.is_nan() && (d.is_nan() || d < best) {
                exit = candidate;
            }
        }

        let width = from_entry(exit);
        let on = &standing[&exit];
        // C#'s List.Find: the first match, or 0 where none.
        let inside = on.iter().copied().find(|t| area.contains(t)).unwrap_or(0);
        let across = on.iter().copied().find(|t| !area.contains(t)).unwrap_or(0);
        let in_normal = normal_of(positions, indices[inside as usize]);
        let out_normal = normal_of(positions, indices[across as usize]);
        let axis = in_normal.cross(out_normal);
        if !(width > 0.0) || !(axis.length() > 0.0) {
            return Ok(false);
        }

        widths.push(width);
        turns.push(in_normal.dot(out_normal).clamp(-1.0, 1.0).acos());
        axes.push(axis.get_normal());

        // Across the join: every edge of it is the next area's entry.
        entry_from = positions[exit.0 as usize];
        entry_along = (positions[exit.1 as usize] - entry_from).get_normal();
        let previous = area;
        let next = flat_patch(
            positions,
            indices,
            standing,
            chain_keys,
            is_feature_edge,
            &[across],
            Some(&previous),
            end_vertex,
        )?;
        area = next.0;
        exits = next.1;
    }

    // The end vertex's own facet is only bounded ABOVE: where the vertex sits inside it must not
    // decide whether the wall is a round.
    let full = &widths[1..];
    let least = |values: &[f64]| values.iter().copied().fold(f64::INFINITY, f64::min);
    let most = |values: &[f64]| values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    Ok(most(full) <= ROUND_RUN_SPREAD * least(full)
        && widths[0] <= ROUND_RUN_SPREAD * least(full)
        && most(&turns) <= ROUND_RUN_SPREAD * least(&turns)
        && axes
            .iter()
            .all(|axis| axis.dot(axes[0]) > round_run_axis_cosine()))
}
