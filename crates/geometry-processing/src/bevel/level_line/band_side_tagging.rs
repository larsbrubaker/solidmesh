//! The side tagging of the band: which side of the chain each triangle stands on, and the end
//! regions. Ports `Bevel/LevelLine/BandSideTagging.cs` (a C# static class): so far its tags, the
//! both-sided-members switch, the model's scale, the outward direction at a chain end and a facet
//! normal; the floods land with later packages.

use vector_math::Vector3;

use super::LevelLineWork;

/// The end flood's tag at the chain's FIRST vertex.
pub const FIRST_END_TAG: i32 = 3;

/// The end flood's tag at the chain's LAST vertex.
pub const SECOND_END_TAG: i32 = 4;

/// In the end regions' working tags: a triangle on a side another chain of the pick seeds - closed
/// to the end flood but where it stands past the end.
pub const ANOTHER_PICKS_SIDE: i32 = -1;

/// Whether a both-sided member of a pick is built (C# `BothSidedMembersBuild`, an `AsyncLocal`
/// flow switch, so here a field of the budget in force): on by default and with no budget in force;
/// off - a test's way back to the old answer - the band and the pair room refuse such a member as
/// `ChainDoesNotDivideItsSides`.
pub fn both_sided_members_build() -> bool {
    LevelLineWork::current().is_none_or(|budget| !budget.both_sided_members_off.get())
}

/// Sets the both-sided-members switch on the budget in force; with none in force there is no
/// build to carry it, and nothing changes.
pub fn set_both_sided_members_build(on: bool) {
    if let Some(budget) = LevelLineWork::current() {
        budget.both_sided_members_off.set(!on);
    }
}

/// The diagonal of the positions' box; 0 for none.
pub fn model_scale_of(positions: &[Vector3]) -> f64 {
    let mut min = Vector3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max = Vector3::new(f64::MIN, f64::MIN, f64::MIN);
    for &p in positions {
        min = Vector3::component_min(min, p);
        max = Vector3::component_max(max, p);
    }

    if positions.is_empty() {
        0.0
    } else {
        (max - min).length()
    }
}

/// The unit direction out of the chain at its first (or last) end: from the first chain vertex
/// back along the chain at least a thousandth of the model's scale, to the end.
pub fn outward_at(
    positions: &[Vector3],
    chain_edges: &[(i32, i32)],
    first: bool,
    model_scale: f64,
) -> Vector3 {
    let n = chain_edges.len();
    let end_point = positions[if first {
        chain_edges[0].0
    } else {
        chain_edges[n - 1].1
    } as usize];
    let mut back = end_point;
    for i in 0..n {
        let edge = if first {
            chain_edges[i]
        } else {
            chain_edges[n - 1 - i]
        };
        back = positions[if first { edge.1 } else { edge.0 } as usize];
        if (end_point - back).length() >= 1e-3 * model_scale {
            break;
        }
    }

    (end_point - back).get_normal()
}

/// The unit normal of an input triangle.
pub fn normal_of(positions: &[Vector3], t: (i32, i32, i32)) -> Vector3 {
    (positions[t.1 as usize] - positions[t.0 as usize])
        .cross(positions[t.2 as usize] - positions[t.0 as usize])
        .get_normal()
}
