//! A side consumed to its rim. Ports `Bevel/LevelLine/ConsumedToTheRim.cs` (a C# static class):
//! the foreign creases the band at a size ends on exactly ([`rims_the_band_ends_on`]) and whether
//! an edge lies on one of them ([`lies_on_a_rim`]).

use std::cell::RefCell;
use std::rc::Rc;

use vector_math::Vector3;

use super::foreign_edge_room;
use super::{ChainField, LevelLineMesh, WorkError};

/// C# `ConsumedToTheRim.RimsTheBandEndsOn`: the foreign crease segments (of the INPUT mesh, read
/// before the cut) that the band at this size ends on exactly, when it reaches foreign edges only
/// there; `None` when it does not - it reaches none on its level line, or it stands over some
/// foreign edge, at a vertex or part way along it. ON the line is within two snaps: the field
/// value the margin's strip is wide (margin x unit setback = 2 x snap). The limit fails safe - a
/// rim read just past it is not this case and refuses by size.
pub fn rims_the_band_ends_on(
    mesh: &Rc<RefCell<LevelLineMesh>>,
    chain_edges: &[(i32, i32)],
    field: &Rc<ChainField>,
    seam_epsilon: f64,
    above_the_room: f64,
    margin: f64,
    snap: f64,
) -> Result<Option<Vec<(Vector3, Vector3)>>, WorkError> {
    // Past the margin the band would cross the rim into the far face; that is a size too big, as
    // ever.
    if above_the_room.is_nan() || above_the_room <= 0.0 || above_the_room > margin * (1.0 + 1e-6) {
        return Ok(None);
    }

    let on_the_line = 2.0 * snap * (1.0 + 1e-6);
    let mut rims = Vec::new();
    let foreign =
        foreign_edge_room::foreign_edges(mesh, chain_edges, field, false, seam_epsilon, None)?;
    let mesh = mesh.borrow();
    for (low, high) in foreign {
        let a = mesh.position(low);
        let b = mesh.position(high);
        let a_value = field.evaluate(a)?.value;
        let b_value = field.evaluate(b)?.value;
        if field.min_along_segment(a, b)?.value < -on_the_line {
            // The band stands over this edge, at an end or part way along: it crosses a foreign
            // crease.
            return Ok(None);
        }

        if a_value.abs() <= on_the_line && b_value.abs() <= on_the_line {
            rims.push((a, b));
        }
    }

    Ok(if rims.is_empty() { None } else { Some(rims) })
}

/// True when both `p` and `q` stand within `snap` of one rim segment.
pub fn lies_on_a_rim(rims: &[(Vector3, Vector3)], p: Vector3, q: Vector3, snap: f64) -> bool {
    rims.iter()
        .any(|&rim| distance_to_segment(p, rim) <= snap && distance_to_segment(q, rim) <= snap)
}

/// How far `p` stands from the segment.
pub fn distance_to_segment(p: Vector3, segment: (Vector3, Vector3)) -> f64 {
    let along = segment.1 - segment.0;
    let t = if along.length_squared() > 0.0 {
        ((p - segment.0).dot(along) / along.length_squared()).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (p - (segment.0 + (along * t))).length()
}
