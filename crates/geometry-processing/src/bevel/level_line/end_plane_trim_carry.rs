//! The flat ends' carry of a strip on to the end plane's section, split from `end_plane_trim`.

use std::collections::HashSet;

use vector_math::{cs_math, Vector3};

use super::corner_cap::WATCH_END_HOLE_AT;
use super::{corner_fill, flat_hole_fill};
use super::{BandProfile, ChainField, LevelLineMesh, LevelLineMeshError, LevelLineWork};

/// The strip carried from its last row on to the section in the end
/// plane. The hole between that row and the trimmed run lies on the round itself, so it is
/// UNROLLED - each vertex to (distance round the row's arc, distance along the chain) - and the
/// flat hole fill triangulates it there; the triangles are laid on the real vertices. No ear may
/// have all three corners on the section. False where the unrolled hole does not close flat.
pub fn carry(
    mesh: &mut LevelLineMesh,
    chain: &ChainField,
    row: &[i32],
    run: &[i32],
    last: bool,
    profile: BandProfile,
    added: &mut Vec<i32>,
) -> Result<bool, LevelLineMeshError> {
    if row.len() < 2 || row[0] != run[0] || row[row.len() - 1] != run[run.len() - 1] {
        return Ok(false);
    }

    let points = chain.points();
    let count = points.len();
    let end = if last { points[count - 1] } else { points[0] };
    let outward = if last {
        points[count - 1] - points[count - 2]
    } else {
        points[0] - points[1]
    }
    .get_normal();
    let square = |p: Vector3| p - (outward * (p - end).dot(outward));

    let a = square(mesh.position(row[0]));
    let b = square(mesh.position(row[row.len() - 1]));
    let round: Box<dyn Fn(Vector3) -> f64> = if profile == BandProfile::Fillet {
        let sa = a - end;
        let sb = b - end;
        let half = sa.get_normal().dot(sb.get_normal()).clamp(-1.0, 1.0).acos() / 2.0;
        let centre =
            end + ((sa.get_normal() + sb.get_normal()).get_normal() * (sa.length() / half.cos()));
        let radius = sa.length() * half.tan();
        let from_centre = (a - centre).get_normal();
        let axis = from_centre.cross(b - centre).get_normal();
        let across = axis.cross(from_centre);
        Box::new(move |p| {
            let d = square(p) - centre;
            radius * d.dot(across).atan2(d.dot(from_centre))
        })
    } else {
        let chord = (b - a).get_normal();
        Box::new(move |p| (square(p) - a).dot(chord))
    };

    // the loop as ZipEnd's flat path lays it: the row forward, then the run back from T to S
    let inner = if run.len() > 2 {
        &run[1..run.len() - 1]
    } else {
        &[][..]
    };
    let loop_: Vec<i32> = row
        .iter()
        .copied()
        .chain(inner.iter().rev().copied())
        .collect();
    let unrolled: Vec<Vector3> = loop_
        .iter()
        .map(|&v| {
            let p = mesh.position(v);
            Vector3::new(round(p), (p - end).dot(outward), 0.0)
        })
        .collect();

    // The section: the run's stretch between its rail stretches (the leading and trailing vertices
    // at the rails' own distance round the arc) - every section edge is answered by a triangle
    // reaching a row or rail vertex.
    let u_end = round(mesh.position(row[row.len() - 1]));
    let rail_snap = 1e-6 * cs_math::max_f64(1.0, u_end.abs());
    let mut first = 0;
    while first + 1 < run.len() && round(mesh.position(run[first + 1])).abs() <= rail_snap {
        first += 1;
    }

    let mut last_index = run.len() - 1;
    while last_index > first + 1
        && (round(mesh.position(run[last_index - 1])) - u_end).abs() <= rail_snap
    {
        last_index -= 1;
    }

    let section: Vec<i32> = run[first..=last_index].to_vec();
    let on_section: HashSet<i32> = section.iter().copied().collect();

    let snap = if mesh.same_place() > 0.0 {
        mesh.same_place()
    } else {
        1e-9
    };
    let forbidden = |i: i32, j: i32, k: i32| {
        on_section.contains(&loop_[i as usize])
            && on_section.contains(&loop_[j as usize])
            && on_section.contains(&loop_[k as usize])
    };
    let Some((triangles, _)) = flat_hole_fill::fill(&unrolled, snap, Some(&forbidden)) else {
        return Ok(false);
    };

    for triangle in &triangles {
        LevelLineWork::charge(1)?;
        corner_fill::lay(
            mesh,
            loop_[triangle.a as usize],
            loop_[triangle.b as usize],
            loop_[triangle.c as usize],
            added,
        )?;
    }

    // The end hole the carry leaves is the section alone, so the end-hole watch reads it as both.
    let watch = WATCH_END_HOLE_AT.with(|w| w.borrow().clone());
    if let Some(watch) = watch {
        watch(mesh, &section, &section, end);
    }

    Ok(true)
}
