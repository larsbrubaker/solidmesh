//! The corner fill's zip of one chain's two rail pieces into the strip between them, with the
//! last rows it hands the corner caps. The spoke centre, the nearest station and the setback are
//! in `corner_rail_strip`, the knife end overrun in `knife_end_overrun`.

use std::collections::HashMap;
use std::rc::Rc;

use vector_math::{cs_math, Vector3};

use super::corner_cap::CornerCapLastRow;
use super::corner_fill_end_ray_trim::TrimSections;
use super::corner_rail_strip::{nearest, set_back, spoke_centre_parameter};
use super::fillet_arc_rows::{build, build_skew, on_a_rim_joints_seam};
use super::gap_fill::quads;
use super::knife_end_overrun;
use super::rail_stations::{densify, tie_spans};
use super::uneven_strip::cell;
use super::{
    ChainField, CornerFillPiece, GapFillRefusal, LevelLineMesh, LevelLineMeshError, LevelLineWork,
};

/// Each value once, in first-occurrence order, NaN equal to NaN and 0 to -0.
fn distinct(values: impl IntoIterator<Item = f64>) -> Vec<f64> {
    let mut kept: Vec<f64> = Vec::new();
    for v in values {
        if !kept.iter().any(|&k| k == v || (k.is_nan() && v.is_nan())) {
            kept.push(v);
        }
    }

    kept
}

/// The first (or, with `last`, the last) index whose value passes `test`, -1 where none does.
fn find_index(values: &[f64], test: impl Fn(f64) -> bool, last: bool) -> i32 {
    let found = if last {
        values.iter().rposition(|&v| test(v))
    } else {
        values.iter().position(|&v| test(v))
    };
    found.map_or(-1, |i| i as i32)
}

/// Zips the chain `own`'s two rail pieces (`first`, `second`, which
/// must run opposite ways) into the strip between them - densified so each rail has a station at
/// every place the other stands for, set back where the chain ties another (`set_back`), clipped
/// to `within` - laying each cell as quads between matching spokes or an uneven cell, the knife
/// end overruns, and handing the caps one last row at each end. The climbing piece is densified
/// (and read by the overrun) in place, the falling one before its setback. Answers the refusal
/// and the offending vertex.
#[allow(clippy::too_many_arguments)]
pub fn strip(
    mesh: &mut LevelLineMesh,
    chains: &[Rc<ChainField>],
    own: usize,
    with_set_back: bool,
    first: &mut CornerFillPiece,
    second: &mut CornerFillPiece,
    segments: i32,
    added: &mut Vec<i32>,
    added_vertices: &mut Vec<i32>,
    last_rows: &mut Vec<CornerCapLastRow>,
    splits: &mut HashMap<(i32, i32), i32>,
    sections: &TrimSections,
    within: Option<(f64, f64)>,
) -> Result<(GapFillRefusal, i32), LevelLineMeshError> {
    let chain = &chains[own];
    let tie = 1e-9 * cs_math::max_f64(1.0, chain.length);
    let climbs = |p: &CornerFillPiece| p.parameters[p.parameters.len() - 1] > p.parameters[0];
    let first_climbs = climbs(first);
    if first_climbs == climbs(second) {
        return Ok((GapFillRefusal::RailsRunTheSameWay, first.vertices[0]));
    }

    let (a, b) = if first_climbs {
        (first, second)
    } else {
        (second, first)
    };
    let overrun_first = knife_end_overrun::read(mesh, chain, a, b, tie, false)?;
    let overrun_last = knife_end_overrun::read(mesh, chain, a, b, tie, true)?;
    let mut av = a.vertices.clone();
    let mut ap = a.parameters.clone();
    let mut bv: Vec<i32> = b.vertices.iter().rev().copied().collect();
    let mut bp: Vec<f64> = b.parameters.iter().rev().copied().collect();
    for (vertices, parameters) in [(&av, &ap), (&bv, &bp)] {
        for i in 1..parameters.len() {
            if parameters[i] < parameters[i - 1] - tie {
                return Ok((GapFillRefusal::RailIsNotMonotone, vertices[i]));
            }
        }
    }

    let mut jump_ends: HashMap<i32, f64> = HashMap::new();
    let stands =
        |mesh: &LevelLineMesh, v: &[i32], p: &[f64]| -> Result<Vec<f64>, LevelLineMeshError> {
            let spans = tie_spans(mesh, chain, v, p, tie, 0.0)?;
            Ok(distinct(
                p.iter()
                    .copied()
                    .chain(spans.iter().flat_map(|s| [s.0, s.1])),
            ))
        };
    let a_stands = stands(mesh, &av, &ap)?;
    let b_stands = stands(mesh, &bv, &bp)?;
    let mut offending = densify(
        mesh,
        chain,
        &mut av,
        &mut ap,
        &b_stands,
        tie,
        splits,
        &mut jump_ends,
        1,
    )?;
    if offending < 0 {
        offending = densify(
            mesh,
            chain,
            &mut bv,
            &mut bp,
            &a_stands,
            tie,
            splits,
            &mut jump_ends,
            1,
        )?;
    }

    // The climbing piece takes `av`/`ap`: its densified lists stand.
    a.vertices = av.clone();
    a.parameters = ap.clone();
    if offending >= 0 {
        return Ok((GapFillRefusal::RailIsNotMonotone, offending));
    }

    b.vertices = bv.iter().rev().copied().collect();
    b.parameters = bp.iter().rev().copied().collect();

    let mut lo = cs_math::max_f64(ap[0], bp[0]) - tie;
    let mut hi = cs_math::min_f64(ap[ap.len() - 1], bp[bp.len() - 1]) + tie;
    if with_set_back {
        lo = set_back(
            mesh,
            chains,
            own,
            &mut av,
            &mut ap,
            &mut bv,
            &mut bp,
            tie,
            splits,
            &mut jump_ends,
            true,
        )?
        .unwrap_or(lo);
        hi = set_back(
            mesh,
            chains,
            own,
            &mut av,
            &mut ap,
            &mut bv,
            &mut bp,
            tie,
            splits,
            &mut jump_ends,
            false,
        )?
        .unwrap_or(hi);
        // The climbing piece is still aliased by `av`/`ap`; the falling one was written back above.
        a.vertices = av.clone();
        a.parameters = ap.clone();
    }

    if let Some((low, high)) = within {
        lo = cs_math::max_f64(lo, low - tie);
        hi = cs_math::min_f64(hi, high + tie);
    }

    let i0 = nearest(&ap, find_index(&ap, |p| p >= lo, false), lo, -1);
    let i1 = nearest(&ap, find_index(&ap, |p| p <= hi, true), hi, 1);
    let j0 = nearest(&bp, find_index(&bp, |p| p >= lo, false), lo, -1);
    let j1 = nearest(&bp, find_index(&bp, |p| p <= hi, true), hi, 1);
    if i0 < 0 || j0 < 0 || i1 < i0 || j1 < j0 {
        return Ok((GapFillRefusal::SideDoesNotLeaveOneLoop, av[0]));
    }

    let (i0, i1, j0, j1) = (i0 as usize, i1 as usize, j0 as usize, j1 as usize);
    let a_span = tie_spans(mesh, chain, &av, &ap, tie, 0.0)?;
    let b_span = tie_spans(mesh, chain, &bv, &bp, tie, 0.0)?;
    let reads = |i: usize, j: usize| {
        (
            bp[j].clamp(a_span[i].0, a_span[i].1),
            ap[i].clamp(b_span[j].0, b_span[j].1),
        )
    };
    let on_one_section =
        |mesh: &LevelLineMesh, a_vertex: i32, a_reads: f64, b_vertex: i32, b_reads: f64| {
            let a_width = (mesh.position(a_vertex) - chain.point_at(a_reads)).length();
            let b_width = (mesh.position(b_vertex) - chain.point_at(b_reads)).length();
            (a_width - b_width).abs() <= 0.1 * cs_math::max_f64(a_width, b_width)
        };
    let jumps = |vertex: i32, span: (f64, f64), reads: f64| {
        (jump_ends.contains_key(&vertex) || span.1 - span.0 > tie)
            && !(span.1 - span.0 > tie && (reads <= span.0 + tie || reads >= span.1 - tie))
    };

    let mut spokes: HashMap<(usize, usize), Vec<i32>> = HashMap::new();
    let mut spoke =
        |mesh: &mut LevelLineMesh, added_vertices: &mut Vec<i32>, i: usize, j: usize| {
            if let Some(arc) = spokes.get(&(i, j)) {
                return arc.clone();
            }

            let (a_reads, b_reads) = reads(i, j);
            let arc = if let Some(section) = sections.get(&(av[i], bv[j])) {
                section.clone()
            } else if let Some(section) = sections.get(&(bv[j], av[i])) {
                section.iter().rev().copied().collect()
            } else if av[i] == bv[j] {
                vec![av[i]; segments as usize + 1]
            } else if let Some(seam) =
                on_a_rim_joints_seam(mesh, chain, av[i], bv[j], segments, added_vertices)
            {
                seam
            } else if jumps(av[i], a_span[i], a_reads)
                || jumps(bv[j], b_span[j], b_reads)
                || !on_one_section(mesh, av[i], a_reads, bv[j], b_reads)
            {
                let centre = chain.point_at(spoke_centre_parameter(
                    a_span[i], ap[i], b_span[j], bp[j], tie,
                ));
                build(mesh, av[i], bv[j], centre, segments, added_vertices)
            } else {
                build_skew(
                    mesh,
                    av[i],
                    bv[j],
                    chain.point_at(a_reads),
                    chain.point_at((a_reads + b_reads) / 2.0),
                    chain.point_at(b_reads),
                    segments,
                    added_vertices,
                )
            };
            spokes.insert((i, j), arc.clone());
            arc
        };

    let order = |vertices: &[i32], parameters: &[f64], k: usize| match jump_ends.get(&vertices[k]) {
        Some(&foot) => cs_math::min_f64(foot, parameters[k]),
        None => parameters[k],
    };

    let mut ia = i0;
    let mut jb = j0;
    let advances_a = |mesh: &LevelLineMesh, ia: usize, jb: usize, next_a: f64, next_b: f64| {
        if (next_a - next_b).abs() > tie {
            return next_a < next_b;
        }

        let via_a = (mesh.position(av[ia + 1]) - mesh.position(bv[jb])).length_squared();
        let via_b = (mesh.position(av[ia]) - mesh.position(bv[jb + 1])).length_squared();
        via_a <= via_b
    };

    let mut first_step = Vector3::ZERO;
    let mut last_step = Vector3::ZERO;
    let mut first_corner = Vector3::ZERO;
    let mut last_corner = Vector3::ZERO;
    while ia < i1 || jb < j1 {
        LevelLineWork::charge(1)?;
        let advance_a = jb == j1
            || (ia < i1
                && advances_a(
                    mesh,
                    ia,
                    jb,
                    order(&av, &ap, ia + 1),
                    order(&bv, &bp, jb + 1),
                ));
        let (x, y, z) = if advance_a {
            (av[ia + 1], av[ia], bv[jb])
        } else {
            (bv[jb], bv[jb + 1], av[ia])
        };
        last_corner = mesh.position(x);
        last_step = (mesh.position(y) - last_corner).cross(mesh.position(z) - last_corner);
        if ia == i0 && jb == j0 {
            first_corner = last_corner;
            first_step = last_step;
        }

        if ia < i1
            && jb < j1
            && (order(&av, &ap, ia + 1) - order(&bv, &bp, jb + 1)).abs() <= tie
            && spoke(mesh, added_vertices, ia, jb).len()
                == spoke(mesh, added_vertices, ia + 1, jb + 1).len()
        {
            let after = spoke(mesh, added_vertices, ia + 1, jb + 1);
            let before = spoke(mesh, added_vertices, ia, jb);
            quads(mesh, &after, &before, added)?;
            ia += 1;
            jb += 1;
            continue;
        }

        if advance_a {
            let after = spoke(mesh, added_vertices, ia + 1, jb);
            let before = spoke(mesh, added_vertices, ia, jb);
            cell(mesh, &after, &before, added)?;
            ia += 1;
        } else {
            let mut here = spoke(mesh, added_vertices, ia, jb);
            here.reverse();
            let mut next = spoke(mesh, added_vertices, ia, jb + 1);
            next.reverse();
            cell(mesh, &here, &next, added)?;
            jb += 1;
        }
    }

    if let Some(end) = &overrun_first {
        let end_spoke = spoke(mesh, added_vertices, i0, j0);
        knife_end_overrun::lay(
            mesh,
            chain,
            end,
            &end_spoke,
            &bv,
            &bp,
            j0 as i32,
            segments,
            added,
            added_vertices,
        )?;
    }

    if let Some(end) = &overrun_last {
        let end_spoke = spoke(mesh, added_vertices, i1, j1);
        knife_end_overrun::lay(
            mesh,
            chain,
            end,
            &end_spoke,
            &bv,
            &bp,
            j1 as i32,
            segments,
            added,
            added_vertices,
        )?;
    }

    for (i, j, corner, step) in [
        (i0, j0, first_corner, first_step),
        (i1, j1, last_corner, last_step),
    ] {
        if (overrun_first.is_some() && i == i0 && j == j0)
            || (overrun_last.is_some() && i == i1 && j == j1)
        {
            continue;
        }

        let chain_point = chain.point_at((ap[i] + bp[j]) / 2.0);
        let skew = chain.point_at(bp[j]) - chain.point_at(ap[i]);
        let axis = if jump_ends.contains_key(&av[i])
            || jump_ends.contains_key(&bv[j])
            || skew.length() <= tie
        {
            Vector3::ZERO
        } else {
            skew.get_normal()
        };
        let arc = spoke(mesh, added_vertices, i, j);
        last_rows.push(CornerCapLastRow::new(
            arc,
            chain_point,
            mesh,
            step.dot(chain_point - corner) < 0.0,
            axis,
        ));
    }

    Ok((GapFillRefusal::None, -1))
}
