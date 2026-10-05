//! The fill where a growing crease dies out under the band. Ports
//! `Bevel/LevelLine/DyingCreaseFill.cs` (a C# static class): so far the extreme rail vertex
//! ([`extreme`]) and the open run between two loop indices ([`run`]); the fill lands with later
//! packages.

use vector_math::{cs_math, Vector3};

use super::open_gap_fill;
use super::{GapLoop, LevelLineMesh, LevelLineWork, WorkError};

/// The loop vertex at the greatest (or least) extended parameter: among the ties - within the open
/// fill's parameter tie of the chain's `length` - the ones nearest `end_vertex`, and of those the
/// middle one.
pub fn extreme(
    mesh: &LevelLineMesh,
    gap_loop: &GapLoop,
    extended: &[f64],
    length: f64,
    end_vertex: Vector3,
    greatest: bool,
) -> i32 {
    let tie = open_gap_fill::PARAMETER_TIE_RELATIVE * cs_math::max_f64(1.0, length);
    let mut best = extended[0];
    for &value in &extended[1..] {
        best = if greatest {
            cs_math::max_f64(best, value)
        } else {
            cs_math::min_f64(best, value)
        };
    }

    let tied: Vec<usize> = (0..extended.len())
        .filter(|&i| (extended[i] - best).abs() <= tie)
        .collect();

    let reach_of = |i: usize| (mesh.position(gap_loop.vertices[i]) - end_vertex).length_squared();
    let mut reach = reach_of(tied[0]);
    let mut equals = vec![tied[0]];
    for &i in &tied[1..] {
        let d = reach_of(i);
        if d < reach * (1.0 - 1e-12) {
            reach = d;
            equals = vec![i];
        } else if d <= reach * (1.0 + 1e-12) {
            equals.push(i);
        }
    }

    equals[equals.len() / 2] as i32
}

/// The stretch of the closed loop from one index to another, walked the loop's own way and
/// carrying both end points - an OPEN run, in the loop's own sense, so the kept surface still lies
/// to its left.
pub fn run(gap_loop: &GapLoop, from: usize, to: usize) -> Result<GapLoop, WorkError> {
    let count = gap_loop.vertices.len();
    let mut vertices = Vec::new();
    let mut parameters = Vec::new();
    let mut i = from;
    loop {
        LevelLineWork::charge(1)?;
        vertices.push(gap_loop.vertices[i]);
        parameters.push(gap_loop.parameters[i]);
        if i == to {
            break;
        }

        i = (i + 1) % count;
    }

    Ok(GapLoop::new(
        gap_loop.side,
        false,
        vertices,
        parameters,
        gap_loop.chain,
        gap_loop.has_mixed_owners,
        None,
    ))
}
