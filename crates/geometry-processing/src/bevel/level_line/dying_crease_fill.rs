//! The fill where a growing crease dies out under the band. Ports
//! `Bevel/LevelLine/DyingCreaseFill.cs` (a C# static class): so far the extreme rail vertex
//! ([`extreme`]), the open run between two loop indices ([`run`]), which cut it fills
//! ([`is_one_loop_round_an_open_chain`]) and the fill itself ([`try_die_out`]), which cuts the one
//! loop into two runs and hands them to the fade's own fill (`open_gap_fill_fill::fill`).

use std::cell::RefCell;
use std::rc::Rc;

use vector_math::{cs_math, Vector3};

use super::open_gap_fill;
use super::open_gap_fill_fill::{self, OpenFillEnds};
use super::{
    BandGapResult, BandProfile, GapFillRefusal, GapFillResult, GapLoop, LevelLineMesh,
    LevelLineMeshError, LevelLineWork, WorkError,
};

/// A watcher of the dying crease's hole, as directed edges.
pub type WatchDyingHole = Rc<dyn Fn(&[(i32, i32)])>;

thread_local! {
    /// C# `[ThreadStatic] DyingCreaseFill.WatchDyingHole`: a read-only observer of the dying
    /// crease's hole - the directed edges round the two runs the loop is cut into, handed over
    /// just before the fade's fill closes them. Per thread; `None` (the default) costs nothing.
    pub static WATCH_DYING_HOLE: RefCell<Option<WatchDyingHole>> = const { RefCell::new(None) };
}

/// C# `DyingCreaseFill.TryDieOut`: the fill where a growing crease dies out at both ends under the
/// band - one open chain and one closed loop round it. The loop is cut at the two vertices where
/// it passes the chain's ends ([`extreme`]) into two open runs, which go to the fade's own fill.
/// Anything else about a one-loop open gap is refused by name.
pub fn try_die_out(
    gap: &BandGapResult,
    profile: BandProfile,
    fillet_segments: i32,
) -> Result<GapFillResult, LevelLineMeshError> {
    let refused = GapFillResult {
        refusal: GapFillRefusal::OpenChainEnds,
        mesh: gap.mesh.clone(),
        ..GapFillResult::default()
    };
    // C# reads `gap.Mesh` without a null check: a gap with no mesh cannot be one loop round a
    // chain the cut walked, so it is refused here rather than dereferenced.
    let Some(mesh_handle) = gap
        .mesh
        .as_ref()
        .filter(|_| is_one_loop_round_an_open_chain(Some(gap)))
    else {
        open_gap_fill::set_ends_site(Some("dying crease 1".to_string()));
        return Ok(refused);
    };

    let gap_loop = &gap.loops[0];
    let chain = &gap.chains[0];
    if gap_loop.vertices.len() < 4 {
        // Fewer than four boundary vertices cannot make two rails of two points each.
        open_gap_fill::set_ends_site(Some("dying crease 2".to_string()));
        return Ok(refused);
    }

    let (near, far) = {
        let mesh = mesh_handle.borrow();
        let mut extended = vec![0.0; gap_loop.vertices.len()];
        for (i, value) in extended.iter_mut().enumerate() {
            LevelLineWork::charge(1)?;
            *value = open_gap_fill::extended(
                mesh.position(gap_loop.vertices[i]),
                gap_loop.parameters[i],
                chain,
            );
        }

        let points = chain.points();
        (
            extreme(&mesh, gap_loop, &extended, chain.length, points[0], false),
            extreme(
                &mesh,
                gap_loop,
                &extended,
                chain.length,
                points[points.len() - 1],
                true,
            ),
        )
    };

    if near == far {
        open_gap_fill::set_ends_site(Some("dying crease 3".to_string()));
        return Ok(refused);
    }

    let one = run(gap_loop, near as usize, far as usize)?;
    let other = run(gap_loop, far as usize, near as usize)?;
    if one.vertices.len() < 2 || other.vertices.len() < 2 {
        open_gap_fill::set_ends_site(Some("dying crease 4".to_string()));
        return Ok(refused);
    }

    let watch = WATCH_DYING_HOLE.with(|watch| watch.borrow().clone());
    if let Some(watch) = watch {
        watch(&open_gap_fill::hole_between(&one, &other));
    }

    let mut laid = Vec::new();
    let built = open_gap_fill_fill::fill(
        mesh_handle,
        chain,
        &one,
        &other,
        profile,
        fillet_segments,
        &mut laid,
        OpenFillEnds::default(),
    )?;
    Ok(if built.filled() {
        let (triangles, vertices) = (
            built.added_triangles.len() as i32,
            built.added_vertices.len() as i32,
        );
        GapFillResult {
            refusal: GapFillRefusal::None,
            mesh: built.mesh,
            added_triangles: built.added_triangles,
            added_vertices: built.added_vertices,
            triangles_of_chain: vec![triangles],
            vertices_of_chain: vec![vertices],
            ..GapFillResult::default()
        }
    } else {
        built
    })
}

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

/// C# `DyingCreaseFill.IsOneLoopRoundAnOpenChain`: whether this cut is the one this type fills: one
/// OPEN chain, and exactly ONE closed loop left - what a crease dying out at BOTH ends leaves.
pub fn is_one_loop_round_an_open_chain(gap: Option<&BandGapResult>) -> bool {
    gap.is_some_and(|gap| {
        gap.chains.len() == 1
            && !gap.chains[0].is_closed
            && gap.loops.len() == 1
            && gap.loops[0].is_closed
    })
}
