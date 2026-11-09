//! The contract `Ok` makes: after the passes, [`Engine::establish_bound`] refines any triangle
//! still past the deviation bound (measured both ways, on the same-facing sheet, more densely
//! than the edit guards), and [`check`] then holds the exact coordinates being returned to every
//! promise: closed and consistently wound, no crossing triangle pairs, no free edge folded back
//! on itself, the deviation bound, the angle floor and the creases. Whatever cannot be met is returned as a named error carrying the
//! measured value and what to change, never as a surface a later consumer would trust.
//!
//! The crossing count comes from `crossing`, whose signs are exact on the `f64` coordinates
//! given.

use std::collections::HashMap;

use crate::g3::dmesh_aabb_tree3::DMeshAABBTree3;

use super::crossing::crossing_pairs;
use super::engine::{sample_grid, Engine};
use super::sheets::{reliable_normal, Sheets};
use super::{AnalysisRemeshError, AnalysisRemeshOptions, AnalysisRemeshReport, AnalysisSurface};

/// Subdivisions per triangle side when measuring output against input: denser than the guards.
pub(super) const VERIFY_GRID: usize = 6;
/// Subdivisions per input triangle side when measuring input against output.
pub(super) const INPUT_SAMPLE_GRID: usize = 3;
/// How many refine-and-repair rounds the bound is given before the final check decides.
const MAX_BOUND_ROUNDS: u32 = 16;
/// Refinement aims a little inside the bound so the final check is not a coin toss.
const REFINE_FRACTION: f64 = 0.95;
/// Two faces of an edge with normals this opposed are folded back over each other.
const FOLD_DOT: f64 = -0.9;
/// Triangles checked between cancel polls.
const CANCEL_STRIDE: usize = 256;

impl Engine<'_> {
    /// Refines every triangle past the bound, restores shape, and repeats until none is past it.
    pub(super) fn establish_bound(&mut self) -> Result<(), AnalysisRemeshError> {
        for _ in 0..MAX_BOUND_ROUNDS {
            let past = self.triangles_past_bound()?;
            if past.is_empty() {
                return Ok(());
            }
            let mut any = false;
            for t in past {
                self.tick()?;
                if self.mesh().is_triangle(t) && self.split_longest_edge(t, false)? {
                    any = true;
                }
            }
            if !any {
                return Ok(());
            }
            self.repair_slivers()?;
        }
        Ok(())
    }

    /// Whether free edge `e` has its two faces folded back over each other.
    pub(super) fn edge_folds(&self, e: i32) -> bool {
        use crate::g3::vector3d::{cross, dot, length, normalized, sub};
        let [a, b, t0, t1] = self.mesh().get_edge(e);
        if t1 < 0 || self.crease.has(a, b) {
            return false;
        }
        let unit_normal = |t: i32| {
            let [p, q, r] = self.triangle_points(t);
            let n = cross(sub(q, p), sub(r, p));
            (length(n) > 0.0).then(|| normalized(n))
        };
        matches!((unit_normal(t0), unit_normal(t1)), (Some(n0), Some(n1)) if dot(n0, n1) < FOLD_DOT)
    }

    /// The output triangles at fault for any sample past the refinement threshold.
    fn triangles_past_bound(&self) -> Result<Vec<i32>, AnalysisRemeshError> {
        let threshold = self.opts.max_deviation * REFINE_FRACTION;
        let mut past = Vec::new();

        for (count, t) in self.mesh().triangle_indices().enumerate() {
            if count % CANCEL_STRIDE == 0 {
                self.check_cancel()?;
            }
            let [a, b, c] = self.triangle_points(t);
            let Some(normal) = reliable_normal(a, b, c) else {
                continue;
            };
            let over = sample_grid(a, b, c, VERIFY_GRID)
                .into_iter()
                .any(|p| self.sheets().distance(p, normal, threshold) > threshold);
            if over {
                past.push(t);
            }
        }

        let output_tree = DMeshAABBTree3::new(self.mesh());
        let output_sheets = Sheets::new(&output_tree);
        let source = self.sheets().mesh();
        for (count, t) in source.triangle_indices().enumerate() {
            if count % CANCEL_STRIDE == 0 {
                self.check_cancel()?;
            }
            let [a, b, c] = source.get_triangle(t).map(|v| source.vertex(v));
            let Some(normal) = reliable_normal(a, b, c) else {
                continue;
            };
            let samples = sample_grid(a, b, c, INPUT_SAMPLE_GRID);
            for p in samples.into_iter().chain([a, b, c]) {
                if output_sheets.distance(p, normal, threshold) > threshold {
                    if let Some((_, nearest)) = output_sheets.nearest(p, normal, f64::MAX) {
                        past.push(nearest);
                    }
                }
            }
        }
        past.sort_unstable();
        past.dedup();
        Ok(past)
    }
}

/// Holds the returned surface to the contract, or names what it cannot meet.
pub(super) fn check(
    surface: &AnalysisSurface,
    report: &AnalysisRemeshReport,
    options: &AnalysisRemeshOptions<'_>,
    engine: &Engine<'_>,
) -> Result<(), AnalysisRemeshError> {
    if !closed_and_wound(surface) {
        return Err(AnalysisRemeshError::BrokenTopology(
            "the remeshed surface is no longer closed and consistently wound",
        ));
    }
    let pairs = crossing_pairs(surface, engine)?;
    if pairs > 0 {
        return Err(AnalysisRemeshError::SelfIntersecting { pairs });
    }
    let folded = folded_edges(engine);
    if folded > 0 {
        return Err(AnalysisRemeshError::FoldedSurface { edges: folded });
    }
    if report.sampled_deviation > options.max_deviation {
        return Err(AnalysisRemeshError::DeviationNotMet {
            measured: report.sampled_deviation,
            bound: options.max_deviation,
        });
    }
    if report.unforced_small_angle_triangles > 0 {
        return Err(AnalysisRemeshError::AngleFloorNotMet {
            triangles: report.unforced_small_angle_triangles,
            worst_degrees: report.min_unforced_angle_degrees,
            floor_degrees: options.min_angle_floor_degrees,
        });
    }
    if report.sharp_edges_lost > 0 {
        return Err(AnalysisRemeshError::CreaseLost {
            lost: report.sharp_edges_lost,
            total: report.sharp_edges_lost + report.sharp_edges_kept,
        });
    }
    Ok(())
}

/// How many free edges have their two faces folded back over each other. An edge the input
/// creases may be that sharp; any other edge is a fold the remesh made.
fn folded_edges(engine: &Engine<'_>) -> usize {
    engine
        .mesh()
        .edge_indices()
        .filter(|&e| engine.edge_folds(e))
        .count()
}

/// Whether every edge is walked exactly once in each direction.
fn closed_and_wound(surface: &AnalysisSurface) -> bool {
    // Lookup only, so map order cannot reach the result.
    let mut walks: HashMap<(i32, i32), u32> = HashMap::new();
    for f in &surface.faces {
        for (a, b) in [(f[0], f[1]), (f[1], f[2]), (f[2], f[0])] {
            if a == b {
                return false;
            }
            *walks.entry((a, b)).or_insert(0) += 1;
        }
    }
    walks
        .iter()
        .all(|(&(a, b), &n)| n == 1 && walks.get(&(b, a)) == Some(&1))
}
