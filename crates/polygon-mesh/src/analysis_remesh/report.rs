//! What a remesh for analysis measured on the surface it returned, in the returned `f64`
//! coordinates: the smallest angle and worst aspect ratio, the two-way sampled surface deviation,
//! the volume change, how many input crease segments stayed on their chain, and how many small
//! angles the input's own geometry forced. `verify` turns these into pass or a named refusal.

use crate::g3::dmesh3::DMesh3;
use crate::g3::dmesh3::INVALID_ID;
use crate::g3::dmesh_aabb_tree3::DMeshAABBTree3;
use crate::g3::vector3d::{cross, dot, length, sub};

use super::crease::polyline_deviation;
use super::engine::{sample_grid, worst_cos, Engine, P};
use super::sheets::{reliable_normal, Sheets};
use super::verify::{INPUT_SAMPLE_GRID, VERIFY_GRID};
use super::{AnalysisRemeshError, AnalysisSurface};

/// Triangles measured between cancel checks.
const CANCEL_STRIDE: usize = 256;

/// What the remesh measured on its output.
#[derive(Debug, Clone, PartialEq)]
pub struct AnalysisRemeshReport {
    /// Triangles in the output.
    pub triangles: usize,
    /// The smallest triangle angle anywhere, degrees.
    pub min_angle_degrees: f64,
    /// The smallest angle of any triangle whose small corner the input does not force, degrees.
    pub min_unforced_angle_degrees: f64,
    /// The worst longest-edge over twice-inradius, scaled so an equilateral triangle is 1.
    pub worst_aspect_ratio: f64,
    /// The largest same-facing distance between the surfaces in either direction, sampled at
    /// vertices and at a barycentric grid of each triangle, more densely than the edit guards.
    /// A sampled measure: curvature between samples is not bounded by it, which is why
    /// the guards work to a fraction of the bound.
    pub sampled_deviation: f64,
    /// The output's volume change, as a percentage of the input's.
    pub volume_change_percent: f64,
    /// Input crease segments whose chain still follows them within the deviation bound.
    pub sharp_edges_kept: usize,
    /// Input crease segments that no longer do.
    pub sharp_edges_lost: usize,
    /// How many triangle corners are under each of [`Self::CORNER_LIMITS_DEGREES`], so a caller
    /// can see the quality the surface reached short of a refusal.
    pub corners_under_degrees: [usize; 5],
    /// Triangles under the angle floor where the input's own geometry forces every small corner:
    /// a crease sector or a whole vertex fan measuring under the floor in the input, as the
    /// `sliver` module header defines.
    pub forced_small_angle_triangles: usize,
    /// Triangles under the angle floor that no input feature forces.
    pub unforced_small_angle_triangles: usize,
    /// Refinement passes run.
    pub passes_run: u32,
}

impl AnalysisRemeshReport {
    /// The thresholds of [`Self::corners_under_degrees`], degrees.
    pub const CORNER_LIMITS_DEGREES: [f64; 5] = [1.0, 2.0, 5.0, 10.0, 20.0];
}

/// The working mesh as a surface, with only referenced vertices.
pub(super) fn output_surface(mesh: &DMesh3) -> AnalysisSurface {
    let mut vertices = Vec::with_capacity(mesh.vertex_count());
    let mut faces = Vec::with_capacity(mesh.triangle_count());
    let mut map = vec![-1_i32; mesh.max_vertex_id().max(0) as usize];
    for t in mesh.triangle_indices() {
        let verts = mesh.get_triangle(t);
        for v in verts {
            if map[v as usize] < 0 {
                map[v as usize] = vertices.len() as i32;
                vertices.push(mesh.vertex(v));
            }
        }
        faces.push(verts.map(|v| map[v as usize]));
    }
    AnalysisSurface { vertices, faces }
}

fn corner_angle_degrees(at: P, p: P, q: P) -> f64 {
    let (u, v) = (sub(p, at), sub(q, at));
    libm::atan2(length(cross(u, v)), dot(u, v)).to_degrees()
}

fn smallest_angle_degrees(a: P, b: P, c: P) -> f64 {
    corner_angle_degrees(a, b, c)
        .min(corner_angle_degrees(b, c, a))
        .min(corner_angle_degrees(c, a, b))
}

fn aspect_ratio(a: P, b: P, c: P) -> f64 {
    let sides = [length(sub(b, a)), length(sub(c, b)), length(sub(a, c))];
    let area = length(cross(sub(b, a), sub(c, a))) * 0.5;
    if area <= 0.0 {
        return f64::INFINITY;
    }
    let semi = (sides[0] + sides[1] + sides[2]) * 0.5;
    sides[0].max(sides[1]).max(sides[2]) / (2.0 * 3.0_f64.sqrt() * (area / semi))
}

fn signed_volume(a: P, b: P, c: P) -> f64 {
    dot(a, cross(b, c)) / 6.0
}

fn mesh_volume(mesh: &DMesh3) -> f64 {
    mesh.triangle_indices()
        .map(|t| {
            let [a, b, c] = mesh.get_triangle(t).map(|v| mesh.vertex(v));
            signed_volume(a, b, c)
        })
        .sum()
}

/// The largest same-facing distance from `from`'s triangles to `to`, sampled on a grid of each
/// triangle (and its corners). A triangle too thin to have a normal has nothing to compare with.
fn directed_deviation(
    engine: &Engine<'_>,
    from: &DMesh3,
    to: &Sheets<'_>,
    grid: usize,
) -> Result<f64, AnalysisRemeshError> {
    let mut worst = 0.0_f64;
    for (count, t) in from.triangle_indices().enumerate() {
        if count % CANCEL_STRIDE == 0 {
            engine.check_cancel()?;
        }
        let [a, b, c] = from.get_triangle(t).map(|v| from.vertex(v));
        let Some(normal) = reliable_normal(a, b, c) else {
            continue;
        };
        for p in sample_grid(a, b, c, grid).into_iter().chain([a, b, c]) {
            worst = worst.max(to.distance(p, normal, f64::MAX));
        }
    }
    Ok(worst)
}

/// Measures `surface`, which is `engine`'s mesh as returned, against the input in `source`.
pub(super) fn measure(
    source: &DMesh3,
    surface: &AnalysisSurface,
    engine: &Engine<'_>,
    passes_run: u32,
) -> Result<AnalysisRemeshReport, AnalysisRemeshError> {
    let (mut min_angle, mut worst_aspect) = (180.0_f64, 0.0_f64);
    for f in &surface.faces {
        let [a, b, c] = f.map(|v| surface.vertices[v as usize]);
        min_angle = min_angle.min(smallest_angle_degrees(a, b, c));
        worst_aspect = worst_aspect.max(aspect_ratio(a, b, c));
    }
    let mut corners_under = [0; 5];
    for f in &surface.faces {
        let [a, b, c] = f.map(|v| surface.vertices[v as usize]);
        for angle in [
            corner_angle_degrees(a, b, c),
            corner_angle_degrees(b, c, a),
            corner_angle_degrees(c, a, b),
        ] {
            for (count, limit) in corners_under
                .iter_mut()
                .zip(AnalysisRemeshReport::CORNER_LIMITS_DEGREES)
            {
                *count += usize::from(angle < limit);
            }
        }
    }
    let (forced, unforced, min_unforced) = classify_small_angles(engine);

    let out_to_in = directed_deviation(engine, engine.mesh(), engine.sheets(), VERIFY_GRID)?;
    let output_tree = DMeshAABBTree3::new(engine.mesh());
    let output_sheets = Sheets::new(&output_tree);
    let in_to_out = directed_deviation(engine, source, &output_sheets, INPUT_SAMPLE_GRID)?;

    let (kept, lost) = crease_survival(engine)?;
    let (volume_in, volume_out) = (mesh_volume(source), mesh_volume(engine.mesh()));

    Ok(AnalysisRemeshReport {
        triangles: surface.faces.len(),
        min_angle_degrees: min_angle,
        min_unforced_angle_degrees: min_unforced,
        worst_aspect_ratio: worst_aspect,
        sampled_deviation: out_to_in.max(in_to_out),
        volume_change_percent: (volume_out - volume_in) / volume_in * 100.0,
        sharp_edges_kept: kept,
        sharp_edges_lost: lost,
        corners_under_degrees: corners_under,
        forced_small_angle_triangles: forced,
        unforced_small_angle_triangles: unforced,
        passes_run,
    })
}

/// How many small-angle triangles are forced, how many are not, and the smallest angle of
/// those not forced (180 when there are none).
fn classify_small_angles(engine: &Engine<'_>) -> (usize, usize, f64) {
    let mesh = engine.mesh();
    let (mut forced, mut unforced, mut min_unforced) = (0, 0, 180.0_f64);
    for t in mesh.triangle_indices() {
        let [a, b, c] = engine.triangle_points(t);
        let small = worst_cos(a, b, c) > engine.cos_floor;
        if small && engine.is_forced(t, engine.cos_floor, engine.opts.min_angle_floor_degrees) {
            forced += 1;
            continue;
        }
        if small {
            unforced += 1;
        }
        min_unforced = min_unforced.min(smallest_angle_degrees(a, b, c));
    }
    (forced, unforced, min_unforced)
}

/// Of the input crease segments, how many a surviving chain edge still carries with its
/// chord within the deviation bound. Each chain is walked once, and every piece of a segment
/// must be held: the whole net is linear in the polylines, with no search of other creases.
fn crease_survival(engine: &Engine<'_>) -> Result<(usize, usize), AnalysisRemeshError> {
    let mesh = engine.mesh();
    let total = engine.crease.input_segments() as usize;
    let (mut seen, mut held) = (vec![false; total], vec![true; total]);
    for (count, ((a, b), points, segments)) in engine.crease.chains().enumerate() {
        if count % CANCEL_STRIDE == 0 {
            engine.check_cancel()?;
        }
        let edge_is_there = mesh.find_edge(a, b) != INVALID_ID;
        let within = polyline_deviation(points) <= engine.opts.max_deviation;
        for &id in segments {
            seen[id as usize] = true;
            held[id as usize] &= edge_is_there && within;
        }
    }
    let kept = (0..total).filter(|&i| seen[i] && held[i]).count();
    Ok((kept, total - kept))
}
