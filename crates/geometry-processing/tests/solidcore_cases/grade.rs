//! Grading one case's replays against its closed-form expectations, by the limit the faceted
//! meshes converge to rather than by any one mesh.
//!
//! Every case runs at three resolutions (`RESOLUTIONS`): its curves chorded into 48, 96 and 192
//! inscribed sides to a turn, and its rounds into 6, 12 and 24 segments per quarter turn, so every
//! chord's angle halves from one run to the next. The expected volumes are for exact solids:
//!
//! - **A flat case** (boxes, polygon profiles and chamfers only: nothing curved anywhere) has no
//!   faceting. Its volume must meet the closed form within the case's `tol` plus the float floor
//!   at every resolution. The float floor is `FLOAT_FLOOR_MM` times the surface area: the mesh
//!   stores f32 positions (a 100 mm part's ulp is about 8e-6 mm) and outlines snap to a 1 µm grid.
//! - **A faceted case** converges at second order: an inscribed chord of angle `h` misses its arc
//!   by an area that goes as `h^2`, for the input curves and the bevel's arcs alike, and both
//!   halve together here. So, with `V0, V1, V2` the three volumes and `d1 = V1 - V0`,
//!   `d2 = V2 - V1`:
//!   - `d1` and `d2` must be of one sign, and `|d2|` smaller than `|d1|`: volumes that step back
//!     and forth, or move further at each resolution, are not converging and would only widen
//!     their own allowance (a `d2` within the case's `tol` is converged: the limit is `V2` and the
//!     residual `|d2|`);
//!   - the observed order `log2(d1 / d2)` is reported in the detail, not graded: an extrapolation
//!     that assumes the wrong order shows up as a large residual, which the two checks below
//!     catch, so a case whose outline alone converges a little off second order is not failed
//!     for a reason of the grading's own;
//!   - the limit is Richardson's `V2 + d2 / 3`;
//!   - the residual is the gap between that limit and the one the two coarser runs give,
//!     `|(V2 + d2 / 3) - (V1 + d1 / 3)|`: the size of the higher-order terms the extrapolation
//!     drops, measured on the case itself rather than assumed;
//!   - the limit must meet the closed form within `tol` + the residual. No float floor is added:
//!     f32 rounding noise is in the three volumes the residual is measured from, so noise large
//!     enough to matter widens the residual, which the blend share below then bounds;
//!   - and that allowance must be at most `BLEND_SHARE` of the blend's own volume change (the
//!     finest run's body before the case's last fillet or chamfer, against that step's own
//!     output, both read at that step, so later booleans and reused ids do not count as the
//!     blend's; a case with a blend whose change cannot be read fails), so an allowance that
//!     could swallow the blend fails the case instead of passing it. The runner
//!     also proves this case by case: every pass with a volume expectation must fail when its
//!     last blend is left out.
//!
//! A volume that is not a finite number fails outright (a NaN would pass every comparison). A run
//! whose built outline is missing a point it was given (`replay.rs`: extrude and revolve drop
//! points that fall collinear on their 1 µm grid) fails too, with the one phrase `LOST_VERTICES`
//! and the counts in the detail: that run is not the polygon its resolution claims, so no
//! convergence reading of it means anything.
//!
//! `removed` is graded the same way on the volume the result lost against the earlier body it
//! names. `lumps` is the number of connected pieces (`mesh_measurements::shell_count`, by
//! position); `manifold` is a closed, consistently wound 2-manifold once coincident vertices are
//! welded by position (the booleans leave seams split by index on purpose). `refused` passes when
//! the bevel refuses the named step with any code (the detail names it), and with a `fit_size`
//! only when the largest size it reports meets the case's within its `tol`.
//! `cylinder_faces_max` counts B-rep faces, which a mesh has no reading of: a case carrying it is
//! at best partly graded, and so is a case with a blend and no volume, removal or refusal
//! expectation, since nothing then says what the blend did.
//!
//! An issue is a short, number-free phrase, so a known-failing case's `seen` stays the same from
//! run to run until its behaviour really changes; the numbers go in the detail.

use std::collections::HashMap;

use solidmesh::math::Vector3Float;
use solidmesh::mesh::Mesh;

use crate::case_file::{Case, Expect};
use crate::mesh_measurements::{shell_count, signed_volume};
use crate::replay::{replay, Replay, Resolution};

/// The three runs, each chord's angle half the one before.
pub const RESOLUTIONS: [Resolution; 3] = [
    Resolution {
        circle_sides: 48,
        bevel_segments: 6,
    },
    Resolution {
        circle_sides: 96,
        bevel_segments: 12,
    },
    Resolution {
        circle_sides: 192,
        bevel_segments: 24,
    },
];

/// Volume per unit of surface area the f32 storage and one-micron outline grid can move (mm).
pub const FLOAT_FLOOR_MM: f64 = 1e-5;

/// The issue for a run whose built outline is missing points it was given (the counts go in the
/// detail, so the issue reads the same at every resolution).
pub const LOST_VERTICES: &str = "an outline lost vertices on the 1 µm grid";

/// The largest share of the blend's own volume change a faceted case's allowance may be.
pub const BLEND_SHARE: f64 = 0.1;

/// What a case came to: the issues (empty when it passes), a line of measurements, and whether
/// it carries an expectation a mesh cannot be graded on.
#[derive(Debug, Clone)]
pub struct Graded {
    pub issues: Vec<String>,
    pub detail: String,
    pub partly_graded: bool,
    /// Whether it has a volume or removal expectation that was graded.
    pub volume_graded: bool,
}

fn position_key(p: Vector3Float) -> (u32, u32, u32) {
    // `+ 0.0` folds -0 onto 0.
    (
        (p.x + 0.0).to_bits(),
        (p.y + 0.0).to_bits(),
        (p.z + 0.0).to_bits(),
    )
}

/// Whether the mesh is a closed, consistently wound 2-manifold once coincident vertices are one:
/// every edge walked exactly once each way, no face with a repeated corner.
pub fn is_closed_manifold(mesh: &Mesh) -> bool {
    if mesh.faces.is_empty() {
        return false;
    }
    let mut ids: HashMap<(u32, u32, u32), usize> = HashMap::new();
    let welded: Vec<usize> = mesh
        .vertices
        .iter()
        .map(|p| {
            let next = ids.len();
            *ids.entry(position_key(*p)).or_insert(next)
        })
        .collect();
    let mut walks: HashMap<(usize, usize), u32> = HashMap::new();
    for face in mesh.faces.iter() {
        let (a, b, c) = (
            welded[face.v0 as usize],
            welded[face.v1 as usize],
            welded[face.v2 as usize],
        );
        if a == b || b == c || c == a {
            return false;
        }
        for edge in [(a, b), (b, c), (c, a)] {
            *walks.entry(edge).or_insert(0) += 1;
        }
    }
    // Only lookups: the map's order never reaches an output.
    walks
        .iter()
        .all(|(&(a, b), &count)| count == 1 && walks.get(&(b, a)) == Some(&1))
}

fn build_issue(message: &str) -> String {
    if message.starts_with("no feature edge") || message.contains("no feature edges") {
        "a point picks no feature edge".to_string()
    } else if message.starts_with("boolean") {
        "a boolean fails".to_string()
    } else if message.starts_with("bevel_part could not run") {
        "bevel_part could not run".to_string()
    } else {
        "a step could not be built".to_string()
    }
}

/// One volume read at every resolution: the measured values, the finest run's surface area and
/// the finest run's blend change.
struct VolumeReading {
    measured: [f64; 3],
    area: f64,
    /// `None` for a case with no fillet or chamfer of its own to measure.
    blend_change: Option<f64>,
}

fn grade_volume(
    label: &str,
    reading: &VolumeReading,
    expected: f64,
    tol: f64,
    faceted: bool,
    issues: &mut Vec<String>,
) -> String {
    let [v0, v1, v2] = reading.measured;
    if reading.measured.iter().any(|v| !v.is_finite()) {
        issues.push(format!("{label} is not a finite number"));
        return format!("{label} {v0}/{v1}/{v2} want {expected}");
    }
    if !faceted {
        let floor = reading.area * FLOAT_FLOOR_MM;
        let worst = reading
            .measured
            .iter()
            .map(|v| (v - expected).abs())
            .fold(0.0, f64::max);
        if worst > tol + floor {
            issues.push(format!("{label} misses the closed form of a flat case"));
        }
        return format!(
            "{label} {v0:.6}/{v1:.6}/{v2:.6} want {expected} worst err {worst:.3e} allow {:.3e}",
            tol + floor
        );
    }
    let (d1, d2) = (v1 - v0, v2 - v1);
    let (limit, residual, order) = if d2.abs() <= tol {
        (v2, d2.abs(), f64::NAN)
    } else {
        let order = libm::log2(d1.abs() / d2.abs());
        let limit = v2 + d2 / 3.0;
        (limit, (limit - (v1 + d1 / 3.0)).abs(), order)
    };
    let allowance = tol + residual;
    let error = (limit - expected).abs();
    let detail = format!(
        "{label} {v0:.6}/{v1:.6}/{v2:.6} order {order:.2} limit {limit:.6} want {expected} err \
         {error:.3e} allow {allowance:.3e} blend {:?}",
        reading.blend_change
    );
    if d2.abs() > tol && d1 * d2 <= 0.0 {
        issues.push(format!(
            "{label} steps back and forth instead of converging"
        ));
    } else if d2.abs() > tol && d2.abs() >= d1.abs() {
        issues.push(format!(
            "{label} moves further at each resolution instead of converging"
        ));
    } else if error > allowance {
        issues.push(format!("{label}'s converged limit misses the closed form"));
    }
    if let Some(change) = reading.blend_change {
        if !change.is_finite() {
            issues.push(format!("{label}'s blend change cannot be measured"));
        } else if allowance > BLEND_SHARE * change {
            issues.push(format!(
                "{label}'s allowance is not small next to the blend's own change"
            ));
        }
    }
    detail
}

/// The issues one run's result has against the case's expectations.
fn run_issues(
    run: &Result<Replay, String>,
    resolution: Resolution,
    expect: &Expect,
    details: &mut Vec<String>,
) -> Vec<String> {
    let sides = resolution.circle_sides;
    let mut found = Vec::new();
    let replay = match run {
        Err(message) => {
            details.push(format!("[{sides}] {message}"));
            return vec![build_issue(message)];
        }
        Ok(replay) => replay,
    };
    for (step, lost, asked) in &replay.lost_outline_vertices {
        if !found.iter().any(|f| f == LOST_VERTICES) {
            found.push(LOST_VERTICES.to_string());
        }
        details.push(format!(
            "[{sides}] {step} lost {lost} of {asked} outline vertices"
        ));
    }
    match (&replay.refusal, &expect.refused) {
        (Some(refusal), Some(wanted)) if refusal.step == wanted.step => {
            details.push(format!(
                "[{sides}] refused {} with {} (fits {:?})",
                refusal.step,
                refusal.codes.join(","),
                refusal.largest_size_that_fits
            ));
            if let Some(fit) = &wanted.fit_size {
                match refusal.largest_size_that_fits {
                    None => found.push("the refusal reports no largest fit".to_string()),
                    Some(seen) if (seen - fit.value).abs() > fit.tol => {
                        found.push("the refusal's largest fit misses the case's".to_string())
                    }
                    Some(_) => {}
                }
            }
        }
        (Some(refusal), _) => {
            found.push(format!(
                "the bevel refuses {} ({})",
                refusal.step,
                refusal.codes.join(",")
            ));
            details.push(format!(
                "[{sides}] fits {:?}",
                refusal.largest_size_that_fits
            ));
        }
        (None, Some(wanted)) => {
            found.push(format!(
                "{} builds where a refusal is expected",
                wanted.step
            ));
            if let Some(result) = replay.bodies.get(&wanted.step) {
                details.push(format!(
                    "[{sides}] built volume {:.6}, {} lumps, closed manifold {}",
                    signed_volume(result),
                    shell_count(result),
                    is_closed_manifold(result)
                ));
            }
        }
        (None, None) => match replay.bodies.get("result") {
            None => found.push("the case writes no result".to_string()),
            Some(result) => {
                if let Some(lumps) = expect.lumps {
                    let seen = shell_count(result);
                    if seen as usize != lumps {
                        found.push(format!("{seen} lumps where {lumps} are expected"));
                    }
                }
                if let Some(manifold) = expect.manifold {
                    if is_closed_manifold(result) != manifold {
                        found.push(if manifold {
                            "the result is not a closed manifold".to_string()
                        } else {
                            "the result is a closed manifold".to_string()
                        });
                    }
                }
            }
        },
    }
    found
}

/// The case run at every resolution and graded; `skip_last_blend` leaves its last fillet or
/// chamfer out (the runner's check that the grading can see the blend).
pub fn grade(case: &Case, skip_last_blend: bool) -> Graded {
    if let Some(why) = &case.unreadable {
        return Graded {
            issues: vec![why.split(':').next().unwrap_or(why).to_string()],
            detail: why.clone(),
            partly_graded: false,
            volume_graded: false,
        };
    }
    let expect = match &case.expect {
        Ok(expect) => expect,
        Err(error) => {
            return Graded {
                issues: vec!["its expectations are not understood".to_string()],
                detail: error.clone(),
                partly_graded: false,
                volume_graded: false,
            }
        }
    };
    let runs: Vec<Result<Replay, String>> = RESOLUTIONS
        .iter()
        .map(|r| replay(case, *r, skip_last_blend))
        .collect();
    let mut details: Vec<String> = Vec::new();

    // An issue seen at every resolution is plain; one seen at only some says at which.
    let per_run: Vec<Vec<String>> = runs
        .iter()
        .zip(RESOLUTIONS)
        .map(|(run, resolution)| run_issues(run, resolution, expect, &mut details))
        .collect();
    let mut issues: Vec<String> = Vec::new();
    for found in &per_run {
        for issue in found {
            let at: Vec<String> = per_run
                .iter()
                .zip(RESOLUTIONS)
                .filter(|(other, _)| other.contains(issue))
                .map(|(_, r)| r.circle_sides.to_string())
                .collect();
            let worded = if at.len() == RESOLUTIONS.len() {
                issue.clone()
            } else {
                format!("at {} sides only: {issue}", at.join(", "))
            };
            if !issues.contains(&worded) {
                issues.push(worded);
            }
        }
    }

    // Volumes, where every run built a result.
    let built: Vec<&Replay> = runs
        .iter()
        .filter_map(|run| run.as_ref().ok())
        .filter(|replay| replay.refusal.is_none())
        .collect();
    let mut volume_graded = false;
    if built.len() == RESOLUTIONS.len() && expect.refused.is_none() {
        let finest = built[RESOLUTIONS.len() - 1];
        let faceted = finest
            .facets
            .get("result")
            .is_some_and(|facets| facets.is_faceted());
        let volume = |replay: &Replay, id: &str| replay.bodies.get(id).map(signed_volume);
        let area = |id: &str| finest.bodies.get(id).map_or(0.0, |m| m.get_surface_area());
        // A case with a blend always gets a reading; one that cannot be taken is NaN, which the
        // grading refuses rather than skipping the cap.
        let blend_change = case
            .has_blend()
            .then(|| finest.last_blend_change.unwrap_or(f64::NAN));
        let read = |of: &dyn Fn(&Replay) -> Option<f64>| -> Option<[f64; 3]> {
            Some([of(built[0])?, of(built[1])?, of(built[2])?])
        };
        if let Some(wanted) = &expect.volume {
            if let Some(measured) = read(&|r| volume(r, "result")) {
                volume_graded = true;
                let reading = VolumeReading {
                    measured,
                    area: area("result"),
                    blend_change,
                };
                details.push(grade_volume(
                    "volume",
                    &reading,
                    wanted.value,
                    wanted.tol,
                    faceted,
                    &mut issues,
                ));
            }
        }
        if let Some(removed) = &expect.removed {
            let lost = |r: &Replay| Some(volume(r, &removed.from)? - volume(r, "result")?);
            match read(&lost) {
                Some(measured) => {
                    volume_graded = true;
                    let reading = VolumeReading {
                        measured,
                        area: area(&removed.from) + area("result"),
                        blend_change,
                    };
                    details.push(grade_volume(
                        "removed",
                        &reading,
                        removed.value,
                        removed.tol,
                        faceted,
                        &mut issues,
                    ));
                }
                None => issues.push(format!(
                    "no body named {} to measure the removal from",
                    removed.from
                )),
            }
        }
    }
    let blend_unmeasured = case.has_blend()
        && expect.volume.is_none()
        && expect.removed.is_none()
        && expect.refused.is_none();
    if expect.cylinder_faces_max.is_some() {
        details.push("cylinder_faces_max not graded on a mesh".to_string());
    }
    if blend_unmeasured {
        details.push("no volume, removal or refusal says what the blend did".to_string());
    }
    let partly_graded = expect.cylinder_faces_max.is_some() || blend_unmeasured;
    Graded {
        issues,
        detail: details.join(" | "),
        partly_graded,
        volume_graded,
    }
}
