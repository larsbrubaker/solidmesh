//! Replays one case's steps with solidmesh's production code at one resolution: boxes from
//! `platonic_solids::create_cube`, cylinders, extrusions and revolves from `outline_to_mesh`, the
//! booleans from `csg::boolean_processing::do_array`, and every fillet and chamfer through
//! `solidmesh::bevel_part`.
//!
//! A fillet or chamfer step names its edges by points on them. Each point picks the feature edge
//! of the part's pick data (`BevelPickData`, the welded working copy at the operation's bend
//! threshold) nearest it, and the pick grows to that edge's tangent chain
//! (`BevelSelectionKind::TangentChain`), so one point on a faceted rim picks the whole rim, as a
//! B-rep edge would be. A point whose chain an earlier point already picked adds nothing. A point
//! farther from every feature edge than the faceting can explain is a missed pick, not a guess.
//! A pick the bevel refuses, wholly or on some of its rim loops, stops the case there with the
//! refusal codes, which is what a case expecting a refusal is graded on.
//!
//! Each body carries its [`Facets`]: the curves it was faceted from, the fillets already on it and
//! the turns of the edges they rounded, joined through booleans and carried through moves. A
//! pick's miss distance is read from the body being beveled, before the blend about to be built,
//! so the round a step is making never widens its own pick. A replay can also leave out its last
//! fillet or chamfer (that step copies its body unchanged), which is how the runner proves its
//! grading tells a blend from no blend.

use std::collections::{HashMap, HashSet};
use std::sync::Arc;

use solidmesh::clipper::DEFAULT_SCALING;
use solidmesh::math::{Matrix4X4, Vector2, Vector3};
use solidmesh::mesh::csg::boolean_processing::do_array;
use solidmesh::mesh::csg::{BooleanOptions, CsgModes};
use solidmesh::mesh::processors::outline_to_mesh::{extrude_contours, revolve_contours};
use solidmesh::mesh::solids::platonic_solids::create_cube;
use solidmesh::mesh::{CancelToken, Mesh};
use solidmesh::{bevel_part, pick, BevelPartRequest, BevelPickData, BevelSelectionKind, FaceGroup};

use crate::case_file::{Built, Case, MirrorAxis, Step};
use crate::mesh_measurements::signed_volume;
use crate::profile::{arc_chords, flatten_profile, frame_onto, place, plane_frame, rotated, v3};

/// The bend threshold the picks and the bevel read the part at (degrees): a faceted wall's facet
/// joins at the resolutions here turn 7.5 degrees or less and are not edges; a box's are.
pub const BEND_THRESHOLD_DEGREES: f64 = 20.0;

/// How finely curves are faceted for one run of a case.
#[derive(Debug, Clone, Copy)]
pub struct Resolution {
    /// Chords to a full turn of every input circle, arc and revolve.
    pub circle_sides: usize,
    /// The bevel's segments (its `segments`, per quarter turn of a round).
    pub bevel_segments: i32,
}

/// The bevel's refusal of a step: its id and the advisory codes it gave.
#[derive(Debug, Clone)]
pub struct Refusal {
    pub step: String,
    pub codes: Vec<String>,
    pub largest_size_that_fits: Option<f64>,
}

/// What a body was faceted from.
#[derive(Debug, Clone, Copy, Default)]
pub struct Facets {
    /// The largest radius any input curve of the body was chorded at (mm), zero for flat inputs.
    pub largest_curve_radius: f64,
    /// The largest fillet radius on the body (zero with none; a chamfer has no arc to chord).
    pub largest_fillet_radius: f64,
    /// The largest turn of any edge a blend on the body picked (degrees).
    pub largest_picked_turn_degrees: f64,
}

/// What a replay built.
#[derive(Debug, Default)]
pub struct Replay {
    pub bodies: HashMap<String, Mesh>,
    pub facets: HashMap<String, Facets>,
    pub refusal: Option<Refusal>,
    /// The volume the case's last fillet or chamfer changed its body by, read at that step from
    /// the body it was given and the mesh it built, so a step that writes over its own source id,
    /// or a later step that reuses the output id, cannot change what is measured. NaN where the
    /// source could not be read; `None` until that step has built.
    pub last_blend_change: Option<f64>,
    /// Each extrusion of a curved outline whose mesh kept fewer outline vertices than it was
    /// given: the step, how many were lost and how many were asked for.
    pub lost_outline_vertices: Vec<(String, usize, usize)>,
}

/// A point's place on the outline grid (`DEFAULT_SCALING` units per mm, 1 µm), rounded half to
/// even as `outline_to_mesh` snaps it. A mesh vertex the outline code placed is exactly such a
/// point stored in f32, which reads back to the same integer for any part under a few metres.
fn grid_key(x: f64, y: f64) -> (i64, i64) {
    // `as` saturates, which only an absurd coordinate reaches.
    (
        (x * DEFAULT_SCALING).round_ties_even() as i64,
        (y * DEFAULT_SCALING).round_ties_even() as i64,
    )
}

/// How many of the outline's points the built mesh has no vertex at, and how many were checked.
/// Each point is snapped to the grid and looked up among `kept` (the mesh's vertices in the
/// outline's own plane); a point already collinear with its neighbours in the input is skipped,
/// since dropping it changes nothing. `extrude_contours` and `revolve_contours` drop points that
/// fall collinear on the 1 µm grid, which on a small circle at a fine resolution turns the polygon
/// asked for into a coarser one; such a run cannot stand for the resolution it claims. Matching
/// points one by one, rather than counting vertices, neither flags an outline whose own collinear
/// points were merged nor misses a loss that crossing points added back.
fn lost_outline_vertices(
    kept: &HashSet<(i64, i64)>,
    loops: &[Vec<Vector2>],
) -> Option<(usize, usize)> {
    let (mut lost, mut checked) = (0, 0);
    for points in loops {
        let n = points.len();
        for i in 0..n {
            let (before, at, after) = (points[(i + n - 1) % n], points[i], points[(i + 1) % n]);
            let (a, b) = (at - before, after - at);
            let turn = a.x * b.y - a.y * b.x;
            if turn.abs() <= 1e-9 * a.length() * b.length() {
                continue;
            }
            checked += 1;
            if !kept.contains(&grid_key(at.x, at.y)) {
                lost += 1;
            }
        }
    }
    (lost > 0).then_some((lost, checked))
}

/// The grid keys of an extrusion's vertices, in the outline's (x, y) plane.
fn extruded_keys(mesh: &Mesh) -> HashSet<(i64, i64)> {
    mesh.vertices
        .iter()
        .map(|v| grid_key(f64::from(v.x), f64::from(v.y)))
        .collect()
}

/// The grid keys of a revolve's profile as built: its vertices at angle zero, where the profile
/// point `(x, y)` lands at `(x, 0, y)` (a point left of the axis is mirrored onto the right).
fn revolved_keys(mesh: &Mesh) -> HashSet<(i64, i64)> {
    mesh.vertices
        .iter()
        .filter(|v| v.y == 0.0)
        .map(|v| grid_key(f64::from(v.x).abs(), f64::from(v.z)))
        .collect()
}

impl Facets {
    /// Whether anything about the body is chorded.
    pub fn is_faceted(&self) -> bool {
        self.largest_curve_radius > 0.0 || self.largest_fillet_radius > 0.0
    }

    /// Both bodies' facets, as a boolean of them has.
    pub fn join(self, other: Facets) -> Facets {
        Facets {
            largest_curve_radius: self.largest_curve_radius.max(other.largest_curve_radius),
            largest_fillet_radius: self.largest_fillet_radius.max(other.largest_fillet_radius),
            largest_picked_turn_degrees: self
                .largest_picked_turn_degrees
                .max(other.largest_picked_turn_degrees),
        }
    }

    /// The largest chord sagitta of this replay's curves at `resolution` (mm): `R * (1 - cos(pi /
    /// sides))` for the largest input curve radius `R`, plus `r * (1 - cos(step / 2))` for the
    /// largest fillet radius `r`, `step` being the larger of 90 degrees and the largest picked
    /// edge's turn over the segment count (the bevel chords per quarter turn, or per arc where
    /// an arc turns less than that).
    pub fn sagitta(&self, resolution: Resolution) -> f64 {
        let curve = self.largest_curve_radius
            * (1.0 - libm::cos(std::f64::consts::PI / resolution.circle_sides as f64));
        let step_degrees =
            self.largest_picked_turn_degrees.max(90.0) / f64::from(resolution.bevel_segments);
        let round =
            self.largest_fillet_radius * (1.0 - libm::cos((step_degrees / 2.0).to_radians()));
        curve + round
    }
}

fn boolean(items: &[&Mesh], mode: CsgModes) -> Result<Mesh, String> {
    let items: Vec<(&Mesh, Matrix4X4)> = items.iter().map(|m| (*m, Matrix4X4::IDENTITY)).collect();
    do_array(
        &items,
        mode,
        &CancelToken::new(),
        &BooleanOptions::default(),
    )
    .map_err(|e| format!("boolean {mode:?}: {e:?}"))
}

fn body<'a>(bodies: &'a HashMap<String, Mesh>, id: &str) -> Result<&'a Mesh, String> {
    bodies.get(id).ok_or_else(|| format!("no body named {id}"))
}

/// The bevel's picks for points on edges, grown to their tangent chains, and the largest turn
/// among the picked edges.
fn picks_for(
    mesh: &Arc<Mesh>,
    points: &[Vector3],
    miss_distance: f64,
) -> Result<(Vec<solidmesh::BevelEdgeGroup>, f64), String> {
    let data = BevelPickData::build(mesh, 0, |_| None, BEND_THRESHOLD_DEGREES);
    let features = data.pick_features();
    let tolerance = FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES;
    features.prepare_tangent_chains(tolerance);

    let mut groups = Vec::new();
    let mut picked_mesh_edges: Vec<i32> = Vec::new();
    let mut largest_turn: f64 = 0.0;
    for point in points {
        let nearest = features
            .edges()
            .iter()
            .enumerate()
            .map(|(i, edge)| (i, point.distance_to_segment(edge.start, edge.end)))
            // Ties go to the lower index, so the pick never depends on float noise in the sort.
            .min_by(|a, b| a.1.total_cmp(&b.1).then(a.0.cmp(&b.0)));
        let Some((index, distance)) = nearest else {
            return Err("the part has no feature edges to pick".to_string());
        };
        if distance > miss_distance {
            return Err(format!(
                "no feature edge at {point:?} (nearest {distance:.6} mm away)"
            ));
        }
        let index = index as i32;
        let edge = &features.edges()[index as usize];
        if picked_mesh_edges.contains(&edge.mesh_edge_index) {
            continue;
        }
        let chain_edges: Vec<i32> = match features.tangent_chain_through(index, tolerance) {
            Some((_, edges)) => edges.to_vec(),
            None => vec![edge.mesh_edge_index],
        };
        for mesh_edge in &chain_edges {
            let feature = features.index_of_mesh_edge(*mesh_edge);
            if feature >= 0 {
                largest_turn =
                    largest_turn.max(features.edges()[feature as usize].turn_angle_degrees.abs());
            }
        }
        picked_mesh_edges.extend(chain_edges);
        let group =
            pick::group_from_edge(features, index, BevelSelectionKind::TangentChain, tolerance)
                .map_err(|e| format!("group_from_edge: {e}"))?;
        groups.push(group);
    }
    Ok((groups, largest_turn))
}

/// The facets a step's inputs bring (a revolve's radius is only known once its profile is read).
fn input_facets(step: &Built, known: &HashMap<String, Facets>) -> Facets {
    let of = |id: &str| known.get(id).copied().unwrap_or_default();
    let curve = |radius: f64| Facets {
        largest_curve_radius: radius,
        ..Facets::default()
    };
    match step {
        Built::Box { .. } | Built::Revolve { .. } => Facets::default(),
        Built::Cylinder { radius, .. } => curve(*radius),
        Built::Extrude { profile, .. } => curve(profile.iter().fold(0.0, |r, seg| match seg {
            crate::case_file::Seg::Arc { radius, .. }
            | crate::case_file::Seg::Circle { radius, .. } => r.max(*radius),
            _ => r,
        })),
        Built::Union { a, b, .. }
        | Built::Intersection { a, b, .. }
        | Built::Difference { a, b, .. } => of(a).join(of(b)),
        Built::Rotate { body, .. }
        | Built::Mirror { body, .. }
        | Built::Fillet { body, .. }
        | Built::Chamfer { body, .. } => of(body),
    }
}

/// The case built at `resolution` up to its last step, or to the bevel step it refuses; an
/// error is a step that could not run at all (a boolean or outline failure, a missed pick).
/// With `skip_last_blend` the case's last fillet or chamfer copies its body unchanged.
pub fn replay(
    case: &Case,
    resolution: Resolution,
    skip_last_blend: bool,
) -> Result<Replay, String> {
    let sides = resolution.circle_sides;
    let mut replay = Replay::default();
    let last_blend = case.steps.iter().rposition(|step| {
        matches!(
            step,
            Step::Built(Built::Fillet { .. }) | Step::Built(Built::Chamfer { .. })
        )
    });
    for (index, step) in case.steps.iter().enumerate() {
        let Step::Built(step) = step else {
            return Err("an unported op".to_string());
        };
        let mut facets = input_facets(step, &replay.facets);
        let skipped = skip_last_blend && Some(index) == last_blend;
        let (id, mesh) = match step {
            Built::Box { id, origin, size } => {
                let mut cube = create_cube(size[0], size[1], size[2]);
                cube.translate(v3(*origin) + v3(*size) * 0.5);
                (id, cube)
            }
            Built::Cylinder {
                id,
                origin,
                radius,
                height,
                axis,
            } => {
                let contour = crate::profile::flatten_profile(
                    &[crate::case_file::Seg::Circle {
                        center: [0.0, 0.0],
                        radius: *radius,
                    }],
                    sides,
                )?;
                let mut mesh = extrude_contours(&contour, *height).map_err(|e| format!("{e:?}"))?;
                if let Some((lost, asked)) = lost_outline_vertices(&extruded_keys(&mesh), &contour)
                {
                    replay.lost_outline_vertices.push((id.clone(), lost, asked));
                }
                let (ex, ey, ez) = frame_onto(v3(*axis));
                place(&mut mesh, v3(*origin), ex, ey, ez);
                (id, mesh)
            }
            Built::Extrude {
                id,
                profile,
                distance,
                plane,
                origin,
            } => {
                let loops = flatten_profile(profile, sides)?;
                let mut mesh =
                    extrude_contours(&loops, distance.abs()).map_err(|e| format!("{e:?}"))?;
                if let Some((lost, asked)) = lost_outline_vertices(&extruded_keys(&mesh), &loops) {
                    replay.lost_outline_vertices.push((id.clone(), lost, asked));
                }
                let (u, v, n) = plane_frame(*plane);
                let n = if *distance < 0.0 { n * -1.0 } else { n };
                place(&mut mesh, v3(*origin), u, v, n);
                (id, mesh)
            }
            Built::Revolve {
                id,
                profile,
                plane,
                origin,
                axis_point,
                axis_dir,
                angle_deg,
            } => {
                let loops = flatten_profile(profile, sides)?;
                let (u, v, _) = plane_frame(*plane);
                let p0 = v3(*axis_point);
                let a = Vector3::normalized(v3(*axis_dir));
                let world: Vec<Vec<Vector3>> = loops
                    .iter()
                    .map(|l| l.iter().map(|p| v3(*origin) + u * p.x + v * p.y).collect())
                    .collect();
                // The profile plane's radial direction, from the point farthest off the axis.
                let mut radial = Vector3::ZERO;
                for point in world.iter().flatten() {
                    let off = (*point - p0) - a * (*point - p0).dot(a);
                    if off.length() > radial.length() {
                        radial = off;
                    }
                }
                if radial.length() == 0.0 {
                    return Err("the revolve profile lies on its axis".to_string());
                }
                facets.largest_curve_radius = radial.length();
                let e1 = Vector3::normalized(radial);
                let contours: Vec<Vec<Vector2>> = world
                    .iter()
                    .map(|l| {
                        l.iter()
                            .map(|q| Vector2::new((*q - p0).dot(e1), (*q - p0).dot(a)))
                            .collect()
                    })
                    .collect();
                let steps = arc_chords(*angle_deg, sides) as i32;
                let mut mesh =
                    revolve_contours(&contours, steps, 0.0, angle_deg.to_radians(), true)
                        .map_err(|e| format!("{e:?}"))?;
                // The revolve mirrors a point left of the axis onto the right before it builds.
                let mirrored: Vec<Vec<Vector2>> = contours
                    .iter()
                    .map(|l| l.iter().map(|p| Vector2::new(p.x.abs(), p.y)).collect())
                    .collect();
                if let Some((lost, asked)) = lost_outline_vertices(&revolved_keys(&mesh), &mirrored)
                {
                    replay.lost_outline_vertices.push((id.clone(), lost, asked));
                }
                place(&mut mesh, p0, e1, a.cross(e1), a);
                (id, mesh)
            }
            Built::Union { id, a, b } => (
                id,
                boolean(
                    &[body(&replay.bodies, a)?, body(&replay.bodies, b)?],
                    CsgModes::Union,
                )?,
            ),
            Built::Intersection { id, a, b } => (
                id,
                boolean(
                    &[body(&replay.bodies, a)?, body(&replay.bodies, b)?],
                    CsgModes::Intersect,
                )?,
            ),
            Built::Difference { id, a, b, pattern } => {
                let tool = body(&replay.bodies, b)?.clone();
                let mut result = body(&replay.bodies, a)?.clone();
                let offsets: Vec<Vector3> = match pattern {
                    None => vec![Vector3::ZERO],
                    Some(p) => (0..p.count[1])
                        .flat_map(|j| (0..p.count[0]).map(move |i| (i, j)))
                        .map(|(i, j)| v3(p.pitch[0]) * i as f64 + v3(p.pitch[1]) * j as f64)
                        .collect(),
                };
                for offset in offsets {
                    let mut moved = tool.clone();
                    moved.translate(offset);
                    result = boolean(&[&result, &moved], CsgModes::Subtract)?;
                }
                (id, result)
            }
            Built::Rotate {
                id,
                body: from,
                point,
                axis,
                degrees,
            } => {
                let mut mesh = body(&replay.bodies, from)?.clone();
                let (k, angle, p) = (
                    Vector3::normalized(v3(*axis)),
                    degrees.to_radians(),
                    v3(*point),
                );
                let origin = p - rotated(p, k, angle);
                place(
                    &mut mesh,
                    origin,
                    rotated(Vector3::UNIT_X, k, angle),
                    rotated(Vector3::UNIT_Y, k, angle),
                    rotated(Vector3::UNIT_Z, k, angle),
                );
                (id, mesh)
            }
            Built::Mirror {
                id,
                body: from,
                normal,
                at,
            } => {
                let mut mesh = body(&replay.bodies, from)?.clone();
                let (mut ex, mut ey, mut ez) = (Vector3::UNIT_X, Vector3::UNIT_Y, Vector3::UNIT_Z);
                let origin = match normal {
                    MirrorAxis::X => {
                        ex *= -1.0;
                        Vector3::new(2.0 * at, 0.0, 0.0)
                    }
                    MirrorAxis::Y => {
                        ey *= -1.0;
                        Vector3::new(0.0, 2.0 * at, 0.0)
                    }
                    MirrorAxis::Z => {
                        ez *= -1.0;
                        Vector3::new(0.0, 0.0, 2.0 * at)
                    }
                };
                place(&mut mesh, origin, ex, ey, ez);
                (id, mesh)
            }
            Built::Fillet { id, body: from, .. } | Built::Chamfer { id, body: from, .. }
                if skipped =>
            {
                (id, body(&replay.bodies, from)?.clone())
            }
            Built::Fillet {
                id,
                body: from,
                edges,
                radius,
            } => match bevel(
                &mut replay,
                id,
                from,
                edges,
                *radius,
                true,
                facets,
                resolution,
            )? {
                Some((mesh, turn)) => {
                    facets.largest_fillet_radius = facets.largest_fillet_radius.max(*radius);
                    facets.largest_picked_turn_degrees =
                        facets.largest_picked_turn_degrees.max(turn);
                    (id, mesh)
                }
                None => return Ok(replay),
            },
            Built::Chamfer {
                id,
                body: from,
                edges,
                distance,
            } => match bevel(
                &mut replay,
                id,
                from,
                edges,
                *distance,
                false,
                facets,
                resolution,
            )? {
                Some((mesh, turn)) => {
                    facets.largest_picked_turn_degrees =
                        facets.largest_picked_turn_degrees.max(turn);
                    (id, mesh)
                }
                None => return Ok(replay),
            },
        };
        if let Built::Fillet { body: from, .. } | Built::Chamfer { body: from, .. } = step {
            if Some(index) == last_blend {
                let before = replay.bodies.get(from).map_or(f64::NAN, signed_volume);
                replay.last_blend_change = Some((before - signed_volume(&mesh)).abs());
            }
        }
        replay.bodies.insert(id.clone(), mesh);
        replay.facets.insert(id.clone(), facets);
    }
    Ok(replay)
}

/// One fillet or chamfer step through `bevel_part`: the beveled mesh and the largest turn it
/// picked, or `None` with the refusal recorded on the replay. `source_facets` are the body's
/// before this blend.
#[allow(clippy::too_many_arguments)]
fn bevel(
    replay: &mut Replay,
    id: &str,
    from: &str,
    edges: &[crate::case_file::P3],
    size: f64,
    round: bool,
    source_facets: Facets,
    resolution: Resolution,
) -> Result<Option<(Mesh, f64)>, String> {
    let source = Arc::new(body(&replay.bodies, from)?.clone());
    // A point on the exact curve stands off its inscribed chords by at most the sagitta of the
    // curves and fillets already on the body.
    let sagitta = source_facets.sagitta(resolution);
    let points: Vec<Vector3> = edges.iter().map(|p| v3(*p)).collect();
    let (groups, turn) = picks_for(&source, &points, 2.0 * sagitta + 1e-3)?;
    let request = BevelPartRequest::new(
        Some(source),
        groups,
        size,
        resolution.bevel_segments,
        round,
        BEND_THRESHOLD_DEGREES,
    );
    let outcome = bevel_part(request, &CancelToken::new(), None)
        .map_err(|e| format!("bevel_part could not run: {e:?}"))?;

    let mut codes = Vec::new();
    let mut largest_size_that_fits = None;
    for row in outcome.group_results.iter().flatten() {
        if let Some(failure) = &row.failure {
            codes.push(format!("{:?}", failure.code()));
        } else if row.partly_refused() {
            codes.push("PartlyRefused".to_string());
        }
        if row.largest_size_that_fits.is_some() {
            largest_size_that_fits = row.largest_size_that_fits;
        }
    }
    match outcome.mesh {
        Some(mesh) if codes.is_empty() => Ok(Some((mesh, turn))),
        mesh => {
            if mesh.is_none() && codes.is_empty() {
                codes.push("NoMesh".to_string());
            }
            codes.sort();
            codes.dedup();
            replay.refusal = Some(Refusal {
                step: id.to_string(),
                codes,
                largest_size_that_fits,
            });
            Ok(None)
        }
    }
}
