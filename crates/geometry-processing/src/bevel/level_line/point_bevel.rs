//! Point mode's build (`Bevel/LevelLine/PointBevel.cs`: `PointBevel`, `PointBevelKind`,
//! `PointBevelRefusal`, `PointBevelResult`, `PointBevelBuild` and the private `PointShape`): a
//! picked vertex bevelled on its own, the one-hole design with no chains. The kernel's level-line
//! cut runs a [`PointField`] (sphere for a fillet, plane for a chamfer), removes the cap, and the
//! one boundary loop it leaves is closed by [`hole_fill`] (fillet, cross-tangents down the surface
//! toward the point) or [`flat_hole_fill`] (chamfer). The star each point is read from is
//! [`PointStar`].
//!
//! C#'s exceptions out of the cut (the work budget, a cancel, a mesh that refused an edit) leave
//! as [`PointBevelFault`].

use std::cell::RefCell;
use std::rc::Rc;

use vector_math::number_format::format_f64_max_decimals;
use vector_math::Vector3;

use super::band_cut::{self, BandCutError};
use super::band_refinement::{self, RefineError, RefineOptions};
use super::point_star::PointStar;
use super::vertex_side::NegativeSnap;
use super::{
    edge_root, flat_hole_fill, hole_fill, ChainDistanceField, HoleFillFailure, LevelLineMesh,
    LevelLineMeshError, PointField, ScalarField, VertexFieldReading, WorkError,
};

/// C# `PointBevelKind`: a point bevel's profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PointBevelKind {
    Fillet,
    Chamfer,
}

/// C# `PointBevelRefusal`: why a point bevel refused; `None` when it built.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum PointBevelRefusal {
    #[default]
    None,
    /// A fillet asked for at a point where sharp (feature) edges meet: the edges are filleted
    /// instead.
    PointHasSharpEdges,
    /// The size is past the room.
    TooLarge,
    /// The point's faces give the bevel no shape (no inscribed ball, no convex cut, a size not
    /// positive).
    NoShape,
    /// The cut did not leave one ring, or the fill could not close it.
    DidNotClose,
}

/// C# `PointBevelResult`: what a point bevel built, or why it refused.
#[derive(Debug, Clone)]
pub struct PointBevelResult {
    /// Why it refused, by name; `None` when built.
    pub failure: PointBevelRefusal,
    /// The bevelled part (removed triangles flagged, the fill appended), or `None` on a refusal.
    pub mesh: Option<Rc<RefCell<LevelLineMesh>>>,
    /// `None` when built; otherwise why not, in words for the user.
    pub refusal: Option<String>,
    /// The largest size this point takes (see [`room`]).
    pub room: f64,
}

impl PointBevelResult {
    fn built(mesh: Rc<RefCell<LevelLineMesh>>, room: f64) -> Self {
        PointBevelResult {
            failure: PointBevelRefusal::None,
            mesh: Some(mesh),
            refusal: None,
            room,
        }
    }

    fn refused(failure: PointBevelRefusal, refusal: impl Into<String>, room: f64) -> Self {
        PointBevelResult {
            failure,
            mesh: None,
            refusal: Some(refusal.into()),
            room,
        }
    }

    /// C# `Built`.
    pub fn is_built(&self) -> bool {
        self.refusal.is_none()
    }
}

/// C# `PointBevelBuild`: what a several-point [`build`] made.
#[derive(Debug, Clone)]
pub struct PointBevelBuild {
    /// The part with every built point bevelled, or `None` when none built.
    pub mesh: Option<Rc<RefCell<LevelLineMesh>>>,
    /// Each point's result, in the order asked; a built one's mesh is [`PointBevelBuild::mesh`].
    pub points: Vec<PointBevelResult>,
}

/// How a point bevel leaves by a fault: C#'s exceptions out of the cut.
#[derive(Debug, Clone)]
pub enum PointBevelFault {
    /// C# `WorkBudgetExceededException` / `WorkCanceledException`.
    Work(WorkError),
    /// Anything else C# would throw, named by its message.
    Defect(String),
}

impl std::fmt::Display for PointBevelFault {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PointBevelFault::Work(work) => write!(f, "{work}"),
            PointBevelFault::Defect(message) => f.write_str(message),
        }
    }
}

impl std::error::Error for PointBevelFault {}

impl From<WorkError> for PointBevelFault {
    fn from(error: WorkError) -> Self {
        PointBevelFault::Work(error)
    }
}

impl From<LevelLineMeshError> for PointBevelFault {
    fn from(error: LevelLineMeshError) -> Self {
        match error {
            LevelLineMeshError::Work(work) => PointBevelFault::Work(work),
            other => PointBevelFault::Defect(other.to_string()),
        }
    }
}

impl From<RefineError> for PointBevelFault {
    fn from(error: RefineError) -> Self {
        match error {
            RefineError::Mesh(mesh) => PointBevelFault::from(mesh),
            other => PointBevelFault::Defect(other.to_string()),
        }
    }
}

impl From<BandCutError> for PointBevelFault {
    fn from(error: BandCutError) -> Self {
        match error {
            BandCutError::Mesh(mesh) => PointBevelFault::from(mesh),
            other => PointBevelFault::Defect(format!("{other:?}")),
        }
    }
}

impl From<NegativeSnap> for PointBevelFault {
    fn from(error: NegativeSnap) -> Self {
        PointBevelFault::Defect(format!("the snap {} is negative", error.0))
    }
}

/// How near the ball (as a fraction of r) every ring point must be for the round to be set onto the
/// ball: a faceted cone's ring leaves each facet's tangent point by a few percent at 40 sides.
const BALL_FIT: f64 = 0.05;

/// C# `Build(positions, triangles, vertex, size, ...)`: bevels `vertex` of the welded mesh at
/// `size` - the ball radius for a fillet, the setback along the vertex's edges for a chamfer.
/// `segments` sets the fillet's density.
pub fn build_one(
    positions: &[Vector3],
    triangles: &[(i32, i32, i32)],
    vertex: i32,
    size: f64,
    kind: PointBevelKind,
    segments: i32,
    bend_threshold_degrees: f64,
) -> Result<PointBevelResult, PointBevelFault> {
    let mut built = build(
        positions,
        triangles,
        &[(vertex, size)],
        kind,
        segments,
        bend_threshold_degrees,
    )?;
    Ok(built.points.swap_remove(0))
}

/// C# `Build(positions, triangles, points, ...)`: bevels every one of `points` (a vertex of the
/// welded mesh and its size) in one build, each read on the mesh as given - so no point is looked
/// for on another's cut. A point whose ring would reach another point's ring is refused by size
/// (`TooLarge`), with the room beside the other's ring, and so is the other: neither is built first
/// at the other's expense. The rest are built.
pub fn build(
    positions: &[Vector3],
    triangles: &[(i32, i32, i32)],
    points: &[(i32, f64)],
    kind: PointBevelKind,
    segments: i32,
    bend_threshold_degrees: f64,
) -> Result<PointBevelBuild, PointBevelFault> {
    let mut results: Vec<Option<PointBevelResult>> = vec![None; points.len()];
    let mut shapes: Vec<Option<PointShape>> = vec![None; points.len()];
    for (i, &(vertex, size)) in points.iter().enumerate() {
        let mut star = PointStar::new(positions, triangles, vertex, bend_threshold_degrees);

        // Lars, 2026-09-24: a point is filleted only where no sharp edge meets it (a cone's tip).
        // At a cube corner the ball touches each face at a single point, so no round can meet the
        // faces smoothly there and stay in the part; filleting the edges is what rounds such a
        // corner. A chamfer cuts cleanly at every candidate.
        if kind == PointBevelKind::Fillet && star.has_sharp_edges() {
            results[i] = Some(PointBevelResult::refused(
                PointBevelRefusal::PointHasSharpEdges,
                "Sharp edges meet this point - fillet the edges instead.",
                0.0,
            ));
            continue;
        }

        let (per_size, inward) = match star.try_shape(kind) {
            Ok(shape) => shape,
            Err(refusal) => {
                results[i] = Some(PointBevelResult::refused(
                    PointBevelRefusal::NoShape,
                    refusal,
                    0.0,
                ));
                continue;
            }
        };

        let room = star.room(kind, per_size, inward);
        #[allow(clippy::neg_cmp_op_on_partial_ord)] // C#'s `!(size > 0)`, NaN included.
        if !(size > 0.0) {
            results[i] = Some(PointBevelResult::refused(
                PointBevelRefusal::NoShape,
                "the size must be positive",
                room,
            ));
            continue;
        }

        let reach = star.ring_reach(kind, per_size, inward);
        shapes[i] = Some(PointShape {
            point: positions[vertex as usize],
            size,
            star,
            per_size,
            inward,
            room,
            reach,
        });
    }

    // The room beside another point is the size at which this ring would reach the other's, both
    // read as the ball each ring stays within - a lower bound where the caps are not round, never
    // an overlap let through.
    let mut to_build: Vec<usize> = Vec::new();
    for i in 0..points.len() {
        let Some(shape) = &shapes[i] else {
            continue;
        };

        let mut room = shape.room;
        for (j, other) in shapes.iter().enumerate() {
            if let (true, Some(other)) = (j != i, other) {
                let gap = (shape.point - other.point).length() - other.size * other.reach;
                room = room.min(gap.max(0.0) / shape.reach);
            }
        }

        // exactly the room builds; the relative slack is round-off in reading the room back, not a
        // tolerance
        if shape.size > room * (1.0 + 1e-12) {
            results[i] = Some(PointBevelResult::refused(
                PointBevelRefusal::TooLarge,
                format!(
                    "too large: this point has room for {}",
                    format_f64_max_decimals(room, 3)
                ),
                room,
            ));
            continue;
        }

        if let Some(shape) = shapes[i].as_mut() {
            shape.room = room;
        }
        to_build.push(i);
    }

    // Each ring is cut on the mesh the others left, which reads the same as the part there: the
    // rings are apart, so no cut reaches another's triangles. A point whose ring will not close is
    // dropped and the build starts again without it, rather than leave its half-made cut in the
    // part.
    while !to_build.is_empty() {
        let mut mesh = LevelLineMesh::from_triangles(positions, triangles, None)?;
        let same_place = to_build
            .iter()
            .filter_map(|&i| shapes[i].as_ref())
            .map(|shape| snap(shape.size * shape.per_size))
            .fold(f64::NEG_INFINITY, f64::max);
        mesh.set_same_place(same_place)
            .map_err(|bad| PointBevelFault::Defect(format!("{bad:?}")))?;
        let mesh = Rc::new(RefCell::new(mesh));

        let mut failed = None;
        for &i in &to_build {
            let Some(shape) = shapes[i].as_ref() else {
                continue;
            };
            if let Some(refusal) = cut(&mesh, shape, kind, segments)? {
                results[i] = Some(PointBevelResult::refused(
                    PointBevelRefusal::DidNotClose,
                    refusal,
                    shape.room,
                ));
                failed = Some(i);
                break;
            }
        }

        let Some(failed) = failed else {
            for &i in &to_build {
                let room = shapes[i].as_ref().map_or(0.0, |shape| shape.room);
                results[i] = Some(PointBevelResult::built(Rc::clone(&mesh), room));
            }

            return Ok(PointBevelBuild {
                mesh: Some(mesh),
                points: results.into_iter().map(Option::unwrap_or_default).collect(),
            });
        };

        if let Some(at) = to_build.iter().position(|&i| i == failed) {
            to_build.remove(at);
        }
    }

    Ok(PointBevelBuild {
        mesh: None,
        points: results.into_iter().map(Option::unwrap_or_default).collect(),
    })
}

impl Default for PointBevelResult {
    /// Never published: every asked point is given a result above before the build answers.
    fn default() -> Self {
        PointBevelResult::refused(PointBevelRefusal::NoShape, "", 0.0)
    }
}

/// C# `Snap`.
fn snap(setback: f64) -> f64 {
    1e-9 * setback.max(1.0)
}

/// C# `Cut`: cuts one point's ring into `mesh` and fills it; the refusal in words, or `None` when
/// built.
fn cut(
    mesh: &Rc<RefCell<LevelLineMesh>>,
    shape: &PointShape,
    kind: PointBevelKind,
    segments: i32,
) -> Result<Option<String>, PointBevelFault> {
    let point = shape.point;
    let size = shape.size;
    let setback = size * shape.per_size;
    let field = if kind == PointBevelKind::Fillet {
        PointField::sphere(point, setback)
    } else {
        PointField::plane(point, shape.inward, setback)
    };
    let snap = snap(setback);
    let as_field: Rc<dyn ScalarField> = Rc::new(field);
    let reading = VertexFieldReading::new(Rc::clone(mesh), as_field, snap)?;

    // Only a triangle the field reaches at or below the line can be cut, and the field answers its
    // exact least over a triangle, so the refinement is handed just those - not the whole part.
    // Every other triangle's field is positive throughout and it keeps all of itself.
    let mut candidates = Vec::new();
    {
        let mesh = mesh.borrow();
        for t in 0..mesh.triangle_count() {
            if mesh.is_removed(t) {
                continue;
            }

            let tri = mesh.triangles()[t as usize];
            let least = field.min_over_triangle(
                mesh.position(tri.a),
                mesh.position(tri.b),
                mesh.position(tri.c),
            )?;
            if least.value <= snap {
                candidates.push(t);
            }
        }
    }

    let (settled, _) = band_refinement::refine(
        &reading,
        &candidates,
        setback * 1e-3,
        RefineOptions::default(),
    )?;
    if !settled {
        return Ok(Some(
            "the ring could not be resolved on this mesh".to_string(),
        ));
    }

    let roots = edge_root::find(&mesh.borrow(), &reading)?;
    let cut = band_cut::cut(&reading, &roots)?;
    if cut.lines.len() != 1 || !cut.lines[0].closed {
        return Ok(Some(format!(
            "the ring is not one loop ({} lines)",
            cut.lines.len()
        )));
    }

    let mut mesh = mesh.borrow_mut();
    for &t in &cut.inside_triangles {
        mesh.remove_triangle(t);
    }

    let ring = fill_order(&mesh, &cut.lines[0].vertices)?;
    let loop_points: Vec<Vector3> = ring.iter().map(|&v| mesh.position(v)).collect();
    if kind == PointBevelKind::Chamfer {
        let Some((flat, _)) = flat_hole_fill::fill(&loop_points, snap, None) else {
            return Ok(Some("the chamfer ring does not close flat".to_string()));
        };

        for t in flat {
            mesh.add_face(
                ring[t.a as usize],
                ring[t.b as usize],
                ring[t.c as usize],
                0,
            )?;
        }

        return Ok(None);
    }

    let mut cross_tangents = Vec::with_capacity(ring.len());
    let mut edge_lengths = Vec::with_capacity(ring.len());
    let arc_row = setback * std::f64::consts::PI / (2.0 * segments.max(1) as f64);
    let n = ring.len();
    for i in 0..n {
        let q = loop_points[i];
        let to_point = point - q;
        let normal = kept_normal_at(&mesh, ring[i], ring[(i + 1) % n])?;
        cross_tangents.push((to_point - normal * normal.dot(to_point)).get_normal());
        let previous = loop_points[(i + n - 1) % n];
        let next = loop_points[(i + 1) % n];
        edge_lengths.push(arc_row.min(((q - previous).length() + (next - q).length()) / 2.0));
    }

    let fill = hole_fill::fill(&loop_points, &cross_tangents, &edge_lengths)?;
    if fill.failure != HoleFillFailure::None {
        return Ok(Some(format!(
            "the round would not close ({:?})",
            fill.failure
        )));
    }

    // Where the ring lies on the inscribed ball (a right cone: the ball is tangent along it), the
    // round IS that ball, so the faired interior - a thin plate, about 20% of r flat of it on a
    // cone - is set onto it. Where no ring lies on one ball the fair patch is the approximation,
    // and is kept as it is.
    let centre = point + shape.star.ball_offset() * size;
    let on_the_ball = loop_points
        .iter()
        .all(|&q| ((q - centre).length() - size).abs() <= BALL_FIT * size);
    let mut ids = ring.clone();
    for &at in &fill.vertices[ring.len()..] {
        ids.push(mesh.add_vertex(if on_the_ball {
            centre + (at - centre).get_normal() * size
        } else {
            at
        }));
    }

    for t in &fill.triangles {
        mesh.add_face(ids[t.a as usize], ids[t.b as usize], ids[t.c as usize], 0)?;
    }

    Ok(None)
}

/// C# `Room`: the largest size `vertex` takes - where the ring first reaches a feature.
pub fn room(
    positions: &[Vector3],
    triangles: &[(i32, i32, i32)],
    vertex: i32,
    kind: PointBevelKind,
    bend_threshold_degrees: f64,
) -> f64 {
    let mut star = PointStar::new(positions, triangles, vertex, bend_threshold_degrees);
    match star.try_shape(kind) {
        Ok((per_size, inward)) => star.room(kind, per_size, inward),
        Err(_) => 0.0,
    }
}

/// C# `FillOrder`: the ring in the order the fill wants - the kept surface runs its boundary one
/// way, and the fills want it walked the other way.
fn fill_order(mesh: &LevelLineMesh, ring: &[i32]) -> Result<Vec<i32>, PointBevelFault> {
    let mut ring = ring.to_vec();
    if ring.len() > 1 && ring[0] == ring[ring.len() - 1] {
        ring.pop();
    }

    let kept = the_single_triangle_on(mesh, ring[0], ring[1])?;
    let t = mesh.triangles()[kept as usize];
    for k in 0..3 {
        if t.corner(k) == ring[0] && t.corner((k + 1) % 3) == ring[1] {
            ring.reverse();
            break;
        }
    }

    Ok(ring)
}

/// C# `KeptNormalAt`.
fn kept_normal_at(mesh: &LevelLineMesh, a: i32, b: i32) -> Result<Vector3, PointBevelFault> {
    let t = mesh.triangles()[the_single_triangle_on(mesh, a, b)? as usize];
    let p = mesh.position(t.a);
    Ok((mesh.position(t.b) - p)
        .cross(mesh.position(t.c) - p)
        .get_normal())
}

/// C#'s `TrianglesOn(a, b).Single()`: the one kept triangle on a boundary edge, or the fault C#'s
/// LINQ throws (divergence 5036).
fn the_single_triangle_on(mesh: &LevelLineMesh, a: i32, b: i32) -> Result<i32, PointBevelFault> {
    match mesh.triangles_on(a, b) {
        [only] => Ok(*only),
        _ => Err(PointBevelFault::Defect(
            "Sequence contains no elements or more than one element".to_string(),
        )),
    }
}

/// C# `PointShape`: a point read for the build - where it is, its size, its star's shape and its
/// room.
#[derive(Debug, Clone)]
struct PointShape {
    point: Vector3,
    size: f64,
    star: PointStar,
    per_size: f64,
    inward: Vector3,
    /// Its own room, then the room beside the other points too.
    room: f64,
    /// How far the ring reaches from the point per unit of size ([`PointStar::ring_reach`]).
    reach: f64,
}
