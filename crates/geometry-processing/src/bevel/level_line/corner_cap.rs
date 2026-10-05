//! The hole round a corner vertex V after the strips are laid: k last rows (one per chain end at
//! V) alternating with k short rail pieces, possibly a single tie vertex each, closed about the
//! corner's AXIS - the normalised sum of the distinct face normals round V: a chamfer's by a fan, a
//! fillet's by rings. Ports the nested type of `Bevel/LevelLine/CornerCap.cs` (a C# static partial
//! class): a strip's end spoke ([`CornerCapLastRow`], with its constructor and material centre),
//! the corner's axis ([`axis`]), the angle between two directions ([`angle`]) and the turn from one
//! to the other ([`toward`]). The caps' other functions land with later packages.

use vector_math::{cs_math, Vector3};

use super::gap_fill::FILL_TAG;
use super::{corner_fill, LevelLineMesh, LevelLineMeshError, LevelLineWork};

/// C# `CornerCap.LastRow`: a strip's end spoke as a fillet lays it - the arc's vertices, and the
/// circle it is drawn on, its radius the mean of the two legs.
///
/// The round's centre is where the sides' normals at the arc's two ends meet: on the bisector of
/// the two setbacks from the chain point. `p + q - c` is that only where the sides are square; at a
/// 45 degree knife tip it put the centre a setback out and read the radius as the setback, so no
/// test that asks the rows' radius (the ball, the mitre) could ever hold there.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CornerCapLastRow {
    /// The row's first end, on one side face.
    pub first: Vector3,
    /// The row's last end, on the other side face.
    pub last: Vector3,
    /// The unit normal of the plane through the row's two ends and its centre - its round's axis.
    pub plane_normal: Vector3,
    /// The chain's point the row's arc is drawn about - on the chain, beside the corner.
    pub chain_point: Vector3,
    /// True where the row ends a CONCAVE stretch: its ball rolls in the air, not the part.
    pub concave: bool,
    /// The arc's mesh vertices, first end to last.
    pub arc: Vec<i32>,
    /// The chain's unit direction at a SKEW row (its two ends cross the chain at different points),
    /// else zero. A skew row is not a plane arc, so its cross-tangent is the fillet's ruling - this -
    /// and not the normal of the plane through its ends and [`CornerCapLastRow::centre`].
    pub axis: Vector3,
    /// The centre of the circle the row is drawn on.
    pub centre: Vector3,
    /// The row's radius: the mean of its two legs from the centre.
    pub radius: f64,
}

impl CornerCapLastRow {
    /// C# `LastRow.MaterialCentre`: the centre on the MATERIAL side: a concave row's centre
    /// mirrored through its chain point, the centre the same row would be rolled by were its edge
    /// convex; a convex row's own centre.
    pub fn material_centre(&self) -> Vector3 {
        if self.concave {
            (self.chain_point * 2.0) - self.centre
        } else {
            self.centre
        }
    }

    /// The spoke whose arc is `arc` (its first and last vertices on the two sides), drawn from
    /// `chain_point`; C#'s optional `concave = false` and `axis = default` are passed explicitly.
    pub fn new(
        arc: Vec<i32>,
        chain_point: Vector3,
        mesh: &LevelLineMesh,
        concave: bool,
        axis: Vector3,
    ) -> Self {
        let p = mesh.position(arc[0]);
        let q = mesh.position(arc[arc.len() - 1]);

        // The round's centre is where the sides' normals at p and q meet: on the bisector of the
        // two setbacks from the chain point, c + (u + v) a / (1 + u.v) for unit side directions u,
        // v and equal setbacks a. p + q - c is that only where the sides are square (u.v = 0); at a
        // 45 degree knife tip it put the centre a setback out and read the radius as the setback,
        // 8.208 for a round of 3.4, so no test that asks the rows' radius could ever hold there.
        let u = p - chain_point;
        let v = q - chain_point;
        let cosine = if u.length() > 0.0 && v.length() > 0.0 {
            u.get_normal().dot(v.get_normal())
        } else {
            0.0
        };
        let centre = if cosine > -1.0 + 1e-9 {
            chain_point + ((u + v) / (1.0 + cosine))
        } else {
            p + q - chain_point
        };
        let radius = ((p - centre).length() + (q - centre).length()) / 2.0;
        let plane_normal = (p - centre).cross(q - centre).get_normal();
        Self {
            first: p,
            last: q,
            plane_normal,
            chain_point,
            concave,
            arc,
            axis,
            centre,
            radius,
        }
    }
}

/// The corner's AXIS at position `v`: the normalised sum of the distinct face normals of the kept
/// (non-fill) triangles touching it; zero where they cancel or there are none.
pub fn axis(mesh: &LevelLineMesh, v: Vector3) -> Vector3 {
    let mut normals: Vec<Vector3> = Vec::new();
    for face in mesh.triangles() {
        if face.tag == FILL_TAG {
            continue;
        }

        let a = mesh.position(face.a);
        let b = mesh.position(face.b);
        let c = mesh.position(face.c);
        if !corner_fill::same(a, v) && !corner_fill::same(b, v) && !corner_fill::same(c, v) {
            continue;
        }

        let mut normal = (b - a).cross(c - a);
        if normal.length_squared() == 0.0 {
            continue;
        }

        normal = normal.get_normal();
        if !normals.iter().any(|m| m.dot(normal) > 1.0 - 1e-9) {
            normals.push(normal);
        }
    }

    let sum = normals.iter().fold(Vector3::ZERO, |s, &m| s + m);
    if sum.length_squared() == 0.0 {
        Vector3::ZERO
    } else {
        sum.get_normal()
    }
}

/// A watcher of the end holes with the mesh and the chain's end vertex V: the (row, run) pair.
pub type WatchEndHoleAt = std::rc::Rc<dyn Fn(&LevelLineMesh, &[i32], &[i32], Vector3)>;

thread_local! {
    /// C# `[ThreadStatic] CornerCap.WatchEndHoleAt`: a read-only observer of the end holes, with
    /// the mesh and the chain's end vertex, so a measuring test can read the hole's shape. Per
    /// thread; `None` by default.
    pub static WATCH_END_HOLE_AT: std::cell::RefCell<Option<WatchEndHoleAt>> =
        const { std::cell::RefCell::new(None) };
}

/// C# `CornerCap.AxisAngleNoise`: the angle by which a run may step back about the row's axis and
/// still be read as rising (float noise in the angles).
const AXIS_ANGLE_NOISE: f64 = 1e-6;

/// C# `CornerCap.OnTheRound`: how near the row's radius, as a fraction of it, a run vertex must
/// stand from the row's axis to be read as on the round.
const ON_THE_ROUND: f64 = 1e-4;

/// C# `Math.IEEERemainder`, as .NET computes it.
fn ieee_remainder(x: f64, y: f64) -> f64 {
    if x.is_nan() {
        return x;
    }
    if y.is_nan() {
        return y;
    }

    let regular_mod = x % y;
    if regular_mod.is_nan() {
        return f64::NAN;
    }
    if regular_mod == 0.0 && x.is_sign_negative() {
        return -0.0;
    }

    let sign = if x > 0.0 {
        1.0
    } else if x < 0.0 {
        -1.0
    } else {
        0.0
    };
    let alternative = regular_mod - (y.abs() * sign);
    if alternative.abs() == regular_mod.abs() {
        let division = x / y;
        let rounded = division.round_ties_even();
        return if rounded.abs() > division.abs() {
            alternative
        } else {
            regular_mod
        };
    }

    if alternative.abs() < regular_mod.abs() {
        alternative
    } else {
        regular_mod
    }
}

/// C# `CornerCap.ZipsAboutTheRowsAxis`: a fillet's end hole zipped by angle about the row's own
/// axis - the line through the last row's circle centre normal to its plane - rather than about the
/// end vertex. True (and laid) when the row has an arc to read a centre from, the run lies on the
/// round's cylinder and rises from the row's start to its end about that axis; false (nothing laid)
/// otherwise.
pub fn zips_about_the_rows_axis(
    mesh: &mut LevelLineMesh,
    row: &[i32],
    run: &[i32],
    added: &mut Vec<i32>,
) -> Result<bool, LevelLineMeshError> {
    if row.len() < 3 || row[0] != run[0] || row[row.len() - 1] != run[run.len() - 1] {
        return Ok(false);
    }

    // The circle through the row's ends and its middle vertex.
    let p = mesh.position(row[0]);
    let q = mesh.position(row[row.len() / 2]);
    let r = mesh.position(row[row.len() - 1]);
    let pq = q - p;
    let pr = r - p;
    let normal = pq.cross(pr);
    let twice = 2.0 * normal.length_squared();
    if twice <= 1e-24 {
        return Ok(false);
    }

    let centre = p
        + (((normal.cross(pq) * pr.length_squared()) + (pr.cross(normal) * pq.length_squared()))
            / twice);
    let n = (p - centre).cross(r - centre);
    if n.length_squared() <= 1e-24 {
        return Ok(false);
    }

    let n = n.get_normal();

    // Only a hole whose whole boundary lies ON the round's cylinder is a patch of that round; a
    // run vertex may stand inside the round by as much as the row's widest chord sags.
    let radius = (p - centre).length();
    let from_the_axis = |point: Vector3| {
        let d = point - centre;
        (d - (n * d.dot(n))).length()
    };

    let mut sag = 0.0;
    for i in 0..row.len() - 1 {
        let mid = (mesh.position(row[i]) + mesh.position(row[i + 1])) / 2.0;
        sag = cs_math::max_f64(sag, radius - from_the_axis(mid));
    }

    for &vertex in run {
        let off = from_the_axis(mesh.position(vertex)) - radius;
        if off > ON_THE_ROUND * radius || off < -(sag + (ON_THE_ROUND * radius)) {
            return Ok(false);
        }
    }

    let u = (p - centre).get_normal();
    let w = n.cross(u);
    let angle_of = |mesh: &LevelLineMesh, vertex: i32| {
        let d = mesh.position(vertex) - centre;
        d.dot(w).atan2(d.dot(u))
    };

    let a: Vec<f64> = row.iter().map(|&i| angle_of(mesh, i)).collect();
    let b: Vec<f64> = run.iter().map(|&i| angle_of(mesh, i)).collect();
    for i in 0..a.len() - 1 {
        #[allow(clippy::neg_cmp_op_on_partial_ord)]
        let falls = !(a[i + 1] > a[i]);
        if falls {
            return Ok(false);
        }
    }

    for i in 0..b.len().saturating_sub(1) {
        if b[i + 1] < b[i] - AXIS_ANGLE_NOISE {
            return Ok(false);
        }
    }

    let (mut x, mut y) = (0, 0);
    while x < row.len() - 1 || y < run.len() - 1 {
        LevelLineWork::charge(1)?;
        let advances_row =
            y >= run.len() - 1 || (x < row.len() - 1 && a[x + 1] <= b[y + 1] + AXIS_ANGLE_NOISE);
        if advances_row {
            corner_fill::lay(mesh, row[x], row[x + 1], run[y], added)?;
            x += 1;
        } else {
            corner_fill::lay(mesh, run[y + 1], run[y], row[x], added)?;
            y += 1;
        }
    }

    Ok(true)
}

/// C# `CornerCap.FansAboutTheEnd`: the end face that wraps the end vertex `v` - the hole surrounds
/// `v`, star-shaped about it - fanned to a new vertex at `v` when the row (angles `a`) rises
/// strictly, the run (angles `b`) falls strictly to the row's end angle less one full turn, and
/// both lie in the end face's plane (`normal`). False (nothing laid) otherwise.
#[allow(clippy::too_many_arguments)]
pub fn fans_about_the_end(
    mesh: &mut LevelLineMesh,
    row: &[i32],
    run: &[i32],
    a: &[f64],
    b: &[f64],
    v: Vector3,
    normal: Vector3,
    added: &mut Vec<i32>,
    added_vertices: &mut Vec<i32>,
) -> Result<bool, LevelLineMeshError> {
    // A fan to V is flat only if its boundary is (relative 1e-6: walls are flat to float noise).
    // LINQ's Max: NaN reads as the smallest; an empty boundary throws (as the C# does).
    let mut reach: Option<f64> = None;
    for &i in row.iter().chain(run) {
        let x = (mesh.position(i) - v).length();
        reach = Some(match reach {
            Some(value) if !(x > value || value.is_nan()) => value,
            _ => x,
        });
    }
    let reach = reach.expect("the end hole has a boundary");
    if row
        .iter()
        .chain(run)
        .any(|&i| (mesh.position(i) - v).dot(normal).abs() > 1e-6 * reach)
    {
        return Ok(false);
    }

    let tie = 1e-12;
    for i in 0..a.len().saturating_sub(1) {
        if a[i + 1] <= a[i] + tie {
            return Ok(false);
        }
    }

    // The run is unwrapped by continuity: every step is read as the turn in (-pi, pi] it makes.
    let mut unwrapped = b[0];
    for i in 0..b.len() - 1 {
        let step = ieee_remainder(b[i + 1] - b[i], 2.0 * std::f64::consts::PI);
        if step >= -tie {
            return Ok(false);
        }

        unwrapped += step;
    }

    if (unwrapped - (a[a.len() - 1] - (2.0 * std::f64::consts::PI))).abs() > 1e-9 {
        return Ok(false);
    }

    let centre = mesh.add_vertex(v);
    added_vertices.push(centre);
    for x in 0..row.len().saturating_sub(1) {
        LevelLineWork::charge(1)?;
        corner_fill::lay(mesh, row[x], row[x + 1], centre, added)?;
    }

    for y in 0..run.len().saturating_sub(1) {
        LevelLineWork::charge(1)?;
        corner_fill::lay(mesh, run[y + 1], run[y], centre, added)?;
    }

    Ok(true)
}

/// C# `CornerCap.Angle`: the angle between two directions, by atan2 of their cross and dot.
pub fn angle(a: Vector3, b: Vector3) -> f64 {
    a.cross(b).length().atan2(a.dot(b))
}

/// C# `CornerCap.Toward`: the point `t` of the way from `from` to `to` about the origin, turning in
/// their common plane, its length running linearly between theirs.
pub fn toward(from: Vector3, to: Vector3, angle: f64, t: f64) -> Vector3 {
    let length = ((1.0 - t) * from.length()) + (t * to.length());
    if angle <= 1e-12 {
        return ((from * (1.0 - t)) + (to * t)).get_normal() * length;
    }

    let sin = angle.sin();
    let direction = (from.get_normal() * (((1.0 - t) * angle).sin() / sin))
        + (to.get_normal() * ((t * angle).sin() / sin));
    direction.get_normal() * length
}
