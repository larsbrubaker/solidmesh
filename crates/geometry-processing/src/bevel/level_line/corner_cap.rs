//! The hole round a corner vertex V after the strips are laid: k last rows (one per chain end at
//! V) alternating with k short rail pieces, possibly a single tie vertex each, closed about the
//! corner's AXIS - the normalised sum of the distinct face normals round V: a chamfer's by a fan, a
//! fillet's by rings. Ports the nested type of `Bevel/LevelLine/CornerCap.cs` (a C# static partial
//! class): a strip's end spoke ([`CornerCapLastRow`], with its constructor) and the corner's axis
//! ([`axis`]). The caps' other functions land with later packages.

use vector_math::Vector3;

use super::gap_fill::FILL_TAG;
use super::{corner_fill, LevelLineMesh};

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
