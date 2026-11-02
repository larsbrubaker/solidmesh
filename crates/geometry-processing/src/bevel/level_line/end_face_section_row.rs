//! An end face's last row on a LEANING end face: the round's own section in that face's plane.
//!
//! An open fill whose ends are closed by end-face caps lays a last row at each end, from the left
//! rail's end to the right rail's, both on the end face; the cap between that row and the end run
//! the cut left is a flat piece of the end face (`corner_cap_zip_end`, closed flat by
//! `corner_cap_flat_ends::closes_flat`), so the row must lie in the face's plane. On a square end
//! the row is the round's square section, drawn by `open_gap_fill::arcs` as every other row is. On
//! a leaning end face the round's section is that square section carried along the chain into the
//! face: the round is swept along the chain, so near its end every point of the square section
//! slides along the chain's direction until it meets the end face. That slide is an affine map
//! from the square plane onto the end face, and a rational quadratic carried by an affine map is
//! the rational quadratic of its carried control points with the same weight, so the oblique row
//! is the square row's control point slid onto the face with the square row's weight - exactly the
//! section, never a row drawn in the oblique plane by that plane's own angle (which bows off the
//! round: 0.11 inside the round's cylinder on the groove leaning 8 degrees).
//!
//! The square plane is the one through the end vertex, and the square row is the level line's,
//! drawn about the end vertex. Before the slide, both rail ends are slid along the chain into that
//! square plane; the slide takes them back to themselves.
//!
//! Both the end vertex and the slide hold only on a FLAT end face ([`flat_end_faces`]): its end
//! run and the chain's end vertex in one plane within the mesh's snap, the same snap the flat cap
//! closes the hole within. On a curved end face (the D prism's rounded end, faceted round its
//! axis, whose rail on the top face ends on a facet 1.6 off the plane of the facet at the end
//! vertex) the row has no one face to lie in, and it is drawn as every other row is.
//!
//! A face within [`SQUARE_COSINE`] of square to the chain keeps the square row bit for bit, so no
//! square end moves by the projection's rounding.

use vector_math::Vector3;

use super::gap_fill::arc_point;
use super::{ChainField, LevelLineMesh};

/// How near the end face's normal may come to the chain's direction (the cosine between them)
/// before the face is read as square to the chain and its row is left to the square row's drawing:
/// within about 4.5e-5 radians of square.
pub const SQUARE_COSINE: f64 = 1.0 - 1e-9;

/// How far the end face may turn from square before its row is not read as the round's section:
/// a face whose normal stands within this cosine of square to the chain (the face nearly along the
/// chain) has no section the slide can reach.
const ALONG_THE_CHAIN_COSINE: f64 = 1e-6;

/// What the section row of one end asks: the end it closes and the chain's unit direction there.
#[derive(Clone, Copy, Debug)]
pub struct EndRowAsked {
    /// Whether the row closes the chain's far end (parameter `chain.length`) rather than its start.
    pub at_the_far_end: bool,
    /// The chain's unit direction at that end.
    pub tangent: Vector3,
}

/// The row from `p` to `q` (the two rails' ends on a leaning end face) as the round's section in
/// that face: a point for each share `f` from 0 at `p` to 1 at `q`. `None` where the face is
/// square to the chain within [`SQUARE_COSINE`], where it runs along the chain, or where `p`, `q`
/// and the end vertex span no plane; the caller then draws the row as it draws every other.
pub fn section_row(
    chain: &ChainField,
    p: Vector3,
    q: Vector3,
    asked: EndRowAsked,
) -> Option<impl Fn(f64) -> Vector3> {
    let end = if asked.at_the_far_end {
        chain.length
    } else {
        0.0
    };
    let v = chain.point_at(end);
    let t = asked.tangent;
    let normal = (p - v).cross(q - v);
    let length = normal.length();
    // Zero, or not a number: no plane.
    if length.is_nan() || length <= 0.0 || t.length_squared() == 0.0 {
        return None;
    }
    let n = normal * (1.0 / length);
    let lean = t.dot(n);
    if lean.abs() >= SQUARE_COSINE || lean.abs() <= ALONG_THE_CHAIN_COSINE {
        return None;
    }

    // The square plane through the end vertex; e is the chain's line where it crosses that plane.
    // Read as v plus the offset along t rather than v itself, so every bit stays as it was (a -0.0
    // component of v comes out +0.0 this way).
    let reference = v;
    let e = v + (t * (reference - v).dot(t));
    let into_the_square_plane = |x: Vector3| x - (t * (x - e).dot(t));
    let (p_square, q_square) = (into_the_square_plane(p), into_the_square_plane(q));
    let c_square = e;

    // Each point of the square row slides along the chain onto the end face.
    let onto_the_face = move |x: Vector3| x + (t * ((v - x).dot(n) / lean));
    Some(move |f: f64| onto_the_face(arc_point(p_square, c_square, q_square, f)))
}

/// Which of the chain's two ends close on a FLAT end face: the end run there (from one rail's end
/// to the other's) and the chain's end vertex lie in one plane, every run vertex within the mesh's
/// `same_place` of the plane through the run's two ends and the end vertex. That snap is the one
/// the flat cap (`corner_cap_flat_ends::closes_flat`) closes the hole within, so an end read flat
/// here is one the cap can close in its plane. A fade (no end runs), a run of fewer than two
/// vertices, or ends and vertex spanning no plane is not flat.
pub fn flat_end_faces(
    mesh: &LevelLineMesh,
    chain: &ChainField,
    end_runs: Option<&[Vec<i32>]>,
) -> [bool; 2] {
    let Some(end_runs) = end_runs else {
        return [false, false];
    };
    let snap = mesh.same_place();
    [0, 1].map(|e| {
        let Some(run) = end_runs.get(e) else {
            return false;
        };
        if run.len() < 2 {
            return false;
        }
        let v = chain.point_at(if e == 0 { 0.0 } else { chain.length });
        let (p, q) = (mesh.position(run[0]), mesh.position(run[run.len() - 1]));
        let normal = (p - v).cross(q - v);
        let length = normal.length();
        // Zero, or not a number: no plane.
        if length.is_nan() || length <= 0.0 {
            return false;
        }
        let n = normal * (1.0 / length);
        run.iter()
            .all(|&i| (mesh.position(i) - v).dot(n).abs() <= snap)
    })
}
