//! The refinement that splits the band's triangles until the level line runs through them as
//! chords: its depth bound, a
//! triangle's span, the meter's nearness readings, [`refine`] with the Lipschitz halving and the
//! readings it asks ([`line_points_of`], [`wants_splitting`], [`hides_the_line`]). The refinement
//! by asking is `band_refinement_asking`, its mixed edge rule `band_refinement_mixed_edge`.
//!
//! The mesh the refinement edits is always the reading's own, so the refinement takes the reading
//! and edits its mesh, borrowing it only between reads.

use solidmesh_math::{ieee_math, Vector3};

/// How many times one triangle may be halved before the refinement gives up rather than loop.
/// This bounds the LIPSCHITZ halving only: the asking refinement - every field that answers as a
/// chain distance field, which is every real build - never reads it, and its only bound is the
/// deterministic work budget. Do not reason from 24 generations as if it limited a real build.
pub const DEFAULT_MAX_DEPTH: i32 = 24;

/// The longest side of a triangle - what a bounded search over it has to halve away.
pub fn span(a: Vector3, b: Vector3, c: Vector3) -> f64 {
    ieee_math::max_f64(
        (a - b).length(),
        ieee_math::max_f64((b - c).length(), (c - a).length()),
    )
}

/// How many SIZES from the level line the NEAREST of two corners stands - the least absolute field
/// value over the corners, against the size, which is twenty chord tolerances. A meter only.
pub fn nearest_to_the_line(chord_tolerance: f64, a: f64, b: f64) -> f64 {
    ieee_math::min_f64(a.abs(), b.abs()) / (20.0 * chord_tolerance)
}

/// [`nearest_to_the_line`] over three corners.
pub fn nearest_to_the_line3(chord_tolerance: f64, a: f64, b: f64, c: f64) -> f64 {
    ieee_math::min_f64(a.abs(), ieee_math::min_f64(b.abs(), c.abs())) / (20.0 * chord_tolerance)
}

/// How many sizes from the line the FARTHEST of two corners stands.
pub fn farthest_from_the_line(chord_tolerance: f64, a: f64, b: f64) -> f64 {
    ieee_math::max_f64(a.abs(), b.abs()) / (20.0 * chord_tolerance)
}

/// [`farthest_from_the_line`] over three corners.
pub fn farthest_from_the_line3(chord_tolerance: f64, a: f64, b: f64, c: f64) -> f64 {
    ieee_math::max_f64(a.abs(), ieee_math::max_f64(b.abs(), c.abs())) / (20.0 * chord_tolerance)
}

/// Why the refinement could not run.
#[derive(Debug, Clone)]
pub enum RefineError {
    /// The work budget ran out, the build was cancelled, or the mesh refused an edit.
    Mesh(super::LevelLineMeshError),
    /// The chord tolerance or the Lipschitz constant is not positive, with the message saying
    /// which.
    OutOfRange(String),
    /// The halving was asked to split an edge no live triangle stands on, so there is no parent
    /// depth to read.
    NoParents(String),
}

impl From<super::LevelLineMeshError> for RefineError {
    fn from(error: super::LevelLineMeshError) -> Self {
        RefineError::Mesh(error)
    }
}

impl From<super::WorkError> for RefineError {
    fn from(error: super::WorkError) -> Self {
        RefineError::Mesh(error.into())
    }
}

impl std::fmt::Display for RefineError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RefineError::Mesh(error) => write!(f, "{error}"),
            RefineError::OutOfRange(message) | RefineError::NoParents(message) => {
                f.write_str(message)
            }
        }
    }
}

impl std::error::Error for RefineError {}

/// The optional arguments of [`refine`], defaulting to the usual values.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RefineOptions {
    /// The field's Lipschitz constant (default 1).
    pub lipschitz: f64,
    /// Read by the Lipschitz halving only (default [`DEFAULT_MAX_DEPTH`]).
    pub max_depth: i32,
    /// Switches OFF the chord accuracy rule, and only that one (default false).
    pub topology_only: bool,
}

impl Default for RefineOptions {
    fn default() -> Self {
        RefineOptions {
            lipschitz: 1.0,
            max_depth: DEFAULT_MAX_DEPTH,
            topology_only: false,
        }
    }
}

/// Refines the candidate triangles of the reading's mesh in place.
/// Answers `(true, -1)` when every triangle settled, and `(false, triangle)` - naming the triangle
/// that would not settle - rather than looping or quietly accepting a triangle it could not make
/// good. A field that can be asked where it runs is refined by asking
/// (`band_refinement_asking`); any other by Lipschitz halving.
pub fn refine(
    reading: &super::VertexFieldReading,
    candidate_triangles: &[i32],
    chord_tolerance: f64,
    options: RefineOptions,
) -> Result<(bool, i32), RefineError> {
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    if !(chord_tolerance > 0.0) {
        return Err(RefineError::OutOfRange(
            "the chord tolerance is a length, so it must be positive (Parameter 'chordTolerance')"
                .to_string(),
        ));
    }

    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    if !(options.lipschitz > 0.0) {
        return Err(RefineError::OutOfRange(
            "the field's Lipschitz constant must be positive (Parameter 'lipschitz')".to_string(),
        ));
    }

    let field = reading.field().clone();
    if let Some(asked) = field.as_chain_distance_field() {
        return super::band_refinement_asking::refine_by_asking(
            reading,
            asked,
            candidate_triangles,
            chord_tolerance,
            options.topology_only,
        );
    }

    let mut depth: std::collections::HashMap<i32, i32> = std::collections::HashMap::new();
    let mut taken = 0i64;
    let distinct: indexmap::IndexSet<i32> = candidate_triangles.iter().copied().collect();
    let mut pending: std::collections::VecDeque<i32> = distinct.into_iter().collect();
    while let Some(triangle) = pending.pop_front() {
        taken += 1;
        super::LevelLineWork::note_refinement_queue(taken, pending.len() as i64);
        super::LevelLineWork::charge(1)?;
        if triangle >= reading.mesh.borrow().triangle_count() {
            continue;
        }

        if !wants_splitting(reading, triangle, chord_tolerance, options.lipschitz)? {
            continue;
        }

        let mut mesh = reading.mesh.borrow_mut();
        let (a, b) = super::refinement_splits::longest_edge(&mesh, triangle);
        let Some(parent_depth) = mesh
            .triangles_on(a, b)
            .iter()
            .map(|p| depth.get(p).copied().unwrap_or(0))
            .max()
        else {
            return Err(RefineError::NoParents(
                "Sequence contains no elements".to_string(),
            ));
        };
        if parent_depth + 1 > options.max_depth {
            return Ok((false, triangle));
        }

        // Asked as "is the answer a vertex that was already there": a new vertex is always the
        // next id. A refused midpoint divided nothing, so the parents are not asked again.
        let before = mesh.vertex_count();
        let middle = 0.5 * (mesh.position(a) + mesh.position(b));
        let mid = mesh.split_edge(a, b, middle, true)?;
        if mid < before {
            continue;
        }

        let mut children: Vec<i32> = mesh.triangles_on(a, mid).to_vec();
        children.extend_from_slice(mesh.triangles_on(mid, b));
        let children: indexmap::IndexSet<i32> = children.into_iter().collect();
        for child in children {
            depth.insert(child, parent_depth + 1);
            pending.push_back(child);
        }
    }

    Ok((true, -1))
}

/// The line points of a crossed triangle of the reading's mesh -
/// each corner on the line, then each crossed edge's root. Fewer or more than two means the cut's
/// own table handles it, and the chord test stands aside.
pub fn line_points_of(
    reading: &super::VertexFieldReading,
    triangle: i32,
) -> Result<Vec<Vector3>, super::WorkError> {
    let t = reading.mesh.borrow().triangles()[triangle as usize];
    let position = |v: i32| reading.mesh.borrow().position(v);
    let mut points = Vec::new();
    for corner in 0..3 {
        let v = t.corner(corner);
        if reading.side_of(v)? == super::VertexSide::On {
            points.push(position(v));
        }
    }

    for corner in 0..3 {
        let v = t.corner(corner);
        let w = t.corner((corner + 1) % 3);
        let side_v = reading.side_of(v)?;
        let side_w = reading.side_of(w)?;
        if side_v != super::VertexSide::On && side_w != super::VertexSide::On && side_v != side_w {
            points.push(super::edge_root::root_on(
                position(v),
                position(w),
                reading,
            )?);
        }
    }

    Ok(points)
}

/// The Lipschitz halving's test - an uncrossed edge longer
/// than its ends' reach (and than the chord tolerance) may hide a pair of crossings; a triangle the
/// line touches nowhere may hide a loop ([`hides_the_line`]); a crossed triangle's chord midpoint
/// stands off the line by more than the tolerance.
pub fn wants_splitting(
    reading: &super::VertexFieldReading,
    triangle: i32,
    chord_tolerance: f64,
    lipschitz: f64,
) -> Result<bool, super::WorkError> {
    let t = reading.mesh.borrow().triangles()[triangle as usize];
    for corner in 0..3 {
        let v = t.corner(corner);
        let w = t.corner((corner + 1) % 3);
        let side_v = reading.side_of(v)?;
        let side_w = reading.side_of(w)?;
        if side_v == super::VertexSide::On || side_w == super::VertexSide::On || side_v != side_w {
            continue;
        }

        let length = reading.mesh.borrow().length(v, w);
        let reach = (reading.value_at(v)?.abs() + reading.value_at(w)?.abs()) / lipschitz;
        if length > reach && length > chord_tolerance {
            return Ok(true);
        }
    }

    let points = line_points_of(reading, triangle)?;
    if points.is_empty() {
        return hides_the_line(reading, triangle, chord_tolerance);
    }

    if points.len() != 2 {
        return Ok(false);
    }

    let chord_mid = 0.5 * (points[0] + points[1]);
    Ok(reading.value_at_point(chord_mid)?.abs() > chord_tolerance
        && (points[0] - points[1]).length() > chord_tolerance)
}

/// Whether a triangle that touches the line nowhere on its
/// boundary could still hide it in its middle. The field is 1-Lipschitz, so a corner reading
/// further from the line than the triangle reaches from it proves the triangle clear; otherwise it
/// is halved while its longest side is over the chord tolerance.
pub fn hides_the_line(
    reading: &super::VertexFieldReading,
    triangle: i32,
    chord_tolerance: f64,
) -> Result<bool, super::WorkError> {
    let t = reading.mesh.borrow().triangles()[triangle as usize];
    let mut longest = 0.0;
    for corner in 0..3 {
        let v = t.corner(corner);
        let (near, far) = {
            let mesh = reading.mesh.borrow();
            (
                mesh.length(v, t.corner((corner + 1) % 3)),
                mesh.length(v, t.corner((corner + 2) % 3)),
            )
        };
        longest = ieee_math::max_f64(longest, near);
        if reading.value_at(v)?.abs() > ieee_math::max_f64(near, far) {
            return Ok(false);
        }
    }

    Ok(longest > chord_tolerance)
}
