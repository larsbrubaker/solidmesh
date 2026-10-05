//! Which of a cut's new vertices stand along a stretch of a mesh ([`SurfaceSplitsOnASegment`]) -
//! asked of the stretch's two END VERTICES and of nothing else, so that every face meeting along
//! one stretch is told the same thing about it. Ports the fields of
//! `Bevel/Surface/SurfaceSplitsOnASegment.cs`; its constructor and refinement land with the split
//! sewing package as further `impl` blocks.

use std::collections::HashMap;

use super::SurfaceVertexPool;

/// C# `SurfaceSplitsOnASegment`: the splits standing along each stretch, refined to a standstill
/// and remembered by the stretch's two end vertices.
///
/// The sewing reads each side of a face for the splits standing on it (`SurfaceSplitSewing`). Read
/// against the straight chord between the side's two corners that misses points: the splits are
/// arrived at by other roads, so each stands somewhere inside the ruler of the chord rather than on
/// it, and the polyline they draw bows off the chord by nearly a whole ruler. So the stretch is
/// refined: the splits found are spliced in, and each piece between two neighbours is read again,
/// to a standstill.
///
/// The thing that makes it safe is the KEY. The answer depends on the two end vertices and nothing
/// else, so face A's stretch from a split to a split and face B's side from that same split to that
/// same corner are one question asked twice, and get one answer. A reading that depended on what
/// the face asking had already taken in would not: measured on the Round Side at radius 8.5, one
/// face saw a row of four stations between two of its own splits and took them in, while the face
/// across the line saw the same row between a split and a CORNER and did not - six open edges at
/// each of two places.
///
/// Lives for one `SewInTheSplits` pass, which only READS the pool while it does, so the pool is
/// borrowed rather than shared. C#'s initializers - `Settled = true`, an empty memo - are the
/// constructor's to set, as there is no default pool to borrow.
#[derive(Debug, Clone)]
pub struct SurfaceSplitsOnASegment<'a> {
    /// The pool the stretches' vertices are read from.
    pub pool: &'a SurfaceVertexPool,
    /// The candidate splits: one stage's own new vertices, which is what the sewing is handed.
    pub candidates: Vec<i32>,
    /// What has been read so far, keyed by the stretch's two end vertices, low first. Only ever
    /// looked up by key, never iterated, so a `HashMap`'s order cannot reach an output.
    pub known: HashMap<(i32, i32), Vec<i32>>,
    /// Whether every stretch read so far settled (C# initializer `true`). A reading that did not
    /// settle has handed back a stretch that is CUT SHORT of the refinement, and what is built on
    /// it is not to be published: see [`SurfaceSplitsOnASegment::the_stretch_that_did_not_settle`]
    /// and the refusal in `SurfaceSplitSewing.SewInTheSplits`.
    pub settled: bool,
    /// The ends of the first stretch whose refinement did not settle, for the advisory to point at.
    pub the_stretch_that_did_not_settle: Option<(i32, i32)>,
}
