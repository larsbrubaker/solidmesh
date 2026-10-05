//! The SEWING half of the chain sweep (`Bevel/BevelChainSweep.Mesh.cs`, part of the C#
//! `internal static partial class BevelChainSweep`): the per-segment record the finished rings
//! are built from ([`SweptSegment`]). Turning the rings into a solid lands with the core's sweep
//! packages as free functions in this module.

use vector_math::{Vector2, Vector3};

/// C# `BevelChainSweep.SweptSegment`: one edge of the chain with everything the sweep needs from
/// it - where it runs, the frame its cross section is drawn in, and how far each end of its strip
/// is dragged by the joints at its ends.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SweptSegment {
    /// Where the edge starts.
    pub from: Vector3,
    /// Where the edge ends.
    pub to: Vector3,
    /// The unit direction the edge is swept along.
    pub direction: Vector3,
    /// The cross-section frame's x axis, in part space.
    pub x_axis: Vector3,
    /// The cross-section frame's y axis, in part space.
    pub y_axis: Vector3,
    /// The cross section, in the segment's (x, y) frame.
    pub profile: Vec<Vector2>,
    /// Whether the profile was reversed to keep the sewn faces facing outward.
    pub profile_reversed: bool,
    /// The fillet's tangent distance along each face at this segment.
    pub tangent_distance: f64,
    /// The edge's length, `|to - from|` (computed by C#'s constructor).
    pub length: f64,
    /// How far each profile point of the ring at this segment's start is dragged along it by the
    /// joint there; `None` until the joint has been placed.
    pub start_slide: Option<Vec<f64>>,
    /// How far each profile point of the ring at this segment's end is dragged along it by the
    /// joint there; `None` until the joint has been placed.
    pub end_slide: Option<Vec<f64>>,
}
