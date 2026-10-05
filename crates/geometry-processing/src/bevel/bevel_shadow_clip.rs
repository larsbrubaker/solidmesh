//! The column a cutter must not reach into above a face something stands on
//! (`Bevel/BevelShadowClip.cs`, a C# `internal static class`, so a module here): one obstructed
//! face of one selected edge ([`Obstruction`]), recorded by `BevelFaceFit` when it admitted the
//! group. Drawing the column lands with the core's packages as free functions in this module.

use vector_math::Vector3;

/// C# `BevelShadowClip.Obstruction`: one face of one selected edge that something is standing
/// on - what `BevelFaceFit` recorded when it admitted the group, and everything the clip needs to
/// draw the column.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Obstruction {
    /// The obstructed face's outward normal, which is the direction the shadow is cast along.
    pub plane_normal: Vector3,
    /// The face's plane offset: every point whose dot with `plane_normal` is this.
    pub plane_distance: f64,
    /// The selected edge's start, which the band is measured from.
    pub origin: Vector3,
    /// The direction along the selected edge.
    pub along: Vector3,
    /// How far the selected edge runs.
    pub along_length: f64,
    /// The direction that leaves the edge square to it and heads into the obstructed face.
    pub across: Vector3,
    /// How far across the face this edge's tool reaches, which is how deep the band is.
    pub setback: f64,
    /// The radius of the ROUND this cutter carries, or zero where the profile is not a round. It
    /// says only where the cutter's own surface runs across this face: a round is tangent to the
    /// face at `setback` across it, so every point of the cutter is OUTSIDE the cylinder whose
    /// axis lies this far behind the plane on the tangent line.
    pub round_radius: f64,
    /// How far off its own plane the face was allowed to be when `BevelFaceFit` admitted it as
    /// planar - and so the smallest height that can count as standing ON it.
    pub planar_within: f64,
    /// The obstruction's OWN first triangle - the face on the far side of the edge the walk
    /// stopped at - which is where the footprint's flood starts. Without a seed the footprint
    /// would be "everything in front of the plane", which sweeps in things that merely pass over
    /// the face.
    pub seed_face: i32,
}
