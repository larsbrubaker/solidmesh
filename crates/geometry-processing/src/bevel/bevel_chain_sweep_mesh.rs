//! The per-segment record of the chain sweep ([`SweptSegment`]): one chain edge's section, frame and
//! end slides, with its constructor and profile offset. The sections half builds them
//! ([`super::bevel_chain_sweep_sections`]) and the rings half mitres their joints
//! ([`super::bevel_chain_sweep_rings`]).

use vector_math::{Vector2, Vector3};

/// One edge of the chain with everything the sweep needs from it - where it runs, the frame its
/// cross section is drawn in, and how far each end of its strip is dragged by the joints at its
/// ends.
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
    /// The edge's length, `|to - from|` (computed by [`SweptSegment::new`]).
    pub length: f64,
    /// How far each profile point of the ring at this segment's start is dragged along it by the
    /// joint there; `None` until the joint has been placed.
    pub start_slide: Option<Vec<f64>>,
    /// How far each profile point of the ring at this segment's end is dragged along it by the
    /// joint there; `None` until the joint has been placed.
    pub end_slide: Option<Vec<f64>>,
}

impl SweptSegment {
    /// A segment whose length is read off its ends; it starts with its profile the right way round
    /// and no slides.
    pub fn new(
        from: Vector3,
        to: Vector3,
        direction: Vector3,
        x_axis: Vector3,
        y_axis: Vector3,
        profile: Vec<Vector2>,
        tangent_distance: f64,
    ) -> Self {
        SweptSegment {
            from,
            to,
            direction,
            x_axis,
            y_axis,
            profile,
            profile_reversed: false,
            tangent_distance,
            length: (to - from).length(),
            start_slide: None,
            end_slide: None,
        }
    }

    /// Profile point `point` as an offset in 3D, in the segment's frame.
    pub fn offset(&self, point: usize) -> Vector3 {
        (self.x_axis * self.profile[point].x) + (self.y_axis * self.profile[point].y)
    }
}
