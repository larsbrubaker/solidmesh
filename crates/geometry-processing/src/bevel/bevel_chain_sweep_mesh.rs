//! The SEWING half of the chain sweep: the per-segment record the finished rings are built from
//! ([`SweptSegment`]). Its constructor and profile offset, the cap's own profile ([`cap_profile`]),
//! and the solid sewn from the rings ([`build_mesh`]).

use polygon_mesh::Mesh;
use vector_math::{Vector2, Vector3, Vector3Float};

use super::{bevel_profile_polygon, BevelChainEnds};

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

/// A ring read back into `segment`'s profile frame: each point's offset from the segment's start
/// along its two axes.
pub fn cap_profile(ring: &[Vector3], segment: &SweptSegment) -> Vec<Vector2> {
    ring.iter()
        .map(|&point| {
            let offset = point - segment.from;
            Vector2::new(offset.dot(segment.x_axis), offset.dot(segment.y_axis))
        })
        .collect()
}

/// The solid sewn from a run's `rings`: one strip of quads between each consecutive pair (wrapping
/// on a closed run), and on an open run the two end caps triangulated from the profile - from the
/// actual ring where an end drives through, since its profile is tapered. `None` where a cap
/// cannot be triangulated.
pub fn build_mesh(
    swept: &[SweptSegment],
    rings: &[Vec<Vector3>],
    closed: bool,
    point_count: usize,
    ends: Option<&BevelChainEnds>,
) -> Option<Mesh> {
    let mut mesh = Mesh::new();

    for ring in rings {
        for &position in ring {
            mesh.vertices.push(Vector3Float::from_vector3(position));
        }
    }

    let ring_count = rings.len();

    // One strip between every consecutive pair of rings, counted off the RINGS because a
    // drive-through end adds one of its own beyond the run's last ring.
    let strip_count = if closed { ring_count } else { ring_count - 1 };
    let pc = point_count as i32;

    for strip in 0..strip_count {
        let back = (strip as i32) * pc;
        let front = (((strip + 1) % ring_count) as i32) * pc;

        for point in 0..pc {
            let next = (point + 1) % pc;

            // With the profile wound counter clockwise and the rings ordered along the sweep, this
            // is the winding whose normals face out of the tool.
            mesh.faces
                .add(back + point, back + next, front + next, &mesh.vertices);
            mesh.faces
                .add(back + point, front + next, front + point, &mesh.vertices);
        }
    }

    if !closed {
        // A drive-through cap has a tapered profile: triangulate the actual ring.
        let first = &swept[0];
        let last = &swept[swept.len() - 1];
        let start_profile = if ends.is_some_and(|e| e.start.drives_through) {
            cap_profile(&rings[0], first)
        } else {
            first.profile.clone()
        };
        let end_profile = if ends.is_some_and(|e| e.end.drives_through) {
            cap_profile(&rings[ring_count - 1], last)
        } else {
            last.profile.clone()
        };

        let start_cap = bevel_profile_polygon::try_triangulate(&start_profile)?;
        let end_cap = bevel_profile_polygon::try_triangulate(&end_profile)?;

        let last_ring = (ring_count as i32 - 1) * pc;

        for i in (0..start_cap.len()).step_by(3) {
            // reversed, because the start cap faces back down the sweep
            mesh.faces.add(
                start_cap[i],
                start_cap[i + 2],
                start_cap[i + 1],
                &mesh.vertices,
            );
        }

        for i in (0..end_cap.len()).step_by(3) {
            mesh.faces.add(
                last_ring + end_cap[i],
                last_ring + end_cap[i + 1],
                last_ring + end_cap[i + 2],
                &mesh.vertices,
            );
        }
    }

    Some(mesh)
}
