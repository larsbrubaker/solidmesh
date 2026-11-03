//! One bounded end's clipping plane ([`ChainFieldEndClip`]). The clip readings, the clipped
//! sample, the end-plane
//! reader and the clipped term's searches are in `chain_field_clip.rs`. Here too the clips a
//! chain's end-ray bounds give ([`ChainField::find_end_clips`]) and whether an edge lies in a
//! knife end's plane ([`ChainField::lies_in_an_end_plane`]), whether a point stands in a stopped
//! end's plane ([`ChainField::lies_in_a_stopped_end_plane`]) and which one
//! ([`ChainField::stopped_end_plane_at`]), the one rule for standing on a stopped plane
//! ([`ChainField::on_a_stopped_plane`]) and the known-cut reading
//! ([`ChainField::value_across_the_known_cut`]); the clipped least over a piece is in
//! `chain_field_least.rs`.

use vector_math::Vector3;

use super::super::float_grid;
use super::{ChainField, EndRayBound, ScalarField, WorkError};

/// One bounded end's plane - through its vertex, unit normal faced on past
/// the end.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ChainFieldEndClip {
    /// The end vertex the plane passes through.
    pub vertex: Vector3,
    /// The plane's unit normal, faced on past the end.
    pub normal: Vector3,
    /// The chain parameter at the end vertex.
    pub parameter: f64,
    /// How far the normal may lean off the true plane from float storage
    /// ([`EndRayBound::plane_tilt`]); 0 but at a stopped end.
    pub tilt: f64,
}

impl ChainField {
    /// The clip planes of the chain's bounded ends - through its first
    /// point at parameter 0 and its last at `length`, each faced along its bound's plane normal;
    /// `None` where neither end is bounded.
    pub fn find_end_clips(
        points: &[Vector3],
        length: f64,
        bounds: (Option<EndRayBound>, Option<EndRayBound>),
    ) -> Option<Vec<ChainFieldEndClip>> {
        if bounds.0.is_none() && bounds.1.is_none() {
            return None;
        }

        let mut clips = Vec::new();
        if let Some(first) = bounds.0 {
            clips.push(ChainFieldEndClip {
                vertex: points[0],
                normal: first.plane_normal.get_normal(),
                parameter: 0.0,
                tilt: first.plane_tilt,
            });
        }

        if let Some(last) = bounds.1 {
            clips.push(ChainFieldEndClip {
                vertex: points[points.len() - 1],
                normal: last.plane_normal.get_normal(),
                parameter: length,
                tilt: last.plane_tilt,
            });
        }

        Some(clips)
    }

    /// Whether the segment a-b lies in one of the chain's end
    /// planes, both ends within a few float steps of it (the plane stands through a float-stored
    /// vertex). Along such an edge the clipped field reads 0 wherever the chain is within its
    /// setback, so its band boundary is where the chain's OWN value crosses 0 (EdgeRoot).
    ///
    /// Only an end that carries the END RAY and does not fold the chain's own way - the knife end,
    /// whose plane is the neighbour face across a knife. A same-sense square crease end (no ray)
    /// has the side face's stretch in its plane, where the chain's own zero is not the band's
    /// boundary; read there, the whole pick crossed itself (BevelToCenterBlendCreaseTests).
    pub fn lies_in_an_end_plane(&self, a: Vector3, b: Vector3) -> bool {
        let Some(end_clips) = &self.end_clips else {
            return false;
        };

        let on = |p: Vector3, clip: &ChainFieldEndClip| {
            (p - clip.vertex).dot(clip.normal).abs() <= 4.0 * float_grid::step_at_point(p)
        };
        for clip in end_clips {
            let first = clip.parameter == 0.0;
            let rayed = if first {
                self.end_rays.0
            } else {
                self.end_rays.1
            };
            let bound = if first {
                self.end_ray_bounds.0
            } else {
                self.end_ray_bounds.1
            };
            if !rayed || !bound.is_some_and(|bound| !bound.same_sense) {
                continue;
            }

            if on(a, clip) && on(b, clip) {
                return true;
            }
        }

        false
    }

    /// Whether a point stands in a STOPPED end's plane (`stopped_end_clip.rs`), within a few float
    /// steps of it. Such a plane, unlike a knife end's, cuts across faces the mesh carries on with
    /// past the end, so the band's boundary there is the plane itself: a vertex in it that the
    /// clipped field reads on the line IS on the line, though the triangles on the band side of it
    /// reach into the band.
    pub fn lies_in_a_stopped_end_plane(&self, point: Vector3) -> bool {
        self.stopped_end_plane_at(point).is_some()
    }

    /// The stopped end's plane a point stands in ([`ChainField::on_a_stopped_plane`]), as (point
    /// on it, unit normal, tilt); `None` where it stands in none.
    pub fn stopped_end_plane_at(&self, point: Vector3) -> Option<(Vector3, Vector3, f64)> {
        let end_clips = self.end_clips.as_ref()?;
        end_clips
            .iter()
            .find(|clip| {
                let bound = if clip.parameter == 0.0 {
                    self.end_ray_bounds.0
                } else {
                    self.end_ray_bounds.1
                };
                bound.is_some_and(|bound| bound.stopped)
                    && ChainField::on_a_stopped_plane(clip.vertex, clip.normal, clip.tilt, point)
            })
            .map(|clip| (clip.vertex, clip.normal, clip.tilt))
    }

    /// THE rule for a point on a STOPPED end's plane through `vertex` with unit `normal`, leaning
    /// at most `tilt` off the true plane ([`EndRayBound::plane_tilt`]): within
    /// [`ChainField::IN_A_STOPPED_PLANE_STEPS`] float steps of it, the point's own storage, plus
    /// the tilt times its distance from the vertex, which a float-stored seam in any orientation
    /// stands off the stored plane by (the far end of a long seam on a finely faceted curve
    /// hundreds of steps). The field reads such a point
    /// exactly 0 (`clip_at`), the corner finder (`end_clip_corners.rs`) and the refinement's
    /// placement (`stopped_plane_points.rs`) read it in the plane, and the stopped fill reads a run
    /// of such points as lying in its end plane (`stopped_end_fill.rs`, `end_face_section_row.rs`).
    /// A knife end's plane keeps its own reach (`end_clip_corners::ON_THE_PLANE`).
    pub fn on_a_stopped_plane(vertex: Vector3, normal: Vector3, tilt: f64, point: Vector3) -> bool {
        let offset = point - vertex;
        offset.dot(normal).abs()
            <= ChainField::IN_A_STOPPED_PLANE_STEPS * float_grid::step_at_point(point)
                + tilt * offset.length()
    }
}

impl ChainField {
    /// The reading that treats every bounded end's plane as
    /// a KNOWN CUT (flat ends): strictly past a plane the plane's reading (outside by construction),
    /// otherwise the chain's own unclipped value. Only the band's classification of a triangle
    /// lying wholly on the line asks this - never the refinement or a root search, which need the
    /// continuous clipped field (a jump at the plane cannot be settled).
    ///
    /// On the plane, and a float's width in from it, the clipped field reads 0 wherever the chain
    /// is within its setback, so a side sliver along the side-to-end crease whose corners all sit
    /// on the line was kept or removed by its centroid's rounding. The plane is the band's boundary
    /// there, so the side of it a point stands on decides, and on the band side the chain's value
    /// says whether it is in the band.
    pub fn value_across_the_known_cut(&self, point: Vector3) -> Result<f64, WorkError> {
        if self.end_clips.is_none() {
            return self.value_at(point);
        }

        let (plane, _) = self.clip_at(point);
        if plane > 0.0 {
            return Ok(plane);
        }

        self.unclipped_value_at(point)
    }
}
