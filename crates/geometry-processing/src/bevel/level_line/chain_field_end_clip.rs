//! One bounded end's clipping plane ([`ChainFieldEndClip`]), from
//! `Bevel/LevelLine/ChainField.EndClip.cs`. The clip readings, the clipped sample, the end-plane
//! reader and the clipped term's searches are in `chain_field_clip.rs`. Here too the clips a
//! chain's end-ray bounds give ([`ChainField::find_end_clips`]) and whether an edge lies in a
//! knife end's plane ([`ChainField::lies_in_an_end_plane`]); the clipped least over a piece and
//! the known-cut reading land with their method package.

use vector_math::Vector3;

use super::super::float_grid;
use super::{ChainField, EndRayBound};

/// C# `ChainField.EndClip`: one bounded end's plane - through its vertex, unit normal faced on past
/// the end.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ChainFieldEndClip {
    /// The end vertex the plane passes through.
    pub vertex: Vector3,
    /// The plane's unit normal, faced on past the end.
    pub normal: Vector3,
    /// The chain parameter at the end vertex.
    pub parameter: f64,
}

impl ChainField {
    /// C# `ChainField.FindEndClips`: the clip planes of the chain's bounded ends - through its first
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
            });
        }

        if let Some(last) = bounds.1 {
            clips.push(ChainFieldEndClip {
                vertex: points[points.len() - 1],
                normal: last.plane_normal.get_normal(),
                parameter: length,
            });
        }

        Some(clips)
    }

    /// C# `ChainField.LiesInAnEndPlane`: whether the segment a-b lies in one of the chain's end
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
}
