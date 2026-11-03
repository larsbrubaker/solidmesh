//! A pick's chain ends as the band is cut on them ([`PickEnds::of`]): which ends carry an end ray
//! (`end_face_sense::oblique_ends`, none at a corner of the pick), the rays' bounds
//! (`end_face_sense::end_ray_bounds`), a square crease end's clip where an end has no bound
//! (`square_crease_end_clip.rs`) and a stopped end's clip where it still has none
//! (`stopped_end_clip.rs`), always in that order. The band build (`band_gap_setback_fields.rs`)
//! and the room readings (`band_room.rs`, `band_room_pair_room_pick.rs`) all read their ends here,
//! so a clip reaches the readings exactly where it reaches the band. The readings do not read the
//! square crease clips ([`PickEnds::bounds_without_square_crease_clips`]); they never have, and the
//! room those clips leave is read on its own (`square_crease_end_room`).

use std::collections::HashSet;

use vector_math::Vector3;

use super::end_face_sense::{self, Corners};
use super::{pick_validation, square_crease_end_clip, stopped_end_clip, EndRayBound};

/// One chain's two end bounds, as (first, last).
pub type EndBounds = (Option<EndRayBound>, Option<EndRayBound>);

/// Every chain's ends, as [`PickEnds::of`] reads them.
#[derive(Debug, Clone, PartialEq)]
pub struct PickEnds {
    /// Whether each chain's first and last ends carry an end ray.
    pub end_rays: Vec<(bool, bool)>,
    /// Each chain's end bounds: the rays' bounds, a square crease end's clip, a stopped end's clip.
    pub bounds: Vec<EndBounds>,
    /// Which of each chain's ends took a square crease end's clip.
    pub square_crease: Vec<(bool, bool)>,
    /// Whether an end of each chain was stopped.
    pub stopped: Vec<bool>,
}

impl PickEnds {
    /// The pick's ends. `is_feature_edge` names the creases a stopped end's leaving crease is read
    /// among.
    pub fn of(
        positions: &[Vector3],
        indices: &[Corners],
        chains: &[Vec<i32>],
        closed: &[bool],
        sharp_turn_degrees: f64,
        is_feature_edge: &dyn Fn(i32, i32) -> bool,
    ) -> PickEnds {
        let count = chains.len();
        // An end on an OBLIQUE end face carries the end ray; an end at a CORNER is not an end face.
        let corner_ends: HashSet<i32> = if count > 1 {
            pick_validation::corners_of(chains, closed, true)
                .iter()
                .map(|c| c.vertex)
                .collect()
        } else {
            HashSet::new()
        };
        let at_a_corner = |end: Option<&i32>| end.is_some_and(|v| corner_ends.contains(v));
        let mut end_rays = Vec::with_capacity(count);
        let mut bounds = Vec::with_capacity(count);
        for k in 0..count {
            let chain = &chains[k];
            let rays = end_face_sense::oblique_ends(
                positions,
                indices,
                chain,
                closed[k],
                sharp_turn_degrees,
            );
            let rays = (
                rays.0 && !at_a_corner(chain.first()),
                rays.1 && !at_a_corner(chain.last()),
            );
            end_rays.push(rays);
            bounds.push(end_face_sense::end_ray_bounds(
                positions, indices, chain, rays,
            ));
        }

        // A square end on a concave crease whose side runs on past it is clipped in the end face's
        // plane.
        let square_clips = square_crease_end_clip::square_crease_end_clips_of_pick(
            positions, indices, chains, closed, &end_rays,
        );
        let mut square_crease = vec![(false, false); count];
        for k in 0..count {
            let (first, last): EndBounds = bounds[k];
            square_crease[k] = (
                first.is_none() && square_clips[k].0.is_some(),
                last.is_none() && square_clips[k].1.is_some(),
            );
            bounds[k] = (first.or(square_clips[k].0), last.or(square_clips[k].1));
        }

        // A stopped end (the pick ends, the crease carries on), where a caller turned them on.
        let mut stopped = vec![false; count];
        for k in 0..count {
            let chain = &chains[k];
            let (first, last) = stopped_end_clip::stopped_end_clips(
                positions,
                indices,
                chain,
                closed[k],
                end_rays[k],
                is_feature_edge,
            );
            if bounds[k].0.is_none() && !at_a_corner(chain.first()) && first.is_some() {
                bounds[k].0 = first;
                stopped[k] = true;
            }
            if bounds[k].1.is_none() && !at_a_corner(chain.last()) && last.is_some() {
                bounds[k].1 = last;
                stopped[k] = true;
            }
        }

        PickEnds {
            end_rays,
            bounds,
            square_crease,
            stopped,
        }
    }

    /// Chain `k`'s bounds with its square crease clips left out, as the room readings take them.
    pub fn bounds_without_square_crease_clips(&self, k: usize) -> EndBounds {
        let (first, last) = self.bounds[k];
        let (square_first, square_last) = self.square_crease[k];
        (
            if square_first { None } else { first },
            if square_last { None } else { last },
        )
    }
}
