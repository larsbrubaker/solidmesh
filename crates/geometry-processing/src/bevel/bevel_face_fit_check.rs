//! The face-width check of one group (`Bevel/BevelFaceFit.cs`: the three `TryCheckGroup`
//! overloads): every selected edge's setback walked across each of its faces at three samples,
//! and the group refused with the largest size that would have fitted where a walk runs out of
//! face - unless the stop is one of the exemptions, a wall the run ends on, or an obstruction
//! standing on the face (handed back for the shadow clip). Split from `bevel_face_fit.rs` by
//! responsibility.

use std::collections::HashSet;

use vector_math::{cs_math, Vector3};

use super::bevel_face_fit::{try_get_setback, SAMPLE_FRACTIONS};
use super::bevel_shadow_clip::Obstruction;
use super::{BevelAdvisory, BevelAdvisoryCode, BevelFaceFit};

/// What C# `TryCheckGroup` hands back: the refusal (`None` where the group fits), whether a stop
/// was a wall the run ends on (set either way, as C#'s `out` is), and the obstructions found
/// standing on a face where the group fits.
#[derive(Debug, Clone, Default)]
pub struct GroupCheck {
    /// The refusal, or `None` where the group fits.
    pub failure: Option<BevelAdvisory>,
    /// C# `trimmedByAWallItEndsOn`.
    pub trimmed_by_a_wall_it_ends_on: bool,
    /// C# `standingOnAFace`: empty unless the group fits.
    pub standing_on_a_face: Vec<Obstruction>,
}

impl GroupCheck {
    /// Whether the group fits (C#'s return value).
    pub fn fits(&self) -> bool {
        self.failure.is_none()
    }
}

impl BevelFaceFit {
    /// C# `TryCheckGroup(groupIndex, out failureReason)`.
    pub fn try_check_group(&self, group_index: usize) -> Result<(), BevelAdvisory> {
        match self.try_check_group_fully(group_index).failure {
            Some(failure) => Err(failure),
            None => Ok(()),
        }
    }

    /// C# `TryCheckGroup(groupIndex, out failureReason, out trimmedByAWallItEndsOn)`: the
    /// verdict, and whether a wall the run ends on trimmed it.
    pub fn try_check_group_trimmed(&self, group_index: usize) -> (Result<(), BevelAdvisory>, bool) {
        let check = self.try_check_group_fully(group_index);
        let trimmed = check.trimmed_by_a_wall_it_ends_on;
        (
            match check.failure {
                Some(failure) => Err(failure),
                None => Ok(()),
            },
            trimmed,
        )
    }

    /// C# `TryCheckGroup(groupIndex, out failureReason, out trimmedByAWallItEndsOn, out
    /// standingOnAFace)`. An unresolved group fits trivially.
    pub fn try_check_group_fully(&self, group_index: usize) -> GroupCheck {
        let mut check = GroupCheck::default();
        let mut standing_on_a_face_found: Vec<Obstruction> = Vec::new();
        let mut recorded_faces: HashSet<(i32, i32)> = HashSet::new();
        let resolution = &self.resolutions[group_index];
        if !resolution.is_resolved() {
            return check;
        }

        let mut largest_that_fits = f64::MAX;
        let mut worst_at = Vector3::ZERO;
        let mut worst_is_a_collision = false;

        for edge in &resolution.edges {
            let Some(setback) = try_get_setback(edge, self.size, self.round_profile) else {
                continue;
            };

            for side in 0..2 {
                let face = if side == 0 { edge.face0 } else { edge.face1 };
                let Some(across) = self.walk.try_across_direction(face, edge) else {
                    continue;
                };

                let mut samples = Vec::with_capacity(SAMPLE_FRACTIONS.len());
                let mut widest_on_this_face = 0.0;
                for fraction in SAMPLE_FRACTIONS {
                    let at = edge.start + ((edge.end - edge.start) * fraction);
                    let mut crossed_here = Vec::with_capacity(8);
                    let stop = self.walk.walk(
                        face,
                        at,
                        across,
                        edge.start_vertex,
                        edge.end_vertex,
                        setback + self.largest_setback,
                        Some(&mut crossed_here),
                    );
                    samples.push((at, stop.travelled, stop.blocking_edge, crossed_here));
                    widest_on_this_face = cs_math::max_f64(widest_on_this_face, stop.travelled);
                }

                for (from, width, blocking_edge, crossed) in samples {
                    let reserved = if blocking_edge >= 0 {
                        self.setbacks.get(&blocking_edge).copied()
                    } else {
                        None
                    };
                    let collides = reserved.is_some();
                    let required = setback + reserved.unwrap_or(0.0);
                    let fits = self.size * width / required;
                    if width >= required - (width * 1e-6) - 1e-9 || fits >= largest_that_fits {
                        continue;
                    }

                    if self.runs_out_onto_a_wall_the_run_ends_on(
                        group_index,
                        face,
                        edge,
                        blocking_edge,
                    ) {
                        check.trimmed_by_a_wall_it_ends_on = true;
                        continue;
                    }

                    let stands_on = self.stands_on_the_face_rather_than_ending_it(
                        face,
                        edge,
                        blocking_edge,
                        &crossed,
                    );
                    if let Some((planar_within, seed_face)) = stands_on {
                        if recorded_faces.insert((face, edge.mesh_edge_index)) {
                            standing_on_a_face_found.push(Obstruction::on(
                                self.face_normal(face),
                                edge,
                                across,
                                setback,
                                planar_within,
                                seed_face,
                                if self.round_profile { self.size } else { 0.0 },
                            ));
                        }
                    }

                    if stands_on.is_some()
                        || self.two_beads_sharing_the_face(edge, blocking_edge, collides)
                        || self.the_corners_business(
                            edge,
                            blocking_edge,
                            widest_on_this_face,
                            setback,
                        )
                        || self.a_bead_running_out_past_a_convex_edge(
                            edge,
                            blocking_edge,
                            widest_on_this_face,
                            setback,
                        )
                        || self.a_negligible_sliver_past_a_convex_edge(
                            blocking_edge,
                            width,
                            setback,
                        )
                    {
                        continue;
                    }

                    largest_that_fits = fits;
                    worst_at = from;
                    worst_is_a_collision = collides;
                }
            }
        }

        if largest_that_fits == f64::MAX {
            check.standing_on_a_face = standing_on_a_face_found;
            return check;
        }

        check.failure = Some(BevelAdvisory::new(
            if worst_is_a_collision {
                BevelAdvisoryCode::SizeCollidesAcrossFace
            } else {
                BevelAdvisoryCode::SizeExceedsFaceWidth
            },
            vec![self.size.into(), worst_at.into(), largest_that_fits.into()],
        ));
        check
    }
}
