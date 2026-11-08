//! A short chord whose two mitres have CROSSED OVER on some of its rails
//! ([`SurfaceMitreConsumesTheRail`]): the setback
//! each neighbour's mitre takes grows with the rail's own offset from the edge, so a chord shorter
//! than the two setbacks together is eaten from one side of the profile inward while the rails
//! nearer the other face still have room. The strip is not gone - it is gone on rails 0 through
//! [`SurfaceMitreConsumesTheRail::through`], and those rails end on the VIRTUAL CORNER the two
//! NEIGHBOURS' own lines meet at instead.
//!
//! This is the STRIP's own version of the swallowed edge [`super::SurfaceEdgeRoles`] reads: there a
//! blend reaches over a whole chord and the chord has no strip at all, here a mitre reaches over part
//! of the profile and the chord keeps the rest. The walkback has already read it the same way - the
//! face's offset boundary turns straight from the previous edge's tangent line onto the next one's,
//! leaving one vertex where this edge's line would have been - so the seam on that side is that
//! single vertex, and the surface that stands on it is a staircase rather than a ring
//! (`SurfaceStaircaseMitre`).
//!
//! Every way this can fail to be that shape is named rather than built: a strip consumed on EVERY
//! rail is a size that does not fit at all; a consumed block that is not against one end of the
//! profile is two crossings rather than one and there is no single virtual corner for it; two
//! neighbour lines that do not meet have no virtual corner to stand on; and a turn between the
//! neighbours past the plan's own bound is a mitre nothing may be cut at.

// The negated comparisons are deliberate: the comparison inside each is false for NaN, so a NaN
// takes the negated branch.
#![allow(clippy::neg_cmp_op_on_partial_ord)]

use std::rc::Rc;

use solidmesh_math::{math_helper, Vector3};

use super::surface_mitre_joint_step::SideSeams;
use super::{surface_bevel_stitch, surface_strip_ends};
use super::{SurfaceBevelPlan, SurfaceEdgePlan, SurfaceStrip, SurfaceVertexPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

/// One chord's consumed rails and the virtual
/// corner they end on.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceMitreConsumesTheRail {
    /// The edge before the consumed one along its own chain.
    pub previous_index: i32,
    /// The consumed edge.
    pub edge_index: i32,
    /// The edge after the consumed one along its own chain.
    pub next_index: i32,
    /// Which side of the strip is consumed: 0 for the side rail 0 lies on, 1 for the last rail's.
    /// It is the side whose tangent line is set further back from the edge, which is where the two
    /// setbacks first add up to more than the chord.
    pub side: i32,
    /// How many rails in from [`Self::side`] are consumed - at least one.
    pub through: i32,
    /// The point the two NEIGHBOURS' edge lines meet at, which is the corner the chain would have
    /// had if this chord were not there. The consumed rails are cut at the mitre plane through it.
    pub virtual_corner: Vector3,
    /// That plane's normal - the two neighbours' walk directions summed, as a mitre's is.
    pub virtual_normal: Vector3,
    /// The real corner this edge starts at.
    pub corner_before: Vector3,
    /// The real corner this edge ends at.
    pub corner_after: Vector3,
}

/// The advisory every refusal here is named with.
fn consumes_the_strip(at: Vector3) -> BevelAdvisory {
    BevelAdvisory::new(
        BevelAdvisoryCode::SurfaceMitreConsumesTheStrip,
        vec![at.into()],
    )
}

/// The strip at `index`, which every caller has checked is there.
fn strip_at(strips: &[Option<Rc<SurfaceStrip>>], index: i32) -> Option<&SurfaceStrip> {
    strips
        .get(index as usize)
        .and_then(|strip| strip.as_deref())
}

impl SurfaceMitreConsumesTheRail {
    /// Whether `rail` of a strip of `rail_count` is consumed.
    pub fn consumes_rail(&self, rail: i32, rail_count: i32) -> bool {
        if self.side == 0 {
            rail < self.through
        } else {
            rail >= rail_count - self.through
        }
    }

    /// Every consumed strip across every chain, indexed as the builder indexes its
    /// edges, with `None` wherever the ordinary mitre stands. Read once, before any seam is picked,
    /// because a seam on a consumed side is not a run of the tangent line at all.
    ///
    /// A strip whose neighbour has no strip of its own - the open end of a chain, or a chord a blend
    /// swallowed - is left alone here even when it looks consumed: there is no second mitre plane to
    /// have crossed over, and what happens at a free end is the end cut's reading and not this one.
    /// The same for a corner the convexity turns over at, which is a torus patch rather than a mitre.
    pub fn try_survey(
        pool: &SurfaceVertexPool,
        plans: &[Rc<SurfaceBevelPlan>],
        first_of_plan: &[i32],
        edges: &[SurfaceEdgePlan],
        strips: &[Option<Rc<SurfaceStrip>>],
    ) -> Result<Vec<Option<SurfaceMitreConsumesTheRail>>, BevelAdvisory> {
        let mut consumed = vec![None; strips.len()];

        for (p, plan) in plans.iter().enumerate() {
            let in_chain = plan.edges.len() as i32;

            for i in 0..in_chain {
                if !plan.is_closed && (i == 0 || i + 1 == in_chain) {
                    continue;
                }

                if plan
                    .try_mixed_corner_after((i + in_chain - 1) % in_chain)
                    .is_some()
                    || plan.try_mixed_corner_after(i).is_some()
                {
                    continue;
                }

                let here = first_of_plan[p] + i;
                let previous = first_of_plan[p] + ((i + in_chain - 1) % in_chain);
                let next = first_of_plan[p] + ((i + 1) % in_chain);

                if strip_at(strips, here).is_none()
                    || strip_at(strips, previous).is_none()
                    || strip_at(strips, next).is_none()
                {
                    continue;
                }

                consumed[here as usize] = Self::try_one(pool, edges, strips, previous, here, next)?;
            }
        }

        Ok(consumed)
    }

    /// The seam of every consumed side - the ONE vertex the two neighbours'
    /// tangent lines already meet at, taken by index from the neighbours' own seams rather than made
    /// again. Run after every ordinary seam is picked, because it is those two seams' last and first
    /// vertex.
    pub fn try_fill_the_seams(
        pool: &SurfaceVertexPool,
        strips: &[Option<Rc<SurfaceStrip>>],
        seams: &mut [SideSeams],
        consumed: &[Option<SurfaceMitreConsumesTheRail>],
    ) -> Result<(), BevelAdvisory> {
        for reading in consumed.iter().flatten() {
            let vertex = Self::try_the_shared_corner(pool, strips, seams, reading)?;

            let param = strip_at(strips, reading.edge_index)
                .map(|strip| strip.param_of(pool.position_of(vertex)))
                .ok_or_else(|| consumes_the_strip(reading.corner_before))?;

            seams[reading.edge_index as usize][reading.side as usize] = Some(vec![(vertex, param)]);
        }

        Ok(())
    }

    /// The vertex both neighbours' seams end and begin on, checked to be
    /// one vertex and to stand where the two neighbours' rails on this side really cross. Two
    /// different vertices there would be the walkback having read the corner some other way, which
    /// is a contradiction and not a rounding.
    pub fn try_the_shared_corner(
        pool: &SurfaceVertexPool,
        strips: &[Option<Rc<SurfaceStrip>>],
        seams: &[SideSeams],
        reading: &SurfaceMitreConsumesTheRail,
    ) -> Result<i32, BevelAdvisory> {
        let side = reading.side as usize;
        let before = seams[reading.previous_index as usize][side].as_deref();
        let after = seams[reading.next_index as usize][side].as_deref();

        let vertex = match (before, after) {
            (Some(before), Some(after))
                if !before.is_empty()
                    && !after.is_empty()
                    && before[before.len() - 1].0 == after[0].0 =>
            {
                before[before.len() - 1].0
            }
            _ => return Err(consumes_the_strip(reading.corner_before)),
        };

        let previous = strip_at(strips, reading.previous_index)
            .ok_or_else(|| consumes_the_strip(pool.position_of(vertex)))?;
        let rail = if reading.side == 0 {
            0
        } else {
            previous.rail_count() - 1
        };

        match previous.try_cross(rail, reading.virtual_corner, reading.virtual_normal) {
            Some(at)
                if !((pool.position_of(vertex) - previous.point_on(rail, at)).length()
                    > surface_strip_ends::meets_the_seam(pool)) =>
            {
                Ok(vertex)
            }
            _ => Err(consumes_the_strip(pool.position_of(vertex))),
        }
    }

    /// One edge read - whether its two mitres have crossed over on some rail, and if so
    /// the block of rails that is and the virtual corner they end on. `Ok(None)` is an edge read
    /// with nothing crossed over.
    fn try_one(
        pool: &SurfaceVertexPool,
        edges: &[SurfaceEdgePlan],
        strips: &[Option<Rc<SurfaceStrip>>],
        previous_index: i32,
        edge_index: i32,
        next_index: i32,
    ) -> Result<Option<SurfaceMitreConsumesTheRail>, BevelAdvisory> {
        let (Some(previous), Some(here), Some(next)) = (
            strip_at(strips, previous_index),
            strip_at(strips, edge_index),
            strip_at(strips, next_index),
        ) else {
            return Ok(None);
        };

        if here.rail_count() != previous.rail_count() || here.rail_count() != next.rail_count() {
            return Ok(None);
        }

        // The CORNER, not the point the mitre plane really passes through
        // (surface_mitre_seam::the_point_or): where the two beads set back by different amounts
        // those are not the same place, and on a loop whose setback steps round it they can be half
        // a chord apart. Left as it is on purpose - this survey bails above on any rail count
        // mismatch, which is every case where the two readings differ by enough to matter - and
        // written down here so that the next reader knows it is a known difference rather than an
        // oversight.
        let corner_before = edges[edge_index as usize].from();
        let corner_after = edges[edge_index as usize].to();
        let before_plane = (previous.along() + here.along()).get_normal();
        let after_plane = (here.along() + next.along()).get_normal();

        let rail_count = here.rail_count();
        let mut inverted = vec![false; rail_count.max(0) as usize];
        let mut found = 0;

        for rail in 0..rail_count {
            let (Some(opens), Some(closes)) = (
                here.try_cross(rail, corner_before, before_plane),
                here.try_cross(rail, corner_after, after_plane),
            ) else {
                // A rail this mitre cannot be cut at is the ordinary mitre's own refusal to make, in
                // the words it already has for it.
                return Ok(None);
            };

            inverted[rail as usize] = closes < opens - surface_bevel_stitch::one_station(pool);
            found += i32::from(inverted[rail as usize]);
        }

        if found == 0 {
            return Ok(None);
        }

        let middle = (corner_before + corner_after) / 2.0;

        if found == rail_count {
            // Nothing of the chord is left on any rail, so there is no strip to stand between the two
            // neighbours and no part of the profile that still meets this edge. A size that eats a
            // whole chord of a rim is a refusal.
            return Err(consumes_the_strip(middle));
        }

        let side = if inverted[0] {
            0
        } else if inverted[inverted.len() - 1] {
            1
        } else {
            -1
        };

        let through = if side < 0 {
            None
        } else {
            Self::is_one_block_against(&inverted, side)
        };

        let Some(through) = through else {
            // The consumed rails have to be the ones against one face, because that is the only
            // shape the ONE virtual corner below stands for: a block in the middle of the profile
            // would want a corner on either side of it.
            return Err(consumes_the_strip(middle));
        };

        // The mitre the consumed rails really end on is the one between the two NEIGHBOURS, so it
        // is bounded by the same turn any mitre is (SurfaceBevelPlan::MAXIMUM_WEDGE_ANGLE_DEGREES):
        // a plane so nearly along the rails cuts them metres from the part.
        let turn_degrees = math_helper::radians_to_degrees(libm::acos(
            previous.along().dot(next.along()).clamp(-1.0, 1.0),
        ));

        if !(turn_degrees < SurfaceBevelPlan::MAXIMUM_WEDGE_ANGLE_DEGREES) {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceStripsDoNotMitre,
                vec![corner_after.into()],
            ));
        }

        let Some(corner) = Self::try_the_virtual_corner(
            pool,
            &edges[previous_index as usize],
            &edges[next_index as usize],
        ) else {
            // Two lines that do not meet - parallel, or skew across the chain's own plane - leave the
            // consumed rails nothing to end on.
            return Err(consumes_the_strip(middle));
        };

        Ok(Some(SurfaceMitreConsumesTheRail {
            previous_index,
            edge_index,
            next_index,
            side,
            through,
            virtual_corner: corner,
            virtual_normal: (previous.along() + next.along()).get_normal(),
            corner_before,
            corner_after,
        }))
    }

    /// How deep the consumed rails reach where they are one unbroken block
    /// against `side` of the profile, and `None` where they are not.
    fn is_one_block_against(inverted: &[bool], side: i32) -> Option<i32> {
        let count = inverted.len();
        let at = |i: usize| inverted[if side == 0 { i } else { count - 1 - i }];

        let mut through = 0;

        for i in 0..count {
            if !at(i) {
                break;
            }

            through += 1;
        }

        for i in through..count {
            if at(i) {
                return None;
            }
        }

        Some(through as i32)
    }

    /// Where the two neighbours' own edge lines cross. Taken as the middle
    /// of their closest approach and refused when that is a gap rather than a point, so that a chain
    /// bending out of one plane is named rather than given a corner neither line stands on.
    fn try_the_virtual_corner(
        pool: &SurfaceVertexPool,
        previous: &SurfaceEdgePlan,
        next: &SurfaceEdgePlan,
    ) -> Option<Vector3> {
        let along_previous = previous.edge.direction();
        let along_next = next.edge.direction();
        let between = previous.from() - next.from();

        let cross = along_previous.dot(along_next);
        let denominator = 1.0 - (cross * cross);

        if !(denominator.abs() > 1e-9) {
            return None;
        }

        let on_previous =
            ((cross * along_next.dot(between)) - along_previous.dot(between)) / denominator;
        let on_next =
            (along_next.dot(between) - (cross * along_previous.dot(between))) / denominator;

        let from_previous = previous.from() + (along_previous * on_previous);
        let from_next = next.from() + (along_next * on_next);

        if (from_previous - from_next).length() > surface_strip_ends::meets_the_seam(pool) {
            return None;
        }

        Some((from_previous + from_next) / 2.0)
    }
}
