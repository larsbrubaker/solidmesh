//! Step 2 of the band gap's build (`band_gap_build_within.rs`): the pass's SIDES, tagged per
//! triangle and gathered into band GROUPS, or the refusal where a chain does not divide its sides.
//! A pass of one chain tags its two sides (and an open chain's end regions) for itself; a pass of
//! several chains groups the sides of the whole pick (`band_side_groups`). The tags it returns are
//! the ones the working mesh is made with, so everything after it (the cut, the band's delete, the
//! room, the consumed rims, the loop walk and the fill) reads the sides this step chose. Where the
//! rolling ball takes a one-chain pass whose side floods join its two sides into one region, the
//! sides are the ones the ball rolled on (`rolling_ball_band_tags`, whose header says why and why
//! only there). The ball's tags then lose what its contacts never sweep (`rolling_ball_sweep`):
//! a side wrapping round behind the other side's contact, or a pass refused where a side's band
//! folds back past it.

use std::collections::HashSet;
use std::rc::Rc;

use indexmap::IndexMap;
use vector_math::Vector3;

use super::band_gap_build_fault::BuildFault;
use super::band_side_tagging::{both_sided_members_build, FIRST_END_TAG, SECOND_END_TAG};
use super::band_side_tagging_ends::{self, EndReach, SideReach};
use super::foreign_edge_room_readings::FieldAt;
use super::rolling_ball_band_tags::{self, BallSides};
use super::rolling_ball_sweep::{self, ContactPlanes, Sweep};
use super::seam_edges::FeatureEdgeTest;
use super::{band_side_groups, band_side_tagging, narrow_face_flow, step_corner_carry};
use super::{BandGapRefusal, BandGapResult, BandGroup, BandProfile, ChainField};
use crate::bevel::rolling_ball::rolling_ball_routing;

/// What step 2 reads: the pass's mesh and chains, and the fields and settings the one-chain
/// tagging's reaches and the narrow face flow's rooms read.
pub struct SideInputs<'a> {
    pub positions: &'a [Vector3],
    pub indices: &'a [(i32, i32, i32)],
    pub chain_edges: &'a [Vec<(i32, i32)>],
    pub closed: &'a [bool],
    pub is_feature_edge: &'a FeatureEdgeTest,
    pub end_rays: &'a [(bool, bool)],
    pub unit_setbacks: &'a [Vec<f64>],
    /// Every chain's field at the size.
    pub fields: &'a [Rc<ChainField>],
    /// Chain 0's field at the size (a pass of one chain).
    pub field: &'a ChainField,
    /// Chain 0's field at any size, with `room_units`.
    pub field_at_0: FieldAt<'a>,
    pub room_units: &'a [f64],
    pub known_room: Option<f64>,
    pub size: f64,
    pub snap: f64,
    pub seam_epsilon: f64,
    pub profile: BandProfile,
    pub fillet_segments: i32,
    pub step_corners: &'a HashSet<i32>,
    /// The triangles the ball rolled on, where it takes a pass of one chain.
    pub ball_sides: Option<&'a BallSides>,
    /// Where the ball takes a pass of one chain: its contacts' planes and side A's and side B's
    /// fields at the size, which bound the sides its tags give (`rolling_ball_sweep`).
    pub ball_sweep: Option<(&'a ContactPlanes, &'a [Rc<ChainField>; 2])>,
}

/// The sides of the pass: its triangles' tags and its band groups, or the refusal the build hands
/// back as it is.
pub enum Sides {
    /// The tags per triangle, the band groups, and whether the tags are the rolling ball's sides
    /// (`rolling_ball_band_tags`).
    Tagged(Vec<i32>, Vec<BandGroup>, bool),
    /// A chain that does not divide its sides (boxed: a result is far larger than the tags).
    Refused(Box<BandGapResult>),
}

/// One group of a single-chain pick, seeded by chain 0 on `side`.
fn one_chain_group(tag: i32) -> BandGroup {
    BandGroup {
        tag,
        members: vec![0],
        side_of_chain: IndexMap::from([(0, tag)]),
        ..BandGroup::default()
    }
}

/// Tags the pass's sides and gathers its groups; `unflowed_room` receives the plain tagging's room
/// where the flowed tagging is cut instead (see
/// [`tag_flowed_where_it_builds`](super::narrow_face_flow::tag_flowed_where_it_builds)).
pub fn tag_the_sides(
    inputs: &SideInputs<'_>,
    unflowed_room: &mut Option<f64>,
) -> Result<Sides, BuildFault> {
    let SideInputs {
        positions,
        indices,
        chain_edges,
        closed,
        is_feature_edge,
        end_rays,
        unit_setbacks,
        fields,
        field,
        field_at_0,
        room_units,
        known_room,
        size,
        snap,
        seam_epsilon,
        profile,
        fillet_segments,
        step_corners,
        ball_sides,
        ball_sweep,
    } = *inputs;
    let mut tags: Vec<i32>;
    let mut groups: Vec<BandGroup>;
    let mut tagged_by_the_ball = false;
    if chain_edges.len() == 1 {
        let mut divides;
        (tags, divides) = if known_room.is_some() {
            band_side_tagging_ends::tag_band(
                positions,
                indices,
                &chain_edges[0],
                &**is_feature_edge,
                field,
                snap,
                true,
            )?
        } else {
            band_side_tagging_ends::tag_sides(
                indices,
                &chain_edges[0],
                &**is_feature_edge,
                true,
                Some(positions),
                end_rays[0],
                SideReach {
                    ends: EndReach {
                        end_reach: None,
                        end_face_reach: Some(field),
                    },
                    side_reach: None,
                },
            )?
        };
        // Floods that join the two sides give way to the ball's own sides, which never do.
        let from_the_ball = match ball_sides {
            Some(sides) if !divides => {
                tags = rolling_ball_band_tags::tags_of(
                    positions,
                    indices,
                    &chain_edges[0],
                    closed[0],
                    &**is_feature_edge,
                    sides,
                    end_rays[0],
                    field,
                )?;
                if let Some((planes, [a, b])) = ball_sweep {
                    let swept = rolling_ball_sweep::retag_what_the_contacts_never_sweep(
                        positions,
                        indices,
                        &chain_edges[0],
                        &mut tags,
                        [a, b],
                        planes,
                        snap,
                    )?;
                    if let Sweep::FoldsBack { point, side } = swept {
                        let (own, other) = if side == 0 { ("A", "B") } else { ("B", "A") };
                        let why = format!(
                            "chain 0: side {own}'s band runs behind side {other}'s contact at \
                             ({}, {}, {})",
                            point.x, point.y, point.z
                        );
                        rolling_ball_routing::record_refusal(&why);
                        return Ok(Sides::Refused(Box::new(BandGapResult {
                            refusal: BandGapRefusal::RollingBall,
                            rolling_ball_refusal: Some(why),
                            ..BandGapResult::default()
                        })));
                    }
                }
                divides = true;
                true
            }
            _ => false,
        };
        tagged_by_the_ball = from_the_ball;
        if !divides && closed[0] {
            return Ok(Sides::Refused(Box::new(BandGapResult {
                refusal: BandGapRefusal::ChainDoesNotDivideItsSides,
                room: 0.0,
                ..BandGapResult::default()
            })));
        }

        // An end face NARROWER than this size's setback is flowed over onto the next face - but
        // only where the end face's own room would refuse the size and the flowed ends take it.
        if known_room.is_none() && divides && !closed[0] && !from_the_ball {
            tags = narrow_face_flow::tag_flowed_where_it_builds(
                positions,
                indices,
                &chain_edges[0],
                closed[0],
                Rc::clone(is_feature_edge),
                end_rays[0],
                field,
                field_at_0,
                room_units,
                size,
                snap,
                seam_epsilon,
                profile,
                fillet_segments,
                tags,
                unflowed_room,
            )?;
        }

        // An OPEN chain whose sides fall in ONE region is a crease that DIES OUT, carried as ONE
        // group; what the CUT leaves decides whether it can be filled.
        groups = if divides {
            vec![one_chain_group(1), one_chain_group(2)]
        } else {
            vec![one_chain_group(1)]
        };

        // An open chain ending on an END FACE: each end region is a group of its own.
        if divides && !closed[0] {
            for end in [FIRST_END_TAG, SECOND_END_TAG] {
                if tags.contains(&end) {
                    groups.push(one_chain_group(end));
                }
            }
        }
    } else {
        let mut undivided;
        (tags, groups, undivided) = band_side_groups::group_sides(
            indices,
            chain_edges,
            &**is_feature_edge,
            true,
            Some(positions),
            Some(end_rays),
            Some(fields),
        )?;
        step_corner_carry::join_the_larger_setback_piece(
            positions,
            indices,
            chain_edges,
            unit_setbacks,
            &tags,
            &mut groups,
            step_corners,
        );

        // A BOTH-SIDED member divides its sides and is built; only with the switch off does it
        // refuse by the name it had before.
        if undivided < 0 && !both_sided_members_build() {
            undivided = groups
                .iter()
                .flat_map(|g| g.both_sided.iter().copied())
                .min()
                .unwrap_or(-1);
        }

        if undivided < 0 {
            undivided = band_side_tagging::first_both_sided_at_a_mixed_corner(
                Some(positions),
                indices,
                chain_edges,
                &groups,
            )?;
        }

        if undivided >= 0 {
            return Ok(Sides::Refused(Box::new(BandGapResult {
                refusal: BandGapRefusal::ChainDoesNotDivideItsSides,
                room: 0.0,
                offending_chain: undivided,
                ..BandGapResult::default()
            })));
        }
    }

    Ok(Sides::Tagged(tags, groups, tagged_by_the_ball))
}
