//! One continuous tool swept along a whole chain of edges, mitred at every joint - the plan's 5.1
//! per-chain sweep: which section a sweep carries ([`BevelSweptSection`]), its two rulers, and the
//! sweep itself ([`try_create`]); the sections, rings and sewing are its sibling files
//! ([`super::bevel_chain_sweep_sections`], [`super::bevel_chain_sweep_rings`],
//! [`super::bevel_chain_sweep_mesh`]).

/// Which section a sweep carries along the chain. All of them are swept by exactly the same code -
/// the same per-segment frames, mitre planes and end caps - which is the point of asking for the
/// wedge and the ball here rather than building a second solid some other way.
use polygon_mesh::csg::boolean_processing::{classify_boolean_operand, BooleanOperandVerdict};
use polygon_mesh::Mesh;
use vector_math::{cs_math, Matrix4X4};

use super::bevel_chain_sweep_mesh::build_mesh;
use super::bevel_chain_sweep_rings::try_build_rings;
use super::bevel_chain_sweep_sections::try_build_segments_on_rounds;
use super::bevel_round_faces::BevelRoundFaces;
use super::{
    bevel_edge_tool_builder, bevel_overshoot_rings, bevel_round_entry, bevel_straight_run,
    bevel_sweep_back_depth, bevel_sweep_fold_check, BevelAdvisory, BevelAdvisoryCode,
    BevelChainEnds,
};
use crate::topology::ChainEdge;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum BevelSweptSection {
    /// The bevel tool itself - the cutter on a convex run, the bead on a concave one.
    #[default]
    Tool = 0,
    /// The wedge the profile is drawn in rather than the profile
    /// (`BevelEdgeToolBuilder.TryBuildVoidWedgeCrossSection`), which on a concave run is the void
    /// between the two selected faces.
    VoidWedge,
    /// The same wedge with its two lateral walls left ON the crease's faces rather than buried a
    /// hair inside them - how the SQUARED sweep alone is drawn, because its wedge is the only one
    /// the clip unions with a 5.3 corner's void, whose own walls lie on those same faces.
    VoidWedgeOnTheFaces,
    /// The rolling BALL itself rather than the material it clears
    /// (`BevelEdgeToolBuilder.TryBuildBallCrossSection`) - swept, the tube whose surface is the
    /// fillet. `BevelPairCornerTrim` wants it, to take one chain's fillet out of a neighbouring
    /// chain's overshooting cutter.
    Ball,
}

/// The least cosine a joint's mitre may stand at against either segment before the joint is too
/// sharp to mitre.
pub const MINIMUM_MITER_COSINE: f64 = 0.1;

/// The fewest segments a closed run may have.
pub const MINIMUM_CLOSED_SEGMENTS: i32 = 3;

/// One tool swept along the whole `chain`, mitred at every joint, ended by `ends` (overshooting
/// where `None`), at `size` - or why not. Where the tool folds over a round, or (at full back
/// depth) comes back self-intersecting or not a usable solid, it is retried ONCE at the back depth
/// the chain's turn allows ([`bevel_sweep_back_depth::retry_depth`]), the first refusal standing
/// where the retry fails too (the retry's own reason added to `warnings` where it differs).
#[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
pub fn try_create(
    chain: &[ChainEdge],
    is_closed: bool,
    ends: Option<&BevelChainEnds>,
    size: f64,
    segments: i32,
    round_profile: bool,
    section: BevelSweptSection,
    start_minimum_overshoot: f64,
    end_minimum_overshoot: f64,
    mut warnings: Option<&mut Vec<BevelAdvisory>>,
    back_depth: f64,
    rounds: Option<&BevelRoundFaces>,
) -> Result<Mesh, BevelAdvisory> {
    let (first, back_reach, turn_radius, stood_on_a_round) = try_create_at_depth(
        chain,
        is_closed,
        ends,
        size,
        segments,
        round_profile,
        back_depth,
        section,
        start_minimum_overshoot,
        end_minimum_overshoot,
        warnings.as_deref_mut(),
        rounds,
    );

    let first_failure = match first {
        Ok(mesh) => return Ok(mesh),
        Err(failure) => failure,
    };

    let fold_over_a_round =
        first_failure.code() == BevelAdvisoryCode::SizeFoldsToolAtCurve && stood_on_a_round;

    if !(fold_over_a_round
        || (back_depth == 0.0
            && (first_failure.code() == BevelAdvisoryCode::ToolSelfIntersects
                || first_failure.code() == BevelAdvisoryCode::ToolNotUsableSolid)))
    {
        return Err(first_failure);
    }

    let shallower_back =
        bevel_sweep_back_depth::retry_depth(chain, is_closed, size, back_reach, turn_radius);

    if !(shallower_back > 0.0) || (back_depth > 0.0 && shallower_back >= back_depth) {
        return Err(first_failure);
    }

    let (shallower, _, _, _) = try_create_at_depth(
        chain,
        is_closed,
        ends,
        size,
        segments,
        round_profile,
        shallower_back,
        section,
        start_minimum_overshoot,
        end_minimum_overshoot,
        warnings.as_deref_mut(),
        rounds,
    );

    match shallower {
        Ok(mesh) => Ok(mesh),
        Err(retry_failure) => {
            if let Some(warnings) = warnings {
                if retry_failure != first_failure {
                    warnings.push(retry_failure);
                }
            }

            Err(first_failure)
        }
    }
}

/// One attempt at the sweep at `back_depth`: the answer, and what the retry reads off it - the
/// sections' back reach, the run's tightest turn and whether any section stood on a round.
#[allow(clippy::too_many_arguments)]
fn try_create_at_depth(
    chain: &[ChainEdge],
    is_closed: bool,
    ends: Option<&BevelChainEnds>,
    size: f64,
    segments: i32,
    round_profile: bool,
    back_depth: f64,
    section: BevelSweptSection,
    start_minimum_overshoot: f64,
    end_minimum_overshoot: f64,
    warnings: Option<&mut Vec<BevelAdvisory>>,
    rounds: Option<&BevelRoundFaces>,
) -> (Result<Mesh, BevelAdvisory>, f64, f64, bool) {
    let mut back_reach = 0.0;
    let mut turn_radius = f64::INFINITY;
    let mut stood_on_a_round = false;

    let result = (|| {
        if chain.is_empty() {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::NoEdgesResolved,
                vec![],
            ));
        }

        bevel_edge_tool_builder::try_validate_size(size)?;

        if is_closed && (chain.len() as i32) < MINIMUM_CLOSED_SEGMENTS {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::LoopTooShortToEnclose,
                vec![],
            ));
        }

        // Cut where a run crosses onto a round; an open run's overshoot bounds are then held to
        // half its new end pieces.
        let subdivided = bevel_round_entry::subdivide(chain, rounds, size, round_profile);
        let chain: &[ChainEdge] = subdivided.as_deref().unwrap_or(chain);

        let half_of = |piece: &ChainEdge, bound: f64| {
            if bound.is_infinite() {
                bound
            } else {
                cs_math::min_f64(bound, (piece.to() - piece.from()).length() / 2.0)
            }
        };

        let held;
        let ends = match ends {
            Some(ends) if !is_closed && subdivided.is_some() => {
                held = BevelChainEnds {
                    start_maximum_overshoot: half_of(&chain[0], ends.start_maximum_overshoot),
                    end_maximum_overshoot: half_of(
                        &chain[chain.len() - 1],
                        ends.end_maximum_overshoot,
                    ),
                    ..ends.clone()
                };
                Some(&held)
            }
            other => other,
        };

        let (mut swept, stood) = try_build_segments_on_rounds(
            chain,
            size,
            segments,
            round_profile,
            section,
            back_depth,
            rounds,
        )?;
        stood_on_a_round = stood;

        back_reach = bevel_sweep_back_depth::measure_back_reach(&swept);
        turn_radius = bevel_sweep_back_depth::turn_radius_of_the_segments(&swept, is_closed);

        if !is_closed
            && swept.len() > 1
            && (swept[0].from - swept[swept.len() - 1].to).length() < 1e-9
        {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::LoopSurfacesSwapped,
                vec![swept[0].from.into()],
            ));
        }

        let point_count = swept[0].profile.len();
        let overshoot;
        let resolved_ends = match ends {
            Some(ends) => ends,
            None => {
                overshoot = BevelChainEnds::overshoot();
                &overshoot
            }
        };

        let mut rings = try_build_rings(
            &mut swept,
            is_closed,
            resolved_ends,
            size,
            point_count,
            start_minimum_overshoot,
            end_minimum_overshoot,
            section,
            warnings,
        )?;
        bevel_sweep_fold_check::try_check_for_folds(
            &swept,
            resolved_ends,
            is_closed,
            size,
            point_count,
        )?;

        if !is_closed {
            rings = bevel_straight_run::drop_rings_the_cap_has_passed(rings, &swept, resolved_ends);

            rings = bevel_overshoot_rings::add(
                rings,
                &swept[0],
                &swept[swept.len() - 1],
                resolved_ends,
                point_count,
                section,
            );
        }

        let mesh = build_mesh(
            &swept,
            &rings,
            is_closed,
            point_count,
            (section == BevelSweptSection::Tool).then_some(resolved_ends),
        )
        .ok_or_else(|| BevelAdvisory::new(BevelAdvisoryCode::ToolCouldNotClose, vec![]))?;

        match classify_boolean_operand(&mesh, Matrix4X4::IDENTITY, false) {
            BooleanOperandVerdict::Clean => Ok(mesh),
            BooleanOperandVerdict::SelfIntersecting => Err(BevelAdvisory::new(
                BevelAdvisoryCode::ToolSelfIntersects,
                vec![],
            )),
            BooleanOperandVerdict::Refused => Err(BevelAdvisory::new(
                BevelAdvisoryCode::ToolNotUsableSolid,
                vec![],
            )),
        }
    })();

    (result, back_reach, turn_radius, stood_on_a_round)
}
