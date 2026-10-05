//! The SECTIONS half of the chain sweep (`Bevel/BevelChainSweep.Sections.cs`, part of the C#
//! `internal static partial class BevelChainSweep`): one swept segment per chain edge, its
//! cross section drawn in the edge's own frame ([`try_build_segments`]) - or, where one of the
//! edge's faces is a facet of a round, square to the rolling ball's spine
//! ([`try_stand_on_the_round`]).

use vector_math::{Vector2, Vector3};

use super::bevel_chain_sweep_mesh::SweptSegment;
use super::bevel_round_faces::BevelRoundFaces;
use super::{bevel_edge_tool_builder, bevel_profile_polygon};
use super::{BevelAdvisory, BevelAdvisoryCode, BevelSweptSection};
use crate::topology::{ChainEdge, EdgeConvexity};

/// C#'s short overload of `TryBuildSegments`: [`try_build_segments_on_rounds`] without saying
/// whether any segment stood on a round.
#[allow(clippy::too_many_arguments)]
pub fn try_build_segments(
    chain: &[ChainEdge],
    size: f64,
    segments: i32,
    round_profile: bool,
    section: BevelSweptSection,
    back_depth: f64,
    rounds: Option<&BevelRoundFaces>,
) -> Result<Vec<SweptSegment>, BevelAdvisory> {
    try_build_segments_on_rounds(
        chain,
        size,
        segments,
        round_profile,
        section,
        back_depth,
        rounds,
    )
    .map(|(swept, _)| swept)
}

/// One swept segment per chain edge - each edge's `section` (tool, void wedge or ball) drawn in
/// its frame, every section reporting the tool's tangent distance so all stop in the same place -
/// and whether any of them stood on a round. Every profile is then wound the same way, so the faces
/// sewn from them come out facing outward. Refused where the run mixes convexity, an edge has no
/// length, a section cannot be drawn, or a section's point count changes along the run.
#[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
pub fn try_build_segments_on_rounds(
    chain: &[ChainEdge],
    size: f64,
    segments: i32,
    round_profile: bool,
    section: BevelSweptSection,
    back_depth: f64,
    rounds: Option<&BevelRoundFaces>,
) -> Result<(Vec<SweptSegment>, bool), BevelAdvisory> {
    let mut swept: Vec<SweptSegment> = Vec::with_capacity(chain.len());
    let mut stood_on_a_round = false;

    for edge in chain {
        if edge.edge.convexity != chain[0].edge.convexity {
            // A cutter and a bead are mirror images about the edge, so one swept profile cannot be
            // both; the caller splits runs at their convexity changes.
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::RunMixedConvexity,
                vec![edge.from().into()],
            ));
        }

        let along = edge.to() - edge.from();

        if !(along.length() > 0.0) {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::EdgeHasNoLength,
                vec![],
            ));
        }

        let on_the_round = match rounds {
            Some(rounds) if round_profile => try_stand_on_the_round(edge, rounds, size, section),
            _ => None,
        };

        let (left_normal, right_normal, x_axis, y_axis, z_axis, corner) = match on_the_round {
            Some(stood) => {
                stood_on_a_round = true;
                stood
            }
            None => {
                let (left_normal, right_normal) = bevel_edge_tool_builder::try_get_profile_normals(
                    edge.edge.convexity,
                    edge.left_normal(),
                    edge.right_normal(),
                )?;

                // Left and right rather than face 0 and face 1, so the same physical surface is the
                // same profile point on every ring.
                let (x_axis, y_axis, z_axis) = bevel_edge_tool_builder::try_build_frame(
                    left_normal,
                    right_normal,
                    edge.direction(),
                )?;

                (
                    left_normal,
                    right_normal,
                    x_axis,
                    y_axis,
                    z_axis,
                    Vector2::ZERO,
                )
            }
        };

        let (mut profile, tangent_distance) = match section {
            BevelSweptSection::VoidWedge | BevelSweptSection::VoidWedgeOnTheFaces => {
                bevel_edge_tool_builder::try_build_void_wedge_cross_section(
                    left_normal,
                    right_normal,
                    x_axis,
                    y_axis,
                    z_axis,
                    size,
                    round_profile,
                    section == BevelSweptSection::VoidWedge,
                )?
            }
            BevelSweptSection::Ball => bevel_edge_tool_builder::try_build_ball_cross_section(
                left_normal,
                right_normal,
                x_axis,
                y_axis,
                z_axis,
                size,
                segments,
                round_profile,
            )?,
            _ => bevel_edge_tool_builder::try_build_cross_section(
                left_normal,
                right_normal,
                x_axis,
                y_axis,
                z_axis,
                size,
                segments,
                round_profile,
                back_depth,
            )?,
        };

        if !swept.is_empty() && profile.len() != swept[0].profile.len() {
            return Err(BevelAdvisory::new(
                BevelAdvisoryCode::CrossSectionChanged,
                vec![],
            ));
        }

        if corner != Vector2::ZERO {
            // Over a round the section's corner is not the chain vertex, so the whole profile is
            // carried there.
            for point in profile.iter_mut() {
                *point += corner;
            }
        }

        swept.push(SweptSegment::new(
            edge.from(),
            edge.to(),
            z_axis,
            x_axis,
            y_axis,
            profile,
            tangent_distance,
        ));
    }

    // Every ring is walked in the same order, so one polygon's winding decides the whole tool's.
    if bevel_profile_polygon::signed_area(&swept[0].profile) < 0.0 {
        for segment in swept.iter_mut() {
            segment.profile.reverse();
            segment.profile_reversed = true;
        }
    }

    Ok((swept, stood_on_a_round))
}

/// Where exactly one of a CONCAVE edge's two faces is a facet (or tangent plane) of a round, the
/// section's frame square to the rolling ball's SPINE at the edge's middle, and the wedge's apex
/// in that frame: `(left normal, right normal, x, y, z, corner)`. `None` (the planar section
/// stands) where the edge is not concave, both or neither face is a round's, the ball's station
/// cannot be solved, it has not rolled off the crease's own facet (a SOURCE round), or the two
/// surfaces' lines are parallel in the section.
pub fn try_stand_on_the_round(
    edge: &ChainEdge,
    rounds: &BevelRoundFaces,
    size: f64,
    section: BevelSweptSection,
) -> Option<(Vector3, Vector3, Vector3, Vector3, Vector3, Vector2)> {
    if edge.edge.convexity != EdgeConvexity::Concave {
        return None;
    }

    let on_the_left = rounds
        .try_read_face(edge.left_face())
        .map(|(round, _)| round);
    let on_the_right = rounds
        .try_read_face(edge.right_face())
        .map(|(round, _)| round);

    // Exactly one side: the construction is a round against a PLANE.
    if on_the_left.is_none() == on_the_right.is_none() {
        return None;
    }

    let round_is_on_the_left = on_the_left.is_some();
    let round = on_the_left.or(on_the_right)?;
    let slope_normal = if round_is_on_the_left {
        edge.right_normal()
    } else {
        edge.left_normal()
    };
    let anchor = (edge.from() + edge.to()) * 0.5;

    let station = round.try_find_station(anchor, edge.from(), slope_normal, size)?;
    let (phi, mut tangent, contact) = (station.phi, station.tangent, station.contact);

    // THE BALL HAS TO HAVE ROLLED OFF THE CREASE'S OWN FACET - asked of a SOURCE round only.
    let round_face_normal = if round_is_on_the_left {
        edge.left_normal()
    } else {
        edge.right_normal()
    };

    if rounds.read_from_the_source_mesh()
        && !round.contact_has_rolled_off_the_creases_facet(contact, edge.from(), round_face_normal)
    {
        return None;
    }

    let round_normal = round.radial(phi);

    // ALONG THE WALK: the spine's tangent comes back pointing either way.
    if tangent.dot(edge.direction()) < 0.0 {
        tangent = -tangent;
    }

    let (left_normal, right_normal) = bevel_edge_tool_builder::try_get_profile_normals(
        edge.edge.convexity,
        if round_is_on_the_left {
            round_normal
        } else {
            edge.left_normal()
        },
        if round_is_on_the_left {
            edge.right_normal()
        } else {
            round_normal
        },
    )
    .ok()?;
    let (x_axis, y_axis, z_axis) =
        bevel_edge_tool_builder::try_build_frame(left_normal, right_normal, tangent).ok()?;

    // The apex, solved in the section's own 2-D frame, where both surface normals lie.
    let slope_line = Vector2::new(slope_normal.dot(x_axis), slope_normal.dot(y_axis));
    let round_line = Vector2::new(round_normal.dot(x_axis), round_normal.dot(y_axis));
    let determinant = (slope_line.x * round_line.y) - (slope_line.y * round_line.x);

    if determinant.abs() < 1e-9 {
        return None;
    }

    let to_slope = (edge.from() - anchor).dot(slope_normal);
    let mut to_round = (contact - anchor).dot(round_normal);

    // THE WEDGE'S ROUND WALL IS BURIED AT THE CREASE, not at the ball's contact - a SOURCE round
    // only.
    if rounds.read_from_the_source_mesh()
        && (section == BevelSweptSection::VoidWedge
            || section == BevelSweptSection::VoidWedgeOnTheFaces)
    {
        to_round = 0.0;
    }

    let corner = Vector2::new(
        ((to_slope * round_line.y) - (to_round * slope_line.y)) / determinant,
        ((slope_line.x * to_round) - (round_line.x * to_slope)) / determinant,
    );

    Some((left_normal, right_normal, x_axis, y_axis, z_axis, corner))
}
