//! The side tagging of ONE open chain flowed over a face narrower than the size's setback onto the
//! next face (an end face, by the side tagging's end flow, or a side onto its gentle convex
//! neighbour), taken only where the plain tagging refuses the size and the flowed one takes it.
//! Ports `Bevel/LevelLine/NarrowFaceFlow.cs` (a C# static class split from `BandGap`); the rooms it
//! compares are read by `band_room::room_of_cut_mesh`.

use std::cell::RefCell;
use std::collections::HashSet;
use std::rc::Rc;

use vector_math::Vector3;

use super::band_gap_reading::ReadRoomError;
use super::band_room::{self, RoomWithinError};
use super::band_side_tagging::{FIRST_END_TAG, SECOND_END_TAG};
use super::band_side_tagging_ends::{self, EndReach, SideReach};
use super::foreign_edge_room_readings::FieldAt;
use super::level_line_mesh_edits::LevelLineMeshError;
use super::seam_edges::FeatureEdgeTest;
use super::triangle_edges;
use super::{BandProfile, ChainField, LevelLineMesh};

/// C# `NarrowFaceFlow.TagFlowedWhereItBuilds`: the tagging the build cuts: the flowed one where it
/// holds a size the plain `tags` refuse, setting `unflowed_room` to the plain tagging's room;
/// otherwise `tags`.
#[allow(clippy::too_many_arguments)]
pub fn tag_flowed_where_it_builds(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain_edges: &[(i32, i32)],
    closed: bool,
    is_feature_edge: FeatureEdgeTest,
    end_rays: (bool, bool),
    field: &ChainField,
    field_at: FieldAt,
    unit_setbacks: &[f64],
    size: f64,
    snap: f64,
    seam_epsilon: f64,
    profile: BandProfile,
    fillet_segments: i32,
    tags: Vec<i32>,
    unflowed_room: &mut Option<f64>,
) -> Result<Vec<i32>, RoomWithinError> {
    let end_reach = EndReach {
        end_reach: Some(field),
        end_face_reach: None,
    };
    let (flowed, _) = band_side_tagging_ends::tag_sides(
        indices,
        chain_edges,
        is_feature_edge.as_ref(),
        true,
        Some(positions),
        end_rays,
        SideReach {
            ends: end_reach,
            side_reach: None,
        },
    )?;

    let an_end_flows_to_the_other = |tagging: &[i32]| {
        let first = chain_edges[0].0;
        let last = chain_edges[chain_edges.len() - 1].1;
        for (i, &(a, b, c)) in indices.iter().enumerate() {
            let at_first = a == first || b == first || c == first;
            let at_last = a == last || b == last || c == last;
            if (at_last && tagging[i] == FIRST_END_TAG && tags[i] != FIRST_END_TAG)
                || (at_first && tagging[i] == SECOND_END_TAG && tags[i] != SECOND_END_TAG)
            {
                return true;
            }
        }

        false
    };

    let room_on_the_tagging = |tagging: &[i32]| -> Result<f64, RoomWithinError> {
        let mut reading = match LevelLineMesh::from_triangles(positions, indices, Some(tagging)) {
            Ok(mesh) => mesh,
            Err(LevelLineMeshError::Work(work)) => return Err(RoomWithinError::Work(work)),
            Err(other) => return Err(RoomWithinError::Read(ReadRoomError::Mesh(other))),
        };
        let picked: HashSet<(i32, i32)> = chain_edges
            .iter()
            .map(|&(from, to)| triangle_edges::key(from, to))
            .collect();
        let feature = is_feature_edge.clone();
        reading.is_chain_edge = Some(Rc::new(move |a: i32, b: i32| {
            picked.contains(&triangle_edges::key(a, b)) || feature(a, b)
        }));
        let (room, _) = band_room::room_of_cut_mesh(
            &Rc::new(RefCell::new(reading)),
            chain_edges,
            field_at,
            unit_setbacks,
            closed,
            snap,
            seam_epsilon,
            profile,
            fillet_segments,
            None,
            None,
        )?;
        Ok(room)
    };

    // An end that flows round to the chain's OTHER end is not an end at all: the band has cut
    // through the part's whole thickness there (the apex prism of EndFaceRoomTests at 1.1x its
    // end-face room, 0.33 of setback on a part 0.3 thick - one end region took the back wall and
    // the far cap, and the cut split the part in two). That stays a refusal by size, naming the end
    // face's room. C#'s `&&` chain reads each room only when the tests before it pass.
    let mut room_unflowed = f64::NAN;
    if flowed != tags && !an_end_flows_to_the_other(&flowed) {
        room_unflowed = room_on_the_tagging(&tags)?;
        if size > room_unflowed && size <= room_on_the_tagging(&flowed)? {
            *unflowed_room = Some(room_unflowed);
            return Ok(flowed);
        }
    }

    // A SIDE narrower than the setback flows onto its gentle convex neighbour the same way
    // (decision 4; the side tagging's FlowsOntoAGentleNeighbour) - a chamfer of 2 on a 40-gon wall
    // crease, whose 1.57 facet was read as a wall. Taken on the same terms as the end flow: only
    // where the plain tagging refuses the size and the flowed one takes it.
    let (side_flowed, side_flow_divides) = band_side_tagging_ends::tag_sides(
        indices,
        chain_edges,
        is_feature_edge.as_ref(),
        true,
        Some(positions),
        end_rays,
        SideReach {
            ends: end_reach,
            side_reach: Some(field),
        },
    )?;
    if side_flow_divides && side_flowed != tags && !an_end_flows_to_the_other(&side_flowed) {
        if room_unflowed.is_nan() {
            room_unflowed = room_on_the_tagging(&tags)?;
        }

        if size > room_unflowed && size <= room_on_the_tagging(&side_flowed)? {
            *unflowed_room = Some(room_unflowed);
            return Ok(side_flowed);
        }
    }

    Ok(tags)
}
