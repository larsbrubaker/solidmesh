//! The band gap's room reading. Ports `Bevel/LevelLine/BandGap.Reading.cs` (a partial of the C#
//! static class `BandGap`, free functions here): the half of the band gap that READS and builds
//! nothing - what room a pick has, before any band is cut ([`room_of`], [`room_of_pick`],
//! [`read_room`], [`read_room_of_pick`]) and the margin a room reading keeps under the room
//! ([`room_margin`]). The readings themselves are `band_room.rs` (one chain) and
//! `band_room_read_pick.rs` (several).

use std::rc::Rc;

use vector_math::Vector3;

use super::band_room::{self, RoomWithinError};
use super::chain_field_from_polyline::ChainFieldArgumentError;
use super::level_line_mesh_edits::LevelLineMeshError;
use super::pick_validation::{self, PickArgumentError};
use super::seam_edges::FeatureEdgeTest;
use super::{band_room_read_pick, chain_on_the_mesh};
use super::{BandProfile, RoomReading, SeamEdges, WorkBudget, WorkError};

/// Why a room reading has no reading at all: C#'s exceptions. A refusal or a reading that ran out
/// of work is a [`RoomReading`]; these are a caller's bug or the caller's own cancel.
#[derive(Debug, Clone)]
pub enum ReadRoomError {
    /// The pick's arguments are malformed (C# `ArgumentException` from `CheckPickArguments`).
    Arguments(PickArgumentError),
    /// The caller cancelled (C# `OperationCanceledException`).
    Cancelled,
    /// The chain's polyline builds no field (C# `ArgumentException` from `ChainField.FromPolyline`).
    ChainField(ChainFieldArgumentError),
    /// The mesh is malformed (C# `ArgumentException` from `LevelLineMesh.FromTriangles`).
    Mesh(LevelLineMeshError),
}

impl From<PickArgumentError> for ReadRoomError {
    fn from(error: PickArgumentError) -> Self {
        ReadRoomError::Arguments(error)
    }
}

/// C# `BandGap.RoomOf(chain)`: the room, without building anything: the largest size the picked
/// sides' WIDTH allows.
///
/// The room is a statement about the PART and the PICK - the foreign features bounding the picked
/// sides and how much of itself each side holds - and not about the size asked for, so a drag reads
/// it once and hands it to every build through the build's known room. It is a reading of WIDTH
/// only: where a side PINCHES, two stretches of one chain closing across a neck, the field between
/// them is a saddle that no width reading looks at, and the build refuses by size below this
/// number, naming the smaller room then - nothing is spent here looking for a pinch that most picks
/// do not have. Read off the mesh as it came in, before any refinement - and a build handed no room
/// reads it there too, on the same mesh at the same reference size, so the two are one number.
/// They would NOT be otherwise: the greatest field value over a parent triangle is indeed the
/// greatest over its children, but neither reading finds that greatest exactly - each is a sample
/// proved within a search tolerance, and on a refined mesh the search starts from different
/// vertices and stops at a tolerance scaled to the size asked, so it lands on a different number
/// (2.29167 against 2.34229 on an L face of arm 4).
///
/// Returns the largest size the width allows, positive infinity where nothing bounds the sides, or
/// zero when the chain does not divide its sides - the one refusal that is not about size. C#'s
/// optional `snap = 1e-9` is passed explicitly.
#[allow(clippy::too_many_arguments)]
pub fn room_of(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    chain_is_closed: bool,
    is_feature_edge: Option<FeatureEdgeTest>,
    profile: BandProfile,
    snap: f64,
) -> Result<f64, ReadRoomError> {
    Ok(read_room(
        positions,
        indices,
        chain,
        chain_is_closed,
        is_feature_edge,
        profile,
        snap,
        None,
        0.0,
        f64::INFINITY,
    )?
    .room)
}

/// C# `BandGap.RoomOf(chains)`: the room of a PICK - one chain or several - without building
/// anything. Positive infinity where nothing bounds the sides, or zero for every refusal that is
/// not about size (the chain not dividing its sides, a chain edge not standing on two triangles,
/// two chains touching).
#[allow(clippy::too_many_arguments)]
pub fn room_of_pick(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: Option<FeatureEdgeTest>,
    profile: BandProfile,
    snap: f64,
) -> Result<f64, ReadRoomError> {
    Ok(read_room_of_pick(
        positions,
        indices,
        chains,
        closed,
        is_feature_edge,
        profile,
        snap,
        None,
        f64::NAN,
        0.0,
        f64::INFINITY,
    )?
    .room)
}

/// C# `BandGap.ReadRoom(chain)`: the room, spending no more than the given work - see
/// [`WorkBudget`]. A reading that ran out of work refuses BY NAME (`read` false) rather than
/// failing or spinning, and its room is NaN, which no size compares above. `budget` `None` takes
/// the default for the input's size.
#[allow(clippy::too_many_arguments)]
pub fn read_room(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    chain_is_closed: bool,
    is_feature_edge: Option<FeatureEdgeTest>,
    profile: BandProfile,
    snap: f64,
    budget: Option<Rc<WorkBudget>>,
    seam_epsilon: f64,
    sharp_turn_degrees: f64,
) -> Result<RoomReading, ReadRoomError> {
    // The list of one: one chain is a pick like any other, and takes the same validation.
    read_room_of_pick(
        positions,
        indices,
        &[chain.to_vec()],
        &[chain_is_closed],
        is_feature_edge,
        profile,
        snap,
        budget,
        f64::NAN,
        seam_epsilon,
        sharp_turn_degrees,
    )
}

/// C# `BandGap.ReadRoom(chains)`: the room of a PICK - one chain or several - spending no more than
/// the given work. A pick of one chain reads exactly what that chain reads, to the bit; a pick of
/// several reads ONE number for the whole pick - see `band_room_pair_room::room_of_pick` for the
/// three readings it is the least of.
///
/// The reading builds NOTHING, and a probe inside it - one topology-only build at the room, to
/// catch a pinch the widths cannot see - was tried and dropped: at the largest size the pick allows
/// the band covers the whole part, so that probe is the dearest build there is, and a
/// twenty-thousand-triangle cylinder rim went from under 1.7 s to about 4 s to learn nothing.
/// Pinches are found where they are paid for, in the pinch ladder search, only once a build has
/// actually pinched. `name_every_candidate_under` is the size a caller means to have every
/// too-tight reading NAMED for; NaN reads exactly what this always read.
#[allow(clippy::too_many_arguments)]
pub fn read_room_of_pick(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: Option<FeatureEdgeTest>,
    profile: BandProfile,
    snap: f64,
    budget: Option<Rc<WorkBudget>>,
    name_every_candidate_under: f64,
    seam_epsilon: f64,
    sharp_turn_degrees: f64,
) -> Result<RoomReading, ReadRoomError> {
    pick_validation::check_pick_arguments(positions, indices, chains, closed)?;
    // As in Build: the pick's seams stop being features once, and every reading below inherits it.
    let is_feature_edge = SeamEdges::not_the_chains_seam(
        Some(positions),
        Some(chains),
        Some(closed),
        is_feature_edge,
        seam_epsilon,
    );
    let budget = match budget {
        Some(budget) => budget,
        None => Rc::new(
            WorkBudget::new(WorkBudget::default_limit(indices.len() as i32))
                .expect("the default limit is a positive count of work"),
        ),
    };
    if budget.cancellation_token.is_cancelled() {
        return Err(ReadRoomError::Cancelled);
    }

    let (fault, _) = pick_validation::validate_on_the_input(
        positions,
        indices,
        chains,
        closed,
        is_feature_edge.as_deref(),
    );
    if let Some(fault) = fault {
        return Ok(fault.as_room_reading(&budget));
    }

    // The room is read on the chains the build cuts - split where their edges' setbacks step.
    // Validation has already found every chain edge on two triangles, so the split cannot fail on
    // one; were it to, the reading answers as the one-chain reading does for such an edge.
    let (chains, closed) = match chain_on_the_mesh::split_at_setback_steps(
        positions,
        indices,
        chains,
        closed,
        profile,
        sharp_turn_degrees,
        is_feature_edge.as_deref(),
    ) {
        Ok(split) => split,
        Err(bad) => {
            return Ok(RoomReading {
                read: true,
                room: 0.0,
                chain_edge_stands_on_two_triangles: false,
                offending_edge: (bad.from, bad.to),
                work_spent: budget.spent.get(),
                work_limit: budget.limit,
                ..RoomReading::default()
            })
        }
    };
    if chains.len() > 1 {
        return band_room_read_pick::read_room_of_pick(
            positions,
            indices,
            &chains,
            &closed,
            is_feature_edge,
            profile,
            snap,
            &budget,
            name_every_candidate_under,
            seam_epsilon,
            sharp_turn_degrees,
        )
        .map_err(|_: WorkError| ReadRoomError::Cancelled);
    }

    band_room::read_room_of_one(
        positions,
        indices,
        &chains[0],
        closed[0],
        is_feature_edge,
        profile,
        snap,
        &budget,
        seam_epsilon,
        sharp_turn_degrees,
    )
}

/// C# `BandGap.RoomMargin`: how much of the geometric room is not usable: the size that moves the
/// level line by TWO snaps, one for each side of the ridge of kept surface the strip closes on. A
/// strip narrower than that is not a strip - its two boundaries fall onto each other within the
/// snap and the side leaves no loop to fill.
///
/// The snap is a LENGTH and the room is a SIZE, and the one place a size becomes a distance in the
/// field is the setback: a size s moves the line back s * unitSetback, so a length of two snaps is
/// worth 2 * snap / unitSetback of size. Where the setback varies along the chain the LARGEST is
/// used, which is the smallest margin - the room is the largest size that builds, and taking more
/// off it than the geometry asks for would refuse sizes that do build. C#'s optional
/// `snap = 1e-9` is passed explicitly; zero for a pick with no setback.
pub fn room_margin(largest_unit_setback: f64, snap: f64) -> f64 {
    if largest_unit_setback > 0.0 {
        2.0 * snap / largest_unit_setback
    } else {
        0.0
    }
}

/// Keeps [`RoomWithinError`] reachable from the reading's error type (the one-chain reading's
/// argument faults travel inside it).
impl From<ReadRoomError> for RoomWithinError {
    fn from(error: ReadRoomError) -> Self {
        RoomWithinError::Read(error)
    }
}
