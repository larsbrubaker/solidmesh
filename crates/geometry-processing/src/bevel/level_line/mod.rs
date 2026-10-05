//! `MatterCAD.GeometryProcessing/Bevel/LevelLine`: the level-line bevel build. Ported so far: its
//! leaf records - why a gap was refused ([`BandGapRefusal`]), the band's profile
//! ([`BandProfile`]), a side group ([`BandGroup`]), the per-side loop reading
//! ([`ChainSideLoops`]), the chain-edge error ([`ChainEdgeNotOnTwoTriangles`]), a field reading
//! ([`FieldSample`]), why a fill was refused ([`GapFillRefusal`]), a gap boundary ([`GapLoop`]),
//! the hole filler's answer ([`HoleFillResult`]), which reading named the room
//! ([`RoomReadingKind`]) and what it was about ([`RoomSource`]), one candidate the room is the
//! least of ([`RoomCandidate`]) - the setback a size asks
//! ([`chain_setback`]), the curved end face test ([`curved_end_face`]), the Lipschitz bounds
//! ([`field_bounds`]), the field traits ([`ScalarField`], [`ChainDistanceField`]), the chain's
//! segment box tree ([`segment_box_tree`]), the convex cell bound the ridge searches prune with
//! ([`convex_cell_bound`]) and the flat end-face fill ([`flat_hole_fill`]). The build itself
//! lands with the cyclic bevel core, whose record types come first: a crossed edge's root
//! ([`EdgeRoot`]), a pick's fault ([`PickFault`]), a vertex's side of the line ([`VertexSide`])
//! and the work phase and progress ([`LevelLineWorkPhase`], [`LevelLineProgress`]).

mod band_gap_refusal;
mod band_group;
mod band_profile;
mod chain_edge_not_on_two_triangles;
pub mod chain_setback;
mod chain_side_loops;
pub mod convex_cell_bound;
pub mod curved_end_face;
mod edge_root;
pub mod field_bounds;
mod field_sample;
pub mod flat_hole_fill;
mod gap_fill_refusal;
mod gap_loop;
mod hole_fill_result;
mod pick_fault;
mod room_candidate;
mod room_reading_kind;
mod room_source;
mod scalar_field;
pub mod segment_box_tree;
mod vertex_side;
mod work_budget;

pub use band_gap_refusal::BandGapRefusal;
pub use band_group::BandGroup;
pub use band_profile::BandProfile;
pub use chain_edge_not_on_two_triangles::ChainEdgeNotOnTwoTriangles;
pub use chain_side_loops::ChainSideLoops;
pub use edge_root::EdgeRoot;
pub use field_sample::FieldSample;
pub use gap_fill_refusal::GapFillRefusal;
pub use gap_loop::GapLoop;
pub use hole_fill_result::{HoleFillFailure, HoleFillResult, HoleFillTriangle};
pub use pick_fault::PickFault;
pub use room_candidate::RoomCandidate;
pub use room_reading_kind::RoomReadingKind;
pub use room_source::RoomSource;
pub use scalar_field::{ChainDistanceField, FieldMaximum, FieldMinimum, ScalarField};
pub use segment_box_tree::SegmentNode;
pub use vertex_side::VertexSide;
pub use work_budget::{LevelLineProgress, LevelLineWorkPhase};
