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
//! and the work phase and progress ([`LevelLineWorkPhase`], [`LevelLineProgress`]). The core's
//! further type definitions are landed too, methods to follow: the group field with run-outs and
//! its gate scope ([`BandGroupFieldsWithRunOuts`], [`BandGroupFieldsRunOutsScope`]), the bounded
//! search's cell bound and best-so-far ([`BoundedMaximumCellBound`],
//! [`BoundedMaximumCanonicalMaximum`]), a corner cap's last row ([`CornerCapLastRow`]), a corner
//! fill's loop piece ([`CornerFillPiece`]), a crease run-out and its field ([`CreaseRunOut`],
//! [`RunOutField`]), the refinement rules ([`LevelLineRule`]), a zip rail and its two senses
//! ([`ZipRail`], [`GapFillRailSenses`]), a hull clip's supporting plane ([`HullClipPlane`]), a
//! knife end's overrun ([`KnifeEndOverrunEnd`]), the route's watch record
//! ([`LevelLineChainWatch`]), the needle and split meters' records ([`NeedleMeterMadeBy`],
//! [`SplitMeterSplitKind`], [`SplitMeterSplitRecord`]), the open fill's rail and flat run-out
//! region ([`OpenGapFillOpenRail`], [`OpenGapFillFlatRunOutRegion`]), the pair-corner end-face field
//! and its scope ([`PairCornerEndFaceField`], [`PairCornerEndFaceFieldScope`]), a pick corner
//! ([`PickCorner`]), a pinch ladder's finding ([`PinchLadderSearchPinchLadder`]), a picked point's
//! field ([`PointField`]), where a point stands in a triangle ([`PointPlace`]), the seam reading
//! ([`SeamEdges`]) and the square-crease clip's scope
//! ([`SquareCreaseEndClipSquareCreaseClipsOffScope`]).
//!
//! The core's working types are landed as well, methods to follow: the chain's distance field
//! ([`ChainField`], with its end-ray bound [`EndRayBound`], mitre and end rays
//! [`ChainFieldExtension`], end clips [`ChainFieldEndClip`], query pieces [`ChainFieldMeshPiece`]
//! and smooth-joint scope [`ChainFieldSmoothJointsScope`]), the field over a group of chains
//! ([`PickField`]) and the one a room reading asks ([`RoomField`]), the working mesh
//! ([`LevelLineMesh`], [`LevelLineTriangle`]) and its per-vertex reading
//! ([`VertexFieldReading`]), the band cut's line and halves ([`LevelLineChain`],
//! [`BandCutResult`]), the foreign-edge filing and its box tree ([`ForeignEdgeRegions`],
//! [`ForeignEdgeRegionsChainGrowth`], [`ForeignEdgeIndex`], [`ForeignEdgeIndexNode`],
//! [`ForeignApproach`]), the growing-crease room ([`GrowingCreaseRuns`]), and the build's three
//! answers ([`BandGapResult`], [`GapFillResult`], [`RoomReading`]).

mod band_cut;
mod band_gap_refusal;
mod band_gap_result;
mod band_group;
mod band_group_fields;
mod band_profile;
pub mod band_room;
pub mod band_triangles;
mod bounded_maximum;
pub mod chain_approach;
mod chain_edge_not_on_two_triangles;
mod chain_field;
mod chain_field_end_clip;
mod chain_field_end_ray;
mod chain_field_mitre;
mod chain_field_queries;
pub mod chain_on_the_mesh;
pub mod chain_setback;
mod chain_side_loops;
pub mod convex_cell_bound;
pub mod corner_cap;
pub mod corner_fill;
mod crease_run_out;
pub mod curved_end_face;
pub mod dying_crease_fill;
mod edge_root;
pub mod end_face_flow;
pub mod field_bounds;
pub mod field_counters;
mod field_sample;
pub mod fillet_arc;
pub mod flat_hole_fill;
mod foreign_edge_index;
pub mod foreign_edge_regions;
pub mod foreign_edge_room;
pub mod foreign_room_counters;
pub mod gap_fill;
mod gap_fill_refusal;
mod gap_fill_result;
mod gap_loop;
mod growing_crease_runs;
mod hole_fill_result;
pub mod hull_clip;
mod knife_end_overrun;
mod level_line_mesh;
mod level_line_routing;
pub mod needle_meter;
pub mod open_gap_fill;
mod open_gap_fill_cut_section;
pub mod open_gap_fill_sunk_arc;
mod pair_corner_end_face_field;
mod pick_fault;
mod pick_field;
mod pick_validation;
mod pinch_ladder_search;
mod point_field;
pub mod refinement_splits;
mod room_candidate;
mod room_field;
mod room_reading;
mod room_reading_kind;
mod room_source;
mod run_out_field;
mod scalar_field;
mod seam_edges;
pub mod segment_box_tree;
pub mod sharp_joint_corners;
pub mod side_witness;
pub mod split_meter;
mod square_crease_end_clip;
pub mod stopwatch;
pub mod strip_clip;
pub mod swallowed_side;
pub mod triangle_edges;
mod vertex_side;
pub mod virtual_rail;
mod work_budget;
mod zip_rail;

pub use band_cut::{BandCutResult, LevelLineChain};
pub use band_gap_refusal::BandGapRefusal;
pub use band_gap_result::BandGapResult;
pub use band_group::BandGroup;
pub use band_group_fields::{BandGroupFieldsRunOutsScope, BandGroupFieldsWithRunOuts};
pub use band_profile::BandProfile;
pub use bounded_maximum::{BoundedMaximumCanonicalMaximum, BoundedMaximumCellBound};
pub use chain_edge_not_on_two_triangles::ChainEdgeNotOnTwoTriangles;
pub use chain_field::{ChainField, ChainFieldEndPlaneReader, ChainFieldSmoothJointsScope};
pub use chain_field_end_clip::ChainFieldEndClip;
pub use chain_field_end_ray::EndRayBound;
pub use chain_field_mitre::ChainFieldExtension;
pub use chain_field_queries::{ChainFieldMeshPiece, ClosestOnPiece};
pub use chain_side_loops::ChainSideLoops;
pub use corner_cap::CornerCapLastRow;
pub use corner_fill::CornerFillPiece;
pub use crease_run_out::CreaseRunOut;
pub use edge_root::EdgeRoot;
pub use field_counters::{FieldCounterTally, FieldCounters, LevelLineRule};
pub use field_sample::FieldSample;
pub use foreign_edge_index::{ForeignApproach, ForeignEdgeIndex, ForeignEdgeIndexNode};
pub use foreign_edge_regions::{ForeignEdgeRegions, ForeignEdgeRegionsChainGrowth};
pub use gap_fill::GapFillRailSenses;
pub use gap_fill_refusal::GapFillRefusal;
pub use gap_fill_result::GapFillResult;
pub use gap_loop::GapLoop;
pub use growing_crease_runs::{CreaseBounds, DistanceToChain, GrowingCreaseRuns, IsOwnSide};
pub use hole_fill_result::{HoleFillFailure, HoleFillResult, HoleFillTriangle};
pub use hull_clip::HullClipPlane;
pub use knife_end_overrun::KnifeEndOverrunEnd;
pub use level_line_mesh::{IsChainEdge, LevelLineMesh, LevelLineTriangle, SamePlaceNotALength};
pub use level_line_routing::LevelLineChainWatch;
pub use needle_meter::{NeedleMeter, NeedleMeterMadeBy};
pub use open_gap_fill::OpenGapFillOpenRail;
pub use open_gap_fill_cut_section::OpenGapFillFlatRunOutRegion;
pub use pair_corner_end_face_field::{PairCornerEndFaceField, PairCornerEndFaceFieldScope};
pub use pick_fault::PickFault;
pub use pick_field::PickField;
pub use pick_validation::PickCorner;
pub use pinch_ladder_search::PinchLadderSearchPinchLadder;
pub use point_field::PointField;
pub use refinement_splits::PointPlace;
pub use room_candidate::RoomCandidate;
pub use room_field::RoomField;
pub use room_reading::RoomReading;
pub use room_reading_kind::RoomReadingKind;
pub use room_source::RoomSource;
pub use run_out_field::RunOutField;
pub use scalar_field::{ChainDistanceField, FieldMaximum, FieldMinimum, ScalarField};
pub use seam_edges::SeamEdges;
pub use segment_box_tree::SegmentNode;
pub use split_meter::{SplitMeter, SplitMeterSplitKind, SplitMeterSplitRecord, SplitMeterState};
pub use square_crease_end_clip::SquareCreaseEndClipSquareCreaseClipsOffScope;
pub use vertex_side::{VertexFieldReading, VertexSide};
pub use work_budget::{
    LevelLineProgress, LevelLineProgressReport, LevelLineWork, LevelLineWorkPhase,
    LevelLineWorkScope, WorkBudget, WorkBudgetExceeded, WorkCanceled, LEVEL_LINE_WORK_PHASE_COUNT,
};
pub use zip_rail::ZipRail;
