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

pub mod band_cut;
pub mod band_gap;
pub mod band_gap_after_the_rounds;
pub mod band_gap_build_fault;
pub mod band_gap_build_within;
pub mod band_gap_entry_points;
pub mod band_gap_extra_fields;
pub mod band_gap_in_turn;
pub mod band_gap_reading;
mod band_gap_refusal;
mod band_gap_result;
pub mod band_gap_rounds_first;
mod band_group;
mod band_group_fields;
mod band_profile;
pub mod band_refinement;
pub mod band_refinement_asking;
pub mod band_refinement_mixed_edge;
pub mod band_room;
pub mod band_room_pair_room;
pub mod band_room_pair_room_pick;
pub mod band_room_read_pick;
pub mod band_side_groups;
pub mod band_side_tagging;
pub mod band_side_tagging_ends;
pub mod band_triangles;
pub mod bounded_maximum;
pub mod chain_approach;
mod chain_edge_not_on_two_triangles;
mod chain_field;
mod chain_field_clip;
mod chain_field_end_clip;
mod chain_field_end_ray;
pub mod chain_field_from_polyline;
mod chain_field_least;
mod chain_field_mitre;
mod chain_field_nearest;
mod chain_field_queries;
mod chain_field_rays;
mod chain_field_smooth_joint_spans;
mod chain_field_swallow;
pub mod chain_on_the_mesh;
pub mod chain_setback;
mod chain_side_loops;
mod chains_that_cannot_fit;
pub mod consumed_to_the_rim;
pub mod convex_cell_bound;
pub mod corner_cap;
pub mod corner_cap_flat_ends;
pub mod corner_cap_hole_fill;
pub mod corner_cap_hole_fill_end;
pub mod corner_cap_hole_fill_switch;
pub mod corner_cap_zip_end;
pub mod corner_fill;
pub mod corner_fill_end_ray_trim;
pub mod corner_fill_fill;
pub mod corner_fill_free_ends;
pub mod corner_fill_pieces;
pub mod corner_mitre_patch;
pub mod corner_rail_strip;
pub mod corner_rail_strip_zip;
pub mod corner_tie_points;
mod crease_run_out;
pub mod curved_end_face;
pub mod dying_crease_fill;
pub mod edge_bump_rule;
pub mod edge_root;
pub mod end_clip_corners;
pub mod end_face_crease_plane;
pub mod end_face_fill;
pub mod end_face_fill_try;
pub mod end_face_flow;
pub mod end_face_sense;
mod end_face_sense_oblique;
pub mod end_plane_trim;
pub mod end_plane_trim_carry;
pub mod end_plane_trim_run;
pub mod field_bounds;
pub mod field_counters;
mod field_sample;
pub mod fill_closure;
pub mod fillet_arc;
pub mod fillet_arc_rows;
pub mod flat_hole_fill;
mod foreign_edge_index;
pub mod foreign_edge_regions;
pub mod foreign_edge_regions_filing;
pub mod foreign_edge_room;
pub mod foreign_edge_room_readings;
pub mod foreign_room_counters;
pub mod gap_fill;
pub mod gap_fill_one_chain;
mod gap_fill_refusal;
mod gap_fill_result;
mod gap_loop;
pub mod gap_loop_walk;
mod growing_crease_runs;
pub mod growing_crease_runs_reads;
pub mod hole_fill;
pub mod hole_fill_fairing;
mod hole_fill_result;
pub mod hull_clip;
pub mod knife_end_overrun;
mod level_line_mesh;
mod level_line_mesh_edits;
pub mod level_line_mesh_input_strips;
mod level_line_mesh_insert_polyline;
pub mod level_line_mesh_needles;
pub mod level_line_routing;
pub mod line_walk;
pub mod narrow_face_flow;
pub mod needle_apex_retile;
pub mod needle_meter;
pub mod open_gap_fill;
pub mod open_gap_fill_cut_section;
pub mod open_gap_fill_fill;
pub mod open_gap_fill_stations;
pub mod open_gap_fill_sunk_arc;
pub mod pair_corner_end_face_field;
mod pair_corner_end_face_field_queries;
pub mod pick_end_floods;
mod pick_fault;
mod pick_field;
pub mod pick_validation;
pub mod pinch_ladder_search;
mod point_field;
pub mod rail_stations;
pub mod refinement_splits;
pub mod refinement_splits_enqueue;
mod room_candidate;
mod room_field;
mod room_reading;
mod room_reading_kind;
mod room_source;
pub mod run_out_field;
mod scalar_field;
pub mod seam_edges;
pub mod segment_box_tree;
pub mod self_room;
pub mod sharp_joint_corners;
pub mod side_flow;
pub mod side_roots_settling;
pub mod side_witness;
pub mod split_meter;
mod square_crease_end_clip;
pub mod step_corner_carry;
pub mod stopwatch;
pub mod strip_clip;
pub mod strip_clip_neighbour;
pub mod strip_clip_valley;
pub mod swallowed_side;
pub mod swallowed_side_runs;
pub mod triangle_edges;
pub mod uneven_strip;
mod vertex_side;
pub mod virtual_rail;
pub mod virtual_rail_past_the_end;
pub mod work_budget;
mod zip_rail;

pub use band_cut::{BandCutResult, LevelLineChain};
pub use band_gap_refusal::BandGapRefusal;
pub use band_gap_result::BandGapResult;
pub use band_group::BandGroup;
pub use band_group_fields::unwrapped as band_group_fields_unwrapped;
pub use band_group_fields::{of as band_group_fields_of, GroupSampler};
pub use band_group_fields::{run_outs as band_group_run_outs, BandGroupExtraFields};
pub use band_group_fields::{run_outs_on, turn_run_outs_on};
pub use band_group_fields::{BandGroupFieldsRunOutsScope, BandGroupFieldsWithRunOuts};
pub use band_profile::BandProfile;
pub use bounded_maximum::{BoundedMaximumCanonicalMaximum, BoundedMaximumCellBound};
pub use chain_edge_not_on_two_triangles::ChainEdgeNotOnTwoTriangles;
pub use chain_field::{ChainField, ChainFieldEndPlaneReader, ChainFieldSmoothJointsScope};
pub use chain_field_end_clip::ChainFieldEndClip;
pub use chain_field_end_ray::EndRayBound;
pub use chain_field_mitre::ChainFieldExtension;
pub use chain_field_nearest::ChainFieldNearest;
pub use chain_field_queries::{ChainFieldMeshPiece, ClosestOnPiece};
pub use chain_field_rays::ChainFieldRays;
pub use chain_side_loops::ChainSideLoops;
pub use chains_that_cannot_fit::{ChainCannotFit, ChainsThatCannotFit};
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
pub use level_line_mesh_edits::LevelLineMeshError;
pub use level_line_routing::LevelLineChainWatch;
pub use needle_meter::{NeedleMeter, NeedleMeterMadeBy};
pub use open_gap_fill::OpenGapFillOpenRail;
pub use open_gap_fill_cut_section::OpenGapFillFlatRunOutRegion;
pub use pair_corner_end_face_field::{PairCornerEndFaceField, PairCornerEndFaceFieldScope};
pub use pick_fault::PickFault;
pub use pick_field::{PickField, PickFieldArgumentError};
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
pub use scalar_field::FieldKind;
pub use scalar_field::{ChainDistanceField, FieldMaximum, FieldMinimum, ScalarField};
pub use seam_edges::SeamEdges;
pub use segment_box_tree::SegmentNode;
pub use split_meter::{SplitMeter, SplitMeterSplitKind, SplitMeterSplitRecord, SplitMeterState};
pub use square_crease_end_clip::SquareCreaseEndClipSquareCreaseClipsOffScope;
pub use square_crease_end_clip::{
    cap_room, square_crease_clips_on, square_crease_end_clips, square_crease_end_clips_of_pick,
    square_crease_end_room, turn_square_crease_clips_off,
};
pub use vertex_side::{VertexFieldReading, VertexSide};
pub use work_budget::{
    LevelLineProgress, LevelLineProgressReport, LevelLineWork, LevelLineWorkPhase,
    LevelLineWorkScope, WatchHoleFills, WorkBudget, WorkError, LEVEL_LINE_WORK_PHASE_COUNT,
};
pub use zip_rail::ZipRail;
