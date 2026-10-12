//! `MatterCAD.GeometryProcessing/Bevel/Surface`: the surface bevel construction.
//!
//! - **Plan.** The chain's plan ([`SurfaceBevelPlan`]) and what it was drawn from
//!   ([`SurfacePlanRecipe`]), one edge's plan ([`SurfaceEdgePlan`]), which edges own a strip
//!   ([`SurfaceEdgeRoles`]), the plan's infinite line ([`SurfaceLine`]), a band end's half space
//!   ([`SurfaceBandEnd`]), the closed loop the planner left without a plan
//!   ([`SurfaceBevelPlanlessLoop`]) and the chain readings before any plan: the join of runs
//!   meeting end to end ([`surface_chain_join`]), the cut at every junction a chain passes through
//!   ([`surface_chain_opens_at_the_junction`]) and the crease a feature interrupts
//!   ([`surface_interrupted_crease`]).
//! - **Shared pieces.** The welding vertex pool ([`SurfaceVertexPool`]), the closure gate the
//!   answer is refused by ([`surface_mesh_closure`]), the half-space clip every cut is drawn with
//!   ([`surface_cut_clip`]), the tidying of the rings a clip leaves ([`surface_ring`]), the
//!   per-region face grid the wall-clearance questions are asked of ([`SurfaceRegionFaceGrid`]),
//!   and the triangulators every stage emits through: the fan ([`surface_fan`]), the ear clipper
//!   for rings no apex sees whole ([`surface_ring_ears`]), the zipper between two rows
//!   ([`surface_row_zip`]) and the side reading the sewing shares with them
//!   ([`surface_split_sewing`]); the stitch of one strip's quads between its two rings
//!   ([`surface_bevel_stitch`]).
//! - **Strips.** The ruled strip ([`SurfaceStrip`]) and its stations ([`SurfaceStripStation`]),
//!   the half space a face is walked back to ([`SurfaceFacetCut`]), the walk across a curved
//!   region ([`SurfaceRunAcrossSectionStep`], [`SurfaceRunAcrossSectionStart`],
//!   [`SurfaceRunAcrossRegionMaps`]), the wall a rolled edge was planned from
//!   ([`SurfaceWallSection`]) and the rolled edge's reading ([`SurfaceRolledEdge`]), the roll
//!   along a curved wall ([`SurfaceRollReading`], [`SurfaceRollAcrossTheWallSides`],
//!   [`SurfaceRollAcrossTheWallLanding`]), the splits along a stretch
//!   ([`SurfaceSplitsOnASegment`]) and a band running off the face it lands on, over a stretch or
//!   the whole span ([`SurfaceBandRunsOffOverAStretch`],
//!   [`surface_band_runs_off_the_face_beyond`]), and the band's outline borrowed across a chorded
//!   wall's creases ([`surface_band_outline`]); the run of a walked-back tangent line each strip
//!   sews to ([`surface_seam_over_the_edge`]).
//! - **Strip ends.** A chain end's reading ([`SurfaceChainEndReading`]) and the face carrying on
//!   past it ([`SurfaceFaceCarriesOnWayOn`]); the strip ending on the hull
//!   ([`SurfaceStripEndsOnTheHull`]), trimmed by the face beyond
//!   ([`SurfaceStripTrimmedByTheFaceBeyond`]) or gaining the section
//!   ([`SurfaceEndGainsTheSectionReading`]); the end cut's run readings ([`surface_end_cut`]), the
//!   rolled section's concave cap ([`surface_end_follows_the_walked_chords`]), the rim sewing of a
//!   section given up ([`surface_end_gives_up_the_section`]) and the convex bite's side reading
//!   ([`surface_convex_bite`]).
//! - **Room.** The room ([`SurfaceRoomCut`], [`SurfaceRoomFootprint`], [`SurfaceRoomCrossing`])
//!   and the reading of a feature standing on a region ([`surface_room_standing_feature`]).
//! - **Standing features.** The features standing in a band ([`SurfaceStandingFeatureWithheld`],
//!   [`SurfaceStripNotchedByAFeature`], [`SurfaceEndCapAroundAFeatureMeeting`]), a feature's bite
//!   out of a strip ([`SurfaceStripBittenByAFeature`]) with the rows laid across it
//!   ([`SurfaceStripBiteRows`]) and its faces extended out to the strip
//!   ([`surface_standing_feature_faces`]), the stretches of a feature a band passes under
//!   ([`surface_standing_feature_stretch`]) and the builder's step that reads every standing
//!   feature into bites and notches ([`surface_builder_standing_features`]).
//! - **Run-outs and crossings.** The run-out's columns ([`SurfaceRunOutColumnWindow`],
//!   [`SurfaceRunOutColumnWalk`]) and its crossings of a band ([`SurfaceRunOutMeetsTheBand`],
//!   [`SurfaceStripTrimmedAtTheCrossing`], [`SurfaceCrossingAtTheMitre`]), and the builder's
//!   crossings of a band by a run-out ([`surface_bevel_crossings`]).
//! - **Mitres.** The step laid at a mitre joint ([`surface_mitre_joint_step`]) and the sector a
//!   mitre of unequal setbacks gives up ([`surface_mitre_sector`]), the mitre whose tip stands
//!   past the shared face's rim ([`SurfaceMitreOverTheFaceBeyond`]), the short chord whose two
//!   mitres crossed over on some of its rails ([`SurfaceMitreConsumesTheRail`]), the patch
//!   a reflex mitre's narrower arc covers past its far rim ([`SurfaceMitreSliver`]), the mitre of two
//!   arcs chorded at different steps ([`surface_mitre_chorded_apart`]) and the ring two strips share
//!   at a mitre ([`surface_strip_mitre`], its rails [`surface_strip_mitre_rails`]).
//! - **Corners and junctions.** The mixed corner ([`SurfaceMixedCorner`]), the rolling ball's
//!   spine round it ([`SurfaceMixedCornerSpine`]), its dual canal ([`SurfaceMixedCornerDualFrame`],
//!   [`SurfaceMixedCornerDualCanal`], [`SurfaceMixedCornerDualGrid`],
//!   [`SurfaceMixedCornerDualWallCrossing`]) and the cap a leaning corner's run-out gives up
//!   ([`surface_mixed_corner_dual_cap`]) and its dual-canal patch
//!   ([`surface_mixed_corner_dual_patch`]), its rows ([`SurfaceMixedCornerRows`]), walls
//!   ([`SurfaceMixedCornerWall`], [`SurfaceMixedCornerFaces`]) and run-out
//!   ([`SurfaceRunOutCarry`], [`SurfaceRunOutMeetsAWallBand`]); the three-edge junction
//!   ([`SurfaceMixedJunction`], [`SurfaceJunctionKind`]) with its two-crease rings
//!   ([`SurfaceTwoCreaseJunctionRings`]); the strips arriving at a corner
//!   ([`SurfaceCornerPatchArrival`], [`SurfaceMixedJunctionPatchArrival`],
//!   [`SurfaceTwoCreaseJunctionPatchEndRing`], [`SurfaceTwoCreaseJunctionWallCrossing`]) and the
//!   bite's shared sides ([`SurfaceBiteSharedSides`]).
//! - **The pass around the build.** The groups sharing an edge linked as one selection
//!   ([`surface_shared_edge_network`]), a region and the cuts asked of it
//!   ([`SurfaceRegionWalkback`]) with the walkback's result ([`SurfaceWalkbackRuns`]), its band
//!   reach and standing-feature guard ([`surface_walkback_bands`]) and the tangent runs each cut
//!   leaves ([`surface_walkback_tangent_runs`]), the two-pass rewrite of the mesh the walkback
//!   cuts ([`surface_walkback_rewrite`]), the walkback itself, which drives them all
//!   ([`surface_face_walkback`]), the room
//!   and the build agreeing on their run-offs ([`surface_run_off_agreement`]), the seams re-read
//!   after the later cuts ([`surface_seam_re_read`]) and the warning where a run stops short
//!   ([`surface_run_stops_short`]).

mod surface_band_end;
pub mod surface_band_outline;
mod surface_band_runs_off_over_a_stretch;
pub mod surface_band_runs_off_the_face_beyond;
pub mod surface_bevel_crossings;
mod surface_bevel_plan;
mod surface_bevel_planless_loop;
pub mod surface_bevel_stitch;
mod surface_bite_shared_sides;
pub mod surface_bitten_feature_faces;
pub mod surface_builder_mixed_corners;
pub mod surface_builder_standing_features;
mod surface_chain_end_reading;
pub mod surface_chain_join;
pub mod surface_chain_opens_at_the_junction;
pub mod surface_convex_bite;
pub mod surface_corner_mitre;
pub mod surface_corner_patch;
pub mod surface_corner_patch_rows;
mod surface_crossing_at_the_mitre;
pub mod surface_cut_clip;
mod surface_edge_plan;
mod surface_edge_roles;
pub mod surface_end_across_the_corner;
pub mod surface_end_cap_around_a_feature;
pub mod surface_end_cut;
pub mod surface_end_cut_above_the_crossing;
pub mod surface_end_follows_the_walked_chords;
pub mod surface_end_gains_the_section;
pub mod surface_end_gives_up_the_section;
pub mod surface_end_notch;
pub mod surface_face_carries_on;
pub mod surface_face_walkback;
mod surface_facet_cut;
pub mod surface_fan;
pub mod surface_handover_leaves_a_strip;
pub mod surface_interrupted_crease;
pub mod surface_junction_band_ends;
mod surface_line;
pub mod surface_mesh_closure;
pub mod surface_mitre_chorded_apart;
mod surface_mitre_consumes_the_rail;
pub mod surface_mitre_joint_step;
pub mod surface_mitre_on_a_chorded_run;
mod surface_mitre_over_the_face_beyond;
pub mod surface_mitre_seam;
pub mod surface_mitre_sector;
mod surface_mitre_sliver;
mod surface_mitre_sliver_sew;
mod surface_mixed_corner;
pub mod surface_mixed_corner_bites;
pub mod surface_mixed_corner_cut_edge;
mod surface_mixed_corner_dual_canal;
pub mod surface_mixed_corner_dual_cap;
mod surface_mixed_corner_dual_frame;
mod surface_mixed_corner_dual_grid;
pub mod surface_mixed_corner_dual_patch;
pub mod surface_mixed_corner_dual_wall;
pub mod surface_mixed_corner_faces;
pub mod surface_mixed_corner_hands_over_at_the_mitre;
pub mod surface_mixed_corner_patch;
pub mod surface_mixed_corner_read_off_the_edge_plans;
pub mod surface_mixed_corner_readings;
pub mod surface_mixed_corner_ring_schedule;
mod surface_mixed_corner_rows;
pub mod surface_mixed_corner_run_out;
pub mod surface_mixed_corner_run_out_strip;
pub mod surface_mixed_corner_sector;
mod surface_mixed_corner_spine;
mod surface_mixed_corner_wall;
pub mod surface_mixed_corner_wall_ends;
pub mod surface_mixed_corner_which_canal;
mod surface_mixed_junction;
pub mod surface_mixed_junction_patch;
pub mod surface_plan_band_ends;
pub mod surface_plan_edge_reading;
pub mod surface_plan_recipe;
mod surface_region_face_grid;
mod surface_region_walkback;
pub mod surface_ring;
pub mod surface_ring_ears;
pub mod surface_roll_across_the_wall;
mod surface_rolled_edge;
pub mod surface_room;
mod surface_room_crossing;
pub mod surface_room_end_cut;
pub mod surface_room_fixed_point;
pub mod surface_room_measure;
pub mod surface_room_mitre;
pub mod surface_room_notch_sides;
pub mod surface_room_run_out;
pub mod surface_room_standing_feature;
pub mod surface_row_zip;
pub mod surface_run_across;
pub mod surface_run_across_walk;
pub mod surface_run_off_agreement;
pub mod surface_run_out_carry;
pub mod surface_run_out_column_ends;
mod surface_run_out_column_walk;
mod surface_run_out_column_window;
mod surface_run_out_meets_a_wall_band;
mod surface_run_out_meets_the_band;
pub mod surface_run_stops_short;
pub mod surface_seam_over_the_edge;
pub mod surface_seam_re_read;
pub mod surface_shared_edge_network;
pub mod surface_split_sewing;
mod surface_splits_on_a_segment;
pub mod surface_standing_feature_faces;
pub mod surface_standing_feature_stretch;
mod surface_standing_feature_withheld;
mod surface_strip;
mod surface_strip_bite_rows;
mod surface_strip_bitten_by_a_feature;
pub mod surface_strip_ends;
pub mod surface_strip_ends_at_a_landing_notch;
mod surface_strip_ends_on_the_hull;
pub mod surface_strip_ends_over_the_face_beyond;
pub mod surface_strip_mitre;
pub mod surface_strip_mitre_rails;
mod surface_strip_notched_by_a_feature;
mod surface_strip_station;
pub mod surface_strip_tail_around_a_feature;
mod surface_strip_trimmed_at_the_crossing;
mod surface_strip_trimmed_by_the_face_beyond;
pub mod surface_two_corners_at_one_vertex;
pub mod surface_two_crease_junction;
pub mod surface_two_crease_junction_patch;
mod surface_two_crease_junction_rings;
pub mod surface_two_crease_junction_wall;
mod surface_vertex_pool;
pub mod surface_walkback_bands;
mod surface_walkback_result;
pub mod surface_walkback_rewrite;
pub mod surface_walkback_standing_features;
pub mod surface_walkback_tangent_runs;
pub mod surface_wall_ran_out;
mod surface_wall_section;
pub mod surface_wall_stands_clear;

pub use surface_band_end::SurfaceBandEnd;
pub use surface_band_runs_off_over_a_stretch::SurfaceBandRunsOffOverAStretch;
pub use surface_bevel_plan::SurfaceBevelPlan;
pub use surface_bevel_planless_loop::SurfaceBevelPlanlessLoop;
pub use surface_bite_shared_sides::SurfaceBiteSharedSides;
pub use surface_chain_end_reading::{SurfaceChainEndReading, SurfaceChainEndReadingMemos};
pub use surface_corner_patch::SurfaceCornerPatchArrival;
pub use surface_crossing_at_the_mitre::SurfaceCrossingAtTheMitre;
pub use surface_edge_plan::SurfaceEdgePlan;
pub use surface_edge_roles::SurfaceEdgeRoles;
pub use surface_end_cap_around_a_feature::SurfaceEndCapAroundAFeatureMeeting;
pub use surface_end_gains_the_section::SurfaceEndGainsTheSectionReading;
pub use surface_face_carries_on::SurfaceFaceCarriesOnWayOn;
pub use surface_facet_cut::SurfaceFacetCut;
pub use surface_line::SurfaceLine;
pub use surface_mitre_consumes_the_rail::SurfaceMitreConsumesTheRail;
pub use surface_mitre_over_the_face_beyond::{SurfaceMitreHull, SurfaceMitreOverTheFaceBeyond};
pub use surface_mitre_sliver::{SurfaceMitreSliver, SurfaceMitreSliverRun};
pub use surface_mixed_corner::SurfaceMixedCorner;
pub use surface_mixed_corner_dual_canal::SurfaceMixedCornerDualCanal;
pub use surface_mixed_corner_dual_frame::SurfaceMixedCornerDualFrame;
pub use surface_mixed_corner_dual_grid::SurfaceMixedCornerDualGrid;
pub use surface_mixed_corner_dual_wall::SurfaceMixedCornerDualWallCrossing;
pub use surface_mixed_corner_faces::SurfaceMixedCornerFaces;
pub use surface_mixed_corner_rows::{SurfaceMixedCornerRowRunsTo, SurfaceMixedCornerRows};
pub use surface_mixed_corner_spine::{ContactArms, SurfaceMixedCornerSpine};
pub use surface_mixed_corner_wall::{SurfaceMixedCornerWall, SurfaceMixedCornerWallFacet};
pub use surface_mixed_junction::{
    SurfaceJunctionKind, SurfaceMixedJunction, SurfaceMixedJunctionArm,
    SurfaceMixedJunctionChainEnd,
};
pub use surface_mixed_junction_patch::SurfaceMixedJunctionPatchArrival;
pub use surface_plan_recipe::SurfacePlanRecipe;
pub use surface_region_face_grid::{nearest_on_the_triangle, SurfaceRegionFaceGrid};
pub use surface_region_walkback::SurfaceRegionWalkback;
pub use surface_roll_across_the_wall::{
    SurfaceRollAcrossTheWallLanding, SurfaceRollAcrossTheWallSides, SurfaceRollReading,
};
pub use surface_rolled_edge::SurfaceRolledEdge;
pub use surface_room::{SurfaceRoomCut, SurfaceRoomFootprint};
pub use surface_room_crossing::SurfaceRoomCrossing;
pub use surface_run_across::{
    SurfaceRunAcrossRegionMaps, SurfaceRunAcrossSectionStart, SurfaceRunAcrossSectionStep,
};
pub use surface_run_out_carry::SurfaceRunOutCarry;
pub use surface_run_out_column_walk::SurfaceRunOutColumnWalk;
pub use surface_run_out_column_window::SurfaceRunOutColumnWindow;
pub use surface_run_out_meets_a_wall_band::SurfaceRunOutMeetsAWallBand;
pub use surface_run_out_meets_the_band::SurfaceRunOutMeetsTheBand;
pub use surface_splits_on_a_segment::SurfaceSplitsOnASegment;
pub use surface_standing_feature_stretch::SurfaceStandingFeatureStretch;
pub use surface_standing_feature_withheld::{
    SurfaceStandingFeatureWithheld, SurfaceStandingFeatureWithheldFace,
};
pub use surface_strip::SurfaceStrip;
pub use surface_strip_bite_rows::SurfaceStripBiteRows;
pub use surface_strip_bitten_by_a_feature::SurfaceStripBittenByAFeature;
pub use surface_strip_ends_on_the_hull::{
    SurfaceStripEndsOnTheHull, SurfaceStripEndsOnTheHullSupportingPlane,
};
pub use surface_strip_notched_by_a_feature::{
    SurfaceStripNotchedByAFeature, SurfaceStripNotchedByAFeatureBetween, SurfaceStripNotches,
};
pub use surface_strip_station::SurfaceStripStation;
pub use surface_strip_trimmed_at_the_crossing::{
    SurfaceStripTrimmedAtTheCrossing, SurfaceStripTrims,
};
pub use surface_strip_trimmed_by_the_face_beyond::SurfaceStripTrimmedByTheFaceBeyond;
pub use surface_two_crease_junction_patch::SurfaceTwoCreaseJunctionPatchEndRing;
pub use surface_two_crease_junction_rings::SurfaceTwoCreaseJunctionRings;
pub use surface_two_crease_junction_wall::SurfaceTwoCreaseJunctionWallCrossing;
pub use surface_vertex_pool::SurfaceVertexPool;
pub use surface_walkback_result::{SurfaceWalkbackResult, SurfaceWalkbackRuns};
pub use surface_wall_section::SurfaceWallSection;
