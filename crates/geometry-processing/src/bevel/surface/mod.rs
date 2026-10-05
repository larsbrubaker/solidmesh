//! `MatterCAD.GeometryProcessing/Bevel/Surface`: the surface bevel construction. Ported so far:
//! its leaf records - a band end's half space ([`SurfaceBandEnd`]), a partial run-off
//! ([`SurfaceBandRunsOffOverAStretch`]), which edges own a strip ([`SurfaceEdgeRoles`]) and the
//! plan's infinite line ([`SurfaceLine`]) - and the pieces every stage of it shares: the welding
//! vertex pool ([`SurfaceVertexPool`]), the closure gate the answer is refused by
//! ([`surface_mesh_closure`]), the step laid at a mitre joint ([`surface_mitre_joint_step`]) and
//! the reading of a feature standing on a region ([`surface_room_standing_feature`]), the rolling
//! ball's spine round a mixed corner ([`SurfaceMixedCornerSpine`]) and the per-region face grid
//! the wall-clearance questions are asked of ([`SurfaceRegionFaceGrid`]), and the tidying of
//! the rings a clip leaves ([`surface_ring`]); and the half-space clip every cut is drawn with
//! ([`surface_cut_clip`]), the whole-span and partial readings of a band running off the face it
//! lands on ([`surface_band_runs_off_the_face_beyond`]) and the closed loop the planner left
//! without a plan ([`SurfaceBevelPlanlessLoop`]).
//! And the types of the construction's middle: the walk across a curved region
//! ([`SurfaceRunAcrossSectionStep`], [`SurfaceRunAcrossSectionStart`],
//! [`SurfaceRunAcrossRegionMaps`]) and the wall a rolled edge was planned from
//! ([`SurfaceWallSection`]); one edge's plan ([`SurfaceEdgePlan`]) and a rolled edge's reading
//! ([`SurfaceRolledEdge`]); the mixed corner ([`SurfaceMixedCorner`]), its dual canal
//! ([`SurfaceMixedCornerDualFrame`], [`SurfaceMixedCornerDualCanal`],
//! [`SurfaceMixedCornerDualGrid`], [`SurfaceMixedCornerDualWallCrossing`]), its rows
//! ([`SurfaceMixedCornerRows`]), its walls ([`SurfaceMixedCornerWall`],
//! [`SurfaceMixedCornerFaces`]) and its run-out ([`SurfaceRunOutCarry`],
//! [`SurfaceRunOutMeetsAWallBand`]); and the three-edge junction ([`SurfaceMixedJunction`],
//! [`SurfaceJunctionKind`]) with its two-crease rings ([`SurfaceTwoCreaseJunctionRings`]).
//! And the strips and what cuts them: the ruled strip ([`SurfaceStrip`]) and its stations
//! ([`SurfaceStripStation`]), the half space a face is walked back to ([`SurfaceFacetCut`]), the
//! roll along a curved wall ([`SurfaceRollReading`], [`SurfaceRollAcrossTheWallSides`],
//! [`SurfaceRollAcrossTheWallLanding`]), a chain end's reading ([`SurfaceChainEndReading`]) and
//! the face carrying on past it ([`SurfaceFaceCarriesOnWayOn`]); the room ([`SurfaceRoomCut`],
//! [`SurfaceRoomFootprint`], [`SurfaceRoomCrossing`]); the strip's ends - on the hull
//! ([`SurfaceStripEndsOnTheHull`]), trimmed by the face beyond
//! ([`SurfaceStripTrimmedByTheFaceBeyond`]), gaining the section
//! ([`SurfaceEndGainsTheSectionReading`]) - the features standing in a band
//! ([`SurfaceStandingFeatureWithheld`], [`SurfaceStripNotchedByAFeature`],
//! [`SurfaceEndCapAroundAFeatureMeeting`]); the run-out's columns ([`SurfaceRunOutColumnWindow`],
//! [`SurfaceRunOutColumnWalk`]) and its crossings of a band ([`SurfaceRunOutMeetsTheBand`],
//! [`SurfaceStripTrimmedAtTheCrossing`], [`SurfaceCrossingAtTheMitre`]); the strips arriving at a
//! corner ([`SurfaceCornerPatchArrival`], [`SurfaceMixedJunctionPatchArrival`],
//! [`SurfaceTwoCreaseJunctionPatchEndRing`], [`SurfaceTwoCreaseJunctionWallCrossing`]); and the
//! bite's shared sides ([`SurfaceBiteSharedSides`]) and the splits along a stretch
//! ([`SurfaceSplitsOnASegment`]); and the chain's plan itself ([`SurfaceBevelPlan`]) with what it
//! was drawn from ([`SurfacePlanRecipe`]). And the triangulators every stage emits through: the fan
//! ([`surface_fan`]), the ear clipper for rings no apex sees whole ([`surface_ring_ears`]), the
//! zipper between two rows ([`surface_row_zip`]) and the side reading the sewing shares with them
//! ([`surface_split_sewing`]); the end cut's run readings ([`surface_end_cut`]), the rolled
//! section's concave cap ([`surface_end_follows_the_walked_chords`]), the rim sewing of a section
//! given up ([`surface_end_gives_up_the_section`]) and the convex bite's side reading
//! ([`surface_convex_bite`]). The types land as type definitions; their methods, and the
//! construction itself, land in later slices.

mod surface_band_end;
mod surface_band_runs_off_over_a_stretch;
pub mod surface_band_runs_off_the_face_beyond;
mod surface_bevel_plan;
mod surface_bevel_planless_loop;
mod surface_bite_shared_sides;
mod surface_chain_end_reading;
pub mod surface_convex_bite;
mod surface_corner_patch;
mod surface_crossing_at_the_mitre;
pub mod surface_cut_clip;
mod surface_edge_plan;
mod surface_edge_roles;
mod surface_end_cap_around_a_feature;
pub mod surface_end_cut;
pub mod surface_end_follows_the_walked_chords;
mod surface_end_gains_the_section;
pub mod surface_end_gives_up_the_section;
mod surface_face_carries_on;
mod surface_facet_cut;
pub mod surface_fan;
mod surface_line;
pub mod surface_mesh_closure;
pub mod surface_mitre_joint_step;
mod surface_mixed_corner;
pub mod surface_mixed_corner_cut_edge;
mod surface_mixed_corner_dual_canal;
mod surface_mixed_corner_dual_frame;
mod surface_mixed_corner_dual_grid;
pub mod surface_mixed_corner_dual_wall;
mod surface_mixed_corner_faces;
pub mod surface_mixed_corner_hands_over_at_the_mitre;
pub mod surface_mixed_corner_patch;
pub mod surface_mixed_corner_read_off_the_edge_plans;
pub mod surface_mixed_corner_ring_schedule;
mod surface_mixed_corner_rows;
pub mod surface_mixed_corner_run_out_strip;
mod surface_mixed_corner_spine;
mod surface_mixed_corner_wall;
mod surface_mixed_junction;
mod surface_mixed_junction_patch;
pub mod surface_plan_band_ends;
mod surface_plan_recipe;
mod surface_region_face_grid;
pub mod surface_ring;
pub mod surface_ring_ears;
mod surface_roll_across_the_wall;
mod surface_rolled_edge;
mod surface_room;
mod surface_room_crossing;
pub mod surface_room_measure;
pub mod surface_room_standing_feature;
pub mod surface_row_zip;
mod surface_run_across;
mod surface_run_out_carry;
mod surface_run_out_column_walk;
mod surface_run_out_column_window;
mod surface_run_out_meets_a_wall_band;
mod surface_run_out_meets_the_band;
pub mod surface_split_sewing;
mod surface_splits_on_a_segment;
mod surface_standing_feature_withheld;
mod surface_strip;
mod surface_strip_ends_on_the_hull;
mod surface_strip_notched_by_a_feature;
mod surface_strip_station;
mod surface_strip_trimmed_at_the_crossing;
mod surface_strip_trimmed_by_the_face_beyond;
pub mod surface_two_crease_junction;
mod surface_two_crease_junction_patch;
mod surface_two_crease_junction_rings;
pub mod surface_two_crease_junction_wall;
mod surface_vertex_pool;
mod surface_wall_section;

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
pub use surface_standing_feature_withheld::{
    SurfaceStandingFeatureWithheld, SurfaceStandingFeatureWithheldFace,
};
pub use surface_strip::SurfaceStrip;
pub use surface_strip_ends_on_the_hull::{
    SurfaceStripEndsOnTheHull, SurfaceStripEndsOnTheHullSupportingPlane,
};
pub use surface_strip_notched_by_a_feature::{
    SurfaceStripNotchedByAFeature, SurfaceStripNotchedByAFeatureBetween,
};
pub use surface_strip_station::SurfaceStripStation;
pub use surface_strip_trimmed_at_the_crossing::SurfaceStripTrimmedAtTheCrossing;
pub use surface_strip_trimmed_by_the_face_beyond::SurfaceStripTrimmedByTheFaceBeyond;
pub use surface_two_crease_junction_patch::SurfaceTwoCreaseJunctionPatchEndRing;
pub use surface_two_crease_junction_rings::SurfaceTwoCreaseJunctionRings;
pub use surface_two_crease_junction_wall::SurfaceTwoCreaseJunctionWallCrossing;
pub use surface_vertex_pool::SurfaceVertexPool;
pub use surface_wall_section::SurfaceWallSection;
