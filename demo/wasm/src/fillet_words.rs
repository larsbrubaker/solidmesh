//! The demo's words for what the bevel kernel says. `bevel_part` answers codes and numbers only
//! (`BevelAdvisory`: which thing happened and the values it quotes); a caller puts them into its
//! own language, and this is the demo's English: one sentence per `BevelAdvisoryCode`, the same
//! sentences MatterCAD shows its users, so the page reads as the application does. The switch is
//! split three ways for the 800-line limit: the kernel and surface sentences here, the blend,
//! operation and resolution sentences in `fillet_words_blend`, and the shared pieces (positions,
//! numbers, the size refusals and the template filler) in `fillet_words_parts`.

use geometry_processing::bevel::{BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode};

use crate::fillet_words_blend::format_blend_or_operation;
use crate::fillet_words_parts::{
    at, describe_at, face_angle_sentence, format, held_back_sentence, int_at, localize,
    size_sentence, stops_square_sentence, text,
};

/// The sentence for one advisory.
pub fn format_advisory(advisory: &BevelAdvisory) -> String {
    let args = advisory.args();
    if let Some(sentence) = size_sentence(advisory.code(), args) {
        return sentence;
    }

    match advisory.code() {
            BevelAdvisoryCode::CapClipCouldNotCut => format(
                &localize("The cap where these edges end at {0} could not be cut, so a size of {1:0.###} cannot be stopped there."),
                &[text(describe_at(args, 0)), at(args, 1)],
            ),
            BevelAdvisoryCode::HeldBackAtOneEnd => held_back_sentence(
                format(&localize("these edges end at {0}"), &[text(describe_at(args, 0))]),
                at(args, 1),
                int_at(args, 2),
            ),
            BevelAdvisoryCode::HeldBackAtBothEnds => held_back_sentence(
                format(
                    &localize("these edges end at {0} and at {1}"),
                    &[text(describe_at(args, 0)), text(describe_at(args, 1))],
                ),
                at(args, 2),
                int_at(args, 3),
            ),
            BevelAdvisoryCode::SurfaceRunStopsSquareAtOneEnd => stops_square_sentence(
                format(&localize("this run ends at {0}"), &[text(describe_at(args, 0))]),
                int_at(args, 1),
            ),
            BevelAdvisoryCode::SurfaceRunStopsSquareAtBothEnds => stops_square_sentence(
                format(
                    &localize("this run ends at {0} and at {1}"),
                    &[text(describe_at(args, 0)), text(describe_at(args, 1))],
                ),
                int_at(args, 2),
            ),
            BevelAdvisoryCode::BeadSquaredAtCorner => format(
                &localize("The bevel stops square at the corner at {0} rather than on it: the faces bounding that corner either lie too nearly along the edges to trim the tool against or would slide the bevel's end further than its last segment can absorb, so the material it adds ends in a flat face there."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::BeadDoesNotSpanInterruption => format(
                &localize("The bevel runs into the feature standing on the edge at {0} but does not come out the far side of it: that feature is {2:0.###} across, and a size of {1:0.###} reaches less than half of it from each side, so the edge inside it is left unbeveled between them. A larger size closes it."),
                &[text(describe_at(args, 0)), at(args, 1), at(args, 2)],
            ),
            BevelAdvisoryCode::CutRunsPastCorner => format(
                &localize("The bevel runs past the corner at {0} instead of stopping on it: every face bounding that corner lies too nearly along the edges to trim the tool against."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceStripSeamDidNotMeet => format(
                &localize("The bevel does not meet the part cleanly at {0}. Try a smaller size, or bevel this edge in a group of its own."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceStripsDoNotMitre => format(
                &localize("The bevels along the two edges meeting at {0} do not join up at that corner. Try a smaller size, or bevel those two edges in separate groups."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceStripEndNotSupported => format(
                &localize("The bevel ends at {0}, where there is no single flat face past that corner for it to stop against. Try a smaller size, or pick more of the edge so the bevel carries on round that corner."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceCreaseFacesAreOneSurface => format(
                &localize("The edge at {0} stops where a rounded blend takes the corner over, and every turn around that blend is gentler than the Bend Threshold of {1} degrees, so the blend and the faces beside it are read as one surface. Lower the Bend Threshold below the blend's own turn to bevel this edge apart from the blend."),
                &[text(describe_at(args, 0)), at(args, 1)],
            ),
            BevelAdvisoryCode::BooleanCutterUnionBlewUp => format(
                &localize("Joining the cutting tools ran away: after {2} of {3} tools the joined solid carried {0} faces where the tools themselves hold only {1}, so the join was stopped rather than left to run for minutes. Set this feature's Construction to Surface, which builds this shape without joining any tools."),
                &[at(args, 0), at(args, 1), at(args, 2), at(args, 3)],
            ),
            BevelAdvisoryCode::SurfaceSmoothSideNotCut => format(
                &localize("The surface beside the edge at {0} is one this bevel cannot cut to lay the fillet into yet. Try a smaller size, or bevel this edge in a group of its own."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceCornerNotSupported => format(
                &localize("The corner at {0} is a shape this bevel cannot turn yet. Bevel the edges meeting there in separate groups, or leave one of them out of the selection."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceObliqueMixedCornerRowsNotBuilt => format(
                &localize("The corner at {0} leans {1:0.##} degrees off square, which this bevel cannot turn yet. Bevel the edges meeting there in separate groups, or leave one of them out of the selection."),
                &[text(describe_at(args, 0)), at(args, 1)],
            ),
            BevelAdvisoryCode::SurfaceMixedCornerBeadRingCrossesNoRail => format(
                &localize("The bevel cannot be brought to a finish at the corner at {0}. Try a smaller size, or bevel the edges meeting there in separate groups."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceMixedCornerCutLiesInTheWall => format(
                &localize("At the corner at {0} an outside edge lies flat in the face an inside edge stands against, and this bevel cannot round both at once. Bevel the edges meeting there in separate groups, or leave one of them out of the selection."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceJunctionOfTwoCreases => format(
                &localize("Edges meet at {0} and two of them are inside edges, which this bevel does not turn yet. Bevel the edges meeting there in separate groups, or leave one of them out of the selection."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceJunctionOfThreeCreases => format(
                &localize("Edges meet at {0} and three of them are inside edges, which this bevel cannot round together yet. Bevel the edges meeting there in separate groups, or leave one of them out of the selection."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceJunctionOfTwoCreasesOnTwoWalls => format(
                &localize("The two inside edges meeting at {0} stand against two different faces rather than one, so this bevel cannot carry a single round through that corner. Bevel them in separate groups, or leave one of them out of the selection."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceJunctionBeadsDoNotShareOneCanal => format(
                &localize("The two inside edges at {0} do not round into one another at that corner. Try a smaller size, or bevel them in separate groups."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceJunctionBeadsSampledDifferently => format(
                &localize("The two inside edges at {0} are not rounded the same way, so this bevel cannot close the corner between them. Bevel them in separate groups, or give them the same size."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceMixedCornerRunOutPastTheCrease => format(
                &localize("At this size the bevel at {0} runs out past the end of the edge it would fade away on. Try a smaller size, which keeps it on that edge."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceJunctionCanalRunsOffTheBead => format(
                &localize("The corner at {0} is wider than the inside edge it would end on. Try a smaller size, which keeps it on that edge."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceJunctionOfFourOrMoreEdges => format(
                &localize("Four or more selected edges meet at {0}, which this bevel does not turn yet. Leave some of them out of the selection, or bevel them in separate groups."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceMixedJunctionOfChamfers => format(
                &localize("The corner at {0} is a chamfer where two cuts meet a crease, which this bevel does not turn yet. Use a fillet there instead, or bevel the edges meeting there in separate groups."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceMixedJunctionNotRead => format(
                &localize("The selected edges meeting at {0} are not a corner this bevel can turn yet. Bevel them in separate groups, or leave one of them out of the selection."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceLandsOnTwoCurvedFaces => format(
                &localize("The edge at {0} has a curved face on both of its sides, which this bevel cannot follow yet. Bevel this edge in a group of its own, or leave it out of the selection."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceCutMeetsStandingFeature => format(
                &localize("At {0} the bevel reaches back into something standing on that face - a boss, a rib or the mouth of a hole - and would leave its walls resting on nothing. Try a smaller size, which keeps clear of it."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceCreaseInterruptedByAFeature => format(
                &localize("At this size the bevel at {0} would pass under something standing on the crease and out the other side of it. Try a smaller size, whose rounding keeps clear of that thing's root and ends on its faces instead."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceStandingFeatureInTheBand => format(
                &localize("Something stands inside the width of this bevel at {0} - the rounding reaches past its foot, so the ground it stands on is ground the bevel would take away. Try a smaller size, which keeps clear of its foot."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceBiteDoesNotRead => format(
                &localize("Something standing on the face beside the edge at {0} cuts into the middle of this bevel, in a shape that could not be read. Try a smaller size, which keeps clear of its foot."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceStandingFeatureDoesNotClose => format(
                &localize("At {0} the bevel reached back under something standing on that face, and the shape that thing stands on does not close where the bevel ends. Try a smaller size, which keeps clear of it."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceStandingFeatureUnderTwoCuts => format(
                &localize("At {0} two of the selected edges both reach back under the same thing standing on that face, which this bevel cannot close from both sides at once. Try a smaller size, or bevel those two edges in separate groups."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend => format(
                &localize("At {0} this bevel cannot carry a shape standing on that face out over the rounding it took the ground from. Try a smaller size, which keeps clear of it."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceStandingFeatureExtensionFolds => format(
                &localize("At {0} carrying a shape standing on that face out over the rounding would fold its own wall back on itself. Try a smaller size, which keeps clear of it."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceSectorLeavesTheFace => format(
                &localize("At {0} the bevel turning that corner reaches past the far edge of the face it runs across, so what it takes out of that face has no boundary to close along. Try a smaller size, or bevel the edges meeting there in separate groups."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceCrossingLeavesTheBand => format(
                &localize("At {0} the bevel turning that corner crosses the middle of another selected edge's bevel rather than running off its end, which would cut that bevel in two. Try a smaller size, or bevel those edges in separate groups."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceCrossingMeetsAMitredBand => format(
                &localize("At {0} the bevel turning that corner crosses another selected edge's bevel where that bevel turns a corner of its own, which is not built yet. Try a smaller size, or bevel those edges in separate groups."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceCrossingsShareOneBand => format(
                &localize("At {0} two corners' bevels cross one selected edge's bevel, which is not built yet. Try a smaller size, or bevel those edges in separate groups."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceCrossingProfilesDoNotMatch => format(
                &localize("At {0} the bevel turning that corner crosses another selected edge's bevel with the two rounded at different steps, so the two have no shared line to be joined along. Try a smaller size, or bevel those edges in separate groups."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceCrossingBandNotAlongTheAxis => format(
                &localize("At {0} the bevel turning that corner crosses another selected edge's bevel at an angle this construction cannot state exactly. Try a smaller size, or bevel those edges in separate groups."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceCrossingStandsOnTheWall => format(
                &localize("At {0} the bevel turning that corner crosses another selected edge's bevel standing on the wall it turns against, which is not built yet. Try a smaller size, or bevel those edges in separate groups."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceCrossingMissesTheMitre => format(
                &localize("At {0} the bevel turning that corner crosses two bevels meeting at a corner of another face, and nowhere along the line those two share does it hand over from one to the other. Try a smaller size, or bevel those edges in separate groups."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceCrossingMissesTheChord => format(
                &localize("At {0} the bevel turning that corner crosses two bevels meeting at a corner of another face, and part way along that crossing the two have no shared line left to meet on. Try a smaller size, or bevel those edges in separate groups."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceCrossingSwallowsTheMitre => format(
                &localize("At {0} the bevel turning that corner has swallowed a whole corner of another face's bevel, leaving nothing of it standing. Try a smaller size, or bevel those edges in separate groups."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceCrossingAtTheRunOutsDeath => format(
                &localize("At {0} the bevel turning that corner only just reaches another selected edge's bevel, where the two share a point rather than a line. Try a smaller size, or bevel those edges in separate groups."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceCrossingTurnsBackBeyondTheRunOut => format(
                &localize("At {0} the bevel turning that corner crosses another selected edge's bevel along a curve this construction does not join yet. Try a smaller size, or bevel those edges in separate groups."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceStripRunsOffACurvedFace => format(
                &localize("At {0} the bevel takes the whole of the face it lands on and runs off its edge onto a curved face beyond, which it cannot trim against yet. Try a smaller size, which stays on the face."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceStripRunsOffAtACorner => format(
                &localize("At {0} the bevel takes the whole of the face it lands on and runs off its edge onto two faces meeting at a corner beyond, which it cannot trim against both at once yet. Try a smaller size, which stays on the face."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceBandRunsOffMidSpan => format(
                &localize("At {0} the bevel runs off the middle of the face it lands on and back onto it again, which it cannot do over one edge. Try a smaller size, which stays on the face."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceRunOffMeetsAMitredEnd => format(
                &localize("At {0} the bevel runs off the face it lands on right where it turns into the next edge of the same chain, and those two cannot both end it there. Try a smaller size, which stays on the face."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceHullEndNotSupported => format(
                &localize("At {0} the bevel runs off the end of the part in a way it cannot close exactly on the faces there. Try a smaller size, which ends on the face."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceHullEndGivesUp => format(
                &localize("At {0} the bevel leaves the part across a face it runs INTO, which would have to be cut back for it rather than closed against. Try a smaller size, which ends on the face."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceHullEndOnThreePlanes => format(
                &localize("At {0} the bevel leaves the part through three of its faces at once, which it does not cap yet. Try a smaller size, which leaves through fewer."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceHullEndCrossesTheCorner => format(
                &localize("At {0} the bevel leaves the part across a corner of it more than once, so there is no single point for its end to turn at. Try a smaller size, or pick fewer edges."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceStripRunsOffBothSides => format(
                &localize("At {0} the bevel takes the whole of the faces on BOTH sides of the edge and runs off each of them, which it cannot trim against two planes at once. Try a smaller size, which stays on the faces."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceMitreConsumesTheStrip => format(
                &localize("At {0} the size is wide enough that the bevels either side of one short edge eat the whole of it. Try a smaller size, which leaves that edge something to stand on."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceRunOffMeetsACrossing => format(
                &localize("At {0} one end of a bevel is both crossed by the bevel turning a corner and trimmed by the face it runs off onto, which is not built yet. Try a smaller size, or bevel those edges in separate groups."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceRunOffReadingDisagrees => format(
                &localize("The bevel could not be measured the same way twice at {0}, so it was not built rather than built on a guess. Try a smaller size, or bevel this edge in a group of its own."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceRunOffReadingsDisagree => format(
                &localize("The bevel could not be measured the same way twice where it leaves the part at {0}. Try a smaller size, or bevel this edge in a group of its own."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceTwoCornersAtOneVertex => format(
                &localize("At {0} two of the loops you picked turn a corner at the same point of the part, which this bevel does not build yet. Bevel them in separate groups, or leave one of them out of the selection."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceRunOutLiesOnAFaceAlreadyDrawn => format(
                &localize("At {0} the piece where one bevel runs out over another lands on top of a piece that is already there. Try a smaller size, or bevel those edges in separate groups."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::SurfaceResultNotClosed => format(
                &localize("The surface this bevel built does not close at {0}. Try a smaller size, or bevel this edge in a group of its own."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::LoopTouchesAnotherPickedLoop => format(
                &localize("This loop meets another selected loop at a corner ({0}), so the two can't be rounded in the same bevel."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::BuiltWithTheOtherConstruction => format(
                &localize("Built with the {0} construction."),
                &[at(args, 0)],
            ),
            BevelAdvisoryCode::ConstructionCouldNotBuildThisGroup => localize("Something went wrong while building the bevel along these edges. A smaller size may work."),
            BevelAdvisoryCode::SurfaceConstructionNotBuilt => localize("The bevel could not be built along these edges. Try a smaller size, or bevel fewer edges at once."),
            BevelAdvisoryCode::SurfaceMixedSizesInOnePass => localize("One part cannot be bevelled at two sizes or two shapes at once: where those bevels meet there is no single corner to build. Give the groups the same size and shape, or bevel them one after another."),
            BevelAdvisoryCode::NoEdgesResolved => localize("The group resolved to no edges at all."),
            BevelAdvisoryCode::LoopTooShortToEnclose => localize("This selection closes into a loop of fewer than three edges, which encloses nothing to cut with."),
            BevelAdvisoryCode::LoopSurfacesSwapped => format(
                &localize("This selection comes back to {0} with its two surfaces swapped over, so no single tool can be swept round it."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::ToolCouldNotClose => localize("The cutting tool for this chain of edges could not be closed up."),
            BevelAdvisoryCode::ToolSelfIntersects => localize("The cutting tool swept along this chain of edges crosses itself, so the bevel is too big for the shape it follows."),
            BevelAdvisoryCode::ToolNotUsableSolid => localize("The cutting tool swept along this chain of edges is not a solid the boolean can use."),
            BevelAdvisoryCode::RunMixedConvexity => format(
                &localize("The edges in this run do not all turn the same way at {0}, so no single tool can be swept along it."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::EdgeHasNoLength => localize("One of the edges in this chain has no length."),
            BevelAdvisoryCode::CornerSeamNotCarried => format(
                &localize("The rounded corner at {0} and the run meeting it could not be made to share the same points, so the two may meet roughly there."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::CrossSectionChanged => localize("The cross section changed shape along this chain of edges."),
            BevelAdvisoryCode::ChainDoublesBack => format(
                &localize("The chain of edges doubles straight back on itself at {0}, so no tool can be swept through that corner."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::ChainTurnsTooSharply => format(
                &localize("The chain of edges turns too sharply at {0} to sweep a tool through."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::NoPlaneAtJoint => format(
                &localize("The two edges meeting at {0} do not leave a plane to place the bevel in."),
                &[text(describe_at(args, 0))],
            ),
            BevelAdvisoryCode::CornerBallDoesNotTouchAllInsideEdges => format(
                &localize("The {0} rounded inside edges meeting at {1} do not all touch one ball of that size, so no rounded corner could be built there."),
                &[at(args, 0), text(describe_at(args, 1))],
            ),
            BevelAdvisoryCode::CornerBallDoesNotTouchAllEdges => format(
                &localize("The {0} rounded edges meeting at {1} do not all touch one ball of that size, so no rounded corner could be built there."),
                &[at(args, 0), text(describe_at(args, 1))],
            ),
            BevelAdvisoryCode::CornerPatchNotSolidToAdd => format(
                &localize("The rounded corner where {0} inside edges meet at {1} could not be built as a solid to add."),
                &[at(args, 0), text(describe_at(args, 1))],
            ),
            BevelAdvisoryCode::CornerPatchNotSolidToCut => format(
                &localize("The rounded corner where {0} edges meet at {1} could not be built as a solid to cut with."),
                &[at(args, 0), text(describe_at(args, 1))],
            ),
            BevelAdvisoryCode::SizeRemovesNothing => format(
                &localize("A size of {0} removes nothing."),
                &[at(args, 0)],
            ),
            BevelAdvisoryCode::FacesCoplanar => localize("The two faces along this edge lie in the same plane, so there is no corner to bevel."),
            BevelAdvisoryCode::FacesFoldBack => localize("The two faces along this edge fold back on each other, so there is no corner to cut."),
            BevelAdvisoryCode::NoWedgeToClipAgainst => localize("The faces along this edge do not open a wedge to clip the bevel against."),
            BevelAdvisoryCode::ChamferHasNoBall => localize("A chamfer has no rolling ball to sweep."),
            BevelAdvisoryCode::FacesDoNotCloseCorner => localize("The faces along this edge do not close a corner for a ball to sit in."),
            BevelAdvisoryCode::FaceAngleTooSharp => face_angle_sentence(at(args, 0), localize("sharp")),
            BevelAdvisoryCode::FaceAngleTooFlat => face_angle_sentence(at(args, 0), localize("flat")),
            BevelAdvisoryCode::BevelDoesNotFitCorner => localize("The bevel does not fit in the corner between these faces."),
            BevelAdvisoryCode::ChainNeverGrewPastPick => format(
                &localize("This tangent chain never grew past the edge that was picked: at a flat tolerance of {0:0.###} degrees every facet join of this surface is a feature edge of its own, so three of them meet at each end of that edge and there is no single way on. Choose a coarser reading of this pick - from the row's dropdown in the Edges list, or from the badge beside the edge in the 3D view - because the tolerance ladder goes up to {1:0.###} degrees, which reads a tessellated surface as one surface and walks the whole run."),
                &[at(args, 0), at(args, 1)],
            ),            _ => format_blend_or_operation(advisory),
        }
}

/// A refusal with the size that would fit said after it, where there is one.
pub fn with_the_size_that_fits(refusal: &str, largest_that_fits: Option<f64>) -> String {
    let Some(largest_that_fits) = largest_that_fits else {
        return refusal.to_string();
    };
    refusal.to_string()
        + " "
        + &format(
            &localize("Up to {0:0.###} mm fits here - try that size or less."),
            &[BevelAdvisoryArg::Double(largest_that_fits)],
        )
}

/// Several advisories as one line, or `None` when there are none.
pub fn join_advisories(advisories: &[BevelAdvisory]) -> Option<String> {
    let sentences: Vec<String> = advisories.iter().map(format_advisory).collect();
    if sentences.is_empty() {
        None
    } else {
        Some(sentences.join(" "))
    }
}
