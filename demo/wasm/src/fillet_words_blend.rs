//! The second and third parts of the demo's sentence switch (`fillet_words` is the first): the
//! mixed corner, blend and whole-operation sentences (`format_blend_or_operation`), and the
//! sentences a pick comes back with when it cannot find its edge or corner again
//! (`format_resolution`).

use solidmesh::{BevelAdvisory, BevelAdvisoryCode};

use crate::fillet_words_parts::{at, describe_at, format, localize, text};

/// The mixed corner, blend and whole-operation sentences, and the orchestration's own refusals
/// and notes.
pub fn format_blend_or_operation(advisory: &BevelAdvisory) -> String {
    let args = advisory.args();

    match advisory.code() {
            BevelAdvisoryCode::SourceMeshHadFoldedFlaps => format(
                &localize("This source mesh folds back on itself in {0} place(s) - the same triangle is in it twice, facing both ways at once, which is what a boolean can leave where two shapes met flush. Those doubled faces have been dropped to bevel it; the source itself is unchanged."),
                &[at(args, 0)],
            ),
            BevelAdvisoryCode::SourceMeshHadTVertexSlivers => format(
                &localize("This source mesh leaves a seam half finished in {0} place(s) - a triangle with no area at all, laid where one edge ends against the middle of another, which is what a boolean can leave where one shape's face met another shape's edge part way along. Those flat triangles have been dropped and the face beside each one split at the point it stood on, to bevel it; the source itself is unchanged."),
                &[at(args, 0)],
            ),
            BevelAdvisoryCode::SourceMeshHadThinSlivers => format(
                &localize("This source mesh has {0} triangle(s) in it only microns thick - slivers standing on edge where one shape pokes a hair through another's face and the boolean writes the same point down twice, a few millionths apart. Each of those doubled points has been put back together and the slivers with it, to bevel the surface they were lying across; the source itself is unchanged."),
                &[at(args, 0)],
            ),
            BevelAdvisoryCode::SourceMeshWasInsideOut => localize("This part was inside out - every one of its faces pointed into the solid rather than out of it - and has been turned right way out so it could be bevelled. The bevelled part comes back facing the right way; check that it is the shape you expect."),
            BevelAdvisoryCode::PartHasNoMeshToBevel => localize("This selection's part has no mesh to bevel."),
            BevelAdvisoryCode::CornerFormulaEmpty => localize("The corner formula is empty. Write an expression that starts with =, or switch to Select or Angle."),
            BevelAdvisoryCode::CornerFormulaCouldNotBeEvaluated => format(
                &localize("The corner formula could not be evaluated: {0}"),
                &[at(args, 0)],
            ),
            BevelAdvisoryCode::PointMeetsAPickedEdge => localize("This point meets a picked edge - bevel it with the edges, or remove the point."),
            BevelAdvisoryCode::PointIsNoLongerACorner => localize("This point is no longer a corner or tip of the part."),
            BevelAdvisoryCode::PointPickedTwice => localize("Another pick already bevels this point - remove one of them."),
            BevelAdvisoryCode::PointTooLargeForAnySize => localize("too large for this point - try a much smaller size"),
            BevelAdvisoryCode::PointTooLargeRoomFor => format(
                &localize("too large: this point has room for {0:0.###}"),
                &[at(args, 0)],
            ),
            BevelAdvisoryCode::PointHasSharpEdges => localize("Sharp edges meet this point - fillet the edges instead."),
            BevelAdvisoryCode::PointHoldsNoBall => localize("the faces around this point hold no ball"),
            BevelAdvisoryCode::PointHasNoDirectionToCutSquareTo => localize("this point has no direction to cut square to"),
            BevelAdvisoryCode::PointIsNotConvex => localize("this point is not convex"),
            BevelAdvisoryCode::PointSizeNotPositive => localize("the size must be positive"),
            BevelAdvisoryCode::PointRingNotResolved => localize("the ring could not be resolved on this mesh"),
            BevelAdvisoryCode::PointRingNotOneLoop => format(
                &localize("the ring is not one loop ({0} lines)"),
                &[at(args, 0)],
            ),
            BevelAdvisoryCode::PointChamferRingNotFlat => localize("the chamfer ring does not close flat"),
            BevelAdvisoryCode::PointRoundDidNotClose => format(
                &localize("the round would not close ({0})"),
                &[at(args, 0)],
            ),
            _ => format_resolution(advisory),
    }
}

/// The two resolvers' refusals, on the mesh leg and the 2D one.
fn format_resolution(advisory: &BevelAdvisory) -> String {
    let args = advisory.args();

    match advisory.code() {
            BevelAdvisoryCode::NoFeatureEdgesAtTolerance => format(
                &localize("The mesh has no feature edges at a flat tolerance of {0} degrees, so there is nothing this group could be beveling. An unwelded mesh - triangles that do not share vertices - reads this way however sharp it looks."),
                &[at(args, 0)],
            ),
            BevelAdvisoryCode::GroupHasNoAnchorDirection => localize("The group has no anchor direction, so it was never given an edge to remember."),
            BevelAdvisoryCode::NoFeatureEdgeMatchesAnchor => format(
                &localize("No feature edge matches this group's anchor at {0} any more. None of the {1} feature edges is within {2:0.###} of it with its direction inside {3} degrees and both faces inside {4} degrees, so the edge it was made on is gone."),
                &[text(describe_at(args, 0)), at(args, 1), at(args, 2), at(args, 3), at(args, 4)],
            ),
            BevelAdvisoryCode::AnchorMatchesMoreThanOneEdge => format(
                &localize("More than one edge fits this group's anchor at {0} about equally well (best {1:0.###} against next best {2:0.###}), so which one it meant can no longer be told. Repeated features - teeth, fins, a bolt circle - read this way once the one that was selected is deleted."),
                &[text(describe_at(args, 0)), at(args, 1), at(args, 2)],
            ),
            BevelAdvisoryCode::FaceRegionRimIsPartial => format(
                &localize("This face is beveled all the way round except at {0} of its edges, where the part is open or folds back on itself and has no second face to bevel against. Pick the edges you want beveled on their own, or repair the part so it closes there."),
                &[at(args, 0)],
            ),
            BevelAdvisoryCode::FaceRegionBoundedOnlyByOpenEdges => format(
                &localize("The face region at {0} is bounded only by open or non-manifold edges ({1} of them), which have no second face to bevel against."),
                &[text(describe_at(args, 0)), at(args, 1)],
            ),
            BevelAdvisoryCode::FaceRegionHasNoBevelableBoundary => format(
                &localize("The face region at {0} has no bevelable boundary: nothing around it turns more than {1} degrees."),
                &[text(describe_at(args, 0)), at(args, 1)],
            ),
            BevelAdvisoryCode::PathHasNoCorners => format(
                &localize("The path has no corners that bend more than {0} degrees, so there is nothing this group could be rounding."),
                &[at(args, 0)],
            ),
            BevelAdvisoryCode::GroupHasNoAnchorDirections => localize("The group has no anchor directions, so it was never given a corner to remember."),
            BevelAdvisoryCode::NoCornerMatchesAnchor => format(
                &localize("No corner matches this group's anchor at {0} any more. None of the {1} corners is within {2:0.###} of it with both of its edges inside {3} degrees, so the corner it was made on is gone."),
                &[text(describe_at(args, 0)), at(args, 1), at(args, 2), at(args, 3)],
            ),
            BevelAdvisoryCode::AnchorMatchesMoreThanOneCorner => format(
                &localize("More than one corner fits this group's anchor at {0} about equally well (best {1:0.###} against next best {2:0.###}), so which one it meant can no longer be told. Repeated features - teeth, the points of a star, a row of tabs - read this way once the one that was selected is deleted."),
                &[text(describe_at(args, 0)), at(args, 1), at(args, 2)],
            ),        // Every other code is formatted by `bevel_advisory_text::format_advisory`, the first part
        code => code.to_string(),
    }
}
