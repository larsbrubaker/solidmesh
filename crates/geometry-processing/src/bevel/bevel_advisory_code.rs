//! Every distinct thing the bevel kernel has to say to a user. Ports `Bevel/BevelAdvisoryCode.cs`:
//! one member per sentence, in the C# declaration order, with the values the sentence quotes
//! carried as [`super::BevelAdvisory::args`] in the order documented on each member. The words
//! live in the application (`BevelAdvisoryText`), which keeps the kernel free of localization.
//!
//! Positions are carried as raw `Vector3` values and formatted by the app, so a test can assert on
//! the corner an advisory names without parsing a sentence. Where a sentence used to be chosen by
//! a bool (fill or cut, sharp or flat) the two are separate codes, so that the argument list of
//! any one code is fixed. `Display` prints the member name, as C# `Enum.ToString` does.

use std::fmt;

/// Which sentence a [`super::BevelAdvisory`] is (C# `BevelAdvisoryCode`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BevelAdvisoryCode {
    /// BevelCapClip.ApplyAsync: a clip the kernel refused. Args: (Vector3 at, double size).
    CapClipCouldNotCut,
    /// BevelChainEndBudget.HeldBack, one end bounded. Args: (Vector3 at, double size, int edgeCount).
    HeldBackAtOneEnd,
    /// BevelChainEndBudget.HeldBack, both ends bounded. Args: (Vector3 start, Vector3 end, double size, int edgeCount).
    HeldBackAtBothEnds,
    /// BevelChainEnds.ResolveEnd: a bead squared off where its faces could not cap it. Args: (Vector3 at).
    BeadSquaredAtCorner,
    /// BevelChainEnds.Resolve: a bead running on through what interrupts its crease
    /// (`BevelInterruptedCrease`) whose reach does not carry it to the far side, so a stretch
    /// of the crease inside the interruption is left unbeveled. Args: (Vector3 at, double size, double gap).
    BeadDoesNotSpanInterruption,
    /// BevelChainEnds.ResolveEnd: a cutter overshooting a corner no face could trim. Args: (Vector3 at).
    CutRunsPastCorner,
    /// BevelChainSweep.TryCreate and BevelGroupTools.TryBuild: an empty chain. No args.
    NoEdgesResolved,
    /// BevelChainSweep.TryCreate: a closed loop of fewer than three edges. No args.
    LoopTooShortToEnclose,
    /// BevelChainSweep.TryCreate: an open run that returns to its start with its surfaces swapped. Args: (Vector3 at).
    LoopSurfacesSwapped,
    /// BevelChainSweep.TryCreate: the rings would not sew into a mesh. No args.
    ToolCouldNotClose,
    /// BevelChainSweep.TryCreate: the swept tool crosses itself. No args.
    ToolSelfIntersects,
    /// BevelChainSweep.TryCreate: the swept tool is not a boolean operand. No args.
    ToolNotUsableSolid,
    /// BevelChainSweep.TryBuildSegments: a run mixing convex and concave edges. Args: (Vector3 at).
    RunMixedConvexity,
    /// BevelChainSweep.TryBuildSegments: a zero length edge. No args.
    EdgeHasNoLength,
    /// BevelChainSweep.TryBuildSegments: profiles of different point counts along one run. No args.
    CrossSectionChanged,
    /// BevelChainSweep.TryBuildJoint: two segments meeting at 180 degrees. Args: (Vector3 at).
    ChainDoublesBack,
    /// BevelChainSweep.TryBuildJoint: a miter past the sharpest turn a sweep carries. Args: (Vector3 at).
    ChainTurnsTooSharply,
    /// BevelChainSweep.TryBuildJoint: no across axis survives in the miter plane. Args: (Vector3 at).
    NoPlaneAtJoint,
    /// BevelSweepFoldCheck.TryCheckForFolds: a cap's slide outruns its segment. Args: (Vector3 at, double size, double largestThatFits).
    CapPullsToolBackPastSegment,
    /// BevelSweepFoldCheck.TryCheckForFolds: a miter folds the strip. Args: (double size, Vector3 near, double largestThatFits).
    SizeFoldsToolAtCurve,
    /// BevelCornerGroupResolver.Resolve: the path has no corners at its bend threshold. Args: (double bendThresholdDegrees).
    PathHasNoCorners,
    /// BevelCornerGroupResolver.Resolve: a group whose anchor directions are zero. No args.
    GroupHasNoAnchorDirections,
    /// BevelCornerGroupResolver.Resolve: no corner passes every gate. Args: (Vector2 anchorPosition, int cornerCount, double maximumDrift, double directionToleranceDegrees).
    NoCornerMatchesAnchor,
    /// BevelCornerGroupResolver.Resolve: two corners fit about equally well. Args: (Vector2 anchorPosition, double bestScore, double runnerUpScore).
    AnchorMatchesMoreThanOneCorner,
    /// BevelCornerPatch.TryBuild, inside corner: no one ball touches every edge. Args: (int edgeCount, Vector3 at).
    CornerBallDoesNotTouchAllInsideEdges,
    /// BevelCornerPatch.TryBuild, outside corner: no one ball touches every edge. Args: (int edgeCount, Vector3 at).
    CornerBallDoesNotTouchAllEdges,
    /// BevelCornerPatch.TryBuild, inside corner: the fill is not a solid. Args: (int edgeCount, Vector3 at).
    CornerPatchNotSolidToAdd,
    /// BevelCornerPatch.TryBuild, outside corner: the scoop is not a solid. Args: (int edgeCount, Vector3 at).
    CornerPatchNotSolidToCut,
    /// BevelEdgeGroupResolver.TryMatchAnchor: the mesh has no feature edges at the group's flat tolerance. Args: (double flatAngleDegrees).
    NoFeatureEdgesAtTolerance,
    /// BevelEdgeGroupResolver.TryMatchAnchor: a group whose anchor direction is zero. No args.
    GroupHasNoAnchorDirection,
    /// BevelEdgeGroupResolver.TryMatchAnchor: no feature edge passes every gate. Args: (Vector3 anchorMidpoint, int featureEdgeCount, double maximumDrift, double directionToleranceDegrees, double normalToleranceDegrees).
    NoFeatureEdgeMatchesAnchor,
    /// BevelEdgeGroupResolver.TryMatchAnchor: two edges fit about equally well. Args: (Vector3 anchorMidpoint, double bestScore, double runnerUpScore).
    AnchorMatchesMoreThanOneEdge,
    /// BevelEdgeGroupResolver.DescribeEmptyBoundary: the flooded region is rimmed only by open or non-manifold edges. Args: (Vector3 anchorMidpoint, int openBoundaryEdgeCount).
    FaceRegionBoundedOnlyByOpenEdges,
    /// A face region's rim is only PART of its boundary: the rest of what bounds it is open or
    /// non-manifold, and a bevel has no second face to build against there. The group still applies to
    /// what it did select. Args: (int droppedEdgeCount).
    FaceRegionRimIsPartial,
    /// BevelEdgeGroupResolver.DescribeEmptyBoundary: the flooded region never turns past its curve tolerance. Args: (Vector3 anchorMidpoint, double curveToleranceDegrees).
    FaceRegionHasNoBevelableBoundary,
    /// BevelEdgeToolBuilder.TryValidateSize: a size of zero or less. Args: (double size).
    SizeRemovesNothing,
    /// BevelEdgeToolBuilder.TryGetProfileNormals: a flat edge. No args.
    FacesCoplanar,
    /// BevelEdgeToolBuilder.TryBuildFrame: the two normals cancel. No args.
    FacesFoldBack,
    /// BevelEdgeToolBuilder.TryBuildVoidWedgeCrossSection: no positive tangent distance. No args.
    NoWedgeToClipAgainst,
    /// BevelEdgeToolBuilder.TryBuildBallCrossSection: asked for a chamfer. No args.
    ChamferHasNoBall,
    /// BevelEdgeToolBuilder.TryBuildBallCrossSection: the faces open to a straight line. No args.
    FacesDoNotCloseCorner,
    /// BevelEdgeToolBuilder.TryMeasureCorner: interior angle under the minimum. Args: (double interiorAngleDegrees).
    FaceAngleTooSharp,
    /// BevelEdgeToolBuilder.TryMeasureCorner: interior angle over the maximum. Args: (double interiorAngleDegrees).
    FaceAngleTooFlat,
    /// BevelEdgeToolBuilder.TryMeasureCorner: a tangent distance that is not a number. No args.
    BevelDoesNotFitCorner,
    /// BevelFaceFit.TryCheckGroup: two setbacks meet across one face. Args: (double size, Vector3 at, double largestThatFits).
    SizeCollidesAcrossFace,
    /// BevelFaceFit.TryCheckGroup: a setback wider than its face. Args: (double size, Vector3 at, double largestThatFits).
    SizeExceedsFaceWidth,
    /// BevelGroupFailures.Diagnose: a tangent chain stopped at its seed by the tolerance. Args: (double flatAngleDegrees, double coarseFlatAngleDegrees).
    ChainNeverGrewPastPick,
    /// BevelMixedBlends.Blend: two edges of one convexity in one group. Args: (Vector3 at).
    BlendSameGroupNotBlended,
    /// BevelMixedBlends.Blend: a three-edge corner between more than three faces. Args: (Vector3 at).
    BlendMoreThanThreeFaces,
    /// BevelMixedBlends.TryBuild: the inner corner runs along the cap face. Args: (Vector3 at).
    BlendRimRunsAlongFace,
    /// BevelMixedBlends.TryBuild: the ball's basis has collapsed. Args: (Vector3 at).
    BlendFacesTooNearlyAligned,
    /// BevelMixedBlends.TryBuild: the inner corner turns less than the minimum. Args: (Vector3 at).
    BlendCornerFoldsTooLittle,
    /// BevelMixedBlends.TryBuild: the inner corner leans past the off-square budget. Args: (Vector3 at).
    BlendCornerNotSquareEnough,
    /// BevelMixedBlends.TryBuild: a handover seam no run can stop on. Args: (Vector3 at).
    BlendRimTooNearlyAlongCorner,
    /// BevelMixedBlends.TryBuild: the blend tool is not a solid. Args: (Vector3 at).
    BlendNotSolidToCut,
    /// BevelMixedCorners.CappedCorner: a mixed corner every run stops at. Args: (int edgeCount, Vector3 at).
    MixedCornerCapped,
    /// BevelMixedCorners.Describe: why four or more mixed edges are not blended; always follows MixedCornerCapped. No args.
    MixedCornerTooManyEdges,
    /// BevelRebuildAdvisories.BeadWentOnUnclipped. No args.
    BeadWentOnUnclipped,
    /// BevelRebuildAdvisories.ShadowClipWasRefused. No args.
    ShadowClipWasRefused,
    /// BevelRebuildAdvisories.ShadowBodyWasGivenUp. No args.
    ShadowBodyWasGivenUp,
    /// BevelRebuildAdvisories.CornerWentUnrounded. No args.
    CornerWentUnrounded,
    /// BevelRebuildAdvisories.SecondPassToolWasNotABead. No args.
    SecondPassToolWasNotABead,
    /// BevelSecondPass.Rebead: two groups whose creases the merge put in one run read stretches of it
    /// that will not join into a single chain, so the run swept the longest of them and this group's own
    /// reading was not covered by it. Its first-pass bead is kept. No args.
    SecondPassCreaseDidNotJoin,
    /// BevelChainSweep.TryBuildRings: a run could not take the corner patch's own seam points onto its ring. Args: (Vector3 at).
    CornerSeamNotCarried,
    /// BevelWorkingMesh.CreateFrom: the source was reversed to bevel it. No args.
    SourceMeshWasInsideOut,
    /// BevelSurfacePartBuilder: the group is one `Surface` would build,
    /// and that construction is not written yet, so it is refused rather than quietly handed to the
    /// boolean one - the plan's no-fallback rule. Goes with the mode, at the end of the plan. No args.
    SurfaceConstructionNotBuilt,
    /// BevelConstructionFallThrough: the construction THREW on this group rather than refusing in words, which to the user is the same thing - nothing was built at this size. A complaint, said on the group's own row. Args: (string construction).
    ConstructionCouldNotBuildThisGroup,
    /// SurfaceBevelBuilder: one pass carries chains planned at different sizes or with different
    /// profiles. A corner reads its size and profile off whichever of the chains meeting there seeded
    /// it, so a mixed pass would build one chain's bevel at another's corner; it is refused at the top
    /// of the build instead. No args.
    SurfaceMixedSizesInOnePass,
    /// SurfaceBevelBuilder: the strip's end ring does not land on the vertices the walkback left on the
    /// tangent lines, so sewing it would leave a crack or a T junction rather than a seam. Args:
    /// (Vector3 at).
    SurfaceStripSeamDidNotMeet,
    /// SurfaceBevelBuilder: two strips meeting at a corner cross the mitre plane at different points, so
    /// there is no one ring for them to share - the two sides of the chain are not one face each there.
    /// Args: (Vector3 at).
    SurfaceStripsDoNotMitre,
    /// SurfaceEndCut: the open end of a chain has no single unselected face beyond its corner holding the
    /// whole end profile, so there is nothing to notch the profile out of. Args: (Vector3 at).
    SurfaceStripEndNotSupported,
    /// SurfaceCornerPatch: more than one chain ends at one vertex of the part, in a way this construction
    /// does not close - two ends, which are a mitre the linker would not walk through; four or more; or
    /// three whose faces do not cut one ball centre out between them. Args: (Vector3 at).
    SurfaceCornerNotSupported,
    /// SurfaceBuilderMixedCorners: a LEANING cap-riding corner reads (`SurfaceMixedCornerSpine`) but
    /// hands over on a MITRE RING, whose grid `SurfaceMixedCornerRingSchedule` turns rigidly - the
    /// torus's carry, right only for a square axis. Raised so right reading with wrong rows is never
    /// published as a silently wrong blend. Args: (Vector3 at, double leanDegrees).
    SurfaceObliqueMixedCornerRowsNotBuilt,
    /// SurfaceMixedCornerSpine: the plane of the ball's contact ring on the TUBE
    /// (`SurfaceMixedCornerSpine.TubeRingPlane`) runs ALONG the tube's axis, so it crosses none of
    /// that strip's rails. Only reachable just inside the lean `SurfaceMixedCorner.TryRead` refuses.
    /// "Bead" in the name is historical: it is the tube's ring, kept for the advisory text tests. Args: (Vector3 at).
    SurfaceMixedCornerBeadRingCrossesNoRail,
    /// SurfaceMixedCornerDualFrame: the corner's OTHER canal - the void ball rolling along the wall -
    /// has no spine, because the CUT's centre line lies in that wall. The plane a radius outside the
    /// wall then meets the cylinder about that line in two straight lines or in nothing at all, and
    /// there is no ellipse to roll a ball on. Asked before anything divides by it. Args: (Vector3 at).
    SurfaceMixedCornerCutLiesInTheWall,
    /// SurfaceMixedCornerDualCanal: a LEANING mixed corner's blend runs out past the end of the edge it
    /// would die on. The canal keeps turning after the cut's own strip has ended, and it stops where the
    /// ball's contact on the wall comes back to the edge the cap and the wall meet in - an edge nobody
    /// picked. Where that return stands past the far end of that edge, or where the ball's rail never
    /// comes back to it at all, the blend is larger than the edge it is running out on and there is
    /// nothing there for it to die against. A smaller size brings it back. Args: (Vector3 at).
    SurfaceMixedCornerRunOutPastTheCrease,
    /// SurfaceMixedJunction, SurfaceTwoCreaseJunction, SurfaceMixedJunctionPatch: TWO of the selected edges
    /// at a vertex are creases in a shape neither reading answers - e.g. one arm handing over at a MITRE,
    /// so the two canals stand about different cut lines. A two-crease junction that READS is still
    /// refused here at the PATCH, whose surface between the end rings is not laid yet. Shapes
    /// `SurfaceTwoCreaseJunction` recognizes get its own names; three creases go
    /// to `SurfaceCornerPatch` (`SurfaceJunctionOfThreeCreases`). Args: (Vector3 at).
    SurfaceJunctionOfTwoCreases,
    /// SurfaceCornerPatch: three CONCAVE beads meet at one vertex and their faces give no ball centre -
    /// the arms span fewer or more than three distinct planes, or three that meet in no single point
    /// (two parallel, or all through one line). Named for the SHAPE rather than the failed solve. A
    /// pocket corner on three planes that meet is BUILT; later failures there keep their own names
    /// (`SurfaceStripSeamDidNotMeet`, `SurfaceCornerNotSupported`). Args: (Vector3 corner).
    SurfaceJunctionOfThreeCreases,
    /// SurfaceTwoCreaseJunction: the two creases at the vertex do NOT stand against one face. The blend
    /// there is a single canal whose ball rolls along the face BOTH beads stand against, pressed against
    /// the cut's tube (`SurfaceMixedCornerDualFrame`); with two faces there are two such canals and
    /// no one surface between them. Args: (Vector3 at).
    SurfaceJunctionOfTwoCreasesOnTwoWalls,
    /// SurfaceTwoCreaseJunction: the two arms of a two-crease junction read the canal between them
    /// differently - the ball the first hands over is not the ball the second starts from, or the two
    /// turns are not one turn. Both follow from the geometry wherever the corner is the shape this
    /// reading answers, so a miss says the reading NEARLY found it rather than found it. Args:
    /// (Vector3 at).
    SurfaceJunctionBeadsDoNotShareOneCanal,
    /// SurfaceTwoCreaseJunction: the canal between the two creases ends on a ball standing PAST the far
    /// end of one of them - the blend is larger than the crease it is drawn on, so the ring that should
    /// close it stands off the bead altogether. A smaller size brings it back. Args: (Vector3 at).
    SurfaceJunctionCanalRunsOffTheBead,
    /// SurfaceTwoCreaseJunctionRings: the two creases of a two-crease junction are sampled at DIFFERENT
    /// numbers of rails, so their two end rings have different numbers of points. The patch between them
    /// pairs row for row - the canal's ring at every station carries the same rails from the wall round
    /// to the cut's tube - and two rings of unequal length have no such pairing. Args: (Vector3 at).
    SurfaceJunctionBeadsSampledDifferently,
    /// SurfaceBevelBuilder: FOUR or more selected edges meet at one vertex of mixed convexity - the
    /// corner readings here answer three arms and no more. The count is the whole of what this says:
    /// three edges that meet and cannot be read are
    /// `SurfaceMixedJunctionNotRead`, and three arriving as one closed chain are neither,
    /// because the chain is cut open at the vertex first
    /// (`SurfaceChainOpensAtTheJunction`) and the junction reads its three
    /// ends. Args: (Vector3 at).
    SurfaceJunctionOfFourOrMoreEdges,
    /// SurfaceRollAcrossTheWall: the edge lands on a surface that CURVES along its section on both of its
    /// sides, so there are two rolls to solve at once and the facet each of them touches depends on the
    /// other. One curved side rolls (`SurfaceRollAcrossTheWall`); two is a
    /// construction of its own and is turned down rather than approximated. Args: (Vector3 at).
    SurfaceLandsOnTwoCurvedFaces,
    /// SurfaceFaceWalkback: the band one face gives up runs into a feature STANDING on that face - a boss,
    /// a rib, the mouth of a bore - whose footprint is a hole in the face. Taking the band would leave
    /// that feature's walls standing on a rim with nothing on the other side of it, which is a mesh no
    /// boolean and no viewer can make sense of, so the bevel is refused until the surface construction
    /// knows how to run a bevel into a standing feature. Args: (Vector3 at).
    SurfaceCutMeetsStandingFeature,
    /// SurfaceInterruptedCrease: the crease is INTERRUPTED by a feature standing on it (a nose welded over
    /// the line) and picked up beyond, and at THIS SIZE the feet reach past the feature's root, so the two
    /// strips would overlap inside it - one joined run notched mid strip, which is not built. A feature
    /// that SPANS the strip is not this: its two pieces are ordinary ends and build ("Center Flows
    /// Through.mcx"). A smaller size builds. Told from `SurfaceCutMeetsStandingFeature`, a band
    /// reaching into a feature BESIDE it. Args: (Vector3 corner).
    SurfaceCreaseInterruptedByAFeature,
    /// SurfaceStandingFeatureStretch: the band passed UNDER a feature standing on the face - which is
    /// allowed, the region going on behind it - but the feature cannot be read as one thing to extend.
    /// Its rim FORKS, so there is no telling which facet stands across which crease, or it runs over
    /// facets that are not one plane, which no flat extension can follow. Args: (Vector3 at).
    SurfaceStandingFeatureDoesNotClose,
    /// SurfaceStandingFeatureStretch: TWO of the pass's bands reach under one feature standing on a
    /// face. Each undercuts part of the same facets, and the two extensions would OVERLAP, so building
    /// them one band at a time lays the same triangle down twice. Extending into the union of two
    /// footprints is a construction of its own - each footprint is convex and their union is not - so
    /// the pair is named rather than built folded. Args: (Vector3 at).
    SurfaceStandingFeatureUnderTwoCuts,
    /// SurfaceStripNotchedByAFeature: a feature standing inside the band's footprint has to be EXTENDED
    /// through the corner the band took until it meets the fillet, and its plane's trace across the
    /// strip cannot be followed. Named rather than guessed at, because every one of these means the
    /// extension would leave the strip somewhere the construction has nothing to say about: a plane
    /// PARALLEL to the rails crosses none of them; a trace that cuts a broken set of rails is two
    /// notches and not one; a trace leaving through the strip's other seam, or through a mitre, runs off
    /// onto a face this end cut does not own. Args: (Vector3 at).
    SurfaceStandingFeatureDoesNotExtend,
    /// SurfaceStandingFeatureFaces: the extended facet is not star-shaped from the vertex opposite its
    /// crease, so the fan that emits it reads a triangle facing BACKWARDS against the facet's own plane.
    /// A fan laid down anyway is a fold in the middle of the feature's wall, so it is named instead.
    /// Args: (Vector3 at).
    SurfaceStandingFeatureExtensionFolds,
    /// SurfaceStripTrimmedAtTheCrossing: a mixed corner's RUN-OUT crosses another chain's BAND part way
    /// ALONG it rather than at one of its ends, so trimming the band there would leave its strip in two
    /// pieces with the crossing curve between them. The construction cuts a strip back from an end and
    /// has nothing to say about a bite out of its middle, so it is named rather than closed at whichever
    /// end happens to be nearer. Args: (Vector3 at).
    SurfaceCrossingLeavesTheBand,
    /// SurfaceMixedCornerBites: a mixed corner's RUN-OUT reaches past the face it runs across - its
    /// ideal die point stands beyond that face's own far edge - and the touch row it traces does not
    /// meet that edge either, so there is no boundary for the cap's sector to close along. The sector is
    /// what the face gives up, and a rim that left the face would be asking for cap that is not there.
    /// Args: (Vector3 at).
    SurfaceSectorLeavesTheFace,
    /// SurfaceStripEnds: a mixed corner's RUN-OUT crosses another chain's BAND at an end that end does
    /// not OPEN at - a mitre with the next edge of a closed rim, or a junction - so there is no end cut
    /// to close on the crossing and the strip's ring there belongs to the corner beside it instead. The
    /// crossing is real and the size is not the problem; what is missing is the reading of the SECOND
    /// band that mitre hands the crossing's own foot to. Args: (Vector3 at).
    SurfaceCrossingMeetsAMitredBand,
    /// SurfaceBevelBuilder: two mixed corners' RUN-OUTS cross ONE band, so that band would have to be cut
    /// back at two curves at once. Each crossing is read against the whole strip, so the second would
    /// stand on material the first has already taken; the pair is named rather than the later one
    /// silently replacing the earlier. Args: (Vector3 at).
    SurfaceCrossingsShareOneBand,
    /// SurfaceRunOutMeetsTheBand: a mixed corner's RUN-OUT crosses another chain's BAND, and the two are
    /// sampled differently - the torus rows and the strip rails do not stand at one another's heights, so
    /// no crossing is one shared vertex and the curve between the two surfaces would have to be
    /// approximated. Both profiles are the same arc at the same size, so this is a chording the pass
    /// cannot reconcile rather than anything about the part, and it is named rather than sewn to the
    /// nearest rail. Args: (Vector3 at).
    SurfaceCrossingProfilesDoNotMatch,
    /// SurfaceRunOutMeetsAWallBand: a mixed corner's RUN-OUT crosses a band standing on the WALL it turns
    /// against, and that band's edge does not run along the bead's own axis. The crossing of the two is
    /// then a quartic rather than the one square root a band parallel to the axis gives, and this pass
    /// reads it in closed form only. Named rather than searched for, so that a curve the construction
    /// cannot state exactly is never approximated. Args: (Vector3 at).
    SurfaceCrossingBandNotAlongTheAxis,
    /// SurfaceBevelBuilder: a mixed corner's RUN-OUT crosses a band standing on the WALL it turns against
    /// rather than on the cap it runs across. The room reading measures that pair and reports it as a
    /// crossing - it is a pair to SEW and not a size to refuse - but the band there ends the run-out's
    /// own COLUMNS where a band on the cap ends its rows, and that sewing is not built yet. The size is
    /// not the problem; the geometry is real and is named rather than published as an open shell.
    /// Args: (Vector3 at).
    ///
    ///
    /// RESERVED, not retired: nothing emits this now that the pair IS sewn
    /// (`SurfaceCrossingAtTheMitre`). Kept because the sewing covers only the pair that MITRES, and
    /// a run-out over a band on the cap and a band on the wall that are not each other's neighbour has
    /// no ring to hand over on - which is this sentence again and is still owed a construction.
    SurfaceCrossingStandsOnTheWall,
    /// SurfaceCrossingAtTheMitre: a mixed corner's RUN-OUT crosses both bands of one MITRED corner, and
    /// the ring those two bands share at their mitre never leaves the run-out's own swept ball along one
    /// chord of itself - it stands clear of the run-out all the way, or it goes under and never comes
    /// back. The hand-over from the one band to the other is the point where that ring leaves the ball,
    /// and without it there is no shared vertex for the two staircases to meet at. Named rather than
    /// approximated to the nearest rail, because a hand-over a chord out is a crack the length of a
    /// chord. Args: (Vector3 at).
    SurfaceCrossingMissesTheMitre,
    /// SurfaceRunOutColumnWalk: the same mitred pair, at a station standing BETWEEN two of the cap
    /// band's row crossings. The column's top there belongs on the band's own chord from one crossing to
    /// the next, and the row below it never put a vertex on the band to start that chord at - either it
    /// stood inside the WALL band's cut at its own crossing turn, or the lapping reading took it back
    /// off the column after it was placed. Topping the column with the row the chord was to have
    /// replaced would draw it over material the cap band removed, so the pair is named instead.
    /// Args: (Vector3 at).
    SurfaceCrossingMissesTheChord,
    /// SurfaceCrossingAtTheMitre: the same pair, with the mitre ring under the run-out at its very FIRST
    /// rail - the one on the cap face - so nothing of the mitre stands outside the removal at all. Both
    /// bands would then be drawn entirely inside material the run-out has taken, and there is no part of
    /// their shared ring left to hand over on. A size that swallows a whole corner of a picked rim is a
    /// refusal and not a crossing. Args: (Vector3 at).
    SurfaceCrossingSwallowsTheMitre,
    /// SurfaceRunOutMeetsAWallBand: a mixed corner's RUN-OUT crosses a band on the WALL it turns against
    /// so late that the band reaches only ONE of its own rails into the run-out, and the hairpin where
    /// the band's profile turns back on itself stands on the turn the run-out dies at anyway. The curve
    /// the two would be sewn along is then a single chord standing on the run-out's own last station,
    /// which is a crossing with nothing either side of it to sew. Named rather than drawn, because what
    /// the two really share there is a point. Args: (Vector3 at).
    SurfaceCrossingAtTheRunOutsDeath,
    /// SurfaceRunOutMeetsAWallBand: a mixed corner's RUN-OUT crosses a band on the WALL it turns
    /// against, and the turn that band's profile TURNS BACK at stands outside the run-out's own quarter
    /// of the torus. The crossing curve then leaves the patch on its near side rather than folding back
    /// on itself at a rail, which is a curve of another shape than the one this construction sews - the
    /// hairpin it collapses to a single chord is not there to collapse. Named rather than sewn against a
    /// turn-back the curve never reaches. Args: (Vector3 at).
    SurfaceCrossingTurnsBackBeyondTheRunOut,
    /// SurfaceBandRunsOffTheFaceBeyond: a band takes the WHOLE of the face it lands on and the strip runs
    /// off the region's convex rim onto a CURVED surface - flooded at
    /// `DefaultCurveToleranceDegrees`
    /// into one tangent-continuous surface whose facets are not one plane. Trimming along a curve is a
    /// construction of its own. (The corpus case, "Chain and Face Should Both work.mcx", is really a plane
    /// tilted 0.0011 degrees; see `BevelChainAndFaceRegressionTests`.) Args: (Vector3 at).
    SurfaceStripRunsOffACurvedFace,
    /// SurfaceBandRunsOffTheFaceBeyond: the same run-off, but beyond is two or more FLATS meeting at a
    /// corner - each rim the band runs off has a different plane past it. The strip would need a trim
    /// that turns a corner, and no size makes two planes one. Told from
    /// `SurfaceStripRunsOffACurvedFace`, which needs a trim along a curve. Measured case:
    /// `SurfaceStripRunsOffAConvexEdgeTests.RunningOffOntoTwoFlatsAtACornerIsNamedACorner`. Args: (Vector3 at).
    SurfaceStripRunsOffAtACorner,
    /// SurfaceBandRunsOffTheFaceBeyond: a band runs off the face it lands on over a stretch in the MIDDLE
    /// of its own edge - or over two stretches, one at each end - so there is face still standing on both
    /// sides of the run-off. The strip would have to leave the part and land on it again over one edge,
    /// which is not one trim, and there is no size at which it becomes one: a smaller size lands the whole
    /// band on the face and a larger one takes the whole of it. Named rather than answered with a size.
    /// Args: (Vector3 at).
    SurfaceBandRunsOffMidSpan,
    /// SurfaceBandRunsOffTheFaceBeyond: a band runs off part of the face it lands on at an end of its edge
    /// the chain carries ON past - a mitre with the next strip, a junction, or a closed rim. The mitre's
    /// own plane stands exactly where the run-off would end the strip, and the two cannot both end it.
    /// Named rather than read as a wall, which would answer with a size for a shape that is not built at
    /// any size. Args: (Vector3 at).
    SurfaceRunOffMeetsAMitredEnd,
    /// SurfaceStripEndsOnTheHull: the strip's end could not be read against the part's own hull - a rail
    /// that meets none of the supporting planes at all, one whose exit stands outside the edge's own
    /// span, or a corner the two planes meet at that the part has no vertex of. The end is then a section
    /// this construction has nothing exact to close on, so it is named rather than closed on a point
    /// minted in the middle of a face. Args: (Vector3 at).
    SurfaceHullEndNotSupported,
    /// SurfaceStripEndsOnTheHull: one piece of a strip's end cap stands on a hull plane whose face the
    /// chain runs INTO. Such a face GIVES the section up - it is cut back and the bevel's surface takes
    /// over there - where this end closes by capping the pieces onto the faces that gain them, so the cap
    /// would be laid over a face that is still standing. A construction of its own, named rather than
    /// built. Args: (Vector3 at).
    SurfaceHullEndGivesUp,
    /// SurfaceStripEndsOnTheHull: the rails of one strip end leave the part through THREE or more of its
    /// hull planes. The end is then a staircase over three planes with two corners in it, and the cap it
    /// leaves is in three pieces rather than two, which is a construction of its own. Args: (Vector3 at).
    SurfaceHullEndOnThreePlanes,
    /// SurfaceStripEndsOnTheHull: the rails of one strip end leave through two hull planes, but not in
    /// two contiguous runs - the end crosses the corner the two planes meet at more than once, or it
    /// crosses it outside the band the rails rule. Either way there is no ONE point the end turns at, so
    /// it is named rather than closed on a turn that is not there. Args: (Vector3 at).
    SurfaceHullEndCrossesTheCorner,
    /// SurfaceBevelBuilder: BOTH sides of one strip ran off the faces they landed on, so the strip has no
    /// seam anywhere and is bounded by two trims that meet each other. Each trim collapses the rails on
    /// its own side and mints a point where the surface ends there; two of them on one strip would have
    /// to agree about the rails in between, which is a construction of its own rather than two readings
    /// run one after the other. Args: (Vector3 at).
    SurfaceStripRunsOffBothSides,
    /// SurfaceMitreConsumesTheRail: a chord's two mitres have crossed over on some of its rails - the
    /// setback each neighbour takes grows with the rail's own offset from the edge, so a chord shorter
    /// than the two together is eaten from one side of the profile inward - and what is left is not the
    /// staircase that construction stands on: the chord is gone on EVERY rail, or the rails it is gone
    /// on are a block in the middle of the profile rather than against one face, or the two neighbours'
    /// own lines do not meet in a point for those rails to end on, or the walkback read the corner the
    /// two of them turn at some other way. Named rather than built over a corner nothing stands at.
    /// Args: (Vector3 at).
    SurfaceMitreConsumesTheStrip,
    /// SurfaceStripEnds: one end of a band is BOTH crossed by a mixed corner's run-out and cut back by
    /// the face its own band ran off onto. Two curves want the same end ring - the run-out's section and
    /// the trim's staircase - and closing on either one alone leaves the other's seam cut back under a
    /// ring that never met it. Named rather than built with one of them quietly dropped. Args: (Vector3 at).
    SurfaceRunOffMeetsACrossing,
    /// SurfaceStripTrimmedByTheFaceBeyond: the room read this band as running OFF its landing face onto
    /// the plane beyond, and the strip's own rails do not agree - one of them runs along that plane
    /// instead of crossing it, or the tangent rail is crossed inside the edge's span after all, or a rail
    /// past the first one crossed lands outside the span. The two readings are of the same geometry and
    /// must say the same thing, so the disagreement is named rather than papered over with a clamp.
    /// Args: (Vector3 at).
    SurfaceRunOffReadingDisagrees,
    /// SurfaceBevelBuilder: the PARTIAL run-offs the room read on these plans and the ones this build read
    /// on them are not the same set - a side one of them has and the other has not, or at the other end of
    /// the same edge. The two are the same reading of the same plans, so a difference is a size allowed
    /// for an end nobody then closes: the build skips a side whose strip a blend swallowed or whose rail
    /// the mitres consumed, where the room asks of every side of every edge. Named rather than built with
    /// a strip standing off the part. Args: (Vector3 at).
    SurfaceRunOffReadingsDisagree,
    /// SurfaceMixedCornerRunOut: a piece of the run-out lies on a chord a face of the corner already lies
    /// on, walked the same way round - a fold. Checked station by station so it is named where it is MADE,
    /// rather than later as the causeless `SurfaceResultNotClosed`. Seen on the mitred-pair
    /// sweep: the foot fan doubling back on its own column, and a column crossing the wall plane onto a
    /// bite that already covers it. Args: (Vector3 at).
    SurfaceRunOutLiesOnAFaceAlreadyDrawn,
    /// BevelSurfacePartBuilder and SectionAssembly: the mesh the surface construction built does not close - an edge of it is
    /// used by one face, or by three - so it is refused and the source is republished rather than a part
    /// with a crack or a fold in it being shown. The construction shares every seam vertex as it makes
    /// it, so this is a defect in the construction rather than anything the user chose, and it is caught
    /// on the answer because no single step of the build can see it. Args: (Vector3 at).
    SurfaceResultNotClosed,
    /// SurfaceBevelBuilder: TWO plans turn a corner at one and the same vertex of the part - two closed
    /// rims sharing a vertex, or a vertex four or more picked edges meet at
    /// (`SurfaceTwoCornersAtOneVertex`). A vertex a THIRD chain merely ends at
    /// carries no corner at all: the chain running through it is cut open there before anything is
    /// planned (`SurfaceChainOpensAtTheJunction`), and the three ends close
    /// the ball's octant between them. Two corners are four arms and no octant, so they are named here
    /// rather than mitred into a seam that cannot meet. Args: (Vector3 at).
    SurfaceTwoCornersAtOneVertex,
    /// SurfaceStripNotchedByAFeature.TryRead: a feature the band passed under (its facets held back by
    /// `SurfaceStandingFeatureStretch.TryWithhold`) stands part way ALONG the strip - no crease end
    /// of it lies in either END plane - so there is no strip end for its extension to leave through.
    /// Named at the FOOT. Told from `SurfaceCutMeetsStandingFeature` (the foot as a HOLE in the
    /// landing rim) and `SurfaceCreaseInterruptedByAFeature` (a feature ON the picked crease).
    ///
    /// Not reached by the pass any more: the builder asks `SurfaceStripBittenByAFeature` first and
    /// BUILDS the mid-strip bite (`SurfaceBiteDoesNotRead`). Kept as the notch reader's
    /// precondition for direct callers, who are owed an answer about the feature. Args: (Vector3 at).
    SurfaceStandingFeatureInTheBand,
    /// SurfaceStripBittenByAFeature.TryRead: a feature standing part way ALONG a band (mid strip, see
    /// `SurfaceStandingFeatureInTheBand`) takes a BITE out of the strip, and the bite does not
    /// read as one hole: nothing reaches the landing seam, covered rails reach the far seam instead of a
    /// tip, a ridge leaves off a chord's end, a hole side spans more than one plane, or the walkback's
    /// tangent-line break disagrees with the planes' corners. Told from
    /// `SurfaceStandingFeatureDoesNotExtend`, the same at an END of the band. Args: (Vector3 at).
    SurfaceBiteDoesNotRead,
    /// BevelWorkingMesh.CreateFrom: the source carried one or more FOLDED FLAPS - two faces on the same
    /// three vertices wound opposite ways, which a boolean can leave where two solids met flush. A flap
    /// encloses nothing, but every edge it lies along is used by four faces, so the mesh is not manifold
    /// and the surface construction, which republishes the source's own faces, cannot close over it. Both
    /// faces of each pair are dropped from the working copy. Args: (int pairCount).
    SourceMeshHadFoldedFlaps,
    /// BevelWorkingMesh.CreateFrom: the source carried one or more T-VERTEX SLIVERS - a zero-area
    /// triangle whose three corners are collinear, stitching a vertex that sits in the MIDDLE of a
    /// longer edge into the surface, which is what a boolean leaves where one solid's facet met another
    /// solid's edge part way along. It encloses nothing and the topology reads as closed, but the end
    /// cut of a surface bevel reads the sliver as a face of the part and finds it facing both ways at
    /// once. Each sliver is dropped from the working copy and the face across its long edge is split at
    /// the middle vertex, which puts the T-junction back together. Args: (int sliverCount).
    SourceMeshHadTVertexSlivers,
    /// BevelWorkingMesh.CreateFrom: the source carried one or more THIN SLIVERS - triangles on a mesh edge
    /// shorter than the weld distance (one point a boolean wrote twice, microns apart, where a solid poked
    /// a hair through another's face). Such a sliver floods as its own face region with a normal of
    /// neither side, and a bevel landing on it is refused. The edge is collapsed to its midpoint in the
    /// working copy, taking its triangles with it. Args: (int sliverCount).
    SourceMeshHadThinSlivers,
    /// SurfaceRunStopsShort: the surface construction's answer to `HeldBackAtOneEnd` - one
    /// free end of a picked run stops SQUARE, because the crease it arrives at carries straight on past
    /// that corner in an edge nobody picked (`SurfaceChainEndReading`) or one of the faces being
    /// walked back does (`SurfaceFaceCarriesOn`). The cut is made at the size asked for; what falls
    /// short of what the user expected is how FAR along the part it goes, and the thing to change is the
    /// reading rather than the size. Args: (Vector3 at, int edgeCount).
    SurfaceRunStopsSquareAtOneEnd,
    /// SurfaceRunStopsShort: the same at both free ends of one run, said once.
    /// Args: (Vector3 start, Vector3 end, int edgeCount).
    SurfaceRunStopsSquareAtBothEnds,
    /// SurfaceMixedJunction.TryRead: three selected edges of mixed convexity meet at one vertex and the
    /// profile there is a CHAMFER - a straight cut with no rolling ball behind it. The junction is the
    /// blend between two cuts and one bead swept about the bead's own axis, and a chamfer has no centre
    /// line to swing about (`HasCenterLine`), so the surface
    /// between three straight cuts at that corner is a construction of its own and is not built yet. The
    /// corner is left alone rather than guessed at. Args: (Vector3 at).
    SurfaceMixedJunctionOfChamfers,
    /// SurfaceMixedJunction.TryRead: selected edges of mixed convexity meet at one vertex and the corner
    /// they make is not one the junction reading answers - three ends that do not read as two cuts and a
    /// bead about one axis, a chain still running THROUGH the vertex rather than ending at it, or fewer
    /// than three ends there at all. Told apart from
    /// `SurfaceJunctionOfFourOrMoreEdges`, which is the count alone. Args: (Vector3 at).
    SurfaceMixedJunctionNotRead,
    /// SurfacePlanEdgeReading.TryPlanEdge: the two faces of a picked crease flood into ONE region,
    /// because they join up again round the far side of the part in steps under the flood's tolerance -
    /// an L prism's inner corner fading into a cone blend half way up is the fixture. Told apart from
    /// `FaceRegionHasNoBevelableBoundary`, which is a region that really is smooth all round:
    /// here the crease itself is square, and saying nothing turns would be false. Args:
    /// (Vector3 creaseMidpoint, double flatAngleDegrees).
    SurfaceCreaseFacesAreOneSurface,
    /// BevelCutStage: the BOOLEAN construction's cutter union ran away. The kernel's n-ary fold of the
    /// whole batch came back a solid it would not certify, so the ladder fell to the pairwise fold, and
    /// there the accumulated solid grew far past the material actually in the operands - a self cut whose
    /// cost squares with every step, not a shape. The fold is stopped at the step that crossed the guard
    /// rather than run to the end, because past that point one more 68 face cutter costs half a minute and
    /// there are dozens left. Args: (int accumulatedFaces, int operandFaces, int foldedCount, int total).
    BooleanCutterUnionBlewUp,
    /// BevelWorkingMesh.CreateFrom: the source carried one or more SPLIT CHORDS - a vertex standing part
    /// way along a straight crease, with nothing turning there and the surface flat on both sides of it,
    /// which is what a boolean leaves where its own seam crossed a rim. The two pieces read as two edges
    /// of a picked chain, each with its own tangent line and its own mitre, and two tangent lines a few
    /// ten-millionths from parallel put a crossing wherever the arithmetic lands. The vertex is taken out
    /// of the working copy and the fan each side of it re-triangulated, so the chord is the one edge it
    /// always was. Args: (int splitCount).
    SourceMeshHadSplitChords,
    /// SectionSmoothSeamReading: the SMOOTH surface beside a picked edge - the one the fillet's boundary
    /// runs across rather than stops inside - would not take the cut that boundary asks of it, so the
    /// band has nothing watertight to end on. The refusal is carried by NAME rather than collapsed into
    /// a corner: the second argument is the `SmoothSideRefusal` the cut answered, which is what a
    /// test failure and a bug report both need in order to tell a facet with a hole in it apart from a
    /// boundary that never left one facet. Args: (Vector3 at, string refusal).
    SurfaceSmoothSideNotCut,
    /// BevelConstructionFallThrough: the construction this bevel is set to published nothing for these
    /// picks, so the other one built them. A NOTE and not a complaint - the result is the one the user
    /// asked for - and the operation's own Construction setting is left as they left it.
    /// Args: (string construction).
    BuiltWithTheOtherConstruction,
    /// BevelSurfacePartBuilder (TouchingLoops): this closed loop of the pick shares a vertex with another
    /// closed loop of it - two hole mouths, or a mouth and the face's own rim, meeting at one corner. The
    /// level line refuses such a pick by name (ChainsTouch), so one loop of each touching pair is left
    /// alone and the rest built, rather than the whole pass going to a construction that drops unrelated
    /// loops until the pair is gone. Args: (Vector3 at).
    LoopTouchesAnotherPickedLoop,
    /// BevelSurfacePartBuilder (level line): the room was set by a crease AWAY from the picked face - the far
    /// edge of a neighbour that grows over the crease the band first reaches (RoomSource.GrowingCrease) -
    /// not by the face's own width. Args: (double size, Vector3 at, double largestThatFits), as
    /// `SizeExceedsFaceWidth`, with "at" standing on that crease.
    SizeReachesPastCrease,
}

impl fmt::Display for BevelAdvisoryCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(self, f)
    }
}
