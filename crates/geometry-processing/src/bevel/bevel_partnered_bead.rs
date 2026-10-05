//! A straight open concave run's bead built from the balls that roll along it and its PARTNERS'
//! tubes, rather than swept and clipped (`Bevel/BevelPartneredBead.cs`, the RUN file of the C#
//! `static partial class BevelPartneredBead`): what the run is ([`BevelPartneredBeadRun`]), what
//! each of its ends does ([`BevelPartneredBeadEnd`]) and one place along it a section is taken
//! ([`BevelPartneredBeadStation`]). Its functions - reading the run, its partners, its stations
//! and sections (`BevelPartneredBeadPartners.cs`, `...Stations.cs`, `...Section.cs`,
//! `...Tolerances.cs`) - land in this module with the core's packages.

use std::rc::Rc;

use vector_math::{Vector2, Vector3};

use super::{BevelFaceRunOut, BevelPartnerTube, BevelRunOutBound};
use crate::topology::ChainEdge;

/// C# `BevelPartneredBead.Run` (private): everything about the run that is the same at every one
/// of its stations. Built by one read of the run and owned by that build.
#[derive(Debug)]
pub struct BevelPartneredBeadRun {
    /// A point of the rolling ball's centre line, the origin `along` is measured from.
    pub axis_point: Vector3,
    /// The centre line's unit direction, along the run.
    pub direction: Vector3,
    /// The station frame's x axis.
    pub x_axis: Vector3,
    /// The station frame's y axis.
    pub y_axis: Vector3,
    /// The rolling ball's radius.
    pub size: f64,
    /// How many points one station's polygon has (`segments + 6`).
    pub point_count: i32,
    /// How many CHORDS the arc is drawn with - one fewer than its vertices, and the number a
    /// partner's tube has to be given so its prism has the same facets this bead's own arc does.
    pub arc_facets: i32,
    /// The outward normal of the face the bead rides.
    pub riding_normal: Vector3,
    /// The outward normal of the other face of the crease.
    pub second_normal: Vector3,
    /// The run's chain edges, in run order.
    pub edges: Vec<ChainEdge>,
    /// How far past its crease each end may ride, read off the part.
    pub run_out: Rc<BevelFaceRunOut>,
    /// The arc, slots 0 to segments + 2, from the riding face's tangent point to the second
    /// face's. The same at every station, which is what keeps the swept surface untwisted.
    pub arc: Vec<Vector2>,
    /// The crease line, which is where a station inside the crease closes its polygon.
    pub crease: Vector2,
    /// Where along the run the SECOND face - the one the arc's far end is drawn against -
    /// actually exists, measured by walking its own tangent line out of the crease's middle. A
    /// station outside `second_face_from..second_face_to` takes the past-the-end layout even
    /// though it is inside the crease.
    pub second_face_from: f64,
    /// The far end of that span.
    pub second_face_to: f64,
    /// The riding face plane's trace in the station frame: a point on it ...
    pub riding_tangent_point: Vector2,
    /// ... and its unit normal.
    pub riding_normal2: Vector2,
    /// How far the two flat sides are carried INSIDE the part.
    pub clearance: f64,
    /// One clearance straight into the riding face, in the station frame.
    pub into_the_riding_face: Vector2,
    /// And one into the face that ends, which is where the section's other flat side is buried.
    pub into_the_second_face: Vector2,
    /// And one along the CORNER between them, where the section's own closing point goes - the
    /// point a full clearance inside BOTH planes, `p - (n0 + n1) c / (1 + n0 . n1)`, not the sum of
    /// the other two.
    pub into_the_corner: Vector2,
    /// Whether every polygon is walked the other way round, so the sewn faces come out facing
    /// outward.
    pub reverse: bool,
}

/// C# `BevelPartneredBead.End` (private): what one end of the run does - which face it rides on
/// past the crease, how far, and against whom. A run has two, held as `[start, end]`;
/// [`BevelPartneredBeadStation::past_end`] names one by its index there.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct BevelPartneredBeadEnd {
    /// Whether this is the run's start end.
    pub at_start: bool,
    /// How far past the crease's end the run carries on.
    pub distance: f64,
    /// What stopped it there.
    pub bound: BevelRunOutBound,
    /// The end vertex's position.
    pub vertex: Vector3,
    /// The mesh vertex the run ends at, which is what partnership is keyed on - the INDEX and not
    /// the position, because a boolean's mesh carries vertices a micron apart where its operands
    /// met.
    pub vertex_index: i32,
    /// The runs sharing this end vertex, as tubes. Never empty once built - an end with none
    /// refuses the whole run.
    pub partners: Vec<BevelPartnerTube>,
    /// Whether the riding face ENDS over a convex edge here, so `face_end_point` /
    /// `face_end_normal` are set.
    pub has_face_end: bool,
    /// A point of the convex edge the riding face ends at.
    pub face_end_point: Vector3,
    /// The unit normal of the plane that edge stands in, pointing at the material.
    pub face_end_normal: Vector3,
    /// Whether the run-out ran out onto a WALL, so the three `wall_*` fields are set.
    pub has_wall: bool,
    /// A point of the concave edge the run-out stopped at.
    pub wall_point: Vector3,
    /// The way that edge runs.
    pub wall_direction: Vector3,
    /// The outward normal of the face across it.
    pub wall_normal: Vector3,
    /// Whether the plane of the face ACROSS the convex edge this run's second face ran out over
    /// was read, so `across_point` / `across_normal` are set.
    pub has_across_plane: bool,
    /// A point of that edge.
    pub across_point: Vector3,
    /// The unit normal of the face beyond it, pointed at the side the riding face's own tangent
    /// line is on.
    pub across_normal: Vector3,
    /// Whether this end really is bounded by that plane rather than by a partner or a wall.
    pub ends_on_the_across_plane: bool,
}

/// C# `BevelPartneredBead.Station` (a private readonly struct): one place along the run a section
/// is taken, and how that section closes.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct BevelPartneredBeadStation {
    /// Where along the run, from [`BevelPartneredBeadRun::axis_point`].
    pub along: f64,
    /// Whether this station stands past the crease's own end vertex, rather than merely past the
    /// end of the SECOND face's span inside it.
    pub past_the_crease: bool,
    /// The end whose partner closes this station, as its index in the run's `[start, end]` pair
    /// (C# holds the `End` itself), or `None` for a station inside the crease, which closes on the
    /// crease line.
    pub past_end: Option<usize>,
}
