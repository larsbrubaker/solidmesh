//! The rolling ball's contact ARC at one chain vertex (`Bevel/Sections/BevelSection.cs`): the
//! section itself ([`BevelSection`]), what kind of curve its profile is ([`SectionKind`]) and how
//! each of its ends meets the face beyond ([`SectionSide`]). Its constructor lands with the
//! core's section packages as an `impl` block.
//!
//! One per vertex, not one per strip end: a section is computed once at the vertex and held by
//! the two strips meeting there by reference (`SectionRun`), so the seam between two strips is
//! one row of points read twice and there is nothing at a joint left to match by position.
//!
//! The centre is a SEGMENT. Where one side slides along a crease that is not square to the chain,
//! the arc skews: sample `k` is a radius from `lerp(centre_from, centre_to, k / segments)`, and
//! because every profile direction is square to [`BevelSection::tangent`] - the direction the
//! centre segment runs along - every sample is exactly a radius from the segment.

use vector_math::Vector3;

/// C# `SectionSide`: how one end of a strip meets the face beyond it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum SectionSide {
    /// The boundary point rides an existing crease of the part that leaves the chain vertex on
    /// this side, so the faces either side of that crease keep their shared edge and only its
    /// end moves.
    #[default]
    Slide = 0,
    /// Nothing leaves the chain vertex on this side, so the boundary point is the setback taken
    /// square off the chain into the face - or, where that reach passes the facet it started in,
    /// the setback WALKED along the surface ([`BevelSection::left_walk_face`]).
    Offset,
}

/// C# `SectionKind`: what kind of curve a section's profile is, which says whether its centre
/// may be read as a centre at all.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum SectionKind {
    /// The rolling ball's arc between two sides, every sample a radius from the centre segment.
    #[default]
    Smooth = 0,
    /// The MITRE at a sharp corner (`SectionCorner`): one edge's canal cut by the corner's
    /// bisector plane, an ellipse-like curve. Its centre is a point every sample stands OUTWARD
    /// of, but no sample stands a radius from it.
    Corner,
    /// The END of an open run (`SectionEnd`): the one chain edge's canal cut by the plane of the
    /// FLAT face the run dies on. Read exactly as [`SectionKind::Corner`] is.
    End,
    /// The JUNCTION (`SectionJunction`): the same mitre ellipse as [`SectionKind::Corner`] at two
    /// DIFFERENT wedges, so the two strips meeting there end on two different points of it and a
    /// patch of wall bridges the stretch between. The two strips' profiles are stretches of ONE
    /// shared row of pool points, the arriving strip's being the opening stretch of the leaving
    /// strip's.
    Junction,
}

/// C# `BevelSection`: where the ball stands at one chain vertex, which way the chain runs through
/// it, the two points it hands to the faces on either side, and the row of points between them.
/// Points are indices into the run's `SectionPool`.
#[derive(Debug, Clone, PartialEq)]
pub struct BevelSection {
    /// What kind of curve the profile is. A [`SectionKind::Corner`] section's centre is a point to
    /// read OUTWARD from and nothing more - "a radius from the centre segment" holds for
    /// [`SectionKind::Smooth`] sections only.
    pub kind: SectionKind,
    /// The mesh vertex of the chain this section stands at.
    pub chain_vertex: i32,
    /// Where that vertex stands - the section's origin, on the part's own edge.
    pub station: Vector3,
    /// The unit direction the chain runs through the station, the bisector of the two edges
    /// meeting there: the axis the section's plane is square to and the centre segment runs
    /// along.
    pub tangent: Vector3,
    /// The pool index of the point this section hands the face on the LEFT of the walk (seen
    /// from outside the solid - `ChainEdge::left_face`'s side).
    pub left_boundary: i32,
    /// The pool index of the point it hands the face on the right.
    pub right_boundary: i32,
    /// The row of pool indices from the left boundary to the right one INCLUSIVE - segments + 1
    /// of them for a fillet, just the two boundaries for a chamfer - so a strip lofted between
    /// two sections shares its rails with the faces the boundaries were handed to.
    pub profile: Vec<i32>,
    /// The centre the LEFT boundary and profile sample 0 stand a radius from.
    pub centre_from: Vector3,
    /// The centre the RIGHT boundary and the last profile sample stand a radius from.
    pub centre_to: Vector3,
    /// How the left boundary was found.
    pub left_kind: SectionSide,
    /// How the right boundary was found.
    pub right_kind: SectionSide,
    /// The mesh edge the left boundary slides down, or -1 where that side is an
    /// [`SectionSide::Offset`].
    pub left_slide_edge: i32,
    /// The same for the right boundary.
    pub right_slide_edge: i32,
    /// The mesh face the LEFT boundary point lies in when that side's reach was laid ALONG the
    /// surface, or -1 where it was not walked. A flag BESIDE [`BevelSection::left_kind`] rather
    /// than a third [`SectionSide`]: everything that asks "is this side an offset?" means "does
    /// nothing leave the vertex here", which a walked side still answers yes to.
    pub left_walk_face: i32,
    /// The same for the right boundary.
    pub right_walk_face: i32,
    /// The mesh edge the LEFT boundary point lies ON where that side's walk ENDED on one, or -1
    /// where it ended strictly inside [`BevelSection::left_walk_face`]. Carried so the smooth
    /// side's boundary can drop the join's duplicate crossing of that fold BY IDENTITY.
    pub left_walk_edge: i32,
    /// The same for the right boundary.
    pub right_walk_edge: i32,
    /// Where along [`BevelSection::left_walk_edge`] that point stands, 0 at vertex 0 and 1 at 1.
    pub left_walk_parameter: f64,
    /// The same for the right boundary.
    pub right_walk_parameter: f64,
    /// The mesh VERTEX the LEFT boundary point stands on where that side's walk ended on a
    /// corner, or -1. The edge can name only one of the folds meeting there; the vertex is the
    /// reading that stands. Recorded by the walk, never re-derived.
    pub left_walk_vertex: i32,
    /// The same for the right boundary.
    pub right_walk_vertex: i32,
    /// Whether the LEFT side's reach crossed a FOLD - an edge whose two faces are not coplanar -
    /// on its way along the surface. This, and not "did the walk end outside the station's own
    /// fan", is what says a side is SMOOTH: a flat face a boolean tessellated into thin triangles
    /// hands a reach several coplanar diagonals to cross while bending nowhere.
    pub left_walk_crossed_a_fold: bool,
    /// The same for the right boundary.
    pub right_walk_crossed_a_fold: bool,
    /// Whether the LEFT boundary point was placed ON the join between its two unsplit
    /// neighbours' boundary points, which is what a SPLIT station's side does instead of walking.
    /// The one fact the smooth-side cut cannot re-derive: such a point is collinear with those
    /// neighbours by construction.
    pub left_boundary_is_join_placed: bool,
    /// The same for the right boundary.
    pub right_boundary_is_join_placed: bool,
    /// The direction the left side's surface walk was travelling when it ended.
    pub left_walk_direction: Vector3,
    /// The same for the right boundary.
    pub right_walk_direction: Vector3,
    /// How far back into each face the section reaches measured SQUARE to the chain,
    /// `radius * tan(phi / 2)` for the angle phi between the two side normals. A boundary that
    /// slides stands farther from the station than this, by `1 / cos(beta)`.
    pub setback: f64,
}
