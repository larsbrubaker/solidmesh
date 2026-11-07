//! The section at a JUNCTION, where two chain edges share one FLAT side at DIFFERENT wedges: the
//! section itself ([`JunctionSection`]), which piece of its neighbourhood one directed edge belongs
//! to ([`JunctionPiece`]) and one chain edge's canal there ([`Canal`]).
//!
//! Both canals have the same radius and ride the shared flat side, so their axes meet and the
//! mitre is still ONE plane cutting both in ONE ellipse. What unequal wedges break is only the
//! wall end: each canal's tangency with its own wall is a different point of that ellipse. A
//! junction section carries BOTH ends on one row of samples and hands the stretch between them to
//! a patch of wall standing on the fold.

use std::rc::Rc;

use vector_math::Vector3;

use super::bevel_section::BevelSection;

/// Which piece of the junction's neighbourhood one directed edge belongs to.
/// The junction closes BY IDENTITY - every directed edge it lists is used exactly twice, once
/// each way - so the piece is what the wiring reads to know who must lay that edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum JunctionPiece {
    /// The ARRIVING strip's junction rung, the ellipse from the flat-side meet to P1.
    #[default]
    ArrivingStrip = 0,
    /// The LEAVING strip's junction rung, the same ellipse carried on past P1 to P2.
    LeavingStrip,
    /// The patch of wall standing on the fold between the two wall ends.
    Patch,
    /// The NEAR wall's rim after the fillet - the wall that holds the fold - the segment between
    /// the fold point F1 and that canal's own wall end.
    NearWallRim,
    /// The FAR wall's rim after the fillet - the tessellated facet's - the segment between its own
    /// wall end and F1.
    FarWallRim,
}

/// One ellipse, used twice, plus the patch of wall that bridges the two
/// strips' different wall ends. Points are indices into the run's `SectionPool`.
#[derive(Debug, Clone, PartialEq)]
pub struct JunctionSection {
    /// The mesh vertex the junction stands at.
    pub chain_vertex: i32,
    /// Where that vertex stands.
    pub station: Vector3,
    /// The point the mitre plane passes through - the two canal axes' meet.
    pub through: Vector3,
    /// The mitre plane's unit normal, the two edge directions summed.
    pub normal: Vector3,
    /// The one row of pool indices along the mitre ellipse: index 0 the SHARED FLAT side's
    /// offset-meet, index [`JunctionSection::arriving_wall_end`] the arriving canal's wall
    /// tangency P1, and the last the leaving canal's wall tangency P2.
    pub samples: Vec<i32>,
    /// Per entry of [`JunctionSection::samples`], the MESH FACE that sample was carried out onto,
    /// or -1 where it stands on the canal as drawn. Only the FAR stretch - whose ideal wall is the
    /// tessellated one - is ever carried.
    pub carried_onto: Vec<i32>,
    /// The index in [`JunctionSection::samples`] of P1 - always the segment count.
    pub arriving_wall_end: i32,
    /// The index in [`JunctionSection::samples`] of the LEAVING canal's wall end.
    pub leaving_wall_end: i32,
    /// The pool index of F1, where the arriving canal's tangency line meets the fold.
    pub fold_point: i32,
    /// The outward unit normal of the wall the ARRIVING canal ends its contact on - the wall P1
    /// stands in.
    pub arriving_wall: Vector3,
    /// The LEAVING canal's own wall's, the one P2 stands in.
    pub leaving_wall: Vector3,
    /// Whether the SHARED flat side is on the left of the walk.
    pub flat_on_the_left: bool,
    /// The arriving edge's own setback on the shared face, `r tan(wedge / 2)`.
    pub setback_arriving: f64,
    /// The leaving edge's, which at a junction is a DIFFERENT number.
    pub setback_leaving: f64,
    /// The section the strip ARRIVING at the junction ends on - samples `[meet..P1]`. Shared with
    /// the run that holds it, so both read the one section.
    pub arriving_section: Rc<BevelSection>,
    /// The section the strip LEAVING it starts on - samples `[meet..P2]`.
    pub leaving_section: Rc<BevelSection>,
}

/// One chain edge's canal at the junction, read in the sweep
/// parameter the junction shares - 0 at the SHARED FLAT side, 1 at that edge's OWN wall, and
/// carried on past 1 for the stretch of ellipse no wall contact reaches.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Canal {
    /// The unit direction the edge runs.
    pub along: Vector3,
    /// The SHARED flat side's outward normal - the sweep's v = 0.
    pub flat: Vector3,
    /// This edge's OWN wall's outward normal - the sweep's v = 1.
    pub wall: Vector3,
    /// The radius the canal stands at.
    pub radius: f64,
    /// The lattice the canal's absolute readings are taken against.
    pub lattice: f64,
    /// The angle between the two normals.
    pub wedge: f64,
    /// Where the canal's axis stands abreast of the vertex.
    pub axis: Vector3,
    /// The in-sweep direction square to [`Canal::along`], for reading an angle back.
    pub across: Vector3,
    /// How steeply the edge runs into the mitre plane, set once the plane is known.
    pub lean: f64,
}
