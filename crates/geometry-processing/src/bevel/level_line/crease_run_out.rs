//! A fillet's RUN-OUT along a crease nobody picked ([`CreaseRunOut`]). Ports the type of
//! `Bevel/LevelLine/CreaseRunOut.cs`; its finder and closed-form readings land with later packages.

use vector_math::Vector3;

/// C# `CreaseRunOut`: a fillet's RUN-OUT along a crease nobody picked. At a pick corner where a
/// convex chain meets a concave one, the convex chain's round does not stop on the unpicked concave
/// crease its side face ends on - the concave chain's void ball keeps rolling round the round and
/// dies tangentially on that crease, a way along it (4.5846 radii on the Top Center Face nose).
/// This reads where, in closed form.
///
/// **The corner.** A picked convex edge and a picked concave edge share a corner vertex and a
/// SHARED face (the nose's upper face). The convex edge's other face is its SIDE face (the lower
/// face); the concave edge's other face is its WALL (the notch slope). The run-out exists where an
/// unpicked concave feature crease leaves the corner between the side face and the wall. A picked
/// crease, a crease between other faces, or no opposite-sense chain at the corner is not this case.
///
/// **The law** (measured against the Onshape control in `BevelTopCenterFaceCanalTests`). The void
/// ball of radius r stays r OUTSIDE the wall and 2r from the round's own axis (it touches the
/// round's tube from outside), so its centre runs an ellipse - the dual spine E. Turned from the
/// shared face's normal to the side face's, E is the corner canal (112.62 degrees on the nose);
/// turned further, the ball's contact with the wall leaves the crease and comes back to it, and the
/// first return is where the run-out ends (158.5036 degrees). There the centre stands exactly r
/// from the crease's line: the ball touches the sharp edge at a point, which is why the blend fades
/// out instead of cutting a step. Nothing here depends on the frozen Surface namespace;
/// `SurfaceMixedCornerDualFrame` is the same spine written for it.
///
/// `BandGroupFields` reads it for the run-out field (gated off by default). There is no profile
/// filter - a chamfer takes a run-out too. On the nose both faces of the wedge land in one group,
/// so one corner hole spans both ridge ends and the apex strip.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CreaseRunOut {
    /// Where the round's axis meets the wall's plane: the origin the ball's centre slides from.
    pub rail_centre: Vector3,
    /// The unit direction of the convex round's axis.
    pub axis_direction: Vector3,
    /// The wall's unit normal, out of the wall.
    pub out_of_the_wall: Vector3,
    /// `axis_direction . out_of_the_wall`: how fast sliding along the axis leaves the wall.
    pub along_the_axis_out_of_the_wall: f64,
    /// The unit direction square to the axis toward the shared face - the canal's turn zero.
    pub towards_the_shared_face: Vector3,
    /// The unit direction square to the axis and to
    /// [`CreaseRunOut::towards_the_shared_face`], toward the side face - a quarter turn on.
    pub towards_the_side_square: Vector3,
    /// The pick corner's vertex.
    pub corner: i32,
    /// The unpicked crease's far vertex.
    pub crease_end: i32,
    /// Where the corner stands.
    pub corner_point: Vector3,
    /// The unit direction along the crease, away from the corner.
    pub crease_direction: Vector3,
    /// The crease's length as one mesh edge - past it the run-out does not fit.
    pub crease_length: f64,
    /// The fillet radius the run-out was read at.
    pub size: f64,
    /// The turn the corner canal ends at: from the shared face's normal to the side face's.
    pub canal_turn_radians: f64,
    /// The turn the run-out ends at: the wall contact's first return onto the crease past the
    /// canal.
    pub end_turn_radians: f64,
    /// How far along the crease the run-out ends (P*).
    pub reach: f64,
    /// The wall contact T(theta) sampled from the canal's end to the run-out's.
    pub wall_contact: Vec<Vector3>,
    /// The contact on the round's tube, sampled at the same turns as
    /// [`CreaseRunOut::wall_contact`].
    pub tube_contact: Vec<Vector3>,
}
