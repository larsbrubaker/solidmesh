//! The blend tool's geometry, and nothing else: how one mixed corner's rim, axis and radius become
//! a mesh. Why there is a tool there at all, which corners get one and what each refusal means live
//! on `BevelMixedBlends`, which is this module's only caller; the record it fills is
//! [`super::BevelMixedBlend`].
//!
//! Split out because the two halves answer different questions and change for different reasons.
//! The half next door reads topology - which edges met, which faces they share, whether the corner
//! leans further than the budget - and every line of it is about a decision. This half is
//! trigonometry in one frame, and the only decision left in it is the wedge bound a station can be
//! refused on ([`bevel_edge_tool_builder`]'s interior-angle bounds).

use polygon_mesh::Mesh;
use vector_math::{cs_math, math_helper, Vector2, Vector3, Vector3Float};

use super::bevel_edge_tool_builder;
use super::BevelEndCap;

/// What [`build`] hands back: the mesh and five readings beside it. The reaches and both handover
/// planes are filled whether or not the mesh could be drawn, so a caller can still read them after
/// a refusal.
#[derive(Debug, Clone)]
pub struct MixedBlendTool {
    /// The tool, or `None` where one station's cross section could not be drawn (its wedge fell
    /// outside the bounds every cross section is refused outside).
    pub mesh: Option<Mesh>,
    /// The widest of the ARC stations' tangent distances - [`super::BevelMixedBlend::cap_face_reach`].
    /// The runout's stations are left out of it on purpose: its consumer sets this against the
    /// setback walked from the CONVEX edge, and past the arc the footprint is on the other wall's
    /// rim rather than across the cap face from that edge. `void_face_reach` and `runout_reach`
    /// are what describe it there.
    pub cap_face_reach: f64,
    /// How far past the void face this tool reaches - [`super::BevelMixedBlend::void_face_reach`].
    pub void_face_reach: f64,
    /// How far along the unselected rim the runout carries - [`super::BevelMixedBlend::runout_reach`].
    pub runout_reach: f64,
    /// The plane the FIRST ring is drawn in, which the convex run on the shared face's side stops on.
    pub handover: BevelEndCap,
    /// The plane the ARC's last ring is drawn in, read the same way off the direction that ring
    /// leaves the axis by. Only ever asked for at a three-edge mixed corner, where the far side of
    /// the rim is a second selected convex edge rather than a rim the cut never reached. Handed back
    /// unconditionally because it costs a cross product to compute and a branch to withhold. It is
    /// the plane of the tool's own last ring only where there is no runout, which is exactly the
    /// case that asks for it.
    pub far_handover: BevelEndCap,
}

/// The tool itself: the convex fillet's own cross section carried round the rim the bead left on
/// the cap face, one sector per chord of the bead's arc - and, where the far end of that rim
/// belongs to nobody, on past it until the ball rolls off the face (see [`stations`]).
///
/// Everything is placed in cylindrical coordinates about the concave edge's rolling-ball axis: a
/// radial distance from the axis, a depth along the edge (measured from the cap face), and the
/// angle round the axis, which the bead's own chords already divide into `segments` equal steps.
/// Each ring sits in the plane through the axis at one of those angles - which is the MITER plane
/// between the two chords either side of it - and the ring is that station's cross section
/// stretched by `1 / cos(step / 2)` radially and not at all in depth, which is `BevelChainSweep`'s
/// miter stretch written in the one frame that makes it a scalar.
///
/// THAT station's, because the wedge each miter plane presents is its own once the edge leans off
/// square (see the remarks on `BevelMixedBlends`); square to the cap face every station presents a
/// right angle and the whole sweep is one congruent profile carried round, which is the closed form
/// `BevelConcaveTests` measures the tool against.
///
/// Two consequences are worth stating, because the volume and the boolean both live on them, and
/// both survive the profile varying from station to station. Every ring's wall points land at
/// radius exactly `r` - the ridge where two of the bead's chords meet - so each wall face of the
/// tool spans two of those ridges and is the bead's own chord face, coplanar with it and not merely
/// near it. And the two points either side of the rim corner - the corner itself and where the ball
/// touches the cap face - both sit at depth zero whichever way the walk was wound, so the strip
/// between them lies IN the cap face. What does move is the strips between ARC points, which are no
/// longer planar quads off square; they are triangulated either way and they are the curved face,
/// so nothing rests on it.
///
/// `run_out_past_the_rim` is whether the far end of the rim is an UNSELECTED edge, which is where
/// the runout belongs. It is false at a three-edge mixed corner, where the far end is a second
/// selected cut that takes `far_handover` instead - see `BevelMixedBlends`.
#[allow(clippy::too_many_arguments)]
pub fn build(
    vertex: Vector3,
    axis_point: Vector3,
    along_edge: Vector3,
    cap_face: Vector3,
    from_shared: Vector3,
    from_void: Vector3,
    turn: f64,
    radius: f64,
    segments: i32,
    run_out_past_the_rim: bool,
) -> MixedBlendTool {
    let mut cap_face_reach = 0.0;

    // The planes the first ring and the ARC's last ring are drawn in: through the axis, spanned by
    // the axis and the direction that ring leaves it by. Unoriented - the chain that stops on one
    // flips it to face its own way out.
    let handover = seam(from_shared, along_edge, axis_point);
    let far_handover = seam(from_void, along_edge, axis_point);

    let step = turn / segments as f64;
    let cosine_of_half_step = (step / 2.0).cos();

    // How far the bead's chords stand from the axis, which is where the tool's wall has to be
    let wall_radius = radius * cosine_of_half_step;

    // Which way round the axis the rim turns, so the rings come out wound with their normals facing
    // out of the tool rather than into it.
    let turning_with_the_edge = from_shared.cross(from_void).dot(along_edge) >= 0.0;

    let stations = stations(
        along_edge,
        cap_face,
        from_shared,
        from_void,
        turn,
        step,
        cosine_of_half_step,
        wall_radius,
        radius,
        segments,
        run_out_past_the_rim,
    );

    let refused = |cap_face_reach, void_face_reach, runout_reach| MixedBlendTool {
        mesh: None,
        cap_face_reach,
        void_face_reach,
        runout_reach,
        handover: handover.clone(),
        far_handover: far_handover.clone(),
    };

    let Some(Stations {
        directions,
        void_face_reach,
        runout_reach,
    }) = stations
    else {
        return refused(cap_face_reach, 0.0, 0.0);
    };

    let point_count = segments + 2;
    let mut mesh = Mesh::new();

    for (station, &direction) in directions.iter().enumerate() {
        let Some((mut profile, tangent_distance)) = try_station_profile(
            direction,
            along_edge,
            cap_face,
            wall_radius,
            radius,
            segments,
        ) else {
            return refused(cap_face_reach, void_face_reach, runout_reach);
        };

        if station as i32 <= segments {
            // Only the ARC's stations, because the reach this publishes is measured from the bead's
            // own wall and past the arc there is no bead - see the remarks on runout_reach for what
            // the runout's footprint is measured from instead.
            cap_face_reach = cs_math::max_f64(cap_face_reach, tangent_distance);
        }

        if turning_with_the_edge {
            profile.reverse();
        }

        for point in profile {
            let out_from_axis = point.x / cosine_of_half_step;

            // The depth is measured along the edge from the CAP FACE rather than from the axis
            // point, so the tool's top faces land exactly on that face however far the edge leans
            // off square to it. Shearing along the edge like this leaves every wall face on its
            // chord plane, which is the other coplanarity the boolean is owed.
            let to_cap_face = -(axis_point - vertex + (direction * out_from_axis)).dot(cap_face)
                / along_edge.dot(cap_face);

            mesh.vertices.push(Vector3Float::from_vector3(
                axis_point + (direction * out_from_axis) + (along_edge * (point.y + to_cap_face)),
            ));
        }
    }

    let ring_count = directions.len() as i32;

    for strip in 0..ring_count - 1 {
        let back = strip * point_count;
        let front = (strip + 1) * point_count;

        for point in 0..point_count {
            let next = (point + 1) % point_count;

            mesh.faces
                .add(back + point, back + next, front + next, &mesh.vertices);
            mesh.faces
                .add(back + point, front + next, front + point, &mesh.vertices);
        }
    }

    // The cross section is star shaped about the rim corner - every ray from it leaves through the
    // arc - so a fan from that one point is a triangulation, no ear clipping needed. Which point
    // that is moved when the profile was reversed, and it is the wall corner either way.
    let fan_from = if turning_with_the_edge {
        point_count - 1
    } else {
        0
    };
    let last_ring = (ring_count - 1) * point_count;

    for i in 1..=point_count - 2 {
        let a = (fan_from + i) % point_count;
        let b = (fan_from + i + 1) % point_count;

        // reversed at the start, because that cap faces back down the sweep
        mesh.faces.add(fan_from, b, a, &mesh.vertices);
        mesh.faces.add(
            last_ring + fan_from,
            last_ring + a,
            last_ring + b,
            &mesh.vertices,
        );
    }

    MixedBlendTool {
        mesh: Some(mesh),
        cap_face_reach,
        void_face_reach,
        runout_reach,
        handover,
        far_handover,
    }
}

/// [`stations`]' result: the ring directions and the two reaches.
struct Stations {
    directions: Vec<Vector3>,
    /// How far past the void face the tool reaches, which is the same number with the runout or
    /// without it: the deepest station is the arc's own last one, where the ring is square to that
    /// face.
    void_face_reach: f64,
    /// How far along the unselected rim the runout carries, or zero where there is none.
    runout_reach: f64,
}

/// Which way out of the axis each ring of the sweep leaves: the bead's own chord ridges over its
/// arc, and then - where the far end of the rim is nobody's selected edge - the RUNOUT past it.
/// `None` where a station's own cross section could not be drawn, which is the same refusal
/// [`build`] makes on one.
///
/// **Why the sweep does not stop where the rim does.** Over the arc the tool is the ball of radius
/// `r` rolling under the cap face along the rim the bead left there, and the rim ends where the
/// bead's quarter arc does - on the plane through the axis square to the void face, where the bead
/// is tangent to that face. At a THREE edge mixed corner that plane is a handover: a second
/// selected cut starts there and carries the surface on, so the tool stops and hands it the plane.
/// At a TWO edge corner there is no second cut, and stopping there caps the torus flat and leaves
/// the original sharp point standing where the bead's rim met the unselected edge. A CAD kernel
/// does not stop there and neither does the ball: the rim has run out, but the ball is still
/// tangent to the cap face and still a fixed distance from the axis, so it keeps CIRCLING the axis -
/// through the air where the bead's cylinder would have gone on, which cuts nothing because
/// nothing is there - until its contact with the cap face runs off that face across the unselected
/// edge. That taper is the fall off a user sees, and it is what the control mesh in
/// `BevelFallOffRegressionTests` has.
///
/// **Where it ends.** The axis is the centre of the ball that fills the crease, so it stands
/// exactly `r` from the void face by construction, and the tool stands `reach` from the axis at its
/// widest. A point `reach` out along the direction `psi` past the arc's end is on the material side
/// of that face while `reach cos(psi) > r`, so the whole ring has cleared it at
/// `psi_max = acos(r / reach)`, which is the runout's whole turn. Square to the cap face `reach` is
/// `r + r / cos(step / 2)` - the wall's own radius plus the ball's, both stretched by the miter -
/// so it is a hair over `2r` and `psi_max` a hair over 60 degrees WHATEVER the radius is, which is
/// the 1:1 case the control was cut at. Derived rather than written down, because the reach moves
/// with the tessellation and with the lean.
///
/// **And it is swept PAST `psi_max`, deliberately, by a whole station.** The count is
/// `ceil(psi_max / step) + 1`: the ceiling to get past the crossing at all, then one more. A cap
/// laid exactly AT `psi_max` touches the material along a single line - the tangential crossing
/// this whole construction refuses to hand a kernel - so landing ON it is the one place the cap
/// may not go, and the ceiling alone can land arbitrarily close to it.
///
/// What that actually sweeps is worth stating, because "one station" understates it at a coarse
/// tessellation. The overshoot is between one and two whole steps, so at 8 segments the tool turns
/// 78.8 degrees past the arc against a `psi_max` of 60.1 - about 1.7 steps - and at 2 segments,
/// where a step is 45 degrees, it turns 135 against a `psi_max` of 61.3. The sweep is bounded below
/// a full turn by its own guard, and at the coarse end that guard is the only thing keeping it
/// modest.
///
/// The extra is free of CONSEQUENCE, not merely of cost. Past `psi_max` every point of the ring is
/// on the void side of the void face, so the overshoot removes nothing from the wall it just left;
/// and it removes nothing from anywhere else either, for exactly the reason the ARC already assumes -
/// the tool is read from the corner alone and takes the neighbourhood of `2r` about the concave
/// axis to be clear of other features (the known limit on `BevelMixedBlends`). The runout stays
/// inside that same `2r`, so it asks for no clearance the arc had not already assumed; what it does
/// do is spend that clearance in a DIFFERENT direction, which is the exposure named there. The cost
/// is a sector of mesh per extra station.
///
/// The extra stations keep the ARC's own angular step, so the chords of the runout line up with the
/// chords of the arc and the strip between the last arc ring and the first runout ring is no
/// different from any other.
#[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
fn stations(
    along_edge: Vector3,
    cap_face: Vector3,
    from_shared: Vector3,
    from_void: Vector3,
    turn: f64,
    step: f64,
    cosine_of_half_step: f64,
    wall_radius: f64,
    radius: f64,
    segments: i32,
    run_out_past_the_rim: bool,
) -> Option<Stations> {
    let mut directions = Vec::with_capacity((segments.max(0) + 1) as usize);

    for station in 0..=segments {
        directions.push(slerp(
            from_shared,
            from_void,
            station as f64 / segments as f64,
        ));
    }

    // How far the tool stands from the axis, read off the ARC's last ring rather than off the
    // closed form, because off square the wedge that ring is drawn in is its own and its widest
    // point may be the ball's contact with the cap face or may be a point of the arc.
    let (at_the_arcs_end, _) = try_station_profile(
        from_void,
        along_edge,
        cap_face,
        wall_radius,
        radius,
        segments,
    )?;

    let mut reach_from_axis = 0.0;

    for point in &at_the_arcs_end {
        reach_from_axis = cs_math::max_f64(reach_from_axis, point.x / cosine_of_half_step);
    }

    // That last ring lies IN the plane through the axis square to the void face, so what it reaches
    // past that face is simply how much further from the axis it stands than the face does. It is a
    // reach the tool has always had - it is where the bead's rim meets the other wall - and it is
    // published now because the runout below is measured against the same wall.
    let void_face_reach = cs_math::max_f64(0.0, reach_from_axis - radius);
    let arc_only = |directions| {
        Some(Stations {
            directions,
            void_face_reach,
            runout_reach: 0.0,
        })
    };

    if !run_out_past_the_rim || !(reach_from_axis > radius) {
        return arc_only(directions);
    }

    let runout_turn = (radius / reach_from_axis).clamp(-1.0, 1.0).acos();

    // The unit direction in the rim's own plane square to the way out at the arc's end, on the
    // side the arc came FROM: the sweep carries on turning away from it. Square rims make this
    // simply -from_shared; a rim that folds by anything other than a right angle needs the
    // component of from_shared along from_void taken out first, or the "rotation" would not be one.
    let back_along_the_arc = from_shared - (from_void * from_shared.dot(from_void));

    if !(back_along_the_arc.length_squared() > 0.0) {
        return arc_only(directions);
    }

    let back_along_the_arc = back_along_the_arc.get_normal();

    // Both quotients are finite and small here (the turn and step come from a bead's arc), so `as`
    // never saturates.
    let runout_stations = (runout_turn / step).ceil() as i32 + 1;

    // Never round so far that the sweep comes back onto its own first ring, which would stop the
    // tool being an embedded solid. Only a crease whose two faces lie nearly back to back can ask
    // for it, and such a corner has no rim worth blending, but the guard is cheap and the
    // alternative is a mesh the kernel has to have an opinion about.
    let runout_stations =
        runout_stations.min((((2.0 * std::f64::consts::PI) - turn) / step).floor() as i32 - 1);

    if runout_stations < 1 {
        return arc_only(directions);
    }

    for station in 1..=runout_stations {
        let psi = step * station as f64;

        directions.push((from_void * psi.cos()) - (back_along_the_arc * psi.sin()));
    }

    // How far along the unselected rim the cut runs, which is the tool's widest point carried round
    // to where it crosses the void face: reach sin(psi_max), and sqrt(reach^2 - r^2) says the same
    // thing without asking for the angle back. About r sqrt 3 at a square rim.
    let runout_reach =
        cs_math::max_f64(0.0, (reach_from_axis * reach_from_axis) - (radius * radius)).sqrt();

    Some(Stations {
        directions,
        void_face_reach,
        runout_reach,
    })
}

/// The plane through the axis that one end ring is drawn in, or nothing where the direction it
/// leaves the axis by runs along the axis itself - which no ring does, and is guarded because the
/// normal is read off a cross product.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn seam(from_axis: Vector3, along_edge: Vector3, axis_point: Vector3) -> BevelEndCap {
    let seam = from_axis.cross(along_edge);

    if !(seam.length_squared() > 0.0) {
        return BevelEndCap::default();
    }

    let normal = seam.get_normal();

    BevelEndCap::new(normal, normal.dot(axis_point))
}

/// One station's cross section, in that station's own miter plane: the wedge between the bead's
/// wall and the cap face, with a ball of radius `radius` inscribed in it. Returns the closed
/// section, walked from the rim corner round the arc, and how far the section reaches from the
/// wall along the cap face (the tangent distance); `None` where the wedge cannot hold a ball.
///
/// The plane is spanned by `direction` - out from the axis - and `along_edge`, which runs down the
/// axis into the material, and those two are square to each other, so the plane's own frame is
/// orthonormal and the wedge below is a true angle rather than a projected one. The WALL is the
/// line at radius `wall_radius` running along the edge; it is the bead's chord, and the tool has to
/// land on it exactly. The CAP FACE is a line through the corner where those meet, and it only runs
/// radially where the edge is square to that face: in general it runs along
/// `-(e . n) rho + (rho . n) e`, which is the one direction in this plane square to the cap face's
/// normal.
///
/// Two returned points are worth naming. The tangent distance is [`bevel_edge_tool_builder`]'s own
/// `r / tan(theta / 2)` read at the wedge THIS plane presents, and the arc runs between the two
/// tangent points the short way round, which for a wedge of `theta` is `180 - theta` of turn.
/// Square to the cap face both collapse to the right angle the construction started life with: the
/// tangent distance is the radius and the arc is a quarter circle from `-90` to `-180` degrees,
/// which is what it used to be written as outright.
///
/// **What is deliberately NOT done here, and it was on the list.** The ball is inscribed IN this
/// plane, so its centre stands a perpendicular `r sqrt(1 - sin^2(phi) sin^2(psi))` from the cap face
/// rather than a perpendicular `r` - the plane cuts that face obliquely, so an in-plane radius is not
/// a perpendicular one. Correcting that, by pushing the centre down to a true perpendicular `r`,
/// was tried and dropped: the circle then no longer TOUCHES the cap face's trace in this plane, so
/// the section cannot close on it, and closing it anyway lifts the arc's first point off `T` and
/// leaves a lip of tool standing above the face it is supposed to be flush with. That trades a
/// smooth second-order error for a glancing crossing, which is the one trade this whole
/// construction refuses to make.
///
/// The error it keeps instead is one-sided, and that is why it is the safe one. The shipped depth
/// runs between `r` and `r / cos(phi)` down the edge - `r` at the station where the plane cuts the
/// cap face square, and exactly `r / cos(phi)` at the stations where it does not cut it obliquely
/// at all - and the ideal is `r / cos(phi)` throughout. So the ball is NEVER deeper than the true
/// rolling ball: wherever the tool is wrong it stops short and leaves a hair of material, and it
/// never reaches past where the ball would have stopped and gouges. The residual is measured and
/// gated on `BevelMixedBlends`; the step it leaves at the seam is measured and pinned in
/// `BevelMixedBlendTests`.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn try_station_profile(
    direction: Vector3,
    along_edge: Vector3,
    cap_face: Vector3,
    wall_radius: f64,
    radius: f64,
    segments: i32,
) -> Option<(Vec<Vector2>, f64)> {
    // The cap face's trace in this plane, in the plane's own (out from the wall, down the edge)
    // frame
    let trace = Vector2::new(-along_edge.dot(cap_face), direction.dot(cap_face));
    let trace_length = trace.length();

    if !(trace_length > 0.0) {
        return None;
    }

    // The wedge the tool is drawn in: from the wall, which runs straight down the edge, round to
    // that trace. A right angle is the square case; the lean opens or closes it by up to the lean
    // itself.
    let wedge = (trace.y / trace_length).clamp(-1.0, 1.0).acos();
    let wedge_degrees = math_helper::radians_to_degrees(wedge);

    if wedge_degrees < bevel_edge_tool_builder::MINIMUM_INTERIOR_ANGLE_DEGREES
        || wedge_degrees > bevel_edge_tool_builder::MAXIMUM_INTERIOR_ANGLE_DEGREES
    {
        // The same bounds a cross section is refused outside anywhere else: below them the two
        // surfaces have folded onto each other and there is no corner to inscribe a ball in.
        return None;
    }

    let tangent_distance = radius / (wedge / 2.0).tan();

    // The ball, a radius out from the wall and level with where it touches it, and the two points
    // it touches - down the wall, and out along the cap face.
    let centre = Vector2::new(radius, tangent_distance);
    let on_cap_face = trace * (tangent_distance / trace_length);

    let start_angle = (on_cap_face.y - centre.y).atan2(on_cap_face.x - centre.x);

    // Where the arc ends is the same angle whatever the wedge: the wall is at the centre's own
    // depth, a radius to its inside, which is exactly half a turn round.
    let mut sweep = std::f64::consts::PI - start_angle;

    while sweep > std::f64::consts::PI {
        sweep -= 2.0 * std::f64::consts::PI;
    }

    while sweep < -std::f64::consts::PI {
        sweep += 2.0 * std::f64::consts::PI;
    }

    // The section is drawn above in this plane's own SQUARE frame - out from the wall, down the
    // edge - and the caller places it in a SHEARED one, where depth is measured from the cap face
    // at the point's own radius rather than from the rim. That shear is what lands every depth-zero
    // point exactly ON the cap face however far the edge leans, and it has to be undone here or the
    // section arrives leaning by the same amount: going out from the rim by u moves the face it is
    // measured from by u (rho . n) / (e . n) along the edge. Square to the face the numerator is
    // zero and the two frames are one, which is why nothing needed saying while the corner had to
    // be square.
    let per_radius = direction.dot(cap_face) / along_edge.dot(cap_face);

    let mut profile = Vec::with_capacity((segments.max(0) + 2) as usize);
    profile.push(Vector2::new(wall_radius, 0.0));

    for i in 0..=segments {
        let angle = start_angle + (sweep * i as f64 / segments as f64);
        let out_from_wall = centre.x + (radius * angle.cos());

        profile.push(Vector2::new(
            wall_radius + out_from_wall,
            centre.y + (radius * angle.sin()) + (per_radius * out_from_wall),
        ));
    }

    Some((profile, tangent_distance))
}

/// The point `fraction` of the way along the great circle from one unit vector to another - equal
/// angles, which is how [`bevel_edge_tool_builder`] samples the bead's arc, so these stations are
/// the bead's own chord ridges.
fn slerp(from: Vector3, to: Vector3, fraction: f64) -> Vector3 {
    let angle = from.dot(to).clamp(-1.0, 1.0).acos();

    if angle < 1e-9 {
        return from;
    }

    ((from * ((1.0 - fraction) * angle).sin()) + (to * (fraction * angle).sin())) / angle.sin()
}
