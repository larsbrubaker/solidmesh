//! A straight open concave run's bead built from the balls that roll along it and its PARTNERS'
//! tubes, rather than swept and clipped - the RUN file of the partnered bead: what the run is
//! ([`BevelPartneredBeadRun`]), what each of its ends does ([`BevelPartneredBeadEnd`]) and one
//! place along it a section is taken ([`BevelPartneredBeadStation`]), reading the run and building
//! the bead ([`try_build`]). The partial class's other files are the sibling modules
//! `bevel_partnered_bead_partners`, `..._stations`, `..._section` and `..._tolerances`.

use std::rc::Rc;

use vector_math::{Vector2, Vector3};

use vector_math::cs_math::min_f64;
use vector_math::math_helper::degrees_to_radians;

use polygon_mesh::csg::boolean_processing::{classify_boolean_operand, BooleanOperandVerdict};
use polygon_mesh::Mesh;
use vector_math::Matrix4X4;

use super::bevel_partnered_bead_partners::try_read_partners;
use super::bevel_partnered_bead_section::{close, sew, signed_area, try_station};
use super::bevel_partnered_bead_stations::stations;
use super::bevel_partnered_bead_tolerances::{
    second_face_stops_short_by, ACROSS_PLANE_TIP_IN_SIZES, CLEARANCE_IN_SIZES, DEEPEST_BURIAL,
    RUNS_OUT_AT_LEAST, RUN_OUT_LIMIT_IN_SIZES, SAME_FACE_WITHIN_DEGREES,
};
use super::bevel_run_partners::try_ball_centre_line;
use super::{BevelFaceRunOut, BevelPartnerTube, BevelRunOutBound, BevelSweptChain};
use crate::topology::{ChainEdge, EdgeConvexity};

/// Everything about the run that is the same at every one of its stations. Built by one read of the
/// run and owned by that build.
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

/// What one end of the run does - which face it rides on past the crease, how far, and against
/// whom. A run has two, held as `[start, end]`; [`BevelPartneredBeadStation::past_end`] names one
/// by its index there.
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

/// One place along the run a section is taken, and how that section closes.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct BevelPartneredBeadStation {
    /// Where along the run, from [`BevelPartneredBeadRun::axis_point`].
    pub along: f64,
    /// Whether this station stands past the crease's own end vertex, rather than merely past the
    /// end of the SECOND face's span inside it.
    pub past_the_crease: bool,
    /// The end whose partner closes this station, as its index in the run's `[start, end]` pair (an
    /// index, so the station does not borrow the run), or `None` for a station inside the crease,
    /// which closes on the crease line.
    pub past_end: Option<usize>,
}

impl BevelPartneredBeadRun {
    /// How far along the run `point` stands, from the axis point.
    pub fn along(&self, point: Vector3) -> f64 {
        (point - self.axis_point).dot(self.direction)
    }

    /// The ball centre `along` the run.
    pub fn origin(&self, along: f64) -> Vector3 {
        self.axis_point + (self.direction * along)
    }

    /// A station-frame point at `along`, in the part.
    pub fn in_part(&self, along: f64, point: Vector2) -> Vector3 {
        self.origin(along) + (self.x_axis * point.x) + (self.y_axis * point.y)
    }
}

impl BevelPartneredBeadStation {
    /// A station at `along`, with the end that closes it and whether it is past the crease.
    pub fn new(along: f64, past_end: Option<usize>, past_the_crease: bool) -> Self {
        BevelPartneredBeadStation {
            along,
            past_the_crease,
            past_end,
        }
    }
}

/// The run, and its `[start, end]` pair, read off the part - or the refusal that says why this run
/// has no partnered bead.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn try_read_run(
    chain: &BevelSweptChain,
    size: f64,
    segments: i32,
    clearance: f64,
    run_out: Option<Rc<BevelFaceRunOut>>,
) -> Result<(BevelPartneredBeadRun, [BevelPartneredBeadEnd; 2]), String> {
    let Some(run_out) = run_out else {
        return Err(
            "this run is not an open concave run with a size and a part to read".to_string(),
        );
    };

    if chain.edges.is_empty() || chain.is_closed || !(size > 0.0) || segments < 1 {
        return Err(
            "this run is not an open concave run with a size and a part to read".to_string(),
        );
    }

    let first = chain.edges[0];
    let last = chain.edges[chain.edges.len() - 1];

    if chain
        .edges
        .iter()
        .any(|edge| edge.edge.convexity != EdgeConvexity::Concave)
    {
        return Err("this run is not all concave, so it has no bead to fill".to_string());
    }

    let Some((axis_point, direction)) = try_ball_centre_line(chain, size) else {
        return Err(
            "this run turns, so its rolling ball has no one straight centre line to build on"
                .to_string(),
        );
    };

    let same = degrees_to_radians(SAME_FACE_WITHIN_DEGREES).cos();

    for edge in &chain.edges {
        if edge.left_normal().dot(first.left_normal()) < same
            || edge.right_normal().dot(first.right_normal()) < same
        {
            return Err(
                "this run's faces turn along it, so one section frame does not fit the whole run"
                    .to_string(),
            );
        }
    }

    let bisector = first.left_normal() + first.right_normal();

    if !(bisector.length() > 1e-9) {
        return Err(
            "this run's two faces fold back on each other, so there is no wedge to fill"
                .to_string(),
        );
    }

    let y_axis = -bisector.get_normal();
    let x_axis = y_axis.cross(direction).get_normal();

    let mut run = BevelPartneredBeadRun {
        axis_point,
        direction,
        x_axis,
        y_axis,
        size,
        point_count: segments + 6,
        arc_facets: segments + 2,
        riding_normal: Vector3::default(),
        second_normal: Vector3::default(),
        edges: chain.edges.clone(),
        run_out,
        arc: Vec::new(),
        crease: Vector2::default(),
        second_face_from: 0.0,
        second_face_to: 0.0,
        riding_tangent_point: Vector2::default(),
        riding_normal2: Vector2::default(),
        clearance,
        into_the_riding_face: Vector2::default(),
        into_the_second_face: Vector2::default(),
        into_the_corner: Vector2::default(),
        reverse: false,
    };

    let mut start = BevelPartneredBeadEnd {
        at_start: true,
        vertex: first.from(),
        vertex_index: first.from_vertex(),
        ..Default::default()
    };
    let mut end = BevelPartneredBeadEnd {
        at_start: false,
        vertex: last.to(),
        vertex_index: last.to_vertex(),
        ..Default::default()
    };

    let start_rides_left = try_ride_face(&run, &first, &mut start)?;
    let end_rides_left = try_ride_face(&run, &last, &mut end)?;

    if start_rides_left != end_rides_left {
        return Err("this run's two ends run out onto different faces, so its section has no one face to be drawn from".to_string());
    }

    run.riding_normal = if start_rides_left {
        first.left_normal()
    } else {
        first.right_normal()
    };
    run.second_normal = if start_rides_left {
        first.right_normal()
    } else {
        first.left_normal()
    };

    measure_the_second_faces_span(&mut run, &first, start_rides_left, &mut start, &mut end);

    try_build_arc(&mut run, first.from(), segments)?;

    Ok((run, [start, end]))
}

/// Which of `edge`'s two faces this end rides past the crease on (`true` for the left), filling in
/// how far and what stops it; refused where nothing runs out here.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn try_ride_face(
    run: &BevelPartneredBeadRun,
    edge: &ChainEdge,
    end: &mut BevelPartneredBeadEnd,
) -> Result<bool, String> {
    let outward = if end.at_start {
        -run.direction
    } else {
        run.direction
    };
    let ball_centre = run.origin(run.along(end.vertex));
    let limit = RUN_OUT_LIMIT_IN_SIZES * run.size;

    let left = run.run_out.try_measure(
        edge.left_face(),
        ball_centre - (edge.left_normal() * run.size),
        outward,
        limit,
    );
    let right = run.run_out.try_measure(
        edge.right_face(),
        ball_centre - (edge.right_normal() * run.size),
        outward,
        limit,
    );

    if left.is_none() && right.is_none() {
        return Err("neither of this end's faces has the bead's tangent line on it, so there is nothing to measure a run-out along".to_string());
    }

    let measured_left = left.is_some();
    let measured_right = right.is_some();
    let left = left.unwrap_or_default();
    let right = right.unwrap_or_default();

    let rides_left = measured_left && (!measured_right || left.distance >= right.distance);

    let second_distance = if rides_left {
        right.distance
    } else {
        left.distance
    };
    let measured_second = if rides_left {
        measured_right
    } else {
        measured_left
    };
    let riding = if rides_left { left } else { right };

    if riding.distance <= RUNS_OUT_AT_LEAST * run.size {
        return Err("neither of this end's faces carries the bead past the crease, so nothing runs out here and the bead is the one the sweep has always built".to_string());
    }

    if measured_second && second_distance > RUNS_OUT_AT_LEAST * run.size {
        return Err(format!(
            "the face across this run's crease carries on {} mm past this end rather than stopping there, so the crease does not run out here and the bead is the one the sweep has always built",
            format_three_places(second_distance)
        ));
    }

    end.distance = riding.distance;
    end.bound = riding.bound;

    let riding_normal = if rides_left {
        edge.left_normal()
    } else {
        edge.right_normal()
    };
    let bounding_edge = riding.bounding_edge;

    if end.bound == BevelRunOutBound::ConcaveEdge {
        if let (Some((wall_from, wall_to)), Some((across_a, across_b))) = (
            run.run_out.try_edge_ends(bounding_edge),
            run.run_out.try_edge_face_normals(bounding_edge),
        ) {
            end.has_wall = true;
            end.wall_point = wall_from;
            end.wall_direction = (wall_to - wall_from).get_normal();
            end.wall_normal = if across_a.dot(riding_normal) > across_b.dot(riding_normal) {
                across_b
            } else {
                across_a
            };
        }
    }

    if end.bound == BevelRunOutBound::ConvexEdge {
        if let Some((edge_from, edge_to)) = run.run_out.try_edge_ends(bounding_edge) {
            let mut normal = (edge_to - edge_from).get_normal().cross(riding_normal);

            if normal.length() > 1e-9 {
                normal = normal.get_normal();

                if (edge.from() - edge_from).dot(normal) < 0.0 {
                    normal = -normal;
                }

                end.has_face_end = true;
                end.face_end_point = edge_from;
                end.face_end_normal = normal;
            }
        }
    }

    Ok(rides_left)
}

/// Up to three decimal places, trailing zeros dropped.
pub(crate) fn format_three_places(value: f64) -> String {
    let text = format!("{value:.3}");
    let text = text.trim_end_matches('0').trim_end_matches('.');
    if text == "-0" {
        "0".to_string()
    } else {
        text.to_string()
    }
}

/// Where along the run the second face really exists, walked out of the crease's middle along its
/// own tangent line; an end whose second face runs out over a convex edge also reads that edge's
/// across plane.
pub fn measure_the_second_faces_span(
    run: &mut BevelPartneredBeadRun,
    first: &ChainEdge,
    rides_left: bool,
    start: &mut BevelPartneredBeadEnd,
    end: &mut BevelPartneredBeadEnd,
) {
    let start_along = run.along(start.vertex);
    let end_along = run.along(end.vertex);

    run.second_face_from = start_along;
    run.second_face_to = end_along;

    let second_face = if rides_left {
        first.right_face()
    } else {
        first.left_face()
    };
    let middle = (start_along + end_along) / 2.0;
    let from = run.origin(middle) - (run.second_normal * run.size);
    let limit = RUN_OUT_LIMIT_IN_SIZES * run.size;

    let (Some(forward), Some(backward)) = (
        run.run_out
            .try_measure(second_face, from, run.direction, limit),
        run.run_out
            .try_measure(second_face, from, -run.direction, limit),
    ) else {
        return;
    };

    if backward.bound == BevelRunOutBound::ConvexEdge {
        read_the_across_plane(run, start, backward.bounding_edge);
    }

    if forward.bound == BevelRunOutBound::ConvexEdge {
        read_the_across_plane(run, end, forward.bounding_edge);
    }

    if backward.bound == BevelRunOutBound::ConvexEdge
        && middle - backward.distance > start_along + second_face_stops_short_by(run)
    {
        run.second_face_from = middle - backward.distance;
    }

    if forward.bound == BevelRunOutBound::ConvexEdge
        && middle + forward.distance < end_along - second_face_stops_short_by(run)
    {
        run.second_face_to = middle + forward.distance;
    }
}

/// The plane of the face across the convex edge the second face ran out over, its normal turned
/// toward the riding face's tangent line.
pub fn read_the_across_plane(
    run: &BevelPartneredBeadRun,
    end: &mut BevelPartneredBeadEnd,
    convex_edge: i32,
) {
    let (Some((on_the_edge, _)), Some((normal0, normal1))) = (
        run.run_out.try_edge_ends(convex_edge),
        run.run_out.try_edge_face_normals(convex_edge),
    ) else {
        return;
    };

    let across = if normal0.dot(run.second_normal) > normal1.dot(run.second_normal) {
        normal1
    } else {
        normal0
    };

    let tangent_line = run.origin(run.along(end.vertex)) - (run.riding_normal * run.size);

    end.has_across_plane = true;
    end.across_point = on_the_edge;
    end.across_normal = if (tangent_line - on_the_edge).dot(across) > 0.0 {
        across
    } else {
        -across
    };
}

/// Whether this end's riding line closes on its across plane, and if so the end stops a tip short
/// of it, on a convex edge.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn try_end_on_the_across_plane(
    run: &BevelPartneredBeadRun,
    end: &mut BevelPartneredBeadEnd,
) -> bool {
    let outward = if end.at_start {
        -run.direction
    } else {
        run.direction
    };
    let closing = outward.dot(end.across_normal);

    if !(closing < -1e-9) {
        return false;
    }

    let tangent_line = run.origin(run.along(end.vertex)) - (run.riding_normal * run.size);
    let reach = -(tangent_line - end.across_point).dot(end.across_normal) / closing;

    end.ends_on_the_across_plane = true;
    end.distance = (reach - (ACROSS_PLANE_TIP_IN_SIZES * run.size)).clamp(0.0, end.distance);
    end.bound = BevelRunOutBound::ConvexEdge;

    true
}

/// The run's arc and the station frame's fixed points, and which way round its polygons are walked.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn try_build_arc(
    run: &mut BevelPartneredBeadRun,
    crease_point: Vector3,
    segments: i32,
) -> Result<(), String> {
    let (x_axis, y_axis) = (run.x_axis, run.y_axis);
    let flat = |from_the_ball_centre: Vector3| {
        Vector2::new(
            from_the_ball_centre.dot(x_axis),
            from_the_ball_centre.dot(y_axis),
        )
    };

    let riding_tangent = flat(-run.riding_normal * run.size);
    let second_tangent = flat(-run.second_normal * run.size);

    run.riding_tangent_point = riding_tangent;
    run.riding_normal2 = flat(run.riding_normal);
    run.crease = flat(crease_point - run.origin(run.along(crease_point)));
    run.into_the_riding_face = flat(-run.riding_normal * run.clearance);
    run.into_the_second_face = flat(-run.second_normal * run.clearance);

    let between = run.riding_normal + run.second_normal;
    let closer = 1.0 + run.riding_normal.dot(run.second_normal);

    let reach = if closer > 1e-9 && between.length() > 1e-9 {
        min_f64(
            run.clearance / closer,
            DEEPEST_BURIAL * run.clearance / between.length(),
        )
    } else {
        0.0
    };

    run.into_the_corner = flat(-between * reach);

    let from_angle = riding_tangent.y.atan2(riding_tangent.x);
    let to_angle = second_tangent.y.atan2(second_tangent.x);
    let mut sweep = to_angle - from_angle;

    while sweep > std::f64::consts::PI {
        sweep -= std::f64::consts::PI * 2.0;
    }

    while sweep < -std::f64::consts::PI {
        sweep += std::f64::consts::PI * 2.0;
    }

    if !((from_angle + (sweep / 2.0)).sin() > 0.0) {
        return Err("this run's arc does not cross its own bisector, so its two faces were not read as one wedge".to_string());
    }

    run.arc = (0..=segments + 2)
        .map(|i| {
            let angle = from_angle + (sweep * f64::from(i) / f64::from(segments + 2));
            Vector2::new(angle.cos() * run.size, angle.sin() * run.size)
        })
        .collect();

    run.reverse = signed_area(&close(
        run,
        run.crease,
        run.into_the_corner,
        run.into_the_second_face,
        1.0,
    )) < 0.0;

    Ok(())
}

/// A straight open concave run's partnered bead - read the run and both ends' partners, take every
/// station's section and sew them - or the refusal that says this run's bead is the one the sweep
/// has always built. `clearance` is optional (`None` takes [`CLEARANCE_IN_SIZES`] sizes).
/// `concave_runs` is every concave run of the rebuild.
pub fn try_build(
    chain: &BevelSweptChain,
    concave_runs: &[BevelSweptChain],
    size: f64,
    segments: i32,
    run_out: Option<Rc<BevelFaceRunOut>>,
    clearance: Option<f64>,
) -> Result<Mesh, String> {
    let (run, mut ends) = try_read_run(
        chain,
        size,
        segments,
        clearance.unwrap_or(size * CLEARANCE_IN_SIZES),
        run_out,
    )?;

    for end in ends.iter_mut() {
        try_read_partners(chain, concave_runs, &run, end)?;
    }

    let stations = stations(&run, &ends);
    let mut rings = Vec::with_capacity(stations.len());
    let mut sections = Vec::with_capacity(stations.len());

    for station in &stations {
        let section = try_station(&run, &ends, station)?;

        let ring: Vec<Vector3> = (0..run.point_count as usize)
            .map(|point| run.in_part(station.along, section[point]))
            .collect();

        sections.push(section);
        rings.push(ring);
    }

    let Some(mesh) = sew(
        &rings,
        &sections[0],
        &sections[sections.len() - 1],
        run.point_count,
    ) else {
        return Err(
            "the bead's own stations could not be capped, so it does not close".to_string(),
        );
    };

    let verdict = classify_boolean_operand(&mesh, Matrix4X4::IDENTITY, false);

    if verdict != BooleanOperandVerdict::Clean {
        return Err(if verdict == BooleanOperandVerdict::SelfIntersecting {
            "the bead this run's stations sew up to intersects itself".to_string()
        } else {
            "the bead this run's stations sew up to is not a usable solid".to_string()
        });
    }

    Ok(mesh)
}
