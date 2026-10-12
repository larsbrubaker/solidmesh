//! The measuring half of `R/Tests/GeometryProcessingTests/BevelKnifeEdgeRimTests.cs`, split out
//! of `bevel_knife_edge_rim_tests.rs` by responsibility: the prism and box parts, and the readings
//! that slice a cut part square to an edge (or read every vertex of a corner box) and say how far
//! its surface is off the arc, chord or pair of rounds that edge should carry.

use polygon_mesh::Mesh;
use vector_math::{Vector2, Vector3};

pub const SIZE: f64 = 20.0;
pub const RADIUS: f64 = 2.0;

/// How far off its arc a point of the cut surface may sit. A 16 segment arc's own chord sag over a
/// radius 2 fillet is under 0.01, so 0.05 is loose enough to ignore the faceting and tight enough
/// that a blend that has wandered a whole millimetre cannot hide in it.
pub const TOLERANCE: f64 = 0.05;

/// Where the knife edge's fillet centre sits: 2 in from the x = 0 face, so x = 2, and 2 in from
/// the slope x + y = 20, so (20 - x - y) / sqrt(2) = 2, which puts it at y = 18 - 2 sqrt(2) =
/// 15.172.
pub fn knife_centre_y() -> f64 {
    18.0 - (2.0 * 2.0_f64.sqrt())
}

/// One station's answer: how far the worst point of its section is off the arc, and the sentence
/// that says where. Every station is measured before any is asserted, so one failure reports the
/// whole length of the edge rather than only the first slice that goes wrong.
pub struct SectionReading {
    pub worst: f64,
    pub report: String,
}

impl SectionReading {
    fn new(worst: f64, report: String) -> Self {
        SectionReading { worst, report }
    }
}

/// Fails on the worst station, quoting all of them.
pub fn assert_every_section(readings: impl IntoIterator<Item = SectionReading>, expected: &str) {
    let stations: Vec<SectionReading> = readings.into_iter().collect();
    let worst = stations
        .iter()
        .map(|station| station.worst)
        .fold(f64::NEG_INFINITY, f64::max);

    assert!(
        worst <= TOLERANCE,
        "every section has to be {expected} - {}",
        stations
            .iter()
            .map(|station| station.report.as_str())
            .collect::<Vec<_>>()
            .join("; ")
    );
}

/// The section of the top edge's fillet in the plane y = `y`. That edge runs along y at x = 0,
/// z = 20, so the plane is square to it and the fillet's centre in it is (2, 18).
pub fn measure_top_edge_section(cut: &Mesh, y: f64) -> SectionReading {
    measure_section(
        cut,
        1,
        y,
        // Inside the fillet's own quadrant and clear of both tangent lines, so the flat faces the
        // fillet runs out onto are not counted as points off the arc.
        |point| point.x > 0.05 && point.x < 2.6 && point.z > 17.4 && point.z < 19.95,
        |point| Vector2::new(point.x, point.z),
        Vector2::new(2.0, 18.0),
        &format!("top edge at y = {y:.1}"),
    )
}

/// The section of the knife edge's fillet in the plane z = `z`. That edge runs up z at (0, 20), and
/// its fillet is the circle 2 in from the x = 0 face and 2 in from the slope.
pub fn measure_knife_edge_section(cut: &Mesh, z: f64) -> SectionReading {
    measure_section(
        cut,
        2,
        z,
        |point| point.x > 0.05 && point.y > 15.2 && point.x + point.y < 19.95,
        |point| Vector2::new(point.x, point.y),
        Vector2::new(2.0, knife_centre_y()),
        &format!("knife edge at z = {z:.1}"),
    )
}

/// The section of the top edge's CHAMFER in the plane y = `y`. The chamfer reaches 2 along each
/// face from the edge at x = 0, z = 20, so it is the plane through (0, 18) and (2, 20) in the
/// slice - the line `z = x + 18`.
pub fn measure_top_edge_chamfer_section(cut: &Mesh, y: f64) -> SectionReading {
    measure_section_against_line(
        cut,
        1,
        y,
        // Between the two tangent lines and clear of both, so neither flat face is counted as a
        // point off the chamfer.
        |point| point.x > 0.05 && point.x < 1.95 && point.z > 18.05 && point.z < 19.95,
        |point| Vector2::new(point.x, point.z),
        Vector2::new(0.0, 18.0),
        Vector2::new(2.0, 20.0),
        &format!("top edge at y = {y:.1}"),
    )
}

/// The section of the knife edge's CHAMFER in the plane z = `z`. Two along the x = 0 face from the
/// edge at (0, 20) is (0, 18); two along the slope x + y = 20 is (2 / sqrt 2, 20 - 2 / sqrt 2) =
/// (1.414, 18.586).
pub fn measure_knife_edge_chamfer_section(cut: &Mesh, z: f64) -> SectionReading {
    let along_the_slope = RADIUS / 2.0_f64.sqrt();

    measure_section_against_line(
        cut,
        2,
        z,
        // Well inside both tangent points: x = 0 is the flat face and x + y = 20 is the slope, and
        // the chamfer runs between them from (0, 18) to (1.414, 18.586).
        |point| point.x > 0.12 && point.x < 1.3 && point.y > 17.9 && point.x + point.y < 19.9,
        |point| Vector2::new(point.x, point.y),
        Vector2::new(0.0, SIZE - RADIUS),
        Vector2::new(along_the_slope, SIZE - along_the_slope),
        &format!("knife edge at z = {z:.1}"),
    )
}

/// The back corner at z = 0 (`centre_z` 2) or z = 20 (18): the top or bottom round about (x 2,
/// z `centre_z`) meeting the back round about (x 2, y 2).
pub fn measure_back_corner(cut: &Mesh, centre_z: f64) -> SectionReading {
    let mut worst = 0.0;
    let mut at = Vector3::ZERO;
    let mut count = 0;
    for v in cut.vertices.iter().map(|p| p.as_vector3()) {
        if v.x < 0.05
            || v.x > 2.0
            || v.y < 0.05
            || v.y > 4.0
            || (v.z - centre_z).abs() > 2.0
            || v.z < 0.05
            || v.z > 19.95
        {
            continue;
        }

        let mut excess = f64::NEG_INFINITY;
        if if centre_z > SIZE / 2.0 {
            v.z > centre_z
        } else {
            v.z < centre_z
        } {
            excess = excess
                .max((Vector2::new(v.x, v.z) - Vector2::new(2.0, centre_z)).length() - RADIUS);
        }

        if v.y < 2.0 {
            excess =
                excess.max((Vector2::new(v.x, v.y) - Vector2::new(2.0, 2.0)).length() - RADIUS);
        }

        if excess == f64::NEG_INFINITY {
            continue;
        }

        count += 1;
        if excess.abs() > worst {
            worst = excess.abs();
            at = v;
        }
    }

    if count == 0 {
        SectionReading::new(
            f64::INFINITY,
            format!("the back corner by z = {centre_z} has NO points"),
        )
    } else {
        SectionReading::new(
            worst,
            format!(
                "the back corner by z = {centre_z} is off its two rounds by up to {worst:.3} at \
                 {}, over {count} vertices",
                show(at)
            ),
        )
    }
}

/// The corner where the top edge meets the knife edge, (0, 20, 20). The true part there is the
/// prism with both rounds taken out: a point in the top round's wedge (x below 2, z above 18) is at
/// most 2 from its axis (x = 2, z = 18); a point in the knife's wedge (the sector from its centre
/// (2, 15.172) between the two face normals) is at most 2 from its axis. The surface is where the
/// larger of the two applicable excesses is zero. Every triangle vertex of the cut in the corner box
/// is read - a slice would pass between them.
pub fn measure_top_knife_corner(cut: &Mesh) -> SectionReading {
    let knife_centre = Vector2::new(2.0, knife_centre_y());
    let mut worst = 0.0;
    let mut at = Vector3::ZERO;
    let mut count = 0;
    for v in cut.vertices.iter().map(|p| p.as_vector3()) {
        if v.x < 0.05 || v.x > 2.0 || v.y < 14.0 || v.z < 15.0 || v.z > 19.95 || v.x + v.y > 19.95 {
            continue;
        }

        let mut excess = f64::NEG_INFINITY;
        if v.z > 18.0 {
            excess =
                excess.max((Vector2::new(v.x, v.z) - Vector2::new(2.0, 18.0)).length() - RADIUS);
        }

        // The knife round's wedge is the sector from its centre between the slope's normal (45
        // degrees) and the x = 0 face's (180 degrees).
        let from_knife = Vector2::new(v.x, v.y) - knife_centre;
        if from_knife.y >= 0.0 && from_knife.y >= from_knife.x {
            excess = excess.max((Vector2::new(v.x, v.y) - knife_centre).length() - RADIUS);
        }

        if excess == f64::NEG_INFINITY {
            continue;
        }

        count += 1;
        if excess.abs() > worst {
            worst = excess.abs();
            at = v;
        }
    }

    if count == 0 {
        SectionReading::new(
            f64::INFINITY,
            "the top-knife corner has NO points".to_string(),
        )
    } else {
        SectionReading::new(
            worst,
            format!(
                "the top-knife corner is off its two rounds by up to {worst:.3} at {}, over \
                 {count} vertices",
                show(at)
            ),
        )
    }
}

/// The same slice as [`measure_section`], measured against the straight chord a chamfer carries
/// instead of against an arc.
#[allow(clippy::too_many_arguments)]
fn measure_section_against_line(
    cut: &Mesh,
    axis: usize,
    value: f64,
    in_window: impl Fn(Vector3) -> bool,
    section: impl Fn(Vector3) -> Vector2,
    from: Vector2,
    to: Vector2,
    what: &str,
) -> SectionReading {
    let points: Vec<Vector2> = slice_points(cut, axis, value)
        .into_iter()
        .filter(|&point| in_window(point))
        .map(section)
        .collect();

    if points.is_empty() {
        return SectionReading::new(
            f64::INFINITY,
            format!("{what} has NO section points in the chamfer window"),
        );
    }

    let along = (to - from).get_normal();
    let across = Vector2::new(-along.y, along.x);
    let offsets: Vec<f64> = points
        .iter()
        .map(|&point| (point - from).dot(across))
        .collect();
    let worst = offsets
        .iter()
        .map(|offset| offset.abs())
        .fold(f64::NEG_INFINITY, f64::max);
    let at = points[offsets
        .iter()
        .position(|offset| offset.abs() == worst)
        .expect("the worst is one of them")];

    SectionReading::new(
        worst,
        format!(
            "{what} is off its chamfer through ({:.3}, {:.3}) and ({:.3}, {:.3}) by up to \
             {worst:.3} at ({:.3}, {:.3}), over {} section points",
            from.x,
            from.y,
            to.x,
            to.y,
            at.x,
            at.y,
            points.len()
        ),
    )
}

/// Slices the cut part square to one edge and asks every point of the section that falls in the
/// fillet's window how far it is from the arc that edge should carry.
fn measure_section(
    cut: &Mesh,
    axis: usize,
    value: f64,
    in_window: impl Fn(Vector3) -> bool,
    section: impl Fn(Vector3) -> Vector2,
    centre: Vector2,
    what: &str,
) -> SectionReading {
    let points: Vec<Vector2> = slice_points(cut, axis, value)
        .into_iter()
        .filter(|&point| in_window(point))
        .map(section)
        .collect();

    if points.is_empty() {
        // Nothing in the window is a failure in its own right - the cut did not reach this edge
        // at all.
        return SectionReading::new(
            f64::INFINITY,
            format!("{what} has NO section points in the fillet window"),
        );
    }

    let offsets: Vec<f64> = points
        .iter()
        .map(|&point| (point - centre).length() - RADIUS)
        .collect();
    let worst = offsets
        .iter()
        .map(|offset| offset.abs())
        .fold(f64::NEG_INFINITY, f64::max);
    let at = points[offsets
        .iter()
        .position(|offset| offset.abs() == worst)
        .expect("the worst is one of them")];

    SectionReading::new(
        worst,
        format!(
            "{what} is off its circle centred ({:.3}, {:.3}) by up to {worst:.3} at ({:.3}, \
             {:.3}), over {} section points",
            centre.x,
            centre.y,
            at.x,
            at.y,
            points.len()
        ),
    )
}

/// Every point where the mesh's triangle edges cross the plane `component[axis] == value`.
fn slice_points(mesh: &Mesh, axis: usize, value: f64) -> Vec<Vector3> {
    let mut points = Vec::new();

    for face in mesh.faces.iter() {
        let corners = [
            mesh.vertices[face.v0 as usize].as_vector3(),
            mesh.vertices[face.v1 as usize].as_vector3(),
            mesh.vertices[face.v2 as usize].as_vector3(),
        ];

        for i in 0..3 {
            let from = corners[i];
            let to = corners[(i + 1) % 3];
            let from_side = component(from, axis) - value;
            let to_side = component(to, axis) - value;

            if from_side == 0.0 {
                // A vertex sitting exactly on the plane is a section point in its own right. It is
                // added once per triangle that owns it, which is harmless - the measurement is a
                // maximum over the points, not a count of them.
                points.push(from);
            } else if (from_side < 0.0) != (to_side < 0.0) && to_side != 0.0 {
                points.push(from + ((to - from) * (from_side / (from_side - to_side))));
            }
        }
    }

    points
}

fn component(v: Vector3, axis: usize) -> f64 {
    match axis {
        0 => v.x,
        1 => v.y,
        _ => v.z,
    }
}

/// The right triangular prism "Center Right and Left Side.mcx" is a slice of: legs on x = 0 and
/// y = 0, hypotenuse x + y = 20, extruded z = 0 to 20. Its x = 0 face is bounded by three right
/// angles and, at (0, 20), the 45 degree knife where it meets the hypotenuse's slope.
pub fn prism() -> Mesh {
    extrude(&[
        Vector2::new(0.0, 0.0),
        Vector2::new(SIZE, 0.0),
        Vector2::new(0.0, SIZE),
    ])
}

/// The control part: a box over the same 0 to 20 cube of space, so every slice, window and arc
/// centre the prism is measured with applies to it unchanged.
pub fn r#box() -> Mesh {
    extrude(&[
        Vector2::new(0.0, 0.0),
        Vector2::new(SIZE, 0.0),
        Vector2::new(SIZE, SIZE),
        Vector2::new(0.0, SIZE),
    ])
}

/// Extrudes a counter-clockwise plan from z = 0 to z = 20, wound so every face points out.
fn extrude(plan: &[Vector2]) -> Mesh {
    let mut mesh = Mesh::new();

    let bottom: Vec<Vector3> = plan
        .iter()
        .map(|corner| Vector3::new(corner.x, corner.y, 0.0))
        .rev()
        .collect();
    mesh.create_face(&bottom);
    let top: Vec<Vector3> = plan
        .iter()
        .map(|corner| Vector3::new(corner.x, corner.y, SIZE))
        .collect();
    mesh.create_face(&top);

    for i in 0..plan.len() {
        let from = plan[i];
        let to = plan[(i + 1) % plan.len()];

        mesh.create_face(&[
            Vector3::new(from.x, from.y, 0.0),
            Vector3::new(to.x, to.y, 0.0),
            Vector3::new(to.x, to.y, SIZE),
            Vector3::new(from.x, from.y, SIZE),
        ]);
    }

    // CreateFace adds fresh vertices every call, so what is built above is triangle soup.
    mesh.clean_and_merge();

    mesh
}

/// The index of a triangle of the x = 0 face, which is the face the rim is taken from.
pub fn x_min_face(mesh: &Mesh) -> i32 {
    for (i, face) in mesh.faces.iter().enumerate() {
        if [face.v0, face.v1, face.v2]
            .iter()
            .all(|&vertex| f64::from(mesh.vertices[vertex as usize].x).abs() < 1e-4)
        {
            return i as i32;
        }
    }

    panic!("the part has no face at x = 0");
}

pub fn show(point: Vector3) -> String {
    format!("({:.2}, {:.2}, {:.2})", point.x, point.y, point.z)
}
