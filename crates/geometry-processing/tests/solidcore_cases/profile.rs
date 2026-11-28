//! The geometry a case's steps are drawn with, turned into what solidmesh's outline and mesh code
//! takes: profiles flattened into closed contours, a profile plane's frame, and rigid or mirror
//! placements as row-vector matrices.
//!
//! Curves are flattened into INSCRIBED chords, `circle_sides` to a full turn (an arc gets its
//! share, at least one chord), with every chord end on the exact curve. That is what makes the
//! faceting's error the chord sagitta, `r * (1 - cos(pi / circle_sides))`, which the grading in
//! `grade.rs` bounds against. Pieces that share end points (within `JOIN_DISTANCE`) are joined end
//! to end, either way round, into loops; a polygon or a circle is a loop of its own.

use solidmesh::math::{Matrix4X4, Vector2, Vector3, Vector4};
use solidmesh::mesh::Mesh;

use crate::case_file::{ProfilePlane, Seg, P3};

/// How near two piece ends must be to be the same point of a loop (mm).
const JOIN_DISTANCE: f64 = 1e-6;

/// A case point as a vector.
pub fn v3(p: P3) -> Vector3 {
    Vector3::new(p[0], p[1], p[2])
}

/// The coarsest resolution's chords to a full turn; every finer one is this times a power of two.
pub const BASE_CIRCLE_SIDES: usize = 48;

/// The chords an arc of `sweep_degrees` gets at `circle_sides` to the full turn: its share of
/// [`BASE_CIRCLE_SIDES`], rounded up, times `circle_sides / BASE_CIRCLE_SIDES`. Each finer run then
/// halves every chord's angle exactly, which the Richardson extrapolation in `grade.rs` relies on
/// (rounding each run's share up on its own would not: a 320 degree arc would get 43, 86 and 171).
pub fn arc_chords(sweep_degrees: f64, circle_sides: usize) -> usize {
    // `as` saturates; a sweep is at most a turn here, so the count is at most `circle_sides`.
    let share = (BASE_CIRCLE_SIDES as f64 * sweep_degrees / 360.0).ceil() as usize;
    share.max(1) * (circle_sides / BASE_CIRCLE_SIDES).max(1)
}

fn arc_points(
    center: [f64; 2],
    radius: f64,
    start_deg: f64,
    sweep_deg: f64,
    chords: usize,
) -> Vec<Vector2> {
    (0..=chords)
        .map(|i| {
            let angle = (start_deg + sweep_deg * i as f64 / chords as f64).to_radians();
            Vector2::new(
                center[0] + radius * libm::cos(angle),
                center[1] + radius * libm::sin(angle),
            )
        })
        .collect()
}

fn same_point(a: Vector2, b: Vector2) -> bool {
    (a - b).length() <= JOIN_DISTANCE
}

/// The profile's closed loops, in the plane's (u, v) coordinates, each without its closing point.
pub fn flatten_profile(profile: &[Seg], circle_sides: usize) -> Result<Vec<Vec<Vector2>>, String> {
    let mut loops = Vec::new();
    let mut open_pieces: Vec<Vec<Vector2>> = Vec::new();
    for seg in profile {
        match seg {
            Seg::Polygon { points } => {
                loops.push(points.iter().map(|p| Vector2::new(p[0], p[1])).collect());
            }
            Seg::Circle { center, radius } => {
                let mut points = arc_points(*center, *radius, 0.0, 360.0, circle_sides);
                points.pop();
                loops.push(points);
            }
            Seg::Line { from, to } => {
                open_pieces.push(vec![
                    Vector2::new(from[0], from[1]),
                    Vector2::new(to[0], to[1]),
                ]);
            }
            Seg::Arc {
                center,
                radius,
                start_deg,
                end_deg,
            } => {
                let mut sweep = end_deg - start_deg;
                while sweep <= 0.0 {
                    sweep += 360.0;
                }
                open_pieces.push(arc_points(
                    *center,
                    *radius,
                    *start_deg,
                    sweep,
                    arc_chords(sweep, circle_sides),
                ));
            }
        }
    }

    // Join the lines and arcs end to end, in file order, into loops.
    let mut used = vec![false; open_pieces.len()];
    while let Some(first) = used.iter().position(|u| !u) {
        used[first] = true;
        let mut chain = open_pieces[first].clone();
        loop {
            let end = chain[chain.len() - 1];
            if chain.len() > 2 && same_point(end, chain[0]) {
                chain.pop();
                break;
            }
            let next = (0..open_pieces.len()).find(|&i| {
                !used[i]
                    && (same_point(open_pieces[i][0], end)
                        || same_point(open_pieces[i][open_pieces[i].len() - 1], end))
            });
            let Some(next) = next else {
                return Err(format!(
                    "the profile's pieces do not close into a loop at {end:?}"
                ));
            };
            used[next] = true;
            let mut piece = open_pieces[next].clone();
            if !same_point(piece[0], end) {
                piece.reverse();
            }
            chain.extend(piece.into_iter().skip(1));
        }
        loops.push(chain);
    }
    Ok(loops)
}

/// The plane's u axis, v axis and normal in the world.
pub fn plane_frame(plane: ProfilePlane) -> (Vector3, Vector3, Vector3) {
    match plane {
        ProfilePlane::Xy => (Vector3::UNIT_X, Vector3::UNIT_Y, Vector3::UNIT_Z),
        ProfilePlane::Yz => (Vector3::UNIT_Y, Vector3::UNIT_Z, Vector3::UNIT_X),
        ProfilePlane::Zx => (Vector3::UNIT_Z, Vector3::UNIT_X, Vector3::UNIT_Y),
        ProfilePlane::Xz => (
            Vector3::UNIT_X,
            Vector3::UNIT_Z,
            Vector3::new(0.0, -1.0, 0.0),
        ),
    }
}

/// `v` turned `radians` about the unit `axis` by the right-hand rule.
pub fn rotated(v: Vector3, axis: Vector3, radians: f64) -> Vector3 {
    let (sin, cos) = (libm::sin(radians), libm::cos(radians));
    v * cos + axis.cross(v) * sin + axis * (axis.dot(v) * (1.0 - cos))
}

/// The turn carrying +Z onto `axis` about the axis square to both (none when they agree, a half
/// turn about X when they are opposite), as images of the world axes.
pub fn frame_onto(axis: Vector3) -> (Vector3, Vector3, Vector3) {
    let a = Vector3::normalized(axis);
    let k = Vector3::UNIT_Z.cross(a);
    let sin = k.length();
    let cos = Vector3::UNIT_Z.dot(a);
    if sin < 1e-15 {
        if cos > 0.0 {
            return (Vector3::UNIT_X, Vector3::UNIT_Y, Vector3::UNIT_Z);
        }
        return (
            Vector3::UNIT_X,
            Vector3::new(0.0, -1.0, 0.0),
            Vector3::new(0.0, 0.0, -1.0),
        );
    }
    let k = k * (1.0 / sin);
    let angle = libm::atan2(sin, cos);
    (
        rotated(Vector3::UNIT_X, k, angle),
        rotated(Vector3::UNIT_Y, k, angle),
        a,
    )
}

/// Moves every vertex of `mesh` from local `(x, y, z)` to `origin + x*ex + y*ey + z*ez`, turning
/// the faces over where that map mirrors, so the solid still faces out.
pub fn place(mesh: &mut Mesh, origin: Vector3, ex: Vector3, ey: Vector3, ez: Vector3) {
    let matrix = Matrix4X4::from_rows(
        Vector4::new(ex.x, ex.y, ex.z, 0.0),
        Vector4::new(ey.x, ey.y, ey.z, 0.0),
        Vector4::new(ez.x, ez.y, ez.z, 0.0),
        Vector4::new(origin.x, origin.y, origin.z, 1.0),
    );
    mesh.transform(matrix);
    if ex.cross(ey).dot(ez) < 0.0 {
        mesh.reverse_faces();
    }
}
