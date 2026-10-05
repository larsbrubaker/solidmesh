//! The room, read off a mesh and a pick without building anything: the least of what foreign
//! features bound, what two chains of one pick leave each other, and what each side holds of
//! itself, less the margin a kept strip needs to exist at all. Ports `Bevel/LevelLine/BandRoom.cs`
//! (a C# static partial class; its pair room is `band_room_pair_room.rs`): so far the chains'
//! lengths ([`lengths_of`]), the past-the-end test ([`is_past_an_end_face`]), the convex end faces
//! as a list ([`present`]), which foreign reading names the room ([`the_foreign_source`]) and the
//! room's rounding down ([`steady_down`], [`on_grid`]); the readings land with later packages.

use vector_math::Vector3;

use super::chain_on_the_mesh;
use super::end_face_sense::PointPlane;
use super::{LevelLineMesh, RoomSource};

/// C# `BandRoom.RoomDigits`: significant digits the room is rounded down to, so it reads the same
/// at any size asked.
pub const ROOM_DIGITS: i32 = 12;

/// C# `BandRoom.ReferenceSize`: the ONE size every room is read at. A setback is held per unit of
/// size, so the reading is a ratio and the number that comes out is a size; reading at the size a
/// caller asked for instead would tie the answer to the question, because the searches underneath
/// carry tolerances that scale with it.
pub const REFERENCE_SIZE: f64 = 1.0;

/// Each chain's length along its own edges.
pub fn lengths_of(positions: &[Vector3], chains: &[Vec<i32>], closed: &[bool]) -> Vec<f64> {
    let mut lengths = vec![0.0; chains.len()];
    for (k, chain) in chains.iter().enumerate() {
        let mut total = 0.0;
        for (from, to) in chain_on_the_mesh::chain_edges(chain, closed[k]) {
            total += (positions[to as usize] - positions[from as usize]).length();
        }

        lengths[k] = total;
    }

    lengths
}

/// True when both ends of `edge` stand more than `snap` beyond one of the convex end faces (each
/// a point on it and its outward normal).
pub fn is_past_an_end_face(
    mesh: &LevelLineMesh,
    edge: (i32, i32),
    convex_end_faces: &[(Vector3, Vector3)],
    snap: f64,
) -> bool {
    let beyond = |p: Vector3, face: &(Vector3, Vector3)| (p - face.0).dot(face.1) > snap;
    convex_end_faces
        .iter()
        .any(|face| beyond(mesh.position(edge.0), face) && beyond(mesh.position(edge.1), face))
}

/// C# `BandRoom.Present`: the convex end faces `EndFaceSense.ConvexEndFaces` found, as a list.
pub fn present(ends: (Option<PointPlane>, Option<PointPlane>)) -> Vec<PointPlane> {
    let mut faces = Vec::new();
    if let Some(first) = ends.0 {
        faces.push(first);
    }

    if let Some(last) = ends.1 {
        faces.push(last);
    }

    faces
}

/// C# `BandRoom.TheForeignSource`: which foreign reading a one-chain room names: a growing
/// neighbour's far crease only where it alone sets the room - strictly under every edge on the face
/// (a tie stays the face's own reading, the sentence the user has always read) and not undercut by
/// the fillet-pierce room, which is the ridge leaving the round and no crease at all. `crease_at` is
/// read only then.
pub fn the_foreign_source(
    near_room: f64,
    far_room: f64,
    room: f64,
    crease_at: &dyn Fn() -> Vector3,
) -> RoomSource {
    if room == f64::INFINITY {
        return RoomSource::nothing();
    }

    if far_room < near_room && room == far_room {
        RoomSource::growing_crease(0, crease_at())
    } else {
        RoomSource::foreign(0)
    }
}

/// C# `BandRoom.SteadyDown`: the room rounded DOWN onto a grid of [`ROOM_DIGITS`] significant
/// digits, which is what makes the room a number a user can hand straight back.
///
/// A room is a division and a subtraction over floating point, so two readings of the same
/// geometry can differ in their last bits - the strip's face read its half width as
/// 2.0000000000000004 one way and 2.0 the other, and the lower of the two refused the very size the
/// higher had named. Rounding down lands both on one number, and down rather than near, so the
/// rounding can only ever offer LESS than the geometry has.
pub fn steady_down(room: f64) -> f64 {
    if !room.is_finite() {
        // Infinity is a room - nothing bounds the sides - and NaN is a reading that ran out of
        // work, which no size compares above. Neither is a number to clamp.
        return room;
    }

    if room <= 0.0 {
        // A room is a SIZE, so it never reads below zero. Every reading here has had the kept
        // strip's margin taken off it, and where the geometry holds less room than that margin the
        // subtraction goes past zero - a face pick in the app read -1.67e-9 and handed that back as
        // the size a drag was allowed. Zero says exactly what a negative says - nothing builds - in
        // a number a user can be shown.
        return 0.0;
    }

    on_grid(room, ROOM_DIGITS)
}

/// C# `BandRoom.OnGrid`: a positive number rounded DOWN onto a grid of the given significant
/// digits.
pub fn on_grid(value: f64, digits: i32) -> f64 {
    if !value.is_finite() || value <= 0.0 {
        return value;
    }

    let scale = 10f64.powf(f64::from(digits - 1) - value.log10().floor());
    (value * scale).floor() / scale
}
