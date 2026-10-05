//! The room, read off a mesh and a pick without building anything: the least of what foreign
//! features bound, what two chains of one pick leave each other, and what each side holds of
//! itself, less the margin a kept strip needs to exist at all. Ports `Bevel/LevelLine/BandRoom.cs`
//! (a C# static partial class): so far the chains' lengths ([`lengths_of`]) and the past-the-end
//! test ([`is_past_an_end_face`]); the readings land with later packages.

use vector_math::Vector3;

use super::chain_on_the_mesh;
use super::LevelLineMesh;

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
