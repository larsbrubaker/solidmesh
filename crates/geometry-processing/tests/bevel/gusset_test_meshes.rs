//! The plate a gusset pokes through, kept as the triangles the boolean actually left, and the
//! sliver seam it leaves on the top face.

use solidmesh_math::{Vector3, Vector3Float};
use solidmesh_mesh::{Face, Mesh};

/// The plate-and-gusset union of "Chain and Face Should
/// Both work.mcx", as the kernel's own boolean left it - a 55 x 4 x 20 plate with a 5 x 5 x 20
/// triangular gusset beside it, and the gusset POKING 0.1 mm THROUGH the plate's y = -6 face.
/// Eighteen vertices and 32 triangles, every coordinate the float the boolean wrote.
///
/// **What the poke-through leaves.** The gusset's buried tip lands on z = 20 as THREE vertices
/// rather than one, all within 2e-06 of each other, because the gusset carries a 1.3113417e-07 term
/// in its own rotation that tilts its top 0.0011 degrees off z = 20. Two triangles hang on those
/// three: one with all three corners on them, whose area is nothing at all, and one running out to
/// (25.599997, -6, 20) where the hypotenuse leaves the plate. That second one is the SLIVER - 0.1414
/// long, 2e-06 thick, its normal (-0.696, 0.696, 0.174) nothing like either surface it lies between
/// - and a 0.4 fillet's setback of 0.3355 takes the whole of it.
///
/// **Built by index, and NOT cleaned.** `CleanAndMerge` welds vertices two microns apart, so running
/// it here would remove the very thing the fixture exists to carry. This is the part's left half
/// only: the upright and its three holes are no part of the sliver.
pub fn create_plate_with_gusset_poking_through() -> Mesh {
    let vertices = [
        Vector3Float::new_f32(-24.5, -10.0, 5.463922E-07),
        Vector3Float::new_f32(-24.5, -6.0, 5.463922E-07),
        Vector3Float::new_f32(-24.5, -10.0, 20.0),
        Vector3Float::new_f32(-24.5, -6.0, 20.0),
        Vector3Float::new_f32(30.5, -10.0, 5.463922E-07),
        Vector3Float::new_f32(25.599998, -6.0, 1.070929E-06),
        Vector3Float::new_f32(30.5, -6.0, 5.463922E-07),
        Vector3Float::new_f32(30.499998, -6.0, 6.425574E-07),
        Vector3Float::new_f32(30.5, -10.0, 20.0),
        Vector3Float::new_f32(25.599997, -6.0, 19.600018),
        Vector3Float::new_f32(25.5, -6.099996, 20.000002),
        Vector3Float::new_f32(25.5, -6.0999956, 20.0),
        Vector3Float::new_f32(25.5, -6.099996, 20.0),
        Vector3Float::new_f32(25.599997, -6.0, 20.0),
        Vector3Float::new_f32(30.5, -6.099996, 20.0),
        Vector3Float::new_f32(30.5, -6.0, 20.0),
        Vector3Float::new_f32(30.499998, -1.0999997, -1.2434498E-13),
        Vector3Float::new_f32(30.5, -1.0999961, 20.0),
    ];

    let triangles = [
        (10, 11, 12),
        (10, 13, 11),
        (10, 12, 14),
        (10, 14, 15),
        (16, 7, 5),
        (16, 5, 9),
        (16, 17, 7),
        (17, 15, 7),
        (10, 17, 13),
        (17, 9, 13),
        (10, 15, 17),
        (16, 9, 17),
        (0, 3, 1),
        (0, 1, 6),
        (0, 2, 3),
        (0, 8, 2),
        (1, 3, 9),
        (2, 12, 3),
        (0, 6, 4),
        (0, 4, 8),
        (1, 5, 6),
        (1, 9, 5),
        (4, 6, 8),
        (6, 5, 7),
        (6, 7, 15),
        (2, 8, 14),
        (3, 12, 11),
        (3, 11, 13),
        (3, 13, 9),
        (2, 14, 12),
        (8, 6, 14),
        (6, 15, 14),
    ];

    let mut mesh = Mesh::new();

    for vertex in vertices {
        mesh.vertices.push(vertex);
    }

    for (a, b, c) in triangles {
        let face = Face::new(a, b, c, &mesh.vertices);
        mesh.faces.push(face);
    }

    mesh
}

/// The sliver the poke-through leaves on z = 20, by the corners it is known by
/// - `(buried tip, where the hypotenuse leaves the plate)`.
pub fn the_gusset_seam() -> (Vector3, Vector3) {
    (
        Vector3::new(25.5, -6.099996, 20.0),
        Vector3::new(25.599997, -6.0, 20.0),
    )
}
