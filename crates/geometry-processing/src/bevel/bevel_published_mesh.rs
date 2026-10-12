//! The last thing a bevel does to its result before anyone sees it (`Bevel/BevelPublishedMesh.cs`):
//! weld the seams the boolean left open and drop the triangles it left with no area ([`clean`]).
//! It welds at the tolerance `bevel_working_mesh` welds the mesh a bevel works against
//! ([`super::bevel_working_mesh::seam_tolerance_for`]) and then collapses the long slivers the weld
//! cannot see ([`super::bevel_sliver_collapse::collapse`]).
//!
//! A bevel is a stack of booleans, and a boolean whose operands touch almost tangentially - which
//! is what beads of one radius laid along the several creases of one feature ARE, since each is
//! tangent to the faces the others are tangent to - shreds the surface where they meet. The output
//! is correct in position to a few thousandths and still wrong to look at: slivers of near-zero
//! area, and pairs of them folded back on each other into fins, so a surface that should read as
//! one blend renders as a set of hard seams. That is a triangulation fault, not a shape one, and
//! nothing the topology reads upstream can see it coming - it depends on where the kernel happened
//! to cut.
//!
//! So it is repaired here, at the end, once. `BevelWorkingMesh` does the same job on the way IN - a
//! bevel welds the mesh it works against - and this is the other half of the same decision, at the
//! same tolerance, so the mesh a bevel publishes is as clean as the one it was handed.
//!
//! **Only where it is an improvement.** A weld can fuse vertices a manifold deliberately kept
//! apart, and a mesh that came back non-manifold, empty, or a different size from this pass is
//! worse than the one the boolean gave. Every one of those cases silently hands the uncleaned mesh
//! back: the result is then exactly what shipped before this stage existed, and the fault it was
//! meant to fix is one the user can already see, so there is nothing an advisory would tell them
//! that the part does not.

use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use super::bevel_sliver_collapse;
use super::bevel_working_mesh;
use super::level_line::WorkError;

/// How much of the part's volume the cleanup may move before it is reshaping rather than tidying.
/// One part in a million: welding across a seam tolerance moves a face by at most that tolerance,
/// so on a 20 mm part the honest changes here are orders of magnitude below this.
const VOLUME_CHANGE_FRACTION: f64 = 1e-6;

/// C# `BevelPublishedMesh.Clean`: the mesh to publish for a rebuilt bevel - welded and de-slivered
/// where that leaves a closed solid of the same volume, and the mesh handed in where it does not.
///
/// C# returns the SAME instance when there was nothing to do (or nothing worth doing), so a result
/// that was already clean is published unchanged rather than copied - which also keeps face colors
/// and every other per-face datum exactly as the boolean wrote them. Here that answer is `None`:
/// publish `published_mesh` as it is. `Some` is the cleaned copy. A cancel the sliver collapse
/// sees comes back as [`WorkError::Cancelled`].
pub fn clean(
    published_mesh: &Mesh,
    cancellation_token: &CancelToken,
) -> Result<Option<Mesh>, WorkError> {
    if published_mesh.faces.is_empty() {
        return Ok(None);
    }

    let tolerance = bevel_working_mesh::seam_tolerance_for(published_mesh);

    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    if !(tolerance > 0.0) {
        // Nothing to scale a tolerance against, and a mesh with no extent is not one welding helps.
        return Ok(None);
    }

    let minimum_face_area = bevel_working_mesh::minimum_face_area_for(tolerance);

    let mut cleaned = published_mesh.copy();

    // MergeVertices is the short-edge collapse as well as the weld: an edge shorter than the
    // tolerance has both its ends fused, which leaves the triangles along it with a repeated index,
    // and the same pass drops those. RemoveDegenerateFaces then takes the slivers that survived
    // with three distinct but nearly collinear vertices.
    cleaned.merge_vertices(tolerance, minimum_face_area, None);
    cleaned.remove_degenerate_faces(minimum_face_area);
    cleaned.remove_unused_vertices();

    let mut welded = cleaned.vertices.len() != published_mesh.vertices.len()
        || cleaned.faces.len() != published_mesh.faces.len();

    if welded && !is_an_improvement_on(&cleaned, published_mesh) {
        // The weld is judged on its own, before the collapse below runs, because the two stages
        // fail differently and a collapse that would have helped must not be thrown out with a
        // weld that did not. RemoveDegenerateFaces DROPS a face rather than closing the hole it
        // leaves, and merging two vertices that were at one point merges the two EDGES that ran to
        // them, which on a boolean result whose surface touches itself puts four faces on one
        // edge. Measured on "5mm Bevel Around Center.mcx" the weld does both: 4072 faces to 3322,
        // and the surface open. So there the whole cleanup used to be skipped - on exactly the part
        // that most needed it - and what runs instead now is the collapse, alone, on the mesh as
        // the boolean wrote it.
        cleaned = published_mesh.copy();
        welded = false;
    }

    // And then the slivers the weld cannot see: triangles the boolean left lying IN a face with a
    // full length edge and no height at all, whose area is orders of magnitude ABOVE the minimum
    // above because they are long. The tolerance is the same one - a triangle whose apex is nearer
    // to its opposite edge than two vertices have to be to be one vertex is a triangle whose apex
    // is on that edge - so the two stages here say one thing at one number. See
    // BevelSliverCollapse.
    let collapsed = bevel_sliver_collapse::collapse(&mut cleaned, tolerance, cancellation_token)?;

    if collapsed {
        // A recut leaves the vertex it cut to in use, but a face it dropped may have been the last
        // user of one of its own.
        cleaned.remove_unused_vertices();
    }

    if !welded && !collapsed {
        // Nothing was fused, nothing was dropped and nothing was recut, so the copy is the original
        // with an allocation behind it. Hand back what came in.
        return Ok(None);
    }

    if !is_an_improvement_on(&cleaned, published_mesh) {
        return Ok(None);
    }

    // The stored per-face normals are what the viewport flat-shades with and what the outline pass
    // reads, and a boolean's are not always the ones its own vertices give - measured on "5mm Bevel
    // Around Center.mcx", 115 of 7228 were not even unit length. Everything above has finished
    // moving faces about, so this is the one place they can all be made to agree with the mesh
    // being published.
    cleaned.calculate_normals();

    Ok(Some(cleaned))
}

/// C# `IsAnImprovementOn`: whether a cleaned mesh is one worth publishing in place of the one the
/// boolean gave - a closed solid enclosing the same material, wound the same way round.
///
/// A weld can fuse vertices a manifold deliberately kept apart, and dropping a face leaves a hole
/// where the surface was. A sign change is windings that stopped agreeing with each other - a
/// cleaned mesh that reads inside out is a part that prints inside out. None of those is the shape
/// the bevel computed, and a mesh that fails any of them is worse than the one it was made from.
fn is_an_improvement_on(cleaned: &Mesh, published_mesh: &Mesh) -> bool {
    if cleaned.faces.is_empty() || !cleaned.is_manifold() {
        return false;
    }

    let before = signed_volume(published_mesh);
    let after = signed_volume(cleaned);

    if before.is_nan() || after.is_nan() {
        // A volume that cannot be measured is not one the cleanup can be shown to keep, so the
        // mesh the boolean gave is handed back (divergence 4956; C#'s `Math.Sign` throws on NaN).
        // `clean` never gets here with one: a NaN coordinate makes the seam tolerance 0 first.
        return false;
    }

    sign(after) == sign(before) && (after - before).abs() <= before.abs() * VOLUME_CHANGE_FRACTION
}

/// C#'s `Math.Sign` of a double: -1, 0 or 1 (zero of either sign is 0). C# throws on NaN, which
/// [`is_an_improvement_on`] answers before asking.
fn sign(value: f64) -> i32 {
    if value > 0.0 {
        1
    } else if value < 0.0 {
        -1
    } else {
        0
    }
}

/// C# `SignedVolume`: the volume the mesh's faces enclose, signed by which way they wind, from the
/// divergence theorem. Positive when they face out.
fn signed_volume(mesh: &Mesh) -> f64 {
    let mut total = 0.0;

    for face in mesh.faces.iter() {
        let v0 = Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]);
        let v1 = Vector3::from_vector3_float(mesh.vertices[face.v1 as usize]);
        let v2 = Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]);

        total += v0.dot(v1.cross(v2));
    }

    total / 6.0
}

#[cfg(test)]
#[path = "bevel_published_mesh_tests.rs"]
mod tests;
