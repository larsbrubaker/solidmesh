//! The whole vertex-centric build for one or several chains: sections, rebuilt faces, strips, and
//! the mesh they make together.
//!
//! **One pool, one face list.** The rebuilt faces ([`super::section_face_rebuild`]) and the strips
//! ([`super::section_loft`]) are laid into the SAME list over the SAME [`SectionPool`], so the rail
//! a face was handed and the rail a strip was lofted along are the same integers. The part and the
//! fillet are therefore already one surface before anything is published: there is no weld, no
//! find-near and no stitching pass here.
//!
//! **Winding needs no reconciliation either.** The rebuilt faces keep the input mesh's own winding,
//! and the loft winds each run from the way OUT of the material at its first quad, so along the
//! boundary rings each ring edge is used once by a rebuilt face and once by a strip triangle the
//! other way round - the condition for the published mesh to be closed and manifold.
//!
//! **The one renumbering.** The pool's indices never move while the construction runs, but a
//! published mesh may not carry points no face uses, and after the substitution the chain's own
//! vertices are used by nothing. So exactly here, once, at the end, the used points are gathered in
//! first-seen order and the faces rewritten onto them.
//!
//! A build that fails says why with [`SectionBuildRefusal`]: a named refusal, a decline with no
//! advisory, or a NaN point.

use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::Vector3Float;

use super::section_builder_walked_side::SectionBuildRefusal;
use super::section_smooth_seam::{self, SectionSmoothSeam, SmoothSeamFailure};
use super::{section_builder_run, section_face_rebuild, section_loft, SectionPool, SectionRun};
use crate::bevel::surface::surface_mesh_closure;
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};
use crate::topology::FeatureEdgeChain;

/// The filleted mesh for one simple closed chain, built from the chain itself.
/// A missing mesh, graph or chain is refused as `SurfaceConstructionNotBuilt`.
pub fn try_build(
    mesh: Option<&Mesh>,
    graph: Option<&MeshEdgeGraph>,
    chain: Option<&FeatureEdgeChain>,
    radius: f64,
    segments: i32,
) -> Result<Mesh, SectionBuildRefusal> {
    let (Some(mesh), Some(graph), Some(chain)) = (mesh, graph, chain) else {
        return Err(SectionBuildRefusal::Refused(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceConstructionNotBuilt,
            vec![],
        )));
    };

    let mut pool = SectionPool::new(&mesh.vertices);
    let run = section_builder_run::try_build_chain(
        Some(chain),
        mesh,
        graph,
        &mut pool,
        radius,
        segments,
        true,
    )?;

    try_assemble(mesh, graph, &run, &mut pool)
}

/// The same from a run that has already been built.
pub fn try_assemble(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    run: &SectionRun,
    pool: &mut SectionPool,
) -> Result<Mesh, SectionBuildRefusal> {
    try_assemble_runs(mesh, graph, std::slice::from_ref(run), pool)
}

/// The whole build for SEVERAL chains at once,
/// every run built over the ONE `pool` against the ORIGINAL mesh.
pub fn try_assemble_runs(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    runs: &[SectionRun],
    pool: &mut SectionPool,
) -> Result<Mesh, SectionBuildRefusal> {
    assemble(mesh, graph, runs, pool).map(|(mesh, _)| mesh)
}

/// The several-run build, saying as well WHICH of the
/// published faces the fillet itself is - the indices, into the result's own face list, of the
/// faces the LOFT laid. What they are FOR is fencing: a second bevel pass floods its picked face
/// regions at the row's flat angle, and a fillet's facets turn less than that.
pub fn try_assemble_with_blend(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    runs: &[SectionRun],
    pool: &mut SectionPool,
) -> Result<(Mesh, Vec<i32>), SectionBuildRefusal> {
    assemble(mesh, graph, runs, pool)
}

/// The one body both several-run entries run.
fn assemble(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    runs: &[SectionRun],
    pool: &mut SectionPool,
) -> Result<(Mesh, Vec<i32>), SectionBuildRefusal> {
    // The SMOOTH sides go first, because both of the stages below read what they did: the rebuild
    // leaves a cut side's faces alone and drops the ones it took out, and the loft ends its last row
    // on the very polyline the cut ran along.
    let seams = section_smooth_seam::try_cut_every_smooth_side(mesh, graph, pool, runs).map_err(
        |failure| match failure {
            SmoothSeamFailure::Refused(why) => SectionBuildRefusal::Refused(why),
            SmoothSeamFailure::NotANumber(nan) => SectionBuildRefusal::NotANumber(nan),
        },
    )?;

    let mut faces = section_face_rebuild::try_rebuild(mesh, graph, runs, pool, Some(&seams))
        .map_err(SectionBuildRefusal::Refused)?;

    // Everything the loft lays goes into the one list between these two marks, and the publish
    // keeps the list's order, so the blend is a contiguous range of the published face list.
    let first_of_the_blend = faces.len();

    for (index, run) in runs.iter().enumerate() {
        section_loft::try_emit(run, pool, &mut faces, seam_of(&seams, index))
            .map_err(SectionBuildRefusal::Refused)?;
    }

    let blend_faces: Vec<i32> = (first_of_the_blend..faces.len())
        .map(|i| i as i32)
        .collect();

    for seam in &seams {
        faces.extend(seam.cut.triangles.iter().copied());
    }

    let published = publish(pool, &faces);

    // A true answer is a CLOSED mesh, read here on the answer because no stage above can see it:
    // the cut, the rebuild and the loft each check what they make, and a gap between two of them -
    // the level tapered barrel's top rim, 228 open edges - passed all three.
    let (closed, open_at) = surface_mesh_closure::is_closed(&published);
    if !closed {
        return Err(SectionBuildRefusal::Refused(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceResultNotClosed,
            vec![open_at.into()],
        )));
    }

    Ok((published, blend_faces))
}

/// The seam of the run at `run` (an index into the runs, as
/// [`SectionSmoothSeam::run`] is), or `None` where neither of its sides is smooth.
fn seam_of(seams: &[SectionSmoothSeam], run: usize) -> Option<&SectionSmoothSeam> {
    seams.iter().find(|seam| seam.run as usize == run)
}

/// The pool and the face list as a mesh, keeping only the points some face uses and
/// renumbering them once, in face order - so the numbering is a fact about the face list, which is
/// itself pure topology over the run.
fn publish(pool: &SectionPool, faces: &[(i32, i32, i32)]) -> Mesh {
    let mut published = vec![-1i32; pool.count() as usize];
    let mut vertices: Vec<Vector3Float> = Vec::with_capacity(pool.count() as usize);

    let mut keep = |pooled: i32, vertices: &mut Vec<Vector3Float>| -> i32 {
        if published[pooled as usize] < 0 {
            published[pooled as usize] = vertices.len() as i32;
            vertices.push(Vector3Float::from_vector3(pool.position_of(pooled)));
        }

        published[pooled as usize]
    };

    let mut drawn = Vec::with_capacity(faces.len());
    for &(v0, v1, v2) in faces {
        let a = keep(v0, &mut vertices);
        let b = keep(v1, &mut vertices);
        let c = keep(v2, &mut vertices);
        drawn.push((a, b, c));
    }

    let mut built = Mesh::new();
    for &(v0, v1, v2) in &drawn {
        built.faces.add(v0, v1, v2, &vertices);
    }

    built.vertices = vertices;
    built
}
