//! The Mesh Repair page: a closed solid broken on purpose - holes punched through it, faces
//! flipped inside out, the vertices unwelded into triangle soup - then mended by the library's
//! Repair operation: `Mesh::auto_repaired` (the default path) or `Mesh::advanced_repaired` with the
//! page's `AdvancedRepairOptions` (weld, orient faces, weld edges, fill holes).
//!
//! The breakage is seeded (`shapes::Lcg`), so a seed always breaks the mesh the same way. The note
//! carries the broken mesh's open edges and flipped faces next to the result's, which is the
//! before and after the page reports.

use std::collections::BTreeSet;

use polygon_mesh::{AdvancedRepairOptions, CancelToken, FaceList, Mesh, RemoveModes};
use vector_math::Vector3Float;

use crate::shapes::{Lcg, Shape};
use crate::Scene;

/// What the page's controls send.
pub struct RepairParams {
    /// 0 sphere, 1 rounded box, 2 torus.
    pub shape: u32,
    /// Hole patches to punch.
    pub holes: u32,
    /// Faces to flip.
    pub flips: u32,
    /// Unweld every face into its own three vertices.
    pub soup: bool,
    pub seed: u32,
    /// 0 show the broken mesh, 1 auto repair, 2 advanced repair.
    pub mode: u32,
    pub fill_holes: bool,
    pub face_orientation: bool,
    pub weld_edges: bool,
}

fn source(shape: u32) -> Result<Mesh, String> {
    match shape {
        0 => Shape::Sphere.build(30.0, 40),
        1 => Shape::RoundedBox.build(28.0, 24),
        2 => Shape::Torus.build(34.0, 48),
        other => Err(format!("There is no repair shape {other}.")),
    }
}

/// `mesh` with `holes` patches removed (a random face and every face touching its corners) and
/// `flips` random faces reversed; the second value is how many faces were flipped.
fn broken(mesh: &Mesh, holes: u32, flips: u32, soup: bool, seed: u32) -> (Mesh, usize) {
    let mut rng = Lcg::new(seed);
    let face_count = mesh.faces.len();
    let mut doomed_vertices = BTreeSet::new();
    for _ in 0..holes {
        let face = mesh.faces[rng.below(face_count)];
        doomed_vertices.extend([face.v0, face.v1, face.v2]);
    }
    let mut flipped = BTreeSet::new();
    for _ in 0..flips {
        flipped.insert(rng.below(face_count));
    }
    let mut vertices: Vec<Vector3Float> = if soup {
        Vec::new()
    } else {
        mesh.vertices.clone()
    };
    let mut faces = FaceList::new();
    let mut flipped_kept = 0;
    for (index, face) in mesh.faces.iter().enumerate() {
        if [face.v0, face.v1, face.v2]
            .iter()
            .any(|v| doomed_vertices.contains(v))
        {
            continue;
        }
        let mut corners = [face.v0, face.v1, face.v2];
        if flipped.contains(&index) {
            corners.swap(1, 2);
            flipped_kept += 1;
        }
        if soup {
            let base = vertices.len() as i32;
            vertices.extend(corners.iter().map(|&v| mesh.vertices[v as usize]));
            corners = [base, base + 1, base + 2];
        }
        faces.add(corners[0], corners[1], corners[2], &vertices);
    }
    let mut result = Mesh::from_vertices_and_faces(&vertices, &faces);
    if !soup {
        result.remove_unused_vertices();
    }
    (result, flipped_kept)
}

/// The broken mesh, or its repair.
pub fn repair_scene(params: &RepairParams) -> Result<Scene, String> {
    let solid = source(params.shape)?;
    let (damaged, flipped) = broken(&solid, params.holes, params.flips, params.soup, params.seed);
    let before = format!(
        "Broken: {} open edges, {} flipped faces{}.",
        damaged.get_non_manifold_edges().len(),
        flipped,
        if params.soup { ", unwelded" } else { "" }
    );
    let mesh = match params.mode {
        0 => return Ok(Scene::new(damaged, before)),
        1 => damaged.auto_repaired(),
        2 => {
            let options = AdvancedRepairOptions {
                weld_vertices: true,
                weld_tolerance: 0.0,
                face_orientation: params.face_orientation,
                weld_edges: params.weld_edges,
                fill_holes: params.fill_holes,
                remove_mode: RemoveModes::None,
            };
            damaged.advanced_repaired(&options, &CancelToken::new())
        }
        other => return Err(format!("There is no repair mode {other}.")),
    }
    .map_err(|e| format!("The repair failed: {e}"))?;
    let after = if mesh.is_closed_consistently_wound() {
        "Repaired: closed and consistently wound.".to_string()
    } else {
        format!(
            "Repaired: {} open edges left.",
            mesh.get_non_manifold_edges().len()
        )
    };
    Ok(Scene::new(mesh, format!("{before} {after}")))
}
