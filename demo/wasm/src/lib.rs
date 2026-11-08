//! The demo site's WebAssembly bindings: one `#[wasm_bindgen]` function per demo page, each
//! building its scene with the kernel crates and handing it back as a [`MeshData`].
//!
//! The pages run these calls in a web worker (`demo/src/mesh-worker.ts`), which times each call
//! and keeps the last result so the page can ask it for the STL. Every scene builder lives in its
//! own module and returns a plain [`Scene`] (or a `String` error), so the native test
//! `tests/scenes.rs` runs the same code the browser does; this file only converts to and from
//! JavaScript:
//!
//! - `shapes`: the primitives and 2D profiles the pages share, built with the library (a sphere
//!   is a revolved half circle, a cylinder an extruded circle);
//! - `booleans`: union / difference / intersection, clean or messy operands;
//! - `outline`: extrude (square or bevelled) and revolve of editable contours in mm;
//! - `implicit`: signed distance fields with smooth blends, meshed by marching cubes or dual
//!   contouring, and the implicit boolean of meshes;
//! - `morph`: Minkowski sum and difference, convex hull;
//! - `repair`: a mesh broken on purpose, then auto or advanced repair;
//! - `showcase`: the part of the Measure & Export page.

use js_sys::{Float32Array, Uint32Array, Uint8Array};
use polygon_mesh::processors::stl_writer;
use polygon_mesh::processors::MeshOutputSettings;
use polygon_mesh::{CancelToken, Mesh};
use wasm_bindgen::prelude::*;

pub mod booleans;
pub mod implicit;
pub mod morph;
pub mod outline;
pub mod repair;
pub mod shapes;
pub mod showcase;

/// What a scene builder hands back: the result, an optional ghost to draw with it (the operands
/// of a boolean, the source of a hull) and a line of text saying what the library made of it.
pub struct Scene {
    pub mesh: Mesh,
    pub ghost: Option<Mesh>,
    pub note: String,
}

impl Scene {
    /// A scene with no ghost.
    pub fn new(mesh: Mesh, note: impl Into<String>) -> Self {
        Self {
            mesh,
            ghost: None,
            note: note.into(),
        }
    }
}

/// Runs once when the module loads: panic messages (with file and line) go to console.error.
#[wasm_bindgen(start)]
pub fn wasm_start() {
    console_error_panic_hook::set_once();
}

/// The vertex positions of `mesh`, flattened xyz.
fn positions_of(mesh: &Mesh) -> Vec<f32> {
    mesh.vertices.iter().flat_map(|v| [v.x, v.y, v.z]).collect()
}

/// The triangle corners of `mesh`, three vertex indices per face.
fn indices_of(mesh: &Mesh) -> Vec<u32> {
    mesh.faces
        .iter()
        .flat_map(|f| [f.v0 as u32, f.v1 as u32, f.v2 as u32])
        .collect()
}

/// One page's result as the browser sees it: typed arrays for Three.js, the measures the page
/// shows, and the binary STL on request.
#[wasm_bindgen]
pub struct MeshData {
    scene: Scene,
}

#[wasm_bindgen]
impl MeshData {
    #[wasm_bindgen(getter)]
    pub fn positions(&self) -> Float32Array {
        Float32Array::from(&positions_of(&self.scene.mesh)[..])
    }

    #[wasm_bindgen(getter)]
    pub fn indices(&self) -> Uint32Array {
        Uint32Array::from(&indices_of(&self.scene.mesh)[..])
    }

    #[wasm_bindgen(getter)]
    pub fn has_ghost(&self) -> bool {
        self.scene.ghost.is_some()
    }

    #[wasm_bindgen(getter)]
    pub fn ghost_positions(&self) -> Float32Array {
        let ghost = self
            .scene
            .ghost
            .as_ref()
            .map(positions_of)
            .unwrap_or_default();
        Float32Array::from(&ghost[..])
    }

    #[wasm_bindgen(getter)]
    pub fn ghost_indices(&self) -> Uint32Array {
        let ghost = self
            .scene
            .ghost
            .as_ref()
            .map(indices_of)
            .unwrap_or_default();
        Uint32Array::from(&ghost[..])
    }

    #[wasm_bindgen(getter)]
    pub fn num_tri(&self) -> u32 {
        self.scene.mesh.faces.len() as u32
    }

    #[wasm_bindgen(getter)]
    pub fn num_vert(&self) -> u32 {
        self.scene.mesh.vertices.len() as u32
    }

    /// Volume in mm³ (`Mesh::get_volume`).
    #[wasm_bindgen(getter)]
    pub fn volume(&self) -> f64 {
        self.scene.mesh.get_volume()
    }

    /// Surface area in mm² (`Mesh::get_surface_area`).
    #[wasm_bindgen(getter)]
    pub fn surface_area(&self) -> f64 {
        self.scene.mesh.get_surface_area()
    }

    /// Whether the result is a closed, consistently wound solid.
    #[wasm_bindgen(getter)]
    pub fn closed(&self) -> bool {
        self.scene.mesh.is_closed_consistently_wound()
    }

    /// Edges that do not have exactly two faces (holes and seams).
    #[wasm_bindgen(getter)]
    pub fn open_edges(&self) -> u32 {
        self.scene.mesh.get_non_manifold_edges().len() as u32
    }

    #[wasm_bindgen(getter)]
    pub fn note(&self) -> String {
        self.scene.note.clone()
    }

    /// The result as a binary STL, written by the library's `stl_writer`.
    pub fn stl(&self) -> Result<Uint8Array, JsError> {
        let mut bytes = Vec::new();
        stl_writer::save(
            &self.scene.mesh,
            &mut bytes,
            &CancelToken::new(),
            &MeshOutputSettings::new(),
        )
        .map_err(|e| JsError::new(&format!("STL export failed: {e:?}")))?;
        Ok(Uint8Array::from(&bytes[..]))
    }
}

/// A builder's result for JavaScript: the scene, or its error as a thrown `Error`.
fn to_js(result: Result<Scene, String>) -> Result<MeshData, JsError> {
    result
        .map(|scene| MeshData { scene })
        .map_err(|e| JsError::new(&e))
}

/// Boolean page: `op` 0 union, 1 difference, 2 intersection; see [`booleans::boolean_scene`].
#[wasm_bindgen]
#[allow(clippy::too_many_arguments)]
pub fn boolean_scene(
    shape_a: u32,
    shape_b: u32,
    op: u32,
    offset_x: f64,
    offset_y: f64,
    offset_z: f64,
    rotate_deg: f64,
    messy: u32,
    nonzero: bool,
    repair: bool,
) -> Result<MeshData, JsError> {
    to_js(booleans::boolean_scene(&booleans::BooleanParams {
        shape_a,
        shape_b,
        op,
        offset: [offset_x, offset_y, offset_z],
        rotate_deg,
        messy,
        nonzero,
        repair,
    }))
}

/// Outline page, extrude: contours as flat xy pairs in mm (`coords`) split by `counts`.
#[wasm_bindgen]
pub fn extrude_scene(
    coords: &[f64],
    counts: &[u32],
    height: f64,
    top_bevel: f64,
    bottom_bevel: f64,
    bevel_steps: u32,
    round_joins: bool,
) -> Result<MeshData, JsError> {
    let contours = outline::contours_from_flat(coords, counts);
    to_js(outline::extrude_scene(
        &contours,
        height,
        top_bevel,
        bottom_bevel,
        bevel_steps,
        round_joins,
    ))
}

/// Outline page, revolve: the contours (x out from the axis, y along it) swept about z.
#[wasm_bindgen]
pub fn revolve_scene(
    coords: &[f64],
    counts: &[u32],
    steps: u32,
    start_deg: f64,
    end_deg: f64,
) -> Result<MeshData, JsError> {
    let contours = outline::contours_from_flat(coords, counts);
    to_js(outline::revolve_scene(&contours, steps, start_deg, end_deg))
}

/// Implicit page; see [`implicit::implicit_scene`].
#[wasm_bindgen]
pub fn implicit_scene(
    scene: u32,
    blend: f64,
    resolution: u32,
    mesher: u32,
) -> Result<MeshData, JsError> {
    to_js(implicit::implicit_scene(scene, blend, resolution, mesher))
}

/// Minkowski page; see [`morph::minkowski_scene`].
#[wasm_bindgen]
pub fn minkowski_scene(
    solid: u32,
    tool: u32,
    tool_size: f64,
    erode: bool,
    segments: u32,
) -> Result<MeshData, JsError> {
    to_js(morph::minkowski_scene(
        solid, tool, tool_size, erode, segments,
    ))
}

/// Convex hull page; see [`morph::hull_scene`].
#[wasm_bindgen]
pub fn hull_scene(source: u32, count: u32, seed: u32) -> Result<MeshData, JsError> {
    to_js(morph::hull_scene(source, count, seed))
}

/// Repair page; see [`repair::repair_scene`].
#[wasm_bindgen]
#[allow(clippy::too_many_arguments)]
pub fn repair_scene(
    shape: u32,
    holes: u32,
    flips: u32,
    soup: bool,
    seed: u32,
    mode: u32,
    fill_holes: bool,
    face_orientation: bool,
    weld_edges: bool,
) -> Result<MeshData, JsError> {
    to_js(repair::repair_scene(&repair::RepairParams {
        shape,
        holes,
        flips,
        soup,
        seed,
        mode,
        fill_holes,
        face_orientation,
        weld_edges,
    }))
}

/// Measure & Export page; see [`showcase::showcase_part`].
#[wasm_bindgen]
pub fn showcase_scene(
    holes: u32,
    flange: f64,
    blend: f64,
    detail: u32,
) -> Result<MeshData, JsError> {
    to_js(showcase::showcase_part(holes, flange, blend, detail))
}
