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
//! - `ray_trace_scene`, `ray_trace_shade`, `ray_trace`: the Ray Tracing page - a part on a floor,
//!   its material and light presets over the library's path tracer, and the per-pixel
//!   accumulation the worker refines in short passes (this page draws on a 2D canvas, so it
//!   answers an image rather than a [`MeshData`]);
//! - `showcase`: the part of the Measure & Export page;
//! - `fillet_parts`, `fillet_pick`, `fillet`: the Fillet & Chamfer page - its parts, picking on
//!   them in wasm, and the picks applied at one radius in one `solidmesh::bevel_part` call;
//!   `fillet_words*` word the kernel's refusal codes in English and `fillet_json` writes the
//!   picks and rows the page reads.

use js_sys::{Float32Array, Object, Reflect, Uint32Array, Uint8Array};
use solidmesh::mesh::processors::stl_writer;
use solidmesh::mesh::processors::MeshOutputSettings;
use solidmesh::mesh::{CancelToken, Mesh};
use wasm_bindgen::prelude::*;

pub mod booleans;
pub mod fillet;
pub mod fillet_json;
pub mod fillet_parts;
pub mod fillet_pick;
pub mod fillet_words;
pub mod fillet_words_blend;
pub mod fillet_words_parts;
pub mod implicit;
pub mod morph;
pub mod outline;
pub mod ray_trace;
pub mod ray_trace_scene;
pub mod ray_trace_shade;
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
    /// A page's own answer beside the mesh, as JSON (the fillet page's rows); empty elsewhere.
    data: String,
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
    pub fn data(&self) -> String {
        self.data.clone()
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
        .map(|scene| MeshData {
            scene,
            data: String::new(),
        })
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

thread_local! {
    /// The part the pick worker last prepared, by index, ready to pick on.
    static PICKING: std::cell::RefCell<Option<(u32, fillet_pick::PickSession)>> =
        const { std::cell::RefCell::new(None) };
}

thread_local! {
    /// The part the Ray Tracing page last prepared and its image so far.
    static RAY_TRACE: std::cell::RefCell<Option<ray_trace::Session>> =
        const { std::cell::RefCell::new(None) };
}

/// Ray Tracing page: builds part `part` at detail `detail` and the tree and surface table over
/// it on this worker, and answers its triangle count and how long each took as JSON. A tree over
/// a million triangles takes seconds, which is why it is built here and not on the page.
#[wasm_bindgen]
pub fn ray_trace_prepare(part: u32, detail: u32) -> Result<String, JsError> {
    // The old part goes first, so its tree and the new one are never in memory at once.
    RAY_TRACE.with(|session| *session.borrow_mut() = None);
    let start = js_sys::Date::now();
    let (mesh, note) = ray_trace_scene::part_mesh(part, detail).map_err(|e| JsError::new(&e))?;
    let meshed = js_sys::Date::now();
    let scene = ray_trace_scene::TraceScene::new(&mesh, note).map_err(|e| JsError::new(&e))?;
    let finished = js_sys::Date::now();
    let json = format!(
        "{{\"part\":{part},\"detail\":{detail},\"triangles\":{},\"mesh_ms\":{},\"bvh_ms\":{},\"note\":{:?}}}",
        scene.triangles,
        meshed - start,
        finished - meshed,
        scene.note,
    );
    RAY_TRACE.with(|session| *session.borrow_mut() = Some(ray_trace::Session::new(scene)));
    Ok(json)
}

/// Ray Tracing page: one short pass on the prepared part (see `ray_trace`), answering the image
/// so far and what the pass did as a plain object the worker can hand to the page. `budget_ms` 0
/// finishes the sample under way instead of watching the clock. `material`, `light` and the color
/// are the page's controls (`ray_trace_shade`); a different material or light refines from scratch
/// and never rebuilds the part.
#[wasm_bindgen]
#[allow(clippy::too_many_arguments)]
pub fn ray_trace_pass(
    yaw: f64,
    pitch: f64,
    zoom: f64,
    mode: u32,
    material: u32,
    red: u8,
    green: u8,
    blue: u8,
    roughness: f64,
    metallic: f64,
    transmission: f64,
    light: u32,
    sun_degrees: f64,
    seed: u32,
    width: u32,
    height: u32,
    preview: bool,
    budget_ms: f64,
) -> Result<Object, JsError> {
    use ray_trace_shade::{CustomSliders, Light, Look, Material, Mode};
    let mode = Mode::from_index(mode).map_err(|e| JsError::new(&e))?;
    let look = Look {
        material: Material::from_index(material).map_err(|e| JsError::new(&e))?,
        color: [red, green, blue],
        custom: CustomSliders {
            roughness,
            metallic,
            transmission,
        },
        light: Light::from_index(light).map_err(|e| JsError::new(&e))?,
        sun_degrees,
    };
    RAY_TRACE.with(|session| {
        let mut session = session.borrow_mut();
        let session = session
            .as_mut()
            .ok_or_else(|| JsError::new("That part is not ready to trace yet."))?;
        let request = ray_trace::TraceRequest {
            yaw,
            pitch,
            zoom,
            mode,
            look,
            seed,
            width,
            height,
            preview,
        };
        let start = js_sys::Date::now();
        let report = session
            .accumulation
            .pass(&session.scene, request, budget_ms, &mut js_sys::Date::now)
            .map_err(|e| JsError::new(&e))?;
        let ms = js_sys::Date::now() - start;
        let result = Object::new();
        let pixels = Uint8Array::from(session.accumulation.pixels());
        let fields: [(&str, JsValue); 7] = [
            ("pixels", pixels.into()),
            ("width", session.accumulation.width().into()),
            ("height", session.accumulation.height().into()),
            ("samples", report.samples.into()),
            // A double holds any count of rays or pixels one pass could fire exactly.
            ("rays", (report.rays as f64).into()),
            ("pixelSamples", (report.pixels as f64).into()),
            ("ms", ms.into()),
        ];
        for (name, value) in fields {
            Reflect::set(&result, &name.into(), &value)
                .map_err(|_| JsError::new("The traced image could not be handed to the page."))?;
        }
        Ok(result)
    })
}

/// Fillet & Chamfer page: reads an STL as the upload part (`fillet_parts::UPLOAD`) on this
/// worker; answers its triangle count.
#[wasm_bindgen]
pub fn fillet_upload(bytes: &[u8]) -> Result<u32, JsError> {
    PICKING.with(|picking| {
        if picking
            .borrow()
            .as_ref()
            .is_some_and(|(at, _)| *at == fillet_parts::UPLOAD)
        {
            *picking.borrow_mut() = None;
        }
    });
    fillet_parts::set_upload(bytes)
        .map(|count| count as u32)
        .map_err(|e| JsError::new(&e))
}

/// Fillet & Chamfer page: prepares part `part` for picking on this worker and answers its feature
/// edges and corners as JSON (`fillet_json::prepared`).
#[wasm_bindgen]
pub fn fillet_prepare(part: u32) -> Result<String, JsError> {
    let mesh = fillet_parts::part(part).map_err(|e| JsError::new(&e))?;
    let session = fillet_pick::PickSession::new(&mesh);
    let json = fillet_json::prepared(&session);
    PICKING.with(|picking| *picking.borrow_mut() = Some((part, session)));
    Ok(json)
}

/// Fillet & Chamfer page: what a click along the ray (origin and direction in the part's space)
/// would pick on the prepared part in `mode` (0 chain, 1 edge, 2 face rim, 3 point), as JSON
/// (`fillet_json::pick`). `pixel_size` is the mm one pixel spans at 1 mm from the eye.
#[wasm_bindgen]
#[allow(clippy::too_many_arguments)]
pub fn fillet_pick(
    part: u32,
    ox: f64,
    oy: f64,
    oz: f64,
    dx: f64,
    dy: f64,
    dz: f64,
    mode: u32,
    pixel_size: f64,
    round: bool,
) -> Result<String, JsError> {
    let mode = fillet_pick::PickMode::from_index(mode).map_err(|e| JsError::new(&e))?;
    PICKING.with(|picking| {
        let picking = picking.borrow();
        let Some((_, session)) = picking.as_ref().filter(|(at, _)| *at == part) else {
            return Err(JsError::new("That part is not prepared for picking yet."));
        };
        let pick = session
            .pick(
                solidmesh::math::Vector3::new(ox, oy, oz),
                solidmesh::math::Vector3::new(dx, dy, dz),
                mode,
                pixel_size,
                round,
            )
            .map_err(|e| JsError::new(&e))?;
        Ok(fillet_json::pick(pick.as_ref()))
    })
}

/// Fillet & Chamfer page: part `part` with every pick applied at `size` mm (`fillet::fillet_part`).
/// `picks` is each pick's group as `fillet_pick::GROUP_NUMBERS` numbers; rounds when `round`,
/// chamfers otherwise. The rows come back as JSON in `data` (`fillet_json::rows`).
///
/// The `CancelToken` is never cancelled: the worker runs one wasm call to its end before it reads
/// its next message, so nothing could reach the token while the build runs (that would need
/// shared memory, which GitHub Pages does not allow). A superseded drag is instead dropped before
/// it starts, by the page's request coalescing.
#[wasm_bindgen]
pub fn fillet_scene(
    part: u32,
    picks: &[f64],
    size: f64,
    round: bool,
    segments: u32,
) -> Result<MeshData, JsError> {
    let source = fillet_parts::part(part).map_err(|e| JsError::new(&e))?;
    let stride = fillet_pick::GROUP_NUMBERS;
    if !picks.len().is_multiple_of(stride) {
        return Err(JsError::new("The picks are not whole."));
    }
    let picks = picks
        .chunks(stride)
        .map(fillet_pick::group_from_numbers)
        .collect::<Result<Vec<_>, String>>()
        .map_err(|e| JsError::new(&e))?;
    let build = fillet::fillet_part(
        &source,
        &picks,
        size,
        round,
        segments.clamp(1, 32) as i32,
        &CancelToken::new(),
    )
    .map_err(|e| JsError::new(&e))?;
    Ok(MeshData {
        data: fillet_json::rows(&build.rows),
        scene: build.scene,
    })
}
