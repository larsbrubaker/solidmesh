//! The parts the Fillet & Chamfer page offers, every one built with the library: an L-bracket (an
//! L outline extruded and stood up), a plate with a rounded pocket and four through holes (one
//! n-ary boolean), a revolved hub with a flange and a bore, the Measure & Export showcase part
//! without its implicit fillet (so its edges are the library's own to round), a box, and an
//! uploaded STL.
//!
//! The pick worker and the build worker each build the part from its index, so both hold the same
//! mesh without sending it between them; an upload is handed to both and kept here per thread.
//! Every part sits on the z = 0 plane, centred in x and y, in mm.

use std::cell::RefCell;
use std::f64::consts::{FRAC_PI_2, TAU};
use std::sync::Arc;

use solidmesh::math::{Matrix4X4, Vector2, Vector3};
use solidmesh::mesh::csg::boolean_processing::do_array;
use solidmesh::mesh::csg::{BooleanOptions, CsgModes};
use solidmesh::mesh::processors::outline_to_mesh::{extrude_contours, revolve_contours};
use solidmesh::mesh::processors::stl_processing;
use solidmesh::mesh::solids::platonic_solids::create_cube;
use solidmesh::mesh::{CancelToken, Mesh};

use crate::shapes::{cylinder, placed};
use crate::showcase::showcase_part;

/// The menu index of an uploaded STL.
pub const UPLOAD: u32 = 5;

thread_local! {
    /// The last uploaded STL, already welded and placed on the plate.
    static UPLOADED: RefCell<Option<Arc<Mesh>>> = const { RefCell::new(None) };
    /// The last part built, by index, so a slider drag does not rebuild it.
    static BUILT: RefCell<Option<(u32, Arc<Mesh>)>> = const { RefCell::new(None) };
}

/// `mesh` moved so it is centred in x and y and stands on z = 0.
pub fn on_the_plate(mut mesh: Mesh) -> Mesh {
    let bounds = mesh.get_axis_aligned_bounding_box();
    let center = bounds.center();
    mesh.translate(Vector3::new(-center.x, -center.y, -bounds.min_xyz.z));
    mesh
}

/// An L-bracket: a 50 mm base and a 40 mm upright, both 8 mm thick, 30 mm deep. Its outer edges
/// are convex and the edge inside the L is concave.
pub fn l_bracket() -> Result<Mesh, String> {
    let l = vec![
        Vector2::new(0.0, 0.0),
        Vector2::new(50.0, 0.0),
        Vector2::new(50.0, 8.0),
        Vector2::new(8.0, 8.0),
        Vector2::new(8.0, 40.0),
        Vector2::new(0.0, 40.0),
    ];
    let mut mesh = extrude_contours(&[l], 30.0).map_err(|e| format!("extrude failed: {e}"))?;
    // the outline was drawn in x and z; stand it up so its 30 mm run along y
    mesh.transform(Matrix4X4::create_rotation_x(FRAC_PI_2));
    Ok(on_the_plate(mesh))
}

/// A rounded rectangle `w` by `h` with corners of `r`, `per_corner` segments each,
/// counter-clockwise about the origin.
fn rounded_rectangle(w: f64, h: f64, r: f64, per_corner: u32) -> Vec<Vector2> {
    let (hx, hy) = (w / 2.0 - r, h / 2.0 - r);
    let centers = [(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)];
    let mut points = Vec::new();
    for (k, (cx, cy)) in centers.iter().enumerate() {
        for i in 0..=per_corner {
            let a = TAU / 4.0 * (k as f64 + f64::from(i) / f64::from(per_corner));
            points.push(Vector2::new(cx + r * a.cos(), cy + r * a.sin()));
        }
    }
    points
}

/// A 70 x 46 x 10 mm plate with a 36 x 22 mm pocket 5 mm deep, its corners rounded so the
/// pocket's rim is one smooth loop, and a 6 mm through hole near each corner.
pub fn pocket_plate() -> Result<Mesh, String> {
    let cancel = CancelToken::new();
    let options = BooleanOptions::default();
    let plate = placed(
        &create_cube(70.0, 46.0, 10.0),
        Matrix4X4::create_translation(0.0, 0.0, 5.0),
    );
    let pocket = extrude_contours(&[rounded_rectangle(36.0, 22.0, 5.0, 8)], 10.0)
        .map_err(|e| format!("extrude failed: {e}"))?;
    let hole = cylinder(3.0, 30.0, 32)?;
    let mut cuts: Vec<(&Mesh, Matrix4X4)> = vec![
        (&plate, Matrix4X4::IDENTITY),
        (&pocket, Matrix4X4::create_translation(0.0, 0.0, 5.0)),
    ];
    for (x, y) in [(28.0, 16.0), (-28.0, 16.0), (-28.0, -16.0), (28.0, -16.0)] {
        cuts.push((&hole, Matrix4X4::create_translation(x, y, 5.0)));
    }
    let mesh = do_array(&cuts, CsgModes::Subtract, &cancel, &options)
        .map_err(|e| format!("The pocket cut failed: {e}"))?;
    Ok(on_the_plate(mesh))
}

/// A hub: a flange 30 mm in radius and 8 mm thick, a neck 14 mm in radius up to 26 mm, and a
/// 6 mm bore, its profile revolved in 48 steps.
pub fn hub() -> Result<Mesh, String> {
    let profile = vec![
        Vector2::new(6.0, 0.0),
        Vector2::new(30.0, 0.0),
        Vector2::new(30.0, 8.0),
        Vector2::new(14.0, 8.0),
        Vector2::new(14.0, 26.0),
        Vector2::new(6.0, 26.0),
    ];
    let mesh = revolve_contours(&[profile], 48, 0.0, TAU, true)
        .map_err(|e| format!("The hub revolve failed: {e}"))?;
    Ok(on_the_plate(mesh))
}

/// The part `index` names: 0 L-bracket, 1 pocket plate, 2 hub, 3 the Measure & Export part, 4 a
/// box, 5 the uploaded STL. Built once per thread and kept until another index is asked for.
pub fn part(index: u32) -> Result<Arc<Mesh>, String> {
    if let Some(mesh) = BUILT.with(|built| {
        built
            .borrow()
            .as_ref()
            .filter(|(at, _)| *at == index)
            .map(|(_, mesh)| Arc::clone(mesh))
    }) {
        return Ok(mesh);
    }
    let mesh = match index {
        0 => Arc::new(l_bracket()?),
        1 => Arc::new(pocket_plate()?),
        2 => Arc::new(hub()?),
        // without its implicit fillet: its rounds are this page's to make
        3 => Arc::new(on_the_plate(showcase_part(6, 26.0, 0.0, 48)?.mesh)),
        4 => Arc::new(on_the_plate(create_cube(40.0, 30.0, 20.0))),
        UPLOAD => UPLOADED
            .with(|uploaded| uploaded.borrow().clone())
            .ok_or_else(|| "Upload an STL first.".to_string())?,
        _ => return Err(format!("There is no part {index}.")),
    };
    BUILT.with(|built| *built.borrow_mut() = Some((index, Arc::clone(&mesh))));
    Ok(mesh)
}

/// Reads an STL as the upload part, placed on the plate, and forgets any part built from an
/// earlier upload. Answers the triangle count.
pub fn set_upload(bytes: &[u8]) -> Result<usize, String> {
    let mesh = stl_processing::parse_file_contents(bytes, &CancelToken::new(), None)
        .map_err(|e| format!("That STL could not be read: {e:?}"))?;
    if mesh.faces.is_empty() {
        return Err("That STL has no triangles.".to_string());
    }
    let mesh = on_the_plate(mesh);
    let count = mesh.faces.len();
    UPLOADED.with(|uploaded| *uploaded.borrow_mut() = Some(Arc::new(mesh)));
    BUILT.with(|built| {
        if built.borrow().as_ref().is_some_and(|(at, _)| *at == UPLOAD) {
            *built.borrow_mut() = None;
        }
    });
    Ok(count)
}
