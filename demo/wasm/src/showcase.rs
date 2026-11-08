//! The showcase part: one solid that uses most of the library at once, for the Measure & Export
//! page and the README hero image.
//!
//! - a flanged hub, its profile revolved (`revolve_contours`);
//! - a fillet where the neck meets the flange, from a smooth-blended signed distance field meshed
//!   by marching cubes (`implicit`), clipped so only the blend shows above the hub;
//! - a star boss on top, extruded with a rounded top bevel (`extrude_bevelled_contours`);
//! - a bore and a ring of bolt holes, extruded circles, subtracted in one n-ary boolean.
//!
//! The pieces are combined by `boolean_processing::do_array`, so the result is one closed solid
//! whose volume and area the page reports and whose STL it downloads.

use std::f64::consts::TAU;

use polygon_mesh::csg::boolean_processing::do_array;
use polygon_mesh::csg::{BooleanOptions, CsgModes};
use polygon_mesh::processors::outline_to_mesh::revolve_contours;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{AxisAlignedBoundingBox, Matrix4X4, Vector2, Vector3};

use crate::implicit::{mesh_marching_cubes, smooth_min, Field};
use crate::outline::extrude_scene;
use crate::shapes::{cylinder, star};
use crate::Scene;

const FLANGE_THICKNESS: f64 = 6.0;
const NECK_RADIUS: f64 = 10.0;
const NECK_TOP: f64 = 22.0;

/// The hub's profile (x out from the axis, z up), revolved a full turn.
fn hub(flange: f64, segments: u32) -> Result<Mesh, String> {
    let profile = vec![
        Vector2::new(0.0, 0.0),
        Vector2::new(flange, 0.0),
        Vector2::new(flange, FLANGE_THICKNESS - 1.0),
        Vector2::new(flange - 1.0, FLANGE_THICKNESS),
        Vector2::new(NECK_RADIUS, FLANGE_THICKNESS),
        Vector2::new(NECK_RADIUS, NECK_TOP),
        Vector2::new(0.0, NECK_TOP),
    ];
    revolve_contours(&[profile], segments as i32, 0.0, TAU, true)
        .map_err(|e| format!("The hub revolve failed: {e}"))
}

/// The neck-to-flange fillet of radius `blend`: the smooth union of the neck and the flange
/// fields, kept only inside a cylinder whose walls lie within the hub, so what shows outside the
/// hub is the blend alone.
fn fillet(blend: f64, flange: f64) -> Result<Mesh, String> {
    let reach = NECK_RADIUS + blend * 1.5;
    let clip_radius = reach.min(flange - 1.5);
    let clip_top = FLANGE_THICKNESS + blend * 1.5;
    let f = move |p: Vector3| {
        let r = (p.x * p.x + p.y * p.y).sqrt();
        let neck = r - NECK_RADIUS;
        let disk = (r - flange).max(p.z - FLANGE_THICKNESS).max(-p.z);
        let blended = smooth_min(neck, disk, blend * 2.0);
        let clip = (r - clip_radius).max(p.z - clip_top).max(0.5 - p.z);
        blended.max(clip)
    };
    let bounds =
        AxisAlignedBoundingBox::from_coordinates(-reach, -reach, 0.0, reach, reach, clip_top);
    let field = Field {
        f: std::boxed::Box::new(f),
        bounds,
    };
    mesh_marching_cubes(&field, 72, 1.0)
}

/// The showcase part with `holes` bolt holes, a flange `flange` mm in radius, a fillet of `blend`
/// mm (0 for none) and `detail` segments around each curve.
pub fn showcase_part(holes: u32, flange: f64, blend: f64, detail: u32) -> Result<Scene, String> {
    let segments = detail.clamp(12, 128);
    let flange = flange.clamp(NECK_RADIUS + 6.0, 40.0);
    let cancel = CancelToken::new();
    let options = BooleanOptions::default();

    let body = hub(flange, segments)?;
    let boss_profile = star(6, 9.0, 6.5);
    let boss = extrude_scene(&[boss_profile], 6.0, 1.5, 0.0, 4, true)?.mesh;
    let lift = Matrix4X4::create_translation(0.0, 0.0, NECK_TOP - 0.5);
    let fillet_mesh = if blend > 0.05 {
        Some(fillet(blend, flange)?)
    } else {
        None
    };
    let mut adds: Vec<(&Mesh, Matrix4X4)> = vec![(&body, Matrix4X4::IDENTITY), (&boss, lift)];
    if let Some(f) = &fillet_mesh {
        adds.push((f, Matrix4X4::IDENTITY));
    }
    let joined = do_array(&adds, CsgModes::Union, &cancel, &options)
        .map_err(|e| format!("The union failed: {e}"))?;

    let height = NECK_TOP + 10.0;
    let bore = cylinder(4.0, height + 4.0, segments)?;
    let bolt = cylinder(2.2, FLANGE_THICKNESS * 3.0, (segments / 2).max(12))?;
    let bolt_ring = (NECK_RADIUS + blend.max(0.0) * 1.2 + flange) / 2.0 + 1.0;
    let mut cuts: Vec<(&Mesh, Matrix4X4)> = vec![
        (&joined, Matrix4X4::IDENTITY),
        (&bore, Matrix4X4::create_translation(0.0, 0.0, height / 2.0)),
    ];
    let bolt_places: Vec<Matrix4X4> = (0..holes.min(16))
        .map(|i| {
            let a = TAU * f64::from(i) / f64::from(holes.max(1));
            Matrix4X4::create_translation(bolt_ring * a.cos(), bolt_ring * a.sin(), 0.0)
        })
        .collect();
    for place in &bolt_places {
        cuts.push((&bolt, *place));
    }
    let mesh = do_array(&cuts, CsgModes::Subtract, &cancel, &options)
        .map_err(|e| format!("The subtract failed: {e}"))?;
    let note = format!(
        "Revolved hub + bevelled star boss{} - bore and {} bolt holes.",
        if fillet_mesh.is_some() {
            " + SDF fillet"
        } else {
            ""
        },
        bolt_places.len()
    );
    Ok(Scene::new(mesh, note))
}
