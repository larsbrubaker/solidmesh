//! The Ray Tracing page's scene: a part made by the library standing on a checkered floor, each
//! with its ray tracing tree and surface table (`ray_tracer::BoundingVolumeHierarchy` over one
//! `MinimalTriangle` per face, as MatterCAD builds it and as the Fillet page's picking does in
//! `fillet_pick::build_bvh`), and the orbit [`Camera`] that turns a pixel into a ray.
//!
//! The trees and tables are built once and shared (`Arc`), so a new material or light is a new
//! `ray_tracer::RenderScene` over the same geometry, built by [`TraceScene::render_scene`] with no
//! rebuild. The picture itself is the library's `PathTracer` (see `ray_trace`).
//!
//! The floor is a real mesh, so the library's path tracer lights it like anything else. It is a
//! grid of checker squares whose face colors the library reads as display colors, with a wide
//! plain border carrying the floor out toward the horizon. The grid has no T-junctions, so no ray
//! can slip through between two of its faces.
//!
//! The world is z-up, as the parts are modeled. The floor sits a whisker under the part so the
//! flat bottom of a part never coincides with it.

use std::sync::Arc;

use solidmesh::math::{Color, Matrix4X4, Ray, Vector3};
use solidmesh::mesh::csg::boolean_processing::do_boolean;
use solidmesh::mesh::csg::{BooleanOptions, CsgModes};
use solidmesh::mesh::ray_tracer::{
    MeshSurface, PartHit, PlacedPart, PlacedParts, RenderCamera, RenderScene, ScatteringSurface,
    TraceableRef,
};
use solidmesh::mesh::{CancelToken, Mesh};

use crate::fillet_pick::build_bvh;
use crate::implicit::{blob, mesh_marching_cubes};
use crate::ray_trace_shade::Look;
use crate::shapes::Shape;

/// The triangle counts the page's Detail menu offers, by the index it sends.
pub const DETAIL_TRIANGLES: [f64; 5] = [10_000.0, 30_000.0, 100_000.0, 300_000.0, 1_000_000.0];

/// The fineness each part is first built at to learn how its triangle count grows.
const PROBE_KNOB: u32 = 64;

/// The operand size of the boolean and the rounded box, in mm.
const PART_SIZE: f64 = 40.0;

/// Clearance under the part as a fraction of its radius, so the ground never coincides with a
/// flat bottom face.
const GROUND_GAP: f64 = 0.001;

/// The floor's index among the scene's placed parts.
pub const GROUND_PART: usize = 1;

/// The width of one checker square, as a fraction of the part's radius.
const CHECKER_SQUARE_RADII: f64 = 0.65;

/// How many checker squares the floor has from its middle to its edge.
const CHECKER_SQUARES: i32 = 20;

/// How far the plain floor reaches, in part radii, far enough that it fills the view to the
/// horizon from any orbit.
const FLOOR_REACH_RADII: f64 = 120.0;

/// The checker's two display colors, close enough that a shadow still reads as a shadow.
const FLOOR_LIGHT: Color = Color::rgb(208, 205, 198);
const FLOOR_DARK: Color = Color::rgb(170, 167, 162);

/// The plain border's color: between the two, so the checker fades into it.
const FLOOR_PLAIN: Color = Color::rgb(188, 185, 180);

/// Edges bent at least this much stay sharp in the smooth shading (the corners of a box); curved
/// surfaces at any detail the menu offers bend less than this between neighbouring faces.
const CREASE_DEGREES: f64 = 38.0;

/// How far from the part's centre the camera starts, in part radii.
const ORBIT_RADII: f64 = 2.9;

/// The field of view across the image's shorter side, in degrees, so the part fits whichever way
/// round the image is.
const FIELD_OF_VIEW_DEGREES: f64 = 38.0;

/// The orbit stays between just above the ground and just short of straight overhead, where the
/// view's "up" would be undefined.
const MIN_PITCH_DEGREES: f64 = 3.0;
const MAX_PITCH_DEGREES: f64 = 87.0;

/// The solids the page offers, by the index its menu sends.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Part {
    /// A rounded box with a ball carved out of it.
    Boolean,
    Torus,
    RoundedBox,
    /// The Implicit page's smooth blend of a sphere, a box and a cylinder.
    Blend,
}

impl Part {
    /// The part a menu index names, or an error naming the index.
    pub fn from_index(index: u32) -> Result<Part, String> {
        Ok(match index {
            0 => Part::Boolean,
            1 => Part::Torus,
            2 => Part::RoundedBox,
            3 => Part::Blend,
            _ => return Err(format!("There is no part {index} on this page.")),
        })
    }

    /// What the part is, for the note under the readout.
    fn describe(self) -> &'static str {
        match self {
            Part::Boolean => "A rounded box with a ball subtracted by the boolean.",
            Part::Torus => "A torus: a circle revolved about z.",
            Part::RoundedBox => "A rounded box.",
            Part::Blend => "A smooth blend of three distance fields, meshed by marching cubes.",
        }
    }

    /// The most fineness the part accepts, kept to what a browser tab can hold.
    fn max_knob(self) -> u32 {
        match self {
            Part::Blend => 700,
            Part::RoundedBox | Part::Boolean | Part::Torus => 2000,
        }
    }

    /// The solid at fineness `knob`: segments around a curve, or cells across for the blend.
    pub fn build(self, knob: u32) -> Result<Mesh, String> {
        match self {
            Part::Boolean => boolean_part(knob),
            Part::Torus => Shape::Torus.build(PART_SIZE * 1.5, knob),
            Part::RoundedBox => Shape::RoundedBox.build(PART_SIZE, knob),
            Part::Blend => mesh_marching_cubes(&blob(4.0), knob, 6.0),
        }
    }

    /// The fineness that gives this part about `target` triangles: a surface's triangle count
    /// goes as the square of its fineness, so one small probe build calibrates the whole range.
    pub fn knob_for(self, target: f64) -> Result<u32, String> {
        let probe = self.build(PROBE_KNOB)?;
        let probed = probe.faces.len().max(1) as f64;
        let knob = f64::from(PROBE_KNOB) * (target / probed).sqrt();
        // `as` saturates; `target` is one of DETAIL_TRIANGLES, so this is a few thousand at most.
        Ok((knob.round() as u32).clamp(8, self.max_knob()))
    }
}

/// A rounded box with a ball carved out of one corner, both at `segments`.
fn boolean_part(segments: u32) -> Result<Mesh, String> {
    let block = Shape::RoundedBox.build(PART_SIZE, segments)?;
    let ball = Shape::Sphere.build(PART_SIZE * 0.9, segments)?;
    let place = Matrix4X4::create_translation(PART_SIZE * 0.35, -PART_SIZE * 0.25, PART_SIZE * 0.4);
    do_boolean(
        &block,
        Matrix4X4::IDENTITY,
        &ball,
        place,
        CsgModes::Subtract,
        &CancelToken::new(),
        &BooleanOptions::default(),
    )
    .map_err(|e| format!("The boolean could not build this part: {e}"))
}

/// Part `part_index` at detail `detail_index`, and what it is for the note.
pub fn part_mesh(part_index: u32, detail_index: u32) -> Result<(Mesh, String), String> {
    let part = Part::from_index(part_index)?;
    let target = *DETAIL_TRIANGLES
        .get(detail_index as usize)
        .ok_or_else(|| format!("There is no detail level {detail_index} on this page."))?;
    let knob = part.knob_for(target)?;
    let mut mesh = part.build(knob)?;
    let built = mesh.faces.len() as f64;
    // A surface can come back coarser than its fineness asks for (a revolve drops nearly
    // collinear contour points), so a part well short of the menu's promise gets one correction.
    if built < target * 0.7 {
        let finer = (f64::from(knob) * target / built.max(1.0)).round() as u32;
        mesh = part.build(finer.clamp(8, part.max_knob()))?;
    }
    Ok((mesh, part.describe().to_string()))
}

/// What a ray found in the scene.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Surface {
    Part,
    Ground,
    Sky,
}

/// One piece of geometry ready to place: its tree and its surface table, shared.
struct Solid {
    tree: TraceableRef,
    surface: Arc<MeshSurface>,
}

impl Solid {
    /// The tree and table for `mesh`, with edges bent `crease` radians or more kept sharp.
    fn new(mesh: &Mesh, crease: f64, empty: &str) -> Result<Solid, String> {
        let tree = build_bvh(mesh).ok_or_else(|| empty.to_string())?;
        let surface = MeshSurface::new(mesh, crease).map_err(|e| e.to_string())?;
        Ok(Solid {
            tree,
            surface: Arc::new(surface),
        })
    }

    /// This solid in the world, sharing its tree and table.
    fn placed(&self) -> Result<PlacedPart, String> {
        PlacedPart::new(self.tree.clone(), self.surface.clone(), Matrix4X4::IDENTITY)
            .map_err(|e| e.to_string())
    }
}

/// The floor under a part: a grid of checker squares around `center`, widened to reach the
/// horizon. Faces come in pairs, one pair per grid cell, each with the cell's color.
fn floor_mesh(center: Vector3, ground_z: f64, radius: f64) -> Mesh {
    let square = checker_square(radius);
    let lines = |around: f64| -> Vec<f64> {
        // `as` saturates; a part's size keeps this index small.
        let middle = (around / square).round() as i32;
        let reach = radius * FLOOR_REACH_RADII;
        let mut values = vec![around - reach];
        values.extend(
            (middle - CHECKER_SQUARES..=middle + CHECKER_SQUARES).map(|k| f64::from(k) * square),
        );
        values.push(around + reach);
        values
    };
    let xs = lines(center.x);
    let ys = lines(center.y);

    let mut corners = Vec::with_capacity(xs.len() * ys.len());
    for &y in &ys {
        for &x in &xs {
            corners.push(Vector3::new(x, y, ground_z));
        }
    }
    let index = |column: usize, row: usize| (row * xs.len() + column) as i32;
    let mut faces = Vec::with_capacity((xs.len() - 1) * (ys.len() - 1) * 6);
    let mut colors = Vec::with_capacity((xs.len() - 1) * (ys.len() - 1) * 2);
    for row in 0..ys.len() - 1 {
        for column in 0..xs.len() - 1 {
            let (a, b, c, d) = (
                index(column, row),
                index(column + 1, row),
                index(column + 1, row + 1),
                index(column, row + 1),
            );
            faces.extend_from_slice(&[a, b, c, a, c, d]);
            let border = column == 0 || row == 0 || column == xs.len() - 2 || row == ys.len() - 2;
            let color = if border {
                FLOOR_PLAIN
            } else if square_is_light(xs[column] + square * 0.5, ys[row] + square * 0.5, radius) {
                FLOOR_LIGHT
            } else {
                FLOOR_DARK
            };
            colors.extend_from_slice(&[color, color]);
        }
    }
    let mut mesh = Mesh::from_vector3_list_and_indices(&corners, &faces);
    mesh.face_colors = Some(colors);
    mesh
}

/// The width of one checker square for a part of this `radius`.
fn checker_square(radius: f64) -> f64 {
    radius * CHECKER_SQUARE_RADII
}

/// True when the checker square containing (`x`, `y`) is one of the light ones, for a part of
/// this `radius`.
fn square_is_light(x: f64, y: f64, radius: f64) -> bool {
    let square = checker_square(radius);
    // `as` saturates; these are coordinates in the part's own range.
    let cell = |value: f64| (value / square).floor() as i64;
    (cell(x) + cell(y)).rem_euclid(2) == 0
}

/// The floor under a part and the numbers its camera orbits with: shared by this page's scene and
/// the GPU Ray Tracing page's, which traces the same part and floor on a flat tree instead.
pub struct PlacedOnFloor {
    pub floor: Mesh,
    pub center: Vector3,
    /// The part's bounding radius.
    pub radius: f64,
    pub ground_z: f64,
}

impl PlacedOnFloor {
    /// The floor under `mesh`, a whisker below it.
    pub fn around(mesh: &Mesh) -> PlacedOnFloor {
        let bounds = mesh.get_axis_aligned_bounding_box();
        let radius = bounds.size().length() * 0.5;
        let ground_z = bounds.min_xyz.z - radius * GROUND_GAP;
        let center = bounds.center();
        PlacedOnFloor {
            floor: floor_mesh(center, ground_z, radius),
            center,
            radius,
            ground_z,
        }
    }
}

/// A part on its floor, ready to trace in any look.
pub struct TraceScene {
    solid: Solid,
    floor: Solid,
    /// The two together under no look at all, for what the data views read.
    data: PlacedParts,
    /// Where the camera orbits.
    pub center: Vector3,
    /// The part's bounding radius.
    pub radius: f64,
    pub ground_z: f64,
    pub triangles: usize,
    pub note: String,
}

impl TraceScene {
    /// Builds the trees and surface tables over `mesh` and its floor. It can take seconds on a
    /// fine part, which is why the page runs it on its worker.
    pub fn new(mesh: &Mesh, note: String) -> Result<TraceScene, String> {
        let crease = CREASE_DEGREES.to_radians();
        let solid = Solid::new(mesh, crease, "That part has no faces to trace.")?;
        let placed = PlacedOnFloor::around(mesh);
        let (center, radius, ground_z) = (placed.center, placed.radius, placed.ground_z);
        let floor = Solid::new(&placed.floor, crease, "The floor has no faces to trace.")?;
        let data = PlacedParts::new(vec![solid.placed()?, floor.placed()?]);
        Ok(TraceScene {
            solid,
            floor,
            data,
            center,
            radius,
            ground_z,
            triangles: mesh.faces.len(),
            note,
        })
    }

    /// A render of this part and floor in `look`, sharing the trees and tables already built.
    pub fn render_scene(&self, look: &Look) -> Result<RenderScene, String> {
        let surfaces = self.surfaces(look);
        let lights = look.lights().map_err(|e| e.to_string())?;
        let parts = PlacedParts::new(vec![self.solid.placed()?, self.floor.placed()?]);
        RenderScene::new(parts, surfaces, lights).map_err(|e| e.to_string())
    }

    /// The surfaces of the placed parts, in part order: what a look gives the part and the floor.
    pub fn surfaces(&self, look: &Look) -> Vec<ScatteringSurface> {
        vec![look.surface(), ScatteringSurface::matte_plastic([1.0; 3])]
    }

    /// The nearest hit of `ray`, for the data views.
    pub fn hit<'a>(&'a self, ray: &mut Ray) -> Option<PartHit<'a>> {
        self.data.trace(ray)
    }

    /// True when the floor's checker square at (`x`, `y`) is one of the light ones.
    pub fn floor_is_light(&self, x: f64, y: f64) -> bool {
        square_is_light(x, y, self.radius)
    }

    /// The width of one of the floor's checker squares.
    pub fn floor_square(&self) -> f64 {
        checker_square(self.radius)
    }

    /// What the ray through pixel position (`px`, `py`) finds, and how far away.
    pub fn surface_at(&self, camera: &Camera, px: f64, py: f64) -> (Surface, f64) {
        let mut ray = camera.ray(px, py);
        let Some(hit) = self.hit(&mut ray) else {
            return (Surface::Sky, f64::INFINITY);
        };
        let surface = if hit.part.index() == GROUND_PART {
            Surface::Ground
        } else {
            Surface::Part
        };
        (surface, hit.info.distance_to_hit)
    }
}

/// An orbit camera: `yaw` about z and `pitch` above the horizon (radians) around the part's
/// centre, `zoom` times closer than the starting distance, for an image `width` x `height` pixels.
///
/// The rays are the library's [`RenderCamera`]; this keeps the orbit, and the projection that
/// finds the pixel a world point falls on. The field of view spans the shorter image side, as the
/// library's camera has it, so the part fits whichever way round the image is.
#[derive(Debug, Clone, Copy)]
pub struct Camera {
    pub eye: Vector3,
    forward: Vector3,
    right: Vector3,
    up: Vector3,
    /// Tangent of half the field of view, which spans the shorter side.
    tan_half: f64,
    /// How much wider than `tan_half` the view is across the image: 1 on the shorter side, and the
    /// aspect ratio on the longer one.
    x_span: f64,
    y_span: f64,
    render: RenderCamera,
    width: u32,
    height: u32,
}

impl Camera {
    /// The camera for this view of `scene`; an orbit the library cannot build a camera from (a
    /// not-finite angle) is an error.
    pub fn new(
        scene: &TraceScene,
        yaw: f64,
        pitch: f64,
        zoom: f64,
        width: u32,
        height: u32,
    ) -> Result<Camera, String> {
        Camera::around(scene.center, scene.radius, yaw, pitch, zoom, width, height)
    }

    /// The orbit camera around `center` for a part of bounding `radius`, without needing a
    /// [`TraceScene`] built for it.
    pub fn around(
        center: Vector3,
        radius: f64,
        yaw: f64,
        pitch: f64,
        zoom: f64,
        width: u32,
        height: u32,
    ) -> Result<Camera, String> {
        let pitch = pitch.clamp(
            MIN_PITCH_DEGREES.to_radians(),
            MAX_PITCH_DEGREES.to_radians(),
        );
        let distance = radius * ORBIT_RADII / zoom.clamp(0.4, 8.0);
        let offset = Vector3::new(
            pitch.cos() * yaw.cos(),
            pitch.cos() * yaw.sin(),
            pitch.sin(),
        );
        let eye = center + offset * distance;
        let render = RenderCamera::perspective(eye, center, Vector3::UNIT_Z, FIELD_OF_VIEW_DEGREES)
            .map_err(|e| e.to_string())?;
        let forward = (center - eye).get_normal();
        let right = forward.cross(Vector3::UNIT_Z).get_normal();
        Ok(Camera {
            eye,
            forward,
            right,
            up: right.cross(forward),
            tan_half: (FIELD_OF_VIEW_DEGREES.to_radians() * 0.5).tan(),
            x_span: (f64::from(width) / f64::from(height)).max(1.0),
            y_span: (f64::from(height) / f64::from(width)).max(1.0),
            render,
            width,
            height,
        })
    }

    /// The library's camera for this view.
    pub fn render_camera(&self) -> RenderCamera {
        self.render
    }

    /// The ray through the continuous pixel position (`px`, `py`), origin top left. The library's
    /// camera counts rows from the bottom.
    pub fn ray(&self, px: f64, py: f64) -> Ray {
        self.render
            .ray_through(px, f64::from(self.height) - py, self.width, self.height)
    }

    /// The unit direction through the continuous pixel position (`px`, `py`), origin top left.
    pub fn ray_direction(&self, px: f64, py: f64) -> Vector3 {
        self.ray(px, py).direction_normal
    }

    /// Where `world` falls in the image, in pixels, or `None` behind the camera.
    pub fn project(&self, world: Vector3) -> Option<(f64, f64)> {
        let to_point = world - self.eye;
        let depth = to_point.dot(self.forward);
        if depth <= 0.0 {
            return None;
        }
        let u = to_point.dot(self.right) / (depth * self.tan_half * self.x_span);
        let v = to_point.dot(self.up) / (depth * self.tan_half * self.y_span);
        Some((
            (u + 1.0) * 0.5 * f64::from(self.width),
            (1.0 - v) * 0.5 * f64::from(self.height),
        ))
    }
}
