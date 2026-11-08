//! The Ray Tracing page's scene: a part made by the library standing on a ground plane, the BVH
//! over it (`ray_tracer::BoundingVolumeHierarchy`, one `MinimalTriangle` per face, as MatterCAD
//! builds it and as the Fillet page's picking does in `fillet_pick::build_bvh`), and the orbit
//! [`Camera`] that turns a pixel into a ray.
//!
//! The library's `ray_tracer::Scene` holds what rays can hit and its `RayTracer` follows a ray
//! into it; both are geometry only. The camera here, the shading in `ray_trace_shade` and the
//! accumulation in `ray_trace` are this page's own, built on that documented API.
//!
//! The world is z-up, as the parts are modeled. The ground sits a whisker under the part so the
//! flat bottom of a part never coincides with it.

use std::sync::Arc;

use solidmesh::math::{IntersectionType, Matrix4X4, Ray, Vector3};
use solidmesh::mesh::csg::boolean_processing::do_boolean;
use solidmesh::mesh::csg::{BooleanOptions, CsgModes};
use solidmesh::mesh::ray_tracer::{
    IntersectInfo, PlaneShape, RayTracer, Scene as RayScene, TraceableRef,
};
use solidmesh::mesh::{CancelToken, Mesh};

use crate::fillet_pick::build_bvh;
use crate::implicit::{blob, mesh_marching_cubes};
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

/// A part on its ground plane, ready to trace.
pub struct TraceScene {
    scene: RayScene,
    tracer: RayTracer,
    ground: Arc<PlaneShape>,
    /// Where the camera orbits.
    pub center: Vector3,
    /// The part's bounding radius.
    pub radius: f64,
    pub ground_z: f64,
    pub triangles: usize,
    pub note: String,
}

impl TraceScene {
    /// Builds the BVH over `mesh`. It can take seconds on a fine part, which is why the page runs
    /// it on its worker.
    pub fn new(mesh: &Mesh, note: String) -> Result<TraceScene, String> {
        let bvh = build_bvh(mesh).ok_or_else(|| "That part has no faces to trace.".to_string())?;
        let bounds = mesh.get_axis_aligned_bounding_box();
        let radius = bounds.size().length() * 0.5;
        let ground_z = bounds.min_xyz.z - radius * GROUND_GAP;
        let ground = Arc::new(PlaneShape::from_normal_and_distance(
            Vector3::UNIT_Z,
            ground_z,
        ));
        let mut scene = RayScene::new();
        scene.shapes.push(bvh);
        scene.shapes.push(ground.clone() as TraceableRef);
        Ok(TraceScene {
            scene,
            tracer: RayTracer::new(),
            ground,
            center: bounds.center(),
            radius,
            ground_z,
            triangles: mesh.faces.len(),
            note,
        })
    }

    /// The nearest hit of `ray`, counting it in `rays`. The hit type is `None` on a miss.
    pub fn trace<'a>(&'a self, ray: &mut Ray, rays: &mut u64) -> IntersectInfo<'a> {
        *rays += 1;
        self.tracer.trace_primary_ray(ray, &self.scene)
    }

    /// True when anything lies along `ray` within its window. The ray is marked as a shadow ray,
    /// so the hierarchy stops at the first hit it finds; the ground ignores a ray's window, which
    /// is why the distance is checked here.
    pub fn occluded(&self, ray: &mut Ray, rays: &mut u64) -> bool {
        ray.is_shadow_ray = true;
        let limit = ray.max_distance_to_consider;
        let hit = self.trace(ray, rays);
        hit.hit_type != IntersectionType::None && hit.distance_to_hit <= limit
    }

    /// True when `hit` landed on the ground plane rather than the part.
    pub fn is_ground(&self, hit: &IntersectInfo<'_>) -> bool {
        hit.hit_object_is(self.ground.as_ref())
    }

    /// What the ray through pixel position (`px`, `py`) finds, and how far away.
    pub fn surface_at(&self, camera: &Camera, px: f64, py: f64) -> (Surface, f64) {
        let mut ray = Ray::new(camera.eye, camera.ray_direction(px, py));
        let mut rays = 0;
        let hit = self.trace(&mut ray, &mut rays);
        if hit.hit_type == IntersectionType::None {
            return (Surface::Sky, f64::INFINITY);
        }
        let surface = if self.is_ground(&hit) {
            Surface::Ground
        } else {
            Surface::Part
        };
        (surface, hit.distance_to_hit)
    }
}

/// An orbit camera: `yaw` about z and `pitch` above the horizon (radians) around the part's
/// centre, `zoom` times closer than the starting distance, for an image `width` x `height` pixels.
#[derive(Debug, Clone, Copy)]
pub struct Camera {
    pub eye: Vector3,
    forward: Vector3,
    right: Vector3,
    up: Vector3,
    /// Tangent of half the field of view, which spans the shorter side.
    tan_half: f64,
    /// How much wider than `tan_half` the view is across the image: 1 on the shorter side, and the
    /// aspect ratio on the longer one. Multiplying by an exact 1 leaves the shorter side's rays
    /// untouched to the last bit.
    x_span: f64,
    y_span: f64,
    width: f64,
    height: f64,
}

impl Camera {
    pub fn new(
        scene: &TraceScene,
        yaw: f64,
        pitch: f64,
        zoom: f64,
        width: u32,
        height: u32,
    ) -> Camera {
        let pitch = pitch.clamp(
            MIN_PITCH_DEGREES.to_radians(),
            MAX_PITCH_DEGREES.to_radians(),
        );
        let distance = scene.radius * ORBIT_RADII / zoom.clamp(0.4, 8.0);
        let offset = Vector3::new(
            pitch.cos() * yaw.cos(),
            pitch.cos() * yaw.sin(),
            pitch.sin(),
        );
        let eye = scene.center + offset * distance;
        let forward = (scene.center - eye).get_normal();
        let right = forward.cross(Vector3::UNIT_Z).get_normal();
        Camera {
            eye,
            forward,
            right,
            up: right.cross(forward),
            tan_half: (FIELD_OF_VIEW_DEGREES.to_radians() * 0.5).tan(),
            x_span: (f64::from(width) / f64::from(height)).max(1.0),
            y_span: (f64::from(height) / f64::from(width)).max(1.0),
            width: f64::from(width),
            height: f64::from(height),
        }
    }

    /// The unit direction through the continuous pixel position (`px`, `py`), origin top left.
    pub fn ray_direction(&self, px: f64, py: f64) -> Vector3 {
        let u = (2.0 * px / self.width - 1.0) * self.tan_half * self.x_span;
        let v = (1.0 - 2.0 * py / self.height) * self.tan_half * self.y_span;
        (self.forward + self.right * u + self.up * v).get_normal()
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
        Some(((u + 1.0) * 0.5 * self.width, (1.0 - v) * 0.5 * self.height))
    }
}
