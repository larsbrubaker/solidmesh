//! The test parts' solids, drawn as straight-edged outlines ([`Outline`]: MoveTo, LineTo and
//! close, in mm) and built by the kernel's extrude and revolve on the integer polygons the outline
//! snaps to (`outline_to_mesh::contours_to_polygons`: every point times `DEFAULT_SCALING`, rounded
//! half to even). That is the conversion an app path made of straight lines gets on its way to a
//! solid, so a part here is the part such a path in the app makes. Shared by the bevel and
//! topology binaries and by the model's and app's bevel tests that build these parts (alongside
//! `topology_test_meshes`).

use polygon_mesh::processors::outline_to_mesh::{self, RevolveError};
use polygon_mesh::Mesh;
use vector_math::Vector2;

/// An outline drawn as a path is: each MoveTo starts a contour, each LineTo adds a point to the
/// current one (starting one if there is none), and a close ends it.
#[derive(Debug, Clone, Default)]
pub struct Outline {
    contours: Vec<Vec<Vector2>>,
    drawing: bool,
}

impl Outline {
    /// An outline with nothing drawn.
    pub fn new() -> Outline {
        Outline::default()
    }

    /// Starts a contour at (x, y).
    pub fn move_to(&mut self, x: f64, y: f64) {
        self.contours.push(vec![Vector2::new(x, y)]);
        self.drawing = true;
    }

    /// Adds (x, y) to the contour being drawn.
    pub fn line_to(&mut self, x: f64, y: f64) {
        self.line_to_vector(Vector2::new(x, y));
    }

    /// Adds `point` to the contour being drawn.
    pub fn line_to_vector(&mut self, point: Vector2) {
        if self.drawing {
            if let Some(contour) = self.contours.last_mut() {
                contour.push(point);
                return;
            }
        }
        self.contours.push(vec![point]);
        self.drawing = true;
    }

    /// Ends the contour being drawn (the solid closes every contour back to its first point).
    pub fn close_polygon(&mut self) {
        self.drawing = false;
    }

    /// The contours, in mm.
    pub fn contours(&self) -> &[Vec<Vector2>] {
        &self.contours
    }
}

/// The outline extruded from z = 0 up to `height`, square-ended. The error is the kernel's clipper
/// error, named only by what it prints so the crates sharing this file need no clipper dependency.
pub fn extrude(outline: &Outline, height: f64) -> Result<Mesh, impl std::fmt::Debug> {
    outline_to_mesh::extrude_contours(outline.contours(), height)
}

/// The outline (x out from the axis, y along it) swept from `angle_start` to `angle_end`.
pub fn revolve(
    outline: &Outline,
    angle_steps: i32,
    angle_start: f64,
    angle_end: f64,
    revolve_around_z: bool,
) -> Result<Mesh, RevolveError> {
    outline_to_mesh::revolve_contours(
        outline.contours(),
        angle_steps,
        angle_start,
        angle_end,
        revolve_around_z,
    )
}
