//! The primitives and 2D profiles the demo pages share, every one built with the library: the
//! cube and icosahedron are `solidmesh::mesh::solids`, the rounded box is `RoundedCornerBox`, a
//! sphere and a torus are half circles and circles revolved by `outline_to_mesh::revolve_contours`,
//! and a cylinder or a star prism is a contour extruded by `extrude_contours`. Sizes are in mm and
//! each solid is centred on the origin, so the pages place them with a matrix.
//!
//! [`Lcg`] is the small seeded generator the hull and repair pages draw their random choices from,
//! so a seed always shows the same scene (and the thumbnails are reproducible).

use std::f64::consts::{PI, TAU};

use solidmesh::math::{Matrix4X4, Vector2, Vector3};
use solidmesh::mesh::processors::outline_to_mesh::{extrude_contours, revolve_contours};
use solidmesh::mesh::solids::platonic_solids::{create_cube, create_icosahedron};
use solidmesh::mesh::solids::RoundedCornerBox;
use solidmesh::mesh::Mesh;

/// The shapes the boolean, Minkowski and repair pages offer, by the index their menus send.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Shape {
    Cube,
    Sphere,
    Cylinder,
    RoundedBox,
    Torus,
    Icosahedron,
    Star,
}

impl Shape {
    /// The shape a menu index names, or an error naming the index.
    pub fn from_index(index: u32) -> Result<Shape, String> {
        Ok(match index {
            0 => Shape::Cube,
            1 => Shape::Sphere,
            2 => Shape::Cylinder,
            3 => Shape::RoundedBox,
            4 => Shape::Torus,
            5 => Shape::Icosahedron,
            6 => Shape::Star,
            _ => return Err(format!("There is no shape {index}.")),
        })
    }

    /// The shape about `size` mm across, centred on the origin, with `segments` around a curve.
    pub fn build(self, size: f64, segments: u32) -> Result<Mesh, String> {
        let r = size / 2.0;
        let mesh = match self {
            Shape::Cube => create_cube(size, size, size),
            Shape::Sphere => sphere(r, segments)?,
            Shape::Cylinder => cylinder(r, size, segments)?,
            Shape::RoundedBox => RoundedCornerBox::create(
                (segments / 4).max(2) as i32,
                Vector3::new(size, size, size),
                size * 0.2,
            ),
            Shape::Torus => torus(r * 0.7, r * 0.3, segments)?,
            Shape::Icosahedron => create_icosahedron(size),
            Shape::Star => prism(&star(5, r, r * 0.45), size * 0.5)?,
        };
        Ok(centered(mesh))
    }
}

/// `mesh` moved so its bounding box is centred on the origin.
pub fn centered(mut mesh: Mesh) -> Mesh {
    let center = mesh.get_axis_aligned_bounding_box().center();
    mesh.translate(Vector3::new(-center.x, -center.y, -center.z));
    mesh
}

/// A copy of `mesh` placed by `matrix`.
pub fn placed(mesh: &Mesh, matrix: Matrix4X4) -> Mesh {
    let mut copy = mesh.copy();
    copy.transform(matrix);
    copy
}

/// A circle of `radius` about `center`, counter-clockwise, `segments` points.
pub fn circle(center: Vector2, radius: f64, segments: u32) -> Vec<Vector2> {
    let n = segments.max(3);
    (0..n)
        .map(|i| {
            let a = TAU * f64::from(i) / f64::from(n);
            Vector2::new(center.x + radius * a.cos(), center.y + radius * a.sin())
        })
        .collect()
}

/// A star of `points` tips between `outer` and `inner` radius, counter-clockwise.
pub fn star(points: u32, outer: f64, inner: f64) -> Vec<Vector2> {
    let n = points.max(3) * 2;
    (0..n)
        .map(|i| {
            let a = PI / 2.0 + TAU * f64::from(i) / f64::from(n);
            let r = if i % 2 == 0 { outer } else { inner };
            Vector2::new(r * a.cos(), r * a.sin())
        })
        .collect()
}

/// A sphere: a half circle on the axis revolved a full turn.
pub fn sphere(radius: f64, segments: u32) -> Result<Mesh, String> {
    let n = (segments / 2).max(3);
    let half: Vec<Vector2> = (0..=n)
        .map(|i| {
            let a = -PI / 2.0 + PI * f64::from(i) / f64::from(n);
            Vector2::new(radius * a.cos(), radius * a.sin())
        })
        .collect();
    let mesh = revolve_contours(&[half], segments.max(3) as i32, 0.0, TAU, true)
        .map_err(|e| format!("revolve failed: {e}"))?;
    Ok(centered(mesh))
}

/// A z-axis cylinder `height` tall: a circle extruded.
pub fn cylinder(radius: f64, height: f64, segments: u32) -> Result<Mesh, String> {
    prism(&circle(Vector2::new(0.0, 0.0), radius, segments), height)
}

/// A torus about z: a circle of `minor` radius, `major` out from the axis, revolved.
pub fn torus(major: f64, minor: f64, segments: u32) -> Result<Mesh, String> {
    let ring = circle(Vector2::new(major, 0.0), minor, (segments / 2).max(6));
    let mesh = revolve_contours(&[ring], segments.max(3) as i32, 0.0, TAU, true)
        .map_err(|e| format!("revolve failed: {e}"))?;
    Ok(centered(mesh))
}

/// `contour` extruded `height` and centred.
pub fn prism(contour: &[Vector2], height: f64) -> Result<Mesh, String> {
    let mesh = extrude_contours(&[contour.to_vec()], height)
        .map_err(|e| format!("extrude failed: {e}"))?;
    Ok(centered(mesh))
}

/// A tiny seeded generator (Numerical Recipes' LCG constants): the same seed, the same scene.
pub struct Lcg(u64);

impl Lcg {
    pub fn new(seed: u32) -> Self {
        Lcg(u64::from(seed).wrapping_mul(2_654_435_761).wrapping_add(1))
    }

    /// The next value in [0, 1).
    pub fn next_f64(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }

    /// The next index below `n` (`n` > 0).
    pub fn below(&mut self, n: usize) -> usize {
        ((self.next_f64() * n as f64) as usize).min(n.saturating_sub(1))
    }
}
