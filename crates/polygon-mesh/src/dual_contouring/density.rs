//! A collection of signed distance field functions (the [`Sdf`] trait) - primitives, a transform
//! and the union / intersection / subtraction combinators. Negative is inside.
//!
//! The box primitive is called [`Box`], so inside this module the standard library's box is spelled
//! `std::boxed::Box`.

use vector_math::{AxisAlignedBoundingBox, Matrix4X4, Vector2, Vector3};

/// A signed distance function and the bounds of its solid.
pub trait Sdf {
    /// The signed distance at `p` (negative inside).
    fn sdf(&self, p: Vector3) -> f64;

    fn bounds(&self) -> AxisAlignedBoundingBox;
}

/// A sphere of `radius` centred on the origin.
#[derive(Debug, Clone, Copy, Default)]
pub struct Sphere {
    pub radius: f64,
}

impl Sdf for Sphere {
    fn sdf(&self, p: Vector3) -> f64 {
        p.length() - self.radius
    }

    fn bounds(&self) -> AxisAlignedBoundingBox {
        let r = self.radius;
        AxisAlignedBoundingBox::from_coordinates(-r, -r, -r, r, r, r)
    }
}

/// Another function placed by `matrix` (sampled through its inverse).
pub struct Transform {
    input: std::boxed::Box<dyn Sdf>,
    pub matrix: Matrix4X4,
    pub inverse: Matrix4X4,
}

impl Transform {
    pub fn new(input: std::boxed::Box<dyn Sdf>, matrix: Matrix4X4) -> Self {
        Self {
            input,
            matrix,
            inverse: matrix.inverted(),
        }
    }

    pub fn input(&self) -> &dyn Sdf {
        self.input.as_ref()
    }
}

impl Sdf for Transform {
    fn sdf(&self, p: Vector3) -> f64 {
        self.input.sdf(p.transform(self.inverse))
    }

    fn bounds(&self) -> AxisAlignedBoundingBox {
        self.input.bounds().new_transformed(self.matrix)
    }
}

/// A z-axis cylinder centred on the origin. It stores half the height, which the distance test
/// uses; [`Cylinder::height`] / [`Cylinder::set_height`] read and write the full height.
#[derive(Debug, Clone, Copy, Default)]
pub struct Cylinder {
    half_height: f64,
    pub radius: f64,
}

impl Cylinder {
    /// A cylinder `height` tall of `radius`.
    pub fn new(height: f64, radius: f64) -> Self {
        Self {
            half_height: height / 2.0,
            radius,
        }
    }

    /// The full height.
    pub fn height(&self) -> f64 {
        self.half_height * 2.0
    }

    /// Sets the full height.
    pub fn set_height(&mut self, value: f64) {
        self.half_height = value / 2.0;
    }
}

impl Sdf for Cylinder {
    fn sdf(&self, p: Vector3) -> f64 {
        let z = p.z.abs();
        if z > self.half_height {
            z - self.half_height
        } else if p.z < -self.half_height {
            // Unreachable after the `abs` test above.
            p.z
        } else {
            Vector2::new(p.x, p.y).length() - self.radius
        }
    }

    fn bounds(&self) -> AxisAlignedBoundingBox {
        let (r, h) = (self.radius, self.half_height);
        AxisAlignedBoundingBox::from_coordinates(-r, -r, -h, r, r, h)
    }
}

/// An axis-aligned box centred on the origin. It stores half the size, which the distance test
/// uses; [`Box::size`] / [`Box::set_size`] read and write the full size.
#[derive(Debug, Clone, Copy, Default)]
pub struct Box {
    half_size: Vector3,
}

impl Box {
    /// A box of `size`.
    pub fn new(size: Vector3) -> Self {
        Self {
            half_size: size / 2.0,
        }
    }

    /// The full size.
    pub fn size(&self) -> Vector3 {
        self.half_size * 2.0
    }

    /// Sets the full size.
    pub fn set_size(&mut self, value: Vector3) {
        self.half_size = value / 2.0;
    }
}

impl Sdf for Box {
    fn sdf(&self, p: Vector3) -> f64 {
        let q = p.abs() - self.half_size;
        Vector3::component_max(q, Vector3::new(0.0, 0.0, 0.0)).length()
            + q.x.max(q.y.max(q.z)).min(0.0)
    }

    fn bounds(&self) -> AxisAlignedBoundingBox {
        AxisAlignedBoundingBox::new(-self.half_size, self.half_size)
    }
}

/// The union of `items` (the least distance). Needs at least one item.
#[derive(Default)]
pub struct Union {
    pub items: Vec<std::boxed::Box<dyn Sdf>>,
}

impl Sdf for Union {
    fn sdf(&self, p: Vector3) -> f64 {
        let mut d = self.items[0].sdf(p);
        for item in &self.items[1..] {
            d = d.min(item.sdf(p));
        }
        d
    }

    fn bounds(&self) -> AxisAlignedBoundingBox {
        let mut b = self.items[0].bounds();
        for item in &self.items[1..] {
            b = AxisAlignedBoundingBox::union(&b, &item.bounds());
        }
        b
    }
}

/// The intersection of `items` (the greatest distance). Needs at least one item.
#[derive(Default)]
pub struct Intersection {
    pub items: Vec<std::boxed::Box<dyn Sdf>>,
}

impl Sdf for Intersection {
    fn sdf(&self, p: Vector3) -> f64 {
        let mut d = self.items[0].sdf(p);
        for item in &self.items[1..] {
            d = d.max(item.sdf(p));
        }
        d
    }

    fn bounds(&self) -> AxisAlignedBoundingBox {
        let mut b = self.items[0].bounds();
        for item in &self.items[1..] {
            b = AxisAlignedBoundingBox::intersection(&b, &item.bounds());
        }
        b
    }
}

/// The first item with every later one removed. Needs at least one item.
#[derive(Default)]
pub struct Subtraction {
    pub items: Vec<std::boxed::Box<dyn Sdf>>,
}

impl Sdf for Subtraction {
    fn sdf(&self, p: Vector3) -> f64 {
        let mut d = self.items[0].sdf(p);
        for item in &self.items[1..] {
            d = d.max(-item.sdf(p));
        }
        d
    }

    fn bounds(&self) -> AxisAlignedBoundingBox {
        self.items[0].bounds()
    }
}
