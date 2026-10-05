//! `AxisAlignedBoundingBox`: a box given by its min and max corners. Ports agg-sharp
//! `VectorMath/AxisAlignedBoundingBox.cs`. `Frustum::get_intersect` and `Ray` test against it;
//! it transforms through `Matrix4X4` (`new_transformed`).
//!
//! Mapping notes: the C# class becomes a `Copy` value; `MinXYZ`/`MaxXYZ` are `min_xyz`/
//! `max_xyz` and serialize as Newtonsoft writes them (`{"MinXYZ":{..},"MaxXYZ":{..}}`).
//! `Center`/`Size`/`XSize`.. are methods; the indexer (0 = min, 1 = max) is `corner(index)`;
//! `Equals(bounds, tolerance)` is `equals_within`; `Contains(x, y, z, errorRange = .001)` is
//! `contains_xyz` and `contains_point` (the `Vector3` overload) with `contains_point_within` for
//! a custom range; `Union(bounds, vertex)` is `union_point`; `Clamp(ref v)` takes `&mut`; the
//! `+` operator is `union`. C# `Math.Min`/`Math.Max` (NaN wins) are `cs_math`'s.
//!
//! The C# caches `GetVolume`/`GetSurfaceArea` in private fields (recomputed only while the cache
//! is 0), so a box that is changed after either was read keeps reporting the old value. Rust
//! computes them on every call; see `docs/CSHARP_DIVERGENCES.md` entry 4.

use crate::cs_math::{max_f64, min_f64};
use crate::{Matrix4X4, Vector3, Vector3Float};
use std::fmt;
use std::ops::Add;

/// An axis-aligned box (C# `MatterHackers.VectorMath.AxisAlignedBoundingBox`).
#[derive(Debug, Clone, Copy, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct AxisAlignedBoundingBox {
    #[serde(rename = "MinXYZ")]
    pub min_xyz: Vector3,
    #[serde(rename = "MaxXYZ")]
    pub max_xyz: Vector3,
}

/// The default `errorRange` of C# `Contains(Vector3)` and `Contains(x, y, z)`.
const CONTAINS_ERROR_RANGE: f64 = 0.001;

impl AxisAlignedBoundingBox {
    /// A box from its corners.
    pub const fn new(min_xyz: Vector3, max_xyz: Vector3) -> Self {
        AxisAlignedBoundingBox { min_xyz, max_xyz }
    }

    /// A box from its corner coordinates.
    pub fn from_coordinates(
        min_x: f64,
        min_y: f64,
        min_z: f64,
        max_x: f64,
        max_y: f64,
        max_z: f64,
    ) -> Self {
        Self::new(
            Vector3::new(min_x, min_y, min_z),
            Vector3::new(max_x, max_y, max_z),
        )
    }

    /// A box from f32 corners, widened.
    pub fn from_vector3_float(min_xyz: Vector3Float, max_xyz: Vector3Float) -> Self {
        Self::new(
            Vector3::from_vector3_float(min_xyz),
            Vector3::from_vector3_float(max_xyz),
        )
    }

    /// The bounds of `vertices_points`; an empty list gives the zero box (C#
    /// `AxisAlignedBoundingBox(IList<Vector3>)`).
    pub fn from_points(vertices_points: &[Vector3]) -> Self {
        let mut result = Self::default();
        if let Some(first) = vertices_points.first() {
            result.min_xyz = *first;
            result.max_xyz = *first;
            for point in &vertices_points[1..] {
                result.expand_to_include_point(*point);
            }
        }
        result
    }

    /// The inverted infinite box that any `expand_to_include` replaces.
    pub fn empty() -> Self {
        Self::new(Vector3::POSITIVE_INFINITY, Vector3::NEGATIVE_INFINITY)
    }

    /// The box with both corners at the origin.
    pub fn zero() -> Self {
        Self::new(Vector3::ZERO, Vector3::ZERO)
    }

    /// A box of `size` centered on `position`.
    pub fn centered_box(size: Vector3, position: Vector3) -> Self {
        Self::new(position - size / 2.0, position + size / 2.0)
    }

    /// A box reaching `halfsize` each way from `position`.
    pub fn centered_half_extents(halfsize: Vector3, position: Vector3) -> Self {
        Self::new(position - halfsize, position + halfsize)
    }

    /// The center (C# property `Center`: the sum divided by 2).
    pub fn center(&self) -> Vector3 {
        (self.min_xyz + self.max_xyz) / 2.0
    }

    /// The center (C# `GetCenter()`: the sum times .5).
    pub fn get_center(&self) -> Vector3 {
        (self.min_xyz + self.max_xyz) * 0.5
    }

    /// The x of the center.
    pub fn get_center_x(&self) -> f64 {
        (self.min_xyz.x + self.max_xyz.x) * 0.5
    }

    /// `max - min`.
    pub fn size(&self) -> Vector3 {
        self.max_xyz - self.min_xyz
    }

    /// The size along x.
    pub fn x_size(&self) -> f64 {
        self.max_xyz.x - self.min_xyz.x
    }

    /// The size along y.
    pub fn y_size(&self) -> f64 {
        self.max_xyz.y - self.min_xyz.y
    }

    /// The size along z.
    pub fn z_size(&self) -> f64 {
        self.max_xyz.z - self.min_xyz.z
    }

    /// True when both corners are strictly within `equality_tolerance` (C# `Equals(bounds,
    /// tolerance = 0)`; with the default 0 the strict test is never true).
    pub fn equals_within(&self, bounds: &AxisAlignedBoundingBox, equality_tolerance: f64) -> bool {
        self.min_xyz
            .equals_within(bounds.min_xyz, equality_tolerance)
            && self
                .max_xyz
                .equals_within(bounds.max_xyz, equality_tolerance)
    }

    /// The axis-aligned bounds of this box's eight corners after `transform`.
    pub fn new_transformed(&self, transform: Matrix4X4) -> AxisAlignedBoundingBox {
        let (lo, hi) = (self.min_xyz, self.max_xyz);
        let mut bounds_verts = [
            Vector3::new(lo.x, lo.y, lo.z),
            Vector3::new(lo.x, lo.y, hi.z),
            Vector3::new(lo.x, hi.y, lo.z),
            Vector3::new(lo.x, hi.y, hi.z),
            Vector3::new(hi.x, lo.y, lo.z),
            Vector3::new(hi.x, lo.y, hi.z),
            Vector3::new(hi.x, hi.y, lo.z),
            Vector3::new(hi.x, hi.y, hi.z),
        ];

        crate::transform_array(&mut bounds_verts, transform);

        let mut new_min = Vector3::new(f64::MAX, f64::MAX, f64::MAX);
        let mut new_max = Vector3::new(f64::MIN, f64::MIN, f64::MIN);

        for vert in &bounds_verts {
            new_min.x = min_f64(new_min.x, vert.x);
            new_min.y = min_f64(new_min.y, vert.y);
            new_min.z = min_f64(new_min.z, vert.z);

            new_max.x = max_f64(new_max.x, vert.x);
            new_max.y = max_f64(new_max.y, vert.y);
            new_max.z = max_f64(new_max.z, vert.z);
        }

        AxisAlignedBoundingBox::new(new_min, new_max)
    }

    /// Grows this box to include `other`.
    pub fn expand_to_include(&mut self, other: &AxisAlignedBoundingBox) {
        self.min_xyz.x = min_f64(self.min_xyz.x, other.min_xyz.x);
        self.min_xyz.y = min_f64(self.min_xyz.y, other.min_xyz.y);
        self.min_xyz.z = min_f64(self.min_xyz.z, other.min_xyz.z);

        self.max_xyz.x = max_f64(self.max_xyz.x, other.max_xyz.x);
        self.max_xyz.y = max_f64(self.max_xyz.y, other.max_xyz.y);
        self.max_xyz.z = max_f64(self.max_xyz.z, other.max_xyz.z);
    }

    /// Grows the box by `amount` on every side.
    pub fn expand(&mut self, amount: f64) {
        self.expand_xyz(amount, amount, amount);
    }

    /// The overlap of two boxes; boxes that do not overlap (or only touch) give the zero box.
    pub fn get_intersection(&self, other: &AxisAlignedBoundingBox) -> AxisAlignedBoundingBox {
        let mut intersection = AxisAlignedBoundingBox::default();
        intersection.min_xyz.x = max_f64(self.min_xyz.x, other.min_xyz.x);
        intersection.min_xyz.y = max_f64(self.min_xyz.y, other.min_xyz.y);
        intersection.min_xyz.z = max_f64(self.min_xyz.z, other.min_xyz.z);

        intersection.max_xyz.x = min_f64(self.max_xyz.x, other.max_xyz.x);
        intersection.max_xyz.y = min_f64(self.max_xyz.y, other.max_xyz.y);
        intersection.max_xyz.z = min_f64(self.max_xyz.z, other.max_xyz.z);

        if intersection.min_xyz.x >= intersection.max_xyz.x
            || intersection.min_xyz.y >= intersection.max_xyz.y
            || intersection.min_xyz.z >= intersection.max_xyz.z
        {
            return AxisAlignedBoundingBox::zero();
        }

        intersection
    }

    /// Grows the box by `x`, `y` and `z` on the matching sides (C# `Expand(x, y, z)`).
    pub fn expand_xyz(&mut self, x: f64, y: f64, z: f64) {
        self.min_xyz.x -= x;
        self.min_xyz.y -= y;
        self.min_xyz.z -= z;

        self.max_xyz.x += x;
        self.max_xyz.y += y;
        self.max_xyz.z += z;
    }

    /// Grows the box to include `position` (C# `ExpandToInclude(Vector3)`).
    pub fn expand_to_include_point(&mut self, position: Vector3) {
        self.min_xyz.x = min_f64(self.min_xyz.x, position.x);
        self.min_xyz.y = min_f64(self.min_xyz.y, position.y);
        self.min_xyz.z = min_f64(self.min_xyz.z, position.z);

        self.max_xyz.x = max_f64(self.max_xyz.x, position.x);
        self.max_xyz.y = max_f64(self.max_xyz.y, position.y);
        self.max_xyz.z = max_f64(self.max_xyz.z, position.z);
    }

    /// Grows the box to include an f32 position, widened.
    pub fn expand_to_include_point_f32(&mut self, position: Vector3Float) {
        self.expand_to_include_point(Vector3::from_vector3_float(position));
    }

    /// A corner of the bottom (min z) face: 0 = (max x, max y), 1 = (min x, max y), 2 = (min x,
    /// min y), 3 = (max x, min y); any other index gives the origin.
    pub fn get_bottom_corner(&self, quadrant_index: i32) -> Vector3 {
        let (lo, hi) = (self.min_xyz, self.max_xyz);
        match quadrant_index {
            0 => Vector3::new(hi.x, hi.y, lo.z),
            1 => Vector3::new(lo.x, hi.y, lo.z),
            2 => Vector3::new(lo.x, lo.y, lo.z),
            3 => Vector3::new(hi.x, lo.y, lo.z),
            _ => Vector3::ZERO,
        }
    }

    /// A corner of the top (max z) face, numbered as [`Self::get_bottom_corner`].
    pub fn get_top_corner(&self, quadrant_index: i32) -> Vector3 {
        let (lo, hi) = (self.min_xyz, self.max_xyz);
        match quadrant_index {
            0 => Vector3::new(hi.x, hi.y, hi.z),
            1 => Vector3::new(lo.x, hi.y, hi.z),
            2 => Vector3::new(lo.x, lo.y, hi.z),
            3 => Vector3::new(hi.x, lo.y, hi.z),
            _ => Vector3::ZERO,
        }
    }

    /// The four top corners, then the four bottom ones.
    pub fn get_corners(&self) -> [Vector3; 8] {
        [
            self.get_top_corner(0),
            self.get_top_corner(1),
            self.get_top_corner(2),
            self.get_top_corner(3),
            self.get_bottom_corner(0),
            self.get_bottom_corner(1),
            self.get_bottom_corner(2),
            self.get_bottom_corner(3),
        ]
    }

    /// The cost of a ray-box test relative to other bounded items (always 1).
    pub fn get_intersect_cost() -> f64 {
        1.0
    }

    /// The volume.
    pub fn get_volume(&self) -> f64 {
        (self.max_xyz.x - self.min_xyz.x)
            * (self.max_xyz.y - self.min_xyz.y)
            * (self.max_xyz.z - self.min_xyz.z)
    }

    /// The surface area of the six faces.
    pub fn get_surface_area(&self) -> f64 {
        let front_and_back =
            (self.max_xyz.x - self.min_xyz.x) * (self.max_xyz.z - self.min_xyz.z) * 2.0;
        let left_and_right =
            (self.max_xyz.y - self.min_xyz.y) * (self.max_xyz.z - self.min_xyz.z) * 2.0;
        let top_and_bottom =
            (self.max_xyz.x - self.min_xyz.x) * (self.max_xyz.y - self.min_xyz.y) * 2.0;
        front_and_back + left_and_right + top_and_bottom
    }

    /// 0 is the min corner, 1 the max (C# indexer); any other index panics, where C# throws
    /// `IndexOutOfRangeException`.
    pub fn corner(&self, index: usize) -> Vector3 {
        match index {
            0 => self.min_xyz,
            1 => self.max_xyz,
            _ => panic!("AxisAlignedBoundingBox index {index} out of range"),
        }
    }

    /// The box holding both.
    pub fn union(bounds_a: &AxisAlignedBoundingBox, bounds_b: &AxisAlignedBoundingBox) -> Self {
        let min_xyz = Vector3::new(
            min_f64(bounds_a.min_xyz.x, bounds_b.min_xyz.x),
            min_f64(bounds_a.min_xyz.y, bounds_b.min_xyz.y),
            min_f64(bounds_a.min_xyz.z, bounds_b.min_xyz.z),
        );

        let max_xyz = Vector3::new(
            max_f64(bounds_a.max_xyz.x, bounds_b.max_xyz.x),
            max_f64(bounds_a.max_xyz.y, bounds_b.max_xyz.y),
            max_f64(bounds_a.max_xyz.z, bounds_b.max_xyz.z),
        );

        Self::new(min_xyz, max_xyz)
    }

    /// True when the boxes overlap or touch.
    pub fn intersects(&self, bounds: &AxisAlignedBoundingBox) -> bool {
        bounds.min_xyz.x <= self.max_xyz.x
            && bounds.max_xyz.x >= self.min_xyz.x
            && bounds.min_xyz.y <= self.max_xyz.y
            && bounds.max_xyz.y >= self.min_xyz.y
            && bounds.min_xyz.z <= self.max_xyz.z
            && bounds.max_xyz.z >= self.min_xyz.z
    }

    /// Sets both corners.
    pub fn set(&mut self, min_x: f64, min_y: f64, min_z: f64, max_x: f64, max_y: f64, max_z: f64) {
        self.min_xyz = Vector3::new(min_x, min_y, min_z);
        self.max_xyz = Vector3::new(max_x, max_y, max_z);
    }

    /// The overlap of two boxes; where they do not overlap the max is pulled up to the min (C#
    /// static `Intersection`, compare [`Self::get_intersection`]).
    pub fn intersection(
        bounds_a: &AxisAlignedBoundingBox,
        bounds_b: &AxisAlignedBoundingBox,
    ) -> Self {
        let min_xyz = Vector3::new(
            max_f64(bounds_a.min_xyz.x, bounds_b.min_xyz.x),
            max_f64(bounds_a.min_xyz.y, bounds_b.min_xyz.y),
            max_f64(bounds_a.min_xyz.z, bounds_b.min_xyz.z),
        );

        let max_xyz = Vector3::new(
            max_f64(min_xyz.x, min_f64(bounds_a.max_xyz.x, bounds_b.max_xyz.x)),
            max_f64(min_xyz.y, min_f64(bounds_a.max_xyz.y, bounds_b.max_xyz.y)),
            max_f64(min_xyz.z, min_f64(bounds_a.max_xyz.z, bounds_b.max_xyz.z)),
        );

        Self::new(min_xyz, max_xyz)
    }

    /// The box grown to include `vertex` (C# `Union(bounds, vertex)`).
    pub fn union_point(bounds: &AxisAlignedBoundingBox, vertex: Vector3) -> Self {
        let mut min_xyz = Vector3::ZERO;
        min_xyz.x = min_f64(bounds.min_xyz.x, vertex.x);
        min_xyz.y = min_f64(bounds.min_xyz.y, vertex.y);
        min_xyz.z = min_f64(bounds.min_xyz.z, vertex.z);

        let mut max_xyz = Vector3::ZERO;
        max_xyz.x = max_f64(bounds.max_xyz.x, vertex.x);
        max_xyz.y = max_f64(bounds.max_xyz.y, vertex.y);
        max_xyz.z = max_f64(bounds.max_xyz.z, vertex.z);

        Self::new(min_xyz, max_xyz)
    }

    /// Moves `position_to_clamp` inside the box.
    pub fn clamp(&self, position_to_clamp: &mut Vector3) {
        for axis in 0..3 {
            if position_to_clamp[axis] < self.min_xyz[axis] {
                position_to_clamp[axis] = self.min_xyz[axis];
            } else if position_to_clamp[axis] > self.max_xyz[axis] {
                position_to_clamp[axis] = self.max_xyz[axis];
            }
        }
    }

    /// True when (x, y, z) is inside, with C#'s default .001 slack.
    pub fn contains_xyz(&self, x: f64, y: f64, z: f64) -> bool {
        self.contains_point_within(Vector3::new(x, y, z), CONTAINS_ERROR_RANGE)
    }

    /// True when `position` is inside, with C#'s default .001 slack.
    pub fn contains_point(&self, position: Vector3) -> bool {
        self.contains_point_within(position, CONTAINS_ERROR_RANGE)
    }

    /// True when `position` is inside, with `error_range` slack.
    pub fn contains_point_within(&self, position: Vector3, error_range: f64) -> bool {
        self.min_xyz.x <= position.x + error_range
            && self.max_xyz.x >= position.x - error_range
            && self.min_xyz.y <= position.y + error_range
            && self.max_xyz.y >= position.y - error_range
            && self.min_xyz.z <= position.z + error_range
            && self.max_xyz.z >= position.z - error_range
    }

    /// True when `bounds` is entirely inside this box.
    pub fn contains(&self, bounds: &AxisAlignedBoundingBox) -> bool {
        self.min_xyz.x <= bounds.min_xyz.x
            && self.max_xyz.x >= bounds.max_xyz.x
            && self.min_xyz.y <= bounds.min_xyz.y
            && self.max_xyz.y >= bounds.max_xyz.y
            && self.min_xyz.z <= bounds.min_xyz.z
            && self.max_xyz.z >= bounds.max_xyz.z
    }

    /// True when no coordinate is NaN or at least `largest_possible_value` in size.
    pub fn valid(&self, largest_possible_value: f64) -> bool {
        valid_vector(self.min_xyz, largest_possible_value)
            && valid_vector(self.max_xyz, largest_possible_value)
    }

    /// The axis (0 = x, 1 = y, 2 = z) the box is longest along; ties go to the later axis.
    pub fn get_largest_axis(&self) -> usize {
        let size = self.size();
        if size.x > size.y && size.x > size.z {
            0
        } else if size.y > size.z {
            1
        } else {
            2
        }
    }
}

fn valid_double(value: f64, largest_possible_value: f64) -> bool {
    !value.is_nan() && value.abs() < largest_possible_value
}

fn valid_vector(vector: Vector3, largest_possible_value: f64) -> bool {
    valid_double(vector.x, largest_possible_value)
        && valid_double(vector.y, largest_possible_value)
        && valid_double(vector.z, largest_possible_value)
}

impl Add for AxisAlignedBoundingBox {
    type Output = AxisAlignedBoundingBox;
    fn add(self, b: AxisAlignedBoundingBox) -> AxisAlignedBoundingBox {
        AxisAlignedBoundingBox::union(&self, &b)
    }
}

/// "min [x, y, z] - max [x, y, z]".
impl fmt::Display for AxisAlignedBoundingBox {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "min {} - max {}", self.min_xyz, self.max_xyz)
    }
}
