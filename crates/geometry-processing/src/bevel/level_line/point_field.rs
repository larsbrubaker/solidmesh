//! A picked point's field for the kernel's level-line cut ([`PointField`]). Ports the type of
//! `Bevel/LevelLine/PointField.cs` with its constructors and field queries. Its
//! `MinOverTriangle` is listed under R2.P04 by the call-graph packing, but a Rust trait impl is one
//! block and it calls only the ported closest-point form, so it lands here with the rest.

use vector_math::Vector3;

use super::{ChainDistanceField, ChainField, FieldMaximum, FieldMinimum, ScalarField, WorkError};

/// C# `PointField`: a picked point's field for the kernel's level-line cut - negative on the cap
/// the bevel removes, zero on the ring where it starts.
///
/// A fillet's is the straight-line distance to the point less the setback (a sphere - its ring is
/// the ball's tangent ring on a cone); a chamfer's is the depth below the point along an inward
/// normal less the plane's depth (the cut plane). Both are true distances, so 1-Lipschitz as
/// `BandRefinement` requires, and both answer their extremes over a piece exactly (the sphere's
/// least is the distance to the piece, its greatest a corner, as it is convex; the plane's are
/// corners), so the refinement takes the asking path and splits only where the field names - a ring
/// tangent to an edge at the room included. It plays `PickField`'s role for a pick with no chains.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PointField {
    /// The picked point.
    pub point: Vector3,
    /// The unit vector into the part for a plane; zero for a sphere.
    pub inward: Vector3,
    /// The setback (sphere) or depth (plane) at which the field is zero.
    pub level: f64,
}

impl PointField {
    /// A fillet's field: the distance to `point` less `setback`.
    pub fn sphere(point: Vector3, setback: f64) -> Self {
        Self {
            point,
            inward: Vector3::ZERO,
            level: setback,
        }
    }

    /// A chamfer's field: the depth below `point` along the unit `inward` less `depth`.
    pub fn plane(point: Vector3, inward: Vector3, depth: f64) -> Self {
        Self {
            point,
            inward,
            level: depth,
        }
    }

    /// The field at a point (uncharged: a point field costs nothing to read).
    pub fn read(&self, x: Vector3) -> f64 {
        if self.inward == Vector3::ZERO {
            (x - self.point).length() - self.level
        } else {
            (x - self.point).dot(self.inward) - self.level
        }
    }

    /// The lesser of the readings at two points, the first on a tie.
    pub fn least(&self, a: Vector3, b: Vector3) -> FieldMinimum {
        let va = self.read(a);
        let vb = self.read(b);
        if va <= vb {
            FieldMinimum::new(va, a)
        } else {
            FieldMinimum::new(vb, b)
        }
    }

    /// The greater of the readings at two points, the first on a tie. Both fields are convex, so
    /// their greatest over a segment or triangle is at a corner.
    pub fn greatest(&self, a: Vector3, b: Vector3) -> FieldMaximum {
        let va = self.read(a);
        let vb = self.read(b);
        if va >= vb {
            FieldMaximum::new(va, a)
        } else {
            FieldMaximum::new(vb, b)
        }
    }
}

impl ScalarField for PointField {
    fn value_at(&self, x: Vector3) -> Result<f64, WorkError> {
        Ok(self.read(x))
    }
}

impl ChainDistanceField for PointField {
    fn min_along_segment(&self, a: Vector3, b: Vector3) -> Result<FieldMinimum, WorkError> {
        if self.inward == Vector3::ZERO {
            let ab = b - a;
            let t = if ab.length_squared() > 0.0 {
                ((self.point - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let closest = a + ab * t;
            return Ok(FieldMinimum::new(self.read(closest), closest));
        }

        Ok(self.least(a, b))
    }

    fn min_over_triangle(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
    ) -> Result<FieldMinimum, WorkError> {
        if self.inward == Vector3::ZERO {
            let closest = ChainField::closest_on_triangle(self.point, a, b, c);
            return Ok(FieldMinimum::new(self.read(closest), closest));
        }

        Ok(self.least(self.least(a, b).point, c))
    }

    fn max_along_segment(
        &self,
        a: Vector3,
        b: Vector3,
        _tolerance: f64,
    ) -> Result<FieldMaximum, WorkError> {
        Ok(self.greatest(a, b))
    }

    fn max_along_segment_at_least(
        &self,
        a: Vector3,
        b: Vector3,
        _tolerance: f64,
        at_least: f64,
    ) -> Result<FieldMaximum, WorkError> {
        let greatest = self.greatest(a, b);
        Ok(if greatest.value > at_least {
            greatest
        } else {
            FieldMaximum::new(at_least, a)
        })
    }

    fn max_over_triangle(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
        _tolerance: f64,
    ) -> Result<FieldMaximum, WorkError> {
        Ok(self.greatest(self.greatest(a, b).point, c))
    }

    fn max_over_triangle_bounded(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
        tolerance: f64,
        _stop_above: f64,
        at_least: f64,
    ) -> Result<FieldMaximum, WorkError> {
        let greatest = self.max_over_triangle(a, b, c, tolerance)?;
        Ok(if greatest.value > at_least {
            greatest
        } else {
            FieldMaximum::new(at_least, a)
        })
    }
}
