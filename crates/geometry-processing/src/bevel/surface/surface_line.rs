//! An infinite line in space, as a point on it and a unit direction along it. The surface bevel's
//! plan is written in these: a tangent line is where a face is walked back to, and a round's
//! center line is where the rolling ball's center runs. Ports `Bevel/Surface/SurfaceLine.cs`.
//!
//! Lines rather than segments on purpose. The plan says WHERE the fillet's tangency lies for one
//! chain edge; where along it the strip starts and stops is the neighbouring edge's business,
//! decided at the mitre in a later phase, and a segment would have to guess that now.

use vector_math::Vector3;

/// C# `SurfaceLine` (a readonly struct).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SurfaceLine {
    /// A point the line passes through.
    pub point: Vector3,
    /// The unit direction the line runs in.
    pub direction: Vector3,
}

impl SurfaceLine {
    /// C#'s constructor.
    pub fn new(point: Vector3, direction: Vector3) -> Self {
        SurfaceLine { point, direction }
    }

    /// The point of this line closest to `to` - the foot of the perpendicular from it.
    pub fn closest_point_to(&self, to: Vector3) -> Vector3 {
        self.point + (self.direction * (to - self.point).dot(self.direction))
    }

    /// How far `from` stands off this line.
    pub fn distance_to(&self, from: Vector3) -> f64 {
        (from - self.closest_point_to(from)).length()
    }

    /// Where this line crosses `other`, when the two really do meet: the mitre corner two
    /// consecutive tangent lines on one face share - the point on this line nearest the crossing.
    /// Fails on parallel lines, and on skew lines that pass farther than `tolerance` apart - which
    /// on a shared face plane means the two edges double back rather than turn a corner.
    pub fn try_cross(&self, other: &SurfaceLine, tolerance: f64) -> Option<Vector3> {
        self.try_cross_missed_by(other, tolerance)
            .map(|(crossing, _)| crossing)
    }

    /// The same, also saying how far the two lines MISSED each other by at their closest approach:
    /// zero where they really lie in one plane, and something under the tolerance where they are
    /// skew and happen to pass near. A caller with two crossings to choose between wants that
    /// difference - a pair that truly meets says more about where a mitre goes than a pair that
    /// nearly does.
    pub fn try_cross_missed_by(
        &self,
        other: &SurfaceLine,
        tolerance: f64,
    ) -> Option<(Vector3, f64)> {
        // The standard closest-approach solve: the pair of parameters where the connecting segment
        // is perpendicular to both lines. Its determinant vanishes exactly when the directions are
        // parallel.
        let between = other.point - self.point;
        let direction_dot = self.direction.dot(other.direction);
        let determinant = 1.0 - (direction_dot * direction_dot);
        // C# `!(Math.Abs(determinant) > 1e-12)`, which a NaN determinant also fails.
        if determinant.abs() <= 1e-12 || determinant.is_nan() {
            return None;
        }

        let along_this = between.dot(self.direction);
        let along_other = between.dot(other.direction);
        let here = (along_this - (direction_dot * along_other)) / determinant;
        let there = ((direction_dot * along_this) - along_other) / determinant;

        let on_this = self.point + (self.direction * here);
        let on_other = other.point + (other.direction * there);
        let apart = (on_this - on_other).length();

        if apart > tolerance {
            return None;
        }

        Some((on_this, apart))
    }
}
