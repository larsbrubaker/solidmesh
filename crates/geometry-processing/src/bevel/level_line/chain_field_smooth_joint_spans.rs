//! The chain field's smooth-joint spans. Ports `Bevel/LevelLine/ChainField.SmoothJointSpans.cs` (a
//! partial of `ChainField`; the type lives in `chain_field.rs`): inside the smooth-joints scope an
//! open chain's interior smooth joint is given a SPAN of parameter - squeezing the half-segments
//! either side - so the parameter read near the joint moves continuously across it, and the
//! readings are carried into and out of that span. Outside the scope nothing changes.

use vector_math::{cs_math, Vector3};

use super::ChainField;

impl ChainField {
    /// Joint k's span: its arc-length parameter and the half-segments either side that the span
    /// squeezes, as (at, in, out).
    pub fn span_of(&self, k: usize) -> (f64, f64, f64) {
        let p = &self.parameter_at_point;
        (p[k], (p[k] - p[k - 1]) / 2.0, (p[k + 1] - p[k]) / 2.0)
    }

    /// Whether joint k is an interior smooth joint of an open chain, the only joints given a span.
    pub fn has_span(&self, k: i32) -> bool {
        self.smooth_joints
            .as_ref()
            .is_some_and(|joints| joints.contains(&k))
            && !self.is_closed
            && k > 0
            && (k as usize) + 1 < self.points.len()
    }

    /// The smooth joints in ascending order - the order C#'s `HashSet<int>` hands them back in,
    /// since they are added ascending and never removed; where two spans touch at a shared end the
    /// lower joint answers.
    fn smooth_joints_in_order(&self) -> Vec<i32> {
        let mut joints: Vec<i32> = self
            .smooth_joints
            .as_ref()
            .map(|joints| joints.iter().copied().collect())
            .unwrap_or_default();
        joints.sort_unstable();
        joints
    }

    /// An arc-length parameter read at `point`, carried into the spans' parameter.
    pub fn across_smooth_joints(&self, parameter: f64, point: Vector3) -> f64 {
        if self.smooth_joints.is_none() || self.is_closed {
            return parameter;
        }

        for k in self.smooth_joints_in_order() {
            if !self.has_span(k) {
                continue;
            }

            let k = k as usize;
            let (at, before, after) = self.span_of(k);
            if parameter < at - before || parameter > at + after {
                continue;
            }

            // The joint's own readers, by a loose tie: the setback changes along the crease, so the
            // region reading the joint is not the geometric wedge, and each segment's least sits a
            // hair either side of it. How far past each segment's end the point's UNCLAMPED least
            // lies is read instead - 0 exactly where the segment's own least reaches the joint,
            // which is the region's edge, so the span joins the squeezed half-segments continuously.
            if (parameter - at).abs() <= Self::JOINT_READ_TIE * cs_math::max_f64(1.0, self.length) {
                let past = cs_math::max_f64(
                    0.0,
                    (self.unclamped_least_on(k - 1, point) - 1.0) * 2.0 * before,
                );
                let short_of =
                    cs_math::max_f64(0.0, -self.unclamped_least_on(k, point) * 2.0 * after);
                let f = if past + short_of > 0.0 {
                    past / (past + short_of)
                } else {
                    0.5
                };
                return at - (before / 2.0) + (f * ((before + after) / 2.0));
            }

            return if parameter < at {
                at - before + ((parameter - (at - before)) / 2.0)
            } else {
                at + (after / 2.0) + ((parameter - at) / 2.0)
            };
        }

        parameter
    }

    /// The stationary point of segment `segment`'s field at `q` before it is clamped to the
    /// segment: below 0 or above 1 where the segment's own least is its end.
    pub fn unclamped_least_on(&self, segment: usize, q: Vector3) -> f64 {
        let from = self.points[segment];
        let along = self.points[segment + 1] - from;
        let length_squared = along.length_squared();
        let length = length_squared.sqrt();
        let k = self.setbacks[segment + 1] - self.setbacks[segment];
        let u0 = (q - from).dot(along) / length_squared;
        if k.abs() >= length {
            return u0;
        }

        let a = (q - (from + (along * u0))).length();
        let sign = if k > 0.0 {
            1.0
        } else if k < 0.0 {
            -1.0
        } else {
            0.0
        };
        u0 + (sign * k.abs() * a / (length * (length_squared - (k * k)).sqrt()))
    }

    /// The arc-length parameter a spans' parameter stands for: the whole of a wedge span is its
    /// joint.
    pub fn on_the_chain_from_spans(&self, parameter: f64) -> f64 {
        if self.smooth_joints.is_none() || self.is_closed {
            return parameter;
        }

        for k in self.smooth_joints_in_order() {
            if !self.has_span(k) {
                continue;
            }

            let (at, before, after) = self.span_of(k as usize);
            if parameter < at - before || parameter > at + after {
                continue;
            }

            if parameter <= at - (before / 2.0) {
                return at - before + (2.0 * (parameter - (at - before)));
            }

            if parameter >= at + (after / 2.0) {
                return at + (2.0 * (parameter - (at + (after / 2.0))));
            }

            return at;
        }

        parameter
    }
}
