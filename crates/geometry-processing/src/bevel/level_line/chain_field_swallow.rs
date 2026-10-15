//! The least size at which a chain's band reaches a point - its SWALLOW SCALE - with the pick's
//! and the room field's readings of it.
//!
//! At scale s every setback is s times its unit value, so the band reaches q once
//! `|q - c(u)| <= s * setback(u)` for some u: the least s is the least over the chain of
//! `|q - c(u)| / setback(u)`. Over one segment that ratio's derivative has the sign of a LINEAR
//! function of u (`L^2 w D0 - k a^2`), so when the setback at q's foot is positive the ratio falls
//! then rises and its one stationary point, clamped to the segment, is the least; otherwise the
//! least stands at an END. The ends are offered in every case. Every segment is read - one pass
//! for one point, asked a handful of times per room reading.

use vector_math::{ieee_math, Vector3};

use super::{ChainField, LevelLineWork, PickField, RoomField, WorkError};

impl ChainField {
    /// The least size at which this chain's band reaches `point`;
    /// +infinity where no setback is positive.
    pub fn swallow_scale(&self, point: Vector3) -> Result<f64, WorkError> {
        LevelLineWork::charge(1)?;
        let mut best = f64::INFINITY;
        let n = self.points.len();
        for segment in 0..self.segment_count() as usize {
            let from = self.points[segment];
            let to = self.points[(segment + 1) % n];
            let s0 = self.setbacks[segment];
            let s1 = self.setbacks[(segment + 1) % n];
            let along = to - from;
            let length_squared = along.length_squared();
            let ratio_at = |at: f64| {
                let denominator = s0 + ((s1 - s0) * at);
                if denominator > 0.0 {
                    (point - (from + (along * at))).length() / denominator
                } else {
                    f64::INFINITY
                }
            };

            best = ieee_math::min_f64(best, ieee_math::min_f64(ratio_at(0.0), ratio_at(1.0)));
            if length_squared <= 0.0 {
                continue;
            }

            let k = s1 - s0;
            let u0 = (point - from).dot(along) / length_squared;
            let at_the_foot = s0 + (k * u0);
            if at_the_foot > 0.0 {
                let a = (point - (from + (along * u0))).length();
                // A two-sided clamp that lets NaN through, so a broken reading stays visible.
                let stationary =
                    (u0 + (k * a * a / (length_squared * at_the_foot))).clamp(0.0, 1.0);
                best = ieee_math::min_f64(best, ratio_at(stationary));
            }
        }

        Ok(best)
    }
}

impl PickField {
    /// The least size at which any member's band reaches the point -
    /// a point is swallowed by whichever chain reaches it first.
    pub fn swallow_scale(&self, point: Vector3) -> Result<f64, WorkError> {
        let mut best = f64::INFINITY;
        for &k in &self.members {
            best = ieee_math::min_f64(best, self.chains[k as usize].swallow_scale(point)?);
        }

        Ok(best)
    }
}

impl RoomField {
    /// The least size at which this field's band reaches the point.
    pub fn swallow_scale(&self, point: Vector3) -> Result<f64, WorkError> {
        match (&self.one, &self.several) {
            (Some(one), _) => one.swallow_scale(point),
            (None, Some(several)) => several.swallow_scale(point),
            (None, None) => panic!("a room field holds a chain's or a group's field"),
        }
    }
}
