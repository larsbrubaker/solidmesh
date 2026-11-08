//! The chain field's nearest-point walk: the chain field's constants, one segment's least reading
//! from a point, the box-tree walk that finds the nearest chain point, the mitre rays offered to
//! it and a ray's reading, and the unclipped readings built on it (the unclipped parameter and
//! value, and the value in an end plane). The type lives in `chain_field.rs`.

use solidmesh_math::Vector3;

use super::segment_box_tree;
use super::{ChainField, ChainFieldExtension, LevelLineWork, WorkError};

/// The best reading a nearest-point walk has found so far.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChainFieldNearest {
    /// The least reading, +infinity before any.
    pub best: f64,
    /// The chain parameter it was read at.
    pub parameter: f64,
    /// The chain point it was read from.
    pub nearest: Vector3,
    /// How far the point stands from that chain point.
    pub distance: f64,
}

impl ChainFieldNearest {
    /// A walk's starting state: nothing found, the chain's first point as the placeholder.
    pub fn start(first_point: Vector3) -> Self {
        Self {
            best: f64::INFINITY,
            parameter: 0.0,
            nearest: first_point,
            distance: 0.0,
        }
    }

    /// True when `value` beats the best under the tie rule: only a strictly smaller value, by more
    /// than the tie allowance, displaces it, so ties go to the LOWER parameter walked first.
    pub fn is_beaten_by(&self, value: f64) -> bool {
        self.best == f64::INFINITY
            || value < self.best - (ChainField::TIE_RELATIVE * (1.0 + self.best.abs()))
    }
}

/// The sign of `value` as a number: -1, 0 or 1 (0 for NaN as well as zero).
fn sign(value: f64) -> f64 {
    if value > 0.0 {
        1.0
    } else if value < 0.0 {
        -1.0
    } else {
        0.0
    }
}

impl ChainField {
    /// A tie is two segments answering the same distance by different arithmetic, so "the same"
    /// has to allow a few bits of drift, scaled to the distance being compared.
    pub const TIE_RELATIVE: f64 = 1e-12;

    /// Fixed rather than a tolerance, so the same input always gives the same bits: (2/3)^100 is
    /// about 2e-18.
    pub const TERNARY_STEPS: i32 = 100;

    /// The coarse scan the least-over-piece search starts from.
    pub const SCAN_STEPS: i32 = 64;

    /// What a bounded ray runs past its reach: relative slack so rounding never stops a rail a hair
    /// short of the end face's plane.
    pub const END_RAY_SLACK: f64 = 1e-3;

    /// Fixed, like the ternary steps, so the same input always gives the same bits: (2/3)^80 is
    /// about 8e-15.
    pub const CLIP_STEPS: i32 = 80;

    /// An end plane ties the chain's reading within rounding at an exact tie (a few 1e-15) - well
    /// under the 1e-12 a root-solved band vertex stands off the level set.
    pub const END_PLANE_TIE_RELATIVE: f64 = 0.1 * Self::TIE_RELATIVE;

    /// How far inside a joined crease's one end plane a point ON it reads: beyond any rounding of the
    /// chain's own value on its line, so the tie with that value never goes to the plane.
    pub const ON_THE_JOINED_PLANE: f64 = 1e-10;

    /// How near a joint's parameter, relative to the chain's length, a reading counts as the
    /// joint's own.
    pub const JOINT_READ_TIE: f64 = 1e-6;

    /// The least of `|q - C(u)| - setback(u)` over segment `segment`, with the u, chain point and
    /// distance attaining it, as (value, u, chain point, distance). The setback runs linearly
    /// along the segment, so the reading is convex in u: its least is at an end or at the one
    /// stationary point, clamped; both ends are offered in every case, so the clamped stationary
    /// point and the two ends together are always the exact least.
    pub fn least_over_segment_from_point(
        &self,
        segment: usize,
        q: Vector3,
    ) -> (f64, f64, Vector3, f64) {
        let n = self.points.len();
        let from = self.points[segment];
        let to = self.points[(segment + 1) % n];
        let s0 = self.setbacks[segment];
        let s1 = self.setbacks[(segment + 1) % n];
        let along = to - from;
        let length_squared = along.length_squared();

        let value_at_u = |at: f64| {
            let c = from + (along * at);
            let d = (q - c).length();
            (d - (s0 + ((s1 - s0) * at)), c, d)
        };

        if length_squared <= 0.0 {
            let (value, c, d) = value_at_u(0.0);
            return (value, 0.0, c, d);
        }

        let length = length_squared.sqrt();
        let k = s1 - s0;

        let mut best_u = 0.0;
        let (mut best, mut chain_point, mut distance) = value_at_u(0.0);
        let (end_value, end_point, end_distance) = value_at_u(1.0);
        if end_value < best {
            best = end_value;
            best_u = 1.0;
            chain_point = end_point;
            distance = end_distance;
        }

        if k.abs() < length {
            let u0 = (q - from).dot(along) / length_squared;
            let foot = from + (along * u0);
            let a = (q - foot).length();
            let offset = sign(k) * k.abs() * a / (length * (length_squared - (k * k)).sqrt());
            // NaN passes through the clamp, so a broken reading stays visible downstream.
            let stationary = (u0 + offset).clamp(0.0, 1.0);
            let (inside, inside_point, inside_distance) = value_at_u(stationary);
            if inside < best {
                best = inside;
                best_u = stationary;
                chain_point = inside_point;
                distance = inside_distance;
            }
        }

        (best, best_u, chain_point, distance)
    }

    /// The box tree's walk for the point query: a subtree whose box stands farther off than the
    /// nearest point found so far is skipped whole, children in segment order so the tie rule
    /// holds.
    pub fn visit_nearest(&self, node: usize, point: Vector3, found: &mut ChainFieldNearest) {
        let here = self.nodes[node];

        // Nothing in this subtree stands closer than the box's gap, and no ball in it has a radius
        // past the subtree's greatest setback, so no chain point under here can read below this.
        let bound = segment_box_tree::box_gap(point, point, here.min, here.max) - here.max_setback;
        if found.best != f64::INFINITY
            && bound >= found.best - (Self::TIE_RELATIVE * (1.0 + found.best.abs()))
        {
            return;
        }

        if here.left < 0 {
            for i in here.start..here.start + here.count {
                let i = i as usize;
                let (value, t, nearest, distance) = self.least_over_segment_from_point(i, point);
                if found.is_beaten_by(value) {
                    found.best = value;
                    found.nearest = nearest;
                    found.distance = distance;
                    let segment_length =
                        self.parameter_at_point[i + 1] - self.parameter_at_point[i];
                    found.parameter = self.parameter_at_point[i] + (segment_length * t);
                }

                self.nearest_over_extensions(i, point, found);
            }

            return;
        }

        self.visit_nearest(here.left as usize, point, found);
        self.visit_nearest(here.right as usize, point, found);
    }

    /// Offers segment `segment`'s rays to a point query, under the same tie rule as the segment
    /// itself.
    pub fn nearest_over_extensions(
        &self,
        segment: usize,
        point: Vector3,
        found: &mut ChainFieldNearest,
    ) {
        let Some(rays) = self
            .extensions
            .as_ref()
            .and_then(|extensions| extensions[segment].as_ref())
        else {
            return;
        };

        for ray in rays {
            let (value, nearest, distance) = ray.value_at(point);
            if found.is_beaten_by(value) {
                found.best = value;
                found.nearest = nearest;
                found.distance = distance;
                found.parameter = ray.parameter
                    + (ray.parameter_slope * (nearest - ray.origin).dot(ray.direction));
            }
        }
    }

    /// The chain parameter of the point's nearest chain point, before any end plane clips it:
    /// where a clipped reading names the END, this is where along the chain (or its end ray) the
    /// point really stands.
    pub fn unclipped_parameter_at(&self, point: Vector3) -> Result<f64, WorkError> {
        LevelLineWork::charge(1)?;
        let mut found = ChainFieldNearest::start(self.points[0]);
        self.visit_nearest(0, point, &mut found);
        Ok(found.parameter)
    }

    /// The chain's reading along an edge lying in one of its end planes: its own, unclipped.
    pub fn value_in_the_end_plane(&self, point: Vector3) -> Result<f64, WorkError> {
        self.unclipped_value_at(point)
    }

    /// The chain's own reading at a point, before any end plane clips it.
    pub fn unclipped_value_at(&self, point: Vector3) -> Result<f64, WorkError> {
        LevelLineWork::charge(1)?;
        let mut found = ChainFieldNearest::start(self.points[0]);
        self.visit_nearest(0, point, &mut found);
        Ok(found.best)
    }
}

impl ChainFieldExtension {
    /// A ray from `origin` along `direction`, kept on the `keep` side and reaching `reach`, with
    /// the setback and parameter at its start and how fast the parameter moves along it.
    pub fn new(
        origin: Vector3,
        direction: Vector3,
        keep: Vector3,
        reach: f64,
        setback: f64,
        parameter: f64,
        parameter_slope: f64,
    ) -> Self {
        Self {
            origin,
            direction,
            keep,
            reach,
            setback,
            parameter,
            parameter_slope,
        }
    }

    /// The ray's far end.
    pub fn end(&self) -> Vector3 {
        self.origin + (self.direction * self.reach)
    }

    /// The ray's reading at a point, as (value, nearest point on the ray, distance to it), or
    /// +infinity off its side of the bisector.
    pub fn value_at(&self, q: Vector3) -> (f64, Vector3, f64) {
        let w = q - self.origin;
        if w.dot(self.keep) < 0.0 {
            return (f64::INFINITY, self.origin, f64::INFINITY);
        }

        let mut t = w.dot(self.direction);
        t = if t < 0.0 {
            0.0
        } else if t > self.reach {
            self.reach
        } else {
            t
        };
        let nearest = self.origin + (self.direction * t);
        let distance = (q - nearest).length();
        (distance - self.setback, nearest, distance)
    }
}
