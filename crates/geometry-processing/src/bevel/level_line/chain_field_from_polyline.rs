//! Building a [`ChainField`] from its polyline: the points and setbacks copied, the parameter at
//! each point
//! walked, the smooth joints in scope read, the mitres and end rays found, the end clips set and
//! the segment box tree built. Also the rebuild with scaled setbacks (for the swallow scale), the
//! exhaustive reading the tree is checked against, and the segment a parameter falls on.

use std::cell::OnceCell;

use vector_math::ieee_math::{max_f64, min_f64};
use vector_math::Vector3;

use super::chain_field_nearest::ChainFieldNearest;
use super::{segment_box_tree, ChainField, EndRayBound, FieldSample, LevelLineWork, WorkError};

/// Why a polyline builds no field: a caller's bug, reported with a message rather than a panic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChainFieldArgumentError {
    /// Fewer than two points.
    TooFewPoints,
    /// Not one setback per point.
    SetbackCountMismatch,
    /// The chain has no length.
    ZeroLength,
}

impl std::fmt::Display for ChainFieldArgumentError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            ChainFieldArgumentError::TooFewPoints => "A chain needs at least two points.",
            ChainFieldArgumentError::SetbackCountMismatch => {
                "There must be exactly one setback per chain point."
            }
            ChainFieldArgumentError::ZeroLength => "A chain of zero length has no field.",
        })
    }
}

impl std::error::Error for ChainFieldArgumentError {}

impl ChainField {
    /// The field over an already-walked polyline. The smooth joints
    /// are read from the scope [`ChainField::smooth_joints_while`] set, and a closed chain has no
    /// end rays and no end bounds whatever it was handed.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        points: Vec<Vector3>,
        is_closed: bool,
        setbacks: Vec<f64>,
        parameter_at_point: Vec<f64>,
        min_setback: f64,
        max_setback: f64,
        sharp_turn_degrees: f64,
        end_rays: (bool, bool),
        end_ray_bounds: (Option<EndRayBound>, Option<EndRayBound>),
    ) -> Self {
        let length = parameter_at_point[parameter_at_point.len() - 1];
        let smooth_joints = ChainField::smooth_joints_in(&points);
        let end_rays = if is_closed { (false, false) } else { end_rays };
        let end_ray_bounds = if is_closed {
            (None, None)
        } else {
            end_ray_bounds
        };

        let is_smooth_joint = |k: i32| {
            smooth_joints
                .as_ref()
                .is_some_and(|joints| joints.contains(&k))
        };

        let extensions = ChainField::with_end_rays(
            ChainField::find_mitres(
                &points,
                is_closed,
                &setbacks,
                &parameter_at_point,
                sharp_turn_degrees,
                &is_smooth_joint,
            ),
            &points,
            &setbacks,
            &parameter_at_point,
            end_rays,
            end_ray_bounds,
        );
        let end_clips = ChainField::find_end_clips(&points, length, end_ray_bounds);
        let reaches = ChainField::reaches_of(extensions.as_deref());
        let nodes = segment_box_tree::build(
            &points,
            &setbacks,
            parameter_at_point.len() as i32 - 1,
            reaches.as_deref(),
        );

        ChainField {
            points,
            setbacks,
            parameter_at_point,
            nodes,
            smooth_joints,
            sharp_turn_degrees,
            end_rays,
            end_ray_bounds,
            is_closed,
            length,
            min_setback,
            max_setback,
            extensions,
            end_clips,
            end_plane_reader: OnceCell::new(),
            sharp_joints: OnceCell::new(),
            known_cuts: OnceCell::new(),
        }
    }

    /// No sharp-turn threshold, so no joint is
    /// mitred.
    pub fn from_polyline(
        points: &[Vector3],
        is_closed: bool,
        setback_at_point: &[f64],
    ) -> Result<ChainField, ChainFieldArgumentError> {
        ChainField::from_polyline_sharp(points, is_closed, setback_at_point, f64::INFINITY)
    }

    /// Joints turning by more than the threshold mitred.
    pub fn from_polyline_sharp(
        points: &[Vector3],
        is_closed: bool,
        setback_at_point: &[f64],
        sharp_turn_degrees: f64,
    ) -> Result<ChainField, ChainFieldArgumentError> {
        ChainField::from_polyline_rayed(
            points,
            is_closed,
            setback_at_point,
            sharp_turn_degrees,
            (false, false),
        )
    }

    /// The named open ends read on past as rays.
    pub fn from_polyline_rayed(
        points: &[Vector3],
        is_closed: bool,
        setback_at_point: &[f64],
        sharp_turn_degrees: f64,
        end_rays: (bool, bool),
    ) -> Result<ChainField, ChainFieldArgumentError> {
        ChainField::from_polyline_bounded(
            points,
            is_closed,
            setback_at_point,
            sharp_turn_degrees,
            end_rays,
            (None, None),
        )
    }

    /// The rays bounded by their
    /// end faces' planes, and the band clipped there.
    pub fn from_polyline_bounded(
        points: &[Vector3],
        is_closed: bool,
        setback_at_point: &[f64],
        sharp_turn_degrees: f64,
        end_rays: (bool, bool),
        end_ray_bounds: (Option<EndRayBound>, Option<EndRayBound>),
    ) -> Result<ChainField, ChainFieldArgumentError> {
        if points.len() < 2 {
            return Err(ChainFieldArgumentError::TooFewPoints);
        }

        if setback_at_point.len() != points.len() {
            return Err(ChainFieldArgumentError::SetbackCountMismatch);
        }

        let ordered = points.to_vec();
        let backs = setback_at_point.to_vec();
        let mut min_setback = f64::INFINITY;
        let mut max_setback = f64::NEG_INFINITY;

        for &back in &backs {
            min_setback = min_f64(min_setback, back);
            max_setback = max_f64(max_setback, back);
        }

        let segment_count = if is_closed {
            ordered.len()
        } else {
            ordered.len() - 1
        };
        let mut parameters = vec![0.0; segment_count + 1];

        for i in 0..segment_count {
            let next = (i + 1) % ordered.len();
            parameters[i + 1] = parameters[i] + (ordered[next] - ordered[i]).length();
        }

        if parameters[segment_count] <= 0.0 {
            return Err(ChainFieldArgumentError::ZeroLength);
        }

        Ok(ChainField::new(
            ordered,
            is_closed,
            backs,
            parameters,
            min_setback,
            max_setback,
            sharp_turn_degrees,
            end_rays,
            end_ray_bounds,
        ))
    }
}

impl ChainField {
    /// The same chain with every
    /// setback times `factor`, rebuilt with the same threshold, end rays and bounds. Each point is
    /// charged like any other reading of one, so a climb that keeps asking is cancellable.
    pub fn with_setbacks_scaled(&self, factor: f64) -> Result<ChainField, WorkError> {
        let mut scaled = Vec::with_capacity(self.setbacks.len());

        for &setback in &self.setbacks {
            LevelLineWork::charge(1)?;
            scaled.push(setback * factor);
        }

        Ok(ChainField::from_polyline_bounded(
            &self.points,
            self.is_closed,
            &scaled,
            self.sharp_turn_degrees,
            self.end_rays,
            self.end_ray_bounds,
        )
        .expect("a field's own polyline always rebuilds"))
    }

    /// The reading by visiting every segment and its rays, no tree - what
    /// the tree's answer is checked against. An exact tie keeps the earlier segment.
    pub fn evaluate_exhaustive(&self, point: Vector3) -> FieldSample {
        let segment_count = self.segment_count() as usize;
        let mut found = ChainFieldNearest::start(self.points[0]);

        for i in 0..segment_count {
            let (value, t, nearest, distance) = self.least_over_segment_from_point(i, point);

            // Strictly less, with a few bits of room; the first segment is taken outright.
            if i == 0 || value < found.best - (ChainField::TIE_RELATIVE * (1.0 + found.best.abs()))
            {
                found.best = value;
                found.nearest = nearest;
                found.distance = distance;
                let segment_length = self.parameter_at_point[i + 1] - self.parameter_at_point[i];
                found.parameter = self.parameter_at_point[i] + (segment_length * t);
            }

            self.nearest_over_extensions(i, point, &mut found);
        }

        self.clipped(
            point,
            FieldSample {
                distance: found.distance,
                parameter: self.across_smooth_joints(found.parameter, point),
                value: found.best,
                nearest: found.nearest,
                ..Default::default()
            },
        )
    }

    /// The segment a chain parameter falls on and how far along it - wrapped on
    /// a closed chain, clamped on an open one, read through the smooth-joint spans first.
    pub fn segment_holding(&self, parameter: f64) -> (i32, f64) {
        let mut parameter = self.on_the_chain_from_spans(parameter);

        if self.is_closed {
            parameter -= (parameter / self.length).floor() * self.length;
            if parameter >= self.length {
                parameter = 0.0;
            }
        } else {
            parameter = parameter.clamp(0.0, self.length);
        }

        let mut low = 0;
        let mut high = self.segment_count() - 1;

        while low < high {
            let middle = (low + high) / 2;
            if self.parameter_at_point[(middle + 1) as usize] >= parameter {
                high = middle;
            } else {
                low = middle + 1;
            }
        }

        (low, parameter - self.parameter_at_point[low as usize])
    }
}
