//! The rays a chain field's segments also answer for: the MITRE rays at sharp vertices and the
//! END RAYS run on past an open chain's ends. The type lives in `chain_field.rs`; a ray's reading
//! is in `chain_field_nearest.rs`.

use vector_math::{ieee_math, Vector3};

use super::{ChainField, ChainFieldExtension, EndRayBound};

/// Per segment, the rays it also answers for (`None` for a segment with none).
pub type ChainFieldRays = Vec<Option<Vec<ChainFieldExtension>>>;

/// The cosine a joint's two unit directions must fall under to turn sharper than
/// `sharp_turn_degrees`.
pub(crate) fn sharp_turn_cosine(sharp_turn_degrees: f64) -> f64 {
    (sharp_turn_degrees * std::f64::consts::PI / 180.0).cos()
}

/// Whether a joint whose chain runs in along unit `d_in` and out along unit `d_out` is mitred:
/// it turns past the threshold (`cos_threshold`, [`sharp_turn_cosine`]), and is no full reversal,
/// which has no bisector. Shared with the rolling ball's legs (`rolling_ball_legs`), so the chain
/// is cut into legs exactly where its field mitres.
pub(crate) fn mitres_at(d_in: Vector3, d_out: Vector3, cos_threshold: f64) -> bool {
    let cos = d_in.dot(d_out);
    !(cos >= cos_threshold || 1.0 + cos < 1e-9)
}

/// The chain's bounding-box diagonal.
fn box_diagonal(points: &[Vector3]) -> f64 {
    let mut box_min = points[0];
    let mut box_max = points[0];
    for &p in points {
        box_min = Vector3::component_min(box_min, p);
        box_max = Vector3::component_max(box_max, p);
    }

    (box_max - box_min).length()
}

impl ChainField {
    /// The rays each segment answers for at the chain's sharp vertices, or `None` when there are
    /// none (or no threshold was given) - in which case the field is exactly the plain polyline's.
    /// A joint turning by more than `sharp_turn_degrees` (and not a smooth joint) is mitred: each
    /// of its two segments carries its line on past the vertex, read only on its side of the
    /// bisector, so the field's level set turns a sharp corner there instead of a round one.
    pub fn find_mitres(
        points: &[Vector3],
        is_closed: bool,
        setbacks: &[f64],
        parameter_at_point: &[f64],
        sharp_turn_degrees: f64,
        is_smooth_joint: &dyn Fn(i32) -> bool,
    ) -> Option<ChainFieldRays> {
        if !sharp_turn_degrees.is_finite() {
            return None;
        }

        let n = points.len();
        let segment_count = parameter_at_point.len() - 1;
        let diagonal = box_diagonal(points);
        let cos_threshold = sharp_turn_cosine(sharp_turn_degrees);
        let mut found: Option<ChainFieldRays> = None;
        for j in 0..n {
            if is_smooth_joint(j as i32) {
                continue;
            }

            if !is_closed && (j == 0 || j == n - 1) {
                continue;
            }

            let in_segment = (j + segment_count - 1) % segment_count;
            let out_segment = j;
            let mut d_in = points[j] - points[(j + n - 1) % n];
            let mut d_out = points[(j + 1) % n] - points[j];
            if d_in.length_squared() <= 0.0 || d_out.length_squared() <= 0.0 {
                continue;
            }

            d_in = d_in.get_normal();
            d_out = d_out.get_normal();
            let cos = d_in.dot(d_out);

            // At or below the threshold the vertex keeps its ball; a full reversal has no bisector.
            if !mitres_at(d_in, d_out, cos_threshold) {
                continue;
            }

            let bisector = d_in + d_out;
            let tan_half = d_in.cross(d_out).length() / (1.0 + cos);
            let reach = (setbacks[j] * tan_half) + diagonal;
            let parameter = parameter_at_point[j];
            let rays = found.get_or_insert_with(|| vec![None; segment_count]);
            rays[in_segment]
                .get_or_insert_with(Vec::new)
                .push(ChainFieldExtension::new(
                    points[j],
                    d_in,
                    -bisector,
                    reach,
                    setbacks[j],
                    parameter,
                    0.0,
                ));
            rays[out_segment]
                .get_or_insert_with(Vec::new)
                .push(ChainFieldExtension::new(
                    points[j],
                    -d_out,
                    bisector,
                    reach,
                    setbacks[j],
                    parameter,
                    0.0,
                ));
        }

        found
    }

    /// `extensions` with an END RAY added past each open end `end_rays` names: the chain's line run
    /// on past that end, read with a parameter beyond it. A bounded ray runs no farther than the
    /// rails need to reach the end face's plane (with [`ChainField::END_RAY_SLACK`]): past it the
    /// band is clipped, and a ray running on through the part reads the far side of whatever the
    /// end face bounds. Never past the unbounded reach, the box diagonal plus four setbacks.
    pub fn with_end_rays(
        extensions: Option<ChainFieldRays>,
        points: &[Vector3],
        setbacks: &[f64],
        parameter_at_point: &[f64],
        end_rays: (bool, bool),
        bounds: (Option<EndRayBound>, Option<EndRayBound>),
    ) -> Option<ChainFieldRays> {
        if !end_rays.0 && !end_rays.1 {
            return extensions;
        }

        let segment_count = parameter_at_point.len() - 1;
        let diagonal = box_diagonal(points);
        let mut result = extensions.unwrap_or_else(|| vec![None; segment_count]);

        let reach = |bound: Option<EndRayBound>, setback: f64| {
            let unbounded = diagonal + (4.0 * setback);
            match bound {
                Some(bound) => ieee_math::min_f64(
                    unbounded,
                    bound.reach_per_setback * setback * (1.0 + Self::END_RAY_SLACK),
                ),
                None => unbounded,
            }
        };

        let n = points.len();
        if end_rays.0 {
            let outward = (points[0] - points[1]).get_normal();
            result[0]
                .get_or_insert_with(Vec::new)
                .push(ChainFieldExtension::new(
                    points[0],
                    outward,
                    outward,
                    reach(bounds.0, setbacks[0]),
                    setbacks[0],
                    parameter_at_point[0],
                    -1.0,
                ));
        }

        if end_rays.1 {
            let outward = (points[n - 1] - points[n - 2]).get_normal();
            result[segment_count - 1]
                .get_or_insert_with(Vec::new)
                .push(ChainFieldExtension::new(
                    points[n - 1],
                    outward,
                    outward,
                    reach(bounds.1, setbacks[n - 1]),
                    setbacks[n - 1],
                    parameter_at_point[segment_count],
                    1.0,
                ));
        }

        Some(result)
    }
}
