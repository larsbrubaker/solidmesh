//! The chain fields the R2.P02 rust-only tests read, assembled as `ChainField.FromPolyline`
//! (R2.P05) will assemble one for an open chain with no sharp turn: points, setbacks, the parameter
//! at each point, the segment box tree and nothing clipped.

use std::cell::OnceCell;

use solidmesh::bevel::level_line::{segment_box_tree, ChainField};
use solidmesh_math::Vector3;

/// An open chain through `points` asking `setbacks`, no sharp turn, no end ray, no clip.
pub fn open_chain(points: Vec<Vector3>, setbacks: Vec<f64>) -> ChainField {
    let mut parameter_at_point = vec![0.0];
    for i in 1..points.len() {
        let previous = parameter_at_point[i - 1];
        parameter_at_point.push(previous + (points[i] - points[i - 1]).length());
    }

    let segment_count = points.len() as i32 - 1;
    let nodes = segment_box_tree::build(&points, &setbacks, segment_count, None);
    let length = parameter_at_point[points.len() - 1];
    let min_setback = setbacks.iter().copied().fold(f64::INFINITY, f64::min);
    let max_setback = setbacks.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    ChainField {
        points,
        setbacks,
        parameter_at_point,
        nodes,
        smooth_joints: None,
        sharp_turn_degrees: f64::INFINITY,
        end_rays: (false, false),
        end_ray_bounds: (None, None),
        is_closed: false,
        length,
        min_setback,
        max_setback,
        extensions: None,
        end_clips: None,
        end_plane_reader: OnceCell::new(),
        sharp_joints: OnceCell::new(),
        known_cuts: OnceCell::new(),
    }
}

/// An L: (0,0,0) to (4,0,0) to (4,3,0), setback 1 then 2 then 1.
pub fn l_chain() -> ChainField {
    open_chain(
        vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(4.0, 0.0, 0.0),
            Vector3::new(4.0, 3.0, 0.0),
        ],
        vec![1.0, 2.0, 1.0],
    )
}
