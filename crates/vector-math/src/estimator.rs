//! Movement-time estimates under a trapezoidal velocity profile: accelerate from the "same as
//! stop" velocity, cruise at the lesser of the requested rate and the maximum, decelerate
//! symmetrically.
//!
//! `get_seconds_for_movement_vector` takes a 3D move: it uses the delta's length and each
//! limit's X.

use crate::Vector3;

/// Seconds to move `delta.length()`; only the X of each limit vector is used.
pub fn get_seconds_for_movement_vector(
    delta: Vector3,
    rate_per_second: f64,
    max_acceleration_per_s2: Vector3,
    max_velocity_per_s: Vector3,
    velocity_same_as_stop_per_s: Vector3,
    speed_multiplier: Vector3,
) -> f64 {
    get_seconds_for_movement(
        delta.length(),
        rate_per_second,
        max_acceleration_per_s2.x,
        max_velocity_per_s.x,
        velocity_same_as_stop_per_s.x,
        speed_multiplier.x,
    )
}

/// Seconds to move `delta` (`speed_multiplier` is usually 1).
pub fn get_seconds_for_movement(
    delta: f64,
    rate_per_second: f64,
    max_acceleration_per_s2: f64,
    max_velocity_per_s: f64,
    velocity_same_as_stop_per_s: f64,
    speed_multiplier: f64,
) -> f64 {
    if delta == 0.0 {
        return 0.0;
    }

    let max_velocity_mm_per_sx = rate_per_second.min(max_velocity_per_s);
    let starting_velocity_mm_per_s = velocity_same_as_stop_per_s.min(max_velocity_mm_per_sx);

    let distance_to_max_velocity = get_distance_to_reach_ending_velocity(
        starting_velocity_mm_per_s,
        max_velocity_mm_per_sx,
        max_acceleration_per_s2,
    );
    if distance_to_max_velocity <= delta / 2.0 {
        // we will reach max velocity then run at it and then decelerate
        let acceleration_time = get_time_to_accelerate_distance(
            starting_velocity_mm_per_s,
            distance_to_max_velocity,
            max_acceleration_per_s2,
        ) * 2.0;
        let running_time = (delta - (distance_to_max_velocity * 2.0)) / max_velocity_mm_per_sx;
        (acceleration_time + running_time) * speed_multiplier
    } else {
        // we will accelerate to the center then decelerate
        let acceleration_time = get_time_to_accelerate_distance(
            starting_velocity_mm_per_s,
            delta / 2.0,
            max_acceleration_per_s2,
        ) * 2.0;
        acceleration_time * speed_multiplier
    }
}

fn get_distance_to_reach_ending_velocity(
    starting_velocity_mm_per_s: f64,
    ending_velocity_mm_per_s: f64,
    acceleration_mm_per_s2: f64,
) -> f64 {
    let ending_velocity_mm_per_s2 = ending_velocity_mm_per_s * ending_velocity_mm_per_s;
    let starting_velocity_mm_per_s2 = starting_velocity_mm_per_s * starting_velocity_mm_per_s;
    (ending_velocity_mm_per_s2 - starting_velocity_mm_per_s2) / (2.0 * acceleration_mm_per_s2)
}

fn get_time_to_accelerate_distance(
    starting_velocity_mm_per_s: f64,
    distance_mm: f64,
    acceleration_mm_per_s2: f64,
) -> f64 {
    // d = vi * t + .5 * a * t^2;
    // t = (√(vi^2+2ad)-vi)/a
    let starting_velocity_mm_per_s2 = starting_velocity_mm_per_s * starting_velocity_mm_per_s;
    let distance_acceleration2 = 2.0 * acceleration_mm_per_s2 * distance_mm;
    ((starting_velocity_mm_per_s2 + distance_acceleration2).sqrt() - starting_velocity_mm_per_s)
        / acceleration_mm_per_s2
}
