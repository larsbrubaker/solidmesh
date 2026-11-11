//! The point-triangle distance (derived from geometry3Sharp; Eberly's region walk), used by
//! [`super::DMeshAABBTree3`], the signed distance grids and the mesh projection target.

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// The squared distance from `point` to the segment `a`-`b` and the segment's point nearest to
/// it; a zero-length segment is its one point.
fn segment_distance_sqr_closest(a: [f64; 3], b: [f64; 3], point: [f64; 3]) -> (f64, [f64; 3]) {
    let edge = sub(b, a);
    let length_sqr = dot(edge, edge);
    let along = if length_sqr > 0.0 {
        (dot(sub(point, a), edge) / length_sqr).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let closest = [
        a[0] + along * edge[0],
        a[1] + along * edge[1],
        a[2] + along * edge[2],
    ];
    let offset = sub(point, closest);
    (dot(offset, offset), closest)
}

/// A zero-area triangle (its corners on one line, or coincident) has no interior, so its nearest
/// point lies on one of its three edges; the region walk would divide by its zero area.
fn degenerate_distance_sqr_closest(v: &[[f64; 3]; 3], point: [f64; 3]) -> (f64, [f64; 3]) {
    let mut best = segment_distance_sqr_closest(v[0], v[1], point);
    for (a, b) in [(v[1], v[2]), (v[2], v[0])] {
        let candidate = segment_distance_sqr_closest(a, b, point);
        // Ties keep the earlier edge, so the result does not depend on rounding order.
        if candidate.0 < best.0 {
            best = candidate;
        }
    }
    best
}

/// The parameters `[s, t]` of the point [`degenerate_distance_sqr_closest`] finds: on the same
/// nearest edge (ties to the earlier one), at the nearest point's place along it.
fn degenerate_params(v: &[[f64; 3]; 3], point: [f64; 3]) -> [f64; 2] {
    let along = |a: [f64; 3], b: [f64; 3]| {
        let d = sub(b, a);
        let len_sqr = dot(d, d);
        if len_sqr > 0.0 {
            (dot(sub(point, a), d) / len_sqr).clamp(0.0, 1.0)
        } else {
            0.0
        }
    };
    let mut best = (segment_distance_sqr_closest(v[0], v[1], point).0, 0);
    for (edge, (a, b)) in [(1, (v[1], v[2])), (2, (v[2], v[0]))] {
        let candidate = segment_distance_sqr_closest(a, b, point).0;
        if candidate < best.0 {
            best = (candidate, edge);
        }
    }
    match best.1 {
        0 => [along(v[0], v[1]), 0.0],
        1 => {
            let l = along(v[1], v[2]);
            [1.0 - l, l]
        }
        _ => [0.0, 1.0 - along(v[2], v[0])],
    }
}

/// The squared distance from `point` to the triangle `v`.
pub(crate) fn tri_distance_sqr(v: &[[f64; 3]; 3], point: [f64; 3]) -> f64 {
    let diff = sub(v[0], point);
    let edge0 = sub(v[1], v[0]);
    let edge1 = sub(v[2], v[0]);
    let a00 = dot(edge0, edge0);
    let a01 = dot(edge0, edge1);
    let a11 = dot(edge1, edge1);
    let b0 = dot(diff, edge0);
    let b1 = dot(diff, edge1);
    let c = dot(diff, diff);
    let det = (a00 * a11 - a01 * a01).abs();
    if det == 0.0 {
        return degenerate_distance_sqr_closest(v, point).0;
    }
    let mut s = a01 * b1 - a11 * b0;
    let mut t = a01 * b0 - a00 * b1;
    let sqr_distance;

    if s + t <= det {
        if s < 0.0 {
            if t < 0.0 {
                // region 4
                if b0 < 0.0 {
                    if -b0 >= a00 {
                        sqr_distance = a00 + 2.0 * b0 + c;
                    } else {
                        s = -b0 / a00;
                        sqr_distance = b0 * s + c;
                    }
                } else if b1 >= 0.0 {
                    sqr_distance = c;
                } else if -b1 >= a11 {
                    sqr_distance = a11 + 2.0 * b1 + c;
                } else {
                    t = -b1 / a11;
                    sqr_distance = b1 * t + c;
                }
            } else {
                // region 3
                if b1 >= 0.0 {
                    sqr_distance = c;
                } else if -b1 >= a11 {
                    sqr_distance = a11 + 2.0 * b1 + c;
                } else {
                    t = -b1 / a11;
                    sqr_distance = b1 * t + c;
                }
            }
        } else if t < 0.0 {
            // region 5
            if b0 >= 0.0 {
                sqr_distance = c;
            } else if -b0 >= a00 {
                sqr_distance = a00 + 2.0 * b0 + c;
            } else {
                s = -b0 / a00;
                sqr_distance = b0 * s + c;
            }
        } else {
            // region 0: minimum at an interior point
            let inv_det = 1.0 / det;
            s *= inv_det;
            t *= inv_det;
            sqr_distance =
                s * (a00 * s + a01 * t + 2.0 * b0) + t * (a01 * s + a11 * t + 2.0 * b1) + c;
        }
    } else if s < 0.0 {
        // region 2
        let tmp0 = a01 + b0;
        let tmp1 = a11 + b1;
        if tmp1 > tmp0 {
            let numer = tmp1 - tmp0;
            let denom = a00 - 2.0 * a01 + a11;
            if numer >= denom {
                sqr_distance = a00 + 2.0 * b0 + c;
            } else {
                s = numer / denom;
                t = 1.0 - s;
                sqr_distance =
                    s * (a00 * s + a01 * t + 2.0 * b0) + t * (a01 * s + a11 * t + 2.0 * b1) + c;
            }
        } else if tmp1 <= 0.0 {
            sqr_distance = a11 + 2.0 * b1 + c;
        } else if b1 >= 0.0 {
            sqr_distance = c;
        } else {
            t = -b1 / a11;
            sqr_distance = b1 * t + c;
        }
    } else if t < 0.0 {
        // region 6
        let tmp0 = a01 + b1;
        let tmp1 = a00 + b0;
        if tmp1 > tmp0 {
            let numer = tmp1 - tmp0;
            let denom = a00 - 2.0 * a01 + a11;
            if numer >= denom {
                sqr_distance = a11 + 2.0 * b1 + c;
            } else {
                t = numer / denom;
                s = 1.0 - t;
                sqr_distance =
                    s * (a00 * s + a01 * t + 2.0 * b0) + t * (a01 * s + a11 * t + 2.0 * b1) + c;
            }
        } else if tmp1 <= 0.0 {
            sqr_distance = a00 + 2.0 * b0 + c;
        } else if b0 >= 0.0 {
            sqr_distance = c;
        } else {
            s = -b0 / a00;
            sqr_distance = b0 * s + c;
        }
    } else {
        // region 1
        let numer = a11 + b1 - a01 - b0;
        if numer <= 0.0 {
            sqr_distance = a11 + 2.0 * b1 + c;
        } else {
            let denom = a00 - 2.0 * a01 + a11;
            if numer >= denom {
                sqr_distance = a00 + 2.0 * b0 + c;
            } else {
                s = numer / denom;
                t = 1.0 - s;
                sqr_distance =
                    s * (a00 * s + a01 * t + 2.0 * b0) + t * (a01 * s + a11 * t + 2.0 * b1) + c;
            }
        }
    }

    // Rounding can leave a tiny negative; clamp it to zero (NaN passes through).
    if sqr_distance < 0.0 {
        0.0
    } else {
        sqr_distance
    }
}

/// The squared distance from `point` to the triangle `v` and the triangle's point nearest to
/// it, which `MeshProjectionTarget::project` (`mesh_projection_target.rs`) returns.
pub(crate) fn distance_sqr_closest(v: &[[f64; 3]; 3], point: [f64; 3]) -> (f64, [f64; 3]) {
    let (sqr_distance, closest, _) = distance_sqr_closest_params(v, point);
    (sqr_distance, closest)
}

/// [`distance_sqr_closest`] with the parameters `[s, t]` of the nearest point,
/// `v0 + s * (v1 - v0) + t * (v2 - v0)`, which the reducer reads a collapse point's UV from
/// (`reducer_face_data.rs`). The same region walk as [`tri_distance_sqr`]; a degenerate
/// triangle reports the place of the nearest point along its nearest edge.
pub(crate) fn distance_sqr_closest_params(
    v: &[[f64; 3]; 3],
    point: [f64; 3],
) -> (f64, [f64; 3], [f64; 2]) {
    let diff = sub(v[0], point);
    let edge0 = sub(v[1], v[0]);
    let edge1 = sub(v[2], v[0]);
    let a00 = dot(edge0, edge0);
    let a01 = dot(edge0, edge1);
    let a11 = dot(edge1, edge1);
    let b0 = dot(diff, edge0);
    let b1 = dot(diff, edge1);
    let c = dot(diff, diff);
    let det = (a00 * a11 - a01 * a01).abs();
    if det == 0.0 {
        let (sqr_distance, closest) = degenerate_distance_sqr_closest(v, point);
        return (sqr_distance, closest, degenerate_params(v, point));
    }
    let mut s = a01 * b1 - a11 * b0;
    let mut t = a01 * b0 - a00 * b1;
    let sqr_distance;

    // `s = numer / denom; t = 1 - s` and its mirror share this interior-of-edge distance.
    let inner = |s: f64, t: f64| {
        s * (a00 * s + a01 * t + 2.0 * b0) + t * (a01 * s + a11 * t + 2.0 * b1) + c
    };

    if s + t <= det {
        if s < 0.0 {
            if t < 0.0 {
                // region 4
                if b0 < 0.0 {
                    t = 0.0;
                    if -b0 >= a00 {
                        s = 1.0;
                        sqr_distance = a00 + 2.0 * b0 + c;
                    } else {
                        s = -b0 / a00;
                        sqr_distance = b0 * s + c;
                    }
                } else {
                    s = 0.0;
                    if b1 >= 0.0 {
                        t = 0.0;
                        sqr_distance = c;
                    } else if -b1 >= a11 {
                        t = 1.0;
                        sqr_distance = a11 + 2.0 * b1 + c;
                    } else {
                        t = -b1 / a11;
                        sqr_distance = b1 * t + c;
                    }
                }
            } else {
                // region 3
                s = 0.0;
                if b1 >= 0.0 {
                    t = 0.0;
                    sqr_distance = c;
                } else if -b1 >= a11 {
                    t = 1.0;
                    sqr_distance = a11 + 2.0 * b1 + c;
                } else {
                    t = -b1 / a11;
                    sqr_distance = b1 * t + c;
                }
            }
        } else if t < 0.0 {
            // region 5
            t = 0.0;
            if b0 >= 0.0 {
                s = 0.0;
                sqr_distance = c;
            } else if -b0 >= a00 {
                s = 1.0;
                sqr_distance = a00 + 2.0 * b0 + c;
            } else {
                s = -b0 / a00;
                sqr_distance = b0 * s + c;
            }
        } else {
            // region 0: minimum at an interior point
            let inv_det = 1.0 / det;
            s *= inv_det;
            t *= inv_det;
            sqr_distance = inner(s, t);
        }
    } else if s < 0.0 {
        // region 2
        let tmp0 = a01 + b0;
        let tmp1 = a11 + b1;
        if tmp1 > tmp0 {
            let numer = tmp1 - tmp0;
            let denom = a00 - 2.0 * a01 + a11;
            if numer >= denom {
                s = 1.0;
                t = 0.0;
                sqr_distance = a00 + 2.0 * b0 + c;
            } else {
                s = numer / denom;
                t = 1.0 - s;
                sqr_distance = inner(s, t);
            }
        } else {
            s = 0.0;
            if tmp1 <= 0.0 {
                t = 1.0;
                sqr_distance = a11 + 2.0 * b1 + c;
            } else if b1 >= 0.0 {
                t = 0.0;
                sqr_distance = c;
            } else {
                t = -b1 / a11;
                sqr_distance = b1 * t + c;
            }
        }
    } else if t < 0.0 {
        // region 6
        let tmp0 = a01 + b1;
        let tmp1 = a00 + b0;
        if tmp1 > tmp0 {
            let numer = tmp1 - tmp0;
            let denom = a00 - 2.0 * a01 + a11;
            if numer >= denom {
                t = 1.0;
                s = 0.0;
                sqr_distance = a11 + 2.0 * b1 + c;
            } else {
                t = numer / denom;
                s = 1.0 - t;
                sqr_distance = inner(s, t);
            }
        } else {
            t = 0.0;
            if tmp1 <= 0.0 {
                s = 1.0;
                sqr_distance = a00 + 2.0 * b0 + c;
            } else if b0 >= 0.0 {
                s = 0.0;
                sqr_distance = c;
            } else {
                s = -b0 / a00;
                sqr_distance = b0 * s + c;
            }
        }
    } else {
        // region 1
        let numer = a11 + b1 - a01 - b0;
        if numer <= 0.0 {
            s = 0.0;
            t = 1.0;
            sqr_distance = a11 + 2.0 * b1 + c;
        } else {
            let denom = a00 - 2.0 * a01 + a11;
            if numer >= denom {
                s = 1.0;
                t = 0.0;
                sqr_distance = a00 + 2.0 * b0 + c;
            } else {
                s = numer / denom;
                t = 1.0 - s;
                sqr_distance = inner(s, t);
            }
        }
    }

    // `v0 + s * edge0 + t * edge1`, added left to right.
    let closest = [
        v[0][0] + s * edge0[0] + t * edge1[0],
        v[0][1] + s * edge0[1] + t * edge1[1],
        v[0][2] + s * edge0[2] + t * edge1[2],
    ];
    // Rounding can leave a tiny negative; clamp it to zero (NaN propagates).
    let sqr_distance = if sqr_distance.is_nan() || sqr_distance >= 0.0 {
        sqr_distance
    } else {
        0.0
    };
    (sqr_distance, closest, [s, t])
}
