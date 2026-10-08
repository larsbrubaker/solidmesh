//! g3's point-triangle distance (`MeshQueries.TriDistanceSqr` and
//! `DistPoint3Triangle3.DistanceSqr`, which are the same computation), used by
//! [`super::DMeshAABBTree3`] and the signed distance grids.

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// g3 `MeshQueries.TriDistanceSqr` / `DistPoint3Triangle3.DistanceSqr` (Eberly's region walk):
/// the squared distance from `point` to the triangle `v`.
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

    // C# `if (sqrDistance < 0) sqrDistance = 0` (NaN passes through, as in C#).
    if sqr_distance < 0.0 {
        0.0
    } else {
        sqr_distance
    }
}

/// g3 `DistPoint3Triangle3.DistanceSqr(point, triangle, out closestPoint, out baryCoords)`: the
/// squared distance from `point` to the triangle `v` and the triangle's point nearest to it,
/// which `MeshProjectionTarget.Project` (`mesh_projection_target.rs`) returns. The same region
/// walk as [`tri_distance_sqr`], also tracking the parameters `s`, `t` of that point.
pub(crate) fn distance_sqr_closest(v: &[[f64; 3]; 3], point: [f64; 3]) -> (f64, [f64; 3]) {
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

    // C# `triangle.V0 + s * edge0 + t * edge1`, left to right.
    let closest = [
        v[0][0] + s * edge0[0] + t * edge1[0],
        v[0][1] + s * edge0[1] + t * edge1[1],
        v[0][2] + s * edge0[2] + t * edge1[2],
    ];
    // C# `Math.Max(sqrDistance, 0)` (NaN propagates, as .NET's Math.Max does).
    let sqr_distance = if sqr_distance.is_nan() || sqr_distance >= 0.0 {
        sqr_distance
    } else {
        0.0
    };
    (sqr_distance, closest)
}
