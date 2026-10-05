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
