//! The MITRE POINT rule of a pick with corners: on every band triangle where two member chains of
//! the group both reach the line, the point where BOTH read zero (where their two level lines meet)
//! is made a mesh vertex before the refinement runs. Ports `Bevel/LevelLine/CornerTiePoints.cs`
//! (a C# static class): so far the Newton solve for that point inside a triangle ([`try_mitre`],
//! [`try_mitre_from`]); the insertion lands with its method package.

use vector_math::{cs_math, Vector3};

use super::{ChainField, LevelLineWork, WorkError};

/// C# `CornerTiePoints.TryMitre`: the point of the triangle abc where both chains read zero, by
/// Newton from a fixed list of starts (centroid, corners, edge midpoints, and the corners pulled a
/// little inside); `None` where no start converges inside the triangle.
///
/// The corners pulled a little inside are starts too. On a coarse fan (the L block's top face)
/// every one of the seven points above can fail: the centre and the long edges' midpoints read the
/// second chain off a segment PARALLEL to the first chain's (a singular Jacobian), and the corners
/// and the short edge's midpoint lie ON a chain, where the distance kinks and the one-sided
/// difference steers Newton to the mirror root outside the face.
pub fn try_mitre(
    first: &ChainField,
    second: &ChainField,
    a: Vector3,
    b: Vector3,
    c: Vector3,
) -> Result<Option<Vector3>, WorkError> {
    let third = 1.0 / 3.0;
    for (s, t) in [
        (third, third),
        (0.0, 0.0),
        (1.0, 0.0),
        (0.0, 1.0),
        (0.5, 0.0),
        (0.5, 0.5),
        (0.0, 0.5),
        (0.1, 0.1),
        (0.8, 0.1),
        (0.1, 0.8),
    ] {
        if let Some(point) = try_mitre_from(first, second, a, b, c, s, t)? {
            return Ok(Some(point));
        }
    }

    Ok(None)
}

/// C# `CornerTiePoints.TryMitreFrom`: Newton on (first, second) = (0, 0) over the triangle's
/// barycentric (s, t) from one start, with a forward-difference Jacobian; at most 40 steps, each
/// charged. `None` where the Jacobian is singular, the iterate runs off, or the root lands outside
/// the triangle (by more than a slack).
#[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
pub fn try_mitre_from(
    first: &ChainField,
    second: &ChainField,
    a: Vector3,
    b: Vector3,
    c: Vector3,
    mut s: f64,
    mut t: f64,
) -> Result<Option<Vector3>, WorkError> {
    let e1 = b - a;
    let e2 = c - a;
    let size = cs_math::max_f64(e1.length(), cs_math::max_f64(e2.length(), (c - b).length()));
    let h = 1e-7;
    let f_at = |u: f64, v: f64| -> Result<(f64, f64), WorkError> {
        let p = a + (e1 * u) + (e2 * v);
        Ok((first.evaluate(p)?.value, second.evaluate(p)?.value))
    };

    for _ in 0..40 {
        LevelLineWork::charge(1)?;
        let (f, g) = f_at(s, t)?;
        if f.abs() <= 1e-13 * size && g.abs() <= 1e-13 * size {
            const SLACK: f64 = 1e-9;
            if s < -SLACK || t < -SLACK || s + t > 1.0 + SLACK {
                return Ok(None);
            }

            return Ok(Some(a + (e1 * s) + (e2 * t)));
        }

        let (fs, gs) = f_at(s + h, t)?;
        let (ft, gt) = f_at(s, t + h)?;
        let j11 = (fs - f) / h;
        let j12 = (ft - f) / h;
        let j21 = (gs - g) / h;
        let j22 = (gt - g) / h;
        let det = (j11 * j22) - (j12 * j21);
        if !(det.abs() > 1e-12 * size * size) {
            return Ok(None);
        }

        s -= ((j22 * f) - (j12 * g)) / det;
        t -= ((j11 * g) - (j21 * f)) / det;
        if !s.is_finite() || !t.is_finite() || s.abs() > 10.0 || t.abs() > 10.0 {
            return Ok(None);
        }
    }

    Ok(None)
}
