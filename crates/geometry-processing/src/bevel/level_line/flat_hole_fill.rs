//! An END FACE end's hole closed FLAT - the Ends decision's cut into the face. The loop
//! is triangulated in its own plane by ear clipping, so no star-shaped test applies (the fan's
//! [`GapFillRefusal::CornerCapNotStarShaped`](super::GapFillRefusal) cannot arise), and a triangle
//! standing less than the snap off its longest edge is dropped rather than laid. Its triangles are
//! [`HoleFillTriangle`]s, as the faired filler's are.

use solidmesh_math::{ieee_math, Vector2, Vector3};

use super::HoleFillTriangle;

/// Corner triples (loop indices) that may never be an ear.
pub type ForbiddenEar<'a> = &'a dyn Fn(i32, i32, i32) -> bool;

/// The triangles closing `loop_points` flat, their corners indices into the loop, with the plane's
/// unit normal on the side the loop faces - or `None` where the loop does not lie in one plane
/// within `snap` (hand it to the faired filler), has no area, or crosses itself. Every triangle
/// winds the way the loop is walked - as `HoleFill.Fill`
/// does, each loop edge i -> i+1 that is laid runs that way in its triangle - so a loop walked as
/// the cap lays its edges faces out of the part.
///
/// The snap is the kernel's own (the value the band cut snaps to), never a tolerance tuned here:
/// a vertex within it of the plane is in the plane, a triangle within it of flat has no area. A
/// vertex standing on a candidate ear's diagonal blocks that ear, so no T-junction is laid; a
/// collinear or coincident vertex is never an ear's tip while a real ear is left, and is then
/// dropped with its no-area triangle. Among the ears the one with the widest smallest angle is cut
/// first, which keeps slivers out of a convex run.
///
/// `forbidden`, where given, names corner triples (loop indices) that may never be an ear: they
/// are skipped as a blocked ear is, and the fill fails (`None`) when only such ears are left.
/// `None` keeps every ear open.
pub fn fill(
    loop_points: &[Vector3],
    snap: f64,
    forbidden: Option<ForbiddenEar>,
) -> Option<(Vec<HoleFillTriangle>, Vector3)> {
    let n = loop_points.len();
    if n < 3 {
        return None;
    }

    let is_forbidden =
        |a: usize, b: usize, c: usize| forbidden.is_some_and(|f| f(a as i32, b as i32, c as i32));

    // Newell's normal: its sign follows the walk, so the loop runs counter-clockwise in the (u, w)
    // plane below and every ear cut from it winds the loop's way
    let mut newell = Vector3::ZERO;
    for i in 0..n {
        let a = loop_points[i];
        let b = loop_points[(i + 1) % n];
        newell += Vector3::new(
            (a.y - b.y) * (a.z + b.z),
            (a.z - b.z) * (a.x + b.x),
            (a.x - b.x) * (a.y + b.y),
        );
    }

    if newell.length() <= snap * snap {
        return None;
    }

    let unit = newell.get_normal();
    let origin = loop_points.iter().fold(Vector3::ZERO, |s, &p| s + p) / n as f64;
    if loop_points
        .iter()
        .any(|&p| (p - origin).dot(unit).abs() > snap)
    {
        return None;
    }

    let normal = unit;
    let u = if unit.x.abs() < 0.9 {
        Vector3::UNIT_X
    } else {
        Vector3::UNIT_Y
    }
    .cross(unit)
    .get_normal();
    let w = unit.cross(u);
    let flat: Vec<Vector2> = loop_points
        .iter()
        .map(|&p| Vector2::new((p - origin).dot(u), (p - origin).dot(w)))
        .collect();

    let mut remaining: Vec<usize> = (0..n).collect();
    let mut triangles = Vec::new();
    while remaining.len() > 3 {
        let m = remaining.len();
        let mut best: Option<usize> = None;
        let mut best_angle = f64::MIN;
        let mut degenerate: Option<usize> = None;
        for k in 0..m {
            let a = remaining[(k + m - 1) % m];
            let b = remaining[k];
            let c = remaining[(k + 1) % m];
            let height = height(flat[a], flat[b], flat[c]);
            if height.abs() <= snap {
                degenerate = degenerate.or(Some(k));
                continue;
            }

            if height < 0.0 || blocks_ear(&flat, &remaining, a, b, c, snap) || is_forbidden(a, b, c)
            {
                continue;
            }

            // The ear is scored by the ears it LEAVES as well as its own: a diagonal lying along a
            // near straight run of the loop leaves, at each end, an ear of that run's next vertex
            // that is all but flat, and every triangulation of the sliver it cuts off is a column
            // of needles. Those needles stood 1e-7 off their longest edge - above the snap, under a
            // float's step - so the app's single-precision answer read them flipped over their
            // neighbours (SelfCrossingChainTests c7 and c17); fanned from across the hole instead,
            // the same run lays triangles 1e-5 tall.
            let mut angle = smallest_angle(flat[a], flat[b], flat[c]);
            if m > 3 {
                let before = remaining[(k + m - 2) % m];
                let after = remaining[(k + 2) % m];
                angle = ieee_math::min_f64(
                    angle,
                    ieee_math::min_f64(
                        left_ear(flat[before], flat[a], flat[c]),
                        left_ear(flat[a], flat[c], flat[after]),
                    ),
                );
            }

            if angle > best_angle {
                best_angle = angle;
                best = Some(k);
            }
        }

        // no ear and nothing flat to drop: the loop crosses itself in its plane
        let cut = best.or(degenerate)?;
        if best.is_some() {
            triangles.push(HoleFillTriangle::new(
                remaining[(cut + m - 1) % m] as i32,
                remaining[cut] as i32,
                remaining[(cut + 1) % m] as i32,
            ));
        }

        remaining.remove(cut);
    }

    if height(flat[remaining[0]], flat[remaining[1]], flat[remaining[2]]) > snap {
        if is_forbidden(remaining[0], remaining[1], remaining[2]) {
            return None;
        }

        triangles.push(HoleFillTriangle::new(
            remaining[0] as i32,
            remaining[1] as i32,
            remaining[2] as i32,
        ));
    }

    if triangles.is_empty() {
        return None;
    }

    Some((triangles, normal))
}

/// The signed height of the triangle a, b, c off its longest edge: positive where it turns
/// counter-clockwise in the plane (the loop's way).
fn height(a: Vector2, b: Vector2, c: Vector2) -> f64 {
    let twice_area = ((b.x - a.x) * (c.y - a.y)) - ((b.y - a.y) * (c.x - a.x));
    let longest = ieee_math::max_f64(
        (b - a).length(),
        ieee_math::max_f64((c - b).length(), (a - c).length()),
    );
    if longest == 0.0 {
        0.0
    } else {
        twice_area / longest
    }
}

/// Whether any other remaining vertex lies in the ear a, b, c or on its edges - the diagonal a to
/// c included, so a vertex on it is never left as a T-junction. A vertex on one of the ear's own
/// corners (a pinch, or a point repeated where the row starts at the end vertex) does not block.
fn blocks_ear(
    flat: &[Vector2],
    remaining: &[usize],
    a: usize,
    b: usize,
    c: usize,
    snap: f64,
) -> bool {
    for &q in remaining {
        if q == a || q == b || q == c {
            continue;
        }

        let p = flat[q];
        if (p - flat[a]).length() <= snap
            || (p - flat[b]).length() <= snap
            || (p - flat[c]).length() <= snap
        {
            continue;
        }

        if side(flat[a], flat[b], p) >= -snap
            && side(flat[b], flat[c], p) >= -snap
            && side(flat[c], flat[a], p) >= -snap
        {
            return true;
        }
    }

    false
}

/// The signed distance of p to the left of the line from a to b.
fn side(a: Vector2, b: Vector2, p: Vector2) -> f64 {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let length = ((dx * dx) + (dy * dy)).sqrt();
    if length == 0.0 {
        0.0
    } else {
        ((dx * (p.y - a.y)) - (dy * (p.x - a.x))) / length
    }
}

/// The smallest angle of the ear p, v, n a cut would leave at `v`; a reflex corner (no ear) forces
/// nothing and scores pi.
fn left_ear(p: Vector2, v: Vector2, n: Vector2) -> f64 {
    if height(p, v, n) > 0.0 {
        smallest_angle(p, v, n)
    } else {
        std::f64::consts::PI
    }
}

fn smallest_angle(a: Vector2, b: Vector2, c: Vector2) -> f64 {
    fn at(o: Vector2, p: Vector2, q: Vector2) -> f64 {
        let x = p - o;
        let y = q - o;
        let dot = (x.x * y.x) + (x.y * y.y);
        libm::acos((dot / (x.length() * y.length())).clamp(-1.0, 1.0))
    }

    ieee_math::min_f64(at(a, b, c), ieee_math::min_f64(at(b, c, a), at(c, a, b)))
}
