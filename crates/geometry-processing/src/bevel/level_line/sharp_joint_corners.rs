//! The inside corner of one chain's own SHARP JOINT, made a mesh vertex before the refinement
//! runs: on a face holding both of the joint's legs, the point standing the joint's setback from
//! each leg - where the level line turns. Ports `Bevel/LevelLine/SharpJointCorners.cs` (a C#
//! static class, so a module of functions): the corner itself ([`the_inside_corner`]), its tests
//! and the insertion over a band's candidates ([`insert`]).
//!
//! Without it the cut meets that corner only when the face's own edges pass through it: on the
//! knife-edge prism the cut chorded a corner off, the rail had no vertex standing for the joint's
//! whole parameter jump, and the strip fanned the back round's rows from the wrong end of the
//! chord.

use vector_math::{cs_math, Vector3};

use super::{ChainField, LevelLineMesh};

/// The noise allowance a corner is read against at `at`, on a face of size `scale`.
pub fn off_the_line(at: Vector3, scale: f64) -> f64 {
    1e-6 * cs_math::max_f64(cs_math::max_f64(1.0, scale), at.length())
}

/// True when `triangle` lies in the plane of the joint's two legs (`legs_plane`, a unit normal)
/// and holds `point` strictly inside, clear of every edge by the noise allowance.
pub fn holds(mesh: &LevelLineMesh, triangle: i32, point: Vector3, legs_plane: Vector3) -> bool {
    let t = mesh.triangles()[triangle as usize];
    let a = mesh.position(t.a);
    let b = mesh.position(t.b);
    let c = mesh.position(t.c);
    let mut normal = (b - a).cross(c - a);
    let area = normal.length();
    if area <= 0.0 {
        return false;
    }

    normal /= area;

    // A face the two legs lie in: the corner is the level line's turn ON that face. Where the
    // joint's legs leave the face (Around Center's knife tips) the line's turn is not a plane
    // offset corner, and inserted there it built one triangle count at the origin and another
    // placed through floats
    // (ChainFieldMitreTests.AroundCenterRotatedAndMovedThroughFloatsBuildsTheSameFill).
    if 1.0 - normal.dot(legs_plane).abs() > 1e-6 {
        return false;
    }

    let scale = cs_math::max_f64(
        (b - a).length(),
        cs_math::max_f64((c - b).length(), (a - c).length()),
    );
    if (point - a).dot(normal).abs() > off_the_line(point, scale) {
        return false;
    }

    // Strictly inside, clear of every edge by the noise allowance: a corner ON an edge (Around
    // Center's knife joints at the origin) is already met by the cut, which splits that edge where
    // the line crosses it, and inserted there by this rule as well it split the edge at the origin
    // but a triangle once the part was placed through floats - two triangle counts for one part.
    let clear = off_the_line(point, scale);
    let side = |p: Vector3, q: Vector3| (q - p).cross(point - p).dot(normal) / (q - p).length();
    side(a, b) > clear && side(b, c) > clear && side(c, a) > clear
}

/// C# `SharpJointCorners.TheInsideCorner`: the point on the bisector of the joint's inside angle,
/// in the plane of its two legs, standing `setback` from each leg's line, with that plane's unit
/// normal; `None` where the legs do not turn or reverse.
pub fn the_inside_corner(chain: &ChainField, at: f64, setback: f64) -> Option<(Vector3, Vector3)> {
    let step = 1e-3 * cs_math::max_f64(setback, 1e-9);
    let apex = chain.point_at(at);
    let back = (chain.point_at(chain.wrapped(at - step)) - apex).get_normal();
    let ahead = (chain.point_at(chain.wrapped(at + step)) - apex).get_normal();
    let cos = back.dot(ahead);
    if cos <= -1.0 + 1e-9 || cos >= 1.0 - 1e-9 {
        return None;
    }

    // Between the two legs as they leave the apex: sin of half the inside angle is the setback
    // over the corner's distance from the apex.
    let bisector = (back + ahead).get_normal();
    let sin_half = ((1.0 - cos) / 2.0).sqrt();
    Some((
        apex + (bisector * (setback / sin_half)),
        back.cross(ahead).get_normal(),
    ))
}

/// C# `SharpJointCorners.Insert`: inserts every member chain's sharp-joint inside corner that a
/// candidate triangle of the reading's mesh holds - on the group's line only - putting each on the
/// line, and hands back the candidates with every triangle a split made in place of a divided one
/// (first-seen order, removed triangles dropped).
pub fn insert<'a>(
    reading: &super::VertexFieldReading,
    group_field: &dyn super::ScalarField,
    members: impl IntoIterator<Item = &'a ChainField>,
    candidates: Vec<i32>,
    snap: f64,
) -> Result<Vec<i32>, super::LevelLineMeshError> {
    let mut corners = Vec::new();
    for chain in members {
        for &(at, setback) in chain.sharp_joints() {
            // On the group's line only: a corner another member reads nearer, or one off the
            // group's own faces, is not where this group's level line turns.
            if let Some((point, plane)) = the_inside_corner(chain, at, setback) {
                if group_field.value_at(point)?.abs() <= off_the_line(point, setback) {
                    corners.push((point, plane));
                }
            }
        }
    }

    if corners.is_empty() {
        return Ok(candidates);
    }

    let mut pending = std::collections::VecDeque::new();
    let mut depth = std::collections::HashMap::new();
    let mut result = candidates;
    for (corner, plane) in corners {
        for triangle in result.clone() {
            super::LevelLineWork::charge(1)?;
            {
                let mesh = reading.mesh.borrow();
                if mesh.is_removed(triangle) || !holds(&mesh, triangle, corner, plane) {
                    continue;
                }
            }

            let before = pending.len();
            // On the line by construction: its reading there is float noise either side of zero,
            // and read as a side the cut crossed its edges on one placement and not the other.
            let made = super::refinement_splits_enqueue::insert_on_the_line(
                &mut reading.mesh.borrow_mut(),
                triangle,
                corner,
                snap,
                &mut pending,
                &mut depth,
            )?;
            if made >= 0 {
                reading.put_on_the_line(made)?;
            }
            result.extend(pending.iter().skip(before).copied());
            break;
        }
    }

    let distinct: indexmap::IndexSet<i32> = result.into_iter().collect();
    let mesh = reading.mesh.borrow();
    Ok(distinct
        .into_iter()
        .filter(|&t| !mesh.is_removed(t))
        .collect())
}
