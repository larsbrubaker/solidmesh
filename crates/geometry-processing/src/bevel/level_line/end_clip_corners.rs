//! The corners where a clipped end plane meets the creases leaving its zero plateau: the
//! constants and [`insert`]. A same-sense plane's corner lies on the side-to-end crease, an edge
//! the mesh has in the plane. A STOPPED end's plane (`stopped_end_clip.rs`) has no such crease on a
//! side whose face it crosses: there the corner lies on the plane's trace across a side triangle,
//! often inside it, and is put there too, so the band's boundary follows the plane all the way to
//! the rail on every side and the fill's last row and flat end lie in the plane. A stopped plane is
//! read by the one rule the field reads it by (`ChainField::on_a_stopped_plane`), and the corner
//! goes onto it exactly (`stopped_plane_points.rs`); a knife end's plane keeps [`ON_THE_PLANE`].

/// Bisection steps along a crease edge to the last point still reading zero.
pub const BISECTION_STEPS: i32 = 60;

/// How far off a KNIFE end's plane a crease vertex may stand and still be on it: the input is float, so a
/// vertex of the side-to-end crease stands up to about 1e-7 off the plane read through the end
/// vertex.
pub const ON_THE_PLANE: f64 = 1e-5;

/// On every candidate SIDE triangle's edge lying in a member's
/// same-sense or stopped end plane (the side-to-end crease, or a stopped end's facet seam), the
/// one point between the field's zero plateau and its rise, found by bisection and put on the
/// line, so the refinement roots no second vertex microns from it. Where a STOPPED plane crosses a
/// side triangle with no edge in it, the same point is found along the plane's trace across the
/// triangle. A crease edge has ONE corner: the edges a split leaves it as are not asked
/// again. Read on the reading's own mesh; hands back the candidates with every triangle a split
/// made in place of a divided one (first-seen order, removed triangles dropped).
pub fn insert<'a>(
    reading: &super::VertexFieldReading,
    field: &dyn super::ChainDistanceField,
    members: impl IntoIterator<Item = &'a super::ChainField>,
    candidates: Vec<i32>,
    snap: f64,
) -> Result<Vec<i32>, super::LevelLineMeshError> {
    use solidmesh_math::Vector3;

    // (point on the plane, unit normal, the plane's tilt where it is a stopped end's)
    let mut planes: Vec<(Vector3, Vector3, Option<f64>)> = Vec::new();
    for chain in members {
        if let Some(first) = chain.end_ray_bounds.0.filter(|b| b.same_sense || b.stopped) {
            planes.push((
                chain.points()[0],
                first.plane_normal.get_normal(),
                first.stopped.then_some(first.plane_tilt),
            ));
        }

        if let Some(last) = chain.end_ray_bounds.1.filter(|b| b.same_sense || b.stopped) {
            planes.push((
                chain.points()[chain.points().len() - 1],
                last.plane_normal.get_normal(),
                last.stopped.then_some(last.plane_tilt),
            ));
        }
    }

    if planes.is_empty() {
        return Ok(candidates);
    }

    let mut pending: std::collections::VecDeque<i32> = candidates.iter().copied().collect();
    let mut depth = std::collections::HashMap::new();
    let mut seen = candidates;

    // The corners already made: a crease edge has ONE corner, and the plateau test reads float
    // noise at its zero along the crease, so a split's far child must not be asked again.
    let mut corners: std::collections::HashSet<i32> = std::collections::HashSet::new();
    while let Some(triangle) = pending.pop_front() {
        super::LevelLineWork::charge(1)?;
        if reading.mesh.borrow().is_removed(triangle) {
            continue;
        }

        let t = reading.mesh.borrow().triangles()[triangle as usize];
        let position = |v: i32| reading.mesh.borrow().position(v);
        let before = pending.len();

        // Only a SIDE triangle's edge in the plane is the side-to-end crease: the end face's own
        // edges lie in the plane too, and splitting them only crowds the face.
        let face_normal = (position(t.b) - position(t.a))
            .cross(position(t.c) - position(t.a))
            .get_normal();
        if planes
            .iter()
            .any(|p| face_normal.dot(p.1).abs() > 1.0 - 1e-6)
        {
            continue;
        }

        let mut corner = 0;
        while corner < 3 && pending.len() == before {
            let (from, to) = (t.corner(corner), t.corner((corner + 1) % 3));
            corner += 1;
            if corners.contains(&from) || corners.contains(&to) {
                continue;
            }

            let a = position(from);
            let b = position(to);
            // The first plane holding both ends, a zero normal read as none.
            let Some(&(vertex, normal, _)) = planes
                .iter()
                .find(|p| on_the_plane(**p, a) && on_the_plane(**p, b))
            else {
                continue;
            };
            if normal == Vector3::ZERO {
                continue;
            }

            let made = corner_between(
                reading,
                field,
                triangle,
                (a, b),
                (vertex, normal),
                snap,
                (&mut pending, &mut depth),
            )?;
            if made.is_some_and(|made| made >= 0) {
                corners.extend(made);
            }
        }

        // A stopped plane crossing the triangle with no edge in it: its trace across the
        // triangle, from a corner on the plane or an edge it crosses to another. A triangle
        // holding a corner already made on THIS plane is a split's child and is not asked again;
        // a corner made on another plane says nothing about this one (the two planes of a short
        // facet meet inside the cap's fan triangles, one plane's corner splitting the triangle the
        // other's trace crosses).
        if pending.len() == before {
            let corners_at = [position(t.a), position(t.b), position(t.c)];
            for &(vertex, normal, stopped) in &planes {
                let Some(tilt) = stopped else {
                    continue;
                };
                if normal == Vector3::ZERO || pending.len() != before {
                    continue;
                }
                if [t.a, t.b, t.c].iter().zip(corners_at).any(|(v, at)| {
                    corners.contains(v)
                        && super::ChainField::on_a_stopped_plane(vertex, normal, tilt, at)
                }) {
                    continue;
                }

                let Some(trace) = trace_across(corners_at, vertex, normal, tilt) else {
                    continue;
                };
                let made = corner_between(
                    reading,
                    field,
                    triangle,
                    trace,
                    (vertex, normal),
                    snap,
                    (&mut pending, &mut depth),
                )?;
                if made.is_some_and(|made| made >= 0) {
                    corners.extend(made);
                }
            }
        }

        seen.extend(pending.iter().skip(before).copied());
    }

    let distinct: indexmap::IndexSet<i32> = seen.into_iter().collect();
    let mesh = reading.mesh.borrow();
    Ok(distinct
        .into_iter()
        .filter(|&t| !mesh.is_removed(t))
        .collect())
}

/// Whether `point` stands on the end plane `(point on it, unit normal, a stopped end's tilt)`: a stopped plane
/// by [`ChainField::on_a_stopped_plane`](super::ChainField::on_a_stopped_plane), a knife end's
/// within [`ON_THE_PLANE`].
fn on_the_plane(
    (vertex, normal, stopped): (
        solidmesh_math::Vector3,
        solidmesh_math::Vector3,
        Option<f64>,
    ),
    point: solidmesh_math::Vector3,
) -> bool {
    if let Some(tilt) = stopped {
        super::ChainField::on_a_stopped_plane(vertex, normal, tilt, point)
    } else {
        (point - vertex).dot(normal).abs() <= ON_THE_PLANE
    }
}

/// A STOPPED plane's trace across a triangle whose corners stand at `corners`: its two ends, each
/// a corner on the plane ([`ChainField::on_a_stopped_plane`](super::ChainField::on_a_stopped_plane))
/// or the point where an edge with its ends on the two sides crosses it. `None` where the plane
/// misses the triangle, touches it at one point, or holds one of its edges (that is a crease edge,
/// read on its own).
pub fn trace_across(
    corners: [solidmesh_math::Vector3; 3],
    vertex: solidmesh_math::Vector3,
    normal: solidmesh_math::Vector3,
    tilt: f64,
) -> Option<(solidmesh_math::Vector3, solidmesh_math::Vector3)> {
    let d = corners.map(|c| (c - vertex).dot(normal));
    let on = corners.map(|c| super::ChainField::on_a_stopped_plane(vertex, normal, tilt, c));
    let mut ends = Vec::with_capacity(2);
    for k in 0..3 {
        if on[k] {
            ends.push(corners[k]);
        }
    }

    if ends.len() != 1 && !ends.is_empty() {
        return None;
    }

    for k in 0..3 {
        let j = (k + 1) % 3;
        if !on[k] && !on[j] && (d[k] < 0.0) != (d[j] < 0.0) {
            let t = d[k] / (d[k] - d[j]);
            ends.push(corners[k] + (corners[j] - corners[k]) * t);
        }
    }

    (ends.len() == 2).then(|| (ends[0], ends[1]))
}

/// The one point between the field's zero plateau and its rise on the segment `from_to` of an end
/// plane, put on the line in `triangle` (its splits queued on `pending`): `None` where both ends read alike, else what
/// `insert_on_the_line` made (its `NOTHING_TO_INSERT` where the point needed no vertex).
#[allow(clippy::too_many_arguments)]
fn corner_between(
    reading: &super::VertexFieldReading,
    field: &dyn super::ChainDistanceField,
    triangle: i32,
    (a, b): (solidmesh_math::Vector3, solidmesh_math::Vector3),
    (vertex, normal): (solidmesh_math::Vector3, solidmesh_math::Vector3),
    snap: f64,
    (pending, depth): (
        &mut std::collections::VecDeque<i32>,
        &mut std::collections::HashMap<i32, i32>,
    ),
) -> Result<Option<i32>, super::LevelLineMeshError> {
    // On the plateau the PLANE term is the reading, or the value is not above zero; off
    // it the chain's reading rises above zero. The corner is the one point between.
    let on_plateau = |q: solidmesh_math::Vector3| -> Result<bool, super::WorkError> {
        let value = field.value_at(q)?;
        Ok(value <= 0.0 || value <= (q - vertex).dot(normal) + 1e-12)
    };
    let a_on = on_plateau(a)?;
    let b_on = on_plateau(b)?;
    if a_on == b_on {
        return Ok(None);
    }

    let mut on = if a_on { a } else { b };
    let mut off = if a_on { b } else { a };
    for _ in 0..BISECTION_STEPS {
        let mid = (on + off) / 2.0;
        if on_plateau(mid)? {
            on = mid;
        } else {
            off = mid;
        }
    }

    // The crease is float input, so the corner may read a hair off zero: it is put on
    // the line outright - and on a stopped plane, on the plane itself.
    let on = super::stopped_plane_points::onto_a_stopped_plane(reading, triangle, on);
    let made = super::refinement_splits_enqueue::insert_on_the_line(
        &mut reading.mesh.borrow_mut(),
        triangle,
        on,
        snap,
        pending,
        depth,
    )?;
    if made >= 0 {
        reading.put_on_the_line(made)?;
    }

    Ok(Some(made))
}
