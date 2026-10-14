//! The corners where a clipped end plane meets the creases leaving its zero plateau: the
//! constants and [`insert`].

/// Bisection steps along a crease edge to the last point still reading zero.
pub const BISECTION_STEPS: i32 = 60;

/// How far off the end plane a crease vertex may stand and still be on it: the input is float, so a
/// vertex of the side-to-end crease stands up to about 1e-7 off the plane read through the end
/// vertex.
pub const ON_THE_PLANE: f64 = 1e-5;

/// On every candidate SIDE triangle's edge lying in a member's
/// same-sense end plane (the side-to-end crease), the one point between the field's zero plateau
/// and its rise, found by bisection and put on the line - so the refinement roots no second vertex
/// microns from it. A crease edge has ONE corner: the edges a split leaves it as are not asked
/// again. Read on the reading's own mesh; hands back the candidates with every triangle a split
/// made in place of a divided one (first-seen order, removed triangles dropped).
pub fn insert<'a>(
    reading: &super::VertexFieldReading,
    field: &dyn super::ChainDistanceField,
    members: impl IntoIterator<Item = &'a super::ChainField>,
    candidates: Vec<i32>,
    snap: f64,
) -> Result<Vec<i32>, super::LevelLineMeshError> {
    use vector_math::Vector3;

    let mut planes: Vec<(Vector3, Vector3)> = Vec::new();
    for chain in members {
        if let Some(first) = chain.end_ray_bounds.0.filter(|b| b.same_sense) {
            planes.push((chain.points()[0], first.plane_normal.get_normal()));
        }

        if let Some(last) = chain.end_ray_bounds.1.filter(|b| b.same_sense) {
            planes.push((
                chain.points()[chain.points().len() - 1],
                last.plane_normal.get_normal(),
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
            let Some(&(vertex, normal)) = planes.iter().find(|p| {
                (a - p.0).dot(p.1).abs() <= ON_THE_PLANE && (b - p.0).dot(p.1).abs() <= ON_THE_PLANE
            }) else {
                continue;
            };
            if normal == Vector3::ZERO {
                continue;
            }

            // On the plateau the PLANE term is the reading, or the value is not above zero; off
            // it the chain's reading rises above zero. The corner is the one point between.
            let on_plateau = |q: Vector3| -> Result<bool, super::WorkError> {
                let value = field.value_at(q)?;
                Ok(value <= 0.0 || value <= (q - vertex).dot(normal) + 1e-12)
            };
            let a_on = on_plateau(a)?;
            let b_on = on_plateau(b)?;
            if a_on == b_on {
                continue;
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
            // the line outright.
            let made = super::refinement_splits_enqueue::insert_on_the_line(
                &mut reading.mesh.borrow_mut(),
                triangle,
                on,
                snap,
                &mut pending,
                &mut depth,
            )?;
            if made >= 0 {
                corners.insert(made);
                reading.put_on_the_line(made)?;
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
