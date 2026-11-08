//! A die point on the section's straight side (or nudged just outside the ruler of it) cuts the
//! section and only the section - its chord takes nothing of the end face the bead stands over.

use solidmesh::bevel::surface::{surface_end_cut, SurfaceVertexPool};
use solidmesh_math::{Vector3, Vector3Float};

const CORNER: Vector3 = Vector3::new(0.0, 0.0, 0.0);
const ARC_CENTRE: Vector3 = Vector3::new(5.0, 5.0, 0.0);
const ARC_RADIUS: f64 = 5.0;
const GRID_LINES: [f64; 7] = [-5.0, -2.0, 0.0, 1.7, 3.3, 5.0, 7.0];
const IN_THE_SECTION: Vector3 = Vector3::new(0.5, 0.5, 0.0);
const THE_END_FACE_KEEPS: Vector3 = Vector3::new(4.0, 4.0, 0.0);

#[test]
fn a_die_point_on_the_straight_side_cuts_the_section_and_only_the_section() {
    the_cut_takes_the_section(0.0);
}

#[test]
fn a_die_point_nudged_just_outside_the_ruler_cuts_the_section_and_only_the_section() {
    the_cut_takes_the_section(10.0);
}

fn the_cut_takes_the_section(rulers: f64) {
    // Started from the two far corners of the face, so the pool's weld distance is the one a part
    // this size really gets.
    let mut pool = SurfaceVertexPool::new(&[
        Vector3Float::new(-5.0, -5.0, 0.0),
        Vector3Float::new(7.0, 7.0, 0.0),
    ]);

    let mut faces: Vec<(i32, i32, i32)> = the_end_face()
        .into_iter()
        .map(|(a, b, c)| (pool.add(a), pool.add(b), pool.add(c)))
        .collect();

    // The die point first, on the straight side from the corner to rail zero, pushed off it by
    // the asked-for number of rulers toward the inside of the wedge.
    let the_die_point = pool.add(Vector3::new(-rulers * pool.near_enough(), 2.0, 0.0));
    let mut profile = vec![the_die_point];
    for point in the_profile() {
        profile.push(pool.add(point));
    }

    let cut = surface_end_cut::try_cut(
        &mut pool,
        &mut faces,
        &profile,
        CORNER,
        false,
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::ZERO,
        -1,
        the_die_point,
    );
    if let Err(failure) = cut {
        panic!("the end face holds the profile and the section has area against the corner - {failure:?}");
    }

    assert!(
        covers(&pool, &faces, THE_END_FACE_KEEPS),
        "{THE_END_FACE_KEEPS} is inside the arc and so under the bead, no part of the section - the end face still has to cover it, and a die point chord cutting the whole wedge is what takes it away"
    );
    assert!(
        !covers(&pool, &faces, IN_THE_SECTION),
        "{IN_THE_SECTION} is beyond the arc and inside the wedge, which is the section itself - leaving it standing would mean the cut never happened"
    );
}

fn covers(pool: &SurfaceVertexPool, faces: &[(i32, i32, i32)], at: Vector3) -> bool {
    for &(v0, v1, v2) in faces {
        let (a, b, c) = (
            pool.position_of(v0),
            pool.position_of(v1),
            pool.position_of(v2),
        );
        if a.z.abs() > pool.near_enough()
            || b.z.abs() > pool.near_enough()
            || c.z.abs() > pool.near_enough()
        {
            continue;
        }

        let (first, second, third) = (side(a, b, at), side(b, c, at), side(c, a, at));
        if (first >= 0.0 && second >= 0.0 && third >= 0.0)
            || (first <= 0.0 && second <= 0.0 && third <= 0.0)
        {
            return true;
        }
    }

    false
}

fn side(from: Vector3, to: Vector3, at: Vector3) -> f64 {
    ((to.x - from.x) * (at.y - from.y)) - ((to.y - from.y) * (at.x - from.x))
}

fn the_end_face() -> Vec<(Vector3, Vector3, Vector3)> {
    let mut triangles = Vec::new();
    for x in 0..GRID_LINES.len() - 1 {
        for y in 0..GRID_LINES.len() - 1 {
            let low_left = Vector3::new(GRID_LINES[x], GRID_LINES[y], 0.0);
            let low_right = Vector3::new(GRID_LINES[x + 1], GRID_LINES[y], 0.0);
            let high_right = Vector3::new(GRID_LINES[x + 1], GRID_LINES[y + 1], 0.0);
            let high_left = Vector3::new(GRID_LINES[x], GRID_LINES[y + 1], 0.0);
            triangles.push((low_left, low_right, high_right));
            triangles.push((low_left, high_right, high_left));
        }
    }
    triangles
}

fn the_profile() -> Vec<Vector3> {
    const CHORDS: i32 = 6;
    (0..=CHORDS)
        .map(|step| {
            let angle = std::f64::consts::PI
                + (f64::from(step) * (std::f64::consts::PI / 2.0) / f64::from(CHORDS));
            ARC_CENTRE + Vector3::new(ARC_RADIUS * angle.cos(), ARC_RADIUS * angle.sin(), 0.0)
        })
        .collect()
}
