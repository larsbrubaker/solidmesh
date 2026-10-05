//! Ports `Tests/GeometryProcessingTests/SurfaceGivingUpTheSectionSewsByIndexTests.cs` 1:1: every
//! point the subtraction leaves standing on the profile is reported, even where the rim sew
//! renumbers the faces that hold them.

use std::collections::HashSet;

use geometry_processing::bevel::surface::{surface_end_gives_up_the_section, SurfaceVertexPool};
use vector_math::cs_sort::introspective_sort;
use vector_math::{Vector3, Vector3Float};

const CORNER: Vector3 = Vector3::new(0.0, 0.0, 0.0);
const OUTWARD: Vector3 = Vector3::new(0.0, 0.0, 1.0);
const GRID_LINES: [f64; 6] = [-2.0, 0.5, 3.0, 5.5, 8.0, 10.0];
const ARC_CENTRE: Vector3 = Vector3::new(5.0, 5.0, 0.0);
const ARC_RADIUS: f64 = 5.0;
const THE_FLOOD_REACHES: f64 = 3.0;

#[test]
fn every_point_standing_on_the_profile_is_reported_even_when_the_rim_sew_renumbers_the_faces() {
    // Started from the two corners of the face below, so the pool's weld distance is the one a
    // part this size really gets.
    let mut pool = SurfaceVertexPool::new(&[
        Vector3Float::new(-2.0, -2.0, 0.0),
        Vector3Float::new(10.0, 10.0, 0.0),
    ]);
    let mut faces: Vec<(i32, i32, i32)> = Vec::new();

    // The skirt under the end face's rim, FIRST in the list: the faces the subtraction does not
    // touch and the sew does.
    for (a, b, c) in the_skirt() {
        faces.push((pool.add(a), pool.add(b), pool.add(c)));
    }

    let the_skirt_before_the_sew = faces.clone();

    let mut in_the_end = indexmap::IndexSet::new();
    for (a, b, c, flooded) in the_end_face() {
        if flooded {
            in_the_end.insert(faces.len() as i32);
        }
        faces.push((pool.add(a), pool.add(b), pool.add(c)));
    }

    let profile: Vec<i32> = the_profile().into_iter().map(|p| pool.add(p)).collect();

    let given = surface_end_gives_up_the_section::try_give_it_up(
        &mut pool,
        &mut faces,
        &profile,
        CORNER,
        OUTWARD,
        &in_the_end,
    );
    let along_the_profile = match given {
        Ok(along) => along,
        Err(failure) => panic!("the section has area against the corner - {failure:?}"),
    };

    assert!(
        the_skirt_before_the_sew.iter().filter(|face| faces.contains(face)).count()
            < the_skirt_before_the_sew.len(),
        "clip lines leave the end face across the skirt's rim runs, so the sew has to have split those skirt triangles - the renumbering this test is about is real"
    );

    let standing = standing_on_the_profile(&pool, &faces, &profile);

    assert!(
        standing.iter().map(Vec::len).sum::<usize>() > 0,
        "the cut has to have left points part way along the profile for there to be anything to report - otherwise this test is measuring nothing"
    );

    for chord in 0..profile.len() - 1 {
        let mut reported = along_the_profile[chord].clone();
        reported.sort_unstable();
        let mut expected = standing[chord].clone();
        expected.sort_unstable();
        assert_eq!(
            reported,
            expected,
            "chord {chord} of the profile, from {} to {}, has {} vertices of the re-cut end face standing on it and {} were reported - a point the strip does not sew to is an open edge",
            pool.position_of(profile[chord]),
            pool.position_of(profile[chord + 1]),
            standing[chord].len(),
            along_the_profile[chord].len()
        );
    }
}

fn standing_on_the_profile(
    pool: &SurfaceVertexPool,
    faces: &[(i32, i32, i32)],
    profile: &[i32],
) -> Vec<Vec<i32>> {
    let mut in_the_plane = HashSet::new();
    for &(v0, v1, v2) in faces {
        let corners = [v0, v1, v2];
        if !corners
            .iter()
            .all(|&c| pool.position_of(c).z.abs() < pool.near_enough())
        {
            continue;
        }

        let middle = (pool.position_of(v0) + pool.position_of(v1) + pool.position_of(v2)) / 3.0;
        if middle.x > THE_FLOOD_REACHES || middle.y > THE_FLOOD_REACHES {
            continue;
        }

        in_the_plane.extend(corners);
    }

    let mut found = Vec::new();
    for chord in 0..profile.len() - 1 {
        let from = pool.position_of(profile[chord]);
        let run = pool.position_of(profile[chord + 1]) - from;
        let mut on = Vec::new();
        for &vertex in &in_the_plane {
            if vertex == profile[chord] || vertex == profile[chord + 1] {
                continue;
            }

            let at = pool.position_of(vertex) - from;
            let at_where = at.dot(run) / run.length_squared();
            if at_where > 0.0
                && at_where < 1.0
                && (at - (run * at_where)).length() < pool.near_enough()
            {
                on.push(vertex);
            }
        }

        found.push(on);
    }

    found
}

type Cell = (f64, Vector3, Vector3, Vector3, Vector3);

fn the_end_face() -> Vec<(Vector3, Vector3, Vector3, bool)> {
    let mut cells: Vec<Cell> = Vec::new();
    for x in 0..GRID_LINES.len() - 1 {
        for y in 0..GRID_LINES.len() - 1 {
            let low_left = Vector3::new(GRID_LINES[x], GRID_LINES[y], 0.0);
            let low_right = Vector3::new(GRID_LINES[x + 1], GRID_LINES[y], 0.0);
            let high_right = Vector3::new(GRID_LINES[x + 1], GRID_LINES[y + 1], 0.0);
            let high_left = Vector3::new(GRID_LINES[x], GRID_LINES[y + 1], 0.0);
            let middle = (low_left + high_right) / 2.0;
            cells.push((
                ((middle - ARC_CENTRE).length() - ARC_RADIUS).abs(),
                low_left,
                low_right,
                high_right,
                high_left,
            ));
        }
    }

    // Nearest the profile first (C#'s unstable List.Sort).
    introspective_sort(&mut cells, &mut |first: &Cell, second: &Cell| {
        first.0.partial_cmp(&second.0).expect("finite")
    });

    let mut triangles = Vec::new();
    for (_, low_left, low_right, high_right, high_left) in cells {
        // The flood stops where the corner's own faces stop reaching.
        let flooded =
            high_right.x <= THE_FLOOD_REACHES + 1e-9 && high_right.y <= THE_FLOOD_REACHES + 1e-9;
        triangles.push((low_left, low_right, high_right, flooded));
        triangles.push((low_left, high_right, high_left, flooded));
    }
    triangles
}

fn the_skirt() -> Vec<(Vector3, Vector3, Vector3)> {
    let mut skirt = Vec::new();
    for step in 0..GRID_LINES.len() - 1 {
        let low = GRID_LINES[step];
        let high = GRID_LINES[step + 1];
        let middle = (low + high) / 2.0;
        skirt.push((
            Vector3::new(low, -2.0, 0.0),
            Vector3::new(middle, -2.0, -5.0),
            Vector3::new(high, -2.0, 0.0),
        ));
        skirt.push((
            Vector3::new(low, 10.0, 0.0),
            Vector3::new(middle, 10.0, -5.0),
            Vector3::new(high, 10.0, 0.0),
        ));
        skirt.push((
            Vector3::new(-2.0, low, 0.0),
            Vector3::new(-2.0, middle, -5.0),
            Vector3::new(-2.0, high, 0.0),
        ));
        skirt.push((
            Vector3::new(10.0, low, 0.0),
            Vector3::new(10.0, middle, -5.0),
            Vector3::new(10.0, high, 0.0),
        ));
    }
    skirt
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
