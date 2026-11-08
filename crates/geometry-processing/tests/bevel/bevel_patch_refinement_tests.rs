//! The corner patch refined on the inside and untouched on its boundary. The boundary is the SEAM
//! the runs ending at that corner carry point for point, so a refinement that moved it, or put a
//! point between two of its points, would leave two solids meeting along two different polylines;
//! the interior belongs to nobody else and is where the chord error lives.

use solidmesh_math::Vector3;

use solidmesh::bevel::bevel_patch_refinement;

/// Every point the refinement adds is ON the sphere, in doubles, and no boundary point moves or
/// gains a point between it and its neighbour.
#[test]
fn every_point_it_adds_is_on_the_sphere_and_no_boundary_point_moves() {
    let mut directions = vec![
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    ];
    let patch = [(0, 1, 2)];
    let boundary = [vec![1, 2]];
    let before = directions.clone();

    let refined = bevel_patch_refinement::refine(&mut directions, &patch, Some(&boundary));

    for i in 0..before.len() {
        assert_eq!(
            directions[i], before[i],
            "a refinement may add directions and may not move one"
        );
    }

    for direction in &directions {
        assert!(
            (direction.length() - 1.0).abs() <= 1e-9,
            "every point of this surface is a direction on the ball"
        );
    }

    assert_eq!(
        directions.len(),
        5,
        "two of the three edges are interior and take a midpoint each; the boundary edge does not"
    );
    assert_eq!(
        refined.len(),
        3,
        "a triangle with two of its edges split comes out as three"
    );

    let carrying = refined
        .iter()
        .filter(|&&(a, b, c)| {
            [(a, b), (b, c), (c, a)]
                .iter()
                .any(|&edge| edge == (1, 2) || edge == (2, 1))
        })
        .count();
    assert_eq!(carrying, 1, "the boundary edge has to survive WHOLE");

    let would_be_midpoint = (directions[1] + directions[2]).get_normal();
    assert!(
        !directions
            .iter()
            .any(|&direction| (direction - would_be_midpoint).length() < 1e-9),
        "and nothing was added ON it"
    );
}
