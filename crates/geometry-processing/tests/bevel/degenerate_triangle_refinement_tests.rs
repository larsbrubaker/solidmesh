//! A triangle of no area in the band must not be able to keep the refinement firing.

use std::cell::RefCell;
use std::rc::Rc;

use solidmesh::bevel::level_line::band_refinement::{refine, RefineOptions};
use solidmesh::bevel::level_line::{ChainField, LevelLineMesh, VertexFieldReading, VertexSide};
use solidmesh_math::Vector3;

use crate::level_line_test_geometry::coincident_vertices;

const SNAP: f64 = 1e-9;

/// The measured cycle from 95498 pick 2, laid out as the smallest mesh that carries it: three
/// collinear corners reading OUTSIDE, INSIDE, INSIDE, with the field rising back above the line
/// between the two inside ones. What has to hold is that the cycle's own link stays broken: no
/// split may mint a second vertex where the mesh already stands one.
#[test]
fn a_triangle_with_no_height_does_not_keep_the_refinement_firing() {
    // A chain that comes close to the strip twice and stands away from it between.
    let field = Rc::new(
        ChainField::from_polyline(
            &[
                Vector3::new(-6.0, -12.0, 0.0),
                Vector3::new(-4.0, -4.0, 0.0),
                Vector3::new(0.0, -8.0, 0.0),
                Vector3::new(5.0, -4.5, 0.0),
            ],
            false,
            &[3.0, 3.0, 3.0, 3.0],
        )
        .expect("a chain"),
    );

    let positions = vec![
        Vector3::new(-9.0, -3.0, 0.0), // 0 - outside
        Vector3::new(-4.0, -3.0, 0.0), // 1 - inside, standing exactly on the segment from 0 to 2
        Vector3::new(5.0, -3.0, 0.0),  // 2 - inside
        Vector3::new(0.0, 3.0, 0.0),   // 3 - the ordinary neighbour across the long edge
    ];

    let indices = vec![
        (0, 1, 2), // no height at all: 0, 1 and 2 are collinear
        (2, 0, 3), // an ordinary triangle sharing the long edge, refined on its own account
    ];

    let mesh = LevelLineMesh::from_triangles(&positions, &indices, None).expect("a mesh");
    let reading =
        VertexFieldReading::new(Rc::new(RefCell::new(mesh)), field.clone(), SNAP).expect("a snap");

    assert_eq!(reading.side_of(0).ok(), Some(VertexSide::Outside));
    assert_eq!(reading.side_of(1).ok(), Some(VertexSide::Inside));
    assert_eq!(reading.side_of(2).ok(), Some(VertexSide::Inside));
    assert!(
        field
            .max_along_segment(positions[1], positions[2], 1e-3)
            .expect("the peak")
            .value
            > SNAP,
        "the fixture only carries the fault if that inside-inside edge really bumps above the line"
    );

    let candidates: Vec<i32> = (0..reading.mesh.borrow().triangle_count()).collect();
    let (settled, offending) =
        refine(&reading, &candidates, 0.05, RefineOptions::default()).expect("the refinement");

    assert!(
        settled,
        "nothing here cannot be settled; it stopped at {offending}"
    );
    let mesh = reading.mesh.borrow();
    assert!(
        mesh.triangle_count() < 200,
        "a refinement that descends settles this in a handful of splits; one that shaves never stops"
    );

    // Size here is the fixture's own span, about 14.
    let doubled = coincident_vertices(&mesh, 1e-6 * 14.0);
    assert_eq!(
        doubled.len(),
        0,
        "a split that mints where a vertex already stands is what let the rules chase each other: {:?}",
        doubled.iter().take(4).collect::<Vec<_>>()
    );
}
