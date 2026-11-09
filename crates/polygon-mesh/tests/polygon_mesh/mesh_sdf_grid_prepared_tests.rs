//! The hand-off between `MeshSdfGrid::prepare_request` and `finish_with_distances`: what a request
//! holds, the refusal when a prepared build is finished on a grid other than the one that made
//! it, and awaited builds on one grid that overlap or are dropped part way.

use std::future::Future;
use std::task::{Context, Poll, Waker};

use solidmesh_math::Matrix4X4;
use solidmesh_mesh::sdf::{DistanceBuildError, MeshSdfGrid, SdfGridError};
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::Mesh;

use crate::mesh_sdf_grid_async_tests::{
    assert_same_grid, block_on, flood, grid, none, shapes, AwaitedFill, Shape,
};

fn assert_refused(result: Result<(), SdfGridError>) {
    match result {
        Err(SdfGridError::PreparedForAnotherGrid) => {}
        other => panic!("expected the wrong-grid refusal, got {other:?}"),
    }
}

fn assert_unbuilt(sdf: &MeshSdfGrid) {
    assert!(sdf.field().is_none(), "nothing was stored");
    assert_eq!(sdf.grid_builds(), 0);
}

fn ready<T>(poll: Poll<T>) -> T {
    match poll {
        Poll::Ready(output) => output,
        Poll::Pending => panic!("the fill suspends only once"),
    }
}

fn cube(side: f64) -> Mesh {
    create_cube(side, side, side)
}

#[test]
fn finishing_on_a_grid_of_another_resolution_is_refused() {
    let mesh = cube(20.0);
    let coarse = grid(&mesh, 2.0, 48);
    let fine = grid(&mesh, 2.0, 64);
    let (prepared, request) = coarse.prepare_request(&none()).expect("prepare");
    fine.prepare_request(&none()).expect("prepare");
    assert_ne!(coarse.cell_size().to_bits(), fine.cell_size().to_bits());
    let answer = vec![1.0f32; (request.ni * request.nj * request.nk) as usize];

    assert_refused(fine.finish_with_distances(prepared, &answer, &none()));
    assert_unbuilt(&fine);

    // The refusal does not harm the grid: its own build still lands on the flood.
    let (reference, distances) = flood(&Shape {
        name: "fine",
        mesh: mesh.clone(),
        max_radius: 2.0,
        resolution: 64,
    });
    let (own, _) = fine.prepare_request(&none()).expect("prepare");
    fine.finish_with_distances(own, &distances, &none())
        .expect("the right grid builds");
    assert_same_grid("fine", &fine, &reference);
}

#[test]
fn finishing_on_a_grid_that_was_never_prepared_is_refused() {
    let mesh = cube(20.0);
    let prepared_grid = grid(&mesh, 2.0, 48);
    let never_prepared = grid(&mesh, 2.0, 48);
    let (prepared, request) = prepared_grid.prepare_request(&none()).expect("prepare");
    let answer = vec![1.0f32; (request.ni * request.nj * request.nk) as usize];

    assert_refused(never_prepared.finish_with_distances(prepared, &answer, &none()));

    assert_unbuilt(&never_prepared);
    assert_eq!(never_prepared.cell_size().to_bits(), 0f64.to_bits());
}

#[test]
fn a_part_with_the_same_layout_is_still_another_grid() {
    // Equal bounds give an equal cell size, origin and sample count, so only the part differs:
    // comparing the layout alone would silently swap one part's surface for the other's.
    let solid = cube(20.0);
    let mut hollow = cube(20.0);
    let mut cavity = cube(8.0);
    cavity.reverse_faces();
    hollow.copy_all_faces(&cavity, Matrix4X4::IDENTITY);
    let solid_grid = grid(&solid, 2.0, 48);
    let hollow_grid = grid(&hollow, 2.0, 48);
    let (prepared, request) = solid_grid.prepare_request(&none()).expect("prepare");
    let (_, other_request) = hollow_grid.prepare_request(&none()).expect("prepare");
    assert_eq!(
        solid_grid.cell_size().to_bits(),
        hollow_grid.cell_size().to_bits()
    );
    assert_eq!(
        [request.ni, request.nj, request.nk],
        [other_request.ni, other_request.nj, other_request.nk]
    );
    let answer = vec![1.0f32; (request.ni * request.nj * request.nk) as usize];

    assert_refused(hollow_grid.finish_with_distances(prepared, &answer, &none()));
    assert_unbuilt(&hollow_grid);
}

#[test]
fn finishing_on_an_already_built_grid_it_does_not_belong_to_is_refused() {
    let shape = &shapes()[0];
    let (reference, distances) = flood(shape);
    let built = grid(&shape.mesh, shape.max_radius, shape.resolution);
    let (own, _) = built.prepare_request(&none()).expect("prepare");
    built
        .finish_with_distances(own, &distances, &none())
        .expect("finish");
    let other = grid(&shape.mesh, shape.max_radius, shape.resolution);
    let (foreign, _) = other.prepare_request(&none()).expect("prepare");

    assert_refused(built.finish_with_distances(foreign, &distances, &none()));

    assert_eq!(built.grid_builds(), 1);
    assert_same_grid(shape.name, &built, &reference);
}

#[test]
fn changing_how_the_part_is_oriented_makes_a_prepared_build_stale() {
    let shape = &shapes()[1];
    let (_, distances) = flood(shape);
    let original = grid(&shape.mesh, shape.max_radius, shape.resolution);
    let (prepared, _) = original.prepare_request(&none()).expect("prepare");
    let changed = original.with_orient_shells(false);

    assert_refused(changed.finish_with_distances(prepared, &distances, &none()));
    assert_unbuilt(&changed);
}

#[test]
fn the_refusal_says_what_happened_and_what_to_do() {
    assert_eq!(
        SdfGridError::PreparedForAnotherGrid.to_string(),
        "These distances were prepared by a different distance grid, so they were not used. Call prepare_request on the grid you finish them on."
    );
}

#[test]
fn the_request_holds_the_parts_triangles_relative_to_the_floods_origin() {
    // Each expectation comes from the mesh and the CPU flood, never from the request's own code.
    // Inside-out parts are left out: their repair rewrites the triangles first.
    for index in [0, 3, 4] {
        let shape = &shapes()[index];
        let (reference, _) = flood(shape);
        let sdf = grid(&shape.mesh, shape.max_radius, shape.resolution);
        let (_, request) = sdf.prepare_request(&none()).expect("prepare");
        assert_eq!(sdf.orient_repairs(), 0, "{}: used as is", shape.name);

        let origin = reference.field_origin();
        let mut expected = Vec::with_capacity(shape.mesh.faces.len() * 9);
        for face in shape.mesh.faces.iter() {
            for corner in [face.v0, face.v1, face.v2] {
                let vertex = shape.mesh.vertices[corner as usize];
                expected.push((vertex.x as f64 - origin.x) as f32);
                expected.push((vertex.y as f64 - origin.y) as f32);
                expected.push((vertex.z as f64 - origin.z) as f32);
            }
        }
        let bits = |values: &[f32]| values.iter().map(|v| v.to_bits()).collect::<Vec<_>>();
        assert_eq!(bits(&request.triangles), bits(&expected), "{}", shape.name);

        let field = reference.field().expect("built");
        assert_eq!(
            [
                request.ni as usize,
                request.nj as usize,
                request.nk as usize
            ],
            [field.ni, field.nj, field.nk],
            "{}",
            shape.name
        );
        let cell = reference.cell_size();
        let dx = cell as f32;
        assert_eq!(request.cell_size.to_bits(), dx.to_bits(), "{}", shape.name);
        assert_eq!(
            request.band.to_bits(),
            ((shape.max_radius + 2.0 * cell) as f32).to_bits(),
            "{}",
            shape.name
        );
        assert_eq!(
            request.cap.to_bits(),
            (((field.ni + field.nj + field.nk) as f32) * dx).to_bits(),
            "{}",
            shape.name
        );
    }
}

#[test]
fn interleaved_awaited_builds_store_the_first_to_finish_once() {
    let shape = &shapes()[0];
    let (reference, distances) = flood(shape);
    let wrong = vec![7.0f32; distances.len()];
    let awaited = grid(&shape.mesh, shape.max_radius, shape.resolution);
    let slower = AwaitedFill::new(Ok(wrong));
    let faster = AwaitedFill::new(Ok(distances));
    let token = none();
    let mut context = Context::from_waker(Waker::noop());
    let mut first = std::pin::pin!(awaited.build_async(&slower, &token));
    let mut second = std::pin::pin!(awaited.build_async(&faster, &token));

    assert!(first.as_mut().poll(&mut context).is_pending());
    assert!(second.as_mut().poll(&mut context).is_pending());
    assert_eq!((slower.calls.get(), faster.calls.get()), (1, 1));
    assert!(awaited.field().is_none());

    ready(second.as_mut().poll(&mut context)).expect("the first to finish is stored");
    assert_same_grid(shape.name, &awaited, &reference);

    ready(first.as_mut().poll(&mut context)).expect("the later finish is not an error");
    assert_same_grid(shape.name, &awaited, &reference);
    assert_eq!(awaited.grid_builds(), 1);
}

#[test]
fn an_interleaved_failure_does_not_stop_the_other_build() {
    let shape = &shapes()[0];
    let (reference, distances) = flood(shape);
    let awaited = grid(&shape.mesh, shape.max_radius, shape.resolution);
    let failing = AwaitedFill::new(Err(DistanceBuildError::Failed("Lost.".to_string())));
    let working = AwaitedFill::new(Ok(distances));
    let token = none();
    let mut context = Context::from_waker(Waker::noop());
    let mut first = std::pin::pin!(awaited.build_async(&failing, &token));
    let mut second = std::pin::pin!(awaited.build_async(&working, &token));

    assert!(first.as_mut().poll(&mut context).is_pending());
    assert!(second.as_mut().poll(&mut context).is_pending());
    match ready(first.as_mut().poll(&mut context)) {
        Err(SdfGridError::DistanceFill(message)) => assert_eq!(message, "Lost."),
        other => panic!("expected the fill's failure, got {other:?}"),
    }
    assert!(awaited.field().is_none());

    ready(second.as_mut().poll(&mut context)).expect("the other build finishes");
    assert_same_grid(shape.name, &awaited, &reference);
    assert_eq!(awaited.grid_builds(), 1);
}

#[test]
fn a_dropped_awaited_build_leaves_the_grid_ready_to_build() {
    let shape = &shapes()[0];
    let (reference, distances) = flood(shape);
    let token = none();
    let mut context = Context::from_waker(Waker::noop());

    let awaited_after = grid(&shape.mesh, shape.max_radius, shape.resolution);
    let blocking_after = grid(&shape.mesh, shape.max_radius, shape.resolution);
    let abandoned = AwaitedFill::new(Ok(distances.clone()));
    for sdf in [&awaited_after, &blocking_after] {
        let mut build = Box::pin(sdf.build_async(&abandoned, &token));
        assert!(build.as_mut().poll(&mut context).is_pending());
        drop(build);
        assert_unbuilt(sdf);
    }

    block_on(awaited_after.build_async(&AwaitedFill::new(Ok(distances)), &none()))
        .expect("an awaited build after the drop");
    assert_same_grid(shape.name, &awaited_after, &reference);

    // A dropped build holds no lock, or this blocking build would never return.
    blocking_after
        .build(&none())
        .expect("a blocking build after the drop");
    assert_same_grid(shape.name, &blocking_after, &reference);
}
