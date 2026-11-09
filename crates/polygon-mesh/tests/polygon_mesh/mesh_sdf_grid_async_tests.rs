//! The awaited distance fill. A browser can only await a GPU read-back, so a grid can be built in
//! the steps `prepare_request`, an awaited `AsyncDistanceGridBuilder` fill and
//! `finish_with_distances`. These tests hold that path to the blocking one bit for bit on several
//! shapes, and pin what happens when a fill fails, is cancelled or returns the wrong count.

use std::cell::Cell;
use std::future::Future;
use std::pin::pin;
use std::rc::Rc;
use std::sync::{Arc, Mutex};
use std::task::{Context, Poll, Waker};

use solidmesh_math::{Matrix4X4, Vector2};
use solidmesh_mesh::csg::minkowski_processing::sphere_mesh;
use solidmesh_mesh::processors::outline_to_mesh::extrude_contours;
use solidmesh_mesh::sdf::{
    AsyncDistanceGridBuilder, DistanceBuildError, DistanceFillFuture, DistanceGridBuilder,
    DistanceGridRequest, MeshSdfGrid, OwnedDistanceGridRequest, PreparedDistanceGrid, SdfGridError,
};
use solidmesh_mesh::solids::platonic_solids::create_cube;
use solidmesh_mesh::{CancelToken, Mesh};

/// The smallest executor that runs these futures: polls until ready. Every fill here is ready at
/// its next poll, so spinning ends.
pub(crate) fn block_on<F: Future>(future: F) -> F::Output {
    let mut context = Context::from_waker(Waker::noop());
    let mut future = pin!(future);
    loop {
        if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
            return output;
        }
    }
}

/// Suspends its task once, the way an awaited read-back does.
struct SuspendOnce(bool);

impl Future for SuspendOnce {
    type Output = ();

    fn poll(mut self: std::pin::Pin<&mut Self>, context: &mut Context<'_>) -> Poll<()> {
        if self.0 {
            return Poll::Ready(());
        }
        self.0 = true;
        context.waker().wake_by_ref();
        Poll::Pending
    }
}

/// An awaited fill with a fixed answer. It holds an `Rc` across its await, as a browser fill holds
/// futures that are not `Send`, so this file only compiles if the trait allows that.
pub(crate) struct AwaitedFill {
    answer: Result<Vec<f32>, DistanceBuildError>,
    /// Cancelled while the fill is suspended, when given.
    cancel_during_fill: Option<CancelToken>,
    pub(crate) calls: Rc<Cell<i32>>,
    recorded: Rc<std::cell::RefCell<Option<OwnedDistanceGridRequest>>>,
}

impl AwaitedFill {
    pub(crate) fn new(answer: Result<Vec<f32>, DistanceBuildError>) -> Self {
        AwaitedFill {
            answer,
            cancel_during_fill: None,
            calls: Rc::new(Cell::new(0)),
            recorded: Rc::new(std::cell::RefCell::new(None)),
        }
    }
}

impl AsyncDistanceGridBuilder for AwaitedFill {
    fn compute_distances_async(&self, request: OwnedDistanceGridRequest) -> DistanceFillFuture<'_> {
        Box::pin(async move {
            let calls = Rc::clone(&self.calls);
            calls.set(calls.get() + 1);
            *self.recorded.borrow_mut() = Some(request);
            SuspendOnce(false).await;
            if let Some(cancel) = &self.cancel_during_fill {
                cancel.cancel();
            }
            drop(calls);
            self.answer.clone()
        })
    }
}

/// The blocking fill with a fixed answer, recording what it was asked.
struct BlockingFill {
    answer: Result<Vec<f32>, DistanceBuildError>,
    recorded: Mutex<Option<RecordedRequest>>,
}

impl BlockingFill {
    fn new(answer: Result<Vec<f32>, DistanceBuildError>) -> Self {
        BlockingFill {
            answer,
            recorded: Mutex::new(None),
        }
    }
}

impl DistanceGridBuilder for BlockingFill {
    fn compute_distances(
        &self,
        request: &DistanceGridRequest<'_>,
    ) -> Result<Vec<f32>, DistanceBuildError> {
        *self.recorded.lock().expect("lock") = Some(RecordedRequest {
            triangles: request.triangles.iter().map(|v| v.to_bits()).collect(),
            size: [request.ni, request.nj, request.nk],
            spacing: [
                request.cell_size.to_bits(),
                request.band.to_bits(),
                request.cap.to_bits(),
            ],
        });
        self.answer.clone()
    }
}

/// A request's contents as exact bits.
#[derive(Debug, PartialEq)]
struct RecordedRequest {
    triangles: Vec<u32>,
    size: [i32; 3],
    spacing: [u32; 3],
}

impl From<&OwnedDistanceGridRequest> for RecordedRequest {
    fn from(request: &OwnedDistanceGridRequest) -> Self {
        RecordedRequest {
            triangles: request.triangles.iter().map(|v| v.to_bits()).collect(),
            size: [request.ni, request.nj, request.nk],
            spacing: [
                request.cell_size.to_bits(),
                request.band.to_bits(),
                request.cap.to_bits(),
            ],
        }
    }
}

pub(crate) fn none() -> CancelToken {
    CancelToken::new()
}

pub(crate) fn grid(mesh: &Mesh, max_radius: f64, resolution: i32) -> MeshSdfGrid {
    MeshSdfGrid::new(mesh.clone(), max_radius, resolution).expect("valid grid arguments")
}

fn inside_out_cube(side: f64) -> Mesh {
    let mut cube = create_cube(side, side, side);
    cube.reverse_faces();
    cube
}

pub(crate) struct Shape {
    pub(crate) name: &'static str,
    pub(crate) mesh: Mesh,
    pub(crate) max_radius: f64,
    pub(crate) resolution: i32,
}

/// One each of: the plain case, a part the orientation repair rewrites, two shells, a curved
/// surface and a concave outline.
pub(crate) fn shapes() -> Vec<Shape> {
    let mut with_cavity = create_cube(20.0, 20.0, 20.0);
    with_cavity.copy_all_faces(&inside_out_cube(8.0), Matrix4X4::IDENTITY);
    let bracket = [
        (0.0, 0.0),
        (40.0, 0.0),
        (40.0, 4.0),
        (4.0, 4.0),
        (4.0, 30.0),
        (0.0, 30.0),
    ]
    .map(|(x, y)| Vector2::new(x, y))
    .to_vec();
    vec![
        Shape {
            name: "cube",
            mesh: create_cube(20.0, 20.0, 20.0),
            max_radius: 2.0,
            resolution: 48,
        },
        Shape {
            name: "inside-out cube",
            mesh: inside_out_cube(20.0),
            max_radius: 2.0,
            resolution: 48,
        },
        Shape {
            name: "cube with a cavity",
            mesh: with_cavity,
            max_radius: 3.0,
            resolution: 64,
        },
        Shape {
            name: "sphere",
            mesh: sphere_mesh(10.0, 24).expect("sphere"),
            max_radius: 1.5,
            resolution: 56,
        },
        Shape {
            name: "bracket",
            mesh: extrude_contours(&[bracket], 20.0).expect("extrude"),
            max_radius: 1.5,
            resolution: 72,
        },
    ]
}

/// The CPU flood's own field, and the unsigned distances it holds: what a fill must return for
/// its grid to land on the flood's values.
pub(crate) fn flood(shape: &Shape) -> (MeshSdfGrid, Vec<f32>) {
    let reference = grid(&shape.mesh, shape.max_radius, shape.resolution);
    reference.build(&none()).expect("the CPU flood builds");
    let distances = reference
        .field()
        .expect("built")
        .buffer
        .iter()
        .map(|value| value.abs())
        .collect();
    (reference, distances)
}

fn field_bits(sdf: &MeshSdfGrid) -> ([usize; 3], Vec<u32>) {
    let field = sdf.field().expect("built");
    (
        [field.ni, field.nj, field.nk],
        field.buffer.iter().map(|value| value.to_bits()).collect(),
    )
}

/// A mesh's vertices, faces and face normals as exact bits and indices.
type MeshBits = (Vec<[u32; 3]>, Vec<[i32; 3]>, Vec<[u32; 3]>);

/// Every number and order a caller could observe in a mesh.
fn mesh_bits(mesh: &Mesh) -> MeshBits {
    (
        mesh.vertices
            .iter()
            .map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()])
            .collect(),
        mesh.faces.iter().map(|f| [f.v0, f.v1, f.v2]).collect(),
        mesh.faces
            .iter()
            .map(|f| {
                [
                    f.normal.x.to_bits(),
                    f.normal.y.to_bits(),
                    f.normal.z.to_bits(),
                ]
            })
            .collect(),
    )
}

pub(crate) fn assert_same_grid(name: &str, actual: &MeshSdfGrid, expected: &MeshSdfGrid) {
    assert_eq!(
        field_bits(actual),
        field_bits(expected),
        "{name}: field bits"
    );
    let (a, e) = (actual.field_origin(), expected.field_origin());
    assert_eq!(
        [a.x.to_bits(), a.y.to_bits(), a.z.to_bits()],
        [e.x.to_bits(), e.y.to_bits(), e.z.to_bits()],
        "{name}: origin bits"
    );
    assert_eq!(
        actual.cell_size().to_bits(),
        expected.cell_size().to_bits(),
        "{name}: cell size bits"
    );
}

#[test]
fn awaited_fill_builds_the_same_grid_and_meshes_as_the_blocking_paths() {
    for shape in shapes() {
        let (reference, distances) = flood(&shape);

        let blocking = grid(&shape.mesh, shape.max_radius, shape.resolution)
            .with_distance_builder(Arc::new(BlockingFill::new(Ok(distances.clone()))));
        blocking.build(&none()).expect("blocking fill");

        let awaited = grid(&shape.mesh, shape.max_radius, shape.resolution);
        let fill = AwaitedFill::new(Ok(distances));
        block_on(awaited.build_async(&fill, &none())).expect("awaited fill");

        assert_eq!(awaited.grid_builds(), 1, "{}", shape.name);
        assert_same_grid(shape.name, &awaited, &blocking);
        assert_same_grid(shape.name, &awaited, &reference);

        for ratio in [1.0, 0.5, -0.5, -1.0] {
            let iso = shape.max_radius * ratio;
            let expected = reference.extract(iso, &none()).expect("flood offset");
            let through_blocking = blocking.extract(iso, &none()).expect("blocking offset");
            let through_awaited =
                block_on(awaited.extract_async(iso, &fill, &none())).expect("awaited offset");
            assert!(!expected.faces.is_empty(), "{}: offset {iso}", shape.name);
            assert_eq!(
                mesh_bits(&through_awaited),
                mesh_bits(&expected),
                "{}: awaited offset {iso}",
                shape.name
            );
            assert_eq!(
                mesh_bits(&through_blocking),
                mesh_bits(&expected),
                "{}: blocking offset {iso}",
                shape.name
            );
        }
        // Extracting on an already built grid never asks the fill again.
        assert_eq!(fill.calls.get(), 1, "{}", shape.name);
        assert_eq!(awaited.grid_builds(), 1, "{}", shape.name);
    }
}

#[test]
fn extract_async_on_an_unbuilt_grid_fills_it_and_returns_the_offset() {
    let shape = &shapes()[0];
    let (reference, distances) = flood(shape);
    let awaited = grid(&shape.mesh, shape.max_radius, shape.resolution);
    let fill = AwaitedFill::new(Ok(distances));

    let mesh = block_on(awaited.extract_async(1.0, &fill, &none())).expect("awaited offset");

    assert_eq!(fill.calls.get(), 1);
    assert_eq!(
        mesh_bits(&mesh),
        mesh_bits(&reference.extract(1.0, &none()).expect("flood offset"))
    );
}

#[test]
fn awaited_fill_is_asked_exactly_what_the_blocking_fill_is_asked() {
    for shape in shapes() {
        let (_, distances) = flood(&shape);
        let blocking_fill = Arc::new(BlockingFill::new(Ok(distances.clone())));
        grid(&shape.mesh, shape.max_radius, shape.resolution)
            .with_distance_builder(blocking_fill.clone())
            .build(&none())
            .expect("blocking fill");

        let fill = AwaitedFill::new(Ok(distances));
        block_on(grid(&shape.mesh, shape.max_radius, shape.resolution).build_async(&fill, &none()))
            .expect("awaited fill");

        let awaited = fill.recorded.borrow();
        assert_eq!(
            RecordedRequest::from(awaited.as_ref().expect("asked")),
            blocking_fill
                .recorded
                .lock()
                .expect("lock")
                .take()
                .expect("asked"),
            "{}",
            shape.name
        );
    }
}

#[test]
fn prepare_request_and_finish_with_distances_build_the_grid_by_hand() {
    let shape = &shapes()[3];
    let (reference, distances) = flood(shape);
    let by_hand = grid(&shape.mesh, shape.max_radius, shape.resolution);

    let (prepared, request): (PreparedDistanceGrid, OwnedDistanceGridRequest) =
        by_hand.prepare_request(&none()).expect("prepare");
    assert_eq!(
        (request.ni * request.nj * request.nk) as usize,
        distances.len()
    );
    assert!(by_hand.field().is_none(), "preparing builds nothing");
    assert_eq!(by_hand.grid_builds(), 0);

    by_hand
        .finish_with_distances(prepared, &distances, &none())
        .expect("finish");

    assert_eq!(by_hand.grid_builds(), 1);
    assert_same_grid(shape.name, &by_hand, &reference);
}

#[test]
fn the_grid_stays_unbuilt_until_the_awaited_fill_resolves() {
    let shape = &shapes()[0];
    let (_, distances) = flood(shape);
    let awaited = grid(&shape.mesh, shape.max_radius, shape.resolution);
    let fill = AwaitedFill::new(Ok(distances));
    let token = none();

    let mut context = Context::from_waker(Waker::noop());
    let mut build = pin!(awaited.build_async(&fill, &token));

    assert!(build.as_mut().poll(&mut context).is_pending());
    assert_eq!(fill.calls.get(), 1);
    assert!(awaited.field().is_none());
    assert_eq!(awaited.grid_builds(), 0);

    match build.as_mut().poll(&mut context) {
        Poll::Ready(result) => result.expect("awaited fill"),
        Poll::Pending => panic!("the fill suspends only once"),
    }
    assert!(awaited.field().is_some());
    assert_eq!(awaited.grid_builds(), 1);
}

#[test]
fn a_failed_awaited_fill_is_the_distance_fill_error_and_never_the_cpu_flood() {
    let shape = &shapes()[0];
    let (_, distances) = flood(shape);
    let awaited = grid(&shape.mesh, shape.max_radius, shape.resolution);
    let failing = AwaitedFill::new(Err(DistanceBuildError::Failed(
        "The device was lost.".to_string(),
    )));

    match block_on(awaited.build_async(&failing, &none())) {
        Err(SdfGridError::DistanceFill(message)) => assert_eq!(message, "The device was lost."),
        other => panic!("expected the fill's failure, got {other:?}"),
    }
    match block_on(awaited.extract_async(1.0, &failing, &none())) {
        Err(SdfGridError::DistanceFill(message)) => assert_eq!(message, "The device was lost."),
        other => panic!("expected the fill's failure, got {other:?}"),
    }

    assert!(awaited.field().is_none(), "nothing was stored");
    assert_eq!(awaited.grid_builds(), 0);
    assert_eq!(awaited.flood_work(), 0, "the CPU flood never ran");

    // The failure is not remembered: the next fill builds the grid.
    block_on(awaited.build_async(&AwaitedFill::new(Ok(distances)), &none())).expect("retry");
    assert_eq!(awaited.grid_builds(), 1);
}

#[test]
fn a_fill_that_reports_cancelled_is_the_cancelled_error() {
    let shape = &shapes()[0];
    let awaited = grid(&shape.mesh, shape.max_radius, shape.resolution);
    let cancelled = AwaitedFill::new(Err(DistanceBuildError::Cancelled));

    assert!(matches!(
        block_on(awaited.build_async(&cancelled, &none())),
        Err(SdfGridError::Cancelled)
    ));
    assert!(awaited.field().is_none());
}

#[test]
fn a_cancel_during_the_awaited_fill_discards_its_distances() {
    let shape = &shapes()[0];
    let (reference, distances) = flood(shape);
    let awaited = grid(&shape.mesh, shape.max_radius, shape.resolution);
    let token = none();
    let mut fill = AwaitedFill::new(Ok(distances.clone()));
    fill.cancel_during_fill = Some(token.clone());

    assert!(matches!(
        block_on(awaited.build_async(&fill, &token)),
        Err(SdfGridError::Cancelled)
    ));
    assert!(awaited.field().is_none());
    assert_eq!(awaited.grid_builds(), 0);

    // A cancelled build starts over.
    block_on(awaited.build_async(&AwaitedFill::new(Ok(distances)), &none())).expect("retry");
    assert_same_grid(shape.name, &awaited, &reference);
}

#[test]
fn a_cancelled_token_stops_before_the_fill_is_asked() {
    let shape = &shapes()[0];
    let awaited = grid(&shape.mesh, shape.max_radius, shape.resolution);
    let fill = AwaitedFill::new(Ok(Vec::new()));
    let token = none();
    token.cancel();

    assert!(matches!(
        block_on(awaited.build_async(&fill, &token)),
        Err(SdfGridError::Cancelled)
    ));
    assert_eq!(fill.calls.get(), 0);
}

#[test]
fn too_few_distances_are_refused_with_the_counts_on_both_paths() {
    let shape = &shapes()[0];
    let (_, distances) = flood(shape);
    let size = distances.len();
    let short = distances[..size - 1].to_vec();
    let expected = format!(
        "The distance fill returned {} distances for a grid of {size} samples.",
        size - 1
    );

    let awaited = grid(&shape.mesh, shape.max_radius, shape.resolution);
    match block_on(awaited.build_async(&AwaitedFill::new(Ok(short.clone())), &none())) {
        Err(SdfGridError::DistanceFill(message)) => assert_eq!(message, expected),
        other => panic!("expected the count refusal, got {other:?}"),
    }
    assert!(awaited.field().is_none());
    assert_eq!(awaited.grid_builds(), 0);
    assert_eq!(awaited.flood_work(), 0, "the CPU flood never ran");

    let blocking = grid(&shape.mesh, shape.max_radius, shape.resolution)
        .with_distance_builder(Arc::new(BlockingFill::new(Ok(short))));
    match blocking.build(&none()) {
        Err(SdfGridError::DistanceFill(message)) => assert_eq!(message, expected),
        other => panic!("expected the count refusal, got {other:?}"),
    }
    assert!(blocking.field().is_none());

    // No distances at all is refused the same way, not a panic.
    let empty = grid(&shape.mesh, shape.max_radius, shape.resolution);
    assert!(matches!(
        block_on(empty.build_async(&AwaitedFill::new(Ok(Vec::new())), &none())),
        Err(SdfGridError::DistanceFill(_))
    ));
}

#[test]
fn extra_distances_are_ignored_on_both_paths() {
    let shape = &shapes()[0];
    let (reference, mut distances) = flood(shape);
    distances.extend([1.0f32, 2.0, 3.0]);

    let awaited = grid(&shape.mesh, shape.max_radius, shape.resolution);
    block_on(awaited.build_async(&AwaitedFill::new(Ok(distances.clone())), &none()))
        .expect("awaited fill");
    assert_same_grid(shape.name, &awaited, &reference);

    let blocking = grid(&shape.mesh, shape.max_radius, shape.resolution)
        .with_distance_builder(Arc::new(BlockingFill::new(Ok(distances))));
    blocking.build(&none()).expect("blocking fill");
    assert_same_grid(shape.name, &blocking, &reference);
}

#[test]
fn an_offset_past_the_range_is_refused_before_the_fill_is_asked() {
    let shape = &shapes()[0];
    let awaited = grid(&shape.mesh, shape.max_radius, shape.resolution);
    let fill = AwaitedFill::new(Ok(Vec::new()));

    assert!(matches!(
        block_on(awaited.extract_async(shape.max_radius * 2.0, &fill, &none())),
        Err(SdfGridError::OutOfRange(_))
    ));
    assert_eq!(fill.calls.get(), 0);
}

#[test]
fn a_built_grid_is_not_filled_again_and_a_late_finish_changes_nothing() {
    let shape = &shapes()[0];
    let (reference, distances) = flood(shape);
    let awaited = grid(&shape.mesh, shape.max_radius, shape.resolution);

    // Two fills under way at once: only the first to finish is stored.
    let (first, _) = awaited.prepare_request(&none()).expect("prepare");
    let (second, _) = awaited.prepare_request(&none()).expect("prepare");
    awaited
        .finish_with_distances(first, &distances, &none())
        .expect("first finish");
    let wrong = vec![7.0f32; distances.len()];
    awaited
        .finish_with_distances(second, &wrong, &none())
        .expect("a late finish is not an error");
    assert_same_grid(shape.name, &awaited, &reference);
    assert_eq!(awaited.grid_builds(), 1);

    let fill = AwaitedFill::new(Ok(wrong));
    block_on(awaited.build_async(&fill, &none())).expect("already built");
    assert_eq!(fill.calls.get(), 0);
    assert_same_grid(shape.name, &awaited, &reference);
}

#[test]
fn the_two_halves_can_run_as_jobs_on_other_threads() {
    fn is_send<T: Send>() {}
    fn is_send_and_sync<T: Send + Sync>() {}
    is_send::<PreparedDistanceGrid>();
    is_send::<OwnedDistanceGridRequest>();
    is_send_and_sync::<MeshSdfGrid>();

    let shape = &shapes()[0];
    let (reference, distances) = flood(shape);
    let shared = Arc::new(grid(&shape.mesh, shape.max_radius, shape.resolution));

    let prepare_grid = Arc::clone(&shared);
    let (prepared, request) =
        std::thread::spawn(move || prepare_grid.prepare_request(&CancelToken::new()))
            .join()
            .expect("prepare job")
            .expect("prepare");
    assert_eq!(
        (request.ni * request.nj * request.nk) as usize,
        distances.len()
    );

    let finish_grid = Arc::clone(&shared);
    std::thread::spawn(move || {
        finish_grid.finish_with_distances(prepared, &distances, &CancelToken::new())
    })
    .join()
    .expect("finish job")
    .expect("finish");

    assert_same_grid(shape.name, &shared, &reference);
}

#[test]
fn the_owned_request_lends_itself_as_the_blocking_request() {
    let shape = &shapes()[0];
    let awaited = grid(&shape.mesh, shape.max_radius, shape.resolution);
    let (_, mut request) = awaited.prepare_request(&none()).expect("prepare");
    let seen = Arc::new(Mutex::new(Vec::new()));
    let sink = Arc::clone(&seen);
    request.stage_done = Some(Arc::new(move |stage| {
        sink.lock().expect("lock").push(stage);
    }));

    let lent = request.as_request();
    assert_eq!(lent.triangles.as_ptr(), request.triangles.as_ptr());
    assert_eq!(
        (lent.ni, lent.nj, lent.nk),
        (request.ni, request.nj, request.nk)
    );
    assert_eq!(lent.cell_size.to_bits(), request.cell_size.to_bits());
    assert_eq!(lent.band.to_bits(), request.band.to_bits());
    assert_eq!(lent.cap.to_bits(), request.cap.to_bits());
    (lent.stage_done.expect("the callback is lent"))(2);
    assert_eq!(*seen.lock().expect("lock"), vec![2]);
}
