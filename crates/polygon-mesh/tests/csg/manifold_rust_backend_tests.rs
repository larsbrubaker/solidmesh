//! The manifold-rust boolean backend, exercised through the public `boolean_processing` entry
//! points.
//!
//! A refused operand is asserted by the kernel status's `Debug` name in the error message
//! (`NotClosed`), the name a user's error report carries.
//!
//! The cancellation and progress cases are in `manifold_rust_backend_cancel_tests.rs`, split for
//! the 800-line limit.

use std::collections::HashMap;

use polygon_mesh::csg::{
    boolean_processing, manifold_import, BooleanError, BooleanOptions, CsgModes, WindingRule,
};
use polygon_mesh::solids::platonic_solids::create_cube;
use polygon_mesh::{CancelToken, Face, Mesh};
use vector_math::seeded_random::SeededRandom;
use vector_math::Color;
use vector_math::{Matrix4X4, Vector3, Vector3Float};

const NOT_CLOSED: &str = "NotClosed";
const NON_FINITE_VERTEX: &str = "NonFiniteVertex";

fn translation(x: f64, y: f64, z: f64) -> Matrix4X4 {
    Matrix4X4::create_translation(x, y, z)
}

fn do_array(
    items: &[(&Mesh, Matrix4X4)],
    operation: CsgModes,
    options: &BooleanOptions,
) -> Result<Mesh, BooleanError> {
    boolean_processing::do_array(items, operation, &CancelToken::new(), options)
}

fn union_subtract_intersect(
    operation: CsgModes,
    a: &Mesh,
    matrix_a: Matrix4X4,
    b: &Mesh,
    matrix_b: Matrix4X4,
) -> Mesh {
    let mut result = do_array(
        &[(a, matrix_a), (b, matrix_b)],
        operation,
        &BooleanOptions::default(),
    )
    .expect("boolean runs");
    // What CombineParticipants does to a boolean result before anyone looks at it. The raw
    // MeshGL output splits vertices at run boundaries, so topology questions are only
    // meaningful after the merge.
    result.clean_and_merge();
    result.remove_unused_vertices();
    result
}

fn assert_within(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {expected} within {tolerance}, got {actual}"
    );
}

#[test]
fn union_of_overlapping_cubes_is_closed_solid() {
    let result = union_subtract_intersect(
        CsgModes::Union,
        &create_cube(10.0, 10.0, 10.0),
        translation(-3.0, 0.0, 0.0),
        &create_cube(10.0, 10.0, 10.0),
        translation(3.0, 0.0, 0.0),
    );
    assert!(!result.faces.is_empty());
    assert!(result.is_manifold());
    let bounds = result.get_axis_aligned_bounding_box();
    assert_within(bounds.x_size(), 16.0, 0.001);
    assert_within(bounds.y_size(), 10.0, 0.001);
    assert_within(bounds.z_size(), 10.0, 0.001);
}

#[test]
fn subtract_of_overlapping_cubes_is_closed_solid() {
    let result = union_subtract_intersect(
        CsgModes::Subtract,
        &create_cube(20.0, 20.0, 20.0),
        Matrix4X4::IDENTITY,
        &create_cube(10.0, 10.0, 10.0),
        translation(10.0, 10.0, 10.0),
    );
    // A corner bite out of a cube: still one closed solid, and more faces than the 12 the
    // cube started with.
    assert!(result.faces.len() > 12);
    assert!(result.is_manifold());
    let bounds = result.get_axis_aligned_bounding_box();
    assert_within(bounds.x_size(), 20.0, 0.001);
    assert_within(bounds.y_size(), 20.0, 0.001);
    assert_within(bounds.z_size(), 20.0, 0.001);
}

#[test]
fn intersect_of_overlapping_cubes_is_the_overlap_box() {
    let result = union_subtract_intersect(
        CsgModes::Intersect,
        &create_cube(10.0, 10.0, 10.0),
        translation(-3.0, 0.0, 0.0),
        &create_cube(10.0, 10.0, 10.0),
        translation(3.0, 0.0, 0.0),
    );
    assert!(!result.faces.is_empty());
    assert!(result.is_manifold());
    let bounds = result.get_axis_aligned_bounding_box();
    assert_within(bounds.x_size(), 4.0, 0.001);
    assert_within(bounds.y_size(), 10.0, 0.001);
    assert_within(bounds.z_size(), 10.0, 0.001);
}

#[test]
fn closed_non_manifold_input_still_runs_on_the_kernel() {
    let pinched = two_cubes_sharing_one_edge();

    assert!(!pinched.is_manifold());
    assert!(
        pinched
            .get_non_manifold_edges()
            .iter()
            .all(|e| e.faces().len() == 4),
        "a closed surface has no edge with fewer than two faces"
    );

    let straddle = create_cube(10.0, 10.0, 20.0);
    let mut result = do_array(
        &[
            (&pinched, Matrix4X4::IDENTITY),
            // Straddles the pinch, so the union is one honest solid rather than two lobes
            // still touching along a single edge.
            (&straddle, translation(5.0, 5.0, 0.0)),
        ],
        CsgModes::Union,
        &BooleanOptions::default(),
    )
    .expect("the robust import takes closed non-manifold input");
    result.clean_and_merge();
    result.remove_unused_vertices();

    assert!(!result.faces.is_empty());
    assert!(result.is_manifold());
}

#[test]
fn soup_operand_with_colors_still_runs_on_the_kernel() {
    let pinched = two_cubes_sharing_one_edge();
    let straddle = create_cube(10.0, 10.0, 20.0);

    let mut result = do_array(
        &[
            (&pinched, Matrix4X4::IDENTITY),
            // Straddles the pinch, so the union is one honest solid rather than two lobes
            // still touching along a single edge.
            (&straddle, translation(5.0, 5.0, 0.0)),
        ],
        CsgModes::Union,
        &BooleanOptions {
            mesh_colors: Some(vec![Color::RED, Color::BLUE]),
            ..BooleanOptions::default()
        },
    )
    .expect("the robust import takes closed non-manifold input");
    result.clean_and_merge();
    result.remove_unused_vertices();

    assert!(!result.faces.is_empty());
    assert!(result.is_manifold());

    // Colours may be lost for the soup operand, but whatever comes back has to be a usable
    // parallel array - a mismatched length is what corrupts rendering later.
    if let Some(face_colors) = &result.face_colors {
        assert_eq!(face_colors.len(), result.faces.len());
    }
}

#[test]
fn open_mesh_throws_rather_than_silently_producing_other_geometry() {
    let cube = create_cube(10.0, 10.0, 10.0);
    let open = open_box(8.0);
    let error = do_array(
        &[(&cube, Matrix4X4::IDENTITY), (&open, Matrix4X4::IDENTITY)],
        CsgModes::Subtract,
        &BooleanOptions::default(),
    )
    .expect_err("an open operand is refused");

    assert!(
        error.to_string().contains(NOT_CLOSED),
        "the status is what tells the user which operand was unusable: {error}"
    );
    // A subtraction is defined by every operand, so there is nothing to degrade to.
    assert!(!matches!(error, BooleanError::Partial { .. }));
}

#[test]
fn a_union_keeps_the_operands_that_worked_and_names_the_one_that_did_not() {
    let cube = create_cube(10.0, 10.0, 10.0);
    let open = open_box(8.0);
    let error = do_array(
        &[
            (&cube, Matrix4X4::IDENTITY),
            (&open, translation(20.0, 0.0, 0.0)),
        ],
        CsgModes::Union,
        &BooleanOptions::default(),
    )
    .expect_err("a refused operand is reported");

    let BooleanError::Partial {
        message,
        partial_result,
        skipped_operands,
    } = error
    else {
        panic!("expected a partial result, got {error:?}");
    };
    assert_eq!(skipped_operands.len(), 1);
    assert_eq!(
        skipped_operands[0].index, 1,
        "the caller names the part from its position in the list it handed in"
    );
    assert!(
        message.contains(NOT_CLOSED),
        "a caller that only logs the message must still learn what was left out"
    );

    let mut result = *partial_result;
    result.clean_and_merge();
    result.remove_unused_vertices();
    assert!(result.is_manifold());

    // The good operand alone, untouched by the one that could not be used.
    let bounds = result.get_axis_aligned_bounding_box();
    assert_within(bounds.x_size(), 10.0, 0.001);
    assert_within(bounds.y_size(), 10.0, 0.001);
    assert_within(bounds.z_size(), 10.0, 0.001);
}

#[test]
fn a_union_whose_operands_are_all_refused_still_names_them_all_partially() {
    let open = open_box(8.0);
    let error = do_array(
        &[
            (&open, Matrix4X4::IDENTITY),
            (&open, translation(20.0, 0.0, 0.0)),
        ],
        CsgModes::Union,
        &BooleanOptions::default(),
    )
    .expect_err("every operand is refused");

    let BooleanError::Partial {
        message,
        partial_result,
        skipped_operands,
    } = error
    else {
        panic!("expected a partial result, got {error:?}");
    };
    assert_eq!(
        skipped_operands.len(),
        2,
        "every operand was refused, so every one of them has to be named"
    );
    let indices: Vec<usize> = skipped_operands.iter().map(|s| s.index).collect();
    assert_eq!(indices, vec![0, 1]);
    assert!(message.contains(NOT_CLOSED));
    assert_eq!(
        partial_result.faces.len(),
        0,
        "no operand contributed geometry - the answer is empty, not wrong"
    );
}

#[test]
fn a_non_finite_operand_is_skipped_per_operand_like_any_other_refused_geometry() {
    let cube = create_cube(10.0, 10.0, 10.0);
    let bad = cube_with_a_non_finite_vertex();
    let error = do_array(
        &[
            (&cube, Matrix4X4::IDENTITY),
            (&bad, translation(20.0, 0.0, 0.0)),
        ],
        CsgModes::Union,
        &BooleanOptions::default(),
    )
    .expect_err("the non-finite operand is reported");

    let BooleanError::Partial {
        partial_result,
        skipped_operands,
        ..
    } = error
    else {
        panic!("expected a partial result, got {error:?}");
    };
    assert_eq!(skipped_operands.len(), 1);
    assert_eq!(skipped_operands[0].index, 1);
    assert!(
        skipped_operands[0].reason.contains(NON_FINITE_VERTEX),
        "the status is the whole diagnostic value of refusing the input: {}",
        skipped_operands[0].reason
    );
    let bounds = partial_result.get_axis_aligned_bounding_box();
    assert_within(bounds.x_size(), 10.0, 0.001);
}

#[test]
fn an_input_the_kernel_rejects_throws_rather_than_going_missing() {
    let cube = create_cube(10.0, 10.0, 10.0);
    let bad = cube_with_a_non_finite_vertex();

    // A NaN coordinate leaves the surface perfectly manifold, so it is the kernel's geometric
    // validation that objects.
    assert!(
        bad.is_manifold(),
        "the rejection under test has to be geometric, not topological"
    );

    // `do_array` is the only n-ary entry point, so one call covers the backend and the seam.
    let error = do_array(
        &[(&cube, Matrix4X4::IDENTITY), (&bad, Matrix4X4::IDENTITY)],
        CsgModes::Union,
        &BooleanOptions::default(),
    )
    .expect_err("the non-finite operand is reported");
    assert!(
        error.to_string().contains(NON_FINITE_VERTEX),
        "the status is the whole diagnostic value of refusing the input: {error}"
    );
}

#[test]
fn cancelled_token_propagates_as_cancellation() {
    let cancelled = CancelToken::new();
    cancelled.cancel();
    let a = create_cube(10.0, 10.0, 10.0);
    let b = create_cube(10.0, 10.0, 10.0);
    let result = boolean_processing::do_array(
        &[
            (&a, translation(-3.0, 0.0, 0.0)),
            (&b, translation(3.0, 0.0, 0.0)),
        ],
        CsgModes::Union,
        &cancelled,
        &BooleanOptions::default(),
    );
    assert!(matches!(result, Err(BooleanError::Cancelled)));
}

#[test]
fn nonzero_winding_keeps_inverted_material_that_positive_discards() {
    // Two operands, not one: a single operand is its own answer and no boolean - and so no
    // winding rule - ever runs.
    let first = outward_cube_with_an_inverted_neighbour();
    let second = create_cube(4.0, 4.0, 4.0);
    let volume_under = |rule: WindingRule| {
        let result = do_array(
            &[
                (&first, Matrix4X4::IDENTITY),
                (&second, translation(40.0, 0.0, 0.0)),
            ],
            CsgModes::Union,
            &BooleanOptions {
                winding_rule: rule,
                ..BooleanOptions::default()
            },
        )
        .expect("boolean runs");
        signed_volume(&result)
    };

    let positive = volume_under(WindingRule::Positive);
    let nonzero = volume_under(WindingRule::Nonzero);

    assert!(positive > 0.0);
    assert!(
        nonzero > positive,
        "nonzero {nonzero} vs positive {positive}"
    );
}

#[test]
fn repair_orientation_makes_an_inverted_operand_union_like_a_correct_one() {
    let mut inverted = create_cube(10.0, 10.0, 10.0);
    inverted.reverse_faces();
    let other = create_cube(10.0, 10.0, 10.0);

    let union_volume = |first: &Mesh, repair_orientation: bool| {
        let result = do_array(
            &[
                (first, translation(-3.0, 0.0, 0.0)),
                (&other, translation(3.0, 0.0, 0.0)),
            ],
            CsgModes::Union,
            &BooleanOptions {
                repair_orientation,
                ..BooleanOptions::default()
            },
        )
        .expect("boolean runs");
        signed_volume(&result)
    };

    let repaired = union_volume(&inverted, true);
    let correct = union_volume(&create_cube(10.0, 10.0, 10.0), false);

    // 16 x 10 x 10 either way.
    assert_within(correct, 1600.0, 0.001);
    assert_within(repaired, correct, 0.001);
}

/// The signed volume of a closed mesh, positive when its faces wind outward.
pub(super) fn signed_volume(mesh: &Mesh) -> f64 {
    let mut total = 0.0;
    for face in mesh.faces.iter() {
        let a = Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]);
        let b = Vector3::from_vector3_float(mesh.vertices[face.v1 as usize]);
        let c = Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]);
        total += a.dot(b.cross(c)) / 6.0;
    }
    total
}

/// One mesh holding two cubes that overlap, the second of them wound inside out.
fn outward_cube_with_an_inverted_neighbour() -> Mesh {
    let mut mesh = create_cube(10.0, 10.0, 10.0);
    let mut inverted = create_cube(10.0, 10.0, 10.0);
    inverted.transform(translation(5.0, 5.0, 5.0));
    inverted.reverse_faces();

    let offset = mesh.vertices.len() as i32;
    mesh.vertices.extend(inverted.vertices.iter().copied());
    for face in inverted.faces.iter() {
        let face = Face::new(
            face.v0 + offset,
            face.v1 + offset,
            face.v2 + offset,
            &mesh.vertices,
        );
        mesh.faces.push(face);
    }
    mesh
}

/// Two cubes welded into one mesh so that they meet along exactly one shared edge.
fn two_cubes_sharing_one_edge() -> Mesh {
    let mut mesh = Mesh::new();
    let mut vertex_map: HashMap<[u32; 3], i32> = HashMap::new();

    let mut append = |mesh: &mut Mesh, matrix: Matrix4X4| {
        let mut cube = create_cube(10.0, 10.0, 10.0);
        cube.transform(matrix);
        let mut weld = |mesh: &mut Mesh, source_index: i32| {
            let position = cube.vertices[source_index as usize];
            let key = [
                position.x.to_bits(),
                position.y.to_bits(),
                position.z.to_bits(),
            ];
            *vertex_map.entry(key).or_insert_with(|| {
                mesh.vertices.push(position);
                mesh.vertices.len() as i32 - 1
            })
        };
        for face in cube.faces.clone().iter() {
            let (v0, v1, v2) = (
                weld(mesh, face.v0),
                weld(mesh, face.v1),
                weld(mesh, face.v2),
            );
            let face = Face::new(v0, v1, v2, &mesh.vertices);
            mesh.faces.push(face);
        }
    };

    // [-5, 5] and [5, 15] in both x and y: the two cubes touch only along the vertical line
    // x = 5, y = 5, and welding makes that one shared mesh edge.
    append(&mut mesh, Matrix4X4::IDENTITY);
    append(&mut mesh, translation(10.0, 10.0, 0.0));
    mesh
}

fn open_box(size: f64) -> Mesh {
    let mut mesh = create_cube(size, size, size);
    // Drop the two triangles of one face; any hole makes the surface open.
    mesh.faces.pop();
    mesh.faces.pop();
    mesh
}

fn cube_with_a_non_finite_vertex() -> Mesh {
    let mut mesh = create_cube(10.0, 10.0, 10.0);
    let corner = mesh.vertices[0];
    mesh.vertices[0] = Vector3Float {
        x: f32::NAN,
        ..corner
    };
    mesh
}

/// A solid whose seams came apart in the last digits still unions. The kernel welds by exact
/// position and has no tolerance of its own, so such a mesh reads as NotClosed to it; the import
/// retries with a tolerance-welded copy rather than refusing the part.
#[test]
fn a_seam_split_by_rounding_still_unions() {
    let split = cube_with_split_seams(10.0, 1e-6);

    // Every triangle owns its own corners, so every edge is a boundary edge.
    assert!(
        !split.is_manifold(),
        "the input under test has to be one the kernel would refuse untouched"
    );

    let result = union_subtract_intersect(
        CsgModes::Union,
        &split,
        translation(-3.0, 0.0, 0.0),
        &create_cube(10.0, 10.0, 10.0),
        translation(3.0, 0.0, 0.0),
    );

    assert!(!result.faces.is_empty());
    assert!(result.is_manifold());

    // The welded operand still has to be the same 10mm cube it looked like.
    let bounds = result.get_axis_aligned_bounding_box();
    assert_within(bounds.x_size(), 16.0, 0.001);
    assert_within(bounds.y_size(), 10.0, 0.001);
    assert_within(bounds.z_size(), 10.0, 0.001);
}

/// The strict import takes no second chance: the split-seam solid the retrying import welds and
/// accepts is refused as the kernel reads it, `NotClosed`.
#[test]
fn rust_only_the_strict_import_does_not_weld_and_retry() {
    let split = cube_with_split_seams(10.0, 1e-6);

    let rejected = manifold_import::import_strict(&split)
        .err()
        .expect("the strict import must refuse what the kernel refuses");
    assert!(
        rejected.message.contains(NOT_CLOSED),
        "{}",
        rejected.message
    );

    // The same mesh through the retrying import is welded and taken - the retry is the only
    // difference between the two.
    assert!(manifold_import::import(&split, false).is_ok());

    // And a mesh the kernel takes as given comes through the strict import unchanged.
    let cube = manifold_import::import_strict(&create_cube(10.0, 10.0, 10.0))
        .expect("a closed cube imports");
    assert!((cube.volume() - 1000.0).abs() < 1e-9);
}

/// The same split-seam solid, out where the coordinates are large, still unions: the float
/// rounding that splits a seam scales with distance from the origin, not with the part.
#[test]
fn a_seam_split_by_rounding_still_unions_far_from_the_origin() {
    const FAR_FROM_ORIGIN: f64 = 5000.0;

    let split =
        cube_with_seams_split_by_float_spacing(10.0, Vector3::new(FAR_FROM_ORIGIN, 0.0, 0.0), 2);

    assert!(
        !split.is_manifold(),
        "the input under test has to be one the kernel would refuse untouched"
    );

    let result = union_subtract_intersect(
        CsgModes::Union,
        &split,
        Matrix4X4::IDENTITY,
        &create_cube(10.0, 10.0, 10.0),
        translation(FAR_FROM_ORIGIN + 6.0, 0.0, 0.0),
    );

    assert!(!result.faces.is_empty());
    assert!(result.is_manifold());

    let bounds = result.get_axis_aligned_bounding_box();
    assert_within(bounds.x_size(), 16.0, 0.01);
    assert_within(bounds.y_size(), 10.0, 0.01);
    assert_within(bounds.z_size(), 10.0, 0.01);
}

/// `MathF.BitIncrement`: the next float towards +infinity.
fn bit_increment(value: f32) -> f32 {
    if value.is_nan() || value == f32::INFINITY {
        return value;
    }
    if value == 0.0 {
        return f32::from_bits(1);
    }
    let bits = value.to_bits();
    f32::from_bits(if value > 0.0 { bits + 1 } else { bits - 1 })
}

/// `MathF.BitDecrement`: the next float towards -infinity.
fn bit_decrement(value: f32) -> f32 {
    -bit_increment(-value)
}

/// Rebuilds a cube so every triangle owns its own three corners, each moved by `perturbed`.
fn split_corners(cube: &Mesh, mut perturbed: impl FnMut(Vector3Float) -> Vector3Float) -> Mesh {
    let mut split = Mesh::new();
    for face in cube.faces.iter() {
        let start = split.vertices.len() as i32;
        for vertex in [face.v0, face.v1, face.v2] {
            let position = perturbed(cube.vertices[vertex as usize]);
            split.vertices.push(position);
        }
        let face = Face::new(start, start + 1, start + 2, &split.vertices);
        split.faces.push(face);
    }
    split
}

/// A cube whose every triangle carries its own copy of its corners, each nudged by up to
/// `jitter`.
fn cube_with_split_seams(size: f64, jitter: f32) -> Mesh {
    let cube = create_cube(size, size, size);
    // Deterministic, so a failure here is always the same failure.
    let mut random = SeededRandom::new(12345);
    let mut nudge = move || ((random.next_double() - 0.5) * 2.0 * f64::from(jitter)) as f32;
    split_corners(&cube, |p| {
        let x = p.x + nudge();
        let y = p.y + nudge();
        let z = p.z + nudge();
        Vector3Float::new_f32(x, y, z)
    })
}

/// A cube at `at` whose every triangle carries its own copy of its corners, each corner nudged
/// `ulps` steps along the float grid.
fn cube_with_seams_split_by_float_spacing(size: f64, at: Vector3, ulps: i32) -> Mesh {
    let mut cube = create_cube(size, size, size);
    cube.transform(Matrix4X4::create_translation_vector(at));
    let mut random = SeededRandom::new(12345);
    let mut nudged = move |mut value: f32| {
        match random.next(3) {
            0 => {
                for _ in 0..ulps {
                    value = bit_decrement(value);
                }
            }
            1 => {
                for _ in 0..ulps {
                    value = bit_increment(value);
                }
            }
            _ => {}
        }
        value
    };
    split_corners(&cube, |p| {
        let x = nudged(p.x);
        let y = nudged(p.y);
        let z = nudged(p.z);
        Vector3Float::new_f32(x, y, z)
    })
}
