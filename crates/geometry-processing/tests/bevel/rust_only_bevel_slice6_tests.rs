//! Rust-only checks of the sixth bevel slice (`BevelRoundFaces`, `ConvexCellBound`, `RoomSource`,
//! `SurfaceMixedCornerSpine`, `SurfaceRegionFaceGrid`). The C# test classes that exercise them
//! read whole bevels (`SurfaceMixedCornerDualOnASquareCornerTests` reads the spine through
//! `SurfaceMixedCorner`), so they land with the core; these pin each piece alone.
//! `FlatHoleFill` has its own ported class (`flat_hole_fill_tests`).

use std::collections::HashSet;
use std::f64::consts::{FRAC_PI_2, PI};

use geometry_processing::bevel::level_line::{
    convex_cell_bound, FieldSample, RoomReadingKind, RoomSource,
};
use geometry_processing::bevel::surface::{
    nearest_on_the_triangle, SurfaceLine, SurfaceMixedCornerSpine, SurfaceRegionFaceGrid,
};
use geometry_processing::bevel::{BevelRoundDescriptor, BevelRoundFaces};
use geometry_processing::topology::FaceGroup;
use polygon_mesh::Mesh;
use vector_math::Vector3;

// ---------------------------------------------------------------- BevelRoundFaces

/// A quarter round of radius 1 about the z axis, from +x to +y, 1 tall.
fn quarter_round() -> BevelRoundDescriptor {
    BevelRoundDescriptor::with_span(
        Vector3::ZERO,
        Vector3::UNIT_Z,
        1.0,
        Vector3::UNIT_X,
        0.0,
        FRAC_PI_2,
        0.0,
        1.0,
        8,
    )
}

fn mesh_of(triangles: &[[Vector3; 3]]) -> Mesh {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for triangle in triangles {
        for &corner in triangle {
            indices.push(vertices.len() as i32);
            vertices.push(corner);
        }
    }

    Mesh::from_vector3_list_and_indices(&vertices, &indices)
}

#[test]
fn rust_only_round_faces_label_a_facet_a_tangent_plane_and_nothing_else() {
    let chord = Vector3::new((PI / 8.0).cos(), (PI / 8.0).sin(), 0.0);
    let part = mesh_of(&[
        // a facet of the round: every corner on the radius
        [
            Vector3::new(1.0, 0.0, 0.0),
            chord,
            Vector3::new(1.0, 0.0, 1.0),
        ],
        // the flat the round runs out onto at +x, facing +x, one corner on the tangent line
        [
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 1.0),
            Vector3::new(1.0, -1.0, 0.0),
        ],
        // far off every round
        [
            Vector3::new(50.0, 50.0, 50.0),
            Vector3::new(51.0, 50.0, 50.0),
            Vector3::new(50.0, 51.0, 50.0),
        ],
    ]);

    let faces = BevelRoundFaces::label(&part, &[quarter_round()], 1.0, true);

    assert!(faces.read_from_the_source_mesh());
    assert_eq!(faces.count(), 1);
    assert_eq!(faces.tangent_plane_count(), 1);
    assert!(matches!(faces.try_read_face(0), Some((_, false))));
    assert!(matches!(faces.try_read_face(1), Some((_, true))));
    assert!(faces.try_read_face(2).is_none());
}

#[test]
fn rust_only_round_faces_with_no_rounds_or_a_chamfer_label_nothing() {
    let part = mesh_of(&[[Vector3::ZERO, Vector3::UNIT_X, Vector3::UNIT_Y]]);

    let none = BevelRoundFaces::label(&part, &[], 1.0, false);
    assert_eq!(none.count(), 0);
    assert!(!none.read_from_the_source_mesh());

    let chamfer = BevelRoundFaces::create(&part, &[], &[], 1.0, 8, false);
    assert_eq!(chamfer.count() + chamfer.tangent_plane_count(), 0);
}

#[test]
fn rust_only_adding_the_same_round_twice_merges_its_axial_extent() {
    let mut rounds = Vec::new();
    BevelRoundFaces::add(&mut rounds, quarter_round(), 1e-3);
    let higher = BevelRoundDescriptor::with_span(
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::UNIT_Z,
        1.0,
        Vector3::UNIT_X,
        0.0,
        FRAC_PI_2,
        0.0,
        2.0,
        8,
    );
    BevelRoundFaces::add(&mut rounds, higher, 1e-3);

    assert_eq!(rounds.len(), 1);
    assert_eq!(rounds[0].axial_from, 0.0);
    assert_eq!(rounds[0].axial_to, 3.0);
}

// ---------------------------------------------------------------- ConvexCellBound

/// The sample a field of one chain point `c` with setback `s` reads at `v`.
fn sample(c: Vector3, s: f64, v: Vector3) -> FieldSample {
    let distance = (v - c).length();
    FieldSample {
        distance,
        value: distance - s,
        nearest: c,
        ..Default::default()
    }
}

#[test]
fn rust_only_the_convex_cell_bound_is_the_farthest_corner_less_the_setback() {
    let c = Vector3::new(0.0, 0.0, 5.0);
    let (v0, v1, v2) = (
        Vector3::ZERO,
        Vector3::new(3.0, 0.0, 0.0),
        Vector3::new(0.0, 4.0, 0.0),
    );
    let (s0, s1, s2) = (sample(c, 1.0, v0), sample(c, 1.0, v1), sample(c, 1.0, v2));

    let farthest = (v2 - c).length();
    assert_eq!(
        convex_cell_bound::over_triangle(&s0, &s1, &s2, v0, v1, v2, None),
        farthest - 1.0
    );
    assert_eq!(
        convex_cell_bound::over_segment(&s0, &s1, v0, v1, None),
        (v1 - c).length() - 1.0
    );

    // and it bounds the field: no point of the cell reads above it
    for (u, w) in [(0.2, 0.3), (0.5, 0.5), (0.9, 0.05)] {
        let q = v0 + ((v1 - v0) * u) + ((v2 - v0) * w);
        assert!((q - c).length() - 1.0 <= farthest - 1.0);
    }
}

#[test]
fn rust_only_a_clipped_sample_gives_no_term_and_end_planes_raise_one() {
    let c = Vector3::ZERO;
    let (v0, v1) = (Vector3::UNIT_X, Vector3::UNIT_Y);
    let clipped = FieldSample {
        clipped: true,
        ..sample(c, 0.5, v0)
    };

    assert_eq!(
        convex_cell_bound::over_segment(&clipped, &clipped, v0, v1, None),
        f64::INFINITY
    );

    let plane = |_chain: i32, at: Vector3| at.x * 10.0;
    let plain = sample(c, 0.5, v1);
    assert_eq!(
        convex_cell_bound::over_segment(&plain, &plain, v0, v1, Some(&plane)),
        10.0
    );
}

// ---------------------------------------------------------------- RoomSource

#[test]
fn rust_only_room_sources_name_their_reading() {
    assert_eq!(RoomSource::nothing().kind, RoomReadingKind::None);
    assert_eq!(RoomSource::nothing().chain, -1);
    assert!(!RoomSource::foreign(2).is_a_growing_crease());

    let growing = RoomSource::growing_crease(3, Vector3::UNIT_Z);
    assert_eq!(growing.kind, RoomReadingKind::Foreign);
    assert!(growing.is_a_growing_crease());
    assert_ne!(growing, RoomSource::foreign(3));

    assert_eq!(RoomSource::self_room(4).kind, RoomReadingKind::SelfRoom);
    let pair = RoomSource::chain_to_chain(1, 5);
    assert_eq!(
        (pair.kind, pair.chain, pair.other_chain),
        (RoomReadingKind::ChainToChain, 1, 5)
    );
}

// ---------------------------------------------------------------- SurfaceMixedCornerSpine

/// The L prism's square mixed corner (`SurfaceMixedCornerDualOnASquareCornerTests`' frame): cap
/// `z = 10`, shared wall `x = 10`, void wall `y = 10`, the bead's axis `(11, 11, z)`, r = 1.
fn square_corner_spine(outside: bool) -> Option<SurfaceMixedCornerSpine> {
    SurfaceMixedCornerSpine::try_create(
        SurfaceLine::new(Vector3::new(11.0, 11.0, 0.0), Vector3::UNIT_Z),
        Vector3::new(11.0, 11.0, 10.0),
        Vector3::UNIT_Z,
        1.0,
        outside,
        Vector3::new(-1.0, 0.0, 0.0),
        Vector3::new(0.0, -1.0, 0.0),
        1.0,
        FRAC_PI_2,
        Vector3::new(0.0, 0.0, -1.0),
        0.5,
    )
}

#[test]
fn rust_only_the_square_corners_cap_riding_spine_is_the_torus_circle() {
    let spine = square_corner_spine(false).expect("a square corner reads");

    // the circle of radius 2 about (11, 11, 9) in the plane z = 9, from (9, 11, 9) to (11, 9, 9)
    assert!(spine
        .centre_at(0.0)
        .equals_within(Vector3::new(9.0, 11.0, 9.0), 1e-12));
    assert!(spine
        .centre_at(FRAC_PI_2)
        .equals_within(Vector3::new(11.0, 9.0, 9.0), 1e-12));
    assert_eq!(spine.slide_at(0.7), -1.0);
    assert_eq!(spine.cap_lean_degrees(), 0.0);
    assert!(spine
        .t_contact_at(0.0)
        .equals_within(Vector3::new(9.0, 11.0, 10.0), 1e-12));
    assert!(spine
        .bead_contact_at(0.0)
        .equals_within(Vector3::new(10.0, 11.0, 9.0), 1e-12));
    assert_eq!(
        spine.along_the_bead_at(
            0.0,
            Vector3::new(10.0, 10.0, 10.0),
            Vector3::new(0.0, 0.0, -1.0)
        ),
        1.0
    );

    // the tube's ring stands where the two cylinders touch, square to the bead and facing down it
    let ring = spine.tube_ring_plane();
    assert!(ring
        .point
        .equals_within(Vector3::new(10.0, 11.0, 9.0), 1e-12));
    assert!(ring
        .inward
        .equals_within(Vector3::new(0.0, 0.0, -1.0), 1e-12));

    // the contact arms are the contact curve, station by station
    let arms = spine.the_contact_arms();
    for turn in [0.0, 0.3, 1.1, FRAC_PI_2] {
        let by_arms = arms.middle + (arms.cosine * f64::cos(turn)) + (arms.sine * f64::sin(turn));
        assert!(by_arms.equals_within(spine.t_contact_at(turn), 1e-12));
    }
}

#[test]
fn rust_only_a_void_ball_keeps_its_radius_outside_the_cap_and_an_axis_in_the_cap_reads_nothing() {
    let dual = square_corner_spine(true).expect("the dual reads by the same algebra");
    assert_eq!(dual.slide_at(0.0), 1.0);
    assert!(dual
        .t_contact_at(0.0)
        .equals_within(Vector3::new(9.0, 11.0, 10.0), 1e-12));

    let in_the_cap = SurfaceMixedCornerSpine::try_create(
        SurfaceLine::new(Vector3::ZERO, Vector3::UNIT_X),
        Vector3::ZERO,
        Vector3::UNIT_Z,
        0.0,
        false,
        Vector3::UNIT_Y,
        Vector3::UNIT_Z,
        1.0,
        FRAC_PI_2,
        Vector3::UNIT_X,
        0.5,
    );
    assert!(in_the_cap.is_none());
}

// ---------------------------------------------------------------- SurfaceRegionFaceGrid

/// A gently waved 20 x 20 sheet, one region at the default curve tolerance.
fn waved_sheet() -> (Mesh, FaceGroup) {
    let n = 20;
    let mut vertices = Vec::new();
    for i in 0..=n {
        for j in 0..=n {
            let (x, y) = (i as f64 * 0.5, j as f64 * 0.5);
            vertices.push(Vector3::new(x, y, 0.3 * (x * 0.7).sin() * (y * 0.5).cos()));
        }
    }

    let at = |i: i32, j: i32| i * (n + 1) + j;
    let mut faces = Vec::new();
    for i in 0..n {
        for j in 0..n {
            faces.extend([at(i, j), at(i + 1, j), at(i + 1, j + 1)]);
            faces.extend([at(i, j), at(i + 1, j + 1), at(i, j + 1)]);
        }
    }

    let mesh = Mesh::from_vector3_list_and_indices(&vertices, &faces);
    let region = FaceGroup::create(&mesh, 0).expect("face 0 is a face");
    (mesh, region)
}

fn corners_of(mesh: &Mesh, face: i32) -> [Vector3; 3] {
    let f = &mesh.faces[face as usize];
    [f.v0, f.v1, f.v2].map(|v| Vector3::from_vector3_float(mesh.vertices[v as usize]))
}

#[test]
fn rust_only_the_face_grid_answers_nearest_exactly_as_the_whole_walk() {
    let (mesh, region) = waved_sheet();
    assert_eq!(region.faces().len(), 800);
    let grid = SurfaceRegionFaceGrid::new(&mesh, &region);

    for at in [
        Vector3::new(3.3, 4.1, 0.9),
        Vector3::new(0.0, 0.0, -0.5),
        Vector3::new(9.9, 2.2, 0.05),
        Vector3::new(5.0, 5.0, 3.0),
    ] {
        let within = 1.0;
        let walked = region
            .faces()
            .iter()
            .map(|&face| {
                let [a, b, c] = corners_of(&mesh, face);
                (at - nearest_on_the_triangle(at, a, b, c)).length()
            })
            .fold(f64::MAX, f64::min);

        let gridded = grid.nearest_to(at, within);
        if walked <= within {
            assert_eq!(gridded, walked, "at {at:?}");
        } else {
            assert!(gridded >= walked, "at {at:?}");
        }
    }
}

#[test]
fn rust_only_the_face_grid_hands_back_every_face_a_line_crosses() {
    let (mesh, region) = waved_sheet();
    let grid = SurfaceRegionFaceGrid::new(&mesh, &region);

    let through = Vector3::new(1.3, 0.2, 0.1);
    let along = Vector3::new(0.8, 0.55, 0.02).get_normal();

    let mut handed = HashSet::new();
    grid.each_face_the_line_could_cross(through, along, |face, a, b, c| {
        assert_eq!([a, b, c], corners_of(&mesh, face));
        handed.insert(face);
    });

    let mut crossed = 0;
    for &face in region.faces() {
        let [a, b, c] = corners_of(&mesh, face);
        if line_crosses(through, along, a, b, c) {
            crossed += 1;
            assert!(
                handed.contains(&face),
                "face {face} is crossed but not handed back"
            );
        }
    }

    assert!(crossed > 0);
    assert!(handed.len() < region.faces().len());
}

/// Moller-Trumbore on the infinite line.
fn line_crosses(through: Vector3, along: Vector3, a: Vector3, b: Vector3, c: Vector3) -> bool {
    let (e1, e2) = (b - a, c - a);
    let p = along.cross(e2);
    let det = e1.dot(p);
    if det.abs() < 1e-12 {
        return false;
    }

    let t = through - a;
    let u = t.dot(p) / det;
    let q = t.cross(e1);
    let v = along.dot(q) / det;
    (0.0..=1.0).contains(&u) && v >= 0.0 && u + v <= 1.0
}

#[test]
fn rust_only_a_one_triangle_region_grid_reads_its_distance_off_the_plane() {
    let mesh = Mesh::from_vector3_list_and_indices(
        &[Vector3::ZERO, Vector3::UNIT_X, Vector3::UNIT_Y],
        &[0, 1, 2],
    );
    let region = FaceGroup::create(&mesh, 0).expect("face 0 is a face");
    let grid = SurfaceRegionFaceGrid::new(&mesh, &region);
    assert!(grid.weld_distance() > 0.0);
    assert_eq!(grid.nearest_to(Vector3::new(0.2, 0.2, 1.0), 2.0), 1.0);
}
