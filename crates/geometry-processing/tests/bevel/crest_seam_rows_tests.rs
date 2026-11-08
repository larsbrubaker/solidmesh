//! The seam rows - each strip's row ending on the other round - checked on the narrow box against
//! the parallel case's closed form, on two edges converging at 60 degrees across one face against
//! hand values, at the face's exact width (the snap), and at the two true limits.

use solidmesh::bevel::level_line::crest_seam_rows::{
    seam_rows, RowEnd, SeamRowsPair, SeamRowsRefusal, SeamStrip,
};
use solidmesh_math::Vector3;

const EPSILON: f64 = 1e-6;

fn v(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}

fn strip(start: Vector3, end: Vector3, other_normal: Vector3, rows: &[f64]) -> SeamStrip {
    SeamStrip {
        start,
        end,
        other_normal,
        row_stations: rows.to_vec(),
    }
}

fn pair<'a>(k: &'a SeamStrip, j: &'a SeamStrip, n: Vector3, radius: f64) -> SeamRowsPair<'a> {
    SeamRowsPair {
        strip_k: k,
        strip_j: j,
        face_normal: n,
        radius,
        seam_epsilon: EPSILON,
        tolerance: 1e-9,
    }
}

/// The box's top edges: +y from x = -5 to 5, -y run back.
fn box_strips(j_other_normal: Vector3) -> (SeamStrip, SeamStrip) {
    (
        strip(
            v(-5.0, 2.0, 5.0),
            v(5.0, 2.0, 5.0),
            v(0.0, 1.0, 0.0),
            &[0.0, 10.0],
        ),
        strip(
            v(5.0, -2.0, 5.0),
            v(-5.0, -2.0, 5.0),
            j_other_normal,
            &[0.0, 10.0],
        ),
    )
}

const UP: Vector3 = Vector3 {
    x: 0.0,
    y: 0.0,
    z: 1.0,
};

/// The parallel case's closed form, the oracle: the seam halfway between the two face rails
/// (y = 0), each crest `depth = r - sqrt(r^2 - (D / 2)^2)` below the face, D = 2 the setbacks'
/// overlap across the 4 mm face - the same operations in the same order, so the same bits.
#[test]
fn on_the_box_the_row_ends_are_the_parallel_closed_form() {
    let (k, j) = box_strips(v(0.0, -1.0, 0.0));
    let rows = seam_rows(&pair(&k, &j, UP, 3.0)).expect("the rounds overlap");
    let (r, half) = (3.0_f64, (3.0 + 3.0 - 4.0) / 2.0);
    let depth = r - (r * r - half * half).sqrt();
    let mut ours = rows.vertices.clone();
    ours.sort_by(|a, b| a.x.total_cmp(&b.x));
    assert_eq!(ours, [v(-5.0, 0.0, 5.0 - depth), v(5.0, 0.0, 5.0 - depth)]);
    assert_eq!(rows.snapped, [false, false]);

    // Both strips' rows at one x end on the one vertex there.
    let at_x = |x: f64| {
        RowEnd::Seam(
            rows.vertices
                .iter()
                .position(|p| p.x == x)
                .expect("a vertex at x"),
        )
    };
    assert_eq!(rows.ends_k, [at_x(-5.0), at_x(5.0)]);
    assert_eq!(rows.ends_j, [at_x(5.0), at_x(-5.0)]);
}

/// The distance from `p` to the line through `a` along unit `d`.
fn to_line(p: Vector3, a: Vector3, d: Vector3) -> f64 {
    let offset = p - a;
    (offset - d * offset.dot(d)).length()
}

/// Two 90 degree edges on the face z = 0 converging at 60 degrees (at +-30 degrees about x, from
/// 2 to 10 along each), rounded at 3: the seam is the plane y = 0. A row at t along its edge meets
/// it sigma = t tan 30 in from the edge, so it ends on the seam while t tan 30 < 3, at
/// x = 2t / sqrt 3, z = -(3 - sqrt(9 - delta^2)) with delta = 3 - t / sqrt 3.
#[test]
fn edges_meeting_at_60_degrees_end_their_rows_on_the_bisecting_plane() {
    let (c, s) = (3.0_f64.sqrt() / 2.0, 0.5);
    let k = strip(
        v(2.0 * c, 2.0 * s, 0.0),
        v(10.0 * c, 10.0 * s, 0.0),
        v(-s, c, 0.0),
        &[0.0, 2.0, 4.0],
    );
    let j = strip(
        v(2.0 * c, -2.0 * s, 0.0),
        v(10.0 * c, -10.0 * s, 0.0),
        v(-s, -c, 0.0),
        &[0.0, 2.0, 4.0],
    );
    let rows = seam_rows(&pair(&k, &j, UP, 3.0)).expect("the rounds overlap");

    // Hand values: t = 2 and t = 4 end on the seam; t = 6 (sigma 3.46) keeps its face rail.
    let hand = |t: f64| {
        let delta = 3.0 - t / 3.0_f64.sqrt();
        v(
            2.0 * t / 3.0_f64.sqrt(),
            0.0,
            -(3.0 - (9.0 - delta * delta).sqrt()),
        )
    };
    assert_eq!(rows.vertices.len(), 2, "{:?}", rows.vertices);
    for (got, want) in rows.vertices.iter().zip([hand(2.0), hand(4.0)]) {
        assert!((*got - want).length() < 1e-12, "{got:?} against {want:?}");
    }
    assert!((rows.vertices[0] - v(2.309401, 0.0, -0.634652)).length() < 1e-6);
    assert!((rows.vertices[1] - v(4.618802, 0.0, -0.080570)).length() < 1e-6);
    let ends = [RowEnd::Seam(0), RowEnd::Seam(1), RowEnd::FaceRail];
    assert_eq!(rows.ends_k, ends);
    assert_eq!(rows.ends_j, ends);

    // Each end is r from both axes: the face rails' lines 3 below the face.
    let axis_k = (
        v(2.0 * c, 2.0 * s, 0.0) + v(s, -c, 0.0) * 3.0 - UP * 3.0,
        v(c, s, 0.0),
    );
    let axis_j = (
        v(2.0 * c, -2.0 * s, 0.0) + v(s, c, 0.0) * 3.0 - UP * 3.0,
        v(c, -s, 0.0),
    );
    for p in &rows.vertices {
        assert!((to_line(*p, axis_k.0, axis_k.1) - 3.0).abs() < 1e-12);
        assert!((to_line(*p, axis_j.0, axis_j.1) - 3.0).abs() < 1e-12);
        assert_eq!(p.y, 0.0);
    }
}

/// At 2 the box's setbacks exactly cover the face: every row ends on its face rail, which is the
/// seam's vertex there, shared by both strips, and no chord is made between them.
#[test]
fn at_the_face_s_exact_width_the_rows_snap_to_their_face_rails() {
    let (k, j) = box_strips(v(0.0, -1.0, 0.0));
    let rows = seam_rows(&pair(&k, &j, UP, 2.0)).expect("the bands meet on the face");
    assert_eq!(rows.vertices.len(), 2);
    assert_eq!(rows.snapped, [true, true]);
    for p in &rows.vertices {
        assert_eq!((p.y, p.z), (0.0, 5.0));
    }
    assert!(rows.ends_k.iter().all(|end| matches!(end, RowEnd::Seam(_))));
    assert!(rows.ends_j.iter().all(|end| matches!(end, RowEnd::Seam(_))));
}

#[test]
fn the_true_limits_refuse_by_name() {
    // Short of the face's width the rounds do not overlap.
    let (k, j) = box_strips(v(0.0, -1.0, 0.0));
    assert_eq!(
        seam_rows(&pair(&k, &j, UP, 1.999)),
        Err(SeamRowsRefusal::BandsDoNotOverlap)
    );

    // A knife-sharp -y edge (turning 150 degrees) sets its round back 3 tan 75 = 11.2, past the
    // whole face: the seam would run beyond the +y round's arc.
    let (k, j) = box_strips(v(0.0, -0.5, -(3.0_f64.sqrt()) / 2.0));
    assert_eq!(
        seam_rows(&pair(&k, &j, UP, 3.0)),
        Err(SeamRowsRefusal::OverlapPastTheRounds)
    );

    // An edge off the face's plane.
    let low = strip(
        v(5.0, -2.0, 4.0),
        v(-5.0, -2.0, 4.0),
        v(0.0, -1.0, 0.0),
        &[0.0],
    );
    assert_eq!(
        seam_rows(&pair(&k, &low, UP, 3.0)),
        Err(SeamRowsRefusal::NotOnTheFace)
    );
}
