//! A cube (20 across) filleted on every edge at 2 with 8 segments, by the level line - each corner
//! of three equal convex fillets is the rolling ball's octant, meeting each fillet tangentially
//! along its end circle. The kernel twin of the app's
//! `BevelCornerPatchTests.TheLevelLinesCornerPatchMeetsEachFilletTangentlyAlongTheirSharedCircle`.

use geometry_processing::bevel::level_line::corner_cap_hole_fill::{
    covers_the_ball_once, on_the_ball,
};
use geometry_processing::bevel::level_line::{
    gap_fill, BandProfile, CornerCapLastRow, GapFillRefusal, GapFillResult, HoleFillFailure,
    HoleFillResult, HoleFillTriangle, LevelLineMesh,
};
use vector_math::{math_helper, Matrix4X4, Vector3};

use crate::hole_fill_corner_tests::{build, every_edge, sign};
use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::cube_solid;
use crate::several_chains_fixtures::FOLD_DEGREES;

const HALF: f64 = 10.0;

const RADIUS: f64 = 2.0;

const SEGMENTS: i32 = 8;

/// Every corner fill vertex is on the ball.
#[test]
fn every_corner_fill_vertex_is_on_the_ball() {
    assert_every_corner_is_on_the_ball(None);
}

/// The same cube turned arbitrarily, carried far
/// from the origin and rounded to float, as a part's vertices are.
#[test]
fn every_corner_of_a_far_turned_float_cube_is_on_the_ball() {
    let motion = Matrix4X4::create_rotation(Vector3::new(0.3, -0.7, 1.1))
        * Matrix4X4::create_translation(1000.5, -2000.25, 1500.125);
    assert_every_corner_is_on_the_ball(Some(motion));
}

/// A hole whose rows are not one ball - two arcs of
/// unequal radius about one centre - is handed back as the filler laid it; an octant of three equal
/// arcs is placed on the ball.
#[test]
fn only_a_hole_rimmed_by_one_ball_is_placed_on_it() {
    let s = 2f64.sqrt();
    let positions = vec![
        Vector3::new(2.0, 0.0, 0.0),
        Vector3::new(s, s, 0.0),
        Vector3::new(0.0, 2.0, 0.0),
        Vector3::new(0.0, s, s),
        Vector3::new(0.0, 0.0, 2.0),
        Vector3::new(s, 0.0, s),
        Vector3::new(0.0, 1.5, 0.0),
        Vector3::new(0.0, 1.5 / s, 1.5 / s),
        Vector3::new(0.0, 0.0, 1.5),
    ];
    let mesh = LevelLineMesh::from_triangles(&positions, &[], None).expect("a mesh");
    let hole: Vec<Vector3> = positions.iter().take(6).copied().collect();
    let triangles: Vec<HoleFillTriangle> = (0..6)
        .map(|i| HoleFillTriangle::new(i, (i + 1) % 6, 6))
        .collect();
    let mut vertices = hole.clone();
    vertices.push(Vector3::new(1.0, 1.0, 1.0));
    let filled = HoleFillResult::new(HoleFillFailure::None, vertices, triangles);

    let row = |arc: Vec<i32>, chain_point: Vector3, concave: bool| {
        CornerCapLastRow::new(arc, chain_point, &mesh, concave, Vector3::ZERO)
    };
    let ball = [
        row(vec![0, 1, 2], Vector3::new(2.0, 2.0, 0.0), false),
        row(vec![2, 3, 4], Vector3::new(0.0, 2.0, 2.0), false),
        row(vec![4, 5, 0], Vector3::new(2.0, 0.0, 2.0), false),
    ];
    let placed = on_the_ball(filled.clone(), &hole, &ball);
    assert!((placed.vertices[6].length() - 2.0).abs() <= 1e-12);

    // Handed back untouched.
    let unequal = [
        ball[0].clone(),
        row(vec![6, 7, 8], Vector3::new(0.0, 1.5, 1.5), false),
    ];
    assert_eq!(on_the_ball(filled.clone(), &hole, &unequal), filled);

    let concave = [
        ball[0].clone(),
        ball[1].clone(),
        row(vec![4, 5, 0], Vector3::new(2.0, 0.0, 2.0), true),
    ];
    assert_eq!(on_the_ball(filled.clone(), &hole, &concave), filled);
}

/// A patch with one triangle turned against the loop - a
/// fold - does not cover the ball once, so it is never placed on the ball; the same patch unfolded
/// does.
#[test]
fn a_folded_patch_does_not_cover_the_ball_once() {
    let vertices = vec![
        Vector3::new(2.0, 0.0, 0.0),
        Vector3::new(0.0, 2.0, 0.0),
        Vector3::new(0.0, 0.0, 2.0),
        Vector3::new(1.0, 1.0, 1.0),
    ];
    let fan = vec![
        HoleFillTriangle::new(0, 1, 3),
        HoleFillTriangle::new(1, 2, 3),
        HoleFillTriangle::new(2, 0, 3),
    ];
    assert!(covers_the_ball_once(Vector3::ZERO, &vertices, 3, &fan));

    let folded = vec![
        HoleFillTriangle::new(0, 1, 3),
        HoleFillTriangle::new(2, 1, 3),
        HoleFillTriangle::new(2, 0, 3),
    ];
    assert!(!covers_the_ball_once(Vector3::ZERO, &vertices, 3, &folded));
}

fn assert_every_corner_is_on_the_ball(motion: Option<Matrix4X4>) {
    let filled = fill_every_edge(motion);
    assert_eq!(filled.refusal, GapFillRefusal::None);

    let back = motion.map_or(Matrix4X4::IDENTITY, |m| m.inverted());
    let built = filled.mesh.as_ref().unwrap().borrow();
    let positions: Vec<Vector3> = (0..built.vertex_count())
        .map(|v| built.position(v).transform(back))
        .collect();
    let indices: Vec<(i32, i32, i32)> = (0..built.triangle_count())
        .filter(|&t| !built.is_removed(t))
        .map(|t| {
            let tri = built.triangles()[t as usize];
            (tri.a, tri.b, tri.c)
        })
        .collect();
    let mesh = LevelLineMesh::from_triangles(&positions, &indices, None).expect("a mesh");
    let sag = RADIUS * (1.0 - (std::f64::consts::PI / 2.0 / SEGMENTS as f64 / 2.0).cos());
    let mut worst: f64 = 0.0;
    let mut counted = 0;
    for &v in &filled.added_vertices {
        let p = mesh.position(v);
        let centre = inset_point_of(p);
        let off = p - centre;

        // Only the vertices in a corner's octant of its ball (outside the inset box on all three
        // axes).
        if p.x.abs() < HALF - RADIUS - 1e-9
            || p.y.abs() < HALF - RADIUS - 1e-9
            || p.z.abs() < HALF - RADIUS - 1e-9
        {
            continue;
        }

        counted += 1;
        worst = worst.max((off.length() - RADIUS).abs());
    }

    let (lean, seam_faces) = seam_lean(&mesh);
    let allowed = 90.0 / SEGMENTS as f64 / 2.0;
    println!(
        "{counted} corner vertices, worst {worst:.4} off the ball against {:.4}; {seam_faces} faces at a seam, worst lean off their own surface's normal {lean:.2} degrees against {allowed:.2}",
        sag * 1.5
    );
    assert!(counted > 0);
    assert!(
        worst < sag * 1.5,
        "three equal convex fillets meet in the rolling ball's octant; the corner fill is that sphere, not a fair patch near it"
    );
    assert!(
        lean < allowed,
        "the patch meets each fillet tangentially along its end circle, so no face at a seam turns further off its own surface than half an angular step"
    );
}

fn inset_point_of(p: Vector3) -> Vector3 {
    let inset = HALF - RADIUS;
    Vector3::new(sign(p.x) * inset, sign(p.y) * inset, sign(p.z) * inset)
}

/// The faces within a chord of a seam circle and the worst lean of their normals off
/// the true surface's normal at their centroid - the ball's in its octant, the fillet cylinder's
/// outside it.
fn seam_lean(mesh: &LevelLineMesh) -> (f64, i32) {
    let near = RADIUS * (std::f64::consts::PI / 2.0) / SEGMENTS as f64;
    let mut worst_lean: f64 = 0.0;
    let mut faces = 0;
    let inset = HALF - RADIUS;
    for x in [-1.0, 1.0] {
        for y in [-1.0, 1.0] {
            for z in [-1.0, 1.0] {
                let centre = Vector3::new(x * inset, y * inset, z * inset);
                for t in 0..mesh.triangle_count() {
                    if mesh.is_removed(t) {
                        continue;
                    }

                    let tri = mesh.triangles()[t as usize];
                    let p0 = mesh.position(tri.a);
                    let p1 = mesh.position(tri.b);
                    let p2 = mesh.position(tri.c);
                    let centroid = (p0 + p1 + p2) / 3.0;
                    if seam_distance(centroid, centre) > near {
                        continue;
                    }

                    faces += 1;
                    let normal = (p1 - p0).cross(p2 - p0);
                    if normal.length_squared() <= 0.0 || normal.length_squared().is_nan() {
                        continue;
                    }

                    let lean = math_helper::radians_to_degrees(
                        normal
                            .get_normal()
                            .dot(surface_normal(centroid, centre))
                            .clamp(-1.0, 1.0)
                            .acos(),
                    );
                    worst_lean = worst_lean.max(lean);
                }
            }
        }
    }

    (worst_lean, faces)
}

/// The true surface's outward normal beside a corner - the ball's radial
/// direction inside its octant, else the fillet cylinder's.
fn surface_normal(position: Vector3, centre: Vector3) -> Vector3 {
    let offset = position - centre;
    let x = if offset.x * sign(centre.x) < 0.0 {
        0.0
    } else {
        offset.x
    };
    let y = if offset.y * sign(centre.y) < 0.0 {
        0.0
    } else {
        offset.y
    };
    let z = if offset.z * sign(centre.z) < 0.0 {
        0.0
    } else {
        offset.z
    };
    Vector3::new(x, y, z).get_normal()
}

fn seam_distance(position: Vector3, centre: Vector3) -> f64 {
    let offset = position - centre;
    let mut closest = f64::MAX;
    for axis in [
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
    ] {
        let along = offset.dot(axis);
        let across = (offset - (axis * along)).length();
        closest = closest.min(((along * along) + ((across - RADIUS) * (across - RADIUS))).sqrt());
    }

    closest
}

fn fill_every_edge(motion: Option<Matrix4X4>) -> GapFillResult {
    let (mut positions, indices) = cube_solid(HALF);
    if let Some(motion) = motion {
        positions = positions
            .iter()
            .map(|p| p.transform(motion))
            .map(|p| Vector3::new(p.x as f32 as f64, p.y as f32 as f64, p.z as f32 as f64))
            .collect();
    }

    let chains = every_edge();
    let open = vec![false; chains.len()];
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    let built = build(
        &positions,
        &indices,
        &chains,
        &open,
        &feature,
        RADIUS,
        BandProfile::Fillet,
    );
    assert!(built.built(), "every edge refused: {:?}", built.refusal);

    gap_fill::fill(&built, BandProfile::Fillet, SEGMENTS).expect("the fill")
}
