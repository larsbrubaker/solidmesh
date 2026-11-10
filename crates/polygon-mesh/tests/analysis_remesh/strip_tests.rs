//! Strips narrower than the deviation bound: a prism whose corner is cut by a chamfer a tenth of a
//! micrometre wide, the kind of meshing leftover exported parts carry along an edge. The caller's
//! bound says a feature that narrow need not show, so the part must come back meshed as if the
//! strip were a single sharp edge: closed, uncrossed, unfolded, every corner over the floor and
//! within the bound of the input. Left in, every vertex the remesh puts inside the strip sits on
//! both of its creases at once and the triangles it makes with them have no width.

use solidmesh_math::Vector2;
use solidmesh_mesh::processors::outline_to_mesh::extrude_contours;
use solidmesh_mesh::{
    remesh_for_analysis, AnalysisRemesh, AnalysisRemeshError, AnalysisRemeshOptions, Mesh,
};

use super::curved_parts::{mesh_of, P};
use super::surface_checks::{self_intersections, soup_of};

/// What the pin test's remesh returned before strips were merged (measured on the remesher
/// without the strip pass).
const PIN_BEFORE_STRIPS: &str = "Ok 96 faces 6b07e02397fb5c5b";

/// The prism's length along z.
const LENGTH: f64 = 29.0;

/// A 20 by 20 prism `LENGTH` long whose corner at (10, 10) is cut by a chamfer `(a0, b0)` wide at
/// z = 0 and `(a1, b1)` wide at the far end: the cut runs from `a` below the corner on the x = 10
/// side to `b` beside it on the y = 10 side.
fn chamfered_prism(a0: f64, b0: f64, a1: f64, b1: f64) -> Mesh {
    let ring = |a: f64, b: f64, z: f64| -> [P; 5] {
        [
            [-10.0, -10.0, z],
            [10.0, -10.0, z],
            [10.0, 10.0 - a, z],
            [10.0 - b, 10.0, z],
            [-10.0, 10.0, z],
        ]
    };
    let mut points = ring(a0, b0, 0.0).to_vec();
    points.extend(ring(a1, b1, LENGTH));
    let mut faces = vec![
        [0, 2, 1],
        [0, 3, 2],
        [0, 4, 3],
        [5, 6, 7],
        [5, 7, 8],
        [5, 8, 9],
    ];
    for i in 0..5 {
        let j = (i + 1) % 5;
        faces.push([i, j, j + 5]);
        faces.push([i, j + 5, i + 5]);
    }
    mesh_of(&points, &faces)
}

/// The smallest corner of any triangle, in degrees.
fn min_angle_degrees(vertices: &[P], faces: &[[i32; 3]]) -> f64 {
    let corner = |p: P, q: P, r: P| {
        let u = [q[0] - p[0], q[1] - p[1], q[2] - p[2]];
        let v = [r[0] - p[0], r[1] - p[1], r[2] - p[2]];
        let c = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        let sine = (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt();
        let cosine = u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
        sine.atan2(cosine).to_degrees()
    };
    faces
        .iter()
        .map(|f| {
            let [a, b, c] = f.map(|v| vertices[v as usize]);
            corner(a, b, c).min(corner(b, c, a)).min(corner(c, a, b))
        })
        .fold(f64::INFINITY, f64::min)
}

/// Edges whose two faces face nearly opposite ways: folded back over each other. A face with no
/// area has no facing at all, and an edge beside one counts as folded.
fn folded_edges(vertices: &[P], faces: &[[i32; 3]]) -> usize {
    use std::collections::HashMap;
    let unit = |f: &[i32; 3]| {
        let [a, b, c] = f.map(|v| vertices[v as usize]);
        let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let n = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        (l > 0.0).then(|| [n[0] / l, n[1] / l, n[2] / l])
    };
    // Counting only, so map order cannot reach the result.
    let mut around: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
    for (t, f) in faces.iter().enumerate() {
        for (a, b) in [(f[0], f[1]), (f[1], f[2]), (f[2], f[0])] {
            around.entry((a.min(b), a.max(b))).or_default().push(t);
        }
    }
    around
        .values()
        .filter(|ts| {
            ts.len() == 2
                && match (unit(&faces[ts[0]]), unit(&faces[ts[1]])) {
                    (Some(p), Some(q)) => p[0] * q[0] + p[1] * q[1] + p[2] * q[2] < -0.9,
                    _ => true,
                }
        })
        .count()
}

/// The shape a strip stands in for: a prism over `outline` (counter-clockwise, in the xy plane)
/// spanning the input's z range, with sharp corners where the input has its strip, which runs
/// along z at `strip_at` (in xy), faces `strip_normal` and is at most `slack` from that shape.
struct Ideal {
    outline: Vec<[f64; 2]>,
    strip_at: [f64; 2],
    strip_normal: P,
    slack: f64,
}

/// How far from the strip's line an output face counts as standing where the strip was.
const STRIP_REACH: f64 = 0.5;

impl Ideal {
    /// The distance from `p`, near the surface, to the ideal prism with caps at `low` and `high`.
    fn distance(&self, p: P, low: f64, high: f64) -> f64 {
        let n = self.outline.len();
        let side = (0..n)
            .map(|i| {
                let (a, b) = (self.outline[i], self.outline[(i + 1) % n]);
                let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
                let span = dx * dx + dy * dy;
                let t = (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / span).clamp(0.0, 1.0);
                let (ex, ey) = (p[0] - (a[0] + dx * t), p[1] - (a[1] + dy * t));
                (ex * ex + ey * ey).sqrt()
            })
            .fold(f64::INFINITY, f64::min);
        side.min((p[2] - low).abs()).min((p[2] - high).abs())
    }
}

/// Holds the remesh of `part` to the bound against the ideal shape, measured here on every
/// output vertex and face centroid rather than taken from the remesher's report, and checks the
/// strip is gone: no output face where the strip was faces the strip's way.
fn assert_matches_ideal(
    part: &Mesh,
    surface_vertices: &[P],
    faces: &[[i32; 3]],
    bound: f64,
    ideal: &Ideal,
    what: &str,
) {
    let (input_vertices, _) = soup_of(part);
    let low = input_vertices
        .iter()
        .map(|p| p[2])
        .fold(f64::INFINITY, f64::min);
    let high = input_vertices
        .iter()
        .map(|p| p[2])
        .fold(f64::NEG_INFINITY, f64::max);
    let mut worst = 0.0_f64;
    for f in faces {
        let [a, b, c] = f.map(|v| surface_vertices[v as usize]);
        let centroid = [
            (a[0] + b[0] + c[0]) / 3.0,
            (a[1] + b[1] + c[1]) / 3.0,
            (a[2] + b[2] + c[2]) / 3.0,
        ];
        for p in [a, b, c, centroid] {
            worst = worst.max(ideal.distance(p, low, high));
        }
        let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let n = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        let length = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        let facing = (n[0] * ideal.strip_normal[0]
            + n[1] * ideal.strip_normal[1]
            + n[2] * ideal.strip_normal[2])
            / length;
        let (dx, dy) = (
            centroid[0] - ideal.strip_at[0],
            centroid[1] - ideal.strip_at[1],
        );
        let near_strip = (dx * dx + dy * dy).sqrt() <= STRIP_REACH;
        assert!(
            !near_strip || facing < 0.99,
            "{what}: an output face still faces the strip's way: {:?}",
            [a, b, c]
        );
    }
    assert!(
        worst <= bound + ideal.slack,
        "{what}: strays {worst} from the ideal shape, past the bound {bound} and the strip's {}",
        ideal.slack
    );
}

fn assert_meshed_as_a_sharp_edge(part: &Mesh, target: f64, what: &str, ideal: &Ideal) {
    let (input_vertices, input_faces) = soup_of(part);
    assert!(
        self_intersections(&input_vertices, &input_faces).is_empty(),
        "{what}: the input itself crosses"
    );
    let bound = target / 40.0;
    let size = move |_: P| target;
    let options = AnalysisRemeshOptions::new(&size, bound);
    let result = remesh_for_analysis(part, &options)
        .unwrap_or_else(|error| panic!("{what} at target {target}: refused: {error:?}"));
    let surface = &result.surface;
    assert!(
        self_intersections(&surface.vertices, &surface.faces).is_empty(),
        "{what} at target {target}: the surface crosses itself"
    );
    assert_eq!(
        folded_edges(&surface.vertices, &surface.faces),
        0,
        "{what} at target {target}: the surface folds"
    );
    let min_angle = min_angle_degrees(&surface.vertices, &surface.faces);
    assert!(
        min_angle >= options.min_angle_floor_degrees,
        "{what} at target {target}: a corner of {min_angle} degrees"
    );
    assert!(
        result.report.sampled_deviation <= bound,
        "{what} at target {target}: strays {} past the bound {bound}",
        result.report.sampled_deviation
    );
    assert_matches_ideal(part, &surface.vertices, &surface.faces, bound, ideal, what);
}

/// The ideal square prism a chamfer of `cut` (its larger leg) at the (10, 10) corner stands in
/// for, the chamfer running from `a` below the corner to `b` beside it.
fn square_ideal(a: f64, b: f64, cut: f64) -> Ideal {
    let length = (a * a + b * b).sqrt();
    Ideal {
        outline: vec![[-10.0, -10.0], [10.0, -10.0], [10.0, 10.0], [-10.0, 10.0]],
        strip_at: [10.0, 10.0],
        strip_normal: [a / length, b / length, 0.0],
        slack: cut,
    }
}

#[test]
fn a_tapered_chamfer_narrower_than_the_bound_meshes_as_a_sharp_edge() {
    let part = chamfered_prism(1e-4, 0.8e-4, 1e-5, 1.5e-5);
    for target in [0.7, 1.06, 1.5] {
        let ideal = square_ideal(1e-4, 0.8e-4, 1e-4);
        assert_meshed_as_a_sharp_edge(&part, target, "tapered chamfer", &ideal);
    }
}

#[test]
fn an_even_chamfer_narrower_than_the_bound_meshes_as_a_sharp_edge() {
    let part = chamfered_prism(2e-4, 1e-4, 2e-4, 1e-4);
    for target in [0.7, 1.06, 1.5] {
        let ideal = square_ideal(2e-4, 1e-4, 2e-4);
        assert_meshed_as_a_sharp_edge(&part, target, "even chamfer", &ideal);
    }
}

#[test]
fn an_extruded_symmetric_chamfer_meshes_as_a_sharp_edge() {
    for cut in [1e-4, 3e-4, 1e-3] {
        let outline = vec![
            Vector2::new(-10.0, -10.0),
            Vector2::new(10.0, -10.0),
            Vector2::new(10.0, 10.0 - cut),
            Vector2::new(10.0 - cut, 10.0),
            Vector2::new(-10.0, 10.0),
        ];
        let part = extrude_contours(&[outline], 30.0).expect("extruded the chamfered square");
        for target in [0.5, 1.0, 2.0] {
            let ideal = square_ideal(cut, cut, cut);
            assert_meshed_as_a_sharp_edge(&part, target, "extruded chamfer", &ideal);
        }
    }
}

/// A step half a micrometre high where a sloping flat meets a level one: the riser faces away
/// from both, so once merged into the corner no surface near it faces its way, and it must be
/// measured against the nearest surface rather than read as a distance across the part.
#[test]
fn a_step_riser_narrower_than_the_bound_meshes_as_a_sharp_edge() {
    let step = 5e-4;
    let outline = vec![
        Vector2::new(0.0, 0.0),
        Vector2::new(50.0, 0.0),
        Vector2::new(50.0, 3.0),
        Vector2::new(28.0, 7.053),
        Vector2::new(28.0, 7.053 - step),
        Vector2::new(25.0, 7.053 - step),
        Vector2::new(0.0, 7.053 - step),
    ];
    // Both handednesses, so the riser's two triangles split it along either diagonal.
    let mirrored: Vec<Vector2> = outline
        .iter()
        .rev()
        .map(|p| Vector2::new(-p.x, p.y))
        .collect();
    // The two-flat shape the step stands in for: the level flat at the sloping flat's height.
    let flats = vec![
        [0.0, 0.0],
        [50.0, 0.0],
        [50.0, 3.0],
        [28.0, 7.053],
        [0.0, 7.053],
    ];
    let mirrored_flats: Vec<[f64; 2]> = flats.iter().rev().map(|p| [-p[0], p[1]]).collect();
    let cases = [
        (outline, flats, [28.0, 7.053], [-1.0, 0.0, 0.0]),
        (mirrored, mirrored_flats, [-28.0, 7.053], [1.0, 0.0, 0.0]),
    ];
    for (shape, flats, strip_at, strip_normal) in cases {
        let part = extrude_contours(&[shape], 11.5).expect("extruded the stepped outline");
        let ideal = Ideal {
            outline: flats,
            strip_at,
            strip_normal,
            slack: step,
        };
        for target in [0.7, 1.43, 2.0] {
            assert_meshed_as_a_sharp_edge(&part, target, "step riser", &ideal);
        }
    }
}

/// A finely faceted pin: every face of its wall and caps is narrower than the strip limit, but
/// the wall is as tall as the pin and the caps as wide as its radius, so nothing about it is a
/// strip and the remesh must return what it returned before strips were merged.
#[test]
fn a_finely_faceted_pin_is_not_taken_for_a_strip() {
    let sides = 400;
    let outline: Vec<Vector2> = (0..sides)
        .map(|i| {
            let angle = std::f64::consts::TAU * i as f64 / sides as f64;
            Vector2::new(0.5 * angle.cos(), 0.5 * angle.sin())
        })
        .collect();
    let part = extrude_contours(&[outline], 3.0).expect("extruded the pin");
    let target = 0.0358 * 40.0;
    let size = move |_: P| target;
    let options = AnalysisRemeshOptions::new(&size, 0.0358);
    let result = remesh_for_analysis(&part, &options);
    let print = fingerprint(&result);
    println!("pin: {print}");
    assert_eq!(print, PIN_BEFORE_STRIPS);
}

/// A remesh outcome as text: a refusal by its value, a surface by its face count and an FNV-1a
/// hash of its exact coordinates and faces, so any change to the output shows.
fn fingerprint(result: &Result<AnalysisRemesh, AnalysisRemeshError>) -> String {
    match result {
        Ok(remesh) => {
            let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
            let mut eat = |word: u64| {
                for byte in word.to_le_bytes() {
                    hash ^= u64::from(byte);
                    hash = hash.wrapping_mul(0x0100_0000_01b3);
                }
            };
            for p in &remesh.surface.vertices {
                for x in p {
                    eat(x.to_bits());
                }
            }
            for f in &remesh.surface.faces {
                for &v in f {
                    eat(u64::from(v as u32));
                }
            }
            format!("Ok {} faces {hash:016x}", remesh.surface.faces.len())
        }
        Err(error) => format!("{error:?}"),
    }
}

/// A 16-sided cylinder with a noise facet poked into one side face: a sliver 2 micrometres wide
/// along the face's diagonal, tilted about 45 degrees out of the face, so its every edge reads
/// as a crease. The wall around it stays one surface, joined round the cylinder, so the sliver
/// is set into one surface, not between two creases, and the strip pass must leave the part as
/// it was. (A second such sliver would cut the wall into two surfaces, each sliver then lying
/// between them.)
fn cylinder_with_noise_facets() -> Mesh {
    let (sides, radius, height) = (16, 10.0, 20.0);
    let mut points: Vec<P> = Vec::new();
    for z in [0.0, height] {
        for i in 0..sides {
            let angle = std::f64::consts::TAU * i as f64 / sides as f64;
            points.push([radius * angle.cos(), radius * angle.sin(), z]);
        }
    }
    let (bottom_center, top_center) = (points.len(), points.len() + 1);
    points.push([0.0, 0.0, 0.0]);
    points.push([0.0, 0.0, height]);
    let mut faces: Vec<[usize; 3]> = Vec::new();
    for i in 0..sides {
        let j = (i + 1) % sides;
        faces.push([bottom_center, j, i]);
        faces.push([top_center, sides + i, sides + j]);
        // The side quad's second triangle, then its first, poked on the first side.
        faces.push([i, sides + j, sides + i]);
        let (a, b, d) = (sides + j, i, j);
        if i != 0 {
            faces.push([a, b, d]);
            continue;
        }
        let (pa, pb, pd) = (points[a], points[b], points[d]);
        let middle = [
            (pa[0] + pb[0]) * 0.5,
            (pa[1] + pb[1]) * 0.5,
            (pa[2] + pb[2]) * 0.5,
        ];
        let inward = [pd[0] - middle[0], pd[1] - middle[1], 0.0];
        let inward_length = (inward[0] * inward[0] + inward[1] * inward[1]).sqrt();
        let outward = [middle[0] / radius, middle[1] / radius, 0.0];
        let offset = 0.0014;
        points.push([
            middle[0] + inward[0] / inward_length * offset + outward[0] * offset,
            middle[1] + inward[1] / inward_length * offset + outward[1] * offset,
            middle[2],
        ]);
        let c = points.len() - 1;
        faces.push([a, b, c]);
        faces.push([b, d, c]);
        faces.push([d, a, c]);
    }
    mesh_of(&points, &faces)
}

/// What the noise-facet cylinder's remesh returned before strips were merged (measured on the
/// remesher without the strip pass).
const NOISE_BEFORE_STRIPS: &str =
    "AngleFloorNotMet { triangles: 23, worst_degrees: 0.09827932285023347, floor_degrees: 1.0 }";

#[test]
fn noise_facets_set_into_one_surface_are_not_taken_for_strips() {
    let part = cylinder_with_noise_facets();
    let target = 1.18;
    let size = move |_: P| target;
    let options = AnalysisRemeshOptions::new(&size, target / 40.0);
    let print = fingerprint(&remesh_for_analysis(&part, &options));
    println!("noise facets: {print}");
    assert_eq!(print, NOISE_BEFORE_STRIPS);
}
