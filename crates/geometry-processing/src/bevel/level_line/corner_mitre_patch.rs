//! The patch a corner hole between TWO convex rounds is filled with, laid onto the mitred solid
//! the two rounds make ([`on_the_mitre`]).

use indexmap::IndexSet;
use vector_math::{cs_math, Vector3};

use super::{CornerCapLastRow, HoleFillResult, HoleFillTriangle};

/// One round as a solid: its row, its unit axis, and the unit directions from its centre to its
/// two tangent lines, squared up to the axis.
struct Round<'a> {
    row: &'a CornerCapLastRow,
    axis: Vector3,
    to_first: Vector3,
    to_last: Vector3,
}

/// The patch `filled` (over the rim `points`) laid onto the mitred solid of the two convex
/// `rows` that meet at a corner, or `filled` unchanged where the corner is not that: two rows of
/// one radius, convex, of three rail points or more, on axes that are not parallel, with every rim
/// point on the mitred surface.
///
/// Each vertex goes onto the round that bounds it; the MITRE'S CREASE is laid into the patch (every
/// edge whose ends went to different rounds is split where the rounds read alike, the point placed
/// on the crease, and crease chords that cut inside a curved crease are split until they stand no
/// farther off it than a row's own segment stands off its arc); and the patch is then refined to
/// the rows' own spacing, each long edge halved onto its own round's surface.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)` refuses a NaN as well.
pub fn on_the_mitre(
    filled: HoleFillResult,
    points: &[Vector3],
    rows: &[CornerCapLastRow],
) -> HoleFillResult {
    if rows.len() != 2 || rows.iter().any(|row| row.concave || row.arc.len() < 3) {
        return filled;
    }

    let radius = rows[0].radius;
    let tolerance = 1e-6 * cs_math::max_f64(radius, rows[0].centre.length());
    if !(radius > tolerance) || (rows[1].radius - radius).abs() > tolerance {
        return filled;
    }

    // Each round as a solid: its two side faces, rounded between their tangent lines about its
    // axis (a plane arc's plane normal; a skew row carries its chain's direction).
    let rounds: Vec<Round> = rows
        .iter()
        .map(|row| {
            let axis = if row.axis.length_squared() > 0.0 {
                row.axis.get_normal()
            } else {
                row.plane_normal
            };
            let square = |w: Vector3| w - (axis * w.dot(axis));
            Round {
                row,
                axis,
                to_first: square(row.first - row.centre).get_normal(),
                to_last: square(row.last - row.centre).get_normal(),
            }
        })
        .collect();
    if rounds.iter().any(|r| r.axis.length_squared() == 0.0)
        || rounds[0].axis.cross(rounds[1].axis).length() < 1e-3
    {
        return filled;
    }

    // Which of the round's three surfaces a reading is of: 0 the round, 1 its first side face, 2
    // its last - a crease point stands on one of each round's.
    let read = |k: usize, q: Vector3| -> (f64, Vector3, i32) {
        let r = &rounds[k];
        let w = q - r.row.centre;
        let along = r.axis * w.dot(r.axis);
        let square = w - along;
        let span = r.to_first.cross(r.to_last);
        if r.to_first.cross(square).dot(span) >= 0.0
            && square.cross(r.to_last).dot(span) >= 0.0
            && square.length() > 0.0
        {
            let onto = r.row.centre + along + (square.get_normal() * radius);
            return (square.length() - radius, onto, 0);
        }

        let past_first = (q - r.row.first).dot(r.to_first);
        let past_last = (q - r.row.last).dot(r.to_last);
        if past_first >= past_last {
            (past_first, q - (r.to_first * past_first), 1)
        } else {
            (past_last, q - (r.to_last * past_last), 2)
        }
    };

    // Every rim point on the mitred solid's surface: outside neither round, on one of them.
    let on_the_mitre = 1e-3 * radius;
    if points
        .iter()
        .any(|&q| cs_math::max_f64(read(0, q).0, read(1, q).0).abs() > on_the_mitre)
    {
        return filled;
    }

    let n = points.len();
    let mut vertices = filled.vertices.clone();
    let mut owner: Vec<i32> = Vec::new();
    let mut triangles = filled.triangles.clone();
    let segment_angle = rows
        .iter()
        .map(|row| {
            (row.first - row.centre)
                .get_normal()
                .dot((row.last - row.centre).get_normal())
                .clamp(-1.0, 1.0)
                .acos()
                / (row.arc.len() - 1) as f64
        })
        .fold(f64::NEG_INFINITY, cs_math::max_f64);
    let chord_sag = radius * (1.0 - (segment_angle / 2.0).cos());

    let placed = |q: Vector3| {
        let first = read(0, q);
        let second = read(1, q);
        if first.0 >= second.0 {
            first.1
        } else {
            second.1
        }
    };

    // A crease point lies on BOTH bounding surfaces: projected onto each in turn until it stands
    // on both, the alternation closing on their crossing.
    let on_the_crease = |mut q: Vector3| {
        for _ in 0..64 {
            q = read(0, q).1;
            q = read(1, q).1;
            if read(0, q).0.abs() <= tolerance && read(1, q).0.abs() <= tolerance {
                break;
            }
        }

        q
    };

    let owner_of = |q: Vector3| {
        let first = read(0, q).0;
        let second = read(1, q).0;
        if (first - second).abs() <= on_the_mitre {
            -1
        } else if first > second {
            0
        } else {
            1
        }
    };

    for (i, vertex) in vertices.iter_mut().enumerate() {
        if i >= n {
            *vertex = placed(*vertex);
        }

        owner.push(owner_of(*vertex));
    }

    let on_the_loop = |a: usize, b: usize| a < n && b < n && ((a + 1) % n == b || (b + 1) % n == a);

    let off_the_crease =
        |q: Vector3| cs_math::max_f64(read(0, q).0.abs(), read(1, q).0.abs()) > chord_sag;

    // Read once: the splits grow the triangle list, and a bound read off it each step would grow
    // with them.
    let split_limit = 10 * (triangles.len() + n);
    for _ in 0..split_limit {
        let crossing = |owner: &[i32], a: usize, b: usize| {
            owner[a] >= 0 && owner[b] >= 0 && owner[a] != owner[b] && !on_the_loop(a, b)
        };
        let a_curved_crease_chord = |owner: &[i32], vertices: &[Vector3], a: usize, b: usize| {
            owner[a] < 0
                && owner[b] < 0
                && !on_the_loop(a, b)
                && read(0, vertices[a]).2 == read(0, vertices[b]).2
                && read(1, vertices[a]).2 == read(1, vertices[b]).2
                && off_the_crease((vertices[a] + vertices[b]) / 2.0)
        };

        let split = triangles
            .iter()
            .flat_map(|t| [(t.a, t.b), (t.b, t.c), (t.c, t.a)])
            .map(|(a, b)| (a as usize, b as usize))
            .find(|&(a, b)| {
                crossing(&owner, a, b) || a_curved_crease_chord(&owner, &vertices, a, b)
            });
        let Some((a, b)) = split else {
            break;
        };

        let middle = vertices.len() as i32;
        if crossing(&owner, a, b) {
            let (owner_a, owner_b) = (owner[a] as usize, owner[b] as usize);
            let lean = |q: Vector3| read(owner_a, q).0 - read(owner_b, q).0;
            let (mut low, mut high) = (0.0, 1.0);
            for _ in 0..50 {
                let mid = (low + high) / 2.0;
                if lean(vertices[a] + ((vertices[b] - vertices[a]) * mid)) > 0.0 {
                    low = mid;
                } else {
                    high = mid;
                }
            }

            let at = vertices[a] + ((vertices[b] - vertices[a]) * ((low + high) / 2.0));
            vertices.push(on_the_crease(at));
        } else {
            vertices.push(on_the_crease((vertices[a] + vertices[b]) / 2.0));
        }
        owner.push(-1);

        triangles = split_edge(&triangles, a as i32, b as i32, middle);
    }

    // THE PATCH IS REFINED to the rows' own spacing once it is placed: an edge longer than a row
    // segment, on one round's surfaces (or from the crease onto them), is halved and its middle
    // carried onto that same surface, in a pass count fixed up front. Edges between two loop
    // vertices are left: their middles are the loop's own rows and face pieces.
    let target = 2.0 * radius * (segment_angle / 2.0).sin();
    let surface_of = |owner: &[i32], a: usize, b: usize| {
        if owner[a] >= 0 && owner[b] >= 0 {
            if owner[a] == owner[b] {
                owner[a]
            } else {
                -1
            }
        } else {
            owner[a].max(owner[b])
        }
    };
    let long_edges = |owner: &[i32], vertices: &[Vector3], triangles: &[HoleFillTriangle]| {
        let distinct: IndexSet<(i32, i32)> = triangles
            .iter()
            .flat_map(|t| [(t.a, t.b), (t.b, t.c), (t.c, t.a)])
            .map(|(a, b)| if a < b { (a, b) } else { (b, a) })
            .collect();
        distinct
            .into_iter()
            .filter(|&(a, b)| {
                let (a, b) = (a as usize, b as usize);
                !(a < n && b < n)
                    && surface_of(owner, a, b) >= 0
                    && (vertices[a] - vertices[b]).length() > target
            })
            .collect::<Vec<_>>()
    };
    let longest = long_edges(&owner, &vertices, &triangles)
        .iter()
        .map(|&(a, b)| (vertices[a as usize] - vertices[b as usize]).length())
        .fold(0.0, cs_math::max_f64);
    let passes = if target > 0.0 && longest > target {
        // The ceiling is a whole number; `as` saturates should it be out of range.
        i32::min(4, (longest / target).log2().ceil() as i32)
    } else {
        0
    };
    for _ in 0..passes {
        for (a, b) in long_edges(&owner, &vertices, &triangles) {
            let k = surface_of(&owner, a as usize, b as usize);
            let middle = vertices.len() as i32;
            let at = read(
                k as usize,
                (vertices[a as usize] + vertices[b as usize]) / 2.0,
            )
            .1;
            vertices.push(at);
            triangles = split_edge(&triangles, a, b, middle);
            owner.push(k);
        }
    }

    HoleFillResult::new(filled.failure, vertices, triangles)
}

/// Every triangle on edge `a`-`b` (either way round) split at vertex `middle`, keeping winding;
/// the rest kept as they are.
fn split_edge(
    triangles: &[HoleFillTriangle],
    a: i32,
    b: i32,
    middle: i32,
) -> Vec<HoleFillTriangle> {
    let mut next = Vec::with_capacity(triangles.len() + 2);
    for t in triangles {
        let corners = [t.a, t.b, t.c];
        let at = (0..3).find(|&c| {
            (corners[c] == a && corners[(c + 1) % 3] == b)
                || (corners[c] == b && corners[(c + 1) % 3] == a)
        });
        let Some(at) = at else {
            next.push(*t);
            continue;
        };

        let (x, y, z) = (corners[at], corners[(at + 1) % 3], corners[(at + 2) % 3]);
        next.push(HoleFillTriangle::new(x, middle, z));
        next.push(HoleFillTriangle::new(middle, y, z));
    }

    next
}
