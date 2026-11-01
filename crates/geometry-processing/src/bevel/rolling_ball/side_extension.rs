//! A side as the ball reads it at a chain that ends on an end face ([`extended_past`]): cut at
//! each end face's plane and carried on past it, each of its triangles that meets the plane
//! extended as its own plane. The walk (`walk.rs`) reads a chain's sides through this wherever
//! the chain has an end face; the overview is the module header (`rolling_ball/mod.rs`).
//!
//! **Why extend.** At a chain end on an end face the round a part gets is the one its sides
//! would make if they ran on past the end face, cut flat by the face: extend and trim. Read only
//! up to the end face's plane, a side that the face leans across ends short of where the ball
//! would touch its continuation, so near a leaning end face the end stations' contacts land on the
//! side's edge with the face and fall short of the rails (the 10 degree sheared wedge's walls,
//! whose round is 2.7235 and 2.3416 from the chain everywhere, read 2.70 and 2.32 at its ends).
//! Read as extended, the end station touches each side where its continuation lies, and on prism
//! walls that is the wall's own plane, so every station reads the same distances.
//!
//! **The construction.** Each end face's plane bounds the side: triangles wholly on the chain's
//! side are kept as they are, triangles wholly beyond are dropped, and triangles the plane crosses
//! are cut there, one new vertex per crossed edge, so neighbouring triangles share it. That clip
//! is why nothing beyond the end face that is not the side's own is read: a side region can wrap
//! on past the plane (the T Handle's pocket wall runs through the web), and only what lies on the
//! chain's side of the plane is the side up to the face. Then every edge of the cut side that lies
//! in the plane and that only one of its triangles uses - the side's trace in the end face - gets
//! a strip in that triangle's own plane, running out square to the edge away from the triangle,
//! far enough that a point the ball's radius beyond the plane is inside it; a facet extends as
//! itself. Where two such strips leave one vertex in different directions a triangle fills the
//! wedge between them, so the extension is one surface with the side, sharing its vertices: the
//! side's trace in the plane is no rim any more, and contact tracking walks on over it.
//!
//! **What it keeps.** The strips lie wholly beyond the plane, so a centre on the chain's side
//! whose ball does not reach the plane is nearer the side's own surface than any strip, and its
//! contact lies where the side alone puts it. At a square end the ball held in the plane touches
//! each side on its trace in the plane, which is the strip's edge as well: the same position, but
//! named `Edge { rim: false }` (the side now runs on past it) where it was a rim, and the cut side
//! is a new triangle list, so a solve can differ from the side's own in the last bits.
//!
//! **Its gaps.** The strips cover the plane of each facet only square out from its edge in the end
//! plane: nothing covers the wedge between a strip and the chain's own line carried on past the
//! end vertex (where the trace meets the chain at a slant), a vertex three or more strips leave, a
//! strip left out because it would run along the plane, or what lies past a strip's reach. A
//! contact there lands on a strip's open edge and reads as a rim; the pass refuses such an end
//! station rather than take its distance as a setback (`level_line::rolling_ball_setbacks`).

use std::collections::HashMap;

use vector_math::Vector3;

use super::side_surface::{HalfSpace, RollingBallSide};

/// How nearly a strip must run along the plane, as the cosine between its direction out and the
/// plane's normal, before it is left out: a triangle lying in the plane extends nowhere past it.
const LEAST_STRIP_COSINE: f64 = 1e-3;

/// How many radii past the plane a strip reaches, at least: the ball's contact beyond the plane is
/// at most a radius from its centre, which stands on the chain's side; twice leaves room.
const STRIP_RADII: f64 = 2.0;

/// `side` cut at each of `ends` (each the chain's side of an end face's plane) and extended past
/// it as the file header says, for a ball of `radius`; `tolerance` is how near a vertex must stand
/// to a plane to count as on it. Vertices keep their indices; new ones follow them.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` on purpose: a NaN extends nothing
pub(super) fn extended_past(
    side: &RollingBallSide,
    ends: &[HalfSpace],
    radius: f64,
    tolerance: f64,
) -> RollingBallSide {
    let mut positions = side.positions.clone();
    let depth = |p: Vector3, end: &HalfSpace| {
        let d = end.depth(p);
        if d.abs() <= tolerance {
            0.0
        } else {
            d
        }
    };

    // Cut each triangle at each plane in turn; a crossed edge's new vertex is keyed on its two
    // ends, low first, and placed from the low end, so both its triangles share it exactly.
    let mut crossings: HashMap<(i32, i32, usize), i32> = HashMap::new();
    let mut kept: Vec<[i32; 3]> = Vec::with_capacity(side.triangles.len());
    for &triangle in &side.triangles {
        let mut polygon: Vec<i32> = triangle.to_vec();
        for (h, end) in ends.iter().enumerate() {
            let depths: Vec<f64> = polygon
                .iter()
                .map(|&v| depth(positions[v as usize], end))
                .collect();
            if depths.iter().all(|&d| d >= 0.0) {
                continue;
            }
            if depths.iter().all(|&d| d <= 0.0) {
                polygon.clear();
                break;
            }
            let mut cut: Vec<i32> = Vec::with_capacity(polygon.len() + 1);
            for i in 0..polygon.len() {
                let j = (i + 1) % polygon.len();
                let (p, q) = (polygon[i], polygon[j]);
                let (dp, dq) = (depths[i], depths[j]);
                if dp >= 0.0 {
                    cut.push(p);
                }
                if (dp > 0.0 && dq < 0.0) || (dp < 0.0 && dq > 0.0) {
                    let (lo, hi, dlo, dhi) = if p < q {
                        (p, q, dp, dq)
                    } else {
                        (q, p, dq, dp)
                    };
                    let at = *crossings.entry((lo, hi, h)).or_insert_with(|| {
                        let (a, b) = (positions[lo as usize], positions[hi as usize]);
                        positions.push(a + ((b - a) * (dlo / (dlo - dhi))));
                        // A part's vertex count fits an i32 (the mesh's own indices are i32).
                        (positions.len() - 1) as i32
                    });
                    cut.push(at);
                }
            }
            polygon = cut;
        }
        for i in 1..polygon.len().saturating_sub(1) {
            kept.push([polygon[0], polygon[i], polygon[i + 1]]);
        }
    }

    // The cut side's edges in a plane that one triangle uses, each with that triangle's far
    // corner, in triangle order.
    let mut uses: HashMap<(i32, i32), u32> = HashMap::new();
    for t in &kept {
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            *uses.entry((a.min(b), a.max(b))).or_insert(0) += 1;
        }
    }
    let in_plane = |v: i32, end: &HalfSpace| depth(positions[v as usize], end) == 0.0;
    // Each strip as its edge `a -> b` in the side's order and how far out each end goes.
    let mut runs: Vec<(i32, i32, Vector3, Vector3)> = Vec::new();
    for t in &kept {
        for (a, b, c) in [(t[0], t[1], t[2]), (t[1], t[2], t[0]), (t[2], t[0], t[1])] {
            if uses.get(&(a.min(b), a.max(b))) != Some(&1) {
                continue;
            }
            let Some(end) = ends.iter().find(|e| in_plane(a, e) && in_plane(b, e)) else {
                continue;
            };
            let (pa, pb, pc) = (
                positions[a as usize],
                positions[b as usize],
                positions[c as usize],
            );
            let normal = (pb - pa).cross(pc - pa);
            let along = pb - pa;
            if !(normal.length() > 0.0) || !(along.length() > 0.0) {
                continue;
            }
            // Away from the triangle, square to the edge, in the triangle's plane.
            let out = along.get_normal().cross(normal.get_normal()).get_normal();
            let past = -out.dot(end.normal);
            if !(past > LEAST_STRIP_COSINE) {
                continue;
            }
            let reach = (STRIP_RADII * radius) / past;
            runs.push((a, b, pa + (out * reach), pb + (out * reach)));
        }
    }

    // Each strip leaves each end `v` of its edge toward a new far vertex; `into` says whether the
    // side's edge runs into `v` (its `b`) or out of it (its `a`), so the wedge fill can orient.
    let mut strips: Vec<[i32; 3]> = Vec::new();
    let mut leaving: HashMap<i32, Vec<(i32, bool)>> = HashMap::new();
    let mut leaving_order: Vec<i32> = Vec::new();
    for (a, b, far_a, far_b) in runs {
        positions.push(far_a);
        positions.push(far_b);
        // A part's vertex count fits an i32 (the mesh's own indices are i32).
        let (fa, fb) = ((positions.len() - 2) as i32, (positions.len() - 1) as i32);
        strips.push([b, a, fa]);
        strips.push([b, fa, fb]);
        for (v, far, into) in [(a, fa, false), (b, fb, true)] {
            let around = leaving.entry(v).or_default();
            if around.is_empty() {
                leaving_order.push(v);
            }
            around.push((far, into));
        }
    }

    // Fill the wedge where two strips leave one vertex, one whose side edge runs into it and one
    // out of it. The strip into `v` runs `far -> v` and the one out of it `v -> far`, so the wedge
    // runs `v, far_in, far_out` and every edge it shares is run both ways.
    for v in leaving_order {
        let around = &leaving[&v];
        let (Some(&(far_in, _)), Some(&(far_out, _))) = (
            around.iter().find(|(_, into)| *into),
            around.iter().find(|(_, into)| !*into),
        ) else {
            continue;
        };
        if around.len() != 2 {
            continue;
        }
        let (p, q_in, q_out) = (
            positions[v as usize],
            positions[far_in as usize],
            positions[far_out as usize],
        );
        if !((q_in - p).cross(q_out - p).length() > tolerance * tolerance) {
            continue;
        }
        strips.push([v, far_in, far_out]);
    }

    kept.extend(strips);
    RollingBallSide::new(&positions, &kept)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A floor `z = 0` from x = 0 to 4 and y = -4 to 4, wound up, as a side; the chain runs along
    /// its edge x = 0.
    fn floor() -> RollingBallSide {
        let positions = vec![
            Vector3::new(0.0, -4.0, 0.0),
            Vector3::new(4.0, -4.0, 0.0),
            Vector3::new(4.0, 4.0, 0.0),
            Vector3::new(0.0, 4.0, 0.0),
        ];
        RollingBallSide::new(&positions, &[[0, 1, 2], [0, 2, 3]])
    }

    /// The chain's side of an end plane through the origin leaning `degrees` off square to the
    /// chain (the y axis), its normal into the chain's side; at a negative lean the floor's trace
    /// in it runs from the origin down to y = -x tan(-degrees).
    fn leaning(degrees: f64) -> HalfSpace {
        let lean = degrees.to_radians();
        HalfSpace {
            origin: Vector3::ZERO,
            normal: Vector3::new(lean.sin(), -lean.cos(), 0.0),
        }
    }

    /// Cut at a plane leaning -25 degrees, the floor and its strip are one surface wound one way:
    /// every edge two triangles share is run once each way, and no edge twice the same way.
    #[test]
    fn the_extended_side_is_wound_one_way() {
        let side = extended_past(&floor(), &[leaning(-25.0)], 2.0, 1e-9);
        let mut runs: HashMap<(i32, i32), u32> = HashMap::new();
        for t in &side.triangles {
            for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                *runs.entry((a, b)).or_insert(0) += 1;
            }
        }
        assert!(side.triangles.len() > 2, "the strip was added");
        for (&(a, b), &count) in &runs {
            assert_eq!(count, 1, "edge {a}-{b} run {count} times one way");
        }
        // The floor's trace in the plane is shared with the strip: run once each way.
        let shared = runs
            .keys()
            .filter(|&&(a, b)| runs.contains_key(&(b, a)))
            .count();
        assert!(shared >= 2, "{runs:?}");
    }

    /// Past the plane the strip carries the floor on, so a point above it touches the floor's own
    /// plane there, on no rim; but beside the chain's line carried on past the end vertex (x = 0),
    /// where the strip runs square to the slanted trace and not along the chain, there is a gap:
    /// the nearest point is the strip's open edge, a rim, which the pass refuses to read.
    #[test]
    fn past_the_plane_the_floor_runs_on_except_in_the_wedge_beside_the_chain() {
        let side = extended_past(&floor(), &[leaning(-25.0)], 2.0, 1e-9);
        // Above the strip: past the trace, which falls 25 degrees in y along x from the origin.
        let above = side
            .closest(Vector3::new(2.0, 0.0, 1.0))
            .expect("the side has triangles");
        assert!((above.distance - 1.0).abs() < 1e-12, "{above:?}");
        assert!(!above.feature.is_rim(), "{above:?}");
        // In the wedge between the chain's line past the origin and the strip.
        let wedge = side
            .closest(Vector3::new(0.1, 1.0, 1.0))
            .expect("the side has triangles");
        assert!(wedge.feature.is_rim(), "{wedge:?}");
        assert!(wedge.distance > 1.0, "{wedge:?}");
    }
}
