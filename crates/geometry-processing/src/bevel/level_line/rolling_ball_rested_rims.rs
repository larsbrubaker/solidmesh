//! The rims a side's ball contacts rest on, read as that side's rail (the rolling ball's route
//! only, `rolling_ball_setbacks`).
//!
//! Where a side is narrower than the ball's reach (a riser below a step's top, the T Handle's
//! pocket wall past |y| about 10.5) the ball rests on the side's far rim, and the band on that
//! side is everything between the chain and that rim: its rail is the rim itself. The cut removes
//! the side's band right up to the rim, so the gap's open edge there stands on a kept triangle of
//! the face past the rim, which belongs to neither side (tag 0). The loop walk takes such an edge
//! as a side's step only across a knife or a rim the band is consumed to
//! (`gap_loop_walk::loops_of_side`); a concave crease is no knife, and a rim the ball rests on at
//! a size within the room is no consumed rim, so without this list the side's rail broke into
//! stubs and the fill refused its open ends. [`of_the_chain`] hands each side the rim segments
//! its contacts rest on, which the build adds to that side group's consumed rims.
//!
//! A rim segment is a boundary edge of the side's region (an edge one of whose triangles is not
//! the side's) that is no chain edge, whose two ends both lie within a run of stations resting on
//! a rim, widened by one chain edge each way: between two stations the contact crosses from the
//! face onto the rim (or round a rim vertex) somewhere along the chain edge, and the rim edges it
//! crosses there are as much the rail as the ones a station names. The ends must also lie within
//! the ball's reach of the chain, so a far rim of a large region (the T Handle's walls are one
//! region round the whole part) is never named.
//!
//! Left out, each for its reason:
//! - a station on an end face (its feature is `None`): its side is read as extended past the face,
//!   so its feature names the extension, not the part; a contact on a hole of the extension is
//!   refused, and a rim the side's own contact rests on there is listed only through a station
//!   beside it resting on that rim (widened by one chain edge);
//! - a contact on a vertex the walk added past an end face (numbered from the part's count up):
//!   no edge of the part carries it;
//! - a contact on the chain itself (a chain edge or vertex, as at a free end where the contacts
//!   close onto the chain): the chain is where the band starts, not where it ends.

use std::collections::HashSet;

use vector_math::Vector3;

use super::rolling_ball_band_tags::BallSides;
use super::rolling_ball_setbacks::SideSetbacks;
use super::triangle_edges;
use crate::bevel::rolling_ball::ContactFeature;

/// A rim segment, its two ends' positions on the input mesh.
pub type RimSegment = (Vector3, Vector3);

/// Each side's rested rims, side A's (tag 1's) then side B's (tag 2's), for the one chain whose
/// vertices are `chain` (each vertex once, a closed chain's last joining its first), read from the
/// ball's walk (`ball`, per unit of `size`) and the triangles it rolled on (`sides`).
pub fn of_the_chain(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    closed: bool,
    ball: &SideSetbacks,
    sides: &BallSides,
    size: f64,
) -> [Vec<RimSegment>; 2] {
    let on_the_chain: HashSet<i32> = chain.iter().copied().collect();
    let mut chain_edges: HashSet<(i32, i32)> = chain
        .windows(2)
        .map(|w| triangle_edges::key(w[0], w[1]))
        .collect();
    if closed && chain.len() > 2 {
        chain_edges.insert(triangle_edges::key(chain[chain.len() - 1], chain[0]));
    }
    let points: Vec<Vector3> = chain.iter().map(|&v| positions[v as usize]).collect();
    let part_vertices = positions.len() as i64;
    let rests_on_a_rim = |feature: &Option<ContactFeature>| match *feature {
        Some(ContactFeature::Edge { from, to, rim }) => {
            rim && i64::from(from.max(to)) < part_vertices
                && !chain_edges.contains(&triangle_edges::key(from, to))
        }
        Some(ContactFeature::Vertex { vertex, rim }) => {
            rim && i64::from(vertex) < part_vertices && !on_the_chain.contains(&vertex)
        }
        _ => false,
    };

    let standing = triangle_edges::standing_map(indices);
    let mut rims: [Vec<RimSegment>; 2] = [Vec::new(), Vec::new()];
    for (s, inside) in [&sides.in_a, &sides.in_b].into_iter().enumerate() {
        let (Some(features), Some(contacts)) = (ball.features.get(s), ball.contacts.get(s)) else {
            continue;
        };
        let rested: Vec<bool> = features.iter().map(rests_on_a_rim).collect();
        if rested.len() != points.len() || !rested.contains(&true) {
            continue;
        }
        // How far out a rested station's contact stands from its chain point, at the size.
        let reach_at = |station: usize| contacts.get(station).map_or(0.0, |c| c.length() * size);
        let within_a_run = |p: Vector3| -> bool {
            let (along, distance, edge_length) = chain_parameter(&points, closed, p);
            let n = points.len() as f64;
            (0..points.len()).filter(|&m| rested[m]).any(|m| {
                let mut apart = (along - m as f64).abs();
                if closed {
                    apart = apart.min(n - apart);
                }
                // One chain edge each way covers the crossing between stations; the reach grows
                // by that edge's length, since the rim between two stations may stand farther
                // out than either station's contact.
                apart <= 1.0 + 1e-9 && distance <= (reach_at(m) * (1.0 + 1e-6)) + edge_length
            })
        };
        // The side's boundary edges in the mesh's own edge order, so the list never depends on
        // hashing.
        for (&(low, high), on) in &standing {
            if chain_edges.contains(&(low, high)) {
                continue;
            }
            let ours = on.iter().filter(|&&t| inside[t as usize]).count();
            if ours == 0 || ours == on.len() {
                continue;
            }
            let (a, b) = (positions[low as usize], positions[high as usize]);
            if within_a_run(a) && within_a_run(b) {
                rims[s].push((a, b));
            }
        }
    }

    rims
}

/// Where `p` lies along the chain: the chain parameter of its nearest point (vertex index plus the
/// share of the edge after it; past an open chain's ends the end edge's line carries on, so a
/// point beyond an end reads below 0 or above the last vertex), its distance from the chain, and
/// the length of the nearest edge. Of two equally near edges, the first.
fn chain_parameter(points: &[Vector3], closed: bool, p: Vector3) -> (f64, f64, f64) {
    let count = points.len();
    let edges = if closed {
        count
    } else {
        count.saturating_sub(1)
    };
    if edges == 0 {
        return (
            0.0,
            points.first().map_or(f64::INFINITY, |&q| (p - q).length()),
            0.0,
        );
    }
    let mut best = (0.0, f64::INFINITY, 0.0);
    for e in 0..edges {
        let (from, to) = (points[e], points[(e + 1) % count]);
        let along = to - from;
        let length_squared = along.length_squared();
        let t = if length_squared > 0.0 {
            ((p - from).dot(along) / length_squared).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let distance = (p - (from + (along * t))).length();
        if distance < best.1 {
            let mut share = t;
            if !closed && length_squared > 0.0 {
                let unclamped = (p - from).dot(along) / length_squared;
                if (e == 0 && unclamped < 0.0) || (e + 1 == edges && unclamped > 1.0) {
                    share = unclamped;
                }
            }
            best = (e as f64 + share, distance, length_squared.sqrt());
        }
    }

    best
}
