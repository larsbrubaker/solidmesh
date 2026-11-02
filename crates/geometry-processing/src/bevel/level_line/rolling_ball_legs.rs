//! One chain of a level-line pass walked by the rolling ball for `rolling_ball_setbacks.rs`, and
//! each station measured as the band's setbacks there: how far each side's contact stands from the
//! chain. The pass's rules (whole pass or none, the sides, the ends) are that file's header; this
//! file is where a chain is cut into LEGS at its sharp turns.
//!
//! **A sharp turn.** A chain vertex where the field mitres (`chain_field_rays::mitres_at`, the one
//! predicate both read: a turn past the pass's bend threshold that is no full reversal, and no
//! smooth joint of the scope in force) and one side's face changes there by more than the
//! threshold too. A closed chain with one such turn is one leg from the turn round to it.
//! At an inside turn - the face beyond the vertex stands across the incoming leg's way, as a
//! pocket's wall does at the floor rim's corner - the real ball touches three faces, both legs'
//! faces on that side and the other side's. Along each leg, the ball touching only that leg's two
//! faces stops where it comes to touch the other leg's face too (that three-face ball's foot, a
//! radius back from the vertex on a right angle), and between there and the vertex no ball square
//! to the leg touches only its own two faces: the other leg's face stands nearer than the radius.
//! A chain point there (the Around Center pick's, R5, the other leg's wall 4.95 from the centre)
//! has no station of its own. At an outside turn - the face beyond falls away, as at a knife's tip
//! where the floor changes over a ridge - the ball rolls round the ridge, its contact on the ridge
//! a little way from the chain, which is not the round the mitre carries to the vertex.
//!
//! **Each leg ends on the other leg's face.** The level line's field mitres a sharp turn: each
//! leg's round is carried on to the vertex and the two meet in the bisector plane (at an inside
//! turn the shared side's rails meet at the three-face ball's contact on it; on flat faces
//! exactly). So what the field needs at a sharp turn is each leg's own round carried on to the
//! vertex, which is the end rule's (extend and trim, `rolling_ball::walk`) with the other leg's
//! face on the side that changes as the leg's end face, read as that face's plane through the
//! vertex. Each leg is walked as an open chain between its turns (the chain's own ends where it is
//! open), its sides flooded from that leg's own triangles only (`rolling_ball_sides`), so the
//! other leg's face is no side of it: it is the leg's end face, neither side nor obstacle, and
//! the leg's sides are read as extended past its plane (a floor running on past the ridge it ends
//! at as its own plane). A leg's end station is measured from its own end edge's line
//! (`from_the_end_edge`), never from the other leg's edge, which a contact beside the turn can lie
//! nearer. On flat faces every setback is then the angle's (`unit_setbacks`), as the switch off
//! reads them, and the mitre the field builds is the level line's (held to the Around Center pick
//! against its control).
//!
//! **One setback per vertex.** The band's field holds one setback per chain vertex and side, so at
//! a turn the two legs' setbacks are averaged (the switch-off rule at every vertex, the mean of the
//! edges either side); where the legs' setbacks differ, the field tapers between that mean and each
//! leg's own over the leg's last edge. A turn's centre and contacts are each leg's carried along
//! its own leg direction onto the mitre plane (through the vertex, square to the sum of the legs'
//! directions), then averaged: on flat faces both legs land on one point, the field's corner (the
//! three-face ball's centre and its contact on the shared side, the other side's contact on the
//! crease), and the mean of two points on two different lines would lie on neither. A turn's
//! contacts lie on end faces of both legs, so no feature is named there
//! (`SideSetbacks::features`). Where both sides' faces change at one vertex, no one face ends
//! either leg, and the chain is walked through it, as it is through a turn the field does not
//! mitre (it keeps the vertex's own ball there).
//!
//! **Cost.** Each leg is a walk of its own: a ball grown from a sixteenth of the radius at its
//! middle station and carried to both ends, and its sides flooded and boxed afresh. A coarse rim
//! sharp at every vertex (a 12-gon's) walks one leg per edge, so it costs about legs times
//! (growth plus the side reading T) where a smooth chain costs one growth and one T.
//!
//! Numbers follow the crate's rules: `f64`, a fixed order of operations, no fused multiply-add;
//! turns are found in chain order, and legs are walked and averaged in chain order.

use vector_math::Vector3;

use super::triangle_edges::{self, StandingMap};
use super::ChainField;
use super::{chain_field_rays, rolling_ball_sides};
use crate::bevel::rolling_ball::{
    ChainPoint, ContactFeature, EndFace, RollingBall, RollingBallSide, WalkEnd,
};

/// How square to a leg the other leg's face must stand (the cosine between the leg and the face's
/// normal) for the leg to meet its plane from one side, as the walk asks of an end face.
const END_PLANE_LEAST_COSINE: f64 = 1e-9;

/// One chain of the pass and everything its walk reads.
pub(super) struct ChainToWalk<'c> {
    pub positions: &'c [Vector3],
    pub indices: &'c [(i32, i32, i32)],
    pub standing: &'c StandingMap,
    /// The chain's number in the pass, for the route's reasons.
    pub k: usize,
    pub chain: &'c [i32],
    pub closed: bool,
    pub edges: &'c [(i32, i32)],
    /// Each chain edge's left (side A) and right (side B) triangle.
    pub lefts: &'c [i32],
    pub rights: &'c [i32],
    pub convex: bool,
    /// The whole chain's sides, by triangle.
    pub in_a: &'c [bool],
    pub in_b: &'c [bool],
    /// The face of neither side each end of an open chain lies on.
    pub ends: [Option<EndFace>; 2],
    pub side_regions: Option<&'c [u32]>,
    /// Whether a side's flood crosses an edge.
    pub crosses: &'c dyn Fn(i32, i32) -> bool,
    pub part: &'c RollingBallSide,
    pub size: f64,
    /// The turn, in degrees, past which the pass's field mitres a chain vertex.
    pub sharp_turn_degrees: f64,
}

/// The ball's reading at one chain point, in the part's units.
#[derive(Clone, Copy, Debug)]
pub(super) struct PointReading {
    /// How far side A's and side B's contacts stand from the chain there.
    pub to_a: f64,
    pub to_b: f64,
    /// The ball's centre less the chain point.
    pub centre: Vector3,
    /// Side A's and side B's contact less the chain point.
    pub contacts: [Vector3; 2],
    /// What each contact lies on, `None` at a station on an end face.
    pub features: [Option<ContactFeature>; 2],
}

/// A chain walked: one reading per chain point, and how each end of an open chain finished.
pub(super) struct WalkedChain {
    pub readings: Vec<PointReading>,
    pub ends: [WalkEnd; 2],
}

/// A sharp turn at chain point `at`: the leg into it ends on the plane of the face beyond it
/// (normal `beyond`), the leg out of it starts on the plane of the face before it (`before`).
#[derive(Clone, Copy, Debug)]
struct SharpTurn {
    at: usize,
    beyond: Vector3,
    before: Vector3,
    /// The chain's unit directions into and out of the turn.
    into: Vector3,
    out_of: Vector3,
}

/// One leg's stretch of the chain: its chain point indices in order (a closed chain's last leg
/// wraps past its start, so an index may be the chain's length or more), and its end faces.
struct Leg {
    at: Vec<usize>,
    ends: [Option<EndFace>; 2],
}

/// The chain walked leg by leg (see the file header), each station measured.
pub(super) fn walked(c: &ChainToWalk<'_>) -> Result<WalkedChain, String> {
    let k = c.k;
    let n = c.chain.len();
    let turns = sharp_turns(c);
    let turn_at = |at: usize| turns.iter().find(|turn| turn.at == at).copied();
    let end_face = |turn: &SharpTurn, normal: Vector3| EndFace {
        point: c.positions[c.chain[turn.at] as usize],
        normal,
    };
    let legs: Vec<Leg> = if turns.is_empty() {
        vec![Leg {
            at: (0..n).collect(),
            ends: c.ends,
        }]
    } else if c.closed {
        (0..turns.len())
            .map(|j| {
                let (from, to) = (turns[j], turns[(j + 1) % turns.len()]);
                let stop = if to.at > from.at { to.at } else { to.at + n };
                Leg {
                    at: (from.at..=stop).collect(),
                    ends: [
                        Some(end_face(&from, from.before)),
                        Some(end_face(&to, to.beyond)),
                    ],
                }
            })
            .collect()
    } else {
        let mut legs = Vec::with_capacity(turns.len() + 1);
        let mut start: (usize, Option<EndFace>) = (0, c.ends[0]);
        for turn in &turns {
            legs.push(Leg {
                at: (start.0..=turn.at).collect(),
                ends: [start.1, Some(end_face(turn, turn.beyond))],
            });
            start = (turn.at, Some(end_face(turn, turn.before)));
        }
        legs.push(Leg {
            at: (start.0..n).collect(),
            ends: [start.1, c.ends[1]],
        });
        legs
    };

    // A closed chain with one sharp turn is one leg from the turn round to it, still a leg.
    let whole = turns.is_empty();
    let leg_closed = c.closed && whole;
    let mut per_point: Vec<Vec<PointReading>> = vec![Vec::new(); n];
    let mut finished = [WalkEnd::LastStation; 2];
    for (l, leg) in legs.iter().enumerate() {
        let vertices: Vec<i32> = leg.at.iter().map(|&i| c.chain[i % n]).collect();
        // A closed chain walked whole keeps its closing edge; a leg has one edge fewer than points.
        let edge_count = if leg_closed { n } else { leg.at.len() - 1 };
        let edge_of = |e: usize| leg.at[e] % n;
        let edges: Vec<(i32, i32)> = (0..edge_count).map(|e| c.edges[edge_of(e)]).collect();
        let lefts: Vec<i32> = (0..edge_count).map(|e| c.lefts[edge_of(e)]).collect();
        let rights: Vec<i32> = (0..edge_count).map(|e| c.rights[edge_of(e)]).collect();
        let leg_sides;
        let (in_a, in_b) = if whole {
            (c.in_a, c.in_b)
        } else {
            leg_sides = rolling_ball_sides::of_the_chain(
                c.indices,
                c.standing,
                k,
                (&lefts, &rights),
                c.side_regions,
                c.crosses,
            )?;
            (&leg_sides.0[..], &leg_sides.1[..])
        };
        let triangles_of = |inside: &[bool]| -> Vec<[i32; 3]> {
            (0..c.indices.len())
                .filter(|&t| inside[t])
                .map(|t| {
                    let (a, b, d) = c.indices[t];
                    [a, b, d]
                })
                .collect()
        };
        let side_a = RollingBallSide::new(c.positions, &triangles_of(in_a));
        let side_b = RollingBallSide::new(c.positions, &triangles_of(in_b));
        let points = chain_points(
            c.positions,
            c.indices,
            &vertices,
            leg_closed,
            &edges,
            &lefts,
            &rights,
        );
        let ball = RollingBall {
            side_a: &side_a,
            side_b: &side_b,
            part: Some(c.part),
            radius: c.size,
            convex: c.convex,
        };
        let walk = ball.walk_ending_on(&points, leg.ends).map_err(|error| {
            if whole {
                format!("chain {k}: the ball has no walk: {error:?}")
            } else {
                format!(
                    "chain {k}: the ball has no walk from point {} to point {}: {error:?}",
                    leg.at[0] % n,
                    leg.at[leg.at.len() - 1] % n
                )
            }
        })?;
        // Which of the chain's own ends each of the leg's ends is, if any.
        let own_end = |end: usize| match end {
            0 if l == 0 && !c.closed => Some(0),
            1 if l + 1 == legs.len() && !c.closed => Some(1),
            _ => None,
        };
        let where_ends = |end: usize| match own_end(end) {
            Some(own) => format!("chain {k} end {own}"),
            None => format!(
                "chain {k} at its turn at point {}",
                leg.at[if end == 0 { 0 } else { leg.at.len() - 1 }] % n
            ),
        };
        for (end, done) in walk.ends.iter().enumerate() {
            if !matches!(done, WalkEnd::EndFace | WalkEnd::LastStation) {
                return Err(match own_end(end) {
                    Some(own) => format!("chain {k}: end {own} finished {done:?}"),
                    None => format!("{}: finished {done:?}", where_ends(end)),
                });
            }
            if let Some(own) = own_end(end) {
                finished[own] = *done;
            }
        }
        if walk.stations.len() != points.len() {
            return Err(format!(
                "chain {k}: {} stations for {} chain points",
                walk.stations.len(),
                points.len()
            ));
        }

        let last = walk.stations.len() - 1;
        for (i, station) in walk.stations.iter().enumerate() {
            let on_an_end_face = !leg_closed
                && ((i == 0 && walk.ends[0] == WalkEnd::EndFace)
                    || (i == last && walk.ends[1] == WalkEnd::EndFace));
            // A contact on a rim of a side read as extended past the end face lies where the
            // extension has a hole (the ones `rolling_ball::side_extension` lists): no surface
            // the round rests on, so its distance is no setback. Fail safe.
            if on_an_end_face {
                let end = usize::from(i != 0);
                for (contact, name) in [(&station.contact_a, "A"), (&station.contact_b, "B")] {
                    if contact.feature.is_rim() {
                        return Err(format!(
                            "{}: side {name}'s contact at the end face rests on a rim of the side \
                             extended past the face, so its setback there is not the round's",
                            where_ends(end)
                        ));
                    }
                }
            }
            let measure = |contact: Vector3| {
                from_the_chain(c.positions, &edges, leg_closed, i, on_an_end_face, contact)
            };
            let off_the_end_face = |feature: ContactFeature| (!on_an_end_face).then_some(feature);
            let reading = PointReading {
                to_a: measure(station.contact_a.point),
                to_b: measure(station.contact_b.point),
                centre: station.centre - station.chain_point,
                contacts: [
                    station.contact_a.point - station.chain_point,
                    station.contact_b.point - station.chain_point,
                ],
                features: [
                    off_the_end_face(station.contact_a.feature),
                    off_the_end_face(station.contact_b.feature),
                ],
            };
            // A leg's end at a turn: its centre and contacts carried along its own leg to the
            // mitre plane (see the file header).
            let turn_end = (i == 0 && !whole && own_end(0).is_none())
                .then(|| turn_at(leg.at[0] % n).map(|t| t.out_of))
                .or_else(|| {
                    (i == last && !whole && own_end(1).is_none())
                        .then(|| turn_at(leg.at[last] % n).map(|t| t.into))
                })
                .flatten();
            let reading = match turn_end {
                Some(along) => {
                    let turn = turn_at(leg.at[i] % n).ok_or_else(|| {
                        format!("chain {k} point {}: a leg ends on no turn", leg.at[i] % n)
                    })?;
                    let mitre = turn.into + turn.out_of;
                    let onto = |offset: Vector3| onto_the_mitre(offset, along, mitre);
                    PointReading {
                        centre: onto(reading.centre),
                        contacts: [onto(reading.contacts[0]), onto(reading.contacts[1])],
                        ..reading
                    }
                }
                None => reading,
            };
            per_point[leg.at[i] % n].push(reading);
        }
    }

    let mut readings = Vec::with_capacity(n);
    for (i, found) in per_point.iter().enumerate() {
        readings.push(match found.as_slice() {
            [only] => *only,
            // A turn: the two legs' setbacks averaged, and their centres and contacts, each
            // already on the mitre plane (see the file header).
            [a, b] => {
                let mean = |x: f64, y: f64| (x + y) * 0.5;
                let middle = |x: Vector3, y: Vector3| (x + y) * 0.5;
                PointReading {
                    to_a: mean(a.to_a, b.to_a),
                    to_b: mean(a.to_b, b.to_b),
                    centre: middle(a.centre, b.centre),
                    contacts: [
                        middle(a.contacts[0], b.contacts[0]),
                        middle(a.contacts[1], b.contacts[1]),
                    ],
                    features: [None, None],
                }
            }
            _ => {
                return Err(format!(
                    "chain {k} point {i}: {} stations where its legs meet",
                    found.len()
                ))
            }
        });
    }

    Ok(WalkedChain {
        readings,
        ends: finished,
    })
}

/// The chain's sharp turns, in chain order (see the file header): none without a finite bend
/// threshold, which mitres nothing.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)` on purpose: a NaN is no turn
fn sharp_turns(c: &ChainToWalk<'_>) -> Vec<SharpTurn> {
    if !c.sharp_turn_degrees.is_finite() {
        return Vec::new();
    }
    let count = c.edges.len();
    let direction = |e: usize| {
        let (from, to) = c.edges[e];
        (c.positions[to as usize] - c.positions[from as usize]).get_normal()
    };
    let normal = |t: i32| triangle_edges::normal(c.positions, c.indices[t as usize]);
    let cos_threshold = chain_field_rays::sharp_turn_cosine(c.sharp_turn_degrees);
    // The joints the field reads as smooth whatever their turn (rounds first's section) are no
    // turn of its, so none of the legs'.
    let points: Vec<Vector3> = c.chain.iter().map(|&v| c.positions[v as usize]).collect();
    let smooth = ChainField::smooth_joints_in(&points);

    let mut turns = Vec::new();
    for v in 0..c.chain.len() {
        let before = if v > 0 {
            v - 1
        } else if c.closed {
            count - 1
        } else {
            continue;
        };
        if v >= count {
            continue;
        }
        if smooth
            .as_ref()
            .is_some_and(|joints| joints.contains(&(v as i32)))
        {
            continue;
        }
        let (into, out_of) = (direction(before), direction(v));
        if !chain_field_rays::mitres_at(into, out_of, cos_threshold) {
            continue;
        }
        let mut turned = Vec::new();
        for (side_in, side_out) in [
            (c.lefts[before], c.lefts[v]),
            (c.rights[before], c.rights[v]),
        ] {
            let (face_before, face_beyond) = (normal(side_in), normal(side_out));
            // Each leg must meet the other's face from one side of its plane.
            if face_before.dot(face_beyond) < cos_threshold
                && into.dot(face_beyond).abs() > END_PLANE_LEAST_COSINE
                && out_of.dot(face_before).abs() > END_PLANE_LEAST_COSINE
            {
                turned.push((face_before, face_beyond));
            }
        }
        // Both sides' faces changing: no one face ends either leg.
        if let [(before_face, beyond_face)] = turned[..] {
            turns.push(SharpTurn {
                at: v,
                beyond: beyond_face,
                before: before_face,
                into,
                out_of,
            });
        }
    }
    turns
}

/// `offset` (from a turn's vertex) carried along the unit leg direction `along` onto the mitre
/// plane through the vertex, normal `mitre` (the sum of the legs' directions). The turn is no full
/// reversal (`chain_field_rays::mitres_at`), so `along` meets the plane: its cosine with the
/// mitre is one plus the turn's.
fn onto_the_mitre(offset: Vector3, along: Vector3, mitre: Vector3) -> Vector3 {
    let t = -(offset.dot(mitre)) / along.dot(mitre);
    offset + (along * t)
}

/// How far `contact` stands from the chain's edges either side of chain vertex `v`: the nearer.
/// At an open chain's end on an end face (`on_an_end_face`), the one edge there is carried on
/// past the end vertex as its line ([`from_the_end_edge`]): the end station there touches each
/// side where the side runs on past the face (`rolling_ball::walk`), which past a face leaning
/// back over the end vertex lies beyond it, and the rail there stands that far from the chain
/// carried on past its end, as the band's own field reads it past the end. At a square end the
/// contact lies in the plane square to the chain at the end vertex, where the two agree.
pub(super) fn from_the_chain(
    positions: &[Vector3],
    edges: &[(i32, i32)],
    is_closed: bool,
    v: usize,
    on_an_end_face: bool,
    contact: Vector3,
) -> f64 {
    let before = if v > 0 {
        Some(v - 1)
    } else if is_closed {
        Some(edges.len() - 1)
    } else {
        None
    };
    let after = (v < edges.len()).then_some(v);
    let mut nearest = f64::INFINITY;
    for e in [before, after].into_iter().flatten() {
        let (from, to) = edges[e];
        let (start, stop) = (positions[from as usize], positions[to as usize]);
        let on = if on_an_end_face {
            from_the_end_edge(start, stop, v == 0, contact)
        } else {
            ChainField::closest_on_segment(contact, start, stop)
        };
        let distance = (contact - on).length();
        if distance < nearest {
            nearest = distance;
        }
    }

    nearest
}

/// The point of an open chain's end edge `start -> stop` nearest `contact`, the edge carried on as
/// its line past the chain's end vertex only (`start` at the first end, `stop` at the last) and
/// clamped at its other vertex, toward the chain's interior, where the next edge in takes over.
#[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` on purpose: a NaN reads the vertex
fn from_the_end_edge(start: Vector3, stop: Vector3, first_end: bool, contact: Vector3) -> Vector3 {
    let along = stop - start;
    let length_squared = along.length_squared();
    if !(length_squared > 0.0) {
        return start;
    }
    let t = (contact - start).dot(along) / length_squared;
    let t = if first_end {
        vector_math::ieee_math::min_f64(t, 1.0)
    } else {
        vector_math::ieee_math::max_f64(t, 0.0)
    };
    start + (along * t)
}

/// The chain's points as the ball reads them: each vertex with the chain's direction there and
/// each side's normal beside it, both blended over the edges either side of the vertex (the one
/// edge at an open chain's end).
fn chain_points(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    is_closed: bool,
    edges: &[(i32, i32)],
    lefts: &[i32],
    rights: &[i32],
) -> Vec<ChainPoint> {
    let count = edges.len();
    let direction =
        |e: usize| (positions[edges[e].1 as usize] - positions[edges[e].0 as usize]).get_normal();
    let left_normal = |e: usize| triangle_edges::normal(positions, indices[lefts[e] as usize]);
    let right_normal = |e: usize| triangle_edges::normal(positions, indices[rights[e] as usize]);
    let blend = |of: &dyn Fn(usize) -> Vector3, before: Option<usize>, after: Option<usize>| match (
        before, after,
    ) {
        (Some(b), Some(a)) => (of(b) + of(a)).get_normal(),
        (Some(only), None) | (None, Some(only)) => of(only),
        (None, None) => Vector3::ZERO,
    };

    (0..chain.len())
        .map(|v| {
            // Vertex v stands between edge v - 1 and edge v.
            let before = if v > 0 {
                Some(v - 1)
            } else if is_closed {
                Some(count - 1)
            } else {
                None
            };
            let after = (v < count).then_some(v);
            ChainPoint {
                point: positions[chain[v] as usize],
                tangent: blend(&direction, before, after),
                normal_a: blend(&left_normal, before, after),
                normal_b: blend(&right_normal, before, after),
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A chain turning a right angle, (0, 0, 0) to (1, 0, 0) to (1, 1, 0): each end edge is 1 long.
    fn turning() -> (Vec<Vector3>, Vec<(i32, i32)>) {
        let positions = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(1.0, 1.0, 0.0),
        ];
        (positions, vec![(0, 1), (1, 2)])
    }

    /// At an end face the end edge runs on as its line past the end vertex only: a contact past
    /// the end stands its distance from that line, and one whose offset along the edge reaches
    /// past the edge's inner vertex stands its distance from that vertex, never from the line.
    #[test]
    fn the_end_edge_runs_on_past_the_end_vertex_only() {
        let (positions, edges) = turning();
        let measure = |v: usize, contact: Vector3| {
            from_the_chain(&positions, &edges, false, v, true, contact)
        };
        // First end: past (0, 0, 0) along -x the line, 1.5 off it.
        assert_eq!(measure(0, Vector3::new(-2.0, 1.5, 0.0)), 1.5);
        // First end, 3 along the edge where it is 1 long: from (1, 0, 0), not the line's 2.
        let inner = measure(0, Vector3::new(3.0, 2.0, 0.0));
        assert!((inner - 8f64.sqrt()).abs() < 1e-15, "{inner}");
        // Last end: past (1, 1, 0) along +y the line, 0.75 off it.
        assert_eq!(measure(2, Vector3::new(0.25, 2.5, 0.0)), 0.75);
        // Last end, 3 back along the edge: from (1, 0, 0).
        let inner = measure(2, Vector3::new(1.5, -3.0, 0.0));
        assert!((inner - 0.5f64.hypot(3.0)).abs() < 1e-15, "{inner}");
    }
}
