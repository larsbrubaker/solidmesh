//! The loops a solid's ring is built on: a ring of an extrusion, a column of a revolve, a station
//! of a sweep (`ring_vertices.rs` places their points, and the caps and walls join them). A
//! clipper clean-up gives the area an outline fills, but not always as loops a solid can be built
//! on directly: a hole can touch its outline at a point partway along one of the outline's edges,
//! two loops can run along the same stretch in opposite directions (a hole sharing an edge with
//! its outline: a seam of no width), and a loop can pass through one point twice.
//! [`simple_loops`] turns any of these into loops whose every point the ring's other loops pass
//! through is a point of theirs too, that share no edge, and that never revisit a point except
//! once per filled wedge there, so the cap fills them with the loops' own edges and every wall
//! edge has exactly two faces. Before that, [`without_crossings`] mends the loops clipper hands
//! back where its rounding to whole units left two edges crossing.

use std::collections::HashMap;

use agg_path::clipper_library::{ClipperError, PolygonsExtensions};
use clipper2_rust::{FillRule, Path64, Paths64, Point64};

/// A ring point as a map key.
type Key = (i64, i64);

/// The loops a ring can be built on: every point of the ring that lies on another edge added to
/// that edge (see [`with_touch_points`]), the stretches two loops run along in opposite directions
/// removed and the rest chained so each pass through a point bounds one filled wedge (see
/// [`chained_by_wedges`]), then the points that add no area dropped (see
/// [`without_straight_points`]).
pub fn simple_loops(loops: Paths64) -> Paths64 {
    without_straight_points(chained_by_wedges(with_touch_points(loops)))
}

/// How many times [`without_crossings`] re-runs the union before taking what it has.
const CROSSING_ROUNDS: usize = 8;

/// The loops with no two edges crossing. Clipper computes where edges meet exactly but rounds
/// each meeting point to whole units, so a loop it returns (an offset's inset, the union a profile
/// is cleaned up with) can cross itself or another loop by less than a unit: a needle of no width
/// whose way back crosses its way out, a tiny loop turned inside out beyond the crossing. No
/// solid can be built on such loops - a flat fill across them needs a point that is on none of
/// them - so the loops are put through clipper's union again, filling what winds positively (an
/// outline's inside, not its holes, not a lobe turned inside out), which finds the crossing and
/// cuts the loops there. That rounds a new point too, which can cross again in turn, so this
/// repeats while any crossing is left, up to [`CROSSING_ROUNDS`] times. Loops that cross nowhere
/// come back unchanged.
pub fn without_crossings(loops: Paths64) -> Result<Paths64, ClipperError> {
    let mut loops = loops;
    for _ in 0..CROSSING_ROUNDS {
        if !has_crossing(&loops) {
            break;
        }
        loops = loops.union(&Paths64::new(), FillRule::Positive)?;
    }
    Ok(loops)
}

/// Whether two edges of the loops cross at a point inside both, found exactly on the integer
/// points. Edges that only touch (at an end, or along a shared line) are not crossings: the
/// steps of [`simple_loops`] handle those.
fn has_crossing(loops: &Paths64) -> bool {
    let mut edges: Vec<(Point64, Point64)> = loops
        .iter()
        .flat_map(|path| (0..path.len()).map(move |i| (path[i], path[(i + 1) % path.len()])))
        .filter(|(a, b)| a != b)
        .collect();
    // by lowest x, so each edge is checked only against the edges that start within its x range
    edges.sort_unstable_by_key(|(a, b)| (a.x.min(b.x), a.x.max(b.x), a.y, b.y));
    let side = |value: i128| value.signum();
    for (i, &(a, b)) in edges.iter().enumerate() {
        let high_x = a.x.max(b.x);
        let (low_y, high_y) = (a.y.min(b.y), a.y.max(b.y));
        for &(c, d) in edges[i + 1..]
            .iter()
            .take_while(|(c, d)| c.x.min(d.x) <= high_x)
        {
            if c.y.max(d.y) < low_y || c.y.min(d.y) > high_y {
                continue;
            }
            let (c_side, d_side) = (side(cross(a, b, c)), side(cross(a, b, d)));
            let (a_side, b_side) = (side(cross(c, d, a)), side(cross(c, d, b)));
            if c_side * d_side < 0 && a_side * b_side < 0 {
                return true;
            }
        }
    }
    false
}

/// Twice the signed area of the triangle (a, b, c): zero when the three lie on one line.
fn cross(a: Point64, b: Point64, c: Point64) -> i128 {
    let (abx, aby) = ((b.x - a.x) as i128, (b.y - a.y) as i128);
    let (acx, acy) = ((c.x - a.x) as i128, (c.y - a.y) as i128);
    abx * acy - aby * acx
}

/// The loops with every point of any loop that lies strictly inside one of their edges added to
/// that edge, in order along it. Where a hole touches its outline partway along an edge, the cap
/// fill meets the touch point and so splits the edge there; the wall must be split at the same
/// point, or the cap and the wall would meet on different edges and the solid would not close.
/// Loops with no such point come back unchanged.
fn with_touch_points(loops: Paths64) -> Paths64 {
    // every point of the ring once, sorted by x then y, so an edge looks only at the points
    // inside its x range
    let mut points: Vec<(i64, i64)> = loops.iter().flatten().map(|p| (p.x, p.y)).collect();
    points.sort_unstable();
    points.dedup();
    loops
        .into_iter()
        .map(|path| {
            let count = path.len();
            let mut out = Vec::with_capacity(count);
            for i in 0..count {
                let (a, b) = (path[i], path[(i + 1) % count]);
                out.push(a);
                let (low_x, high_x) = (a.x.min(b.x), a.x.max(b.x));
                let (low_y, high_y) = (a.y.min(b.y), a.y.max(b.y));
                let first = points.partition_point(|&(x, _)| x < low_x);
                // each point strictly between a and b, with how far along the edge it is
                let mut inside: Vec<(i128, Point64)> = points[first..]
                    .iter()
                    .take_while(|&&(x, _)| x <= high_x)
                    .map(|&(x, y)| Point64::new(x, y))
                    .filter(|&p| {
                        p.y >= low_y && p.y <= high_y && p != a && p != b && cross(a, b, p) == 0
                    })
                    .map(|p| {
                        let along = (p.x - a.x) as i128 * (b.x - a.x) as i128
                            + (p.y - a.y) as i128 * (b.y - a.y) as i128;
                        (along, p)
                    })
                    .collect();
                // distinct points on one line are at distinct distances along it
                inside.sort_unstable_by_key(|&(along, _)| along);
                out.extend(inside.into_iter().map(|(_, p)| p));
            }
            out
        })
        .collect()
}

/// The loops with every edge that some loop also runs the other way removed, both walks of it,
/// and every edge from a point to itself, and what remains chained back into loops that pass
/// each point once for every filled wedge there. Two walks of one edge in opposite directions are
/// a seam of no width (a hole that shares an edge with its outline, or a needle that runs out and
/// back): they enclose nothing, and walls standing on both would put four faces on one edge.
/// Removing a pair takes one edge into and one out of each of its points, so every point still has
/// as many edges in as out and the rest chains into closed loops.
///
/// Where the outline passes one point more than once (a hole touching its outline at a point, two
/// parts meeting at a corner, a bowtie's crossing), the filled area around the point is several
/// wedges, each between an edge leaving the point and the next edge round counter clockwise, which
/// arrives (the filled side of every edge is its left). The loops are chained so that each pass
/// through the point arrives along one wedge's arriving edge and leaves along the same wedge's
/// leaving edge; the ring then gives each pass a vertex of its own (`ring_vertices.rs`), so the wall
/// edge standing on each has the two faces of its own wedge and the cap meets each wedge on its
/// own vertex. Elsewhere each loop keeps its own order. Loops with no removed edge and no point
/// passed twice come back unchanged.
fn chained_by_wedges(loops: Paths64) -> Paths64 {
    let key = |p: Point64| (p.x, p.y);
    // every edge in loop order, with the index of the edge after it in its loop
    let mut edges: Vec<(Point64, Point64)> = Vec::new();
    let mut next_in_loop: Vec<usize> = Vec::new();
    for path in &loops {
        let (start, count) = (edges.len(), path.len());
        for i in 0..count {
            edges.push((path[i], path[(i + 1) % count]));
            next_in_loop.push(start + (i + 1) % count);
        }
    }

    let mut removed: Vec<bool> = edges.iter().map(|(a, b)| a == b).collect();
    let mut by_ends: HashMap<(Key, Key), Vec<usize>> = HashMap::new();
    for (index, &(a, b)) in edges.iter().enumerate() {
        if !removed[index] {
            by_ends.entry((key(a), key(b))).or_default().push(index);
        }
    }
    for index in 0..edges.len() {
        if removed[index] {
            continue;
        }
        let (a, b) = edges[index];
        let opposite = by_ends
            .get(&(key(b), key(a)))
            .and_then(|walks| walks.iter().copied().find(|&other| !removed[other]));
        if let Some(other) = opposite {
            removed[index] = true;
            removed[other] = true;
        }
    }

    // the edges left leaving and arriving at each point, in loop order
    let mut leaving: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    let mut arriving: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    for (index, &(a, b)) in edges.iter().enumerate() {
        if !removed[index] {
            leaving.entry(key(a)).or_default().push(index);
            arriving.entry(key(b)).or_default().push(index);
        }
    }
    let passed_twice = leaving.values().any(|out| out.len() > 1);
    if !passed_twice && !removed.contains(&true) {
        return loops;
    }

    // the edge each arriving edge continues into
    let mut continues: Vec<usize> = next_in_loop.clone();
    for (point, out) in &leaving {
        let into = arriving.get(point).map_or(&[][..], Vec::as_slice);
        let at = Point64::new(point.0, point.1);
        for (arrive, leave) in wedge_pairs(at, into, out, &edges, &next_in_loop) {
            continues[arrive] = leave;
        }
    }

    let mut used = removed;
    let mut chained = Vec::new();
    for start in 0..edges.len() {
        if used[start] {
            continue;
        }
        let mut path = Vec::new();
        let mut edge = start;
        // each edge continues into exactly one other, so the walk comes back to its first edge
        while !used[edge] {
            used[edge] = true;
            path.push(edges[edge].0);
            edge = continues[edge];
        }
        chained.push(path);
    }
    chained
}

/// Which side of the x axis a direction is on, for sorting directions counter clockwise from +x.
fn half(v: (i128, i128)) -> u8 {
    u8::from(!(v.1 > 0 || (v.1 == 0 && v.0 > 0)))
}

/// Pairs each edge arriving at `at` with the edge leaving it that bounds the same filled wedge:
/// going counter clockwise round the point from a leaving edge, the wedge's far side is the next
/// edge met, which arrives. Where the edges round the point do not alternate (only edges that
/// overlap can make that happen), the edges left unpaired keep their loop order, or failing that
/// are paired in turn, so every arriving edge still continues into one leaving edge.
fn wedge_pairs(
    at: Point64,
    arriving: &[usize],
    leaving: &[usize],
    edges: &[(Point64, Point64)],
    next_in_loop: &[usize],
) -> Vec<(usize, usize)> {
    let direction = |toward: Point64| ((toward.x - at.x) as i128, (toward.y - at.y) as i128);
    // (direction from the point, edge, whether it leaves)
    let mut round: Vec<((i128, i128), usize, bool)> = leaving
        .iter()
        .map(|&e| (direction(edges[e].1), e, true))
        .chain(arriving.iter().map(|&e| (direction(edges[e].0), e, false)))
        .collect();
    // counter clockwise from +x; equal directions keep edge order, so the result is fixed
    round.sort_by(|a, b| {
        half(a.0).cmp(&half(b.0)).then_with(|| {
            let turn = a.0 .0 * b.0 .1 - a.0 .1 * b.0 .0;
            0.cmp(&turn).then(a.1.cmp(&b.1))
        })
    });
    let mut pairs = Vec::new();
    let mut paired_in = vec![false; round.len()];
    let mut paired_out = vec![false; round.len()];
    for i in 0..round.len() {
        let j = (i + 1) % round.len();
        if round[i].2 && !round[j].2 && !paired_in[j] {
            pairs.push((round[j].1, round[i].1));
            paired_in[j] = true;
            paired_out[i] = true;
        }
    }
    let mut spare_out: Vec<usize> = (0..round.len())
        .filter(|&i| round[i].2 && !paired_out[i])
        .map(|i| round[i].1)
        .collect();
    for i in (0..round.len()).filter(|&i| !round[i].2 && !paired_in[i]) {
        let arrive = round[i].1;
        let pick = spare_out
            .iter()
            .position(|&leave| leave == next_in_loop[arrive])
            .or((!spare_out.is_empty()).then_some(0));
        if let Some(pick) = pick {
            pairs.push((arrive, spare_out.remove(pick)));
        }
    }
    pairs
}

/// The loops with every point that lies exactly on the line through its neighbours removed
/// (integer cross product zero: the outline runs straight on through it, turns straight back
/// in a needle of no width, or repeats it), and any loop left with fewer than three points
/// dropped. Such a point adds no area to the shape, but a cap or band could close it off only
/// with a zero-area triangle. A point the loops pass through more than once stays: there the
/// outline touches itself, and the fill and the walls of every pass must meet on it.
fn without_straight_points(loops: Paths64) -> Paths64 {
    // how many times the loops pass through each point
    let mut passes: HashMap<(i64, i64), usize> = HashMap::new();
    for point in loops.iter().flatten() {
        *passes.entry((point.x, point.y)).or_insert(0) += 1;
    }
    let shared = |p: Point64| passes.get(&(p.x, p.y)).is_some_and(|&n| n > 1);
    loops
        .into_iter()
        .map(|path| {
            let mut points = path;
            // a point repeated in place adds nothing, shared or not: one copy stays
            points.dedup();
            while points.len() > 1 && points.first() == points.last() {
                points.pop();
            }
            // Each removal can straighten a neighbour, so repeat until nothing changes.
            loop {
                let count = points.len();
                if count < 3 {
                    break;
                }
                let kept: Path64 = (0..count)
                    .filter(|&i| {
                        let (prev, here, next) = (
                            points[(i + count - 1) % count],
                            points[i],
                            points[(i + 1) % count],
                        );
                        cross(prev, here, next) != 0 || shared(here)
                    })
                    .map(|i| points[i])
                    .collect();
                if kept.len() == count {
                    break;
                }
                points = kept;
            }
            points
        })
        .filter(|path| path.len() >= 3)
        .collect()
}
