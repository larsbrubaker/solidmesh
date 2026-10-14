//! Where the level line crosses one mesh edge ([`EdgeRoot`]): found once, shared by both
//! triangles on that edge, with the bisection's constants ([`BISECTION_STEPS`],
//! [`BRACKET_TOLERANCE`]) and the bracketed search that finds the roots ([`find`], [`find_over`],
//! [`root_on`]).

use vector_math::Vector3;

use super::{
    FieldCounters, LevelLineMesh, LevelLineWork, VertexFieldReading, VertexSide, WorkError,
};

/// Where the level line crosses one mesh edge, keyed by the edge's two vertex
/// indices in ascending order.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct EdgeRoot {
    /// The lower vertex index of the edge.
    pub a: i32,
    /// The higher vertex index of the edge.
    pub b: i32,
    /// Where on the edge the line crosses.
    pub point: Vector3,
}

impl EdgeRoot {
    /// The root at `point` on the edge from `a` to `b`.
    pub fn new(a: i32, b: i32, point: Vector3) -> Self {
        Self { a, b, point }
    }
}

/// The worst case, so a pathological field still stops - 60
/// halvings take any edge below 1e-18 of its length.
pub const BISECTION_STEPS: i32 = 60;

/// How tight the bracket round the root is taken, as a fraction
/// of the segment's length. Fixed rather than adaptive so the same input always gives the same
/// bits, and far below the 1e-9 the module's answers are held to.
pub const BRACKET_TOLERANCE: f64 = 1e-13;

/// Every crossed edge's root over the whole mesh.
pub fn find(
    mesh: &LevelLineMesh,
    reading: &VertexFieldReading,
) -> Result<Vec<EdgeRoot>, WorkError> {
    find_over(mesh, reading, mesh.edges())
}

/// Every crossed edge's root over a named set of
/// edges, after settling which vertices the line passes exactly through. Promoting an end to
/// on-the-line can uncross its other edges, so the pass repeats until nothing more is promoted -
/// which terminates because each pass promotes at least one vertex and never demotes one.
pub fn find_over(
    mesh: &LevelLineMesh,
    reading: &VertexFieldReading,
    over_edges: impl IntoIterator<Item = (i32, i32)>,
) -> Result<Vec<EdgeRoot>, WorkError> {
    // Sorted whatever order the caller named them in: the roots come back in this order, the
    // caller splits them in it, and each split appends vertices - so the indices the whole cut is
    // built on are fixed by the edge SET. Distinct keys sort with no ties to order.
    let mut edges: Vec<(i32, i32)> = over_edges.into_iter().collect();
    edges.sort_unstable();
    edges.dedup();
    loop {
        // One pass per promotion, bounded by the band's vertices - charged all the same, because
        // that bound is not local here.
        LevelLineWork::charge(1)?;
        let mut roots = Vec::new();
        let mut promoted = false;
        for &(low, high) in &edges {
            let side_low = reading.side_of(low)?;
            let side_high = reading.side_of(high)?;
            if side_low == VertexSide::On || side_high == VertexSide::On || side_low == side_high {
                continue;
            }

            let point = root_on(mesh.position(low), mesh.position(high), reading)?;
            if (point - mesh.position(low)).length() <= reading.snap {
                reading.put_on_the_line(low)?;
                promoted = true;
                break;
            }

            if (point - mesh.position(high)).length() <= reading.snap {
                reading.put_on_the_line(high)?;
                promoted = true;
                break;
            }

            roots.push(EdgeRoot::new(low, high, point));
        }

        if !promoted {
            return Ok(roots);
        }
    }
}

/// The memo key: the exact ordered points' bits, -0.0 read as 0.0 since the two are equal.
fn root_key(from: Vector3, to: Vector3) -> [u64; 6] {
    [from.x, from.y, from.z, to.x, to.y, to.z].map(|c| (c + 0.0).to_bits())
}

/// The root of the field on the segment from one point to another, by a
/// bracketed (Illinois secant) search that never leaves the bracket. Remembered per reading, keyed
/// on the exact ordered points; the work units the first search charged are charged again on
/// every hit, so the budget sees exactly what it saw before the memo.
pub fn root_on(
    from: Vector3,
    to: Vector3,
    reading: &VertexFieldReading,
) -> Result<Vector3, WorkError> {
    let key = root_key(from, to);
    let remembered = reading.remembered_roots.borrow().get(&key).copied();
    if let Some((root, units)) = remembered {
        LevelLineWork::charge(units)?;
        return Ok(root);
    }

    let budget = LevelLineWork::current();
    let spent_before = budget.as_ref().map_or(0, |b| b.spent.get());
    let root = search_root_on(from, to, reading)?;
    let spent = budget.as_ref().map_or(0, |b| b.spent.get()) - spent_before;
    reading
        .remembered_roots
        .borrow_mut()
        .insert(key, (root, spent));
    Ok(root)
}

/// The root on an edge lying in a bounded end's plane, where the
/// chain's own value crosses 0, by halving - or `None` where that value does not change sign.
fn root_in_the_end_plane(
    from: Vector3,
    to: Vector3,
    reading: &VertexFieldReading,
) -> Result<Option<Vector3>, WorkError> {
    let value_from = reading.value_in_the_end_plane(from, from, to)?;
    let value_to = reading.value_in_the_end_plane(to, from, to)?;
    if (value_from < 0.0) == (value_to < 0.0) {
        return Ok(None);
    }

    let from_is_negative = value_from < 0.0;
    let mut lo = 0.0;
    let mut hi = 1.0;
    let mut step = 0;
    while step < 200 && hi - lo > BRACKET_TOLERANCE {
        let mid = 0.5 * (lo + hi);
        if (reading.value_in_the_end_plane(Vector3::lerp(from, to, mid), from, to)? < 0.0)
            == from_is_negative
        {
            lo = mid;
        } else {
            hi = mid;
        }
        step += 1;
    }

    Ok(Some(Vector3::lerp(from, to, 0.5 * (lo + hi))))
}

/// The bracketed search [`root_on`] remembers the answer of.
fn search_root_on(
    from: Vector3,
    to: Vector3,
    reading: &VertexFieldReading,
) -> Result<Vector3, WorkError> {
    if reading.edge_lies_in_a_known_cut(from, to) {
        if let Some(in_the_plane) = root_in_the_end_plane(from, to, reading)? {
            LevelLineWork::charge(1)?;
            return Ok(in_the_plane);
        }
    }

    FieldCounters::with(|t| t.roots += 1);
    LevelLineWork::charge(1)?;
    let value_from = reading.value_at_point(from)?;
    let value_to = reading.value_at_point(to)?;
    let mut lo = 0.0;
    let mut hi = 1.0;
    let span = (to - from).length();

    // Zero counts with the positives, except an end ON a known cut, which reads the clipped
    // field's 0 though it stands in the band: the known-cut reading gives it its vertex's side.
    let from_is_negative = value_from < 0.0
        || (value_from == 0.0
            && reading.has_known_cuts()
            && reading.known_cut_value_at(from)? < 0.0);
    let mut value_lo = value_from;
    let mut value_hi = value_to;
    let to_is_negative = value_to < 0.0
        || (value_to == 0.0 && reading.has_known_cuts() && reading.known_cut_value_at(to)? < 0.0);
    let bracketed = to_is_negative != from_is_negative;

    let mut step = 0;
    while step < BISECTION_STEPS && hi - lo > BRACKET_TOLERANCE {
        // A secant that would leave the bracket, or a bracket whose ends read the same value, is
        // no guess at all - and every fourth step is halved regardless, so the bracket cannot stall.
        let mut mid = 0.5 * (lo + hi);
        if bracketed && step % 4 != 3 && value_lo != value_hi {
            let secant = ((value_lo * hi) - (value_hi * lo)) / (value_lo - value_hi);
            let margin = 1e-3 * (hi - lo);
            if secant > lo + margin && secant < hi - margin {
                mid = secant;
            }
        }

        let value = reading.value_at_point(Vector3::lerp(from, to, mid))?;

        // The field is a distance, so a residual this far below the segment's length puts the
        // point on the line to more places than the module's answers are held to.
        if value.abs() <= BRACKET_TOLERANCE * span {
            return Ok(Vector3::lerp(from, to, mid));
        }

        if (value < 0.0) == from_is_negative {
            lo = mid;
            value_lo = value;
            value_hi *= 0.5;
        } else {
            hi = mid;
            value_hi = value;
            value_lo *= 0.5;
        }
        step += 1;
    }

    Ok(Vector3::lerp(from, to, 0.5 * (lo + hi)))
}
