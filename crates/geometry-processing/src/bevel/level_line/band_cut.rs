//! The band cut's records: one run of the level line through the cut mesh ([`LevelLineChain`])
//! and what the cut leaves ([`BandCutResult`]), and the cut itself ([`cut`],
//! [`put_the_root_on_the_line`], [`side_of_triangle`]).
//!
//! The mesh the cut edits is always the reading's own, so the cut takes the reading and edits its
//! mesh, borrowing it only between the reading's reads.

/// One run of the level line through the cut mesh, as mesh vertices in order.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct LevelLineChain {
    /// The vertices of the run in order; a closed run does not repeat its first vertex at the end.
    pub vertices: Vec<i32>,
    /// True when the run comes back to where it started.
    pub closed: bool,
}

/// What the cut leaves - the two halves of the mesh, and the line between
/// them.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct BandCutResult {
    /// The triangles of the band that is to be deleted.
    pub inside_triangles: Vec<i32>,
    /// The triangles that stay.
    pub outside_triangles: Vec<i32>,
    /// The level line, chained into closed loops and open polylines. Empty when nothing is
    /// crossed.
    pub lines: Vec<LevelLineChain>,
}

impl LevelLineChain {
    /// A run of the given mesh vertices, closed or open.
    pub fn new(vertices: Vec<i32>, closed: bool) -> Self {
        LevelLineChain { vertices, closed }
    }
}

impl BandCutResult {
    /// The cut's two halves and the line between them.
    pub fn new(inside: Vec<i32>, outside: Vec<i32>, lines: Vec<LevelLineChain>) -> Self {
        BandCutResult {
            inside_triangles: inside,
            outside_triangles: outside,
            lines,
        }
    }
}

/// The level line's crossed edges, as `(low, high)` vertex pairs, joined into
/// chains. Each vertex's neighbours are sorted and the walk starts from the ends first, then in
/// vertex order, so only the edge set reaches the chains - never a map's own key order.
pub fn chain(edges: &[(i32, i32)]) -> Vec<LevelLineChain> {
    let mut chains = Vec::new();

    if edges.is_empty() {
        return chains;
    }

    let mut neighbour_map: std::collections::HashMap<i32, Vec<i32>> =
        std::collections::HashMap::new();

    for &(low, high) in edges {
        neighbours(&mut neighbour_map, low).push(high);
        neighbours(&mut neighbour_map, high).push(low);
    }

    for list in neighbour_map.values_mut() {
        list.sort();
    }

    let mut starts: Vec<i32> = neighbour_map.keys().copied().collect();
    // Ends first, so an open run is walked from its end and comes out as one chain rather than two.
    starts.sort_by_key(|v| (if neighbour_map[v].len() == 1 { 0 } else { 1 }, *v));

    let mut used = std::collections::HashSet::new();

    for start in starts {
        for first in neighbour_map[&start].clone() {
            if used.contains(&key(start, first)) {
                continue;
            }

            let mut run = vec![start];
            let mut previous = start;
            let mut current = first;
            used.insert(key(previous, current));
            let mut closed = false;

            loop {
                run.push(current);

                if current == start {
                    closed = true;
                    run.pop();
                    break;
                }

                let next = neighbour_map[&current]
                    .iter()
                    .copied()
                    .find(|&n| n != previous && !used.contains(&key(current, n)))
                    .unwrap_or(-1);

                if next < 0 {
                    break;
                }

                used.insert(key(current, next));
                previous = current;
                current = next;
            }

            chains.push(LevelLineChain::new(run, closed));
        }
    }

    chains
}

/// `vertex`'s neighbour list, made on first ask.
pub fn neighbours(
    map: &mut std::collections::HashMap<i32, Vec<i32>>,
    vertex: i32,
) -> &mut Vec<i32> {
    map.entry(vertex).or_insert_with(|| Vec::with_capacity(2))
}

/// An edge's `(low, high)` vertex pair.
pub fn key(a: i32, b: i32) -> (i32, i32) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

/// Why the cut could not be taken.
#[derive(Debug, Clone)]
pub enum BandCutError {
    /// The work budget ran out, the build was cancelled, or the mesh refused a split.
    Mesh(super::LevelLineMeshError),
    /// A root's edge is no longer in the mesh.
    RootEdgeGone(String),
    /// A triangle still straddles the line after the cut, with the message saying which.
    Straddles(String),
}

impl From<super::LevelLineMeshError> for BandCutError {
    fn from(error: super::LevelLineMeshError) -> Self {
        BandCutError::Mesh(error)
    }
}

impl From<super::WorkError> for BandCutError {
    fn from(error: super::WorkError) -> Self {
        BandCutError::Mesh(error.into())
    }
}

impl std::fmt::Display for BandCutError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BandCutError::Mesh(error) => write!(f, "{error}"),
            BandCutError::RootEdgeGone(message) | BandCutError::Straddles(message) => {
                f.write_str(message)
            }
        }
    }
}

impl std::error::Error for BandCutError {}

/// Splits an edge at a root and reads the vertex there onto the
/// line, leaving the edge with a vertex on it however the mesh answered. An INTERIOR vertex the
/// mesh already carried (a T-vertex through a needle) divided nothing, so the edge's triangles are
/// re-tiled through it: nothing is minted and no position moves.
pub fn put_the_root_on_the_line(
    reading: &super::VertexFieldReading,
    a: i32,
    b: i32,
    point: vector_math::Vector3,
) -> Result<(), super::LevelLineMeshError> {
    // Asked as "is the answer a vertex that was already there": a new vertex is always the next id.
    // A root in a stopped end's plane goes onto the plane exactly.
    let ends = {
        let mesh = reading.mesh.borrow();
        (mesh.position(a), mesh.position(b))
    };
    let point = super::stopped_plane_points::edge_root_onto_a_stopped_plane(reading, ends, point)?;
    let (before, mid) = {
        let mut mesh = reading.mesh.borrow_mut();
        let before = mesh.vertex_count();
        (before, mesh.split_edge(a, b, point, true)?)
    };
    reading.put_on_the_line(mid)?;
    if mid < before && mid != a && mid != b {
        reading.mesh.borrow_mut().retile_through(a, b, mid);
    }

    Ok(())
}

/// Splits every root into the reading's mesh and reads off the partition and the
/// line, threaded from the mesh's canonical edges.
pub fn cut(
    reading: &super::VertexFieldReading,
    roots: &[super::EdgeRoot],
) -> Result<BandCutResult, BandCutError> {
    for root in roots {
        if !reading.mesh.borrow().has_edge(root.a, root.b) {
            return Err(BandCutError::RootEdgeGone(format!(
                "the root's edge {}-{} is no longer in the mesh (Parameter 'roots')",
                root.a, root.b
            )));
        }

        put_the_root_on_the_line(reading, root.a, root.b, root.point)?;
    }

    let triangle_count = reading.mesh.borrow().triangle_count();
    let mut inside = Vec::new();
    let mut outside = Vec::new();
    for triangle in 0..triangle_count {
        // A triangle out of the mesh is on neither side.
        if reading.mesh.borrow().is_removed(triangle) {
            continue;
        }

        if side_of_triangle(reading, triangle)? == super::VertexSide::Inside {
            inside.push(triangle);
        } else {
            outside.push(triangle);
        }
    }

    let mut on_inside = vec![false; triangle_count as usize];
    for &t in &inside {
        on_inside[t as usize] = true;
    }

    // Canonical by the mesh's edges, so the line's edges - and the chains threaded from them - read
    // the same whatever splitting went before.
    let mesh = reading.mesh.borrow();
    let line_edges: Vec<(i32, i32)> = mesh
        .edges()
        .into_iter()
        .filter(|&(low, high)| {
            let standing = mesh.triangles_on(low, high);
            standing.len() == 2
                && on_inside[standing[0] as usize] != on_inside[standing[1] as usize]
        })
        .collect();

    Ok(BandCutResult::new(inside, outside, chain(&line_edges)))
}

/// Which half a triangle belongs to. Any inside corner puts it inside,
/// any outside corner outside (after the cut it cannot have both); one entirely on the line is
/// placed by its centroid - on a known cut, by the chain's own value there.
pub fn side_of_triangle(
    reading: &super::VertexFieldReading,
    triangle: i32,
) -> Result<super::VertexSide, BandCutError> {
    let t = reading.mesh.borrow().triangles()[triangle as usize];
    let mut saw_inside = false;
    let mut saw_outside = false;
    for corner in 0..3 {
        match reading.side_of(t.corner(corner))? {
            super::VertexSide::Inside => saw_inside = true,
            super::VertexSide::Outside => saw_outside = true,
            super::VertexSide::On => {}
        }
    }

    // Inside corners all ON a known-cut plane and outside corners all PAST it: the triangle stands
    // on the plane's far side, outside, its inside corners only touching the boundary.
    if saw_inside && saw_outside && stands_past_a_known_cut(reading, &t)? {
        return Ok(super::VertexSide::Outside);
    }

    if saw_inside && saw_outside {
        return Err(BandCutError::Straddles(format!(
            "triangle {triangle} still straddles the line after the cut - a root was missed"
        )));
    }

    if saw_inside {
        return Ok(super::VertexSide::Inside);
    }

    if saw_outside {
        return Ok(super::VertexSide::Outside);
    }

    let centroid = reading.mesh.borrow().centroid(triangle);
    let value = if reading.has_known_cuts() {
        reading.known_cut_value_at(centroid)?
    } else {
        reading.value_at_point(centroid)?
    };
    Ok(if value < 0.0 {
        super::VertexSide::Inside
    } else {
        super::VertexSide::Outside
    })
}

/// Whether a triangle's inside corners all stand ON one known-cut plane of one chain, inside by
/// that chain's own value, and its outside corners all PAST that same plane (`known_cut_trace.rs`).
fn stands_past_a_known_cut(
    reading: &super::VertexFieldReading,
    t: &super::LevelLineTriangle,
) -> Result<bool, BandCutError> {
    let mut insides = Vec::new();
    let mut outsides = Vec::new();
    for corner in 0..3 {
        let vertex = t.corner(corner);
        let position = reading.mesh.borrow().position(vertex);
        match reading.side_of(vertex)? {
            super::VertexSide::Inside => insides.push(position),
            super::VertexSide::Outside => outsides.push(position),
            super::VertexSide::On => {}
        }
    }

    Ok(reading.leaves_across_a_known_cut(&insides, &outsides)?)
}
