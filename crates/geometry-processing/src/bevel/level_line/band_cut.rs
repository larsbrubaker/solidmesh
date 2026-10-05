//! The band cut's records (`Bevel/LevelLine/BandCut.cs`): one run of the level line through the
//! cut mesh ([`LevelLineChain`]) and what the cut leaves ([`BandCutResult`]). The cut itself (C#'s
//! static `BandCut`) lands with its method package as functions of this module.

/// C# `LevelLineChain`: one run of the level line through the cut mesh, as mesh vertices in order.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct LevelLineChain {
    /// The vertices of the run in order; a closed run does not repeat its first vertex at the end.
    pub vertices: Vec<i32>,
    /// True when the run comes back to where it started.
    pub closed: bool,
}

/// C# `BandCutResult`: what the cut leaves - the two halves of the mesh, and the line between
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
    /// C#'s constructor.
    pub fn new(vertices: Vec<i32>, closed: bool) -> Self {
        LevelLineChain { vertices, closed }
    }
}

impl BandCutResult {
    /// C#'s constructor.
    pub fn new(inside: Vec<i32>, outside: Vec<i32>, lines: Vec<LevelLineChain>) -> Self {
        BandCutResult {
            inside_triangles: inside,
            outside_triangles: outside,
            lines,
        }
    }
}

/// C# `BandCut.Chain`: the level line's crossed edges, as `(low, high)` vertex pairs, joined into
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

/// C# `BandCut.Neighbours`: `vertex`'s neighbour list, made on first ask.
pub fn neighbours(
    map: &mut std::collections::HashMap<i32, Vec<i32>>,
    vertex: i32,
) -> &mut Vec<i32> {
    map.entry(vertex).or_insert_with(|| Vec::with_capacity(2))
}

/// C# `BandCut.Key`: an edge's `(low, high)` vertex pair.
pub fn key(a: i32, b: i32) -> (i32, i32) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}
