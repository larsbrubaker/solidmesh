//! The boundary of a SMOOTH side - one the cut is laid on the surface for rather than drawn straight
//! off the chain (`Bevel/Sections/SectionSmoothSide.Boundary.cs`, part of the C# `internal static
//! partial class SectionSmoothSide`; a module of free functions here). What is here is the walk down
//! a crease to the chord an end section's landing lies on ([`try_the_crease_chord_under`]) and the
//! creases at each vertex it reads ([`the_creases_at_each_vertex`]); the boundary itself lands with a
//! later package.

use indexmap::IndexMap;
use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::Vector3;

use super::section_fan_walk::EdgeAtPair;

/// C# `LatticeFraction`: how near an end of a ring edge an inserted point may not be taken as a new
/// point.
pub const LATTICE_FRACTION: f64 = 1e-9;

/// C# `TheLongestCreaseWalk`: how many chords of a crease a slide may run down before the walk gives
/// it up.
const THE_LONGEST_CREASE_WALK: i32 = 64;

/// The FEATURE edges leaving each vertex.
pub type CreasesAtEachVertex = IndexMap<i32, Vec<i32>>;

/// C# `TryTheCreaseChordUnder`: which CHORD of a crease an end section's landing (`at`) lies on,
/// walked for from the chain's end vertex - `Ok` where a crease leaving `from_vertex` holds it, and
/// `Err(first_chord)` (C#'s false with the chord left at the first one) where none does.
///
/// `SectionEnd` records only the crease's first chord leaving that vertex, and on a tessellated wall
/// that chord is far shorter than the slide: the raked bore's mouth starts with a chord 0.1214 long
/// and a 2 mm radius slides 0.2346, so the landing is two or three chords down. Several creases
/// generally leave an end vertex, and where the named one does not hold the landing, the others are
/// walked in turn; the one the landing stands nearest (lowest chord on a tie) wins.
#[allow(clippy::too_many_arguments)]
pub fn try_the_crease_chord_under(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
    first_chord: i32,
    from_vertex: i32,
    at: Vector3,
) -> Result<i32, i32> {
    let mut chord = -1;
    let mut nearest = f64::MAX;
    let mut creases = vec![first_chord];

    let at_each_vertex = the_creases_at_each_vertex(graph, edge_at_pair, is_a_feature_edge);

    if let Some(also_leaving) = at_each_vertex.get(&from_vertex) {
        creases.extend(
            also_leaving
                .iter()
                .copied()
                .filter(|&other| other != first_chord),
        );
    }

    for crease in creases {
        if let Some((found, off)) =
            try_the_crease_chord_down(mesh, graph, &at_each_vertex, crease, from_vertex, at)
        {
            if off < nearest || (off == nearest && found < chord) {
                nearest = off;
                chord = found;
            }
        }
    }

    if chord < 0 {
        return Err(first_chord);
    }

    Ok(chord)
}

/// C# `TryTheCreaseChordDown`: the walk itself, down ONE crease leaving the vertex, to the first chord
/// whose span holds the landing's projection - and how far the landing stands OFF that chord, which
/// is what tells one crease leaving the vertex from another. At each far vertex the walk takes the
/// feature edge that most nearly carries on the way it is going (the first such on a tie), and gives
/// up where nothing carries on.
///
/// C# builds the creases-at-each-vertex table lazily inside the walk; it is handed in here, built
/// once by the caller - the same table either way.
pub fn try_the_crease_chord_down(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    crease_at_vertex: &CreasesAtEachVertex,
    first_chord: i32,
    from_vertex: i32,
    at: Vector3,
) -> Option<(i32, f64)> {
    let mut chord = first_chord;
    let mut from = from_vertex;

    for _ in 0..THE_LONGEST_CREASE_WALK {
        let a = position(mesh, from);
        let far = other_end(graph, chord, from);
        let along = position(mesh, far) - a;

        if along.length_squared() < 1e-300 {
            return None;
        }

        let how_far = (at - a).dot(along) / along.length_squared();

        if how_far <= 1.0 {
            return Some((chord, (at - a - (along * how_far)).length()));
        }

        let leaving = crease_at_vertex.get(&far)?;

        let mut carries_on = -1;
        let mut straightest = 0.0;
        let way = along.get_normal();

        for &next in leaving {
            if next == chord {
                continue;
            }

            let other = other_end(graph, next, far);
            let direction = (position(mesh, other) - position(mesh, far)).get_normal();
            let straight = direction.dot(way);

            if carries_on < 0 || straight > straightest {
                carries_on = next;
                straightest = straight;
            }
        }

        if carries_on < 0 || straightest <= 0.0 {
            // Nothing leaves that vertex going on the way the crease was going, so the crease ENDS
            // there and the landing is past the end of it - which is not a slide this can place.
            return None;
        }

        chord = carries_on;
        from = far;
    }

    None
}

/// C# `TheCreasesAtEachVertex`: the FEATURE edges leaving each vertex, read off the edges-by-pair
/// table the operation already built. Nothing where there is no feature test.
///
/// C# walks its `Dictionary<long, int>`'s values in insertion order, which is edge-index order (the
/// table is filled edge by edge); the Rust table is a `HashMap`, so the edges are walked in ascending
/// index instead, which is the same order wherever no two edges share a vertex pair.
pub fn the_creases_at_each_vertex(
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
) -> CreasesAtEachVertex {
    let mut at: CreasesAtEachVertex = IndexMap::new();

    let Some(is_a_feature_edge) = is_a_feature_edge else {
        return at;
    };

    let mut edges: Vec<i32> = edge_at_pair.values().copied().collect();
    edges.sort_unstable();

    for edge in edges {
        if !is_a_feature_edge(edge) {
            continue;
        }

        for vertex in [graph.get_vertex0(edge), graph.get_vertex1(edge)] {
            let leaving = at.entry(vertex).or_default();

            if !leaving.contains(&edge) {
                leaving.push(edge);
            }
        }
    }

    at
}

/// The end of `edge` that is not `vertex`.
fn other_end(graph: &MeshEdgeGraph, edge: i32, vertex: i32) -> i32 {
    if graph.get_vertex0(edge) == vertex {
        graph.get_vertex1(edge)
    } else {
        graph.get_vertex0(edge)
    }
}

/// A vertex's position, read as doubles.
fn position(mesh: &Mesh, vertex: i32) -> Vector3 {
    Vector3::from_vector3_float(mesh.vertices[vertex as usize])
}
