//! The section at the END of an open run (`Bevel/Sections/SectionEnd.cs`, a C# `internal static
//! class`, so a module here): what is read off the part at one such end ([`EndReading`],
//! [`try_read`], [`can_end_at`]) - the transverse face the run dies on and the two creases its
//! sides slide down - and the walk of a profile end down its crease
//! ([`walk_the_boundary_onto_the_crease`]). The section built from the reading lands with a later
//! package.

use std::collections::{HashMap, HashSet};

use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::{math_helper, Vector3};

use super::section_fan_walk::{self, EdgeAtPair};
use super::{NanSectionPoint, SectionPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};
use crate::topology::{ChainEdge, FaceGroup, FeatureEdge};

/// C# `SectionEnd.EndReading`: what `TryRead` reads off the part at one end of an open run.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct EndReading {
    /// The chain's end vertex.
    pub vertex: i32,
    /// Where that vertex stands.
    pub station: Vector3,
    /// The unit direction the chain runs INTO the end - the axis the profile slides along. At the
    /// run's start that is the leaving edge read backwards.
    pub along: Vector3,
    /// The unit normal of the face on the left of the walk.
    pub left: Vector3,
    /// The unit normal of the face on the right of the walk.
    pub right: Vector3,
    /// The unit normal of the transverse face the run dies on.
    pub transverse: Vector3,
    /// `along . transverse`: how steeply the chain runs into the transverse face.
    pub lean: f64,
    /// `left . right`: the cosine between the two side normals.
    pub cosine: f64,
    /// The crease the left boundary rides: the mesh edge the transverse face shares with the left
    /// side.
    pub left_slide: i32,
    /// The crease the right boundary rides.
    pub right_slide: i32,
}

/// How near two unit normals have to stand to be one SURFACE: the curve tolerance's cosine.
pub fn same_surface_dot() -> f64 {
    math_helper::degrees_to_radians(FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES).cos()
}

/// How near two unit normals have to stand to be one FLAT region.
pub const ONE_FLAT_REGION_DOT: f64 = 1.0 - 1e-6;

/// The least lean of the chain into the transverse face that still reaches it.
pub const SMALLEST_LEAN: f64 = 1e-9;

/// How many crease edges a profile end may walk down before the walk is read as going nowhere.
pub const THE_LONGEST_CREASE_WALK: i32 = 64;

impl EndReading {
    /// C#'s constructor.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        vertex: i32,
        station: Vector3,
        along: Vector3,
        left: Vector3,
        right: Vector3,
        transverse: Vector3,
        lean: f64,
        cosine: f64,
        left_slide: i32,
        right_slide: i32,
    ) -> Self {
        EndReading {
            vertex,
            station,
            along,
            left,
            right,
            transverse,
            lean,
            cosine,
            left_slide,
            right_slide,
        }
    }
}

/// Whether the run can END at `edge`'s start (or end) by sliding its profile down to the face it
/// dies on; the refusal says why not. `edge_at_pair` defaults to the graph's own index.
pub fn can_end_at(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge: &ChainEdge,
    at_the_start: bool,
    edge_at_pair: Option<&EdgeAtPair>,
) -> Result<(), BevelAdvisory> {
    let owned;
    let edge_at_pair = match edge_at_pair {
        Some(map) => map,
        None => {
            owned = section_fan_walk::edges_by_vertex_pair(graph);
            &owned
        }
    };

    try_read(edge, at_the_start, mesh, graph, edge_at_pair).map(|_| ())
}

/// Reads the end of the run at `edge`'s start (or end): the whole one-ring at the end vertex, the
/// transverse face beyond it, how the chain leans into that face, and the crease each side's
/// boundary rides. Refused where the side beyond bends, the chain runs along it, the two sides are
/// one plane or fold flat back, or either side has no crease of its own to slide down.
pub fn try_read(
    edge: &ChainEdge,
    at_the_start: bool,
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
) -> Result<EndReading, BevelAdvisory> {
    let vertex = if at_the_start {
        edge.from_vertex()
    } else {
        edge.to_vertex()
    };
    let station = if at_the_start { edge.from() } else { edge.to() };
    let refused = || {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceStripEndNotSupported,
            vec![station.into()],
        )
    };

    // The direction the chain runs INTO the end, which is the axis the profile slides along. At
    // the run's start that is the leaving edge read backwards.
    let mut along = edge.direction().get_normal();

    if at_the_start {
        along = -along;
    }

    let left = edge.left_normal().get_normal();
    let right = edge.right_normal().get_normal();

    // The WHOLE one-ring at the end vertex, turned as the fan walk turns it: the chain edge read
    // as both the arriving and the leaving edge walks all the way round back to itself.
    let walk =
        section_fan_walk::try_walk(mesh, graph, edge_at_pair, vertex, edge, edge, true, station)?;
    let ring = walk.faces;
    let mut spokes = walk.crossed;

    spokes.push(edge.edge.mesh_edge_index);

    let transverse = try_the_transverse_plane(&ring, left, right, mesh).ok_or_else(refused)?;

    let lean = along.dot(transverse);

    if lean.abs() < SMALLEST_LEAN {
        // The chain runs ALONG the transverse face, so sliding down it never reaches the plane.
        return Err(refused());
    }

    let cosine = left.dot(right);

    if cosine > FeatureEdge::COPLANAR_FACE_DOT || cosine < -0.999999 {
        return Err(refused());
    }

    // Which crease each side's boundary rides: the one the transverse face shares with that side.
    let left_slide = the_crease_between(&spokes, left, transverse, mesh, graph);
    let right_slide = the_crease_between(&spokes, right, transverse, mesh, graph);

    if left_slide < 0 || right_slide < 0 || left_slide == right_slide {
        // Without both creases the faces on either side have nothing to carry their moved end
        // along.
        return Err(refused());
    }

    Ok(EndReading::new(
        vertex,
        station,
        along,
        left,
        right,
        transverse,
        lean,
        cosine,
        left_slide,
        right_slide,
    ))
}

/// Carries profile end `profile[end]` - slid `distance` from the `station` - down the crease from
/// `vertex` along `first_chord` instead, the crease being the transverse region's fold, so the
/// boundary stays on the part. The vertex index `at_the_vertex` is built on first need. Only a
/// NaN landing fails (the pool refuses it).
#[allow(clippy::too_many_arguments)]
pub fn walk_the_boundary_onto_the_crease(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    pool: &mut SectionPool,
    station: Vector3,
    transverse: Vector3,
    first_chord: i32,
    vertex: i32,
    profile: &mut [i32],
    end: usize,
    at_the_vertex: &mut Option<HashMap<i32, Vec<i32>>>,
) -> Result<(), NanSectionPoint> {
    let slid = pool.position_of(profile[end]);
    let distance = (slid - station).length();

    if distance < 1e-12 {
        return Ok(());
    }

    let at_the_vertex =
        at_the_vertex.get_or_insert_with(|| the_edges_at_each_vertex(graph, edge_at_pair));

    let Some(landing) = try_walk_down_the_crease(
        mesh,
        graph,
        transverse,
        at_the_vertex,
        first_chord,
        vertex,
        distance,
    ) else {
        return Ok(());
    };
    if (landing - slid).length() < 1e-9 {
        return Ok(());
    }

    profile[end] = pool.add(landing)?;
    Ok(())
}

/// Where a walk of `distance` down the crease from `vertex` along `first_chord` lands, carrying
/// on at each crease vertex along the straightest edge of the transverse region's fold; `None`
/// where the crease ends (or forks back) before the walk does.
pub fn try_walk_down_the_crease(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    transverse: Vector3,
    at_the_vertex: &HashMap<i32, Vec<i32>>,
    first_chord: i32,
    vertex: i32,
    distance: f64,
) -> Option<Vector3> {
    let mut chord = first_chord;
    let mut from = vertex;
    let mut left = distance;

    for _ in 0..THE_LONGEST_CREASE_WALK {
        let a = Vector3::from_vector3_float(mesh.vertices[from as usize]);
        let far = if graph.get_vertex0(chord) == from {
            graph.get_vertex1(chord)
        } else {
            graph.get_vertex0(chord)
        };
        let along = Vector3::from_vector3_float(mesh.vertices[far as usize]) - a;
        let span = along.length();

        if span < 1e-300 {
            return None;
        }

        if left <= span {
            return Some(a + (along * (left / span)));
        }

        left -= span;

        let leaving = at_the_vertex.get(&far)?;

        let mut carries_on = -1;
        let mut straightest = 0.0;
        let way = along / span;

        for &next in leaving {
            if next == chord || !is_the_regions_fold(next, transverse, mesh, graph) {
                continue;
            }

            let other = if graph.get_vertex0(next) == far {
                graph.get_vertex1(next)
            } else {
                graph.get_vertex0(next)
            };
            let direction = (Vector3::from_vector3_float(mesh.vertices[other as usize])
                - Vector3::from_vector3_float(mesh.vertices[far as usize]))
            .get_normal();
            let straight = direction.dot(way);

            if carries_on < 0 || straight > straightest {
                carries_on = next;
                straightest = straight;
            }
        }

        if carries_on < 0 || straightest <= 0.0 {
            // The crease ENDS there and the landing is past the end of it, which is not a slide.
            return None;
        }

        chord = carries_on;
        from = far;
    }

    None
}

/// Whether `edge` is the transverse region's FOLD: two faces, exactly one of them on the
/// transverse surface.
pub fn is_the_regions_fold(
    edge: i32,
    transverse: Vector3,
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
) -> bool {
    if graph.get_face_count(edge) != 2 {
        return false;
    }

    let on_it = |offset: i32| {
        Vector3::from_vector3_float(mesh.faces[graph.get_face(edge, offset) as usize].normal)
            .get_normal()
            .dot(transverse)
            > same_surface_dot()
    };

    on_it(0) != on_it(1)
}

/// The graph edges at each vertex, over the edges `edge_at_pair` names, each listed once. Taken
/// in the order C#'s `Dictionary.Values` gives them: each vertex pair at its first edge's place,
/// carrying the edge the index kept for it.
pub fn the_edges_at_each_vertex(
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
) -> HashMap<i32, Vec<i32>> {
    let mut at: HashMap<i32, Vec<i32>> = HashMap::new();
    let mut seen: HashSet<i64> = HashSet::new();

    for index in 0..graph.edge_count() {
        let key = section_fan_walk::pair_key(graph.get_vertex0(index), graph.get_vertex1(index));
        if !seen.insert(key) {
            continue;
        }
        let Some(&edge) = edge_at_pair.get(&key) else {
            continue;
        };

        for vertex in [graph.get_vertex0(edge), graph.get_vertex1(edge)] {
            let leaving = at.entry(vertex).or_default();

            if !leaving.contains(&edge) {
                leaving.push(edge);
            }
        }
    }

    at
}

/// `point` slid along `along` onto the plane through `station` facing `normal`, where the chain
/// leans `lean` into it.
pub fn slid_onto_the_plane(
    point: Vector3,
    station: Vector3,
    normal: Vector3,
    along: Vector3,
    lean: f64,
) -> Vector3 {
    point - (along * ((point - station).dot(normal) / lean))
}

/// The ONE flat face beyond the end in the one-`ring`: every ring face that is neither side, all
/// in one plane; `None` where there is none, or the side beyond bends (a chain that dies on a
/// curved wall is the next family).
pub fn try_the_transverse_plane(
    ring: &[i32],
    left: Vector3,
    right: Vector3,
    mesh: &Mesh,
) -> Option<Vector3> {
    let mut transverse: Option<Vector3> = None;

    for &face in ring {
        let normal = Vector3::from_vector3_float(mesh.faces[face as usize].normal).get_normal();

        if normal.dot(left) > same_surface_dot() || normal.dot(right) > same_surface_dot() {
            continue;
        }

        match transverse {
            None => transverse = Some(normal),
            Some(found) if normal.dot(found) < ONE_FLAT_REGION_DOT => return None,
            Some(_) => {}
        }
    }

    transverse
}

/// The ONE spoke that joins the `side` surface to the `transverse` one, or -1 where none does or
/// more than one does.
pub fn the_crease_between(
    spokes: &[i32],
    side: Vector3,
    transverse: Vector3,
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
) -> i32 {
    let mut found = -1;

    for &edge in spokes {
        if graph.get_face_count(edge) != 2 {
            continue;
        }

        let normal_of = |offset: i32| {
            Vector3::from_vector3_float(mesh.faces[graph.get_face(edge, offset) as usize].normal)
                .get_normal()
        };
        let one = normal_of(0);
        let other = normal_of(1);

        let joins_them = (one.dot(side) > same_surface_dot()
            && other.dot(transverse) > same_surface_dot())
            || (other.dot(side) > same_surface_dot() && one.dot(transverse) > same_surface_dot());

        if !joins_them {
            continue;
        }

        if found >= 0 {
            return -1;
        }

        found = edge;
    }

    found
}
