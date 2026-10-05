//! Laying a straight reach ALONG the surface, unfolding across the edges it crosses
//! (`Bevel/Sections/SectionSurfaceWalk.cs`): why a walk stopped short ([`SurfaceWalkRefusal`]),
//! where it crossed one mesh edge ([`SurfaceWalkCrossing`]) and what it came to
//! ([`SurfaceWalkResult`]), with the walk's small readings (the mesh's grid, folds, feet along an
//! edge, a refusal). The walk itself (`TryWalk`) and its face-holding readings land with later runs
//! of this package.

use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::{cs_math, Vector3};

use crate::bevel::float_grid;
use crate::bevel::surface::SurfaceVertexPool;
use crate::topology::FeatureEdge;

/// C# `SurfaceWalkRefusal`: why a surface walk stopped short of what was asked of it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum SurfaceWalkRefusal {
    /// It did not.
    #[default]
    None = 0,
    /// The reach met a fold the caller asked it to stop at.
    CreaseReached,
    /// The reach met an edge of the surface - nothing to cross into.
    BoundaryReached,
    /// No face of a vertex's fan held the unfolded direction leaving that vertex.
    NoFaceHoldsTheDirection,
    /// The two ends of a join are not joined through non-crease edges within the strip searched.
    FacesNotConnected,
    /// The straight unfolded line between two ends left the strip of faces between them.
    JoinLeftTheStrip,
    /// The walk was asked for from a point or in a direction that names no ray at all.
    NothingToWalk,
}

/// C# `SurfaceWalkCrossing`: where a walk crossed one mesh edge.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SurfaceWalkCrossing {
    /// The mesh edge crossed.
    pub edge: i32,
    /// Where along that edge, 0 at the graph's vertex 0 and 1 at its vertex 1.
    pub parameter: f64,
    /// The crossing point, which lies in the planes of BOTH the faces the edge joins.
    pub at: Vector3,
}

/// C# `SurfaceWalkResult`: what a walk laid on the surface came to. `Default` is C#'s field
/// initializers: no end edge or vertex (-1), no refusal.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceWalkResult {
    /// Where the walk ended, on [`SurfaceWalkResult::end_face`].
    pub end: Vector3,
    /// The face the end point lies in.
    pub end_face: i32,
    /// The direction the walk was travelling when it ended, in the end face's plane.
    pub end_direction: Vector3,
    /// The mesh edges crossed on the way, in the order they were crossed.
    pub crossings: Vec<SurfaceWalkCrossing>,
    /// How far the walk actually went - the distance asked for, unless it was refused.
    pub travelled: f64,
    /// Why it stopped short, where it did.
    pub refusal: SurfaceWalkRefusal,
    /// The mesh edge [`SurfaceWalkResult::end`] lies ON, or -1 where the end lies strictly inside
    /// [`SurfaceWalkResult::end_face`]. A walk that stops on an edge KNOWS it did - refused at a
    /// crease or a boundary, or out of distance exactly at a crossing - without any position
    /// being compared, which is what lets a walked boundary carry an EDGE as its place.
    pub end_edge: i32,
    /// Where along [`SurfaceWalkResult::end_edge`], 0 at the graph's vertex 0 and 1 at its
    /// vertex 1.
    pub end_parameter: f64,
    /// The mesh VERTEX [`SurfaceWalkResult::end`] stands on, or -1 where it stands on no corner.
    /// A walk that runs out at a corner stands on every edge of that vertex's fan at once, so the
    /// vertex - topological, and incident to every fold meeting there - is the one reading that
    /// does not move with the part's placement.
    pub end_vertex: i32,
}

impl Default for SurfaceWalkResult {
    fn default() -> Self {
        Self {
            end: Vector3::default(),
            end_face: 0,
            end_direction: Vector3::default(),
            crossings: Vec::new(),
            travelled: 0.0,
            refusal: SurfaceWalkRefusal::None,
            end_edge: -1,
            end_parameter: 0.0,
            end_vertex: -1,
        }
    }
}

impl SurfaceWalkCrossing {
    /// C#'s `SurfaceWalkCrossing` constructor.
    pub fn new(edge: i32, parameter: f64, at: Vector3) -> Self {
        Self {
            edge,
            parameter,
            at,
        }
    }
}

impl SurfaceWalkResult {
    /// C#'s `SurfaceWalkResult` constructor, with its `init` properties at their initializers (`<init>`:
    /// no end edge or vertex, nothing travelled, no refusal).
    pub fn new(
        end: Vector3,
        end_face: i32,
        end_direction: Vector3,
        crossings: Vec<SurfaceWalkCrossing>,
    ) -> Self {
        Self {
            end,
            end_face,
            end_direction,
            crossings,
            ..Self::default()
        }
    }
}

/// C# `LatticeFraction`: the fraction of a reach under which two points are one, and a crossing
/// parameter within which of an end of its edge is snapped to that end.
pub const LATTICE_FRACTION: f64 = 1e-9;

/// C# `AlongAnArm`: how nearly a direction has to run along a face's arm to be read as running along
/// it.
pub const ALONG_AN_ARM: f64 = 1e-6;

/// C# `GridUlps`: how many of the mesh's own float steps its grid is read as.
const GRID_ULPS: f64 = 4.0;

/// C# `JoinIterations`: how many times the join re-aims before it gives up.
pub const JOIN_ITERATIONS: i32 = 8;

/// C# `GridOf`: the mesh's own grid - the larger of the part's weld distance and a few float steps
/// at its largest coordinate - which a walk's landing on a corner or an edge is read to.
pub fn grid_of(mesh: &Mesh) -> f64 {
    let mut largest = 0.0f32;

    for at in &mesh.vertices {
        largest = largest.max(at.x.abs().max(at.y.abs().max(at.z.abs())));
    }

    let ulp = float_grid::step_at(largest);

    cs_math::max_f64(
        SurfaceVertexPool::weld_distance_for(&mesh.vertices),
        GRID_ULPS * ulp,
    )
}

/// C# `IsTheSamePoint`: whether two points are one, to [`LATTICE_FRACTION`] of a reach of `over`
/// (at least one).
pub fn is_the_same_point(one: Vector3, other: Vector3, over: f64) -> bool {
    (one - other).length() <= LATTICE_FRACTION * cs_math::max_f64(1.0, over)
}

/// C# `FootOn`: the point `parameter` of the way along `edge` from the graph's vertex 0.
pub fn foot_on(mesh: &Mesh, graph: &MeshEdgeGraph, edge: i32, parameter: f64) -> Vector3 {
    let from = position(mesh, graph.get_vertex0(edge));

    from + ((position(mesh, graph.get_vertex1(edge)) - from) * parameter)
}

/// C# `AlongFrom`: how far along `edge` (from the graph's vertex 0) the point `at` projects,
/// clamped to the edge; zero on an edge of no length.
pub fn along_from(mesh: &Mesh, graph: &MeshEdgeGraph, edge: i32, at: Vector3) -> f64 {
    let from = position(mesh, graph.get_vertex0(edge));
    let along = position(mesh, graph.get_vertex1(edge)) - from;

    if along.length_squared() < 1e-300 {
        0.0
    } else {
        ((at - from).dot(along) / along.length_squared()).clamp(0.0, 1.0)
    }
}

/// C# `Refused`: a walk that stopped short, where it stopped and why - on `stopped_on_edge` at
/// `stopped_at_parameter` where it stopped on an edge (C# defaults -1 and 0).
#[allow(clippy::too_many_arguments)]
pub fn refused(
    at: Vector3,
    face: i32,
    direction: Vector3,
    crossings: Vec<SurfaceWalkCrossing>,
    travelled: f64,
    why: SurfaceWalkRefusal,
    stopped_on_edge: i32,
    stopped_at_parameter: f64,
) -> SurfaceWalkResult {
    SurfaceWalkResult {
        travelled,
        refusal: why,
        end_edge: stopped_on_edge,
        end_parameter: stopped_at_parameter,
        ..SurfaceWalkResult::new(at, face, direction, crossings)
    }
}

/// C# `NormalOf`: a face's unit normal, read off its corners rather than its stored normal.
pub fn normal_of(mesh: &Mesh, face: i32) -> Vector3 {
    let drawn = &mesh.faces[face as usize];
    let a = position(mesh, drawn.v0);

    (position(mesh, drawn.v1) - a)
        .cross(position(mesh, drawn.v2) - a)
        .get_normal()
}

/// C# `CrossedAFold`: whether any of a walk's crossings is a fold.
pub fn crossed_a_fold(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    crossings: Option<&[SurfaceWalkCrossing]>,
) -> bool {
    crossings.is_some_and(|crossings| {
        crossings
            .iter()
            .any(|crossing| is_a_fold(mesh, graph, crossing.edge))
    })
}

/// C# `IsAFold`: whether an edge is a fold of the surface - not two faces, or two whose normals are
/// further apart than the topology's coplanar reading.
pub fn is_a_fold(mesh: &Mesh, graph: &MeshEdgeGraph, edge: i32) -> bool {
    if graph.get_face_count(edge) != 2 {
        return true;
    }

    normal_of(mesh, graph.get_face(edge, 0)).dot(normal_of(mesh, graph.get_face(edge, 1)))
        < FeatureEdge::COPLANAR_FACE_DOT
}

/// C# `ParameterOn`: a parameter `along` an edge measured from `from`, turned to the graph's own
/// direction (from its vertex 0).
pub fn parameter_on(graph: &MeshEdgeGraph, edge: i32, from: i32, along: f64) -> f64 {
    if graph.get_vertex0(edge) == from {
        along
    } else {
        1.0 - along
    }
}

/// C# `PairKey`: the key a mesh edge is filed under by its two vertices (the same key
/// `SectionFanWalk.EdgesByVertexPair` uses).
pub fn pair_key(one: i32, other: i32) -> i64 {
    if one < other {
        ((one as i64) << 32) | (other as u32 as i64)
    } else {
        ((other as i64) << 32) | (one as u32 as i64)
    }
}

/// A vertex's position, read as doubles.
fn position(mesh: &Mesh, vertex: i32) -> Vector3 {
    Vector3::from_vector3_float(mesh.vertices[vertex as usize])
}
