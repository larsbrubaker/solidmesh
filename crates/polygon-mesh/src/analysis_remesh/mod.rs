//! Graded surface remeshing for stress analysis: [`remesh_for_analysis`] rebuilds a closed,
//! manifold, consistently wound surface with well-shaped triangles near a size the caller gives
//! per position, so a downstream volume mesher can take every triangle as an exact constraint.
//! Sharp input creases survive as chains of mesh edges within the caller's deviation bound;
//! input that is open or non-manifold is refused by name, never repaired here.
//!
//! `Ok` is a promise about the exact `f64` coordinates returned ([`AnalysisSurface`]): closed and
//! wound, no triangles touching or crossing, no free edge folded back on itself, within the angle
//! floor, creases kept, and within `max_deviation` both ways on the same-facing sheet of the
//! input, with one exception: an input face `clean` removed as a strip (a facet between two
//! creases narrower than a quarter of the bound) is measured to the nearest output surface of
//! any facing, because once merged into one sharp edge nothing faces its way. Output to input
//! stays same-facing throughout. `verify` checks all of this on the result, and anything that
//! cannot be met comes back as a named error carrying the measured value and what to change,
//! never as a surface that looks successful. A triangle left under the
//! floor is such an error, [`AnalysisRemeshError::AngleFloorNotMet`], unless the input's own
//! geometry forces it: a crease sector that measures under the floor in the input, or a whole
//! vertex fan too narrow to give three corners at the floor (`sliver` defines this exactly).
//! Remeshing never narrows a corner past the floor and
//! calls it forced, because the comparison is with the input, not with the coarsened chords.
//!
//! `max_deviation` is a guarantee at sample points, not a proof over a continuum: the guards hold
//! a sample grid per triangle to a fraction of the bound, the final refinement and the report
//! measure a denser grid both ways. Between samples the distance can still exceed the sampled
//! value where curvature is finer than the grid; the guards stop at a fraction of the bound to
//! leave room for that, not to prove it away. A part with a feature finer than the sample
//! spacing and not on a crease is the case the bound says least about.
//!
//! The angle limit is two numbers because they do different jobs. The hard floor (1 degree by
//! default) is what a downstream solve cannot do without: measured on 10-node tetrahedra over 72
//! cases of bad surface triangles, corners down to 0.1 degrees moved tip deflection by under
//! 0.001% and whole-part peak stress by at most 0.11%, so what a corner under the old 5-degree
//! hard limit costs is solver iterations (in clusters below 2 to 3 degrees), not answers, and that
//! limit was refusing 134 of 528 ordinary poses with worst corners of 1.9 to 4.6 degrees while
//! protecting no result. Fillets, notches and curved elements were not measured. The soft target
//! (5 degrees by default) is what the repair aims for; it never refuses, and the report's corner
//! histogram says how close the surface came.
//!
//! The `g3` remesher is length-driven and carries no triangle-shape guard: collapses and flips
//! are refused only when a normal flips, so nothing stops a collapse or a smoothing move from
//! leaving a free vertex on the line of a crease edge. On boolean output that shows up as
//! triangles of essentially zero area along the creases: one crease edge and a free vertex
//! collinear with it. `engine` therefore runs its own split, collapse, flip and tangential-smooth passes on the same
//! `DMesh3`, each edit accepted only if it keeps the triangles it touches shaped and on the
//! input surface; `sliver` works the remaining triangles below the floor; `crease` keeps each
//! crease edge's original polyline so a coarsened chain stays on the crease; `report` measures
//! the returned mesh. Before any of that, `clean` removes the input faces too thin to have a
//! normal, so `crease` and the passes only ever see faces with one, and merges into one sharp edge
//! any crease-bordered strip narrower than a quarter of the bound, whose two creases no remesh
//! could keep apart. The existing remesher and its callers are untouched.

mod clean;
mod crease;
mod crossing;
mod engine;
mod report;
mod sheets;
mod sliver;
mod verify;

use std::fmt;

use crate::g3::dmesh3::DMesh3;
use crate::g3::dmesh_aabb_tree3::DMeshAABBTree3;
use crate::{CancelToken, Mesh};

pub use report::AnalysisRemeshReport;

use crease::CreaseNet;
use engine::Engine;

/// What [`remesh_for_analysis`] should build. Lengths are in the mesh's units, angles in degrees.
pub struct AnalysisRemeshOptions<'a> {
    /// The target edge length at a position. Clamped to the minimum and maximum below; a
    /// non-finite or non-positive answer is refused.
    pub target_edge_length: &'a dyn Fn([f64; 3]) -> f64,
    /// The shortest edge the remesh will create by splitting, and the floor on the target.
    pub min_edge_length: f64,
    /// The ceiling on the target.
    pub max_edge_length: f64,
    /// Input edges whose faces bend by more than this are held as creases.
    pub sharp_edge_degrees: f64,
    /// How far the output may stray from the input surface, and a crease chain from its crease.
    pub max_deviation: f64,
    /// The smallest triangle angle `Ok` allows. A corner under it reaches `Ok` only where the
    /// input's own geometry forces it, as the module header defines; any other is refused with
    /// [`AnalysisRemeshError::AngleFloorNotMet`]. A field rather than a constant because a
    /// coarser or finer consumer of the surface wants a different floor.
    pub min_angle_floor_degrees: f64,
    /// The angle the repair works towards. Never causes a refusal: missing it costs quality the
    /// report's corner histogram shows, not validity. Taken as at least the floor.
    pub min_angle_target_degrees: f64,
    /// Upper limit on refinement passes.
    pub max_passes: u32,
    /// Refuses with [`AnalysisRemeshError::TriangleBudgetExceeded`] past this many triangles.
    pub max_triangles: usize,
    /// Checked between edits; a cancelled run returns [`AnalysisRemeshError::Cancelled`].
    pub cancel: Option<&'a CancelToken>,
}

impl<'a> AnalysisRemeshOptions<'a> {
    /// Options with the given sizing and deviation bound and the defaults a typical CAD part
    /// wants: creases past 30 degrees, a 1 degree hard angle floor, a 5 degree angle target, 12
    /// passes and 2 million triangles.
    pub fn new(target_edge_length: &'a dyn Fn([f64; 3]) -> f64, max_deviation: f64) -> Self {
        Self {
            target_edge_length,
            min_edge_length: 0.0,
            max_edge_length: f64::INFINITY,
            sharp_edge_degrees: 30.0,
            max_deviation,
            min_angle_floor_degrees: 1.0,
            min_angle_target_degrees: 5.0,
            max_passes: 12,
            max_triangles: 2_000_000,
            cancel: None,
        }
    }

    /// The angle the repair aims for: the soft target, but never less than the hard floor.
    pub(crate) fn aim_degrees(&self) -> f64 {
        self.min_angle_target_degrees
            .max(self.min_angle_floor_degrees)
    }

    fn validate(&self) -> Result<(), AnalysisRemeshError> {
        let bad = |what: &'static str| Err(AnalysisRemeshError::InvalidOptions(what));
        if !self.max_deviation.is_finite() || self.max_deviation <= 0.0 {
            return bad("max_deviation must be a positive finite length");
        }
        if !self.min_edge_length.is_finite() || self.min_edge_length < 0.0 {
            return bad("min_edge_length must be finite and not negative");
        }
        if self.max_edge_length.is_nan() || self.max_edge_length <= 0.0 {
            return bad("max_edge_length must be positive");
        }
        if self.max_edge_length < self.min_edge_length {
            return bad("max_edge_length must not be below min_edge_length");
        }
        if !self.sharp_edge_degrees.is_finite() || !(0.0..=180.0).contains(&self.sharp_edge_degrees)
        {
            return bad("sharp_edge_degrees must be between 0 and 180");
        }
        if !self.min_angle_floor_degrees.is_finite()
            || !(0.0..60.0).contains(&self.min_angle_floor_degrees)
        {
            return bad("min_angle_floor_degrees must be at least 0 and below 60");
        }
        if !self.min_angle_target_degrees.is_finite()
            || !(0.0..60.0).contains(&self.min_angle_target_degrees)
        {
            return bad("min_angle_target_degrees must be at least 0 and below 60");
        }
        if self.max_triangles == 0 {
            return bad("max_triangles must be positive");
        }
        Ok(())
    }
}

/// A remeshed surface in `f64`, the coordinates its report and verification were measured on.
#[derive(Debug, Clone, PartialEq)]
pub struct AnalysisSurface {
    /// Vertex positions.
    pub vertices: Vec<[f64; 3]>,
    /// Faces as indices into `vertices`, wound outward.
    pub faces: Vec<[i32; 3]>,
}

impl AnalysisSurface {
    /// This surface as a [`Mesh`], for callers who want its `f32` vertices. The narrowing rounds
    /// coordinates, so the report's guarantees do not carry over to the result far from the
    /// origin, where `f32` steps grow past the features and the deviation bound.
    pub fn to_mesh(&self) -> Mesh {
        let mut mesh = Mesh::new();
        for v in &self.vertices {
            mesh.vertices
                .push(solidmesh_math::Vector3Float::new(v[0], v[1], v[2]));
        }
        for f in &self.faces {
            mesh.faces.add(f[0], f[1], f[2], &mesh.vertices);
        }
        mesh
    }
}

/// The remeshed surface and what the remesh measured on it.
#[derive(Debug, Clone)]
pub struct AnalysisRemesh {
    /// Closed, manifold and consistently wound, as the input was.
    pub surface: AnalysisSurface,
    /// Quality, deviation, volume change and crease survival of `surface`.
    pub report: AnalysisRemeshReport,
}

/// Why a remesh for analysis was not produced.
#[derive(Debug, Clone, PartialEq)]
pub enum AnalysisRemeshError {
    /// The mesh has no faces.
    EmptyInput,
    /// The surface has a boundary: edges with a single face. Close it first.
    OpenInput { boundary_edges: usize },
    /// Edges with more than two faces, faces repeating a vertex or faces the mesh could not
    /// hold, or vertices joining separate fans of faces. Repair the surface first.
    NonManifoldInput {
        edges_over_two_faces: usize,
        faces_refused: usize,
        non_manifold_vertices: usize,
    },
    /// Adjacent faces walk their shared edge the same way, so outward is undefined.
    InconsistentWinding { edges: usize },
    /// An option is out of range; the message names it and what it must be.
    InvalidOptions(&'static str),
    /// The target edge length callback gave `value` at `at`, which is not a positive finite
    /// length.
    InvalidTargetSize { at: [f64; 3], value: f64 },
    /// The remesh passed `limit` triangles (it had reached `triangles`). Raise `max_triangles`
    /// or give a larger target edge length.
    TriangleBudgetExceeded { triangles: usize, limit: usize },
    /// The caller's cancel check fired; no result is produced.
    Cancelled,
    /// A mesh edit found broken topology.
    BrokenTopology(&'static str),
    /// A vertex position is not finite.
    NonFiniteVertex { index: usize },
    /// Triangles of the result cross each other: two walls closer than the deviation bound
    /// allows met. Lower `max_deviation` below half the thinnest wall or use a smaller target.
    SelfIntersecting { pairs: usize },
    /// The surface could not be brought within the bound; the largest sampled distance is
    /// `measured`. Raise `max_deviation`, lower `min_edge_length` or give a smaller target.
    DeviationNotMet { measured: f64, bound: f64 },
    /// Triangles remain under the angle floor with no input feature forcing them. Change the
    /// target, the floor, or fix the feature in the input.
    AngleFloorNotMet {
        triangles: usize,
        worst_degrees: f64,
        floor_degrees: f64,
    },
    /// Neighbouring triangles fold back over each other along `edges` edges the input does not
    /// crease: a tetrahedral mesher cannot start from that surface.
    FoldedSurface { edges: usize },
    /// Input crease segments are no longer held on a chain within the bound.
    CreaseLost { lost: usize, total: usize },
}

impl fmt::Display for AnalysisRemeshError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyInput => f.write_str("the mesh has no faces to remesh"),
            Self::OpenInput { boundary_edges } => write!(
                f,
                "the surface is open ({boundary_edges} edges have one face): close it before \
                 remeshing for analysis"
            ),
            Self::NonManifoldInput {
                edges_over_two_faces,
                faces_refused,
                non_manifold_vertices,
            } => write!(
                f,
                "the surface is not manifold ({edges_over_two_faces} edges with more than two \
                 faces, {faces_refused} faces that could not be held, {non_manifold_vertices} \
                 vertices joining separate fans): repair it before remeshing for analysis"
            ),
            Self::InconsistentWinding { edges } => write!(
                f,
                "{edges} edges have neighbouring faces wound the same way: orient the surface \
                 before remeshing for analysis"
            ),
            Self::InvalidOptions(what) => write!(f, "invalid remesh options: {what}"),
            Self::InvalidTargetSize { at, value } => write!(
                f,
                "the target edge length at ({}, {}, {}) was {value}, which is not a positive \
                 finite length",
                at[0], at[1], at[2]
            ),
            Self::TriangleBudgetExceeded { triangles, limit } => write!(
                f,
                "the remesh reached {triangles} triangles, past the limit of {limit}: raise \
                 max_triangles or use a larger target edge length"
            ),
            Self::Cancelled => f.write_str("the remesh was cancelled"),
            Self::BrokenTopology(what) => write!(f, "the remesh found broken topology: {what}"),
            Self::NonFiniteVertex { index } => write!(
                f,
                "vertex {index} has a non-finite position: fix the mesh before remeshing"
            ),
            Self::SelfIntersecting { pairs } => write!(
                f,
                "the remeshed surface has {pairs} crossing triangle pairs: lower max_deviation \
                 below half the thinnest wall or use a smaller target edge length"
            ),
            Self::DeviationNotMet { measured, bound } => write!(
                f,
                "the surface strays {measured} from the input, past the bound of {bound}: \
                 raise max_deviation, lower min_edge_length or use a smaller target edge length"
            ),
            Self::AngleFloorNotMet {
                triangles,
                worst_degrees,
                floor_degrees,
            } => write!(
                f,
                "{triangles} triangles stay under the {floor_degrees} degree floor (worst \
                 {worst_degrees} degrees) with no sharp input feature forcing them: change the \
                 target edge length or the floor, or fix the feature in the input"
            ),
            Self::FoldedSurface { edges } => write!(
                f,
                "{edges} edges of the remeshed surface fold neighbouring triangles back over each \
                 other: use a smaller target edge length or a tighter max_deviation"
            ),
            Self::CreaseLost { lost, total } => write!(
                f,
                "{lost} of {total} input crease segments were not held within the bound: raise \
                 max_deviation or use a smaller target edge length"
            ),
        }
    }
}

impl std::error::Error for AnalysisRemeshError {}

/// Remeshes `mesh` for analysis: see the module header for what it guarantees and why it is
/// separate from the `g3` remesher.
pub fn remesh_for_analysis(
    mesh: &Mesh,
    options: &AnalysisRemeshOptions<'_>,
) -> Result<AnalysisRemesh, AnalysisRemeshError> {
    options.validate()?;
    let source = checked_input(mesh)?;
    let tree = DMeshAABBTree3::new(&source);
    let mut working = source.clone();
    let removed_strips = clean::remove_thin_faces(&mut working, options)?;
    let crease = CreaseNet::from_mesh(&working, options.sharp_edge_degrees, options.aim_degrees());

    let absorbed = clean::absorbed_faces(&source, &removed_strips);
    let mut engine = Engine::new(working, &tree, crease, options, absorbed);
    let passes_run = engine.run()?;

    let surface = report::output_surface(engine.mesh());
    let measured = report::measure(&source, &surface, &engine, passes_run)?;
    verify::check(&surface, &measured, options, &engine)?;
    Ok(AnalysisRemesh {
        surface,
        report: measured,
    })
}

/// The input as a `DMesh3`, or the refusal naming what is wrong with it. Only surfaces that
/// close up edge by edge pass: the remesh keeps closed input closed and has nothing sensible to
/// do with a boundary.
fn checked_input(mesh: &Mesh) -> Result<DMesh3, AnalysisRemeshError> {
    use std::collections::HashMap;

    if mesh.faces.is_empty() {
        return Err(AnalysisRemeshError::EmptyInput);
    }
    if let Some(index) = mesh
        .vertices
        .iter()
        .position(|v| !v.x.is_finite() || !v.y.is_finite() || !v.z.is_finite())
    {
        return Err(AnalysisRemeshError::NonFiniteVertex { index });
    }
    // Counting only, so map order cannot reach the result.
    let mut walks: HashMap<(i32, i32), u32> = HashMap::new();
    let mut repeating_faces = 0;
    for face in mesh.faces.iter() {
        if face.v0 == face.v1 || face.v1 == face.v2 || face.v2 == face.v0 {
            repeating_faces += 1;
            continue;
        }
        for edge in [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)] {
            *walks.entry(edge).or_insert(0) += 1;
        }
    }

    let (mut boundary, mut over_two, mut wound) = (0, 0, 0);
    for (&(a, b), &forward) in &walks {
        let backward = walks.get(&(b, a)).copied().unwrap_or(0);
        if forward + backward == 1 {
            boundary += 1;
        } else if forward + backward > 2 {
            // Each undirected edge is seen from both directions: count it from one.
            if a < b || backward == 0 {
                over_two += 1;
            }
        } else if forward != 1 || backward != 1 {
            // Two faces walking the edge the same way: the reverse direction is absent, so
            // this edge is seen exactly once here.
            wound += 1;
        }
    }

    let source = mesh.to_dmesh3();
    let faces_refused = mesh.faces.len() - source.triangle_count();
    let non_manifold_vertices = source
        .vertex_indices()
        .filter(|&v| source.get_vtx_edge_count(v) > 0 && source.is_bowtie_vertex(v))
        .count();

    if over_two > 0 || repeating_faces > 0 || faces_refused > 0 || non_manifold_vertices > 0 {
        return Err(AnalysisRemeshError::NonManifoldInput {
            edges_over_two_faces: over_two,
            faces_refused: faces_refused.max(repeating_faces),
            non_manifold_vertices,
        });
    }
    if boundary > 0 {
        return Err(AnalysisRemeshError::OpenInput {
            boundary_edges: boundary,
        });
    }
    if wound > 0 {
        return Err(AnalysisRemeshError::InconsistentWinding { edges: wound });
    }
    Ok(source)
}
