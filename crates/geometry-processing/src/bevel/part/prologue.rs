//! A part's prologue and its edge groups' build: what is done between reading one part's mesh and
//! handing it to the surface construction (`surface_part_builder`). [`prepare`] refuses a part
//! with no mesh, or makes the welded working copy, carries it into the operation's space less its
//! translation, and resolves the groups on it ([`SharedPart`]); [`build_edges`] then builds the
//! part on the surface construction. The Point groups (`point_groups`) call [`build_edges`] for
//! their part's edge groups; [`bevel_part`](super::bevel_part) for a part with none.

use std::rc::Rc;
use std::sync::Arc;

use polygon_mesh::{CancelToken, Mesh, MeshEdgeGraph};
use vector_math::{Color, Matrix4X4, Vector4};

use super::surface_part_builder::BevelSurfacePartBuilder;
use super::{BevelPartError, BevelPartOutcome, BevelPartRequest, BevelProgressReport};
use crate::bevel::bevel_working_mesh::{self, StampedVerdict};
use crate::bevel::{
    BevelAdvisory, BevelAdvisoryCode, BevelGroupResult, BevelRebuildAdvisories, BevelResultGroup,
};
use crate::topology::bevel_edge_group_resolver;
use crate::topology::{BevelEdgeGroup, BevelEdgeGroupResolution};

/// The prologue's work, done before the surface construction starts: what is known at the line
/// where it is handed the part.
#[derive(Debug)]
pub struct SharedPart {
    /// The welded working copy, carried into the operation's space less its translation.
    pub working_mesh: Arc<Mesh>,
    /// The resolver's graph of that mesh (built here where the resolver built none).
    pub working_mesh_graph: Rc<MeshEdgeGraph>,
    /// The translation held back from the working mesh, which the surface construction carries
    /// its result out by.
    pub build_translation: Matrix4X4,
    /// The groups carried into the build's space and read at the operation's bend threshold.
    pub groups_in_build_space: Vec<BevelEdgeGroup>,
    /// What each group resolved to on the working mesh.
    pub resolutions: Vec<BevelEdgeGroupResolution>,
    /// What the repair had to say, which the rebuild adds to.
    pub advisories: BevelRebuildAdvisories,
}

impl SharedPart {
    /// What is offered to the surface construction: every group that resolved to edges is offered,
    /// and the surface construction refuses what it cannot fit in its own words. The only thing
    /// decided here is what no construction owns: a group that resolved to nothing at all.
    pub fn offered(&self) -> Vec<bool> {
        self.resolutions
            .iter()
            .map(|resolution| resolution.is_resolved() && !resolution.edges.is_empty())
            .collect()
    }
}

/// What [`prepare`] comes to: a part with nothing to bevel is refused there and then; any other is
/// ready for the surface construction.
#[derive(Debug)]
pub enum PreparedPart {
    /// The part has no mesh: one refusal per group.
    Refused(Box<BevelPartOutcome>),
    /// The prologue's work, for the surface construction to continue from.
    Ready(Box<SharedPart>),
}

/// The build up to the hand-over to the surface construction: the part refused for having no
/// mesh, or the welded working copy in the build's space with the request's groups resolved on it.
pub fn prepare(request: &BevelPartRequest) -> PreparedPart {
    let original_mesh = request
        .mesh
        .as_ref()
        .filter(|mesh| !mesh.vertices.is_empty());

    let Some(original_mesh) = original_mesh else {
        // Refused in words, one refusal per group, rather than answered with nothing: a group
        // answered with nothing simply vanishes from the caller's list - no row, no reason, and
        // nothing for the user to act on.
        let refusals = request
            .groups
            .iter()
            .enumerate()
            .map(|(i, group)| {
                let refusal =
                    BevelAdvisory::new(BevelAdvisoryCode::PartHasNoMeshToBevel, Vec::new());
                Some(BevelGroupResult::new(
                    i as i32,
                    BevelResultGroup::Edge(group.clone()),
                    0,
                    Some(refusal),
                    Vec::new(),
                ))
            })
            .collect();

        return PreparedPart::Refused(Box::new(BevelPartOutcome::new(
            None,
            Color::default(),
            refusals,
            None,
            None,
        )));
    };

    // The same welded copy viewport picking works against, so the edge the user clicked is the
    // edge this resolves. The set the working mesh builds to make that decision is thrown away and
    // the resolver builds its own (one per stored tolerance, which need not be that one) - a
    // redundancy kept on purpose: the set it throws away was read BEFORE the weld and the reverse,
    // so on exactly the meshes the repair exists for it describes a mesh that no longer exists.
    // The whole stage measures about 0.5 ms on a 960 face part against a 330 ms rebuild. Anything
    // the repair had to do travels with the results: the repair is made on this copy and never on
    // the caller's mesh, so what is said here is the user's only trace of it.
    let (mut working_mesh, source_advisories) = bevel_working_mesh::create_from_with_advisories(
        original_mesh,
        &mut StampedVerdict::default(),
    );

    // AND CARRIED INTO THE SPACE THE SIZE IS QUOTED IN, before anything reads it: the matrix from
    // the part out to the operation is what a Scale, a rotation or a translation between the two
    // amounts to, and everything below is measured on the mesh as it is here.
    let source_to_operation = request.source_to_operation;

    // EXCEPT FOR ITS TRANSLATION, which is applied last. A translation cannot change a size, an
    // angle or a shape - and carrying it costs real accuracy, because mesh vertices are FLOAT: a
    // part built 125 mm out from the origin carries about 1e-3 of storage noise where the same part
    // built round it carries 1e-5, and the build's plane tolerance follows the part's SIZE while
    // the noise follows its distance from the origin. So the build happens in the operation's
    // space SHIFTED BACK ONTO the source mesh's own origin, and the shift travels on to carry the
    // finished mesh out (`SharedPart::build_translation`). A part that is only moved keeps exactly
    // the vertices it had.
    let row3 = source_to_operation.row3;
    let build_translation = Matrix4X4::create_translation(row3.x, row3.y, row3.z);
    let mut source_to_build = source_to_operation;
    source_to_build.row3 = Vector4::new(0.0, 0.0, 0.0, row3.w);

    bevel_working_mesh::carry_into(&mut working_mesh, source_to_build);

    // The anchors travel with it, into the same space - copies made here and thrown away with the
    // build, because the originals are what the results are reported against. And read at ONE flat
    // angle, the operation's bend threshold, so one operation reads its part one way.
    let bend_threshold_degrees = request.bend_threshold_degrees;
    let groups_in_build_space: Vec<BevelEdgeGroup> = if source_to_build == Matrix4X4::IDENTITY {
        request
            .groups
            .iter()
            .map(|group| group.read_at(bend_threshold_degrees))
            .collect()
    } else {
        request
            .groups
            .iter()
            .map(|group| {
                group
                    .transformed(source_to_build)
                    .read_at(bend_threshold_degrees)
            })
            .collect()
    };

    // Whatever the repair had to say, and whatever the rebuild itself has to add to it. One
    // object carried through so every way out publishes the same list.
    let advisories = BevelRebuildAdvisories::new(Some(source_advisories));

    // The graph comes back so the fit check can walk the same one rather than build a second
    // copy of it. Safe to share only because both read THIS mesh: welded and turned right way
    // out before either sees it, and the tools are built beside it, never into it.
    let (resolutions, graph) =
        bevel_edge_group_resolver::resolve_all(&groups_in_build_space, &working_mesh);
    // The resolver's graph is handed on, and built here where it built none, so every reader
    // below shares one graph of the mesh instead of building its own.
    let working_mesh_graph = Rc::new(graph.unwrap_or_else(|| working_mesh.get_mesh_edge_graph()));

    PreparedPart::Ready(Box::new(SharedPart {
        working_mesh: Arc::new(working_mesh),
        working_mesh_graph,
        build_translation,
        groups_in_build_space,
        resolutions,
        advisories,
    }))
}

/// The prologue, then the surface construction, for a request whose groups are all edge groups.
/// It does not award shared edges: in polygon space one edge is one STRIP between its two faces,
/// whoever named it, so it unions the groups that name a common edge and links the union as one
/// network, and it measures its own room. `progress` is told the bar's phases.
pub fn build_edges(
    request: &BevelPartRequest,
    cancel: &CancelToken,
    progress: Option<BevelProgressReport>,
) -> Result<BevelPartOutcome, BevelPartError> {
    let shared = match prepare(request) {
        PreparedPart::Refused(outcome) => return Ok(*outcome),
        PreparedPart::Ready(shared) => *shared,
    };

    let offered = shared.offered();
    let SharedPart {
        working_mesh,
        working_mesh_graph,
        build_translation,
        resolutions,
        advisories,
        ..
    } = shared;
    let working_mesh_graph =
        Rc::try_unwrap(working_mesh_graph).unwrap_or_else(|graph| (*graph).clone());

    // The working mesh and the translation held back from it, which is all the surface
    // construction needs of the space bookkeeping: it rewrites that mesh and carries the answer
    // out by that translation. The memo is the caller's own, which is what lets a DRAG measure the
    // room once. The groups it reports against are the request's, as the caller stored them.
    let mut surface = BevelSurfacePartBuilder::new(
        request.groups.clone(),
        resolutions,
        offered,
        Rc::new(advisories),
        working_mesh,
        working_mesh_graph,
        request.size,
        request.segments,
        request.profile_is_round,
        build_translation,
        request.color,
        None,
        progress,
        request.pick_memo.clone(),
    );
    surface.set_size_search_budget_ms(request.size_search_budget_ms);
    surface.set_routing(request.routing);
    Ok(surface.build(cancel)?)
}
