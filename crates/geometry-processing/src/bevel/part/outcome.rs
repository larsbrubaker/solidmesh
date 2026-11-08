//! What one part's bevel came to ([`BevelPartOutcome`]): the mesh to show, or none, and everything
//! the build has to say about it, as codes. The surface construction (`surface_part_builder`), the
//! Point groups (`point_groups`) and the part's prologue (`prologue`) all answer in it, and
//! [`bevel_part`](super::bevel_part) hands it to the caller, who words the rows, the part-level
//! failures and the advisories in its own language.

use solidmesh_math::Color;
use solidmesh_mesh::Mesh;

use crate::bevel::{
    BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode, BevelGroupResult, BevelResultGroup,
};
use crate::topology::BevelEdgeGroup;

/// The mesh to show for one part, and everything the rebuild has to say about it.
#[derive(Debug, Clone)]
pub struct BevelPartOutcome {
    /// The finished mesh, in the space the request's `source_to_operation` carries the part into,
    /// or `None` when this part built nothing - which is a refusal with a reason and not an error.
    pub mesh: Option<Mesh>,

    /// What to paint that mesh, worked out from the source's own colors (unset is
    /// [`Color::default`], transparent black).
    pub color: Color,

    /// One entry per group the request carried, indexed into THAT list. A row can be missing
    /// where nothing wrote it; every reader skips those.
    pub group_results: Vec<Option<BevelGroupResult>>,

    /// What went wrong with this part as a whole rather than with one group; `None` when there
    /// is nothing to say.
    pub failures: Option<Vec<BevelAdvisory>>,

    /// What had to be put right to build it, or was built with a caveat; `None` when there is
    /// nothing to say.
    pub advisories: Option<Vec<BevelAdvisory>>,
}

impl BevelPartOutcome {
    /// An outcome from each of its fields, in order.
    pub fn new(
        mesh: Option<Mesh>,
        color: Color,
        group_results: Vec<Option<BevelGroupResult>>,
        failures: Option<Vec<BevelAdvisory>>,
        advisories: Option<Vec<BevelAdvisory>>,
    ) -> Self {
        BevelPartOutcome {
            mesh,
            color,
            group_results,
            failures,
            advisories,
        }
    }

    /// What a build that could not run answers: no mesh, and every one of `groups` refused with
    /// [`BevelAdvisoryCode::ConstructionCouldNotBuildThisGroup`]. A caller who picked an edge is
    /// owed a row that says it could not be built, not an error about the kernel; this is the
    /// outcome to show for a [`BevelPartError`](super::BevelPartError) that is not a cancel.
    pub fn every_group_refused(groups: &[BevelEdgeGroup]) -> Self {
        let advisory = BevelAdvisory::new(
            BevelAdvisoryCode::ConstructionCouldNotBuildThisGroup,
            vec![BevelAdvisoryArg::String("Surface".to_string())],
        );
        let refusals = groups
            .iter()
            .enumerate()
            .map(|(i, group)| {
                Some(BevelGroupResult::new(
                    i as i32,
                    BevelResultGroup::Edge(group.clone()),
                    0,
                    Some(advisory.clone()),
                    Vec::new(),
                ))
            })
            .collect();

        BevelPartOutcome::new(None, Color::default(), refusals, None, None)
    }
}
