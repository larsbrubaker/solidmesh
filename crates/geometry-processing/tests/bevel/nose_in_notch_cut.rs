//! Port of the test fixture `Fixtures/NoseInNotchCut.cs`: the nose-in-notch part with its ridge
//! ROUNDED - the first pass of a mixed bevel, cut exactly as the rebuild cuts it - and the two
//! concave picks whose creases die on that ridge. The second pass's tests re-read those creases on
//! the cut part.

use std::rc::Rc;
use std::sync::Arc;

use geometry_processing::bevel::bevel_cap_clip::{self, Candidate};
use geometry_processing::bevel::bevel_face_fit::DEFAULT_SURFACE_TOLERANCE_DEGREES;
use geometry_processing::bevel::bevel_group_tools::{self, BevelGroupToolsOptions};
use geometry_processing::bevel::{bevel_published_mesh, bevel_working_mesh, BevelFaceFit};
use geometry_processing::topology::bevel_edge_group_resolver;
use geometry_processing::topology::{BevelEdgeGroup, FeatureEdgeSet};
use polygon_mesh::csg::{BooleanError, CsgModes};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{Matrix4X4, Vector3};

use crate::kernel_cut_helpers::{boolean, edge_group};
use crate::nose_in_notch_test_meshes as meshes;

/// C# `CutRidgeAsync`'s answer: the boolean's own result, the tidied and welded part the second
/// pass reads, and the upper and lower concave picks.
pub struct RidgeCut {
    /// What the boolean returned, before it was tidied and welded; only the tests of what the tidy
    /// takes away (`bevel_crease_over_a_rounded_ridge_tests`) read it.
    pub raw: Mesh,
    pub cut: Arc<Mesh>,
    pub upper: BevelEdgeGroup,
    pub lower: BevelEdgeGroup,
}

/// C# `CutRidgeAsync`: the part with its ridge rounded at `radius` in `segments`, as the rebuild
/// would cut it, and the two concave picks.
pub fn cut_ridge(radius: f64, segments: i32) -> RidgeCut {
    let part = meshes::create();
    let features = FeatureEdgeSet::create(&part);

    let ridge = bevel_edge_group_resolver::resolve(
        &anchor(
            &features,
            meshes::left_ridge_end(),
            meshes::right_ridge_end(),
        ),
        &part,
    );

    let fit = BevelFaceFit::create(
        Arc::new(part.copy()),
        Rc::new(vec![ridge.clone()]),
        true,
        radius,
        DEFAULT_SURFACE_TOLERANCE_DEGREES,
        None,
    );
    let (_, trimmed_by_a_wall) = fit.try_check_group_trimmed(0);

    let built = bevel_group_tools::try_build(
        &ridge,
        true,
        radius,
        segments,
        BevelGroupToolsOptions {
            may_clip_caps: trimmed_by_a_wall,
            ..BevelGroupToolsOptions::default()
        },
    )
    .unwrap_or_else(|refusal| {
        panic!(
            "the ridge cutter did not build: {:?}",
            refusal.reason.code()
        )
    });

    let mut cutters = vec![built.tools[0].mesh.clone()];

    if let Some(clips) = built.tools[0].cap_clips.clone() {
        let combine =
            |operands: &[(Mesh, Matrix4X4)], mode: CsgModes| -> Result<Mesh, BooleanError> {
                let meshes: Vec<&Mesh> = operands.iter().map(|(mesh, _)| mesh).collect();
                Ok(boolean(&meshes, mode))
            };

        let withdrawals = bevel_cap_clip::apply(
            &mut cutters,
            &[Candidate::new(0, 0, radius, clips)],
            &combine,
        )
        .expect("not cancelled");

        assert!(
            withdrawals.is_empty(),
            "the cap clip withdrew the ridge cutter: {:?}",
            withdrawals
                .first()
                .map(|withdrawal| withdrawal.reason.code())
        );
    }

    // Both meshes the second pass could be given: what the boolean actually returned, and what the
    // rebuild reads - the same subtract TIDIED and then welded, in that order, exactly as
    // BevelFeatureMeshBuilder does it. The difference between them is a whole crease
    // (TheCreaseWalkNeedsTheCutTidiedFirst).
    let raw = boolean(&[&part, &cutters[0]], CsgModes::Subtract);

    let cancel = CancelToken::new();
    let cleaned = bevel_published_mesh::clean(&raw, &cancel).expect("not cancelled");
    let tidied = cleaned.as_ref().unwrap_or(&raw);
    let cut = bevel_working_mesh::create_from(tidied, &mut Default::default());

    RidgeCut {
        raw,
        cut: Arc::new(cut),
        upper: anchor(
            &features,
            meshes::right_ridge_end(),
            meshes::upper_crease_end(),
        ),
        lower: anchor(
            &features,
            meshes::right_ridge_end(),
            meshes::lower_crease_end(),
        ),
    }
}

/// C# `Anchor`: a group anchored on the feature edge running between `from` and `to`, either way
/// round - the same helper as the shadow and seam classes' `Group`, so it is that one.
pub fn anchor(features: &FeatureEdgeSet, from: Vector3, to: Vector3) -> BevelEdgeGroup {
    edge_group(features, from, to)
}
