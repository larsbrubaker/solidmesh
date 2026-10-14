//! The one case `SectionRobustnessTests` measured down to its
//! cause - a RAKED chord carrying a collinear SPLIT vertex, where an independently placed offset
//! boundary is overtaken by its neighbours' sliding ones and the wall facet between them turns
//! over. The rule that answers it is `section_split_blend`, and these claims are what it owes: the
//! raked split bore builds, and the unraked one it must not disturb is unchanged.
//!
//! One test moves the shared absorb switch (`section_split_blend`); it sets the switch to its
//! default (on) and puts back what it found, so a test running beside it reads the same value
//! either way.

use std::sync::Arc;

use geometry_processing::bevel::sections::section_builder_run::try_build_chain;
use geometry_processing::bevel::sections::{
    section_assembly, section_split_blend, SectionPool, SectionRun,
};
use geometry_processing::bevel::surface::surface_mesh_closure;
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelSelectionKind, FaceGroup, FeatureEdgeSet, MatchTolerances,
};
use polygon_mesh::Mesh;
use vector_math::{Matrix4X4, Vector3};

use crate::bore_test_meshes::create_plate_with_a_raked_bore_whose_mouth_chords_are_split;
use crate::section_robustness_tests::{code_of, worst_reach_off_the_canal};
use crate::surface_bevel_robustness_tests as cases;
use crate::{mesh_measurements, rigid_motion_meshes};

struct Built {
    refusal: String,
    run: Option<SectionRun>,
    pool: SectionPool,
    mesh: Option<Mesh>,
    worst_reach: f64,
}

/// Puts the absorb switch back to what it was found at, however the test leaves.
struct AbsorbSwitchRestored(bool);

impl Drop for AbsorbSwitchRestored {
    fn drop(&mut self) {
        section_split_blend::set_absorb_split_vertices(self.0);
    }
}

/// The same two scenarios with the split vertices ABSORBED - each split station laying no profile
/// row of its own, only its boundary point into its gap's boundary rows. Both still build closed,
/// manifold and in one shell, and the radius invariant reads no worse than the blend's 1e-7.
#[test]
fn the_split_chords_build_with_the_split_vertices_absorbed() {
    let _restored = AbsorbSwitchRestored(section_split_blend::absorb_split_vertices());

    section_split_blend::set_absorb_split_vertices(true);

    for rake in [45.0, 0.0] {
        let built = build(rake, 40, if rake == 45.0 { 0.4 } else { 0.2 }, true);

        assert_eq!(built.refusal, "", "rake {rake}");
        let mesh = built
            .mesh
            .as_ref()
            .expect("a build with no refusal has a mesh");
        assert_eq!(surface_mesh_closure::open_edge_count(mesh).0, 0);
        assert_eq!(rigid_motion_meshes::most_faces_on_an_edge(mesh), 2);
        assert_eq!(mesh_measurements::shell_count(mesh), 1);
        assert_eq!(rigid_motion_meshes::two_cornered_triangle_count(mesh), 0);

        assert!(
            built.worst_reach < 1e-7,
            "rake {rake} read {} off the radius",
            built.worst_reach
        );
    }
}

/// rake45 / 40 sides / 0.4 / split at identity - the row the fault was measured on. Before the
/// blend the assembly refused `SurfaceCornerNotSupported`, because the face rebuild was handed a
/// ring that crossed itself.
#[test]
fn the_raked_split_bore_mouth_builds_through_blended_split_sections() {
    let built = build(45.0, 40, 0.4, true);

    assert_eq!(built.refusal, "");
    let mesh = built
        .mesh
        .as_ref()
        .expect("a build with no refusal has a mesh");
    assert_eq!(surface_mesh_closure::open_edge_count(mesh).0, 0);
    assert_eq!(rigid_motion_meshes::most_faces_on_an_edge(mesh), 2);
    assert_eq!(mesh_measurements::shell_count(mesh), 1);
    assert_eq!(rigid_motion_meshes::two_cornered_triangle_count(mesh), 0);

    // The slide is ALONG the chord only, so every sample keeps the offset from its own centre that
    // the section drew and the radius invariant is untouched by the rule - the same reading
    // SectionRobustnessTests takes, here well inside its 1e-6 x diagonal.
    assert!(built.worst_reach < 1e-7, "read {}", built.worst_reach);
}

/// What the blend may and may not move on an UNRAKED split chord, where nothing was wrong to begin
/// with. It may not move a sample ACROSS the chord at all, and on the wall side, where the
/// neighbours' folds are square to the chord and have no along-chord reach, it does not move the
/// boundary at all. On the flat side it DOES slide, by 0.0063 here, because the neighbours'
/// boundaries there stand out at their mitres and the line between them is longer than the chord.
#[test]
fn an_unraked_split_chord_slides_only_along_its_own_chord() {
    let blended = build(0.0, 40, 0.2, true);
    let independent = build(0.0, 40, 0.2, false);

    assert_eq!(blended.refusal, "");
    assert_eq!(independent.refusal, "");

    let blended_run = blended.run.as_ref().expect("the blended build has a run");
    let independent_run = independent
        .run
        .as_ref()
        .expect("the independent build has a run");

    let mut worst_across = 0.0_f64;
    let mut worst_on_the_unfolded_side = f64::MAX;

    for i in 0..blended_run.sections.len() {
        let one = &blended_run.sections[i];
        let other = &independent_run.sections[i];

        assert_eq!(one.profile.len(), other.profile.len());

        let mut moved = 0.0_f64;

        for k in 0..one.profile.len() {
            let r#move: Vector3 = blended.pool.position_of(one.profile[k])
                - independent.pool.position_of(other.profile[k]);

            worst_across =
                worst_across.max((r#move - (one.tangent * r#move.dot(one.tangent))).length());
            moved = moved.max(r#move.length());
        }

        if moved > 1e-9 {
            // The wall boundary of a section the blend touched: unmoved, because an unraked fold
            // leaves the chord square and the line between the neighbours' wall boundaries is the
            // chord itself.
            let last = one.profile.len() - 1;
            let other_last = other.profile.len() - 1;

            worst_on_the_unfolded_side = worst_on_the_unfolded_side.min(
                (blended.pool.position_of(one.profile[0])
                    - independent.pool.position_of(other.profile[0]))
                .length()
                .min(
                    (blended.pool.position_of(one.profile[last])
                        - independent.pool.position_of(other.profile[other_last]))
                    .length(),
                ),
            );
        }
    }

    assert!(
        worst_across < 1e-9,
        "the blend moved an unraked sample across its chord by {worst_across}"
    );
    assert!(
        worst_on_the_unfolded_side < 1e-9,
        "the blend moved an unraked wall boundary by {worst_on_the_unfolded_side}"
    );
    assert!(
        blended.worst_reach < independent.worst_reach + 1e-12,
        "the radius invariant may not read worse for the rule having run"
    );
}

/// One bore mouth built through the sections, with the refusal (if any) read as a sentence.
fn build(rake: f64, sides: i32, size: f64, blend_split_vertices: bool) -> Built {
    let part = create_plate_with_a_raked_bore_whose_mouth_chords_are_split(
        sides,
        cases::RADIUS,
        cases::THICKNESS,
        rake,
        &cases::splits_within(sides),
        6.0,
        Vector3::ZERO,
    );

    let graph = part.get_mesh_edge_graph();
    let features = Arc::new(FeatureEdgeSet::create_with_graph(
        &part,
        &graph,
        cases::BEND_THRESHOLD_DEGREES,
    ));

    let group = bevel_edge_group_resolver::create_from_edge(
        &features,
        cases::the_upper_mouth(&features, Matrix4X4::IDENTITY, rake),
        BevelSelectionKind::TangentChain,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("the upper mouth is one of the part's feature edges");

    let chain = bevel_edge_group_resolver::resolve_crease(
        &group,
        features.clone(),
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
        MatchTolerances::default(),
    )
    .chain;

    let mut pool = SectionPool::new(&part.vertices);

    let refused = |refusal: String, run: Option<SectionRun>, pool: SectionPool| Built {
        refusal,
        run,
        pool,
        mesh: None,
        worst_reach: 0.0,
    };

    let chain = match chain {
        Some(chain) if chain.is_closed() => chain,
        other => {
            return refused(
                format!(
                    "the pick walked {} edges and did not close",
                    other.map_or(0, |c| c.count())
                ),
                None,
                pool,
            )
        }
    };

    let run = match try_build_chain(
        Some(&chain),
        &part,
        &graph,
        &mut pool,
        size,
        cases::SEGMENTS,
        blend_split_vertices,
    ) {
        Ok(run) => run,
        Err(reason) => {
            return refused(
                format!("the sections refused {}", code_of(&reason)),
                None,
                pool,
            )
        }
    };

    let mesh = match section_assembly::try_assemble(&part, &graph, &run, &mut pool) {
        Ok(mesh) => mesh,
        Err(reason) => {
            return refused(
                format!("the assembly refused {}", code_of(&reason)),
                Some(run),
                pool,
            )
        }
    };

    // The same reading SectionRobustnessTests takes, called rather than copied: off a radius from
    // the station's own canal.
    let worst_reach = worst_reach_off_the_canal(&run, &pool, size);

    Built {
        refusal: String::new(),
        run: Some(run),
        pool,
        mesh: Some(mesh),
        worst_reach,
    }
}
