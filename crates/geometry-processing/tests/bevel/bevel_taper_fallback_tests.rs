//! Port of `BevelTaperFallbackTests.cs`: a run whose run-out the kernel refuses falls back to the
//! plain tool it would have had with the taper turned off (`bevel_group_tools::try_build`).

use std::sync::Arc;

use geometry_processing::bevel::bevel_group_tools::{try_build, BevelGroupToolsOptions};
use geometry_processing::bevel::BevelSurfaceLabels;
use geometry_processing::topology::bevel_edge_group_resolver::create_from_edge;
use geometry_processing::topology::{BevelEdgeGroupResolution, BevelSelectionKind, FeatureEdgeSet};
use polygon_mesh::csg::boolean_processing::{classify_boolean_operand, BooleanOperandVerdict};
use vector_math::{Matrix4X4, Vector3};

use crate::topology_test_meshes::create_cylinder;

/// C# `ARunWhoseTaperWillNotSweepFallsBackToThePlainTool`, both profiles.
#[test]
fn a_run_whose_taper_will_not_sweep_falls_back_to_the_plain_tool() {
    for round in [true, false] {
        let mesh = create_cylinder(10.0, 10.0, 24);
        let features = Arc::new(FeatureEdgeSet::create(&mesh));

        // One facet of the top rim: the tapering case BevelRunTaperTests sweeps, on a wall whose
        // neighbours carry on from it at 15 degrees. A quarter millimetre, because the plain sweep
        // this falls back TO has to be buildable for the fallback to mean anything.
        let index = features
            .edges()
            .iter()
            .position(|e| e.start.z == 5.0 && e.end.z == 5.0)
            .expect("a top rim edge") as i32;
        let group = create_from_edge(&features, index, BevelSelectionKind::Edge, 30.0)
            .expect("the rim edge makes a group");
        let resolution = BevelEdgeGroupResolution {
            group,
            features: Arc::clone(&features),
            failure_reason: None,
            seed_edge_index: index,
            edges: vec![features.edges()[index as usize]],
            chain: None,
            face_group: None,
        };
        let segments = if round { 8 } else { 1 };

        // The one branch a mesh cannot reach: a taper the kernel refuses. No fixture in the corpus
        // produces one, so the sweep is replaced rather than the geometry contrived.
        let mut tapered_sweeps = 0;
        let mut labels = BevelSurfaceLabels::new(&mesh, None);
        let mut refuse = |_: &_, _: &_, _| {
            tapered_sweeps += 1;
            None
        };
        let built = try_build(
            &resolution,
            round,
            0.25,
            segments,
            BevelGroupToolsOptions {
                surface_labels: Some(&mut labels),
                sweep_taper: Some(&mut refuse),
                ..Default::default()
            },
        );

        assert!(
            tapered_sweeps > 0,
            "the run must actually have grown a run-out, or the fallback is not what is under test"
        );
        let tools = match built {
            Ok(built) => built.tools,
            Err(refusal) => panic!("{:?}", refusal.reason),
        };
        assert_eq!(tools.len(), 1);
        assert!(tools[0].mesh.is_manifold());
        assert_eq!(
            classify_boolean_operand(&tools[0].mesh, Matrix4X4::IDENTITY, false),
            BooleanOperandVerdict::Clean
        );

        // And it is not merely SOME tool: it is the very solid this group builds with the taper
        // turned off, which is the tool that shipped before run-outs existed.
        let mut plain_labels = BevelSurfaceLabels::new(&mesh, None);
        let plain = match try_build(
            &resolution,
            round,
            0.25,
            segments,
            BevelGroupToolsOptions {
                surface_labels: Some(&mut plain_labels),
                allow_taper: false,
                ..Default::default()
            },
        ) {
            Ok(built) => built.tools,
            Err(refusal) => panic!("{:?}", refusal.reason),
        };
        assert_eq!(tools[0].mesh.vertices.len(), plain[0].mesh.vertices.len());

        for i in 0..plain[0].mesh.vertices.len() {
            assert!(
                (Vector3::from_vector3_float(tools[0].mesh.vertices[i])
                    - Vector3::from_vector3_float(plain[0].mesh.vertices[i]))
                .length()
                    < 1e-9
            );
        }
    }
}
