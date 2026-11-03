//! The tangent chain of "Unexpected Refusals.mcx"'s half cylinder cut, alone, filleted on the
//! level line: the edge where the cylinder's outer wall meets the cut's curved wall, running from
//! the cut's floor (its end face; the outer wall carries on below it) up over the top. The floor
//! end is a convex chain stopping on an unpicked concave crease (floor against the cut wall)
//! against one flat end face, but the chain's first edge leans 5.1 degrees off the floor's normal
//! (the cut's 40-gon facet beside the floor): too far for the square crease clip, not far enough
//! for an end ray. With neither, the field's end is the end vertex's ball and the outer wall's rail
//! curls round it under the floor at every size; between about 0.985 and 1.02 the fill's rows over
//! that curl also cross the floor's end-face fill. The fixture is the app's first hand-over at 1
//! (`unexpected_refusals_regression_tests`).
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::BandGapBuildOptions;
use geometry_processing::bevel::level_line::gap_fill;
use geometry_processing::bevel::level_line::open_gap_fill::ends_site;
use geometry_processing::bevel::level_line::GapFillRefusal;
use polygon_mesh::csg::boolean_processing::BooleanOperandVerdict;

use crate::boolean_kernel_gate;
use crate::end_face_builds::{as_saved, build_chains, default_budget};
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::LevelLineFixture;
use crate::round_mouth_fillet_crossings::crossing_pairs;
use crate::sweep_mesh_measures;

#[test]
#[ignore = "between about 0.985 and 1.02 the round's rows at the floor end reach through the \
            floor's end-face fill; the cause is not yet found"]
fn the_chain_alone_builds_uncrossed_from_0_9_to_2() {
    for size in [0.9, 0.98, 0.985, 0.99, 1.0, 1.01, 1.02, 1.05, 1.5, 2.0] {
        let fixture = Rc::new(LevelLineFixture::load(
            "unexpected-refusals-half-cut-chain-r1",
        ));
        let options = BandGapBuildOptions {
            budget: Some(default_budget(&fixture, false)),
            ..as_saved(&fixture, fixture.known_room)
        };
        let all: Vec<usize> = (0..fixture.chains.len()).collect();
        let gap = build_chains(&fixture, &all, size, &options);
        assert!(gap.built(), "band {:?} at {size}", gap.refusal);

        let filled =
            gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).expect("the fill runs");
        assert_eq!(
            filled.refusal,
            GapFillRefusal::None,
            "{:?} at {size}",
            ends_site()
        );

        let mesh = filled.mesh.as_ref().expect("a filled mesh").borrow();
        let kept = sweep_mesh_measures::kept(&mesh);
        assert!(
            sweep_mesh_measures::is_closed_and_consistent(mesh.vertices(), &kept),
            "open at {size}"
        );
        assert_eq!(
            sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
            1
        );
        assert_eq!(crossing_pairs(&mesh), 0, "crossing at {size}");
        assert_eq!(
            boolean_kernel_gate::verdict(&mesh),
            BooleanOperandVerdict::Clean,
            "the app's gate at {size}"
        );
    }
}

/// The floor's height, the chain's first end vertex (source vertex 10) on it.
const FLOOR: f64 = 6.6508283615112305;
const FLOOR_END: (f64, f64) = (44.451480865478516, -36.34162902832031);

/// The band's rails stop at the floor's plane: none runs on under the floor round the end vertex.
#[test]
#[ignore = "the floor end leans 5.1 degrees off square, so it takes neither the square crease \
            clip nor an end ray, and the outer wall's rail curls round the end vertex's ball \
            under the floor at every size"]
fn the_rails_stop_at_the_floor() {
    for size in [0.5, 0.98, 1.0, 1.05, 2.0] {
        let fixture = Rc::new(LevelLineFixture::load(
            "unexpected-refusals-half-cut-chain-r1",
        ));
        let options = BandGapBuildOptions {
            budget: Some(default_budget(&fixture, false)),
            ..as_saved(&fixture, fixture.known_room)
        };
        let gap = build_chains(&fixture, &[0], size, &options);
        assert!(gap.built(), "band {:?} at {size}", gap.refusal);
        let mesh = gap.mesh.as_ref().expect("a built band has a mesh").borrow();
        let lowest = gap
            .loops
            .iter()
            .flat_map(|l| l.vertices.iter().map(|&v| mesh.position(v)))
            .filter(|p| (p.x - FLOOR_END.0).hypot(p.y - FLOOR_END.1) < 2.0 * size + 0.5)
            .map(|p| p.z)
            .fold(f64::INFINITY, f64::min);
        assert!(
            lowest >= FLOOR - 1e-6,
            "a rail runs down to {lowest} under the floor at {FLOOR} at {size}"
        );
    }
}
