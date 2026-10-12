//! Port of `SectionPiercedFaceTests.Junction.cs`, one part of the partial class
//! `SectionPiercedFaceTests`: the section the junction is drawn as
//! (`section_junction_build::try_build`) on a raked bore cube's top-face pick.
//!
//! Only the case that builds its own mesh is here. Every other test of this part - and of
//! `SectionPiercedFaceTests.Pass.cs`, `.Seam.cs` and the class's main file - reads its plans
//! through the class's `ThePlans`, which builds the pick through `SurfaceTestBuild.TryBevel`, and
//! waits on slice S28 with those helpers (`ThePlans`, `TheTopFace`, `TheSharpTurnsOf`,
//! `TheJunctionAt`, ...): `TheJunctionStationIsRecognised`,
//! `TheCornerDefersTheJunctionRatherThanRefusingIt`, `TheJunctionsSamplesStandOnBothCanals`,
//! `TheJunctionsPointsAreTheOnesTheReadingNames`, `ThePatchStandsOnTheFoldInsideThePart`,
//! `TheJunctionsNeighbourhoodClosesByIdentity`, `TheJunctionIsTheSameUnderTheRigidMotions`,
//! `ARefusedJunctionLeavesNoOrphansInThePool`, `TheRunKeepsTheJunctionsAndLeavesEveryOtherStationAlone`,
//! `TheRebuiltPartClosesAroundTheJunctions`, `EachJunctionsWallEndsArePlacedInTheirOwnWalls`,
//! `EachJunctionsWallEndsStandOnThePartsSurface` (both radii) and
//! `ASplitStationBesideAJunctionIsLaidOnTheCarriedWallEnd`.

use std::time::Instant;

use geometry_processing::bevel::sections::section_junction_build::{self, JunctionRefusal};
use geometry_processing::bevel::sections::SectionPool;
use geometry_processing::bevel::BevelAdvisoryCode;
use geometry_processing::topology::{ChainEdge, FeatureEdge};
use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::{math_helper, Vector3};

/// A LEAVING WALL THAT DOES NOT STAND ON THE ARRIVING CANAL'S SWEEP SIDE is refused by name, and in
/// bounded time. The sweep runs 0 at the shared flat side and 1 at the arriving edge's own wall; a
/// leaving wall whose own fold turns the OTHER way off the flat side reads at a NEGATIVE parameter,
/// and the row of samples the junction draws between them has a step of zero or less - the
/// carry-on loop marches away from its end and never reaches it, growing a list until the drag runs
/// out of memory. The refusal must come before any sampling; this test's own completion is the
/// proof.
#[test]
fn a_junction_whose_leaving_wall_turns_the_other_way_is_refused_rather_than_sampled_forever() {
    let part = the_wall_turned_the_other_way();
    let graph = part.get_mesh_edge_graph();

    // The two edges of the shared flat top face that meet at the origin, each running so that the
    // top face is on the same side of both - which is what makes them one chain through the vertex.
    let arriving = the_chain_edge(&part, &graph, Vector3::new(0.0, -10.0, 0.0), Vector3::ZERO);
    let leaving = the_chain_edge(&part, &graph, Vector3::ZERO, Vector3::new(10.0, 0.0, 0.0));
    let mut pool = SectionPool::new(&part.vertices);
    let watch = Instant::now();

    let built = section_junction_build::try_build(
        &arriving,
        &leaving,
        Vector3::UNIT_X,
        &mut pool,
        1.0,
        8,
        None,
    );

    let elapsed = watch.elapsed();

    let failure = match built {
        Ok(_) => panic!(
            "the leaving wall stands behind the shared flat side, so there is no row to draw"
        ),
        Err(JunctionRefusal::Refused(failure)) => failure,
        Err(JunctionRefusal::NotANumber(nan)) => panic!("refused on a NaN instead: {nan:?}"),
    };

    assert_eq!(failure.code(), BevelAdvisoryCode::SurfaceCornerNotSupported);

    assert!(
        elapsed.as_secs_f64() < 5.0,
        "and it is refused before any sampling - the reading itself is the whole cost"
    );

    assert_eq!(
        pool.count(),
        part.vertices.len() as i32,
        "a refused junction mints nothing"
    );
}

/// A SHARED FLAT TOP FACE whose two boundary edges at the origin carry walls that turn OPPOSITE
/// ways off it: the x = 0 wall hangs straight down at ninety degrees, and the y = 0 face dips twenty
/// degrees back UNDER the top face rather than away from it. Slivery on purpose - it is the input
/// the junction has to refuse, not a part anyone would bevel.
fn the_wall_turned_the_other_way() -> Mesh {
    let mut mesh = Mesh::new();
    let dip = math_helper::degrees_to_radians(20.0);
    let under = Vector3::new(0.0, -5.0, -5.0 * dip.tan());
    let far_under = Vector3::new(10.0, -5.0, -5.0 * dip.tan());

    // The top face, z = 0, over x in [0, 10] and y in [-10, 0].
    mesh.create_face(&[
        Vector3::ZERO,
        Vector3::new(0.0, -10.0, 0.0),
        Vector3::new(10.0, -10.0, 0.0),
    ]);
    mesh.create_face(&[
        Vector3::ZERO,
        Vector3::new(10.0, -10.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
    ]);

    // The x = 0 wall, hanging down - the ARRIVING edge's own wall, at ninety degrees to the top.
    mesh.create_face(&[
        Vector3::ZERO,
        Vector3::new(0.0, 0.0, -5.0),
        Vector3::new(0.0, -10.0, -5.0),
    ]);
    mesh.create_face(&[
        Vector3::ZERO,
        Vector3::new(0.0, -10.0, -5.0),
        Vector3::new(0.0, -10.0, 0.0),
    ]);

    // The y = 0 face, dipping back under the top face - the LEAVING edge's wall, turned the other
    // way.
    mesh.create_face(&[Vector3::ZERO, under, far_under]);
    mesh.create_face(&[Vector3::ZERO, far_under, Vector3::new(10.0, 0.0, 0.0)]);

    mesh.clean_and_merge();

    mesh
}

/// The chain edge running from one position to another, classified off the mesh itself.
fn the_chain_edge(mesh: &Mesh, graph: &MeshEdgeGraph, from: Vector3, to: Vector3) -> ChainEdge {
    for i in 0..graph.edge_count() {
        let Some(edge) =
            FeatureEdge::try_classify(mesh, graph, i, FeatureEdge::default_flat_angle_degrees())
        else {
            continue;
        };

        if (edge.start - from).length() < 1e-6 && (edge.end - to).length() < 1e-6 {
            return ChainEdge::new(i, edge, false);
        }

        if (edge.end - from).length() < 1e-6 && (edge.start - to).length() < 1e-6 {
            return ChainEdge::new(i, edge, true);
        }
    }

    panic!("no feature edge from {from} to {to}");
}
