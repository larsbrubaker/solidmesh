//! SEVERAL closed chains filleted in ONE pass of the
//! vertex-centric construction - both mouths of a bore, which is the smallest part where two runs'
//! substitutions have to meet inside one triangle.
//!
//! **Where they meet.** The plate's bore wall is a quad per side, drawn as two triangles running
//! the full thickness, so ONE wall triangle stands on both the top mouth and the bottom mouth.
//! Each run rewrites only the corner at its own chain vertex, and the fan walks are read off the
//! ORIGINAL mesh, per run and per vertex - which is why all the runs are built over one pool
//! against the unmodified part and only then rebuilt in a single pass.
//!
//! **Why the volume reads as a sum.** Each mouth removes its own ring of material and the two rings
//! do not touch, so the two-mouth solid's volume is the top-only volume plus the bottom-only volume
//! less the untouched plate counted twice - an exact identity, read here at 1e-9 relative.

use std::sync::Arc;

use solidmesh::bevel::sections::section_builder_run::try_build_chain;
use solidmesh::bevel::sections::{section_assembly, SectionPool, SectionRun};
use solidmesh::bevel::surface::surface_mesh_closure;
use solidmesh::bevel::{BevelAdvisoryArg, BevelAdvisoryCode};
use solidmesh::topology::{
    bevel_edge_group_resolver, BevelSelectionKind, FaceGroup, FeatureEdgeChain, FeatureEdgeSet,
    MatchTolerances,
};
use solidmesh_math::{Matrix4X4, Vector3};
use solidmesh_mesh::{Mesh, MeshEdgeGraph};

use crate::bore_test_meshes::create_plate_with_a_round_bore;
use crate::{mesh_measurements, rigid_motion_meshes};

const SIDES: i32 = 24;
const BORE_RADIUS: f64 = 3.0;
const THICKNESS: f64 = 10.0;
const RADIUS: f64 = 0.4;
const SEGMENTS: i32 = 8;
const BEND_THRESHOLD_DEGREES: f64 = 20.0;

struct BothMouths {
    built: Mesh,
    top_only: Mesh,
    bottom_only: Mesh,
    pool: SectionPool,
    runs: Vec<SectionRun>,
}

#[test]
fn both_mouths_of_the_bore_are_filleted_in_one_pass() {
    let part = the_plate();
    let both = both_mouths(&part, Matrix4X4::IDENTITY);
    let built = &both.built;

    let (open, open_at) = surface_mesh_closure::open_edge_count(built);
    assert_eq!(
        open, 0,
        "one rebuild and two lofts share one face list over one pool - open at {open_at:?}"
    );

    assert_eq!(
        rigid_motion_meshes::most_faces_on_an_edge(built),
        2,
        "the two runs' substitutions never doubled a face onto an edge"
    );

    assert_eq!(
        mesh_measurements::shell_count(built),
        1,
        "both fillets are part of the plate, not shells standing beside it"
    );

    let (unreadable, unreadable_at) = mesh_measurements::unclassifiable_edges(Some(built));
    assert_eq!(
        unreadable, 0,
        "a second pass reads {unreadable_at} as a torn face"
    );

    assert_eq!(rigid_motion_meshes::two_cornered_triangle_count(built), 0);

    // The identity: what the two mouths take away together is what each takes away on its own,
    // with the plate they were both cut out of counted once.
    let plate = mesh_measurements::signed_volume(&part);
    let top = mesh_measurements::signed_volume(&both.top_only);
    let bottom = mesh_measurements::signed_volume(&both.bottom_only);
    let expected = top + bottom - plate;
    let volume = mesh_measurements::signed_volume(built);

    assert!(
        (volume - expected).abs() / expected.abs() < 1e-9,
        "both mouths remove {}, top {} plus bottom {}",
        plate - volume,
        plate - top,
        plate - bottom
    );

    assert!(
        both.pool.count() > part.vertices.len() as i32,
        "both runs minted their boundaries into the one pool"
    );

    assert_eq!(both.runs.len(), 2);
}

/// No triangle of the answer stands on a mouth vertex of the part: each of the two runs has
/// replaced every one of its own chain vertices on every side, and the wall triangles that carried
/// one of each have had BOTH corners rewritten.
#[test]
fn no_mouth_vertex_of_the_part_survives_either_run() {
    let part = the_plate();
    let built = both_mouths(&part, Matrix4X4::IDENTITY).built;

    let mouths: Vec<Vector3> = part
        .vertices
        .iter()
        .map(|&v| Vector3::from_vector3_float(v))
        .filter(|&at| on_a_mouth(at))
        .collect();

    assert_eq!(
        mouths.len(),
        2 * SIDES as usize,
        "the fixture's two mouths each walk a closed loop of 24"
    );

    let survivors = built
        .vertices
        .iter()
        .map(|&v| Vector3::from_vector3_float(v))
        .filter(|&at| mouths.iter().any(|&mouth| (at - mouth).length() < 1e-9))
        .count();

    assert_eq!(
        survivors, 0,
        "a surviving mouth vertex would be a corner one run rewrote and the other left standing"
    );
}

/// The same topology under every rigid motion the harness stands the part by - the two-run pass
/// is as much a fact about the picks as the one-run pass is.
#[test]
fn the_two_run_pass_has_the_same_topology_wherever_the_part_stands() {
    let at_the_origin = rigid_motion_meshes::signature_of(Some(
        &both_mouths(&the_plate(), Matrix4X4::IDENTITY).built,
    ));

    for (name, motion) in rigid_motion_meshes::the_motions() {
        let mut part = the_plate();

        rigid_motion_meshes::move_mesh(&mut part, motion);

        let moved =
            rigid_motion_meshes::signature_of(Some(&both_mouths(&part, motion.inverted()).built));

        assert_eq!(
            moved, at_the_origin,
            "the build moved by {name} has the topology it had at the origin"
        );
    }
}

/// The same mouth handed twice is two runs on one chain vertex, and the rebuild refuses it AT THE
/// STATION rather than rewriting a corner that has already been given away.
#[test]
fn two_runs_on_one_chain_vertex_are_refused_at_the_station() {
    let part = the_plate();
    let graph = part.get_mesh_edge_graph();
    let features = Arc::new(FeatureEdgeSet::create_with_graph(
        &part,
        &graph,
        BEND_THRESHOLD_DEGREES,
    ));
    let mut pool = SectionPool::new(&part.vertices);

    let once = the_run_on(
        &the_mouth(&features, THICKNESS / 2.0, Matrix4X4::IDENTITY),
        &part,
        &graph,
        &mut pool,
    );
    let twice = the_run_on(
        &the_mouth(&features, THICKNESS / 2.0, Matrix4X4::IDENTITY),
        &part,
        &graph,
        &mut pool,
    );

    let failure =
        match section_assembly::try_assemble_runs(&part, &graph, &[once, twice], &mut pool) {
            Ok(_) => panic!("one chain vertex cannot be the centre of two fillets"),
            Err(failure) => failure,
        };

    let advisory = failure
        .advisory()
        .unwrap_or_else(|| panic!("the refusal carries an advisory - instead {failure:?}"));

    assert_eq!(
        advisory.code(),
        BevelAdvisoryCode::SurfaceCornerNotSupported
    );

    assert_eq!(
        advisory.args().len(),
        1,
        "the refusal points at the place on the part, not at the pick"
    );

    let BevelAdvisoryArg::Vector3(station) = advisory.args()[0] else {
        panic!(
            "the refusal's argument is a place - instead {:?}",
            advisory.args()[0]
        );
    };

    assert!(
        on_a_mouth(station),
        "the station is a mouth vertex of the fixture - instead {station:?}"
    );
}

fn the_plate() -> Mesh {
    create_plate_with_a_round_bore(SIDES, BORE_RADIUS, THICKNESS, 6.0)
}

/// Both mouths in one pass, together with each mouth's own single-run build for the volume
/// identity.
fn both_mouths(part: &Mesh, back_to_where_it_was_made: Matrix4X4) -> BothMouths {
    let graph = part.get_mesh_edge_graph();
    let features = Arc::new(FeatureEdgeSet::create_with_graph(
        part,
        &graph,
        BEND_THRESHOLD_DEGREES,
    ));
    let top = the_mouth(&features, THICKNESS / 2.0, back_to_where_it_was_made);
    let bottom = the_mouth(&features, -THICKNESS / 2.0, back_to_where_it_was_made);

    let mut pool = SectionPool::new(&part.vertices);

    // Both runs are built against the UNMODIFIED part over the one pool, which is what lets the
    // single rebuild pass below substitute them independently.
    let runs = vec![
        the_run_on(&top, part, &graph, &mut pool),
        the_run_on(&bottom, part, &graph, &mut pool),
    ];

    let built = section_assembly::try_assemble_runs(part, &graph, &runs, &mut pool).unwrap_or_else(
        |failure| panic!("both mouths of the straight bore assemble - instead {failure:?}"),
    );

    let top_only = one_mouth(part, &graph, &top);
    let bottom_only = one_mouth(part, &graph, &bottom);

    BothMouths {
        built,
        top_only,
        bottom_only,
        pool,
        runs,
    }
}

fn one_mouth(part: &Mesh, graph: &MeshEdgeGraph, chain: &FeatureEdgeChain) -> Mesh {
    let mut pool = SectionPool::new(&part.vertices);
    let run = the_run_on(chain, part, graph, &mut pool);

    section_assembly::try_assemble(part, graph, &run, &mut pool).unwrap_or_else(|failure| {
        panic!("one mouth of the straight bore assembles - instead {failure:?}")
    })
}

fn the_run_on(
    chain: &FeatureEdgeChain,
    part: &Mesh,
    graph: &MeshEdgeGraph,
    pool: &mut SectionPool,
) -> SectionRun {
    try_build_chain(Some(chain), part, graph, pool, RADIUS, SEGMENTS, true)
        .unwrap_or_else(|failure| panic!("the mouth's sections build - instead {failure:?}"))
}

/// The mouth loop at `height`, picked the way the robustness harness picks: through
/// `bevel_edge_group_resolver::resolve_crease`, which at 24 sides chooses among the three feature
/// edges meeting at every mouth vertex.
fn the_mouth(
    features: &Arc<FeatureEdgeSet>,
    height: f64,
    back_to_where_it_was_made: Matrix4X4,
) -> FeatureEdgeChain {
    for i in 0..features.count() {
        let edge = &features.edges()[i];

        if !at_the_height(
            edge.start.transform_position(back_to_where_it_was_made),
            height,
        ) || !at_the_height(
            edge.end.transform_position(back_to_where_it_was_made),
            height,
        ) {
            continue;
        }

        let group = bevel_edge_group_resolver::create_from_edge(
            features,
            i as i32,
            BevelSelectionKind::TangentChain,
            FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
        )
        .expect("the mouth edge is one of the part's feature edges");

        let chain = bevel_edge_group_resolver::resolve_crease(
            &group,
            features.clone(),
            FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
            FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
            MatchTolerances::default(),
        )
        .chain;

        match chain {
            Some(chain) if chain.is_closed() && chain.count() == SIDES as usize => return chain,
            other => panic!(
                "the mouth at {height} walks a closed loop of {SIDES} - it walked {}",
                other.map_or(0, |c| c.count())
            ),
        }
    }

    panic!("the fixture has no mouth at {height}");
}

/// On the mouth ring at `height`, read as loosely as the robustness harness reads it: a part stood
/// 250 mm out and turned rides a float lattice of about 2e-5, and the nearest thing this could
/// confuse the ring with is 6 mm away at the plate's rim.
fn at_the_height(at: Vector3, height: f64) -> bool {
    (at.z - height).abs() < 1e-3
        && (Vector3::new(at.x, at.y, 0.0).length() - BORE_RADIUS).abs() < 1e-3
}

fn on_a_mouth(at: Vector3) -> bool {
    at_the_height(at, THICKNESS / 2.0) || at_the_height(at, -THICKNESS / 2.0)
}
