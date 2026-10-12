//! Port of `SectionManyHolesTests.cs`: THE ONE ROW OF "Many Holes.mcx", on the part's own mesh - a
//! ported barrel picked as one FaceGroup whose rim is fifty closed loops, asked for a 0.5 mm fillet.
//! What the KERNEL answers for it, loop by loop, is read here off [`crate::many_holes_test_mesh`]
//! through [`crate::many_holes_rim_reading`] instead of through a MatterCADLib build and an .mcx
//! load per turn.
//!
//! **The reading as it stands, loop by loop, each loop built and assembled ALONE at 0.5.** Of the
//! fifty closed loops FORTY are sound - closed, manifold, one shell - ONE builds and then refuses in
//! the assembly (`SurfaceSmoothSideNotCut`, the cut's own name `ARingWillNotTriangulate`), 7 refuse
//! `SizeExceedsFaceWidth` in the build and 2 refuse `SurfaceCornerNotSupported` there: the 65-edge
//! mouth and the 104-edge top rim, which meet at the pick's one degree-four vertex (9064). The room
//! counts its creases through the build's own `SectionBuilder.TryCreasesInTheFan`, so 41 of the
//! fifty have room for the 0.5 and only those two read nothing.
//!
//! **What the pick is.** Chained on shared vertex indices the pick's 3301 edges come to FIFTY runs
//! and every one is CLOSED: 47 clean hole mouths of 64 edges, one odd mouth of 65 that breaks out
//! through the top rim, and the barrel's two flat end rims, 104 edges at the top and 124 at the
//! bottom. The 104 of the app's `FaceRegionRimIsPartial(104)` is the top rim, edge for edge.
//!
//! The fixture IS conditioned: it was dumped where `BevelFeaturePartBuilder` hands the construction
//! its mesh, after `BevelWorkingMesh.CreateFrom`, and nothing is re-conditioned here.
//!
//! Two C# tests read the barrel's outer wall through `SurfaceTestBuild.TryBevel` (C#'s
//! `TheOuterWallAt`), which lands with the surface bevel builder (S28):
//! `TheRoomRefusesThePickBeforeTheRouterIsReached` lands with it, and
//! `EveryHoleTakesItsFilletThroughTheSections` is `[Skip]` (RED) in C# - forty of the fifty loops
//! take 0.5 soundly and ten still refuse - so it is recorded as skipped, not ported.

use std::collections::HashSet;

use polygon_mesh::Mesh;
use vector_math::Vector3;

use geometry_processing::bevel::bevel_chain_linker::link;
use geometry_processing::bevel::sections::section_builder_run::try_build_loop;
use geometry_processing::bevel::sections::section_fan_walk::edges_by_vertex_pair;
use geometry_processing::bevel::sections::section_smooth_seam::a_walk_crossed_a_fold;
use geometry_processing::bevel::sections::section_smooth_side::{
    try_cut_the_smooth_side, SmoothSideRefusal,
};
use geometry_processing::bevel::sections::section_smooth_side_boundary::SmoothSideFailure;
use geometry_processing::bevel::sections::SectionPool;
use geometry_processing::bevel::BevelAdvisoryArg;
use geometry_processing::topology::bevel_edge_group_resolver::{self, BevelEdgeGroupResolution};

use crate::many_holes_rim_reading::{self, LoopAnswer};
use crate::many_holes_test_mesh;

/// THE FIXTURE IS THE PART THE APP BUILT: the text resource reads back to the welded working mesh
/// the part builder hands over, and its anchors resolve to the pick that was dumped beside it.
#[test]
fn the_fixture_is_the_working_mesh_and_the_pick_the_app_resolved() {
    let part = many_holes_test_mesh::the_part();

    assert_eq!(part.vertices.len(), 15536);
    assert_eq!(part.faces.len(), 31268);

    let picked = many_holes_test_mesh::the_picked_edges();

    assert_eq!(
        picked.len() as i32,
        many_holes_test_mesh::RESOLVED_EDGE_COUNT,
        "the dump carries the pick the app came to, edge by edge"
    );

    let resolution = the_rim(&part);

    assert!(
        resolution.is_resolved(),
        "{}",
        resolution
            .failure_reason
            .as_ref()
            .map_or_else(|| "resolved".to_string(), |why| format!("{:?}", why.code()))
    );

    assert_eq!(
        resolution.edges.len() as i32,
        many_holes_test_mesh::RESOLVED_EDGE_COUNT,
        "re-resolving the anchors on this mesh comes to the same edges that were dumped - which is \
         3301, where the owner file's panel row reads 3315"
    );
}

/// THE LOOP CENSUS. Chained on shared vertex indices, the pick's 3301 edges come to FIFTY runs and
/// every one of them is CLOSED - there is not one open run in this pick, and `BevelChainLinker`
/// reads the same fifty closed chains off the resolution itself. One vertex of degree four is the
/// whole of the difference from the app's 3315/52: a mouth that breaks out through the top rim and
/// shares that vertex with it.
#[test]
fn the_rim_is_fifty_closed_loops_and_the_hundred_and_four_is_the_top_rim() {
    let part = many_holes_test_mesh::the_part();
    let census = many_holes_rim_reading::chain(&many_holes_test_mesh::the_picked_edges());

    assert_eq!(census.runs.len(), 50);

    assert_eq!(
        census.open_runs.len(),
        0,
        "chained on vertex indices this pick has NO open run - {}",
        many_holes_rim_reading::describe(&part, &census)
    );

    assert_eq!(census.loose_edge_count, 0);

    // The linker the walkback's own rim check runs is the one that must agree, so ask it too.
    let linked = link(&the_rim(&part).edges);

    assert_eq!(linked.len(), 50);
    assert_eq!(linked.iter().filter(|chain| !chain.is_closed).count(), 0);

    let shapes: Vec<_> = census
        .closed_loops
        .iter()
        .map(|run| many_holes_rim_reading::shape(&part, run))
        .collect();

    let top = single(shapes.iter().filter(|shape| shape.edge_count == 104));

    assert!(
        (11.4..=11.5).contains(&top.mean_radius),
        "the 104 of FaceRegionRimIsPartial(104) is this loop - the barrel's flat top rim: {}",
        top.mean_radius
    );

    assert!(top.planarity < 1e-4, "{}", top.planarity);
    assert!(top.radius_spread < 1e-4, "{}", top.radius_spread);

    let bottom = single(shapes.iter().filter(|shape| shape.edge_count == 124));

    assert!(
        (16.3..=16.4).contains(&bottom.mean_radius),
        "{}",
        bottom.mean_radius
    );

    // One vertex of degree four is the whole of the difference: a mouth that breaks out through
    // the top rim and shares that vertex with it.
    let junctions: Vec<(usize, i32)> = census
        .degree_histogram
        .iter()
        .filter(|(&degree, _)| degree != 2)
        .map(|(&degree, &count)| (degree, count))
        .collect();

    assert_eq!(junctions.len(), 1);
    assert_eq!(junctions[0].0, 4);
    assert_eq!(junctions[0].1, 1);
}

/// THE MOUTH TABLE. Forty-eight of the fifty loops are hole mouths and they are all one kind: a
/// 64-edge ring of mean radius 1.477 and perimeter 9.28, tilted with the barrel. Forty-seven are
/// clean rings; ONE is not (65 edges, spread 0.197) - the mouth that breaks out through the top
/// rim. None of them is a circle on a FLAT face: the barrel's outer wall is TAPERED, so every
/// mouth's own fitted plane leans +0.1128 in z, and the odd one's highest vertex lies ON the top
/// rim's plane.
#[test]
fn every_mouth_is_a_ring_on_the_barrels_tapered_wall_and_one_touches_the_top_rim() {
    let part = many_holes_test_mesh::the_part();
    let census = many_holes_rim_reading::chain(&many_holes_test_mesh::the_picked_edges());

    let shapes: Vec<_> = census
        .closed_loops
        .iter()
        .map(|run| many_holes_rim_reading::shape(&part, run))
        .collect();
    let mouths: Vec<_> = shapes
        .iter()
        .filter(|shape| shape.edge_count < 100)
        .collect();

    assert_eq!(mouths.len(), 48);

    assert_eq!(
        mouths.iter().filter(|shape| shape.edge_count == 64).count(),
        47
    );
    assert_eq!(
        mouths.iter().filter(|shape| shape.edge_count == 65).count(),
        1
    );

    for mouth in &mouths {
        assert!(
            (1.47..=1.49).contains(&mouth.mean_radius),
            "every hole is the same bore: {}",
            mouth.mean_radius
        );

        // The taper: the mouth's own plane leans the same +0.1128 in z on every hole.
        assert!(
            (0.112..=0.114).contains(&mouth.normal.z),
            "the picked face is a TAPERED barrel, so no mouth sits on a flat face: {}",
            mouth.normal.z
        );
    }

    let plain: Vec<_> = mouths
        .iter()
        .filter(|shape| shape.edge_count == 64)
        .collect();

    let widest = plain
        .iter()
        .map(|shape| shape.radius_spread)
        .fold(f64::MIN, f64::max);
    assert!(
        widest < 0.021,
        "a clean ring wanders a hundredth; the odd one wanders ten times that: {widest}"
    );

    let least_flat = plain
        .iter()
        .map(|shape| shape.planarity)
        .fold(f64::MIN, f64::max);
    assert!(
        least_flat < 0.05,
        "not zero - a bore through a tapered wall cuts a non-planar curve: {least_flat}"
    );

    let odd = single(mouths.iter().filter(|shape| shape.edge_count == 65));

    assert!(odd.radius_spread > 0.19, "{}", odd.radius_spread);

    let top = single(shapes.iter().filter(|shape| shape.edge_count == 104));
    let odd_run = single(
        census
            .closed_loops
            .iter()
            .filter(|run| run.edges.len() == 65),
    );

    // C#'s OrderByDescending(..).First(): the first of the highest, in the run's own order.
    let mut highest = odd_run.vertices[0];
    for &vertex in &odd_run.vertices {
        if part.vertices[vertex as usize].z > part.vertices[highest as usize].z {
            highest = vertex;
        }
    }

    // The odd mouth's highest vertex is not NEAR the top rim's plane, it is ON it - and it is the
    // pick's one degree-four vertex. The mouth and the rim meet at exactly one point.
    let off = (f64::from(part.vertices[highest as usize].z) - top.centre.z).abs();
    assert!(
        off < 1e-4,
        "the odd mouth reaches the rim's own plane: {off}"
    );

    let degree_four: Vec<i32> = census
        .degree_histogram
        .iter()
        .filter(|(&degree, _)| degree == 4)
        .map(|(_, &count)| count)
        .collect();
    assert_eq!(
        single(degree_four.iter()),
        &1,
        "and that touch is the pick's only vertex of degree four - two loops through one point, \
         which is what makes the top rim's chain a place a linker or a plan can read two ways, and \
         where the app's extra 14 edges are taken"
    );
}

/// THE WHOLE PICK READ LOOP BY LOOP, alone: each of the fifty closed loops handed to
/// `SectionBuilder.TryBuildLoop` and then to `SectionAssembly.TryAssemble` by itself, with
/// `SectionRoom.TryFit`'s answer for the same loop beside it. The ratchet is every row, by name -
/// a loop's name is its edge count and its centre to a tenth of a millimetre, so a row survives
/// any reordering of the pick and names a place on the barrel.
#[test]
fn every_loop_of_the_pick_read_alone_is_forty_sound_mouths_and_ten_refusals() {
    let read = the_loop_table();

    let mut rows: Vec<String> = read
        .iter()
        .map(|answer| format!("{} {}", answer.name, answer.outcome()))
        .collect();
    let mut wanted: Vec<String> = THE_RATCHET.iter().map(|row| row.to_string()).collect();

    // C#'s IsEquivalentTo: the same rows, in any order.
    rows.sort();
    wanted.sort();
    assert_eq!(rows, wanted);

    assert_eq!(
        read.iter()
            .filter(|answer| answer.outcome() == "sound")
            .count(),
        40
    );

    assert_eq!(
        read.iter().filter(|answer| answer.fits).count(),
        41,
        "FORTY-ONE loops have room for the 0.5 the owner file asks - it was six while the room \
         counted its creases by coplanarity - because BOTH the build's fan and the room's read them \
         with the operation's own feature set"
    );

    assert_eq!(
        read.iter()
            .filter(|answer| answer.largest_that_fits <= 0.0)
            .count(),
        2,
        "only the 65-edge mouth and the 104-edge top rim, the two loops that meet at the pick's one \
         degree-four vertex, are left without a size at all"
    );

    let most = read
        .iter()
        .map(|answer| answer.largest_that_fits)
        .fold(f64::MIN, f64::max);
    assert!(most < 1.1, "the most room any loop has is 1.091: {most}");
}

/// THE MOUTH THAT TOUCHES THE TOP RIM and the TOP RIM ITSELF refuse at the SAME point - vertex
/// 9064, the pick's one vertex of degree four, where the two loops meet. A fan walked at a station
/// that two picked loops pass through cannot be read as one corner, and neither loop can be built
/// until it can - so this one vertex refuses two of the fifty rows, not one.
#[test]
fn the_touching_mouth_and_the_top_rim_both_refuse_at_their_shared_vertex() {
    let read = the_loop_table();

    let touching = single(read.iter().filter(|answer| answer.edge_count == 65));
    let rim = single(read.iter().filter(|answer| answer.edge_count == 104));

    assert!(!touching.built);
    assert!(!rim.built);

    assert_eq!(
        LoopAnswer::code_and_station(touching.build_why.as_ref()),
        LoopAnswer::code_and_station(rim.build_why.as_ref()),
        "both loops stop at the vertex they share"
    );

    let station = match touching
        .build_why
        .as_ref()
        .and_then(|why| why.args().first())
    {
        Some(BevelAdvisoryArg::Vector3(at)) => *at,
        other => panic!("the refusal names its station as a point, not {other:?}"),
    };

    let shared = Vector3::from_vector3_float(many_holes_test_mesh::the_part().vertices[9064]);

    assert!(
        (station - shared).length() < 1e-9,
        "and that shared vertex is 9064, the pick's only vertex of degree four"
    );
}

/// WHAT THE ASSEMBLY'S REFUSALS WERE, MEASURED: the smooth side's facet is a flat piece of the wall
/// WITH HOLES IN IT. On the mouth this row reads - (-127.2, 111.5, -89.3), which CUTS now and is
/// sound in the loop table - the piece is 178 faces and its outline is three rings, 64, 64 and 40:
/// an outer ring round a patch holding TWO WHOLE BORE MOUTHS, one of which is the chain being
/// filleted. The facet is cut from its rings AS THEY STAND, each untouched mouth handed to the
/// region whose ring encloses it and a kept region with holes spliced back to a single ring
/// (`SectionSmoothSide.Holes.cs`), and the ears take that spliced ring now that they read its
/// crossings in the ring's own plane (`SurfaceRingEars.ItIsSimple`). So this row's claim is that THE
/// FACET CUTS.
#[test]
fn the_refused_facet_is_a_flat_piece_with_two_whole_bore_mouths_inside_it() {
    let part = many_holes_test_mesh::the_part();
    let graph = part.get_mesh_edge_graph();
    let resolution = the_rim(&part);

    let loops = many_holes_rim_reading::the_loops_in_order(&part, &resolution);
    let one = &loops[1];

    let mut pool = SectionPool::new(&part.vertices);
    let ring: Vec<i32> = one.edges.iter().map(|edge| edge.from_vertex()).collect();

    let run = try_build_loop(
        &ring,
        Some(&resolution.features),
        &part,
        &graph,
        &mut pool,
        many_holes_test_mesh::SIZE,
        many_holes_test_mesh::SEGMENTS,
        true,
    )
    .unwrap_or_else(|why| panic!("built: {why:?}"));

    // The side asked for is the one this run's OWN walks folded on. Its region is not one plane -
    // neither side's is, on a tessellated ball - but a region only sends a side that folded nowhere
    // to the cut when some other run's seam is already on it, and this row builds one run.
    let left = a_walk_crossed_a_fold(&run, true);

    assert!(
        left || a_walk_crossed_a_fold(&run, false),
        "a mouth on this tessellated wall has one smooth side"
    );

    let (cut, done) = try_cut_the_smooth_side(
        &part,
        &graph,
        &edges_by_vertex_pair(&graph),
        &mut pool,
        &run,
        left,
        run.is_a_feature_edge.as_deref(),
    );

    // The outline does not stop this facet, and nor do the ears any longer stop the spliced ring:
    // it is 121 vertices threaded through two whole mouths, and its two SIDES FACING ONE ANOTHER
    // ACROSS THE PIECE - all but antiparallel, a hundredth of a millimetre off the piece's plane -
    // were read as crossing in a plane made of their own two directions. The same wall on the small
    // fixture is `SectionSmoothSideBridgedHolesTests.TheMouthInAPieceWithHolesCuts`.
    let refusal = refusal_of(done, &format!("rings {:?}", cut.rings_of_the_refused_facet));

    assert_eq!(
        refusal,
        SmoothSideRefusal::None,
        "rings {:?}",
        cut.rings_of_the_refused_facet
    );

    assert!(
        cut.rings_of_the_refused_facet.is_empty(),
        "no facet was refused, so there are no rings for it to report"
    );
}

/// THE ELEVEN LOOPS THAT ONCE REFUSED IN THE ASSEMBLY, cut one by one: TEN CUT and one refuses
/// `ARingWillNotTriangulate`. All ten joins that used to refuse read `FacesNotConnected` with not
/// one unfolding taken: the two faces stood NINE OR TEN faces apart on this tapered wall's long thin
/// slivers, past the strip search's old reach of eight. `SectionSurfaceWalk`'s JoinStripReach
/// carries that reading.
#[test]
fn the_assembly_refusals_are_one_ring_and_no_join() {
    let part = many_holes_test_mesh::the_part();
    let graph = part.get_mesh_edge_graph();
    let resolution = the_rim(&part);
    let edge_at_pair = edges_by_vertex_pair(&graph);

    let loops = many_holes_rim_reading::the_loops_in_order(&part, &resolution);

    let mut read = Vec::new();

    // The eleven rows that answered assembly:SurfaceSmoothSideNotCut while the strip search reached
    // eight faces - by index into the table's own order.
    for i in [9usize, 11, 23, 24, 25, 33, 34, 35, 36, 37, 38] {
        let mut pool = SectionPool::new(&part.vertices);
        let ring: Vec<i32> = loops[i]
            .edges
            .iter()
            .map(|edge| edge.from_vertex())
            .collect();

        let run = try_build_loop(
            &ring,
            Some(&resolution.features),
            &part,
            &graph,
            &mut pool,
            many_holes_test_mesh::SIZE,
            many_holes_test_mesh::SEGMENTS,
            true,
        )
        .unwrap_or_else(|why| panic!("loop {i}: {why:?}"));

        for left in [true, false] {
            if !a_walk_crossed_a_fold(&run, left) {
                continue;
            }

            let (_, done) = try_cut_the_smooth_side(
                &part,
                &graph,
                &edge_at_pair,
                &mut pool,
                &run,
                left,
                run.is_a_feature_edge.as_deref(),
            );

            let refusal = refusal_of(done, &format!("loop {i}"));

            read.push(if refusal == SmoothSideRefusal::None {
                format!("{i} cut")
            } else {
                format!("{i} {refusal:?}")
            });
        }
    }

    let mut wanted: Vec<String> = [
        "9 cut",
        "11 ARingWillNotTriangulate",
        "23 cut",
        "24 cut",
        "25 cut",
        "33 cut",
        "34 cut",
        "35 cut",
        "36 cut",
        "37 cut",
        "38 cut",
    ]
    .iter()
    .map(|row| row.to_string())
    .collect();

    // C#'s IsEquivalentTo: the same rows, in any order.
    read.sort();
    wanted.sort();
    assert_eq!(read, wanted);
}

/// The cut's answer as C#'s `out refusal`: `None` where it cut. A NaN point is C#'s throw.
fn refusal_of(done: Result<(), SmoothSideFailure>, what: &str) -> SmoothSideRefusal {
    match done {
        Ok(()) => SmoothSideRefusal::None,
        Err(SmoothSideFailure::Refused(refusal, _)) => refusal,
        Err(SmoothSideFailure::NotANumber(nan)) => {
            panic!("{what}: the cut minted a NaN point: {nan}")
        }
    }
}

/// Every closed loop read alone, once per test.
fn the_loop_table() -> Vec<LoopAnswer> {
    let part = many_holes_test_mesh::the_part();
    let resolution = the_rim(&part);

    many_holes_rim_reading::read_every_loop(
        &part,
        &resolution,
        many_holes_test_mesh::SIZE,
        many_holes_test_mesh::SEGMENTS,
        None,
        None::<&HashSet<usize>>,
    )
}

fn the_rim(part: &Mesh) -> BevelEdgeGroupResolution {
    let (mut resolutions, _) =
        bevel_edge_group_resolver::resolve_all(&[many_holes_test_mesh::the_outer_wall()], part);

    resolutions.remove(0)
}

/// C#'s `Single`: the one item, failing on none or several.
fn single<T>(mut items: impl Iterator<Item = T>) -> T {
    let first = items.next().expect("one item, not none");
    assert!(items.next().is_none(), "one item, not several");
    first
}

/// WHAT EACH OF THE FIFTY LOOPS ANSWERS ALONE at 0.5, in the reading's own order (up the barrel by
/// the loop's centre). Kept whole rather than counted so that a loop which changes its answer names
/// ITSELF in the failure.
const THE_RATCHET: [&str; 50] = [
    "n124@(-111.5,111.5,-95.0) sound",
    "n64@(-127.2,111.5,-89.3) sound",
    "n64@(-95.8,111.5,-89.3) sound",
    "n64@(-122.4,100.3,-89.3) sound",
    "n64@(-100.3,100.6,-89.3) sound",
    "n64@(-100.3,122.4,-89.3) sound",
    "n64@(-122.4,122.7,-89.3) sound",
    "n64@(-111.2,95.8,-89.3) sound",
    "n64@(-111.2,127.2,-89.3) sound",
    "n64@(-126.4,111.5,-82.1) sound",
    "n64@(-96.6,111.5,-82.1) sound",
    "n64@(-111.2,96.6,-82.1) assembly:SurfaceSmoothSideNotCut",
    "n64@(-121.8,100.8,-82.1) sound",
    "n64@(-100.8,101.2,-82.1) sound",
    "n64@(-100.8,121.8,-82.1) sound",
    "n64@(-121.8,122.2,-82.1) sound",
    "n64@(-111.2,126.4,-82.1) sound",
    "n64@(-125.5,111.5,-74.9) sound",
    "n64@(-97.5,111.5,-74.9) sound",
    "n64@(-121.3,101.4,-74.9) sound",
    "n64@(-101.4,101.7,-74.9) sound",
    "n64@(-101.4,121.3,-74.9) sound",
    "n64@(-121.3,121.6,-74.9) sound",
    "n64@(-111.2,97.5,-74.9) sound",
    "n64@(-111.2,125.5,-74.9) sound",
    "n64@(-124.7,111.5,-67.7) sound",
    "n64@(-98.3,111.5,-67.7) sound",
    "n64@(-120.7,102.0,-67.7) sound",
    "n64@(-102.0,102.3,-67.7) sound",
    "n64@(-102.0,120.7,-67.7) sound",
    "n64@(-120.7,121.0,-67.7) sound",
    "n64@(-111.2,98.3,-67.7) sound",
    "n64@(-111.2,124.7,-67.7) sound",
    "n64@(-99.1,111.5,-60.5) sound",
    "n64@(-102.6,102.9,-60.5) sound",
    "n64@(-102.6,120.1,-60.5) sound",
    "n64@(-123.9,111.5,-60.5) sound",
    "n64@(-120.1,102.6,-60.5) sound",
    "n64@(-120.1,120.4,-60.5) sound",
    "n64@(-111.2,99.1,-60.5) sound",
    "n64@(-111.2,123.9,-60.5) sound",
    "n64@(-99.9,111.5,-53.3) build:SizeExceedsFaceWidth",
    "n64@(-103.1,103.5,-53.3) build:SizeExceedsFaceWidth",
    "n64@(-103.1,119.5,-53.3) build:SizeExceedsFaceWidth",
    "n64@(-123.1,111.5,-53.3) build:SizeExceedsFaceWidth",
    "n64@(-119.5,103.1,-53.3) build:SizeExceedsFaceWidth",
    "n64@(-119.5,119.9,-53.3) build:SizeExceedsFaceWidth",
    "n64@(-111.2,99.9,-53.3) build:SizeExceedsFaceWidth",
    "n65@(-111.2,123.1,-53.3) build:SurfaceCornerNotSupported",
    "n104@(-111.5,111.5,-51.7) build:SurfaceCornerNotSupported",
];
