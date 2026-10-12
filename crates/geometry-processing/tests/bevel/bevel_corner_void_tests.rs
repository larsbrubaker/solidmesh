//! Ports `GeometryProcessingTests/BevelCornerVoidTests.cs`: the pieces a bead's 5.3 corner is made
//! of, each read on its own - the wedge's squared-off end (`BevelChainEnds::squared_at`), the ring
//! of faces the corner's void is drawn from (`bevel_corner_face_ring::try_order_at_vertex`), and
//! the pair of wedges a chain carries away from it (`bevel_group_tools::sweep_void_wedges`).
//!
//! C#'s `IsSameReferenceAs` checks read as equality here: Rust's `squared_at` hands back a clone
//! where it squares nothing, and `sweep_void_wedges` takes `None` for "nothing was squared" (what
//! C# reads off that identity) and returns owned meshes, told apart by their size.

use geometry_processing::bevel::bevel_corner_face_ring::{try_order, try_order_at_vertex};
use geometry_processing::bevel::bevel_corner_patch_mesher::try_build_void_cone;
use geometry_processing::bevel::bevel_group_tools::sweep_void_wedges;
use geometry_processing::bevel::{BevelChainEnds, BevelEndCap};
use geometry_processing::topology::{ChainEdge, EdgeConvexity, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::Mesh;
use vector_math::Vector3;

use crate::bevel_volume_probes::{probe_cube, volume_inside};
use crate::topology_test_meshes::{create_corner_pocket_block, create_square_boss_on_plate};

/// The plane a squared end lands on: through that end's own vertex, square to the run, and facing
/// the way the sweep LEAVES the end - which at the START is the run direction reversed, the one
/// sign this is easy to get backwards and the one that decides whether the wedge keeps the run or
/// the overshoot.
#[test]
fn squaring_an_end_puts_the_plane_through_its_vertex_facing_the_way_the_sweep_leaves() {
    let block = create_corner_pocket_block(24.0, 20.0);
    let features = FeatureEdgeSet::create(&block);
    let run = one_concave_run(&features, Vector3::new(4.0, 4.0, 4.0));

    let first = run[0];
    let last = run[run.len() - 1];

    let overshoot = BevelChainEnds::overshoot();

    assert!(!overshoot.start.is_capped());
    assert!(!overshoot.end.is_capped());

    assert_eq!(
        overshoot.squared_at(&run, false, false),
        overshoot,
        "squaring nothing is what a run with no 5.3 corner asks for, and the caller reads that \
         identity to tell it apart from a wedge it did square"
    );

    let squared = overshoot.squared_at(&run, true, true);

    assert!(squared.start.is_capped());
    assert!(squared.end.is_capped());

    assert!(
        (squared.start.plane_normal - -first.direction()).length() < 1e-9,
        "the sweep leaves the START against the run, so the plane faces {:?} - instead {:?}",
        -first.direction(),
        squared.start.plane_normal
    );
    assert!(
        (squared.end.plane_normal - last.direction()).length() < 1e-9,
        "and leaves the far end along it - instead {:?}",
        squared.end.plane_normal
    );

    // On the plane means the vertex satisfies it exactly, which is what makes the ring slide zero
    // and the fold test see no drag at all.
    assert!(
        (squared.start.plane_normal.dot(first.from()) - squared.start.plane_distance).abs() <= 1e-9,
        "the plane passes through the end's own vertex rather than a radius either side of it"
    );
    assert!((squared.end.plane_normal.dot(last.to()) - squared.end.plane_distance).abs() <= 1e-9);

    // And one end at a time, since a run with a 5.3 corner at one end and a capped face at the
    // other is the commonest shape this is asked for.
    let start_only = overshoot.squared_at(&run, true, false);

    assert!(start_only.start.is_capped());
    assert!(
        !start_only.end.is_capped(),
        "the far end was not squared and keeps whatever it had"
    );
}

/// The ring the corner's void is drawn from closes over every crease at the vertex, and refuses a
/// vertex that is not a corner at all. This is the reading that makes a TWO-crease pick workable:
/// `try_order` asked about the two selected creases alone says no - they separate three planes
/// between them and the ring is open - while the same walk over all three creases closes on the
/// octant of void the pocket really opens.
#[test]
fn the_corners_ring_closes_over_every_crease_at_the_vertex_and_refuses_a_plain_edge() {
    let block = create_corner_pocket_block(24.0, 20.0);
    let features = FeatureEdgeSet::create(&block);

    let corner = Vector3::new(4.0, 4.0, 4.0);
    let at_corner = features
        .edges_at_vertex(vertex_at(&features, corner))
        .to_vec();

    assert_eq!(
        at_corner.len(),
        3,
        "the bite leaves three concave creases meeting at the pocket's inner corner"
    );

    let normals = try_order_at_vertex(Some(&features), vertex_at(&features, corner))
        .expect("three creases at one vertex separate three planes and the walk closes on them");

    assert_eq!(normals.len(), 3);

    // The pocket is the octant beyond the corner, so the three outward normals are the three
    // axes - in some order and some winding, neither of which this cares about.
    for axis in [
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
    ] {
        assert_eq!(
            normals
                .iter()
                .filter(|&&normal| (normal - axis).length() < 1e-6)
                .count(),
            1,
            "the pocket's three walls face out along the axes, and {axis:?} is missing from \
             {normals:?}"
        );
    }

    // The same walk asked about only TWO of those creases - what a pick of two hands the patch -
    // is open, which is exactly why the void is read off the part rather than off the selection.
    let two: Vec<FeatureEdge> = at_corner
        .iter()
        .take(2)
        .map(|&index| features.edges()[index as usize])
        .collect();

    assert!(
        try_order(&two).is_none(),
        "two creases separate three planes, so one of them is bounded on one side only and the \
         ring cannot close"
    );

    // And a vertex in the middle of a crease is not a corner. Every vertex along the pocket's own
    // edges has exactly two feature edges on it, and there is no cone to draw there.
    let along_a_crease = features
        .edges()
        .iter()
        .flat_map(|edge| [edge.start_vertex, edge.end_vertex])
        .find(|&vertex| features.edges_at_vertex(vertex).len() < 3)
        .expect("a vertex along a crease");

    assert!(
        try_order_at_vertex(Some(&features), along_a_crease).is_none(),
        "fewer than three creases is a crease running through rather than a corner opening"
    );
}

/// The two wedges a chain that squared its ends carries away, and the one a chain that squared
/// nothing does: which is used, which is kept in reserve, and whether the corners it read are
/// still good.
///
/// Driven rather than provoked: the branches that matter are the ones where a sweep comes back
/// `None`, and no fixture in the corpus produces one at a 5.3 corner (see the C# remarks). Three
/// cases, and the caller acts differently on each: squaring worked, so the squared wedge is used
/// and the overshooting one is kept in reserve for the clip to fall back to; squaring would not
/// sweep, so the overshooting one is used, there is nothing further to fall back to, and the
/// corners are dropped with it; and nothing was squared at all, where one sweep is the whole of
/// the work.
#[test]
fn a_chain_that_squared_its_wedge_carries_the_overshooting_one_in_reserve() {
    let ends = BevelChainEnds::overshoot();
    let squared = BevelChainEnds::new(
        BevelEndCap::new(Vector3::new(1.0, 0.0, 0.0), 0.0),
        BevelEndCap::default(),
    );

    let squared_wedge = platonic_solids::create_cube(1.0, 1.0, 1.0);
    let overshot_wedge = platonic_solids::create_cube(2.0, 2.0, 2.0);

    let mut asked: Vec<BevelChainEnds> = Vec::new();
    let mut told_it_was_squaring: Vec<bool> = Vec::new();

    let sweep = |ask: &BevelChainEnds,
                 squaring_a_corner: bool,
                 when_squared: Option<&Mesh>,
                 asked: &mut Vec<BevelChainEnds>,
                 told: &mut Vec<bool>|
     -> Option<Mesh> {
        asked.push(ask.clone());
        told.push(squaring_a_corner);

        if *ask == squared {
            when_squared.cloned()
        } else {
            Some(overshot_wedge.clone())
        }
    };

    // Squaring worked: the squared wedge is the clip's, the overshooting one is its reserve.
    let (wedge, fallback, was_squared) = sweep_void_wedges(&ends, Some(&squared), |ask, s| {
        sweep(
            ask,
            s,
            Some(&squared_wedge),
            &mut asked,
            &mut told_it_was_squaring,
        )
    });

    assert!(is(&wedge, &squared_wedge));
    assert!(
        is(&fallback, &overshot_wedge),
        "the keep-set the squared wedge belongs to is one the kernel will sometimes not build, and \
         the answer there is a clip rather than no clip"
    );
    assert!(was_squared);
    assert_eq!(
        asked.len(),
        2,
        "both wedges are swept while sweeping is still cheap - a later stage cannot go back for one"
    );
    assert_eq!(
        told_it_was_squaring,
        vec![true, false],
        "the sweep is told which of the two it is being asked for, because only the squared wedge \
         is clipped with beside a corner void and it is drawn differently for it"
    );

    // Squaring would not sweep: the overshooting wedge is the clip's, and the corners go with it.
    asked.clear();
    told_it_was_squaring.clear();

    let (wedge, fallback, was_squared) = sweep_void_wedges(&ends, Some(&squared), |ask, s| {
        sweep(ask, s, None, &mut asked, &mut told_it_was_squaring)
    });

    assert!(
        is(&wedge, &overshot_wedge),
        "losing the wedge would cost the bead its trim along its whole length to buy one vertex of \
         corner"
    );
    assert!(
        fallback.is_none(),
        "the wedge in hand IS the fallback, and there is nothing further back to go"
    );
    assert!(
        !was_squared,
        "which is the caller's cue to drop the cones - a corner void beside an overshooting wedge \
         is the leak it was built to stop"
    );

    // Nothing was squared at all: one sweep, and nothing held in reserve behind it.
    asked.clear();
    told_it_was_squaring.clear();

    let (wedge, fallback, was_squared) = sweep_void_wedges(&ends, None, |ask, s| {
        sweep(ask, s, None, &mut asked, &mut told_it_was_squaring)
    });

    assert!(is(&wedge, &overshot_wedge));
    assert!(fallback.is_none());
    assert!(!was_squared);
    assert_eq!(
        asked.len(),
        1,
        "the injected sweep was asked once and by identity - a second attempt on the same ends \
         would be the same answer"
    );
}

/// A MIXED vertex - two concave creases and the unselected CONVEX corner between them, which is a
/// square boss's foot and the commonest 5.3 shape there is - reads a cone like any other, and that
/// cone is inside the void.
///
/// The walk takes concave and convex creases alike: `K_v` is the intersection of the OUTWARD half
/// spaces of the faces at the vertex, so a mixed corner can only ever UNDERSTATE the void - the
/// failure direction is a notch, never a leak. The boss is 8 square standing on a 20 square plate
/// 4 thick, so its foot corners are at `(6, 6, 4)`; the three probes are the three ways out of the
/// void: into the quadrant, into the boss, and into the plate.
#[test]
fn a_mixed_vertex_reads_a_cone_and_that_cone_lies_in_the_void() {
    let boss = create_square_boss_on_plate();
    let features = FeatureEdgeSet::create(&boss);

    let corner = Vector3::new(6.0, 6.0, 4.0);
    let vertex = vertex_at(&features, corner);
    let at_the_corner = features.edges_at_vertex(vertex);

    assert_eq!(
        at_the_corner.len(),
        3,
        "two concave foot edges and the boss's own vertical corner meet here"
    );
    assert_eq!(
        at_the_corner
            .iter()
            .filter(|&&index| features.edges()[index as usize].convexity == EdgeConvexity::Convex)
            .count(),
        1,
        "and the vertical one folds the other way, which is what makes this a mixed vertex"
    );

    let normals = try_order_at_vertex(Some(&features), vertex)
        .expect("the ring is walked over the faces, not over the convexities");

    let cone = try_build_void_cone(corner, &normals, 3.0, 8, false).expect(
        "the two walls and the plate bound a quadrant, which is a convex cone like any other",
    );

    // Into the quadrant: outside both walls, above the plate - void, and the cone has to hold it.
    let in_the_void = probe_cube(Vector3::new(5.0, 5.0, 5.0), 1.0);
    let held = volume_inside(&cone, &in_the_void);

    assert!(
        (held - 1.0).abs() <= 0.01,
        "the corner opens the quadrant outside both walls and above the plate, and that is where a \
         bead's overshoot goes - held {held}"
    );

    // Into the boss, and into the plate: material, and the cone must hold neither.
    assert_eq!(
        volume_inside(&cone, &probe_cube(Vector3::new(7.0, 7.0, 5.0), 1.0)),
        0.0,
        "past the walls is the boss itself, which is on the inward side of both of them"
    );

    assert_eq!(
        volume_inside(&cone, &probe_cube(Vector3::new(5.0, 5.0, 3.0), 1.0)),
        0.0,
        "and below the corner is the plate, which is on the inward side of its top face"
    );
}

/// The leg a corner FILL asks for: the same cone with its apex set back into the corner, so that
/// its lateral walls are buried in material instead of lying in the part's own face planes - the
/// difference between a clip that runs and one an exact kernel calls self-intersecting ("5mm Bevel
/// Center Bottom.mcx"). What is pinned is the geometry: the apex moves strictly to the MATERIAL
/// side of all three faces, by no more than the set-back, and the void the cone is FOR is held.
#[test]
fn a_fills_cone_sets_its_apex_back_so_its_walls_cross_the_faces_rather_than_lying_in_them() {
    let boss = create_square_boss_on_plate();
    let features = FeatureEdgeSet::create(&boss);

    let corner = Vector3::new(6.0, 6.0, 4.0);
    let length = 3.0;

    let normals = try_order_at_vertex(Some(&features), vertex_at(&features, corner))
        .expect("the ring at this corner is the one the cone above is built from");

    let as_it_lies = try_build_void_cone(corner, &normals, length, 8, false)
        .expect("the corner builds a cone either way");
    let set_back = try_build_void_cone(corner, &normals, length, 8, true)
        .expect("and asking for the set-back does not refuse it");

    // A thousandth of the cone's own length, which is what the mesher sets the apex back by.
    let most = length * 1e-3;

    for normal in &normals {
        let lying = normal.dot(apex(&as_it_lies, corner) - corner);
        let moved = normal.dot(apex(&set_back, corner) - corner);

        assert!(
            lying.abs() < 1e-9,
            "left alone, the apex is the corner vertex itself, so every wall through it lies \
             exactly in one of the part's own faces - this one stands {lying:.7} off it"
        );

        assert!(
            moved < -most / 100.0,
            "set back, the apex has to be on the MATERIAL side of every face at the corner, which \
             is what makes the wall through it CROSS that face rather than share it - this one \
             stands {moved:.6} off, where negative is into the material"
        );

        assert!(
            moved > -most,
            "and no further in than the whole set-back of {most:.4} mm, or the cone is keeping \
             material rather than clearing a coincidence"
        );
    }

    // And the void it exists to hold is still held - the set-back only ever GROWS the keep set.
    let in_the_void = probe_cube(Vector3::new(5.0, 5.0, 5.0), 1.0);
    let held = volume_inside(&set_back, &in_the_void);

    assert!(
        (held - 1.0).abs() <= 0.01,
        "the quadrant outside both walls and above the plate is what the cone is for, and moving \
         the apex into the corner cannot take any of it away - held {held}"
    );
}

/// Whether a returned wedge is the given one (C#'s `IsSameReferenceAs`): the test's two wedges are
/// cubes of different sizes, so their vertices tell them apart.
fn is(wedge: &Option<Mesh>, expected: &Mesh) -> bool {
    wedge.as_ref().is_some_and(|mesh| {
        mesh.vertices == expected.vertices && mesh.faces.len() == expected.faces.len()
    })
}

/// The cone's apex: its one vertex at the corner, the rest being a whole reach away.
fn apex(cone: &Mesh, corner: Vector3) -> Vector3 {
    let mut vertices: Vec<Vector3> = cone.vertices.iter().map(|v| v.as_vector3()).collect();

    // C# OrderBy is stable; a stable sort keeps the first of any tie, as First() does.
    vertices.sort_by(|a, b| (*a - corner).length().total_cmp(&(*b - corner).length()));

    vertices[0]
}

/// The one concave run leaving a vertex, oriented so its FIRST edge starts there - which is how a
/// chain that ends at a 5.3 corner reaches this file's questions.
fn one_concave_run(features: &FeatureEdgeSet, corner: Vector3) -> Vec<ChainEdge> {
    for (i, edge) in features.edges().iter().enumerate() {
        if edge.convexity != EdgeConvexity::Concave {
            continue;
        }

        if (edge.start - corner).length() < 1e-4 {
            return vec![ChainEdge::new(i as i32, *edge, false)];
        }

        if (edge.end - corner).length() < 1e-4 {
            return vec![ChainEdge::new(i as i32, *edge, true)];
        }
    }

    panic!("no concave feature edge starts at {corner:?}");
}

/// The mesh vertex a position belongs to, read through the feature set that is about to be asked
/// about it.
fn vertex_at(features: &FeatureEdgeSet, position: Vector3) -> i32 {
    for edge in features.edges() {
        if (edge.start - position).length() < 1e-4 {
            return edge.start_vertex;
        }

        if (edge.end - position).length() < 1e-4 {
            return edge.end_vertex;
        }
    }

    panic!("no feature edge reaches {position:?}");
}
