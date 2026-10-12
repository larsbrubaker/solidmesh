//! Port of `R/Tests/GeometryProcessingTests/BevelCollinearRunFoldTests.cs`: a vertex sitting in
//! the middle of a straight crease (the collinear vertex a boolean leaves) must not change what
//! the sweep carries there, nor the size a refusal quotes, and two caps of one straight run may
//! not each spend the whole of it.

use geometry_processing::bevel::{bevel_chain_linker, bevel_chain_sweep, bevel_tapered_sweep};
use geometry_processing::bevel::{
    BevelAdvisory, BevelAdvisoryArg, BevelAdvisoryCode, BevelChainEnds, BevelEndCap, BevelRunTaper,
    BevelSweptSection,
};
use geometry_processing::topology::{ChainEdge, FeatureEdgeSet};
use polygon_mesh::Mesh;
use vector_math::{math_helper, Vector2, Vector3};

const CUBE_SIZE: f64 = 20.0;

const SEGMENTS: i32 = 8;

/// How far off square the cap plane leans. Steep enough that the ring's slide onto it outruns
/// HALF the edge at [`CARRIED_SIZE`] - which is what makes the split matter - and not the whole.
const LEAN_DEGREES: f64 = 75.0;

/// A size the straight run carries and the segment the split leaves does not.
const CARRIED_SIZE: f64 = 3.0;

/// A size no run of this length carries however it is cut up, so both ends refuse.
const REFUSED_SIZE: f64 = 12.0;

/// How far back along the run before the selected edge a finite runout tapers.
const START_RUNOUT: f64 = 8.0;

/// A lean whose slide stops well short of the joint, so no ring is passed at all.
const GENTLE_LEAN_DEGREES: f64 = 30.0;

fn sweep(run: &[ChainEdge], ends: &BevelChainEnds, size: f64) -> Result<Mesh, BevelAdvisory> {
    bevel_chain_sweep::try_create(
        run,
        false,
        Some(ends),
        size,
        SEGMENTS,
        true,
        BevelSweptSection::Tool,
        0.0,
        0.0,
        None,
        0.0,
        None,
    )
}

#[test]
fn a_straight_run_carries_the_same_cap_whichever_way_the_mesh_cuts_it() {
    let whole = run(false);
    let split = run(true);

    assert_eq!(
        whole.len(),
        1,
        "the plain box's top edge is one feature edge, which is the shape both runs have"
    );
    assert_eq!(
        split.len(),
        2,
        "and the split box's is two collinear ones - the fixture is worth nothing otherwise"
    );

    if let Err(whole_reason) = sweep(&whole, &ends(&whole, LEAN_DEGREES), CARRIED_SIZE) {
        panic!("the whole crease is long enough for this cap - instead: {whole_reason:?}");
    }

    if let Err(split_reason) = sweep(&split, &ends(&split, LEAN_DEGREES), CARRIED_SIZE) {
        panic!(
            "a vertex dropped in the middle of a straight crease is a tessellation accident and \
             changes no swept geometry at all, so it cannot cost the run a cap the same crease \
             carried without it - instead: {split_reason:?}"
        );
    }
}

#[test]
fn a_refusal_quotes_the_straight_run_rather_than_the_segment_it_was_cut_into() {
    let whole = run(false);
    let split = run(true);

    let whole_reason = sweep(&whole, &ends(&whole, LEAN_DEGREES), REFUSED_SIZE).expect_err(
        "this leg only means something while the size really is past what the crease carries",
    );
    let split_reason = sweep(&split, &ends(&split, LEAN_DEGREES), REFUSED_SIZE)
        .expect_err("and past it on the split crease as well, which is the same crease");

    assert_eq!(
        split_reason.code(),
        whole_reason.code(),
        "both are the same cap on the same run - instead: {whole_reason:?} against {split_reason:?}"
    );

    let difference = (largest_that_fits(&split_reason) - largest_that_fits(&whole_reason)).abs();
    assert!(
        difference <= 1e-9,
        "the largest size that fits at a corner is a property of the part, so a vertex the mesh \
         happens to carry in the middle of the run may not shrink the number the user is told to \
         try - instead: {split_reason:?} against {whole_reason:?}"
    );
}

/// A finite runout has to find a ring at both ends of every segment it walks, whatever the plain
/// sweep did with the rings on its own way to a mesh.
///
/// The tapered path indexes the ring array BY SEGMENT, so dropping the rings a cap has already
/// slid past renumbers that array and the last segment of a collinear-split run asks for a ring
/// that is no longer there. The gentle leg proves the fixture would otherwise sweep, so the steep
/// leg's refusal is about the cap and not about the runout.
#[test]
fn a_tapered_runout_still_finds_a_ring_at_every_segment_when_a_cap_has_passed_one() {
    let split = run(true);

    assert_eq!(
        split.len(),
        2,
        "the fixture is a straight crease the mesh carries as two edges - nothing to test otherwise"
    );

    // The SECOND of the two collinear edges is the selected one, so the runout tapers back over
    // the first; the far end keeps its cap, which is the one that can slide past the joint.
    let taper = BevelRunTaper {
        edges: split.clone(),
        selected_index: 1,
        selected_count: 1,
        start_length: START_RUNOUT,
        end_length: 0.0,
    };

    let gentle = bevel_tapered_sweep::try_create(
        &taper,
        &ends(&split, GENTLE_LEAN_DEGREES),
        CARRIED_SIZE,
        SEGMENTS,
        true,
        BevelSweptSection::Tool,
        0.0,
    );
    if let Err(gentle_reason) = gentle {
        panic!(
            "a cap that stops short of the joint leaves the runout the run it has always had - \
             instead: {gentle_reason:?}"
        );
    }

    bevel_tapered_sweep::try_create(
        &taper,
        &ends(&split, LEAN_DEGREES),
        CARRIED_SIZE,
        SEGMENTS,
        true,
        BevelSweptSection::Tool,
        0.0,
    )
    .expect_err(
        "a cap the runout cannot carry has to come back as a reason, and reaching for a ring \
         the plain sweep had taken out threw instead",
    );
}

/// Two capped ends of ONE straight run may not each spend the whole of it: each cap alone pulls
/// back 11.2 mm of a 20 mm crease, and the pair together drags the end rings through each other.
#[test]
fn two_caps_may_not_both_spend_the_same_straight_run() {
    let split = run(true);
    let ends = steep_at_both_ends(&split);

    let reason = sweep(&split, &ends, CARRIED_SIZE).expect_err(
        "each cap alone fits inside the 20 mm crease and the two together do not, so the run \
         cannot carry both and has to say so rather than sweep a tube wound backwards",
    );

    assert_eq!(
        reason.code(),
        BevelAdvisoryCode::CapPullsToolBackPastSegment,
        "a cap outrunning the run it has is the fold this already reports, and it is the only \
         reading that can name the corner and the size that would fit there - instead: {reason:?}"
    );
}

/// Both ends capped on planes leaning [`LEAN_DEGREES`] off square, tilted the same way ACROSS the
/// run so that the two of them pull on the same profile points.
fn steep_at_both_ends(run: &[ChainEdge]) -> BevelChainEnds {
    let last = run[run.len() - 1];
    let mut sideways = last.direction().cross(Vector3::new(0.0, 0.0, 1.0));

    if sideways.length() < 1e-9 {
        sideways = last.direction().cross(Vector3::new(0.0, 1.0, 0.0));
    }

    sideways = sideways.get_normal();

    BevelChainEnds::new(
        leaning(-run[0].direction(), sideways, run[0].from()),
        leaning(last.direction(), sideways, last.to()),
    )
}

/// A cap through `at` whose plane leans [`LEAN_DEGREES`] off square to `outward`, toward
/// `sideways`.
fn leaning(outward: Vector3, sideways: Vector3, at: Vector3) -> BevelEndCap {
    let normal = ((outward * math_helper::degrees_to_radians(LEAN_DEGREES).cos())
        + (sideways * math_helper::degrees_to_radians(LEAN_DEGREES).sin()))
    .get_normal();

    BevelEndCap::new(normal, normal.dot(at))
}

/// The largest-that-fits a fold refusal quotes, which both codes carry as their last argument.
fn largest_that_fits(advisory: &BevelAdvisory) -> f64 {
    match advisory.args().last() {
        Some(BevelAdvisoryArg::Double(value)) => *value,
        other => panic!("a fold refusal ends on the largest size that fits, not {other:?}"),
    }
}

/// The run's far end capped on a plane leaning `lean_degrees` off square to it, and its near end
/// capped SQUARE, so the two runs are compared on the same terms (an overshoot is a negative drag
/// that would lend the whole crease budget the split one does not have).
fn ends(run: &[ChainEdge], lean_degrees: f64) -> BevelChainEnds {
    let last = run[run.len() - 1];
    let along_the_run = last.direction();
    let mut sideways = along_the_run.cross(Vector3::new(0.0, 0.0, 1.0));

    if sideways.length() < 1e-9 {
        sideways = along_the_run.cross(Vector3::new(0.0, 1.0, 0.0));
    }

    let normal = ((along_the_run * math_helper::degrees_to_radians(lean_degrees).cos())
        + (sideways.get_normal() * math_helper::degrees_to_radians(lean_degrees).sin()))
    .get_normal();

    let square = -run[0].direction();

    BevelChainEnds::new(
        BevelEndCap::new(square, square.dot(run[0].from())),
        BevelEndCap::new(normal, normal.dot(last.to())),
    )
}

/// The box's top edge along x as a run, either whole or cut in two at its midpoint.
fn run(split_the_edge: bool) -> Vec<ChainEdge> {
    let mesh = create_box(split_the_edge);
    let features = FeatureEdgeSet::create(&mesh);
    let half = CUBE_SIZE / 2.0;

    let along_the_top_edge: Vec<_> = features
        .edges()
        .iter()
        .filter(|edge| {
            (edge.start.y - half).abs() < 1e-5
                && (edge.end.y - half).abs() < 1e-5
                && (edge.start.z - CUBE_SIZE).abs() < 1e-5
                && (edge.end.z - CUBE_SIZE).abs() < 1e-5
        })
        .copied()
        .collect();

    bevel_chain_linker::link(&along_the_top_edge)
        .remove(0)
        .edges
}

/// A 20 mm box, optionally with one extra vertex halfway along the top edge at y = 10 - the
/// collinear vertex a boolean leaves. It is carried by both faces on that edge, so the mesh stays
/// closed and the crease simply reads as two edges instead of one.
fn create_box(split_the_edge: bool) -> Mesh {
    let mut mesh = Mesh::new();

    let corners = [
        Vector2::new(-10.0, -10.0),
        Vector2::new(10.0, -10.0),
        Vector2::new(10.0, 10.0),
        Vector2::new(-10.0, 10.0),
    ];

    // The split runs from (10, 10) to (-10, 10) - the top face's own winding order - so the
    // midpoint goes in between those two, and the wall below it carries the same point the other
    // way round.
    let mut top = Vec::new();

    for corner in &corners {
        top.push(Vector3::new(corner.x, corner.y, CUBE_SIZE));

        if split_the_edge && corner.x == 10.0 && corner.y == 10.0 {
            top.push(Vector3::new(0.0, 10.0, CUBE_SIZE));
        }
    }

    let bottom: Vec<Vector3> = corners
        .iter()
        .rev()
        .map(|corner| Vector3::new(corner.x, corner.y, 0.0))
        .collect();
    mesh.create_face(&bottom);
    mesh.create_face(&top);

    for i in 0..corners.len() {
        let from = corners[i];
        let to = corners[(i + 1) % corners.len()];

        let mut wall = vec![
            Vector3::new(from.x, from.y, 0.0),
            Vector3::new(to.x, to.y, 0.0),
            Vector3::new(to.x, to.y, CUBE_SIZE),
        ];

        if split_the_edge && from.x == 10.0 && from.y == 10.0 && to.y == 10.0 {
            wall.push(Vector3::new(0.0, 10.0, CUBE_SIZE));
        }

        wall.push(Vector3::new(from.x, from.y, CUBE_SIZE));

        mesh.create_face(&wall);
    }

    // create_face adds fresh vertices every call, so what is built above is triangle soup
    mesh.clean_and_merge();

    mesh
}
