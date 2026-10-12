//! Port of `SurfaceRakedBoreSplitChordTests.cs`: the picture hanger's raked mouth at its own numbers
//! (forty facets, 1.5 radius, 45 degrees through a 4 mm wall) with the eight chord splits the part's
//! own boolean left (`bore_test_meshes::create_plate_with_a_raked_bore_whose_mouth_chords_are_split`),
//! built at the origin and at the hanger's own stand-off, where the float lattice is thirty-two
//! times coarser and the split pieces' tangent lines become two parallel lines a few
//! ten-millionths apart. The stand-off's remaining open edges are pinned rather than asserted away.
//!
//! Legacy-only: every test pins the sections router off, as C# does.
//! `TheRimOnABorrowedRailIsEmittedAsAContinuation` is `[Skip]` in C# ("RED: the emitter is built,
//! but the stand-off's corner is not among the ten cuts that borrowed a rail") and is not ported,
//! nor its only helper `TheSplitMouthWalkedBack`.

use geometry_processing::bevel::surface::surface_mesh_closure;
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelEdgeGroup, BevelSelectionKind, FaceGroup, FeatureEdgeSet,
};
use indexmap::IndexMap;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{math_helper, Vector3};

use crate::bore_test_meshes::{
    create_plate_with_a_raked_bore, create_plate_with_a_raked_bore_whose_mouth_chords_are_split,
};
use crate::mesh_measurements::shell_count;
use crate::surface_test_build::{
    expect_built, pinned_to_the_legacy_construction, try_bevel, TryBevelOptions,
};

/// The part's own bore: forty facets, 1.5 radius, 45 degrees, through a 4 mm wall.
const SIDES: i32 = 40;

const RADIUS: f64 = 1.5;

const THICKNESS: f64 = 4.0;

const RAKE_DEGREES: f64 = 45.0;

const SIZE: f64 = 0.4;

const SEGMENTS: i32 = 8;

const BEND_THRESHOLD_DEGREES: f64 = 10.0;

/// How much plate stands outside the mouth, chosen so the box round this fixture has the HANGER's
/// own diagonal - because the weld ruler every reading of the build is made against is a millionth
/// of that box.
const THE_HANGERS_OWN_LAND: f64 = 23.01;

/// The eight splits the part's own boolean left, read off its mouth loop and turned into this
/// fixture's frame by the ellipse's own parameter.
const THE_PARTS_OWN_SPLITS: [(i32, f64); 8] = [
    (9, 0.365755),
    (10, 0.354150),
    (12, 0.331480),
    (13, 0.320984),
    (24, 0.299840),
    (25, 0.307971),
    (29, 0.350692),
    (30, 0.362577),
];

fn the_hangers_own_stand_off() -> Vector3 {
    Vector3::new(-28.5, 23.66, 12.0)
}

/// The split plate at a rake, standing where it is put.
fn the_split_part(rake_degrees: f64, standing_at: Vector3) -> Mesh {
    create_plate_with_a_raked_bore_whose_mouth_chords_are_split(
        SIDES,
        RADIUS,
        THICKNESS,
        rake_degrees,
        &THE_PARTS_OWN_SPLITS,
        THE_HANGERS_OWN_LAND,
        standing_at,
    )
}

/// The tangent chain seeded on the upper mouth of a part standing at `standing_at`.
fn the_mouth_group(part: &Mesh, standing_at: Vector3, rake_degrees: f64) -> BevelEdgeGroup {
    let features = FeatureEdgeSet::create_with_graph(
        part,
        &part.get_mesh_edge_graph(),
        BEND_THRESHOLD_DEGREES,
    );

    bevel_edge_group_resolver::create_from_edge(
        &features,
        the_upper_mouth(&features, standing_at, rake_degrees),
        BevelSelectionKind::TangentChain,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("the mouth's edge is in the set")
}

fn the_chain_length(part: &Mesh, group: &BevelEdgeGroup) -> usize {
    bevel_edge_group_resolver::resolve_all(std::slice::from_ref(group), part).0[0]
        .edges
        .len()
}

fn built(part: &Mesh, group: BevelEdgeGroup, because: &str) -> Mesh {
    expect_built(
        try_bevel(
            part,
            &[group],
            SIZE,
            SEGMENTS,
            &CancelToken::new(),
            TryBevelOptions::default(),
        ),
        because,
    )
}

/// The claim, as the part asks it: a raked mouth one of whose chords a boolean has split still
/// rounds all the way round, closed and manifold.
#[test]
fn split_chords_still_leave_the_mouth_rounded_all_the_way_round() {
    let _pinned = pinned_to_the_legacy_construction();

    let part = the_split_part(RAKE_DEGREES, Vector3::ZERO);
    let group = the_mouth_group(&part, Vector3::ZERO, RAKE_DEGREES);

    assert_eq!(
        the_chain_length(&part, &group),
        SIDES as usize + THE_PARTS_OWN_SPLITS.len(),
        "each split chord is two edges of the chain, which is the part's own forty-eight"
    );

    let mesh = built(
        &part,
        group,
        "a chord a boolean split is still a chord of the same facet",
    );

    let (open, open_at) = surface_mesh_closure::open_edge_count(&mesh);
    assert_eq!(
        open, 0,
        "a loop all the way round leaves a closed solid, and this one is open at {open_at:?}"
    );
    assert!(mesh.is_manifold());
    assert_eq!(shell_count(&mesh), 1);
}

/// THE SAME SPLITS, STANDING WHERE THE HANGER'S MOUTH STANDS. The refusal is gone and the mouth is
/// rounded all the way round; what it still leaves open is pinned - two FOLDS, edges carrying four
/// faces apiece, the sliver's twin fan plus the joint step's copy of a face already standing.
#[test]
fn the_same_splits_stand_where_the_hangers_mouth_stands() {
    let _pinned = pinned_to_the_legacy_construction();

    let part = the_split_part(RAKE_DEGREES, the_hangers_own_stand_off());
    let group = the_mouth_group(&part, the_hangers_own_stand_off(), RAKE_DEGREES);

    assert_eq!(
        the_chain_length(&part, &group),
        SIDES as usize + THE_PARTS_OWN_SPLITS.len(),
        "the same forty-eight edges, only further from the origin"
    );

    let mesh = built(
        &part,
        group,
        "where a part stands cannot decide whether it bevels",
    );

    let (open, open_at) = surface_mesh_closure::open_edge_count(&mesh);
    assert_eq!(
        open, 2,
        "the closure this stand-off still leaves open is the next rung - those 2 are FOLDS rather \
         than cracks, two edges carrying four faces apiece - first open at {open_at:?}"
    );
}

/// THE SMALLEST REPRODUCTION OF THE STAND-OFF FAULT: the same part an eighth of the way out, which
/// once left exactly one edge open and now closes.
#[test]
fn an_eighth_of_the_stand_off_already_leaves_one_edge_open() {
    let _pinned = pinned_to_the_legacy_construction();

    let standing_at = the_hangers_own_stand_off() * 0.12;
    let part = the_split_part(RAKE_DEGREES, standing_at);
    let group = the_mouth_group(&part, standing_at, RAKE_DEGREES);

    let mesh = built(
        &part,
        group,
        "an eighth of the way out is still the same part",
    );

    let (open, open_at) = surface_mesh_closure::open_edge_count(&mesh);
    assert_eq!(
        open, 0,
        "and a crossing that IS its own rim leaves nothing for the stitch to cover twice - before \
         the rule this read 1, open at {open_at:?}"
    );
}

#[test]
fn the_same_plate_with_the_chord_left_whole_builds() {
    let _pinned = pinned_to_the_legacy_construction();

    let part = create_plate_with_a_raked_bore(
        SIDES,
        RADIUS,
        THICKNESS,
        RAKE_DEGREES,
        THE_HANGERS_OWN_LAND,
    );
    let group = the_mouth_group(&part, Vector3::ZERO, RAKE_DEGREES);

    let mesh = built(
        &part,
        group,
        "the four millimetre wall is no different from the three",
    );

    let (open, open_at) = surface_mesh_closure::open_edge_count(&mesh);
    assert_eq!(open, 0, "and it closes, open at {open_at:?}");
}

#[test]
fn a_shallower_rake_puts_the_band_over_the_run_and_still_starves_a_piece() {
    let _pinned = pinned_to_the_legacy_construction();

    const SHALLOW_RAKE_DEGREES: f64 = 30.0;

    let part = the_split_part(SHALLOW_RAKE_DEGREES, the_hangers_own_stand_off());
    let group = the_mouth_group(&part, the_hangers_own_stand_off(), SHALLOW_RAKE_DEGREES);

    assert_eq!(
        the_chain_length(&part, &group),
        SIDES as usize + THE_PARTS_OWN_SPLITS.len(),
        "the same forty-eight edges, only less steeply raked"
    );

    let mesh = built(
        &part,
        group,
        "a corner that leaves one piece of its chord nothing is the joint's to move, whether the \
         band stands clear of the run or over it",
    );

    let (open, open_at) = surface_mesh_closure::open_edge_count(&mesh);
    assert_eq!(
        open, 11,
        "what this stand-off leaves open is the SAME rung the 45 degree fixture's are - 11 once \
         the joint's own step is laid (SurfaceMitreJointStep) - first open at {open_at:?}"
    );

    // And the same part at the ORIGIN, the control that says the open edges above are about where
    // it stands.
    let at_the_origin = the_split_part(SHALLOW_RAKE_DEGREES, Vector3::ZERO);
    let origin_group = the_mouth_group(&at_the_origin, Vector3::ZERO, SHALLOW_RAKE_DEGREES);

    let at_the_origins_mesh = built(
        &at_the_origin,
        origin_group,
        "where the part stands cannot decide whether it bevels",
    );

    let (open_there, open_at_the_origin) =
        surface_mesh_closure::open_edge_count(&at_the_origins_mesh);
    assert_eq!(
        open_there, 0,
        "and at the origin the same chain closes outright - first open at {open_at_the_origin:?}"
    );
}

#[test]
fn no_open_edge_at_the_stand_off_is_forked_by_a_twin_of_its_own_end() {
    let _pinned = pinned_to_the_legacy_construction();

    let part = the_split_part(RAKE_DEGREES, the_hangers_own_stand_off());
    let group = the_mouth_group(&part, the_hangers_own_stand_off(), RAKE_DEGREES);

    let mesh = built(&part, group, "the stand-off builds");

    let (forked, forked_at, uses) = the_forked_open_edges(&mesh, 4e-3);

    // The representative edge, read by PLACE rather than by index so the claim survives any
    // renumber: the wall's rim segment at the stand-off's fork foot, which the wall plus the
    // patch's ear have to draw twice.
    let corner = the_vertex_nearest(&mesh, Vector3::new(-24.765312, 24.336022, 13.841670));
    let rim = the_vertex_nearest(&mesh, Vector3::new(-24.768727, 24.340986, 13.841163));
    let (segment_uses, segment_balance) = uses
        .get(&(corner.min(rim), corner.max(rim)))
        .copied()
        .unwrap_or((0, 0));

    assert_eq!(
        segment_uses, 2,
        "the wall's rim segment between the named band corner {corner} and the rim {rim} is drawn \
         by the wall and by the patch's ear - instead {segment_uses} uses"
    );
    assert_eq!(
        segment_balance, 0,
        "and the two walk it opposite ways - instead balance {segment_balance}"
    );

    let (open, open_at) = surface_mesh_closure::open_edge_count(&mesh);
    assert_eq!(
        open, 2,
        "the named feet ears, the patch's winding, the joint's own step and the declined patch's \
         crossings take the stand-off's 154 open edges to 2 - first open at {open_at:?}"
    );

    assert!(
        forked < 27,
        "the ear closes the fork at every named foot - 102 falls to 27 - and the patch wound off \
         the rim side as it draws it takes 27 to 8 - {forked}, first forked at {forked_at:?}"
    );
}

fn the_vertex_nearest(mesh: &Mesh, place: Vector3) -> i32 {
    let mut nearest = -1;
    let mut away = f64::MAX;

    for (vertex, at) in mesh.vertices.iter().enumerate() {
        let distance = (at.as_vector3() - place).length();

        if distance < away {
            away = distance;
            nearest = vertex as i32;
        }
    }

    nearest
}

type EdgeUses = IndexMap<(i32, i32), (i32, i32)>;

/// Every edge the mesh does not use exactly twice opposite ways round, counted where another
/// vertex stands within `ruler` of one of its ends - a fork by a twin. Answers the count, the first
/// such edge's middle, and every edge's (uses, balance) in the order the faces first draw them.
fn the_forked_open_edges(mesh: &Mesh, ruler: f64) -> (i32, Vector3, EdgeUses) {
    let mut forked_at = Vector3::ZERO;
    let mut uses: EdgeUses = IndexMap::new();

    for face in mesh.faces.iter() {
        for (first, second) in [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)] {
            let entry = uses
                .entry((first.min(second), first.max(second)))
                .or_insert((0, 0));

            entry.0 += 1;
            entry.1 += if first < second { 1 } else { -1 };
        }
    }

    let mut forked = 0;
    let mut found = false;

    for (&(low_index, high_index), &(count, balance)) in &uses {
        if count == 2 && balance == 0 {
            continue;
        }

        let low = mesh.vertices[low_index as usize].as_vector3();
        let high = mesh.vertices[high_index as usize].as_vector3();

        let is_forked = mesh.vertices.iter().enumerate().any(|(vertex, at)| {
            let vertex = vertex as i32;
            let at = at.as_vector3();

            vertex != low_index
                && vertex != high_index
                && ((at - low).length() <= ruler || (at - high).length() <= ruler)
        });

        if !is_forked {
            continue;
        }

        forked += 1;

        if !found {
            found = true;
            forked_at = (low + high) / 2.0;
        }
    }

    (forked, forked_at, uses)
}

fn the_upper_mouth(features: &FeatureEdgeSet, standing_at: Vector3, rake_degrees: f64) -> i32 {
    (0..features.count())
        .find(|&i| {
            on_the_mouth(features.edges()[i].start - standing_at, rake_degrees)
                && on_the_mouth(features.edges()[i].end - standing_at, rake_degrees)
        })
        .map(|i| i as i32)
        .expect("the fixture has no upper mouth")
}

fn on_the_mouth(at: Vector3, rake_degrees: f64) -> bool {
    let rake = math_helper::degrees_to_radians(rake_degrees);
    let axis = Vector3::new(rake.sin(), 0.0, rake.cos());

    (at.z - (THICKNESS / 2.0)).abs() < 1e-4
        && ((at - (axis * at.dot(axis))).length() - RADIUS).abs() < 1e-4
}
