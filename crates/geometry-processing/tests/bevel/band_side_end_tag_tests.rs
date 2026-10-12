//! Ports `Tests/GeometryProcessingTests/LevelLine/BandSideEndTagTests.cs`: an OPEN chain that ends
//! on an END FACE tags the faces round each end with an end tag of its own - 3 at the chain's first
//! vertex, 4 at its last - while tags 1 and 2, and every closed chain's and every running-on or
//! dying end's tags, stay bit-identical to the tagging without end floods. C# `[MethodDataSource]`
//! rows are loops over the same-named test.

use std::cell::RefCell;
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::band_room;
use geometry_processing::bevel::level_line::band_side_groups::group_sides;
use geometry_processing::bevel::level_line::band_side_tagging::{FIRST_END_TAG, SECOND_END_TAG};
use geometry_processing::bevel::level_line::band_side_tagging_ends::{
    tag_band, tag_sides, SideReach,
};
use geometry_processing::bevel::level_line::chain_on_the_mesh;
use geometry_processing::bevel::level_line::gap_fill;
use geometry_processing::bevel::level_line::{
    BandProfile, ChainField, GapFillRefusal, LevelLineMesh,
};
use vector_math::{Vector2, Vector3};

use crate::crease_dies_out_fixtures;
use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::{box_with_square_hole, hourglass_solid, SolidWithChain};
use crate::level_line_test_geometry::cube_solid;
use crate::level_line_test_solids::{cube_with_mid_edge_vertex, prism_on_outline, square_pocket};
use crate::open_chain_fixtures;
use crate::several_chains_fixtures::{plate_with_offset_square_hole, plate_with_two_bosses};
use crate::sweep_mesh_measures;

const FOLD_DEGREES: f64 = 10.0;

const SNAP: f64 = 1e-7;

/// C# `Pick`: a pick over a fixture, named for the test's line.
struct Pick {
    name: String,
    positions: Vec<Vector3>,
    indices: Vec<(i32, i32, i32)>,
    chains: Vec<Vec<i32>>,
    closed: bool,
}

fn pick(
    name: &str,
    positions: Vec<Vector3>,
    indices: Vec<(i32, i32, i32)>,
    chains: Vec<Vec<i32>>,
    closed: bool,
) -> Pick {
    Pick {
        name: name.to_string(),
        positions,
        indices,
        chains,
        closed,
    }
}

/// C# `Picks`.
fn picks() -> Vec<Pick> {
    let mut picks = Vec::new();

    // Closed chains, one and several.
    let (p, i) = cube_solid(1.0);
    picks.push(pick("cube top rim", p, i, vec![vec![4, 5, 6, 7]], true));
    let (p, i) = open_chain_fixtures::cylinder();
    let sides = open_chain_fixtures::SIDES;
    picks.push(pick(
        "cylinder top rim",
        p,
        i,
        vec![(sides..sides + sides).collect()],
        true,
    ));
    let (p, i, rim) = box_with_square_hole(2.0, 0.5, 1.0);
    picks.push(pick("box hole mouth", p, i, vec![rim], true));
    let (p, i, outer, hole) = plate_with_offset_square_hole(1.0);
    picks.push(pick("plate rim and hole", p, i, vec![outer, hole], true));
    let (p, i, a, b) = plate_with_two_bosses();
    picks.push(pick("two bosses", p, i, vec![a, b], true));
    let (p, i, rim) = hourglass_solid(10.0, 2.0, 4.0, 4.0);
    picks.push(pick("hourglass rim", p, i, vec![rim], true));
    let (p, i, rim) = square_pocket(2.0, 1.0, 1.0, 0.0);
    picks.push(pick("pocket floor rim", p, i, vec![rim], true));

    // Open chains whose ends run on along a crease or die into a smooth surface.
    for edges in [1, 3] {
        let (p, i) = open_chain_fixtures::cylinder();
        picks.push(pick(
            &format!("cylinder rim run of {edges}"),
            p,
            i,
            vec![open_chain_fixtures::top_rim_run(edges)],
            false,
        ));
    }

    let (p, i) = crease_dies_out_fixtures::swollen_ridge_block();
    let chain = crease_dies_out_fixtures::chain(&p, &i, crease_dies_out_fixtures::RING, 4);
    picks.push(pick("swollen ridge", p, i, vec![chain], false));
    picks
}

/// C# `EndFacePicks`.
fn end_face_picks() -> Vec<Pick> {
    let mut picks = Vec::new();
    let (p, i) = cube_solid(1.0);
    picks.push(pick("cube top edge", p, i, vec![vec![4, 5]], false));
    let (p, i, top) = hex_prism();
    picks.push(pick(
        "hexagonal prism top edge",
        p,
        i,
        vec![vec![top[0], top[1]]],
        false,
    ));
    let (p, i, mid) = cube_with_mid_edge_vertex(1.0);
    picks.push(pick("half a cube edge", p, i, vec![vec![4, mid]], false));
    let (p, i, chain, _) = slab_with_a_rounded_end();
    picks.push(pick(
        "slab edge whose end face rounds into the bottom",
        p,
        i,
        vec![chain],
        false,
    ));
    picks
}

fn sides_of(
    pick: &Pick,
    edges: &[(i32, i32)],
    feature: &dyn Fn(i32, i32) -> bool,
    end_floods: bool,
) -> (Vec<i32>, bool) {
    sides_on(&pick.positions, &pick.indices, edges, feature, end_floods)
}

/// C# `TagSides(indices, edges, feature, out divides, positions: positions)` with end floods, or
/// `endFloods: false` without positions.
fn sides_on(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    edges: &[(i32, i32)],
    feature: &dyn Fn(i32, i32) -> bool,
    end_floods: bool,
) -> (Vec<i32>, bool) {
    tag_sides(
        indices,
        edges,
        feature,
        end_floods,
        if end_floods { Some(positions) } else { None },
        (false, false),
        SideReach::default(),
    )
    .expect("the sides tag")
}

/// C# `ClosedChainsAndRunningOnEndsTagAsBefore`.
#[test]
fn closed_chains_and_running_on_ends_tag_as_before() {
    for pick in picks() {
        let name = &pick.name;
        let feature = feature_edges_over(&pick.positions, &pick.indices, FOLD_DEGREES);
        let chain_edges: Vec<Vec<(i32, i32)>> = pick
            .chains
            .iter()
            .map(|c| chain_on_the_mesh::chain_edges(c, pick.closed))
            .collect();

        let (grouped, _, _) = group_sides(
            &pick.indices,
            &chain_edges,
            &*feature,
            true,
            Some(&pick.positions),
            None,
            None,
        )
        .expect("the sides group");
        let (grouped_before, _, _) = group_sides(
            &pick.indices,
            &chain_edges,
            &*feature,
            false,
            None,
            None,
            None,
        )
        .expect("the sides group");
        assert_eq!(grouped, grouped_before, "{name}");
        assert!(grouped.iter().any(|&t| t != 0), "{name}");

        if chain_edges.len() == 1 {
            let (sides, divides) = sides_of(&pick, &chain_edges[0], &*feature, true);
            let (sides_before, divided_before) = sides_of(&pick, &chain_edges[0], &*feature, false);
            assert_eq!(divides, divided_before, "{name}");
            assert_eq!(sides, sides_before, "{name}");

            // The drag's known-room path tags only the band.
            let field = field_of(
                &pick.positions,
                &pick.indices,
                &pick.chains[0],
                pick.closed,
                0.3,
            );
            let (band, band_divides) = tag_band(
                &pick.positions,
                &pick.indices,
                &chain_edges[0],
                &*feature,
                &field,
                SNAP,
                true,
            )
            .expect("the band tags");
            let (band_before, band_divided_before) = tag_band(
                &pick.positions,
                &pick.indices,
                &chain_edges[0],
                &*feature,
                &field,
                SNAP,
                false,
            )
            .expect("the band tags");
            assert_eq!(band_divides, band_divided_before, "{name}");
            assert_eq!(band, band_before, "{name}");
        }
    }
}

/// C# `RoomOf` of `TheRoomAtAnEndIsNeverAboveTheRoomBlindToIt` and
/// `AnEndFloodOverAForwardNoseStopsShortOfTheBottom`: `BandRoom.RoomOfCutMesh` over the solid
/// tagged as given, with C#'s defaults for every optional argument.
fn room_of_tags(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    edges: &[(i32, i32)],
    tags: &[i32],
) -> f64 {
    let unit_setback = chain_on_the_mesh::unit_setbacks(
        positions,
        indices,
        chain,
        false,
        edges,
        BandProfile::Fillet,
    )
    .expect("every chain edge stands on two triangles");
    let mesh = Rc::new(RefCell::new(
        LevelLineMesh::from_triangles(positions, indices, Some(tags)).expect("the mesh builds"),
    ));
    let field_at = |s: f64| field_of(positions, indices, chain, false, s);
    band_room::room_of_cut_mesh(
        &mesh,
        edges,
        &field_at,
        &unit_setback,
        false,
        SNAP,
        0.0,
        BandProfile::Fillet,
        0,
        None,
        None,
    )
    .expect("the room reads")
    .0
}

/// C# `TheRoomAtAnEndIsNeverAboveTheRoomBlindToIt`: the room SEES end tags - an end region's
/// boundary with the rest of the part is foreign - and on these fixtures that only ever holds the
/// room down.
#[test]
fn the_room_at_an_end_is_never_above_the_room_blind_to_it() {
    for pick in end_face_picks() {
        let name = &pick.name;
        let feature = feature_edges_over(&pick.positions, &pick.indices, FOLD_DEGREES);
        let chain = &pick.chains[0];
        let edges = chain_on_the_mesh::chain_edges(chain, false);
        let (with_ends, _) = sides_of(&pick, &edges, &*feature, true);
        let (without, _) = sides_of(&pick, &edges, &*feature, false);
        assert!(
            with_ends
                .iter()
                .any(|&t| t == FIRST_END_TAG || t == SECOND_END_TAG),
            "{name}: the pick must actually carry an end tag for this to say anything"
        );

        let room = room_of_tags(&pick.positions, &pick.indices, chain, &edges, &with_ends);
        let room_before = room_of_tags(&pick.positions, &pick.indices, chain, &edges, &without);
        assert!(room <= room_before, "{name}: {room} against {room_before}");
    }
}

/// C# `AnEndFloodStopsShortOfTheBottomThroughASmoothRounding`: where the end face meets the rest
/// of the part SMOOTHLY by a rounding that turns back ALONG the chain, the end flood stops at that
/// first bend.
#[test]
fn an_end_flood_stops_short_of_the_bottom_through_a_smooth_rounding() {
    let (positions, indices, chain, bottom_wall) = slab_with_a_rounded_end();
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    let edges = chain_on_the_mesh::chain_edges(&chain, false);
    let (tags, _) = sides_on(&positions, &indices, &edges, &*feature, true);

    assert!(
        bottom_wall.iter().all(|&t| tags[t] == 0),
        "the bottom is not the end face"
    );
    assert_eq!(tags.iter().filter(|&&t| t == FIRST_END_TAG).count(), 2);
    let flat_end: Vec<usize> = (0..indices.len())
        .filter(|&t| {
            corners(indices[t])
                .iter()
                .all(|&v| positions[v as usize].x == 1.0)
        })
        .collect();
    assert_eq!(flat_end.len(), 2);
    assert!(flat_end.iter().all(|&t| tags[t] == SECOND_END_TAG));
    assert_eq!(
        tags.iter().filter(|&&t| t == SECOND_END_TAG).count(),
        flat_end.len()
    );
}

/// C# `AnEndFloodOverAForwardNoseStopsShortOfTheBottom`: the size-free end flood reads a forward
/// nose as one end face, but must not run on round it into the bottom, which runs back ALONGSIDE
/// the chain.
#[test]
fn an_end_flood_over_a_forward_nose_stops_short_of_the_bottom() {
    let (positions, indices, chain, bottom, thickness) = slab_with_a_forward_nose();
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    let edges = chain_on_the_mesh::chain_edges(&chain, false);
    let (tags, _) = sides_on(&positions, &indices, &edges, &*feature, true);

    assert!(
        tags.contains(&SECOND_END_TAG),
        "the nose is the second end's face"
    );
    assert!(
        bottom.iter().all(|&t| tags[t] == 0),
        "the bottom runs alongside the chain, it is not the end face"
    );

    let room = room_of_tags(&positions, &indices, &chain, &edges, &tags);
    println!("forward nose: room {room}, thickness {thickness}");
    assert!(room <= thickness, "{room} against {thickness}");
}

/// C# `ACubeEdgeTagsItsTwoEndFaces`: one top edge of a cube - the top and the front are the two
/// sides, the end face round vertex 4 is end 3 and the one round vertex 5 end 4, and the bottom
/// and the back stay 0.
#[test]
fn a_cube_edge_tags_its_two_end_faces() {
    let (positions, indices) = cube_solid(1.0);
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    let edges = chain_on_the_mesh::chain_edges(&[4, 5], false);

    let taggings = [
        sides_on(&positions, &indices, &edges, &*feature, true).0,
        group_sides(
            &indices,
            std::slice::from_ref(&edges),
            &*feature,
            true,
            Some(&positions),
            None,
            None,
        )
        .expect("the sides group")
        .0,
    ];

    let on_face = |t: usize, on: &dyn Fn(Vector3) -> bool| {
        corners(indices[t])
            .iter()
            .all(|&v| on(positions[v as usize]))
    };
    let left: Vec<usize> = (0..indices.len())
        .filter(|&t| on_face(t, &|p| p.x == -1.0))
        .collect();
    let right: Vec<usize> = (0..indices.len())
        .filter(|&t| on_face(t, &|p| p.x == 1.0))
        .collect();
    let count = |tags: &[i32], tag: i32| tags.iter().filter(|&&t| t == tag).count();
    for tags in &taggings {
        assert!(left.iter().all(|&t| tags[t] == FIRST_END_TAG));
        assert!(right.iter().all(|&t| tags[t] == SECOND_END_TAG));
        assert_eq!(count(tags, 3), left.len());
        assert_eq!(count(tags, 4), right.len());
        assert_eq!(count(tags, 1), 2);
        assert_eq!(count(tags, 2), 2);
        assert_eq!(count(tags, 0), 4);
    }
}

/// C# `OneEndOnAFaceAndOneRunningOnTagsOnlyTheFace`: HALF a cube's top edge, [4, Mid] - the end at
/// 4 stands on an end face and gets end tag 3; the end at Mid runs on along the crease, so there is
/// no tag 4, and every triangle off the end face is tagged as before end tags.
#[test]
fn one_end_on_a_face_and_one_running_on_tags_only_the_face() {
    let (positions, indices, mid) = cube_with_mid_edge_vertex(1.0);
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    let edges = chain_on_the_mesh::chain_edges(&[4, mid], false);
    let (tags, _) = sides_on(&positions, &indices, &edges, &*feature, true);
    let (before, _) = sides_on(&positions, &indices, &edges, &*feature, false);

    let end_face: Vec<usize> = (0..indices.len())
        .filter(|&t| {
            corners(indices[t])
                .iter()
                .all(|&v| positions[v as usize].x == -1.0)
        })
        .collect();
    assert_eq!(end_face.len(), 2);
    assert!(end_face.iter().all(|&t| tags[t] == FIRST_END_TAG));
    assert_eq!(tags.iter().filter(|&&t| t == SECOND_END_TAG).count(), 0);
    assert!((0..tags.len())
        .filter(|t| !end_face.contains(t))
        .all(|t| tags[t] == before[t]));
}

/// C# `AHexagonalPrismEdgeTagsItsTwoEndFaces`: one top edge of a hexagonal prism (obtuse ends) -
/// the top cap and the edge's wall quad are the sides, the two neighbouring wall quads the ends,
/// and the bottom cap and the three far walls stay 0.
#[test]
fn a_hexagonal_prism_edge_tags_its_two_end_faces() {
    let (positions, indices, top) = hex_prism();
    let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
    let edges = chain_on_the_mesh::chain_edges(&[top[0], top[1]], false);
    let (tags, divides) = sides_on(&positions, &indices, &edges, &*feature, true);

    assert!(divides);
    let count = |tag: i32| tags.iter().filter(|&&t| t == tag).count();
    let mut side_counts = [count(1), count(2)];
    side_counts.sort();
    assert_eq!(side_counts, [2, 6]);
    assert_eq!(count(3), 2);
    assert_eq!(count(4), 2);
    assert_eq!(count(0), 6 + 6);

    // The end at top[0] is the wall before it (outline edge 5-0), the end at top[1] the wall after
    // (1-2).
    let bottom0 = top.len() as i32;
    let is_wall = |t: usize, i: i32| {
        let j = (i + 1) % 6;
        corners(indices[t]).iter().all(|&c| {
            c == top[i as usize] || c == top[j as usize] || c == bottom0 + i || c == bottom0 + j
        })
    };

    for (t, &tag) in tags.iter().enumerate() {
        if is_wall(t, 5) {
            assert_eq!(tag, FIRST_END_TAG);
        }

        if is_wall(t, 1) {
            assert_eq!(tag, SECOND_END_TAG);
        }
    }
}

/// C# `AnObtuseEndFacePickFillsClosed`: a hexagonal prism's top edge (obtuse ends) fills closed
/// through the end cap.
#[test]
fn an_obtuse_end_face_pick_fills_closed() {
    let (positions, indices, top) = hex_prism();
    let gap = band_gap_entry_points::build(
        &positions,
        &indices,
        &[top[0], top[1]],
        false,
        Some(feature_edges_over(&positions, &indices, FOLD_DEGREES)),
        0.3,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick builds or refuses by name");

    let fill = gap_fill::fill(&gap, BandProfile::Fillet, gap_fill::DEFAULT_FILLET_SEGMENTS)
        .expect("the fill runs");
    assert_eq!(fill.refusal, GapFillRefusal::None);
    let mesh = fill.mesh.as_ref().expect("a filled mesh").borrow();
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        mesh.vertices(),
        &sweep_mesh_measures::kept(&mesh)
    ));
}

fn corners(t: (i32, i32, i32)) -> [i32; 3] {
    [t.0, t.1, t.2]
}

/// C# `FieldOf`: the chain's field at `size`, from the unit setbacks of a fillet.
fn field_of(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    closed: bool,
    size: f64,
) -> ChainField {
    let unit = chain_on_the_mesh::unit_setbacks(
        positions,
        indices,
        chain,
        closed,
        &chain_on_the_mesh::chain_edges(chain, closed),
        BandProfile::Fillet,
    )
    .expect("every chain edge stands on two triangles");
    let points: Vec<Vector3> = chain.iter().map(|&v| positions[v as usize]).collect();
    let setbacks: Vec<f64> = unit.iter().map(|u| u * size).collect();
    ChainField::from_polyline(&points, closed, &setbacks).expect("the chain makes a field")
}

/// C# `SlabWithARoundedEnd`: a slab 2 x 2 x 1 whose edge between its x = +1 face and its bottom
/// is rounded (radius 0.5, 12 facets of 7.5 degrees - smooth under the fold threshold). The pick
/// is the top front edge from x = -1 to x = +1. Answers the solid, the chain and the bottom wall's
/// triangles.
#[allow(clippy::type_complexity)]
fn slab_with_a_rounded_end() -> (Vec<Vector3>, Vec<(i32, i32, i32)>, Vec<i32>, Vec<usize>) {
    let mut outline = vec![Vector2::new(-1.0, 0.0), Vector2::new(0.5, 0.0)];
    for k in 1..=12 {
        let angle = (-std::f64::consts::PI / 2.0) + (k as f64 * std::f64::consts::PI / 24.0);
        outline.push(Vector2::new(
            0.5 + (0.5 * angle.cos()),
            0.5 + (0.5 * angle.sin()),
        ));
    }

    outline.push(Vector2::new(1.0, 1.0));
    outline.push(Vector2::new(-1.0, 1.0));
    let n = outline.len() as i32;
    let (prism, prism_indices, _) = prism_on_outline(&outline, Vector2::new(0.0, 0.5), 2.0);

    // (x, y, z) -> (x, z - 1, y) swaps two axes, which mirrors the solid, so every triangle is
    // re-wound.
    let positions: Vec<Vector3> = prism
        .iter()
        .map(|p| Vector3::new(p.x, p.z - 1.0, p.y))
        .collect();
    let indices: Vec<(i32, i32, i32)> = prism_indices.iter().map(|t| (t.0, t.2, t.1)).collect();

    // The prism's bottom ring (index n + i) lands at y = -1; the last two profile corners are
    // (1, 1) and (-1, 1), so the chain is the front top edge from x = -1 to x = +1.
    let chain = vec![n + (n - 1), n + (n - 2)];
    let bottom_wall = (0..indices.len())
        .filter(|&t| {
            corners(indices[t]).iter().all(|&v| {
                let p = positions[v as usize];
                p.z == 0.0 && p.x <= 0.5
            })
        })
        .collect();
    (positions, indices, chain, bottom_wall)
}

/// C# `SlabWithAForwardNose`: a slab whose end is a faceted soft NOSE bulging FORWARD past the
/// chain's end and rolling round under into the bottom (32 facets, smooth under the fold
/// threshold). Answers the solid, the chain, the bottom's triangles and the slab's thickness.
#[allow(clippy::type_complexity)]
fn slab_with_a_forward_nose() -> (
    Vec<Vector3>,
    Vec<(i32, i32, i32)>,
    Vec<i32>,
    Vec<usize>,
    f64,
) {
    const L: f64 = 1.0;
    const T: f64 = 1.0;
    const E: f64 = 0.5;
    let r = ((E * E) + (T * T)).sqrt();
    let from = -std::f64::consts::PI / 2.0;
    let to = T.atan2(-E);
    const FACETS: i32 = 32;
    let mut outline = vec![Vector2::new(-1.0, -r)];
    for k in 0..FACETS {
        let angle = from + (k as f64 * (to - from) / FACETS as f64);
        outline.push(Vector2::new(L + E + (r * angle.cos()), r * angle.sin()));
    }

    outline.push(Vector2::new(L, T));
    outline.push(Vector2::new(-1.0, T));
    let n = outline.len() as i32;
    let (prism, prism_indices, _) = prism_on_outline(&outline, Vector2::new(0.0, 0.0), 2.0);
    let positions: Vec<Vector3> = prism
        .iter()
        .map(|p| Vector3::new(p.x, p.z - 1.0, p.y))
        .collect();
    let indices: Vec<(i32, i32, i32)> = prism_indices.iter().map(|t| (t.0, t.2, t.1)).collect();
    let chain = vec![n + (n - 1), n + (n - 2)];
    let bottom = (0..indices.len())
        .filter(|&t| {
            corners(indices[t])
                .iter()
                .all(|&v| (positions[v as usize].z + r).abs() < 1e-9)
        })
        .collect();
    (positions, indices, chain, bottom, T + r)
}

/// C# `HexPrism`: a hexagonal prism of circumradius 2, height 4 (`PrismOnOutline`'s default).
fn hex_prism() -> SolidWithChain {
    let outline: Vec<Vector2> = (0..6)
        .map(|i| {
            let a = i as f64 * std::f64::consts::PI / 3.0;
            Vector2::new(2.0 * a.cos(), 2.0 * a.sin())
        })
        .collect();
    prism_on_outline(&outline, Vector2::new(0.0, 0.0), 4.0)
}
