//! Port of `LevelLine/EndRegionShapesTests.cs`: end faces of shapes the other end-face tests do
//! not reach - a flat end face carrying export noise, a CURVED end face (the round of a D prism)
//! and an edge running into the inner corner of an L prism. C# `[Arguments]` rows are loops.

use std::collections::HashMap;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::band_gap_reading::{read_room, room_of};
use geometry_processing::bevel::level_line::band_side_tagging::{FIRST_END_TAG, SECOND_END_TAG};
use geometry_processing::bevel::level_line::band_side_tagging_ends::{tag_sides, SideReach};
use geometry_processing::bevel::level_line::chain_on_the_mesh::chain_edges;
use geometry_processing::bevel::level_line::end_face_flow::END_FACE_BEND_DEGREES;
use geometry_processing::bevel::level_line::gap_fill;
use geometry_processing::bevel::level_line::{BandProfile, GapFillRefusal};
use vector_math::dotnet_random::DotNetRandom;
use vector_math::{Vector2, Vector3};

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::l_prism;
use crate::level_line_room_geometry::SolidWithChain;
use crate::level_line_test_geometry::distance_to_surface;
use crate::level_line_test_geometry::winding_number;
use crate::level_line_test_solids::prism_on_outline;
use crate::sweep_mesh_measures;

const FOLD_DEGREES: f64 = 10.0;

/// Cells across each face of the noisy box.
const CELLS: i32 = 6;

/// The lattice row (in z) below which the noisy box's end face leans inward.
const BEND_ROW: i32 = 2;

fn corners(t: (i32, i32, i32)) -> [i32; 3] {
    [t.0, t.1, t.2]
}

fn tags_of(solid: &SolidWithChain) -> Vec<i32> {
    let (positions, indices, chain) = solid;
    let feature = feature_edges_over(positions, indices, FOLD_DEGREES);
    tag_sides(
        indices,
        &chain_edges(chain, false),
        &*feature,
        true,
        Some(positions),
        (false, false),
        SideReach::default(),
    )
    .expect("the chain has two sides")
    .0
}

fn room_of_solid(solid: &SolidWithChain) -> f64 {
    let (positions, indices, chain) = solid;
    room_of(
        positions,
        indices,
        chain,
        false,
        Some(feature_edges_over(positions, indices, FOLD_DEGREES)),
        BandProfile::Fillet,
        1e-9,
    )
    .expect("the room reads")
}

/// C#'s `BandGap.Build(positions, indices, chain, chainIsClosed: false, feature, size, Fillet)`
/// then `GapFill.Fill(built, Fillet)`: asserts both and that the result is closed; hands back the
/// kept triangles and the filled positions.
fn builds_and_fills_closed(
    solid: &SolidWithChain,
    size: f64,
    label: &str,
) -> (Vec<(i32, i32, i32)>, Vec<Vector3>) {
    let (positions, indices, chain) = solid;
    let built = band_gap_entry_points::build(
        positions,
        indices,
        chain,
        false,
        Some(feature_edges_over(positions, indices, FOLD_DEGREES)),
        size,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
    )
    .expect("a valid pick");
    assert!(built.built(), "{label}: {:?}", built.refusal);
    let fill = gap_fill::fill(
        &built,
        BandProfile::Fillet,
        gap_fill::DEFAULT_FILLET_SEGMENTS,
    )
    .expect("the fill runs");
    assert_eq!(fill.refusal, GapFillRefusal::None, "{label}");
    let mesh = fill.mesh.as_ref().expect("the filled mesh").borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    assert!(
        sweep_mesh_measures::is_closed_and_consistent(mesh.vertices(), &kept),
        "{label}"
    );
    (kept, mesh.vertices().to_vec())
}

/// The end region is the end face up to its first bend past `END_FACE_BEND_DEGREES`: noise well
/// under half a degree does not stop it, and a one-degree lean does.
#[test]
fn a_noisy_flat_end_face_stops_only_at_its_first_real_bend() {
    // (4, 2): a second one-degree lean below the first - two soft joins in a row are no round.
    for (bend_row, second_bend_row) in [(BEND_ROW, -1), (4, 2)] {
        let solid = noisy_end_box(bend_row, second_bend_row);
        let tags = tags_of(&solid);
        let (positions, indices, _) = &solid;

        // The end face's triangles, split at the bend row.
        let bend_z = -1.0 + (2.0 * bend_row as f64 / CELLS as f64);
        let end_face: Vec<usize> = (0..indices.len())
            .filter(|&t| {
                corners(indices[t])
                    .iter()
                    .all(|&v| positions[v as usize].x > 0.9)
            })
            .collect();
        let upper: Vec<usize> = end_face
            .iter()
            .copied()
            .filter(|&t| {
                corners(indices[t])
                    .iter()
                    .all(|&v| positions[v as usize].z >= bend_z - 1e-12)
            })
            .collect();
        let lower: Vec<usize> = end_face
            .iter()
            .copied()
            .filter(|t| !upper.contains(t))
            .collect();
        let noise = largest_bend_degrees(positions, indices, &upper);

        println!(
            "noisy end face: largest noise bend {noise:.3} degrees, upper {}, lower {}, tagged upper {}, tagged lower {}",
            upper.len(),
            lower.len(),
            upper.iter().filter(|&&t| tags[t] == SECOND_END_TAG).count(),
            lower.iter().filter(|&&t| tags[t] == SECOND_END_TAG).count()
        );

        assert!(
            noise < END_FACE_BEND_DEGREES,
            "the fixture's noise must stay under the bend it is testing"
        );
        assert_eq!(upper.len() as i32, 2 * CELLS * (CELLS - bend_row));
        assert!(
            upper.iter().all(|&t| tags[t] == SECOND_END_TAG),
            "noise under half a degree is not a bend"
        );
        assert!(
            lower.iter().all(|&t| tags[t] == 0),
            "the one-degree lean is the first bend, and the end region stops there"
        );
    }
}

/// The top edge of a D prism's flat ends on the faceted ROUND at both ends: the bevel builds and
/// fills closed at half and nine tenths of its room, no fill vertex outside the part by more than
/// the chord tolerance.
#[test]
fn a_curved_end_face_builds_closed() {
    builds_closed_under_its_room("D prism flat edge", &d_prism(24));
}

/// The L prism's top edge along y = 2 runs into the inner corner at (2, 2), where it meets the
/// wall x = 2 head on: the bevel builds and fills closed at half and nine tenths of its room.
#[test]
fn an_edge_into_the_inner_corner_builds_closed() {
    let (positions, indices, top) = l_prism();

    // top[2] = (6, 2), top[3] = (2, 2) - the inner corner.
    let chain = vec![top[2], top[3]];
    builds_closed_under_its_room(
        "L prism edge into the inner corner",
        &(positions, indices, chain),
    );
}

/// A curved end face must not cap the room at one facet: each end's region is the round up to
/// where it comes to face away from the flat (45 degrees round), so the room is cos 45 = 0.7071 at
/// any facet count. A TRADE, not the right answer (see the C# remarks).
#[test]
fn a_curved_end_face_room_does_not_shrink_with_its_facets() {
    let coarse = room_of_solid(&d_prism(24));
    let fine = room_of_solid(&d_prism(48));
    println!("D prism room: 24 facets {coarse}, 48 facets {fine}");
    const TRADE: &str =
        "the round is end region to 45 degrees (a trade against a round rolling under), never one facet";
    assert!((coarse - 0.5f64.sqrt()).abs() <= 1e-6, "{TRADE}: {coarse}");
    assert!((fine - 0.5f64.sqrt()).abs() <= 1e-6, "{TRADE}: {fine}");
}

/// The same room when the reading is given the operation's bend threshold, as the app always
/// gives it.
#[test]
fn a_curved_end_face_room_holds_at_the_operations_bend_threshold() {
    for facets in [24, 48] {
        let (positions, indices, chain) = d_prism(facets);
        let feature = feature_edges_over(&positions, &indices, FOLD_DEGREES);
        let reading = read_room(
            &positions,
            &indices,
            &chain,
            false,
            Some(feature),
            BandProfile::Fillet,
            1e-9,
            None,
            0.0,
            FOLD_DEGREES,
        )
        .expect("the room reads");
        assert!(
            (reading.room - 0.5f64.sqrt()).abs() <= 1e-6,
            "{facets} facets: a curved end face is no plane to stand a ray on ({})",
            reading.room
        );
    }
}

/// Every size the D prism's flat edge is offered builds and fills closed, at 24 and 48 facets
/// alike, and each end has a region of its own on the round.
#[test]
fn every_size_in_the_curved_end_faces_room_builds_closed() {
    for facets in [24, 48] {
        let solid = d_prism(facets);
        let tags = tags_of(&solid);
        assert!(
            tags.contains(&FIRST_END_TAG) && tags.contains(&SECOND_END_TAG),
            "{facets} facets: both ends carry a region"
        );

        let room = room_of_solid(&solid);
        for step in 1..=19 {
            let size = step as f64 * 0.05 * room;
            builds_and_fills_closed(
                &solid,
                size,
                &format!(
                    "{facets} facets at {:.2} of room {room}",
                    step as f64 * 0.05
                ),
            );
        }
    }
}

/// A slab whose end is a quarter round rolling under into the bottom: its end region stops where
/// the round comes to face away from the top, so the bottom is never end region and its fold with
/// the front still bounds the room at the slab's thickness.
#[test]
fn a_round_rolling_under_into_the_bottom_stops_short_of_it() {
    let solid = slab_rolling_under();
    let tags = tags_of(&solid);
    let (positions, indices, _) = &solid;
    let bottom: Vec<usize> = (0..indices.len())
        .filter(|&t| {
            corners(indices[t])
                .iter()
                .all(|&v| positions[v as usize].z.abs() < 1e-9)
        })
        .collect();
    let room = room_of_solid(&solid);
    println!(
        "slab rolling under: bottom {}, tagged {}, end region {}, room {room}",
        bottom.len(),
        bottom.iter().filter(|&&t| tags[t] != 0).count(),
        tags.iter().filter(|&&t| t == SECOND_END_TAG).count()
    );

    assert!(!bottom.is_empty());
    assert!(
        bottom.iter().all(|&t| tags[t] == 0),
        "the bottom runs back alongside the chain"
    );
    assert!(tags.contains(&SECOND_END_TAG), "the round is the end face");
    assert!(room <= 1.0 + 1e-6, "the slab is 1 thick: {room}");
}

/// Where the end vertex sits inside the round's facet does not decide the bevel: on a corner, a
/// micron along, and half way along read rooms within a tenth of each other, and all three build
/// and fill closed at a quarter, half and three quarters of the smallest room.
#[test]
fn where_the_end_sits_on_a_facet_does_not_decide_the_bevel() {
    let parts: Vec<(f64, SolidWithChain)> = [0.0, 1e-6, 0.5]
        .iter()
        .map(|&t| (t, chord_prism(t)))
        .collect();
    let mut rooms = Vec::new();
    for (t, part) in &parts {
        rooms.push(room_of_solid(part));
        println!(
            "chord prism, end {t} along the facet: room {}",
            rooms.last().expect("a room")
        );
    }

    assert!((rooms[1] - rooms[0]).abs() <= 1e-4 * rooms[0], "{rooms:?}");
    assert!((rooms[2] - rooms[0]).abs() <= 0.1 * rooms[0], "{rooms:?}");

    let least = rooms.iter().copied().fold(f64::INFINITY, f64::min);
    for (t, part) in &parts {
        for fraction in [0.25, 0.5, 0.75] {
            builds_and_fills_closed(
                part,
                fraction * least,
                &format!("end {t} along the facet at {fraction}"),
            );
        }
    }
}

fn builds_closed_under_its_room(name: &str, solid: &SolidWithChain) {
    let (positions, indices, _) = solid;
    let room = room_of_solid(solid);
    println!("{name}: room {room}");
    assert!(room.is_finite() && room > 0.0, "{name}: room {room}");

    for fraction in [0.5, 0.9] {
        let size = fraction * room;
        let label = format!("{name} at {fraction} of the room");
        let (kept, vertices) = builds_and_fills_closed(solid, size, &label);

        // A bevel only takes material away, so every vertex it lays is inside the part or on it.
        let tolerance = size / 20.0;
        let mut used: Vec<i32> = Vec::new();
        for &t in &kept {
            for v in corners(t) {
                if !used.contains(&v) {
                    used.push(v);
                }
            }
        }
        let worst = used
            .iter()
            .map(|&v| vertices[v as usize])
            .filter(|&p| winding_number(positions, indices, p) < 0.5)
            .map(|p| distance_to_surface(positions, indices, p))
            .fold(0.0, f64::max);
        println!(
            "{label}: {} triangles, farthest outside {worst}",
            kept.len()
        );
        assert!(worst <= tolerance, "{label}: {worst}");
    }
}

/// The largest turn, in degrees, between two of the given triangles sharing an edge.
fn largest_bend_degrees(positions: &[Vector3], indices: &[(i32, i32, i32)], of: &[usize]) -> f64 {
    let normal = |t: (i32, i32, i32)| {
        let at = |v: i32| positions[v as usize];
        (at(t.1) - at(t.0)).cross(at(t.2) - at(t.0)).get_normal()
    };

    let mut largest: f64 = 0.0;
    let subset: Vec<(i32, i32, i32)> = of.iter().map(|&t| indices[t]).collect();
    let standing = sweep_mesh_measures::standing(&subset);
    for pair in standing.values().filter(|on| on.len() == 2) {
        let dot = normal(indices[of[pair[0] as usize]])
            .dot(normal(indices[of[pair[1] as usize]]))
            .clamp(-1.0, 1.0);
        largest = largest.max(dot.acos() * 180.0 / std::f64::consts::PI);
    }

    largest
}

/// The cube [-1, 1]^3 with every face cut into CELLS x CELLS squares. Its x = +1 end face leans
/// inward by one degree below the row `bend_row`, and above it carries a fixed pseudo-random x
/// noise of up to 3e-4 on its inner vertices. The chain is the top front edge (y = -1, z = +1) from
/// x = -1 to x = +1, so its second end stands on the noisy face.
fn noisy_end_box(bend_row: i32, second_bend_row: i32) -> SolidWithChain {
    let lean = (std::f64::consts::PI / 180.0).tan();
    let mut random = DotNetRandom::new(7);
    let cells = CELLS as f64;
    let bend_z = -1.0 + (2.0 * bend_row as f64 / cells);
    let second_bend_z = -1.0 + (2.0 * second_bend_row as f64 / cells);
    let mut positions: Vec<Vector3> = Vec::new();
    let mut vertex_at: HashMap<(i32, i32, i32), i32> = HashMap::new();

    let mut vertex = |i: i32, j: i32, k: i32, positions: &mut Vec<Vector3>| -> i32 {
        if let Some(&found) = vertex_at.get(&(i, j, k)) {
            return found;
        }

        let mut p = Vector3::new(
            -1.0 + (2.0 * i as f64 / cells),
            -1.0 + (2.0 * j as f64 / cells),
            -1.0 + (2.0 * k as f64 / cells),
        );
        if i == CELLS && k < bend_row {
            p.x -= (bend_z - p.z) * lean;
            if k < second_bend_row {
                p.x -= (second_bend_z - p.z) * lean;
            }
        } else if i == CELLS && j > 0 && j < CELLS && k > bend_row && k < CELLS {
            p.x += (random.next_double() - 0.5) * 6e-4;
        }

        vertex_at.insert((i, j, k), positions.len() as i32);
        positions.push(p);
        positions.len() as i32 - 1
    };

    let mut indices = Vec::new();

    // Each face: one axis held at 0 or CELLS, the other two walked; wound so the ideal normal
    // points out.
    for axis in 0..3 {
        for at in [0, CELLS] {
            for a in 0..CELLS {
                for b in 0..CELLS {
                    let l = |u: i32, v: i32| match axis {
                        0 => (at, u, v),
                        1 => (u, at, v),
                        _ => (u, v, at),
                    };
                    let mut quad = [l(a, b), l(a + 1, b), l(a + 1, b + 1), l(a, b + 1)];

                    // (u, v) walks (y, z), (x, z), (x, y): the quad's right-handed normal is +x,
                    // -y, +z.
                    let outward = if at == CELLS { 1 } else { -1 };
                    let handed = if axis == 1 { -1 } else { 1 };
                    if outward * handed < 0 {
                        quad.reverse();
                    }

                    let q: Vec<i32> = quad
                        .iter()
                        .map(|c| vertex(c.0, c.1, c.2, &mut positions))
                        .collect();
                    indices.push((q[0], q[1], q[2]));
                    indices.push((q[0], q[2], q[3]));
                }
            }
        }
    }

    let chain: Vec<i32> = (0..=CELLS)
        .map(|i| vertex(i, 0, CELLS, &mut positions))
        .collect();
    (positions, indices, chain)
}

/// A slab 2 x 2 x 1 (x, y, z) whose x = +1 end is a quarter round of radius 1 in 12 facets of 7.5
/// degrees (soft), from the top's end (1, 1) down and back to the bottom at x = 0, tangent. The
/// chain is the top front edge, y = -1, z = 1, from x = -1 to x = +1.
fn slab_rolling_under() -> SolidWithChain {
    let mut outline = vec![Vector2::new(-1.0, 0.0)];
    for k in 0..=12 {
        let angle = (-std::f64::consts::PI / 2.0) + (k as f64 * std::f64::consts::PI / 24.0);
        outline.push(Vector2::new(angle.cos(), 1.0 + angle.sin()));
    }

    outline[1] = Vector2::new(0.0, 0.0);
    outline[13] = Vector2::new(1.0, 1.0);
    outline.push(Vector2::new(-1.0, 1.0));
    let n = outline.len() as i32;
    let (prism, prism_indices, _) = prism_on_outline(&outline, Vector2::new(0.0, 0.5), 2.0);
    let positions = prism
        .iter()
        .map(|p| Vector3::new(p.x, p.z - 1.0, p.y))
        .collect();
    let indices = prism_indices.iter().map(|t| (t.0, t.2, t.1)).collect();
    (positions, indices, vec![n + (n - 1), n + (n - 2)])
}

/// A prism 4 tall on a cylinder of radius 1 in 48 facets, cut by a flat on the +x side whose top
/// edge is the chain. The flat meets the round at `along` of the way along the facet from the
/// corner at 60 degrees to the one at 67.5, and at its mirror.
fn chord_prism(along: f64) -> SolidWithChain {
    let corner = |degrees: f64| {
        Vector2::new(
            (degrees * std::f64::consts::PI / 180.0).cos(),
            (degrees * std::f64::consts::PI / 180.0).sin(),
        )
    };
    let end = corner(60.0) + ((corner(67.5) - corner(60.0)) * along);
    let mut outline = vec![end];
    for k in 9..=39 {
        outline.push(corner(k as f64 * 7.5));
    }

    outline.push(Vector2::new(end.x, -end.y));
    let (positions, indices, top) = prism_on_outline(&outline, Vector2::new(0.0, 0.0), 4.0);
    let chain = vec![top[outline.len() - 1], top[0]];
    (positions, indices, chain)
}

/// A prism 4 tall on a D: a half circle of radius 1 in `facets` facets bulging to +x, closed by the
/// flat x = 0. The chain is the flat's top edge, from (0, 1) to (0, -1).
fn d_prism(facets: i32) -> SolidWithChain {
    let mut outline: Vec<Vector2> = (0..=facets)
        .map(|k| (std::f64::consts::PI * k as f64 / facets as f64) - (std::f64::consts::PI / 2.0))
        .map(|angle| Vector2::new(angle.cos(), angle.sin()))
        .collect();
    outline[0] = Vector2::new(0.0, -1.0);
    outline[facets as usize] = Vector2::new(0.0, 1.0);

    let (positions, indices, top) = prism_on_outline(&outline, Vector2::new(0.3, 0.0), 4.0);
    let chain = vec![top[facets as usize], top[0]];
    (positions, indices, chain)
}
