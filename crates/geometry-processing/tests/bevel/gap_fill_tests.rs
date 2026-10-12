//! Port of `LevelLine/GapFillTests.cs`: the fill, held to the requirement - what comes out is a
//! closed, consistently oriented, single-shell solid with a bevel in it; the volume it lost is
//! positive, grows with the size, matches the closed form where there is one, and moves only a
//! little when the size does. C# `[Arguments]` rows are loops over the same-named test.

use std::collections::{HashMap, HashSet};
use std::time::Instant;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::gap_fill::{
    self, arc_point, DEFAULT_FILLET_SEGMENTS, FILL_TAG,
};
use geometry_processing::bevel::level_line::{
    BandGapResult, BandProfile, GapFillRefusal, GapFillResult, LevelLineMesh, LevelLineTriangle,
};
use vector_math::{Matrix4X4, Vector3};

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::box_with_square_hole;
use crate::level_line_test_geometry::{
    cube_solid, cylinder_solid, signed_volume, signed_volume_of, winding_number,
};

/// A fold threshold that reads a 90 degree corner as a feature and a 16-sided cylinder as smooth.
const FOLD_DEGREES: f64 = 45.0;

/// The sizes every pick is asked to build at - the ladder, all of it well under the room.
const LADDER: [f64; 5] = [0.05, 0.1, 0.2, 0.3, 0.4];

const PICKS: [&str; 3] = ["cube top rim", "box mouth rim", "cylinder top rim"];

type Solid = (Vec<Vector3>, Vec<(i32, i32, i32)>);

/// The one claim the arc rests on: control points P, C, Q with weight cos(theta / 2), theta the
/// supplement of the angle PCQ, draw the circular arc centred on P + Q - C when P and Q stand the
/// same distance from C. Proven at a right angle, where the centre and radius are known by hand.
#[test]
fn a_right_angled_arc_is_the_circle_about_the_opposite_corner() {
    let r = 0.7;
    let c = Vector3::new(1.0, 2.0, -3.0);
    let p = c + Vector3::new(r, 0.0, 0.0);
    let q = c + Vector3::new(0.0, r, 0.0);
    let centre = p + q - c;

    for step in 0..=16 {
        let point = arc_point(p, c, q, step as f64 / 16.0);
        assert!(
            ((point - centre).length() - r).abs() <= 1e-12,
            "every point of the arc stands the setback from the corner opposite the edge"
        );
    }

    // And its ends are its ends, tangent to the two legs: the first step leaves P along CP's normal.
    assert!((arc_point(p, c, q, 0.0) - p).length() < 1e-15);
    assert!((arc_point(p, c, q, 1.0) - q).length() < 1e-15);
}

/// The central claim of the fill. Every fixture, both profiles, every rung of the ladder: the
/// result is closed (every directed edge used once and its opposite once), one shell, and no
/// triangle is degenerate.
#[test]
fn every_pick_builds_a_closed_consistently_oriented_single_shell() {
    for pick in PICKS {
        for profile in [BandProfile::Chamfer, BandProfile::Fillet] {
            for size in LADDER {
                let fill = build(pick, size, profile, DEFAULT_FILLET_SEGMENTS);
                assert!(fill.filled(), "{pick} at {size} is well within the room");
                let mesh = mesh(&fill);
                assert!(
                    is_closed_and_consistent(&mesh),
                    "{pick} {profile:?} at {size}: every directed edge once, its opposite once"
                );
                assert_eq!(shell_count(&mesh), 1);
                assert!(
                    smallest_live_area(&mesh) > 0.0,
                    "no triangle of the fill is degenerate"
                );
            }
        }
    }
}

/// The cube's top rim chamfer has a closed form, because the fill's surface there IS the four 45
/// degree planes: four prisms of side s and length 2, less the s^3 / 3 each pair of them shares at
/// a corner.
#[test]
fn the_cube_top_rim_chamfer_removes_exactly_the_closed_form_volume() {
    for s in LADDER {
        let fill = build(
            "cube top rim",
            s,
            BandProfile::Chamfer,
            DEFAULT_FILLET_SEGMENTS,
        );
        let expected = (4.0 * s * s) - (4.0 * s * s * s / 3.0);

        assert!(
            (removed(&fill, &cube_solid(1.0)) - expected).abs() <= 1e-6,
            "four prisms of {s} less their four corner overlaps"
        );
    }
}

/// The fillet cuts the same rails as the chamfer of the same size (a 90 degree edge sets back by
/// r tan 45 = r), and bulges towards the edge between them, so it takes strictly less away and
/// strictly more than nothing. That is the true bound; it is not any fixed fraction, because the
/// arc's chord and the corner move together.
#[test]
fn a_fillet_takes_less_than_its_chamfer_and_more_than_nothing() {
    for s in LADDER {
        let chamfer = removed(
            &build(
                "cube top rim",
                s,
                BandProfile::Chamfer,
                DEFAULT_FILLET_SEGMENTS,
            ),
            &cube_solid(1.0),
        );
        let fillet = removed(
            &build(
                "cube top rim",
                s,
                BandProfile::Fillet,
                DEFAULT_FILLET_SEGMENTS,
            ),
            &cube_solid(1.0),
        );

        assert!(fillet > 0.0);
        assert!(
            fillet < chamfer,
            "the arc stands outside its chord, towards the edge, so less material goes"
        );
    }
}

/// The volume that leaves is positive and climbs with the size asked for, on every fixture.
#[test]
fn the_removed_volume_is_positive_and_grows_with_the_size() {
    for pick in PICKS {
        let solid = solid_of(pick);
        for profile in [BandProfile::Chamfer, BandProfile::Fillet] {
            let mut last = 0.0;
            for size in LADDER {
                let removed = removed(&build(pick, size, profile, DEFAULT_FILLET_SEGMENTS), &solid);
                assert!(removed > last, "{pick} {profile:?} at {size}");
                last = removed;
            }
        }
    }
}

/// Consistency, as the requirement states it: nearby sizes give nearby results. A step of 1e-3 in
/// the size moves the volume by about the surface the bevel covers times that step, never by a
/// jump.
#[test]
fn nearby_sizes_remove_nearly_the_same_volume() {
    let solid = cube_solid(1.0);
    let h = 1e-3;
    for s in [0.1, 0.2, 0.3] {
        let here = removed(
            &build(
                "cube top rim",
                s,
                BandProfile::Fillet,
                DEFAULT_FILLET_SEGMENTS,
            ),
            &solid,
        );
        let there = removed(
            &build(
                "cube top rim",
                s + h,
                BandProfile::Fillet,
                DEFAULT_FILLET_SEGMENTS,
            ),
            &solid,
        );

        // d/ds of 4s^2 is 8s, so at these sizes the honest bound on one step is a few times 8 s h.
        assert!((there - here).abs() < 10.0 * 8.0 * s * h);
        assert!(there > here);
    }
}

/// A fillet takes material away and never adds any: every arc point it invents stands inside the
/// solid it came from - on the hull side of the edge it rounds.
#[test]
fn every_fillet_arc_vertex_stands_inside_the_original_solid() {
    for s in LADDER {
        let fill = build(
            "cube top rim",
            s,
            BandProfile::Fillet,
            DEFAULT_FILLET_SEGMENTS,
        );
        assert!(!fill.added_vertices.is_empty());
        let mesh = mesh(&fill);

        for &v in &fill.added_vertices {
            let p = mesh.position(v);
            let outside = f64::max(f64::max(p.x.abs(), p.y.abs()), p.z.abs()) - 1.0;
            assert!(
                outside <= 1e-9,
                "the cube is convex, so inside it is every half space of its faces"
            );
        }
    }
}

/// The same claim for every pick, and for a solid that is not convex: a fillet of CONVEX edges may
/// only take material away, so not one point the fill invents may stand outside the solid that
/// went in.
///
/// Read by the solid angle the original triangles subtend at the point - one turn inside, none
/// outside, half a turn on the surface - which needs nothing of the solid but its own triangles,
/// so the claim is the same sentence for a box with a mouth through it as for a cube. A vertex
/// sitting exactly ON the surface reads half, so the bar is the half-way mark: anything at or above
/// it is inside or on.
#[test]
fn no_pick_invents_a_point_outside_the_solid() {
    for pick in PICKS {
        let solid = solid_of(pick);
        for s in LADDER {
            let fill = build(pick, s, BandProfile::Fillet, DEFAULT_FILLET_SEGMENTS);
            assert!(!fill.added_vertices.is_empty());
            let mesh = mesh(&fill);

            for &v in &fill.added_vertices {
                let turns = winding_number(&solid.0, &solid.1, mesh.position(v));
                assert!(
                    turns > 0.45,
                    "{pick} at {s} put a fill vertex outside the solid it is cutting into"
                );
            }
        }
    }
}

/// Tangency, to the accuracy a chorded arc can have: the first segment leaving the top face leans
/// away from that face by no more than the angle one segment of the arc subtends - which is what
/// "tangent, drawn with N segments" means.
#[test]
fn the_fillet_leaves_the_kept_face_along_it() {
    let segments = 6;
    let fill = build("cube top rim", 0.3, BandProfile::Fillet, segments);
    let subtended = (std::f64::consts::PI / 2.0) / segments as f64;
    let mesh = mesh(&fill);

    let mut checked_any = false;
    for t in live_triangles(&mesh) {
        for (from, to) in [(t.a, t.b), (t.b, t.c), (t.c, t.a)] {
            let p = mesh.position(from);
            let q = mesh.position(to);
            let leaves_the_top_face = (p.z - 1.0).abs() < 1e-9 && (q.z - 1.0).abs() > 1e-9;
            if !leaves_the_top_face || t.tag != FILL_TAG {
                continue;
            }

            let step = (q - p).get_normal();
            let lean = step.z.abs().asin();
            assert!(lean <= subtended + 1e-9);
            checked_any = true;
        }
    }

    assert!(checked_any, "the fill does touch the top face");
}

/// The fill is a property of the part and not of where the part stands - and it is the same twice.
///
/// A fillet used to move by about half a percent of what it removed: at a chain corner both rails
/// carry a vertex meaning the corner's own parameter, reached by two different roots whose last
/// bits a motion perturbs, and the zipper's exact comparison then fanned the corner about the other
/// apex. The zipper reads that tie within a relative 1e-9 of the chain's length now, so both
/// profiles are invariant.
#[test]
fn a_moved_part_loses_the_same_volume_and_the_same_input_gives_the_same_answer() {
    let (positions, indices) = cube_solid(1.0);
    let motion = Matrix4X4::create_rotation(Vector3::new(0.3, -0.7, 0.2))
        * Matrix4X4::create_translation(5.0, -3.0, 11.0);
    let moved: Vec<Vector3> = positions.iter().map(|&p| p.transform(motion)).collect();

    for profile in [BandProfile::Chamfer, BandProfile::Fillet] {
        let still = build("cube top rim", 0.3, profile, DEFAULT_FILLET_SEGMENTS);
        let again = build("cube top rim", 0.3, profile, DEFAULT_FILLET_SEGMENTS);
        let gap = band_gap(&moved, &indices, &[4, 5, 6, 7], true, 0.3, profile);
        let travelled =
            gap_fill::fill(&gap, profile, DEFAULT_FILLET_SEGMENTS).expect("the fill runs");

        assert!(travelled.filled());
        let still_mesh = mesh(&still);
        let travelled_mesh = mesh(&travelled);
        assert_eq!(
            signed_volume(&mesh(&again)),
            signed_volume(&still_mesh),
            "the same input gives the same answer, bit for bit"
        );
        assert_eq!(travelled_mesh.triangle_count(), still_mesh.triangle_count());
        assert_eq!(travelled.added_triangles.len(), still.added_triangles.len());
        assert!(is_closed_and_consistent(&travelled_mesh));

        let apart = (signed_volume(&travelled_mesh) - signed_volume(&still_mesh)).abs();
        let removed = signed_volume_of(&positions, &indices) - signed_volume(&still_mesh);
        assert!(
            apart < removed * 1e-9,
            "the corner's tie is read within a tolerance, so every placement fans it the same way"
        );
    }
}

/// An open chain ending on a cube's end faces fills closed, each end capped in its end face.
#[test]
fn an_open_chain_on_end_faces_fills_closed() {
    let (positions, indices) = cube_solid(1.0);
    let gap = band_gap(
        &positions,
        &indices,
        &[4, 5],
        false,
        0.3,
        BandProfile::Fillet,
    );

    let fill =
        gap_fill::fill(&gap, BandProfile::Fillet, DEFAULT_FILLET_SEGMENTS).expect("the fill runs");

    assert_eq!(fill.refusal, GapFillRefusal::None);
    assert!(is_closed_and_consistent(&mesh(&fill)));
}

/// What the fill costs and how big it is - rows recorded, not a budget asserted.
#[test]
fn the_cost_of_a_fill_is_recorded() {
    let mut rows = Vec::new();
    for pick in PICKS {
        for profile in [BandProfile::Chamfer, BandProfile::Fillet] {
            let watch = Instant::now();
            let fill = build(pick, 0.3, profile, DEFAULT_FILLET_SEGMENTS);
            let elapsed = watch.elapsed();
            let live = live_triangles(&mesh(&fill)).len();
            rows.push(format!(
                "{pick}, {profile:?} 0.3: {:.1} ms, {} fill triangles, {} new vertices, {live} live triangles, removed {:.6}",
                elapsed.as_secs_f64() * 1000.0,
                fill.added_triangles.len(),
                fill.added_vertices.len(),
                removed(&fill, &solid_of(pick))
            ));
        }
    }

    for row in &rows {
        println!("{row}");
    }

    assert_eq!(rows.len(), 6);
}

fn mesh(fill: &GapFillResult) -> std::cell::Ref<'_, LevelLineMesh> {
    fill.mesh.as_ref().expect("the filled mesh").borrow()
}

/// C# `BandGap.Build(positions, indices, chain, closed, FeatureEdgesOver(...), size, profile)`.
fn band_gap(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    closed: bool,
    size: f64,
    profile: BandProfile,
) -> BandGapResult {
    band_gap_entry_points::build(
        positions,
        indices,
        chain,
        closed,
        Some(feature_edges_over(positions, indices, FOLD_DEGREES)),
        size,
        profile,
        &BandGapBuildOptions::default(),
    )
    .expect("a valid pick")
}

/// C# `SolidOf`.
fn solid_of(pick: &str) -> Solid {
    match pick {
        "cube top rim" => cube_solid(1.0),
        "box mouth rim" => {
            let (p, i, _) = box_with_square_hole(2.0, 0.5, 1.0);
            (p, i)
        }
        "cylinder top rim" => cylinder_solid(16, 1.0, 2.0),
        other => panic!("{other}"),
    }
}

/// C# `ChainOf`.
fn chain_of(pick: &str) -> Vec<i32> {
    match pick {
        "cube top rim" => vec![4, 5, 6, 7],
        "box mouth rim" => box_with_square_hole(2.0, 0.5, 1.0).2,
        "cylinder top rim" => (16..32).collect(),
        other => panic!("{other}"),
    }
}

/// C# `Build`.
fn build(pick: &str, size: f64, profile: BandProfile, fillet_segments: i32) -> GapFillResult {
    let (positions, indices) = solid_of(pick);
    let gap = band_gap(&positions, &indices, &chain_of(pick), true, size, profile);

    gap_fill::fill(&gap, profile, fillet_segments).expect("the fill runs")
}

/// How much smaller the filled solid is than the one that went in.
fn removed(fill: &GapFillResult, solid: &Solid) -> f64 {
    signed_volume_of(&solid.0, &solid.1) - signed_volume(&mesh(fill))
}

/// C# `LiveTriangles`.
fn live_triangles(mesh: &LevelLineMesh) -> Vec<LevelLineTriangle> {
    (0..mesh.triangle_count())
        .filter(|&t| !mesh.is_removed(t))
        .map(|t| mesh.triangles()[t as usize])
        .collect()
}

/// C# `SmallestLiveArea` (LINQ `Min`).
fn smallest_live_area(mesh: &LevelLineMesh) -> f64 {
    live_triangles(mesh)
        .iter()
        .map(|t| {
            (mesh.position(t.b) - mesh.position(t.a))
                .cross(mesh.position(t.c) - mesh.position(t.a))
                .length()
                / 2.0
        })
        .fold(f64::INFINITY, f64::min)
}

/// Closed and consistently oriented in one statement: count every directed edge of the live mesh
/// and require each to appear exactly once, with its opposite appearing exactly once too.
fn is_closed_and_consistent(mesh: &LevelLineMesh) -> bool {
    let mut seen: HashMap<(i32, i32), i32> = HashMap::new();
    for t in live_triangles(mesh) {
        for e in [(t.a, t.b), (t.b, t.c), (t.c, t.a)] {
            *seen.entry(e).or_insert(0) += 1;
        }
    }

    seen.iter()
        .all(|(e, &count)| count == 1 && seen.get(&(e.1, e.0)) == Some(&1))
}

/// How many separate pieces the live mesh is in - one, for a solid with a bevel cut into it.
pub fn shell_count(mesh: &LevelLineMesh) -> i32 {
    let live: Vec<i32> = (0..mesh.triangle_count())
        .filter(|&t| !mesh.is_removed(t))
        .collect();

    let mut neighbours: HashMap<i32, Vec<i32>> = live.iter().map(|&t| (t, Vec::new())).collect();
    for (low, high) in mesh.edges() {
        let standing = mesh.triangles_on(low, high);
        for i in 0..standing.len() {
            for j in i + 1..standing.len() {
                neighbours
                    .get_mut(&standing[i])
                    .expect("a live triangle")
                    .push(standing[j]);
                neighbours
                    .get_mut(&standing[j])
                    .expect("a live triangle")
                    .push(standing[i]);
            }
        }
    }

    let mut seen = HashSet::new();
    let mut shells = 0;
    for &start in &live {
        if !seen.insert(start) {
            continue;
        }

        shells += 1;
        let mut stack = vec![start];
        while let Some(at) = stack.pop() {
            for &next in &neighbours[&at] {
                if seen.insert(next) {
                    stack.push(next);
                }
            }
        }
    }

    shells
}
