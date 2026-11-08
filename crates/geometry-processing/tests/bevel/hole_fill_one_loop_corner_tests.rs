//! The corner filler for the picks whose band leaves ONE loop but whose chains do not each keep one
//! rail piece per side - with the filler on, `CornerFill` lays the strips it can and hands
//! everything else open to the hole filler. Each row is held to the requirement set across the room
//! and under a rigid motion. Parameter cases of one fixture are one test each, suffixed with the
//! argument; the cases that range over several fixtures run as one loop.

use std::cell::{Cell, RefCell};
use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::corner_cap_hole_fill::last_hole_fill_failure;
use solidmesh::bevel::level_line::corner_fill_fill::WATCH_OPEN_HOLES;
use solidmesh::bevel::level_line::{
    flat_hole_fill, BandGapRefusal, BandGapResult, BandProfile, GapFillRefusal, GapFillResult,
};
use solidmesh_math::Vector3;
use solidmesh_mesh::csg::boolean_processing::BooleanOperandVerdict;

use crate::boolean_kernel_gate;
use crate::hole_fill_corner_tests::{
    fill_with_the_filler, fill_with_the_filler_watched, fixture_feature, fixture_profile,
    smallest_kept_area,
};
use crate::level_line_fixture::LevelLineFixture;
use crate::level_line_test_geometry::signed_volume;
use crate::round_mouth_fillet_crossings::crossing_pairs;
use crate::sweep_mesh_measures;

/// At 0.1 of the room. Inner Right Sides and Center's mixed corner: with each
/// strip end set back to its field tie every row at the corner is flat at radius r and the filler's
/// patch is small, so it builds across the room and at the size the user picked.
#[test]
fn inner_right_sides_corner_builds_0_1() {
    meets_the_requirement("inner-right-sides-and-center-op0", Some(0.1));
}

/// The same corner at 0.5 of the room.
#[test]
fn inner_right_sides_corner_builds_0_5() {
    meets_the_requirement("inner-right-sides-and-center-op0", Some(0.5));
}

/// The same corner at 0.9 of the room.
#[test]
fn inner_right_sides_corner_builds_0_9() {
    meets_the_requirement("inner-right-sides-and-center-op0", Some(0.9));
}

/// The same corner at the whole room.
#[test]
fn inner_right_sides_corner_builds_1_0() {
    meets_the_requirement("inner-right-sides-and-center-op0", Some(1.0));
}

/// The same corner at the size the user picked.
#[test]
fn inner_right_sides_corner_builds_null() {
    meets_the_requirement("inner-right-sides-and-center-op0", None);
}

/// At half its room it builds the same wherever the
/// part sits (turned 2.45 rad about Z and moved, as Windows' round-off reached with the part
/// unmoved).
#[test]
fn inner_right_sides_corner_builds_the_same_turned() {
    let fixture = Rc::new(LevelLineFixture::load("inner-right-sides-and-center-op0"));
    let size = 0.5 * room_of(&fixture);
    let source = sweep_mesh_measures::signed_volume(&fixture.positions, &fixture.indices);
    let (sin, cos) = 2.45f64.sin_cos();
    let turned: Vec<Vector3> = fixture
        .positions
        .iter()
        .map(|p| {
            Vector3::new(
                (p.x * cos) - (p.y * sin) - 5.245347151181356,
                (p.x * sin) + (p.y * cos) - 9.39248496638261,
                p.z + 1.3633845426903015,
            )
        })
        .collect();

    let (unmoved, _) = build_and_fill(
        &fixture,
        &fixture.positions,
        size,
        &fixture.chains,
        &fixture.closed,
    );
    let (moved, _) = build_and_fill(&fixture, &turned, size, &fixture.chains, &fixture.closed);
    let unmoved = unmoved.expect("the band builds");
    let moved = moved.expect("the turned band builds");
    assert_eq!(unmoved.refusal, GapFillRefusal::None);
    assert_eq!(moved.refusal, GapFillRefusal::None);
    let volume = signed_volume(&unmoved.mesh.as_ref().unwrap().borrow());
    let moved_volume = signed_volume(&moved.mesh.as_ref().unwrap().borrow());
    println!(
        "unmoved {volume} ({} fill triangles), turned {moved_volume} ({})",
        unmoved.added_triangles.len(),
        moved.added_triangles.len()
    );
    assert!((moved_volume - volume).abs() <= 1e-6 * source.abs());
}

/// Meeting Edge Round at 0.1 of the room: with the knife read past round-off it builds the same at
/// every placement.
#[test]
fn meeting_edge_round_closes_by_the_filler_0_1() {
    meets_the_requirement("meeting-edge-round-op0", Some(0.1));
}

/// Meeting Edge Round at 0.5 of the room.
#[test]
fn meeting_edge_round_closes_by_the_filler_0_5() {
    meets_the_requirement("meeting-edge-round-op0", Some(0.5));
}

/// Meeting Edge Round at 0.9 of the room.
#[test]
fn meeting_edge_round_closes_by_the_filler_0_9() {
    meets_the_requirement("meeting-edge-round-op0", Some(0.9));
}

/// Every case: with the turned-back stop vertex
/// handed to the corner hole the rail zips and the pick builds.
#[test]
fn a_rail_running_past_its_end_to_the_end_plane_builds() {
    for name in [
        "five-mm-center-bottom-op0",
        "bottom-center-flows-into-center-op0",
    ] {
        for fraction in [0.1, 0.5, 0.9] {
            meets_the_requirement(name, Some(fraction));
        }
    }
}

/// Bottom center flows into center does not cross itself at its saved size.
#[test]
fn bottom_center_flows_into_center_does_not_cross_itself_at_its_saved_size() {
    does_not_cross_itself_at_its_saved_size("bottom-center-flows-into-center-op0");
}

/// Five mm center bottom does not cross itself at its saved size.
#[test]
fn five_mm_center_bottom_does_not_cross_itself_at_its_saved_size() {
    does_not_cross_itself_at_its_saved_size("five-mm-center-bottom-op0");
}

/// Where each round's rail on the slanted face
/// reaches crease 1-9 it stands at the setback from its chain.
#[test]
fn the_rail_meets_the_end_plane_crease_at_the_setback() {
    for name in [
        "five-mm-center-bottom-op0",
        "bottom-center-flows-into-center-op0",
    ] {
        let fixture = Rc::new(LevelLineFixture::load(name));
        let gap = build_at(
            &fixture,
            &fixture.positions,
            fixture.size,
            &fixture.chains,
            &fixture.closed,
            fixture.known_room,
        );
        assert!(gap.built());

        // The crease 1-9 runs along x at y = -113.5, z = 10; its two ends are the chains' end
        // vertices 1 and 9.
        let on_the_crease = |p: Vector3| {
            (p.y + 113.5).abs() < 1e-6
                && (p.z - 10.0).abs() < 1e-6
                && p.x > -15.0 + 1e-6
                && p.x < -5.0 - 1e-6
        };
        let mesh = gap.mesh.as_ref().expect("a built mesh").borrow();
        let mut checked_chains = HashSet::new();
        for gap_loop in &gap.loops {
            let owners = gap_loop
                .vertex_owners
                .as_ref()
                .expect("the walk read the owners");
            for (i, &vertex) in gap_loop.vertices.iter().enumerate() {
                let p = mesh.position(vertex);
                let owner = owners[i];
                let chain = &gap.chains[owner as usize];
                let (a, b) = (chain.points[0], chain.points[chain.points.len() - 1]);
                let along = (b - a).get_normal();
                let width = ((p - a) - (along * (p - a).dot(along))).length();
                let setback = chain.setback_at(chain.length / 2.0);
                if !on_the_crease(p) || width > setback + 1e-3 {
                    continue;
                }

                println!(
                    "{name}: chain {owner} rail on crease 1-9 at {p:?}, {width} from the chain, setback {setback}"
                );
                assert!((width - setback).abs() <= 1e-6 * setback, "{name}");
                checked_chains.insert(owner);
            }
        }

        assert_eq!(checked_chains.len(), 2, "{name}");
    }
}

/// Five mm Center Bottom is mirror-symmetric about x = -10, and
/// so is what the strips leave to the caps - both end holes close flat, and the corner hole at the
/// junction is its own mirror image to float noise.
#[test]
fn the_mirrored_holes_are_mirrored() {
    let fixture = Rc::new(LevelLineFixture::load("five-mm-center-bottom-op0"));
    let gap = build_at(
        &fixture,
        &fixture.positions,
        fixture.size,
        &fixture.chains,
        &fixture.closed,
        fixture.known_room,
    );
    let holes: Rc<RefCell<Vec<Vec<Vector3>>>> = Rc::new(RefCell::new(Vec::new()));
    let seen = Rc::clone(&holes);
    let mesh_rc = Rc::clone(gap.mesh.as_ref().expect("a built mesh"));
    WATCH_OPEN_HOLES.with(|w| {
        *w.borrow_mut() = Some(Rc::new(move |open: &HashSet<(i32, i32)>| {
            let next: HashMap<i32, i32> = open.iter().copied().collect();
            let mut keys: Vec<i32> = next.keys().copied().collect();
            keys.sort();
            let mesh = mesh_rc.borrow();
            let mut walked = HashSet::new();
            for start in keys {
                let mut hole = Vec::new();
                let mut at = start;
                while walked.insert(at) {
                    let Some(&to) = next.get(&at) else { break };
                    hole.push(mesh.position(at));
                    at = to;
                }

                if !hole.is_empty() {
                    seen.borrow_mut().push(hole);
                }
            }
        }));
    });
    fill_with_the_filler(&gap, fixture_profile(&fixture), fixture.segments);
    WATCH_OPEN_HOLES.with(|w| *w.borrow_mut() = None);

    let mean =
        |hole: &Vec<Vector3>| hole.iter().fold(Vector3::ZERO, |s, &p| s + p) / hole.len() as f64;
    let mirror = |p: Vector3| Vector3::new(-20.0 - p.x, p.y, p.z);
    let same_place = gap.mesh.as_ref().unwrap().borrow().same_place();
    let snap = if same_place > 0.0 { same_place } else { 1e-9 };
    let holes = holes.borrow();
    let single = |pick: &dyn Fn(&Vec<Vector3>) -> bool| {
        let found: Vec<&Vec<Vector3>> = holes.iter().filter(|h| pick(h)).collect();
        assert_eq!(found.len(), 1);
        found[0].clone()
    };
    let left = single(&|h| mean(h).x < -12.0 && mean(h).z > 5.0);
    let right = single(&|h| mean(h).x > -8.0 && mean(h).z > 5.0);
    let left_flat = flat_hole_fill::fill(&left, snap, None).is_some();
    let right_flat = flat_hole_fill::fill(&right, snap, None).is_some();
    println!(
        "end holes: left {} flat {left_flat}, right {} flat {right_flat}",
        left.len(),
        right.len()
    );
    assert!(left_flat);
    assert!(right_flat);

    let corner = single(&|h| (mean(h).x + 10.0).abs() < 1e-3 && mean(h).z > 5.0);
    let worst = corner
        .iter()
        .map(|&p| {
            corner
                .iter()
                .map(|&q| (q - mirror(p)).length())
                .fold(f64::INFINITY, f64::min)
        })
        .fold(f64::NEG_INFINITY, f64::max);
    println!(
        "corner hole: {} vertices, worst mirror miss {worst:e}",
        corner.len()
    );
    assert!(worst <= 1e-6);
}

/// Five mm Center Bottom's three concave beads meet at the
/// junction with one ball, and the patch is laid on it as a convex corner's is.
#[test]
fn the_three_concave_beads_meet_on_the_ball() {
    let fixture = Rc::new(LevelLineFixture::load("five-mm-center-bottom-op0"));
    let gap = build_at(
        &fixture,
        &fixture.positions,
        fixture.size,
        &fixture.chains,
        &fixture.closed,
        fixture.known_room,
    );
    let worst = Rc::new(Cell::new(f64::NAN));
    let seen = Rc::clone(&worst);
    let filled = fill_with_the_filler_watched(
        &gap,
        fixture_profile(&fixture),
        fixture.segments,
        Some(Rc::new(move |_, vertices, n, _, rows| {
            if rows.len() == 3 {
                let centre = rows[0].centre;
                seen.set(
                    vertices
                        .iter()
                        .skip(n as usize)
                        .map(|&p| ((p - centre).length() - rows[0].radius).abs())
                        .fold(f64::NEG_INFINITY, f64::max),
                );
            }
        })),
    );
    assert_eq!(filled.refusal, GapFillRefusal::None);

    println!(
        "the junction's patch stands at most {:e} off the ball",
        worst.get()
    );
    // A NaN fails this comparison, so an unmeasured worst is a failure.
    assert!(worst.get() <= 1e-6);
}

fn does_not_cross_itself_at_its_saved_size(name: &str) {
    let fixture = Rc::new(LevelLineFixture::load(name));
    let (filled, band) = build_and_fill(
        &fixture,
        &fixture.positions,
        fixture.size,
        &fixture.chains,
        &fixture.closed,
    );
    assert_eq!(band, BandGapRefusal::None);
    let filled = filled.expect("the band builds");
    assert_eq!(filled.refusal, GapFillRefusal::None);
    let mesh = filled.mesh.as_ref().unwrap().borrow();
    assert_eq!(crossing_pairs(&mesh), 0);
    assert_ne!(
        boolean_kernel_gate::verdict(&mesh),
        BooleanOperandVerdict::SelfIntersecting
    );
}

fn meets_the_requirement(name: &str, fraction_of_room: Option<f64>) {
    let fixture = Rc::new(LevelLineFixture::load(name));
    let room = room_of(&fixture);
    let size = fraction_of_room.map_or(fixture.size, |fraction| fraction * room);
    let source = sweep_mesh_measures::signed_volume(&fixture.positions, &fixture.indices);

    let (filled, band) = build_and_fill(
        &fixture,
        &fixture.positions,
        size,
        &fixture.chains,
        &fixture.closed,
    );
    let label = format!("{name} at {size:.4} ({fraction_of_room:?} of room {room:.4})");
    println!(
        "{label}: band {band:?}, fill {:?} ({:?}), chain {:?}, vertex {:?}, edge {:?}",
        filled.as_ref().map(|f| f.refusal),
        last_hole_fill_failure(),
        filled.as_ref().map(|f| f.offending_chain),
        filled.as_ref().map(|f| f.offending_vertex),
        filled.as_ref().map(|f| f.open_edge)
    );
    assert_eq!(band, BandGapRefusal::None, "{label}");
    let filled = filled.expect("the band builds");
    assert_eq!(filled.refusal, GapFillRefusal::None, "{label}");

    let mesh = filled.mesh.as_ref().unwrap().borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    assert!(
        sweep_mesh_measures::is_closed_and_consistent(mesh.vertices(), &kept),
        "{label}"
    );
    assert_eq!(
        sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
        1,
        "{label}"
    );
    let smallest_area = smallest_kept_area(&mesh, &kept);
    assert!(smallest_area > 1e-12, "{label}");

    // Wound as the input, and moved the way the chains move it: each chain built alone moves the
    // volume by its own amount (a convex chain takes material, a concave one adds it), and the pick
    // together moves it the way their sum does.
    let volume = signed_volume(&mesh);
    let mut alone = 0.0;
    let mut alone_built = 0;
    for k in 0..fixture.chains.len() {
        let (one, one_band) = build_and_fill(
            &fixture,
            &fixture.positions,
            size,
            std::slice::from_ref(&fixture.chains[k]),
            &[fixture.closed[k]],
        );
        let moved_by = match &one {
            Some(one) if one.refusal == GapFillRefusal::None => {
                signed_volume(&one.mesh.as_ref().unwrap().borrow()) - source
            }
            _ => f64::NAN,
        };
        println!(
            "{label}: chain {k} alone: band {one_band:?}, fill {:?}, moved {moved_by}",
            one.as_ref().map(|f| f.refusal)
        );
        if !moved_by.is_nan() {
            alone += moved_by;
            alone_built += 1;
        }
    }

    println!(
        "{label}: volume {volume} of {source} (moved {}); chains alone {alone_built}/{} built, summed move {alone}; smallest area {smallest_area}, {} fill triangles",
        volume - source,
        fixture.chains.len(),
        filled.added_triangles.len()
    );
    assert_eq!(sign(volume), sign(source), "{label}");
    assert_ne!(volume, source, "{label}");
    // The pick moves the volume as the chains alone do, up to what the pick does and they do not:
    // at each vertex two or more chains share, the pick fills ONE corner where each chain alone
    // capped its own end, which differs by at most the shape verdict's corner allowance
    // (BevelShapeVerdict, 3.375 s^3 at the largest setback there).
    if alone_built == fixture.chains.len() {
        let (senses, allowance) = senses_and_corner_allowance(&fixture, size);
        println!(
            "{label}: pick minus chains alone {}, corner allowance {allowance}",
            volume - source - alone
        );
        assert!(
            (volume - source - alone).abs() <= allowance + 1e-6 * source.abs(),
            "{label}"
        );
        let distinct: HashSet<i32> = senses.iter().copied().collect();
        if distinct.len() == 1 && alone.abs() > 1e-9 * source.abs() {
            assert_eq!(sign(volume - source), sign(alone), "{label}");
        }
    }

    // Unchanged under a rigid motion: the same pick turned and moved builds the same volume.
    let moved: Vec<Vector3> = fixture.positions.iter().map(|&p| move_point(p)).collect();
    let (moved_fill, moved_band) =
        build_and_fill(&fixture, &moved, size, &fixture.chains, &fixture.closed);
    println!(
        "{label}: moved band {moved_band:?}, fill {:?} ({:?}), vertex {:?}",
        moved_fill.as_ref().map(|f| f.refusal),
        last_hole_fill_failure(),
        moved_fill.as_ref().map(|f| f.offending_vertex)
    );
    assert_eq!(moved_band, BandGapRefusal::None, "{label}");
    let moved_fill = moved_fill.expect("the moved band builds");
    assert_eq!(moved_fill.refusal, GapFillRefusal::None, "{label}");
    let moved_volume = signed_volume(&moved_fill.mesh.as_ref().unwrap().borrow());
    println!("{label}: moved volume {moved_volume}");
    assert!(
        (moved_volume - volume).abs() <= 1e-6 * source.abs(),
        "{label}"
    );
}

/// The sign of `x`: -1, 0 or 1, zero for either zero.
fn sign(x: f64) -> i32 {
    if x > 0.0 {
        1
    } else if x < 0.0 {
        -1
    } else {
        0
    }
}

/// Each chain's sense (+1 convex, -1 concave, read off its first
/// edge), and the corner allowance BevelShapeVerdict grants a pick: 3.375 s^3 at every vertex two
/// or more chains share, s the largest setback of any chain edge there.
fn senses_and_corner_allowance(fixture: &LevelLineFixture, size: f64) -> (Vec<i32>, f64) {
    let p = &fixture.positions;
    let round = fixture_profile(fixture) == BandProfile::Fillet;

    let turn_degrees = |a: i32, b: i32| {
        let on: Vec<(i32, i32, i32)> = fixture
            .indices
            .iter()
            .copied()
            .filter(|t| (t.0 == a || t.1 == a || t.2 == a) && (t.0 == b || t.1 == b || t.2 == b))
            .collect();
        let normal = |t: (i32, i32, i32)| {
            (p[t.1 as usize] - p[t.0 as usize])
                .cross(p[t.2 as usize] - p[t.0 as usize])
                .get_normal()
        };
        let n0 = normal(on[0]);
        let angle = n0.dot(normal(on[1])).clamp(-1.0, 1.0).acos() * 180.0 / std::f64::consts::PI;
        let far = [on[1].0, on[1].1, on[1].2]
            .into_iter()
            .find(|&v| v != a && v != b)
            .expect("a third corner");
        if (p[far as usize] - p[a as usize]).dot(n0) < 0.0 {
            angle
        } else {
            -angle
        }
    };

    let setback = |turn: f64| {
        if round {
            size * (turn.abs() * std::f64::consts::PI / 360.0).tan()
        } else {
            size
        }
    };

    let mut senses = Vec::new();
    // Insertion-ordered; the sum below does not depend on the order
    // beyond floating-point association, which this keeps.
    let mut setback_at: indexmap::IndexMap<i32, (i32, f64)> = indexmap::IndexMap::new();
    for (k, chain) in fixture.chains.iter().enumerate() {
        let count = if fixture.closed[k] {
            chain.len()
        } else {
            chain.len() - 1
        };
        senses.push(sign(turn_degrees(chain[0], chain[1])));
        let mut ends = HashSet::new();
        for i in 0..count {
            let (a, b) = (chain[i], chain[(i + 1) % chain.len()]);
            let edge_setback = setback(turn_degrees(a, b));
            for v in [a, b] {
                let seen = setback_at.get(&v).copied().unwrap_or((0, 0.0));
                let added = if ends.insert(v) { 1 } else { 0 };
                setback_at.insert(v, (seen.0 + added, seen.1.max(edge_setback)));
            }
        }
    }

    let allowance = setback_at
        .values()
        .filter(|v| v.0 > 1)
        .map(|v| 3.375 * v.1.powi(3))
        .sum();
    (senses, allowance)
}

/// The room the fixture records, or - when it records none - the room its build
/// reads.
fn room_of(fixture: &Rc<LevelLineFixture>) -> f64 {
    if let Some(known) = fixture.known_room {
        return known;
    }

    build_at(
        fixture,
        &fixture.positions,
        fixture.size,
        &fixture.chains,
        &fixture.closed,
        None,
    )
    .room
}

/// The fixture's `BandGap.Build` with its seam, sharp turn and segments, at `size` with `known_room`.
fn build_at(
    fixture: &Rc<LevelLineFixture>,
    positions: &[Vector3],
    size: f64,
    chains: &[Vec<i32>],
    closed: &[bool],
    known_room: Option<f64>,
) -> BandGapResult {
    let options = BandGapBuildOptions {
        chord_tolerance: 0.0,
        snap: 1e-9,
        known_room,
        budget: None,
        seam_epsilon: fixture.seam_epsilon,
        sharp_turn_degrees: fixture.sharp_turn_degrees,
        fillet_segments: fixture.segments,
    };
    band_gap_entry_points::build_pick(
        positions,
        &fixture.indices,
        chains,
        closed,
        Some(fixture_feature(fixture)),
        size,
        fixture_profile(fixture),
        &options,
    )
    .expect("the pick builds or refuses by name")
}

/// `None` where the band refuses, with the band's refusal.
fn build_and_fill(
    fixture: &Rc<LevelLineFixture>,
    positions: &[Vector3],
    size: f64,
    chains: &[Vec<i32>],
    closed: &[bool],
) -> (Option<GapFillResult>, BandGapRefusal) {
    let known = if chains.len() == fixture.chains.len() {
        fixture.known_room
    } else {
        None
    };
    let gap = build_at(fixture, positions, size, chains, closed, known);
    let band = gap.refusal;
    if !gap.built() {
        return (None, band);
    }

    (
        Some(fill_with_the_filler(
            &gap,
            fixture_profile(fixture),
            fixture.segments,
        )),
        band,
    )
}

/// Carries a point by a fixed rigid motion.
fn move_point(p: Vector3) -> Vector3 {
    const ANGLE: f64 = 0.7;
    let (sin, cos) = ANGLE.sin_cos();
    Vector3::new(
        (p.x * cos) - (p.y * sin) + 13.25,
        (p.x * sin) + (p.y * cos) - 7.5,
        p.z + 3.125,
    )
}
