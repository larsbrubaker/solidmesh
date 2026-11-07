//! Stopped ends on a faceted prism's top rim, read with stopped ends turned on: a short arc's ends
//! stop on their mitre planes; an arc over half the circle keeps fading, because its own far points
//! stand past each end's plane and a clip there would cut its own band; a leaving edge that is not
//! a feature crease stops nothing; and a sector's radial edge and arc meeting at a corner, the
//! arc's far end stopped, read a room the band builds at; that arc alone, stopped at its far end
//! only, fills closed with the rim past it sharp. Filled, a stopped arc is closed with its
//! neighbours sharp and cut flat on both end planes, showing its whole section there, on exact and
//! on single-precision input. The three-facet arc's removed volume is not asserted here: its rail
//! chords across the inner joints' corners within the chord tolerance, which a vertex at every
//! inner joint's rail corner answers on its own.

use std::collections::HashSet;
use std::f64::consts::PI;
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{
    build_pick, BandGapBuildOptions,
};
use geometry_processing::bevel::level_line::band_gap_reading::read_room_of_pick;
use geometry_processing::bevel::level_line::corner_cap_flat_ends::WATCH_FLAT_ENDS;
use geometry_processing::bevel::level_line::seam_edges::FeatureEdgeTest;
use geometry_processing::bevel::level_line::stopped_end_fill::try_stopped_ends;
use geometry_processing::bevel::level_line::{
    gap_fill, turn_stopped_ends_on, BandGapResult, BandProfile, ChainField, GapLoop,
    HoleFillTriangle, LevelLineMesh,
};
use vector_math::Vector3;

pub(crate) type Mesh = (Vec<Vector3>, Vec<(i32, i32, i32)>);

const RADIUS: f64 = 10.0;
const HEIGHT: f64 = 10.0;
/// A crease folds more than this; a 24-facet wall's seams (15 degrees) do not.
const FEATURE_DEGREES: f64 = 20.0;

/// A closed prism over the outline (counter-clockwise, seen from above), its caps fanned from
/// `fan` (an outline index), wound outward. Bottom ring is 0..n, top ring n..2n.
fn prism(outline: &[(f64, f64)], fan: usize) -> Mesh {
    let n = outline.len() as i32;
    let mut positions = Vec::new();
    for z in [0.0, HEIGHT] {
        for &(x, y) in outline {
            positions.push(Vector3::new(x, y, z));
        }
    }
    let bottom = |i: i32| i % n;
    let top = |i: i32| n + (i % n);
    let mut indices = Vec::new();
    for i in 0..n {
        indices.push((bottom(i), bottom(i + 1), top(i + 1)));
        indices.push((bottom(i), top(i + 1), top(i)));
    }
    let f = fan as i32;
    for k in 1..n - 1 {
        let (a, b) = ((f + k) % n, (f + k + 1) % n);
        indices.push((top(f), top(a), top(b)));
        indices.push((bottom(f), bottom(b), bottom(a)));
    }
    (positions, indices)
}

/// A 24-facet cylinder.
fn cylinder() -> Mesh {
    let outline: Vec<(f64, f64)> = (0..24)
        .map(|i| {
            let angle = f64::from(i) * 2.0 * PI / 24.0;
            (RADIUS * angle.cos(), RADIUS * angle.sin())
        })
        .collect();
    prism(&outline, 0)
}

/// The cylinder as a part stores it: every coordinate rounded to single precision. Upright, each
/// facet seam stays exactly vertical, so it lies exactly in the stored end plane through its top
/// vertex (a horizontal normal read from the rounded rim), though that plane leans a few float
/// steps off the exact radial one; a tilted part's seams stand off it (`stopped_end_tilted_tests.rs`).
fn single_precision_cylinder() -> Mesh {
    let (positions, indices) = cylinder();
    let rounded = positions
        .iter()
        .map(|p| {
            Vector3::new(
                f64::from(p.x as f32),
                f64::from(p.y as f32),
                f64::from(p.z as f32),
            )
        })
        .collect();
    (rounded, indices)
}

/// Every edge whose two triangles fold more than [`FEATURE_DEGREES`].
pub(crate) fn feature_edges((positions, indices): &Mesh) -> FeatureEdgeTest {
    let normal = |t: (i32, i32, i32)| {
        let a = positions[t.0 as usize];
        (positions[t.1 as usize] - a)
            .cross(positions[t.2 as usize] - a)
            .get_normal()
    };
    let mut features = HashSet::new();
    for (i, &t) in indices.iter().enumerate() {
        for &o in &indices[i + 1..] {
            let shared: Vec<i32> = [t.0, t.1, t.2]
                .into_iter()
                .filter(|&c| c == o.0 || c == o.1 || c == o.2)
                .collect();
            if shared.len() == 2 && normal(t).dot(normal(o)) < FEATURE_DEGREES.to_radians().cos() {
                features.insert((shared[0].min(shared[1]), shared[0].max(shared[1])));
            }
        }
    }
    Rc::new(move |a: i32, b: i32| features.contains(&(a.min(b), a.max(b))))
}

pub(crate) fn band(
    mesh: &Mesh,
    chains: &[Vec<i32>],
    feature: FeatureEdgeTest,
    size: f64,
) -> BandGapResult {
    let closed = vec![false; chains.len()];
    build_pick(
        &mesh.0,
        &mesh.1,
        chains,
        &closed,
        Some(feature),
        size,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
    )
    .expect("the band build reads the pick")
}

fn removed(gap: &BandGapResult) -> usize {
    let mesh = gap.mesh.as_ref().expect("the build kept its mesh").borrow();
    mesh.removed.iter().filter(|&&r| r).count()
}

/// The top rim's vertices from `first` through `last` on the cylinder.
fn top_rim(first: i32, last: i32) -> Vec<i32> {
    (first..=last).map(|i| 24 + (i % 24)).collect()
}

#[test]
fn a_short_arc_stops_at_both_ends_on_its_mitre_planes() {
    let mesh = cylinder();
    let chain = vec![top_rim(0, 3)];
    let gap = {
        let _on = turn_stopped_ends_on();
        band(&mesh, &chain, feature_edges(&mesh), 2.0)
    };
    assert!(gap.built(), "the stopped arc builds: {:?}", gap.refusal);
    let clips = gap.chains[0].end_clips.clone().expect("both ends stop");
    assert_eq!(clips.len(), 2);
    // The mitre plane at the first end is the radial plane through the axis at angle 0.
    let first = clips
        .iter()
        .find(|c| c.parameter == 0.0)
        .expect("the first end's clip");
    assert!((first.normal.dot(Vector3::new(0.0, -1.0, 0.0)) - 1.0).abs() < 1e-12);
}

#[test]
fn an_arc_over_half_the_circle_keeps_fading_and_builds_its_whole_band() {
    let mesh = cylinder();
    // 14 of 24 facets: 210 degrees, so each end's mitre plane has the arc's far points past it.
    let chain = vec![top_rim(0, 14)];
    let plain = band(&mesh, &chain, feature_edges(&mesh), 2.0);
    let stopped = {
        let _on = turn_stopped_ends_on();
        band(&mesh, &chain, feature_edges(&mesh), 2.0)
    };
    assert!(
        stopped.built(),
        "the long arc builds: {:?}",
        stopped.refusal
    );
    assert!(
        stopped.chains[0].end_clips.is_none(),
        "neither end of an arc over half the circle stops"
    );
    assert_eq!(
        removed(&stopped),
        removed(&plain),
        "the band is the fading one, uncut"
    );
}

#[test]
fn a_leaving_edge_that_is_no_feature_stops_nothing() {
    let mesh = cylinder();
    let chain = vec![top_rim(0, 3)];
    let chain_edges: HashSet<(i32, i32)> = chain[0]
        .windows(2)
        .map(|w| (w[0].min(w[1]), w[0].max(w[1])))
        .collect();
    let only_the_chain: FeatureEdgeTest =
        Rc::new(move |a: i32, b: i32| chain_edges.contains(&(a.min(b), a.max(b))));
    let gap = {
        let _on = turn_stopped_ends_on();
        band(&mesh, &chain, only_the_chain, 2.0)
    };
    assert!(
        gap.chains[0].end_clips.is_none(),
        "no feature crease carries on"
    );
}

#[test]
fn a_corner_pair_with_a_stopped_far_end_reads_a_room_the_band_builds_at() {
    // A 60 degree sector: centre, then an 8-facet arc. The radial top edge centre -> arc start, the
    // arc's first two facets and the vertical edge down from the arc start meet at a corner; the
    // arc's far end stops.
    let mut outline = vec![(0.0, 0.0)];
    for i in 0..=8 {
        let angle = f64::from(i) * (PI / 3.0) / 8.0;
        outline.push((RADIUS * angle.cos(), RADIUS * angle.sin()));
    }
    let mesh = prism(&outline, 0);
    let n = outline.len() as i32;
    let chains = vec![vec![n, n + 1], vec![n + 1, n + 2, n + 3], vec![n + 1, 1]];
    let closed = vec![false, false, false];
    let feature = feature_edges(&mesh);
    let options = BandGapBuildOptions::default();
    let _on = turn_stopped_ends_on();
    let reading = read_room_of_pick(
        &mesh.0,
        &mesh.1,
        &chains,
        &closed,
        Some(Rc::clone(&feature)),
        BandProfile::Fillet,
        options.snap,
        None,
        f64::NAN,
        options.seam_epsilon,
        options.sharp_turn_degrees,
    )
    .expect("the pick reads");
    // The vertical edge's band covers the arc's stretch of its end plane on the wall - the facet
    // seam, 2 R sin(7.5 deg) from the vertical edge - once its setback, tan(93.75 deg / 2) a unit
    // of size, reaches the seam; past that the arc keeps no rail on the wall and the band refuses.
    let covered =
        (2.0 * RADIUS * (7.5f64).to_radians().sin()) / (93.75f64 / 2.0).to_radians().tan();
    assert!(
        !band(&mesh, &chains, Rc::clone(&feature), covered * 1.001).built(),
        "the band refuses once the arc's wall rail is covered"
    );
    // The band's loop test hands the arc's last wall vertex to the vertical edge too once the
    // vertical edge's field there ties the arc's (zero on the plane) by the pick's tie rule,
    // 1e-9 of the longer chain (the 10 high vertical edge); a unit of size moves that field by
    // the vertical edge's unit setback. So the band refuses within `tie_size` under `covered`,
    // and the reading stands under that by the reading's margin, 2 snaps a unit, and no more than
    // that again.
    let unit = (93.75f64 / 2.0).to_radians().tan();
    let tie_size = 1e-9 * HEIGHT / unit;
    let margin = 2.0 * options.snap / unit;
    println!(
        "sector reading {} ({:?}), covered at {covered}, tie {tie_size}, margin {margin}",
        reading.room, reading.room_set_by
    );
    assert!(
        reading.room <= covered - tie_size && reading.room >= covered - tie_size - 2.0 * margin,
        "the reading stops where the arc's wall rail is covered to the tie, {}: {} ({:?})",
        covered - tie_size,
        reading.room,
        reading.room_set_by
    );
    assert_eq!(
        (reading.room_set_by.chain, reading.room_set_by.other_chain),
        (1, 2),
        "the arc's stopped end, covered by the vertical edge"
    );
    let gap = band(&mesh, &chains, feature, reading.room);
    assert!(gap.chains[1].end_clips.is_some(), "the arc's far end stops");
    assert!(
        gap.built(),
        "the band builds at the room read, {}: {:?}",
        reading.room,
        gap.refusal
    );
}

/// The signed volume of the live triangles.
fn volume(mesh: &LevelLineMesh) -> f64 {
    let mut six = 0.0;
    for (i, t) in mesh.triangles.iter().enumerate() {
        if mesh.removed[i] {
            continue;
        }
        let (a, b, c) = (
            mesh.vertices[t.a as usize],
            mesh.vertices[t.b as usize],
            mesh.vertices[t.c as usize],
        );
        six += a.dot(b.cross(c));
    }
    six / 6.0
}

/// Every live edge a->b has exactly one live b->a and no second a->b.
pub(crate) fn is_closed(mesh: &LevelLineMesh) -> bool {
    let mut edges = std::collections::HashMap::new();
    for (i, t) in mesh.triangles.iter().enumerate() {
        if mesh.removed[i] {
            continue;
        }
        for (a, b) in [(t.a, t.b), (t.b, t.c), (t.c, t.a)] {
            *edges.entry((a, b)).or_insert(0) += 1;
        }
    }
    edges
        .iter()
        .all(|(&(a, b), &n)| n == 1 && edges.get(&(b, a)) == Some(&1))
}

/// Whether the live mesh keeps the edge a-b.
pub(crate) fn keeps_edge(mesh: &LevelLineMesh, a: i32, b: i32) -> bool {
    mesh.triangles
        .iter()
        .enumerate()
        .any(|(i, t)| !mesh.removed[i] && [(t.a, t.b), (t.b, t.c), (t.c, t.a)].contains(&(a, b)))
}

/// The area of the live triangles lying in the radial plane through top vertex `rim`: all three
/// corners within 1e-9 of it on exact input, and on single-precision input on it as the kernel
/// reads a stopped plane (`ChainField::on_a_stopped_plane`).
fn area_in_the_radial_plane(mesh: &LevelLineMesh, rim: i32, single: bool) -> f64 {
    let p = mesh.vertices[(24 + rim % 24) as usize];
    let normal = Vector3::new(-p.y, p.x, 0.0).get_normal();
    let mut area = 0.0;
    for (i, t) in mesh.triangles.iter().enumerate() {
        if mesh.removed[i] {
            continue;
        }
        let corners = [t.a, t.b, t.c].map(|v| mesh.vertices[v as usize]);
        let on = |c: Vector3| {
            if single {
                ChainField::on_a_stopped_plane(p, normal, 0.0, c)
            } else {
                (c - p).dot(normal).abs() <= 1e-9
            }
        };
        if corners.iter().all(|&c| on(c)) {
            area += (corners[1] - corners[0])
                .cross(corners[2] - corners[0])
                .length()
                / 2.0;
        }
    }
    area
}

/// The round's section in an end plane with the arc laid in `segments` even chords: the square section
/// (the corner square less the chord fan from the arc's centre) slid along the facet onto the
/// radial plane, which stretches it across by 1 / cos of the facet's half angle.
fn chorded_section_in_the_end_plane(size: f64, segments: i32) -> f64 {
    let alpha = PI / 2.0 / f64::from(segments);
    let square = size * size - f64::from(segments) * size * size / 2.0 * alpha.sin();
    square / (PI / 24.0).cos()
}

/// Fills a stopped arc over `top_rim(0, facets)` at size 2 in 4 segments, and checks it closed,
/// its neighbours' rim edges sharp and both ends cut flat with the whole section in the plane.
/// Hands back the filled part's volume.
fn fills_flat_and_closed(mesh: &Mesh, facets: i32, single: bool) -> f64 {
    let size = 2.0;
    let segments = 4;
    let gap = {
        let _on = turn_stopped_ends_on();
        band(mesh, &[top_rim(0, facets)], feature_edges(mesh), size)
    };
    assert!(gap.built(), "the stopped arc builds: {:?}", gap.refusal);
    let closed_flat = std::rc::Rc::new(std::cell::Cell::new(0));
    {
        let seen = Rc::clone(&closed_flat);
        WATCH_FLAT_ENDS.with(|w| {
            *w.borrow_mut() = Some(Rc::new(
                move |_: &LevelLineMesh, _: &[i32], _: &[HoleFillTriangle]| {
                    seen.set(seen.get() + 1);
                },
            ));
        });
    }
    let filled = gap_fill::fill_with_budget(&gap, BandProfile::Fillet, segments, None);
    WATCH_FLAT_ENDS.with(|w| *w.borrow_mut() = None);
    let filled = filled.expect("the fill runs");
    assert_eq!(
        closed_flat.get(),
        2,
        "each end hole is closed by the flat fill in its plane"
    );
    assert!(
        filled.filled(),
        "the stopped arc fills: {:?}",
        filled.refusal
    );
    let handle = Rc::clone(filled.mesh.as_ref().expect("the fill keeps its mesh"));
    let filled_mesh = handle.borrow();
    assert!(is_closed(&filled_mesh), "the filled arc is closed");
    // The unpicked neighbours' rim edges stay sharp, kept as the part had them.
    assert!(
        keeps_edge(&filled_mesh, 24 + 23, 24) || keeps_edge(&filled_mesh, 24, 24 + 23),
        "the rim edge before the arc stays sharp"
    );
    let after = 24 + (facets + 1) % 24;
    let last = 24 + facets % 24;
    assert!(
        keeps_edge(&filled_mesh, last, after) || keeps_edge(&filled_mesh, after, last),
        "the rim edge after the arc stays sharp"
    );
    let chorded = chorded_section_in_the_end_plane(size, segments);
    let exact = (1.0 - PI / 4.0) * size * size / (PI / 24.0).cos();
    for rim in [0, facets] {
        let area = area_in_the_radial_plane(&filled_mesh, rim, single);
        println!(
            "end plane at rim vertex {rim}: in-plane fill {area}, exact section {exact}, chorded \
             {chorded}"
        );
        // The profile's points are not evenly spaced in angle, so the chorded section is read to
        // a thousandth.
        assert!(
            area >= exact && (area - chorded).abs() <= 1e-3 * chorded,
            "the end plane at rim vertex {rim} shows the round's whole section, cut flat: {area} \
             against {chorded}"
        );
    }
    volume(&filled_mesh)
}

#[test]
fn one_facet_rounded_with_stopped_ends_is_cut_flat_with_its_whole_profile_between_its_end_planes() {
    let mesh = cylinder();
    let kept = fills_flat_and_closed(&mesh, 1, false);
    let n = 24.0;
    let half = PI / n;
    let part = n * RADIUS * RADIUS * half.sin() * half.cos() * HEIGHT;
    let removed = part - kept;
    // The facet's round is a prism of the profile's corner area between the two mitre planes
    // through the axis: the area times the facet's length at the area's centroid, which stands
    // r (10 - 3 pi) / (12 - 3 pi) in from each face, and a length 2 (a - u) tan(h) at depth u.
    let size = 2.0;
    let apothem = RADIUS * half.cos();
    let area = (1.0 - PI / 4.0) * size * size;
    let centroid = size * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI);
    let exact = area * 2.0 * (apothem - centroid) * half.tan();
    // The arc laid in 4 chords adds each chord's segment, standing at most the face's length.
    let alpha = PI / 2.0 / 4.0;
    let chord_area = 4.0 * size * size / 2.0 * (alpha - alpha.sin());
    let chorded = exact + chord_area * 2.0 * apothem * half.tan();
    println!("removed {removed}, exact {exact}, chorded at most {chorded}");
    assert!(
        removed >= exact && removed <= chorded,
        "the round removes its whole profile between its end planes: {removed} against {exact} \
         exact, {chorded} with the chords"
    );
}

#[test]
fn three_facets_rounded_with_stopped_ends_are_cut_flat_and_closed() {
    fills_flat_and_closed(&cylinder(), 3, false);
}

#[test]
fn one_facet_of_a_single_precision_cylinder_is_cut_flat_with_its_whole_profile() {
    fills_flat_and_closed(&single_precision_cylinder(), 1, true);
}

#[test]
fn runs_that_do_not_start_in_their_end_planes_are_left_to_the_fade() {
    // A stopped one-facet arc's ends are the radial planes through rim vertices 0 and 1. Runs
    // between bottom vertices 6 and 7, which stand in neither plane, have no stretch in a plane at
    // all: the stopped fill hands them back to the fade instead of reading a stretch before the
    // run's start.
    let mesh = cylinder();
    let gap = {
        let _on = turn_stopped_ends_on();
        band(&mesh, &[top_rim(0, 1)], feature_edges(&mesh), 2.0)
    };
    assert!(gap.built(), "the stopped arc builds: {:?}", gap.refusal);
    let handle = Rc::clone(gap.mesh.as_ref().expect("the build kept its mesh"));
    let left = GapLoop::new(
        1,
        false,
        vec![6, 30, 7],
        vec![0.0, 0.5, 1.0],
        0,
        false,
        None,
    );
    let right = GapLoop::new(
        2,
        false,
        vec![7, 31, 6],
        vec![1.0, 0.5, 0.0],
        0,
        false,
        None,
    );
    let fill = try_stopped_ends(&gap, &handle, &left, &right, BandProfile::Fillet, 4)
        .expect("the stopped fill reads the runs");
    assert!(
        fill.is_none(),
        "runs off both end planes are no stopped end"
    );
}

#[test]
fn an_arc_stopped_at_one_end_only_fills_closed() {
    // The 60 degree sector's arc from its corner with the radial edge to its second facet seam:
    // the corner end, where two creases leave, does not stop; the far end does.
    let mut outline = vec![(0.0, 0.0)];
    for i in 0..=8 {
        let angle = f64::from(i) * (PI / 3.0) / 8.0;
        outline.push((RADIUS * angle.cos(), RADIUS * angle.sin()));
    }
    let mesh = prism(&outline, 0);
    let n = outline.len() as i32;
    let gap = {
        let _on = turn_stopped_ends_on();
        band(
            &mesh,
            &[vec![n + 1, n + 2, n + 3]],
            feature_edges(&mesh),
            1.0,
        )
    };
    assert!(gap.built(), "the arc builds: {:?}", gap.refusal);
    let bounds = gap.chains[0].end_ray_bounds;
    assert!(
        bounds.0.is_none_or(|b| !b.stopped) && bounds.1.is_some_and(|b| b.stopped),
        "only the far end stops"
    );
    let filled =
        gap_fill::fill_with_budget(&gap, BandProfile::Fillet, 4, None).expect("the fill runs");
    assert!(filled.filled(), "the arc fills: {:?}", filled.refusal);
    let handle = Rc::clone(filled.mesh.as_ref().expect("the fill keeps its mesh"));
    let filled_mesh = handle.borrow();
    assert!(is_closed(&filled_mesh), "the filled arc is closed");
    assert!(
        keeps_edge(&filled_mesh, n + 3, n + 4) || keeps_edge(&filled_mesh, n + 4, n + 3),
        "the rim edge past the stopped end stays sharp"
    );
}
