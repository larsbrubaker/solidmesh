//! Stopped ends on a faceted prism's top rim, read with stopped ends turned on: a short arc's ends
//! stop on their mitre planes; an arc over half the circle keeps fading, because its own far points
//! stand past each end's plane and a clip there would cut its own band; a leaving edge that is not
//! a feature crease stops nothing; and a sector's radial edge and arc meeting at a corner, the
//! arc's far end stopped, read a room the band builds at.

use std::collections::HashSet;
use std::f64::consts::PI;
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{
    build_pick, BandGapBuildOptions,
};
use geometry_processing::bevel::level_line::band_gap_reading::read_room_of_pick;
use geometry_processing::bevel::level_line::seam_edges::FeatureEdgeTest;
use geometry_processing::bevel::level_line::{turn_stopped_ends_on, BandGapResult, BandProfile};
use vector_math::Vector3;

type Mesh = (Vec<Vector3>, Vec<(i32, i32, i32)>);

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

/// Every edge whose two triangles fold more than [`FEATURE_DEGREES`].
fn feature_edges((positions, indices): &Mesh) -> FeatureEdgeTest {
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

fn band(mesh: &Mesh, chains: &[Vec<i32>], feature: FeatureEdgeTest, size: f64) -> BandGapResult {
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
