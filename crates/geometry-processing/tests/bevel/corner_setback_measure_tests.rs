//! The corner setbacks, measured before they were built - at every corner hole the filler closes,
//! each strip end's distance from the corner vertex (in the size r), its last row's arc radius
//! against r, and what stopped it - each arc end's distance to its own chain and to the
//! NEIGHBOURING chains at the corner. Beside it the IDEAL setback for straight chains and flat
//! faces: in each face the chain shares with a neighbouring picked chain, the point where the two
//! rails cross, s = (d_j + d_k cos theta) / sin theta, d = r tan(phi / 2) each chain's rail offset
//! (phi its faces' turn) and theta the angle between the chains in that face - a cube corner gives
//! 1 r.
//!
//! `where_each_strip_end_stops` is gated on `BEVEL_SETBACKS=1` (`BEVEL_SETBACKS_OUT` appends to a
//! file) and prints only: `BEVEL_SETBACKS=1 cargo nextest run --workspace --test bevel --profile
//! measure --no-capture corner_setback_measure_tests`. `every_corner_strip_end_is_a_flat_row_of_radius_r`
//! always runs. It runs alone (a `threads-required` override in `.config/nextest.toml`): the
//! corner-hole watch is process-wide, so its holes are also told apart by the thread that reads
//! them. Each test's cases run as a loop inside it.

use std::rc::Rc;
use std::sync::{Arc, Mutex};
use std::thread::ThreadId;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::corner_cap_hole_fill::WATCH_CORNER_HOLES;
use geometry_processing::bevel::level_line::corner_cap_hole_fill_switch::set_use_hole_fill_override;
use geometry_processing::bevel::level_line::corner_rail_strip::WATCH_SETBACKS;
use geometry_processing::bevel::level_line::seam_edges::FeatureEdgeTest;
use geometry_processing::bevel::level_line::{
    band_gap_reading, BandGapResult, BandProfile, CornerCapLastRow, LevelLineMesh,
};
use vector_math::Vector3;

use crate::end_face_builds::feature_of;
use crate::hole_fill_corner_tests::fill_with_the_filler;
use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_fixture::LevelLineFixture;
use crate::level_line_test_geometry::cube_solid;
use crate::several_chains_fixtures;
use crate::topology_test_meshes::create_rib_on_plate;

const RUN_VARIABLE: &str = "BEVEL_SETBACKS";

/// One strip end's last row as the watch reads it: its two arc ends, the arc's middle, the chain
/// point it is drawn about and whether it is concave.
#[derive(Clone, Copy)]
struct Row {
    p: Vector3,
    q: Vector3,
    mid: Vector3,
    chain_point: Vector3,
    concave: bool,
}

/// One corner hole: its loop, its corner and the strip ends on it.
type Hole = (Vec<Vector3>, Vector3, Vec<Row>);

/// The pick, its size and segments, and how it builds.
struct CornerFixture {
    positions: Vec<Vector3>,
    indices: Vec<(i32, i32, i32)>,
    chains: Vec<Vec<i32>>,
    closed: Vec<bool>,
    feature: FeatureEdgeTest,
    size: f64,
    segments: i32,
    options: BandGapBuildOptions,
}

impl CornerFixture {
    fn build(&self) -> BandGapResult {
        band_gap_entry_points::build_pick(
            &self.positions,
            &self.indices,
            &self.chains,
            &self.closed,
            Some(Rc::clone(&self.feature)),
            self.size,
            BandProfile::Fillet,
            &self.options,
        )
        .expect("a valid pick")
    }
}

fn fixture(name: &str, scale: f64) -> CornerFixture {
    if name == "cube-two-faces" || name == "rib-foot" {
        let (positions, indices, chains, segments) = if name == "cube-two-faces" {
            let (positions, indices) = cube_solid(1.0);
            (
                positions,
                indices,
                vec![vec![4, 5], vec![5, 6, 7, 4], vec![5, 1, 0, 4]],
                16,
            )
        } else {
            // the rib's foot of BevelMixedCornerTests / SkewRowCrossTangentTests, fillet r 1
            let source = create_rib_on_plate();
            let positions: Vec<Vector3> = source
                .vertices
                .iter()
                .map(|&v| Vector3::from_vector3_float(v))
                .collect();
            let indices: Vec<(i32, i32, i32)> =
                source.faces.iter().map(|f| (f.v0, f.v1, f.v2)).collect();
            let at = |x: f64, y: f64, z: f64| {
                positions
                    .iter()
                    .position(|&p| (p - Vector3::new(x, y, z)).length() < 1e-9)
                    .map_or(-1, |i| i as i32)
            };
            let chains = vec![
                vec![at(8.0, 5.0, 5.0), at(12.0, 5.0, 5.0)],
                vec![at(8.0, 5.0, 5.0), at(8.0, 20.0, 5.0)],
                vec![at(8.0, 5.0, 5.0), at(8.0, 5.0, 15.0)],
            ];
            (positions, indices, chains, 8)
        };

        let closed = vec![false; chains.len()];
        let feature =
            feature_edges_over(&positions, &indices, several_chains_fixtures::FOLD_DEGREES);
        return CornerFixture {
            positions,
            indices,
            chains,
            closed,
            feature,
            size: scale,
            segments,
            options: BandGapBuildOptions::default(),
        };
    }

    let loaded = Rc::new(LevelLineFixture::load(name));
    let feature = feature_of(&loaded);
    let size = if name == "outer-faces-r04" {
        scale
            * band_gap_reading::room_of_pick(
                &loaded.positions,
                &loaded.indices,
                &loaded.chains,
                &loaded.closed,
                Some(Rc::clone(&feature)),
                BandProfile::Fillet,
                1e-9,
            )
            .expect("the room reads")
    } else {
        scale * loaded.size
    };
    CornerFixture {
        positions: loaded.positions.clone(),
        indices: loaded.indices.clone(),
        chains: loaded.chains.clone(),
        closed: loaded.closed.clone(),
        feature,
        size,
        segments: loaded.segments,
        options: BandGapBuildOptions {
            chord_tolerance: 0.0,
            snap: 1e-9,
            known_room: if scale == 1.0 {
                loaded.known_room
            } else {
                None
            },
            budget: None,
            seam_epsilon: loaded.seam_epsilon,
            sharp_turn_degrees: loaded.sharp_turn_degrees,
            fillet_segments: loaded.segments,
        },
    }
}

/// Sets the process-wide corner-hole watch to collect this thread's holes, and clears it on drop.
struct CornerHoles {
    holes: Arc<Mutex<Vec<Hole>>>,
}

impl CornerHoles {
    fn watch() -> CornerHoles {
        let holes: Arc<Mutex<Vec<Hole>>> = Arc::new(Mutex::new(Vec::new()));
        let seen = Arc::clone(&holes);
        let thread: ThreadId = std::thread::current().id();
        *WATCH_CORNER_HOLES.lock().unwrap_or_else(|p| p.into_inner()) = Some(Arc::new(
            move |mesh: &LevelLineMesh,
                  l: &[Vector3],
                  rows: &[CornerCapLastRow],
                  corner: Vector3| {
                if std::thread::current().id() != thread {
                    return;
                }

                let rows = rows
                    .iter()
                    .map(|r| Row {
                        p: mesh.position(r.arc[0]),
                        q: mesh.position(r.arc[r.arc.len() - 1]),
                        mid: mesh.position(r.arc[r.arc.len() / 2]),
                        chain_point: r.chain_point,
                        concave: r.concave,
                    })
                    .collect();
                seen.lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push((l.to_vec(), corner, rows));
            },
        ));
        CornerHoles { holes }
    }

    fn taken(&self) -> Vec<Hole> {
        self.holes.lock().unwrap_or_else(|p| p.into_inner()).clone()
    }
}

impl Drop for CornerHoles {
    fn drop(&mut self) {
        *WATCH_CORNER_HOLES.lock().unwrap_or_else(|p| p.into_inner()) = None;
    }
}

/// Where each strip end stops (gated, prints only).
#[test]
fn where_each_strip_end_stops() {
    if std::env::var(RUN_VARIABLE).as_deref() != Ok("1") {
        return;
    }

    for (name, scale) in [
        ("inner-right-sides-and-center-op0", 1.0),
        ("inner-right-sides-and-center-op0", 0.25),
        ("outer-faces-r04", 0.5),
        ("meeting-edge-round-op0", 1.0),
        ("cube-two-faces", 0.5),
        ("rib-foot", 1.0),
        ("rib-foot", 0.5),
    ] {
        where_each_strip_end_stops_row(name, scale);
    }
}

fn where_each_strip_end_stops_row(name: &str, scale: f64) {
    let f = fixture(name, scale);
    let size = f.size;
    let (filled, holes) = {
        let watch = CornerHoles::watch();
        let gap = f.build();
        if !gap.built() {
            log(&format!("{name} at {size:.3}: band {:?}", gap.refusal));
            return;
        }

        let filled = fill_with_the_filler(&gap, BandProfile::Fillet, f.segments);
        (filled, watch.taken())
    };

    log(&format!(
        "{name} fillet r {size:.3}: fill {:?}, {} holes read",
        filled.refusal,
        holes.len()
    ));

    let ends = corner_vertices(&f.chains);
    let positions = &f.positions;
    for (h, (l, corner, rows)) in holes.iter().enumerate() {
        if rows.len() < 2 || ends.is_empty() {
            log(&format!(
                "  hole {h}: loop {}, {} strip end(s) - a free end, not read",
                l.len(),
                rows.len()
            ));
            continue;
        }

        let v = nearest_end(&ends, positions, *corner);
        let at = chains_at(&f.chains, v);
        let reach = l
            .iter()
            .map(|&p| (p - positions[v as usize]).length())
            .fold(f64::NEG_INFINITY, f64::max)
            / size;
        log(&format!(
            "  hole {h}: loop {}, {} strip ends, corner {}, loop reaches {reach:.3} r from it",
            l.len(),
            rows.len(),
            show(positions[v as usize])
        ));
        for row in rows {
            let own = own_chain(&at, positions, &f.chains, row.chain_point);
            let setback = (row.chain_point - positions[v as usize]).length();
            let radius = circumradius(row.p, row.mid, row.q);
            let neighbours: Vec<usize> = at.iter().copied().filter(|&k| k != own).collect();
            let rail = |p: Vector3| {
                let off: Vec<String> = neighbours
                    .iter()
                    .map(|&k| {
                        format!(
                            "c{k} {:.3}",
                            distance_to_chain(positions, &f.chains[k], p) / size
                        )
                    })
                    .collect();
                format!(
                    "{} off own chain {:.3}, along it from v {:.3}, off neighbours {}",
                    show(p),
                    distance_to_chain(positions, &f.chains[own], p) / size,
                    (p - positions[v as usize]).dot(direction(positions, &f.chains[own], v)) / size,
                    off.join(" / ")
                )
            };
            log(&format!(
                "    chain {own} {}: setback {:.3} r (arc radius {:.3} r, setback/arc {:.3}); ideal: {}",
                if row.concave { "concave" } else { "convex" },
                setback / size,
                radius / size,
                setback / radius,
                ideal(positions, &f.indices, &f.chains, &at, own, v, size)
            ));
            log(&format!("      rail P {}", rail(row.p)));
            log(&format!("      rail Q {}", rail(row.q)));
        }
    }
}

/// The setbacks' invariant - with the filler on,
/// every strip end at a corner is a flat cross-section square to its chain (its two rail ends at
/// one chain parameter), and a fillet's last row there is an arc of radius r, however far apart the
/// band cut stopped the two rails.
///
/// meeting-edge-round-op0 is held to the flat row only: its chains are round, and each rail stands
/// off its chain by the turn at the corner vertex while the faces at the row turn otherwise -
/// chain 0's rails 0.924 r off (r tan(phi / 2) for 85.5 degrees) where its faces turn 79.2, an arc
/// of 1.116 r; chain 1's 0.978 r. Both radii read the same before the setback as after it.
#[test]
fn every_corner_strip_end_is_a_flat_row_of_radius_r() {
    for (name, scale, radius_held) in [
        ("outer-faces-r04", 0.5, true),
        ("cube-two-faces", 0.5, true),
        ("rib-foot", 1.0, true),
        ("rib-foot", 0.5, true),
        ("meeting-edge-round-op0", 1.0, false),
        ("inner-right-sides-and-center-op0", 1.0, true),
        ("inner-right-sides-and-center-op0", 0.25, true),
        ("inner-right-sides-and-center-op0", 0.1, true),
        ("inner-right-sides-and-center-op0", 0.05, true),
        ("rib-foot", 0.25, true),
        ("cube-two-faces", 0.1, true),
    ] {
        every_corner_strip_end_is_a_flat_row_of_radius_r_row(name, scale, radius_held);
    }
}

fn every_corner_strip_end_is_a_flat_row_of_radius_r_row(name: &str, scale: f64, radius_held: bool) {
    let f = fixture(name, scale);
    let size = f.size;
    let counts = Rc::new(std::cell::Cell::new((0, 0)));
    let (gap, holes) = {
        let watch = CornerHoles::watch();
        set_use_hole_fill_override(Some(true));
        let seen = Rc::clone(&counts);
        WATCH_SETBACKS.with(|w| {
            *w.borrow_mut() = Some(Rc::new(move |trim: bool| {
                let (trimmed, short) = seen.get();
                seen.set(if trim {
                    (trimmed + 1, short)
                } else {
                    (trimmed, short + 1)
                });
            }))
        });

        let gap = f.build();
        if gap.built() {
            fill_with_the_filler(&gap, BandProfile::Fillet, f.segments);
        }

        // The watch and the override go off whatever the build did, so the next build is plain.
        WATCH_SETBACKS.with(|w| *w.borrow_mut() = None);
        set_use_hole_fill_override(None);
        (gap, watch.taken())
    };

    let (trimmed, short) = counts.get();
    println!(
        "{name} at {scale}: {trimmed} strip-end rails trimmed to their setback, {short} stopping short of it"
    );
    assert!(gap.built(), "{name} at {scale}: band {:?}", gap.refusal);

    let ends = corner_vertices(&f.chains);
    let positions = &f.positions;
    let mut read = 0;
    for (_, corner, rows) in holes.iter().filter(|h| h.2.len() >= 2) {
        let v = nearest_end(&ends, positions, *corner);
        let at = chains_at(&f.chains, v);
        for row in rows {
            let own = own_chain(&at, positions, &f.chains, row.chain_point);
            // square to the chain WHERE the row crosses it: a round chain turns between v and the
            // row
            let along = tangent_near(positions, &f.chains[own], row.chain_point);
            let skew = (row.p - row.q).dot(along).abs() / size;
            let radius = circumradius(row.p, row.mid, row.q) / size;
            assert!(
                skew < 1e-3,
                "{name} at {scale} chain {own} at {}: rails {} and {} stop {skew:.4} r apart along it",
                show(positions[v as usize]),
                show(row.p),
                show(row.q)
            );
            read += 1;
            if radius_held {
                assert!(
                    (radius - 1.0).abs() < 0.01,
                    "{name} at {scale} chain {own} at {}: last row's arc radius {radius:.4} r",
                    show(positions[v as usize])
                );
            }
        }
    }

    assert!(read > 0, "{name} at {scale}");
}

/// Corner vertices: chain ends shared by two or more chains, in first-seen order.
fn corner_vertices(chains: &[Vec<i32>]) -> Vec<i32> {
    let mut counts: indexmap::IndexMap<i32, i32> = indexmap::IndexMap::new();
    for c in chains {
        *counts.entry(c[0]).or_insert(0) += 1;
        *counts.entry(c[c.len() - 1]).or_insert(0) += 1;
    }

    counts
        .into_iter()
        .filter(|&(_, n)| n >= 2)
        .map(|(v, _)| v)
        .collect()
}

/// The end nearest `corner`; the first of any tie.
fn nearest_end(ends: &[i32], positions: &[Vector3], corner: Vector3) -> i32 {
    let mut best = ends[0];
    for &e in &ends[1..] {
        if (positions[e as usize] - corner).length() < (positions[best as usize] - corner).length()
        {
            best = e;
        }
    }

    best
}

fn chains_at(chains: &[Vec<i32>], v: i32) -> Vec<usize> {
    (0..chains.len())
        .filter(|&k| chains[k][0] == v || chains[k][chains[k].len() - 1] == v)
        .collect()
}

/// The chain nearest `p`; the first of any tie.
fn own_chain(at: &[usize], positions: &[Vector3], chains: &[Vec<i32>], p: Vector3) -> usize {
    let mut best = at[0];
    for &k in &at[1..] {
        if distance_to_chain(positions, &chains[k], p)
            < distance_to_chain(positions, &chains[best], p)
        {
            best = k;
        }
    }

    best
}

/// The straight-chain, flat-face ideal setback of chain `own` at `v` in r - per face of
/// own's first edge, the point along own where its rail crosses the rail of the neighbouring picked
/// chain lying in that face.
fn ideal(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    at: &[usize],
    own: usize,
    v: i32,
    r: f64,
) -> String {
    let e = direction(positions, &chains[own], v);
    let (offset, normals) = rail_offset(positions, indices, &chains[own], v, r);
    let Some(normals) = normals else {
        return "-".to_string();
    };

    let into = into(positions, indices, &chains[own], v);

    let mut parts = Vec::new();
    for (f, &normal) in normals.iter().enumerate() {
        let neighbour = at
            .iter()
            .copied()
            .filter(|&k| k != own)
            .find(|&k| direction(positions, &chains[k], v).dot(normal).abs() < 1e-3);
        let Some(neighbour) = neighbour else {
            parts.push("a face with no picked neighbour".to_string());
            continue;
        };

        let ej = direction(positions, &chains[neighbour], v);
        let theta = e.dot(ej).clamp(-1.0, 1.0).acos();
        let (d, _) = rail_offset(positions, indices, &chains[neighbour], v, r);

        // The face's side of own: when the neighbour leaves v on it, the face is the theta sector
        // and the two rails cross; otherwise the face wraps the reflex side of the pair and the
        // rails diverge, each ending on the perpendicular through v (setback 0, the gap between
        // them an arc about v).
        if ej.dot(into[f]) <= 0.0 {
            parts.push(format!(
                "with c{neighbour} (theta {:.1}, REFLEX side, rails diverge) 0 r",
                theta * 180.0 / std::f64::consts::PI
            ));
            continue;
        }

        let s = (d + (offset * theta.cos())) / theta.sin();
        parts.push(format!(
            "with c{neighbour} (theta {:.1}, its offset {:.3}) {:.3} r",
            theta * 180.0 / std::f64::consts::PI,
            d / r,
            s / r
        ));
    }

    format!(
        "turn {:.1}, rail offset {:.3} r; {}",
        turn_degrees(positions, indices, &chains[own], v),
        offset / r,
        parts.join(", ")
    )
}

/// The triangles on the chain's first edge at `v`, in order, and that edge's other end.
fn on_first_edge(
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    v: i32,
) -> (i32, Vec<(i32, i32, i32)>) {
    let next = if chain[0] == v {
        chain[1]
    } else {
        chain[chain.len() - 2]
    };
    let has = |t: &(i32, i32, i32), x: i32| t.0 == x || t.1 == x || t.2 == x;
    let on = indices
        .iter()
        .filter(|t| has(t, v) && has(t, next))
        .copied()
        .collect();
    (next, on)
}

/// Per triangle on the chain's first edge at `v` (`rail_offset`'s order), the in-face
/// unit direction from the chain into it.
fn into(positions: &[Vector3], indices: &[(i32, i32, i32)], chain: &[i32], v: i32) -> Vec<Vector3> {
    let (next, on) = on_first_edge(indices, chain, v);
    let e = (positions[next as usize] - positions[v as usize]).get_normal();
    on.iter()
        .map(|t| {
            let third = if t.0 != v && t.0 != next {
                t.0
            } else if t.1 != v && t.1 != next {
                t.1
            } else {
                t.2
            };
            let off = positions[third as usize] - positions[v as usize];
            (off - (e * off.dot(e))).get_normal()
        })
        .collect()
}

fn turn_degrees(positions: &[Vector3], indices: &[(i32, i32, i32)], chain: &[i32], v: i32) -> f64 {
    match rail_offset(positions, indices, chain, v, 1.0).1 {
        None => f64::NAN,
        Some(normals) => {
            normals[0].dot(normals[1]).clamp(-1.0, 1.0).acos() * 180.0 / std::f64::consts::PI
        }
    }
}

/// R tan(phi / 2) for the chain's first edge at `v`, phi the turn between its two
/// triangles' normals - and those normals, `None` where the edge does not stand on two triangles.
fn rail_offset(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    v: i32,
    r: f64,
) -> (f64, Option<Vec<Vector3>>) {
    let (_, on) = on_first_edge(indices, chain, v);
    if on.len() != 2 {
        return (f64::NAN, None);
    }

    let normals: Vec<Vector3> = on
        .iter()
        .map(|t| {
            let a = positions[t.0 as usize];
            (positions[t.1 as usize] - a)
                .cross(positions[t.2 as usize] - a)
                .get_normal()
        })
        .collect();
    let phi = normals[0].dot(normals[1]).clamp(-1.0, 1.0).acos();
    (r * (phi / 2.0).tan(), Some(normals))
}

fn direction(positions: &[Vector3], chain: &[i32], v: i32) -> Vector3 {
    let next = if chain[0] == v {
        chain[1]
    } else {
        chain[chain.len() - 2]
    };
    (positions[next as usize] - positions[v as usize]).get_normal()
}

/// The unit direction of the chain's segment nearest `p` (the first of equals).
fn tangent_near(positions: &[Vector3], chain: &[i32], p: Vector3) -> Vector3 {
    let mut best: Option<(f64, Vector3)> = None;
    for i in 0..chain.len() - 1 {
        let a = positions[chain[i] as usize];
        let b = positions[chain[i + 1] as usize];
        let d = segment_distance(a, b, p);
        if best.is_none_or(|(least, _)| d < least) {
            best = Some((d, (b - a).get_normal()));
        }
    }

    best.expect("a chain of two points or more").1
}

fn segment_distance(a: Vector3, b: Vector3, p: Vector3) -> f64 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0);
    (p - (a + (ab * t))).length()
}

fn distance_to_chain(positions: &[Vector3], chain: &[i32], p: Vector3) -> f64 {
    let mut best = f64::MAX;
    for i in 0..chain.len().saturating_sub(1) {
        let a = positions[chain[i] as usize];
        let b = positions[chain[i + 1] as usize];
        best = best.min(segment_distance(a, b, p));
    }

    best
}

fn circumradius(a: Vector3, b: Vector3, c: Vector3) -> f64 {
    let ab = b - a;
    let ac = c - a;
    let normal = ab.cross(ac);
    if normal.length_squared() < 1e-24 {
        return ac.length();
    }

    let centre = a
        + (((normal.cross(ab) * ac.length_squared()) + (ac.cross(normal) * ab.length_squared()))
            / (2.0 * normal.length_squared()));
    (a - centre).length()
}

fn show(p: Vector3) -> String {
    format!("({:.3},{:.3},{:.3})", p.x, p.y, p.z)
}

/// To the console, and appended to `BEVEL_SETBACKS_OUT` when set.
fn log(line: &str) {
    use std::io::Write as _;

    println!("{line}");
    if let Ok(output) = std::env::var(format!("{RUN_VARIABLE}_OUT")) {
        if !output.is_empty() {
            std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(&output)
                .and_then(|mut file| writeln!(file, "{line}"))
                .expect("the log appends");
        }
    }
}
