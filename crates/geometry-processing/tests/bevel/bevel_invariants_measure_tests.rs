//! Ports `Tests/GeometryProcessingTests/LevelLine/BevelInvariantsMeasureTests.cs`: the two
//! invariants the hole filler checks, MEASURED on every dumped kernel fixture without changing what
//! the build does. Gated on `BEVEL_INVARIANTS=1`; prints a table and, when `BEVEL_INVARIANTS_OUT`
//! names a file, writes it there:
//! `BEVEL_INVARIANTS=1 cargo nextest run --workspace --test bevel --profile measure --no-capture
//! bevel_invariants_measure_tests`. C# `[NotInParallel]` is a `threads-required` override in
//! `.config/nextest.toml`.
//!
//! Invariant 1: the deleted band around one connected pick has Euler characteristic V - E of the
//! pick's edge graph, and (reading the band as planar, genus 0) 2 - (V - E) boundary loops - read
//! off the band's removed triangles per connected band component, matched to the pick component
//! whose chain vertices it holds. Invariant 2: every hole the corner caps face is a disk - one
//! boundary loop, every vertex on it once - read off the directed edges the corner fill leaves open;
//! end holes are read too (the zip's row and run, and the fade, dying-crease and hull and strip clip
//! cap holes). The fixture folder's control mesh is left out (divergence 5015).

use std::cell::RefCell;
use std::collections::HashSet;
use std::rc::Rc;
use std::sync::mpsc;
use std::time::Duration;

use indexmap::IndexMap;

use geometry_processing::bevel::level_line::band_gap_entry_points;
use geometry_processing::bevel::level_line::corner_fill_fill::WATCH_OPEN_HOLES;
use geometry_processing::bevel::level_line::{
    band_gap_in_turn, corner_cap_zip_end, dying_crease_fill, gap_fill, hull_clip, open_gap_fill,
    strip_clip, BandGapResult, GapFillRefusal,
};

use crate::end_face_builds::{as_saved, feature_of};
use crate::fillet_thin_front_fin::profile_of;
use crate::level_line_fixture::{self, LevelLineFixture};

const RUN_VARIABLE: &str = "BEVEL_INVARIANTS";

const PER_FIXTURE: Duration = Duration::from_secs(90);

/// C# `MeasureTheInvariantsOnEveryKernelFixture`.
#[test]
fn measure_the_invariants_on_every_kernel_fixture() {
    if std::env::var(RUN_VARIABLE).as_deref() != Ok("1") {
        println!("bevel invariants: set {RUN_VARIABLE}=1 to run it");
        return;
    }

    let mut table = String::from("fixture | band | fill | inv1 (per pick component: chi band/pick, loops band/expected) | inv2 (corner holes) | end zips | other end fills (fade, dying crease, hull and strip clip caps)\n");
    for name in level_line_fixture::fixture_names() {
        // C#'s `Task.WhenAny(work, Task.Delay(PerFixture))`: a run past its time box is reported
        // and left to finish on its own thread.
        let (sender, receiver) = mpsc::channel();
        let fixture_name = name.clone();
        std::thread::spawn(move || {
            let _ = sender.send(measure(LevelLineFixture::load(&fixture_name)));
        });
        let line = receiver
            .recv_timeout(PER_FIXTURE)
            .unwrap_or_else(|_| format!("timed out after {} s", PER_FIXTURE.as_secs()));
        table.push_str(&format!("{name} | {line}\n"));
        println!("{name} | {line}");
    }

    if let Ok(output) = std::env::var(format!("{RUN_VARIABLE}_OUT")) {
        if !output.is_empty() {
            std::fs::write(&output, &table).expect("the table writes");
        }
    }
}

/// C# `Measure`.
fn measure(fixture: LevelLineFixture) -> String {
    let fixture = Rc::new(fixture);
    let gap = match band_gap_entry_points::build_pick(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        &fixture.closed,
        Some(feature_of(&fixture)),
        fixture.size,
        profile_of(&fixture),
        &as_saved(&fixture, fixture.known_room),
    ) {
        Ok(gap) => gap,
        Err(e) => return format!("threw {e:?} | - | - | -"),
    };

    if !gap.built() {
        return format!("{:?} (room {:.3}) | - | not cut | -", gap.refusal, gap.room);
    }

    let invariant1 = invariant_one(&fixture, &gap);

    type Holes = Rc<RefCell<Vec<Vec<(i32, i32)>>>>;
    let holes: Holes = Rc::new(RefCell::new(Vec::new()));
    let ends: Rc<RefCell<Vec<String>>> = Rc::new(RefCell::new(Vec::new()));
    let other_ends: Rc<RefCell<Vec<String>>> = Rc::new(RefCell::new(Vec::new()));
    {
        let holes = Rc::clone(&holes);
        WATCH_OPEN_HOLES.with(|w| {
            *w.borrow_mut() = Some(Rc::new(move |open: &HashSet<(i32, i32)>| {
                holes.borrow_mut().push(open.iter().copied().collect())
            }))
        });
        let ends = Rc::clone(&ends);
        corner_cap_zip_end::WATCH_END_HOLE.with(|w| {
            *w.borrow_mut() = Some(Rc::new(move |row: &[i32], run: &[i32]| {
                ends.borrow_mut().push(end_hole(row, run))
            }))
        });
    }
    let other = |kind: &'static str| {
        let other_ends = Rc::clone(&other_ends);
        Rc::new(move |open: &[(i32, i32)]| {
            other_ends
                .borrow_mut()
                .push(format!("{kind} {}", invariant_two(open)))
        })
    };
    open_gap_fill::WATCH_FADE_HOLE.with(|w| *w.borrow_mut() = Some(other("fade")));
    dying_crease_fill::WATCH_DYING_HOLE.with(|w| *w.borrow_mut() = Some(other("dying")));
    hull_clip::WATCH_CAP_HOLE.with(|w| *w.borrow_mut() = Some(other("hull cap")));
    strip_clip::WATCH_CAP_HOLE.with(|w| *w.borrow_mut() = Some(other("strip cap")));

    let filled = if band_gap_in_turn::takes_open_chains_in_turn(Some(&gap), &fixture.closed) {
        band_gap_in_turn::build_open_chains_in_turn(
            &fixture.positions,
            &fixture.indices,
            &fixture.chains,
            Some(feature_of(&fixture)),
            fixture.size,
            profile_of(&fixture),
            fixture.segments,
            Some(gap.room),
            1e-9,
            fixture.seam_epsilon,
            fixture.sharp_turn_degrees,
            None,
            None,
        )
        .map_err(|e| format!("{e:?}"))
    } else {
        gap_fill::fill(&gap, profile_of(&fixture), fixture.segments).map_err(|e| format!("{e:?}"))
    };
    let fill = match filled {
        Ok(filled) if filled.refusal == GapFillRefusal::None => "builds".to_string(),
        Ok(filled) => format!("{:?}", filled.refusal),
        Err(e) => format!("threw {e}"),
    };

    // C#'s `finally`.
    WATCH_OPEN_HOLES.with(|w| *w.borrow_mut() = None);
    corner_cap_zip_end::WATCH_END_HOLE.with(|w| *w.borrow_mut() = None);
    open_gap_fill::WATCH_FADE_HOLE.with(|w| *w.borrow_mut() = None);
    dying_crease_fill::WATCH_DYING_HOLE.with(|w| *w.borrow_mut() = None);
    hull_clip::WATCH_CAP_HOLE.with(|w| *w.borrow_mut() = None);
    strip_clip::WATCH_CAP_HOLE.with(|w| *w.borrow_mut() = None);

    let holes = holes.borrow();
    let invariant2 = if holes.is_empty() {
        "no corner fill".to_string()
    } else {
        holes
            .iter()
            .map(|h| invariant_two(h))
            .collect::<Vec<_>>()
            .join("; ")
    };
    let ends = ends.borrow();
    let end_holes = if ends.is_empty() {
        "no end zip".to_string()
    } else {
        // C#'s `GroupBy(e => e)`: groups in first-seen order.
        let mut grouped: IndexMap<&str, i32> = IndexMap::new();
        for e in ends.iter() {
            *grouped.entry(e.as_str()).or_insert(0) += 1;
        }

        format!(
            "{} end holes [{}]",
            ends.len(),
            grouped
                .iter()
                .map(|(key, count)| format!("{count}x {key}"))
                .collect::<Vec<_>>()
                .join(", ")
        )
    };
    let other_ends = other_ends.borrow();
    let other_end_holes = if other_ends.is_empty() {
        "no other end fill".to_string()
    } else {
        other_ends.join("; ")
    };
    format!("cut | {fill} | {invariant1} | {invariant2} | {end_holes} | {other_end_holes}")
}

/// C# `EndHole`: invariant 2 for one end hole - the row and the run both run from S to T, so the
/// hole is a disk when they share exactly those two ends and neither repeats a vertex.
fn end_hole(row: &[i32], run: &[i32]) -> String {
    if row.len() < 2
        || run.len() < 2
        || row[0] != run[0]
        || row[row.len() - 1] != run[run.len() - 1]
    {
        return "NOT disk(ends differ)".to_string();
    }

    let inner: Vec<i32> = row[1..row.len() - 1]
        .iter()
        .chain(run[1..run.len() - 1].iter())
        .copied()
        .collect();
    let mut boundary = inner.clone();
    boundary.push(row[0]);
    boundary.push(row[row.len() - 1]);
    let distinct: HashSet<i32> = boundary.iter().copied().collect();
    if row[0] == row[row.len() - 1] || distinct.len() != boundary.len() {
        return "NOT disk(repeats a vertex)".to_string();
    }

    if inner.is_empty() {
        "NOT disk(no area)".to_string()
    } else {
        "disk".to_string()
    }
}

/// C# `InvariantOne`: invariant 1 per connected pick component, read off the deleted band.
fn invariant_one(fixture: &LevelLineFixture, gap: &BandGapResult) -> String {
    // The pick's edge graph and its connected components.
    let mut pick_edges: HashSet<(i32, i32)> = HashSet::new();
    let mut pick_parent: IndexMap<i32, i32> = IndexMap::new();
    for (k, chain) in fixture.chains.iter().enumerate() {
        let count = if fixture.closed[k] {
            chain.len()
        } else {
            chain.len() - 1
        };
        for &v in chain {
            pick_parent.entry(v).or_insert(v);
        }

        for i in 0..count {
            let a = chain[i];
            let b = chain[(i + 1) % chain.len()];
            pick_edges.insert(if a < b { (a, b) } else { (b, a) });
            union(&mut pick_parent, a, b);
        }
    }

    // The deleted band's triangles, joined through shared edges.
    let mesh = gap.mesh.as_ref().expect("a built band").borrow();
    let mut band_parent: IndexMap<i32, i32> = IndexMap::new();
    let mut edge_use: IndexMap<(i32, i32), i32> = IndexMap::new();
    let mut directed: Vec<(i32, i32)> = Vec::new();
    for &t in &gap.removed_triangles {
        let face = mesh.triangles()[t as usize];
        let corners = [face.a, face.b, face.c];
        for c in 0..3 {
            let a = corners[c];
            let b = corners[(c + 1) % 3];
            band_parent.entry(a).or_insert(a);
            band_parent.entry(b).or_insert(b);
            union(&mut band_parent, a, b);
            let key = if a < b { (a, b) } else { (b, a) };
            *edge_use.entry(key).or_insert(0) += 1;
            directed.push((a, b));
        }
    }

    // C#'s `GroupBy(v => Find(pickParent, v))`: components in first-seen order.
    let keys: Vec<i32> = pick_parent.keys().copied().collect();
    let mut components: IndexMap<i32, Vec<i32>> = IndexMap::new();
    for v in keys {
        let root = find(&mut pick_parent, v);
        components.entry(root).or_default().push(v);
    }

    let mut parts = Vec::new();
    for component in components.values() {
        let vertices: HashSet<i32> = component.iter().copied().collect();
        let pick_v = vertices.len() as i32;
        let pick_e = pick_edges
            .iter()
            .filter(|e| vertices.contains(&e.0))
            .count() as i32;
        let pick_chi = pick_v - pick_e;
        let expected_loops = 2 - pick_chi;

        let mut band_roots: Vec<i32> = Vec::new();
        for &v in component {
            if band_parent.contains_key(&v) {
                let root = find(&mut band_parent, v);
                if !band_roots.contains(&root) {
                    band_roots.push(root);
                }
            }
        }

        if band_roots.len() != 1 {
            parts.push(format!(
                "pick V{pick_v} E{pick_e}: band components {} BREAK",
                band_roots.len()
            ));
            continue;
        }

        let root = band_roots[0];
        let all: Vec<i32> = band_parent.keys().copied().collect();
        let band_vertices: HashSet<i32> = all
            .into_iter()
            .filter(|&v| find(&mut band_parent, v) == root)
            .collect();
        let band_edges: Vec<(i32, i32)> = edge_use
            .keys()
            .filter(|e| band_vertices.contains(&e.0))
            .copied()
            .collect();
        let band_faces = directed
            .iter()
            .filter(|d| band_vertices.contains(&d.0))
            .count() as i32
            / 3;
        let chi = band_vertices.len() as i32 - band_edges.len() as i32 + band_faces;
        let boundary: Vec<(i32, i32)> = directed
            .iter()
            .filter(|d| {
                band_vertices.contains(&d.0)
                    && edge_use[&if d.0 < d.1 { (d.0, d.1) } else { (d.1, d.0) }] == 1
            })
            .copied()
            .collect();
        let non_manifold = band_edges.iter().filter(|e| edge_use[*e] > 2).count();
        let (loops, simple) = loops_of(&boundary);
        let holds = chi == pick_chi && loops == expected_loops && simple && non_manifold == 0;
        parts.push(format!(
            "chi {chi}/{pick_chi}, loops {loops}/{expected_loops}{}{} {}",
            if simple { "" } else { " pinched" },
            if non_manifold > 0 {
                format!(" nm{non_manifold}")
            } else {
                String::new()
            },
            if holds { "holds" } else { "BREAK" }
        ));
    }

    parts.join("; ")
}

/// C# `InvariantTwo`: invariant 2 for one call's open edges - each connected hole one simple loop.
fn invariant_two(open: &[(i32, i32)]) -> String {
    let mut parent: IndexMap<i32, i32> = IndexMap::new();
    for &(a, b) in open {
        parent.entry(a).or_insert(a);
        parent.entry(b).or_insert(b);
        union(&mut parent, a, b);
    }

    let mut groups: IndexMap<i32, Vec<(i32, i32)>> = IndexMap::new();
    for &e in open {
        let root = find(&mut parent, e.0);
        groups.entry(root).or_default().push(e);
    }

    let holes: Vec<String> = groups
        .values()
        .map(|g| {
            let (loops, simple) = loops_of(g);
            if loops == 1 && simple {
                format!("disk({})", g.len())
            } else {
                format!(
                    "NOT disk({} edges, {loops} loops{})",
                    g.len(),
                    if simple { "" } else { ", pinched" }
                )
            }
        })
        .collect();
    format!("{} holes [{}]", holes.len(), holes.join(", "))
}

/// C# `Loops`: the cycles a set of directed edges falls into, and whether every vertex on them has
/// exactly one edge in and one out (a simple loop set); a vertex with more is a pinch, walked by
/// taking any unused edge.
fn loops_of(edges: &[(i32, i32)]) -> (i32, bool) {
    let mut outs: IndexMap<i32, Vec<i32>> = IndexMap::new();
    let mut ins: IndexMap<i32, i32> = IndexMap::new();
    for &(a, b) in edges {
        outs.entry(a).or_default().push(b);
        *ins.entry(b).or_insert(0) += 1;
    }

    let mut simple = outs.values().all(|o| o.len() == 1)
        && ins.values().all(|&i| i == 1)
        && outs.len() == ins.len();
    let mut loops = 0;
    while let Some(start) = outs.iter().find(|(_, o)| !o.is_empty()).map(|(&k, _)| k) {
        let mut at = start;
        let mut steps = 0;
        loop {
            let Some(next) = outs.get_mut(&at).filter(|n| !n.is_empty()) else {
                // A dead end: the edges do not close.
                simple = false;
                break;
            };

            at = next.remove(0);
            steps += 1;
            if at == start || steps > edges.len() {
                break;
            }
        }

        loops += 1;
    }

    (loops, simple)
}

fn find(parent: &mut IndexMap<i32, i32>, mut v: i32) -> i32 {
    while parent[&v] != v {
        let grand = parent[&parent[&v]];
        parent.insert(v, grand);
        v = grand;
    }

    v
}

fn union(parent: &mut IndexMap<i32, i32>, a: i32, b: i32) {
    let ra = find(parent, a);
    let rb = find(parent, b);
    if ra != rb {
        parent.insert(ra, rb);
    }
}
