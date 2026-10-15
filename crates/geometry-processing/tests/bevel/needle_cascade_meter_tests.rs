//! WHAT the Many Holes witness's needle cascade is made of - how thin the input triangles are
//! against the floor the build repairs at, and, over one metered build, how many repairs each
//! per-split repair call makes and whether the needles it repairs came in with the part, from a
//! split, or from an earlier repair. A measurement, not a contract.
//!
//! Gated on `LEVELLINE_COST=1` beside `pass_one_pinch_tests`, whose fixture and build this shares
//! (`level_line_cost_gate`): the build is minutes long. The needle meter is process-wide, so a
//! `threads-required` override in `.config/nextest.toml` keeps the rows apart.

use std::collections::{HashMap, HashSet};
use std::rc::Rc;
use std::time::Instant;

use geometry_processing::bevel::level_line::band_gap;
use geometry_processing::bevel::level_line::band_gap_entry_points::{
    build_pick, BandGapBuildOptions,
};
use geometry_processing::bevel::level_line::{FieldCounters, NeedleMeter, PickThatFits};
use vector_math::Vector3;

use crate::level_line_cost_gate::{self as gate, say};
use crate::level_line_fixture::LevelLineFixture;

/// The cap for this file: the cascade's build is long, so it gets more room than the default.
const DEFAULT_CAP: i64 = 30_000_000;

/// The floor the witness's build repairs needles at - the build's own expression, not a copy.
fn floor_of(fixture: &LevelLineFixture) -> f64 {
    band_gap::needle_floor_of(1e-9, band_gap::chord_tolerance_of(fixture.size, 0.0))
}

/// A triangle's height over its longest edge, that edge's length, its two ends and the apex.
fn shape(p: &[Vector3], t: (i32, i32, i32)) -> (f64, f64, i32, i32, i32) {
    let corners = [t.0, t.1, t.2];
    let at = |v: i32| p[v as usize];
    let mut longest = -1.0;
    let mut k = 0;
    for i in 0..3 {
        let length = (at(corners[(i + 1) % 3]) - at(corners[i])).length();
        if length > longest {
            longest = length;
            k = i;
        }
    }

    let area2 = (at(t.1) - at(t.0)).cross(at(t.2) - at(t.0)).length();
    let height = if longest > 0.0 { area2 / longest } else { 0.0 };
    (
        height,
        longest,
        corners[k],
        corners[(k + 1) % 3],
        corners[(k + 2) % 3],
    )
}

fn off_the_line(a: Vector3, b: Vector3, point: Vector3) -> f64 {
    let along = b - a;
    if along.length_squared() <= 0.0 {
        return (point - a).length();
    }

    let s = (point - a).dot(along) / along.length_squared();
    (point - (a + (along * s))).length()
}

/// Distance from a point to the segment's interior, or infinity where it projects outside it.
fn inside_the_edge(a: Vector3, b: Vector3, point: Vector3) -> f64 {
    let along = b - a;
    if along.length_squared() <= 0.0 {
        return f64::INFINITY;
    }

    let s = (point - a).dot(along) / along.length_squared();
    if s <= 1e-6 || s >= 1.0 - 1e-6 {
        return f64::INFINITY;
    }

    (point - (a + (along * s))).length()
}

fn bands(values: impl Iterator<Item = f64>, edges: &[f64], names: &[&str]) -> String {
    let mut counts = vec![0i64; names.len()];
    for v in values {
        let mut i = 0;
        while i < edges.len() && v >= edges[i] {
            i += 1;
        }

        counts[i] += 1;
    }

    names
        .iter()
        .zip(&counts)
        .map(|(n, &c)| format!("{n}: {}", gate::n0(c)))
        .collect::<Vec<_>>()
        .join(", ")
}

fn aspect(s: &(f64, f64, i32, i32, i32)) -> f64 {
    if s.1 > 0.0 {
        s.0 / s.1
    } else {
        0.0
    }
}

/// The input alone, no build - every triangle's height against
/// the floor and its aspect, the ten thinnest, and how many triangles have a corner standing
/// inside a NEIGHBOUR's edge (a T-vertex in waiting).
#[test]
fn input_heights_against_the_floor() {
    if !gate::asked() {
        return;
    }

    let fixture = LevelLineFixture::load(&gate::witness_name());
    let p = &fixture.positions;
    let tris = &fixture.indices;
    let floor = floor_of(&fixture);
    let shapes: Vec<_> = tris.iter().map(|&t| shape(p, t)).collect();

    say(&format!(
        "input: {} triangles, {} vertices, size {}, needle floor {floor:.4e}",
        gate::n0(tris.len() as i64),
        gate::n0(p.len() as i64),
        fixture.size
    ));
    say(&format!(
        "  height/floor: {}",
        bands(
            shapes.iter().map(|s| s.0 / floor),
            &[1.0, 10.0, 100.0, 1000.0],
            &["< 1x", "1-10x", "10-100x", "100-1000x", "> 1000x"]
        )
    ));
    say(&format!(
        "  aspect (height/longest): {}",
        bands(
            shapes.iter().map(aspect),
            &[1e-9, 1e-7, 1e-5, 1e-3],
            &["< 1e-9", "1e-9..1e-7", "1e-7..1e-5", "1e-5..1e-3", "> 1e-3"]
        )
    ));

    say("  thinnest 10 by aspect:");
    let mut order: Vec<usize> = (0..tris.len()).collect();
    // A stable sort, so ties keep their order.
    order.sort_by(|&i, &j| aspect(&shapes[i]).total_cmp(&aspect(&shapes[j])));
    for &i in order.iter().take(10) {
        let s = shapes[i];
        let t = tris[i];
        let a = p[s.2 as usize];
        let b = p[s.3 as usize];
        let all_on = [t.0, t.1, t.2]
            .iter()
            .all(|&v| off_the_line(a, b, p[v as usize]) <= floor);
        say(&format!(
            "    triangle {i} [{},{},{}]: aspect {:.3e}, height {:.3e} ({:.4} floors), longest edge \
             {:.4e}, all corners within the floor of the long edge's line {all_on}",
            t.0,
            t.1,
            t.2,
            aspect(&s),
            s.0,
            s.0 / floor,
            s.1
        ));
    }

    // Edge -> triangles, so each triangle's edge neighbours can be read.
    let key = |x: i32, y: i32| if x < y { (x, y) } else { (y, x) };
    let mut on_edge: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
    for (i, t) in tris.iter().enumerate() {
        for (x, y) in [(t.0, t.1), (t.1, t.2), (t.2, t.0)] {
            on_edge.entry(key(x, y)).or_default().push(i);
        }
    }

    for reach in [1.0, 100.0] {
        let h = reach * floor;
        let mut triangles_with_one = 0i64;
        let mut apexes = HashSet::new();
        for (i, t) in tris.iter().enumerate() {
            let corners = [t.0, t.1, t.2];
            let mut found = false;
            for (x, y) in [(t.0, t.1), (t.1, t.2), (t.2, t.0)] {
                for &n in &on_edge[&key(x, y)] {
                    if n == i {
                        continue;
                    }

                    let nt = tris[n];
                    for (u, w) in [(nt.0, nt.1), (nt.1, nt.2), (nt.2, nt.0)] {
                        for &v in &corners {
                            if v != u
                                && v != w
                                && inside_the_edge(p[u as usize], p[w as usize], p[v as usize]) <= h
                            {
                                found = true;
                                apexes.insert(v);
                            }
                        }
                    }
                }
            }

            if found {
                triangles_with_one += 1;
            }
        }

        say(&format!(
            "  T-vertex-like within {reach}x floor ({h:.3e}): {} triangles have a corner strictly \
             inside an edge-neighbour's edge, {} distinct such corners",
            gate::n0(triangles_with_one),
            gate::n0(apexes.len() as i64)
        ));
    }
}

/// The build `pass_one_pinch_tests::pass1_says_which_sides_fail` runs,
/// with the needle meter on - repairs per per-split call, what made each repaired needle, how thin
/// it was, and the first repairs.
#[test]
fn the_cascade_is_metered() {
    if !gate::asked() {
        return;
    }

    let fixture = Rc::new(LevelLineFixture::load(&gate::witness_name()));
    let cap = gate::cap_or(DEFAULT_CAP);
    let settled = PickThatFits::read(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        &fixture.closed,
        gate::feature_test(&fixture),
        gate::profile(&fixture),
        fixture.size,
        1e-9,
        Some(gate::budget_of(cap)),
        0.0,
        f64::INFINITY,
    )
    .expect("the settling");

    let survivors: Vec<Vec<i32>> = settled
        .chains
        .iter()
        .map(|&k| fixture.chains[k as usize].clone())
        .collect();
    let closed: Vec<bool> = settled
        .chains
        .iter()
        .map(|&k| fixture.closed[k as usize])
        .collect();
    let budget = gate::budget_of(cap);
    let started = Instant::now();
    say(&format!(
        "cascade: {} survivors of {} at size {}, floor {:.4e}",
        survivors.len(),
        fixture.chains.len(),
        fixture.size,
        floor_of(&fixture)
    ));

    FieldCounters::reset();
    FieldCounters::set_meter_on(true);
    NeedleMeter::begin(fixture.indices.len() as i32, fixture.size);
    let options = BandGapBuildOptions {
        chord_tolerance: 0.0,
        snap: 1e-9,
        known_room: None,
        budget: Some(Rc::clone(&budget)),
        ..BandGapBuildOptions::default()
    };
    let verdict = match build_pick(
        &fixture.positions,
        &fixture.indices,
        &survivors,
        &closed,
        gate::feature_test(&fixture),
        fixture.size,
        gate::profile(&fixture),
        &options,
    ) {
        Ok(built) => {
            let mesh = built.mesh.as_ref().map(|m| m.borrow());
            let of = |read: &dyn Fn(
                &geometry_processing::bevel::level_line::LevelLineMesh,
            ) -> String| { mesh.as_ref().map_or(String::new(), |m| read(m)) };
            format!(
                "{:?}, needles repaired {}, left alone {}, on a chain edge {}, hit its bound {}, \
                 triangles made {}, vertices {}",
                built.refusal,
                of(&|m| m.needles_repaired.to_string()),
                of(&|m| m.needles_left_alone().to_string()),
                of(&|m| m.needles_left_on_a_chain_edge.to_string()),
                of(&|m| m.repair_hit_its_bound.to_string()),
                of(&|m| m.triangle_count().to_string()),
                of(&|m| m.vertex_count().to_string())
            )
        }
        Err(badly) => format!("stopped by {badly:?}"),
    };
    FieldCounters::set_meter_on(false);

    say(&format!(
        "cascade: {verdict}; {:.1} s, spent {} of {}, exhausted in {:?}",
        started.elapsed().as_secs_f64(),
        gate::n0(budget.spent.get()),
        gate::n0(budget.limit),
        budget.exhausted_in.get()
    ));
    say(&NeedleMeter::report());
    NeedleMeter::end();
}
