//! WHERE the joint cut BRIDGES two chains' level lines, and what the mesh and the field look like
//! at each bridge. `pass_one_pinch_tests` says the witness pick leaves mixed loops; this walks
//! those loops and prints, at every owner change, the mesh edge the boundary stands on, the
//! triangles either side, every corner's reading, and what the field truly does over the band
//! triangle removed there.
//!
//! A measurement, not a contract - the assertions only keep the run honest. The witness walk is
//! gated on `LEVELLINE_COST=1` (`level_line_cost_gate`), because a pass of the pick is tens of
//! seconds; the control count is not.

use std::collections::HashMap;
use std::rc::Rc;
use std::time::Instant;

use geometry_processing::bevel::level_line::band_gap_entry_points::{
    build_pick, BandGapBuildOptions,
};
use geometry_processing::bevel::level_line::{
    ChainField, FieldCounters, GapLoop, LevelLineMesh, PickField, PickThatFits, SplitMeter,
};
use indexmap::IndexMap;
use vector_math::Vector3;

use crate::grid_hole_plate_fixture;
use crate::level_line_cost_gate::{self as gate, say};
use crate::level_line_fixture::LevelLineFixture;

/// The chain pairs whose bridges are wanted, as survivor indices - "3,8;4,5" by default, the two
/// pairs `pass_one_pinch_tests` named on the witness. Overridden by `LEVELLINE_BRIDGE_PAIRS`.
fn wanted_pairs() -> Vec<(i32, i32)> {
    let asked = std::env::var("LEVELLINE_BRIDGE_PAIRS").unwrap_or_default();
    let text = if asked.is_empty() {
        "3,8;4,5".to_string()
    } else {
        asked
    };
    text.split(';')
        .filter(|p| !p.is_empty())
        .map(|p| {
            let n: Vec<i32> = p
                .split(',')
                .map(|s| s.parse().expect("a chain index"))
                .collect();
            (n[0], n[1])
        })
        .collect()
}

fn point(p: Vector3) -> String {
    format!("({:.4}, {:.4}, {:.4})", p.x, p.y, p.z)
}

/// Every triangle of the INPUT mesh whose three corners lie on chains of the pick, how many of
/// those span TWO chains, and the cross-chain edge lengths (sorted) - which tell a sliver whose
/// middle stands far from both chains from two chains that really do run together.
fn slivers_across_the_gap(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
) -> (i32, i32, Vec<f64>) {
    let mut chain_of = HashMap::new();
    for (k, chain) in chains.iter().enumerate() {
        for &v in chain {
            // A vertex shared by two chains would be a chain junction; the first reading names it.
            chain_of.entry(v).or_insert(k);
        }
    }

    let mut all = 0;
    let mut spanning = 0;
    let mut cross_edges = Vec::new();
    for &(a, b, c) in indices {
        let (Some(&ka), Some(&kb), Some(&kc)) =
            (chain_of.get(&a), chain_of.get(&b), chain_of.get(&c))
        else {
            continue;
        };

        all += 1;
        if ka == kb && kb == kc {
            continue;
        }

        spanning += 1;
        for (u, v, ku, kv) in [(a, b, ka, kb), (b, c, kb, kc), (c, a, kc, ka)] {
            if ku != kv {
                cross_edges.push((positions[u as usize] - positions[v as usize]).length());
            }
        }
    }

    cross_edges.sort_by(f64::total_cmp);
    (all, spanning, cross_edges)
}

fn spread(lengths: &[f64], size: f64) -> String {
    if lengths.is_empty() {
        return "none".to_string();
    }

    format!(
        "{} cross-chain edges, least {:.3} sizes, middling {:.3}, greatest {:.3}",
        lengths.len(),
        lengths[0] / size,
        lengths[lengths.len() / 2] / size,
        lengths[lengths.len() - 1] / size
    )
}

/// The cheap control: how many input triangles have all three corners on picked chains, and how
/// many of those span two DIFFERENT chains - for the grid plate at 5 loops and for the witness. No
/// build, so it is not gated.
#[test]
fn triangles_spanning_two_chains_are_counted() {
    let (positions, indices, chains) = grid_hole_plate_fixture::pick(2);
    let grid = slivers_across_the_gap(&positions, &indices, &chains);
    say(&format!(
        "control: grid plate at {} loops, {} triangles - {} with all three corners on chains, {} \
         of them spanning TWO chains; {}",
        chains.len(),
        indices.len(),
        grid.0,
        grid.1,
        spread(&grid.2, grid_hole_plate_fixture::SIZE)
    ));

    let name = gate::witness_name();
    let fixture = LevelLineFixture::load(&name);
    let witness = slivers_across_the_gap(&fixture.positions, &fixture.indices, &fixture.chains);
    say(&format!(
        "control: witness {name}, {} triangles, {} chains - {} with all three corners on chains, \
         {} of them spanning TWO chains; {}",
        fixture.indices.len(),
        fixture.chains.len(),
        witness.0,
        witness.1,
        spread(&witness.2, fixture.size)
    ));

    // Nothing is claimed about the counts; the run only has to reach them.
    assert!(grid.1 >= 0);
    assert!(witness.1 >= 0);
}

/// Walks the witness's mixed loops for the wanted chain pairs and prints, at every owner change,
/// what the mesh and the field are doing there.
#[test]
fn bridges_are_named_on_the_witness() {
    if !gate::asked() {
        return;
    }

    let fixture = Rc::new(LevelLineFixture::load(&gate::witness_name()));
    let profile = gate::profile(&fixture);
    let settled = PickThatFits::read(
        &fixture.positions,
        &fixture.indices,
        &fixture.chains,
        &fixture.closed,
        gate::feature_test(&fixture),
        profile,
        fixture.size,
        1e-9,
        Some(gate::capped_budget()),
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
    say(&format!(
        "bridges: {} survivors at size {}, fixture chains {}",
        survivors.len(),
        fixture.size,
        join(&settled.chains)
    ));

    let budget = gate::capped_budget();
    let started = Instant::now();

    // Every vertex the cut adds is weighed against the ones already on the mesh: a second id at a
    // position already taken is where the joint's bridges stand, so the meter names the rule and
    // the split that made each copy. It changes nothing the build does.
    FieldCounters::set_meter_on(true);
    SplitMeter::begin(fixture.size);
    let options = BandGapBuildOptions {
        chord_tolerance: 0.0,
        snap: 1e-9,
        known_room: None,
        budget: Some(Rc::clone(&budget)),
        ..BandGapBuildOptions::default()
    };
    let built = match build_pick(
        &fixture.positions,
        &fixture.indices,
        &survivors,
        &closed,
        gate::feature_test(&fixture),
        fixture.size,
        profile,
        &options,
    ) {
        Ok(built) => built,
        Err(badly) => {
            say(&format!(
                "bridges: stopped by {badly:?} after {:.1} s, spent {} of {} - RAISE \
                 LEVELLINE_COST_CAP",
                started.elapsed().as_secs_f64(),
                gate::n0(budget.spent.get()),
                gate::n0(budget.limit)
            ));
            say(&SplitMeter::report());
            SplitMeter::end();
            FieldCounters::set_meter_on(false);
            return;
        }
    };

    say(&format!(
        "bridges: {:?}, {} loops, {:.1} s, spent {}",
        built.refusal,
        built.loops.len(),
        started.elapsed().as_secs_f64(),
        gate::n0(budget.spent.get())
    ));
    say(&SplitMeter::report());
    SplitMeter::end();
    FieldCounters::set_meter_on(false);

    let Some(mesh) = built.mesh.as_ref().filter(|_| !built.loops.is_empty()) else {
        say("bridges: the build left before the loops were walked - nothing to name at this cap");
        return;
    };
    let mesh = mesh.borrow();

    // The groups are not carried on the result, but the per-side rows name each picked chain's
    // group, and a group's field is the pick field of its members - which the loop walk read.
    let mut members_of_group: IndexMap<i32, Vec<i32>> = IndexMap::new();
    for row in &built.side_loops {
        members_of_group
            .entry(row.group_tag)
            .or_default()
            .push(row.chain);
    }
    for members in members_of_group.values_mut() {
        members.sort();
        members.dedup();
    }
    let field_of_group: IndexMap<i32, PickField> = members_of_group
        .iter()
        .map(|(&tag, members)| {
            (
                tag,
                PickField::new(built.chains.clone(), members).expect("a group's members"),
            )
        })
        .collect();

    // A mesh vertex below the input's count is an input vertex; anything above it the refinement
    // or the cut made. The chain vertices are input vertices by construction.
    let input_vertices = fixture.positions.len() as i32;
    let mut on_a_chain = HashMap::new();
    for (k, chain) in survivors.iter().enumerate() {
        for &v in chain {
            on_a_chain.entry(v).or_insert(k as i32);
        }
    }

    let wanted = wanted_pairs();
    let mut named = 0;
    for &pair in &wanted {
        let loops: Vec<&GapLoop> = built
            .loops
            .iter()
            .filter(|l| {
                l.has_mixed_owners && l.owners.contains(&pair.0) && l.owners.contains(&pair.1)
            })
            .collect();
        say(&format!(
            "=== pair ({},{}): {} mixed loops ===",
            pair.0,
            pair.1,
            loops.len()
        ));
        for gap_loop in loops {
            let field = match field_of_group.get(&gap_loop.side) {
                Some(direct) => direct,
                None => {
                    let group = members_of_group
                        .iter()
                        .find(|(_, m)| m.contains(&pair.0) && m.contains(&pair.1))
                        .map(|(&tag, _)| tag)
                        .unwrap_or_default();
                    &field_of_group[&group]
                }
            };
            let context = Context {
                mesh: &mesh,
                field,
                chains: &built.chains,
                input_vertices,
                on_a_chain: &on_a_chain,
                size: fixture.size,
            };
            named += context.name_the_bridges(gap_loop, pair);
        }
    }

    say(&format!(
        "bridges: {named} owner changes named over {} pairs",
        wanted.len()
    ));
}

fn join(values: &[i32]) -> String {
    values
        .iter()
        .map(|v| v.to_string())
        .collect::<Vec<_>>()
        .join(",")
}

/// What the bridge naming, describing and corner readings share as parameters.
struct Context<'a> {
    mesh: &'a LevelLineMesh,
    field: &'a PickField,
    chains: &'a [Rc<ChainField>],
    input_vertices: i32,
    on_a_chain: &'a HashMap<i32, i32>,
    size: f64,
}

impl Context<'_> {
    fn owner(&self, p: Vector3) -> i32 {
        self.field.evaluate(p).expect("an unbudgeted reading").chain
    }

    /// Walks one mixed loop and prints every place the minimising chain changes, with the mesh
    /// and the field either side of it. Returns how many changes it found.
    fn name_the_bridges(&self, gap_loop: &GapLoop, pair: (i32, i32)) -> i32 {
        let owners: Vec<i32> = gap_loop
            .vertices
            .iter()
            .map(|&v| self.owner(self.mesh.position(v)))
            .collect();
        let mut changes = Vec::new();
        for i in 0..owners.len() {
            let next = (i + 1) % owners.len();
            if !gap_loop.is_closed && next == 0 {
                break;
            }

            if owners[i] != owners[next] {
                changes.push(i);
            }
        }

        say(&format!(
            "  loop side {}, closed {}, {} vertices, owners {} - {} owner changes",
            gap_loop.side,
            gap_loop.is_closed,
            gap_loop.vertices.len(),
            join(&gap_loop.owners),
            changes.len()
        ));

        // Only the changes BETWEEN the two wanted chains are worth the print; a mixed loop of many
        // chains has changes that are not this pair's bridge.
        let interesting: Vec<usize> = changes
            .iter()
            .copied()
            .filter(|&i| {
                let a = owners[i];
                let b = owners[(i + 1) % owners.len()];
                (a == pair.0 && b == pair.1) || (a == pair.1 && b == pair.0)
            })
            .collect();
        say(&format!(
            "    {} of them cross between {} and {}",
            interesting.len(),
            pair.0,
            pair.1
        ));

        let to_print = if interesting.is_empty() {
            &changes
        } else {
            &interesting
        };
        for &i in to_print.iter().take(4) {
            let next = (i + 1) % owners.len();
            self.describe(
                gap_loop.vertices[i],
                gap_loop.vertices[next],
                owners[i],
                owners[next],
            );
        }

        changes.len() as i32
    }

    /// One owner change: the boundary step from `a` to `b`, the kept and removed triangles on that
    /// mesh edge, every corner's reading, and what the field truly does over the removed triangle
    /// and along the edge.
    fn describe(&self, a: i32, b: i32, owner_a: i32, owner_b: i32) {
        let mesh = self.mesh;
        let pa = mesh.position(a);
        let pb = mesh.position(b);
        say(&format!(
            "    -- change {owner_a} -> {owner_b} on mesh edge {a}-{b}, length {:.5} = {:.3} sizes",
            mesh.length(a, b),
            mesh.length(a, b) / self.size
        ));
        say(&format!("       {}", self.corner(a, "A")));
        say(&format!("       {}", self.corner(b, "B")));

        let standing = mesh.triangles_on(a, b);
        say(&format!(
            "       live triangles on the edge: {}",
            standing.len()
        ));
        for &t in standing {
            self.say_triangle("KEPT", t, a, b);
        }

        // The band triangle on the other side of the boundary was REMOVED, so it is not on the
        // edge any more; it is found by scanning for a removed triangle naming both endpoints.
        let removed_here: Vec<i32> = (0..mesh.triangle_count())
            .filter(|&t| {
                if !mesh.is_removed(t) {
                    return false;
                }

                let tri = &mesh.triangles()[t as usize];
                (tri.a == a || tri.b == a || tri.c == a) && (tri.a == b || tri.b == b || tri.c == b)
            })
            .collect();

        say(&format!(
            "       removed triangles on the edge: {}",
            removed_here.len()
        ));
        for &t in &removed_here {
            self.say_triangle("REMOVED", t, a, b);
        }

        let mid = (pa + pb) / 2.0;
        let at_mid = self.field.evaluate(mid).expect("an unbudgeted reading");
        say(&format!(
            "       true field at edge midpoint {}: value {:.6}, owner {}, setback {:.5}",
            point(mid),
            at_mid.value,
            at_mid.chain,
            self.field.setback_of(&at_mid)
        ));
        say(&format!("       along the edge: {}", self.along(pa, pb)));

        // Where on the step the owner actually changes, by halving - the bridge's own point.
        let mut lo = pa;
        let mut hi = pb;
        for _ in 0..40 {
            let m = (lo + hi) / 2.0;
            if self.owner(m) == owner_a {
                lo = m;
            } else {
                hi = m;
            }
        }

        let crossing = (lo + hi) / 2.0;
        let at_crossing = self
            .field
            .evaluate(crossing)
            .expect("an unbudgeted reading");
        say(&format!(
            "       the owner changes at {}: value {:.6}, owner {}",
            point(crossing),
            at_crossing.value,
            at_crossing.chain
        ));
        let distance_to = |k: i32| {
            self.chains[k as usize]
                .evaluate(crossing)
                .expect("an unbudgeted reading")
                .distance
        };
        say(&format!(
            "       its distance to chain {owner_a} is {:.5}, to chain {owner_b} {:.5}",
            distance_to(owner_a),
            distance_to(owner_b)
        ));
    }

    fn say_triangle(&self, state: &str, t: i32, a: i32, b: i32) {
        let tri = &self.mesh.triangles()[t as usize];
        let third = [tri.a, tri.b, tri.c]
            .into_iter()
            .find(|&v| v != a && v != b)
            .expect("a third corner");
        say(&format!(
            "       {state} triangle {t} (tag {}), third corner: {}",
            tri.tag,
            self.corner(third, "T")
        ));
        say(&format!(
            "         field over it: {}",
            self.over(
                self.mesh.position(tri.a),
                self.mesh.position(tri.b),
                self.mesh.position(tri.c)
            )
        ));
    }

    fn corner(&self, v: i32, label: &str) -> String {
        let p = self.mesh.position(v);
        let s = self.field.evaluate(p).expect("an unbudgeted reading");
        let (low, high) = self.mesh.input_edge_behind(v);
        let chain = self
            .on_a_chain
            .get(&v)
            .map_or("-".to_string(), |k| k.to_string());
        format!(
            "{label} vertex {v} {}: value {:.6}, owner {}, setback {:.5}, distance {:.5}, {}, \
             ancestry {low}-{high}, on chain {chain}",
            point(p),
            s.value,
            s.chain,
            self.field.setback_of(&s),
            s.distance,
            if v < self.input_vertices {
                "INPUT"
            } else {
                "split"
            }
        )
    }

    fn over(&self, a: Vector3, b: Vector3, c: Vector3) -> String {
        let least = self.field.min_over_triangle(a, b, c).expect("a reading");
        let most = self
            .field
            .max_over_triangle(a, b, c, self.size * 1e-4)
            .expect("a reading");
        format!(
            "least {:.6} at {}, greatest {:.6} at {}",
            least.value,
            point(least.point),
            most.value,
            point(most.point)
        )
    }

    fn along(&self, a: Vector3, b: Vector3) -> String {
        let least = self.field.min_along_segment(a, b).expect("a reading");
        let most = self
            .field
            .max_along_segment(a, b, self.size * 1e-4)
            .expect("a reading");
        format!(
            "least {:.6} at {}, greatest {:.6} at {}",
            least.value,
            point(least.point),
            most.value,
            point(most.point)
        )
    }
}
