//! What `LevelLineMesh::repair_needles` does to a needle, and what it deliberately does not. A
//! needle is a triangle with no height whose apex lies ON its long edge - a T-VERTEX the
//! tessellation wrote as a triangle. Repair re-tiles the NEIGHBOUR through the apex instead, which
//! costs no vertex and leaves that run belonging to triangles that have height. The one test that
//! meters its build switches the meter for its own thread.
//! The meter row writes its report to `LEVELLINE_COST_LOG` when set.

use std::cell::RefCell;
use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::{
    band_cut, band_gap_reading, BandProfile, ChainField, EdgeRoot, FieldCounters, LevelLineMesh,
    LevelLineWork, VertexFieldReading, VertexSide, WorkBudget, WorkError,
};
use vector_math::Vector3;

use crate::graze_fixture;
use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_test_geometry::signed_volume;
use crate::line_to_line_fixture::{self, Defect};
use crate::meter_lock::meter_lock;
use crate::several_chains_fixtures;
use crate::sweep_mesh_measures;

/// The floor every shape here is measured against - the needles are well under it.
const FLOOR: f64 = 1e-3;

/// The needle-with-a-neighbour shape: 0-1 is the long edge, 2 the apex a ten-thousandth off its
/// middle, 3 the neighbour's corner.
fn needle_and_neighbour() -> (Vec<Vector3>, Vec<(i32, i32, i32)>) {
    (
        vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(4.0, 0.0, 0.0),
            Vector3::new(2.0, 1e-4, 0.0),
            Vector3::new(2.0, 3.0, 0.0),
        ],
        vec![(0, 1, 2), (1, 0, 3)],
    )
}

fn mesh_of(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    tags: Option<&[i32]>,
) -> LevelLineMesh {
    let mut mesh = LevelLineMesh::from_triangles(positions, indices, tags).expect("a mesh");
    mesh.needle_floor = FLOOR;
    mesh
}

fn chain_edge_0_1() -> Rc<dyn Fn(i32, i32) -> bool> {
    Rc::new(|a, b| (a == 0 && b == 1) || (a == 1 && b == 0))
}

/// A needle with a neighbour across its long edge: the needle goes, the neighbour becomes two
/// triangles through the apex, and the mesh is left conforming - no vertex minted, no edge on three
/// triangles, no triangle of no area standing.
#[test]
fn a_needle_with_a_neighbour_becomes_the_neighbours_t_vertex() {
    let (positions, indices) = needle_and_neighbour();
    let mut mesh = mesh_of(&positions, &indices, None);
    let touched = mesh.repair_needles().expect("no budget in force");

    assert_eq!(
        mesh.needles_repaired, 1,
        "there is one needle and it can be mended"
    );
    assert!(
        !mesh.repair_hit_its_bound,
        "two triangles cannot need a bound"
    );
    assert_eq!(
        mesh.vertex_count(),
        4,
        "the apex is already there: repair mints nothing"
    );
    assert!(mesh.is_removed(0), "the needle itself is dropped");
    assert_eq!(
        mesh.removed_count(),
        0,
        "a repaired needle is not band: it left no hole"
    );
    assert_eq!(
        live(&mesh).len(),
        2,
        "one neighbour re-tiled through the apex is two triangles"
    );
    assert_eq!(touched.len(), 2, "the two it made or changed");

    // Both live triangles name the apex, and between them they cover the neighbour.
    for live in live(&mesh) {
        let t = mesh.triangles()[live as usize];
        assert!(
            t.a == 2 || t.b == 2 || t.c == 2,
            "triangle {live} is one half of the neighbour, so it stands on the apex: {}",
            corners(&mesh, live)
        );
    }

    assert!(!mesh.has_edge(0, 1), "the long edge was re-tiled away");
    assert_eq!(mesh.triangles_on(0, 2).len(), 1);
    assert_eq!(mesh.triangles_on(2, 1).len(), 1);
    assert_eq!(
        mesh.triangles_on(2, 3).len(),
        2,
        "the new interior edge divides the neighbour in two"
    );

    for (low, high) in mesh.edges() {
        assert!(
            mesh.triangles_on(low, high).len() <= 2,
            "edge {low}-{high} stands on more than two triangles"
        );
    }

    for live in live(&mesh) {
        assert!(
            twice_the_area(&mesh, live) > 1e-9,
            "triangle {live} has no area left: {}",
            corners(&mesh, live)
        );

        // The WINDING, not just the area: the neighbour (1, 0, 3) faces -z, so both halves of it
        // must.
        assert!(
            normal(&mesh, live).z < 0.0,
            "triangle {live} winds the other way from the neighbour it replaces: {}",
            corners(&mesh, live)
        );
    }
}

/// A needle whose neighbour across the long edge carries a DIFFERENT tag is left standing: both
/// halves of a re-tiling would take the NEIGHBOUR's tag, which on a needle that is the only tagged
/// link between two runs of a side severs the side the band is read off.
#[test]
fn a_needle_facing_another_tag_is_left_alone() {
    let (positions, indices) = needle_and_neighbour();
    let mut mesh = mesh_of(&positions, &indices, Some(&[1, 0]));
    let touched = mesh.repair_needles().expect("no budget in force");

    assert_eq!(
        mesh.needles_repaired, 0,
        "the neighbour is on the other side of the tagging frontier"
    );
    assert_eq!(
        mesh.needles_left_alone(),
        1,
        "one needle, counted once however often it was examined"
    );
    assert_eq!(touched.len(), 0);
    assert!(mesh.has_edge(0, 1));
    assert!(!mesh.is_removed(0));
}

/// A needle with NOTHING across its long edge - a face-region boundary - is left exactly as it was
/// and counted. There is no neighbour to re-tile, and dropping it would open a hole in the mesh.
#[test]
fn a_boundary_needle_is_left_alone_and_counted() {
    let positions = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(4.0, 0.0, 0.0),
        Vector3::new(2.0, 1e-4, 0.0),
    ];
    let mut mesh = mesh_of(&positions, &[(0, 1, 2)], None);
    let touched = mesh.repair_needles().expect("no budget in force");

    assert_eq!(
        mesh.needles_repaired, 0,
        "there is nothing across the long edge to re-tile"
    );
    assert_eq!(
        mesh.needles_left_alone(),
        1,
        "and the needle is counted, not passed over in silence"
    );
    assert_eq!(touched.len(), 0);
    assert!(!mesh.is_removed(0), "dropping it would open a hole");
    assert_eq!(mesh.vertex_count(), 3);
}

/// A needle whose apex is at an END of its long edge - a zero-length edge in waiting - is left
/// alone too: re-tiling through it would mint a triangle with two corners in the same place.
#[test]
fn a_needle_whose_apex_is_at_an_end_is_left_alone() {
    let positions = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(4.0, 0.0, 0.0),
        Vector3::new(4.0, 1e-5, 0.0),
        Vector3::new(2.0, 3.0, 0.0),
    ];
    let mut mesh = mesh_of(&positions, &[(0, 1, 2), (1, 0, 3)], None);
    mesh.repair_needles().expect("no budget in force");

    assert_eq!(
        mesh.needles_repaired, 0,
        "the apex is vertex 1's own place: a re-tiling there is a zero-length edge"
    );
    assert_eq!(mesh.needles_left_alone(), 1);
}

/// A needle with a NEEDLE across its long edge - the line-to-line fixture's own shape - settles:
/// after the build's input cleanup and repair pass no needle is left, no repair made one, and the
/// part is still one closed, consistently wound shell of the same volume. Two needles face to face
/// are a STRIP, and it is `remove_input_needle_strips` that takes it.
#[test]
fn a_needle_against_a_needle_settles() {
    // 0-1 the long edge, 2 and 3 the apexes, 4..7 the corners of the slab's 6 by 6 top face. The
    // bottom face mirrors the top, so it carries the same pair.
    let top = [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(4.0, 0.0, 0.0),
        Vector3::new(2.0, 1e-4, 0.0),
        Vector3::new(2.0, -1e-4, 0.0),
        Vector3::new(-1.0, -3.0, 0.0),
        Vector3::new(5.0, -3.0, 0.0),
        Vector3::new(5.0, 3.0, 0.0),
        Vector3::new(-1.0, 3.0, 0.0),
    ];
    let faces = [
        (0, 1, 2),
        (1, 0, 3),
        (0, 2, 7),
        (2, 1, 6),
        (2, 6, 7),
        (0, 3, 4),
        (3, 1, 5),
        (3, 5, 4),
        (0, 7, 4),
        (1, 5, 6),
    ];

    let n = top.len() as i32;
    let positions: Vec<Vector3> = top
        .iter()
        .copied()
        .chain(top.iter().map(|&p| p - Vector3::new(0.0, 0.0, 1.0)))
        .collect();
    let at = |v: i32| top[v as usize];
    let mut indices = Vec::new();
    for (a, b, c) in faces {
        // Wound up, by the sign of its area - the top face is flat at z = 0.
        let up = (at(b) - at(a)).cross(at(c) - at(a)).z > 0.0;
        indices.push(if up { (a, b, c) } else { (a, c, b) });
        indices.push(if up {
            (a + n, c + n, b + n)
        } else {
            (a + n, b + n, c + n)
        });
    }

    let rim = [4, 5, 6, 7];
    for k in 0..rim.len() {
        let u = rim[k];
        let v = rim[(k + 1) % rim.len()];
        indices.push((v, u, u + n));
        indices.push((v, u + n, v + n));
    }

    let mut mesh = mesh_of(&positions, &indices, None);
    let volume_before = signed_volume(&mesh);
    let needles_before = needle_count(&mesh);

    // The build's own order (BandGap): the strip removal, then the per-needle repair.
    mesh.remove_input_needle_strips()
        .expect("no budget in force");
    mesh.repair_needles().expect("no budget in force");

    let kept = sweep_mesh_measures::kept(&mesh);
    let volume_after = signed_volume(&mesh);

    assert_eq!(
        needles_before, 4,
        "the pair on the top face and its mirror on the bottom, and nothing else"
    );
    assert_eq!(
        needle_count(&mesh),
        0,
        "no needle over the floor may remain"
    );
    assert!(!mesh.repair_hit_its_bound);
    assert_eq!(
        mesh.vertex_count(),
        positions.len() as i32,
        "nothing is minted, ever"
    );
    assert!(
        sweep_mesh_measures::is_closed_and_consistent(&positions, &kept),
        "every edge on two triangles, wound the opposite way on each"
    );
    assert_eq!(
        sweep_mesh_measures::shell_count(mesh.vertex_count(), &kept),
        1
    );
    assert!(
        (volume_after - volume_before).abs() <= FLOOR * 4.0,
        "the surface moves by no more than the floor, over the strip's length: {volume_before} -> {volume_after}"
    );
}

/// A needle whose LONG edge is the user's own picked crease is left standing, even though it is
/// the very shape the first case mends: repair deletes the long edge, and the build's setbacks,
/// field and side tagging were all read off that edge. The same holds for a PIECE of that edge,
/// after the splits have cut it up.
#[test]
fn a_needle_whose_long_edge_is_the_chain_is_left_alone() {
    let (positions, indices) = needle_and_neighbour();
    let mut mesh = mesh_of(&positions, &indices, None);
    mesh.is_chain_edge = Some(chain_edge_0_1());
    let touched = mesh.repair_needles().expect("no budget in force");

    assert_eq!(mesh.needles_repaired, 0, "mending it would delete the pick");
    assert_eq!(
        mesh.needles_left_on_a_chain_edge, 1,
        "and it is counted under its own reason, not lumped in with the unmendable"
    );
    assert_eq!(mesh.needles_left_alone(), 1);
    assert_eq!(touched.len(), 0);
    assert!(!mesh.is_removed(0));
    assert!(
        mesh.has_edge(0, 1),
        "the chain edge is still there to be read off"
    );

    // Now the same needle where the long edge is HALF a chain edge: split 0-1 first, so the
    // needle's long edge descends from the pick without naming its ends. Read through
    // InputEdgeBehind, it is held exactly as the whole edge was.
    let mut split = LevelLineMesh::from_triangles(
        &[
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(8.0, 0.0, 0.0),
            Vector3::new(2.0, 1e-4, 0.0),
            Vector3::new(2.0, 3.0, 0.0),
        ],
        &[(0, 1, 2), (1, 0, 3)],
        None,
    )
    .expect("a mesh");
    let mid = split
        .split_edge(0, 1, Vector3::new(4.0, 0.0, 0.0), true)
        .expect("the edge splits");
    split.needle_floor = FLOOR;
    split.is_chain_edge = Some(chain_edge_0_1());
    split.repair_needles().expect("no budget in force");

    assert!(split.has_edge(0, mid));
    assert_eq!(
        split.needles_repaired, 0,
        "0-mid is a piece of the pick, and deleting a piece of it is deleting it"
    );
    assert!(split.needles_left_on_a_chain_edge >= 1);
}

/// WHAT THE REPAIR MEETS: the needles each of the fixtures that carry them hands in, counted three
/// ways - mended, left standing, and of those left standing on a chain edge. A measurement, not a
/// contract: the only thing asserted is that the meter was actually reading.
#[test]
fn the_needle_counts_of_every_fixture_that_carries_one() {
    let _meter = meter_lock();
    let mut rows: Vec<String> = Vec::new();

    let mut measure = |what: String, build: &dyn Fn()| {
        FieldCounters::reset();
        FieldCounters::set_meter_on(true);
        // The meter is switched off even when the build panics.
        struct Off;
        impl Drop for Off {
            fn drop(&mut self) {
                FieldCounters::set_meter_on(false);
            }
        }
        let _off = Off;
        build();
        let (repaired, left_alone, on_a_chain_edge) = FieldCounters::with(|t| {
            (
                t.needles_repaired,
                t.needles_left_alone,
                t.needles_on_a_chain_edge,
            )
        });
        rows.push(format!(
            "{what}: repaired {repaired}, left alone {left_alone}, of those on a chain edge {on_a_chain_edge}"
        ));
    };

    for (name, size) in [("A", 0.3), ("C", 0.0)] {
        let (positions, indices, chains) = several_chains_fixtures::named(name);
        let feature = || {
            Some(feature_edges_over(
                &positions,
                &indices,
                several_chains_fixtures::FOLD_DEGREES,
            ))
        };
        let closed = vec![true; chains.len()];

        // C's two rims close on each other at 0.15, so the size it is measured at is 0.9 of its
        // OWN room - the case the room tests build - and not a number that would refuse.
        let at = if size > 0.0 {
            size
        } else {
            band_gap_reading::room_of_pick(
                &positions,
                &indices,
                &chains,
                &closed,
                feature(),
                BandProfile::Chamfer,
                1e-9,
            )
            .expect("the room")
                * 0.9
        };
        measure(format!("several chains {name} at {at:.4}"), &|| {
            band_gap_entry_points::build_pick(
                &positions,
                &indices,
                &chains,
                &closed,
                feature(),
                at,
                BandProfile::Chamfer,
                &BandGapBuildOptions::default(),
            )
            .expect("a valid pick");
        });
    }

    for defect in [Defect::InsideToInsideNeedle, Defect::OnToOnNeedle] {
        let pick = line_to_line_fixture::pick(
            defect,
            line_to_line_fixture::RING_SEGMENTS,
            line_to_line_fixture::SIZE,
        );
        measure(format!("line to line, {defect:?}"), &|| {
            line_to_line_fixture::build(&pick, line_to_line_fixture::SIZE, BandProfile::Chamfer);
        });
    }

    for graze in [
        graze_fixture::GRAZE_AS_NEEDLE,
        graze_fixture::GRAZE_WITH_HEIGHT,
    ] {
        let pick = graze_fixture::pick(graze, graze_fixture::RING_SEGMENTS, graze_fixture::SIZE);
        measure(format!("graze at {graze:.4} setbacks"), &|| {
            graze_fixture::build(&pick, graze_fixture::SIZE, BandProfile::Chamfer);
        });
    }

    let report = rows.join("\n");
    println!("{report}");
    if let Ok(log) = std::env::var("LEVELLINE_COST_LOG") {
        if !log.is_empty() {
            use std::io::Write;
            let mut file = std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(&log)
                .expect("the cost log opens");
            writeln!(file, "{report}").expect("the cost log takes the report");
        }
    }

    assert_eq!(rows.len(), 6, "every fixture named here was measured");
    assert!(
        !FieldCounters::meter_on(),
        "the meter is left off for whoever runs next"
    );
}

/// A root standing exactly on the apex of a needle the repair had to LEAVE. The mesh answers such
/// a split with the apex and divides nothing, so the long edge would be left crossing the line with
/// no vertex on it. The cut asks for the T-vertex repair by name instead, and the edge ends up
/// carrying the apex - with a neighbour across the pick, and as a BOUNDARY needle with none.
#[test]
fn a_root_on_the_apex_of_a_needle_left_alone_still_puts_a_vertex_on_its_edge() {
    // A field whose line runs along x = 2: the long edge from (0,0) to (4,0) crosses it at the apex.
    let field = Rc::new(
        ChainField::from_polyline(
            &[Vector3::new(2.0, -5.0, 0.0), Vector3::new(2.0, 5.0, 0.0)],
            false,
            &[0.0, 0.0],
        )
        .expect("a field"),
    );

    for with_a_neighbour in [true, false] {
        let positions = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(4.0, 0.0, 0.0),
            Vector3::new(2.0, 1e-9, 0.0), // the apex, standing inside the long edge and ON the line
            Vector3::new(2.0, 3.0, 0.0),
        ];
        let mut indices = vec![(0, 1, 2)];
        if with_a_neighbour {
            indices.push((1, 0, 3));
        }

        let mut mesh = mesh_of(&positions, &indices, None);

        // Held off the repair for the one reason that is about the pick, so the needle is still
        // there when the root is found on its long edge.
        mesh.is_chain_edge = Some(chain_edge_0_1());
        mesh.repair_needles().expect("no budget in force");

        assert_eq!(
            mesh.needles_repaired, 0,
            "the fixture needs the needle left standing"
        );
        assert!(mesh.has_edge(0, 1));

        let mesh = Rc::new(RefCell::new(mesh));
        let reading =
            VertexFieldReading::new(Rc::clone(&mesh), field.clone(), 1e-9).expect("a snap");
        let cut = band_cut::cut(&reading, &[EdgeRoot::new(0, 1, positions[2])]);

        assert!(
            cut.is_ok(),
            "the cut must answer, not throw: {:?}",
            cut.err()
        );
        let mesh = mesh.borrow();
        assert_eq!(
            mesh.vertex_count(),
            4,
            "the apex was already there: nothing is minted"
        );
        assert_eq!(
            side_of(&reading, 2),
            VertexSide::On,
            "the root's vertex is the apex, and it is read onto the line"
        );

        // Whatever stands on the crossed run now names the apex, so no live triangle has one corner
        // inside and one outside with nothing on the line between them.
        assert_no_triangle_straddles(&mesh, &reading);

        assert_eq!(
            live(&mesh).len(),
            if with_a_neighbour { 2 } else { 1 },
            "the neighbour is re-tiled in two and the needle dropped; alone, the needle stays"
        );
    }
}

/// A repair pass is CHARGED to the work budget, and a budget with nothing left stops it - the
/// turns are counted at all, and a spent budget throws instead of letting the pass run to its
/// bound.
#[test]
fn a_repair_pass_is_charged_to_the_work_budget_and_an_exhausted_one_stops_it() {
    let (positions, indices) = needle_and_neighbour();

    let mut counted = mesh_of(&positions, &indices, None);
    let budget = Rc::new(WorkBudget::new(1_000_000).expect("a limit"));
    {
        let _work = LevelLineWork::begin(Some(Rc::clone(&budget)));
        counted.repair_needles().expect("the budget is large");
    }

    assert_eq!(
        counted.needles_repaired, 1,
        "the shape still has its one mendable needle"
    );
    assert!(
        budget.spent.get() > 0,
        "every turn of the repair worklist is work, and work the budget cannot see cannot be refused by name"
    );

    // A budget with one unit in it: the pass seeds both triangles, so its SECOND turn must throw
    // rather than let the worklist run on to its own bound.
    let mut stopped = mesh_of(&positions, &indices, None);
    let spent = Rc::new(WorkBudget::new(1).expect("a limit"));
    let threw = {
        let _work = LevelLineWork::begin(Some(spent));
        matches!(stopped.repair_needles(), Err(WorkError::BudgetExceeded(_)))
    };

    assert!(
        threw,
        "a build whose repairs stop converging has to refuse by name, not run the machine out of memory"
    );
}

/// FOUR NEARLY COLLINEAR POINTS cannot be tiled without a needle, so a repair there would make
/// what it mends. The repair is refused instead: the needle is left standing, counted, and the
/// pass ends at once rather than flipping the pair's diagonal back and forth.
#[test]
fn a_repair_that_would_make_a_needle_is_refused() {
    // Heights, in floors: A over P0-B about 0.53 and P0 over A-P1 about 0.87 - the two needles -
    // while P0 over B-P1 is 2.2 and A over B-P1 is 2.
    let top = [
        Vector3::new(-2.0, 0.0, 0.0),        // P1
        Vector3::new(0.0, 2.2 * FLOOR, 0.0), // P0
        Vector3::new(1.0, 2.0 * FLOOR, 0.0), // A
        Vector3::new(3.0, 0.0, 0.0),         // B
    ];
    const P1: i32 = 0;
    const P0: i32 = 1;
    const A: i32 = 2;
    const B: i32 = 3;

    // A closed slab: the top face is the two triangles, the bottom the same reversed a unit below,
    // and a wall stands on each of the top's four rim edges.
    let positions: Vec<Vector3> = top
        .iter()
        .copied()
        .chain(top.iter().map(|&p| p - Vector3::new(0.0, 0.0, 1.0)))
        .collect();
    let mut indices = vec![
        (P0, B, A), // the needle: A stands half a floor off P0-B, a third of the way along
        (B, P0, P1),
        (P0 + 4, A + 4, B + 4),
        (B + 4, P1 + 4, P0 + 4),
    ];
    let rim = [P1, B, A, P0];
    for k in 0..rim.len() {
        let u = rim[k];
        let v = rim[(k + 1) % rim.len()];
        indices.push((v, u, u + 4));
        indices.push((v, u + 4, v + 4));
    }

    let mut mesh = mesh_of(&positions, &indices, None);
    let needles_before = needle_count(&mesh);

    let budget = Rc::new(WorkBudget::new(100_000).expect("a limit"));
    let exhausted = {
        let _work = LevelLineWork::begin(Some(budget));
        matches!(
            mesh.repair_needles_around([0]),
            Err(WorkError::BudgetExceeded(_))
        )
    };

    let needles_after = needle_count(&mesh);

    assert!(
        !exhausted,
        "a pass that cannot make a needle has nothing to run on for"
    );
    assert!(
        !mesh.repair_hit_its_bound,
        "every repair must lower the needle count, so the worklist empties by construction"
    );
    assert!(mesh.needles_repaired <= 1, "one needle was handed in");
    assert!(
        needles_after <= needles_before,
        "a repair never adds a needle"
    );
    assert_eq!(
        mesh.needles_left_alone(),
        1,
        "the needle whose every re-tiling is a needle is left, and counted"
    );

    for (low, high) in mesh.edges() {
        assert_eq!(
            mesh.triangles_on(low, high).len(),
            2,
            "edge {low}-{high} must stay on two triangles: the slab is closed"
        );
    }
}

/// A needle the repair LEAVES because every re-tiling of it is a needle, with a root landing on
/// its apex: the cut still puts a vertex on the crossed edge and answers, rather than throwing "a
/// root was missed". The T-vertex repair the cut asks for by name is deliberately NOT held to the
/// repair's no-needle rule, and this is the case that says why.
#[test]
fn a_root_on_the_apex_of_a_needle_the_rule_left_still_cuts() {
    // 0-1 the long edge, 2 the apex, 3 the neighbour's far corner - both apexes under the floor,
    // so the needle and its neighbour are each other's only re-tiling and every re-tiling is a
    // needle.
    let positions = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(4.0, 0.0, 0.0),
        Vector3::new(2.0, 1e-9, 0.0),
        Vector3::new(2.0, -FLOOR / 2.0, 0.0),
    ];
    let mut mesh = mesh_of(&positions, &[(0, 1, 2), (1, 0, 3)], None);
    mesh.repair_needles().expect("no budget in force");

    // A level line along x = 2, through both apexes: the chain runs down x = -1 with a setback of
    // 3, so vertex 0 reads Inside, vertex 1 Outside, and 2 and 3 On.
    let field = Rc::new(
        ChainField::from_polyline(
            &[Vector3::new(-1.0, -5.0, 0.0), Vector3::new(-1.0, 5.0, 0.0)],
            false,
            &[3.0, 3.0],
        )
        .expect("a field"),
    );
    let mesh = Rc::new(RefCell::new(mesh));
    let reading = VertexFieldReading::new(Rc::clone(&mesh), field, 1e-9).expect("a snap");

    assert_eq!(
        mesh.borrow().needles_repaired,
        0,
        "every re-tiling of either triangle is a needle"
    );
    assert_eq!(
        mesh.borrow().needles_left_alone(),
        2,
        "and both are reported as left alone"
    );
    assert_eq!(side_of(&reading, 0), VertexSide::Inside);
    assert_eq!(side_of(&reading, 1), VertexSide::Outside);

    let cut = band_cut::cut(&reading, &[EdgeRoot::new(0, 1, positions[2])]);

    assert!(
        cut.is_ok(),
        "a root on a needle's apex must still be cut, not refused unnamed: {:?}",
        cut.err()
    );
    let mesh = mesh.borrow();
    assert_eq!(
        mesh.vertex_count(),
        4,
        "the apex was already there: nothing is minted"
    );
    assert_no_triangle_straddles(&mesh, &reading);
}

fn side_of(reading: &VertexFieldReading, vertex: i32) -> VertexSide {
    reading.side_of(vertex).expect("no budget in force")
}

fn assert_no_triangle_straddles(mesh: &LevelLineMesh, reading: &VertexFieldReading) {
    for live in live(mesh) {
        let t = mesh.triangles()[live as usize];
        let sides = [
            side_of(reading, t.a),
            side_of(reading, t.b),
            side_of(reading, t.c),
        ];
        assert!(
            !(sides.contains(&VertexSide::Inside) && sides.contains(&VertexSide::Outside)),
            "triangle {live} still straddles the line: {}",
            corners(mesh, live)
        );
    }
}

fn needle_count(mesh: &LevelLineMesh) -> usize {
    live(mesh)
        .into_iter()
        .filter(|&t| mesh.has_no_height(mesh.triangles()[t as usize], mesh.needle_floor))
        .count()
}

fn live(mesh: &LevelLineMesh) -> Vec<i32> {
    (0..mesh.triangle_count())
        .filter(|&t| !mesh.is_removed(t))
        .collect()
}

fn corners(mesh: &LevelLineMesh, triangle: i32) -> String {
    let t = mesh.triangles()[triangle as usize];
    format!("({}, {}, {})", t.a, t.b, t.c)
}

fn normal(mesh: &LevelLineMesh, triangle: i32) -> Vector3 {
    let t = mesh.triangles()[triangle as usize];
    let a = mesh.position(t.a);
    (mesh.position(t.b) - a).cross(mesh.position(t.c) - a)
}

fn twice_the_area(mesh: &LevelLineMesh, triangle: i32) -> f64 {
    normal(mesh, triangle).length()
}
