//! One part of the pierced face tests: THE WHOLE PASS'S EDGES - the seam part's ledger
//! ([`crate::section_pierced_face_seam_tests`]) run over every run the routed PASS builds and
//! counted the way the OPERATION counts, which is undirected. The class's helpers are
//! [`crate::section_pierced_face_readings`].

use std::collections::HashMap;
use std::fmt::Write;

use solidmesh::bevel::sections::section_routing::{self, SectionRouteAnswer};
use solidmesh::bevel::sections::{section_face_rebuild, section_loft, section_smooth_seam};
use solidmesh::topology::FeatureEdgeSet;

use crate::inner_face_test_mesh;
use crate::section_pierced_face_readings::the_plans;
use crate::section_pierced_face_seam_tests::the_names_of_the_points;

/// THE PASS'S TWO OPEN EDGES ARE THE SEAM PART'S OWN RUN, READ THE OTHER WAY. The pick opens ONE plan
/// and the pass builds ONE run - the very run `TheOpenEdgesOfTheSeamAreNamed` reads - so the
/// "different construction of the same pick" the parked row spoke of was never a second
/// construction at all, only a second READING.
///
/// The seam part counts a DIRECTED edge unmatched; the operation is refused by
/// `SurfaceMeshClosure.OpenEdgeCount`, which is UNDIRECTED - an edge run by four faces counts there
/// however well its directions pair. The rim chord P1 <-> P2 is run four times at each junction:
/// by the junction's PATCH (laid by the rebuild as a face of the part), by the CUT's own ear - the
/// same triangle wound the other way - and by the loft's and the cut's two ordinary faces on the
/// chord. So the WEDGE between the chord and the fold is laid TWICE, once by each construction,
/// coincident and opposite. Which of the three closures to take (the cut's ring runs the chord and
/// the patch keeps the wedge; the cut keeps the wedge and the patch goes; neither lays it) is the
/// owner's; dropping only ONE of the two copies does not close.
#[test]
fn the_whole_pass_runs_its_rim_chord_four_times_at_each_junction() {
    let (said, open, not_two) = the_passs_open_edges();

    assert_eq!(
        open.len(),
        0,
        "no DIRECTED edge of the pass is unmatched - {said}"
    );

    assert_eq!(
        not_two.len(),
        2,
        "one rim chord per junction, and nothing else - {said}"
    );

    for (_, uses) in &not_two {
        assert_eq!(*uses, 4, "{said}");
    }
}

/// An undirected edge of the pass, with how many faces run it.
type EdgeUses = ((i32, i32), usize);

/// Runs the pass's three stages by hand over EVERY run and names every
/// directed edge nothing lays back, and every undirected edge some number of faces other than two
/// runs.
fn the_passs_open_edges() -> (String, Vec<(i32, i32)>, Vec<EdgeUses>) {
    let part = inner_face_test_mesh::the_part();
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(
        &part,
        &graph,
        inner_face_test_mesh::BEND_THRESHOLD_DEGREES,
    );
    let (plans, route, _) = the_plans(&part, None);

    let mut said = String::new();

    let _ = writeln!(said, "route {route:?}, {} plans", plans.len());

    for plan in &plans {
        let _ = writeln!(
            said,
            "plan size {} edges {} vertices {:?}",
            plan.size,
            plan.edges.len(),
            plan.edges
                .iter()
                .map(|one| one.edge.from_vertex())
                .collect::<Vec<_>>()
        );
    }

    let routed = section_routing::is_simple(
        &plans,
        Some(&part),
        Some(&graph),
        inner_face_test_mesh::SEGMENTS,
        Some(&features),
    )
    .unwrap_or_else(|nan| panic!("a section point is not a number: {nan:?}"));

    let route = match routed {
        SectionRouteAnswer::Sections(route) => route,
        SectionRouteAnswer::Legacy { why_not, .. } => {
            panic!("the pass routes to the sections - instead {why_not}; {said}")
        }
    };

    let runs = route.runs;
    let mut pool = route.pool;

    let _ = writeln!(said, "routed true runs {}", runs.len());

    let seams = section_smooth_seam::try_cut_every_smooth_side(&part, &graph, &mut pool, &runs)
        .unwrap_or_else(|uncut| panic!("every smooth side cuts - instead {uncut:?}; {said}"));

    let _ = writeln!(said, "cut {} seams", seams.len());

    let mut faces =
        section_face_rebuild::try_rebuild(&part, &graph, &runs, &pool, Some(seams.as_slice()))
            .unwrap_or_else(|unbuilt| {
                panic!("the rebuild lays - instead {:?}; {said}", unbuilt.code())
            });

    let _ = writeln!(said, "rebuilt {} faces", faces.len());

    let mut laid_by: HashMap<usize, String> = (0..faces.len())
        .map(|i| (i, "rebuilt face".to_string()))
        .collect();

    for (r, run) in runs.iter().enumerate() {
        let before = faces.len();
        let lofting = seams.iter().find(|one| one.run == r as i32);

        let unlofted = section_loft::try_emit(run, &pool, &mut faces, lofting);

        let _ = writeln!(
            said,
            "run {r} ({} stations) lofted to {} {:?}",
            run.count(),
            faces.len(),
            unlofted.err().map(|failure| failure.code())
        );

        for i in before..faces.len() {
            laid_by.insert(i, format!("loft run {r}"));
        }
    }

    for (s, seam) in seams.iter().enumerate() {
        let before = faces.len();

        faces.extend(seam.cut.triangles.iter().copied());

        for i in before..faces.len() {
            laid_by.insert(i, format!("cut facet seam {s}"));
        }
    }

    let mut name: HashMap<i32, String> = HashMap::new();

    for (r, run) in runs.iter().enumerate() {
        let named = seams.iter().find(|one| one.run == r as i32);

        for (point, called) in the_names_of_the_points(run, named.map(|seam| &seam.cut)) {
            name.insert(point, format!("run {r} {called}"));
        }
    }

    let name_of = |point: i32| name.get(&point).cloned().unwrap_or_else(|| "?".to_string());

    let mut laid: HashMap<(i32, i32), Vec<usize>> = HashMap::new();

    for (i, face) in faces.iter().enumerate() {
        for pair in [(face.0, face.1), (face.1, face.2), (face.2, face.0)] {
            laid.entry(pair).or_default().push(i);
        }
    }

    let mut open: Vec<(i32, i32)> = laid
        .keys()
        .copied()
        .filter(|edge| !laid.contains_key(&(edge.1, edge.0)))
        .collect();
    open.sort();

    for edge in &open {
        let _ = writeln!(
            said,
            "OPEN {} ({} {:?}) -> {} ({} {:?}) laid by {}",
            edge.0,
            name_of(edge.0),
            pool.position_of(edge.0),
            edge.1,
            name_of(edge.1),
            pool.position_of(edge.1),
            laid[edge]
                .iter()
                .map(|&face| format!("{} {face} {:?}", laid_by[&face], faces[face]))
                .collect::<Vec<_>>()
                .join(", ")
        );
    }

    // The reading the OPERATION is refused by is UNDIRECTED: an edge run by four faces counts as
    // open there however well its directions pair - see SurfaceMeshClosure.OpenEdgeCount.
    let mut uses: HashMap<(i32, i32), Vec<usize>> = HashMap::new();

    for (key, by) in &laid {
        let edge = (key.0.min(key.1), key.0.max(key.1));

        uses.entry(edge).or_default().extend(by.iter().copied());
    }

    let mut not_two: Vec<EdgeUses> = uses
        .iter()
        .filter(|one| one.1.len() != 2)
        .map(|(edge, by)| (*edge, by.len()))
        .collect();

    // Ordered by the low end, the high end breaking a tie, so the message lists edges the same way
    // every run.
    not_two.sort();

    for (edge, _) in &not_two {
        let _ = writeln!(
            said,
            "NOT TWO {} ({} {:?}) -> {} ({} {:?}) by {}",
            edge.0,
            name_of(edge.0),
            pool.position_of(edge.0),
            edge.1,
            name_of(edge.1),
            pool.position_of(edge.1),
            uses[edge]
                .iter()
                .map(|&face| format!("{} {:?}", laid_by[&face], faces[face]))
                .collect::<Vec<_>>()
                .join(", ")
        );
    }

    (said, open, not_two)
}
