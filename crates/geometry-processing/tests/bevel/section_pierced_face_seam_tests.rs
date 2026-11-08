//! One part of the pierced face tests: THE SEAM AT THE JUNCTION PATCH - the assembly's three stages
//! (the smooth side's cut, the face rebuild, the loft) run by hand over the pierced top face's one
//! run at 2 * Radius, so each face keeps the name of the construction that laid it, and every
//! directed edge nothing lays back is named. Nothing is open. Its point-naming reader
//! (`TheNamesOfThePoints`) is shared with [`crate::section_pierced_face_pass_tests`]; the class's
//! helpers are [`crate::section_pierced_face_readings`].

use std::collections::{HashMap, HashSet};
use std::fmt::Write;

use solidmesh::bevel::sections::section_smooth_side::SmoothSideCut;
use solidmesh::bevel::sections::{
    section_face_rebuild, section_loft, section_smooth_seam, SectionRun,
};

use crate::inner_face_test_mesh;
use crate::section_pierced_face_junction_run_tests::the_run;
use crate::section_pierced_face_readings::RADIUS;

/// EVERY OPEN EDGE OF THE ASSEMBLY, NAMED. The three stages are run here by hand so each face keeps
/// the name of the construction that laid it, and every unmatched directed edge is reported with
/// both of its ends read out of the junction's own ledger and the cut's places.
///
/// MEASURED on the top face's pick at 2 * Radius: NOTHING IS OPEN. The two
/// patches that used to be owed - the takeover triangle at vertex 19 (fanned by the loft's
/// `TryFanTheTakeover`) and its mirror at vertex 59 (`TryFanTheArrivingTakeover`, once the split
/// station beside the junction was laid on the carried wall end) - were the same wedge read from
/// either end. Both junctions ALSO lay their rim chord `P1 <-> P2` FOUR times, balanced but
/// non-manifold; that is recorded here and not claimed either way.
#[test]
fn the_open_edges_of_the_seam_are_named() {
    let (said, open) = the_assemblys_open_edges();

    assert_eq!(open.len(), 0, "{said}");

    // Kept for the next reading: a hole here has always been a RING - every open edge's far end the
    // near end of exactly one other - which is what says a patch is owed and not a boundary torn.
    let rings = the_rings_of(&open);

    assert!(rings.is_empty(), "{said}");
}

/// The cycles a set of directed edges falls into - each open edge followed by its
/// successor.
fn the_rings_of(open: &[(i32, i32)]) -> Vec<Vec<(i32, i32)>> {
    let mut rings = Vec::new();
    let mut taken: HashSet<(i32, i32)> = HashSet::new();

    for &start in open {
        if !taken.insert(start) {
            continue;
        }

        let mut ring = vec![start];
        let mut at = start;

        loop {
            let next: Vec<(i32, i32)> = open
                .iter()
                .copied()
                .filter(|edge| edge.0 == at.1 && !taken.contains(edge))
                .collect();

            if next.len() != 1 {
                break;
            }

            taken.insert(next[0]);
            ring.push(next[0]);
            at = next[0];
        }

        rings.push(ring);
    }

    rings
}

/// Runs the assembly's three stages by hand and writes out every open
/// directed edge with the construction that laid it and the name of each end.
fn the_assemblys_open_edges() -> (String, Vec<(i32, i32)>) {
    let part = inner_face_test_mesh::the_part();
    let graph = part.get_mesh_edge_graph();
    let (run, mut pool) = the_run(&part, 2.0 * RADIUS, inner_face_test_mesh::SEGMENTS);
    let runs = std::slice::from_ref(&run);

    let mut said = String::new();

    let seams = section_smooth_seam::try_cut_every_smooth_side(&part, &graph, &mut pool, runs)
        .unwrap_or_else(|uncut| panic!("the smooth side cuts - instead {uncut:?}"));

    let _ = writeln!(said, "cut {} seams", seams.len());

    assert_eq!(seams.len(), 1, "one seam");
    let seam = &seams[0];

    let mut faces =
        section_face_rebuild::try_rebuild(&part, &graph, runs, &pool, Some(seams.as_slice()))
            .unwrap_or_else(|unbuilt| panic!("the rebuild lays - instead {:?}", unbuilt.code()));

    let _ = writeln!(said, "rebuilt {} faces", faces.len());

    let mut laid_by: HashMap<usize, &str> = (0..faces.len()).map(|i| (i, "rebuilt face")).collect();

    let before_the_loft = faces.len();

    let unlofted = section_loft::try_emit(&run, &pool, &mut faces, Some(seam));

    let _ = writeln!(
        said,
        "lofted to {} {:?}",
        faces.len(),
        unlofted.err().map(|failure| failure.code())
    );

    for i in before_the_loft..faces.len() {
        laid_by.insert(i, "loft");
    }

    let before_the_cut = faces.len();

    faces.extend(seam.cut.triangles.iter().copied());

    for i in before_the_cut..faces.len() {
        laid_by.insert(i, "cut facet");
    }

    // Which construction minted each pool index, so an open edge can be read by its ENDS as well.
    let name = the_names_of_the_points(&run, Some(&seam.cut));
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

    for (key, by) in laid.iter().filter(|one| one.1.len() != 1) {
        let _ = writeln!(
            said,
            "TWICE {} ({}) -> {} ({}) by {}",
            key.0,
            name_of(key.0),
            key.1,
            name_of(key.1),
            by.iter()
                .map(|face| laid_by[face])
                .collect::<Vec<_>>()
                .join(", ")
        );
    }

    for station in [0usize, 1, 2, 3, 39, 40, 41] {
        let section = &run.sections[station];

        let _ = writeln!(
            said,
            "station {station}: left {} right {} joinPlacedLeft {} joinPlacedRight {} cut {:?} \
             crossingsAfter {:?}",
            section.left_boundary,
            section.right_boundary,
            section.left_boundary_is_join_placed,
            section.right_boundary_is_join_placed,
            seam.cut.station_points[station],
            seam.cut
                .crossings_after_station(station as i32)
                .cloned()
                .unwrap_or_default()
        );
    }

    let _ = writeln!(said, "smooth side is left {}", seam.is_left);

    (said, open)
}

/// What minted each pool index the run and its cut use - for a reading's
/// message only. A run with no smooth side has no cut at all, handed in as `None`.
pub(crate) fn the_names_of_the_points(
    run: &SectionRun,
    cut: Option<&SmoothSideCut>,
) -> HashMap<i32, String> {
    let mut name = HashMap::new();

    for (station, section) in run.sections.iter().enumerate() {
        for (i, &point) in section.profile.iter().enumerate() {
            name.insert(point, format!("station {station} profile {i}"));
        }

        name.insert(
            section.left_boundary,
            format!("station {station} left boundary"),
        );
        name.insert(
            section.right_boundary,
            format!("station {station} right boundary"),
        );
    }

    for (station, junction) in &run.junctions {
        for (i, &sample) in junction.samples.iter().enumerate() {
            let role = if i as i32 == junction.arriving_wall_end {
                " P1 arriving wall end"
            } else if i as i32 == junction.leaving_wall_end {
                " P2 leaving wall end"
            } else {
                ""
            };

            let carried = if junction.carried_onto[i] >= 0 {
                format!(" carried onto face {}", junction.carried_onto[i])
            } else {
                String::new()
            };

            name.insert(
                sample,
                format!("junction {station} sample {i}{role}{carried}"),
            );
        }
    }

    let Some(cut) = cut else {
        return name;
    };

    for (station, points) in cut.station_points.iter().enumerate() {
        for (i, &point) in points.iter().enumerate() {
            let entry = name.entry(point).or_default();

            if entry.is_empty() {
                *entry = format!("cut station {station} boundary {i}");
            } else {
                *entry += &format!(" / cut station {station} boundary {i}");
            }
        }
    }

    for &point in &cut.boundary {
        name.entry(point)
            .or_insert_with(|| "cut crossing".to_string());
    }

    name
}
