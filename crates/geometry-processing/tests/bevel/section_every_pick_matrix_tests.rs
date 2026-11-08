//! Every pick a user could make on the raked bore cube
//! (`inner_face_test_mesh`) - each coplanar face region, each distinct tangent chain, each straight
//! edge between two axis-aligned faces - built at sizes 2 and 0.5 with the sections router off and
//! on, and every failure held to a QUEUE: a queued pick that now builds must leave the queue, a pick
//! outside it may not fail, and a queued pick failing a new way is a regression. Each corner's rows
//! are also appended table-ready to `every-pick-matrix.txt` in `BEVEL_MATRIX_DIR` (or the temp
//! directory). The parameter rows are a loop inside one test.

use std::collections::{HashMap, HashSet};

use solidmesh::bevel::sections::section_routing;
use solidmesh::bevel::surface::{surface_mesh_closure, surface_shared_edge_network};
use solidmesh::bevel::BevelAdvisoryArg;
use solidmesh::topology::{
    bevel_edge_group_resolver, BevelEdgeGroup, BevelEdgeGroupResolution, BevelSelectionKind,
    EdgeConvexity, FaceGroup, FeatureEdge, FeatureEdgeSet,
};
use solidmesh_math::number_format::format_f64_max_decimals;
use solidmesh_math::Vector3;
use solidmesh_mesh::{CancelToken, Mesh};

use crate::inner_face_test_mesh::{the_part, BEND_THRESHOLD_DEGREES, SEGMENTS};
use crate::mesh_measurements::{shell_count, signed_volume};
use crate::surface_test_build::{
    pinned_to_the_legacy_construction, pinned_to_the_sections, try_bevel_resolutions,
    TryBevelOptions,
};
use crate::surface_test_chains::join_and_open;

const MATRIX_FILE_NAME: &str = "every-pick-matrix.txt";

const CHAIN_CURVE_TOLERANCE_DEGREES: f64 = FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES;

const EXPECTED_CASE_COUNT: usize = 39;

const QUEUE_AT_2_OFF: &[(&str, &str)] = &[
    ("chain 1@(-19, -31.5, 20)", "SurfaceStripEndNotSupported"),
    ("chain 1@(-19, -46.5, 20)", "SurfaceStripEndNotSupported"),
    ("chain 1@(1, -31.5, 0)", "SurfaceStripEndNotSupported"),
    ("chain 1@(1, -46.5, 0)", "SurfaceStripEndNotSupported"),
    (
        "chain 36@(-19, -43.97, 19.39)",
        "SurfaceStripSeamDidNotMeet",
    ),
    (
        "chain 36@(-6.06, -39.1, 0)",
        "SurfaceCutMeetsStandingFeature",
    ),
    ("chain 40@(-18.94, -34, 20)", "SurfaceStripSeamDidNotMeet"),
    (
        "chain 40@(1, -34.03, 0.48)",
        "SurfaceCutMeetsStandingFeature",
    ),
    ("edge @(-19, -31.5, 20)", "SurfaceStripEndNotSupported"),
    ("edge @(-19, -46.5, 20)", "SurfaceStripEndNotSupported"),
    ("edge @(1, -31.5, 0)", "SurfaceStripEndNotSupported"),
    ("edge @(1, -46.5, 0)", "SurfaceStripEndNotSupported"),
    ("face (-1, 0, 0) x39", "SurfaceMitreConsumesTheStrip"),
    ("face (0, 0, -1) x39", "SurfaceCutMeetsStandingFeature"),
    ("face (0, 0, 1) x43", "SurfaceMitreConsumesTheStrip"),
    (
        "face (0.54, 0.65, 0.54) x152",
        "SurfaceMitreConsumesTheStrip",
    ),
    ("face (1, 0, -0) x43", "SurfaceCutMeetsStandingFeature"),
];

const QUEUE_AT_2_ON: &[(&str, &str)] = &[
    ("chain 1@(-19, -31.5, 20)", "SurfaceStripEndNotSupported"),
    ("chain 1@(-19, -46.5, 20)", "SurfaceStripEndNotSupported"),
    ("chain 1@(1, -31.5, 0)", "SurfaceStripEndNotSupported"),
    ("chain 1@(1, -46.5, 0)", "SurfaceStripEndNotSupported"),
    ("chain 36@(-19, -43.97, 19.39)", "SurfaceCornerNotSupported"),
    ("chain 36@(-6.06, -39.1, 0)", "SurfaceCornerNotSupported"),
    ("chain 40@(-18.94, -34, 20)", "SurfaceCornerNotSupported"),
    ("chain 40@(1, -34.03, 0.48)", "SurfaceCornerNotSupported"),
    ("edge @(-19, -31.5, 20)", "SurfaceStripEndNotSupported"),
    ("edge @(-19, -46.5, 20)", "SurfaceStripEndNotSupported"),
    ("edge @(1, -31.5, 0)", "SurfaceStripEndNotSupported"),
    ("edge @(1, -46.5, 0)", "SurfaceStripEndNotSupported"),
    ("face (-1, 0, 0) x39", "SurfaceMitreConsumesTheStrip"),
    ("face (0, 0, -1) x39", "SurfaceCutMeetsStandingFeature"),
    ("face (0, 0, 1) x43", "SurfaceMitreConsumesTheStrip"),
    ("face (1, 0, -0) x43", "SurfaceCutMeetsStandingFeature"),
];

const QUEUE_AT_05_OFF: &[(&str, &str)] = &[
    ("chain 1@(-19, -31.5, 20)", "SurfaceStripEndNotSupported"),
    ("chain 1@(-19, -46.5, 20)", "SurfaceStripEndNotSupported"),
    ("chain 1@(-9, -29, 0)", "left open"),
    ("chain 1@(1, -31.5, 0)", "SurfaceStripEndNotSupported"),
    ("chain 1@(1, -46.5, 0)", "SurfaceStripEndNotSupported"),
    (
        "chain 36@(-19, -43.97, 19.39)",
        "SurfaceStripSeamDidNotMeet",
    ),
    (
        "chain 36@(-6.06, -39.1, 0)",
        "SurfaceCutMeetsStandingFeature",
    ),
    ("chain 40@(-18.94, -34, 20)", "SurfaceStripSeamDidNotMeet"),
    (
        "chain 40@(1, -34.03, 0.48)",
        "SurfaceCutMeetsStandingFeature",
    ),
    ("edge @(-19, -31.5, 20)", "SurfaceStripEndNotSupported"),
    ("edge @(-19, -46.5, 20)", "SurfaceStripEndNotSupported"),
    ("edge @(-9, -29, 0)", "left open"),
    ("edge @(1, -31.5, 0)", "SurfaceStripEndNotSupported"),
    ("edge @(1, -46.5, 0)", "SurfaceStripEndNotSupported"),
    ("face (-1, 0, 0) x39", "SurfaceMitreConsumesTheStrip"),
    ("face (0, 0, -1) x39", "SurfaceCutMeetsStandingFeature"),
    ("face (0, 0, 1) x43", "SurfaceMitreConsumesTheStrip"),
    (
        "face (0.54, 0.65, 0.54) x152",
        "SurfaceMitreConsumesTheStrip",
    ),
    ("face (1, 0, -0) x43", "SurfaceCutMeetsStandingFeature"),
];

const QUEUE_AT_05_ON: &[(&str, &str)] = &[
    ("chain 1@(-19, -31.5, 20)", "SurfaceStripEndNotSupported"),
    ("chain 1@(-19, -46.5, 20)", "SurfaceStripEndNotSupported"),
    ("chain 1@(-9, -29, 0)", "left open"),
    ("chain 1@(1, -31.5, 0)", "SurfaceStripEndNotSupported"),
    ("chain 1@(1, -46.5, 0)", "SurfaceStripEndNotSupported"),
    ("chain 36@(-19, -43.97, 19.39)", "SurfaceResultNotClosed"),
    ("chain 36@(-6.06, -39.1, 0)", "SurfaceResultNotClosed"),
    ("chain 40@(-18.94, -34, 20)", "SurfaceResultNotClosed"),
    ("chain 40@(1, -34.03, 0.48)", "SurfaceResultNotClosed"),
    ("edge @(-19, -31.5, 20)", "SurfaceStripEndNotSupported"),
    ("edge @(-19, -46.5, 20)", "SurfaceStripEndNotSupported"),
    ("edge @(-9, -29, 0)", "left open"),
    ("edge @(1, -31.5, 0)", "SurfaceStripEndNotSupported"),
    ("edge @(1, -46.5, 0)", "SurfaceStripEndNotSupported"),
    ("face (-1, 0, 0) x39", "SurfaceMitreConsumesTheStrip"),
    ("face (0, 0, -1) x39", "SurfaceCutMeetsStandingFeature"),
    ("face (0, 0, 1) x43", "SurfaceMitreConsumesTheStrip"),
    ("face (1, 0, -0) x43", "SurfaceCutMeetsStandingFeature"),
];

/// One pick a user could make, named by where it is on the part.
struct Pick {
    name: String,
    group: BevelEdgeGroup,
}

#[test]
fn every_pick_on_the_raked_bore_cube_is_on_the_queue() {
    for (size, use_sections, queue) in [
        (2.0, false, QUEUE_AT_2_OFF),
        (2.0, true, QUEUE_AT_2_ON),
        (0.5, false, QUEUE_AT_05_OFF),
        (0.5, true, QUEUE_AT_05_ON),
    ] {
        let part = the_part();
        let cases = cases(&part);

        // The extent of the matrix, pinned: a matrix that stopped covering what it covers is not
        // the same matrix.
        assert_eq!(
            cases.len(),
            EXPECTED_CASE_COUNT,
            "the pick enumeration changed: {}",
            cases
                .iter()
                .map(|one| one.name.as_str())
                .collect::<Vec<_>>()
                .join("; ")
        );

        let mut said: HashMap<String, String> = HashMap::new();
        let mut rows = Vec::new();

        {
            let _pin = if use_sections {
                pinned_to_the_sections()
            } else {
                pinned_to_the_legacy_construction()
            };

            for pick in &cases {
                let (row, complaint) = read(&part, pick, size);
                rows.push(row);

                if let Some(complaint) = complaint {
                    said.insert(pick.name.clone(), complaint);
                }
            }
        }

        write(size, use_sections, &rows);

        is_on_the_queue(
            &cases.iter().map(|one| one.name.clone()).collect::<Vec<_>>(),
            &said,
            queue,
            &corner(size, use_sections),
        );
    }
}

fn corner(size: f64, use_sections: bool) -> String {
    format!(
        "{}{}",
        format_f64_max_decimals(size, 2),
        if use_sections { " on" } else { " off" }
    )
}

/// The matrix's rows, appended where a person running it can read them; a file that cannot be
/// written is no failure of the matrix.
fn write(size: f64, use_sections: bool, rows: &[String]) {
    use std::io::Write;

    let scratch = std::env::var("BEVEL_MATRIX_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|_| std::env::temp_dir().join("bevel-matrix"));

    if std::fs::create_dir_all(&scratch).is_err() {
        return;
    }

    let mut text = format!("\n==== size {} ====\n", corner(size, use_sections));

    for row in rows {
        text.push_str(row);
        text.push('\n');
    }

    if let Ok(mut file) = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(scratch.join(MATRIX_FILE_NAME))
    {
        let _ = file.write_all(text.as_bytes());
    }
}

/// One pick built: its table row, and what it complains of (`None` where it built soundly).
fn read(part: &Mesh, pick: &Pick, size: f64) -> (String, Option<String>) {
    let resolution = bevel_edge_group_resolver::resolve(&pick.group, part);

    if !resolution.is_resolved() {
        let unresolved = format!(
            "unresolved {:?}",
            resolution
                .failure_reason
                .as_ref()
                .map(|reason| reason.code())
        );

        return (format!("{} | {unresolved}", pick.name), Some(unresolved));
    }

    let shape = shape(&resolution);
    let outcome = try_bevel_resolutions(
        part,
        std::slice::from_ref(&resolution),
        size,
        SEGMENTS,
        &CancelToken::new(),
        TryBevelOptions::default(),
    );

    let (complaint, said) = match outcome.mesh_or_reason() {
        Err(reason) => {
            let complaint = match reason {
                Some(reason) => format!(
                    "{:?} at {}",
                    reason.code(),
                    reason.args().first().map_or("no point".to_string(), show)
                ),
                None => "None at no point".to_string(),
            };
            let said = format!("refused {complaint}");

            (Some(complaint), said)
        }
        Ok(mesh) => {
            let complaint = complaint_about(mesh, &resolution);
            let said = match &complaint {
                None => "built".to_string(),
                Some(complaint) => format!("built badly: {complaint}"),
            };

            (complaint, said)
        }
    };

    (
        format!(
            "{} | {} edges | {shape} | {said} | route \"{}\"",
            pick.name,
            resolution.edges.len(),
            section_routing::last_route().unwrap_or_default()
        ),
        complaint,
    )
}

fn shape(resolution: &BevelEdgeGroupResolution) -> String {
    let chains = join_and_open(&surface_shared_edge_network::link_together(
        std::slice::from_ref(resolution),
    ));
    let closed = chains.iter().filter(|chain| chain.is_closed).count();

    format!(
        "{} chains ({closed} closed, {} open)",
        chains.len(),
        chains.len() - closed
    )
}

// `!(volume < source)` on purpose: a NaN volume counts as wrong.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn complaint_about(mesh: &Mesh, resolution: &BevelEdgeGroupResolution) -> Option<String> {
    let mut complaints = Vec::new();

    let (open, open_at) = surface_mesh_closure::open_edge_count(mesh);
    if open != 0 {
        complaints.push(format!("left open at {}", point(open_at)));
    }

    if !mesh.is_manifold() {
        complaints.push("is not manifold".to_string());
    }

    let shells = shell_count(mesh);
    if shells != 1 {
        complaints.push(format!("came out as {shells} shells"));
    }

    // Which way the volume may go is the SELECTION's business: a convex edge is only ever cut
    // back, a concave one only ever filled in, and a pick of both says nothing either way.
    let volume = signed_volume(mesh);
    let source = signed_volume(&the_part());

    if resolution
        .edges
        .iter()
        .all(|edge| edge.convexity == EdgeConvexity::Convex)
        && !(volume < source)
    {
        complaints.push(format!(
            "came out at {}, not under the part's {}, though every edge picked is convex",
            format_f64_max_decimals(volume, 3),
            format_f64_max_decimals(source, 3)
        ));
    }

    if resolution
        .edges
        .iter()
        .all(|edge| edge.convexity == EdgeConvexity::Concave)
        && !(volume > source)
    {
        complaints.push(format!(
            "came out at {}, not over the part's {}, though every edge picked is concave",
            format_f64_max_decimals(volume, 3),
            format_f64_max_decimals(source, 3)
        ));
    }

    if complaints.is_empty() {
        None
    } else {
        Some(complaints.join(" / "))
    }
}

fn is_on_the_queue(
    names: &[String],
    said: &HashMap<String, String>,
    expected_failures: &[(&str, &str)],
    corner: &str,
) {
    let known: HashSet<&str> = names.iter().map(String::as_str).collect();

    let gone_from_the_fixture: Vec<&str> = expected_failures
        .iter()
        .map(|(name, _)| *name)
        .filter(|name| !known.contains(name))
        .collect();
    assert!(
        gone_from_the_fixture.is_empty(),
        "{corner}: the queue names picks this part no longer has, so the list is stale: {}",
        gone_from_the_fixture.join("; ")
    );

    let now_passing: Vec<&str> = expected_failures
        .iter()
        .map(|(name, _)| *name)
        .filter(|name| !said.contains_key(*name))
        .collect();
    assert!(
        now_passing.is_empty(),
        "{corner}: these build now - remove them from the queue, which is the only way it ever \
         shrinks: {}",
        now_passing.join("; ")
    );

    let mut newly_failing: Vec<String> = said
        .iter()
        .filter(|(name, _)| !expected_failures.iter().any(|(queued, _)| queued == name))
        .map(|(name, complaint)| format!("{name} -> {complaint}"))
        .collect();
    newly_failing.sort();
    assert!(
        newly_failing.is_empty(),
        "{corner}: nothing outside the queue may fail:\n{}",
        newly_failing.join("\n")
    );

    let failing_differently: Vec<String> = expected_failures
        .iter()
        .filter_map(|&(name, expected)| {
            let complaint = said.get(name)?;

            (kind(complaint) != expected).then(|| {
                format!(
                    "{name} was '{expected}' and is now '{}': {complaint}",
                    kind(complaint)
                )
            })
        })
        .collect();
    assert!(
        failing_differently.is_empty(),
        "{corner}: a queued pick failing a NEW way is a regression, not the queue:\n{}",
        failing_differently.join("\n")
    );
}

pub(crate) fn kind(complaint: &str) -> &str {
    complaint
        .find(" at ")
        .map_or(complaint, |at| &complaint[..at])
}

pub(crate) fn show(value: &BevelAdvisoryArg) -> String {
    match value {
        BevelAdvisoryArg::Vector3(at) => point(*at),
        other => format!("{other:?}"),
    }
}

/// A point as `(x, y, z)` with at most two decimals each, "-0" kept, for the matrix table.
pub(crate) fn point(at: Vector3) -> String {
    format!(
        "({}, {}, {})",
        format_f64_max_decimals(at.x, 2),
        format_f64_max_decimals(at.y, 2),
        format_f64_max_decimals(at.z, 2)
    )
}

fn cases(part: &Mesh) -> Vec<Pick> {
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(part, &graph, BEND_THRESHOLD_DEGREES);

    let mut picks = face_picks(part, &features);
    picks.extend(chain_picks(part, &features));
    picks.extend(edge_picks(&features));

    // A stable sort by the names' bytes, so the table's row order never depends on the platform.
    picks.sort_by(|a, b| a.name.cmp(&b.name));

    picks
}

fn face_picks(part: &Mesh, features: &FeatureEdgeSet) -> Vec<Pick> {
    let graph = part.get_mesh_edge_graph();
    let mut seen = HashSet::new();
    let mut picks = Vec::new();

    for face in 0..part.faces.len() {
        if seen.contains(&(face as i32)) {
            continue;
        }

        let region =
            FaceGroup::create_with_graph(part, &graph, face as i32, features.flat_angle_degrees())
                .expect("every face is a face of the part");

        seen.extend(region.faces().iter().copied());

        if region.boundary_edges().is_empty() {
            continue;
        }

        let normal = part.faces[face].normal.as_vector3();

        picks.push(Pick {
            name: format!("face {} x{}", point(normal), region.faces().len()),
            group: bevel_edge_group_resolver::create_from_face_group(
                &region,
                &region.boundary_edges()[0],
                features.flat_angle_degrees(),
            )
            .expect("the boundary edge is the region's own"),
        });
    }

    picks
}

fn chain_picks(part: &Mesh, features: &FeatureEdgeSet) -> Vec<Pick> {
    let mut already = HashSet::new();
    let mut picks = Vec::new();

    for index in 0..features.count() as i32 {
        let group = bevel_edge_group_resolver::create_from_edge(
            features,
            index,
            BevelSelectionKind::TangentChain,
            CHAIN_CURVE_TOLERANCE_DEGREES,
        )
        .expect("every index of the set is a crease of the part");
        let resolution = bevel_edge_group_resolver::resolve(&group, part);

        if !resolution.is_resolved() {
            // An unresolvable seed is still a pick someone could make, named by its own edge.
            if already.insert(format!("unresolved {index}")) {
                picks.push(Pick {
                    name: format!("chain unresolved@{}", point(group.anchor_midpoint)),
                    group,
                });
            }

            continue;
        }

        let mut edges: Vec<i32> = resolution
            .edges
            .iter()
            .map(|edge| edge.mesh_edge_index)
            .collect();
        edges.sort_unstable();

        let key = edges
            .iter()
            .map(i32::to_string)
            .collect::<Vec<_>>()
            .join(",");

        if !already.insert(key) {
            continue;
        }

        picks.push(Pick {
            name: format!("chain {}@{}", edges.len(), point(lowest(&resolution.edges))),
            group,
        });
    }

    picks
}

fn edge_picks(features: &FeatureEdgeSet) -> Vec<Pick> {
    (0..features.count())
        .filter(|&index| {
            let edge = &features.edges()[index];

            is_axis_aligned(edge.normal0) && is_axis_aligned(edge.normal1)
        })
        .map(|index| Pick {
            name: format!("edge @{}", point(midpoint(&features.edges()[index]))),
            group: bevel_edge_group_resolver::create_from_edge(
                features,
                index as i32,
                BevelSelectionKind::Edge,
                CHAIN_CURVE_TOLERANCE_DEGREES,
            )
            .expect("every index of the set is a crease of the part"),
        })
        .collect()
}

fn is_axis_aligned(normal: Vector3) -> bool {
    let unit = normal.get_normal();

    (unit.x.abs() + unit.y.abs() + unit.z.abs() - 1.0).abs() < 1e-4
        && (unit.x.abs() > 0.9999 || unit.y.abs() > 0.9999 || unit.z.abs() > 0.9999)
}

fn midpoint(edge: &FeatureEdge) -> Vector3 {
    (edge.start + edge.end) / 2.0
}

fn lowest(edges: &[FeatureEdge]) -> Vector3 {
    let mut lowest = midpoint(&edges[0]);

    for edge in edges {
        let mid = midpoint(edge);

        if mid.x < lowest.x
            || (mid.x == lowest.x && mid.y < lowest.y)
            || (mid.x == lowest.x && mid.y == lowest.y && mid.z < lowest.z)
        {
            lowest = mid;
        }
    }

    lowest
}
