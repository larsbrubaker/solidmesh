//! THE SAME sixty builds `SurfaceBevelRobustnessTests`
//! measures the edge-centric construction by, run through the vertex-centric one -
//! `SectionAssembly` - and read by the same invariants plus the one the new path owns: every
//! profile point stands a radius from its own section's canal.
//!
//! Everything that DESCRIBES a row - the twelve cases, the hanger's own splits, the bore fixtures,
//! the mouth pick, the bevel size and land - is taken from `surface_bevel_robustness_tests` by
//! reference, so a case cannot drift between the two tables. Only the build and the reading are
//! this class's own.
//!
//! **The sections ratchet is EMPTY** (measured): every one of the twelve cases
//! closes at all five motions - closed, manifold, one shell, no needle - with the identical
//! topology signature, and every profile point stands a radius from its own section's canal
//! inside the tolerance below. The table is kept empty rather than deleted so a regression names
//! itself the same way a ratchet row always did; regenerate it by running with
//! `BEVEL_SECTIONS_BASELINE` pointed at a file.
//!
//! The one test walks the twelve cases and reports every disagreement together.

use std::sync::Arc;

use solidmesh::bevel::sections::section_builder::try_the_canals_corner;
use solidmesh::bevel::sections::section_builder_run::try_build_chain;
use solidmesh::bevel::sections::{section_assembly, SectionPool, SectionRun};
use solidmesh::bevel::surface::surface_mesh_closure;
use solidmesh::topology::{
    bevel_edge_group_resolver, BevelSelectionKind, FaceGroup, FeatureEdgeSet, MatchTolerances,
};
use solidmesh_math::{Matrix4X4, Vector3};

use crate::bore_test_meshes::{
    create_plate_with_a_raked_bore, create_plate_with_a_raked_bore_whose_mouth_chords_are_split,
};
use crate::mesh_measurements;
use crate::rigid_motion_meshes::{self, TopologySignature};
use crate::surface_bevel_robustness_tests as cases;

/// How far a profile point may stand off its own section's centre segment, as a fraction of the
/// part's diagonal. Relative because the far motions stand the part a thousand times its own size
/// out, where a float32 round trip of the input positions already costs more than any absolute
/// tolerance a build at the origin would want.
const RADIUS_TOLERANCE_OF_THE_DIAGONAL: f64 = 1e-6;

/// WHAT THE SECTIONS CONSTRUCTION FAILS, measured 2026-09-17: NOTHING. The key is
/// "`case | motion`"; the value is every invariant the row breaks. A row not listed passes every
/// invariant, and none is listed.
const THE_FAILURES_OF_THE_SECTIONS_CONSTRUCTION: &[(&str, &str)] = &[];

/// Each case, built where it was made and at each of `rigid_motion_meshes::the_motions`, through
/// the vertex-centric construction, with every invariant read on each and the answers held to the
/// recorded table.
#[test]
fn the_sections_build_the_same_bore_mouth_wherever_the_part_stands() {
    let mut disagreements = Vec::new();

    for (rake, sides, size, split) in cases::the_matrix() {
        let case_name = cases::name_of(rake, sides, size, split);
        let mut at_the_origin: Option<TopologySignature> = None;

        for (motion_name, motion) in rigid_motion_meshes::the_motions() {
            let (mut broken, signature, worst_reach) =
                what_is_broken_by(rake, sides, size, split, motion);

            if motion_name == "identity" {
                at_the_origin = Some(signature.clone());
            } else if let Some(origin) = &at_the_origin {
                if signature != *origin {
                    broken.push(how_it_differs_from(&signature, origin));
                }
            }

            let key = format!("{case_name} | {motion_name}");
            let reading = broken.join("; ");
            let recorded = THE_FAILURES_OF_THE_SECTIONS_CONSTRUCTION
                .iter()
                .find(|(k, _)| *k == key)
                .map(|(_, v)| *v);

            if let Ok(dump) = std::env::var("BEVEL_SECTIONS_BASELINE") {
                use std::io::Write;
                // The worst radius reading rides along as a trailing comment on every row,
                // passing or not, because the question the harness was written to answer is what
                // the raked rows read.
                if let Ok(mut file) = std::fs::OpenOptions::new()
                    .create(true)
                    .append(true)
                    .open(dump)
                {
                    let _ = writeln!(
                        file,
                        "    (\"{key}\", \"{reading}\"), // reach {worst_reach:.4e}, v{} f{}",
                        signature.vertices, signature.faces
                    );
                }
            }

            if reading != recorded.unwrap_or("") {
                disagreements.push(format!(
                    "{key}: reads \"{reading}\", table records \"{}\"\n    (\"{key}\", \"{reading}\"),",
                    recorded.unwrap_or("(nothing - it passes)")
                ));
            }
        }
    }

    assert_eq!(
        disagreements.join("\n"),
        "",
        "UPDATE THE RATCHET - what this case does and what is recorded for it disagree"
    );
}

/// Shared by both robustness files: the moved build's signature against
/// the identity build's, as the numbers that differ.
pub fn how_it_differs_from(moved: &TopologySignature, at_the_origin: &TopologySignature) -> String {
    let mut differences = Vec::new();

    if moved.vertices != at_the_origin.vertices {
        differences.push(format!("v{}/{}", moved.vertices, at_the_origin.vertices));
    }

    if moved.faces != at_the_origin.faces {
        differences.push(format!("f{}/{}", moved.faces, at_the_origin.faces));
    }

    if moved.open_edges != at_the_origin.open_edges {
        differences.push(format!(
            "open{}/{}",
            moved.open_edges, at_the_origin.open_edges
        ));
    }

    if moved.valences != at_the_origin.valences {
        differences.push("valences".to_string());
    }

    format!("moved {}", differences.join(" "))
}

/// Builds one case stood by one motion through the sections and reads every invariant of the
/// answer. The run is built here rather than left inside `section_assembly::try_build` because the
/// radius reading is a question about the SECTIONS, and asking it of the published mesh would need
/// a nearest-centre search that could hide the very error it is looking for.
fn what_is_broken_by(
    rake: f64,
    sides: i32,
    size: f64,
    split: bool,
    motion: Matrix4X4,
) -> (Vec<String>, TopologySignature, f64) {
    let mut broken = Vec::new();

    let mut part = if split {
        create_plate_with_a_raked_bore_whose_mouth_chords_are_split(
            sides,
            cases::RADIUS,
            cases::THICKNESS,
            rake,
            &cases::splits_within(sides),
            6.0,
            Vector3::ZERO,
        )
    } else {
        create_plate_with_a_raked_bore(sides, cases::RADIUS, cases::THICKNESS, rake, 6.0)
    };

    rigid_motion_meshes::move_mesh(&mut part, motion);

    let graph = part.get_mesh_edge_graph();
    let features = Arc::new(FeatureEdgeSet::create_with_graph(
        &part,
        &graph,
        cases::BEND_THRESHOLD_DEGREES,
    ));

    // Picked through the RESOLVER, exactly as SurfaceBevelRobustnessTests picks: a bare
    // FeatureEdgeChain.Propagate stops on its seed at 24 sides, where a 15 degree wall facet is
    // over the harness's 10 degree flat angle and three feature edges meet at every mouth vertex.
    let group = bevel_edge_group_resolver::create_from_edge(
        &features,
        cases::the_upper_mouth(&features, motion.inverted(), rake),
        BevelSelectionKind::TangentChain,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("the upper mouth is one of the part's feature edges");

    let chain = bevel_edge_group_resolver::resolve_crease(
        &group,
        features.clone(),
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
        MatchTolerances::default(),
    )
    .chain;

    let mut pool = SectionPool::new(&part.vertices);
    let radius = size;

    let chain = match chain {
        Some(chain) if chain.is_closed() => chain,
        other => {
            // Read and reported separately from a refusal, because it is the HARNESS's pick that
            // came up short and not the construction.
            broken.push(format!(
                "the pick walked {} edges and did not close",
                other.map_or(0, |c| c.count())
            ));

            return (broken, rigid_motion_meshes::signature_of(None), 0.0);
        }
    };

    // WHICH STAGE refused rides in the reading, because the two carry the same advisory code and
    // the table cannot otherwise tell a section that could not be drawn from one that was drawn
    // and then could not be substituted into the part's faces.
    let run = match try_build_chain(
        Some(&chain),
        &part,
        &graph,
        &mut pool,
        radius,
        cases::SEGMENTS,
        true,
    ) {
        Ok(run) => run,
        Err(reason) => {
            broken.push(format!("the sections refused {}", code_of(&reason)));

            return (broken, rigid_motion_meshes::signature_of(None), 0.0);
        }
    };

    let mesh = match section_assembly::try_assemble(&part, &graph, &run, &mut pool) {
        Ok(mesh) => mesh,
        Err(reason) => {
            broken.push(format!("the assembly refused {}", code_of(&reason)));

            return (broken, rigid_motion_meshes::signature_of(None), 0.0);
        }
    };

    let signature = rigid_motion_meshes::signature_of(Some(&mesh));

    let (open, _) = surface_mesh_closure::open_edge_count(&mesh);

    if open != 0 {
        broken.push(format!("open {open}"));
    }

    let most = rigid_motion_meshes::most_faces_on_an_edge(&mesh);

    if most > 2 {
        broken.push(format!("fold {most}"));
    }

    let shells = mesh_measurements::shell_count(&mesh);

    if shells != 1 {
        broken.push(format!("shells {shells}"));
    }

    // A result no second pass can read is not a finished result: see
    // mesh_measurements::unclassifiable_edges.
    let (unreadable, unreadable_at) = mesh_measurements::unclassifiable_edges(Some(&mesh));

    if unreadable != 0 {
        broken.push(format!(
            "unclassifiable {unreadable} first at {unreadable_at}"
        ));
    }

    let needles = rigid_motion_meshes::two_cornered_triangle_count(&mesh);

    if needles != 0 {
        broken.push(format!("two-cornered {needles}"));
    }

    let worst_reach = worst_reach_off_the_canal(&run, &pool, radius);

    let allowed =
        RADIUS_TOLERANCE_OF_THE_DIAGONAL * part.get_axis_aligned_bounding_box().size().length();

    if worst_reach > allowed {
        broken.push(format!("reach {worst_reach:.4e} of {allowed:.4e}"));
    }

    (broken, signature, worst_reach)
}

/// A refusal's code as text, or "with no code" where there is none.
pub fn code_of(
    reason: &solidmesh::bevel::sections::section_builder_walked_side::SectionBuildRefusal,
) -> String {
    reason
        .advisory()
        .map_or_else(|| "with no code".to_string(), |a| format!("{:?}", a.code()))
}

/// The farthest any profile point of any section stands off a radius from its OWN station's
/// canal, the point of the sample along the section being the point of the profile along the arc,
/// with no nearest-point search: which station a point belongs to is known, and a search would
/// let a point that had slipped to a neighbouring section read as correct.
///
/// The canal at a turning station is the pair of cylinders the two edges' rolling balls sweep,
/// whose axes cross at a corner, so the reading is the distance to the NEARER of the two axes and
/// not to the centre segment: a section is the ellipse the two cut, and measuring it against a
/// circle about one centre reads the stretch as an error (up to 3e-4 on a forty sided bore).
/// Where the chain does not turn there is no corner, the two coincide, and the centre segment is
/// the axis.
pub fn worst_reach_off_the_canal(run: &SectionRun, pool: &SectionPool, radius: f64) -> f64 {
    let mut worst = 0.0_f64;
    let edge_count = run.edges.len();

    for i in 0..run.sections.len() {
        let section = &run.sections[i];

        let has_a_turn = run.is_closed || (i > 0 && i < run.sections.len() - 1);
        let arriving = &run.edges[if has_a_turn {
            (i + edge_count - 1) % edge_count
        } else {
            0
        }];
        let leaving = &run.edges[i.min(edge_count - 1)];

        let corner = if has_a_turn {
            try_the_canals_corner(arriving, leaving, radius, section.tangent).map(|c| c.0)
        } else {
            None
        };

        let last = section.profile.len() - 1;

        for (k, &sample) in section.profile.iter().enumerate() {
            let along = k as f64 / section.segments() as f64;
            let centre = section.centre_from + ((section.centre_to - section.centre_from) * along);
            let at = pool.position_of(sample);

            // The two BOUNDARY samples are the boundary rule's, not the profile's: they are set
            // back from the edge line rather than from where two stations' tangent lines cross,
            // so they stand a radius from their own centre and up to 8.5e-4 off the canal on a
            // raked wall. They keep the centre-segment reading until TryBoundary crosses them.
            if let Some(corner) = corner.filter(|_| k > 0 && k < last) {
                // The boundaries' own drift along the chain rides the axis; only the arm is the
                // canal's.
                let axis_at = corner + (section.tangent * (centre - corner).dot(section.tangent));

                worst = worst.max(
                    (off_the_axis(at, axis_at, arriving.direction()) - radius)
                        .abs()
                        .min((off_the_axis(at, axis_at, leaving.direction()) - radius).abs()),
                );

                continue;
            }

            worst = worst.max(((at - centre).length() - radius).abs());
        }
    }

    worst
}

/// How far a point stands off the line through `on` along `way`.
fn off_the_axis(at: Vector3, on: Vector3, way: Vector3) -> f64 {
    let apart = at - on;

    (apart - (way * apart.dot(way))).length()
}
