//! Port of `R/Tests/GeometryProcessingTests/BevelKnifeEdgeRimTests.cs`: a face rim whose four
//! edges are NOT all right angles - "Center Right and Left Side.mcx" reduced to the prism it is a
//! slice of. Three of the rim's edges are ordinary 90 degree box edges; the fourth is a 45 degree
//! KNIFE edge where the selected face meets a slope, and a radius 2 fillet there spans 135 degrees
//! and touches down `R / tan(22.5) = 4.83` from the edge instead of 2.
//!
//! The sections are measured by slicing the cut part with planes square to the edge being checked
//! and asking every point of the cut surface how far it is from the arc that edge is supposed to
//! carry (`knife_edge_rim_measures`). The same measurement is run over a plain box rim in
//! `a_box_face_rim_of_four_right_angles_is_exact`, so a failure here is the bevel and not the
//! ruler. The rim is cut both as a rebuild's swept tools do it and by the level line
//! (`band_gap_entry_points::build_pick` and `gap_fill::fill`).

use std::collections::HashMap;
use std::rc::Rc;

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::bevel_group_tools::{self, BevelGroupToolsOptions};
use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::gap_fill;
use geometry_processing::bevel::level_line::{BandGapResult, BandProfile};
use geometry_processing::topology::bevel_edge_group_resolver;
use geometry_processing::topology::{FaceGroup, FeatureEdgeSet};
use polygon_mesh::csg::CsgModes;
use polygon_mesh::{FaceList, Mesh};
use vector_math::math_helper::degrees_to_radians;
use vector_math::{Vector2, Vector3, Vector3Float};

use crate::end_face_measures::to_triangle;
use crate::kernel_cut_helpers::boolean;
use crate::knife_edge_rim_measures::{
    assert_every_section, knife_centre_y, measure_back_corner, measure_knife_edge_chamfer_section,
    measure_knife_edge_section, measure_top_edge_chamfer_section, measure_top_edge_section,
    measure_top_knife_corner, prism, r#box, show, x_min_face, SectionReading, RADIUS, SIZE,
};
use crate::sweep_mesh_measures;

const SEGMENTS: i32 = 16;

const THE_ARC: &str = "the radius 2 arc its edge carries";

/// The two right-angle edges beside the knife edge have to keep the section they would have had
/// on their own. This slices the TOP edge - the one at x = 0, z = 20, running along y - near the
/// far end (y = 3), in the middle (y = 8) and as near the knife end as a plain arc still reaches
/// (y = 14), so a blend that is only right where it starts is caught by where it has got to.
///
/// The stations stop short of both corners because the true surface near a corner is the corner
/// blend and not the plain arc: the knife's fillet is tangent 4.83 back along the x = 0 face, so
/// nothing past y = 15.17 is this edge's own section, and the far corner's own 90 degree edge
/// reaches 2 from y = 0.
#[test]
fn the_right_angle_edges_beside_the_knife_edge_keep_their_section() {
    let part = prism();

    assert_the_rim_is_what_we_think(&part);

    let cut = round_the_x_min_rim(&part, true);

    assert_every_section(
        [3.0, 8.0, 14.0].map(|y| measure_top_edge_section(&cut, y)),
        THE_ARC,
    );
}

/// And the knife edge itself carries a radius 2 arc tangent to both of its faces through its whole
/// 135 degrees, at both ends of its length.
#[test]
fn the_knife_edge_is_rounded_through_its_whole_arc() {
    let part = prism();

    assert_the_rim_is_what_we_think(&part);

    let cut = round_the_x_min_rim(&part, true);

    assert_every_section(
        [5.0, 15.0].map(|z| measure_knife_edge_section(&cut, z)),
        THE_ARC,
    );
}

/// The same rim CHAMFERED, which is the same claim with the roundness taken out of it: the two
/// right angle edges beside the knife have to keep their own 45 degree chamfer plane.
///
/// A chamfer reaches `RADIUS` along each face whatever the dihedral is, so the two segments'
/// tangent distances are IDENTICAL here while their sections are not - the 90 degree edge's chamfer
/// is a chord of a right angle and the knife's is a chord of 135 degrees. That is why the mitre fit
/// compares the profiles themselves rather than their reach; measured on tangent distance this
/// joint reads as no change of section at all and the averaged ring goes through unnoticed.
#[test]
fn the_right_angle_edges_beside_the_knife_edge_keep_their_chamfer() {
    let part = prism();

    assert_the_rim_is_what_we_think(&part);

    let cut = round_the_x_min_rim(&part, false);

    assert_every_section(
        [3.0, 8.0, 14.0].map(|y| measure_top_edge_chamfer_section(&cut, y)),
        "the 45 degree chamfer plane its edge carries",
    );
}

/// And the chamfered knife edge carries its OWN chamfer - the chord of 135 degrees, 2 along the
/// x = 0 face and 2 along the slope - rather than the 90 degree one averaged in from its
/// neighbours.
#[test]
fn the_knife_edge_carries_its_own_chamfer() {
    let part = prism();

    assert_the_rim_is_what_we_think(&part);

    let cut = round_the_x_min_rim(&part, false);

    assert_every_section(
        [5.0, 15.0].map(|z| measure_knife_edge_chamfer_section(&cut, z)),
        "the chamfer chord of its own 135 degrees",
    );
}

/// The control: the same rim, the same slices, the same arc - on a box, where all four edges are
/// 90 degrees. This passes, which is what says the two tests above are measuring the bevel and not
/// the measurement.
#[test]
fn a_box_face_rim_of_four_right_angles_is_exact() {
    let part = r#box();
    let cut = round_the_x_min_rim(&part, true);

    assert_every_section(
        [3.0, 10.0, 17.0].map(|y| measure_top_edge_section(&cut, y)),
        THE_ARC,
    );
}

/// At the rim's step corners the knife piece, whose setback (4.83) is the larger, carries its band
/// past the step onto the top piece's side faces (the top and bottom faces), out to its own reach:
/// the band's boundary there runs on the top face beyond the top round's rail at x = 2, round the
/// corner at the knife's reach, to the slope. In the control the knife round cuts the top face
/// there, and the corner hole's rim must lie on the two rounds' mitred solid for
/// `CornerMitrePatch.OnTheMitre` to place the filler (it declined on (2, 18, 20), 0.8284 outside
/// the knife's round, while the band stopped at the step).
#[test]
fn the_knifes_band_carries_past_the_step_onto_the_top_face() {
    let gap = band_of_the_x_min_rim(&prism(), RADIUS, false);
    assert!(gap.built(), "the rim's band: {:?}", gap.refusal);

    // The kept boundary on the top (z = 20) and bottom (z = 0) faces near the knife: the top
    // round's rail at x = 2 no longer runs on to the slope crease at (2, 18) - that point lies in
    // the knife round's removed sector, 0.8284 outside it - but meets the knife's own cut short of
    // it, and no boundary vertex on those faces stands inside the knife round's removed sector at
    // all.
    let knife_centre = Vector2::new(2.0, knife_centre_y());
    let mesh = gap
        .mesh
        .as_ref()
        .expect("a built gap has its mesh")
        .borrow();
    for z in [20.0, 0.0] {
        let on_the_face: Vec<Vector3> = gap
            .loops
            .iter()
            .flat_map(|l| l.vertices.iter().map(|&v| mesh.position(v)))
            .filter(|p| (p.z - z).abs() < 1e-6)
            .collect();
        let removed: Vec<String> = on_the_face
            .iter()
            .filter(|p| {
                let d = Vector2::new(p.x, p.y) - knife_centre;
                d.y >= 0.0 && d.y >= d.x && d.length() - RADIUS > 1e-3
            })
            .map(|&p| show(p))
            .collect();
        assert!(
            removed.is_empty(),
            "boundary on z = {z} inside the knife round's removed sector: {}",
            removed.join(" ")
        );
        let cut_by_the_knife = on_the_face
            .iter()
            .filter(|p| {
                (p.x - 2.0).abs() < 1e-6
                    && p.y > knife_centre_y()
                    && p.y < knife_centre_y() + RADIUS
            })
            .count();
        assert!(
            cut_by_the_knife > 0,
            "the top round's rail on z = {z} meets the knife's cut"
        );
    }
}

#[test]
fn the_level_line_rim_keeps_every_section_and_the_step_corner() {
    // The knife piece is carried past its step corners (StepCornerCarry) and the corner hole is
    // laid on the mitre of the two rounds (CornerMitrePatch.OnTheMitre). Without the carry the
    // corner stood 0.209 outside the knife's round: the hole's rim ran out to (2, 18, 20), 0.8284
    // outside it, and the patch declined.
    let cut = round_the_x_min_rim_on_the_level_line(&prism(), false);

    let readings: Vec<SectionReading> = [3.0, 8.0, 14.0]
        .into_iter()
        .map(|y| measure_top_edge_section(&cut, y))
        .chain([5.0, 15.0].map(|z| measure_knife_edge_section(&cut, z)))
        .chain([measure_top_knife_corner(&cut)])
        .collect();
    assert_every_section(
        readings,
        "the radius 2 arcs, and at the step corner their two rounds meeting",
    );
}

/// The rim's two right-angle corners at the back face (y = 0), where the top and bottom edges turn
/// into the back edge inside one piece of the rim (no step there, so no corner hole): each is the
/// mitre of its two radius 2 rounds. Read like `measure_top_knife_corner`, every vertex in the
/// corner box against the larger of the two rounds' excesses, each in its own wedge. Walked both
/// ways round: the x = 0 face's diagonal (y + z = 20) passes through the top-back joint's inside
/// corner (0, 2, 18) but not the bottom-back's (0, 2, 2), and until SharpJointCorners made that
/// corner a vertex the cut chorded it and the bottom-back read 0.355 walked one way, 0 the other.
fn the_right_angle_corners_are_the_mitres_of_their_rounds(reverse: bool) {
    let cut = round_the_x_min_rim_on_the_level_line(&prism(), reverse);
    assert_every_section(
        [18.0, 2.0].map(|z| measure_back_corner(&cut, z)),
        "the mitre of its two rounds",
    );
}

#[test]
fn the_right_angle_corners_are_the_mitres_of_their_rounds_false() {
    the_right_angle_corners_are_the_mitres_of_their_rounds(false);
}

#[test]
fn the_right_angle_corners_are_the_mitres_of_their_rounds_true() {
    the_right_angle_corners_are_the_mitres_of_their_rounds(true);
}

/// The control's side of the same corners: every point of the two rounds' mitre crease at each back
/// corner lies on the rebuild. The crease is where the bottom (or top) round about (x 2, z 2) and
/// the back round about (x 2, y 2) meet, on the bisector plane y = z (y = 20 - z at the top).
///
/// The corner strip drew the mitre spoke - inside corner (0, 2, 2) to outer corner (2, 0, 0) - as
/// the bottom round's arc slid linearly across, and the crease stood 0.27 off the rebuild at
/// (0.89, 0.34, 0.34); the rebuild's own vertices all lay on the rounds, so only this direction saw
/// it. Center Right and Left Side read the same 0.274 at its two back corners. Drawn on the seam
/// (FilletArc.OnARimJointsSeam) it is 0.003.
fn the_mitre_crease_of_a_right_angle_corner_is_on_the_rebuild(reverse: bool) {
    let cut = round_the_x_min_rim_on_the_level_line(&prism(), reverse);
    let at: Vec<Vector3> = cut.vertices.iter().map(|p| p.as_vector3()).collect();
    let triangles: Vec<(i32, i32, i32)> = cut.faces.iter().map(|f| (f.v0, f.v1, f.v2)).collect();
    let mut readings = Vec::new();
    for bottom in [true, false] {
        let mut worst = 0.0;
        let mut worst_at = Vector3::ZERO;
        for i in 1..40 {
            let phi = std::f64::consts::PI / 2.0 * f64::from(i) / 40.0;
            let across = 2.0 * phi.sin();
            let p = Vector3::new(
                2.0 - (2.0 * phi.cos()),
                2.0 - across,
                if bottom {
                    2.0 - across
                } else {
                    SIZE - 2.0 + across
                },
            );
            let d = triangles
                .iter()
                .map(|t| to_triangle(p, at[t.0 as usize], at[t.1 as usize], at[t.2 as usize]))
                .fold(f64::INFINITY, f64::min);
            if d > worst {
                worst = d;
                worst_at = p;
            }
        }

        readings.push(SectionReading {
            worst,
            report: format!(
                "the {}-back crease is {worst:.3} off the rebuild at {}",
                if bottom { "bottom" } else { "top" },
                show(worst_at)
            ),
        });
    }

    assert_every_section(readings, "on the rebuild");
}

#[test]
fn the_mitre_crease_of_a_right_angle_corner_is_on_the_rebuild_false() {
    the_mitre_crease_of_a_right_angle_corner_is_on_the_rebuild(false);
}

#[test]
fn the_mitre_crease_of_a_right_angle_corner_is_on_the_rebuild_true() {
    the_mitre_crease_of_a_right_angle_corner_is_on_the_rebuild(true);
}

/// The same rim at 0.5 builds closed and does not cross itself with the step corners carried and
/// mitred. It folded at the bottom step corner (edge 310-258 piercing 311 272 257) while
/// CornerMitrePatch placed a crease point onto one round only, 0.086 off the crease; Center Flows
/// Through f3 at 0.05 (StepCornerRimTests) folded the same way.
fn the_mitred_step_corner_does_not_cross_itself_at_small_sizes(size: f64) {
    let part = prism();
    let gap = band_of_the_x_min_rim(&part, size, false);
    assert!(gap.built(), "the rim's band: {:?}", gap.refusal);
    let filled = gap_fill::fill(&gap, BandProfile::Fillet, SEGMENTS).expect("the fill runs");
    assert!(filled.filled(), "the rim's fill: {:?}", filled.refusal);
    let mesh = filled
        .mesh
        .as_ref()
        .expect("a filled gap has its mesh")
        .borrow();
    let kept = sweep_mesh_measures::kept(&mesh);
    assert!(sweep_mesh_measures::is_closed_and_consistent(
        &mesh.vertices,
        &kept
    ));
    let source_positions: Vec<Vector3> = part.vertices.iter().map(|p| p.as_vector3()).collect();
    let source_indices: Vec<(i32, i32, i32)> =
        part.faces.iter().map(|f| (f.v0, f.v1, f.v2)).collect();
    assert!(
        !sweep_mesh_measures::intersects_itself_where_the_source_did_not(
            &mesh.vertices,
            &kept,
            &source_positions,
            &source_indices
        )
    );
}

#[test]
fn the_mitred_step_corner_does_not_cross_itself_at_small_sizes_0_5() {
    the_mitred_step_corner_does_not_cross_itself_at_small_sizes(0.5);
}

/// The x = 0 face's rim rounded by the level line, as the app's surface route would.
fn round_the_x_min_rim_on_the_level_line(part: &Mesh, reverse: bool) -> Mesh {
    let gap = band_of_the_x_min_rim(part, RADIUS, reverse);
    assert!(gap.built(), "the rim's band: {:?}", gap.refusal);
    let filled = gap_fill::fill(&gap, BandProfile::Fillet, SEGMENTS).expect("the fill runs");
    assert!(filled.filled(), "the rim's fill: {:?}", filled.refusal);

    let mesh = filled
        .mesh
        .as_ref()
        .expect("a filled gap has its mesh")
        .borrow();
    let vertices: Vec<Vector3Float> = mesh
        .vertices
        .iter()
        .map(|&v| Vector3Float::from_vector3(v))
        .collect();
    let kept: Vec<i32> = sweep_mesh_measures::kept(&mesh)
        .into_iter()
        .flat_map(|(a, b, c)| [a, b, c])
        .collect();
    Mesh::from_vertices_and_faces(&vertices, &FaceList::from_int_array(&kept, &vertices))
}

/// The level line's band cut for the x = 0 face's rim, before any fill.
fn band_of_the_x_min_rim(part: &Mesh, size: f64, reverse: bool) -> BandGapResult {
    let positions: Rc<Vec<Vector3>> =
        Rc::new(part.vertices.iter().map(|p| p.as_vector3()).collect());
    let indices: Rc<Vec<(i32, i32, i32)>> =
        Rc::new(part.faces.iter().map(|f| (f.v0, f.v1, f.v2)).collect());
    let mut standing: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
    for (i, t) in indices.iter().enumerate() {
        for (a, b) in [(t.0, t.1), (t.1, t.2), (t.2, t.0)] {
            let key = if a < b { (a, b) } else { (b, a) };
            standing.entry(key).or_default().push(i);
        }
    }

    let is_feature_edge = {
        let positions = Rc::clone(&positions);
        let indices = Rc::clone(&indices);
        let normal_of = move |t: (i32, i32, i32)| {
            (positions[t.1 as usize] - positions[t.0 as usize])
                .cross(positions[t.2 as usize] - positions[t.0 as usize])
                .get_normal()
        };
        move |a: i32, b: i32| {
            standing
                .get(&if a < b { (a, b) } else { (b, a) })
                .is_some_and(|on| {
                    on.len() == 2
                        && normal_of(indices[on[0]]).dot(normal_of(indices[on[1]]))
                            < degrees_to_radians(1.0).cos()
                })
        }
    };

    // The rim: the feature edges of the x = 0 face, walked round.
    let on_face: Vec<usize> = (0..indices.len())
        .filter(|&i| {
            let t = indices[i];
            [t.0, t.1, t.2]
                .iter()
                .all(|&v| positions[v as usize].x.abs() < 1e-4)
        })
        .collect();
    let rim_edges: Vec<(i32, i32)> = on_face
        .iter()
        .flat_map(|&i| {
            let t = indices[i];
            [(t.0, t.1), (t.1, t.2), (t.2, t.0)]
        })
        .filter(|e| is_feature_edge(e.0, e.1))
        .collect();
    let mut chain = vec![rim_edges[0].0];
    while chain.len() < rim_edges.len() {
        let last = *chain.last().expect("the chain has its start");
        chain.push(
            rim_edges
                .iter()
                .find(|e| e.0 == last)
                .expect("the rim walks on")
                .1,
        );
    }

    if reverse {
        chain.reverse();
    }

    band_gap_entry_points::build_pick(
        &positions,
        &indices,
        &[chain],
        &[true],
        Some(Rc::new(is_feature_edge)),
        size,
        BandProfile::Fillet,
        &BandGapBuildOptions {
            chord_tolerance: 0.0,
            snap: 1e-9,
            known_room: None,
            seam_epsilon: 0.0,
            sharp_turn_degrees: 10.0,
            fillet_segments: SEGMENTS,
            ..BandGapBuildOptions::default()
        },
    )
    .expect("the pick is well formed")
}

/// The rim really is three right angles and one 45 degree knife - if that stopped being true the
/// tests above would be measuring some other part's bevel.
fn assert_the_rim_is_what_we_think(part: &Mesh) {
    let face_group = FaceGroup::create_with_graph(
        part,
        &part.get_mesh_edge_graph(),
        x_min_face(part),
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("the x = 0 face groups");

    assert_eq!(
        face_group.boundary_edges().len(),
        4,
        "the x = 0 face of the prism is bounded by four edges"
    );

    for edge in face_group.boundary_edges() {
        // The knife edge is the one running up the corner at (0, 20), where the face meets the
        // slope.
        let on_the_knife = (edge.start.y - SIZE).abs() < 1e-6 && (edge.end.y - SIZE).abs() < 1e-6;
        let expected = if on_the_knife { 45.0 } else { 90.0 };

        assert!(
            (edge.dihedral_angle_degrees() - expected).abs() <= 0.01,
            "the edge from {} to {} is {}",
            show(edge.start),
            show(edge.end),
            if on_the_knife {
                "where the face meets the 45 degree slope"
            } else {
                "an ordinary box edge"
            }
        );
    }
}

/// Bevels the rim of the x = 0 face exactly as a rebuild does - face group, resolve, link, sweep
/// one tool per run, union them and subtract - and hands back the cut part.
fn round_the_x_min_rim(part: &Mesh, round_profile: bool) -> Mesh {
    let features = FeatureEdgeSet::create(part);
    let face_group = FaceGroup::create_with_graph(
        part,
        &part.get_mesh_edge_graph(),
        x_min_face(part),
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("the x = 0 face groups");
    let group = bevel_edge_group_resolver::create_from_face_group(
        &face_group,
        &face_group.boundary_edges()[0],
        features.flat_angle_degrees(),
    )
    .expect("the rim makes a group");
    let resolution = bevel_edge_group_resolver::resolve(&group, part);

    assert_eq!(
        resolution.edges.len(),
        4,
        "the rim resolves to its own four edges"
    );

    let chains = bevel_chain_linker::link(&resolution.edges);

    assert_eq!(chains.len(), 1, "a face rim is one run");

    assert_eq!(chains[0].edges.len(), 4, "with all four of its edges in it");

    assert!(chains[0].is_closed, "and it comes back to where it started");

    let tools = match bevel_group_tools::try_build(
        &resolution,
        round_profile,
        RADIUS,
        SEGMENTS,
        BevelGroupToolsOptions::default(),
    ) {
        Ok(built) => built.tools,
        Err(failure) => panic!(
            "the rim has to produce a tool - {:?}",
            failure.reason.code()
        ),
    };

    let solids: Vec<Mesh> = tools.into_iter().map(|tool| tool.mesh).collect();
    // The tools have to union, and the cutter has to come out of the part: the shared boolean
    // panics where the kernel refuses either (C#'s null).
    let cutter = if solids.len() == 1 {
        solids[0].clone()
    } else {
        boolean(&solids.iter().collect::<Vec<_>>(), CsgModes::Union)
    };

    boolean(&[part, &cutter], CsgModes::Subtract)
}
