//! The bore mouth tests, RAKED half: a 1.5 radius bore drilled at 45 degrees through a 3 thick
//! plate (`bore_test_meshes::create_plate_with_a_raked_bore`), and everything measured on its
//! mouth's walkback (`surface_face_walkback`) - the wall left unrolled, a run of tangent line on
//! every chord, each strip finding its run (`surface_seam_over_the_edge`) and the mitre point read
//! on the shared face whose lines cross (`surface_mitre_seam`).
//!
//! Also holds the class's shared constants and the raked mouth's walkback
//! ([`the_raked_mouth_walked_back`]), which the other raked-bore tests read. The straight bore's
//! half is `surface_bore_mouth_tests.rs`, which also holds this half's
//! build-side readings (`AssertNothingStandsInTheBore`,
//! `AssertEveryRakedBlendVertexStandsARadiusFromItsOwnCentreLine` and the raked mouth's edges).

use std::rc::Rc;
use std::sync::Arc;

use solidmesh::bevel::surface::{
    surface_face_walkback, surface_mitre_seam, surface_plan_band_ends, surface_seam_over_the_edge,
    surface_shared_edge_network, SurfaceBevelPlan, SurfaceLine, SurfaceStrip, SurfaceVertexPool,
    SurfaceWalkbackResult,
};
use solidmesh::topology::{
    bevel_edge_group_resolver, BevelSelectionKind, FaceGroup, FaceRegionCache, FeatureEdgeSet,
};
use solidmesh_math::{math_helper, Vector3};
use solidmesh_mesh::Mesh;

use crate::bore_test_meshes;
use crate::mesh_measurements;
use crate::surface_test_chains;
use solidmesh::bevel::surface::surface_mesh_closure;

/// How many facets round, on the bore and on the outside alike.
pub const SIDES: i32 = 40;

/// The round's radius.
pub const SIZE: f64 = 0.4;

/// Chording of the strip, per quarter turn.
pub const SEGMENTS: i32 = 8;

/// The bend threshold the raked mouth is read at, under which the faceted wall is one surface.
pub const BEND_THRESHOLD_DEGREES: f64 = 10.0;

/// A place named on the mesh is three floats.
pub const AS_NEAR_AS_A_FLOAT_HOLDS: f64 = 1e-5;

/// The raked fixture's bore: a 1.5 radius drilled at 45 degrees through a 3 thick plate.
pub const RAKED_RADIUS: f64 = 1.5;

pub const RAKED_THICKNESS: f64 = 3.0;

pub const RAKE_DEGREES: f64 = 45.0;

/// The drill's own direction, leaning 45 degrees off the plate's normal in the x z plane.
pub fn raked_axis() -> Vector3 {
    let rake = math_helper::degrees_to_radians(RAKE_DEGREES);

    Vector3::new(rake.sin(), 0.0, rake.cos())
}

/// The raked part at this file's dimensions, with the fixture's default land of 6.
pub fn the_raked_part() -> Mesh {
    bore_test_meshes::create_plate_with_a_raked_bore(
        SIDES,
        RAKED_RADIUS,
        RAKED_THICKNESS,
        RAKE_DEGREES,
        6.0,
    )
}

/// The fixture itself, before any bevel: a closed solid whose bore takes exactly its own section
/// times the slant length away. Pinned because everything below is measured against a hand built
/// mesh with no boolean under it, so a fault in the mesh would read as a fault in the bevel.
#[test]
fn a_raked_bore_is_drilled_clean_through_the_plate() {
    let part = the_raked_part();

    let (open, open_at) = surface_mesh_closure::open_edge_count(&part);
    assert_eq!(
        open, 0,
        "the two faces, the bore and the outside close on each other, and they do not at {open_at:?}"
    );

    assert!(part.is_manifold());
    assert_eq!(mesh_measurements::shell_count(&part), 1);

    let rake = math_helper::degrees_to_radians(RAKE_DEGREES);
    let outer_radius = (RAKED_RADIUS / rake.cos()) + ((RAKED_THICKNESS / 2.0) * rake.tan()) + 6.0;
    let step = (2.0 * std::f64::consts::PI / f64::from(SIDES)).sin() / 2.0;
    let sides = f64::from(SIDES);

    let expected = (sides * step * outer_radius * outer_radius * RAKED_THICKNESS)
        - (sides * step * RAKED_RADIUS * RAKED_RADIUS * RAKED_THICKNESS / rake.cos());
    let volume = mesh_measurements::signed_volume(&part);

    assert!(
        (volume - expected).abs() <= 1e-3,
        "a prism drilled through a plate takes its own section times the slant length - {volume} against {expected}"
    );
}

/// The raked mouth's wall is NOT rolled along, and so its wall-side tangent line lies in the plane
/// of the facet its own chord rims and the walkback leaves a run on every one of the forty.
///
/// The rolling ball keeps to its own chord's facet over the whole of that strip's own extent; a
/// section square to a chord of a LEANED bore leaves that facet's span without leaving its PLANE,
/// and `SurfaceWallStandsClear` asks the ball's own question before the roll runs.
// `!(z > 0.9)` on purpose: a NaN normal counts as the wall.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
#[test]
fn the_raked_mouths_wall_is_not_rolled_and_every_chord_leaves_a_run() {
    let (walked, plan) = the_raked_mouth_walked_back();

    let on_the_line = SurfaceVertexPool::near_enough_for(&walked.mesh.vertices);
    let axis = raked_axis();

    let mut rolled = Vec::new();
    let mut off_its_own_facet = Vec::new();
    let mut with_no_run = Vec::new();
    let mut worst_off = 0.0_f64;
    let mut biggest_step = 0.0_f64;

    let count = plan.edges.len();

    for i in 0..count {
        let edge = &plan.edges[i];

        // The wall is whichever of the two faces is NOT the plate's top, which is the one the
        // mouth turns on and the only one whose normal stands up.
        let wall_is_left = !(edge.edge.left_normal().z > 0.9);
        let line = if wall_is_left {
            &edge.left_tangent_line
        } else {
            &edge.right_tangent_line
        };

        if edge.wall_section.is_some() {
            rolled.push(i);
        }

        // The chord's own facet's plane, taken off the chord and the drill's axis rather than off
        // the roll's walk, since there is no walk any more.
        let chord = (edge.to() - edge.from()).get_normal();
        let mut facet_normal = chord.cross(axis).get_normal();
        let middle = (edge.from() + edge.to()) / 2.0;

        if facet_normal.dot(middle - (axis * middle.dot(axis))) < 0.0 {
            facet_normal = -facet_normal;
        }

        let off = (line.point - middle).dot(facet_normal).abs();

        if off > on_the_line {
            off_its_own_facet.push(i);
        }

        worst_off = worst_off.max(off);

        let has_run = walked
            .tangent_runs
            .get(&(i as i32, wall_is_left))
            .is_some_and(|runs| !runs.is_empty());
        if !has_run {
            with_no_run.push(i);
        }

        let next = &plan.edges[(i + 1) % count];
        let next_wall_is_left = !(next.edge.left_normal().z > 0.9);

        biggest_step = biggest_step.max(
            ((if next_wall_is_left {
                next.left_setback
            } else {
                next.right_setback
            }) - (if wall_is_left {
                edge.left_setback
            } else {
                edge.right_setback
            }))
            .abs(),
        );
    }

    assert!(
        rolled.is_empty(),
        "no chord of a leaned bore's mouth is rolled along its wall - instead {} are",
        rolled.len()
    );

    assert!(
        off_its_own_facet.is_empty(),
        "every chord's wall tangent line lies in its own facet's plane - instead {} do not, the \
         worst by {worst_off}",
        off_its_own_facet.len()
    );

    assert!(
        with_no_run.is_empty(),
        "so the walkback's cut stands on that line and leaves a run for the strip to sew to - \
         instead {} chords are offered nothing",
        with_no_run.len()
    );

    assert!(
        (biggest_step - 0.074151).abs() <= 1e-6,
        "the setback still steps this far between neighbouring chords - 0.074151 unrolled - which \
         is the mitre's business and was never what emptied the runs - instead {biggest_step}"
    );
}

/// The band the walkback takes off the TOP face over each of the forty chords: one run of the
/// tangent line per chord, stretching between that chord's own two mitre points.
#[test]
fn the_raked_mouths_bands_are_walked_back_over_their_own_chords() {
    let (walked, plan) = the_raked_mouth_walked_back();

    let mut missing = Vec::new();

    for i in 0..plan.edges.len() {
        let edge = &plan.edges[i];
        let left = edge.edge.left_normal().z > 0.9;
        let line = if left {
            &edge.left_tangent_line
        } else {
            &edge.right_tangent_line
        };
        let ends = plan.band_ends_of(i, left);

        let runs = walked.tangent_runs.get(&(i as i32, left));
        let Some(runs) = runs.filter(|runs| runs.len() == 1 && runs[0].len() >= 2) else {
            missing.push(format!(
                "edge {i} has {} runs on the top face",
                runs.map_or(0, |runs| runs.len())
            ));
            continue;
        };

        // The band ends at a mitre stand ON this edge's own tangent line, so the run the walkback
        // leaves is the stretch between them, read on the pool's own ruler - not to the float:
        // the mitre plane's crossing of a facet edge and each bead's tangent line's are three
        // readings of one place a few ten thousandths apart, all one pooled point.
        let one_pooled_point = SurfaceVertexPool::near_enough_for(&walked.mesh.vertices);

        let mut wanted: Vec<f64> = ends.iter().map(|end| along(line, end.point)).collect();
        wanted.sort_by(f64::total_cmp);

        let mut run: Vec<f64> = runs[0]
            .iter()
            .map(|&vertex| {
                along(
                    line,
                    Vector3::from_vector3_float(walked.mesh.vertices[vertex as usize]),
                )
            })
            .collect();
        run.sort_by(f64::total_cmp);

        if wanted.len() != 2
            || (run[0] - wanted[0]).abs() > one_pooled_point
            || (run[run.len() - 1] - wanted[1]).abs() > one_pooled_point
        {
            let wanted_text: Vec<String> = wanted.iter().map(|at| format!("{at:.4}")).collect();
            missing.push(format!(
                "edge {i} runs {:.4}..{:.4} where its band ends are {}",
                run[0],
                run[run.len() - 1],
                wanted_text.join("..")
            ));
        }
    }

    assert_eq!(
        missing.len(),
        0,
        "every chord's band is taken off the stretch of face its own mitre points bound - instead \
         {} of {} are not: {}",
        missing.len(),
        plan.edges.len(),
        missing
            .iter()
            .take(4)
            .cloned()
            .collect::<Vec<_>>()
            .join("; ")
    );
}

/// And that each chord's strip FINDS the run its own band left: the window the band's own mitre
/// planes cut off the strip's outermost rail is the right one, and it is the very stretch the
/// strip will be trimmed to.
#[test]
fn every_chords_strip_finds_the_run_its_own_band_left() {
    let (walked, plan) = the_raked_mouth_walked_back();

    // The builder's own pool over the walked mesh, which is what names the run's vertices.
    let pool = SurfaceVertexPool::new(&walked.mesh.vertices);

    let mut missed = Vec::new();

    for i in 0..plan.edges.len() {
        let edge = &plan.edges[i];
        let left = edge.edge.left_normal().z > 0.9;

        let strip = SurfaceStrip::try_create(edge, SEGMENTS)
            .unwrap_or_else(|why| panic!("edge {i} has no strip to sew: {why}"));

        let Some(runs) = walked.tangent_runs.get(&(i as i32, left)) else {
            // No entry at all is a different fault from an entry with no run in it.
            missed.push(format!(
                "edge {i} has no walkback entry on the face it turns on"
            ));
            continue;
        };

        let ends = plan.band_ends_of(i, left);

        if let Err(refused_at) = surface_seam_over_the_edge::try_pick_the_run_over_the_edge(
            &pool,
            &strip,
            edge,
            Some(runs),
            Some(&ends),
            left,
        ) {
            missed.push(format!("edge {i} refused at {refused_at:?}"));
        }
    }

    assert_eq!(
        missed.len(),
        0,
        "the run the walkback left over each chord is the one its strip sews to - instead {} of \
         {} are refused: {}",
        missed.len(),
        plan.edges.len(),
        missed
            .iter()
            .take(4)
            .cloned()
            .collect::<Vec<_>>()
            .join("; ")
    );
}

/// Which of two SHARED faces the mitre point is read on. A hole mouth's chords share both their
/// regions - the flat face they turn on and the bore's own wall, at this bend threshold one smooth
/// surface - and only one of the two pairs of lines crosses: on the faceted wall each chord's
/// tangent line lies in its own facet's plane, so neighbouring chords' lines are skew. It also
/// pins the TIE-BREAK at the corners where both shared faces have a crossing:
/// `SurfaceMitreSeam.TryThePoint` takes the smaller miss.
#[test]
fn the_mitre_point_is_read_on_the_shared_face_whose_lines_actually_cross() {
    let (_, plan) = the_raked_mouth_walked_back();

    let mut walls_that_miss = 0;
    let count = plan.edges.len();

    for i in 0..count {
        let here = &plan.edges[i];
        let next = &plan.edges[(i + 1) % count];

        assert!(
            Arc::ptr_eq(&here.left_region, &next.left_region),
            "the flat face is shared all the way round the mouth, and at {i} it is not"
        );
        assert!(
            Arc::ptr_eq(&here.right_region, &next.right_region),
            "and so is the wall, at this threshold one smooth surface - not at {i}"
        );

        // The plan's own ruler for this question, asked of production rather than restated here.
        let near_enough = surface_plan_band_ends::crosses_within(here, next);

        if here
            .right_tangent_line
            .try_cross(&next.right_tangent_line, near_enough)
            .is_none()
        {
            walls_that_miss += 1;
        }

        let on_the_flat = here
            .left_tangent_line
            .try_cross(&next.left_tangent_line, near_enough)
            .unwrap_or_else(|| {
                panic!(
                    "the two chords' lines on the face they turn on lie in one plane, and at {i} \
                     they do not meet"
                )
            });

        let read =
            surface_mitre_seam::try_the_point(here, next, near_enough).unwrap_or_else(|| {
                panic!("one of the two shared faces has a crossing to read, at corner {i}")
            });

        assert!(
            (read - on_the_flat).length() < AS_NEAR_AS_A_FLOAT_HOLDS,
            "and it is the flat face's, at corner {i} - instead {read:?} against {on_the_flat:?}"
        );
    }

    // Thirty eight of the forty, measured: at two corners of the loop the two wall lines do
    // happen to meet. Held from below rather than exactly, because what it is here for is to say
    // that this fixture really does put the answer on the second face it looks at.
    assert!(
        walls_that_miss >= 38,
        "the wall is the shared face whose lines do NOT cross, nearly all the way round - instead \
         at {walls_that_miss} of {count}, which would make this fixture no measurement of the \
         choice at all"
    );
}

/// How far along a line a point stands, measured from the line's own point.
pub fn along(line: &SurfaceLine, at: Vector3) -> f64 {
    (at - line.point).dot(line.direction)
}

/// The raked mouth planned and walked back, by the build's own steps up to the walkback - which is
/// as far as the band reading goes, and short of the strips its runs are sewn to.
pub fn the_raked_mouth_walked_back() -> (SurfaceWalkbackResult, Rc<SurfaceBevelPlan>) {
    a_bores_mouth_walked_back(RAKE_DEGREES)
}

/// A bore of the raked
/// fixture's own size, drilled at the rake asked for, planned and walked back by the build's own
/// steps. At a rake of zero it is the straight control; at [`RAKE_DEGREES`] it is
/// [`the_raked_mouth_walked_back`], whose seed edge (the first on the upper mouth) is the same one.
pub fn a_bores_mouth_walked_back(
    rake_degrees: f64,
) -> (SurfaceWalkbackResult, Rc<SurfaceBevelPlan>) {
    let rake = math_helper::degrees_to_radians(rake_degrees);
    let axis = Vector3::new(rake.sin(), 0.0, rake.cos());

    let part = Arc::new(bore_test_meshes::create_plate_with_a_raked_bore(
        SIDES,
        RAKED_RADIUS,
        RAKED_THICKNESS,
        rake_degrees,
        6.0,
    ));
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(&part, &graph, BEND_THRESHOLD_DEGREES);

    let mouth = (0..features.count())
        .find(|&i| {
            on_a_mouth_of(features.edges()[i].start, axis)
                && on_a_mouth_of(features.edges()[i].end, axis)
        })
        .map_or(-1, |i| i as i32);

    let group = bevel_edge_group_resolver::create_from_edge(
        &features,
        mouth,
        BevelSelectionKind::TangentChain,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("the mouth's edge is in the set");

    let (resolutions, _) = bevel_edge_group_resolver::resolve_all(&[group], &part);
    let regions = Arc::new(FaceRegionCache::new(
        part.clone(),
        Some(graph.clone()),
        resolutions[0].features.flat_angle_degrees(),
    ));
    let curve_regions = Arc::new(FaceRegionCache::new(
        part.clone(),
        Some(graph.clone()),
        resolutions[0].group.curve_tolerance_degrees,
    ));

    let chains = surface_test_chains::join_and_open(&surface_shared_edge_network::link_together(
        &resolutions,
    ));
    assert_eq!(chains.len(), 1, "the mouth is one chain");

    let plan = SurfaceBevelPlan::try_create_with_curve_regions(
        &chains[0],
        &regions,
        Some(&curve_regions),
        SIZE,
        true,
    )
    .unwrap_or_else(|why| panic!("the bore at {rake_degrees} degrees does not plan: {why}"));

    let walked = surface_face_walkback::try_apply_one(&part, &graph, &regions, &plan)
        .unwrap_or_else(|failure| {
            panic!("the bore at {rake_degrees} degrees does not walk back: {failure}")
        });

    (walked, plan)
}

/// On the upper mouth of a bore drilled along the given axis: on the top face, and a bore radius
/// from that axis, which the plate's outer rim is not.
pub fn on_a_mouth_of(at: Vector3, axis: Vector3) -> bool {
    (at.z - (RAKED_THICKNESS / 2.0)).abs() < 1e-6
        && ((at - (axis * at.dot(axis))).length() - RAKED_RADIUS).abs() < 1e-4
}
