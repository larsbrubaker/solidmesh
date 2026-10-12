//! Ports `GeometryProcessingTests/BevelSeamRingTests.cs`: the seam a corner patch and the runs that
//! end on it meet along - the ring of points on each ball-centre plane. Both solids have to carry
//! the SAME points there, not points a rounding apart, or the union of the operands is self
//! intersecting; so the test is EXACT equality rather than a tolerance. C#'s `[Arguments]` rows are
//! a loop, and its `[Timeout(120_000)]` is left to nextest's slow-test settings.

use std::collections::HashSet;

use geometry_processing::bevel::bevel_corner_patch::{self, BevelCornerPatchSet};
use geometry_processing::bevel::bevel_group_tools::{self, BevelGroupToolsOptions};
use geometry_processing::bevel::bevel_seam_ring::{points, snap};
use geometry_processing::bevel::{BevelAdvisory, BevelAdvisoryCode};
use geometry_processing::topology::bevel_edge_group_resolver;
use geometry_processing::topology::{
    BevelEdgeGroup, BevelSelectionKind, FaceGroup, FeatureEdge, FeatureEdgeSet,
};
use polygon_mesh::solids::platonic_solids;
use polygon_mesh::Mesh;
use vector_math::math_helper::degrees_to_radians;
use vector_math::Vector3;

use crate::kernel_cut_helpers::edge_group as group;
use crate::topology_test_meshes::{create_corner_pocket_block, create_isosceles_prism};

const RADIUS: f64 = 3.4;
const APEX_DEGREES: f64 = 45.0;
const LENGTH: f64 = 20.0;
const HEIGHT: f64 = 20.0;

/// How near a ball-centre plane a vertex has to sit to be read as being on it. The rings are laid
/// on that plane in doubles and stored as floats, so this only has to clear float noise - and the
/// next ring along is millimetres away, not microns.
const ON_THE_PLANE: f64 = 1e-4;

/// How near the ball's own surface a vertex has to sit to be part of the seam ARC rather than one
/// of the tool's clearance walls, which stand a whole size further out.
const ON_THE_BALL: f64 = 1e-3;

/// The two rings were computed by two routes: the patch slerps between the two face normals on the
/// ball's own sphere, and the run builds an arc in a 2D profile frame and sweeps it onto the plane.
/// The two agree as mathematics and disagree by about a float ulp as arithmetic - enough, at twelve
/// segments, for one vertex to survive outside Manifold's tolerance and the six-operand union to
/// come back `SelfIntersecting`.
#[test]
fn the_patch_and_its_runs_carry_the_same_points_on_every_ball_centre_plane() {
    for (fixture, segments) in [
        ("cube corner", 8),
        ("cube corner", 12),
        ("knife tip", 8),
        ("knife tip", 12),
    ] {
        let (part, picks) = part(fixture);
        let features = FeatureEdgeSet::create(&part);
        let groups: Vec<BevelEdgeGroup> = picks
            .iter()
            .map(|&(from, to)| group(&features, from, to))
            .collect();
        let (resolutions, _) = bevel_edge_group_resolver::resolve_all(&groups, &part);
        let eligible: Vec<bool> = resolutions.iter().map(|r| r.is_resolved()).collect();
        let patches = bevel_corner_patch::create(&resolutions, &eligible, true, RADIUS, segments);

        assert_eq!(
            patches.patches.len(),
            1,
            "{fixture} {segments}: the three runs meet at the one corner under test"
        );

        assert_eq!(patches.ball_centres.len(), 1);
        let centre = *patches
            .ball_centres
            .values()
            .next()
            .expect("one ball centre");
        let selected: Vec<FeatureEdge> = resolutions
            .iter()
            .flat_map(|resolution| resolution.edges.iter().copied())
            .collect();
        let drive_through: HashSet<i32> = patches.runs_may_drive_through.iter().copied().collect();

        let mut misses = Vec::new();
        let mut matched = 0;

        for resolution in &resolutions {
            let built = bevel_group_tools::try_build(
                resolution,
                true,
                RADIUS,
                segments,
                BevelGroupToolsOptions {
                    selected_across_groups: Some(&selected),
                    patched_corners: Some(&patches.ball_centres),
                    runs_may_drive_through: Some(&drive_through),
                    patch_seam_rings: Some(&patches.seam_rings),
                    ..BevelGroupToolsOptions::default()
                },
            )
            .unwrap_or_else(|refusal| panic!("{fixture} {segments}: {:?}", refusal.reason));

            assert_the_sweep_carried_the_seam(built.warnings.as_deref());

            let cutters: Vec<&Mesh> = built
                .tools
                .iter()
                .filter(|tool| !tool.is_bead)
                .map(|tool| &tool.mesh)
                .collect();
            assert_eq!(cutters.len(), 1, "one cutter per run");

            matched += sharing(
                &patches,
                cutters[0],
                centre,
                resolution.edges[0].direction,
                &mut misses,
            );
        }

        assert_shared(&misses, matched, 3 * (segments as usize + 1));
    }
}

/// The mirror: an INSIDE corner, where the patch is a fill and the runs meeting it are BEADS drawn
/// in the turned-round frame. The same seam, read from the other side of the ball.
///
/// It is the same code on both sides - one map of seam rings, chosen by the run's own convexity
/// (`BevelChainEnds.ResolveEnd`) - so what this pins is that the MIRRORED cross section still
/// presents its arc where the fill put it.
#[test]
fn the_fill_patch_and_its_beads_carry_the_same_points_on_their_ball_centre_plane() {
    for segments in [8, 12] {
        let block = create_corner_pocket_block(20.0, 10.0);
        let features = FeatureEdgeSet::create(&block);
        let corner = Vector3::new(10.0, 10.0, 10.0);

        let mut groups = Vec::new();

        for (i, edge) in features.edges().iter().enumerate() {
            if (edge.start - corner).length() < 1e-4 || (edge.end - corner).length() < 1e-4 {
                groups.push(
                    bevel_edge_group_resolver::create_from_edge(
                        &features,
                        i as i32,
                        BevelSelectionKind::Edge,
                        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
                    )
                    .expect("an edge of the set"),
                );
            }
        }

        let (resolutions, _) = bevel_edge_group_resolver::resolve_all(&groups, &block);
        let eligible: Vec<bool> = resolutions.iter().map(|r| r.is_resolved()).collect();
        let patches = bevel_corner_patch::create(&resolutions, &eligible, true, RADIUS, segments);

        assert_eq!(
            patches.patches.len(),
            1,
            "the three concave edges meet at the one inside corner under test"
        );
        assert!(
            patches.patches[0].is_fill,
            "an inside corner is under-filled, so its patch is material to ADD"
        );

        assert_eq!(patches.fill_ball_centres.len(), 1);
        let centre = *patches
            .fill_ball_centres
            .values()
            .next()
            .expect("one ball centre");
        let selected: Vec<FeatureEdge> = resolutions
            .iter()
            .flat_map(|resolution| resolution.edges.iter().copied())
            .collect();

        let mut misses = Vec::new();
        let mut matched = 0;

        for resolution in &resolutions {
            let built = bevel_group_tools::try_build(
                resolution,
                true,
                RADIUS,
                segments,
                BevelGroupToolsOptions {
                    selected_across_groups: Some(&selected),
                    filled_corners: Some(&patches.fill_ball_centres),
                    patch_seam_rings: Some(&patches.seam_rings),
                    ..BevelGroupToolsOptions::default()
                },
            )
            .unwrap_or_else(|refusal| panic!("{segments}: {:?}", refusal.reason));

            assert_the_sweep_carried_the_seam(built.warnings.as_deref());

            let beads: Vec<&Mesh> = built
                .tools
                .iter()
                .filter(|tool| tool.is_bead)
                .map(|tool| &tool.mesh)
                .collect();
            assert_eq!(beads.len(), 1, "one bead per run");

            matched += sharing(
                &patches,
                beads[0],
                centre,
                resolution.edges[0].direction,
                &mut misses,
            );
        }

        assert_shared(&misses, matched, 3 * (segments as usize + 1));
    }
}

/// The snap's own contract, which the wired tests read only through its consequences: it takes the
/// WHOLE ring or none of it, and what it takes it takes literally.
#[test]
fn the_snap_takes_the_whole_seam_or_leaves_the_ring_alone() {
    let centre = Vector3::new(3.0, -4.0, 5.0);
    let seam = points(
        centre,
        RADIUS,
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        4,
    );

    // A ring as a sweep builds one: the arc, a rounding away, with the tool's clearance points
    // around it. The order is deliberately not the seam's - which is the whole reason the match is
    // by nearness rather than by index.
    let mut ring = vec![Vector3::ZERO; seam.len() + 2];

    ring[0] = centre + Vector3::new(9.0, 9.0, 9.0);

    for i in 0..seam.len() {
        ring[seam.len() - i] = seam[i] + Vector3::new(1e-9, -1e-9, 1e-9);
    }

    let last = ring.len() - 1;
    ring[last] = centre + Vector3::new(-9.0, -9.0, -9.0);

    let mut carried = ring.clone();

    assert!(
        snap(&mut carried, &seam, RADIUS),
        "a ring that presents the whole arc, however it is ordered, carries the seam"
    );

    for i in 0..seam.len() {
        assert_eq!(
            carried[seam.len() - i],
            seam[i],
            "and it carries it LITERALLY - the published point, not the ring's own near miss"
        );
    }

    assert_eq!(carried[0], ring[0], "nothing that is not the seam is moved");

    // One point of the arc genuinely somewhere else. A ten-thousandth of the radius is a hundredth
    // of a chord and a thousand times any rounding - small enough that the tolerance this file
    // used to carry (a thousandth of the radius) would have swallowed it and moved the ring, which
    // is the thing that must not happen quietly.
    let mut wrong = ring.clone();

    wrong[2] += Vector3::new(1e-4 * RADIUS, 0.0, 0.0);

    let before = wrong.clone();

    assert!(
        !snap(&mut wrong, &seam, RADIUS),
        "a ring missing one of the seam's points is a different ring, and half a seam is worse than \
         none - the two solids would then meet along part of one polyline and part of another"
    );

    for i in 0..wrong.len() {
        assert_eq!(
            wrong[i], before[i],
            "and a refused snap leaves the ring exactly as the sweep made it"
        );
    }
}

/// The seam as the sweep itself reports it: a run handed a patch's points and unable to take them
/// says so (`BevelAdvisoryCode::CornerSeamNotCarried`), and on these corners none of them does.
/// Asserting on the ADVISORY rather than on the boolean is what makes it a test of the wiring - the
/// warning has to reach the group.
fn assert_the_sweep_carried_the_seam(said: Option<&[BevelAdvisory]>) {
    let not_carried = said.map_or(0, |said| {
        said.iter()
            .filter(|advisory| advisory.code() == BevelAdvisoryCode::CornerSeamNotCarried)
            .count()
    });

    assert_eq!(
        not_carried, 0,
        "every end here is offered the patch's own seam points and every one of them has to take \
         the whole ring - the sweep says so itself where it cannot, and it said it {not_carried} \
         times"
    );
}

/// How many of a tool's vertices ON the ball-centre plane are vertices of the patch too, to the
/// bit - collecting the ones that are not into `misses` with how far they missed by.
fn sharing(
    patches: &BevelCornerPatchSet,
    tool: &Mesh,
    centre: Vector3,
    direction: Vector3,
    misses: &mut Vec<String>,
) -> usize {
    // C#'s HashSet<Vector3> matches by exact component equality; Vector3 is not Eq here, so the
    // same exact test is a scan of a list (`Vector3`'s `==` is exact).
    let patch_points: Vec<Vector3> = patches.patches[0]
        .mesh
        .vertices
        .iter()
        .map(|vertex| vertex.as_vector3())
        .collect();
    let mut matched = 0;

    for vertex in tool.vertices.iter().map(|v| v.as_vector3()) {
        if (vertex - centre).dot(direction).abs() > ON_THE_PLANE
            || ((vertex - centre).length() - RADIUS).abs() > ON_THE_BALL
        {
            continue;
        }

        if patch_points.contains(&vertex) {
            matched += 1;
        } else {
            let nearest = patch_points
                .iter()
                .map(|point| (*point - vertex).length())
                .fold(f64::INFINITY, f64::min);
            misses.push(format!(
                "({:.7}, {:.7}, {:.7}) nearest patch vertex {nearest:e} away",
                vertex.x, vertex.y, vertex.z
            ));
        }
    }

    matched
}

fn assert_shared(misses: &[String], matched: usize, expected: usize) {
    assert!(
        misses.is_empty(),
        "every point a run puts on a ball-centre plane has to be a point the patch put there too, \
         to the bit - a near miss is two polylines on one circle, which is what the kernel answers \
         as a self-intersecting union. These missed: {}",
        misses.join("; ")
    );
    assert!(
        matched >= expected,
        "the three runs each end on a ball-centre plane carrying their own arc, so the seam is \
         {expected} points in all - fewer than that ({matched}) means the test found no seam to \
         check rather than a seam that met"
    );
}

fn part(fixture: &str) -> (Mesh, Vec<(Vector3, Vector3)>) {
    if fixture == "cube corner" {
        let corner = Vector3::new(10.0, 10.0, 10.0);
        let picks = vec![
            (corner, Vector3::new(-10.0, 10.0, 10.0)),
            (corner, Vector3::new(10.0, -10.0, 10.0)),
            (corner, Vector3::new(10.0, 10.0, -10.0)),
        ];
        return (platonic_solids::create_cube(20.0, 20.0, 20.0), picks);
    }

    let top = HEIGHT / 2.0;
    let spread = LENGTH * (degrees_to_radians(APEX_DEGREES) / 2.0).tan();
    let tip = Vector3::new(0.0, 0.0, top);
    let picks = vec![
        (tip, Vector3::new(0.0, 0.0, -top)),
        (tip, Vector3::new(LENGTH, spread, top)),
        (tip, Vector3::new(LENGTH, -spread, top)),
    ];
    (create_isosceles_prism(APEX_DEGREES, LENGTH, HEIGHT), picks)
}
