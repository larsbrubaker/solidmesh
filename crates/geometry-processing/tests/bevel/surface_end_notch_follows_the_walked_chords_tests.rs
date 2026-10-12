//! Port of `SurfaceEndNotchFollowsTheWalkedChordsTests.cs`: the END of a chain whose bevel ROLLED
//! along a polygonal wall. The section the end cap gives up is not a wedge with two straight sides,
//! because on the wall side the cap's own rim runs along the facets the walkback took off - whole
//! chords of them - before it reaches the foot the strip stands on.
//!
//! The half disc's upright seam filleted 3 is the kernel reading of the two cylinder-to-cube seams
//! of "Every Chamfer Works.mcx". The section the cap must give up is the polygon `corner, flat
//! foot, profile arc, wall foot, facet k's rim back to the corner` - and a wedge whose wall side is
//! the STRAIGHT run from the corner to the wall foot leaves the slivers between that run and the
//! rim standing on the cap with the wall above them already gone. On this fixture the arc reaches
//! so far round the corner that part of it stands OUTSIDE that straight wedge, so the end refused
//! outright (`SurfaceStripEndNotSupported`). Both are the one cause.

use std::sync::Arc;

use agg_path::vertex_storage::VertexStorage;
use geometry_processing::bevel::bevel_arc_chording;
use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, surface_ring_ears, SurfaceBevelPlan,
    SurfaceEdgePlan, SurfaceVertexPool,
};
use geometry_processing::topology::{EdgeConvexity, FaceRegionCache, FeatureEdge, FeatureEdgeSet};
use polygon_mesh::processors::vertex_source_to_mesh::extrude;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{math_helper, Vector2, Vector3, Vector3Float};

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::topology_test_meshes::create_arc_prism;

/// The bend threshold the fixture is read at, which the arc's facet joins are under.
const THRESHOLD_DEGREES: f64 = 10.0;

const RADIUS: f64 = 10.0;

const HEIGHT: f64 = 20.0;

const ARC_FACETS: i32 = 20;

/// The fillet the seam is cut with - big enough that its wall foot is three facets round.
const SIZE: f64 = 3.0;

/// The operation's chording, per quarter turn, which is what the strip rules its arc at.
const SEGMENTS_PER_QUARTER: i32 = 8;

#[test]
fn a_rolled_seam_closes_once_the_notch_follows_the_walls_own_chords() {
    let prism = create_arc_prism(RADIUS, HEIGHT, ARC_FACETS);
    let (plan, regions) = seam_plan_of(&prism);

    let built = surface_bevel_builder::try_build_one(
        &prism,
        &prism.get_mesh_edge_graph(),
        &regions,
        plan.clone(),
        SEGMENTS_PER_QUARTER,
        &CancelToken::new(),
    )
    .unwrap_or_else(|reason| panic!("{reason:?}"));

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
    assert_eq!(
        open, 0,
        "the cap gives up the section the wall's chords bound, and this one opened at {open_at:?}"
    );
    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);

    // The whole of the answer in one number: the prism less the section, along the seam's own
    // height. The section is worked out from the plan's own feet and the mesh's own rim rather
    // than from the ideal cylinder, because a fillet rolled onto a TESSELLATED wall takes neither
    // the ideal disc's area nor the unrolled construction's.
    let section = the_section_area(&prism, &plan.edges[0]);

    let expected = signed_volume(&prism) - (section * HEIGHT);
    let measured = signed_volume(&built);
    assert!(
        (measured - expected).abs() <= 1e-3,
        "the seam gives up {section} of each of its {HEIGHT} mm and nothing else - {measured}, \
         expected {expected}"
    );
}

/// And what it gave up, said on the cap itself: nothing of the built mesh stands inside the
/// section, and the rim vertices of the facets the walkback took off are gone from the part
/// altogether - the wall above them was removed over the seam's whole height, so a cap still
/// carrying them would be carrying the slivers the straight wedge used to leave.
#[test]
fn the_cap_keeps_nothing_inside_the_section_and_none_of_the_walked_chords_corners() {
    let prism = create_arc_prism(RADIUS, HEIGHT, ARC_FACETS);
    let (plan, regions) = seam_plan_of(&prism);

    let built = surface_bevel_builder::try_build_one(
        &prism,
        &prism.get_mesh_edge_graph(),
        &regions,
        plan.clone(),
        SEGMENTS_PER_QUARTER,
        &CancelToken::new(),
    )
    .unwrap_or_else(|reason| panic!("{reason:?}"));

    let edge = &plan.edges[0];
    let walked = the_walked_chord_corners(&prism, edge);

    assert_eq!(
        walked.len(),
        2,
        "the foot stands in the THIRD facet of the wall, so two whole chord corners are walked past"
    );

    for corner in walked {
        let still: Vec<Vector3> = built
            .vertices
            .iter()
            .map(|&vertex| Vector3::from_vector3_float(vertex))
            .filter(|&at| (at - corner).length() < 1e-3)
            .collect();

        assert!(
            still.is_empty(),
            "the wall over {corner:?} came off over the whole height, so the cap may not keep its \
             corner: {still:?}"
        );
    }

    let section = the_section(&prism, edge);

    let inside: Vec<Vector3> = built
        .vertices
        .iter()
        .map(|&vertex| Vector3::from_vector3_float(vertex))
        .filter(|&at| at.z.abs() < 1e-6 && is_inside(&section, Vector2::new(at.x, at.y), 1e-3))
        .collect();

    assert!(
        inside.is_empty(),
        "the cap gave the section up, so nothing of it stands inside it: {inside:?}"
    );
}

/// The same seam CONCAVE: a half disc standing proud of a slab, so the fillet is a bead filling
/// the reentrant corner rather than a cut taking it off. The cap there GAINS the section instead
/// of giving it up, and the section it gains is bounded by the very same walked chords.
///
/// Built as one extruded outline rather than by a boolean, which is what makes it cheap: the
/// slab's top edge runs out to the seam at (10, 0) and the arc leaves it there, so the material
/// fills 270 degrees at that corner where the half disc's own seam fills 90.
#[test]
fn a_concave_rolled_seams_bead_cap_reaches_the_same_chords() {
    let boss = boss_on_a_slab();
    let (plan, regions) = seam_plan_of(&boss);

    assert_eq!(
        plan.edges[0].convexity(),
        EdgeConvexity::Concave,
        "a half disc standing proud of a slab fills 270 degrees where its seam meets the top"
    );

    assert!(
        plan.edges[0].left_band.is_some() || plan.edges[0].right_band.is_some(),
        "and the bead rolls along the wall exactly as the cut does, or this fixture proves nothing"
    );

    let built = surface_bevel_builder::try_build_one(
        &boss,
        &boss.get_mesh_edge_graph(),
        &regions,
        plan.clone(),
        SEGMENTS_PER_QUARTER,
        &CancelToken::new(),
    )
    .unwrap_or_else(|reason| panic!("{reason:?}"));

    let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
    assert_eq!(
        open, 0,
        "the bead's own cap reaches the rim the wall left, and this one opened at {open_at:?}"
    );
    assert!(built.is_manifold());
    assert_eq!(shell_count(&built), 1);

    // A bead ADDS what a cut takes off, so the same section area comes back the other way about.
    let section = the_section_area(&boss, &plan.edges[0]);

    let expected = signed_volume(&boss) + (section * HEIGHT);
    let measured = signed_volume(&built);
    assert!(
        (measured - expected).abs() <= 1e-3,
        "the seam gains {section} of each of its {HEIGHT} mm and nothing else - {measured}, \
         expected {expected}"
    );
}

/// A half disc of [`RADIUS`] standing proud of a slab, extruded [`HEIGHT`]: the concave reading of
/// `create_arc_prism`, with the same wall and the same seam at (10, 0) and the material on the
/// other side of it.
fn boss_on_a_slab() -> Mesh {
    let mut outline = VertexStorage::new();

    outline.move_to(20.0, -20.0);
    outline.line_to(20.0, 0.0);
    outline.line_to(RADIUS, 0.0);

    for i in 1..=ARC_FACETS {
        let angle = std::f64::consts::PI * i as f64 / ARC_FACETS as f64;

        outline.line_to(RADIUS * angle.cos(), RADIUS * angle.sin());
    }

    outline.line_to(-20.0, 0.0);
    outline.line_to(-20.0, -20.0);
    outline.close_polygon();

    extrude(&outline, HEIGHT).expect("the outline extrudes")
}

/// The ring the section is cut into triangles by, asked of the two things it has to say: a
/// polygon that caves in BOTH ways - which is what a rolled concave section is, and what no fan
/// covers - is triangulated exactly, and a ring that crosses itself is REFUSED with nothing
/// emitted.
///
/// The refusal is the one that matters in production. A partial triangulation, or a fan from a
/// middle that stands outside the ring, comes out closed and manifold and carries volume nobody
/// asked for - which is to say the closure gate cannot see it, so the only place to catch it is
/// here.
#[test]
fn the_sections_ring_is_ear_clipped_exactly_or_refused() {
    // A crescent: a straight side, an arc caving in one way and a chain of chords caving in the
    // other, which between them leave no vertex that sees the whole ring.
    let crescent = [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(8.0, 3.0, 0.0),
        Vector3::new(5.0, 4.5, 0.0),
        Vector3::new(2.0, 4.0, 0.0),
        Vector3::new(1.5, 2.5, 0.0),
        Vector3::new(0.5, 1.5, 0.0),
    ];

    let mut pool = SurfaceVertexPool::new(&[] as &[Vector3Float]);
    let ring: Vec<i32> = crescent.iter().map(|&at| pool.add(at)).collect();
    let mut faces = Vec::new();

    assert!(surface_ring_ears::try_triangulate_the_ring(
        &ring,
        &pool,
        &mut faces,
        Vector3::UNIT_Z,
        false
    ));

    assert_eq!(
        faces.len(),
        crescent.len() - 2,
        "a simple ring of n corners is n - 2 triangles, however it caves in"
    );

    let covered = area_of_faces(&faces, &pool);
    let enclosed = area_of_ring(&crescent);
    assert!(
        (covered - enclosed).abs() <= 1e-9,
        "and they cover it exactly once - a fold would cover part of it twice: {covered} against \
         {enclosed}"
    );

    // The same ring with two of its corners swapped, which crosses itself.
    let mut crossed = ring.clone();

    crossed.swap(2, 5);

    let mut refused = Vec::new();

    assert!(
        !surface_ring_ears::try_triangulate_the_ring(
            &crossed,
            &pool,
            &mut refused,
            Vector3::UNIT_Z,
            false
        ),
        "a ring that crosses itself has no triangulation, and half of one is a fold"
    );

    assert!(
        refused.is_empty(),
        "a refusal leaves nothing behind for the caller to publish"
    );
}

/// The area the triangles cover, counting anything covered twice twice.
fn area_of_faces(faces: &[(i32, i32, i32)], pool: &SurfaceVertexPool) -> f64 {
    let mut area = 0.0;

    for &(v0, v1, v2) in faces {
        let at = pool.position_of(v0);

        area += (pool.position_of(v1) - at)
            .cross(pool.position_of(v2) - at)
            .length()
            / 2.0;
    }

    area
}

/// The area a ring of the z = 0 plane encloses.
fn area_of_ring(ring: &[Vector3]) -> f64 {
    let mut twice_area = 0.0;

    for i in 0..ring.len() {
        let at = ring[i];
        let next = ring[(i + 1) % ring.len()];

        twice_area += (at.x * next.y) - (next.x * at.y);
    }

    twice_area.abs() / 2.0
}

/// The section polygon in the bottom cap, corner first and then round the way the cap is wound.
fn the_section(prism: &Mesh, edge: &SurfaceEdgePlan) -> Vec<Vector2> {
    let corner = Vector2::new(RADIUS, 0.0);
    let wall_on_the_left = edge.left_band.is_some();
    let wall_foot = flat(if wall_on_the_left {
        edge.left_tangent_line.closest_point_to(Vector3::ZERO)
    } else {
        edge.right_tangent_line.closest_point_to(Vector3::ZERO)
    });
    let flat_foot = flat(if wall_on_the_left {
        edge.right_tangent_line.closest_point_to(Vector3::ZERO)
    } else {
        edge.left_tangent_line.closest_point_to(Vector3::ZERO)
    });
    let centre = flat(edge.center_line.closest_point_to(Vector3::ZERO));

    let mut section = vec![corner, flat_foot];

    // The arc, sampled the way the strip rules it: uniformly from foot to foot, at the
    // operation's own chording for the sweep it makes.
    let from = flat_foot - centre;
    let to = wall_foot - centre;
    let sweep = (from.dot(to) / (from.length() * to.length()))
        .clamp(-1.0, 1.0)
        .acos();
    let samples = bevel_arc_chording::segments_for(
        math_helper::radians_to_degrees(sweep),
        SEGMENTS_PER_QUARTER,
    )
    .max(1);

    // Which way round the arc turns is the convexity's: the turn that lands on the wall foot.
    let turning = if (centre + from.get_rotated(sweep) - wall_foot).length() < 1e-6 {
        1.0
    } else {
        -1.0
    };

    for i in 1..samples {
        section.push(centre + from.get_rotated(turning * sweep * i as f64 / samples as f64));
    }

    section.push(wall_foot);

    let walked = the_walked_chord_corners(prism, edge);

    for at in walked.iter().rev() {
        section.push(Vector2::new(at.x, at.y));
    }

    section
}

/// Its area, which is what the seam takes off each millimetre of its height.
fn the_section_area(prism: &Mesh, edge: &SurfaceEdgePlan) -> f64 {
    let section = the_section(prism, edge);
    let mut twice_area = 0.0;

    for i in 0..section.len() {
        let at = section[i];
        let next = section[(i + 1) % section.len()];

        twice_area += (at.x * next.y) - (next.x * at.y);
    }

    twice_area.abs() / 2.0
}

/// The corners of the bottom rim standing between the seam and the wall foot: the joins of the
/// facets the roll walked past, which is what the section has to follow rather than cut across.
fn the_walked_chord_corners(prism: &Mesh, edge: &SurfaceEdgePlan) -> Vec<Vector3> {
    let wall_on_the_left = edge.left_band.is_some();
    let foot = if wall_on_the_left {
        edge.left_tangent_line.closest_point_to(Vector3::ZERO)
    } else {
        edge.right_tangent_line.closest_point_to(Vector3::ZERO)
    };
    let reach = foot.y.atan2(foot.x);
    let mut found: Vec<(f64, Vector3)> = Vec::new();

    for &vertex in &prism.vertices {
        let at = Vector3::from_vector3_float(vertex);

        if at.z.abs() > 1e-6 || (at.length() - RADIUS).abs() > 1e-3 {
            continue;
        }

        let angle = at.y.atan2(at.x);

        if angle > 1e-6 && angle < reach {
            found.push((angle, at));
        }
    }

    // The angles are distinct corners of one rim, so the order has no ties.
    found.sort_by(|first, second| first.0.total_cmp(&second.0));

    found.into_iter().map(|(_, at)| at).collect()
}

/// Whether a point of the cap's plane stands inside the section, by more than `margin`.
fn is_inside(section: &[Vector2], at: Vector2, margin: f64) -> bool {
    // The section is wound one way round and is simple, so "inside" is the winding rule - and a
    // point within a margin of any of its sides is read as ON it rather than in it.
    for i in 0..section.len() {
        let from = section[i];
        let run = section[(i + 1) % section.len()] - from;
        let where_on = ((at - from).dot(run) / run.length_squared()).clamp(0.0, 1.0);

        if (at - (from + (run * where_on))).length() < margin {
            return false;
        }
    }

    let mut crossings = 0;

    for i in 0..section.len() {
        let from = section[i];
        let to = section[(i + 1) % section.len()];

        if (from.y > at.y) != (to.y > at.y)
            && at.x < from.x + ((at.y - from.y) / (to.y - from.y) * (to.x - from.x))
        {
            crossings += 1;
        }
    }

    crossings % 2 == 1
}

fn flat(at: Vector3) -> Vector2 {
    Vector2::new(at.x, at.y)
}

/// The plan for the seam at (10, 0), flooded at the fixture's own threshold.
fn seam_plan_of(mesh: &Mesh) -> (std::rc::Rc<SurfaceBevelPlan>, Arc<FaceRegionCache>) {
    let features = FeatureEdgeSet::create_with_angle(mesh, THRESHOLD_DEGREES);

    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(mesh.clone()),
        Some(mesh.get_mesh_edge_graph()),
        THRESHOLD_DEGREES,
    ));

    let seam: Vec<FeatureEdge> = features
        .edges()
        .iter()
        .filter(|edge| at_the_seam(edge.start) && at_the_seam(edge.end))
        .copied()
        .collect();

    let chains = bevel_chain_linker::link(&seam);
    assert_eq!(chains.len(), 1, "the seam links into one chain");

    let plan = SurfaceBevelPlan::try_create(&chains[0], &regions, SIZE, true)
        .unwrap_or_else(|reason| panic!("the seam has to plan: {:?}", reason.code()));

    (plan, regions)
}

/// Whether a point stands on the seam itself - the upright edge at (10, 0) where the wall leaves
/// the flat face. Pinned to that corner rather than to the plane `y = 0`, because the flat face's
/// own rim runs along the same plane and would link into the chain as a second edge.
fn at_the_seam(point: Vector3) -> bool {
    point.y.abs() < 1e-3 && (point.x - RADIUS).abs() < 1e-3
}
