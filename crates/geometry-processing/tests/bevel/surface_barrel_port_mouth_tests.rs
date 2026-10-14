//! The mouth of a radial PORT through a barrel - the
//! shape "Many Holes.mcx" carries forty one of - at the part's own numbers
//! (`bore_test_meshes_barrel_port`), the first loop in the corpus with a CURVED surface on both
//! sides of it. The plain two-plane ball stands clear of the whole BORE and is cut into by the
//! BARREL, so a port mouth is the one-curved-side case the roll already builds; what stops it at
//! each size is one named fault, pinned here.
//!
//! Legacy-only: every test pins the sections router off
//! (`pinned_to_the_legacy_construction`). The parameter rows are a loop.

use geometry_processing::bevel::surface::surface_mesh_closure;
use geometry_processing::bevel::BevelAdvisoryCode;
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelSelectionKind, FaceGroup, FeatureEdge, FeatureEdgeSet,
};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::bore_test_meshes_barrel_port::{
    create_barrel_with_a_radial_port, BARREL_RADIUS, BARREL_SIDES, PORT_RADIUS, PORT_SIDES,
};
use crate::mesh_measurements::{shell_count, signed_volume};
use crate::surface_test_build::{pinned_to_the_legacy_construction, try_bevel, TryBevelOptions};

/// The fillet the part asks for on every one of its port mouths.
const SIZE: f64 = 0.5;

const SEGMENTS: i32 = 8;

/// The bend threshold "Many Holes.mcx" is stored at, at which the barrel is one surface.
const BEND_THRESHOLD_DEGREES: f64 = 10.0;

/// How many edges the mouth carries: one per bore ruling, and one more wherever the mouth crosses
/// a barrel crease - four creases, each crossed on the way out and on the way back.
const MOUTH_EDGES: usize = PORT_SIDES as usize + 8;

fn barrel_step() -> f64 {
    2.0 * std::f64::consts::PI / f64::from(BARREL_SIDES)
}

fn barrel_inradius() -> f64 {
    BARREL_RADIUS * (barrel_step() / 2.0).cos()
}

fn bore_step() -> f64 {
    2.0 * std::f64::consts::PI / f64::from(PORT_SIDES)
}

fn bore_inradius() -> f64 {
    PORT_RADIUS * (bore_step() / 2.0).cos()
}

/// The fixture itself, before any bevel: a closed solid whose every face lies exactly in the plane
/// of the facet it belongs to.
#[test]
fn the_barrel_segment_is_bored_clean_through_its_wall() {
    let _pinned = pinned_to_the_legacy_construction();
    let part = create_barrel_with_a_radial_port();

    let (open, open_at) = surface_mesh_closure::open_edge_count(&part);
    assert_eq!(
        open, 0,
        "the barrel, the port, the underside and the four sides close on each other, and they do \
         not at {open_at:?}"
    );
    assert!(part.is_manifold());
    assert_eq!(shell_count(&part), 1);
    assert!(signed_volume(&part) > 0.0, "the solid is wound outward");

    let mut worst_off_its_facet = 0.0_f64;

    for face in (0..part.faces.len()).filter(|&face| on_the_barrel(&part, face)) {
        for corner in corners_of(&part, face) {
            let facet =
                f64::from((corner.x.atan2(corner.z) / barrel_step()).round_ties_even() as i32);
            let normal = Vector3::new(
                (facet * barrel_step()).sin(),
                0.0,
                (facet * barrel_step()).cos(),
            );

            worst_off_its_facet =
                worst_off_its_facet.max((corner.dot(normal) - barrel_inradius()).abs());
        }
    }

    assert!(
        worst_off_its_facet < 1e-4,
        "every barrel face lies in one facet's own plane, and one stands {worst_off_its_facet} off it"
    );

    let features = FeatureEdgeSet::create_with_graph(
        &part,
        &part.get_mesh_edge_graph(),
        BEND_THRESHOLD_DEGREES,
    );
    let mouth = the_mouth_edges(&features).len();

    assert_eq!(
        mouth, MOUTH_EDGES,
        "the mouth is a ruling's hit per bore facet and a crossing per crease - instead {mouth}"
    );
}

/// The measurement the whole reordering rests on: the plain two-plane ball stands CLEAR of the
/// bore all the way round the mouth, and is cut INTO by the barrel - 0.476750 at its nearest, a
/// 0.023250 bite. A ball inside a convex faceted wall is under facets that close in on it; a ball
/// outside one is over facets that fall away. Nothing here reads the plan.
#[test]
fn the_plain_ball_stands_clear_of_the_bore_and_not_of_the_barrel() {
    let _pinned = pinned_to_the_legacy_construction();
    let part = create_barrel_with_a_radial_port();
    let features = FeatureEdgeSet::create_with_graph(
        &part,
        &part.get_mesh_edge_graph(),
        BEND_THRESHOLD_DEGREES,
    );
    let barrel: Vec<usize> = (0..part.faces.len())
        .filter(|&face| on_the_barrel(&part, face))
        .collect();
    let bore: Vec<usize> = (0..part.faces.len())
        .filter(|&face| on_the_bore(&part, face))
        .collect();

    assert!(!bore.is_empty(), "the port has a wall");

    let mut nearest_to_the_barrel = f64::MAX;
    let mut nearest_to_the_bore = f64::MAX;

    for edge in the_mouth_edges(&features) {
        for at in [0.0, 0.5, 1.0] {
            let centre = the_plain_balls_centre(&edge, at);

            nearest_to_the_barrel = nearest_to_the_barrel.min(nearest_of(&part, &barrel, centre));
            nearest_to_the_bore = nearest_to_the_bore.min(nearest_of(&part, &bore, centre));
        }
    }

    assert!(
        (nearest_to_the_bore - SIZE).abs() <= 1e-5,
        "a bore's facets fall away from a ball placed outside them - instead {nearest_to_the_bore}"
    );
    assert!(
        (nearest_to_the_barrel - 0.476750).abs() <= 1e-5,
        "a barrel's facets close in on a ball placed inside them - instead {nearest_to_the_barrel}"
    );
}

/// Lars's claim on the real part, asked of the kernel at eight sizes: a port mouth takes its
/// fillet. It does not yet, and this pins on which ONE named fault each size stops - no size opens
/// a mesh, and none of them is the both-curved refusal any more.
#[test]
fn a_port_mouth_is_refused_by_one_named_fault_at_every_size() {
    let _pinned = pinned_to_the_legacy_construction();

    for (size, expected) in [
        (0.05, BevelAdvisoryCode::SurfaceResultNotClosed),
        (0.1, BevelAdvisoryCode::SurfaceResultNotClosed),
        (0.15, BevelAdvisoryCode::SurfaceResultNotClosed),
        (0.2, BevelAdvisoryCode::SurfaceResultNotClosed),
        (0.3, BevelAdvisoryCode::SurfaceResultNotClosed),
        (0.4, BevelAdvisoryCode::SurfaceResultNotClosed),
        (0.5, BevelAdvisoryCode::SurfaceResultNotClosed),
        (0.6, BevelAdvisoryCode::SurfaceStripSeamDidNotMeet),
    ] {
        let part = create_barrel_with_a_radial_port();
        let features = FeatureEdgeSet::create_with_graph(
            &part,
            &part.get_mesh_edge_graph(),
            BEND_THRESHOLD_DEGREES,
        );
        let seed = the_mouth_edge_index(&features);
        let group = bevel_edge_group_resolver::create_from_edge(
            &features,
            seed,
            BevelSelectionKind::TangentChain,
            FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
        )
        .expect("the mouth's edge is in the set");

        let (resolutions, _) =
            bevel_edge_group_resolver::resolve_all(std::slice::from_ref(&group), &part);
        let walked = resolutions[0].edges.len();

        assert_eq!(
            walked, MOUTH_EDGES,
            "a tangent chain seeded on the mouth walks the whole mouth and stops there - instead \
             {walked}"
        );

        let outcome = try_bevel(
            &part,
            &[group],
            size,
            SEGMENTS,
            &CancelToken::new(),
            TryBevelOptions::default(),
        );

        // A refusal hands back no mesh, so nothing open is ever published.
        assert!(
            !outcome.built(),
            "the claim is not met yet at {size} - green here would mean it is"
        );

        let reason = outcome
            .reason()
            .unwrap_or_else(|| panic!("a refusal at {size} names its fault"));

        assert_ne!(
            reason.code(),
            BevelAdvisoryCode::SurfaceLandsOnTwoCurvedFaces,
            "the bore stands clear of the plain ball, so only one side of the mouth is curved"
        );
        assert_eq!(
            reason.code(),
            expected,
            "one named fault stands between this mouth and the claim at {size} - instead {reason:?}"
        );
    }
}

/// Where the plain two-plane ball's centre stands over a chain edge, a fraction `at` along it: a
/// radius inside the material from each of the edge's two faces.
fn the_plain_balls_centre(edge: &FeatureEdge, at: f64) -> Vector3 {
    let opening = 1.0 + edge.normal0.dot(edge.normal1);

    edge.start + ((edge.end - edge.start) * at) - ((edge.normal0 + edge.normal1) * (SIZE / opening))
}

/// How near a set of faces comes to a point.
fn nearest_of(mesh: &Mesh, faces: &[usize], at: Vector3) -> f64 {
    let mut nearest = f64::MAX;

    for &face in faces {
        let corners = corners_of(mesh, face);

        nearest = nearest
            .min((at - nearest_on_the_triangle(at, corners[0], corners[1], corners[2])).length());
    }

    nearest
}

/// The point of one triangle nearest a point - the perpendicular foot where it lands inside, and
/// the nearest point of the nearest side where it does not.
fn nearest_on_the_triangle(at: Vector3, a: Vector3, b: Vector3, c: Vector3) -> Vector3 {
    let mut normal = (b - a).cross(c - a);

    if normal.length_squared() > 0.0 {
        normal = normal.get_normal();
        let foot = at - (normal * (at - a).dot(normal));

        if inside(foot, a, b, c, normal)
            && inside(foot, b, c, a, normal)
            && inside(foot, c, a, b, normal)
        {
            return foot;
        }
    }

    let mut best = nearest_on_the_side(at, a, b);
    let second = nearest_on_the_side(at, b, c);
    let third = nearest_on_the_side(at, c, a);

    if (at - second).length_squared() < (at - best).length_squared() {
        best = second;
    }

    if (at - third).length_squared() < (at - best).length_squared() {
        third
    } else {
        best
    }
}

fn inside(at: Vector3, from: Vector3, to: Vector3, third: Vector3, normal: Vector3) -> bool {
    let side = (to - from).cross(normal);

    (at - from).dot(side) * (third - from).dot(side) >= 0.0
}

// `!(x > 0)` on purpose: a NaN length counts as degenerate.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn nearest_on_the_side(at: Vector3, from: Vector3, to: Vector3) -> Vector3 {
    let run = to - from;

    if !(run.length_squared() > 0.0) {
        return from;
    }

    let along = 1.0_f64.min(0.0_f64.max((at - from).dot(run) / run.length_squared()));

    from + (run * along)
}

fn corners_of(mesh: &Mesh, face: usize) -> [Vector3; 3] {
    let face = mesh.faces[face];

    [
        mesh.vertices[face.v0 as usize].as_vector3(),
        mesh.vertices[face.v1 as usize].as_vector3(),
        mesh.vertices[face.v2 as usize].as_vector3(),
    ]
}

/// A face of the barrel's own surface: standing out at the facets' own reach and facing out.
fn on_the_barrel(mesh: &Mesh, face: usize) -> bool {
    let corners = corners_of(mesh, face);
    let normal = mesh.faces[face].normal.as_vector3();
    let middle = (corners[0] + corners[1] + corners[2]) / 3.0;
    let outward = Vector3::new(middle.x, 0.0, middle.z);

    corners
        .iter()
        .all(|corner| Vector3::new(corner.x, 0.0, corner.z).length() > barrel_inradius() - 1e-4)
        && normal.y.abs() < 1e-4
        && normal.dot(outward.get_normal()) > 0.9
}

/// A face of the port's wall: within the bore's own reach of its axis and facing it.
fn on_the_bore(mesh: &Mesh, face: usize) -> bool {
    let corners = corners_of(mesh, face);
    let normal = mesh.faces[face].normal.as_vector3();
    let middle = (corners[0] + corners[1] + corners[2]) / 3.0;

    corners
        .iter()
        .all(|corner| Vector3::new(corner.x, corner.y, 0.0).length() < PORT_RADIUS + 1e-4)
        && normal.z.abs() < 1e-4
        && normal.dot(Vector3::new(middle.x, middle.y, 0.0).get_normal()) < -0.9
}

/// The index of one edge of the port's mouth, to seed the tangent chain on.
fn the_mouth_edge_index(features: &FeatureEdgeSet) -> i32 {
    (0..features.count())
        .find(|&i| on_the_mouth(&features.edges()[i]))
        .map(|i| i as i32)
        .expect("the fixture has a port mouth")
}

fn the_mouth_edges(features: &FeatureEdgeSet) -> Vec<FeatureEdge> {
    features
        .edges()
        .iter()
        .filter(|edge| on_the_mouth(edge))
        .cloned()
        .collect()
}

/// An edge of the port's mouth: both ends a port radius from the port's axis and out at the
/// barrel's own reach, which no other feature edge of the segment is.
fn on_the_mouth(edge: &FeatureEdge) -> bool {
    at_the_mouth(edge.start) && at_the_mouth(edge.end)
}

/// A point of the mouth stands out at the barrel's own reach and between the bore's inradius and
/// its circumradius.
fn at_the_mouth(at: Vector3) -> bool {
    let round = Vector3::new(at.x, at.y, 0.0).length();

    round > bore_inradius() - 1e-4
        && round < PORT_RADIUS + 1e-4
        && Vector3::new(at.x, 0.0, at.z).length() > barrel_inradius() - 1e-4
}
