//! Port of `SurfaceRakedBoreBandOutlineTests.cs` (a partial of `SurfaceBoreMouthTests`): THE
//! BAND'S OUTLINE at a chorded wall's mitre - how deep the walkback (`surface_face_walkback`) is
//! allowed to cut into the bore's wall, read facet by facet against that facet's OWN rail, on the
//! straight control and on the raked bore. The bore and its walkback are
//! `surface_raked_bore_mouth_tests`' ([`a_bores_mouth_walked_back`]).

use geometry_processing::bevel::surface::{SurfaceBevelPlan, SurfaceLine, SurfaceVertexPool};
use vector_math::{math_helper, Vector3};

use crate::surface_raked_bore_mouth_tests::{
    a_bores_mouth_walked_back, RAKED_RADIUS, RAKE_DEGREES,
};

/// THE CONTROL: the same bore drilled STRAIGHT. Nothing the walkback leaves on its wall stands
/// inside the band - a vertex lying in a facet's plane is on the kept side of that facet's own
/// rail - and a crease is the one exemption, because that is where the rim steps between two
/// rails' depths.
#[test]
fn the_straight_band_takes_off_nothing_past_each_facets_own_rail() {
    assert_the_band_takes_off_nothing_past_each_facets_own_rail(0.0);
}

/// And the same of the RAKED bore, which is where it was false: 38 of its 385 wall vertices - one
/// per surplus corner - stood inside the band, the worst 0.0446 deep.
///
/// Depth is the reading and not distance: the lip the half-space flood cut past a crease lies on
/// the corner's own MITRE plane, as does every honest split the walkback makes down a facet, so
/// distance to the nearest rail cannot name it; depth past the facet's own rail can, because an
/// honest split stands on the kept side of it and the lip is inside the band.
#[test]
fn the_raked_band_takes_off_nothing_past_each_facets_own_rail() {
    assert_the_band_takes_off_nothing_past_each_facets_own_rail(RAKE_DEGREES);
}

fn assert_the_band_takes_off_nothing_past_each_facets_own_rail(rake_degrees: f64) {
    let rake = math_helper::degrees_to_radians(rake_degrees);
    let axis = Vector3::new(rake.sin(), 0.0, rake.cos());

    let (walked, plan) = a_bores_mouth_walked_back(rake_degrees);

    let ruler = SurfaceVertexPool::near_enough_for(&walked.mesh.vertices);

    let facets = the_wall_facets(&plan, axis);
    let creases: Vec<SurfaceLine> = plan
        .edges
        .iter()
        .map(|edge| SurfaceLine::new(edge.from(), axis))
        .collect();

    let mut strays: Vec<(Vector3, f64)> = Vec::new();
    let mut on_the_wall = 0;

    for vertex in walked
        .mesh
        .vertices
        .iter()
        .map(|&at| Vector3::from_vector3_float(at))
    {
        // The wall's facets sag to cos(pi / sides) of the bore's radius and nothing else of the
        // plate stands that near the drill's own axis.
        if (vertex - (axis * vertex.dot(axis))).length() >= RAKED_RADIUS + 1e-6 {
            continue;
        }

        on_the_wall += 1;

        let nearest_crease = creases
            .iter()
            .map(|crease| crease.distance_to(vertex))
            .fold(f64::INFINITY, f64::min);
        if nearest_crease <= ruler {
            // A crease is where the rim steps between two rails' depths, so a vertex on one stands
            // inside the neighbouring facet's band by exactly that step and is meant to.
            continue;
        }

        let mut into = 0.0_f64;

        for (normal, rail, inward) in &facets {
            if (vertex - rail.point).dot(*normal).abs() > ruler {
                continue;
            }

            into = into.max(-(vertex - rail.point).dot(*inward));
        }

        if into > ruler {
            strays.push((vertex, into));
        }
    }

    let worst = strays
        .iter()
        .copied()
        .fold((Vector3::ZERO, 0.0), |worst, stray| {
            if stray.1 > worst.1 {
                stray
            } else {
                worst
            }
        });

    assert_eq!(
        strays.len(),
        0,
        "nothing stands inside the band the walkback took off, so a wall vertex lying in a facet's \
         plane is on the kept side of that facet's own rail unless it is on a crease, where the rim \
         steps - instead {} of {on_the_wall} wall vertices stand inside, the worst {:?} by {} \
         against a ruler of {ruler}",
        strays.len(),
        worst.0,
        worst.1
    );
}

/// The bore's forty wall facets, each with its own outward normal, the RAIL the band's far side
/// runs along on it - that chord's wall-side tangent line - and the direction from the rail into
/// the material the band leaves standing.
fn the_wall_facets(plan: &SurfaceBevelPlan, axis: Vector3) -> Vec<(Vector3, SurfaceLine, Vector3)> {
    plan.edges
        .iter()
        .map(|edge| {
            // The wall is whichever of the two faces is NOT the plate's top.
            let rail = if edge.edge.left_normal().z > 0.9 {
                edge.right_tangent_line
            } else {
                edge.left_tangent_line
            };

            // The facet's own plane holds the chord and the drill's axis, so its normal is their
            // cross, turned to face AWAY from the axis, which is the side the plate's material is
            // on.
            let chord = (edge.to() - edge.from()).get_normal();
            let middle = (edge.from() + edge.to()) / 2.0;
            let mut normal = chord.cross(axis).get_normal();

            if normal.dot(middle - (axis * middle.dot(axis))) < 0.0 {
                normal = -normal;
            }

            // Square to the rail in that plane, pointing away from the mouth's own chord: the way
            // the band's material lies, and so the side of the rail a vertex may stand on.
            let mut inward = normal.cross(rail.direction).get_normal();

            if inward.dot(rail.point - middle) < 0.0 {
                inward = -inward;
            }

            (normal, rail, inward)
        })
        .collect()
}
