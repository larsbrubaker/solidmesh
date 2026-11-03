//! A stopped round on one facet of a finely faceted cylinder stored in single precision and
//! turned to a generic tilt, so no seam lies exactly in its stored end plane (the far end of a
//! long seam stands hundreds of float steps off it): the round is still cut flat on both end
//! planes, each end hole closed by the flat fill in its plane, with its whole section there,
//! closed, and its neighbours' rim edges sharp.

use std::cell::RefCell;
use std::f64::consts::PI;
use std::rc::Rc;

use geometry_processing::bevel::level_line::corner_cap_flat_ends::WATCH_FLAT_ENDS;
use geometry_processing::bevel::level_line::{
    gap_fill, turn_stopped_ends_on, BandProfile, HoleFillTriangle, LevelLineMesh,
};
use vector_math::Vector3;

use super::stopped_end_arc_tests::{band, feature_edges, is_closed, keeps_edge, Mesh};

const RADIUS: f64 = 10.0;
const HEIGHT: f64 = 50.0;

/// `v` turned by `angle` about the unit `axis`.
fn turned(v: Vector3, axis: Vector3, angle: f64) -> Vector3 {
    let (sin, cos) = angle.sin_cos();
    v * cos + axis.cross(v) * sin + axis * (axis.dot(v) * (1.0 - cos))
}

fn tilt(v: Vector3, k: &Case) -> Vector3 {
    turned(v, k.axis.get_normal(), k.angle)
}

struct Case {
    facets: i32,
    size: f64,
    axis: Vector3,
    angle: f64,
}

/// The cylinder's outline point at facet corner `i`, before the tilt.
fn corner(i: i32, z: f64, k: &Case) -> Vector3 {
    let angle = f64::from(i) * 2.0 * PI / f64::from(k.facets);
    Vector3::new(RADIUS * angle.cos(), RADIUS * angle.sin(), z)
}

/// The tilted cylinder as a part stores it, every coordinate rounded to single precision; bottom
/// ring 0..n, top ring n..2n, caps fanned from corner 0, wound outward.
fn tilted_single_precision_cylinder(k: &Case) -> Mesh {
    let n = k.facets;
    let mut positions = Vec::new();
    for z in [0.0, HEIGHT] {
        for i in 0..n {
            let p = tilt(corner(i, z, k), k);
            positions.push(Vector3::new(
                f64::from(p.x as f32),
                f64::from(p.y as f32),
                f64::from(p.z as f32),
            ));
        }
    }
    let bottom = |i: i32| i % n;
    let top = |i: i32| n + (i % n);
    let mut indices = Vec::new();
    for i in 0..n {
        indices.push((bottom(i), bottom(i + 1), top(i + 1)));
        indices.push((bottom(i), top(i + 1), top(i)));
    }
    for k in 1..n - 1 {
        indices.push((top(0), top(k), top(k + 1)));
        indices.push((bottom(0), bottom(k + 1), bottom(k)));
    }
    (positions, indices)
}

/// How near an exact end plane a point of the cut is read on it: the stored plane leans off the
/// exact one by a few millionths over the section's reach.
const REACH: f64 = 1e-4;

/// The area of the live triangles within [`REACH`] of the plane through `v` with unit `normal`.
fn area_in_the_plane(mesh: &LevelLineMesh, v: Vector3, normal: Vector3) -> f64 {
    let mut area = 0.0;
    for (i, t) in mesh.triangles.iter().enumerate() {
        if mesh.removed[i] {
            continue;
        }
        let corners = [t.a, t.b, t.c].map(|c| mesh.vertices[c as usize]);
        if corners.iter().all(|&c| (c - v).dot(normal).abs() <= REACH) {
            area += (corners[1] - corners[0])
                .cross(corners[2] - corners[0])
                .length()
                / 2.0;
        }
    }
    area
}

/// Fills the stopped round on the facet between top corners 0 and 1 of the case's cylinder and
/// checks it closed, its neighbours' rim edges sharp, each end hole closed flat in its own plane
/// and showing the round's whole section there. Hands back the smallest live triangle's area over
/// the facet's chord squared.
fn cuts_flat(k: &Case) -> f64 {
    let n = k.facets;
    let mesh = tilted_single_precision_cylinder(k);
    let segments = 4;
    let gap = {
        let _on = turn_stopped_ends_on();
        band(&mesh, &[vec![n, n + 1]], feature_edges(&mesh), k.size)
    };
    assert!(gap.built(), "the stopped facet builds: {:?}", gap.refusal);
    let bounds = gap.chains[0].end_ray_bounds;
    assert!(
        bounds.0.is_some_and(|b| b.stopped) && bounds.1.is_some_and(|b| b.stopped),
        "both ends stop"
    );
    // Each end plane, exact: through the stored top corner, faced along the tilted tangent.
    let planes: Vec<(Vector3, Vector3)> = [0, 1]
        .map(|i| {
            let along = corner(i, 0.0, k);
            let normal = tilt(Vector3::new(-along.y, along.x, 0.0), k).get_normal();
            (mesh.0[(n + i) as usize], normal)
        })
        .to_vec();
    let closed_flat = Rc::new(RefCell::new([0; 2]));
    {
        let seen = Rc::clone(&closed_flat);
        let planes = planes.clone();
        WATCH_FLAT_ENDS.with(|w| {
            *w.borrow_mut() = Some(Rc::new(
                move |m: &LevelLineMesh, hole: &[i32], _: &[HoleFillTriangle]| {
                    for (end, &(v, normal)) in planes.iter().enumerate() {
                        if hole
                            .iter()
                            .all(|&i| (m.vertices[i as usize] - v).dot(normal).abs() <= REACH)
                        {
                            seen.borrow_mut()[end] += 1;
                        }
                    }
                },
            ));
        });
    }
    let filled = gap_fill::fill_with_budget(&gap, BandProfile::Fillet, segments, None);
    WATCH_FLAT_ENDS.with(|w| *w.borrow_mut() = None);
    let filled = filled.expect("the fill runs");
    assert!(filled.filled(), "the facet fills: {:?}", filled.refusal);
    assert_eq!(
        *closed_flat.borrow(),
        [1, 1],
        "each end hole is closed by the flat fill in its own plane"
    );
    let handle = Rc::clone(filled.mesh.as_ref().expect("the fill keeps its mesh"));
    let filled_mesh = handle.borrow();
    assert!(is_closed(&filled_mesh), "the filled facet is closed");
    let (before, first, last, after) = (n + n - 1, n, n + 1, n + 2);
    assert!(
        keeps_edge(&filled_mesh, before, first) || keeps_edge(&filled_mesh, first, before),
        "the rim edge before the facet stays sharp"
    );
    assert!(
        keeps_edge(&filled_mesh, last, after) || keeps_edge(&filled_mesh, after, last),
        "the rim edge after the facet stays sharp"
    );
    let alpha = PI / 2.0 / f64::from(segments);
    let chorded = (k.size * k.size - f64::from(segments) * k.size * k.size / 2.0 * alpha.sin())
        / (PI / f64::from(n)).cos();
    for (end, &(v, normal)) in planes.iter().enumerate() {
        let area = area_in_the_plane(&filled_mesh, v, normal);
        assert!(
            (area - chorded).abs() <= 1e-3 * chorded,
            "the end plane at top corner {end} shows the round's whole section, cut flat: {area} \
             against {chorded}"
        );
    }
    let chord = 2.0 * RADIUS * (PI / f64::from(n)).sin();
    let smallest = filled_mesh
        .triangles
        .iter()
        .enumerate()
        .filter(|&(i, _)| !filled_mesh.removed[i])
        .map(|(_, t)| {
            let [a, b, c] = [t.a, t.b, t.c].map(|v| filled_mesh.vertices[v as usize]);
            (b - a).cross(c - a).length() / 2.0
        })
        .fold(f64::INFINITY, f64::min);
    smallest / (chord * chord)
}

#[test]
fn one_facet_of_a_tilted_single_precision_fine_cylinder_is_cut_flat_with_its_whole_profile() {
    let smallest = cuts_flat(&Case {
        facets: 128,
        size: 5.0,
        axis: Vector3::new(1.0, 2.0, 3.0),
        angle: 0.7,
    });
    assert!(smallest >= 1e-6, "no sliver is left: {smallest}");
}

#[test]
fn tilted_single_precision_cylinders_of_other_facet_counts_and_sizes_are_cut_flat() {
    for case in [
        Case {
            facets: 128,
            size: 2.0,
            axis: Vector3::new(1.0, 2.0, 3.0),
            angle: 0.7,
        },
        Case {
            facets: 64,
            size: 5.0,
            axis: Vector3::new(-0.3, 0.9, 0.2),
            angle: 1.9,
        },
        Case {
            facets: 96,
            size: 3.0,
            axis: Vector3::new(0.6, 0.6, 0.1),
            angle: 0.37,
        },
    ] {
        let smallest = cuts_flat(&case);
        assert!(smallest >= 1e-6, "no sliver is left: {smallest}");
    }
}

/// The far end of each 256-facet seam stands 150-250 steps off its stored end plane: read in the
/// plane only within a fixed few steps, the end's cap is refused. The fill's smallest triangles
/// here are not asserted.
#[test]
fn a_256_facet_tilted_cylinder_whose_seams_stand_far_off_the_stored_plane_is_cut_flat() {
    cuts_flat(&Case {
        facets: 256,
        size: 5.0,
        axis: Vector3::new(0.31, 0.47, -0.83),
        angle: 1.1,
    });
}

/// A round as large as most of the facet's reach along the tilted seam: the end plane at each
/// corner still holds the whole section.
#[test]
fn a_large_round_on_a_tilted_single_precision_cylinder_shows_its_whole_section_at_both_ends() {
    let smallest = cuts_flat(&Case {
        facets: 128,
        size: 8.0,
        axis: Vector3::new(0.8, -0.1, 0.5),
        angle: 2.6,
    });
    assert!(smallest >= 1e-6, "no sliver is left: {smallest}");
}
