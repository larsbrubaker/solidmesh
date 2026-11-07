//! The fan's sliver
//! rings. A pinched sliver of the lobe wall and a three-corner ring of one line emit nothing, a fan
//! takes no diagonal through a corner of its own ring, a collinear ring is fanned from a farthest
//! corner, and a collapsed strip quad is emitted as needles along its own chord (or nothing, where
//! it doubles back).

use geometry_processing::bevel::surface::{surface_fan, surface_split_sewing, SurfaceVertexPool};
use vector_math::{Vector3, Vector3Float};

use crate::lobed_prism_meshes;

fn listed(faces: &[(i32, i32, i32)]) -> String {
    faces
        .iter()
        .map(|f| format!("({},{},{})", f.0, f.1, f.2))
        .collect::<Vec<_>>()
        .join(" ")
}

#[test]
fn the_pinched_sliver_of_the_lobe_wall_emits_nothing() {
    // The lobe prism's OWN vertices, so the ruler is the one the measurement was made with.
    let mut pool = SurfaceVertexPool::new(&lobed_prism_meshes::create().vertices);

    let corner = pool.add(Vector3::new(29.8509, 0.0, 19.9)); // 189
    let far = pool.add(Vector3::new(29.8964, 0.0, 19.9431)); // 564

    let split_from = pool.vertices().len() as i32;

    let out0f = pool.add(Vector3::new(29.8702, 0.0, 19.9191)); // 475
    pool.add(Vector3::new(29.8910, 0.0, 19.9386)); // 482

    let mut faces = vec![(corner, far, out0f)];

    surface_split_sewing::sew_in_the_splits_since_unchecked(&mut pool, &mut faces, split_from);

    assert_eq!(
        faces.len(),
        0,
        "the face is three points of one line to within the ruler, so it covers nothing - instead {}",
        listed(&faces)
    );
}

#[test]
fn a_three_corner_ring_of_one_line_emits_nothing() {
    let mut pool = SurfaceVertexPool::new(&lobed_prism_meshes::create().vertices);

    let first = pool.add(Vector3::new(29.7916, 0.0, 19.9838)); // 469
    let second = pool.add(Vector3::new(29.7969, 0.0, 19.9886)); // 551
    let third = pool.add(Vector3::new(29.7958, 0.0, 19.9877)); // 476

    assert!(
        surface_fan::covers_nothing(&[first, second, third], &pool),
        "the three stand on one line to within the ruler, so two rings fanning them would emit the same triangle twice"
    );
}

#[test]
fn a_fan_takes_no_diagonal_through_a_corner_of_its_own_ring() {
    // A hundred across, so the ruler is a thousandth.
    let mut pool = SurfaceVertexPool::new(&[
        Vector3Float::new(0.0, 0.0, 0.0),
        Vector3Float::new(57.735, 57.735, 57.735),
    ]);

    let apex = pool.add(Vector3::new(0.5, 0.0, 0.004)); // 186
    let from = pool.add(Vector3::new(0.0, 0.0, 0.0)); // 559
    let split = pool.add(Vector3::new(0.5, 0.0, 0.0009)); // 476
    let to = pool.add(Vector3::new(1.0, 0.0, 0.0)); // 469

    let mut faces = Vec::new();
    surface_fan::emit(&[apex, from, split, to], &mut pool, &mut faces);

    assert_eq!(
        uses(&faces, from, to),
        0,
        "a diagonal from 559 to 469 spans the split standing between them"
    );
    assert_eq!(
        uses(&faces, from, split),
        1,
        "the split is a corner of the triangle on its own side"
    );
    assert_eq!(uses(&faces, split, to), 1, "and of the one beyond it");
}

#[test]
fn a_collinear_ring_is_fanned_from_a_farthest_corner_rather_than_a_minted_middle() {
    let mut pool = SurfaceVertexPool::new(&[
        Vector3Float::new(0.0, 0.0, 0.0),
        Vector3Float::new(20.0, 20.0, 20.0),
    ]);

    let first = pool.add(Vector3::new(0.0, 20.0, 20.0));
    let middle = pool.add(Vector3::new(10.0, 10.0, 20.0));
    let landed = pool.add(Vector3::new(17.0, 3.0, 20.0)); // the band end's tangent point, knitted in
    let last = pool.add(Vector3::new(20.0, 0.0, 20.0));

    let ring = [first, middle, landed, last];
    let minted = pool.vertices().len();
    let mut faces = Vec::new();
    surface_fan::emit(&ring, &mut pool, &mut faces);

    assert_eq!(
        pool.vertices().len(),
        minted,
        "a ring covering nothing has no middle worth minting - its neighbours would know nothing of the new point"
    );
    assert_eq!(
        faces.len(),
        2,
        "four corners fan to two triangles - instead {}",
        listed(&faces)
    );
    for f in &faces {
        assert!(
            ring.contains(&f.0) && ring.contains(&f.1) && ring.contains(&f.2),
            "({},{},{}) may only use the ring's own corners",
            f.0,
            f.1,
            f.2
        );
    }
    for (from, to) in [
        (first, middle),
        (middle, landed),
        (landed, last),
        (last, first),
    ] {
        assert_eq!(
            uses(&faces, from, to),
            1,
            "every side of the ring is used exactly once, so the faces across it pair with these triangles exactly as they would have with the needle"
        );
    }
}

#[test]
fn a_collapsed_strip_quad_is_emitted_as_needles_along_its_own_chord() {
    let mut pool = SurfaceVertexPool::new(&[
        Vector3Float::new(0.0, 0.0, 0.0),
        Vector3Float::new(20.0, 20.0, 20.0),
    ]);

    let at = Vector3::new(10.9600, 7.4300, 11.6200);
    let chord = Vector3::new(10.2900, 7.5000, 11.6700) - at;
    let off = chord.cross(Vector3::UNIT_Z).get_normal() * 1e-7;

    // The two rails of the quad, each clamped onto the vertex the station before it already had.
    let rail = pool.add(at);
    let other_rail = pool.add(at + chord);

    // And the point each station carries along that chord, drifted off it by a float's worth.
    let here = pool.add(at + (chord * 0.3) + off);
    let there = pool.add(at + (chord * 0.6) - off);

    let ring = [rail, here, other_rail, there];
    let minted = pool.vertices().len();
    let mut faces = Vec::new();
    surface_fan::emit_covering_nothing(&ring, &pool, &mut faces);

    assert_eq!(
        pool.vertices().len(),
        minted,
        "nothing is minted for a ring that covers nothing"
    );
    assert_eq!(
        faces.len(),
        2,
        "four corners fan to two needles - instead {}",
        listed(&faces)
    );
    for f in &faces {
        assert!(
            f.0 != f.1 && f.1 != f.2 && f.2 != f.0,
            "({},{},{}) would have two corners at one pooled vertex",
            f.0,
            f.1,
            f.2
        );
        assert!(
            ring.contains(&f.0) && ring.contains(&f.1) && ring.contains(&f.2),
            "({},{},{}) may only use the ring's own corners",
            f.0,
            f.1,
            f.2
        );
    }
    for (from, to) in [
        (rail, here),
        (here, other_rail),
        (other_rail, there),
        (there, rail),
    ] {
        assert_eq!(
            uses(&faces, from, to),
            1,
            "every side of the ring is used exactly once, so the faces across it pair with these needles exactly as they would have with the ring as one face"
        );
    }
    assert_eq!(
        uses(&faces, rail, other_rail),
        2,
        "the fan's one diagonal is the chord itself, used once each way, which is what lets the needles cancel against each other rather than opening a seam"
    );
}

#[test]
fn a_collapsed_quad_that_doubles_back_on_itself_emits_nothing() {
    let mut pool = SurfaceVertexPool::new(&[
        Vector3Float::new(0.0, 0.0, 0.0),
        Vector3Float::new(20.0, 20.0, 20.0),
    ]);

    let at = Vector3::new(10.9600, 7.4300, 11.6200);
    let chord = Vector3::new(10.2900, 7.5000, 11.6700) - at;

    let rail = pool.add(at);
    let other_rail = pool.add(at + chord);
    let shared = pool.add(at + (chord * 0.5));

    let mut faces = Vec::new();
    surface_fan::emit_covering_nothing(&[rail, shared, other_rail, shared], &pool, &mut faces);

    assert_eq!(
        faces.len(),
        0,
        "both of its triangles would be the same three vertices - instead {}",
        listed(&faces)
    );
}

fn uses(faces: &[(i32, i32, i32)], first: i32, second: i32) -> i32 {
    let mut uses = 0;
    for &(v0, v1, v2) in faces {
        for (from, to) in [(v0, v1), (v1, v2), (v2, v0)] {
            if (from == first && to == second) || (from == second && to == first) {
                uses += 1;
            }
        }
    }
    uses
}
