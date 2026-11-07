//! A slab whose LEDGE narrows - a block standing on a plate, with the strip of plate in front of
//! the block (the ledge the foot crease's band lands on) tapering to a pinch, either along the
//! block or to the middle of it and out again. Built for the partial run-off readings
//! (`surface_room_partial_run_off_tests.rs`): a band that eats the ledge whole over a stretch and
//! runs off its rim onto the plate's tapering side.

#![allow(dead_code)]

use polygon_mesh::Mesh;
use vector_math::Vector3;

/// How tall the block stands above the plate's top.
pub const BLOCK_HEIGHT: f64 = 8.0;

/// How thick the plate is below its top at z = 0.
pub const PLATE_THICKNESS: f64 = 4.0;

/// Where the plate's far side stands, behind the block.
pub const FAR_SIDE: f64 = 10.0;

/// How wide the ledge is at the block's start.
pub const WIDTH_AT_THE_RIM: f64 = 6.0;

/// How much width the ledge loses per millimetre along the block.
pub const TAPER_RATE: f64 = 0.5;

/// The width the ledge narrows to.
pub const PINCH: f64 = 1.0;

/// How long the block is, with the default sizes.
pub fn length_of() -> f64 {
    length_of_with(WIDTH_AT_THE_RIM, TAPER_RATE, PINCH, false)
}

/// How long the block is - as long as the taper takes to reach the pinch, twice that
/// where it is pinched in the middle.
pub fn length_of_with(
    width_at_the_rim: f64,
    taper_rate: f64,
    pinch: f64,
    pinched_in_the_middle: bool,
) -> f64 {
    (width_at_the_rim - pinch) / taper_rate * (if pinched_in_the_middle { 2.0 } else { 1.0 })
}

/// The ledge's width at `x`.
pub fn width_at(
    x: f64,
    width_at_the_rim: f64,
    taper_rate: f64,
    pinch: f64,
    pinched_in_the_middle: bool,
) -> f64 {
    let length = length_of_with(width_at_the_rim, taper_rate, pinch, pinched_in_the_middle);

    width_at_the_rim
        - (taper_rate
            * (if pinched_in_the_middle {
                x.min(length - x)
            } else {
                x
            }))
}

/// The wall tapering to a knife edge, with the default sizes.
pub fn create_wall_tapering_to_a_knife_edge(pinched_in_the_middle: bool) -> Mesh {
    create_wall_tapering_to_a_knife_edge_with(
        WIDTH_AT_THE_RIM,
        TAPER_RATE,
        PINCH,
        pinched_in_the_middle,
    )
}

/// The ledge narrowing from `width_at_the_rim` to `pinch`
/// along the block, or to the middle of it and out again.
pub fn create_wall_tapering_to_a_knife_edge_with(
    width_at_the_rim: f64,
    taper_rate: f64,
    pinch: f64,
    pinched_in_the_middle: bool,
) -> Mesh {
    let length = length_of_with(width_at_the_rim, taper_rate, pinch, pinched_in_the_middle);

    if pinched_in_the_middle {
        create_wall_of_widths(&[
            (0.0, width_at_the_rim),
            (length / 2.0, pinch),
            (length, width_at_the_rim),
        ])
    } else {
        create_wall_of_widths(&[(0.0, width_at_the_rim), (length, pinch)])
    }
}

/// The slab with its ledge `(at, width)` wide at each station, linear
/// between them; the block runs from the first station to the last.
pub fn create_wall_of_widths(stations: &[(f64, f64)]) -> Mesh {
    let mut mesh = Mesh::new();
    let length = stations[stations.len() - 1].0;

    let w = |x: f64| {
        for at in 1..stations.len() {
            if x <= stations[at].0 || at == stations.len() - 1 {
                let run = stations[at].0 - stations[at - 1].0;

                return stations[at - 1].1
                    + ((stations[at].1 - stations[at - 1].1) * ((x - stations[at - 1].0) / run));
            }
        }

        stations[0].1
    };

    // The narrowest station, which is the one corner every face that follows the width can be
    // fanned from: on a ledge pinched in the middle it is the only corner that sees the whole face,
    // and on every other shape here it sees it as well.
    let mut narrowest = 0;

    for at in 1..stations.len() {
        if stations[at].1 < stations[narrowest].1 {
            narrowest = at;
        }
    }

    let bends_at = stations[narrowest].0;
    let the_ends: Vec<f64> = stations.iter().map(|station| station.0).collect();

    let mut face = |outward: Vector3, mut fan_from: usize, points: &[Vector3]| {
        let mut wound: Vec<Vector3> = points.to_vec();

        // Newell's normal of the polygon as handed over; reverse it where that disagrees with the
        // outward normal asked for, so that a face is described by where it is and never by the
        // order somebody happened to list its corners in.
        let mut normal = Vector3::ZERO;

        for i in 0..wound.len() {
            let a = wound[i];
            let b = wound[(i + 1) % wound.len()];

            normal += Vector3::new(
                (a.y - b.y) * (a.z + b.z),
                (a.z - b.z) * (a.x + b.x),
                (a.x - b.x) * (a.y + b.y),
            );
        }

        if normal.dot(outward) < 0.0 {
            wound.reverse();
            fan_from = wound.len() - 1 - fan_from;
        }

        // Fanned from the corner the caller names, because the faces that follow a taper pinched in
        // the middle are NOT convex: the one corner the taper bends at is the only one that sees
        // the whole of such a face, and it is the one every one of them is fanned from.
        for i in 1..wound.len() - 1 {
            mesh.create_face(&[
                wound[fan_from],
                wound[(fan_from + i) % wound.len()],
                wound[(fan_from + i + 1) % wound.len()],
            ]);
        }
    };

    // One strip of face at one height, between two y readings of x, cornered where the taper bends.
    let strip = |face: &mut dyn FnMut(Vector3, usize, &[Vector3]),
                 outward: Vector3,
                 z: f64,
                 near: &dyn Fn(f64) -> f64,
                 far: &dyn Fn(f64) -> f64| {
        let mut points: Vec<Vector3> = Vec::new();

        for &x in &the_ends {
            points.push(Vector3::new(x, near(x), z));
        }

        for at in (0..the_ends.len()).rev() {
            points.push(Vector3::new(the_ends[at], far(the_ends[at]), z));
        }

        // A station on the side of the strip that does NOT follow the taper is no corner of it, and
        // a point in the middle of a straight edge is a point the face BEYOND that edge has not
        // got.
        for at in (0..points.len()).rev() {
            let before = points[(at + points.len() - 1) % points.len()];
            let after = points[(at + 1) % points.len()];

            if (points[at] - before).cross(after - points[at]).length() < 1e-9 {
                points.remove(at);
            }
        }

        // And the narrowest station is the corner the face is fanned from, because it is the one
        // that sees the whole of it.
        let mut bend = 0;

        for (at, point) in points.iter().enumerate() {
            if (point.x - bends_at).abs() < 1e-9 {
                bend = at;
            }
        }

        face(outward, bend, &points);
    };

    // The ledge the bands land on, and the plate's far top, and the block's own top.
    strip(&mut face, Vector3::UNIT_Z, 0.0, &|x| -w(x), &|_| 0.0);
    strip(&mut face, Vector3::UNIT_Z, 0.0, &w, &|_| FAR_SIDE);
    strip(&mut face, Vector3::UNIT_Z, BLOCK_HEIGHT, &|_| 0.0, &w);

    // The plate's underside, split at y = 0 and y = w(x) so that its corners meet the end faces'.
    strip(
        &mut face,
        -Vector3::UNIT_Z,
        -PLATE_THICKNESS,
        &|x| -w(x),
        &|_| 0.0,
    );
    strip(&mut face, -Vector3::UNIT_Z, -PLATE_THICKNESS, &|_| 0.0, &w);
    strip(&mut face, -Vector3::UNIT_Z, -PLATE_THICKNESS, &w, &|_| {
        FAR_SIDE
    });

    // The two faces the taper cuts - the plate's near side under the ledge's rim, and the block's
    // far side - one quad per station because a taper that bends is two planes and not one.
    for at in 0..the_ends.len() - 1 {
        let x0 = the_ends[at];
        let x1 = the_ends[at + 1];
        let outward = Vector3::new(w(x1) - w(x0), -(x1 - x0), 0.0).get_normal();

        face(
            outward,
            0,
            &[
                Vector3::new(x0, -w(x0), -PLATE_THICKNESS),
                Vector3::new(x1, -w(x1), -PLATE_THICKNESS),
                Vector3::new(x1, -w(x1), 0.0),
                Vector3::new(x0, -w(x0), 0.0),
            ],
        );

        face(
            -outward,
            0,
            &[
                Vector3::new(x0, w(x0), 0.0),
                Vector3::new(x1, w(x1), 0.0),
                Vector3::new(x1, w(x1), BLOCK_HEIGHT),
                Vector3::new(x0, w(x0), BLOCK_HEIGHT),
            ],
        );
    }

    // The block's near side, which the ledge's crease is the foot of, and the plate's far side.
    face(
        -Vector3::UNIT_Y,
        0,
        &[
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(length, 0.0, 0.0),
            Vector3::new(length, 0.0, BLOCK_HEIGHT),
            Vector3::new(0.0, 0.0, BLOCK_HEIGHT),
        ],
    );

    face(
        Vector3::UNIT_Y,
        0,
        &[
            Vector3::new(0.0, FAR_SIDE, -PLATE_THICKNESS),
            Vector3::new(length, FAR_SIDE, -PLATE_THICKNESS),
            Vector3::new(length, FAR_SIDE, 0.0),
            Vector3::new(0.0, FAR_SIDE, 0.0),
        ],
    );

    // And the two ends, each in the four pieces its neighbours leave it in.
    for x in [0.0, length] {
        let outward = if x > 0.0 {
            Vector3::UNIT_X
        } else {
            -Vector3::UNIT_X
        };
        let width = w(x);

        let mut panel = |from_y: f64, to_y: f64, from_z: f64, to_z: f64| {
            face(
                outward,
                0,
                &[
                    Vector3::new(x, from_y, from_z),
                    Vector3::new(x, to_y, from_z),
                    Vector3::new(x, to_y, to_z),
                    Vector3::new(x, from_y, to_z),
                ],
            );
        };

        panel(-width, 0.0, -PLATE_THICKNESS, 0.0);
        panel(0.0, width, -PLATE_THICKNESS, 0.0);
        panel(width, FAR_SIDE, -PLATE_THICKNESS, 0.0);
        panel(0.0, width, 0.0, BLOCK_HEIGHT);
    }

    drop(face);
    mesh.clean_and_merge();

    mesh
}
