//! The barrel-port half of the bore test meshes (`create_barrel_with_a_radial_port` and
//! everything it lays): a segment of the "Many Holes" flash hider's BARREL with one radial PORT
//! bored through it, at the part's own numbers - a 124 sided barrel of radius 16.3565 with a 64
//! facet bore of radius 1.484 drilled square through its 3 mm wall, the barrel's axis along y and
//! the port's down z. Built exactly, with no boolean: every face lies in the plane of the facet it
//! belongs to. The plate bores are in `bore_test_meshes.rs`, the edge bore in
//! `bore_test_meshes_edge_bore.rs`.

use solidmesh_math::{Vector2, Vector3};
use solidmesh_mesh::Mesh;

/// How many facets round the barrel of the flash hider the port fixture is cut from.
pub const BARREL_SIDES: i32 = 124;

/// That barrel's circumradius, which is 2.903 degrees and 0.829 mm to a facet.
pub const BARREL_RADIUS: f64 = 16.3565;

/// How many facets round one of its ports.
pub const PORT_SIDES: i32 = 64;

/// A port's circumradius, which is 5.625 degrees and 0.146 mm to a facet.
pub const PORT_RADIUS: f64 = 1.484;

/// How thick the barrel wall the port is drilled through is.
pub const BARREL_WALL_THICKNESS: f64 = 3.0;

/// How many whole barrel facets stand either side of the port's own, which sets the land.
pub const BARREL_FACETS_EITHER_SIDE: i32 = 8;

/// How far the segment runs along the barrel's axis either way from the port's centre.
pub const BARREL_HALF_LENGTH: f64 = 6.0;

const TAU: f64 = 2.0 * std::f64::consts::PI;

/// Banker's rounding, then truncation, so a value on a half step always rounds the same way.
fn round_to_int(value: f64) -> i32 {
    value.round_ties_even() as i32
}

/// The barrel with a radial port at its default arguments.
pub fn create_barrel_with_a_radial_port() -> Mesh {
    create_barrel_with_a_radial_port_sized(
        BARREL_SIDES,
        BARREL_RADIUS,
        PORT_SIDES,
        PORT_RADIUS,
        BARREL_WALL_THICKNESS,
        BARREL_FACETS_EITHER_SIDE,
        BARREL_HALF_LENGTH,
    )
}

/// The segment closed off flat underneath and square at its four
/// sides; the port runs from its mouth down through the flat underside, so the only loop of the
/// barrel's own rim with a curved surface on both sides is the mouth itself.
pub fn create_barrel_with_a_radial_port_sized(
    barrel_sides: i32,
    barrel_radius: f64,
    port_sides: i32,
    port_radius: f64,
    wall_thickness: f64,
    facets_either_side: i32,
    half_length: f64,
) -> Mesh {
    let step = TAU / f64::from(barrel_sides);
    let inradius = barrel_radius * (step / 2.0).cos();
    let underside = inradius - wall_thickness;

    let mut mesh = Mesh::new();
    let (mouth, at_each_ruling) =
        the_mouth_of_the_radial_port(barrel_sides, barrel_radius, port_sides, port_radius);

    // One rim serves all three: the barrel stops at it, the four sides drop from it and the
    // underside is bounded by its own shadow, so no two of them can disagree about where a vertex
    // of it is.
    let rim = the_barrels_own_rim(
        barrel_sides,
        barrel_radius,
        facets_either_side,
        half_length,
        TAU / mouth.len() as f64 / 2.0,
    );

    lay_the_bore_wall(
        &mut mesh,
        &mouth,
        &at_each_ruling,
        port_sides,
        port_radius,
        underside,
    );
    lay_the_underside(
        &mut mesh,
        &rim,
        barrel_sides,
        barrel_radius,
        port_sides,
        port_radius,
        underside,
    );
    lay_the_barrel_around_the_port(&mut mesh, &mouth, &rim, barrel_sides, barrel_radius);
    lay_the_four_sides(&mut mesh, &rim, barrel_sides, barrel_radius, underside);

    mesh.clean_and_merge();

    mesh
}

/// The port mouth as the exact intersection of the two faceted
/// cylinders - the hit of each bore ruling on the barrel, and between two of those every crossing
/// of a barrel crease by that bore facet's own plane. Answers the loop and where in it each
/// ruling's hit sits.
pub fn the_mouth_of_the_radial_port(
    barrel_sides: i32,
    barrel_radius: f64,
    port_sides: i32,
    port_radius: f64,
) -> (Vec<Vector3>, Vec<usize>) {
    let step = TAU / f64::from(barrel_sides);
    let inradius = barrel_radius * (step / 2.0).cos();
    let bore_inradius = port_radius * (std::f64::consts::PI / f64::from(port_sides)).cos();
    let sides = port_sides as usize;
    let mut hits = Vec::with_capacity(sides);

    for j in 0..port_sides {
        let turn = TAU * f64::from(j) / f64::from(port_sides);
        let x = port_radius * turn.cos();
        let mut z = inradius;

        // Which facet the ruling comes out on depends on where it comes out, so the facet and its
        // own plane are settled together; four passes is more than a 1.79 facet reach ever needs.
        for _ in 0..4 {
            let facet = f64::from(round_to_int(x.atan2(z) / step));

            z = (inradius - (x * (facet * step).sin())) / (facet * step).cos();
        }

        hits.push(Vector3::new(x, port_radius * turn.sin(), z));
    }

    let mut ring = Vec::new();
    let mut starts = vec![0; sides];

    for j in 0..sides {
        starts[j] = ring.len();
        ring.push(hits[j]);

        // Every barrel crease between the two rulings' own facets is crossed by this bore facet's
        // plane, and where it crosses is a point of both facet planes and so exactly on both.
        let psi = TAU * (j as f64 + 0.5) / f64::from(port_sides);
        let from = facet_the_barrel_point_is_on(hits[j], step);
        let to = facet_the_barrel_point_is_on(hits[(j + 1) % sides], step);

        let crossing_of = |crease: i32| {
            let crease_turn = (f64::from(crease) + 0.5) * step;
            let crease_x = barrel_radius * crease_turn.sin();

            Vector3::new(
                crease_x,
                (bore_inradius - (psi.cos() * crease_x)) / psi.sin(),
                barrel_radius * crease_turn.cos(),
            )
        };

        if to > from {
            for crease in from..to {
                ring.push(crossing_of(crease));
            }
        } else {
            for crease in (to..from).rev() {
                ring.push(crossing_of(crease));
            }
        }
    }

    (ring, starts)
}

/// Which barrel facet a point of the barrel's surface lies in, by the angle it stands at.
fn facet_the_barrel_point_is_on(at: Vector3, step: f64) -> i32 {
    round_to_int(at.x.atan2(at.z) / step)
}

/// The port's wall, one facet at a time: its two rulings' feet on the flat underside and, above
/// them, the stretch of the mouth that is this facet's own.
fn lay_the_bore_wall(
    mesh: &mut Mesh,
    mouth: &[Vector3],
    at_each_ruling: &[usize],
    port_sides: i32,
    port_radius: f64,
    underside: f64,
) {
    let foot_of_the_ruling = |j: usize| {
        let turn = TAU * j as f64 / f64::from(port_sides);

        Vector3::new(
            port_radius * turn.cos(),
            port_radius * turn.sin(),
            underside,
        )
    };
    let sides = port_sides as usize;

    for j in 0..sides {
        let next = (j + 1) % sides;
        let psi = TAU * (j as f64 + 0.5) / f64::from(port_sides);
        let mut wall = vec![foot_of_the_ruling(j), foot_of_the_ruling(next)];
        let mut at = at_each_ruling[next];

        loop {
            wall.push(mouth[at]);

            if at == at_each_ruling[j] {
                break;
            }

            at = (at + mouth.len() - 1) % mouth.len();
        }

        // A bore wall's own normal points IN toward the drill's axis, the material of a barrel
        // being outside its ports.
        emit_the_face_facing(mesh, &wall, Vector3::new(-psi.cos(), -psi.sin(), 0.0));
    }
}

/// The flat underside, between the port's plain polygon and the rim's own shadow.
fn lay_the_underside(
    mesh: &mut Mesh,
    rim: &[Vector2],
    barrel_sides: i32,
    barrel_radius: f64,
    port_sides: i32,
    port_radius: f64,
    underside: f64,
) {
    let hole: Vec<Vector2> = (0..port_sides)
        .map(|j| {
            let turn = TAU * f64::from(j) / f64::from(port_sides);

            Vector2::new(port_radius * turn.cos(), port_radius * turn.sin())
        })
        .collect();

    let sides: Vec<Vector2> = rim
        .iter()
        .map(|&at| {
            let stood = the_rim_point_for(at, barrel_sides, barrel_radius);

            Vector2::new(stood.x, stood.y)
        })
        .collect();

    walk_the_ring_between(&hole, &sides, |a, b, c| {
        emit_the_face_facing(
            mesh,
            &[
                Vector3::new(a.x, a.y, underside),
                Vector3::new(b.x, b.y, underside),
                Vector3::new(c.x, c.y, underside),
            ],
            Vector3::new(0.0, 0.0, -1.0),
        );
    });
}

/// The barrel's own surface around the port, walked in the unrolled (arc length, axis) plane and
/// cut at every barrel crease it crosses so that no face of it chords between facet planes.
fn lay_the_barrel_around_the_port(
    mesh: &mut Mesh,
    mouth: &[Vector3],
    rim: &[Vector2],
    barrel_sides: i32,
    barrel_radius: f64,
) {
    let step = TAU / f64::from(barrel_sides);
    let inradius = barrel_radius * (step / 2.0).cos();
    let arc_step = inradius * step;

    let flat: Vec<Vector2> = mouth
        .iter()
        .map(|at| Vector2::new(inradius * at.x.atan2(at.z), at.y))
        .collect();

    walk_the_ring_between(&flat, rim, |a, b, c| {
        emit_the_barrel_piece_of(mesh, vec![a, b, c], arc_step, inradius, step);
    });
}

/// Where the barrel's own surface stops, in the unrolled plane: the four sides of the segment,
/// carrying a vertex at every crease along the two that run ACROSS the barrel, and split further
/// wherever a step of it would turn further about the port than `biggest_turn`.
fn the_barrels_own_rim(
    barrel_sides: i32,
    barrel_radius: f64,
    facets_either_side: i32,
    half_length: f64,
    biggest_turn: f64,
) -> Vec<Vector2> {
    let step = TAU / f64::from(barrel_sides);
    let arc_step = barrel_radius * (step / 2.0).cos() * step;
    let edge_arc = (f64::from(facets_either_side) + 0.5) * arc_step;
    let mut rim = vec![
        Vector2::new(edge_arc, -half_length),
        Vector2::new(edge_arc, half_length),
    ];

    for k in (-facets_either_side..facets_either_side).rev() {
        rim.push(Vector2::new((f64::from(k) + 0.5) * arc_step, half_length));
    }

    rim.push(Vector2::new(-edge_arc, half_length));
    rim.push(Vector2::new(-edge_arc, -half_length));

    for k in -facets_either_side..facets_either_side {
        rim.push(Vector2::new((f64::from(k) + 0.5) * arc_step, -half_length));
    }

    subdivided_by_angle(&rim, biggest_turn)
}

/// A loop with a vertex inserted wherever two of its own turn further about the origin than the
/// limit, which the ring walk needs of its outer loop.
fn subdivided_by_angle(ring: &[Vector2], biggest_turn: f64) -> Vec<Vector2> {
    let mut split = Vec::new();

    for i in 0..ring.len() {
        let from = ring[i];
        let to = ring[(i + 1) % ring.len()];
        let turn = ((from.x * to.y) - (from.y * to.x))
            .atan2((from.x * to.x) + (from.y * to.y))
            .abs();
        let pieces = ((turn / biggest_turn).ceil() as i32).max(1);

        for piece in 0..pieces {
            split.push(from + ((to - from) * (f64::from(piece) / f64::from(pieces))));
        }
    }

    split
}

/// One walked triangle of the barrel, cut at each crease that runs through it and each piece laid
/// in the plane of the facet it fell in.
fn emit_the_barrel_piece_of(
    mesh: &mut Mesh,
    triangle: Vec<Vector2>,
    arc_step: f64,
    inradius: f64,
    step: f64,
) {
    let low = triangle[0].x.min(triangle[1].x.min(triangle[2].x));
    let high = triangle[0].x.max(triangle[1].x.max(triangle[2].x));
    let mut pieces = Vec::new();
    let mut left = triangle;

    for k in ((low / arc_step).floor() as i32 - 1)..=((high / arc_step).ceil() as i32 + 1) {
        let crease = (f64::from(k) + 0.5) * arc_step;

        if crease <= low || crease >= high || left.len() < 3 {
            continue;
        }

        let below = cut_to_one_side_of(&left, crease, true);

        if below.len() >= 3 {
            pieces.push(below);
        }

        left = cut_to_one_side_of(&left, crease, false);
    }

    if left.len() >= 3 {
        pieces.push(left);
    }

    for piece in pieces {
        let mut middle = 0.0;

        for corner in &piece {
            middle += corner.x / piece.len() as f64;
        }

        let facet = round_to_int(middle / arc_step);
        let laid: Vec<Vector3> = piece
            .iter()
            .map(|&at| on_the_barrel_at(at, facet, inradius, step))
            .collect();

        let turn = f64::from(facet) * step;

        emit_the_face_facing(mesh, &laid, Vector3::new(turn.sin(), 0.0, turn.cos()));
    }
}

/// The part of a convex polygon on one side of a line of constant x, in the unrolled plane.
fn cut_to_one_side_of(polygon: &[Vector2], at: f64, below: bool) -> Vec<Vector2> {
    let mut kept = Vec::new();

    for i in 0..polygon.len() {
        let from = polygon[i];
        let to = polygon[(i + 1) % polygon.len()];
        let from_in = if below { from.x <= at } else { from.x >= at };
        let to_in = if below { to.x <= at } else { to.x >= at };

        if from_in {
            kept.push(from);
        }

        if from_in != to_in {
            let span = to.x - from.x;
            let along = if span.abs() > 1e-15 {
                (at - from.x) / span
            } else {
                0.0
            };

            kept.push(Vector2::new(at, from.y + ((to.y - from.y) * along)));
        }
    }

    kept
}

/// The segment's four square sides, taken as one skirt: each step of the barrel's own rim dropped
/// straight down to the flat underside.
fn lay_the_four_sides(
    mesh: &mut Mesh,
    rim: &[Vector2],
    barrel_sides: i32,
    barrel_radius: f64,
    underside: f64,
) {
    for i in 0..rim.len() {
        let from = the_rim_point_for(rim[i], barrel_sides, barrel_radius);
        let to = the_rim_point_for(rim[(i + 1) % rim.len()], barrel_sides, barrel_radius);
        let run = Vector3::new(to.x - from.x, to.y - from.y, 0.0);
        let mut facing = Vector3::new(run.y, -run.x, 0.0);
        let middle = Vector3::new(from.x + to.x, from.y + to.y, 0.0);

        if facing.dot(middle) < 0.0 {
            facing = -facing;
        }

        emit_the_face_facing(
            mesh,
            &[
                from,
                to,
                Vector3::new(to.x, to.y, underside),
                Vector3::new(from.x, from.y, underside),
            ],
            facing,
        );
    }
}

/// Where one point of the barrel's rim stands, on the facet its own turn falls in.
fn the_rim_point_for(at: Vector2, barrel_sides: i32, barrel_radius: f64) -> Vector3 {
    let step = TAU / f64::from(barrel_sides);
    let inradius = barrel_radius * (step / 2.0).cos();

    on_the_barrel_at(at, round_to_int(at.x / (inradius * step)), inradius, step)
}

/// Where a point of the unrolled plane lands on one barrel facet's own plane.
fn on_the_barrel_at(flat: Vector2, facet: i32, inradius: f64, step: f64) -> Vector3 {
    let turn = flat.x / inradius;

    // The ray out of the axis at this turn meets that facet's plane a secant of the inradius away.
    let reach = inradius / (turn - (f64::from(facet) * step)).cos();

    Vector3::new(reach * turn.sin(), flat.y, reach * turn.cos())
}

/// One planar polygon, wound so that it faces the way it is meant to, its winding read by Newell's
/// sum.
fn emit_the_face_facing(mesh: &mut Mesh, polygon: &[Vector3], facing: Vector3) {
    let mut points = polygon.to_vec();
    let mut wound = Vector3::new(0.0, 0.0, 0.0);

    for i in 0..points.len() {
        wound += points[i].cross(points[(i + 1) % points.len()]);
    }

    if wound.dot(facing) < 0.0 {
        points.reverse();
    }

    mesh.create_face(&points);
}

/// The ring walk for this fixture: the ring between an inner loop and an outer one,
/// both wound the same way about the origin and both containing it, taken by whichever loop's next
/// vertex comes first in ANGLE about it.
fn walk_the_ring_between(
    inner: &[Vector2],
    outer: &[Vector2],
    mut emit: impl FnMut(Vector2, Vector2, Vector2),
) {
    let sides = inner.len();
    let rim_sides = outer.len();
    let start = inner[0].y.atan2(inner[0].x);
    let ahead = |at: Vector2| {
        let turn = at.y.atan2(at.x) - start;

        if turn < 0.0 {
            turn + TAU
        } else {
            turn
        }
    };

    let mut walk = vec![0.0; sides + 1];

    for k in 1..sides {
        walk[k] = ahead(inner[k]);
    }

    walk[sides] = TAU;

    let mut first = 0;
    let mut least = f64::MAX;

    for (k, &at) in outer.iter().enumerate() {
        let turn = ahead(at);

        if turn > 0.0 && turn < least {
            least = turn;
            first = k;
        }
    }

    let mut round = vec![0.0; rim_sides + 1];
    round[0] = least;

    for k in 1..=rim_sides {
        let turn = ahead(outer[(first + k) % rim_sides]);
        round[k] = if turn <= round[k - 1] {
            turn + TAU
        } else {
            turn
        };
    }

    let mut walked = 0;
    let mut turned = 0;

    while walked < sides || turned < rim_sides {
        if walked < sides && (turned >= rim_sides || walk[walked + 1] <= round[turned + 1]) {
            emit(
                inner[walked],
                outer[(first + turned) % rim_sides],
                inner[(walked + 1) % sides],
            );
            walked += 1;
        } else {
            emit(
                inner[walked % sides],
                outer[(first + turned) % rim_sides],
                outer[(first + turned + 1) % rim_sides],
            );
            turned += 1;
        }
    }
}
