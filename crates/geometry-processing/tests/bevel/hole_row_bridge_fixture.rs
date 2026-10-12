//! Port of the test fixture `LevelLine/HoleRowBridgeFixture.cs`: a ROW of round through-holes in
//! one stadium plate, every adjacent pair joined by a strip of triangles that SPAN the two rims.
//! It is `two_hole_bridge_fixture` grown along the three knobs the Many Holes witness has and the
//! two-hole part does not: MORE chains on the one face (a row of four or six, so a middle hole is
//! spanned from BOTH sides at once), ASYMMETRY (a ring rotated by half a segment, two rings of
//! different segment counts, a hole shifted off the row's pitch, so no spanning chord is horizontal
//! and the strip's triangles are lopsided slivers), and the plate's OUTER RIM picked as a chain of
//! its own - the witness's chain 0, which the biggest mixed loop of the witness ran through.
//!
//! The row is tiled with no ear clipping and no T-vertex: each hole's ring is cut at the vertex
//! nearest +90 degrees and the one nearest -90, which leaves a LEFT arc and a RIGHT arc; the region
//! between hole k and hole k+1 is the strip between k's right arc and k+1's left arc, the two END
//! holes wear a radial cap out to the plate's boundary, and one quad above and one below each strip
//! closes the stadium. Every hole is spanned on both of its halves, so a middle hole's rim carries
//! long edges to two different chains.
//!
//! The strip between two arcs is walked greedily - at each step the shorter of the two diagonals -
//! rather than quad by quad, because once a ring is rotated, re-segmented or shifted the facing
//! points no longer pair up and a quad tiling would leave slivers of the wrong shape or not tile at
//! all. Greedy gives the witness's own shape of triangle: a long cross-chain edge with the field
//! rising over its middle while both its ends read exactly zero. Read by `hole_row_bridge_tests`.

use std::collections::HashMap;
use std::f64::consts::PI;

use geometry_processing::bevel::level_line::BandGapResult;
use vector_math::Vector3;

use crate::level_line_room_geometry::SolidWithChains;
use crate::two_hole_bridge_fixture;

/// The size the fixture is about. Every other length is quoted as a multiple of it.
pub const SIZE: f64 = two_hole_bridge_fixture::SIZE;

/// Each hole's radius, in sizes.
pub const RADIUS_IN_SIZES: f64 = two_hole_bridge_fixture::RADIUS_IN_SIZES;

/// How far neighbouring rims stand apart, edge to edge, in sizes - shared between two bands.
pub const GAP_IN_SIZES: f64 = two_hole_bridge_fixture::GAP_IN_SIZES;

/// How much plate there is outside the end holes' arcs and above and below the row, in sizes.
pub const MARGIN_IN_SIZES: f64 = 6.0;

/// Thick enough that no chain's own wall bounds it at `SIZE`.
const THICKNESS: f64 = 3.0;

/// C# `HoleVariation`: how one hole of the row differs from the plain ring the others carry.
#[derive(Debug, Clone, Copy, Default)]
pub struct HoleVariation {
    pub hole: i32,
    pub ring_segments: Option<i32>,
    pub turn_in_segments: f64,
    pub shift_in_segments: f64,
}

impl HoleVariation {
    /// The hole's ring turned by half a segment, so its vertices face its neighbours' edges.
    pub fn turned(hole: i32) -> Self {
        HoleVariation {
            hole,
            turn_in_segments: 0.5,
            ..HoleVariation::default()
        }
    }

    /// The hole's ring cut into a different number of segments than its neighbours.
    pub fn resegmented(hole: i32, ring_segments: i32) -> Self {
        HoleVariation {
            hole,
            ring_segments: Some(ring_segments),
            ..HoleVariation::default()
        }
    }

    /// The hole slid along the row by a third of a segment, which is what breaks the mirror
    /// symmetry the two-hole fixture leans on: no spanning chord stays horizontal and the strip's
    /// triangles come out lopsided, the witness's peak sitting a thousandth of its length off the
    /// edge's middle.
    pub fn shifted(hole: i32) -> Self {
        HoleVariation {
            hole,
            shift_in_segments: 1.0 / 3.0,
            ..HoleVariation::default()
        }
    }
}

/// C# `Pick(holes, variations: ..., includeOuterRim: ...)` with the default 32 ring segments,
/// radius, gap and size: the plate and the chains of its top face - one per hole rim, left to
/// right, and, when `include_outer_rim`, the plate's own outer boundary LAST.
pub fn pick(holes: i32, variations: &[HoleVariation], include_outer_rim: bool) -> SolidWithChains {
    let ring_segments = 32;
    let size = SIZE;
    assert!(holes >= 2, "a bridge wants two rims to span");

    let r = RADIUS_IN_SIZES * size;
    let pitch = (2.0 * r) + (GAP_IN_SIZES * size);
    let outer = r + (MARGIN_IN_SIZES * size);

    let rings: Vec<Ring> = (0..holes)
        .map(|k| {
            let variation = variations
                .iter()
                .find(|v| v.hole == k)
                .copied()
                .unwrap_or_default();
            let segments = variation.ring_segments.unwrap_or(ring_segments);
            let step = 2.0 * PI / segments as f64;
            let centre = (k as f64 * pitch) + (variation.shift_in_segments * r * step);
            Ring::new(centre, r, segments, variation.turn_in_segments * step)
        })
        .collect();

    let mut builder = Builder::default();

    // The strip between neighbours: hole k's right arc and hole k+1's left arc, both walked from
    // the top of the row to the bottom, joined greedily so every triangle has corners on BOTH
    // chains.
    for k in 0..rings.len() - 1 {
        let (a, b) = (&rings[k], &rings[k + 1]);
        builder.strip_between(a, &a.right_arc(), b, &b.left_arc());

        // The quads that close the stadium above and below this strip.
        let top_left = a.outer(a.top_vertex, outer);
        let top_right = b.outer(b.top_vertex, outer);
        let bot_left = a.outer(a.bottom_vertex, outer);
        let bot_right = b.outer(b.bottom_vertex, outer);
        let q = [
            builder.ring0(a, a.top_vertex),
            builder.ring0(b, b.top_vertex),
            builder.at(top_right.0, top_right.1, 0.0),
            builder.at(top_left.0, top_left.1, 0.0),
        ];
        builder.quad(q[0], q[1], q[2], q[3]);
        let q = [
            builder.at(bot_left.0, bot_left.1, 0.0),
            builder.at(bot_right.0, bot_right.1, 0.0),
            builder.ring0(b, b.bottom_vertex),
            builder.ring0(a, a.bottom_vertex),
        ];
        builder.quad(q[0], q[1], q[2], q[3]);
    }

    // The two END holes wear a radial cap out to the plate's boundary: one quad per rim edge,
    // walked with the angle INCREASING, which is the winding the two-hole fixture's caps use.
    let first = &rings[0];
    builder.cap(first, &first.left_arc(), outer);
    let last = &rings[rings.len() - 1];
    let mut last_arc = last.right_arc();
    last_arc.reverse();
    builder.cap(last, &last_arc, outer);

    let top_face = builder.indices.clone();

    // The bottom face is the same tiling at -THICKNESS, wound the other way.
    for (a, b, c) in top_face {
        let pa = builder.positions[a as usize];
        let pb = builder.positions[b as usize];
        let pc = builder.positions[c as usize];
        let t = (
            builder.at(pa.x, pa.y, -THICKNESS),
            builder.at(pc.x, pc.y, -THICKNESS),
            builder.at(pb.x, pb.y, -THICKNESS),
        );
        builder.indices.push(t);
    }

    // Each hole's rim, counter-clockwise seen from +z, and the wall under it facing INTO the hole.
    let mut chains = Vec::new();
    for ring in &rings {
        let rim: Vec<i32> = (0..ring.segments).map(|j| builder.ring0(ring, j)).collect();

        for j in 0..ring.segments {
            let from = (j + 1) % ring.segments;
            let q = [
                builder.at(ring.x(from), ring.y(from), -THICKNESS),
                builder.at(ring.x(j), ring.y(j), -THICKNESS),
                builder.ring0(ring, j),
                builder.ring0(ring, from),
            ];
            builder.quad(q[0], q[1], q[2], q[3]);
        }

        chains.push(rim);
    }

    // The plate's own outer wall, facing out, one quad per boundary edge.
    let boundary = outer_loop(&rings, outer);
    for k in 0..boundary.len() {
        let from = boundary[k];
        let to = boundary[(k + 1) % boundary.len()];
        let q = [
            builder.at(from.0, from.1, -THICKNESS),
            builder.at(to.0, to.1, -THICKNESS),
            builder.at(to.0, to.1, 0.0),
            builder.at(from.0, from.1, 0.0),
        ];
        builder.quad(q[0], q[1], q[2], q[3]);
    }

    if include_outer_rim {
        let rim: Vec<i32> = boundary.iter().map(|p| builder.at(p.0, p.1, 0.0)).collect();
        chains.push(rim);
    }

    (builder.positions, builder.indices, chains)
}

/// C# `Build` (chamfer, its default profile): one JOINT band build of every picked chain - the
/// build the bridge defect shows up in. The same build as the two-hole fixture's.
pub fn build(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    size: f64,
) -> BandGapResult {
    two_hole_bridge_fixture::build(positions, indices, chains, size)
}

/// The welded vertex list and triangles the pick is built into (C#'s locals `positions`,
/// `byPosition`, `indices` and the local functions over them).
#[derive(Default)]
struct Builder {
    positions: Vec<Vector3>,
    by_position: HashMap<(i64, i64, i64), i32>,
    indices: Vec<(i32, i32, i32)>,
}

impl Builder {
    /// C# `At`: rounding to a nanometre welds the pieces that meet; every region shares its
    /// boundary points by construction, so nothing that should stay apart is ever within it.
    fn at(&mut self, x: f64, y: f64, z: f64) -> i32 {
        // C#'s `(long)Math.Round(...)`: banker's rounding, then a cast (in range here).
        let key = (
            (x * 1e6).round_ties_even() as i64,
            (y * 1e6).round_ties_even() as i64,
            (z * 1e6).round_ties_even() as i64,
        );
        if let Some(&index) = self.by_position.get(&key) {
            return index;
        }

        let index = self.positions.len() as i32;
        self.positions.push(Vector3::new(x, y, z));
        self.by_position.insert(key, index);
        index
    }

    fn quad(&mut self, a: i32, b: i32, c: i32, d: i32) {
        self.indices.push((a, b, c));
        self.indices.push((a, c, d));
    }

    fn ring0(&mut self, ring: &Ring, j: i32) -> i32 {
        self.at(ring.x(j), ring.y(j), 0.0)
    }

    /// C# `StripBetween`.
    fn strip_between(&mut self, ring_a: &Ring, arc_a: &[i32], ring_b: &Ring, arc_b: &[i32]) {
        let (mut i, mut j) = (0usize, 0usize);
        while i + 1 < arc_a.len() || j + 1 < arc_b.len() {
            let can_a = i + 1 < arc_a.len();
            let can_b = j + 1 < arc_b.len();
            let step_a = can_a
                && (!can_b
                    || ring_a.to(arc_a[i + 1], ring_b, arc_b[j])
                        <= ring_a.to(arc_a[i], ring_b, arc_b[j + 1]));
            if step_a {
                let t = (
                    self.ring0(ring_a, arc_a[i]),
                    self.ring0(ring_a, arc_a[i + 1]),
                    self.ring0(ring_b, arc_b[j]),
                );
                self.indices.push(t);
                i += 1;
            } else {
                let t = (
                    self.ring0(ring_a, arc_a[i]),
                    self.ring0(ring_b, arc_b[j + 1]),
                    self.ring0(ring_b, arc_b[j]),
                );
                self.indices.push(t);
                j += 1;
            }
        }
    }

    /// C# `Cap`.
    fn cap(&mut self, ring: &Ring, arc: &[i32], outer: f64) {
        for j in 0..arc.len() - 1 {
            let o0 = ring.outer(arc[j], outer);
            let o1 = ring.outer(arc[j + 1], outer);
            let q = [
                self.ring0(ring, arc[j]),
                self.at(o0.0, o0.1, 0.0),
                self.at(o1.0, o1.1, 0.0),
                self.ring0(ring, arc[j + 1]),
            ];
            self.quad(q[0], q[1], q[2], q[3]);
        }
    }
}

/// C# `OuterLoop`: the plate's outer boundary, walked the way the two-hole fixture walks its own -
/// the top edge right to left, the first hole's cap arc, the bottom edge left to right, the last
/// hole's cap arc.
fn outer_loop(rings: &[Ring], outer: f64) -> Vec<(f64, f64)> {
    let mut points = Vec::new();
    for ring in rings.iter().rev() {
        points.push(ring.outer(ring.top_vertex, outer));
    }

    // The cap arcs carry the boundary round the end holes; their first point is the top corner
    // already added and their last is the bottom corner the bottom edge starts from, so both ends
    // are dropped.
    let first = rings[0].left_arc();
    for &j in &first[1..] {
        points.push(rings[0].outer(j, outer));
    }

    for ring in &rings[1..] {
        points.push(ring.outer(ring.bottom_vertex, outer));
    }

    let last_ring = &rings[rings.len() - 1];
    let mut last = last_ring.right_arc();
    last.reverse();
    for &j in last.iter().take(last.len().saturating_sub(1)).skip(1) {
        points.push(last_ring.outer(j, outer));
    }

    points
}

/// C# `Ring`: one hole's ring - its centre on the row, its radius, how many segments it carries
/// and how far it is turned off the plain start at angle zero.
struct Ring {
    centre_x: f64,
    radius: f64,
    segments: i32,
    turn: f64,
    /// The ring vertex nearest the top of the plate, where both arcs start.
    top_vertex: i32,
    /// The ring vertex nearest the bottom of the plate, where both arcs end.
    bottom_vertex: i32,
}

impl Ring {
    fn new(centre_x: f64, radius: f64, segments: i32, turn: f64) -> Ring {
        // The ring is cut at whichever vertices sit nearest the two quadrants, so a turned or
        // re-segmented ring still splits into a left arc and a right arc with no T-vertex between
        // them.
        Ring {
            centre_x,
            radius,
            segments,
            turn,
            top_vertex: nearest(PI / 2.0, segments, turn),
            bottom_vertex: nearest(-PI / 2.0, segments, turn),
        }
    }

    fn angle(&self, j: i32) -> f64 {
        self.turn + (j as f64 * 2.0 * PI / self.segments as f64)
    }

    fn x(&self, j: i32) -> f64 {
        self.centre_x + (self.radius * self.angle(j).cos())
    }

    fn y(&self, j: i32) -> f64 {
        self.radius * self.angle(j).sin()
    }

    /// A point on the plate's outer arc at the same angle as ring vertex `j`.
    fn outer(&self, j: i32, outer: f64) -> (f64, f64) {
        (
            self.centre_x + (outer * self.angle(j).cos()),
            outer * self.angle(j).sin(),
        )
    }

    /// The squared distance from one ring's vertex to another's - the diagonal the strip chooses
    /// by.
    fn to(&self, j: i32, other: &Ring, other_j: i32) -> f64 {
        let dx = self.x(j) - other.x(other_j);
        let dy = self.y(j) - other.y(other_j);
        (dx * dx) + (dy * dy)
    }

    /// The half of the ring facing the next hole to the RIGHT, top vertex to bottom vertex.
    fn right_arc(&self) -> Vec<i32> {
        self.walk(-1)
    }

    /// The half of the ring facing the next hole to the LEFT, top vertex to bottom vertex.
    fn left_arc(&self) -> Vec<i32> {
        self.walk(1)
    }

    fn walk(&self, direction: i32) -> Vec<i32> {
        let mut arc = vec![self.top_vertex];
        let mut j = self.top_vertex;
        while j != self.bottom_vertex {
            j = ((j + direction) % self.segments + self.segments) % self.segments;
            arc.push(j);
        }

        arc
    }
}

fn nearest(angle: f64, segments: i32, turn: f64) -> i32 {
    let step = 2.0 * PI / segments as f64;
    let j = ((angle - turn) / step).round_ties_even() as i32;
    ((j % segments) + segments) % segments
}
