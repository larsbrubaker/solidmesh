//! Where a one-chain band tagged from the rolling ball's sides (`rolling_ball_band_tags`) stops:
//! never behind the other side's contact. Run by `band_gap_build_sides::tag_the_sides` right after
//! the ball's tags, it only takes triangles out of a side (tag 1 or 2 to 0); everything after it
//! (the cut, the roots, the band's delete, the room's filing of foreign edges, the fill) reads the
//! tags it leaves.
//!
//! **Why.** A side's band is cut on every triangle of that side its field reaches, the field being
//! the distance to the chain less the side's setback per chain point. Where a side wraps round
//! behind the other side's contact (a step's tread, one region with the top, curling round under
//! the riser the ball rests on; the T Handle's sphere side running on past the wall's sliver
//! alongside the chain), points of that side lie within its field that the ball's contact on it
//! never sweeps: the round would cut them though the ball never touches them, and the room reads
//! the twin crease beyond them as a reach.
//!
//! **The rule.** At the station nearest a point q, take the plane through the ball's centre that
//! holds the chain's direction and the OTHER side's contact ([`ContactPlanes::in_front`]: positive
//! toward the side's own contact). A point of side X stays in X's band only where X's field reaches
//! it and it is not behind that plane by more than the plane's own error there
//! ([`ContactPlanes::reading`]: how far the other side's contacts at the four stations round the
//! chain segment nearest the point - its two and one past each - stand off the same plane, at least
//! the snap). On a flat side, or any side that
//! does not wrap round, no point the field reaches is ever behind the plane (the plane meets the
//! other face square at its contact, so it stands at least the other side's setback times one less
//! the cosine of the angle from the side's own face), so nothing is retagged and those passes build
//! as before. Between stations the centre and contacts are read linearly along the nearest chain
//! segment and the plane holds that segment's direction; at a chain vertex it holds the vertex's
//! tangent; past an open end the end station stands. A free end's contacts are the walk's own, not
//! the taper's: the test does not read the side's setback, so the taper only narrows the band
//! further. Where the plane is not defined (the two contacts within the snap of each other, or
//! opposite through the centre) nothing is excluded.
//!
//! **Per triangle.** A triangle with a chain edge is never retagged; nor is an end region's (tags
//! 3 and 4). A side triangle is read on a grid of points no farther apart than a quarter of the
//! size (`samples`), and only samples its side's field reaches (below minus the snap) count, each
//! behind or in front only by more than the plane's error there. A sample on an edge or corner where
//! the two sides meet off the chain counts as neither: it lies on the other side's own surface, the
//! rail the plane stands for (the T Handle's twin crease, which the wall's contact rests on, reads
//! up to a hundredth off the plane at a size of 12). Some behind and none in front
//! takes it out of the side; some on each side is a band that folds back past the other side's
//! contact, which the ball's strict mode refuses ([`Sweep::FoldsBack`]) rather than cut. From the
//! triangles taken out, every triangle of the side reached across its edges with no sample in
//! front of the plane is taken out too, the field's reach aside, so the side's untagged stretch
//! ends where the plane leaves it (on the step, the tread up to the riser|tread crease) and not
//! where the field happens to stop at this size: an edge there would read as a foreign edge as far
//! out as the band reaches now, a room no larger than the size asked.
//!
//! **The new boundary.** A crease the side wrapped round past (the riser|tread rim, the T Handle's
//! twin crease) becomes an edge between the other side and no side, filed on the other side by
//! `foreign_edge_room_per_side` and read against that side's own field: the ball resting on the
//! crease, the plan's next step (a crease a side's contacts rest on is that side's rail). Where the
//! plane cuts a side's smooth surface where the side's field reaches, the pass is refused
//! ([`Sweep::FoldsBack`]: any in-field crossing has in-field samples on both sides of the plane);
//! only where the plane cuts the side outside the samples the field reaches does an edge between the
//! side and no side stand, read against the side's field (`foreign_edge_room`): the band stops
//! short of it or the size is refused.
//!
//! **Cost.** Each side triangle the field may reach and each triangle the flood reads charges one
//! unit of the pass's work (`LevelLineWork`), as each field reading does, so a large side cannot run
//! the sweep past the pass's budget or its cancel. Each plane reading searches every chain segment
//! for the nearest, as the chain field does; the four stations its error reads are indexed
//! directly.

use std::collections::HashSet;

use vector_math::Vector3;

use super::rolling_ball_setbacks::SideSetbacks;
use super::triangle_edges;
use super::{ChainField, LevelLineWork, WorkError};

/// One chain's stations as the ball's planes read them: the chain's points, and at each the
/// ball's centre and its two contacts, in place.
#[derive(Clone, Debug)]
pub struct ContactPlanes {
    chain_points: Vec<Vector3>,
    centres: Vec<Vector3>,
    contacts: [Vec<Vector3>; 2],
    closed: bool,
    size: f64,
    snap: f64,
}

/// What the sweep did to a pass's tags.
#[derive(Clone, Debug, PartialEq)]
pub enum Sweep {
    /// How many triangles of side A (tag 1) and side B (tag 2) were taken out of their side.
    Retagged([usize; 2]),
    /// Side `side` (0 for A, 1 for B) has a triangle whose samples within its field lie on both
    /// sides of the other side's contact plane, the first such sample behind it at `point`.
    FoldsBack { point: Vector3, side: usize },
}

impl ContactPlanes {
    /// The planes of one chain whose vertices are `chain_points`, from the ball's `setbacks` at
    /// `size` (per unit of size, so scaled back here); `None` where the stations do not line up
    /// with the chain's points.
    pub fn of(
        setbacks: &SideSetbacks,
        chain_points: &[Vector3],
        closed: bool,
        size: f64,
        snap: f64,
    ) -> Option<ContactPlanes> {
        let count = chain_points.len();
        if count < 2
            || setbacks.centres.len() != count
            || setbacks.contacts.iter().any(|side| side.len() != count)
        {
            return None;
        }
        let place = |units: &[Vector3]| -> Vec<Vector3> {
            chain_points
                .iter()
                .zip(units)
                .map(|(&point, &unit)| point + (unit * size))
                .collect()
        };
        Some(ContactPlanes {
            chain_points: chain_points.to_vec(),
            centres: place(&setbacks.centres),
            contacts: [place(&setbacks.contacts[0]), place(&setbacks.contacts[1])],
            closed,
            size,
            snap,
        })
    }

    /// How far `q` stands in front of the plane through the ball's centre that holds the chain's
    /// direction and the contact of the side other than `own` (0 for A, 1 for B), toward `own`'s
    /// contact, at the station nearest `q` (see the file header); infinite where that plane is not
    /// defined.
    pub fn in_front(&self, own: usize, q: Vector3) -> f64 {
        self.reading(own, q).0
    }

    /// [`in_front`](Self::in_front) with the plane's own error at `q`: how far the other side's
    /// contacts at the stations round `q` (the two of its chain segment and one past each) stand
    /// off the same plane, or the snap where that is less. Those contacts are the other side's
    /// rail, which the rule puts on the plane, so a point no farther from it than they are cannot
    /// be told from the rail: the plane holds the chain's direction at one point while the rail
    /// runs along its own surface, skew to the chain and bending between stations (the T Handle's
    /// twin crease, where the wall's contact rests, a few thousandths off at a size of 12).
    pub fn reading(&self, own: usize, q: Vector3) -> (f64, f64) {
        let Some((centre, normal, segment)) = self.plane_at(own, q) else {
            return (f64::INFINITY, self.snap);
        };
        let rail = &self.contacts[1 - own];
        let count = rail.len();
        let mut tolerance = self.snap;
        // The segment's two stations and one past each: wrapped round a closed chain, and on an
        // open one only those that exist.
        for step in 0..4 {
            let j = if self.closed {
                Some((segment + count + step - 1) % count)
            } else {
                (segment + step).checked_sub(1).filter(|&j| j < count)
            };
            if let Some(j) = j {
                tolerance = tolerance.max((rail[j] - centre).dot(normal).abs());
            }
        }
        ((q - centre).dot(normal), tolerance)
    }

    /// The plane the rule reads `q` against for side `own`: the ball's centre, the plane's unit
    /// normal turned toward `own`'s contact, and the chain segment the station lies on; `None` where the plane is not defined.
    fn plane_at(&self, own: usize, q: Vector3) -> Option<(Vector3, Vector3, usize)> {
        let (centre, contacts, direction, segment) = self.station_nearest(q);
        let (mine, other) = (contacts[own], contacts[1 - own]);
        if (mine - other).length() <= self.snap {
            return None;
        }
        let normal = direction.cross(other - centre);
        let across = normal.length();
        // Opposite contacts (or one at the centre) hold the chain's direction in no one plane.
        if across <= self.snap * direction.length() {
            return None;
        }
        let normal = normal * (1.0 / across);
        let toward = (mine - centre).dot(normal);
        if toward.abs() <= self.snap {
            return None;
        }
        let normal = if toward < 0.0 { -normal } else { normal };
        Some((centre, normal, segment))
    }

    /// The ball's centre, its two contacts and the chain's direction at the point of the chain
    /// nearest `q`, and the segment it lies on, ties to the lower segment.
    fn station_nearest(&self, q: Vector3) -> (Vector3, [Vector3; 2], Vector3, usize) {
        let count = self.chain_points.len();
        let segments = if self.closed { count } else { count - 1 };
        let (mut best, mut best_distance, mut best_t) = (0, f64::INFINITY, 0.0);
        for i in 0..segments {
            let (a, b) = (self.chain_points[i], self.chain_points[(i + 1) % count]);
            let along = b - a;
            let length_squared = along.length_squared();
            let t = if length_squared > 0.0 {
                ((q - a).dot(along) / length_squared).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let distance = (q - (a + (along * t))).length_squared();
            if distance < best_distance {
                (best, best_distance, best_t) = (i, distance, t);
            }
        }
        let next = (best + 1) % count;
        let lerp = |from: Vector3, to: Vector3| from + ((to - from) * best_t);
        let centre = lerp(self.centres[best], self.centres[next]);
        let contacts = [
            lerp(self.contacts[0][best], self.contacts[0][next]),
            lerp(self.contacts[1][best], self.contacts[1][next]),
        ];
        let direction = if best_t <= 0.0 {
            self.tangent_at(best)
        } else if best_t >= 1.0 {
            self.tangent_at(next)
        } else {
            (self.chain_points[next] - self.chain_points[best]).get_normal()
        };
        (centre, contacts, direction, best)
    }

    /// The chain's unit direction at vertex `i`: the mean of its two segments' where it has two,
    /// its one segment's at an open end.
    fn tangent_at(&self, i: usize) -> Vector3 {
        let count = self.chain_points.len();
        let has_before = self.closed || i > 0;
        let has_after = self.closed || i + 1 < count;
        let before = (i + count - 1) % count;
        let after = (i + 1) % count;
        let incoming = (self.chain_points[i] - self.chain_points[before]).get_normal();
        let outgoing = (self.chain_points[after] - self.chain_points[i]).get_normal();
        match (has_before, has_after) {
            (true, true) => {
                let mean = incoming + outgoing;
                if mean.length() > 0.0 {
                    mean.get_normal()
                } else {
                    outgoing
                }
            }
            (true, false) => incoming,
            _ => outgoing,
        }
    }
}

/// How many steps a triangle's sample grid takes along each edge at most: a triangle far longer
/// than the size is read on this many, enough for a side stretch the band reaches past its
/// neighbour's crease to be seen, at a bounded cost per triangle.
const MOST_SAMPLE_STEPS: usize = 16;

/// The points a triangle is read at: a barycentric grid whose steps are no longer than a quarter
/// of `size` (at least two steps, so its edge midpoints, and at most [`MOST_SAMPLE_STEPS`]), and
/// its centroid. A triangle much longer than the band is wide (the step's tread, one triangle
/// across) would otherwise be read only where the field does not reach it.
fn samples(positions: &[Vector3], (a, b, c): (i32, i32, i32), size: f64) -> Vec<Vector3> {
    let (pa, pb, pc) = (
        positions[a as usize],
        positions[b as usize],
        positions[c as usize],
    );
    let (ab, ac) = (pb - pa, pc - pa);
    let longest = ab.length().max(ac.length()).max((pc - pb).length());
    let wanted = (longest / (size * 0.25)).ceil();
    // `as` saturates; the clamp holds the count to the grid's bounds either way.
    let steps = if wanted.is_finite() {
        wanted as usize
    } else {
        MOST_SAMPLE_STEPS
    };
    let steps = steps.clamp(2, MOST_SAMPLE_STEPS);
    let mut points = Vec::with_capacity(((steps + 1) * (steps + 2)) / 2 + 1);
    let scale = 1.0 / steps as f64;
    for i in 0..=steps {
        for j in 0..=(steps - i) {
            points.push(pa + (ab * (i as f64 * scale)) + (ac * (j as f64 * scale)));
        }
    }
    points.push((pa + pb + pc) * (1.0 / 3.0));
    points
}

/// Takes out of sides A and B (`tags` 1 and 2, the ball's) every triangle their contacts never
/// sweep, as the file header says: `side_fields` are side A's and side B's fields at the size,
/// `chain_edges` the chain's. Returns how many of each side it took out, or the first side whose
/// band folds back past the other side's contact (the tags then as they were).
pub fn retag_what_the_contacts_never_sweep(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain_edges: &[(i32, i32)],
    tags: &mut [i32],
    side_fields: [&ChainField; 2],
    planes: &ContactPlanes,
    snap: f64,
) -> Result<Sweep, WorkError> {
    let chain_keys: HashSet<(i32, i32)> = chain_edges
        .iter()
        .map(|&(a, b)| triangle_edges::key(a, b))
        .collect();
    let on_the_chain = |t: usize| {
        triangle_edges::corners(indices[t])
            .iter()
            .any(|&(a, b)| chain_keys.contains(&triangle_edges::key(a, b)))
    };
    let standing = triangle_edges::standing_map(indices);
    // Where the two sides meet off the chain: a crease the other side's contacts rest on.
    let seam = |a: i32, b: i32| {
        let key = triangle_edges::key(a, b);
        !chain_keys.contains(&key)
            && standing.get(&key).is_some_and(|on| {
                on.len() == 2 && {
                    let pair = (tags[on[0] as usize], tags[on[1] as usize]);
                    pair == (1, 2) || pair == (2, 1)
                }
            })
    };
    // A triangle's samples less those on its edges along the seam: a point where the two sides
    // meet lies on the other side's own surface, the rail the plane stands for, so it is read as
    // neither behind nor in front (the plane is that rail only to its error there; the T Handle's
    // twin crease reads up to a hundredth off it at a size of 12).
    let seam_vertices: HashSet<i32> = standing
        .keys()
        .filter(|&&(a, b)| seam(a, b))
        .flat_map(|&(a, b)| [a, b])
        .collect();
    let off_the_seam = |t: usize| -> Vec<Vector3> {
        let (a, b, c) = indices[t];
        let along: Vec<(Vector3, Vector3)> = triangle_edges::corners(indices[t])
            .iter()
            .filter(|&&(a, b)| seam(a, b))
            .map(|&(a, b)| (positions[a as usize], positions[b as usize]))
            .collect();
        // A corner on the seam is on it however the triangle meets it, by an edge or a vertex.
        let at: Vec<Vector3> = [a, b, c]
            .into_iter()
            .filter(|v| seam_vertices.contains(v))
            .map(|v| positions[v as usize])
            .collect();
        samples(positions, indices[t], planes.size)
            .into_iter()
            .filter(|p| {
                along
                    .iter()
                    .all(|&(a, b)| p.distance_to_segment(a, b) > snap)
                    && at.iter().all(|&v| (*p - v).length() > snap)
            })
            .collect()
    };
    let mut taken = [Vec::new(), Vec::new()];
    for side in 0..2 {
        let tag = side as i32 + 1;
        let field = side_fields[side];
        let mut out = vec![false; indices.len()];
        let mut stack = Vec::new();
        for t in 0..indices.len() {
            if tags[t] != tag || on_the_chain(t) {
                continue;
            }
            let (a, b, c) = indices[t];
            let corners = [a, b, c].map(|v| positions[v as usize]);
            let (mut min, mut max) = (corners[0], corners[0]);
            for p in &corners[1..] {
                min = Vector3::new(min.x.min(p.x), min.y.min(p.y), min.z.min(p.z));
                max = Vector3::new(max.x.max(p.x), max.y.max(p.y), max.z.max(p.z));
            }
            if field.nothing_reaches_below(min, max, -snap) {
                continue;
            }
            LevelLineWork::charge(1)?;
            let (mut behind, mut front) = (None, false);
            for p in off_the_seam(t) {
                if field.evaluate(p)?.value >= -snap {
                    continue;
                }
                let (height, tolerance) = planes.reading(side, p);
                if height < -tolerance {
                    behind = behind.or(Some(p));
                } else if height > tolerance {
                    front = true;
                }
            }
            match (behind, front) {
                (Some(point), true) => return Ok(Sweep::FoldsBack { point, side }),
                (Some(_), false) => {
                    out[t] = true;
                    stack.push(t);
                }
                _ => {}
            }
        }
        // On from the triangles taken out, over the side's triangles with nothing in front.
        while let Some(t) = stack.pop() {
            for (a, b) in triangle_edges::corners(indices[t]) {
                let beside = standing
                    .get(&triangle_edges::key(a, b))
                    .map_or(&[][..], Vec::as_slice);
                for &next in beside {
                    let u = next as usize;
                    if out[u] || tags[u] != tag || on_the_chain(u) {
                        continue;
                    }
                    LevelLineWork::charge(1)?;
                    let points = off_the_seam(u);
                    let in_front = |&p: &Vector3| {
                        let (height, tolerance) = planes.reading(side, p);
                        height > tolerance
                    };
                    if !points.iter().any(in_front) {
                        out[u] = true;
                        stack.push(u);
                    }
                }
            }
        }
        taken[side] = (0..indices.len()).filter(|&t| out[t]).collect();
    }
    for side in &taken {
        for &t in side {
            tags[t] = 0;
        }
    }
    Ok(Sweep::Retagged([taken[0].len(), taken[1].len()]))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A chain along y from (0, 0, 0) to (0, 2, 0) and the ball of radius 1 between two flat
    /// faces meeting along it at `degrees`: side A the face along -x, side B the face turned
    /// `degrees` from it about the chain, toward -z. Returns the planes and each face's direction
    /// away from the chain.
    fn wedge(degrees: f64) -> (ContactPlanes, [Vector3; 2]) {
        let half = (degrees * 0.5).to_radians();
        let turned = degrees.to_radians();
        let faces = [
            Vector3::new(-1.0, 0.0, 0.0),
            Vector3::new(-turned.cos(), 0.0, -turned.sin()),
        ];
        let bisector = Vector3::new(-half.cos(), 0.0, -half.sin());
        let centre = bisector * (1.0 / half.sin());
        let setback = 1.0 / half.tan();
        let points = vec![
            Vector3::ZERO,
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(0.0, 2.0, 0.0),
        ];
        let ball = SideSetbacks {
            left: vec![setback; 3],
            right: vec![setback; 3],
            caps: [None, None],
            centres: vec![centre; 3],
            contacts: [vec![faces[0] * setback; 3], vec![faces[1] * setback; 3]],
        };
        let planes = ContactPlanes::of(&ball, &points, false, 1.0, 1e-9).expect("lined up");
        (planes, faces)
    }

    /// On flat wedges of 90 and 60 degrees every point of each side within its field (closer to
    /// the chain than its setback), along the chain and past both its ends, stands in front of
    /// the plane through the other side's contact.
    #[test]
    fn every_point_a_flat_sides_field_reaches_is_in_front_of_the_other_contacts_plane() {
        for degrees in [90.0, 60.0] {
            let (planes, faces) = wedge(degrees);
            let setback = 1.0 / (f64::to_radians(degrees) * 0.5).tan();
            for (side, face) in faces.iter().enumerate() {
                for step in 0..10 {
                    let out = setback * f64::from(step) / 10.0;
                    for y in [-0.5, 0.0, 0.7, 1.0, 2.0, 2.5] {
                        let q = (*face * out) + Vector3::new(0.0, y, 0.0);
                        let (height, tolerance) = planes.reading(side, q);
                        assert!(
                            height > 0.0,
                            "{degrees} degrees, side {side}: {q:?} {height}"
                        );
                        // The other side's contacts run along the chain: the plane has no error.
                        assert_eq!(tolerance, 1e-9);
                    }
                }
            }
        }
    }

    /// A step 0.5 high under a ball of radius 2 resting on its top and on the riser's lower rim:
    /// the top point stands in front of the plane through the rim contact, a point of the tread
    /// past the rim behind it.
    #[test]
    fn a_point_that_wraps_round_behind_the_other_contact_is_behind() {
        let a = 1.75_f64.sqrt();
        let points = vec![Vector3::ZERO, Vector3::new(0.0, 1.0, 0.0)];
        let ball = SideSetbacks {
            left: vec![a / 2.0; 2],
            right: vec![0.25; 2],
            caps: [None, None],
            centres: vec![Vector3::new(-a, 0.0, -2.0) * 0.5; 2],
            contacts: [
                vec![Vector3::new(-a, 0.0, 0.0) * 0.5; 2],
                vec![Vector3::new(0.0, 0.0, -0.5) * 0.5; 2],
            ],
        };
        let planes = ContactPlanes::of(&ball, &points, false, 2.0, 1e-9).expect("lined up");
        let top = planes.in_front(0, Vector3::new(-0.5, 0.5, 0.0));
        assert!(top > 0.0, "{top}");
        let tread = planes.in_front(0, Vector3::new(1.0, 0.5, -0.5));
        // By hand: the plane's normal is (-1.5, a) / 2 in (x, z), so the tread reads -0.75 x.
        assert!((tread + 0.75).abs() < 1e-12, "{tread}");
        // The riser, side B, stands in front of the plane through the top's contact.
        let riser = planes.in_front(1, Vector3::new(0.0, 0.5, -0.25));
        assert!((riser - a).abs() < 1e-12, "{riser}");
    }

    /// Side B's contacts resting on a crease that bends gently away from the chain (z = -1 +
    /// 0.01 y^2 below a chain along y, the ball's centre 1 toward -x of each), as the T Handle's
    /// twin crease bends off its chain at a size of 12. Read halfway between stations 1 and 2, the
    /// plane holds the chain's direction and the contact read linearly between them, and the
    /// crease's own point stands 0.0025 below it, by hand: a point of side A on the crease B's
    /// contact rests on is B's rail, so it is within the plane's error there (0.065, the contact at
    /// station 3 against that plane) and neither behind nor in front of it, while a point of A half
    /// a unit past the crease is behind.
    #[test]
    fn a_point_on_the_crease_the_other_contact_rests_on_is_within_the_planes_error() {
        let points: Vec<Vector3> = (0..5)
            .map(|j| Vector3::new(0.0, f64::from(j), 0.0))
            .collect();
        let crease = |y: f64| -1.0 + (0.01 * y * y);
        let ball = SideSetbacks {
            left: vec![1.0; 5],
            right: vec![1.0; 5],
            caps: [None, None],
            centres: (0..5)
                .map(|j| Vector3::new(-1.0, 0.0, crease(f64::from(j))))
                .collect(),
            contacts: [
                (0..5)
                    .map(|j| Vector3::new(-1.0, 0.0, crease(f64::from(j)) + 1.0))
                    .collect(),
                (0..5)
                    .map(|j| Vector3::new(0.0, 0.0, crease(f64::from(j))))
                    .collect(),
            ],
        };
        let planes = ContactPlanes::of(&ball, &points, false, 1.0, 1e-9).expect("lined up");
        let (on, tolerance) = planes.reading(0, Vector3::new(0.0, 1.5, crease(1.5)));
        assert!((on + 0.0025).abs() < 1e-12, "{on}");
        assert!((tolerance - 0.065).abs() < 1e-12, "{tolerance}");
        assert!(on.abs() < tolerance);
        let (past, _) = planes.reading(0, Vector3::new(0.5, 1.5, crease(1.5) - 0.5));
        assert!(past < -tolerance, "{past}");
    }

    /// Contacts within the snap of each other (a collapsing tip) define no plane: nothing is
    /// behind it.
    #[test]
    fn contacts_that_meet_exclude_nothing() {
        let points = vec![Vector3::ZERO, Vector3::new(0.0, 1.0, 0.0)];
        let ball = SideSetbacks {
            left: vec![0.0; 2],
            right: vec![0.0; 2],
            caps: [None, None],
            centres: vec![Vector3::new(-1.0, 0.0, -1.0); 2],
            contacts: [vec![Vector3::ZERO; 2], vec![Vector3::ZERO; 2]],
        };
        let planes = ContactPlanes::of(&ball, &points, false, 1.0, 1e-9).expect("lined up");
        assert_eq!(
            planes.in_front(0, Vector3::new(5.0, 0.5, 5.0)),
            f64::INFINITY
        );
    }
}
