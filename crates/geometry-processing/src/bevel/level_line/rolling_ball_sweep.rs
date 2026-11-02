//! Where a one-chain band tagged from the rolling ball's sides (`rolling_ball_band_tags`) stops:
//! never past the chain into the other side's sweep. Run by `band_gap_build_sides::tag_the_sides`
//! right after the ball's tags, it only takes triangles out of a side (tag 1 or 2 to 0);
//! everything after it (the cut, the roots, the band's delete, the room's filing of foreign edges,
//! the fill) reads the tags it leaves.
//!
//! **Why.** A side's band is cut on every triangle of that side its field reaches, the field being
//! the distance to the chain less the side's setback per chain point. Where a side's region wraps
//! round past the chain (a step's tread, one region with the top, curling round under the riser the
//! ball rests on; the T Handle's sphere side running on, past the wall's rim, up the far face of
//! the wall's fin beside the chain), points of that side lie within its straight-line field that
//! the ball's contact on it never sweeps: the round would cut them though the ball never touches
//! them, and the room reads the creases beyond them as a reach.
//!
//! **The rule.** In the ball's cross-section at a station, side X's band is the sector from X's
//! contact round to the chain, and the other side's band the sector from the chain to its contact:
//! the line dividing them is the chain point. So at the station nearest a point q, take the plane
//! through the ball's centre that holds the chain's direction and the chain point
//! ([`ContactPlanes::in_front`]: positive toward the side's own contact). A point of side X stays
//! in X's band only where X's field reaches it and it is not behind that plane by more than the
//! plane's own error there ([`ContactPlanes::reading`]: how far the chain's points at the four
//! stations round the chain segment nearest the point - its two and one past each - stand off the
//! same plane, at least the snap; the chain bends between stations while the plane holds one
//! direction). A plane through the other side's contact instead would keep whatever wraps no
//! further round than that contact: the T Handle's fin face, between the chain and the wall's
//! contact in angle, read in front of it by up to 6 at a size of 12. On a flat side, or any side
//! that does not wrap round, every point the field reaches off the chain is in front of the plane
//! (a face leaving the chain runs into its own sector), so nothing is retagged and those passes
//! build as before. Between stations the centre, contacts and chain point are read linearly along
//! the nearest chain segment and the plane holds that segment's direction; at a chain vertex it
//! holds the vertex's tangent. Past an open end (the nearest chain point is the end vertex and q
//! lies beyond it along the chain) no station's cross-section holds q, so the sweep has no evidence
//! there of its own: q reads on the plane, neither behind nor in front, and no triangle reaching
//! past an end is taken out by its own samples (read against the end station's plane, the T
//! Handle's wall past its web end crosses it, and would refuse the pass). A free end's contacts
//! are the walk's own, not the taper's: the test does not read the side's setback, so the taper
//! only narrows the band further.
//! Where the plane is not defined (the two contacts within the snap of each other, or the side's
//! own contact on the plane) nothing is excluded.
//!
//! **Per triangle.** A triangle with a chain edge is never retagged; nor is an end region's (tags
//! 3 and 4). A side triangle is read on a grid of points no farther apart than a quarter of the
//! size (`samples`), and only samples its side's field reaches (below minus the snap) count, each
//! behind or in front only by more than the plane's error there. A sample where the two sides meet
//! off the chain (the T Handle's twin crease) is read like any other: the plane stands for the
//! chain, not for the other side's surface, so such a point is behind only where the side's
//! region has wrapped round past the chain to reach it. Some behind and none in front takes it out
//! of the side; some on each side is a band that folds back past the chain, which the ball's
//! strict mode refuses ([`Sweep::FoldsBack`]) rather than cut; one with a sample past an open end
//! is never taken out by its own samples. From the triangles taken out, every triangle of the side
//! reached across its edges with no sample in front of the plane is taken out too, the field's
//! reach aside, its samples past an open end read against the end station's plane
//! ([`ContactPlanes::reading_round_the_ends`]): a stretch behind the chain carries on past the end
//! while it stays behind (the step's tread, the same past its free ends), and stops where a side
//! wraps round the end onto its own sector (a plate round the end of a fin thinner than the size).
//! Read as on the plane there, the flood ran round the T Handle's ends over about 13,500
//! triangles of the sphere side and 4,500 of the wall's at a size of 12, up to 215 of them within
//! the field; so the side's untagged stretch ends where the plane leaves it (on the step,
//! the tread up to the riser|tread crease) and not where the field happens to stop at this size: an
//! edge there would read as a foreign edge as far out as the band reaches now, a room no larger
//! than the size asked.
//!
//! **The new boundary.** A crease the side wrapped round past (the riser|tread rim, the T Handle's
//! wall rim) becomes an edge between the other side and no side, filed on the other side by
//! `foreign_edge_room_per_side` and read against that side's own field: the ball resting on the
//! crease, the plan's next step (a crease a side's contacts rest on is that side's rail). Where the
//! plane cuts a side's smooth surface where the side's field reaches, the pass is refused
//! ([`Sweep::FoldsBack`]: any in-field crossing has in-field samples on both sides of the plane);
//! only where the plane cuts the side outside the samples the field reaches does an edge between
//! the side and no side stand, read against the side's field (`foreign_edge_room`): the band stops
//! short of it or the size is refused.
//!
//! **Cost.** Each side triangle the field may reach and each triangle the flood reads charges one
//! unit of the pass's work (`LevelLineWork`), as each field reading does, so a large side cannot
//! run the sweep past the pass's budget or its cancel. Each plane reading searches every chain
//! segment for the nearest, as the chain field does; the four stations its error reads are indexed
//! directly.

use std::collections::HashSet;

use vector_math::Vector3;

use super::rolling_ball_setbacks::SideSetbacks;
use super::triangle_edges;
use super::{ChainField, LevelLineWork, WorkError};

/// One chain's stations as the ball's planes read them: the chain's points, and at each the
/// ball's centre and its two contacts, in place. The planes go through each station's centre and
/// chain point (see the file header); the contacts only turn each plane toward its side and tell
/// where the plane is not defined.
#[derive(Clone, Debug)]
pub struct ContactPlanes {
    chain_points: Vec<Vector3>,
    centres: Vec<Vector3>,
    contacts: [Vec<Vector3>; 2],
    closed: bool,
    size: f64,
    snap: f64,
}

/// One station as a plane reading sees it, read linearly between the chain's points.
struct Station {
    centre: Vector3,
    contacts: [Vector3; 2],
    chain_point: Vector3,
    /// The chain's unit direction there (a vertex's tangent at a chain vertex).
    direction: Vector3,
    /// The chain segment the station lies on.
    segment: usize,
    /// True where the point read lies beyond an open chain's end vertex along the chain.
    past_an_open_end: bool,
}

/// What the sweep did to a pass's tags.
#[derive(Clone, Debug, PartialEq)]
pub enum Sweep {
    /// How many triangles of side A (tag 1) and side B (tag 2) were taken out of their side.
    Retagged([usize; 2]),
    /// Side `side` (0 for A, 1 for B) has a triangle whose samples within its field lie on both
    /// sides of the plane through the ball's centre and the chain, the first such sample behind it
    /// at `point`.
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
    /// direction and the chain point, toward the contact of side `own` (0 for A, 1 for B), at the
    /// station nearest `q` (see the file header); infinite where that plane is not defined, and
    /// zero past an open end.
    pub fn in_front(&self, own: usize, q: Vector3) -> f64 {
        self.reading(own, q).0
    }

    /// [`in_front`](Self::in_front) with the plane's own error at `q`: how far the chain's points
    /// at the stations round `q` (the two of its chain segment and one past each) stand off the
    /// same plane, or the snap where that is less. The chain is the line the rule puts on the
    /// plane, so a point no farther from it than the chain itself cannot be told from the chain:
    /// the plane holds the chain's direction at one point while the chain bends between stations.
    ///
    /// Past an open end (the chain point nearest `q` is the end vertex, `q` beyond it along the
    /// chain) no station's cross-section holds `q`, so the sweep has no evidence about it: the
    /// reading is zero, on the plane within its error, neither behind nor in front, and the cut
    /// there stays the field's and the end's own (the end clip, the end face, the cap).
    pub fn reading(&self, own: usize, q: Vector3) -> (f64, f64) {
        self.read(own, q, false)
    }

    /// [`reading`](Self::reading), but past an open end against the end station's plane: what the
    /// flood reads, to carry a stretch already taken out on past an end only where the side stays
    /// behind that plane (see the file header).
    pub fn reading_round_the_ends(&self, own: usize, q: Vector3) -> (f64, f64) {
        self.read(own, q, true)
    }

    fn read(&self, own: usize, q: Vector3, at_the_end_station: bool) -> (f64, f64) {
        let station = self.station_nearest(q);
        if station.past_an_open_end && !at_the_end_station {
            return (0.0, self.snap);
        }
        let Some(normal) = self.normal_at(own, &station) else {
            return (f64::INFINITY, self.snap);
        };
        let (centre, segment) = (station.centre, station.segment);
        let count = self.chain_points.len();
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
                let off = (self.chain_points[j] - centre).dot(normal).abs();
                tolerance = tolerance.max(off);
            }
        }
        ((q - centre).dot(normal), tolerance)
    }

    /// The unit normal of the plane the rule reads against for side `own` at `station`: through
    /// the ball's centre, holding the chain's direction and the chain point, turned toward `own`'s
    /// contact; `None` where the plane is not defined (the two contacts within the snap of each
    /// other, the chain point on the centre's line along the chain, or `own`'s contact on the
    /// plane).
    fn normal_at(&self, own: usize, station: &Station) -> Option<Vector3> {
        let (mine, other) = (station.contacts[own], station.contacts[1 - own]);
        if (mine - other).length() <= self.snap {
            return None;
        }
        let normal = station
            .direction
            .cross(station.chain_point - station.centre);
        let across = normal.length();
        if across <= self.snap * station.direction.length() {
            return None;
        }
        let normal = normal * (1.0 / across);
        let toward = (mine - station.centre).dot(normal);
        if toward.abs() <= self.snap {
            return None;
        }
        Some(if toward < 0.0 { -normal } else { normal })
    }

    /// The station at the point of the chain nearest `q`, read linearly along the segment it lies
    /// on, ties to the lower segment.
    fn station_nearest(&self, q: Vector3) -> Station {
        let count = self.chain_points.len();
        let segments = if self.closed { count } else { count - 1 };
        let (mut best, mut best_distance, mut best_t, mut best_raw) = (0, f64::INFINITY, 0.0, 0.0);
        for i in 0..segments {
            let (a, b) = (self.chain_points[i], self.chain_points[(i + 1) % count]);
            let along = b - a;
            let length_squared = along.length_squared();
            let raw = if length_squared > 0.0 {
                (q - a).dot(along) / length_squared
            } else {
                0.0
            };
            let t = raw.clamp(0.0, 1.0);
            let distance = (q - (a + (along * t))).length_squared();
            if distance < best_distance {
                (best, best_distance, best_t, best_raw) = (i, distance, t, raw);
            }
        }
        let next = (best + 1) % count;
        let lerp = |from: Vector3, to: Vector3| from + ((to - from) * best_t);
        let direction = if best_t <= 0.0 {
            self.tangent_at(best)
        } else if best_t >= 1.0 {
            self.tangent_at(next)
        } else {
            (self.chain_points[next] - self.chain_points[best]).get_normal()
        };
        Station {
            centre: lerp(self.centres[best], self.centres[next]),
            contacts: [
                lerp(self.contacts[0][best], self.contacts[0][next]),
                lerp(self.contacts[1][best], self.contacts[1][next]),
            ],
            chain_point: lerp(self.chain_points[best], self.chain_points[next]),
            direction,
            segment: best,
            past_an_open_end: !self.closed
                && ((best == 0 && best_raw < 0.0) || (best + 1 == segments && best_raw > 1.0)),
        }
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
/// band folds back past the chain (the tags then as they were).
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
    let points_of = |t: usize| samples(positions, indices[t], planes.size);
    // Past an open end no station's cross-section holds a point, so a triangle with a sample there
    // is never taken out by its own samples (the flood reads it against the end station's plane).
    let past_the_end = |p: &Vector3| planes.station_nearest(*p).past_an_open_end;
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
            let (mut behind, mut front, mut past_an_end) = (None, false, false);
            for p in points_of(t) {
                past_an_end |= past_the_end(&p);
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
                // A triangle reaching past an open end stays: the cut there is the end's own.
                (Some(_), false) if !past_an_end => {
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
                    let points = points_of(u);
                    let in_front = |&p: &Vector3| {
                        let (height, tolerance) = planes.reading_round_the_ends(side, p);
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
            features: [Vec::new(), Vec::new()],
            centres: vec![centre; 3],
            contacts: [vec![faces[0] * setback; 3], vec![faces[1] * setback; 3]],
        };
        let planes = ContactPlanes::of(&ball, &points, false, 1.0, 1e-9).expect("lined up");
        (planes, faces)
    }

    /// On flat wedges of 90 and 60 degrees every point of each side within its field (closer to
    /// the chain than its setback) along the chain stands in front of the plane through the centre
    /// and the chain, and the chain itself on it: a straight chain lies in the plane, so the
    /// plane's error is the snap. Past either open end no station holds the point and it reads on
    /// the plane, neither behind nor in front.
    #[test]
    fn every_point_a_flat_sides_field_reaches_is_in_front_of_the_plane_through_the_chain() {
        for degrees in [90.0, 60.0] {
            let (planes, faces) = wedge(degrees);
            let setback = 1.0 / (f64::to_radians(degrees) * 0.5).tan();
            for (side, face) in faces.iter().enumerate() {
                for step in 0..10 {
                    let out = setback * f64::from(step) / 10.0;
                    for y in [0.0, 0.7, 1.0, 2.0] {
                        let q = (*face * out) + Vector3::new(0.0, y, 0.0);
                        let (height, tolerance) = planes.reading(side, q);
                        assert_eq!(tolerance, 1e-9);
                        if step == 0 {
                            assert!(height.abs() <= tolerance, "{degrees}: {q:?} {height}");
                        } else {
                            assert!(height > tolerance, "{degrees}, side {side}: {q:?} {height}");
                        }
                    }
                    for y in [-0.5, 2.5] {
                        let q = (*face * out) + Vector3::new(0.0, y, 0.0);
                        assert_eq!(planes.reading(side, q), (0.0, 1e-9), "{q:?}");
                    }
                }
            }
        }
    }

    /// A step 0.5 high under a ball of radius 2 resting on its top and on the riser's lower rim:
    /// the top stands in front of the plane through the centre and the chain, the tread past the
    /// rim behind it, and the riser in front of the same plane turned toward its own contact.
    #[test]
    fn a_point_that_wraps_round_behind_the_other_contact_is_behind() {
        let a = 1.75_f64.sqrt();
        let points = vec![Vector3::ZERO, Vector3::new(0.0, 1.0, 0.0)];
        let ball = SideSetbacks {
            left: vec![a / 2.0; 2],
            right: vec![0.25; 2],
            caps: [None, None],
            features: [Vec::new(), Vec::new()],
            centres: vec![Vector3::new(-a, 0.0, -2.0) * 0.5; 2],
            contacts: [
                vec![Vector3::new(-a, 0.0, 0.0) * 0.5; 2],
                vec![Vector3::new(0.0, 0.0, -0.5) * 0.5; 2],
            ],
        };
        let planes = ContactPlanes::of(&ball, &points, false, 2.0, 1e-9).expect("lined up");
        // By hand: the centre is (-a, -2) in (x, z) and the chain (0, 0), so the plane's normal
        // toward the top's contact is (-2, a) / sqrt(5.75).
        let across = 5.75_f64.sqrt();
        let top = planes.in_front(0, Vector3::new(-0.5, 0.5, 0.0));
        assert!((top - (1.0 / across)).abs() < 1e-12, "{top}");
        let tread = planes.in_front(0, Vector3::new(1.0, 0.5, -0.5));
        assert!(
            (tread + ((2.0 + (0.5 * a)) / across)).abs() < 1e-12,
            "{tread}"
        );
        let riser = planes.in_front(1, Vector3::new(0.0, 0.5, -0.25));
        assert!((riser - ((0.25 * a) / across)).abs() < 1e-12, "{riser}");
    }

    /// A chain that bends (z = 0.01 y^2 along y) in a 90 degree wedge under a ball of radius 1:
    /// read halfway between stations 1 and 2 the plane holds that segment and its centre, and the
    /// chain's points at stations 0 to 3 stand 0.01 (j - 1)(j - 2) / sqrt(2.0009) off it by hand,
    /// so its error is 0.02 / sqrt(2.0009). A point of side A half a unit from the chain stands in
    /// front by 0.4975 / sqrt(2.0009), and a point of side B half a unit below the chain behind
    /// by 0.5025 / sqrt(2.0009): B's sector, which A's contact never sweeps.
    #[test]
    fn the_planes_error_is_how_far_the_chain_bends_off_it_round_the_station() {
        let lift = |y: f64| 0.01 * y * y;
        let at = |x: f64, j: f64, z: f64| Vector3::new(x, j, lift(j) + z);
        let points: Vec<Vector3> = (0..5).map(|j| at(0.0, f64::from(j), 0.0)).collect();
        let ball = SideSetbacks {
            left: vec![1.0; 5],
            right: vec![1.0; 5],
            caps: [None, None],
            features: [Vec::new(), Vec::new()],
            centres: (0..5).map(|_| Vector3::new(-1.0, 0.0, -1.0)).collect(),
            contacts: [
                (0..5).map(|_| Vector3::new(-1.0, 0.0, 0.0)).collect(),
                (0..5).map(|_| Vector3::new(0.0, 0.0, -1.0)).collect(),
            ],
        };
        let planes = ContactPlanes::of(&ball, &points, false, 1.0, 1e-9).expect("lined up");
        let across = 2.0009_f64.sqrt();
        let (on_a, tolerance) = planes.reading(0, Vector3::new(-0.5, 1.5, lift(1.5)));
        assert!((tolerance - (0.02 / across)).abs() < 1e-12, "{tolerance}");
        assert!((on_a - (0.4975 / across)).abs() < 1e-12, "{on_a}");
        let (on_b, _) = planes.reading(0, Vector3::new(0.0, 1.5, lift(1.5) - 0.5));
        assert!((on_b + (0.5025 / across)).abs() < 1e-12, "{on_b}");
    }

    /// A fin standing beside a 90 degree wedge's chain: a triangle of side A's region at x = 0.3,
    /// past the chain on side B's sector (behind B's face x = 0, as a thin wall's far face stands
    /// behind its near one), from 0.2 to 0.8 below the chain, so within A's field (its setback 1)
    /// straight through the wall. It stands 0.2 or more in front of the plane through the centre
    /// and B's contact (z = -1), which would keep it, and behind the plane through the centre and
    /// the chain (z = x), so the ball's contact on A never sweeps it: it is taken out of side A,
    /// and A's and B's own faces stay as they are.
    #[test]
    fn a_face_wrapped_round_past_the_chain_into_the_other_sides_sector_leaves_the_side() {
        let (planes, _) = wedge(90.0);
        let positions = vec![
            Vector3::ZERO,
            Vector3::new(0.0, 2.0, 0.0),
            Vector3::new(-2.0, 0.0, 0.0),
            Vector3::new(-2.0, 2.0, 0.0),
            Vector3::new(0.0, 0.0, -2.0),
            Vector3::new(0.0, 2.0, -2.0),
            Vector3::new(0.3, 0.5, -0.2),
            Vector3::new(0.3, 1.5, -0.2),
            Vector3::new(0.3, 1.0, -0.8),
        ];
        let indices = vec![(0, 3, 1), (0, 2, 3), (0, 1, 5), (0, 5, 4), (6, 7, 8)];
        let mut tags = vec![1, 1, 2, 2, 1];
        let chain = [positions[0], Vector3::new(0.0, 1.0, 0.0), positions[1]];
        let field = ChainField::from_polyline(&chain, false, &[1.0; 3]).expect("a field");
        for q in [positions[6], positions[7], positions[8]] {
            assert!(field.evaluate(q).expect("read").value < 0.0, "{q:?}");
        }
        let swept = retag_what_the_contacts_never_sweep(
            &positions,
            &indices,
            &[(0, 1)],
            &mut tags,
            [&field, &field],
            &planes,
            1e-9,
        )
        .expect("within the budget");
        assert_eq!(swept, Sweep::Retagged([1, 0]));
        assert_eq!(tags, vec![1, 1, 2, 2, 0]);
    }

    /// The 90 degree wedge's sides as a mesh (side A's face z = 0 toward -x, side B's x = 0 toward
    /// -z, both along the chain from y = 0 to 2) with `more` triangles of side A after them, run
    /// through the sweep with side A's and B's fields at setback 1. Returns the sweep and the tags.
    fn sweep_the_wedge_with(more: &[[Vector3; 3]]) -> (Sweep, Vec<i32>) {
        let (planes, _) = wedge(90.0);
        let mut positions = vec![
            Vector3::ZERO,
            Vector3::new(0.0, 2.0, 0.0),
            Vector3::new(-2.0, 0.0, 0.0),
            Vector3::new(-2.0, 2.0, 0.0),
            Vector3::new(0.0, 0.0, -2.0),
            Vector3::new(0.0, 2.0, -2.0),
        ];
        let mut indices = vec![(0, 3, 1), (0, 2, 3), (0, 1, 5), (0, 5, 4)];
        let mut tags = vec![1, 1, 2, 2];
        for corners in more {
            let mut index = [0; 3];
            for (k, &p) in corners.iter().enumerate() {
                // Corners shared by position, so triangles meeting along an edge share it.
                index[k] = match positions.iter().position(|&q| q == p) {
                    Some(at) => at as i32,
                    None => {
                        positions.push(p);
                        positions.len() as i32 - 1
                    }
                };
            }
            indices.push((index[0], index[1], index[2]));
            tags.push(1);
        }
        let chain = [positions[0], Vector3::new(0.0, 1.0, 0.0), positions[1]];
        let field = ChainField::from_polyline(&chain, false, &[1.0; 3]).expect("a field");
        let swept = retag_what_the_contacts_never_sweep(
            &positions,
            &indices,
            &[(0, 1)],
            &mut tags,
            [&field, &field],
            &planes,
            1e-9,
        )
        .expect("within the budget");
        (swept, tags)
    }

    /// The flood stops where a side wraps round an open end onto its own sector. A fin of side A
    /// beside the wedge's chain, past the chain on side B's sector: its first triangle within A's
    /// field (0.42 from the chain at (0.3, 1, -0.3)) and behind the plane, so taken out by its own
    /// samples; its second, sharing the first's lower edge and wholly outside the field (1.53 from
    /// the chain and farther), taken out by the flood, nothing of it in front. A third triangle
    /// shares the first's edge toward the chain's y = 0 end and wraps round past that end within
    /// the field to (-0.3, -0.4, -0.1), 0.51 from the end vertex: behind the plane where a station
    /// holds it, but no station holds its part past the end, so its own samples do not take it
    /// out, and that part stands in front of the end station's plane (z - x = 0.2 at its far
    /// corner), so the flood does not either. The floor's triangles stay as they are.
    #[test]
    fn the_sweep_takes_out_a_fin_past_the_chain_and_stops_where_it_wraps_round_an_open_end() {
        let at = |y: f64, z: f64| Vector3::new(0.3, y, z);
        let (swept, tags) = sweep_the_wedge_with(&[
            [at(1.0, -0.3), at(0.5, -1.5), at(1.5, -1.5)],
            [at(0.5, -1.5), at(1.0, -2.5), at(1.5, -1.5)],
            [at(1.0, -0.3), Vector3::new(-0.3, -0.4, -0.1), at(0.5, -1.5)],
        ]);
        assert_eq!(swept, Sweep::Retagged([2, 0]));
        assert_eq!(tags, vec![1, 1, 2, 2, 0, 0, 1]);
    }

    /// A triangle of side A within its field that runs from A's own sector (z - x > 0, in front)
    /// across the plane through the centre and the chain into B's (behind): a band folding back
    /// past the chain, which the sweep refuses, naming a sample behind the plane, and leaves the
    /// tags as they were.
    #[test]
    fn a_side_triangle_on_both_sides_of_the_plane_through_the_chain_folds_back() {
        let (planes, _) = wedge(90.0);
        let (swept, tags) = sweep_the_wedge_with(&[[
            Vector3::new(-0.5, 0.5, -0.2),
            Vector3::new(0.3, 1.0, -0.2),
            Vector3::new(-0.5, 1.5, -0.2),
        ]]);
        let Sweep::FoldsBack { point, side } = swept else {
            panic!("no fold back: {swept:?}");
        };
        assert_eq!(side, 0);
        let (height, tolerance) = planes.reading(0, point);
        assert!(height < -tolerance, "{point:?} {height}");
        assert_eq!(tags, vec![1, 1, 2, 2, 1]);
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
            features: [Vec::new(), Vec::new()],
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
