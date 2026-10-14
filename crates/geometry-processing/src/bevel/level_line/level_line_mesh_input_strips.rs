//! The input-strip half of the working mesh: a strip of needles the tessellator handed in, taken
//! out ONCE by an ear cut before the build reads the mesh: the ear cut
//! ([`LevelLineMesh::ear_cut`]), its charge ([`LevelLineMesh::charge_strip_work`]) and small
//! readings - the first key of a map ([`first_key`]), whether an edge stands outside
//! the region ([`LevelLineMesh::edge_outside_region`]) and a triangle's least height
//! ([`LevelLineMesh::least_height`]) - and the strip removal itself
//! ([`LevelLineMesh::remove_input_needle_strips`]).

use std::collections::{HashSet, VecDeque};

use indexmap::{IndexMap, IndexSet};
use vector_math::{cs_math, Vector3};

use super::{LevelLineMesh, LevelLineTriangle, LevelLineWork, WorkError};

/// An ear cut's tiles: index triples of rim vertices.
pub type Tiling = Vec<(i32, i32, i32)>;

/// The first key of a map in its insertion order, or -1 for an empty one.
pub fn first_key(map: &IndexMap<i32, i32>) -> i32 {
    map.keys().next().copied().unwrap_or(-1)
}

impl LevelLineMesh {
    /// Charges the strip removal's work to the budget in force, and counts it beside the charge
    /// so the tally still says where the work went when the budget runs out inside the removal.
    pub fn charge_strip_work(&mut self, units: i64) -> Result<(), WorkError> {
        self.input_strip_work_spent += units;
        LevelLineWork::charge(units)
    }

    /// True when a triangle outside `region` stands on edge `a b`.
    pub fn edge_outside_region(&self, a: i32, b: i32, region: &HashSet<i32>) -> bool {
        self.triangles_on(a, b)
            .iter()
            .any(|triangle| !region.contains(triangle))
    }

    /// The least height of triangle `a b c`: twice its area over its longest side; 0 when it has
    /// no extent.
    pub fn least_height(&self, a: i32, b: i32, c: i32) -> f64 {
        let pa = self.vertices[a as usize];
        let pb = self.vertices[b as usize];
        let pc = self.vertices[c as usize];
        let longest = cs_math::max_f64(
            (pb - pa).length(),
            cs_math::max_f64((pc - pb).length(), (pa - pc).length()),
        );
        if longest <= 0.0 {
            0.0
        } else {
            (pb - pa).cross(pc - pa).length() / longest
        }
    }
}

impl LevelLineMesh {
    /// Tiles a rim loop - wound counter-clockwise seen down the normal - with triangles of its own
    /// vertices, taking at each turn the ear that stands HIGHEST, and refusing any ear no higher
    /// than the floor or whose new edge the mesh already carries outside the region. `Ok(None)`
    /// where no such tiling is found.
    pub fn ear_cut(
        &mut self,
        loop_vertices: &[i32],
        origin: Vector3,
        normal: Vector3,
        region: &HashSet<i32>,
    ) -> Result<Option<Tiling>, WorkError> {
        self.longest_rim_cut = self.longest_rim_cut.max(loop_vertices.len() as i32);

        let u = normal
            .cross(if normal.x.abs() < 0.9 {
                Vector3::UNIT_X
            } else {
                Vector3::UNIT_Y
            })
            .get_normal();
        let v = normal.cross(u);
        let flat = |mesh: &LevelLineMesh, vertex: i32| {
            let d = mesh.vertices[vertex as usize] - origin;
            (d.dot(u), d.dot(v))
        };
        let turn = |a: (f64, f64), b: (f64, f64), c: (f64, f64)| {
            ((b.0 - a.0) * (c.1 - a.1)) - ((b.1 - a.1) * (c.0 - a.0))
        };

        // (u, v, normal) is right-handed, so a loop wound counter-clockwise about the normal turns
        // left here. Being within the floor of the plane is not being FLAT: a ring triangle folded
        // back on itself inside the floor flattens to a clockwise triangle, the flattened rim can
        // then cross itself, and ear clipping a self-crossing rim lays overlapping tiles. So every
        // region triangle that has height must turn left once flattened, or the region is not cut.
        for &triangle in region {
            let t = self.triangles[triangle as usize];
            if self.least_height(t.a, t.b, t.c) > self.needle_floor
                && turn(flat(self, t.a), flat(self, t.b), flat(self, t.c)) <= 0.0
            {
                return Ok(None);
            }
        }

        let mut remaining = loop_vertices.to_vec();
        let mut tiles = Vec::new();
        while remaining.len() > 3 {
            // Charged for what a turn DOES: it weighs every rim vertex as an ear, and each ear that
            // survives the cheap tests scans the whole rim again (charged below). One unit a turn
            // would have let a rim of n vertices spend ~n^3 work on n units.
            self.charge_strip_work(remaining.len() as i64)?;
            let count = remaining.len();
            let mut best: Option<usize> = None;
            let mut best_height = self.needle_floor;
            for i in 0..count {
                let a = remaining[(i + count - 1) % count];
                let b = remaining[i];
                let c = remaining[(i + 1) % count];
                let pa = flat(self, a);
                let pb = flat(self, b);
                let pc = flat(self, c);
                if turn(pa, pb, pc) <= 0.0 || self.edge_outside_region(a, c, region) {
                    continue;
                }

                let height = self.least_height(a, b, c);
                if height <= best_height {
                    continue;
                }

                // An ear holds no other rim vertex - one ON its edges included, which would be a
                // T-vertex.
                self.charge_strip_work(count as i64)?;
                let empty = !remaining.iter().any(|&other| {
                    if other == a || other == b || other == c {
                        return false;
                    }

                    let p = flat(self, other);
                    turn(pa, pb, p) >= 0.0 && turn(pb, pc, p) >= 0.0 && turn(pc, pa, p) >= 0.0
                });

                if empty {
                    best = Some(i);
                    best_height = height;
                }
            }

            let Some(best) = best else {
                return Ok(None);
            };

            tiles.push((
                remaining[(best + count - 1) % count],
                remaining[best],
                remaining[(best + 1) % count],
            ));
            remaining.remove(best);
        }

        if self.least_height(remaining[0], remaining[1], remaining[2]) <= self.needle_floor {
            return Ok(None);
        }

        tiles.push((remaining[0], remaining[1], remaining[2]));
        Ok(Some(tiles))
    }
}

impl LevelLineMesh {
    /// Takes each strip of input needles out ONCE, before the build reads the mesh: the strip and
    /// the ring across its edges re-tiled by an ear cut (widened once to its apexes' stars where the
    /// ring alone has no needle-free tiling), counting the strips removed and the strips left for the
    /// per-needle repair. Only the triangles that came in are seeds.
    pub fn remove_input_needle_strips(&mut self) -> Result<(), WorkError> {
        if self.needle_floor <= 0.0 {
            return Ok(());
        }

        let mut visited = HashSet::new();
        let input_count = self.triangles.len() as i32;
        for seed in 0..input_count {
            if self.removed[seed as usize]
                || visited.contains(&seed)
                || !self.has_no_height(self.triangles[seed as usize], self.needle_floor)
            {
                continue;
            }

            let strip = self.collect_strip(seed, &mut visited)?;
            if self.retile_strip(&strip)? {
                self.input_strips_removed += 1;
            } else {
                self.input_strips_left += 1;
            }
        }

        Ok(())
    }

    /// The needles reached from `seed` across shared edges, breadth first.
    pub fn collect_strip(
        &mut self,
        seed: i32,
        visited: &mut HashSet<i32>,
    ) -> Result<Vec<i32>, WorkError> {
        let mut strip = Vec::new();
        let mut pending = VecDeque::from([seed]);
        visited.insert(seed);
        while let Some(triangle) = pending.pop_front() {
            self.charge_strip_work(1)?;
            strip.push(triangle);
            let t = self.triangles[triangle as usize];
            for k in 0..3 {
                for &next in self.triangles_on(t.corner(k), t.corner((k + 1) % 3)) {
                    if !visited.contains(&next)
                        && self.has_no_height(self.triangles[next as usize], self.needle_floor)
                    {
                        visited.insert(next);
                        pending.push_back(next);
                    }
                }
            }
        }

        Ok(strip)
    }

    /// Re-tiles the strip with the ring across its edges, or - where that ring has no needle-free
    /// tiling because the strip's APEXES stand on its rim - with the whole star of every apex.
    pub fn retile_strip(&mut self, strip: &[i32]) -> Result<bool, WorkError> {
        // The region: the strip, and every triangle across one of its edges (first-added order).
        let mut region: IndexSet<i32> = strip.iter().copied().collect();
        for &needle in strip {
            let t = self.triangles[needle as usize];
            for k in 0..3 {
                let standing = self.triangles_on(t.corner(k), t.corner((k + 1) % 3));
                if standing.len() != 2 {
                    return Ok(false);
                }

                let (first, second) = (standing[0], standing[1]);
                region.insert(first);
                region.insert(second);
            }
        }

        if self.retile_region(strip, &region)? {
            return Ok(true);
        }

        // Widened once to the whole star of every apex, which leaves each apex inside the region;
        // every condition the ring had to meet is asked again of the wider region, unchanged.
        let mut widened = region.clone();
        for &needle in strip {
            let apex = self.apex_of(self.triangles[needle as usize]);
            if !self.add_star_of(apex, needle, &mut widened)? {
                return Ok(false);
            }
        }

        Ok(widened.len() > region.len() && self.retile_region(strip, &widened)?)
    }

    /// Adds to `region` every triangle round `vertex` reached from `from` across edges at the
    /// vertex; false where such an edge does not stand on exactly two triangles.
    pub fn add_star_of(
        &mut self,
        vertex: i32,
        from: i32,
        region: &mut IndexSet<i32>,
    ) -> Result<bool, WorkError> {
        let mut seen = HashSet::from([from]);
        let mut pending = VecDeque::from([from]);
        while let Some(triangle) = pending.pop_front() {
            self.charge_strip_work(1)?;
            region.insert(triangle);
            let t = self.triangles[triangle as usize];
            for k in 0..3 {
                let a = t.corner(k);
                let b = t.corner((k + 1) % 3);
                if a != vertex && b != vertex {
                    continue;
                }

                let standing = self.triangles_on(a, b).to_vec();
                if standing.len() != 2 {
                    return Ok(false);
                }

                for next in standing {
                    if seen.insert(next) {
                        pending.push_back(next);
                    }
                }
            }
        }

        Ok(true)
    }

    /// Replaces the region with an ear cut of its rim - where the region is one tag, flat within the
    /// needle floor of its largest triangle's plane, deletes no pick edge, has ONE simple rim, and
    /// the cut finds a needle-free tiling. Every edge inside the region stops existing, so the
    /// clean-edge memo is cleared.
    pub fn retile_region(
        &mut self,
        strip: &[i32],
        region: &IndexSet<i32>,
    ) -> Result<bool, WorkError> {
        let tag = self.triangles[strip[0] as usize].tag;
        let mut next: IndexMap<i32, i32> = IndexMap::new();
        let mut region_vertices = HashSet::new();
        let mut biggest = -1;
        let mut biggest_area = 0.0;
        for &triangle in region {
            let t = self.triangles[triangle as usize];
            if t.tag != tag {
                return Ok(false);
            }

            let pa = self.vertices[t.a as usize];
            let area = (self.vertices[t.b as usize] - pa)
                .cross(self.vertices[t.c as usize] - pa)
                .length();
            if area > biggest_area {
                biggest_area = area;
                biggest = triangle;
            }

            for k in 0..3 {
                let a = t.corner(k);
                let b = t.corner((k + 1) % 3);
                region_vertices.insert(a);
                let standing = self.triangles_on(a, b);
                if standing.len() > 2 {
                    return Ok(false);
                }

                if standing.len() == 2
                    && region.contains(&standing[0])
                    && region.contains(&standing[1])
                {
                    // An interior edge is deleted, so it may not be the pick or a fold the tagging
                    // stopped at.
                    if self
                        .is_chain_edge
                        .as_ref()
                        .is_some_and(|is_chain_edge| is_chain_edge(a, b))
                    {
                        return Ok(false);
                    }

                    continue;
                }

                // A rim edge, walked in its triangle's winding; a vertex the rim leaves twice is a
                // pinch.
                if next.contains_key(&a) {
                    return Ok(false);
                }

                next.insert(a, b);
            }
        }

        if biggest < 0 || next.len() < 3 {
            return Ok(false);
        }

        // The plane every region vertex must lie within the floor of: the region's largest
        // triangle's.
        let big = self.triangles[biggest as usize];
        let origin = self.vertices[big.a as usize];
        let normal = (self.vertices[big.b as usize] - origin)
            .cross(self.vertices[big.c as usize] - origin)
            .get_normal();
        if region_vertices.iter().any(|&vertex| {
            (self.vertices[vertex as usize] - origin).dot(normal).abs() > self.needle_floor
        }) {
            return Ok(false);
        }

        // The rim as ONE loop: a walk that comes home before it has used every rim edge is two.
        let mut rim = Vec::new();
        let start = first_key(&next);
        let mut at = start;
        loop {
            rim.push(at);
            if rim.len() > next.len() {
                return Ok(false);
            }

            match next.get(&at) {
                Some(&following) => at = following,
                None => return Ok(false),
            }

            if at == start {
                break;
            }
        }

        if rim.len() != next.len() {
            return Ok(false);
        }

        let region_set: HashSet<i32> = region.iter().copied().collect();
        let Some(tiles) = self.ear_cut(&rim, origin, normal, &region_set)? else {
            return Ok(false);
        };

        if let Some(clean) = &mut self.answered_clean {
            clean.clear();
        }

        for &triangle in region {
            self.discard(triangle);
        }

        for (a, b, c) in tiles {
            self.add_triangle(LevelLineTriangle::new(a, b, c, tag));
        }

        Ok(true)
    }
}
