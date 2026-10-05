//! The vertex list of a mesh being rewritten, with a point-to-index lookup over it: ask for a
//! position and get back the index of the vertex already standing there, or a new one appended.
//! Ports `Bevel/Surface/SurfaceVertexPool.cs`.
//!
//! The walkback's whole no-crack guarantee is index sharing, and the same point is arrived at by
//! more than one road: two faces either side of a rim edge each cut that edge, and two tangent lines
//! of one closed chain each cut the vertical edge their mitre stands on. Those arrive at the same
//! point in double arithmetic and then land in the mesh as [`Vector3Float`], so "the same point" can
//! only mean "nearer than a float can tell them apart" - and how near that is depends on how big the
//! part is, which is why the weld distance is read off the mesh rather than written down as a
//! micron. A float's ulp at 30 mm is about two microns and at 1000 mm about sixty, so a fixed micron
//! says "two points" of what is one point on anything much larger than a pencil, and every seam in
//! the construction comes apart. The whole surface bevel takes its tolerances from here, so there is
//! one number to reason about.
//!
//! This is a weld done BEFORE the vertices exist rather than `Mesh.CleanAndMerge` afterwards, which
//! is the point: a merge pass cannot tell a crack it should close from two surfaces that genuinely
//! touch, and it renumbers everything, which would lose the tangent-line vertices the strip has to
//! sew to. [`super::surface_mesh_closure`] is the reading that checks the result.
//!
//! The buckets are only ever looked up by cell, never iterated, so a `HashMap`'s order cannot reach
//! a result; within a cell candidates keep their insertion order, as C#'s lists do.

use std::collections::HashMap;

use vector_math::{Vector3, Vector3Float};

/// A bucket key: a cell `near_enough` across.
type Cell = (i64, i64, i64);

/// C# `SurfaceVertexPool`.
#[derive(Debug, Clone)]
pub struct SurfaceVertexPool {
    vertices: Vec<Vector3Float>,
    buckets: HashMap<Cell, Vec<i32>>,
    weld_distance: f64,
}

impl SurfaceVertexPool {
    /// The weld distance for a part too small for its own size to set one - a micron, which was
    /// the fixed distance before the scale was read. Anything under a few tens of millimetres
    /// across gets this.
    pub const SMALLEST_WELD_DISTANCE: f64 = 1e-6;

    /// The weld distance as a fraction of the part's bounding box diagonal: a millionth of it,
    /// which is about a quarter of a [`Vector3Float`]'s ulp there and so is comfortably above the
    /// rounding while staying far below anything anybody models.
    const WELD_FRACTION_OF_EXTENT: f64 = 1e-6;

    /// How much wider than the weld a point may miss a line or a plane and still be read as on
    /// it. A weld asks whether two points that were BUILT to be one are one, and this asks whether
    /// a point that has been through a float, a dot product and a plane crossing still lies where
    /// it was put, which accumulates more.
    const NEARNESS_OVER_WELD: f64 = 10.0;

    /// Starts from a mesh's own vertices, every one of which keeps its index.
    pub fn new(existing: &[Vector3Float]) -> Self {
        let mut pool = Self {
            vertices: Vec::with_capacity(existing.len()),
            buckets: HashMap::new(),
            weld_distance: Self::weld_distance_for(existing),
        };

        for &vertex in existing {
            pool.remember(pool.vertices.len() as i32, vertex);
            pool.vertices.push(vertex);
        }

        pool
    }

    /// How near two points have to be to be one vertex, for the part this pool was started from.
    pub fn weld_distance(&self) -> f64 {
        self.weld_distance
    }

    /// How near a point has to come to a line or a plane to be read as standing on it - wider
    /// than [`Self::weld_distance`], and scaled with it.
    pub fn near_enough(&self) -> f64 {
        self.weld_distance * Self::NEARNESS_OVER_WELD
    }

    /// How far a CUT will reach along a side for a point already standing where its crossing
    /// would go - twice [`Self::near_enough`]. It is not the ruler itself because the reach is
    /// ALONG the side while the ruler is across the cut: where a side runs nearly along the
    /// cutting plane, a point a ruler away along it stands a small fraction of one off the plane,
    /// and the plane is what it is really on. Two rulers is enough for that and far short of
    /// anything that is a second point rather than the same one (`SurfaceCutClip.ClipBy`,
    /// `SurfaceEndNotch.SnappedToTheProfile`).
    pub fn snap_near(&self) -> f64 {
        self.near_enough() * 2.0
    }

    /// The [`Self::near_enough`] a set of vertices asks for, without a pool: the reading a plan
    /// makes before anything is welded, on the same ruler the build will use.
    pub fn near_enough_for(existing: &[Vector3Float]) -> f64 {
        Self::weld_distance_for(existing) * Self::NEARNESS_OVER_WELD
    }

    /// The vertices, in index order.
    pub fn vertices(&self) -> &[Vector3Float] {
        &self.vertices
    }

    /// The vertices, in index order, taking the pool apart - what the builder hands the mesh.
    pub fn into_vertices(self) -> Vec<Vector3Float> {
        self.vertices
    }

    /// The weld distance a set of vertices asks for: a millionth of the diagonal of the box round
    /// them, never finer than [`Self::SMALLEST_WELD_DISTANCE`]. Read once, from the mesh handed
    /// in, so that every phase of one bevel measures with the same ruler even as the rewrite adds
    /// vertices.
    pub fn weld_distance_for(existing: &[Vector3Float]) -> f64 {
        let Some(&first) = existing.first() else {
            return Self::SMALLEST_WELD_DISTANCE;
        };

        let mut least = Vector3::from_vector3_float(first);
        let mut most = least;

        for &vertex in existing {
            let at = Vector3::from_vector3_float(vertex);

            least = Vector3::component_min(least, at);
            most = Vector3::component_max(most, at);
        }

        Self::SMALLEST_WELD_DISTANCE.max((most - least).length() * Self::WELD_FRACTION_OF_EXTENT)
    }

    /// How many vertices stand in the pool. What it is for is the reading that has to ask about a
    /// LINE rather than about a point - which vertices stand along one rim between two of them -
    /// where the buckets answer the wrong question (`SurfaceMitreSliver`).
    pub fn count(&self) -> i32 {
        self.vertices.len() as i32
    }

    /// The position of a vertex, read as doubles.
    pub fn position_of(&self, index: i32) -> Vector3 {
        Vector3::from_vector3_float(self.vertices[index as usize])
    }

    /// The index of the vertex at `position`, appending one if nothing is there yet.
    pub fn add(&mut self, position: Vector3) -> i32 {
        let stored = Vector3Float::from_vector3(position);

        if let Some(already) = self.try_find(stored) {
            return already;
        }

        let index = self.vertices.len() as i32;

        self.remember(index, stored);
        self.vertices.push(stored);

        index
    }

    /// The vertex NEAREST `position` standing within `how_near` of it, or `None` where none does.
    /// Unlike [`Self::add`] this asks about the ruler rather than the weld: what it is for is the
    /// clip, which has to call one place one point or leave a crack between two pieces bounded by
    /// vertices nothing downstream can tell apart (`SurfaceCutClip.ClipBy`).
    ///
    /// `qualifies` says what makes a vertex an answer at all, or `None` for any of them. It is
    /// asked of every candidate rather than of the nearest one afterwards: the nearest vertex and
    /// the nearest one the caller can USE are not the same vertex, and a reading that finds the
    /// first and then rejects it would answer differently as the pool grows under it - which is
    /// exactly what happens when one side is clipped twice over
    /// (`SurfaceEndGivesUpTheSection.PiecesOutsideOf`).
    ///
    /// The buckets are [`Self::near_enough`] across, so a `how_near` of one bucket is the 27
    /// cells round the point and anything more is a wider box: two buckets' reach is 125 cells.
    /// Looking too far costs candidates to measure and nothing else, since the distance is what
    /// decides. A later candidate at exactly the nearest distance so far replaces it, as in C#.
    pub fn try_find_near(
        &self,
        position: Vector3,
        how_near: f64,
        qualifies: Option<&dyn Fn(i32) -> bool>,
    ) -> Option<i32> {
        let stored = Vector3Float::from_vector3(position);
        let cell = self.cell_of(stored);
        let mut nearest = how_near;
        let mut index = -1;
        // C# `(int)Math.Ceiling(...)`; `as` saturates where C# would be undefined, which only a
        // `how_near` billions of rulers wide could reach.
        let reach = (how_near / self.near_enough()).ceil() as i32;

        for x in -reach..=reach {
            for y in -reach..=reach {
                for z in -reach..=reach {
                    let key = (cell.0 + x as i64, cell.1 + y as i64, cell.2 + z as i64);

                    let Some(candidates) = self.buckets.get(&key) else {
                        continue;
                    };

                    for &candidate in candidates {
                        let how_far =
                            (Vector3::from_vector3_float(self.vertices[candidate as usize])
                                - position)
                                .length();

                        if how_far <= nearest && qualifies.is_none_or(|accept| accept(candidate)) {
                            nearest = how_far;
                            index = candidate;
                        }
                    }
                }
            }
        }

        (index >= 0).then_some(index)
    }

    fn try_find(&self, position: Vector3Float) -> Option<i32> {
        let cell = self.cell_of(position);
        let at = Vector3::from_vector3_float(position);

        // The 27 cells around it as well as its own: two points a hair apart can still fall either
        // side of a cell boundary, and a weld that misses is the crack this type exists to prevent.
        for x in -1..=1 {
            for y in -1..=1 {
                for z in -1..=1 {
                    let Some(candidates) = self.buckets.get(&(cell.0 + x, cell.1 + y, cell.2 + z))
                    else {
                        continue;
                    };

                    for &candidate in candidates {
                        if (Vector3::from_vector3_float(self.vertices[candidate as usize]) - at)
                            .length()
                            <= self.weld_distance
                        {
                            return Some(candidate);
                        }
                    }
                }
            }
        }

        None
    }

    fn remember(&mut self, index: i32, position: Vector3Float) {
        let cell = self.cell_of(position);
        self.buckets.entry(cell).or_default().push(index);
    }

    /// Which bucket a point falls in. The cells are [`Self::near_enough`] across rather than the
    /// weld, so that [`Self::try_find_near`] can reach the RULER within the 27 cells round a
    /// point. What it costs is the weld's own search, which measures the candidates in a box ten
    /// times as wide before finding that they are not within a weld of anything.
    fn cell_of(&self, position: Vector3Float) -> Cell {
        let near_enough = self.near_enough();

        // C# `(long)Math.Floor(float / double)`: the float coordinate widens before the divide.
        (
            (f64::from(position.x) / near_enough).floor() as i64,
            (f64::from(position.y) / near_enough).floor() as i64,
            (f64::from(position.z) / near_enough).floor() as i64,
        )
    }
}
