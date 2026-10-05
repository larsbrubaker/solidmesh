//! One region's faces, laid out once in a uniform grid of cells so that a question asked of a
//! small part of the region does not have to walk the whole of it. Ports
//! `Bevel/Surface/SurfaceRegionFaceGrid.cs`. Reads a [`FaceGroup`]'s faces off its mesh and the
//! part's weld distance off [`SurfaceVertexPool`]; `SurfaceWallStandsClear` asks it its questions.
//!
//! **Why it exists.** `SurfaceWallStandsClear` asks two questions of a region PER CHAIN EDGE - what
//! stands within a radius of a point, and which of its faces the foot's line crosses - and a face
//! pick on a part like the flash hider gives 3925 faces and 3315 chain edges. Walked whole, that is
//! thirteen million face tests per question and it took a rebuild of that part from about a second
//! to half a minute.
//!
//! **It changes no answer.** A cell holds every face whose own box reaches into it, so a face
//! nearer than the radius has a point inside the queried box and is in a cell that box covers, and
//! a face the line crosses has a crossing point in a cell the line passes through. The queries
//! below therefore see every face the whole walk would have seen that could have mattered, and the
//! answers are bit-identical to the walk's.
//!
//! A face may be handed back more than once, since it is bucketed into every cell its box reaches.
//! Both callers take a minimum or a span over what they are given, so a repeat costs time and
//! nothing else.
//!
//! C#'s `For(mesh, region)` memoises one grid per region in a `ConditionalWeakTable`; here the
//! grid is built by [`SurfaceRegionFaceGrid::new`] and kept by whoever owns the region
//! (divergence 650).

use polygon_mesh::Mesh;
use vector_math::{cs_math, Vector3};

use super::SurfaceVertexPool;
use crate::topology::FaceGroup;

/// About how many faces a cell is meant to hold, which sets how fine the grid is.
const FACES_TO_A_CELL: f64 = 2.0;

/// No region is worth more cells than this, however oddly shaped it is.
const MOST_CELLS: i64 = 1 << 20;

/// C# `internal sealed class SurfaceRegionFaceGrid`.
#[derive(Debug, Clone)]
pub struct SurfaceRegionFaceGrid {
    faces: Vec<i32>,
    /// Three corners per face, in the same order as `faces`.
    corners: Vec<Vector3>,
    low: Vector3,
    cell: Vector3,
    across: i32,
    deep: i32,
    tall: i32,
    /// Where each cell's slice of `bucketed` begins, one longer than the cell count.
    starts_at: Vec<usize>,
    /// The faces of every cell in turn, as slots into `faces`.
    bucketed: Vec<usize>,
    weld_distance: f64,
}

impl SurfaceRegionFaceGrid {
    /// Lays the grid for one region of `mesh`.
    pub fn new(mesh: &Mesh, region: &FaceGroup) -> SurfaceRegionFaceGrid {
        // Read here rather than at each question: it walks every vertex of the whole part, and
        // asking it twice per chain edge over a rim of thousands was itself most of the cost this
        // type removes.
        let weld_distance = SurfaceVertexPool::weld_distance_for(&mesh.vertices);

        let region_faces = region.faces();
        let mut faces = Vec::with_capacity(region_faces.len());
        let mut corners = Vec::with_capacity(region_faces.len() * 3);

        let mut least = Vector3::new(f64::MAX, f64::MAX, f64::MAX);
        let mut most = Vector3::new(f64::MIN, f64::MIN, f64::MIN);

        for &face in region_faces {
            faces.push(face);
            let mesh_face = &mesh.faces[face as usize];
            for vertex in [mesh_face.v0, mesh_face.v1, mesh_face.v2] {
                let corner = Vector3::from_vector3_float(mesh.vertices[vertex as usize]);
                corners.push(corner);
                least = Vector3::component_min(least, corner);
                most = Vector3::component_max(most, corner);
            }
        }

        if faces.is_empty() {
            least = Vector3::ZERO;
            most = Vector3::ZERO;
        }

        let low = least;

        let span = most - least;
        let mut side = cs_math::max_f64(
            1e-9,
            (span.x + span.y + span.z)
                / 3.0
                / cs_math::max_f64(1.0, (faces.len() as f64 / FACES_TO_A_CELL).cbrt()),
        );

        let mut across = steps(span.x, side);
        let mut deep = steps(span.y, side);
        let mut tall = steps(span.z, side);

        while across as i64 * deep as i64 * tall as i64 > MOST_CELLS {
            side *= 2.0;
            across = steps(span.x, side);
            deep = steps(span.y, side);
            tall = steps(span.z, side);
        }

        let cell = Vector3::new(
            cs_math::max_f64(span.x / across as f64, 1e-9),
            cs_math::max_f64(span.y / deep as f64, 1e-9),
            cs_math::max_f64(span.z / tall as f64, 1e-9),
        );

        let mut grid = SurfaceRegionFaceGrid {
            faces,
            corners,
            low,
            cell,
            across,
            deep,
            tall,
            starts_at: Vec::new(),
            bucketed: Vec::new(),
            weld_distance,
        };

        // Counted first and filled second, so the whole grid is two flat arrays rather than a list
        // a cell.
        let cell_count = (across * deep * tall) as usize;
        let mut starts_at = vec![0usize; cell_count + 1];

        for slot in 0..grid.faces.len() {
            grid.over(slot, |at| starts_at[at + 1] += 1);
        }

        for at in 0..cell_count {
            starts_at[at + 1] += starts_at[at];
        }

        let mut filled = vec![0usize; cell_count];
        let mut bucketed = vec![0usize; starts_at[cell_count]];

        for slot in 0..grid.faces.len() {
            grid.over(slot, |at| {
                bucketed[starts_at[at] + filled[at]] = slot;
                filled[at] += 1;
            });
        }

        grid.starts_at = starts_at;
        grid.bucketed = bucketed;
        grid
    }

    /// The part's own weld distance, read once with the grid.
    pub fn weld_distance(&self) -> f64 {
        self.weld_distance
    }

    /// How near the region comes to a point, looked for only `within` of it: anything further off
    /// comes back as `f64::MAX`, which is all either caller needs of it.
    pub fn nearest_to(&self, at: Vector3, within: f64) -> f64 {
        let mut nearest = f64::MAX;
        let least = self.cell_of(at - Vector3::new(within, within, within));
        let most = self.cell_of(at + Vector3::new(within, within, within));

        for x in least.0..=most.0 {
            for y in least.1..=most.1 {
                for z in least.2..=most.2 {
                    let of = self.index_of(x, y, z);

                    for entry in self.starts_at[of]..self.starts_at[of + 1] {
                        let slot = self.bucketed[entry] * 3;

                        nearest = cs_math::min_f64(
                            nearest,
                            (at - nearest_on_the_triangle(
                                at,
                                self.corners[slot],
                                self.corners[slot + 1],
                                self.corners[slot + 2],
                            ))
                            .length(),
                        );
                    }
                }
            }
        }

        nearest
    }

    /// Every face the line through `through` along `along` could cross, with its three corners -
    /// the cells the line passes through, walked from one end of the region to the other. `visit`
    /// is handed the face index and its corners (C#'s `OnAFace` delegate).
    pub fn each_face_the_line_could_cross(
        &self,
        through: Vector3,
        along: Vector3,
        mut visit: impl FnMut(i32, Vector3, Vector3, Vector3),
    ) {
        let Some((from, to)) = self.the_lines_run_across_the_grid(through, along) else {
            return;
        };

        // Stepped cell by cell rather than sampled, so that no cell the segment so much as clips a
        // corner of is passed over: at each step the axis whose own next boundary comes soonest is
        // the one that moves, which is the whole of the walk.
        let from_cell = self.cell_of(from);
        let to_cell = self.cell_of(to);
        let mut walk = [from_cell.0, from_cell.1, from_cell.2];
        let stop = [to_cell.0, to_cell.1, to_cell.2];
        let run = [to.x - from.x, to.y - from.y, to.z - from.z];
        let side = [self.cell.x, self.cell.y, self.cell.z];
        let corner = [self.low.x, self.low.y, self.low.z];
        let start = [from.x, from.y, from.z];
        let counts = [self.across, self.deep, self.tall];

        let mut step = [0i32; 3];
        let mut next = [0f64; 3];
        let mut onward = [0f64; 3];

        for axis in 0..3 {
            if run[axis].abs() < 1e-15 {
                step[axis] = 0;
                next[axis] = f64::MAX;
                onward[axis] = f64::MAX;

                continue;
            }

            step[axis] = if run[axis] > 0.0 { 1 } else { -1 };
            onward[axis] = (side[axis] / run[axis]).abs();

            let edge = corner[axis]
                + ((walk[axis] + if step[axis] > 0 { 1 } else { 0 }) as f64 * side[axis]);

            next[axis] = (edge - start[axis]) / run[axis];
        }

        for _taken in 0..=(self.across + self.deep + self.tall + 3) {
            let of = self.index_of(walk[0], walk[1], walk[2]);

            for entry in self.starts_at[of]..self.starts_at[of + 1] {
                let slot = self.bucketed[entry] * 3;

                visit(
                    self.faces[self.bucketed[entry]],
                    self.corners[slot],
                    self.corners[slot + 1],
                    self.corners[slot + 2],
                );
            }

            if walk == stop {
                return;
            }

            let soonest = if next[0] <= next[1] && next[0] <= next[2] {
                0
            } else if next[1] <= next[2] {
                1
            } else {
                2
            };

            if next[soonest] > 1.0 || step[soonest] == 0 {
                return;
            }

            walk[soonest] += step[soonest];
            next[soonest] += onward[soonest];

            if walk[soonest] < 0 || walk[soonest] >= counts[soonest] {
                return;
            }
        }
    }

    /// The stretch of the line that lies inside the grid's own box, if any of it does.
    fn the_lines_run_across_the_grid(
        &self,
        through: Vector3,
        along: Vector3,
    ) -> Option<(Vector3, Vector3)> {
        let high = self.low
            + Vector3::new(
                self.cell.x * self.across as f64,
                self.cell.y * self.deep as f64,
                self.cell.z * self.tall as f64,
            );
        let mut enters = f64::MIN;
        let mut leaves = f64::MAX;

        for axis in 0..3 {
            let (run, at, least, most) = match axis {
                0 => (along.x, through.x, self.low.x, high.x),
                1 => (along.y, through.y, self.low.y, high.y),
                _ => (along.z, through.z, self.low.z, high.z),
            };

            if run.abs() < 1e-15 {
                if at < least || at > most {
                    return None;
                }

                continue;
            }

            let one = (least - at) / run;
            let other = (most - at) / run;

            enters = cs_math::max_f64(enters, cs_math::min_f64(one, other));
            leaves = cs_math::min_f64(leaves, cs_math::max_f64(one, other));
        }

        if enters > leaves {
            return None;
        }

        if enters == f64::MIN || leaves == f64::MAX {
            // The line runs along an axis the grid has no thickness in; the whole of it is inside.
            enters = 0.0;
            leaves = 0.0;
        }

        Some((through + (along * enters), through + (along * leaves)))
    }

    /// Which cell a point falls in, clamped to the grid's own edges. C#'s `(int)` cast of an
    /// out-of-range or NaN quotient is clamped straight after, so Rust's saturating `as` gives the
    /// same cell.
    fn cell_of(&self, point: Vector3) -> (i32, i32, i32) {
        let clamp = |at: f64, count: i32| (count - 1).min((at.trunc() as i32).max(0));
        (
            clamp((point.x - self.low.x) / self.cell.x, self.across),
            clamp((point.y - self.low.y) / self.cell.y, self.deep),
            clamp((point.z - self.low.z) / self.cell.z, self.tall),
        )
    }

    fn index_of(&self, x: i32, y: i32, z: i32) -> usize {
        ((((x * self.deep) + y) * self.tall) + z) as usize
    }

    /// Every cell one face's own box reaches into.
    fn over(&self, slot: usize, mut then: impl FnMut(usize)) {
        let a = self.corners[slot * 3];
        let b = self.corners[(slot * 3) + 1];
        let c = self.corners[(slot * 3) + 2];
        let least = Vector3::component_min(a, Vector3::component_min(b, c));
        let most = Vector3::component_max(a, Vector3::component_max(b, c));

        let from = self.cell_of(least);
        let to = self.cell_of(most);

        for x in from.0..=to.0 {
            for y in from.1..=to.1 {
                for z in from.2..=to.2 {
                    then(self.index_of(x, y, z));
                }
            }
        }
    }
}

/// The point of one triangle nearest a point - the perpendicular foot where it lands inside, and
/// the nearest point of the nearest side where it does not.
pub fn nearest_on_the_triangle(at: Vector3, a: Vector3, b: Vector3, c: Vector3) -> Vector3 {
    let normal = (b - a).cross(c - a);

    if normal.length_squared() > 0.0 {
        let normal = normal.get_normal();

        let foot = at - (normal * (at - a).dot(normal));

        if inside(foot, a, b, c, normal)
            && inside(foot, b, c, a, normal)
            && inside(foot, c, a, b, normal)
        {
            return foot;
        }
    }

    let mut best = nearest_on_the_side(at, a, b);
    let second = nearest_on_the_side(at, b, c);
    let third = nearest_on_the_side(at, c, a);

    if (at - second).length_squared() < (at - best).length_squared() {
        best = second;
    }

    if (at - third).length_squared() < (at - best).length_squared() {
        third
    } else {
        best
    }
}

fn inside(at: Vector3, from: Vector3, to: Vector3, third: Vector3, normal: Vector3) -> bool {
    let side = (to - from).cross(normal);

    (at - from).dot(side) * (third - from).dot(side) >= 0.0
}

fn nearest_on_the_side(at: Vector3, from: Vector3, to: Vector3) -> Vector3 {
    let run = to - from;

    let run_squared = run.length_squared();
    if run_squared.is_nan() || run_squared <= 0.0 {
        return from;
    }

    let along = cs_math::min_f64(
        1.0,
        cs_math::max_f64(0.0, (at - from).dot(run) / run.length_squared()),
    );

    from + (run * along)
}

fn steps(span: f64, side: f64) -> i32 {
    // C#'s `(int)Math.Ceiling(...)`; a span over int range saturates here where C# would wrap,
    // which no region's span over its own cell side reaches.
    1.max((span / side).ceil() as i32)
}
