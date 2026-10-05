//! The junction's geometry readings (`Bevel/Sections/SectionJunction.cs`, the C# `internal static
//! class SectionJunction`; free functions and `impl` blocks here): one chain edge's canal at the
//! junction ([`Canal`]'s constructor and readings), the junction's tolerances and lattice, where two
//! lines meet, a facet's normal and whether it holds a point, the strip's own section at the junction,
//! which side of the turn is the shared flat one, and the junction section's readings of its own
//! samples (its wall ends, patch triangles and directed edges). Split from [`super::section_junction`], which
//! keeps the junction's types, by responsibility; the junction's construction lands with a later
//! run of this package.

use polygon_mesh::Mesh;
use vector_math::{cs_math, Vector3};

use super::bevel_section::SectionProfileDoesNotEndOnItsBoundaries;
use super::section_junction::{Canal, JunctionPiece, JunctionSection};
use super::{section_builder, section_corner, BevelSection, SectionKind, SectionSide};
use crate::bevel::float_grid;
use crate::topology::ChainEdge;

/// C# `SameToWithin`: how nearly two lengths or directions must agree to be read as the same.
pub const SAME_TO_WITHIN: f64 = 1e-9;

/// C# `FlatSideDot`: how nearly two normals must line up to be ONE flat side of a junction.
pub const FLAT_SIDE_DOT: f64 = 1.0 - 1e-9;

/// C# `OnBothCanalsToWithinThisMuchOfTheRadius`: how far a point may stand off both canals, as a
/// fraction of the radius, and still be read as on both.
pub const ON_BOTH_CANALS_TO_WITHIN_THIS_MUCH_OF_THE_RADIUS: f64 = 1e-9;

/// C# `GridUlps`: how many float steps the lattice of a junction's own points is read as.
const GRID_ULPS: f64 = 4.0;

impl Canal {
    /// C#'s `Canal` constructor: one chain edge's canal of `radius` at `station`, its flat and wall
    /// faces by which side is flat, its axis a radius clear of both, and the direction across the
    /// wall square to the flat face.
    pub fn new(
        edge: &ChainEdge,
        station: Vector3,
        radius: f64,
        flat_on_the_left: bool,
        lattice: f64,
    ) -> Self {
        let flat = if flat_on_the_left {
            edge.left_normal()
        } else {
            edge.right_normal()
        }
        .get_normal();
        let wall = if flat_on_the_left {
            edge.right_normal()
        } else {
            edge.left_normal()
        }
        .get_normal();

        Self {
            along: edge.direction().get_normal(),
            flat,
            wall,
            radius,
            lattice,
            wedge: flat.dot(wall).clamp(-1.0, 1.0).acos(),
            axis: section_corner::the_axis_at(edge, station, radius),
            across: (wall - (flat * wall.dot(flat))).get_normal(),
            lean: 0.0,
        }
    }

    /// C# `PointAt(v, through, normal)`: [`Canal::point_at_distance`] at the canal's own radius.
    pub fn point_at(&mut self, v: f64, through: Vector3, normal: Vector3) -> Vector3 {
        self.point_at_distance(v, self.radius, through, normal)
    }

    /// C# `PointAt(v, distance, through, normal)`: the point a fraction `v` round the canal's arc
    /// from its flat face to its wall, `distance` from the axis, slid along the edge onto the plane
    /// through `through` square to `normal`. Sets the canal's lean for that plane, as C# does.
    pub fn point_at_distance(
        &mut self,
        v: f64,
        distance: f64,
        through: Vector3,
        normal: Vector3,
    ) -> Vector3 {
        self.set_lean_for(normal);

        let swept =
            self.axis + (section_builder::slerp(self.flat, self.wall, v, self.wedge) * distance);

        swept - (self.along * ((swept - through).dot(normal) / self.lean))
    }

    /// C# `DistanceFromTheAxis`: how far a point stands from the canal's axis.
    pub fn distance_from_the_axis(&self, point: Vector3) -> f64 {
        let offset = point - self.axis;

        (offset - (self.along * offset.dot(self.along))).length()
    }

    /// C# `TryParameterOf`: how far round the canal's arc a point on the canal stands, as a fraction
    /// of the wedge - `None` where the point is off the canal (read against the lattice as well as
    /// the radius, for a part stood far from the origin) or the canal has no wedge.
    pub fn try_parameter_of(&self, point: Vector3) -> Option<f64> {
        let offset = point - self.axis;
        let radial = offset - (self.along * offset.dot(self.along));

        if (radial.length() - self.radius).abs()
            > cs_math::max_f64(SAME_TO_WITHIN * self.radius, self.lattice)
            || self.wedge < SAME_TO_WITHIN
        {
            return None;
        }

        let direction = radial.get_normal();

        Some(direction.dot(self.across).atan2(direction.dot(self.flat)) / self.wedge)
    }

    /// C# `SetLeanFor`: how far the canal's own direction leans onto `normal`.
    pub fn set_lean_for(&mut self, normal: Vector3) {
        self.lean = self.along.dot(normal);
    }
}

/// C# `LatticeOf`: the lattice of a junction's own points - a few float steps at the largest of
/// their coordinates, and never under [`SAME_TO_WITHIN`].
pub fn lattice_of(points: &[Vector3]) -> f64 {
    let mut largest = 0.0f32;

    for at in points {
        let biggest = cs_math::max_f64(at.x.abs(), cs_math::max_f64(at.y.abs(), at.z.abs())) as f32;
        largest = largest.max(biggest);
    }

    let ulp = float_grid::step_at(largest);

    cs_math::max_f64(SAME_TO_WITHIN, GRID_ULPS * ulp)
}

/// C# `TheNormalOf`: a facet's unit normal, read off its corners.
pub fn the_normal_of(mesh: &Mesh, face: i32) -> Vector3 {
    let drawn = &mesh.faces[face as usize];
    let a = position(mesh, drawn.v0);
    let b = position(mesh, drawn.v1);
    let c = position(mesh, drawn.v2);

    (b - a).cross(c - a).get_normal()
}

/// C# `TheTriangleHolds`: whether a point of the facet's plane stands inside the facet, or outside
/// it by no more than `lattice`; a facet with a side of no length holds nothing.
pub fn the_triangle_holds(
    mesh: &Mesh,
    face: i32,
    face_normal: Vector3,
    at: Vector3,
    lattice: f64,
) -> bool {
    let drawn = &mesh.faces[face as usize];
    let corners = [
        position(mesh, drawn.v0),
        position(mesh, drawn.v1),
        position(mesh, drawn.v2),
    ];

    for k in 0..3 {
        let a = corners[k];
        let b = corners[(k + 1) % 3];
        let along = b - a;
        let length = along.length();

        if length < SAME_TO_WITHIN {
            return false;
        }

        if along.cross(at - a).dot(face_normal) / length < -lattice {
            return false;
        }
    }

    true
}

/// C# `TheStripsSection`: the section one strip ends on at a junction - the junction's samples from
/// the flat side (sample 0) to this edge's own wall end, read from the LEFT side's end to the right's.
/// Both ends are offsets: the flat side's is the offset-meet in the shared plane, the wall side's this
/// edge's own canal tangency, which at a junction rides nothing - the fold leaving the vertex is
/// taken by the PATCH. (C#'s constructor cannot refuse this profile; the `Result` is its check.)
#[allow(clippy::too_many_arguments)]
pub fn the_strips_section(
    vertex: i32,
    station: Vector3,
    tangent: Vector3,
    samples: &[i32],
    wall_end: usize,
    flat_on_the_left: bool,
    centre: Vector3,
    setback: f64,
) -> Result<BevelSection, SectionProfileDoesNotEndOnItsBoundaries> {
    let profile: Vec<i32> = (0..=wall_end)
        .map(|k| {
            if flat_on_the_left {
                samples[k]
            } else {
                samples[wall_end - k]
            }
        })
        .collect();

    Ok(BevelSection {
        kind: SectionKind::Junction,
        ..BevelSection::new(
            vertex,
            station,
            tangent,
            profile[0],
            profile[profile.len() - 1],
            profile.clone(),
            centre,
            centre,
            SectionSide::Offset,
            SectionSide::Offset,
            -1,
            -1,
            setback,
        )?
    })
}

/// C# `TryReadTheFlatSide`: which side of the turn two edges share as ONE flat plane, `None` where
/// both do (a chain that does not turn) or neither (a corner with four faces).
pub fn try_read_the_flat_side(arriving: &ChainEdge, leaving: &ChainEdge) -> Option<bool> {
    let left_is_shared = arriving
        .left_normal()
        .get_normal()
        .dot(leaving.left_normal().get_normal())
        > FLAT_SIDE_DOT;
    let right_is_shared = arriving
        .right_normal()
        .get_normal()
        .dot(leaving.right_normal().get_normal())
        > FLAT_SIDE_DOT;

    (left_is_shared != right_is_shared).then_some(left_is_shared)
}

/// C# `MeetOfLines`: the midpoint of the two nearest points of two lines, and how far apart those
/// are (C#'s `skew`) - `from0` and `f64::MAX` where the lines run parallel.
pub fn meet_of_lines(
    from0: Vector3,
    along0: Vector3,
    from1: Vector3,
    along1: Vector3,
) -> (Vector3, f64) {
    let lean = along0.dot(along1);
    let denominator = 1.0 - (lean * lean);

    if denominator < SAME_TO_WITHIN {
        return (from0, f64::MAX);
    }

    let between = from1 - from0;
    let step0 = (between.dot(along0) - (lean * between.dot(along1))) / denominator;
    let step1 = ((lean * between.dot(along0)) - between.dot(along1)) / denominator;
    let one = from0 + (along0 * step0);
    let other = from1 + (along1 * step1);

    ((one + other) / 2.0, (one - other).length())
}

impl JunctionSection {
    /// C#'s `JunctionSection` constructor.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        chain_vertex: i32,
        station: Vector3,
        through: Vector3,
        normal: Vector3,
        samples: Vec<i32>,
        arriving_wall_end: i32,
        leaving_wall_end: i32,
        fold_point: i32,
        arriving_wall: Vector3,
        leaving_wall: Vector3,
        flat_on_the_left: bool,
        setback_arriving: f64,
        setback_leaving: f64,
        arriving_section: std::rc::Rc<BevelSection>,
        leaving_section: std::rc::Rc<BevelSection>,
        carried_onto: Vec<i32>,
    ) -> Self {
        Self {
            chain_vertex,
            station,
            through,
            normal,
            samples,
            carried_onto,
            arriving_wall_end,
            leaving_wall_end,
            fold_point,
            arriving_wall,
            leaving_wall,
            flat_on_the_left,
            setback_arriving,
            setback_leaving,
            arriving_section,
            leaving_section,
        }
    }
}

/// A vertex's position, read as doubles.
fn position(mesh: &Mesh, vertex: i32) -> Vector3 {
    Vector3::from_vector3_float(mesh.vertices[vertex as usize])
}

/// The junction section's readings of its own samples (C# `NearWallEnd`, `FarWallEnd`,
/// `LiesInTheArrivingWall`, `LongStripRunsUpTheSamples`, `PatchTriangles`, `DirectedEdges`).
impl JunctionSection {
    /// C# `NearWallEnd`: the nearer of the two wall ends along the samples.
    pub fn near_wall_end(&self) -> i32 {
        self.arriving_wall_end.min(self.leaving_wall_end)
    }

    /// C# `FarWallEnd`: the farther of the two wall ends along the samples.
    pub fn far_wall_end(&self) -> i32 {
        self.arriving_wall_end.max(self.leaving_wall_end)
    }

    /// C# `LiesInTheArrivingWall`: whether a facet with `face_normal` belongs to the arriving edge's
    /// wall rather than the leaving edge's.
    pub fn lies_in_the_arriving_wall(&self, face_normal: Vector3) -> bool {
        face_normal.dot(self.arriving_wall) >= face_normal.dot(self.leaving_wall)
    }

    /// C# `LongStripRunsUpTheSamples`: whether the strip that walks out to the FAR wall end walks the
    /// samples upwards.
    pub fn long_strip_runs_up_the_samples(&self) -> bool {
        self.flat_on_the_left == (self.far_wall_end() == self.leaving_wall_end)
    }

    /// C# `PatchTriangles`: the patch of wall between the two wall ends, fanned from the fold point,
    /// wound the way [`Self::long_strip_runs_up_the_samples`] says.
    pub fn patch_triangles(&self) -> Vec<[i32; 3]> {
        let up = self.long_strip_runs_up_the_samples();

        (self.near_wall_end()..self.far_wall_end())
            .map(|k| {
                let (here, next) = (self.samples[k as usize], self.samples[k as usize + 1]);

                if up {
                    [self.fold_point, next, here]
                } else {
                    [self.fold_point, here, next]
                }
            })
            .collect()
    }

    /// C# `DirectedEdges`: every directed edge the junction's pieces lay, with the piece that lays it -
    /// the long strip out to the far wall end and the short one to the near one in the opposite sense
    /// (so the stretch they share cancels), the patch's triangles, and the two rebuilt rims' chords
    /// onto and off the fold point.
    pub fn directed_edges(&self) -> Vec<(i32, i32, JunctionPiece)> {
        let mut edges = Vec::new();

        let up = self.long_strip_runs_up_the_samples();
        let (long_strip, short_strip) = if self.far_wall_end() == self.leaving_wall_end {
            (JunctionPiece::LeavingStrip, JunctionPiece::ArrivingStrip)
        } else {
            (JunctionPiece::ArrivingStrip, JunctionPiece::LeavingStrip)
        };
        let sample = |k: i32| self.samples[k as usize];

        for k in 0..self.far_wall_end() {
            edges.push(if up {
                (sample(k), sample(k + 1), long_strip)
            } else {
                (sample(k + 1), sample(k), long_strip)
            });
        }

        for k in 0..self.near_wall_end() {
            edges.push(if up {
                (sample(k + 1), sample(k), short_strip)
            } else {
                (sample(k), sample(k + 1), short_strip)
            });
        }

        for [a, b, c] in self.patch_triangles() {
            edges.push((a, b, JunctionPiece::Patch));
            edges.push((b, c, JunctionPiece::Patch));
            edges.push((c, a, JunctionPiece::Patch));
        }

        // The patch lays one chord onto the fold and one off it, so the two rebuilt rims lay their
        // opposites - and which way round turns on the same sense the patch was wound with.
        let near = sample(self.near_wall_end());
        let far = sample(self.far_wall_end());

        edges.push(if up {
            (self.fold_point, near, JunctionPiece::NearWallRim)
        } else {
            (near, self.fold_point, JunctionPiece::NearWallRim)
        });

        edges.push(if up {
            (far, self.fold_point, JunctionPiece::FarWallRim)
        } else {
            (self.fold_point, far, JunctionPiece::FarWallRim)
        });

        edges
    }
}
