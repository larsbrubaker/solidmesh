//! The column a cutter must not reach into above a face something stands on: one obstructed face of
//! one selected edge ([`Obstruction`]), recorded by `BevelFaceFit` when it admitted the group, with
//! its constructors, the class's constants and the column's small readings (whether a face stands
//! clear of the plane, how far it stands on it, a polygon cut to a half space) and the columns
//! themselves ([`build`]): each obstruction's standing material, flattened into the band, unioned
//! into a footprint and extruded up the obstruction's height - with its shadow body where asked
//! for.

use clipper_library::clipper6::{IntPoint as Point64, Path as Path64, Paths as Paths64};
use clipper_library::clipper_statics;
use clipper_library::polygons_extensions::PolygonsExtensions;
use clipper_library::{ClipperError, DEFAULT_SCALING};
use indexmap::IndexSet;
use polygon_mesh::processors::outline_to_mesh::extrude;
use polygon_mesh::{Mesh, MeshEdgeGraph};
use std::collections::HashMap;
use vector_math::{ieee_math, Matrix4X4, Vector3, Vector4};

use super::{bevel_shadow_body, bevel_working_mesh};

use crate::topology::FeatureEdge;

/// How far past the band's own extent the column is drawn, as a fraction of it.
pub const BAND_MARGIN_FRACTION: f64 = 0.05;

/// How far in front of a face a point has to stand to be off it: four float steps, relative.
pub const IN_FRONT_OF_THE_FACE: f64 = 4.0 * 1.192092896e-7;

/// The scale the column's outline is taken to integers at for the polygon clipper: the extruder's
/// own grid, so the footprint is extruded as it stands.
pub const CLIPPER_SCALE: f64 = DEFAULT_SCALING;

/// One face of one selected edge that something is standing on - what `BevelFaceFit` recorded when
/// it admitted the group, and everything the clip needs to draw the column.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Obstruction {
    /// The obstructed face's outward normal, which is the direction the shadow is cast along.
    pub plane_normal: Vector3,
    /// The face's plane offset: every point whose dot with `plane_normal` is this.
    pub plane_distance: f64,
    /// The selected edge's start, which the band is measured from.
    pub origin: Vector3,
    /// The direction along the selected edge.
    pub along: Vector3,
    /// How far the selected edge runs.
    pub along_length: f64,
    /// The direction that leaves the edge square to it and heads into the obstructed face.
    pub across: Vector3,
    /// How far across the face this edge's tool reaches, which is how deep the band is.
    pub setback: f64,
    /// The radius of the ROUND this cutter carries, or zero where the profile is not a round. It
    /// says only where the cutter's own surface runs across this face: a round is tangent to the
    /// face at `setback` across it, so every point of the cutter is OUTSIDE the cylinder whose
    /// axis lies this far behind the plane on the tangent line.
    pub round_radius: f64,
    /// How far off its own plane the face was allowed to be when `BevelFaceFit` admitted it as
    /// planar - and so the smallest height that can count as standing ON it.
    pub planar_within: f64,
    /// The obstruction's OWN first triangle - the face on the far side of the edge the walk
    /// stopped at - which is where the footprint's flood starts. Without a seed the footprint
    /// would be "everything in front of the plane", which sweeps in things that merely pass over
    /// the face.
    pub seed_face: i32,
}

impl Obstruction {
    /// An obstruction from its parts (`round_radius` is 0 for a flat one).
    #[allow(clippy::too_many_arguments)] // one field per argument.
    pub fn new(
        plane_normal: Vector3,
        plane_distance: f64,
        origin: Vector3,
        along: Vector3,
        along_length: f64,
        across: Vector3,
        setback: f64,
        planar_within: f64,
        seed_face: i32,
        round_radius: f64,
    ) -> Self {
        Self {
            plane_normal,
            plane_distance,
            origin,
            along,
            along_length,
            across,
            setback,
            round_radius,
            planar_within,
            seed_face,
        }
    }

    /// The obstruction a face of normal `face_normal` makes along the selected `edge`.
    pub fn on(
        face_normal: Vector3,
        edge: &FeatureEdge,
        across: Vector3,
        setback: f64,
        planar_within: f64,
        seed_face: i32,
        round_radius: f64,
    ) -> Self {
        Self::new(
            face_normal,
            face_normal.dot(edge.start),
            edge.start,
            edge.direction,
            (edge.end - edge.start).length(),
            across,
            setback,
            planar_within,
            seed_face,
            round_radius,
        )
    }
}

/// True when some corner of `face` stands more than `in_front` in front of the obstruction's plane.
pub fn clear_of_the_plane(
    mesh: &Mesh,
    face: i32,
    obstruction: &Obstruction,
    in_front: f64,
) -> bool {
    let mesh_face = mesh.faces[face as usize];
    [mesh_face.v0, mesh_face.v1, mesh_face.v2]
        .iter()
        .any(|&vertex| {
            obstruction
                .plane_normal
                .dot(Vector3::from_vector3_float(mesh.vertices[vertex as usize]))
                - obstruction.plane_distance
                > in_front
        })
}

/// How far in front of its plane a point may stand and still be standing ON the obstruction: its
/// own planarity, or float noise at the band's reach from the origin, whichever is more.
pub fn standing_on(obstruction: &Obstruction) -> f64 {
    let reach_from_origin =
        obstruction.origin.length() + obstruction.along_length + obstruction.setback;
    ieee_math::max_f64(
        obstruction.planar_within,
        reach_from_origin * IN_FRONT_OF_THE_FACE,
    )
}

/// The part of a convex polygon on the kept side (`normal . p >= distance`) of a plane.
pub fn clip_to_half_space(polygon: &[Vector3], normal: Vector3, distance: f64) -> Vec<Vector3> {
    let mut kept = Vec::with_capacity(polygon.len() + 1);
    for i in 0..polygon.len() {
        let current = polygon[i];
        let next = polygon[(i + 1) % polygon.len()];
        let here = normal.dot(current) - distance;
        let there = normal.dot(next) - distance;
        if here >= 0.0 {
            kept.push(current);
        }

        if (here > 0.0 && there < 0.0) || (here < 0.0 && there > 0.0) {
            kept.push(current + ((next - current) * (here / (here - there))));
        }
    }

    kept
}

/// The columns (and, with `include_bodies`, the shadow bodies) a cutter must not reach into above
/// the obstructions, each a usable operand, with how many bodies were added.
/// `body_clearance_multiplier` is normally 1; Clipper's failures come back as the error.
pub fn build(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    obstructions: &[Obstruction],
    include_bodies: bool,
    body_clearance_multiplier: f64,
) -> Result<(Vec<Mesh>, i32), ClipperError> {
    let mut columns = Vec::new();
    let mut bodies = 0;

    // The graph knows an edge's faces but not which edge joins two vertices, and the flood asks the
    // second question. Built once for the whole group rather than once per obstruction.
    let mut edges_by_vertex_pair = HashMap::with_capacity(graph.edge_count() as usize);
    for edge_index in 0..graph.edge_count() {
        let v0 = graph.get_vertex0(edge_index);
        let v1 = graph.get_vertex1(edge_index);
        edges_by_vertex_pair.insert((v0.min(v1), v0.max(v1)), edge_index);
    }

    for obstruction in obstructions {
        let in_front = standing_on(obstruction);
        let margin = obstruction.setback * BAND_MARGIN_FRACTION;
        let across_at_edge = obstruction.across.dot(obstruction.origin);
        let along_at_start = obstruction.along.dot(obstruction.origin);

        // Every plane the band is cut by, each keeping the side its normal points to. The first is
        // what makes it a SHADOW - only material clear of the face casts one - and the other four
        // are the band, which keeps the footprint down to the region the cutter can occupy.
        let bounds = [
            (
                obstruction.plane_normal,
                obstruction.plane_distance + in_front,
            ),
            (obstruction.across, across_at_edge - margin),
            (
                -obstruction.across,
                -(across_at_edge + obstruction.setback + margin),
            ),
            (obstruction.along, along_at_start - margin),
            (
                -obstruction.along,
                -(along_at_start + obstruction.along_length + margin),
            ),
        ];

        // The frame the footprint is unioned in: across the face, and the direction that makes the
        // three axes RIGHT handed with the face's normal - built, not taken from the edge, whose own
        // direction is whichever way the run was linked (a left-handed frame mirrors and turns the
        // column inside out). Numbers going to Clipper are millimetres across and along the face,
        // so its integer grid is the same size wherever on the part this happens to be.
        let along_frame = obstruction.plane_normal.cross(obstruction.across);
        let along_frame_at_origin = along_frame.dot(obstruction.origin);
        let mut footprint: Paths64 = Vec::new();
        let standing = standing_material(mesh, graph, &edges_by_vertex_pair, obstruction, in_front);
        let mut highest = 0.0f64;

        for &face in &standing {
            let mesh_face = mesh.faces[face as usize];
            let mut polygon: Vec<Vector3> = [mesh_face.v0, mesh_face.v1, mesh_face.v2]
                .iter()
                .map(|&v| Vector3::from_vector3_float(mesh.vertices[v as usize]))
                .collect();

            for &vertex in &polygon {
                highest = ieee_math::max_f64(
                    highest,
                    obstruction.plane_normal.dot(vertex) - obstruction.plane_distance,
                );
            }

            for &(normal, distance) in &bounds {
                polygon = clip_to_half_space(&polygon, normal, distance);
                if polygon.len() < 3 {
                    break;
                }
            }

            if polygon.len() < 3 {
                continue;
            }

            // Truncated toward zero onto Clipper's integer grid.
            let flattened: Path64 = polygon
                .iter()
                .map(|&vertex| {
                    Point64::new(
                        ((obstruction.across.dot(vertex) - across_at_edge) * CLIPPER_SCALE) as i64,
                        ((along_frame.dot(vertex) - along_frame_at_origin) * CLIPPER_SCALE) as i64,
                    )
                })
                .collect();

            // Unioned as it goes rather than triangle by triangle at the end: one prism per patch of
            // footprint is what the boolean wants.
            footprint = footprint.union_polygon(&flattened)?;
        }

        for polygon in &footprint {
            // A column the boolean would refuse is dropped where it is made, not turned into a
            // refused clip that costs the whole cutter its trim.
            if let Some(column) = column(&footprint, polygon, obstruction, along_frame, highest)? {
                if bevel_working_mesh::is_usable_operand(Some(&column), None) {
                    columns.push(column);
                }
            }
        }

        if !include_bodies {
            continue;
        }

        // And the obstruction's own BODY, which the footprint's prism does not describe, offered on
        // the same terms as the columns and unioned with them by the caller's one subtract.
        if let Some(body) = bevel_shadow_body::build(
            mesh,
            graph,
            &edges_by_vertex_pair,
            obstruction,
            in_front,
            highest,
            across_at_edge,
            along_at_start,
            margin,
            body_clearance_multiplier,
        ) {
            if bevel_working_mesh::is_usable_operand(Some(&body), None) {
                columns.push(body);
                bodies += 1;
            }
        }
    }

    Ok((columns, bodies))
}

/// The faces reached from the obstruction's seed face across its edges while each stands clear of
/// the obstruction's plane, in the order reached.
pub fn standing_material(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edges_by_vertex_pair: &HashMap<(i32, i32), i32>,
    obstruction: &Obstruction,
    in_front: f64,
) -> IndexSet<i32> {
    let mut reached = IndexSet::new();
    if obstruction.seed_face < 0 || obstruction.seed_face as usize >= mesh.faces.len() {
        return reached;
    }

    let mut to_visit = vec![obstruction.seed_face];
    reached.insert(obstruction.seed_face);
    while let Some(face) = to_visit.pop() {
        for side in 0..3 {
            let mesh_face = mesh.faces[face as usize];
            let (vertex_a, vertex_b) = match side {
                0 => (mesh_face.v0, mesh_face.v1),
                1 => (mesh_face.v1, mesh_face.v2),
                _ => (mesh_face.v2, mesh_face.v0),
            };
            let Some(&edge_index) =
                edges_by_vertex_pair.get(&(vertex_a.min(vertex_b), vertex_a.max(vertex_b)))
            else {
                continue;
            };

            for i in 0..graph.get_face_count(edge_index) {
                let neighbour = graph.get_face(edge_index, i);
                if reached.contains(&neighbour)
                    || !clear_of_the_plane(mesh, neighbour, obstruction, in_front)
                {
                    continue;
                }

                reached.insert(neighbour);
                to_visit.push(neighbour);
            }
        }
    }

    reached
}

/// One footprint OUTER's column, with the holes it contains, extruded from `2 setbacks` behind
/// the face to the obstruction's height and carried into the part's frame; `None` for a hole (told
/// apart by orientation - it is built with its outer), a sliver, or an empty extrusion.
pub fn column(
    footprint: &Paths64,
    polygon: &Path64,
    obstruction: &Obstruction,
    along_frame: Vector3,
    stands_up_to: f64,
) -> Result<Option<Mesh>, ClipperError> {
    if clipper_statics::area(polygon).abs() < 1e-9 || !clipper_statics::orientation(polygon) {
        return Ok(None);
    }

    let below = (obstruction.setback * 2.0) + obstruction.planar_within;
    let above = stands_up_to + obstruction.planar_within;

    // The outer and the holes inside it, still on the clipper grid, which is the extruder's own
    // ([`CLIPPER_SCALE`] is `DEFAULT_SCALING`).
    let mut outline: Paths64 = vec![polygon.clone()];
    for hole in footprint {
        if !clipper_statics::orientation(hole)
            && clipper_statics::point_in_polygon(hole[0], polygon) != 0
        {
            outline.push(hole.clone());
        }
    }

    // Extruded by the shared extruder, so a footprint with holes in it is triangulated by the
    // same code every other extrusion uses.
    let mut mesh = extrude(&outline, below + above)?;
    if mesh.faces.is_empty() {
        return Ok(None);
    }

    // Out of the extruder's own frame - the band's two directions and z up from zero - into the
    // part's.
    mesh.transform(Matrix4X4::from_rows(
        Vector4::from_vector3_w(obstruction.across, 0.0),
        Vector4::from_vector3_w(along_frame, 0.0),
        Vector4::from_vector3_w(obstruction.plane_normal, 0.0),
        Vector4::from_vector3_w(obstruction.origin - (obstruction.plane_normal * below), 1.0),
    ));
    Ok(Some(mesh))
}
