//! A box with rounded edges and corners, centred on the origin. One segment bevels every edge
//! and corner flat; more segments project a grid on each face onto spheres around the eight
//! inner corners. Either way every vertex is created once, under the name of the point it is (a
//! bevel corner, or a point of the grid), and every face is made of those vertices, so the box is
//! closed by construction. Points that a zero radius, or a radius using up a whole side, puts in
//! one place are given one name, and the faces that would have no area there are left out.

use std::collections::HashMap;

use solidmesh_math::Vector3;

use crate::mesh::Mesh;

/// The vertex and index lists a rounded box is built into.
pub struct RoundedCornerBox {
    indices: Vec<i32>,
    index_to_verts: HashMap<i32, i32>,
    edge_count: i32,
    /// Per axis, the grid line each grid line is: lines a zero radius or a used-up side puts in one
    /// place are one line.
    grid_lines: [Vec<i32>; 3],
    radius: f64,
    normals: Vec<Vector3>,
    vertices: Vec<Vector3>,
}

impl RoundedCornerBox {
    /// The box as a mesh.
    pub fn create(segments: i32, size_xyz: Vector3, radius: f64) -> Mesh {
        let round_box = RoundedCornerBox::new(segments, size_xyz, radius);
        Mesh::from_vector3_list_and_indices(&round_box.vertices, &round_box.indices)
    }

    /// The box's vertex and index lists for `segments` segments per rounded edge.
    pub fn new(segments: i32, size_xyz: Vector3, radius: f64) -> Self {
        let mut round_box = RoundedCornerBox {
            indices: Vec::new(),
            index_to_verts: HashMap::new(),
            edge_count: 0,
            grid_lines: Default::default(),
            radius: 0.0,
            normals: Vec::new(),
            vertices: Vec::new(),
        };
        if segments == 1 {
            round_box.build_single_segment_box(size_xyz, radius);
        } else {
            round_box.build_multi_segment_box(segments - 1, size_xyz, radius);
        }
        round_box
    }

    /// The vertex normals the multi-segment build records ([`RoundedCornerBox::create`] does
    /// not use them).
    pub fn normals(&self) -> &[Vector3] {
        &self.normals
    }

    fn build_single_segment_box(&mut self, size_xyz: Vector3, radius: f64) {
        self.vertices = Vec::new();
        self.indices = Vec::new();

        let min_size = size_xyz.x.min(size_xyz.y.min(size_xyz.z));
        let r = radius.min(min_size / 2.0);
        let half = [size_xyz.x / 2.0, size_xyz.y / 2.0, size_xyz.z / 2.0];
        let mut placed: HashMap<BevelCorner, i32> = HashMap::new();

        // 1) the six faces, inset by the radius
        for axis in 0..3 {
            for sign in [-1, 1] {
                let (b, c) = ((axis + 1) % 3, (axis + 2) % 3);
                let corners = [(-1, -1), (1, -1), (1, 1), (-1, 1)].map(|(sb, sc)| {
                    let mut signs = [0; 3];
                    (signs[axis], signs[b], signs[c]) = (sign, sb, sc);
                    BevelCorner::new(signs, Some(axis))
                });
                self.add_bevel_polygon(&corners, half, r, &mut placed);
            }
        }

        // 2) the flat bevel along each of the twelve edges, between the two faces it joins
        for (a, b) in [(0, 1), (1, 2), (2, 0)] {
            let c = 3 - a - b;
            for (sa, sb) in [(-1, -1), (1, -1), (1, 1), (-1, 1)] {
                let on = |face: usize, sc: i8| {
                    let mut signs = [0; 3];
                    (signs[a], signs[b], signs[c]) = (sa, sb, sc);
                    BevelCorner::new(signs, Some(face))
                };
                let corners = [on(a, -1), on(a, 1), on(b, 1), on(b, -1)];
                self.add_bevel_polygon(&corners, half, r, &mut placed);
            }
        }

        // 3) the triangle cutting off each of the eight corners
        for sx in [-1, 1] {
            for sy in [-1, 1] {
                for sz in [-1, 1] {
                    let corners = [0, 1, 2].map(|face| BevelCorner::new([sx, sy, sz], Some(face)));
                    self.add_bevel_polygon(&corners, half, r, &mut placed);
                }
            }
        }
    }

    /// Adds one face of the single segment box, wound outward, each corner the one vertex its
    /// [`BevelCorner`] names. Corners that are the same point once the radius has used up a side
    /// (or is zero) are the same vertex, so such a face loses them and adds what is left, if that
    /// still has area.
    fn add_bevel_polygon(
        &mut self,
        corners: &[BevelCorner],
        half: [f64; 3],
        r: f64,
        placed: &mut HashMap<BevelCorner, i32>,
    ) {
        // the winding comes from a box with room for every corner, where the faces are the
        // same polygons, so a face that shrinks keeps its outward winding
        let roomy = corners.iter().map(|corner| corner.position([2.0; 3], 1.0));
        let points: Vec<Vector3> = roomy.collect();
        let mut normal = Vector3::new(0.0, 0.0, 0.0);
        let mut center = Vector3::new(0.0, 0.0, 0.0);
        for (i, &point) in points.iter().enumerate() {
            normal += point.cross(points[(i + 1) % points.len()]);
            center += point;
        }
        let outward = normal.dot(center) > 0.0;

        let mut indices: Vec<i32> = Vec::with_capacity(corners.len());
        for corner in corners {
            let corner = corner.collapsed(half, r);
            let index = *placed.entry(corner).or_insert_with(|| {
                self.vertices.push(corner.position(half, r));
                self.vertices.len() as i32 - 1
            });
            if !indices.contains(&index) {
                indices.push(index);
            }
        }
        if !outward {
            indices.reverse();
        }
        for k in 1..indices.len().saturating_sub(1) {
            self.indices
                .extend_from_slice(&[indices[0], indices[k], indices[k + 1]]);
        }
    }

    fn build_multi_segment_box(&mut self, segments: i32, size_xyz: Vector3, radius_in: f64) {
        self.edge_count = 2 * (segments + 1);
        let capacity = (self.edge_count * self.edge_count * self.edge_count * 3) as usize;
        self.index_to_verts = HashMap::with_capacity(capacity);
        // A radius past half the smallest side cannot fit; the face inset, the grid step and the
        // corner spheres all use the clamped one, so the rounded edges meet the flat faces.
        let min_size = size_xyz.x.min(size_xyz.y.min(size_xyz.z));
        let radius = if radius_in > min_size / 2.0 {
            min_size / 2.0
        } else {
            radius_in
        };
        self.radius = radius;
        let dx = if segments > 0 {
            radius / segments as f64
        } else {
            0.0
        };

        let face_xyz = (size_xyz / 2.0) - Vector3::new(radius, radius, radius);

        // Grid line i along an axis is at -face - radius + i * dx for i up to `segments` (where it
        // reaches -face), and the next `segments + 1` lines mirror those from +face out.
        for axis in 0..3 {
            self.grid_lines[axis] = (0..self.edge_count)
                .map(|i| {
                    let low = i <= segments;
                    if radius == 0.0 {
                        // every line of a rounded side is on the flat face's edge
                        if low || face_xyz[axis] == 0.0 {
                            segments
                        } else {
                            segments + 1
                        }
                    } else if face_xyz[axis] == 0.0 && i == segments + 1 {
                        // the flat face has no width: its two edges are one line
                        segments
                    } else {
                        i
                    }
                })
                .collect();
        }

        // which face of each pair is reversed: xy-planes the bottom, xz-planes the back,
        // yz-planes the left
        self.add_grid_plane(segments, face_xyz, radius, dx, [0, 1, 2], 0);
        self.add_grid_plane(segments, face_xyz, radius, dx, [0, 2, 1], 1);
        self.add_grid_plane(segments, face_xyz, radius, dx, [1, 2, 0], 0);
    }

    /// The two parallel faces normal to `axes[2]`. The three pairs differ only in which axes the
    /// grid's `i`/`j` run along (`axes[0]`/`axes[1]`) and which of the pair is wound inward
    /// (`inversed_kid`).
    fn add_grid_plane(
        &mut self,
        segments: i32,
        face_xyz: Vector3,
        radius: f64,
        dx: f64,
        axes: [usize; 3],
        inversed_kid: usize,
    ) {
        let [a, b, fixed] = axes;
        let sign = [-1.0, 1.0];
        let ks = [0, segments * 2 + 1];
        let s = segments;

        // a grid point (i along a, j along b, k along the fixed axis) as (x, y, z) indices
        let grid = |i: i32, j: i32, k: i32| {
            let mut g = [0; 3];
            g[a] = i;
            g[b] = j;
            g[fixed] = k;
            g
        };
        let vector = |va: f64, vb: f64, vf: f64| {
            let mut v = Vector3::new(0.0, 0.0, 0.0);
            v[a] = va;
            v[b] = vb;
            v[fixed] = vf;
            v
        };

        for kid in 0..2 {
            let k = ks[kid];
            let inversed = kid == inversed_kid;
            let origin = vector(
                -face_xyz[a] - radius,
                -face_xyz[b] - radius,
                (face_xyz[fixed] + radius) * sign[kid],
            );
            let base_fixed = face_xyz[fixed] * sign[kid];
            let far_a = 2.0 * face_xyz[a] + radius;
            let far_b = 2.0 * face_xyz[b] + radius;
            for j in 0..=s {
                for i in 0..=s {
                    let (di, dj) = (dx * i as f64, dx * j as f64);
                    let pos = origin + vector(di, dj, 0.0);
                    self.add_vertex(
                        grid(i, j, k),
                        pos,
                        vector(-face_xyz[a], -face_xyz[b], base_fixed),
                    );

                    let pos = origin + vector(di + far_a, dj, 0.0);
                    self.add_vertex(
                        grid(i + s + 1, j, k),
                        pos,
                        vector(face_xyz[a], -face_xyz[b], base_fixed),
                    );

                    let pos = origin + vector(di + far_a, dj + far_b, 0.0);
                    self.add_vertex(
                        grid(i + s + 1, j + s + 1, k),
                        pos,
                        vector(face_xyz[a], face_xyz[b], base_fixed),
                    );

                    let pos = origin + vector(di, dj + far_b, 0.0);
                    self.add_vertex(
                        grid(i, j + s + 1, k),
                        pos,
                        vector(-face_xyz[a], face_xyz[b], base_fixed),
                    );
                }
            }

            // a grid cell (i, j)-(i + 1, j + 1) as two triangles
            let cell = |this: &mut Self, i: i32, j: i32| {
                let t = |this: &Self, i: i32, j: i32| this.translate_indices(grid(i, j, k));
                let (p00, p11, p01, p10) = (
                    t(this, i, j),
                    t(this, i + 1, j + 1),
                    t(this, i, j + 1),
                    t(this, i + 1, j),
                );
                this.add_face(p00, p11, p01, inversed);
                this.add_face(p00, p10, p11, inversed);
            };

            // corners
            for j in 0..s {
                for i in 0..s {
                    cell(self, i, j);
                    cell(self, i, j + s + 1);
                    cell(self, i + s + 1, j + s + 1);
                    cell(self, i + s + 1, j);
                }
            }

            // sides
            for i in 0..s {
                cell(self, i, s);
                cell(self, s, i);
                cell(self, i + s + 1, s);
                cell(self, s, i + s + 1);
            }

            // central
            cell(self, s, s);
        }
    }

    /// Adds a grid triangle, unless two of its corners are the same point (see `grid_lines`).
    fn add_face(&mut self, i: i32, j: i32, k: i32, inversed: bool) {
        if i == j || j == k || k == i {
            return;
        }
        if inversed {
            self.indices.extend_from_slice(&[i, k, j]);
        } else {
            self.indices.extend_from_slice(&[i, j, k]);
        }
    }

    fn grid_index(&self, ijk: [i32; 3]) -> i32 {
        let [i, j, k] = [0, 1, 2].map(|axis| self.grid_lines[axis][ijk[axis] as usize]);
        k * self.edge_count * self.edge_count + j * self.edge_count + i
    }

    fn add_vertex(&mut self, ijk: [i32; 3], pos: Vector3, base_pos: Vector3) {
        let pidx = self.grid_index(ijk);
        if self.index_to_verts.contains_key(&pidx) {
            return;
        }
        self.index_to_verts.insert(pidx, self.vertices.len() as i32);

        let mut dir = pos - base_pos;
        if dir.length_squared() > 0.0 {
            dir.normalize();
            self.vertices.push(base_pos + dir * self.radius);
            self.normals.push(dir);
        } else {
            self.vertices.push(pos);
            self.normals.push(pos.get_normal());
        }
    }

    fn translate_indices(&self, ijk: [i32; 3]) -> i32 {
        self.index_to_verts[&self.grid_index(ijk)]
    }
}

/// A corner of the single segment box's faces: the octant it is in (a sign per axis) and the
/// face it is on (`None` once the radius is zero and the faces meeting at a box corner share it).
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
struct BevelCorner {
    signs: [i8; 3],
    face: Option<usize>,
}

impl BevelCorner {
    fn new(signs: [i8; 3], face: Option<usize>) -> Self {
        BevelCorner { signs, face }
    }

    /// The name of the point this corner is for a box of `half` its size with radius `r`: on an
    /// axis where the radius uses up the whole side the sign no longer moves the point, and with a
    /// zero radius the face no longer does.
    fn collapsed(self, half: [f64; 3], r: f64) -> Self {
        let mut signs = self.signs;
        for (axis, sign) in signs.iter_mut().enumerate() {
            if self.face != Some(axis) && half[axis] - r == 0.0 {
                *sign = 1;
            }
        }
        let face = if r == 0.0 { None } else { self.face };
        BevelCorner { signs, face }
    }

    /// Where the corner is: out to the side on the axis of its face, in by the radius on the
    /// others.
    fn position(self, half: [f64; 3], r: f64) -> Vector3 {
        let coordinate = |axis: usize| {
            let extent = if self.face == Some(axis) || self.face.is_none() {
                half[axis]
            } else {
                half[axis] - r
            };
            f64::from(self.signs[axis]) * extent
        };
        Vector3::new(coordinate(0), coordinate(1), coordinate(2))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn is_closed_and_outward(mesh: &Mesh, expected_volume_at_most: f64) -> f64 {
        let volume = mesh.get_volume();
        assert!(
            volume > 0.0 && volume <= expected_volume_at_most + 1e-6,
            "volume {volume}"
        );
        assert!(mesh.is_manifold(), "not closed");
        volume
    }

    #[test]
    fn rust_only_single_segment_box_is_closed_and_bevelled() {
        let mesh = RoundedCornerBox::create(1, Vector3::new(10.0, 20.0, 30.0), 2.0);
        let bounds = mesh.get_axis_aligned_bounding_box();
        assert_eq!(bounds.min_xyz, Vector3::new(-5.0, -10.0, -15.0));
        assert_eq!(bounds.max_xyz, Vector3::new(5.0, 10.0, 15.0));
        let volume = is_closed_and_outward(&mesh, 6000.0);
        assert!(volume < 6000.0);
    }

    #[test]
    fn rust_only_multi_segment_box_is_closed_and_rounded() {
        let mesh = RoundedCornerBox::create(4, Vector3::new(10.0, 10.0, 10.0), 2.0);
        let bounds = mesh.get_axis_aligned_bounding_box();
        assert!((bounds.max_xyz.x - 5.0).abs() < 1e-5);
        assert!((bounds.min_xyz.z + 5.0).abs() < 1e-5);
        let volume = is_closed_and_outward(&mesh, 1000.0);
        assert!(volume > 900.0, "volume {volume}");
    }
}
