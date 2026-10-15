//! A box with rounded edges and corners, centred on the origin. One segment bevels every edge
//! and corner flat; more segments project a grid on each face onto spheres around the eight
//! inner corners.

use std::collections::HashMap;

use vector_math::Vector3;

use crate::mesh::Mesh;

/// The vertex and index lists a rounded box is built into.
pub struct RoundedCornerBox {
    indices: Vec<i32>,
    index_to_verts: HashMap<i32, i32>,
    edge_count: i32,
    radius: f64,
    normals: Vec<Vector3>,
    vertices: Vec<Vector3>,
}

impl RoundedCornerBox {
    /// The box as a cleaned mesh.
    pub fn create(segments: i32, size_xyz: Vector3, radius: f64) -> Mesh {
        let round_box = RoundedCornerBox::new(segments, size_xyz, radius);
        let mut mesh = Mesh::from_vector3_list_and_indices(&round_box.vertices, &round_box.indices);
        mesh.clean_and_merge();
        mesh
    }

    /// The box's vertex and index lists for `segments` segments per rounded edge.
    pub fn new(segments: i32, size_xyz: Vector3, radius: f64) -> Self {
        let mut round_box = RoundedCornerBox {
            indices: Vec::new(),
            index_to_verts: HashMap::new(),
            edge_count: 0,
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

    fn add_quad(&mut self, corners: [Vector3; 4]) {
        let first = self.vertices.len() as i32;
        self.vertices.extend_from_slice(&corners);
        self.indices
            .extend_from_slice(&[first, first + 1, first + 2, first, first + 2, first + 3]);
    }

    fn add_tri(&mut self, corners: [Vector3; 3]) {
        let first = self.vertices.len() as i32;
        self.vertices.extend_from_slice(&corners);
        self.indices
            .extend_from_slice(&[first, first + 1, first + 2]);
    }

    fn build_single_segment_box(&mut self, size_xyz: Vector3, radius: f64) {
        self.vertices = Vec::new();
        self.indices = Vec::new();

        let min_size = size_xyz.x.min(size_xyz.y.min(size_xyz.z));
        let r = radius.min(min_size / 2.0);

        let xr = size_xyz.x / 2.0;
        let yb = size_xyz.y / 2.0;
        let zt = size_xyz.z / 2.0;
        let v = Vector3::new;

        // 1) main faces, inset by the radius: top (wound ccw), bottom, left, right, front, back
        self.add_quad([
            v(xr - r, yb - r, zt),
            v(-xr + r, yb - r, zt),
            v(-xr + r, -yb + r, zt),
            v(xr - r, -yb + r, zt),
        ]);
        self.add_quad([
            v(xr - r, yb - r, -zt),
            v(xr - r, -yb + r, -zt),
            v(-xr + r, -yb + r, -zt),
            v(-xr + r, yb - r, -zt),
        ]);
        self.add_quad([
            v(-xr, yb - r, -zt + r),
            v(-xr, -yb + r, -zt + r),
            v(-xr, -yb + r, zt - r),
            v(-xr, yb - r, zt - r),
        ]);
        self.add_quad([
            v(xr, -yb + r, -zt + r),
            v(xr, yb - r, -zt + r),
            v(xr, yb - r, zt - r),
            v(xr, -yb + r, zt - r),
        ]);
        self.add_quad([
            v(-xr + r, -yb, -zt + r),
            v(xr - r, -yb, -zt + r),
            v(xr - r, -yb, zt - r),
            v(-xr + r, -yb, zt - r),
        ]);
        self.add_quad([
            v(-xr + r, yb, -zt + r),
            v(-xr + r, yb, zt - r),
            v(xr - r, yb, zt - r),
            v(xr - r, yb, -zt + r),
        ]);

        // 2) edge bridging quads: top front, left front, right front, bottom front, left top,
        // left bottom, right top, right bottom, back top, back bottom, back left, back right
        self.add_quad([
            v(xr - r, -yb, zt - r),
            v(xr - r, -yb + r, zt),
            v(-xr + r, -yb + r, zt),
            v(-xr + r, -yb, zt - r),
        ]);
        self.add_quad([
            v(-xr, -yb + r, zt - r),
            v(-xr, -yb + r, -zt + r),
            v(-xr + r, -yb, -zt + r),
            v(-xr + r, -yb, zt - r),
        ]);
        self.add_quad([
            v(xr, -yb + r, zt - r),
            v(xr - r, -yb, zt - r),
            v(xr - r, -yb, -zt + r),
            v(xr, -yb + r, -zt + r),
        ]);
        self.add_quad([
            v(xr - r, -yb, -zt + r),
            v(-xr + r, -yb, -zt + r),
            v(-xr + r, -yb + r, -zt),
            v(xr - r, -yb + r, -zt),
        ]);
        self.add_quad([
            v(-xr, yb - r, zt - r),
            v(-xr, -yb + r, zt - r),
            v(-xr + r, -yb + r, zt),
            v(-xr + r, yb - r, zt),
        ]);
        self.add_quad([
            v(-xr + r, yb - r, -zt),
            v(-xr + r, -yb + r, -zt),
            v(-xr, -yb + r, -zt + r),
            v(-xr, yb - r, -zt + r),
        ]);
        self.add_quad([
            v(xr - r, yb - r, zt),
            v(xr - r, -yb + r, zt),
            v(xr, -yb + r, zt - r),
            v(xr, yb - r, zt - r),
        ]);
        self.add_quad([
            v(xr, yb - r, -zt + r),
            v(xr, -yb + r, -zt + r),
            v(xr - r, -yb + r, -zt),
            v(xr - r, yb - r, -zt),
        ]);
        self.add_quad([
            v(-xr + r, yb, zt - r),
            v(-xr + r, yb - r, zt),
            v(xr - r, yb - r, zt),
            v(xr - r, yb, zt - r),
        ]);
        self.add_quad([
            v(-xr + r, yb - r, -zt),
            v(-xr + r, yb, -zt + r),
            v(xr - r, yb, -zt + r),
            v(xr - r, yb - r, -zt),
        ]);
        self.add_quad([
            v(-xr, yb - r, zt - r),
            v(-xr + r, yb, zt - r),
            v(-xr + r, yb, -zt + r),
            v(-xr, yb - r, -zt + r),
        ]);
        self.add_quad([
            v(xr, yb - r, zt - r),
            v(xr, yb - r, -zt + r),
            v(xr - r, yb, -zt + r),
            v(xr - r, yb, zt - r),
        ]);

        // 3) corners: front top right, front bottom right, front top left, front bottom left,
        // back top right, back bottom right, back top left, back bottom left
        self.add_tri([
            v(xr - r, -yb + r, zt),
            v(xr - r, -yb, zt - r),
            v(xr, -yb + r, zt - r),
        ]);
        self.add_tri([
            v(xr - r, -yb + r, -zt),
            v(xr, -yb + r, -zt + r),
            v(xr - r, -yb, -zt + r),
        ]);
        self.add_tri([
            v(-xr + r, -yb + r, zt),
            v(-xr, -yb + r, zt - r),
            v(-xr + r, -yb, zt - r),
        ]);
        self.add_tri([
            v(-xr, -yb + r, -zt + r),
            v(-xr + r, -yb + r, -zt),
            v(-xr + r, -yb, -zt + r),
        ]);
        self.add_tri([
            v(xr - r, yb, zt - r),
            v(xr - r, yb - r, zt),
            v(xr, yb - r, zt - r),
        ]);
        self.add_tri([
            v(xr, yb - r, -zt + r),
            v(xr - r, yb - r, -zt),
            v(xr - r, yb, -zt + r),
        ]);
        self.add_tri([
            v(-xr + r, yb - r, zt),
            v(-xr + r, yb, zt - r),
            v(-xr, yb - r, zt - r),
        ]);
        self.add_tri([
            v(-xr + r, yb - r, -zt),
            v(-xr, yb - r, -zt + r),
            v(-xr + r, yb, -zt + r),
        ]);
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

    fn add_face(&mut self, i: i32, j: i32, k: i32, inversed: bool) {
        if inversed {
            self.indices.extend_from_slice(&[i, k, j]);
        } else {
            self.indices.extend_from_slice(&[i, j, k]);
        }
    }

    fn grid_index(&self, [i, j, k]: [i32; 3]) -> i32 {
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
