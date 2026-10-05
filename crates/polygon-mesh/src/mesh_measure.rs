//! Whole-mesh measurements: enclosed volume and total surface area, in the mesh's own units.
//! Ports agg-sharp `PolygonMesh/MeshMeasure.cs` as `Mesh` methods.

use vector_math::{Matrix4X4, Vector3};

use crate::mesh::Mesh;

impl Mesh {
    /// The volume a closed, consistently wound mesh encloses, by the divergence theorem: the
    /// sum over every triangle of the signed volume of the tetrahedron it makes with the
    /// origin, a . (b x c) / 6.
    ///
    /// The sign follows the winding: outward-facing (counter-clockwise seen from outside, as
    /// the face normal is computed) gives a positive volume, an inside-out mesh the same volume
    /// negated. Several closed shells add, so a shell wound inward subtracts (a cavity). On an
    /// open mesh or one whose winding is inconsistent the sum is still returned but it is not a
    /// volume: it changes as the mesh moves relative to the origin. Sums in f64 from the f32
    /// vertices.
    pub fn get_volume(&self) -> f64 {
        self.get_volume_transformed(Matrix4X4::IDENTITY)
    }

    /// [`Mesh::get_volume`] of the mesh after `matrix` is applied to every vertex, without
    /// copying the mesh. The vertices are transformed rather than multiplying by the
    /// determinant so the matrix never has to be affine for the result to be right.
    pub fn get_volume_transformed(&self, matrix: Matrix4X4) -> f64 {
        let mut six_times_volume = 0.0;
        for face in self.faces.iter() {
            let (a, b, c) = self.transformed_corners(face.v0, face.v1, face.v2, matrix);
            six_times_volume += a.dot(b.cross(c));
        }
        six_times_volume / 6.0
    }

    /// The total area of every triangle in the mesh, |(b - a) x (c - a)| / 2 each. Winding and
    /// closure do not matter; zero-area faces add nothing.
    pub fn get_surface_area(&self) -> f64 {
        self.get_surface_area_transformed(Matrix4X4::IDENTITY)
    }

    /// [`Mesh::get_surface_area`] of the mesh after `matrix` is applied to every vertex. Area
    /// has no single scale factor under a non-uniform scale, so the vertices are transformed.
    pub fn get_surface_area_transformed(&self, matrix: Matrix4X4) -> f64 {
        let mut twice_area = 0.0;
        for face in self.faces.iter() {
            let (a, b, c) = self.transformed_corners(face.v0, face.v1, face.v2, matrix);
            twice_area += (b - a).cross(c - a).length();
        }
        twice_area / 2.0
    }

    fn transformed_corners(
        &self,
        v0: i32,
        v1: i32,
        v2: i32,
        matrix: Matrix4X4,
    ) -> (Vector3, Vector3, Vector3) {
        let corner =
            |v: i32| Vector3::from_vector3_float(self.vertices[v as usize]).transform(matrix);
        (corner(v0), corner(v1), corner(v2))
    }
}
