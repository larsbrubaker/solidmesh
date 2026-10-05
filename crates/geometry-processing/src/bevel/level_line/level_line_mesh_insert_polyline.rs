//! The polyline insertion half of the working mesh. Ports
//! `Bevel/LevelLine/LevelLineMesh.InsertPolyline.cs` (a partial of `LevelLineMesh`): so far the
//! removal of the region a polyline cuts off ([`LevelLineMesh::remove_enclosed`]); the insertion
//! lands with later packages.

use std::collections::HashSet;

use super::LevelLineMesh;

impl LevelLineMesh {
    /// Removes the region triangles reached from those standing at `seed` without crossing an
    /// edge of `chain` - the part of the flat region the inserted polyline cuts off round the
    /// seed. Returns how many were removed. (The removal order does not reach the mesh: each
    /// removal only detaches its own entries.)
    pub fn remove_enclosed(
        &mut self,
        seed: i32,
        chain: &[i32],
        in_region: &dyn Fn(i32) -> bool,
    ) -> i32 {
        let mut wall = HashSet::new();
        for pair in chain.windows(2) {
            wall.insert(Self::key(pair[0], pair[1]));
        }

        let mut stack: Vec<i32> = (0..self.triangle_count())
            .filter(|&t| {
                let tri = self.triangles[t as usize];
                !self.removed[t as usize]
                    && in_region(t)
                    && (tri.a == seed || tri.b == seed || tri.c == seed)
            })
            .collect();
        let mut reached = HashSet::new();
        while let Some(t) = stack.pop() {
            if !reached.insert(t) {
                continue;
            }

            let tri = self.triangles[t as usize];
            for (a, b) in [(tri.a, tri.b), (tri.b, tri.c), (tri.c, tri.a)] {
                if wall.contains(&Self::key(a, b)) {
                    continue;
                }

                for &other in self.triangles_on(a, b) {
                    if other != t
                        && !self.removed[other as usize]
                        && in_region(other)
                        && !reached.contains(&other)
                    {
                        stack.push(other);
                    }
                }
            }
        }

        let count = reached.len() as i32;
        for t in reached {
            self.remove_triangle(t);
        }

        count
    }
}
