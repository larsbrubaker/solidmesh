//! The face-width check's wall reading: a walk stopped by an unselected crease whose far face is
//! the very wall the run's own free end caps on is no lack of face - the end cap clips the band
//! there anyway. Split from `bevel_face_fit.rs` by responsibility.

use vector_math::Vector3;

use super::bevel_face_fit::same_cap_plane_agreement;
use super::{
    bevel_chain_linker, bevel_corner_seam, BevelChainEnds, BevelFaceFit, BevelFaceFitEndCaps,
};
use crate::topology::FeatureEdge;

impl BevelFaceFit {
    /// Whether the walk on `face` from `edge` stopped at an unselected manifold crease whose OTHER
    /// face (the one off the walked face's plane) is a wall the run (group `group_index` of
    /// `resolutions`) ends on at either end of `edge`.
    pub fn runs_out_onto_a_wall_the_run_ends_on(
        &self,
        group_index: usize,
        face: i32,
        edge: &FeatureEdge,
        blocking_edge: i32,
    ) -> bool {
        if blocking_edge < 0
            || self.setbacks.contains_key(&blocking_edge)
            || self.graph.get_face_count(blocking_edge) != 2
        {
            return false;
        }

        let Some(walked_normal) = self.walk.try_get_face_normal(face) else {
            return false;
        };

        let walked_plane = walked_normal.dot(edge.start);
        let first_face = self.graph.get_face(blocking_edge, 0);
        let second_face = self.graph.get_face(blocking_edge, 1);
        let first_is_the_face = self.lies_in_the_plane(first_face, walked_normal, walked_plane);
        if first_is_the_face == self.lies_in_the_plane(second_face, walked_normal, walked_plane) {
            return false;
        }

        let wall = if first_is_the_face {
            second_face
        } else {
            first_face
        };
        let Some(normal) = self.walk.try_get_face_normal(wall) else {
            return false;
        };

        let through_the_wall = normal.dot(Vector3::from_vector3_float(
            self.mesh.vertices[self.mesh.faces[wall as usize].v0 as usize],
        ));
        self.the_run_ends_on_that_wall(
            group_index,
            edge.start_vertex,
            edge.start,
            normal,
            through_the_wall,
            edge.mesh_edge_index,
        ) || self.the_run_ends_on_that_wall(
            group_index,
            edge.end_vertex,
            edge.end,
            normal,
            through_the_wall,
            edge.mesh_edge_index,
        )
    }

    /// Whether `vertex` (on the wall's plane) is a free end of the run, with only its own selected
    /// edge there, whose resolved cap lies along that wall and stands no further off it than a
    /// bead's stand-off. The group's free-end caps are resolved once and kept.
    pub fn the_run_ends_on_that_wall(
        &self,
        group_index: usize,
        vertex: i32,
        position: Vector3,
        normal: Vector3,
        plane_distance: f64,
        own_edge: i32,
    ) -> bool {
        if (normal.dot(position) - plane_distance).abs() > self.plane_tolerance {
            return false;
        }

        match self.selected_edges_at_vertex.get(&vertex) {
            Some(here) if here.len() == 1 && here.contains(&own_edge) => {}
            _ => return false,
        }

        let mut free_end_caps = self.free_end_caps.borrow_mut();
        let caps = free_end_caps.entry(group_index).or_insert_with(|| {
            let resolution = &self.resolutions[group_index];
            let mut caps = BevelFaceFitEndCaps::new();
            for linked in bevel_chain_linker::link_resolution(resolution) {
                for run in bevel_chain_linker::split_by_convexity(linked) {
                    if run.is_closed {
                        continue;
                    }

                    let ends = BevelChainEnds::resolve(
                        &run.edges,
                        run.is_closed,
                        self.size,
                        Some(&resolution.features),
                        Some(&resolution.edges),
                        None,
                        super::bevel_chain_end_cap::ResolveOptions {
                            round_profile: self.round_profile,
                            ..Default::default()
                        },
                    );
                    let first = &run.edges[0];
                    let last = &run.edges[run.edges.len() - 1];
                    caps.insert(
                        (first.from_vertex(), first.edge.mesh_edge_index),
                        ends.start,
                    );
                    caps.insert((last.to_vertex(), last.edge.mesh_edge_index), ends.end);
                }
            }
            caps
        });

        let Some(cap) = caps.get(&(vertex, own_edge)) else {
            return false;
        };
        if !cap.is_capped() {
            return false;
        }

        let oriented_normal = if normal.dot(cap.plane_normal) < 0.0 {
            -normal
        } else {
            normal
        };
        let displacement = cap.plane_distance - cap.plane_normal.dot(position);
        oriented_normal.dot(cap.plane_normal) >= same_cap_plane_agreement()
            && displacement >= -self.plane_tolerance
            && displacement <= bevel_corner_seam::stand_off(false, position) + self.plane_tolerance
    }
}
