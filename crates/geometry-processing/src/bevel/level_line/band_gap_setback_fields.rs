//! Step 1 of the band gap's build (`band_gap_build_within.rs`), the SETBACK: each chain's edges,
//! its setbacks per unit of size from the angle at each edge (`chain_on_the_mesh::unit_setbacks`),
//! its end rays and their bounds (with the square crease and stopped ends' clips, read in
//! `pick_end_bounds.rs`), and from those
//! the chain fields at any size. Held per unit of size so the room can ask the same question at
//! any size.

use std::rc::Rc;

use vector_math::Vector3;

use super::band_gap_build_fault::BuildFault;
use super::chain_field_from_polyline::ChainFieldArgumentError;
use super::pick_end_bounds::PickEnds;
use super::{chain_on_the_mesh, BandProfile, ChainField, EndRayBound};

/// Every chain's step 1 readings, per unit of size.
pub struct PassSetbacks {
    /// Each chain's edges, in chain order.
    pub chain_edges: Vec<Vec<(i32, i32)>>,
    /// Each chain's setbacks per unit of size from the angle, one per chain vertex.
    pub unit_setbacks: Vec<Vec<f64>>,
    /// Each chain's vertex positions.
    pub chain_points: Vec<Vec<Vector3>>,
    /// Whether each chain's first and last ends carry an end ray.
    pub end_rays: Vec<(bool, bool)>,
    /// Each chain's end rays' bounds, or the plane a square crease end is clipped in.
    pub end_ray_bounds: Vec<(Option<EndRayBound>, Option<EndRayBound>)>,
    closed: Vec<bool>,
    sharp_turn_degrees: f64,
}

impl PassSetbacks {
    /// The turn, in degrees, past which a chain vertex of this pass's fields is mitred.
    pub fn sharp_turn_degrees(&self) -> f64 {
        self.sharp_turn_degrees
    }

    /// The pick's step 1 readings.
    pub fn of(
        positions: &[Vector3],
        indices: &[(i32, i32, i32)],
        chains: &[Vec<i32>],
        closed: &[bool],
        sharp_turn_degrees: f64,
        profile: BandProfile,
        is_feature_edge: &dyn Fn(i32, i32) -> bool,
    ) -> Result<PassSetbacks, BuildFault> {
        let count = chains.len();
        let mut chain_edges: Vec<Vec<(i32, i32)>> = Vec::with_capacity(count);
        let mut unit_setbacks: Vec<Vec<f64>> = Vec::with_capacity(count);
        let mut chain_points: Vec<Vec<Vector3>> = Vec::with_capacity(count);
        for k in 0..count {
            let edges = chain_on_the_mesh::chain_edges(&chains[k], closed[k]);
            unit_setbacks.push(chain_on_the_mesh::unit_setbacks(
                positions, indices, &chains[k], closed[k], &edges, profile,
            )?);
            chain_edges.push(edges);
            chain_points.push(chains[k].iter().map(|&v| positions[v as usize]).collect());
        }

        // The ends, as every reading of the pick takes them: rays, their bounds, square crease
        // and stopped ends' clips.
        let ends = PickEnds::of(
            positions,
            indices,
            chains,
            closed,
            sharp_turn_degrees,
            is_feature_edge,
        );
        let end_rays = ends.end_rays;
        let end_ray_bounds = ends.bounds;

        Ok(PassSetbacks {
            chain_edges,
            unit_setbacks,
            chain_points,
            end_rays,
            end_ray_bounds,
            closed: closed.to_vec(),
            sharp_turn_degrees,
        })
    }

    /// Chain `k`'s field with setbacks `units` per unit of size, at size `s`.
    pub fn field_from(
        &self,
        k: usize,
        units: &[f64],
        s: f64,
    ) -> Result<ChainField, ChainFieldArgumentError> {
        let setbacks: Vec<f64> = units.iter().map(|u| u * s).collect();
        ChainField::from_polyline_bounded(
            &self.chain_points[k],
            self.closed[k],
            &setbacks,
            self.sharp_turn_degrees,
            self.end_rays[k],
            self.end_ray_bounds[k],
        )
    }

    /// Chain `k`'s field from the angle at size `s`.
    pub fn field_of(&self, k: usize, s: f64) -> Result<ChainField, ChainFieldArgumentError> {
        self.field_from(k, &self.unit_setbacks[k], s)
    }

    /// Every chain's own field at `size`, from the angle.
    pub fn fields_at_the_size(
        &self,
        size: f64,
    ) -> Result<Vec<Rc<ChainField>>, ChainFieldArgumentError> {
        (0..self.chain_points.len())
            .map(|k| self.field_of(k, size).map(Rc::new))
            .collect()
    }
}
