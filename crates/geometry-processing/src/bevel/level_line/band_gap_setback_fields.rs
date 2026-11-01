//! Step 1 of the band gap's build (`band_gap_build_within.rs`), the SETBACK: each chain's edges,
//! its setbacks per unit of size from the angle at each edge (`chain_on_the_mesh::unit_setbacks`),
//! its end rays and their bounds, and from those the chain fields at any size. Held per unit of
//! size so the room can ask the same question at any size.
//!
//! At the size itself a pass the rolling ball takes (`rolling_ball_setbacks.rs`) has a field per
//! side per chain from that side's contacts ([`PassSetbacks::fields_at_the_size`]), and the
//! chain's own field holds the larger of the two sides at each point; every other pass has the
//! chain's own field from the angle and no side fields, exactly as the angle gives it.

use std::collections::HashSet;
use std::rc::Rc;

use vector_math::Vector3;

use super::band_gap_build_fault::BuildFault;
use super::chain_field_from_polyline::ChainFieldArgumentError;
use super::rolling_ball_setbacks::SideSetbacks;
use super::{
    chain_on_the_mesh, end_face_sense, pick_validation, square_crease_end_clip, BandProfile,
    ChainField, EndRayBound,
};

/// A chain's left and right fields at the size, as `band_group_fields::of_sides` reads them.
pub type SideFields = Vec<[Rc<ChainField>; 2]>;

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
    /// The pick's step 1 readings.
    pub fn of(
        positions: &[Vector3],
        indices: &[(i32, i32, i32)],
        chains: &[Vec<i32>],
        closed: &[bool],
        sharp_turn_degrees: f64,
        profile: BandProfile,
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

        // An end on an OBLIQUE end face carries the end ray; an end at a CORNER is not an end
        // face.
        let corner_ends: HashSet<i32> = if count > 1 {
            pick_validation::corners_of(chains, closed, true)
                .iter()
                .map(|c| c.vertex)
                .collect()
        } else {
            HashSet::new()
        };
        let mut end_rays: Vec<(bool, bool)> = Vec::with_capacity(count);
        let mut end_ray_bounds = Vec::with_capacity(count);
        for k in 0..count {
            let rays = end_face_sense::oblique_ends(
                positions,
                indices,
                &chains[k],
                closed[k],
                sharp_turn_degrees,
            );
            let rays = (
                rays.0 && !corner_ends.contains(&chains[k][0]),
                rays.1 && !corner_ends.contains(&chains[k][chains[k].len() - 1]),
            );
            end_rays.push(rays);
            end_ray_bounds.push(end_face_sense::end_ray_bounds(
                positions, indices, &chains[k], rays,
            ));
        }

        // A square end on a concave crease whose side runs on past it is clipped in the end face's
        // plane.
        let square_crease_clips = square_crease_end_clip::square_crease_end_clips_of_pick(
            positions, indices, chains, closed, &end_rays,
        );
        for k in 0..count {
            end_ray_bounds[k] = (
                end_ray_bounds[k].0.or(square_crease_clips[k].0),
                end_ray_bounds[k].1.or(square_crease_clips[k].1),
            );
        }

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

    /// Every chain's own field at `size`, and with the rolling ball's setbacks (`ball`) each
    /// chain's left and right fields: the own field from the angle where `ball` is `None`, the
    /// larger of the ball's two sides where it is given.
    pub fn fields_at_the_size(
        &self,
        ball: Option<&[SideSetbacks]>,
        size: f64,
    ) -> Result<(Vec<Rc<ChainField>>, Option<SideFields>), ChainFieldArgumentError> {
        let count = self.chain_points.len();
        let mut fields: Vec<Rc<ChainField>> = Vec::with_capacity(count);
        let Some(ball) = ball else {
            for k in 0..count {
                fields.push(Rc::new(self.field_of(k, size)?));
            }
            return Ok((fields, None));
        };

        let mut sides = Vec::with_capacity(count);
        for (k, chain) in ball.iter().enumerate() {
            fields.push(Rc::new(self.field_from(k, &chain.larger(), size)?));
            sides.push([
                Rc::new(self.field_from(k, &chain.left, size)?),
                Rc::new(self.field_from(k, &chain.right, size)?),
            ]);
        }
        Ok((fields, Some(sides)))
    }
}
