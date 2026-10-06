//! Laying a run's OFFSET sides along the surface (`Sections/SectionBuilderWalkedSide.cs`:
//! `LayTheOffsetsAlongTheSurface`): every station's offset boundaries walked over the faces they
//! reach past, and a split blend's join stations laid from the join's own point where it has
//! one. Split from `section_builder_walked_side.rs` by responsibility.

use indexmap::IndexMap;
use polygon_mesh::{Mesh, MeshEdgeGraph};

use super::section_builder_walked_side::{
    rewritten, the_joins_own_point, try_lay_one_side, SectionBuildRefusal,
};
use super::section_fan_walk::EdgeAtPair;
use super::section_split_blend::SplitPlacement;
use super::{section_surface_walk, BevelSection, JunctionSection, SectionPool};
use crate::topology::ChainEdge;

/// C# `SectionBuilder.LayTheOffsetsAlongTheSurface`: each interior (or, closed, every) station's
/// two sides laid along the surface; then each split station's sides laid at the join's own
/// point, or from its own station where it has none and may be (an open run, or beside a
/// junction). Nothing to lay without a feature-edge test.
#[allow(clippy::too_many_arguments)]
pub fn lay_the_offsets_along_the_surface(
    sections: &mut [BevelSection],
    placed: Option<&[SplitPlacement]>,
    edges: &[ChainEdge],
    is_closed: bool,
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    pool: &mut SectionPool,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
    junctions: Option<&IndexMap<i32, JunctionSection>>,
) -> Result<(), SectionBuildRefusal> {
    let Some(is_a_feature) = is_a_feature_edge else {
        return Ok(());
    };

    let first = if is_closed { 0 } else { 1 };
    let past = if is_closed {
        sections.len()
    } else {
        sections.len().saturating_sub(1)
    };
    let edge_count = edges.len();

    for i in first..past {
        if placed.is_some_and(|placed| i < placed.len() && placed[i].is_split()) {
            continue;
        }

        let leaving = &edges[i];
        let arriving = &edges[(i + edge_count - 1) % edge_count];
        let left = try_lay_one_side(
            &sections[i],
            arriving,
            leaving,
            true,
            mesh,
            graph,
            edge_at_pair,
            pool,
            Some(is_a_feature),
        )?;
        let right = try_lay_one_side(
            &sections[i],
            arriving,
            leaving,
            false,
            mesh,
            graph,
            edge_at_pair,
            pool,
            Some(is_a_feature),
        )?;
        sections[i] = rewritten(&sections[i], &left, &right);
    }

    let Some(placed) = placed else {
        return Ok(());
    };

    let lattice = section_surface_walk::grid_of(mesh);
    for i in first..past.min(placed.len()) {
        if !placed[i].is_split() {
            continue;
        }

        let mut left = the_joins_own_point(
            sections,
            junctions,
            placed[i],
            i as i32,
            true,
            mesh,
            graph,
            edge_at_pair,
            pool,
            Some(is_a_feature),
            lattice,
        )?;
        let mut right = the_joins_own_point(
            sections,
            junctions,
            placed[i],
            i as i32,
            false,
            mesh,
            graph,
            edge_at_pair,
            pool,
            Some(is_a_feature),
            lattice,
        )?;
        let beside_a_junction = junctions.is_some_and(|junctions| {
            junctions.contains_key(&placed[i].before) || junctions.contains_key(&placed[i].after)
        });
        let may_be_laid_from_its_own_station = !is_closed || beside_a_junction;
        let arriving = &edges[(i + edge_count - 1) % edge_count];

        if may_be_laid_from_its_own_station && left.face < 0 {
            left = try_lay_one_side(
                &sections[i],
                arriving,
                &edges[i],
                true,
                mesh,
                graph,
                edge_at_pair,
                pool,
                Some(is_a_feature),
            )?;
        }

        if may_be_laid_from_its_own_station && right.face < 0 {
            right = try_lay_one_side(
                &sections[i],
                arriving,
                &edges[i],
                false,
                mesh,
                graph,
                edge_at_pair,
                pool,
                Some(is_a_feature),
            )?;
        }

        sections[i] = rewritten(&sections[i], &left, &right);
    }

    Ok(())
}
