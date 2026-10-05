//! Filing a pick's foreign creases into [`ForeignEdgeRegions`] (`LevelLine/ForeignEdgeRegions.cs`:
//! `Over`, `PerChain` and both `ForChain`s): every crease between two different tags, grouped by
//! the tag pair, each growing crease of a one-chain pick replaced by the far creases bounding its
//! run, and on a pick of several open chains each chain's own growth read. Split from
//! `foreign_edge_regions.rs` by responsibility.

use std::cell::RefCell;
use std::collections::HashSet;
use std::rc::Rc;

use indexmap::{IndexMap, IndexSet};

use super::band_side_tagging::{FIRST_END_TAG, SECOND_END_TAG};
use super::foreign_edge_regions::{chain_ends, distance_to_chain, open_chain_of};
use super::{
    end_face_sense, foreign_edge_room, triangle_edges, ForeignEdgeIndex, ForeignEdgeRegions,
    ForeignEdgeRegionsChainGrowth, GrowingCreaseRuns, LevelLineMesh, WorkError,
};

/// C# `Func<int, int[]>` naming each chain's end tags per triangle.
pub type EndTagsOf<'a> = &'a dyn Fn(i32) -> Vec<i32>;

/// C# `Func<int, int, bool>`: whether a tag is chain `k`'s own side.
pub type Seeds<'a> = &'a dyn Fn(i32, i32) -> bool;

impl ForeignEdgeRegions {
    /// C# `ForeignEdgeRegions.Over`. `chain_keys` keeps C#'s `HashSet` insertion order (its
    /// `First()` names the one-chain growth's edge where no open chain is given).
    pub fn over(
        mesh: Rc<RefCell<LevelLineMesh>>,
        chain_keys: &IndexSet<(i32, i32)>,
        chain_edges: Option<&[Vec<(i32, i32)>]>,
        closed: Option<&[bool]>,
        end_tags_of: Option<EndTagsOf>,
        seeds: Option<Seeds>,
    ) -> Result<ForeignEdgeRegions, WorkError> {
        let keys: HashSet<(i32, i32)> = chain_keys.iter().copied().collect();
        let mut by_tag_pair: IndexMap<(i32, i32), Vec<(i32, i32)>> = IndexMap::new();
        let mut growing: Option<Rc<GrowingCreaseRuns>> = None;
        let ends = chain_ends(&keys);
        let several_open = match (chain_edges, closed) {
            (Some(edges), Some(closed)) => {
                (0..edges.len())
                    .filter(|&k| !closed[k] && !edges[k].is_empty())
                    .count()
                    > 1
            }
            _ => false,
        };
        let mut per_chain = match (chain_edges, closed) {
            (Some(edges), Some(closed)) if ends.len() > 2 || several_open => Some(Self::per_chain(
                &mesh,
                edges,
                closed,
                end_tags_of.expect("a pick of several chains names its end tags"),
                seeds,
            )?),
            _ => None,
        };

        let edges = mesh.borrow().edges();
        for edge in edges {
            if keys.contains(&edge) {
                continue;
            }

            let (one, other) = {
                let m = mesh.borrow();
                let standing = m.triangles_on(edge.0, edge.1);
                if standing.len() != 2 {
                    continue;
                }
                (
                    m.triangles[standing[0] as usize].tag,
                    m.triangles[standing[1] as usize].tag,
                )
            };
            if one == other {
                continue;
            }

            if (one == 0 || other == 0) && foreign_edge_room::is_a_knife_edge(&mesh.borrow(), edge)
            {
                continue;
            }

            let mut bounding = vec![edge];
            if (one == 0 || other == 0) && ends.len() == 2 && per_chain.is_none() {
                let runs = growing.get_or_insert_with(|| {
                    let distance_mesh = mesh.clone();
                    let distance_keys = keys.clone();
                    let distance = Rc::new(move |p| {
                        distance_to_chain(&distance_mesh.borrow(), &distance_keys, p)
                    });
                    Rc::new(match open_chain_of(chain_edges, closed) {
                        Some(open) => GrowingCreaseRuns::new(
                            mesh.clone(),
                            open[0],
                            distance,
                            Some(open.clone()),
                            None,
                        ),
                        None => GrowingCreaseRuns::new(
                            mesh.clone(),
                            *chain_keys.first().expect("a pick has chain edges"),
                            distance,
                            None,
                            None,
                        ),
                    })
                });
                if let Some(bounds) = runs.bounds_of(edge, &ends)? {
                    bounding = bounds;
                }
            }

            if let Some(per_chain) = per_chain.as_mut() {
                for (k, growth) in per_chain.iter_mut().enumerate() {
                    let k = k as i32;
                    if one == 0
                        || other == 0
                        || seeds.is_some_and(|seeds| seeds(one, k) != seeds(other, k))
                    {
                        if let Some(growth) = growth.as_mut() {
                            growth.read(edge)?;
                        }
                    }
                }
            }

            let pair = if one < other {
                (one, other)
            } else {
                (other, one)
            };
            by_tag_pair.entry(pair).or_default().extend(bounding);
        }

        let mut regions = ForeignEdgeRegions::new(mesh, by_tag_pair, per_chain);
        regions.growing = growing;
        Ok(regions)
    }

    /// C# `ForeignEdgeRegions.PerChain`: each open chain's growth - its runs, its two ends, the
    /// end face's stretch at each end that has no end face of its own, and its end regions read.
    pub fn per_chain(
        mesh: &Rc<RefCell<LevelLineMesh>>,
        chain_edges: &[Vec<(i32, i32)>],
        closed: &[bool],
        end_tags_of: EndTagsOf,
        seeds: Option<Seeds>,
    ) -> Result<Vec<Option<ForeignEdgeRegionsChainGrowth>>, WorkError> {
        let mut per_chain = Vec::with_capacity(chain_edges.len());
        for (k, edges) in chain_edges.iter().enumerate() {
            if closed[k] || edges.is_empty() {
                per_chain.push(None);
                continue;
            }

            let chain = k as i32;
            let end_tags = end_tags_of(chain);
            let first_has_a_face = end_tags.contains(&FIRST_END_TAG);
            let last_has_a_face = end_tags.contains(&SECOND_END_TAG);
            let keys: HashSet<(i32, i32)> = edges
                .iter()
                .map(|&(a, b)| triangle_edges::key(a, b))
                .collect();
            let seeds_of = seeds.expect("a pick of several chains names its seeds");
            let own_side = move |tag: i32| seeds_of(tag, chain);

            let distance_mesh = mesh.clone();
            let distance_keys = keys.clone();
            // The runs keep their own-side test past this call, so it is read here into the
            // tags the pick's triangles carry.
            let own_tags: HashSet<i32> = mesh
                .borrow()
                .triangles
                .iter()
                .map(|t| t.tag)
                .filter(|&tag| own_side(tag))
                .collect();
            let own_for_runs: Option<super::IsOwnSide> =
                seeds.map(|_| Rc::new(move |tag: i32| own_tags.contains(&tag)) as super::IsOwnSide);
            let runs = Rc::new(GrowingCreaseRuns::new(
                mesh.clone(),
                edges[0],
                Rc::new(move |p| distance_to_chain(&distance_mesh.borrow(), &distance_keys, p)),
                Some(edges.clone()),
                own_for_runs,
            ));
            let stretch = end_face_sense::growing_crease_stretch(
                &mesh.borrow(),
                edges,
                !first_has_a_face,
                !last_has_a_face,
                Some(&own_side),
            );
            let mut growth = ForeignEdgeRegionsChainGrowth::new(
                runs,
                HashSet::from([edges[0].0, edges[edges.len() - 1].1]),
                stretch,
                !first_has_a_face,
                !last_has_a_face,
            );
            growth.read_end_regions(&mesh.borrow(), &end_tags, &keys, &own_side);
            per_chain.push(Some(growth));
        }

        Ok(per_chain)
    }

    /// C# `ForChain(seeds)`: [`ForeignEdgeRegions::for_chain_k`] with no chain of its own.
    pub fn for_chain(&self, seeds: &dyn Fn(i32) -> bool) -> Vec<Rc<ForeignEdgeIndex>> {
        self.for_chain_k(-1, seeds)
    }

    /// C# `ForChain(chain, seeds)`: the indexes of every crease that bounds this pick - the
    /// chain's own growth's bounds first (indexed once), then every tag pair the seeds tell apart,
    /// each indexed once and kept.
    pub fn for_chain_k(
        &self,
        chain: i32,
        seeds: &dyn Fn(i32) -> bool,
    ) -> Vec<Rc<ForeignEdgeIndex>> {
        let mut found = Vec::new();
        let growth = if chain >= 0 {
            self.per_chain
                .as_ref()
                .and_then(|per_chain| per_chain.get(chain as usize))
                .and_then(Option::as_ref)
        } else {
            None
        };

        if let Some(growth) = growth {
            if !growth.bounding.is_empty() {
                let index = growth
                    .index
                    .borrow_mut()
                    .get_or_insert_with(|| {
                        Rc::new(ForeignEdgeIndex::over(
                            &self.mesh.borrow(),
                            &growth.bounding,
                        ))
                    })
                    .clone();
                found.push(index);
            }
        }

        for (pair, edges) in &self.by_tag_pair {
            if seeds(pair.0) == seeds(pair.1) {
                continue;
            }

            let index = self
                .indexed
                .borrow_mut()
                .entry(*pair)
                .or_insert_with(|| Rc::new(ForeignEdgeIndex::over(&self.mesh.borrow(), edges)))
                .clone();
            found.push(index);
        }

        found
    }
}
