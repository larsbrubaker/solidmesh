//! A square crease end's plane cut into the mesh as a KNOWN CUT: the end face lies in that plane
//! and a side may carry on past it (across a flat edge in it, or under it across a convex crease),
//! so the band's boundary there is the plane itself. Before the band is cut (and before the
//! needle repair, so it reads the traces), where known cuts are on (`use_flat_ends`), the plane's
//! trace is inserted where it crosses an edge at a point inside the chain's own band
//! ([`insert_traces`]); a crossing outside the band is left alone. Every triangle on such an edge
//! is split, so the mesh stays manifold with shared vertex indices. The band side is then read by
//! the chain's own value (`ChainField::value_across_the_known_cut`), and an edge or triangle
//! leaves the band across the cut only where its inside corners stand on ONE plane of ONE chain,
//! inside by that chain's own value, and its outside corners past that same plane
//! ([`ChainField::leaves_across_its_known_cut`]): such an edge has no crossing
//! (`edge_root::find_over`) and such a triangle is outside (`band_cut::side_of_triangle`). The
//! planes are read once per field ([`ChainField::known_cut_planes`]).

use vector_math::Vector3;

use super::corner_cap_flat_ends::use_flat_ends;
use super::{ChainField, LevelLineMesh, LevelLineMeshError};

impl ChainField {
    /// The planes of this chain's square crease ends (same sense, with a crease room), as
    /// (point on the plane, unit normal faced on past the end); read once per field.
    pub fn known_cut_planes(&self) -> &[(Vector3, Vector3)] {
        self.known_cuts.get_or_init(|| {
            let Some(end_clips) = self.end_clips.as_ref() else {
                return Vec::new();
            };
            end_clips
                .iter()
                .filter(|clip| {
                    let bound = if clip.parameter == 0.0 {
                        self.end_ray_bounds.0
                    } else {
                        self.end_ray_bounds.1
                    };
                    bound.is_some_and(|bound| bound.holds_its_end_face())
                })
                .map(|clip| (clip.vertex, clip.normal))
                .collect()
        })
    }

    /// Whether an edge or triangle with corners `insides` inside the band and `outsides` outside
    /// it leaves the band across ONE of this chain's known-cut planes, not across the level line:
    /// every inside corner stands on that plane, inside by this chain's OWN value (another pick
    /// member's band, or a run-out, reaching the plane is no cut of this chain's), and every
    /// outside corner stands past that same plane. Ends on one plane and past another (a short
    /// chain clipped at both ends) leave across the line.
    pub fn leaves_across_its_known_cut(
        &self,
        insides: &[Vector3],
        outsides: &[Vector3],
    ) -> Result<bool, super::WorkError> {
        for &(vertex, normal) in self.known_cut_planes() {
            let on = |p: Vector3| ChainField::on_a_stopped_plane(vertex, normal, 0.0, p);
            let past = |p: Vector3| !on(p) && (p - vertex).dot(normal) > 0.0;
            if !insides.iter().all(|&p| on(p)) || !outsides.iter().all(|&p| past(p)) {
                continue;
            }

            let mut own = true;
            for &p in insides {
                if self.unclipped_value_at(p)? > 0.0 {
                    own = false;
                    break;
                }
            }
            if own {
                return Ok(true);
            }
        }

        Ok(false)
    }
}

/// Inserts each chain's known-cut planes' trace into the mesh: every edge whose two ends stand
/// strictly on opposite sides of such a plane, crossing it at a point inside that chain's own band
/// (its unclipped value at most 0), is split there; a crossing outside the band is not inserted.
/// Nothing where known cuts are off (`use_flat_ends`). Edges are read in the mesh's canonical
/// order, so the vertices minted are the same on every run.
pub fn insert_traces(
    mesh: &mut LevelLineMesh,
    chains: &[std::rc::Rc<ChainField>],
) -> Result<(), LevelLineMeshError> {
    if !use_flat_ends() {
        return Ok(());
    }

    for chain in chains {
        for &(vertex, normal) in chain.known_cut_planes() {
            for (a, b) in mesh.edges() {
                let (pa, pb) = (mesh.position(a), mesh.position(b));
                let on = |p: Vector3| ChainField::on_a_stopped_plane(vertex, normal, 0.0, p);
                if on(pa) || on(pb) {
                    continue;
                }

                let (da, db) = ((pa - vertex).dot(normal), (pb - vertex).dot(normal));
                if (da > 0.0) == (db > 0.0) {
                    continue;
                }

                let point = Vector3::lerp(pa, pb, da / (da - db));
                if chain.unclipped_value_at(point)? > 0.0 {
                    continue;
                }

                mesh.split_edge(a, b, point, false)?;
            }
        }
    }

    Ok(())
}
