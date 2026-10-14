//! The corner fill's end-ray trim (the piece type is in `corner_fill.rs`): the convexity reading at a
//! run's end ([`is_convex_at`]), the trim of a pick's free rayed ends to their end planes
//! ([`trim_end_ray_ends`]) and of its grow-over ends ([`trim_grow_over_ends`]). The section each
//! lays is `end_plane_trim_run::run`'s.

use super::{GapLoop, LevelLineMesh};

/// Whether the chain a side run borders is convex at the run's end - the other
/// rail's end (`across`) stands behind the face kept on the run's last edge (the reading
/// `EndFaceFill.IsConcave` takes at a run's middle). A run of fewer than two vertices has no last
/// edge and reads as not convex.
pub fn is_convex_at(mesh: &LevelLineMesh, run: &GapLoop, across: i32) -> bool {
    let count = run.vertices.len();

    if count < 2 {
        return false;
    }

    let (a, b) = (run.vertices[count - 2], run.vertices[count - 1]);

    for &t in mesh.triangles_on(a, b) {
        if mesh.is_removed(t) {
            continue;
        }

        let face = mesh.triangles[t as usize];
        let a0 = mesh.position(face.a);
        let n = (mesh.position(face.b) - a0).cross(mesh.position(face.c) - a0);

        if n.length() > 0.0 {
            return n.dot(mesh.position(across) - mesh.position(b)) <= 0.0;
        }
    }

    false
}

/// The sections a trim laid, by the (from, to) of the boundary stretch each replaces.
pub type TrimSections = indexmap::IndexMap<(i32, i32), Vec<i32>>;

/// Where the corner fill's connectors walk from an open run's end
/// to another run's start through a free rayed end vertex of a CONVEX chain, the end face is
/// trimmed to the chain's end plane between the two ([`super::end_plane_trim_run::run`]) and the
/// section recorded in `sections`. True when any was trimmed.
#[allow(clippy::too_many_arguments)]
pub fn trim_end_ray_ends(
    mesh: &mut LevelLineMesh,
    chains: &[std::rc::Rc<super::ChainField>],
    loops: &[GapLoop],
    connectors: &[(i32, i32)],
    profile: super::BandProfile,
    segments: i32,
    sections: &mut TrimSections,
) -> Result<bool, super::LevelLineMeshError> {
    use super::corner_fill::same;

    let mut next = std::collections::HashMap::new();
    for &(a, b) in connectors {
        next.insert(a, b);
    }

    let run_starts: std::collections::HashSet<i32> = loops
        .iter()
        .filter(|l| !l.is_closed)
        .map(|l| l.vertices[0])
        .collect();
    let mut trimmed = false;
    for run in loops.iter().filter(|l| !l.is_closed) {
        let from = run.vertices[run.vertices.len() - 1];
        let mut walk = Vec::new();
        let mut at = from;
        while !run_starts.contains(&at) && walk.len() <= connectors.len() {
            let Some(&step) = next.get(&at) else {
                break;
            };
            at = step;
            walk.push(at);
        }

        if walk.len() < 2 || !run_starts.contains(&at) {
            continue;
        }

        let to = at;

        // Only a CONVEX chain's end: a concave chain adds material, and its rayed end face is
        // trimmed alone only where it folds the chain's way.
        if !is_convex_at(mesh, run, to) {
            continue;
        }

        for &v in &walk[..walk.len() - 1] {
            let p = mesh.position(v);
            for (k, chain) in chains.iter().enumerate() {
                if chain.is_closed {
                    continue;
                }

                let points = chain.points();
                let last = chain.end_rays.1 && same(p, points[points.len() - 1]);
                let first = chain.end_rays.0 && same(p, points[0]);
                let free = !(0..chains.len())
                    .any(|j| j != k && chains[j].points().iter().any(|&q| same(p, q)));
                if (last || first) && free {
                    if let Some(section) = super::end_plane_trim_run::run(
                        mesh, chain, from, to, last, profile, segments, 0,
                    )? {
                        sections.insert((from, to), section);
                        trimmed = true;
                    }
                }
            }
        }
    }

    Ok(trimmed)
}

/// An open run owned by one grow-over member of its group whose
/// interior passes exactly one of that chain's rayed ends is replaced by the end plane's section
/// laid in the group's own face ([`super::end_plane_trim_run::run`]), every vertex owned by (and
/// tied to) the chain and read at the end's parameter; the section is recorded in `sections`.
pub fn trim_grow_over_ends(
    mesh: &mut LevelLineMesh,
    gap: &super::BandGapResult,
    loops: &mut [GapLoop],
    profile: super::BandProfile,
    segments: i32,
    sections: &mut TrimSections,
) -> Result<(), super::LevelLineMeshError> {
    use super::corner_fill::same;

    let chains = &gap.chains;
    for slot in loops.iter_mut() {
        let run = &*slot;
        if run.is_closed || run.vertices.len() < 3 || run.owners.len() != 1 {
            continue;
        }

        let Some(grow_over) = gap.grow_over_by_tag.get(&run.group_tag) else {
            continue;
        };
        if !grow_over.contains(&run.owners[0]) {
            continue;
        }

        let k = run.owners[0];
        let chain = &chains[k as usize];
        if chain.is_closed {
            continue;
        }

        let inner: Vec<_> = run.vertices[1..run.vertices.len() - 1]
            .iter()
            .map(|&v| mesh.position(v))
            .collect();
        let points = chain.points();
        let first = chain.end_rays.0 && inner.iter().any(|&p| same(p, points[0]));
        let last = chain.end_rays.1 && inner.iter().any(|&p| same(p, points[points.len() - 1]));
        if first == last {
            continue;
        }

        let (from, to) = (run.vertices[0], run.vertices[run.vertices.len() - 1]);
        let group_tag = run.group_tag;
        let had_ties = run.vertex_ties.is_some();
        let side = run.side;
        let Some(section) = super::end_plane_trim_run::run(
            mesh, chain, from, to, last, profile, segments, group_tag,
        )?
        else {
            continue;
        };

        sections.insert((from, to), section.clone());
        let end_parameter = if last { chain.length } else { 0.0 };
        let mut trimmed = GapLoop::new(
            side,
            false,
            section.clone(),
            vec![end_parameter; section.len()],
            k,
            false,
            None,
        );
        trimmed.vertex_owners = Some(vec![k; section.len()]);
        trimmed.vertex_ties = if had_ties {
            Some(vec![vec![k]; section.len()])
        } else {
            None
        };
        trimmed.group_tag = group_tag;
        *slot = trimmed;
    }

    Ok(())
}
