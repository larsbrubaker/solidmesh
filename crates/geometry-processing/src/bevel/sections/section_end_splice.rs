//! The TRANSVERSE face's own rebuild: the flat face an OPEN run dies on - the wall that cuts the
//! chain off - drawn again with the end section's PROFILE spliced into its rim where the chain's
//! end vertex stood ([`try_rebuild_the_transverse_faces`]). The rim is read and the spliced ring
//! drawn by `section_fan_remesh` ([`super::section_fan_remesh::try_the_outline`],
//! [`super::section_fan_remesh::try_draw_the_ring`]); the face's flat region is
//! [`super::section_room::faces_of_the_flat_region`].
//!
//! **Why substitution cannot do it.** Every other face the fillet touches loses ONE point and
//! gains ONE - its corner at a chain vertex moves out to that side's boundary. The transverse face
//! loses one point and gains a whole PATH: the fillet dies on it as an arc, not as a corner, and
//! the arc has as many points as the profile has samples. So this face is redrawn from its own rim
//! rather than rewritten triangle by triangle.
//!
//! **What the rim loses.** The chain's end vertex E, and the crease vertices between E and each
//! side's LANDING: a landing slides down a crease that is generally a POLYLINE - the raked bore's
//! mouth starts with a chord an eighth of a millimetre long - so at a radius worth filleting the
//! slide runs past one or more of the rim's own points, and those points are swallowed. Which chord
//! holds a landing is read by the same projection walk `SectionSmoothSide.TryTheCreaseChordUnder`
//! makes, taken here along the RIM (both slide creases bound this face, so their chords ARE rim
//! edges) so that the walk and the splice agree by construction.
//!
//! **What it gains, and which way round.** The end section's profile, whose two ends ARE the two
//! landings (`profile[0]` is the left boundary, the last the right). Its direction is settled by
//! IDENTITY and not by position: the first ring neighbour of E on each side is the far vertex of
//! that side's slide edge, which says which way round the rim the left landing lies.
//!
//! The faces the wall was drawn with are then dropped and the whole rim is eared, exactly as a
//! folded region is. The profile's inner samples bow into the face, so those ears are real
//! triangles and not the degenerate fan a straight chord would give.

use std::collections::HashSet;

use indexmap::IndexSet;
use solidmesh_math::Vector3;
use solidmesh_mesh::{Mesh, MeshEdgeGraph};

use super::section_fan_remesh::{try_draw_the_ring, try_the_outline, Triple};
use super::section_fan_walk::EdgeAtPair;
use super::{section_room, BevelSection, SectionPool, SectionRun};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};

fn refused(code: BevelAdvisoryCode, at: Vector3) -> BevelAdvisory {
    BevelAdvisory::new(code, vec![at.into()])
}

/// Draws the transverse face of every OPEN run again, with each
/// end's profile spliced into its rim.
///
/// `mesh` is the part the chains were picked on and `graph` its edge graph; `edge_at_pair` the
/// mesh edge filed under each vertex pair; `runs` the runs being rebuilt (the closed ones have no
/// end and are passed over); `pool` the pool the sections minted their boundaries into;
/// `chain_edges` the pair keys of the picked edges, which the flat flood may not cross; `rebuilt`
/// the faces as they stand after every substitution. The triangles drawn to replace the transverse
/// faces are appended to `added`, and the faces to leave out of the published list are added to
/// `dropped`. `Err` says why not, where a rim could not be read or spliced.
#[allow(clippy::too_many_arguments)]
pub fn try_rebuild_the_transverse_faces(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    runs: &[SectionRun],
    pool: &SectionPool,
    chain_edges: &HashSet<i64>,
    rebuilt: &[Triple],
    added: &mut Vec<Triple>,
    dropped: &mut HashSet<i32>,
) -> Result<(), BevelAdvisory> {
    let mut regions: Vec<(IndexSet<i32>, Vec<&BevelSection>)> = Vec::new();

    for run in runs {
        if run.is_closed {
            continue;
        }

        for section in [&run.sections[0], &run.sections[run.sections.len() - 1]] {
            let section: &BevelSection = section;
            // After the substitution the ONLY faces still standing on the end vertex are the
            // transverse ones: the two fans took every face between the chain edge and each slide
            // crease, which is all the rest of the one-ring.
            let seed = the_face_still_on(rebuilt, dropped, section.chain_vertex);

            if seed < 0 {
                return Err(refused(
                    BevelAdvisoryCode::SurfaceStripEndNotSupported,
                    section.station,
                ));
            }

            let mut held = -1;

            for (i, region) in regions.iter().enumerate() {
                if region.0.contains(&seed) {
                    held = i as i32;
                }
            }

            if held >= 0 {
                // Both ends of an arc cut off by ONE wall stand on the same flat region, which is
                // then drawn once with both splices in its rim.
                regions[held as usize].1.push(section);

                continue;
            }

            regions.push((
                section_room::faces_of_the_flat_region(
                    mesh,
                    graph,
                    edge_at_pair,
                    seed,
                    Some(chain_edges),
                    None,
                    None,
                ),
                vec![section],
            ));
        }
    }

    for (faces, ends) in &regions {
        let mut piece = Vec::new();

        for &face in faces {
            if dropped.contains(&face) {
                // A fold's re-mesh has already redrawn this wall, and its ears carry the end vertex
                // no rim of ours could then take out.
                return Err(refused(
                    BevelAdvisoryCode::SurfaceCornerNotSupported,
                    ends[0].station,
                ));
            }

            piece.push(face);
        }

        let mut ring = match try_the_outline(rebuilt, &piece) {
            Ok(ring) => ring,
            Err(pinched_at) => {
                return Err(refused(
                    BevelAdvisoryCode::SurfaceCornerNotSupported,
                    if pinched_at >= 0 {
                        pool.position_of(pinched_at)
                    } else {
                        ends[0].station
                    },
                ));
            }
        };

        let mut gone = HashSet::new();

        for end in ends {
            try_splice(mesh, graph, pool, end, &mut ring, &mut gone)?;
        }

        let region: HashSet<i32> = faces.iter().copied().collect();
        try_draw_the_ring(
            mesh,
            pool,
            &piece,
            &region,
            &ring,
            Vector3::from_vector3_float(mesh.faces[piece[0] as usize].normal),
            ends[0].station,
            rebuilt,
            added,
            Some(&gone),
        )?;

        for &face in &piece {
            dropped.insert(face);
        }
    }

    Ok(())
}

/// Puts one end's profile into the rim in place of its end vertex and the crease
/// vertices its two landings slid past.
fn try_splice(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    pool: &SectionPool,
    end: &BevelSection,
    ring: &mut Vec<i32>,
    gone: &mut HashSet<i32>,
) -> Result<(), BevelAdvisory> {
    let Some(at) = ring.iter().position(|&point| point == end.chain_vertex) else {
        return Err(refused(
            BevelAdvisoryCode::SurfaceStripEndNotSupported,
            end.station,
        ));
    };
    let at = at as i32;

    let count = ring.len() as i32;
    let wrap = |index: i32| ((index % count + count) % count) as usize;
    let forward = ring[wrap(at + 1)];
    let backward = ring[wrap(at - 1)];

    let left_first = far_vertex_of(graph, end.left_slide_edge, end.chain_vertex);
    let right_first = far_vertex_of(graph, end.right_slide_edge, end.chain_vertex);

    let forward_is_left = if forward == left_first && backward == right_first {
        true
    } else if forward == right_first && backward == left_first {
        false
    } else {
        // The rim does not leave the end vertex down the two creases the end recorded, so this is
        // not the face the run dies on and nothing here may be spliced into it.
        return Err(refused(
            BevelAdvisoryCode::SurfaceStripEndNotSupported,
            end.station,
        ));
    };

    let ahead = swallowed_along(
        mesh,
        ring,
        at,
        1,
        pool.position_of(if forward_is_left {
            end.left_boundary
        } else {
            end.right_boundary
        }),
    );
    let behind = swallowed_along(
        mesh,
        ring,
        at,
        -1,
        pool.position_of(if forward_is_left {
            end.right_boundary
        } else {
            end.left_boundary
        }),
    );

    if ahead < 0 || behind < 0 || ahead + behind + 1 >= count {
        // A landing past the far end of the rim's own crease: the fillet is wider than the wall it
        // dies on, which is a refusal at the station and not a splice.
        return Err(refused(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            end.station,
        ));
    }

    let mut spliced = Vec::with_capacity(count as usize + end.profile.len());

    // The survivors, from the one just past the swallowed block round to the one just before it,
    // so the block's own place in the rim is where the path goes.
    for step in 0..count - (ahead + behind + 1) {
        spliced.push(ring[wrap(at + 1 + ahead + step)]);
    }

    let profile_length = end.profile.len();
    for i in 0..profile_length {
        // Read from the BEHIND side forward, which is the way the rim runs through the block: the
        // profile's left end goes first where the left landing is the one behind.
        spliced.push(
            end.profile[if forward_is_left {
                profile_length - 1 - i
            } else {
                i
            }],
        );
    }

    for step in -behind..=ahead {
        gone.insert(ring[wrap(at + step)]);
    }

    *ring = spliced;

    Ok(())
}

/// How many rim points one landing swallows going one way round from the end
/// vertex - the projection walk steps chord by chord and stops at the first chord whose span holds
/// the landing, and every point stepped over is swallowed. -1 where the rim ran out before the
/// landing did.
///
/// Also -1 where the walk reaches a rim point that is not a mesh vertex: the rim is ring indices
/// into the section POOL, and once one end has been spliced into a region the rim carries that
/// end's profile, which the pool minted past the mesh. Reading those as mesh vertices would run
/// off the end; a landing that slides into the other end's profile is two splices claiming one
/// stretch of rim, which is the same refusal as a rim that ran out.
pub(super) fn swallowed_along(
    mesh: &Mesh,
    ring: &[i32],
    at: i32,
    way: i32,
    landing: Vector3,
) -> i32 {
    let count = ring.len() as i32;
    let wrap = |index: i32| ((index % count + count) % count) as usize;
    let mesh_vertex = |index: i32| {
        usize::try_from(index)
            .ok()
            .and_then(|index| mesh.vertices.get(index))
            .map(|&vertex| Vector3::from_vector3_float(vertex))
    };

    for swallowed in 0..count - 1 {
        let (Some(from), Some(to)) = (
            mesh_vertex(ring[wrap(at + (way * swallowed))]),
            mesh_vertex(ring[wrap(at + (way * (swallowed + 1)))]),
        ) else {
            return -1;
        };
        let along = to - from;

        if along.length_squared() < 1e-300 {
            return -1;
        }

        if (landing - from).dot(along) / along.length_squared() <= 1.0 {
            return swallowed;
        }
    }

    -1
}

/// The end of a mesh edge that is not `vertex`.
fn far_vertex_of(graph: &MeshEdgeGraph, edge: i32, vertex: i32) -> i32 {
    if graph.get_vertex0(edge) == vertex {
        graph.get_vertex1(edge)
    } else {
        graph.get_vertex0(edge)
    }
}

/// A face of the rebuilt list that still stands on `vertex`, or -1.
fn the_face_still_on(rebuilt: &[Triple], dropped: &HashSet<i32>, vertex: i32) -> i32 {
    for (face, drawn) in rebuilt.iter().enumerate() {
        let face = face as i32;
        if dropped.contains(&face) {
            continue;
        }

        if drawn.0 == vertex || drawn.1 == vertex || drawn.2 == vertex {
            return face;
        }
    }

    -1
}

#[cfg(test)]
#[path = "section_end_splice_tests.rs"]
mod tests;
