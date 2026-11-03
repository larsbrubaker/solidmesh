//! The corner fill: a pick whose chains meet at corners filled chain by chain - a closed chain's
//! own two loops zipped whole, every other chain's two rail pieces zipped into a strip - and the
//! holes left round the corners closed by the corner caps, refusing (and taking every laid
//! triangle back out) where the shell does not close. The pieces and connectors are in
//! `corner_fill`, the trims in
//! `corner_fill_end_ray_trim`, the strip in `corner_rail_strip_zip`, the caps in `corner_cap`.

use std::cell::RefCell;
use std::collections::HashSet;
use std::rc::Rc;

use super::corner_cap::{close, CornerCapLastRow};
use super::corner_fill::{connectors as connectors_of, corner_points, pieces};
use super::corner_fill_end_ray_trim::{trim_end_ray_ends, trim_grow_over_ends, TrimSections};
use super::corner_fill_pieces::{trim_off_the_line, trim_the_turn_back};
use super::corner_rail_strip_zip::strip;
use super::gap_fill_one_chain::fill_one_chain;
use super::{
    BandGapResult, BandProfile, CornerFillPiece, GapFillRefusal, GapFillResult, GapLoop,
    LevelLineMeshError,
};

/// The observer [`WATCH_OPEN_HOLES`] holds.
pub type WatchOpenHoles = Rc<dyn Fn(&HashSet<(i32, i32)>)>;

thread_local! {
    /// Told the open edges the caps are handed.
    /// Per thread; `None` by default.
    pub static WATCH_OPEN_HOLES: RefCell<Option<WatchOpenHoles>> = const { RefCell::new(None) };
}

/// Why the corner fill could not run.
#[derive(Debug, Clone)]
pub enum CornerFillError {
    /// The work budget ran out, the build was cancelled, or the mesh refused an edit.
    Mesh(LevelLineMeshError),
    /// A fillet asked for with fewer than one segment.
    FilletSegments(String),
}

impl From<LevelLineMeshError> for CornerFillError {
    fn from(error: LevelLineMeshError) -> Self {
        CornerFillError::Mesh(error)
    }
}

impl From<super::WorkError> for CornerFillError {
    fn from(error: super::WorkError) -> Self {
        CornerFillError::Mesh(error.into())
    }
}

impl std::fmt::Display for CornerFillError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CornerFillError::Mesh(error) => write!(f, "{error}"),
            CornerFillError::FilletSegments(message) => f.write_str(message),
        }
    }
}

impl std::error::Error for CornerFillError {}

/// Fills the gap's band at its corners. The gap's mesh is the one filled.
pub fn fill(
    gap: &BandGapResult,
    profile: BandProfile,
    fillet_segments: i32,
) -> Result<GapFillResult, CornerFillError> {
    if profile == BandProfile::Fillet && fillet_segments < 1 {
        return Err(CornerFillError::FilletSegments(
            "an arc is drawn with at least one segment (Parameter 'filletSegments')".to_string(),
        ));
    }

    let segments = if profile == BandProfile::Chamfer {
        1
    } else {
        fillet_segments
    };
    let mesh_rc = gap.mesh.clone().expect("a built gap carries its mesh");
    let mut loops: Vec<GapLoop> = gap.loops.clone();
    let hole_fill = super::corner_cap_hole_fill_switch::use_hole_fill();
    let mut connectors: Option<Vec<(i32, i32)>> = None;
    let mut sections = TrimSections::new();
    let refused = |refusal: GapFillRefusal| GapFillResult {
        refusal,
        mesh: Some(mesh_rc.clone()),
        ..GapFillResult::default()
    };
    if loops.iter().any(|l| l.vertex_owners.is_none())
        || (!hole_fill && loops.iter().any(|l| !l.is_closed))
    {
        return Ok(refused(GapFillRefusal::SideDoesNotLeaveOneLoop));
    }

    if loops.iter().any(|l| !l.is_closed) {
        let mut mesh = mesh_rc.borrow_mut();
        trim_grow_over_ends(&mut mesh, gap, &mut loops, profile, segments, &mut sections)?;
        connectors = connectors_of(&mesh, &loops);
        if let Some(found) = &connectors {
            if trim_end_ray_ends(
                &mut mesh,
                &gap.chains,
                &loops,
                found,
                profile,
                segments,
                &mut sections,
            )? {
                connectors = connectors_of(&mesh, &loops);
            }
        }

        if connectors.is_none() {
            drop(mesh);
            return Ok(refused(GapFillRefusal::SideDoesNotLeaveOneLoop));
        }
    }

    let owners_of = |l: &GapLoop| l.vertex_owners.clone().unwrap_or_default();
    let in_mixed: HashSet<i32> = loops
        .iter()
        .filter(|l| {
            let owners = owners_of(l);
            owners.iter().any(|&o| o != owners[0])
        })
        .flat_map(owners_of)
        .collect();
    let own_indices: Vec<usize> = (0..loops.len())
        .filter(|&i| {
            let l = &loops[i];
            let owners = owners_of(l);
            gap.chains[owners[0] as usize].is_closed
                && owners.iter().all(|&o| o == owners[0])
                && !(super::band_side_tagging::both_sided_members_build()
                    && in_mixed.contains(&owners[0]))
                && l.vertex_ties
                    .as_ref()
                    .is_none_or(|ties| ties.iter().all(|t| t.len() == 1))
        })
        .collect();
    let zipped: HashSet<i32> = own_indices
        .iter()
        .map(|&i| owners_of(&loops[i])[0])
        .collect();

    let mut the_pieces: Vec<CornerFillPiece> = Vec::new();
    {
        let mesh = mesh_rc.borrow();
        for (i, l) in loops.iter().enumerate() {
            if own_indices.contains(&i)
                || gap.end_region_tags.contains(&l.group_tag)
                || gap
                    .end_members_by_tag
                    .get(&l.group_tag)
                    .is_some_and(|ends| l.owners.iter().all(|o| ends.contains(o)))
            {
                continue;
            }

            for piece in pieces(&mesh, &gap.chains, l)? {
                if gap
                    .carried_by_tag
                    .get(&l.group_tag)
                    .is_some_and(|carried| carried.contains(&piece.owner))
                {
                    continue;
                }

                the_pieces.push(piece);
            }
        }

        for piece in &mut the_pieces {
            let field = &gap.chains[piece.owner as usize];
            trim_off_the_line(&mesh, field, piece)?;
            trim_the_turn_back(field, piece);
        }
    }

    let chain_count = gap.chains.len();
    let mut added: Vec<i32> = Vec::new();
    let mut added_vertices: Vec<i32> = Vec::new();
    let mut triangles_of_chain = vec![0; chain_count];
    let mut vertices_of_chain = vec![0; chain_count];
    let mut last_rows: Vec<CornerCapLastRow> = Vec::new();
    let mut splits = std::collections::HashMap::new();
    let refuse = |added: &[i32],
                  refusal: GapFillRefusal,
                  vertex: i32,
                  chain: i32,
                  edge: Option<(i32, i32)>| {
        let mut mesh = mesh_rc.borrow_mut();
        for &triangle in added {
            mesh.remove_triangle(triangle);
        }

        GapFillResult {
            refusal,
            mesh: Some(mesh_rc.clone()),
            offending_vertex: vertex,
            offending_chain: chain,
            open_edge: edge.unwrap_or((-1, -1)),
            ..GapFillResult::default()
        }
    };

    for k in 0..chain_count {
        let k32 = k as i32;
        if zipped.contains(&k32) {
            let own: Vec<GapLoop> = own_indices
                .iter()
                .map(|&i| &loops[i])
                .filter(|l| owners_of(l)[0] == k32)
                .map(|l| {
                    GapLoop::new(
                        l.side,
                        true,
                        l.vertices.clone(),
                        l.parameters.clone(),
                        k32,
                        false,
                        None,
                    )
                })
                .collect();
            if own.len() != 2
                || own.iter().filter(|l| l.side == 1).count() != 1
                || the_pieces.iter().any(|p| p.owner == k32)
            {
                return Ok(refuse(
                    &added,
                    GapFillRefusal::SideDoesNotLeaveOneLoop,
                    -1,
                    k32,
                    None,
                ));
            }

            let one = fill_one_chain(
                &mesh_rc,
                &gap.chains[k],
                &own,
                profile,
                segments,
                &mut added,
                &mut splits,
            )?;
            if !one.filled() {
                return Ok(refuse(
                    &added,
                    one.refusal,
                    one.offending_vertex,
                    k32,
                    Some(one.open_edge),
                ));
            }

            added_vertices.extend_from_slice(&one.added_vertices);
            triangles_of_chain[k] = one.added_triangles.len() as i32;
            vertices_of_chain[k] = one.added_vertices.len() as i32;
            continue;
        }

        let mine: Vec<usize> = (0..the_pieces.len())
            .filter(|&i| the_pieces[i].owner == k32)
            .collect();
        if mine.len() > 2 && hole_fill {
            let climbs = |piece: &CornerFillPiece| {
                piece.parameters[piece.parameters.len() - 1] > piece.parameters[0]
            };
            let climbing: Vec<usize> = mine
                .iter()
                .copied()
                .filter(|&i| climbs(&the_pieces[i]))
                .collect();
            let falling: Vec<usize> = mine
                .iter()
                .copied()
                .filter(|&i| !climbs(&the_pieces[i]))
                .collect();
            let single = if climbing.len() == 1 {
                Some(climbing[0])
            } else if falling.len() == 1 {
                Some(falling[0])
            } else {
                None
            };
            let split = single.map(|_| {
                if climbing.len() == 1 {
                    &falling
                } else {
                    &climbing
                }
            });
            if let (Some(single), Some(split)) = (single, split) {
                if split.len() >= 2 {
                    for &part in split {
                        let within = {
                            let p = &the_pieces[part].parameters;
                            (
                                p.iter().copied().fold(f64::INFINITY, f64::min),
                                p.iter().copied().fold(f64::NEG_INFINITY, f64::max),
                            )
                        };
                        let (mut single_piece, mut part_piece) =
                            (the_pieces[single].clone(), the_pieces[part].clone());
                        let (part_refusal, part_offending) = strip(
                            &mut mesh_rc.borrow_mut(),
                            &gap.chains,
                            k,
                            hole_fill,
                            &mut single_piece,
                            &mut part_piece,
                            segments,
                            &mut added,
                            &mut added_vertices,
                            &mut last_rows,
                            &mut splits,
                            &sections,
                            Some(within),
                        )?;
                        the_pieces[single] = single_piece;
                        the_pieces[part] = part_piece;
                        if part_refusal != GapFillRefusal::None {
                            return Ok(refuse(&added, part_refusal, part_offending, k32, None));
                        }
                    }

                    continue;
                }
            }
        }

        if mine.len() != 2 && hole_fill {
            continue;
        }

        if mine.len() != 2 {
            return Ok(refuse(
                &added,
                GapFillRefusal::SideDoesNotLeaveOneLoop,
                -1,
                k32,
                None,
            ));
        }

        let before = added.len();
        let vertices_before = added_vertices.len();
        let (mut first, mut second) = (the_pieces[mine[0]].clone(), the_pieces[mine[1]].clone());
        let (refusal, offending) = strip(
            &mut mesh_rc.borrow_mut(),
            &gap.chains,
            k,
            hole_fill,
            &mut first,
            &mut second,
            segments,
            &mut added,
            &mut added_vertices,
            &mut last_rows,
            &mut splits,
            &sections,
            None,
        )?;
        the_pieces[mine[0]] = first;
        the_pieces[mine[1]] = second;
        if refusal != GapFillRefusal::None {
            return Ok(refuse(&added, refusal, offending, k32, None));
        }

        triangles_of_chain[k] = (added.len() - before) as i32;
        vertices_of_chain[k] = (added_vertices.len() - vertices_before) as i32;
    }

    let mut kept: Vec<(i32, i32)> = Vec::new();
    for l in &loops {
        let n = l.vertices.len();
        let count = if l.is_closed { n } else { n - 1 };
        for i in 0..count {
            kept.extend(super::rail_stations::pieces(
                l.vertices[i],
                l.vertices[(i + 1) % n],
                &splits,
            ));
        }
    }
    for &(a, b) in connectors.as_deref().unwrap_or(&[]) {
        kept.extend(super::rail_stations::pieces(a, b, &splits));
    }

    let mut open: HashSet<(i32, i32)> = kept.iter().map(|&(a, b)| (b, a)).collect();
    {
        let mesh = mesh_rc.borrow();
        for &triangle in &added {
            let face = mesh.triangles()[triangle as usize];
            for corner in 0..3 {
                let edge = (face.corner(corner), face.corner((corner + 1) % 3));
                if !open.remove(&edge) {
                    open.insert((edge.1, edge.0));
                }
            }
        }
    }

    let watch = WATCH_OPEN_HOLES.with(|w| w.borrow().clone());
    if let Some(watch) = watch {
        watch(&open);
    }

    let corners = corner_points(&gap.chains);
    let mut boundary_changes: Vec<(i32, i32, bool)> = Vec::new();
    let (cap_refusal, cap_offending) = close(
        &mut mesh_rc.borrow_mut(),
        &open,
        &corners,
        if segments == 1 {
            None
        } else {
            Some(&last_rows)
        },
        segments,
        &mut added,
        &mut added_vertices,
        Some(&mut boundary_changes),
    )?;
    if cap_refusal != GapFillRefusal::None {
        return Ok(refuse(&added, cap_refusal, cap_offending, -1, None));
    }

    kept.retain(|&(a, b)| !boundary_changes.contains(&(a, b, false)));
    kept.extend(boundary_changes.iter().filter(|c| c.2).map(|c| (c.0, c.1)));
    let open_edge = super::fill_closure::first_open_edge_of(&mesh_rc.borrow(), &kept, &added);
    if open_edge != (-1, -1) {
        return Ok(refuse(
            &added,
            GapFillRefusal::FillWouldNotClose,
            -1,
            -1,
            Some(open_edge),
        ));
    }

    Ok(GapFillResult {
        refusal: GapFillRefusal::None,
        mesh: Some(mesh_rc.clone()),
        added_triangles: added,
        added_vertices,
        triangles_of_chain,
        vertices_of_chain,
        ..GapFillResult::default()
    })
}
