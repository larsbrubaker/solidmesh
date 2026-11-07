//! The room of a pick of several chains: (a) foreign edges per chain, (b) chain to chain per pair,
//! (d) a stopped end's stretch of its plane covered by the other members sharing a side
//! (`stopped_end_room.rs`), (c) each side running out of itself. The pair reading's helpers are in
//! `band_room_pair_room.rs`, which re-exports this. The reading is size-free; read with each
//! chain's field at a size (`room_of_pick_flowing`) its free ends' regions flow over narrow end
//! faces as a chain alone flows them, which is the room a build that flowed cuts by.

use std::cell::RefCell;
use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use indexmap::IndexSet;
use vector_math::{ieee_math, Vector3};

use super::band_gap_reading::room_margin;
use super::band_room::{is_past_an_end_face, present, steady_down};
use super::band_room_pair_room::approach_is_end_to_end;
use super::band_room_pair_room::{
    edges_running_out_of_corners, ends_with_an_end_region, is_uniform, max_or_nan, min_or_nan,
    pair_cannot_lower_the_room, pieces_of_one_chain, sphere_gap,
};
use super::band_side_groups::group_sides;
use super::band_side_tagging::{both_sided_members_build, first_both_sided_at_a_mixed_corner};
use super::band_side_tagging_ends::{end_regions_of, SideTaggingError};
use super::corner_cap_flat_ends::use_flat_ends;
use super::foreign_edge_regions_filing::Seeds;
use super::foreign_edge_room::{cluster_at_the_ends, meets_the_chain_at_an_end};
use super::foreign_edge_room_readings::{room_for, room_from_index, ForeignEdge};
use super::pick_end_bounds::PickEnds;
use super::seam_edges::FeatureEdgeTest;
use super::stopped_end_room::{stopped_end_room, StoppedEnd};
use super::{
    chain_on_the_mesh, end_face_sense, foreign_room_counters, pick_validation, self_room,
    triangle_edges, BandProfile, ChainField, EndRayBound, FieldSample, ForeignEdgeRegions,
    LevelLineMesh, LevelLineWork, PickField, RoomCandidate, RoomField, RoomReadingKind, RoomSource,
};

/// What [`room_of_pick`] answers: the room, and everything else the reading found.
#[derive(Debug, Clone)]
pub struct PickRoom {
    /// The pick's room, less the margin a kept strip needs.
    pub room: f64,
    /// Which reading named the room, and the chain or pair it was about.
    pub set_by: RoomSource,
    /// Every reading, each less the same margin.
    pub candidates: Vec<RoomCandidate>,
    /// The first chain whose two sides fall in one group, or -1.
    pub undivided_chain: i32,
}

/// One raw reading, plus what a consumer needs to say who must go: for a chain-to-chain pair the
/// two unit setbacks the approach was shared between, and for a self reading the group.
type RawReading = (RoomSource, f64, f64, f64, bool, Option<Vec<i32>>);

/// The budget's spend so far, or 0 when no budget is in force.
fn spent_now() -> i64 {
    LevelLineWork::current().map_or(0, |budget| budget.spent.get())
}

/// ONE room for a pick of several chains, read once on the INPUT mesh.
///
/// Four readings, and the room is the least of them: FOREIGN per chain (each chain with its own
/// field), CHAIN TO CHAIN over ordered pairs inside a group (the approach shared between the two
/// unit setbacks), STOPPED END per stopped end (the size at which the other members sharing its
/// side cover its stretch of its plane) and SELF per group (the group's own field under the whole
/// pick's ceiling).
/// `name_every_candidate_under` (NaN for none) keeps every pair that could read under that size,
/// so the candidates are complete up to it; the room and its source cannot move. A chain edge not
/// on two triangles is the `ChainEdge` error.
#[allow(clippy::too_many_arguments)]
pub fn room_of_pick(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: Option<FeatureEdgeTest>,
    profile: BandProfile,
    snap: f64,
    name_every_candidate_under: f64,
    seam_epsilon: f64,
    sharp_turn_degrees: f64,
    fillet_segments: i32,
) -> Result<PickRoom, SideTaggingError> {
    room_of_pick_flowing(
        positions,
        indices,
        chains,
        closed,
        is_feature_edge,
        profile,
        snap,
        name_every_candidate_under,
        seam_epsilon,
        sharp_turn_degrees,
        fillet_segments,
        None,
    )
}

/// [`room_of_pick`] with each open chain's end regions flowed over an end face narrower than the
/// setback by `flow_reaches` (each chain's field at the size being built), as the one-chain room
/// reads the flowed tagging (`narrow_face_flow`); `None` is the plain, size-free reading.
#[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
pub fn room_of_pick_flowing(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: Option<FeatureEdgeTest>,
    profile: BandProfile,
    snap: f64,
    name_every_candidate_under: f64,
    seam_epsilon: f64,
    sharp_turn_degrees: f64,
    fillet_segments: i32,
    flow_reaches: Option<&[Rc<ChainField>]>,
) -> Result<PickRoom, SideTaggingError> {
    let is_feature_edge: FeatureEdgeTest = is_feature_edge.unwrap_or_else(|| Rc::new(|_, _| false));
    let mut source = RoomSource::nothing();
    let mut read: Vec<RawReading> = Vec::new();
    let mut candidates = Vec::new();
    let count = chains.len();
    let mut chain_edges = Vec::with_capacity(count);
    let mut unit_setbacks = Vec::with_capacity(count);
    let mut chain_points: Vec<Vec<Vector3>> = Vec::with_capacity(count);
    for k in 0..count {
        let edges = chain_on_the_mesh::chain_edges(&chains[k], closed[k]);
        unit_setbacks.push(chain_on_the_mesh::unit_setbacks(
            positions, indices, &chains[k], closed[k], &edges, profile,
        )?);
        chain_edges.push(edges);
        chain_points.push(chains[k].iter().map(|&v| positions[v as usize]).collect());
    }

    // No end floods: an end tag is not a GROUP, so the per-chain membership test has nothing to
    // say about it; each chain's own end regions are flooded apart for its foreign reading below.
    let (tags, groups, mut undivided_chain) = group_sides(
        indices,
        &chain_edges,
        &*is_feature_edge,
        false,
        None,
        None,
        None,
    )?;

    // A both-sided member is read like any other member; only with BothSidedMembersBuild off is it
    // read as a chain that does not divide its sides.
    if undivided_chain < 0 && !both_sided_members_build() {
        undivided_chain = groups
            .iter()
            .flat_map(|g| g.both_sided.iter().copied())
            .min()
            .unwrap_or(-1);
    }

    if undivided_chain < 0 {
        undivided_chain =
            first_both_sided_at_a_mixed_corner(Some(positions), indices, &chain_edges, &groups)?;
    }

    if undivided_chain >= 0 {
        return Ok(PickRoom {
            room: 0.0,
            set_by: source,
            candidates,
            undivided_chain,
        });
    }

    let mesh = Rc::new(RefCell::new(
        LevelLineMesh::from_triangles(positions, indices, Some(&tags))
            .expect("the side tagging gives one tag per triangle"),
    ));

    let oblique = |k: usize| {
        end_face_sense::oblique_ends(
            positions,
            indices,
            &chains[k],
            closed[k],
            sharp_turn_degrees,
        )
    };

    // One unit of size, as every room reading here: a setback is linear in the size.
    let same_sense_bounds_of = |k: usize| -> (Option<EndRayBound>, Option<EndRayBound>) {
        if closed[k] {
            (None, None)
        } else {
            end_face_sense::end_ray_bounds(positions, indices, &chains[k], oblique(k))
        }
    };

    // The ends as the band is cut on them (`pick_end_bounds.rs`), less the square crease clips
    // the room is read without: BandGap's end rays (none at a corner) and bounds, from one place.
    let ends = PickEnds::of(
        positions,
        indices,
        chains,
        closed,
        sharp_turn_degrees,
        &*is_feature_edge,
    );
    let rays_of_pick = &ends.end_rays;
    let pick_bounds: Vec<(Option<EndRayBound>, Option<EndRayBound>)> = (0..count)
        .map(|k| ends.bounds_without_square_crease_clips(k))
        .collect();
    let stopped = &ends.stopped;

    let field_of = |k: usize, s: f64, without_convex_rays: bool| -> ChainField {
        let scaled: Vec<f64> = unit_setbacks[k].iter().map(|u| u * s).collect();
        if !use_flat_ends() {
            return ChainField::from_polyline_sharp(
                &chain_points[k],
                closed[k],
                &scaled,
                sharp_turn_degrees,
            )
            .expect("a picked chain has a point and a setback per vertex");
        }

        let mut rays = rays_of_pick[k];
        let bounds = pick_bounds[k];
        if without_convex_rays {
            let convex_ends =
                end_face_sense::convex_end_faces(positions, indices, &chains[k], rays);
            rays = (
                rays.0 && convex_ends.0.is_none(),
                rays.1 && convex_ends.1.is_none(),
            );
        }

        ChainField::from_polyline_bounded(
            &chain_points[k],
            closed[k],
            &scaled,
            sharp_turn_degrees,
            rays,
            bounds,
        )
        .expect("a picked chain has a point and a setback per vertex")
    };

    let fields: Vec<Rc<ChainField>> = (0..count)
        .map(|k| Rc::new(field_of(k, 1.0, false)))
        .collect();

    let mut seeds_of: HashMap<i32, HashSet<i32>> = HashMap::new();
    for group in &groups {
        seeds_of.insert(group.tag, group.members.iter().copied().collect());
    }

    let seeds_of = Rc::new(seeds_of);
    let seeds: Seeds = {
        let seeds_of = Rc::clone(&seeds_of);
        Rc::new(move |tag: i32, chain: i32| tag != 0 && seeds_of[&tag].contains(&chain))
    };

    let chain_keys: IndexSet<(i32, i32)> = chain_edges
        .iter()
        .flat_map(|edges| {
            edges
                .iter()
                .map(|&(from, to)| triangle_edges::key(from, to))
        })
        .collect();

    let mut room = f64::INFINITY;
    let mut offer = |read: &mut Vec<RawReading>,
                     room: &mut f64,
                     candidate: f64,
                     named: RoomSource,
                     unit_chain: f64,
                     unit_other: f64,
                     uniform: bool,
                     group: Option<Vec<i32>>| {
        read.push((named, candidate, unit_chain, unit_other, uniform, group));
        if candidate < *room {
            *room = candidate;
            source = named;
        }
    };

    // (a) FOREIGN, per chain and with that chain's own field. The edges are read off the mesh ONCE
    // for the whole pick and filed by the tag pair they stand between. The end tags are flooded
    // only for a pick of several open chains; their errors are kept and raised once the filing is
    // done, since the filing's callback has no error of its own.
    let end_tags_error: RefCell<Option<SideTaggingError>> = RefCell::new(None);
    let end_tags_of = |k: i32| -> Vec<i32> {
        let ku = k as usize;
        match end_regions_of(
            positions,
            indices,
            &chain_edges[ku],
            &*is_feature_edge,
            &tags,
            &|tag| seeds(tag, k),
            oblique(ku),
            // Flowed, the end regions are read with the end face reach the build's own tagging
            // floods them with (`flood_pick_ends`), so the reading and the cut decide alike.
            flow_reaches.map(|r| &*r[ku]),
            flow_reaches.map(|r| &*r[ku]),
        ) {
            Ok(end_tags) => end_tags,
            Err(error) => {
                end_tags_error.borrow_mut().get_or_insert(error);
                vec![0; indices.len()]
            }
        }
    };
    let regions = ForeignEdgeRegions::over(
        Rc::clone(&mesh),
        &chain_keys,
        Some(&chain_edges),
        Some(closed),
        Some(&end_tags_of),
        Some(&seeds),
    );
    if let Some(error) = end_tags_error.into_inner() {
        return Err(error);
    }

    let regions = regions?;

    // PIECES OF ONE CHAIN: chains meeting end to end at a SETBACK STEP read foreign edges as that
    // one chain did - a region either piece seeds counts as seeded by both.
    let step_corners: HashSet<i32> = if count > 1 {
        let real: HashSet<i32> = pick_validation::corners_of(chains, closed, false)
            .iter()
            .map(|c| c.vertex)
            .collect();
        pick_validation::corners_of(chains, closed, true)
            .iter()
            .map(|c| c.vertex)
            .filter(|v| !real.contains(v))
            .collect()
    } else {
        HashSet::new()
    };
    let pieces_of = pieces_of_one_chain(chains, closed, &step_corners);
    let seeds_as_one_chain =
        |tag: i32, chain: usize| pieces_of[chain].iter().any(|&piece| seeds(tag, piece));

    let m = mesh.borrow();
    let m: &LevelLineMesh = &m;
    for k in 0..count {
        let k32 = k as i32;
        let indexes = regions.for_chain_k(k32, &|tag| seeds_as_one_chain(tag, k));
        // The end cluster is walked ONCE for the chain, not once per edge.
        let end_cluster = cluster_at_the_ends(m, &chain_edges[k], closed[k], seam_epsilon);
        let runs_out = regions.runs_out(k32);
        let meets = |edge: (i32, i32)| {
            meets_the_chain_at_an_end(
                m,
                &chain_edges[k],
                edge,
                end_cluster.as_deref(),
                runs_out.0,
                runs_out.1,
            )
        };
        let set_aside_or_meets = |edge: (i32, i32)| regions.set_aside_for(k32, edge) || meets(edge);
        let mut exempt: Option<&dyn Fn((i32, i32)) -> bool> = if closed[k] {
            None
        } else if regions.sets_aside_any(k32) {
            Some(&set_aside_or_meets)
        } else {
            Some(&meets)
        };

        let before = spent_now();
        let before_at = foreign_room_counters::clock_now();

        // The one-chain reading's convex-end rule: a foreign edge wholly beyond a convex end face
        // is read against the chain without its convex rays.
        let convex_end_faces = if !closed[k] && use_flat_ends() {
            present(end_face_sense::convex_end_faces(
                positions,
                indices,
                &chains[k],
                rays_of_pick[k],
            ))
        } else {
            Vec::new()
        };
        let mut past_an_end_face: Vec<ForeignEdge> = Vec::new();
        let not_past = exempt;
        let past_or_not_past = |edge: (i32, i32)| {
            not_past.is_some_and(|not_past| not_past(edge))
                || is_past_an_end_face(m, edge, &convex_end_faces, snap)
        };
        if !convex_end_faces.is_empty() {
            for index in &indexes {
                for &edge in &index.edges {
                    if !not_past.is_some_and(|not_past| not_past(edge))
                        && is_past_an_end_face(m, edge, &convex_end_faces, snap)
                    {
                        past_an_end_face.push((edge.0, edge.1, 0.0));
                    }
                }
            }

            exempt = Some(&past_or_not_past);
        }

        let from_index = room_from_index(
            m,
            &indexes,
            &|s| field_of(k, s, false),
            &unit_setbacks[k],
            1.0,
            exempt,
        )?;
        let past = room_for(
            m,
            &past_an_end_face,
            &|s| field_of(k, s, true),
            &unit_setbacks[k],
            1.0,
        )?;
        offer(
            &mut read,
            &mut room,
            ieee_math::min_f64(from_index, past),
            RoomSource::foreign(k32),
            f64::NAN,
            f64::NAN,
            false,
            None,
        );
        if let Some(growing) = regions.growing_for(k32) {
            if profile == BandProfile::Fillet {
                offer(
                    &mut read,
                    &mut room,
                    growing.fillet_pierce_room(&unit_setbacks[k], fillet_segments),
                    RoomSource::foreign(k32),
                    f64::NAN,
                    f64::NAN,
                    false,
                    None,
                );
            }
        }

        let spent = spent_now() - before;
        let ticks = foreign_room_counters::since(before_at);
        foreign_room_counters::with(|t| {
            t.foreign_units += spent;
            t.foreign_ticks += ticks;
        });
    }

    // A bounding SPHERE per chain - rigid-motion invariant, where an axis-aligned box is not.
    let mut chain_centre = vec![Vector3::ZERO; count];
    let mut chain_radius = vec![0.0; count];
    for k in 0..count {
        let mut centre = Vector3::ZERO;
        for &point in &chain_points[k] {
            centre += point;
        }

        centre /= ieee_math::max_f64(1.0, chain_points[k].len() as f64);
        let mut radius = 0.0;
        for &point in &chain_points[k] {
            radius = ieee_math::max_f64(radius, (point - centre).length());
        }

        chain_centre[k] = centre;
        chain_radius[k] = radius;
    }

    // The margin every candidate carries, needed HERE: the prune works in raw readings.
    let mut largest_unit = 0.0;
    for setbacks in &unit_setbacks {
        largest_unit = ieee_math::max_f64(
            largest_unit,
            if !setbacks.is_empty() {
                max_or_nan(setbacks)
            } else {
                0.0
            },
        );
    }

    let margin = room_margin(largest_unit, snap);

    // What a pair must be able to beat to be worth reading.
    let pair_threshold = |room: f64| {
        if name_every_candidate_under.is_nan() {
            room
        } else {
            ieee_math::max_f64(room, name_every_candidate_under + margin)
        }
    };

    // (b) CHAIN TO CHAIN, inside a group: the other chain's edges, carrying the other chain's
    // setback; an edge of j running out of a corner j shares with k is not read against k.
    let corner_vertices: HashSet<i32> = pick_validation::corners_of(chains, closed, true)
        .iter()
        .map(|c| c.vertex)
        .collect();
    for group in &groups {
        for &k32 in &group.members {
            for &j32 in &group.members {
                if j32 == k32 {
                    continue;
                }

                // A room already at or below zero cannot be lowered.
                if !(pair_threshold(room) > 0.0) {
                    continue;
                }

                let (k, j) = (k32 as usize, j32 as usize);

                let mut theirs: Vec<ForeignEdge> = Vec::new();
                let us = &unit_setbacks[j];
                let mut most_extra = 0.0;
                let shared_corners: Option<IndexSet<i32>> = if corner_vertices.is_empty() {
                    None
                } else {
                    Some(
                        chains[j]
                            .iter()
                            .copied()
                            .filter(|v| corner_vertices.contains(v) && chains[k].contains(v))
                            .collect(),
                    )
                };
                let out_of_corners = match &shared_corners {
                    Some(shared) if !shared.is_empty() => Some(edges_running_out_of_corners(
                        positions, &chains[j], closed[j], &chains[k], closed[k], shared,
                    )),
                    _ => None,
                };
                let ends_faced = ends_with_an_end_region(
                    &regions,
                    &chain_edges,
                    k32,
                    j32,
                    same_sense_bounds_of(k),
                    same_sense_bounds_of(j),
                );
                // j's edges stand for j's band as balls of its setback, which a stopped end's clip
                // cuts back, so this direction reads j's band through its stopped end. Where k's
                // band is its balls (no ray, no clip) and nothing here drops k's edges from the
                // other direction (no shared corner, no faced end), that other direction - j's
                // clipped field against k's edges - reads the pair alone.
                if stopped[j]
                    && !stopped[k]
                    && rays_of_pick[k] == (false, false)
                    && ends.bounds[k] == (None, None)
                    && shared_corners
                        .as_ref()
                        .is_none_or(|shared| shared.is_empty())
                    && ends_faced.is_none()
                    && ends_with_an_end_region(
                        &regions,
                        &chain_edges,
                        j32,
                        k32,
                        same_sense_bounds_of(j),
                        same_sense_bounds_of(k),
                    )
                    .is_none()
                {
                    continue;
                }

                let bare_of_k = ends_faced.as_ref().map(|_| field_of(k, 0.0, false));
                for (e, &(from, to)) in chain_edges[j].iter().enumerate() {
                    if out_of_corners
                        .as_ref()
                        .is_some_and(|out| out.contains(&(e as i32)))
                    {
                        continue;
                    }

                    if let (Some(ends), Some(bare)) = (&ends_faced, &bare_of_k) {
                        if approach_is_end_to_end(m, bare, (from, to), ends)? {
                            continue;
                        }
                    }

                    let extra = ieee_math::max_f64(us[e], us[(e + 1) % us.len()]);
                    most_extra = ieee_math::max_f64(most_extra, extra);
                    let key = triangle_edges::key(from, to);
                    theirs.push((key.0, key.1, extra));
                }

                if pair_cannot_lower_the_room(
                    pair_threshold(room),
                    sphere_gap(
                        chain_centre[k],
                        chain_radius[k],
                        chain_centre[j],
                        chain_radius[j],
                    ),
                    &unit_setbacks[k],
                    most_extra,
                ) {
                    continue;
                }

                // Only so a measurement can tell this reading from the foreign one above.
                foreign_room_counters::with(|t| t.in_a_pair = true);
                let before_pair = spent_now();
                let before_pair_at = foreign_room_counters::clock_now();

                // The GREATEST of k's unit setbacks and the LEAST of j's (RoomCandidate's
                // RoomWithoutTheOther under-reads rather than over-reads).
                let pair_room = room_for(
                    m,
                    &theirs,
                    &|s| field_of(k, s, false),
                    &unit_setbacks[k],
                    1.0,
                )?;
                offer(
                    &mut read,
                    &mut room,
                    pair_room,
                    RoomSource::chain_to_chain(k32, j32),
                    if !unit_setbacks[k].is_empty() {
                        max_or_nan(&unit_setbacks[k])
                    } else {
                        0.0
                    },
                    if !us.is_empty() { min_or_nan(us) } else { 0.0 },
                    is_uniform(&unit_setbacks[k]) && is_uniform(us),
                    None,
                );
                let spent = spent_now() - before_pair;
                let ticks = foreign_room_counters::since(before_pair_at);
                foreign_room_counters::with(|t| {
                    t.pair_units += spent;
                    t.pair_ticks += ticks;
                    t.in_a_pair = false;
                });
            }
        }
    }

    // (d) STOPPED ENDS: the size at which the other members sharing a side cover a stopped end's
    // own stretch of its plane there (`stopped_end_room.rs`), read below what a pair must beat, so
    // with `name_every_candidate_under` set every stopped end under it is named; the source is the
    // member that covers the last of the stretch.
    for group in &groups {
        for &a in &group.members {
            let au = a as usize;
            let chain = &chains[au];
            let bounded_ends = [
                (pick_bounds[au].0, chain[0]),
                (pick_bounds[au].1, chain[chain.len() - 1]),
            ];
            for (bound, end_vertex) in bounded_ends {
                let Some(bound) = bound.filter(|b| b.stopped) else {
                    continue;
                };

                if !(pair_threshold(room) > 0.0) {
                    continue;
                }

                let others: Vec<usize> = group
                    .members
                    .iter()
                    .map(|&k| k as usize)
                    .filter(|&k| k != au && !chains[k].contains(&end_vertex))
                    .collect();
                if others.is_empty() {
                    continue;
                }

                let end = StoppedEnd {
                    chain: au,
                    end_vertex,
                    vertex: positions[end_vertex as usize],
                    normal: bound.plane_normal.get_normal(),
                };
                let widest = if unit_setbacks[au].is_empty() {
                    0.0
                } else {
                    max_or_nan(&unit_setbacks[au])
                };
                let Some(candidate) = stopped_end_room(
                    positions,
                    indices,
                    &tags,
                    group.tag,
                    &end,
                    &others,
                    &|k, s| field_of(k, s, false),
                    widest,
                    pair_threshold(room),
                )?
                else {
                    continue;
                };
                offer(
                    &mut read,
                    &mut room,
                    candidate.room,
                    RoomSource::chain_to_chain(a, candidate.covered_by as i32),
                    f64::NAN,
                    f64::NAN,
                    false,
                    None,
                );
            }
        }
    }

    // (c) SELF, per group, on the group's own field and under the WHOLE pick's ceiling.
    let mut every_chain_point = Vec::new();
    let mut seen_vertex = HashSet::new();
    for chain in chains {
        for &vertex in chain {
            if seen_vertex.insert(vertex) {
                every_chain_point.push(positions[vertex as usize]);
            }
        }
    }

    let pick_ceiling = self_room::chain_ceiling(&every_chain_point);
    for group in &groups {
        let field = if group.members.len() == 1 {
            RoomField::from_chain(Rc::clone(&fields[group.members[0] as usize]))
        } else {
            RoomField::from_pick(Rc::new(
                PickField::new(fields.clone(), &group.members)
                    .expect("a side group has distinct member chains"),
            ))
        };

        // A sample per group: the groups read different fields. The ceiling is relaxed under an
        // asked size exactly as the pair prune is.
        let mut samples: Vec<Option<FieldSample>> = vec![None; m.vertex_count() as usize];
        let before_self = spent_now();
        let before_self_at = foreign_room_counters::clock_now();
        let self_room = self_room::self_room_of_side(
            m,
            &field,
            group.tag,
            1.0,
            &mut samples,
            ieee_math::min_f64(pick_ceiling, pair_threshold(room)),
        )?;
        offer(
            &mut read,
            &mut room,
            self_room,
            RoomSource::self_room(group.members[0]),
            f64::NAN,
            f64::NAN,
            false,
            Some(group.members.clone()),
        );
        let spent = spent_now() - before_self;
        let ticks = foreign_room_counters::since(before_self_at);
        foreign_room_counters::with(|t| {
            t.self_units += spent;
            t.self_ticks += ticks;
        });
    }

    // Every candidate carries the margin the room carries, so the room IS the least of them.
    for (named, raw, unit_chain, unit_other, uniform, group) in read {
        let mut without = f64::NAN;
        let mut at_most = f64::INFINITY;
        if named.kind == RoomReadingKind::ChainToChain
            && unit_chain > 0.0
            && unit_other >= 0.0
            && raw.is_finite()
        {
            without = steady_down((raw * (unit_chain + unit_other) / unit_chain) - margin);
            // Exact only with both setbacks uniform; otherwise the over-read is unknown.
            at_most = if uniform { without } else { f64::INFINITY };
        }

        candidates.push(RoomCandidate {
            source: named,
            room: steady_down(raw - margin),
            room_without_the_other: without,
            room_without_the_other_at_most: at_most,
            group_members: group,
        });
    }

    Ok(PickRoom {
        room: steady_down(room - margin),
        set_by: source,
        candidates,
        undivided_chain,
    })
}
