//! The band gap's build proper from step 1's fields on (split out of `band_gap.rs` for size; the
//! pass is entered and its step 1 readings made in `band_gap_build_entry.rs`).
//! Steps 1 to 7 of the level line construction - setback, sides (step 2 is
//! `band_gap_build_sides.rs`'s), refinement and roots per group, the band's delete, the room, the
//! gap loops and the pinch - leaving nothing to do but fill. A pick of several chains reads its
//! room before the sides, so the sides can be flowed over narrow end faces where that alone holds
//! the size, by the one-chain rule (`narrow_face_flow::takes_the_flow`). Its faults travel as
//! [`BuildFault`] (`band_gap_build_fault.rs`), which
//! [`band_gap::build_unvalidated`](super::band_gap::build_unvalidated) turns into the caller's
//! outcomes. Step 1's setbacks and fields are `band_gap_setback_fields.rs`'s.

use std::cell::RefCell;
use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use indexmap::IndexMap;
use vector_math::Vector3;

use super::band_gap::{
    chord_tolerance_of, has_a_free_end, needle_floor_of, refused_by_size, BuildWithinArgs,
};
use super::band_gap_build_fault::{defect, BuildFault};
use super::band_gap_build_sides::{self, SideInputs, Sides};
use super::band_gap_reading::room_margin;
use super::band_gap_setback_fields::PassSetbacks;
use super::band_refinement::{self, RefineOptions};
use super::seam_edges::FeatureEdgeTest;
use super::{
    band_gap_extra_fields, band_group_fields, band_room_pair_room_pick, band_side_tagging,
    band_triangles, consumed_to_the_rim, corner_tie_points, end_clip_corners, gap_loop_walk,
    narrow_face_flow, one_chain_room, pick_validation, pinch_ladder_search, sharp_joint_corners,
    side_roots_settling, side_witness, square_crease_end_clip, swallowed_side_runs, triangle_edges,
};
use super::{
    BandGapRefusal, BandGapResult, BandGroup, ChainEdgeNotOnTwoTriangles, FieldCounters, GapLoop,
    LevelLineMesh, LevelLineWork, LevelLineWorkPhase, RoomReadingKind, RoomSource, ScalarField,
    VertexFieldReading,
};

/// The build from step 1's fields on, on the level line's own setbacks.
#[allow(clippy::too_many_arguments)]
pub(super) fn build_from_the_setbacks(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: Option<FeatureEdgeTest>,
    args: &BuildWithinArgs,
    working: &mut Option<Rc<RefCell<LevelLineMesh>>>,
    unflowed_room: &mut Option<f64>,
    setbacks: &PassSetbacks,
) -> Result<BandGapResult, BuildFault> {
    let BuildWithinArgs {
        size,
        profile,
        chord_tolerance,
        snap,
        known_room,
        topology_only,
        seam_epsilon,
        sharp_turn_degrees,
        fillet_segments,
        step_corners,
        handed_in_room,
        ..
    } = *args;
    let is_feature_edge: FeatureEdgeTest =
        is_feature_edge.unwrap_or_else(|| Rc::new(|_: i32, _: i32| false));
    let tolerance = chord_tolerance_of(size, chord_tolerance);
    let count = chains.len();
    let only = count == 1;

    let PassSetbacks {
        chain_edges,
        unit_setbacks,
        end_rays,
        ..
    } = setbacks;
    let fields = setbacks.fields_at_the_size(size)?;

    // Scaling the setbacks moves no point, so a polyline that built a field at the size builds it
    // at every size.
    let field_at_0 = |s: f64| {
        setbacks
            .field_of(0, s)
            .expect("the chain's polyline built a field at the size asked")
    };
    let field = Rc::clone(&fields[0]);

    // A pick of several chains reads its room on the input mesh, so it is read here, before the
    // sides: where it refuses the size and the pick with every free end flowed over a narrow end
    // face holds it, the flowed tagging is cut (the rule a pick of one chain flows by,
    // `narrow_face_flow::takes_the_flow`), and the plain room is kept as the room the user is told.
    let read_the_pick = |flow_reaches: Option<&[Rc<super::ChainField>]>| {
        LevelLineWork::enter(LevelLineWorkPhase::CutRoom);
        band_room_pair_room_pick::room_of_pick_flowing(
            positions,
            indices,
            chains,
            closed,
            Some(Rc::clone(&is_feature_edge)),
            profile,
            snap,
            f64::NAN,
            seam_epsilon,
            sharp_turn_degrees,
            fillet_segments,
            flow_reaches,
        )
    };
    let mut pick_room = None;
    // Which reading named the plain room a flowed pick quotes, where it read one itself.
    let mut unflowed_set_by: Option<RoomSource> = None;
    let mut flow_reaches: Option<&[Rc<super::ChainField>]> = None;
    if !only && known_room.is_none() {
        // A room handed in IS the plain room, read already: only the flowed one is read here, and
        // where the flow does not hold the size the pick refuses with the handed room at once.
        let plain = match handed_in_room {
            Some(_) => None,
            None => Some(read_the_pick(None)?),
        };
        let plain_room = handed_in_room
            .or(plain.as_ref().map(|p| p.room))
            .unwrap_or(f64::INFINITY);
        let mut flowed = None;
        let takes = has_a_free_end(chains, closed)
            && narrow_face_flow::takes_the_flow(size, plain_room, || {
                let read = read_the_pick(Some(&fields))?;
                let room = read.room;
                flowed = Some(read);
                Ok::<f64, super::band_side_tagging_ends::SideTaggingError>(room)
            })?;
        if takes {
            *unflowed_room = Some(plain_room);
            unflowed_set_by = plain.as_ref().map(|p| p.set_by);
            flow_reaches = Some(&fields);
            pick_room = flowed;
        } else if let Some(handed) = handed_in_room {
            return Ok(refused_by_size(handed));
        } else {
            pick_room = plain;
        }
    }

    // 2. SIDES, as GROUPS.
    LevelLineWork::enter(LevelLineWorkPhase::Sides);
    let side_inputs = SideInputs {
        positions,
        indices,
        chain_edges,
        closed,
        is_feature_edge: &is_feature_edge,
        end_rays,
        unit_setbacks,
        fields: &fields,
        flow_reaches,
        field: &field,
        field_at_0: &field_at_0,
        known_room,
        size,
        snap,
        seam_epsilon,
        profile,
        fillet_segments,
        step_corners,
    };
    let (tags, groups) = match band_gap_build_sides::tag_the_sides(&side_inputs, unflowed_room)? {
        Sides::Tagged(tags, groups) => (tags, groups),
        Sides::Refused(refusal) => return Ok(*refusal),
    };

    let mesh = Rc::new(RefCell::new(LevelLineMesh::from_triangles(
        positions,
        indices,
        Some(&tags),
    )?));
    *working = Some(Rc::clone(&mesh));

    {
        let mut m = mesh.borrow_mut();
        // Everything this build decides at a length decides it at the snap.
        m.set_same_place(snap).map_err(defect)?;
        // Needles are repaired out of the mesh BEFORE anything reads it.
        m.needle_floor = needle_floor_of(snap, tolerance);

        // ...but never a needle standing ON the pick, nor a fold the tagging flooded up to.
        let mut on_a_chain: HashSet<(i32, i32)> = HashSet::new();
        for edges in chain_edges.iter() {
            for &(from, to) in edges {
                on_a_chain.insert(triangle_edges::key(from, to));
            }
        }

        let held_folds = Rc::clone(&is_feature_edge);
        m.is_chain_edge = Some(Rc::new(move |a, b| {
            on_a_chain.contains(&triangle_edges::key(a, b)) || held_folds(a, b)
        }));
    }

    // The room of a pick of ONE chain, where none was handed in, is read HERE - on the mesh as it
    // came in, tagged into its two sides.
    let mut read_room: Option<f64> = None;
    let mut read_room_set_by = RoomSource::nothing();
    if only && known_room.is_none() {
        LevelLineWork::enter(LevelLineWorkPhase::CutRoom);
        let (room, set_by) = one_chain_room::read_in_the_build(
            positions,
            indices,
            &chains[0],
            closed[0],
            &mesh,
            setbacks,
            snap,
            seam_epsilon,
            profile,
            fillet_segments,
        )?;
        read_room = Some(room);
        read_room_set_by = set_by;
    }

    // The sizes above the room that build: one open chain's side consumed exactly to its rim.
    let consumed_rims = match read_room {
        Some(room)
            if !closed[0] && size > room && read_room_set_by.kind == RoomReadingKind::Foreign =>
        {
            let largest = unit_setbacks[0]
                .iter()
                .copied()
                .fold(f64::NEG_INFINITY, vector_math::ieee_math::max_f64);
            consumed_to_the_rim::rims_the_band_ends_on(
                &mesh,
                &chain_edges[0],
                &field,
                seam_epsilon,
                size - room,
                room_margin(largest, snap),
                snap,
            )?
        }
        _ => None,
    };
    let ends_on_the_rim = consumed_rims.is_some();

    // The input's precision is applied only from here on; the strips and needles go after the room
    // reading, under their own phase.
    {
        let mut m = mesh.borrow_mut();
        m.input_is_float_stored =
            LevelLineWork::current().is_some_and(|budget| budget.input_is_float_stored);
        LevelLineWork::enter(LevelLineWorkPhase::InputStrips);
        m.remove_input_needle_strips()?;
        m.repair_needles()?;
    }

    // Each group reads ONE field - its pick field, and where a run-out crosses its faces the least
    // of that and the run-out's.
    let input_standing = triangle_edges::standing_map(indices);
    let run_outs = band_gap_extra_fields::gather(
        positions,
        indices,
        chains,
        closed,
        &*is_feature_edge,
        size,
        &tags,
        Some(&input_standing),
    );
    let (group_fields, group_samples) =
        band_group_fields::of(&groups, only, &fields, run_outs.as_ref()).map_err(defect)?;
    let pick_has_corners = !pick_validation::corners_of(chains, closed, true).is_empty();

    // 3. PER GROUP: refine and root the group's own triangles against the group's own reading.
    let mut readings: HashMap<i32, VertexFieldReading> = HashMap::new();
    for (group_at, group) in groups.iter().enumerate() {
        let group_field = Rc::clone(&group_fields[&group.tag]);
        let as_scalar: Rc<dyn ScalarField> = group_field.clone();
        let reading = VertexFieldReading::new(Rc::clone(&mesh), as_scalar, snap).map_err(defect)?;

        LevelLineWork::enter_side_group(group_at as i32, groups.len() as i32);
        LevelLineWork::enter(LevelLineWorkPhase::Refinement);
        let mut candidates =
            band_triangles::candidates(&mesh.borrow(), &*group_field, group.tag, snap)?;
        if !only && pick_has_corners {
            let unwrapped = band_group_fields::unwrapped(&group_field);
            if let Some(members) = unwrapped.as_pick_field() {
                candidates =
                    corner_tie_points::insert(&mut mesh.borrow_mut(), members, candidates, snap)?;
            }
        }

        let group_chain_fields = &fields;
        let member_fields = || {
            group
                .members
                .iter()
                .map(|&m| &*group_chain_fields[m as usize])
        };
        if !only {
            candidates = sharp_joint_corners::insert(
                &reading,
                &*group_field,
                member_fields(),
                candidates,
                snap,
            )?;
        }
        candidates =
            end_clip_corners::insert(&reading, &*group_field, member_fields(), candidates, snap)?;

        FieldCounters::note_refinement_started(group.tag, &group.members, candidates.len() as i32);
        let (settled, offending) = band_refinement::refine(
            &reading,
            &candidates,
            tolerance,
            RefineOptions {
                topology_only,
                ..RefineOptions::default()
            },
        )?;
        if !settled {
            return Ok(BandGapResult {
                refusal: BandGapRefusal::RefinementWouldNotSettle,
                room: 0.0,
                offending_triangle: offending,
                mesh: Some(Rc::clone(&mesh)),
                ..BandGapResult::default()
            });
        }

        // A side the refinement has left with NO surviving vertex loses its loop: only then is its
        // last surviving point searched for and made a vertex - unless a refusal is already coming.
        let refusal_is_coming = read_room.is_some_and(|room| size > room) && !ends_on_the_rim;
        if !refusal_is_coming {
            if let Err(lost) = side_witness::give_the_side_a_witness(
                &reading,
                &*group_field,
                group.tag,
                size,
                tolerance,
                topology_only,
            )? {
                return Ok(BandGapResult {
                    refusal: BandGapRefusal::RefinementWouldNotSettle,
                    room: 0.0,
                    offending_triangle: lost,
                    mesh: Some(Rc::clone(&mesh)),
                    ..BandGapResult::default()
                });
            }
        }

        LevelLineWork::enter(LevelLineWorkPhase::EdgeRoots);
        side_roots_settling::root_the_side(&reading, group.tag)?;
        readings.insert(group.tag, reading);
    }

    // The cut is taken on the FINAL mesh: a later side can split an earlier side's edges.
    let group_tags: Vec<i32> = groups.iter().map(|g| g.tag).collect();
    side_roots_settling::settle_every_side(&group_tags, &readings)?;

    // 4. DELETE: each group's inside triangles that reach one of ITS chains through inside
    // triangles.
    LevelLineWork::enter(LevelLineWorkPhase::BandDelete);
    let mut removed: Vec<i32> = Vec::new();
    for group in &groups {
        let on_the_chain: HashSet<i32> = group
            .members
            .iter()
            .flat_map(|&k| chains[k as usize].iter().copied())
            .collect();
        removed.extend(band_triangles::band_of_side(
            &readings[&group.tag],
            group.tag,
            &on_the_chain,
        )?);
    }

    removed.sort_unstable();

    // 5. COLLISION AND ROOM.
    let mut room;
    let mut room_set_by = RoomSource::nothing();
    if let Some(known) = known_room {
        room = known;
    } else if only {
        room = read_room.expect("a pick of one chain read its room above");
        room_set_by = read_room_set_by;
    } else {
        let pick = match pick_room.take() {
            Some(pick) => pick,
            None => read_the_pick(None)?,
        };
        room = pick.room;
        room_set_by = pick.set_by;
    }

    // A square crease end's section stays on its crease only so far down it.
    if known_room.is_none() {
        square_crease_end_clip::cap_room(
            positions,
            indices,
            chains,
            closed,
            sharp_turn_degrees,
            &mut room,
            &mut room_set_by,
        );
    }

    // ONE number decides: the size asked for is refused exactly when it stands above the room.
    if size > room && !ends_on_the_rim {
        // A flowed band refused by size quotes the room it had without the flow, the one the user
        // is told everywhere else (`unflowed_room`), never the flowed room it was cut by - and the
        // reading that named THAT room, so the place shown is where that number is set.
        return Ok(BandGapResult {
            refusal: BandGapRefusal::Size,
            room: unflowed_room.unwrap_or(room),
            room_set_by: unflowed_set_by.unwrap_or(room_set_by),
            chains: fields,
            mesh: Some(mesh),
            ..BandGapResult::default()
        });
    }

    // A side the strip swallows whole leaves no run, and its made-up one is read off the side
    // before it goes.
    let swallowed_runs = if only && !closed[0] {
        swallowed_side_runs::runs_if_both_swallowed(&mesh.borrow(), &field, profile)?.map(Vec::from)
    } else {
        None
    };
    {
        let mut m = mesh.borrow_mut();
        for &t in &removed {
            m.remove_triangle(t);
        }
    }

    // 6. GAP.
    LevelLineWork::enter(LevelLineWorkPhase::GapLoops);
    let mut loops: Vec<GapLoop> = Vec::new();
    let tie_fields = if !only && pick_has_corners {
        Some(&fields[..])
    } else {
        None
    };

    // Reading a side can find a chain edge not on two triangles mid-walk; the walk's callback
    // cannot return it, so the first such fault is held and leaves once the walk returns.
    let side_fault: RefCell<Option<ChainEdgeNotOnTwoTriangles>> = RefCell::new(None);
    let standing = if groups.iter().any(|g| !g.both_sided.is_empty()) {
        Some(triangle_edges::standing_map(indices))
    } else {
        None
    };
    let side_of_both_sided = standing.as_ref().map(|standing| {
        let chain_edges = &chain_edges;
        let side_fault = &side_fault;
        move |k: i32, point: Vector3| -> i32 {
            match band_side_tagging::side_of(
                positions,
                indices,
                standing,
                &chain_edges[k as usize],
                point,
            ) {
                Ok(side) => side,
                Err(bad) => {
                    side_fault.borrow_mut().get_or_insert(bad);
                    0
                }
            }
        }
    });

    for group in &groups {
        let walked = gap_loop_walk::loops_of_side(
            &mesh.borrow(),
            &*group_samples[&group.tag],
            group,
            tie_fields,
            side_of_both_sided
                .as_ref()
                .map(|f| f as &dyn Fn(i32, Vector3) -> i32),
            consumed_rims.as_deref(),
            snap,
        )?;
        if let Some(bad) = side_fault.borrow_mut().take() {
            return Err(BuildFault::ChainEdge(bad));
        }

        loops.extend(walked);
    }

    let side_loops = gap_loop_walk::what_each_chain_side_left(&groups, closed, &loops);

    // A cut that took NOTHING off and left NO boundary is not a build: there was no crease under
    // the chain to round.
    if only && groups.len() == 1 && (removed.is_empty() || loops.is_empty()) {
        return Ok(BandGapResult {
            refusal: BandGapRefusal::ChainHasNoCreaseToBevel,
            room: 0.0,
            chains: fields,
            mesh: Some(mesh),
            ..BandGapResult::default()
        });
    }

    // 7. PINCH. A side that pinches shows HERE and nowhere else.
    if only
        && closed[0]
        && !topology_only
        && !gap_loop_walk::each_side_leaves_one_closed_loop(&loops)
    {
        let pinch = pinch_ladder_search::pinch_room_below(
            positions,
            indices,
            chains,
            closed,
            Some(Rc::clone(&is_feature_edge)),
            profile,
            snap,
            size,
            sharp_turn_degrees,
        )?;
        if pinch.ran_out_budget.is_some() {
            return Ok(pinch_ladder_search::ladder_ran_out(
                &pinch,
                Some(loops),
                Some(side_loops),
            ));
        }

        return Ok(BandGapResult {
            refusal: BandGapRefusal::Size,
            room: pinch.room,
            pinch_probes: pinch.probes,
            pinches_at_every_size_tried: pinch.pinches_everywhere,
            chains: fields,
            mesh: Some(mesh),
            loops,
            side_loops,
            removed_triangles: removed,
            ..BandGapResult::default()
        });
    }

    if !only {
        let (leaves, failed_at_a_corner) = gap_loop_walk::every_chain_side_leaves_one_owned_loop_at(
            &groups,
            closed,
            &loops,
            Some(chains),
            !topology_only,
        );
        if !leaves {
            if topology_only {
                // This build IS a probe of that ladder; its answer is the refusal.
                return Ok(BandGapResult {
                    refusal: BandGapRefusal::Size,
                    room,
                    room_set_by,
                    chains: fields,
                    mesh: Some(mesh),
                    loops,
                    side_loops,
                    removed_triangles: removed,
                    ..BandGapResult::default()
                });
            }

            if failed_at_a_corner {
                // The corner's shape, the same at every smaller size: nothing for the ladder to find.
                return Ok(BandGapResult {
                    refusal: BandGapRefusal::SidesLeaveNoLoopAtAnySize,
                    room: 0.0,
                    pinches_at_every_size_tried: true,
                    chains: fields,
                    mesh: Some(mesh),
                    loops,
                    side_loops,
                    removed_triangles: removed,
                    ..BandGapResult::default()
                });
            }

            let pinched = pinch_ladder_search::pinch_room_below(
                positions,
                indices,
                chains,
                closed,
                Some(Rc::clone(&is_feature_edge)),
                profile,
                snap,
                size,
                sharp_turn_degrees,
            )?;
            if pinched.ran_out_budget.is_some() {
                return Ok(pinch_ladder_search::ladder_ran_out(
                    &pinched,
                    Some(loops),
                    Some(side_loops),
                ));
            }

            // Every rung pinched: no size builds this pick. Named, so it falls through.
            return Ok(BandGapResult {
                refusal: if pinched.pinches_everywhere {
                    BandGapRefusal::SidesLeaveNoLoopAtAnySize
                } else {
                    BandGapRefusal::Size
                },
                room: if pinched.pinches_everywhere {
                    0.0
                } else {
                    pinched.room
                },
                room_set_by: pinch_ladder_search::who_pinched(Some(&side_loops)),
                pinch_probes: pinched.probes,
                pinches_at_every_size_tried: pinched.pinches_everywhere,
                chains: fields,
                mesh: Some(mesh),
                loops,
                side_loops,
                removed_triangles: removed,
                ..BandGapResult::default()
            });
        }
    }

    let by_tag = |pick: fn(&BandGroup) -> Option<Vec<i32>>| -> IndexMap<i32, Vec<i32>> {
        groups
            .iter()
            .filter_map(|g| pick(g).map(|v| (g.tag, v)))
            .collect()
    };
    let end_members_by_tag = by_tag(|g| (!g.end_members.is_empty()).then(|| g.end_members.clone()));
    let grow_over_by_tag =
        by_tag(|g| (!g.grow_over_members.is_empty()).then(|| g.grow_over_members.clone()));
    let carried_by_tag =
        by_tag(|g| (!g.carried_ends.is_empty()).then(|| g.carried_ends.keys().copied().collect()));

    Ok(BandGapResult {
        refusal: BandGapRefusal::None,
        room,
        room_set_by,
        mesh: Some(mesh),
        chains: fields,
        loops,
        side_loops,
        removed_triangles: removed,
        swallowed_runs,
        end_region_tags: groups
            .iter()
            .filter(|g| g.end_of >= 0)
            .map(|g| g.tag)
            .collect(),
        end_members_by_tag,
        grow_over_by_tag,
        carried_by_tag,
        ..BandGapResult::default()
    })
}
