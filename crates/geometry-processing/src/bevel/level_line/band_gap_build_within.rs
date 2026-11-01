//! The band gap's build proper (split out of `band_gap.rs` for size). Steps 1 to 7 of the level
//! line construction - setback, sides (step 2 is `band_gap_build_sides.rs`'s),
//! refinement and roots per group, the band's delete, the room, the gap loops and the pinch -
//! leaving nothing to do but fill. Its faults travel as [`BuildFault`] (`band_gap_build_fault.rs`), which
//! [`band_gap::build_unvalidated`](super::band_gap::build_unvalidated) turns into the caller's
//! outcomes. Step 1's setbacks and fields are `band_gap_setback_fields.rs`'s.
//!
//! Behind the rolling ball's switch, the outer pass's step 1 fields at the size take their
//! setbacks from the ball's contacts (`rolling_ball_setbacks.rs`): one field per side per chain and
//! the chain's own field, the larger of the two sides at each point.
//! - **Per side:** each side group's reading for the chains it is a side of
//!   (`band_group_fields::of_sides`), and with it that group's refinement, end clip and sharp-joint
//!   corners, witness and roots - everything that places a side's rail. A pass of one chain whose
//!   side floods join its two sides into one region is tagged from the ball's own sides
//!   (`rolling_ball_band_tags`, in step 2), so each side is a group of its own there too.
//! - **Per end:** a single open chain's end region whose sides differ at its end reads its cap
//!   field (`PassSetbacks::caps_of_one_open_chain`), cut to meet each side's rail.
//! - **Per side, judged:** whether the band ends on a rim (`consumed_to_the_rim`, each edge read
//!   with the field of the group its triangles belong to) and the swallowed sides' made-up runs
//!   (`swallowed_side_runs::runs_if_both_swallowed_per_side`, each side's run at its own setback).
//! - **The chain's own field, as a reach or a tolerance:** the tagging's reach, the narrow end
//!   face's flow, which foreign edges the band reaches, the swallowed runs' walls, stations and
//!   margin, the loop walk's ties and the result's `chains` (which the fill reads for its rows'
//!   chain points, end rays and tolerances). The larger side is how far the band can reach; it is
//!   never read as where one side's rail stands: the result carries the side fields
//!   (`side_chains`), which the rail joined past a rib reads (`virtual_rail::joined_sides`). The
//!   fill's rail carried past an oblique end reads one setback for both sides, so the ball takes no
//!   open chain whose sides differ and which has an end ray
//!   (`rolling_ball_setbacks::unless_a_rayed_end_differs`).
//! - **The room of a pass of one chain:** read in the build with the ball's setbacks at the size
//!   asked, scaled (`one_chain_room`, whose header says why): `field_at_0`, the room's convex-ray
//!   field, the narrow end face's flow and the room's margin. Such a pass is handed no room
//!   (`band_gap::build_unvalidated` sets a caller's aside where the ball may take the pass).
//! - **The level line's own setbacks:** the probes and the room of a pass of several chains (its
//!   pair and corner readings). There a closed chain whose sides differ can read a room that names
//!   a size the ball's band does not fit (a side whose contact stands farther out than the angle
//!   formula says), and such a size is then not refused by its room.

use std::cell::RefCell;
use std::collections::{HashMap, HashSet};
use std::rc::Rc;

use indexmap::IndexMap;
use vector_math::Vector3;

use super::band_gap::{chord_tolerance_of, needle_floor_of, BandGapBuildError, BuildWithinArgs};
use super::band_gap_build_fault::{defect, BuildFault};
use super::band_gap_build_sides::{self, SideInputs, Sides};
use super::band_gap_reading::room_margin;
use super::band_gap_setback_fields::PassSetbacks;
use super::band_refinement::{self, RefineOptions};
use super::seam_edges::FeatureEdgeTest;
use super::{
    band_gap_extra_fields, band_group_fields, band_room_pair_room_pick, band_side_tagging,
    band_triangles, consumed_to_the_rim, corner_tie_points, end_clip_corners, gap_loop_walk,
    one_chain_room, pick_validation, pinch_ladder_search, rolling_ball_setbacks,
    sharp_joint_corners, side_roots_settling, side_witness, square_crease_end_clip,
    swallowed_side_runs, triangle_edges,
};
use super::{
    BandGapRefusal, BandGapResult, BandGroup, ChainEdgeNotOnTwoTriangles, FieldCounters, GapLoop,
    LevelLineMesh, LevelLineWork, LevelLineWorkPhase, RoomReadingKind, RoomSource, ScalarField,
    VertexFieldReading,
};

/// The build proper. `working` receives the working mesh as soon as it is made, so a refusal out
/// of the middle of the build can still hand it back; `unflowed_room` receives the plain
/// tagging's room where the flowed tagging is cut instead (see
/// [`tag_flowed_where_it_builds`](super::narrow_face_flow::tag_flowed_where_it_builds)).
#[allow(clippy::too_many_arguments)]
pub fn build_within(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    is_feature_edge: Option<FeatureEdgeTest>,
    args: &BuildWithinArgs,
    working: &mut Option<Rc<RefCell<LevelLineMesh>>>,
    unflowed_room: &mut Option<f64>,
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
        side_regions,
    } = *args;

    if chains.iter().any(|one| one.len() < 2) {
        return Err(BuildFault::Error(BandGapBuildError::Arguments(
            pick_validation::PickArgumentError::ChainTooShort,
        )));
    }

    if size.is_nan() || size <= 0.0 {
        return Err(BuildFault::Error(BandGapBuildError::SizeNotPositive(size)));
    }

    // 1. SETBACK, per chain.
    let setbacks = PassSetbacks::of(
        positions,
        indices,
        chains,
        closed,
        sharp_turn_degrees,
        profile,
    )?;
    // The field at the size takes the rolling ball's setbacks where its switch is on, this is the
    // outer pass and no topology-only probe, and the ball takes the whole pass; probes, the room
    // and every other size keep the level line's own, and only the outer pass writes the route.
    let pass = rolling_ball_setbacks::PassScope::enter();
    let ball_pass = rolling_ball_setbacks::with_the_sides(
        positions,
        indices,
        chains,
        closed,
        is_feature_edge.as_deref(),
        side_regions,
        size,
        profile,
        &setbacks,
        topology_only || !pass.is_outer(),
    );
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
    } = &setbacks;
    let (ball_setbacks, ball_sides) = match ball_pass {
        Some((setbacks, sides)) => (Some(setbacks), Some(sides)),
        None => (None, None),
    };
    let ball_setbacks =
        rolling_ball_setbacks::unless_a_rayed_end_differs(ball_setbacks, closed, end_rays, size);
    // The ball's sides tag a one-chain band only where its setbacks are kept.
    let ball_sides = ball_sides.filter(|_| only && ball_setbacks.is_some());
    let (fields, side_fields) = setbacks.fields_at_the_size(ball_setbacks.as_deref(), size)?;
    let caps = setbacks.caps_of_one_open_chain(ball_setbacks.as_deref(), size)?;

    // Scaling the setbacks moves no point, so a polyline that built a field at the size builds it
    // at every size. A pass of one chain the ball takes reads its room with the ball's setbacks
    // (`one_chain_room`); every other pass with the angle's.
    let ball_room_units = one_chain_room::the_units(ball_setbacks.as_deref(), only);
    let room_units: &[f64] = ball_room_units.as_deref().unwrap_or(&unit_setbacks[0]);
    let field_at_0 = |s: f64| {
        setbacks
            .field_from(0, room_units, s)
            .expect("the chain's polyline built a field at the size asked")
    };
    let field = Rc::clone(&fields[0]);

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
        field: &field,
        field_at_0: &field_at_0,
        room_units,
        known_room,
        size,
        snap,
        seam_epsilon,
        profile,
        fillet_segments,
        step_corners,
        ball_sides: ball_sides.as_ref().map(|sides| &sides[0]),
    };
    let (tags, groups, sides_from_the_ball) =
        match band_gap_build_sides::tag_the_sides(&side_inputs, unflowed_room)? {
            Sides::Tagged(tags, groups, from_the_ball) => (tags, groups, from_the_ball),
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
        m.sides_from_the_ball = sides_from_the_ball;

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
            &setbacks,
            room_units,
            one_chain_room::the_sides(ball_setbacks.as_deref(), only, &tags),
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
            let largest = room_units
                .iter()
                .copied()
                .fold(f64::NEG_INFINITY, vector_math::ieee_math::max_f64);
            consumed_to_the_rim::rims_the_band_ends_on(
                &mesh,
                &chain_edges[0],
                &field,
                &|tag| {
                    band_group_fields::one_chains_field(tag, &field, side_fields.as_deref(), &caps)
                },
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
    let (group_fields, group_samples) = band_group_fields::of_sides(
        &groups,
        only,
        &fields,
        side_fields.as_deref(),
        Some(&caps),
        run_outs.as_ref(),
    )
    .map_err(defect)?;
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

        let group_chain_fields = band_group_fields::chain_fields_of_group(
            group,
            &fields,
            side_fields.as_deref(),
            Some(&caps),
        );
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
        LevelLineWork::enter(LevelLineWorkPhase::CutRoom);
        let pick = band_room_pair_room_pick::room_of_pick(
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
        )?;
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
        return Ok(BandGapResult {
            refusal: BandGapRefusal::Size,
            room,
            room_set_by,
            chains: fields,
            mesh: Some(mesh),
            ..BandGapResult::default()
        });
    }

    // A side the strip swallows whole leaves no run, and its made-up one is read off the side
    // before it goes.
    let swallowed_runs = if only && !closed[0] {
        let sides = [1, 2].map(|tag| {
            band_group_fields::one_chains_field(tag, &field, side_fields.as_deref(), &caps)
        });
        swallowed_side_runs::runs_if_both_swallowed_per_side(
            &mesh.borrow(),
            &field,
            [&*sides[0], &*sides[1]],
            profile,
        )?
        .map(Vec::from)
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
        side_chains: side_fields,
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
