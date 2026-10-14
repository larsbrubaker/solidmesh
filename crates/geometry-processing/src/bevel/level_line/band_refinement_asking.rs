//! The refinement when the field can be ASKED where it runs: no halving anywhere, only splits at
//! points the field itself names. `band_refinement` routes to it, the bump rule is
//! `edge_bump_rule` and the mixed edge rule `band_refinement_mixed_edge`.
//!
//! The rules, in order: (i) an edge dipping below the line between two outside ends is split once
//! at its least point; its mirror, the bump, and its third face, the mixed edge; (ii) a hidden loop
//! inside a triangle with every corner outside; (iv) an island of kept material inside a triangle
//! with no corner outside; (v) a ridge through a crossed triangle's middle; (iii) the chord of a
//! crossed triangle held to the chord tolerance. Each fires on an EXACT reading at the point it
//! inserts, so no edge is divided twice by one rule; a split that would thin a child past the
//! sliver limit bisects the longest-edge path instead. The only bound is the work budget; a
//! triangle already under the accuracy anyone asked for that a rule still fires on is refused by
//! name.

use std::collections::{HashMap, HashSet, VecDeque};

use vector_math::{cs_math, Vector3};

use super::band_refinement::{
    farthest_from_the_line, farthest_from_the_line3, line_points_of, nearest_to_the_line,
    nearest_to_the_line3, span, RefineError,
};
use super::band_refinement_mixed_edge::ask_mixed_edges;
use super::edge_bump_rule::{self, BumpAnswer};
use super::field_bounds;
use super::line_walk::try_find_line_point_near;
use super::needle_apex_retile::try_retile;
use super::refinement_splits::{
    cannot_be_divided_further, has_no_height, longest_edge, meter_would_thin,
    meter_would_thin_on_edge, NOTHING_TO_INSERT,
};
use super::refinement_splits_enqueue::{
    bisect_instead_of_thinning, bisect_instead_of_thinning_on_edge, insert_on_the_line,
    insert_where_it_stands, split_edge_and_enqueue,
};
use super::{
    ChainDistanceField, FieldCounters, FieldMaximum, LevelLineMeshError, LevelLineRule,
    LevelLineWork, VertexFieldReading, VertexSide,
};

/// A rule's point inside a triangle, put in the way every triangle rule puts it: bisected along the
/// longest-edge path where it would thin a child, otherwise inserted where it stands.
fn insert_or_bisect(
    reading: &VertexFieldReading,
    rule: LevelLineRule,
    triangle: i32,
    point: Vector3,
    chord_tolerance: f64,
    pending: &mut VecDeque<i32>,
    depth: &mut HashMap<i32, i32>,
) -> Result<(), LevelLineMeshError> {
    let mut mesh = reading.mesh.borrow_mut();
    meter_would_thin(rule, &mesh, triangle, point, chord_tolerance);
    if !bisect_instead_of_thinning(&mut mesh, triangle, point, chord_tolerance, pending, depth)? {
        insert_where_it_stands(&mut mesh, triangle, point, pending, depth, None)?;
    }

    Ok(())
}

/// See the module header. Answers `(true, -1)` when every
/// triangle settled and `(false, triangle)` for a triangle under the floor a rule still fires on.
pub fn refine_by_asking(
    reading: &VertexFieldReading,
    field: &dyn ChainDistanceField,
    candidate_triangles: &[i32],
    chord_tolerance: f64,
    topology_only: bool,
) -> Result<(bool, i32), RefineError> {
    // A split is NOT a halving, so a generation count says nothing; the honest end of the road is
    // geometric: a triangle already smaller than the accuracy asked for cannot be improved.
    let floor = cs_math::max_f64(4.0 * reading.snap, 1e-3 * chord_tolerance);
    let snap = reading.snap;

    // The mixed edge rule's memo is only good against the field it was filled for, and a
    // many-loop pick refines the same mesh once per GROUP, so the memo starts empty every call.
    reading.mesh.borrow_mut().forget_clean_edges();
    let mut depth: HashMap<i32, i32> = HashMap::new();
    let mut retiled: HashSet<(i32, i32)> = HashSet::new();
    let mut taken = 0i64;
    let distinct: indexmap::IndexSet<i32> = candidate_triangles.iter().copied().collect();
    let mut pending: VecDeque<i32> = distinct.into_iter().collect();
    while let Some(triangle) = pending.pop_front() {
        taken += 1;
        LevelLineWork::note_refinement_queue(taken, pending.len() as i64);

        // One unit per DEQUEUE (and a dequeue is not a bounded amount of memory).
        LevelLineWork::charge(1)?;
        let (count, removed) = {
            let mesh = reading.mesh.borrow();
            (
                mesh.triangle_count(),
                triangle < mesh.triangle_count() && mesh.is_removed(triangle),
            )
        };
        if triangle >= count {
            continue;
        }

        let here = depth.get(&triangle).copied().unwrap_or(0);
        FieldCounters::note_refine_step(count, pending.len() as i32, here);
        if removed {
            // A needle repaired away while this worklist was filled; its replacements are queued.
            continue;
        }

        let t = reading.mesh.borrow().triangles()[triangle as usize];

        // A triangle with no HEIGHT carries no surface, and dividing it only makes more of the
        // same: the dip and bump rules chase each other for ever on one (Thingi10K 95498). Only
        // the mixed edge rule is let past this gate, and the re-tiling through an apex already on
        // the long edge, which mints and adds nothing.
        let no_height = has_no_height(&reading.mesh.borrow(), t, floor);
        if no_height
            && try_retile(
                &mut reading.mesh.borrow_mut(),
                t,
                &mut retiled,
                &mut pending,
                &mut depth,
                here,
            )?
        {
            continue;
        }

        // (i) an edge that dips across the line between two outside ends.
        let mut dipped = false;
        let mut corner = 0;
        while corner < 3 && !dipped && !no_height {
            let v = t.corner(corner);
            let w = t.corner((corner + 1) % 3);
            corner += 1;
            if reading.side_of(v)? != VertexSide::Outside
                || reading.side_of(w)? != VertexSide::Outside
            {
                continue;
            }

            // The edge cannot reach down to the line from where its ends read.
            let (from, to) = {
                let mesh = reading.mesh.borrow();
                (mesh.position(v), mesh.position(w))
            };
            let (value_v, value_w) = (reading.value_at(v)?, reading.value_at(w)?);
            if field_bounds::least_along_edge(value_v, value_w, (to - from).length()) >= -snap {
                continue;
            }

            if FieldCounters::meter_on() {
                FieldCounters::asked_by_reaching(
                    LevelLineRule::EdgeDip,
                    (from - to).length() / (20.0 * chord_tolerance),
                    nearest_to_the_line(chord_tolerance, value_v, value_w),
                    farthest_from_the_line(chord_tolerance, value_v, value_w),
                );
            }

            let least = field.min_along_segment(from, to)?;
            if least.value >= -snap {
                continue;
            }

            // The rule fires on the point it would INSERT, read exactly, which makes the inserted
            // vertex Inside by construction whatever drift the search carries.
            if reading.value_at_point(least.point)? >= -snap {
                continue;
            }

            if cannot_be_divided_further(&reading.mesh.borrow(), triangle, floor) {
                return Ok((false, triangle));
            }

            FieldCounters::fired(LevelLineRule::EdgeDip);

            // A dip within a snap of one end is refused by the mesh and nothing is split - a
            // crossing that near an end is below the accuracy the caller asked for.
            let mut mesh = reading.mesh.borrow_mut();
            meter_would_thin_on_edge(
                LevelLineRule::EdgeDip,
                &mesh,
                v,
                w,
                least.point,
                chord_tolerance,
            );
            if !bisect_instead_of_thinning_on_edge(
                &mut mesh,
                triangle,
                v,
                w,
                least.point,
                chord_tolerance,
                &mut pending,
                &mut depth,
            )? {
                split_edge_and_enqueue(&mut mesh, v, w, least.point, &mut pending, &mut depth)?;
            }

            dipped = true;
        }

        if dipped {
            continue;
        }

        // (i, mirrored) an edge that bumps across the line between inside ends.
        if !no_height {
            match edge_bump_rule::ask(
                reading,
                field,
                triangle,
                t,
                chord_tolerance,
                floor,
                &mut pending,
                &mut depth,
            )? {
                BumpAnswer::Refused => return Ok((false, triangle)),
                BumpAnswer::Split => continue,
                BumpAnswer::Nothing => {}
            }
        }

        // (i, third face) an edge whose ends read DIFFERENT sides, which can still hide a PAIR of
        // crossings past its first root.
        match ask_mixed_edges(
            reading,
            field,
            triangle,
            t,
            chord_tolerance,
            floor,
            &mut pending,
            &mut depth,
        )? {
            BumpAnswer::Split => continue,
            BumpAnswer::Refused => return Ok((false, triangle)),
            BumpAnswer::Nothing => {}
        }

        // Everything below is behind the no-height gate.
        if no_height {
            continue;
        }

        let (pa, pb, pc) = {
            let mesh = reading.mesh.borrow();
            (mesh.position(t.a), mesh.position(t.b), mesh.position(t.c))
        };
        let side_a = reading.side_of(t.a)?;
        let side_b = reading.side_of(t.b)?;
        let side_c = reading.side_of(t.c)?;

        // (ii) a loop lying wholly inside a triangle whose three corners are all outside.
        if side_a == VertexSide::Outside
            && side_b == VertexSide::Outside
            && side_c == VertexSide::Outside
        {
            let (va, vb, vc) = (
                reading.value_at(t.a)?,
                reading.value_at(t.b)?,
                reading.value_at(t.c)?,
            );

            // Nothing in the triangle can reach down to the line from what its corners read.
            if field_bounds::least_over_triangle(va, vb, vc, pa, pb, pc) >= -snap {
                continue;
            }

            if field.nothing_reaches_below(
                Vector3::component_min(Vector3::component_min(pa, pb), pc),
                Vector3::component_max(Vector3::component_max(pa, pb), pc),
                -snap,
            ) {
                continue;
            }

            if FieldCounters::meter_on() {
                FieldCounters::asked_by_reaching(
                    LevelLineRule::HiddenLoop,
                    span(pa, pb, pc) / (20.0 * chord_tolerance),
                    nearest_to_the_line3(chord_tolerance, va, vb, vc),
                    farthest_from_the_line3(chord_tolerance, va, vb, vc),
                );
            }

            let least = field.min_over_triangle(pa, pb, pc)?;
            if least.value < -snap && reading.value_at_point(least.point)? < -snap {
                if cannot_be_divided_further(&reading.mesh.borrow(), triangle, floor) {
                    return Ok((false, triangle));
                }

                // The least may stand ON an edge or at a corner; the router splits the edge, or
                // leaves the corner alone, rather than making three slivers or a duplicate.
                FieldCounters::fired(LevelLineRule::HiddenLoop);
                insert_or_bisect(
                    reading,
                    LevelLineRule::HiddenLoop,
                    triangle,
                    least.point,
                    chord_tolerance,
                    &mut pending,
                    &mut depth,
                )?;
            }

            continue;
        }

        // (iv) the mirror of (ii): a triangle with no outside corner can still hide an ISLAND of
        // kept material in its middle, found by bounded search seeded a tolerance below the snap.
        if side_a != VertexSide::Outside
            && side_b != VertexSide::Outside
            && side_c != VertexSide::Outside
        {
            let (va, vb, vc) = (
                reading.value_at(t.a)?,
                reading.value_at(t.b)?,
                reading.value_at(t.c)?,
            );

            // No island is possible where the corners' own readings cannot rise to the line.
            if field_bounds::greatest_over_triangle(va, vb, vc, pa, pb, pc) <= snap {
                continue;
            }

            if FieldCounters::meter_on() {
                FieldCounters::asked_by_reaching(
                    LevelLineRule::Island,
                    span(pa, pb, pc) / (20.0 * chord_tolerance),
                    nearest_to_the_line3(chord_tolerance, va, vb, vc),
                    farthest_from_the_line3(chord_tolerance, va, vb, vc),
                );
            }

            let greatest = field.island_peak(pa, pb, pc, chord_tolerance, snap)?;
            if greatest.value > snap {
                if cannot_be_divided_further(&reading.mesh.borrow(), triangle, floor) {
                    return Ok((false, triangle));
                }

                FieldCounters::fired(LevelLineRule::Island);
                insert_or_bisect(
                    reading,
                    LevelLineRule::Island,
                    triangle,
                    greatest.point,
                    chord_tolerance,
                    &mut pending,
                    &mut depth,
                )?;
            }

            continue;
        }

        // (v) a crossed triangle whose INTERIOR stands higher than any of its corners by more than
        // the chord tolerance: the ridge of kept surface the chord rule cannot see.
        let value_a = reading.value_at(t.a)?;
        let value_b = reading.value_at(t.b)?;
        let value_c = reading.value_at(t.c)?;
        let highest_corner = cs_math::max_f64(value_a, cs_math::max_f64(value_b, value_c));

        // The ridge has to beat BOTH thresholds, so a bound under the higher settles it without
        // the search; the search is handed that same constant threshold and compared strictly.
        let must_beat = cs_math::max_f64(snap, highest_corner + chord_tolerance);
        let could_ridge =
            field_bounds::greatest_over_triangle(value_a, value_b, value_c, pa, pb, pc) > must_beat;
        FieldCounters::asked_by_reaching(
            if could_ridge {
                LevelLineRule::Ridge
            } else {
                LevelLineRule::None
            },
            span(pa, pb, pc) / (20.0 * chord_tolerance),
            nearest_to_the_line3(chord_tolerance, value_a, value_b, value_c),
            farthest_from_the_line3(chord_tolerance, value_a, value_b, value_c),
        );
        let peak = if could_ridge {
            field.max_over_triangle_bounded(
                pa,
                pb,
                pc,
                chord_tolerance,
                f64::INFINITY,
                must_beat,
            )?
        } else {
            FieldMaximum::new(f64::NEG_INFINITY, pa)
        };
        if peak.value > must_beat {
            if cannot_be_divided_further(&reading.mesh.borrow(), triangle, floor) {
                return Ok((false, triangle));
            }

            // A ridge peak at an existing corner (a fan's apex) needs no vertex: the corner IS it.
            FieldCounters::fired(LevelLineRule::Ridge);
            insert_or_bisect(
                reading,
                LevelLineRule::Ridge,
                triangle,
                peak.point,
                chord_tolerance,
                &mut pending,
                &mut depth,
            )?;
            continue;
        }

        // (iii) the chord of a crossed triangle, held to the chord tolerance - the ONE rule that
        // only moves a line already drawn closer to the true one.
        if topology_only {
            continue;
        }

        FieldCounters::asked_by_reaching(
            LevelLineRule::ChordAccuracy,
            span(pa, pb, pc) / (20.0 * chord_tolerance),
            nearest_to_the_line3(chord_tolerance, value_a, value_b, value_c),
            farthest_from_the_line3(chord_tolerance, value_a, value_b, value_c),
        );
        let points = line_points_of(reading, triangle)?;
        if points.len() != 2 {
            continue;
        }

        let chord = (points[0] - points[1]).length();
        let chord_mid = 0.5 * (points[0] + points[1]);
        // A chord whose ends are ON the line cannot have its middle stand further off it than half
        // its own length, so a chord barely over the tolerance is accurate without asking.
        if chord <= chord_tolerance
            || snap + (0.5 * chord) <= chord_tolerance
            || reading.value_at_point(chord_mid)?.abs() <= chord_tolerance
        {
            continue;
        }

        if cannot_be_divided_further(&reading.mesh.borrow(), triangle, floor) {
            return Ok((false, triangle));
        }

        let on_the_line = {
            let mesh = reading.mesh.borrow();
            try_find_line_point_near(&mesh, reading, triangle, chord_mid, chord)?
        };
        let Some(on_the_line) = on_the_line else {
            // No way to the line from the midpoint - a RIDGE has no slope - so the triangle is
            // halved instead of accepted; the halves stand off the ridge.
            let mut mesh = reading.mesh.borrow_mut();
            let (from, to) = longest_edge(&mesh, triangle);
            let middle = 0.5 * (mesh.position(from) + mesh.position(to));
            split_edge_and_enqueue(&mut mesh, from, to, middle, &mut pending, &mut depth)?;
            continue;
        };

        FieldCounters::fired(LevelLineRule::ChordAccuracy);
        let added = insert_on_the_line(
            &mut reading.mesh.borrow_mut(),
            triangle,
            on_the_line,
            snap,
            &mut pending,
            &mut depth,
        )?;
        if added != NOTHING_TO_INSERT {
            reading.put_on_the_line(added)?;
        }
    }

    Ok((true, -1))
}
