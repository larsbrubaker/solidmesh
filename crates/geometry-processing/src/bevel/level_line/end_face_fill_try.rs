//! The end-face fill itself: the end faces tried, and the growing end faces tried. Its own file
//! because `end_face_fill.rs` holds the rest of the end-face fill's readings and would pass the
//! 800-line limit;
//! both are re-exported from there. It reads each end's run - the cut's run on the end face
//! (`end_face_fill::run`), a growing end face's crease stretch (`end_face_fill::crease_run`), the
//! end-plane trim's section (`end_plane_trim_run::trim_end`), a mixed pick's nose, a free tip's
//! nose (an end the rolling ball read as free, its rails kept whole and closing onto the end
//! vertex: `LevelLineMesh::ball_free_ends`) or a hull end
//! carried past the part (`virtual_rail_past_the_end::extended`) - and hands the strip and its end
//! runs to the open fill (`open_gap_fill_fill::fill`).

use std::collections::HashSet;

use vector_math::Vector3;

use super::band_side_tagging::{FIRST_END_TAG, SECOND_END_TAG};
use super::end_face_fill::{
    crease_run, is_concave, is_planar, meets_the_sides_convexly, rails_to_the_clip_corner, run,
    runs_end_past_the_end, with_whole_end, without_end_face_edges_at,
};
use super::open_gap_fill_fill::{self, OpenFillEnds};
use super::{
    corner_cap_flat_ends, corner_fill, end_plane_trim_run, hull_clip, open_gap_fill,
    virtual_rail_past_the_end, BandGapResult, BandProfile, ChainField, GapFillRefusal,
    GapFillResult, GapLoop, LevelLineMesh, LevelLineMeshError,
};

/// A flag as the refusal sites print it: `True` or `False`.
fn title_case_bool(value: bool) -> &'static str {
    if value {
        "True"
    } else {
        "False"
    }
}

/// The one loop on `side`, or `None` where there is not exactly one (the hull's carried rails are
/// one per side by construction).
fn single_side(loops: &[GapLoop], side: i32) -> Option<GapLoop> {
    let mut on_side = loops.iter().filter(|l| l.side == side);
    match (on_side.next(), on_side.next()) {
        (Some(only), None) => Some(only.clone()),
        _ => None,
    }
}

/// Whether the chain's `last` (or first) end is closed by the
/// END-PLANE TRIM. A convex chain's end carrying the END RAY stands on a same-sense end face left
/// whole for the trim (flow rung 3); a concave chain's rayed end is trimmed the same way where its
/// end face folds the chain's way too and is bounded (`any_bounded_end`) or both rails run past
/// the end. An end whose rails were carried VIRTUALLY past the growing end face's crease (their
/// ends are `minted` points in the air) is no trim's: the strip runs to the minted row and the end
/// face's plane clips it (StripClip).
#[allow(clippy::too_many_arguments)]
fn trims(
    mesh: &LevelLineMesh,
    chain: &ChainField,
    minted: Option<&HashSet<i32>>,
    left: &GapLoop,
    right: &GapLoop,
    grows: [bool; 2],
    last: bool,
    any_bounded_end: bool,
) -> bool {
    let bound = if last {
        chain.end_ray_bounds.1
    } else {
        chain.end_ray_bounds.0
    };
    let (left_end, right_end) = if last {
        (left.vertices[left.vertices.len() - 1], right.vertices[0])
    } else {
        (left.vertices[0], right.vertices[right.vertices.len() - 1])
    };
    if minted.is_some_and(|m| m.contains(&left_end) || m.contains(&right_end)) {
        return false;
    }

    grows[usize::from(last)]
        && (if last {
            chain.end_rays.1
        } else {
            chain.end_rays.0
        })
        && (!is_concave(mesh, left, right)
            || (bound.is_some_and(|b| b.same_sense)
                && (any_bounded_end || runs_end_past_the_end(mesh, chain, left, right, last))))
}

/// Where a rail's crease run starts: a rail carried VIRTUALLY past the crease
/// (`virtual_rail_past_the_end`) ends on a minted point in the air of the growing end face, so its
/// crease run starts at the vertex before it; the rail's end itself otherwise.
fn off_the_virtual_rail(minted: Option<&HashSet<i32>>, vertices: &[i32], at_tail: bool) -> i32 {
    let n = vertices.len();
    let v = if at_tail {
        vertices[n - 1]
    } else {
        vertices[0]
    };
    if minted.is_some_and(|m| m.contains(&v)) {
        if at_tail {
            vertices[n - 2]
        } else {
            vertices[1]
        }
    } else {
        v
    }
}

/// The fill of an open chain that ends on END FACES - the strip
/// between the two side runs, and at each end the hole between the end run (the cut's run on the
/// end face, the crease stretch of a growing end face, the end-plane trim's section, the nose of
/// a mixed pick, or a hull end) and the strip's last row. `None` (noting the site in
/// `open_gap_fill::ends_site`) where the runs do not chain into one cycle; the caller refuses the
/// open-chain ends by name. `minted` is the set of vertices the virtual rail minted (`None` for
/// none), `hull_minted` the list a hull end's carried rails mint into.
#[allow(clippy::too_many_arguments)]
pub fn try_end_faces(
    gap: &BandGapResult,
    mut left: GapLoop,
    mut right: GapLoop,
    profile: BandProfile,
    fillet_segments: i32,
    minted: Option<&HashSet<i32>>,
    hull_minted: Option<&mut Vec<i32>>,
) -> Result<Option<GapFillResult>, LevelLineMeshError> {
    // A guard against a gap with no mesh: only a built gap is filled, so a gap with no mesh
    // answers no fill.
    let Some(mesh_handle) = gap.mesh.as_ref() else {
        return Ok(None);
    };

    // THE TOPOLOGY CHECK: one open run per side and one per end, chaining into ONE cycle - the
    // kept walk goes left run, end 1, right run, end 0, so end 0 runs from the right run's last
    // vertex to the left run's first and end 1 from the left run's last to the right run's first.
    // An end with NO end region stands on an opposite-sense end face the cut never entered: its
    // run is the crease stretch (see try_growing_end_faces). Each end is read on its own, so one
    // end may grow while the other is cut.
    let grows_first = !gap.loops.iter().any(|l| l.side == FIRST_END_TAG);
    let grows_second = !gap.loops.iter().any(|l| l.side == SECOND_END_TAG);
    let grows = [grows_first, grows_second];
    let (whole_left, whole_right) = (left.clone(), right.clone());
    // A FREE end's nose: an end the rolling ball read as free (no end face, no picked chain
    // meeting it: `LevelLineMesh::ball_free_ends`), where both rails close onto the chain's own end
    // vertex, has no end face, so no edge of a rail there is an end face's crease, though a rail
    // may run along a rim onto an untagged face (a rim the side's contacts rest on): the rails
    // stay whole there, the end is neither trimmed nor a hull, and its run is the nose alone. The
    // rails meeting there is not enough on its own: on a growing end face a rail can take knife
    // crease edges down to the end vertex too.
    let noses = {
        let mesh = mesh_handle.borrow();
        let points = gap.chains[0].points();
        let nose = |free: bool, meets: bool, at: i32, point: Vector3| {
            free && meets && corner_fill::same(mesh.position(at), point)
        };
        [
            grows_first
                && nose(
                    mesh.ball_free_ends[0],
                    left.vertices[0] == right.vertices[right.vertices.len() - 1],
                    left.vertices[0],
                    points[0],
                ),
            grows_second
                && nose(
                    mesh.ball_free_ends[1],
                    left.vertices[left.vertices.len() - 1] == right.vertices[0],
                    right.vertices[0],
                    points[points.len() - 1],
                ),
        ]
    };
    if (grows_first && !noses[0]) || (grows_second && !noses[1]) {
        let mesh = mesh_handle.borrow();
        // Left runs end 0 to end 1, right end 1 to end 0.
        let without_left = without_end_face_edges_at(&mesh, &left, [!noses[0], !noses[1]]);
        let without_right = without_end_face_edges_at(&mesh, &right, [!noses[1], !noses[0]]);
        let (Some(without_left), Some(without_right)) = (without_left, without_right) else {
            open_gap_fill::append_ends_site(" / EndFaceFill:85");
            return Ok(None);
        };
        left = without_left;
        right = without_right;
    }

    let chain: &ChainField = &gap.chains[0];
    let points = chain.points();
    let (point_first, point_last) = (points[0], points[points.len() - 1]);

    // Flat ends on: every same-sense bounded end is trimmed and CARRIED on to its end plane,
    // except where the other end is a HULL end: the hull carries the WHOLE rails on past the end,
    // so such a chain keeps the old dispatch - save a same-sense bounded end, trimmed beside the
    // hull (`beside_a_hull`).
    let mut carries = corner_cap_flat_ends::use_flat_ends();
    let (mut hull_first, mut hull_second) = (false, false);
    if carries && hull_minted.is_some() && is_concave(&mesh_handle.borrow(), &left, &right) {
        let mesh = mesh_handle.borrow();
        let grows_to_a_hull = |last: bool| {
            if !grows[usize::from(last)]
                || trims(&mesh, chain, minted, &left, &right, grows, last, true)
            {
                return false;
            }

            let run = if last {
                crease_run(
                    &mesh,
                    off_the_virtual_rail(minted, &left.vertices, true),
                    off_the_virtual_rail(minted, &right.vertices, false),
                    point_last,
                )
            } else {
                crease_run(
                    &mesh,
                    off_the_virtual_rail(minted, &right.vertices, true),
                    off_the_virtual_rail(minted, &left.vertices, false),
                    point_first,
                )
            };
            match run {
                None => true,
                Some(run) => !is_planar(&mesh, &run),
            }
        };

        hull_first = !noses[0] && grows_to_a_hull(false);
        hull_second = !noses[1] && grows_to_a_hull(true);
        carries = !hull_first && !hull_second;
    }

    // A same-sense bounded end BESIDE a hull end is trimmed all the same: grown instead, its end
    // face was never cut and the cap over the last row lay in that face's plane over its own kept
    // triangles, a coplanar fold. Only the trimmed end changes: the hull end keeps its handling.
    let (beside_a_hull, trims_first, trims_second) = {
        let mesh = mesh_handle.borrow();
        let t = |last: bool, any_bounded_end: bool| {
            trims(
                &mesh,
                chain,
                minted,
                &left,
                &right,
                grows,
                last,
                any_bounded_end,
            )
        };
        let beside_a_hull = [
            !t(false, carries) && hull_second && t(false, true),
            !t(true, carries) && hull_first && t(true, true),
        ];
        (
            beside_a_hull,
            !noses[0] && (t(false, carries) || beside_a_hull[0]),
            !noses[1] && (t(true, carries) || beside_a_hull[1]),
        )
    };

    let mut first = if trims_first {
        end_plane_trim_run::trim_end(
            &mut mesh_handle.borrow_mut(),
            chain,
            &mut left,
            &mut right,
            false,
            profile,
            fillet_segments,
        )?
    } else if noses[0] {
        Some(vec![left.vertices[0]])
    } else if grows_first {
        crease_run(
            &mesh_handle.borrow(),
            off_the_virtual_rail(minted, &right.vertices, true),
            off_the_virtual_rail(minted, &left.vertices, false),
            point_first,
        )
    } else {
        run(
            gap,
            FIRST_END_TAG,
            right.vertices[right.vertices.len() - 1],
            left.vertices[0],
        )
    };
    let mut second = if trims_second {
        end_plane_trim_run::trim_end(
            &mut mesh_handle.borrow_mut(),
            chain,
            &mut left,
            &mut right,
            true,
            profile,
            fillet_segments,
        )?
    } else if noses[1] {
        Some(vec![right.vertices[0]])
    } else if grows_second {
        crease_run(
            &mesh_handle.borrow(),
            off_the_virtual_rail(minted, &left.vertices, true),
            off_the_virtual_rail(minted, &right.vertices, false),
            point_last,
        )
    } else {
        run(
            gap,
            SECOND_END_TAG,
            left.vertices[left.vertices.len() - 1],
            right.vertices[0],
        )
    };

    if !trims_first && !grows_first {
        if let Some(run) = &first {
            let (rail, run) =
                rails_to_the_clip_corner(&mesh_handle.borrow(), chain, &right, run, false);
            right = rail;
            first = Some(run);
        }
    }

    if !trims_second && !grows_second {
        if let Some(run) = &second {
            let (rail, run) =
                rails_to_the_clip_corner(&mesh_handle.borrow(), chain, &right, run, true);
            right = rail;
            second = Some(run);
        }
    }

    // A MIXED pick: one end cut into its end face, the other where the crease CARRIES ON past
    // the pick - there the two side runs meet at one vertex on the continuing ridge, the fade's
    // nose (see open_gap_fill::ends_meet). That end's run is the nose alone.
    if first.is_none()
        && grows_first
        && !trims_first
        && right.vertices[right.vertices.len() - 1] == left.vertices[0]
        && open_gap_fill::on_the_ridge_between(
            &mesh_handle.borrow(),
            left.vertices[0],
            left.side,
            right.side,
        )?
    {
        first = Some(vec![left.vertices[0]]);
    }

    if second.is_none()
        && grows_second
        && !trims_second
        && left.vertices[left.vertices.len() - 1] == right.vertices[0]
        && open_gap_fill::on_the_ridge_between(
            &mesh_handle.borrow(),
            right.vertices[0],
            left.side,
            right.side,
        )?
    {
        second = Some(vec![right.vertices[0]]);
    }

    // A HULL end: a concave chain whose growing end has no crease stretch through the end vertex,
    // because the side runs out of the part. The rails are carried on past the end
    // (virtual_rail_past_the_end) and the part's supporting planes clip the strip and are capped
    // there (hull_clip).
    let mut hull_ends = [false, false];
    let (crease_first, crease_second) = (first.clone(), second.clone());

    // A growing end's crease run that does not lie in one plane is no end face's crease: it turns
    // round the part's own corner, and the hole behind it is in two planes, which no zip about
    // the end vertex closes.
    if hull_minted.is_some()
        && grows_first
        && !trims_first
        && first
            .as_ref()
            .is_some_and(|run| !is_planar(&mesh_handle.borrow(), run))
    {
        first = None;
    }

    if hull_minted.is_some()
        && grows_second
        && !trims_second
        && second
            .as_ref()
            .is_some_and(|run| !is_planar(&mesh_handle.borrow(), run))
    {
        second = None;
    }

    hull_clip::note(|| {
        format!(
            "end faces: grows {}/{}, runs {}/{}, trims {}/{}, concave {}",
            title_case_bool(grows_first),
            title_case_bool(grows_second),
            title_case_bool(first.is_some()),
            title_case_bool(second.is_some()),
            title_case_bool(trims_first),
            title_case_bool(trims_second),
            title_case_bool(is_concave(&mesh_handle.borrow(), &left, &right))
        )
    });
    if let Some(hull_minted) = hull_minted {
        if ((first.is_none() && grows_first && !trims_first)
            || (second.is_none() && grows_second && !trims_second))
            && is_concave(&mesh_handle.borrow(), &left, &right)
        {
            let before = hull_minted.len();
            // The hull carries the WHOLE rails on past its end. Where the other end was trimmed
            // beside it (beside_a_hull), the rails keep the trim's cut at that end and the whole
            // run at the hull's.
            let rails = if beside_a_hull[0] {
                vec![
                    with_whole_end(&left, &whole_left, true),
                    with_whole_end(&right, &whole_right, false),
                ]
            } else if beside_a_hull[1] {
                vec![
                    with_whole_end(&left, &whole_left, false),
                    with_whole_end(&right, &whole_right, true),
                ]
            } else {
                vec![whole_left.clone(), whole_right.clone()]
            };
            let extended =
                virtual_rail_past_the_end::extended(gap, &rails, profile, hull_minted, false)?;
            if hull_minted.len() > before {
                let (Some(new_left), Some(new_right)) = (
                    single_side(&extended, left.side),
                    single_side(&extended, right.side),
                ) else {
                    return Ok(None);
                };
                if first.is_none() && hull_minted.contains(&new_left.vertices[0]) {
                    hull_ends[0] = true;
                    first = Some(Vec::new());
                }

                if second.is_none()
                    && hull_minted.contains(&new_left.vertices[new_left.vertices.len() - 1])
                {
                    hull_ends[1] = true;
                    second = Some(Vec::new());
                }

                left = new_left;
                right = new_right;
            }
        }
    }

    // Where no hull end was made, a non-planar crease run is handed back as it was: its zip
    // decides.
    if first.is_none() {
        first = crease_first;
    }

    if second.is_none() {
        second = crease_second;
    }

    let (first_null, second_null) = (first.is_none(), second.is_none());
    let (Some(first), Some(second)) = (first, second) else {
        let end_runs = |tag: i32| {
            gap.loops
                .iter()
                .filter(|l| l.side == tag)
                .map(|l| {
                    format!(
                        "{}{}",
                        if l.is_closed { "closed" } else { "open" },
                        l.vertices.len()
                    )
                })
                .collect::<Vec<_>>()
                .join("+")
        };
        let concave = is_concave(&mesh_handle.borrow(), &left, &right);
        open_gap_fill::append_ends_site(&format!(
            " / EndFaceFill:191 first {} second {}, grows {}/{}, trims {}/{}, concave {}, end runs [{}] [{}], rays {}/{}, meet {}/{}",
            if first_null { "null" } else { "ok" },
            if second_null { "null" } else { "ok" },
            title_case_bool(grows_first),
            title_case_bool(grows_second),
            title_case_bool(trims_first),
            title_case_bool(trims_second),
            title_case_bool(concave),
            end_runs(FIRST_END_TAG),
            end_runs(SECOND_END_TAG),
            title_case_bool(chain.end_rays.0),
            title_case_bool(chain.end_rays.1),
            title_case_bool(right.vertices[right.vertices.len() - 1] == left.vertices[0]),
            title_case_bool(left.vertices[left.vertices.len() - 1] == right.vertices[0]),
        ));
        return Ok(None);
    };

    let clips = minted.is_some_and(|m| {
        m.contains(&left.vertices[0]) || m.contains(&left.vertices[left.vertices.len() - 1])
    });

    // A CONCAVE chain's cut end cap is the hole between the end run and the last row, and it lies
    // in the end face only where the end face holds the last row. An OBLIQUE end face does not,
    // and zipped anyway the cap leaves the face; refused by name until the oblique-end rung
    // builds it. A square end face, a growing one and every convex end cap are unchanged.
    {
        let mesh = mesh_handle.borrow();
        if is_concave(&mesh, &left, &right)
            && ((!grows_first && meets_the_sides_convexly(&mesh, FIRST_END_TAG, point_first))
                || (!grows_second && meets_the_sides_convexly(&mesh, SECOND_END_TAG, point_last)))
        {
            open_gap_fill::append_ends_site(" / EndFaceFill:217");
            return Ok(None);
        }
    }

    let end_runs = [first, second];
    let filled = open_gap_fill_fill::fill(
        mesh_handle,
        chain,
        &left,
        &right,
        profile,
        fillet_segments,
        &mut Vec::new(),
        OpenFillEnds {
            end_runs: Some(&end_runs),
            rows_on_the_ray: trims_first || trims_second,
            clip_ends: clips,
            hull_ends: Some(hull_ends),
            carried_ends: if corner_cap_flat_ends::use_flat_ends() {
                Some([trims_first, trims_second])
            } else {
                None
            },
            flowed: gap.unflowed_room.is_some(),
        },
    )?;
    Ok(Some(if filled.filled() {
        let (triangles, vertices) = (
            filled.added_triangles.len() as i32,
            filled.added_vertices.len() as i32,
        );
        GapFillResult {
            refusal: GapFillRefusal::None,
            mesh: filled.mesh,
            added_triangles: filled.added_triangles,
            added_vertices: filled.added_vertices,
            triangles_of_chain: vec![triangles],
            vertices_of_chain: vec![vertices],
            ends_capped_by_end_faces: true,
            ..GapFillResult::default()
        }
    } else {
        filled
    }))
}

/// The fill of an open chain whose end faces are of the
/// OPPOSITE sense: the end face was never cut, so each end run is the CREASE STRETCH the cut
/// uncovered, and the cap between it and the last row lies in the end face's plane, grown over the
/// bead's section. `None` when either end has no such stretch.
pub fn try_growing_end_faces(
    gap: &BandGapResult,
    left: GapLoop,
    right: GapLoop,
    profile: BandProfile,
    fillet_segments: i32,
    minted: Option<&HashSet<i32>>,
) -> Result<Option<GapFillResult>, LevelLineMeshError> {
    try_end_faces(gap, left, right, profile, fillet_segments, minted, None)
}
