//! The gap fill for an OPEN run: one open run read as a rail ([`OpenGapFillOpenRail`]), the open
//! fill's constants, the thread's refusal site ([`ends_site`]), the
//! straight-row test ([`is_straight_row`]) and the small pieces of the fill: the hole between two
//! runs ([`hole_between`]), a result carrying the minted vertices ([`with_minted`]), a run end's
//! meeting point ([`meeting_point_of`]), a reversed arc ([`backwards`]), a run read as a rail
//! ([`try_rail`]), the extended parameter past an end ([`extended`]) and the rows laid along the
//! spokes ([`arcs`]), and the entry points that route an open chain's gap ([`try_open`]) and fill
//! the fade ([`try_fade`]). The fill itself is in `open_gap_fill_fill.rs`, kept apart for
//! the file-size limit.

use std::cell::RefCell;
use std::collections::HashSet;
use std::rc::Rc;

use vector_math::{cs_math, Vector3};

use super::open_gap_fill_fill::{self, OpenFillEnds};
use super::{
    corner_cap_flat_ends, dying_crease_fill, end_face_fill, gap_fill, open_gap_fill_cut_section,
    open_gap_fill_stations, virtual_rail, virtual_rail_past_the_end, BandGapResult, BandProfile,
    ChainField, GapFillRefusal, GapFillResult, GapLoop, LevelLineMesh, LevelLineMeshError,
};

/// A watcher of the fade's hole, as directed edges.
pub type WatchFadeHole = Rc<dyn Fn(&[(i32, i32)])>;

thread_local! {
    /// Which open-end site last refused the open-chain
    /// ends - or, for a refused growing-neighbour clip, which strip-clip stage - on this thread, for
    /// the route the app reports; set just before each such refusal.
    static ENDS_SITE: RefCell<Option<String>> = const { RefCell::new(None) };

    /// A read-only observer of the fade's hole - the
    /// directed edges round the two runs that meet at both ends, handed over just before the fill
    /// closes them. Per thread; `None` (the default) costs nothing.
    pub static WATCH_FADE_HOLE: RefCell<Option<WatchFadeHole>> = const { RefCell::new(None) };
}

/// This thread's last open-end refusal site; `None` before any.
pub fn ends_site() -> Option<String> {
    ENDS_SITE.with(|site| site.borrow().clone())
}

/// Sets (or, with `None`, clears) this thread's open-end refusal site.
pub fn set_ends_site(site: Option<String>) {
    ENDS_SITE.with(|cell| *cell.borrow_mut() = site);
}

/// Appends to this thread's open-end refusal site (an unset site reads as empty).
pub fn append_ends_site(more: &str) {
    ENDS_SITE.with(|cell| {
        let mut site = cell.borrow_mut();
        let mut text = site.take().unwrap_or_default();
        text.push_str(more);
        *site = Some(text);
    });
}

/// How close two extended parameters must be, relative to the chain's length, to count as one.
pub const PARAMETER_TIE_RELATIVE: f64 = 1e-9;

/// The fraction of a past-end row's reach to its control point below which the arcs lay its spoke
/// straight. A row that short spans under a quarter radian of the nose round the end vertex, where
/// the fade is all but gone (c563's folding rows: 0.006 and 0.0008 against 0.026).
pub const SHORT_PAST_END_SPOKE: f64 = 0.25;

/// A row the arcs laid as a straight edge onto a meeting point, or the degenerate spoke AT one:
/// one vertex repeated from the second (or to the second last) on.
pub fn is_straight_row(arc: &[i32]) -> bool {
    arc.len() > 2 && (arc[1] == arc[arc.len() - 1] || arc[arc.len() - 2] == arc[0])
}

/// One open run read as a rail running from the meeting point at end 0
/// to the one at end 1, every vertex carrying the extended parameter. Its init-only properties are
/// public fields here.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct OpenGapFillOpenRail {
    /// The rail's mesh vertices, end 0 to end 1.
    pub vertices: Vec<i32>,
    /// The EXTENDED parameter of each vertex.
    pub parameters: Vec<f64>,
}

impl OpenGapFillOpenRail {
    /// How many vertices the rail has.
    pub fn count(&self) -> i32 {
        self.vertices.len() as i32
    }
}

/// An open chain's gap, routed by WHAT THE CUT LEFT: one closed loop
/// round the whole chain is a crease that DIES OUT at both ends
/// (`dying_crease_fill::try_die_out`), and two open runs meeting on a continuing ridge is the fade
/// ([`try_fade`]).
pub fn try_open(
    gap: &BandGapResult,
    profile: BandProfile,
    fillet_segments: i32,
) -> Result<GapFillResult, LevelLineMeshError> {
    set_ends_site(None);
    if dying_crease_fill::is_one_loop_round_an_open_chain(Some(gap)) {
        dying_crease_fill::try_die_out(gap, profile, fillet_segments)
    } else {
        try_fade(gap, profile, fillet_segments)
    }
}

/// The closed hole two open runs bound, as its directed edges: `one` whole, then `other` without
/// its two ends (which `one`'s ends meet).
pub fn hole_between(one: &super::GapLoop, other: &super::GapLoop) -> Vec<(i32, i32)> {
    let inner = other.vertices.len().saturating_sub(2);
    let cycle: Vec<i32> = one
        .vertices
        .iter()
        .chain(other.vertices.iter().skip(1).take(inner))
        .copied()
        .collect();
    (0..cycle.len())
        .map(|i| (cycle[i], cycle[(i + 1) % cycle.len()]))
        .collect()
}

/// The one loop on `side`, or `None` where there is not exactly one (every caller has counted the
/// sides first).
fn single_side(loops: &[GapLoop], side: i32) -> Option<GapLoop> {
    let mut on_side = loops.iter().filter(|l| l.side == side);
    match (on_side.next(), on_side.next()) {
        (Some(only), None) => Some(only.clone()),
        _ => None,
    }
}

/// The open gap this rung builds: one chain, one open run per side, and
/// the runs' ends meeting on the continuing ridge. Runs on the END regions too go to the end-face
/// fill, runs ending apart to the growing end faces' fill; anything else about an open gap is
/// refused by name ([`ends_site`]).
pub fn try_fade(
    gap: &BandGapResult,
    profile: BandProfile,
    fillet_segments: i32,
) -> Result<GapFillResult, LevelLineMeshError> {
    let refused = GapFillResult {
        refusal: GapFillRefusal::OpenChainEnds,
        mesh: gap.mesh.clone(),
        ..GapFillResult::default()
    };

    // A side's rail cut in two by a GROWING neighbour is one rail, run on virtually past it (flow
    // rung 2).
    let mut minted = Vec::new();
    let loops = virtual_rail::joined_sides(gap, &mut minted)?;
    let closed = loops.iter().filter(|l| l.is_closed).count();
    let side_count = |side: i32| loops.iter().filter(|l| l.side == side).count();
    if gap.chains.len() != 1 || closed > 0 || side_count(1) != 1 || side_count(2) != 1 {
        set_ends_site(Some(format!(
            "fade shape: chains {}, runs {}, closed {}, side1 {}, side2 {}",
            gap.chains.len(),
            loops.len(),
            closed,
            side_count(1),
            side_count(2)
        )));
        return Ok(refused);
    }

    let (Some(mut left), Some(mut right)) = (single_side(&loops, 1), single_side(&loops, 2)) else {
        return Ok(refused);
    };
    if loops.len() != 2 {
        // Runs on the END regions too: the chain ends on end faces (see end_face_fill).
        set_ends_site(Some("end faces".to_string()));
        let filled = end_face_fill::try_end_faces(
            gap,
            left,
            right,
            profile,
            fillet_segments,
            None,
            Some(&mut minted),
        )?;
        return Ok(with_minted(filled.unwrap_or(refused), &minted));
    }

    // Only a built gap is filled, so a gap with no mesh is refused here rather than read.
    let Some(mesh_handle) = gap.mesh.as_ref() else {
        return Ok(refused);
    };
    let meet = ends_meet(&mesh_handle.borrow(), &left, &right)?;
    if !meet {
        // Runs ending APART with no end region between them: the end faces are of the opposite
        // sense, uncut, and grow over the bead's section (see
        // `end_face_fill::try_growing_end_faces`). Flow rung 3: at an end carrying the end ray the
        // rails run on virtually past the growing end face's crease to where the row last leaves
        // its plane, and the strip is clipped there (StripClip).
        set_ends_site(Some("ends apart (growing end faces)".to_string()));
        let extended =
            virtual_rail_past_the_end::extended(gap, &loops, profile, &mut minted, true)?;
        let (Some(extended_left), Some(extended_right)) =
            (single_side(&extended, 1), single_side(&extended, 2))
        else {
            return Ok(refused);
        };
        left = extended_left;
        right = extended_right;

        let minted_set: HashSet<i32> = minted.iter().copied().collect();
        let filled = if corner_cap_flat_ends::use_flat_ends() {
            end_face_fill::try_end_faces(
                gap,
                left,
                right,
                profile,
                fillet_segments,
                Some(&minted_set),
                Some(&mut minted),
            )?
        } else {
            end_face_fill::try_growing_end_faces(
                gap,
                left,
                right,
                profile,
                fillet_segments,
                Some(&minted_set),
            )?
        };
        return Ok(with_minted(filled.unwrap_or(refused), &minted));
    }

    let watch = WATCH_FADE_HOLE.with(|watch| watch.borrow().clone());
    if let Some(watch) = watch {
        watch(&hole_between(&left, &right));
    }

    let fade = open_gap_fill_fill::fill(
        mesh_handle,
        &gap.chains[0],
        &left,
        &right,
        profile,
        fillet_segments,
        &mut Vec::new(),
        OpenFillEnds::default(),
    )?;
    Ok(if fade.filled() {
        let (triangles, vertices) = (
            fade.added_triangles.len() as i32,
            fade.added_vertices.len() as i32,
        );
        with_minted(
            GapFillResult {
                refusal: GapFillRefusal::None,
                mesh: fade.mesh,
                added_triangles: fade.added_triangles,
                added_vertices: fade.added_vertices,
                triangles_of_chain: vec![triangles],
                vertices_of_chain: vec![vertices],
                ..GapFillResult::default()
            },
            &minted,
        )
    } else {
        fade
    })
}

/// A filled result with the vertices the virtual rail minted listed first among its added ones
/// (and, for a one-chain fill, counted in its chain), everything else about it - its end-face
/// capping included - kept; a refusal, or a fill that minted nothing, unchanged.
pub fn with_minted(result: super::GapFillResult, minted: &[i32]) -> super::GapFillResult {
    if minted.is_empty() || result.refusal != super::GapFillRefusal::None {
        return result;
    }

    let vertices: Vec<i32> = minted
        .iter()
        .chain(&result.added_vertices)
        .copied()
        .collect();
    let vertices_of_chain = if result.vertices_of_chain.len() == 1 {
        vec![vertices.len() as i32]
    } else {
        result.vertices_of_chain.clone()
    };
    super::GapFillResult {
        added_vertices: vertices,
        vertices_of_chain,
        // Every other field is kept, the end-faces-capped flag included: the build reads it to
        // tell the user a run stops square on its end faces, so a fade capped by its end faces
        // keeps that note when its rail minted vertices.
        ..result
    }
}

/// The run end - `a` or else `b` - that is one of the left rail's two ends, or `None`.
pub fn meeting_point_of(left: &OpenGapFillOpenRail, a: i32, b: i32) -> Option<i32> {
    let first = left.vertices[0];
    let last = left.vertices[left.vertices.len() - 1];
    if a == first || a == last {
        return Some(a);
    }

    if b == first || b == last {
        Some(b)
    } else {
        None
    }
}

/// The arc walked the other way.
pub fn backwards(arc: &[i32]) -> Vec<i32> {
    arc.iter().rev().copied().collect()
}

/// One open run read as a rail running from the meeting point at end 0
/// to the one at end 1 (`reversed` reads it from its far end), every vertex carrying the extended
/// parameter, and the reading taken as its MONOTONE ENVELOPE - which the merge needs. A backward
/// reading is flattened onto the envelope only as far as a concave turn can explain it
/// (`gap_fill::within_excursion`, the very rule the closed fill's envelope asks); deeper than that
/// the correspondence really does cross, and the rail is refused by naming the offending vertex
/// (`Err`) rather than crushed monotone - a crossed strip still closes by construction, so nothing
/// downstream would catch it.
pub fn try_rail(
    mesh: &LevelLineMesh,
    chain: &ChainField,
    vertices: &[i32],
    parameters: &[f64],
    reversed: bool,
    slack: f64,
) -> Result<OpenGapFillOpenRail, i32> {
    let count = vertices.len();
    let mut v = vec![0; count];
    let mut p = vec![0.0; count];
    for i in 0..count {
        let index = if reversed { count - 1 - i } else { i };
        v[i] = vertices[index];
        let mut reading = extended(mesh.position(vertices[index]), parameters[index], chain);
        if i > 0 && reading < p[i - 1] {
            // The envelope is read for its SETBACK, so it is asked of the chain itself: a reading
            // out past an end carries that end's setback, which is the one the clamp there was
            // built from.
            let envelope = p[i - 1].clamp(0.0, chain.length);
            if !gap_fill::within_excursion(envelope, reading - (p[i - 1] - envelope), chain, slack)
            {
                return Err(vertices[index]);
            }

            reading = p[i - 1];
        }

        p[i] = reading;
    }

    Ok(OpenGapFillOpenRail {
        vertices: v,
        parameters: p,
    })
}

/// The chain parameter, carried on past each end by the angle the
/// point stands at about the end vertex - the reading a rail's CLAMPED stretch needs, since every
/// vertex of it reads the same raw parameter.
pub fn extended(q: Vector3, parameter: f64, chain: &ChainField) -> f64 {
    let length = chain.length;
    let tolerance = 1e-9 * cs_math::max_f64(1.0, length);
    if parameter > tolerance && parameter < length - tolerance {
        return parameter;
    }

    let at_end = parameter >= length - tolerance;
    let points = &chain.points;
    let vertex = if at_end {
        points[points.len() - 1]
    } else {
        points[0]
    };
    let inward = if at_end {
        points[points.len() - 2]
    } else {
        points[1]
    };

    // The chain's OUTWARD tangent at the end vertex: where the crease carries on, and the
    // direction a rail's clamped stretch curls round towards.
    let outward = vertex - inward;
    let reach = q - vertex;
    let lengths = outward.length() * reach.length();
    let phi = if lengths > 0.0 {
        (outward.dot(reach) / lengths).clamp(-1.0, 1.0).acos()
    } else {
        std::f64::consts::PI / 2.0
    };
    let past =
        chain.setback_at(if at_end { length } else { 0.0 }) * ((std::f64::consts::PI / 2.0) - phi);
    if at_end {
        length + past
    } else {
        -past
    }
}

/// One arc per spoke, from the left rail's vertex to the right rail's
/// through the chain's own point at the mean of the two extended parameters - clamped into the
/// chain, so a spoke past an end takes the END VERTEX as its control point and the nose is the
/// profile swept round that vertex. Every vertex minted is appended to `added_vertices`; each
/// spoke charged. `rows_on_the_ray` and `meets_on_the_crease` are usually `false`.
#[allow(clippy::too_many_arguments)]
pub fn arcs(
    mesh: &mut LevelLineMesh,
    chain: &ChainField,
    spokes: &[(i32, i32)],
    left: &OpenGapFillOpenRail,
    right: &OpenGapFillOpenRail,
    segments: i32,
    added_vertices: &mut Vec<i32>,
    rows_on_the_ray: bool,
    meets_on_the_crease: bool,
) -> Result<Vec<Vec<i32>>, super::WorkError> {
    let mut arcs = Vec::new();
    let tie = PARAMETER_TIE_RELATIVE * cs_math::max_f64(1.0, chain.length);
    let left_span = open_gap_fill_stations::tie_spans(mesh, chain, left, tie)?;
    let right_span = open_gap_fill_stations::tie_spans(mesh, chain, right, tie)?;
    let row_length = (segments + 1) as usize;
    for &(l, r) in spokes {
        super::LevelLineWork::charge(1)?;
        let (lu, ru) = (l as usize, r as usize);
        let a = left.vertices[lu];
        let b = right.vertices[ru];
        if segments == 1 || a == b {
            // A degenerate spoke stands at a meeting point P: it is one vertex, repeated, so the
            // cell beside it comes out as a fan onto P rather than a row of flat quads.
            arcs.push(if a == b && segments > 1 {
                vec![a; row_length]
            } else {
                vec![a, b]
            });
            continue;
        }

        // A row with one end ON a meeting point, where the fade has run out to nothing: that end
        // stands on the crease itself, so the row's fillet has no radius and the row is the
        // straight edge to it. It is carried as that edge alone (the meeting point repeated), so
        // each cell beside it is a flat fan onto the meeting point. Bent round the chain's end
        // vertex instead, every such row shared one tangent at the meeting point and the rows'
        // first cells stood 1e-4 apart and folded in single precision (Many Holes' c904, c1915,
        // c3066: the Boolean kernel read the fade self-intersecting). Only where the rails meet on
        // the crease - an end-face fill's meeting points are the end face's corners.
        if meets_on_the_crease {
            if let Some(meeting) = meeting_point_of(left, a, b) {
                let mut straight = vec![meeting; row_length];
                straight[if meeting == a { segments as usize } else { 0 }] =
                    if meeting == a { b } else { a };
                arcs.push(straight);
                continue;
            }
        }

        // A row past an end the END-PLANE TRIM closes stands on the ray: its control point is the
        // chain's LINE carried on, not the end vertex the clamp gives (at 0.9 of the sheared
        // prism's room a row behind its near end was bent round that vertex and stood 0.0057
        // inside the round). Every other fill keeps the clamp, bit for bit.
        // A tie vertex (see tie_spans) stands for every parameter its jump spans, so it reads the
        // one nearest its partner's: the mitre spoke from a turn's inside corner to the turn's
        // outer vertex is drawn about the turn's vertex itself, in the bisector plane.
        let left_reads = right.parameters[ru].clamp(left_span[lu].0, left_span[lu].1);
        let right_reads = left.parameters[lu].clamp(right_span[ru].0, right_span[ru].1);
        let mean = (left_reads + right_reads) / 2.0;
        let c = if rows_on_the_ray {
            chain.point_on_the_ray(mean).1
        } else {
            chain.point_at(mean.clamp(0.0, chain.length))
        };

        let mut arc = vec![0; row_length];
        arc[0] = a;
        arc[segments as usize] = b;
        let p = mesh.position(a);
        let q = mesh.position(b);
        let mut along: Box<dyn Fn(f64) -> Vector3> =
            Box::new(move |f| gap_fill::arc_point(p, c, q, f));
        // A row past the chain's end whose spoke is short against its reach to the control point:
        // the fade has all but run out there, so it too is laid as its straight edge (the far end
        // repeated, as a meeting-point row is). Past the end both rails stand on the circle round
        // the clamped end vertex, and where the input's own fan of edges at that vertex is dense
        // the rail vertices crowd 3e-4 apart; every row bent round that one vertex left a shared
        // rail vertex on the one tangent toward it, and the cells between them folded in single
        // precision (Many Holes c563 at its search size: rows from left 7530 / 7561 onto right
        // 7570, spokes 0.006 and 0.0008 against a reach of 0.026). Rows inside the chain's span
        // never take this: their mean is on the chain.
        if meets_on_the_crease
            && !rows_on_the_ray
            && (mean < 0.0 || mean > chain.length)
            && (q - p).length()
                < SHORT_PAST_END_SPOKE * cs_math::min_f64((p - c).length(), (q - c).length())
        {
            let mut straight = vec![b; row_length];
            straight[0] = a;
            arcs.push(straight);
            continue;
        }

        if super::corner_cap_flat_ends::use_flat_ends()
            && rows_on_the_ray
            && open_gap_fill_cut_section::fans_from_a_rail_run_out(spokes, l, r, left, right, tie)
        {
            let cut = open_gap_fill_cut_section::cut_section_arc(mesh, chain, left, lu, right, ru)?;
            if cut.in_the_plane {
                // Wholly in the cutting plane: the row is its one straight edge, carried by B
                // repeated, so each cell beside it is a flat fan onto B and a last such row on the
                // end run is no hole.
                for slot in arc.iter_mut().take(segments as usize).skip(1) {
                    *slot = b;
                }

                arcs.push(arc);
                continue;
            }

            if let Some(cut_row) = cut.row {
                // The row is the round alone, from where the section crosses the face A ran out
                // onto down to B. Its straight part in that face is not laid as the row: that
                // face's piece between A, the rows' crossings and the in-plane rows beyond them is
                // one flat region (FlatRunOut), and laid a row at a time the straight parts fanned
                // onto A one way while the in-plane rows fanned back over the same face the other
                // (Bottom Center Edge at its saved 6: a zero-thickness double sheet the app's
                // self-intersection check drops the build for).
                let cut_at = cut.cut_at;
                arc[0] = mesh.add_vertex(cut_row(cut_at));
                added_vertices.push(arc[0]);
                along = Box::new(move |f| cut_row(cut_at + ((1.0 - cut_at) * f)));
            }
        }

        for s in 1..segments {
            let added = mesh.add_vertex(along(f64::from(s) / f64::from(segments)));
            added_vertices.push(added);
            arc[s as usize] = added;
        }

        arcs.push(arc);
    }

    Ok(arcs)
}

/// Whether two open gap loops meet end to end - the left's start the
/// right's end and the left's end the right's start - each on the ridge between their two sides.
pub fn ends_meet(
    mesh: &super::LevelLineMesh,
    left: &super::GapLoop,
    right: &super::GapLoop,
) -> Result<bool, super::WorkError> {
    if left.vertices.len() < 2 || right.vertices.len() < 2 {
        return Ok(false);
    }

    let ends = [
        (left.vertices[0], right.vertices[right.vertices.len() - 1]),
        (left.vertices[left.vertices.len() - 1], right.vertices[0]),
    ];

    for (at_left, at_right) in ends {
        if at_left != at_right || !on_the_ridge_between(mesh, at_left, left.side, right.side)? {
            return Ok(false);
        }
    }

    Ok(true)
}

/// Whether some edge at `vertex` stands between a triangle
/// tagged `left_tag` and one tagged `right_tag`. Each edge looked at is charged.
pub fn on_the_ridge_between(
    mesh: &super::LevelLineMesh,
    vertex: i32,
    left_tag: i32,
    right_tag: i32,
) -> Result<bool, super::WorkError> {
    for (low, high) in mesh.edges() {
        super::LevelLineWork::charge(1)?;
        if low != vertex && high != vertex {
            continue;
        }

        let standing = mesh.triangles_on(low, high);
        if standing.len() != 2 {
            continue;
        }

        let one = mesh.triangles[standing[0] as usize].tag;
        let other = mesh.triangles[standing[1] as usize].tag;
        if (one == left_tag && other == right_tag) || (one == right_tag && other == left_tag) {
            return Ok(true);
        }
    }

    Ok(false)
}
