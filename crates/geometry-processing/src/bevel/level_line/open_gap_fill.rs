//! The gap fill for an OPEN run. Ports the nested type of `Bevel/LevelLine/OpenGapFill.cs` (the
//! primary file of the C# static partial class `OpenGapFill`): one open run read as a rail
//! ([`OpenGapFillOpenRail`]), the class's constants, the thread's refusal site ([`ends_site`]), the
//! straight-row test ([`is_straight_row`]) and the small pieces of the fill: the hole between two
//! runs ([`hole_between`]), a result carrying the minted vertices ([`with_minted`]), a run end's
//! meeting point ([`meeting_point_of`]), a reversed arc ([`backwards`]), a run read as a rail
//! ([`try_rail`]) and the extended parameter past an end ([`extended`]). The fill's other functions
//! land with later packages.

use std::cell::RefCell;

use vector_math::{cs_math, Vector3};

use super::{gap_fill, ChainField, LevelLineMesh};

thread_local! {
    /// C# `[ThreadStatic] OpenGapFill.EndsSite`: which open-end site last refused the open-chain
    /// ends - or, for a refused growing-neighbour clip, which strip-clip stage - on this thread, for
    /// the route the app reports; set just before each such refusal.
    static ENDS_SITE: RefCell<Option<String>> = const { RefCell::new(None) };
}

/// This thread's last open-end refusal site; `None` before any.
pub fn ends_site() -> Option<String> {
    ENDS_SITE.with(|site| site.borrow().clone())
}

/// Sets (or, with `None`, clears) this thread's open-end refusal site.
pub fn set_ends_site(site: Option<String>) {
    ENDS_SITE.with(|cell| *cell.borrow_mut() = site);
}

/// Appends to this thread's open-end refusal site (C# `EndsSite += ...`, where a null site reads
/// as empty).
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

/// C# `OpenGapFill.OpenRail`: one open run read as a rail running from the meeting point at end 0
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

/// A filled result with the vertices the virtual rail minted listed first among its added ones
/// (and, for a one-chain fill, counted in its chain); a refusal, or a fill that minted nothing,
/// unchanged.
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
        // C# copies the init-only properties it names; the one it leaves out keeps its default.
        ends_capped_by_end_faces: false,
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

/// C# `OpenGapFill.TryRail`: one open run read as a rail running from the meeting point at end 0
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

/// C# `OpenGapFill.Extended`: the chain parameter, carried on past each end by the angle the
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
