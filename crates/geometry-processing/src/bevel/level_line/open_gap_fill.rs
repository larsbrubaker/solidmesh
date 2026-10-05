//! The gap fill for an OPEN run. Ports the nested type of `Bevel/LevelLine/OpenGapFill.cs` (the
//! primary file of the C# static partial class `OpenGapFill`): one open run read as a rail
//! ([`OpenGapFillOpenRail`]), the class's constants, the thread's refusal site ([`ends_site`]), the
//! straight-row test ([`is_straight_row`]) and the small pieces of the fill: the hole between two
//! runs ([`hole_between`]), a result carrying the minted vertices ([`with_minted`]), a run end's
//! meeting point ([`meeting_point_of`]) and a reversed arc ([`backwards`]). The fill's functions land with later
//! packages.

use std::cell::RefCell;

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
