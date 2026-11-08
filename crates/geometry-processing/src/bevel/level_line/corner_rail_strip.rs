//! The corner fill's zip of one chain's two rail pieces into the strip between them: the chain
//! parameter a
//! jump-end spoke's arc is drawn about ([`spoke_centre_parameter`]) and the rail vertex nearest an
//! end ([`nearest`]), and a strip end set back to where its chain ties another ([`set_back`]);
//! the strip itself is in `corner_rail_strip_zip.rs`.

/// The chain parameter a jump-end spoke's arc is drawn about: the mean of its two ends, each TIE
/// vertex (a span wider than `tie`) read at the end of its span nearest its partner - on the
/// partner's leg - and never at its own reading, which names whichever leg float noise put it on.
///
/// Read on the far leg, the Outer faces wedge tip's spokes to the wall vertices just past the
/// joint were drawn about the other leg, ran its whole length and crossed the strip. Between TWO
/// tie vertices on one joint each is placed by its span, not its reading, so neither end's noise
/// moves the other's choice. A partner equidistant from both ends (on the joint itself) takes the
/// incoming leg, the zip rail's canonical convention.
pub fn spoke_centre_parameter(
    a_span: (f64, f64),
    a: f64,
    b_span: (f64, f64),
    b: f64,
    tie: f64,
) -> f64 {
    let at_the_partners_leg = |span: (f64, f64), own: f64, partner: f64| {
        if span.1 - span.0 <= tie {
            return own;
        }

        if partner - span.0 <= span.1 - partner + tie {
            span.0
        } else {
            span.1
        }
    };

    // A partner stands where its SPAN does - its own reading when it is no tie, the middle of its
    // span when it is: a tie partner's own reading is as noisy as this end's, so reading it would
    // hand the noise on.
    let a_stands = if a_span.1 - a_span.0 <= tie {
        a
    } else {
        (a_span.0 + a_span.1) / 2.0
    };
    let b_stands = if b_span.1 - b_span.0 <= tie {
        b
    } else {
        (b_span.0 + b_span.1) / 2.0
    };
    (at_the_partners_leg(a_span, a, b_stands) + at_the_partners_leg(b_span, b, a_stands)) / 2.0
}

/// The rail index nearer `end`: `inside`, or the one a step `outward` from it when that stands
/// nearer (strictly); `inside` itself when the step leaves the rail.
pub fn nearest(parameters: &[f64], inside: i32, end: f64, outward: i32) -> i32 {
    let outside = inside + outward;
    if inside < 0 || outside < 0 || outside as usize >= parameters.len() {
        return inside;
    }

    if (parameters[outside as usize] - end).abs() < (parameters[inside as usize] - end).abs() {
        outside
    } else {
        inside
    }
}

/// The observer [`WATCH_SETBACKS`] holds.
pub type WatchSetbacks = std::rc::Rc<dyn Fn(bool)>;

thread_local! {
    /// Told `true` for each rail end a setback
    /// densified and `false` for each that stops short of it. Per thread; `None` by default.
    pub static WATCH_SETBACKS: std::cell::RefCell<Option<WatchSetbacks>> =
        const { std::cell::RefCell::new(None) };
}

/// Where an end of either rail piece (`av`/`ap`, `bv`/`bp`) ties
/// another chain's field, the strip end is set back to the tie's parameter reaching INTO the chain
/// (the greatest of the tied ends at its first end, `low`, the least at its last): a rail running
/// past it is given a station there ([`super::rail_stations::densify`], one other vertex). Answers
/// the setback less (or plus) the tie, or `None` where neither end ties.
#[allow(clippy::too_many_arguments)]
pub fn set_back(
    mesh: &mut super::LevelLineMesh,
    chains: &[std::rc::Rc<super::ChainField>],
    own: usize,
    av: &mut Vec<i32>,
    ap: &mut Vec<f64>,
    bv: &mut Vec<i32>,
    bp: &mut Vec<f64>,
    tie: f64,
    splits: &mut std::collections::HashMap<(i32, i32), i32>,
    jump_ends: &mut std::collections::HashMap<i32, f64>,
    low: bool,
) -> Result<Option<f64>, super::LevelLineMeshError> {
    use solidmesh_math::ieee_math;

    let tied = |mesh: &super::LevelLineMesh, vertex: i32| -> Result<bool, super::WorkError> {
        for (k, other) in chains.iter().enumerate() {
            if k != own
                && super::corner_fill::fields_tie(&chains[own], other, mesh.position(vertex))?
            {
                return Ok(true);
            }
        }

        Ok(false)
    };
    let end_of = |vertices: &[i32], parameters: &[f64]| {
        if low {
            (vertices[0], parameters[0])
        } else {
            (
                vertices[vertices.len() - 1],
                parameters[parameters.len() - 1],
            )
        }
    };
    let (a_vertex, a_end) = end_of(av, ap);
    let (b_vertex, b_end) = end_of(bv, bp);
    let a_tied = tied(mesh, a_vertex)?;
    let b_tied = tied(mesh, b_vertex)?;
    let tied_ends: Vec<f64> = [(a_tied, a_end), (b_tied, b_end)]
        .iter()
        .filter(|e| e.0)
        .map(|e| e.1)
        .collect();
    if tied_ends.is_empty() {
        return Ok(None);
    }

    // The setback reaches INTO the chain: the greater parameter at its first end, the lesser at
    // its last.
    let setback = if low {
        tied_ends
            .iter()
            .copied()
            .fold(f64::NAN, |m, e| if m.is_nan() || e > m { e } else { m })
    } else {
        tied_ends[1..].iter().fold(tied_ends[0], |m, &e| {
            if m.is_nan() || e.is_nan() {
                f64::NAN
            } else {
                ieee_math::min_f64(m, e)
            }
        })
    };
    let watch = WATCH_SETBACKS.with(|w| w.borrow().clone());
    for (vertices, parameters, end) in [(av, ap, a_end), (bv, bp, b_end)] {
        let runs_past = if low {
            end < setback - tie
        } else {
            end > setback + tie
        };
        let stops_short = if low {
            end > setback + tie
        } else {
            end < setback - tie
        };
        if runs_past {
            super::rail_stations::densify(
                mesh,
                &chains[own],
                vertices,
                parameters,
                &[setback],
                tie,
                splits,
                jump_ends,
                1,
            )?;
            if let Some(watch) = &watch {
                watch(true);
            }
        } else if stops_short {
            if let Some(watch) = &watch {
                watch(false);
            }
        }
    }

    Ok(Some(if low { setback - tie } else { setback + tie }))
}
