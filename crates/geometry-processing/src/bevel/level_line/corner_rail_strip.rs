//! The corner fill's zip of one chain's two rail pieces into the strip between them. Ports
//! `Bevel/LevelLine/CornerRailStrip.cs` (a C# static class): so far the chain parameter a
//! jump-end spoke's arc is drawn about ([`spoke_centre_parameter`]) and the rail vertex nearest an
//! end ([`nearest`]); the strip itself lands with later packages.

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
