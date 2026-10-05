//! Which route the app's bevel build takes, and the watch it can leave behind. Ports the record of
//! `Bevel/LevelLine/LevelLineRouting.cs` ([`LevelLineChainWatch`]); the C# static class
//! `LevelLineRouting` (its switch and watch slot) lands with later packages, except the switch's
//! readings of a setting ([`reads_as_on`], [`reads_as_off`], [`the_running_app_wants_it_on`]).

use vector_math::Vector3;

/// C# `LevelLineChainWatch`: the whole PICK exactly as the app route hands it to `BandGap.Build`,
/// for `LevelLineRouting.Watch`. Plain data so a test can write it out and read it back.
///
/// A pick is a LIST of chains with a closedness each, because the route builds them in one call -
/// a face rim with a hole in it arrives here as two chains, not as two watches. C#'s positional
/// record defaults `seam_epsilon` to 0 and `sharp_turn_degrees` to positive infinity; the other
/// members have no default, so none is written here.
#[derive(Debug, Clone, PartialEq)]
pub struct LevelLineChainWatch {
    /// The pick's name, as the route labels it.
    pub pick: String,
    /// The mesh's vertex positions.
    pub positions: Vec<Vector3>,
    /// The mesh's triangles, as three vertex indices each.
    pub indices: Vec<(i32, i32, i32)>,
    /// The picked chains, each a run of vertex indices.
    pub chains: Vec<Vec<i32>>,
    /// Whether each chain is closed.
    pub closed: Vec<bool>,
    /// The feature edges, each as (low, high) vertex indices.
    pub feature_edges: Vec<(i32, i32)>,
    /// The bevel size asked for.
    pub size: f64,
    /// How many segments a fillet's arc is laid in.
    pub fillet_segments: i32,
    /// True for a round (fillet) profile, false for a chamfer.
    pub round_profile: bool,
    /// The seam epsilon the route passed (C# default 0).
    pub seam_epsilon: f64,
    /// The sharp-turn angle the route passed, in degrees (C# default positive infinity).
    pub sharp_turn_degrees: f64,
}

/// Whether a setting's text reads as ON: "1", or "true" in any case.
pub fn reads_as_on(value: &str) -> bool {
    value == "1" || value.eq_ignore_ascii_case("true")
}

/// C# `LevelLineRouting.ReadsAsOff`: whether a setting's value asks for a switch to be OFF: "0" or
/// "false" in any casing. An unset variable (`None`, C#'s null) asks for nothing.
pub fn reads_as_off(value: Option<&str>) -> bool {
    matches!(value, Some(v) if v == "0" || v.eq_ignore_ascii_case("false"))
}

/// C# `LevelLineRouting.TheRunningAppWantsItOn`: whether the running app takes the level line,
/// given what the developer variable says - on unless it plainly says off, because the level line
/// is what Surface mode does. A plain function of the value rather than a read of the
/// environment, so a test can assert both directions without writing a process-wide variable.
pub fn the_running_app_wants_it_on(value: Option<&str>) -> bool {
    !reads_as_off(value)
}
