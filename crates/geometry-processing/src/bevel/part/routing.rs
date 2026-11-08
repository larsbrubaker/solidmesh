//! Which constructions a part's fillet or chamfer is built by ([`BevelRouting`]), carried on the
//! request ([`BevelPartRequest::routing`](super::BevelPartRequest::routing)) and handed down the
//! surface part builder to every place that chooses: the level line or the older frozen
//! construction for a pass (`surface_part_builder`'s size search, attempts and level-line route),
//! the sections or the legacy edge-centric construction under it (`pass_in_two`,
//! `SurfaceBuildOptions::use_sections`), and the level line's corner cap closes (its build scope,
//! `level_line::corner_cap_hole_fill_switch::hold_corner_cap_switches`).
//!
//! The default is MatterCAD's own: the level line, without the sections, with the corner holes
//! closed by the hole filler and END FACE ends closed flat. A caller that keeps the default gets
//! the routes MatterCAD ships; the other values are there for a developer comparing constructions
//! and for the tests that pin the older ones. The kernel reads no environment and no ambient
//! switch to choose a route: the request is the whole of the choice.

/// The constructions one part's build may take.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BevelRouting {
    /// Whether a pass whose picked edges link into runs of mesh edges (closed loops or open runs)
    /// is built by the level line. Off, every pass is built by the older frozen construction,
    /// which refuses many picks the level line builds.
    pub level_line: bool,
    /// Whether a pass the sections can build (every plan simple in their sense) is built by the
    /// vertex-centric sections rather than the legacy edge-centric construction. Off by default:
    /// the sections are still being finished.
    pub sections: bool,
    /// Whether the level line's corner cap closes each corner hole with the one hole filler
    /// rather than its fans and rings.
    pub corner_hole_fill: bool,
    /// Whether the level line closes an END FACE end flat with the flat hole filler (and bounds
    /// its end ray at the end plane) rather than with zips and fans.
    pub flat_ends: bool,
}

impl BevelRouting {
    /// MatterCAD's routes, which [`Default`] answers.
    pub const MATTERCAD: BevelRouting = BevelRouting {
        level_line: true,
        sections: false,
        corner_hole_fill: true,
        flat_ends: true,
    };
}

impl Default for BevelRouting {
    fn default() -> Self {
        Self::MATTERCAD
    }
}
