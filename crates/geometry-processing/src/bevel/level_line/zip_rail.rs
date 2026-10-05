//! A loop re-read as a cycle running in increasing chain parameter ([`ZipRail`]). Ports the type of
//! `Bevel/LevelLine/ZipRail.cs` with its count, its canonical reading at tie vertices and its
//! reading past the wrap; how the merge reads it across jumps and from its start lands with later
//! packages.

/// C# `ZipRail`: a loop re-read as a cycle running in increasing chain parameter - one of the two
/// rails `GapFill`'s per-chain zip merges. Its init-only properties are public fields here.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ZipRail {
    /// The rail's mesh vertices, in increasing parameter.
    pub vertices: Vec<i32>,
    /// The chain parameter each vertex reads.
    pub parameters: Vec<f64>,
    /// True when the rail runs against its loop's walk - so its fill edges run WITH the rail.
    pub reversed: bool,
}

impl ZipRail {
    /// How many vertices the rail has.
    pub fn count(&self) -> i32 {
        self.vertices.len() as i32
    }

    /// The rail with each vertex read at the low end of the span it stands for: a tie vertex's
    /// incoming leg's reading, any other vertex's own. A vertex on a sharp joint's inner bisector
    /// is on both legs, and which of the two its own reading names is float noise; the merge, its
    /// jump test and the rail start all read this one value, so no choice they make turns on the
    /// noise. The incoming leg is the convention the fillet arc takes for a reading on a joint.
    pub fn canonical(rail: &ZipRail, spans: &[(f64, f64)]) -> ZipRail {
        ZipRail {
            vertices: rail.vertices.clone(),
            parameters: spans.iter().map(|span| span.0).collect(),
            reversed: rail.reversed,
        }
    }

    /// The parameter of rail position `advance`, read on past the wrap: a position at or past the
    /// count is the first vertex again, a chain `length` further on; `offset` shifts every reading.
    pub fn unwrapped(rail: &ZipRail, advance: i32, length: f64, offset: f64) -> f64 {
        if advance < rail.count() {
            rail.parameters[advance as usize] + offset
        } else {
            rail.parameters[0] + offset + length
        }
    }
}
