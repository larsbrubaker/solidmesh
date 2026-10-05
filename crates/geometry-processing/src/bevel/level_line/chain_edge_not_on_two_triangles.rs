//! The error a level-line walk raises when a chain edge does not stand on exactly two triangles.
//! Ports `Bevel/LevelLine/ChainEdgeNotOnTwoTrianglesException.cs`.
//!
//! In C# it is thrown deep in the walk and caught at every entry point, turned into
//! [`BandGapRefusal::ChainEdgeNotOnTwoTriangles`](super::BandGapRefusal::ChainEdgeNotOnTwoTriangles),
//! so no caller ever sees it. Here it is the `Err` those walks return (CLAUDE.md: exceptions
//! become `Result`), carried up the same way and turned into the same refusal.

use std::fmt;

/// C# `ChainEdgeNotOnTwoTrianglesException`: the chain edge `from`-`to`, with the message the
/// thrower wrote.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChainEdgeNotOnTwoTriangles {
    /// The vertex the offending chain edge runs from.
    pub from: i32,
    /// The vertex it runs to.
    pub to: i32,
    /// What the thrower said about it (C#'s `Exception.Message`).
    pub message: String,
}

impl ChainEdgeNotOnTwoTriangles {
    /// C#'s constructor.
    pub fn new(from: i32, to: i32, message: impl Into<String>) -> Self {
        ChainEdgeNotOnTwoTriangles {
            from,
            to,
            message: message.into(),
        }
    }
}

impl fmt::Display for ChainEdgeNotOnTwoTriangles {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for ChainEdgeNotOnTwoTriangles {}
