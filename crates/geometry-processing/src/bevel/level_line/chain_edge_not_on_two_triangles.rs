//! The error a level-line walk raises when a chain edge does not stand on exactly two triangles.
//!
//! It is found deep in the walk and is the `Err` those walks return, carried up to every entry
//! point and turned there into
//! [`BandGapRefusal::ChainEdgeNotOnTwoTriangles`](super::BandGapRefusal::ChainEdgeNotOnTwoTriangles),
//! so no caller ever sees it.

use std::fmt;

/// The chain edge `from`-`to`, with the message the
/// thrower wrote.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChainEdgeNotOnTwoTriangles {
    /// The vertex the offending chain edge runs from.
    pub from: i32,
    /// The vertex it runs to.
    pub to: i32,
    /// What the thrower said about it.
    pub message: String,
}

impl ChainEdgeNotOnTwoTriangles {
    /// The edge `from -> to`, with the message saying what is wrong with it.
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
