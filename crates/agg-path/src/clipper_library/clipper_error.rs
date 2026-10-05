//! The error the `clipper_library` helpers return where agg-sharp's throw: a `ClipperException`
//! out of Clipper 6 ([`crate::clipper6::ClipperError`]) - coordinates beyond +/-2^62, or an
//! internal invariant of the sweep failing. C#'s extension methods let the exception escape to
//! their caller; here it comes back as `Err` for the caller to propagate.

use std::fmt;

use crate::clipper6;

/// A Clipper 6 failure reaching a `clipper_library` caller (C# `ClipperException`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ClipperError(pub clipper6::ClipperError);

impl From<clipper6::ClipperError> for ClipperError {
    fn from(error: clipper6::ClipperError) -> Self {
        ClipperError(error)
    }
}

impl fmt::Display for ClipperError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Display::fmt(&self.0, f)
    }
}

impl std::error::Error for ClipperError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(&self.0)
    }
}
