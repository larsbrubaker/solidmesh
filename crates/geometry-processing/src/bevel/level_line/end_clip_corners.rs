//! The corners where a clipped end plane meets the creases leaving its zero plateau. Ports
//! `Bevel/LevelLine/EndClipCorners.cs` (a C# static class): so far its constants; finding the
//! corners lands with later packages.

/// Bisection steps along a crease edge to the last point still reading zero.
pub const BISECTION_STEPS: i32 = 60;

/// How far off the end plane a crease vertex may stand and still be on it: the input is float, so a
/// vertex of the side-to-end crease stands up to about 1e-7 off the plane read through the end
/// vertex.
pub const ON_THE_PLANE: f64 = 1e-5;
