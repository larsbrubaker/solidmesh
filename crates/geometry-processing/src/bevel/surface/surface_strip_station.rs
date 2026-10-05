//! One cross section of a strip ([`SurfaceStripStation`]). Ports the fields of
//! `Bevel/Surface/SurfaceStripStation.cs`; its constructor lands with the strip's package.

/// C# `SurfaceStripStation`: one cross section of a strip - the vertex on each rail, and whatever
/// else already stands between two consecutive rails there. Only an end ring has anything between:
/// the end face's own triangulation can cross the profile, and the strip sews to those points too.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SurfaceStripStation {
    /// The station's vertex on each rail, rail by rail.
    pub rail: Vec<i32>,
    /// How far along THIS strip's edge each rail meets the station, rail by rail. A ring is one
    /// station but not one parameter: the plane it is cut at is met by each rail where that rail
    /// crosses it, and at a mitre those parameters are a whole setback apart.
    pub param: Vec<f64>,
    /// What already stands between rail `k` and rail `k + 1`, entry `k`, in the chord's own order.
    pub between: Vec<Vec<i32>>,
}
