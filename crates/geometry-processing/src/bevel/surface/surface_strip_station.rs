//! One cross section of a strip ([`SurfaceStripStation`]): its fields, its constructor and the
//! clean station.

/// One cross section of a strip - the vertex on each rail, and whatever
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

impl SurfaceStripStation {
    /// The station's rails, their parameters, and what stands between each pair.
    pub fn new(rail: Vec<i32>, param: Vec<f64>, between: Vec<Vec<i32>>) -> Self {
        Self {
            rail,
            param,
            between,
        }
    }

    /// A station with nothing between its rails, which is every station but an end.
    pub fn clean(rail: Vec<i32>, param: Vec<f64>) -> Self {
        let between = vec![Vec::new(); rail.len().saturating_sub(1)];

        Self::new(rail, param, between)
    }
}
