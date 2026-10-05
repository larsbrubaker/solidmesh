//! The plane a BEAD stops on where its run folds back on a partner into a VALLEY - a hairpin
//! between two concave runs (`Bevel/BevelValleyFoldMitre.cs`, a static class: one function here,
//! [`try_mitre`]).

use vector_math::{math_helper, Vector3};

use super::{BevelChainEnds, BevelEndCap};
use crate::topology::FeatureEdge;

/// How far the two runs have to fold back toward each other, in degrees, to be a hairpin at all.
const MINIMUM_VALLEY_FOLD_DEGREES: f64 = 100.0;

/// How far into the material the mitre has to face, against the void's bisector, to be the
/// valley's mitre rather than a corner leaning away.
const VALLEY_FOLD_MINIMUM: f64 = 0.01;

/// The mitre plane between this run's end at `vertex` (at `position`, running `outward`, its own
/// faces facing `own_normal0` and `own_normal1`) and the `neighbour` run's edge leaving the same
/// vertex - where the two fold back into a valley: the mitre facing the way the run leaves, not
/// too nearly edge on, folded far enough to be a hairpin, and facing into the material against
/// the void's bisector. `None` otherwise, or where the plane caps nothing.
pub fn try_mitre(
    vertex: i32,
    position: Vector3,
    outward: Vector3,
    own_normal0: Vector3,
    own_normal1: Vector3,
    neighbour: &FeatureEdge,
) -> Option<BevelEndCap> {
    let away = if neighbour.start_vertex == vertex {
        neighbour.direction
    } else {
        -neighbour.direction
    };
    let mitre = outward + away;

    if mitre.length_squared() <= 0.0 {
        // The partner doubles exactly back along this run: there is no plane between them.
        return None;
    }

    let mitre = mitre.get_normal();

    let agreement = mitre.dot(outward);

    if agreement
        < math_helper::degrees_to_radians(BevelChainEnds::MAXIMUM_CAP_PLANE_ANGLE_DEGREES).cos()
    {
        // Too nearly edge on to say where the run ends, the same guard every other cap here
        // passes.
        return None;
    }

    if agreement > math_helper::degrees_to_radians(MINIMUM_VALLEY_FOLD_DEGREES / 2.0).cos() {
        // The two runs have not folded back toward each other far enough to be a hairpin at all:
        // a run walking round the OUTSIDE of something turns its mitre a little way into the void.
        return None;
    }

    let void_bisector = own_normal0 + own_normal1;

    if void_bisector.length_squared() <= 0.0
        || void_bisector.get_normal().dot(mitre) >= -VALLEY_FOLD_MINIMUM
    {
        return None;
    }

    let cap = BevelEndCap::new(mitre, mitre.dot(position));
    cap.is_capped().then_some(cap)
}
