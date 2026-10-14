//! The surface plan's reading of one picked edge: its finiteness tests ([`is_finite`],
//! [`is_finite_vector`]) and the reading itself ([`try_plan_edge`]).

use vector_math::math_helper::degrees_to_radians;
use vector_math::Vector3;

use super::super::bevel_edge_tool_builder::{profile_wedge_angle_degrees, tangent_distance};
use super::super::{BevelAdvisory, BevelAdvisoryCode};
use super::surface_roll_across_the_wall::{self, RegionGridSource, SurfaceRollReading};
use super::surface_run_across::SurfaceRunAcrossRegionMapsCache;
use super::{SurfaceBevelPlan, SurfaceEdgePlan, SurfaceLine};
use crate::topology::{ChainEdge, EdgeConvexity, FaceRegionCache};

/// True when the number is neither NaN nor infinite.
pub fn is_finite(value: f64) -> bool {
    !value.is_nan() && !value.is_infinite()
}

/// True when every component of the vector is finite.
pub fn is_finite_vector(value: Vector3) -> bool {
    is_finite(value.x) && is_finite(value.y) && is_finite(value.z)
}

/// The plan of one picked chain edge - refused where its faces are coplanar, its wedge too sharp or
/// too flat, its directions not finite, a face has no region, or both faces are one region; ROLLED
/// along a wall that curves along its section ([`surface_roll_across_the_wall::read`], asking
/// `grid_of` for a curved region's face grid); otherwise the two-plane construction: the setback on
/// each face and, for a round profile, the centre line on the wedge's bisector.
pub fn try_plan_edge(
    chain_edge: &ChainEdge,
    regions: &FaceRegionCache,
    size: f64,
    round_profile: bool,
    region_maps: &mut SurfaceRunAcrossRegionMapsCache,
    grid_of: RegionGridSource,
) -> Result<SurfaceEdgePlan, BevelAdvisory> {
    let does_not_fit = || BevelAdvisory::new(BevelAdvisoryCode::BevelDoesNotFitCorner, Vec::new());
    let edge = &chain_edge.edge;
    let midpoint = (chain_edge.from() + chain_edge.to()) / 2.0;

    if edge.convexity == EdgeConvexity::Flat {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::FacesCoplanar,
            Vec::new(),
        ));
    }

    let wedge_angle_degrees = profile_wedge_angle_degrees(edge);
    #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > y)` on purpose: NaN refuses
    let too_sharp = !(wedge_angle_degrees > SurfaceBevelPlan::MINIMUM_WEDGE_ANGLE_DEGREES);
    if too_sharp {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::FaceAngleTooSharp,
            vec![wedge_angle_degrees.into()],
        ));
    }

    #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x < y)` on purpose: NaN refuses
    let too_flat = !(wedge_angle_degrees < SurfaceBevelPlan::MAXIMUM_WEDGE_ANGLE_DEGREES);
    if too_flat {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::FaceAngleTooFlat,
            vec![wedge_angle_degrees.into()],
        ));
    }

    let along = chain_edge.direction();
    let left_normal = chain_edge.left_normal();
    let right_normal = chain_edge.right_normal();
    if !is_finite_vector(along) || !is_finite_vector(left_normal) || !is_finite_vector(right_normal)
    {
        return Err(does_not_fit());
    }

    let (Some(left_region), Some(right_region)) = (
        regions.region(chain_edge.left_face()),
        regions.region(chain_edge.right_face()),
    ) else {
        return Err(does_not_fit());
    };

    // One region on both sides: the flood came back to this edge the LONG way, so the two faces
    // are read as one surface and a bevel has no second face to build against.
    if SurfaceBevelPlan::key_of(&left_region) == SurfaceBevelPlan::key_of(&right_region) {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCreaseFacesAreOneSurface,
            vec![midpoint.into(), regions.flat_angle_degrees().into()],
        ));
    }

    // The direction that lies IN a face, square to the edge, pointing into the face.
    let left_inward = left_normal.cross(along);
    let right_inward = along.cross(right_normal);

    // A landing that CURVES along its section is rolled instead, facet by facet.
    let (reading, rolled, refusal) = surface_roll_across_the_wall::read(
        Some(regions.mesh()),
        chain_edge,
        &left_region,
        &right_region,
        along,
        left_normal,
        right_normal,
        left_inward,
        right_inward,
        size,
        round_profile,
        edge.convexity == EdgeConvexity::Convex,
        region_maps,
        grid_of,
    );
    match reading {
        SurfaceRollReading::Refused => {
            return Err(refusal.expect("a refused roll says why"));
        }
        SurfaceRollReading::Rolled => {
            let rolled = rolled.expect("a rolled reading carries its edge");
            return Ok(rolled.as_plan(
                *chain_edge,
                left_region,
                right_region,
                wedge_angle_degrees,
                left_inward,
                right_inward,
            ));
        }
        SurfaceRollReading::NotCurved => {}
    }

    let setback = tangent_distance(size, wedge_angle_degrees, round_profile);
    if !is_finite(setback) {
        return Err(does_not_fit());
    }

    let left_tangent_line = SurfaceLine::new(chain_edge.from() + (left_inward * setback), along);
    let right_tangent_line = SurfaceLine::new(chain_edge.from() + (right_inward * setback), along);

    let mut has_center_line = false;
    let mut center_line = SurfaceLine::default();
    let mut center_distance = 0.0;

    if round_profile {
        // The bisector of the two in-face inward directions, which is the bisector of the wedge.
        let bisector = left_inward + right_inward;
        #[allow(clippy::neg_cmp_op_on_partial_ord)] // `!(x > 0)` on purpose: NaN refuses
        let no_bisector = !(bisector.length_squared() > 0.0) || !is_finite_vector(bisector);
        if no_bisector {
            return Err(does_not_fit());
        }

        center_distance = size / (degrees_to_radians(wedge_angle_degrees) / 2.0).sin();
        if !is_finite(center_distance) {
            return Err(does_not_fit());
        }

        center_line = SurfaceLine::new(
            chain_edge.from() + (bisector.get_normal() * center_distance),
            along,
        );
        has_center_line = true;
    }

    Ok(SurfaceEdgePlan::new(
        *chain_edge,
        left_region,
        right_region,
        wedge_angle_degrees,
        setback,
        setback,
        left_inward,
        right_inward,
        left_inward,
        right_inward,
        left_tangent_line,
        right_tangent_line,
        has_center_line,
        center_line,
        center_distance,
        None,
        None,
        None,
    ))
}
