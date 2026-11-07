//! One station's section: a junction, a corner's
//! mitre, or the ordinary section square to the chain - its two boundaries, its profile's samples
//! between them and its centres. Split from `section_builder.rs` by responsibility.

use std::rc::Rc;

use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::ieee_math;

use super::section_builder::{
    place_the_interior_samples, try_boundary, try_junction, SMALLEST_WEDGE_COSINE,
};
use super::section_builder_walked_side::SectionBuildRefusal;
use super::section_corner::{self, CornerRefusal};
use super::section_fan_walk::EdgeAtPair;
use super::section_junction_build::{self, JunctionRefusal};
use super::{BevelSection, JunctionSection, SectionPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};
use crate::topology::{ChainEdge, FeatureEdge};

impl From<JunctionRefusal> for SectionBuildRefusal {
    fn from(refusal: JunctionRefusal) -> Self {
        match refusal {
            JunctionRefusal::Refused(advisory) => SectionBuildRefusal::Refused(advisory),
            JunctionRefusal::NotANumber(nan) => SectionBuildRefusal::NotANumber(nan),
        }
    }
}

impl From<CornerRefusal> for SectionBuildRefusal {
    fn from(refusal: CornerRefusal) -> Self {
        match refusal {
            CornerRefusal::AJunction => SectionBuildRefusal::Declined,
            CornerRefusal::Refused(advisory) => SectionBuildRefusal::Refused(advisory),
            CornerRefusal::NotANumber(nan) => SectionBuildRefusal::NotANumber(nan),
        }
    }
}

/// The section at the vertex `leaving` starts from, and the
/// junction where that station is one - or why there is none.
#[allow(clippy::too_many_arguments)]
pub fn try_section(
    arriving: &ChainEdge,
    leaving: &ChainEdge,
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    pool: &mut SectionPool,
    radius: f64,
    segments: i32,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
) -> Result<(Rc<BevelSection>, Option<JunctionSection>), SectionBuildRefusal> {
    let vertex = leaving.from_vertex();
    let station = leaving.from();

    if section_corner::is_sharp_turn(arriving, leaving) {
        if section_junction_build::is_a_junction_station(
            mesh,
            graph,
            arriving,
            leaving,
            vertex,
            is_a_feature_edge,
            Some(edge_at_pair),
        ) {
            let (section, junction) = try_junction(
                arriving,
                leaving,
                vertex,
                station,
                mesh,
                graph,
                edge_at_pair,
                pool,
                radius,
                segments,
                is_a_feature_edge,
            )?;
            return Ok((section, Some(junction)));
        }

        if !section_corner::looks_like_a_corner(arriving, leaving) {
            return Err(SectionBuildRefusal::Refused(BevelAdvisory::new(
                BevelAdvisoryCode::SurfaceCornerNotSupported,
                vec![station.into()],
            )));
        }

        let section = section_corner::try_build(
            arriving,
            leaving,
            mesh,
            graph,
            edge_at_pair,
            pool,
            radius,
            segments,
            is_a_feature_edge,
        )?;
        return Ok((Rc::new(section), None));
    }

    let left = (arriving.left_normal() + leaving.left_normal()).get_normal();
    let right = (arriving.right_normal() + leaving.right_normal()).get_normal();
    let cosine = left.dot(right);
    if cosine > FeatureEdge::COPLANAR_FACE_DOT || cosine < SMALLEST_WEDGE_COSINE {
        return Err(SectionBuildRefusal::Refused(BevelAdvisory::new(
            if cosine > 0.0 {
                BevelAdvisoryCode::FacesCoplanar
            } else {
                BevelAdvisoryCode::FacesFoldBack
            },
            vec![vertex.into()],
        )));
    }

    let mut tangent = left.cross(right).get_normal();
    if tangent.dot(arriving.direction() + leaving.direction()) < 0.0 {
        tangent = -tangent;
    }

    let wedge = cosine.clamp(-1.0, 1.0).acos();
    let setback = radius * (wedge / 2.0).tan();
    let centre = station - ((left + right) * radius / (1.0 + cosine));
    let across = ieee_math::max_f64(1.0 - (cosine * cosine), 0.0).sqrt();
    let left_away = ((left * cosine) - right) / across;
    let right_away = ((right * cosine) - left) / across;

    let left_side = try_boundary(
        vertex,
        station,
        arriving,
        leaving,
        true,
        left,
        left_away,
        setback,
        radius,
        centre,
        tangent,
        mesh,
        graph,
        edge_at_pair,
        is_a_feature_edge,
    )
    .map_err(SectionBuildRefusal::Refused)?;
    let right_side = try_boundary(
        vertex,
        station,
        arriving,
        leaving,
        false,
        right,
        right_away,
        setback,
        radius,
        centre,
        tangent,
        mesh,
        graph,
        edge_at_pair,
        is_a_feature_edge,
    )
    .map_err(SectionBuildRefusal::Refused)?;

    let mut profile = vec![0; (segments.max(1) + 1) as usize];
    let last = profile.len() - 1;
    profile[0] = pool.add(left_side.at)?;
    profile[last] = pool.add(right_side.at)?;
    place_the_interior_samples(
        arriving,
        leaving,
        tangent,
        left,
        right,
        wedge,
        radius,
        left_side.at,
        right_side.at,
        left_side.side_centre,
        right_side.side_centre,
        pool,
        &mut profile,
    )?;

    let section = BevelSection::new(
        vertex,
        station,
        tangent,
        profile[0],
        profile[last],
        profile.clone(),
        left_side.side_centre,
        right_side.side_centre,
        left_side.kind,
        right_side.kind,
        left_side.slide_edge,
        right_side.slide_edge,
        setback,
    )
    .expect("a section's profile ends on its boundaries");
    Ok((Rc::new(section), None))
}
