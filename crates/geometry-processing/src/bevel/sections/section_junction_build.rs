//! The junction's construction: whether a chain vertex is a junction at all
//! ([`is_a_junction_station`]), the junction section itself - one row of samples on the mitre
//! ellipse the two canals share, carrying both wall ends - ([`try_build`]), and the far stretch of that
//! row carried out onto the facets of the tessellated wall ([`try_carry_the_far_stretch_onto_the_facets`]).
//! The canal and the section's own readings are in [`super::section_junction_canal`].

use std::rc::Rc;

use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::{cs_math, Vector3};

use super::section_fan_walk::{self, EdgeAtPair};
use super::section_junction::Canal;
use super::section_junction_canal::{
    lattice_of, meet_of_lines, the_normal_of, the_strips_section, the_triangle_holds,
    try_read_the_flat_side, ON_BOTH_CANALS_TO_WITHIN_THIS_MUCH_OF_THE_RADIUS, SAME_TO_WITHIN,
};
use super::{section_builder, section_corner, JunctionSection, NanSectionPoint, SectionPool};
use crate::bevel::{BevelAdvisory, BevelAdvisoryCode};
use crate::topology::ChainEdge;

/// Why a junction could not be built: the named corner refusal, or a point that is not a number.
#[derive(Debug, Clone, PartialEq)]
pub enum JunctionRefusal {
    Refused(BevelAdvisory),
    NotANumber(NanSectionPoint),
}

impl From<NanSectionPoint> for JunctionRefusal {
    fn from(nan: NanSectionPoint) -> Self {
        JunctionRefusal::NotANumber(nan)
    }
}

/// Whether the chain's turn at `vertex` is a JUNCTION - a sharp turn the
/// corner's mitre declines for its WEDGE, with one flat side shared, two canal axes that meet, and
/// exactly one crease (the fold the patch stands on) leaving the vertex into the wall side. The
/// feature test and the edge pair table are optional; a missing pair table is built here.
pub fn is_a_junction_station(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    arriving: &ChainEdge,
    leaving: &ChainEdge,
    vertex: i32,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
    edge_at_pair: Option<&EdgeAtPair>,
) -> bool {
    if leaving.from_vertex() != vertex || arriving.to_vertex() != vertex {
        return false;
    }

    if !section_corner::is_sharp_turn(arriving, leaving)
        || section_corner::looks_like_a_corner(arriving, leaving)
    {
        // A station the chain reads as a point of a curve is drawn by the smooth rule, and one the
        // mitre can draw belongs to the corner - the junction is only the sharp turn it refuses.
        return false;
    }

    let Some(flat_on_the_left) = try_read_the_flat_side(arriving, leaving) else {
        return false;
    };

    if section_corner::try_the_mitre_plane(arriving, leaving, 1.0).is_none() {
        // Two axes that never meet: there is no one ellipse for the two strips to share.
        return false;
    }

    let built;
    let pairs = match edge_at_pair {
        Some(pairs) => pairs,
        None => {
            built = section_fan_walk::edges_by_vertex_pair(graph);
            &built
        }
    };

    // The FOLD the patch stands on: exactly one crease leaving the vertex into the wall side.
    section_builder::try_creases_in_the_fan_reading(
        vertex,
        arriving,
        leaving,
        !flat_on_the_left,
        mesh,
        graph,
        pairs,
        is_a_feature_edge,
    )
    .is_ok_and(|(creases, _)| creases.len() == 1)
}

/// The junction section at the vertex where `arriving` hands over to `leaving`, its
/// patch standing on `fold`. Both canals have the same radius and ride the shared flat side, so they
/// cut the mitre plane in ONE ellipse; what unequal wedges break is only the wall end. One row of
/// samples is drawn on the ARRIVING canal's sweep, in equal steps out to the NEAR wall end and on in
/// the same step to the FAR one, every sample checked to lie on the leaving canal too. F1 - where the
/// near canal's wall contact meets the fold - is the patch's apex. With a `mesh`, the far stretch is
/// carried out onto the tessellated wall's facets. Nothing is added to the pool until every refusal
/// is past.
#[allow(clippy::too_many_arguments)]
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn try_build(
    arriving: &ChainEdge,
    leaving: &ChainEdge,
    fold: Vector3,
    pool: &mut SectionPool,
    radius: f64,
    segments: i32,
    mesh: Option<&Mesh>,
) -> Result<JunctionSection, JunctionRefusal> {
    let station = leaving.from();
    let refused = || {
        JunctionRefusal::Refused(BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![station.into()],
        ))
    };

    // Every reading below is an ABSOLUTE distance, so all of them are taken against the lattice the
    // points handed in are stored on.
    let lattice = lattice_of(&[station, arriving.from(), arriving.to(), leaving.to()]);

    let Some(flat_on_the_left) = try_read_the_flat_side(arriving, leaving) else {
        return Err(refused());
    };
    let Some((through, normal)) = section_corner::try_the_mitre_plane(arriving, leaving, radius)
    else {
        return Err(refused());
    };

    let mut arriving_canal = Canal::new(arriving, station, radius, flat_on_the_left, lattice);
    let mut leaving_canal = Canal::new(leaving, station, radius, flat_on_the_left, lattice);

    arriving_canal.set_lean_for(normal);
    leaving_canal.set_lean_for(normal);

    if arriving_canal.lean.abs() < SAME_TO_WITHIN || leaving_canal.lean.abs() < SAME_TO_WITHIN {
        // A canal lying IN the mitre plane has no slide onto it.
        return Err(refused());
    }

    // The LEAVING canal's tangency with its own wall, read as a point of the ARRIVING canal's sweep.
    // Which of the two wall ends stands FARTHER out is not fixed, so the section is drawn in near and
    // far, not arriving and leaving.
    let wall_end_of_leaving = leaving_canal.point_at(1.0, through, normal);

    let leaving_wall = match arriving_canal.try_parameter_of(wall_end_of_leaving) {
        Some(v)
            if !((v - 1.0).abs() < SAME_TO_WITHIN)
                && !(v <= lattice / cs_math::max_f64(radius, lattice)) =>
        {
            v
        }
        // Not on the arriving canal, the arriving canal's own end (a CORNER's case), or at or behind
        // the shared flat side - no row of samples to draw between them.
        _ => return Err(refused()),
    };

    let near = cs_math::min_f64(1.0, leaving_wall);
    let far = cs_math::max_f64(1.0, leaving_wall);
    let rungs = segments.max(1);
    let step = near / rungs as f64;
    let mut samples: Vec<f64> = (0..=rungs).map(|k| k as f64 * step).collect();

    // The samples past the NEAR wall end keep that same step and the last lands exactly on the far one.
    let mut v = near + step;
    while v < far - (step / 2.0) {
        samples.push(v);
        v += step;
    }

    samples.push(far);

    let mut row = Vec::with_capacity(samples.len());

    for &sample in &samples {
        let at = arriving_canal.point_at(sample, through, normal);

        if (leaving_canal.distance_from_the_axis(at) - radius).abs()
            > cs_math::max_f64(
                ON_BOTH_CANALS_TO_WITHIN_THIS_MUCH_OF_THE_RADIUS * radius,
                lattice,
            )
        {
            // Not one ellipse after all, so the two strips would not meet on it.
            return Err(refused());
        }

        row.push(at);
    }

    // F1: where the NEAR canal's contact with its own wall ends, which is ON the fold, exactly.
    let leaving_is_far = leaving_wall > 1.0;
    let near_canal = if leaving_is_far {
        &arriving_canal
    } else {
        &leaving_canal
    };

    let (fold_point, skew) = meet_of_lines(
        near_canal.axis + (near_canal.wall * radius),
        near_canal.along,
        station,
        fold.get_normal(),
    );

    if skew > lattice {
        // The near canal's tangency line is skew to the fold, so there is no F1 to stand the patch on.
        return Err(refused());
    }

    let mut carried = vec![-1; row.len()];

    if let Some(mesh) = mesh {
        let far_canal = if leaving_is_far {
            &mut leaving_canal
        } else {
            &mut arriving_canal
        };

        try_carry_the_far_stretch_onto_the_facets(
            mesh,
            far_canal,
            through,
            normal,
            rungs,
            &mut row,
            &mut carried,
            lattice,
            station,
        )
        .map_err(JunctionRefusal::Refused)?;
    }

    // EVERY REFUSAL IS PAST, so the points may be minted: the pool never removes, so a point minted
    // before a refusal is an orphan a later pass reads as a point of the bevel.
    let mut drawn = Vec::with_capacity(row.len());

    for &at in &row {
        drawn.push(pool.add(at)?);
    }

    let fold1 = pool.add(fold_point)?;
    let tangent = (arriving_canal.along + leaving_canal.along).get_normal();
    let (arriving_end, leaving_end) = if leaving_is_far {
        (rungs as usize, drawn.len() - 1)
    } else {
        (drawn.len() - 1, rungs as usize)
    };
    let setback_arriving = radius * (arriving_canal.wedge / 2.0).tan();
    let setback_leaving = radius * (leaving_canal.wedge / 2.0).tan();
    let strips_section = |wall_end: usize, setback: f64| {
        the_strips_section(
            leaving.from_vertex(),
            station,
            tangent,
            &drawn,
            wall_end,
            flat_on_the_left,
            through,
            setback,
        )
        .map_err(|_| refused())
    };
    let arriving_section = strips_section(arriving_end, setback_arriving)?;
    let leaving_section = strips_section(leaving_end, setback_leaving)?;

    Ok(JunctionSection::new(
        leaving.from_vertex(),
        station,
        through,
        normal,
        drawn,
        arriving_end as i32,
        leaving_end as i32,
        fold1,
        arriving_canal.wall,
        leaving_canal.wall,
        flat_on_the_left,
        setback_arriving,
        setback_leaving,
        Rc::new(arriving_section),
        Rc::new(leaving_section),
        carried,
    ))
}

/// Each sample past the near wall end (`near_end`) carried
/// out along its own ray from the far canal's axis onto the nearest facet of the part that holds the
/// crossing - so the far stretch lies on the tessellated wall rather than its ideal cylinder - with
/// the facet it was carried onto recorded in `carried`. A sample off the far canal, or one no facet
/// holds, is refused by name rather than drawn off the surface.
#[allow(clippy::too_many_arguments)]
pub fn try_carry_the_far_stretch_onto_the_facets(
    mesh: &Mesh,
    far_canal: &mut Canal,
    through: Vector3,
    normal: Vector3,
    near_end: i32,
    row: &mut [Vector3],
    carried: &mut [i32],
    lattice: f64,
    station: Vector3,
) -> Result<(), BevelAdvisory> {
    let refused = || {
        BevelAdvisory::new(
            BevelAdvisoryCode::SurfaceCornerNotSupported,
            vec![station.into()],
        )
    };

    for k in (near_end as usize + 1)..row.len() {
        let Some(v) = far_canal.try_parameter_of(row[k]) else {
            // The sample is not on the far canal at all, so there is no ray to carry it out along.
            return Err(refused());
        };

        let at_the_axis = far_canal.point_at_distance(v, 0.0, through, normal);
        let a_unit_out = far_canal.point_at_distance(v, 1.0, through, normal) - at_the_axis;
        let mut held = -1;
        let mut at = row[k];
        let mut nearest = f64::MAX;

        for face in 0..mesh.faces.len() as i32 {
            let face_normal = the_normal_of(mesh, face);
            let lean = a_unit_out.dot(face_normal);

            if lean.abs() < SAME_TO_WITHIN {
                continue;
            }

            let corner =
                Vector3::from_vector3_float(mesh.vertices[mesh.faces[face as usize].v0 as usize]);
            let meet =
                at_the_axis + (a_unit_out * ((corner - at_the_axis).dot(face_normal) / lean));

            if (meet - at_the_axis).dot(a_unit_out) <= 0.0 {
                // Behind the axis: that is the ray's other half, which points into the part.
                continue;
            }

            let carry = (meet - row[k]).length();

            if carry < nearest && the_triangle_holds(mesh, face, face_normal, meet, lattice) {
                nearest = carry;
                held = face;
                at = meet;
            }
        }

        if held < 0 {
            // No facet of the part draws the stretch this sample has walked into.
            return Err(refused());
        }

        row[k] = at;
        carried[k] = held;
    }

    Ok(())
}
