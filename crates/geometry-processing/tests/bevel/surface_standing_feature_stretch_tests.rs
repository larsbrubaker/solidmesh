//! Port of `SurfaceStandingFeatureStretchTests.cs`: what the standing-feature reader will not read,
//! each refused by name. Here: a finely tessellated CURVED wall is not one face
//! (`surface_standing_feature_stretch::try_the_region_of`).

use geometry_processing::bevel::surface::surface_standing_feature_stretch::try_the_region_of;
use geometry_processing::bevel::surface::SurfaceVertexPool;
use geometry_processing::bevel::BevelAdvisoryCode;
use polygon_mesh::Mesh;
use vector_math::{math_helper, Vector3};

/// A finely tessellated CURVED wall is not one face, however small its steps are. The flood that
/// reads a feature's face crosses anything flat to within half a degree, so a wall stepping 0.3
/// degrees a facet is crossed at every step and the whole curve arrives as one "face" - and a face
/// is extended as a PLANE and published as one flat outline, which would silently straighten the
/// curve. Refused by name instead. Asked of the region reader with a wall built by hand, so the
/// guard working cannot be confused with the part being out of reach.
#[test]
fn a_curved_wall_tessellated_finely_enough_to_flood_is_not_one_face() {
    let flat = a_strip_of_wall(0.0);
    let curved = a_strip_of_wall(0.3);

    let flat_pool = SurfaceVertexPool::new(&flat.vertices);

    let whole = try_the_region_of(
        &flat,
        &flat.get_mesh_edge_graph(),
        (flat.faces.len() / 2) as i32,
        &flat_pool,
    )
    .expect("a FLAT wall cut into triangles is one face however many pieces it is in");

    assert_eq!(
        whole.len(),
        flat.faces.len(),
        "and the flood takes the whole of it - that is what makes the region the unit"
    );

    let curved_pool = SurfaceVertexPool::new(&curved.vertices);

    let failure = try_the_region_of(
        &curved,
        &curved.get_mesh_edge_graph(),
        (curved.faces.len() / 2) as i32,
        &curved_pool,
    )
    .expect_err(
        "a wall turning 0.3 degrees a facet is crossed at every step and would arrive as one \
         face, twenty times further from its seed than the flat bound allows",
    );

    assert_eq!(
        failure.code(),
        BevelAdvisoryCode::SurfaceStandingFeatureDoesNotExtend,
        "and it is named rather than flattened"
    );
}

/// A strip of wall 20 facet pairs long, each pair turned `turn_per_facet_degrees` from the last
/// about the y axis - which is flat at zero and a tessellated cylinder otherwise. Open at the rim,
/// which is all the flood needs: the edges it walks across are the inner ones.
fn a_strip_of_wall(turn_per_facet_degrees: f64) -> Mesh {
    let mut mesh = Mesh::new();
    let radius = 40.0;

    let at = |step: i32, y: f64| {
        let angle = math_helper::degrees_to_radians(turn_per_facet_degrees * step as f64);

        if turn_per_facet_degrees == 0.0 {
            Vector3::new(step as f64, y, 0.0)
        } else {
            Vector3::new(radius * angle.sin(), y, radius * (1.0 - angle.cos()))
        }
    };

    for step in 0..20 {
        mesh.create_face(&[at(step, 0.0), at(step + 1, 0.0), at(step + 1, 5.0)]);
        mesh.create_face(&[at(step, 0.0), at(step + 1, 5.0), at(step, 5.0)]);
    }

    mesh.clean_and_merge();

    mesh
}
