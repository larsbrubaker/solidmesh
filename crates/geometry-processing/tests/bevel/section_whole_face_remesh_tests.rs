//! Port of `SectionWholeFaceRemeshTests.cs`: THE WHOLE FLAT FACE handed to the re-mesh, on a
//! chamfered L shelf whose top face is the long thin fan a boolean leaves, with a turn-zero split
//! vertex part way along its long bottom edge. The top rim's 0.4 fillet builds through the sections
//! closed, manifold and one shell, no face of the answer stands outside the shelf, and every rigid
//! motion answers with the same topology. The rim is reached through the FACE GROUP pick's own plan,
//! made through the shared `SurfaceTestBuild` port.

use std::rc::Rc;

use geometry_processing::bevel::sections::section_builder_walked_side::SectionBuildRefusal;
use geometry_processing::bevel::sections::{section_assembly, section_builder_run, SectionPool};
use geometry_processing::bevel::surface::{surface_mesh_closure, SurfaceBevelPlan};
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelSelectionKind, FaceGroup, FeatureEdgeSet,
};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{Matrix4X4, Vector2, Vector3, Vector3Float};

use crate::mesh_measurements::shell_count;
use crate::rigid_motion_meshes::{most_faces_on_an_edge, move_mesh, signature_of, the_motions};
use crate::surface_test_build::{try_bevel, TryBevelOptions};

const RADIUS: f64 = 0.4;

const SEGMENTS: i32 = 8;

const BEND_THRESHOLD_DEGREES: f64 = 20.0;

const HEIGHT: f64 = 4.0;

/// The whole point: the shelf's top rim fillets, where the partial region refused it.
#[test]
fn the_chamfered_ell_top_rim_builds() {
    let part = create_chamfered_ell_shelf();
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(&part, &graph, BEND_THRESHOLD_DEGREES);
    let plan = the_top_rim(&part, &features, Matrix4X4::IDENTITY);

    // The edge count and the open ends are named here because an open rim is how a mis-cut top face
    // shows up: a degenerate triangle swallows its two rim edges and the walk comes back six of
    // eight.
    assert!(
        plan.is_closed,
        "the fixture's top rim is one closed loop - {} edges, from {:?} to {:?}",
        plan.edges.len(),
        plan.edges[0].edge.from(),
        plan.edges[plan.edges.len() - 1].edge.to()
    );

    let mesh = try_build_the_top_rim(&part, Matrix4X4::IDENTITY).unwrap_or_else(|failure| {
        panic!("the 0.4 fillet fits this shelf's top face; refused {failure:?}")
    });

    assert_eq!(
        surface_mesh_closure::open_edge_count(&mesh).0,
        0,
        "the answer is closed"
    );

    assert_eq!(most_faces_on_an_edge(&mesh), 2, "the answer is manifold");

    assert_eq!(shell_count(&mesh), 1, "the answer is one shell");
}

/// No triangle of the answer stands outside the shelf's own outline: the poking kept triangle the
/// partial region left behind would show up here as a top-plane face outside the original L.
#[test]
fn no_face_stands_outside_the_shelf() {
    let part = create_chamfered_ell_shelf();

    let mesh = try_build_the_top_rim(&part, Matrix4X4::IDENTITY)
        .unwrap_or_else(|failure| panic!("the fixture refused {failure:?}"));

    let outline = the_outline();

    for drawn in mesh.faces.iter() {
        for corner in [drawn.v0, drawn.v1, drawn.v2] {
            let at = Vector3::from_vector3_float(mesh.vertices[corner as usize]);

            assert!(
                is_inside_the_outline(&outline, at),
                "({:.3}, {:.3}, {:.3}) stands outside the shelf",
                at.x,
                at.y,
                at.z
            );
        }
    }
}

/// Every rigid motion of the fixture answers with the same topology: the whole-region rule is a fact
/// about the face and not about which way the part happens to be turned.
#[test]
fn the_answer_is_the_same_under_every_rigid_motion() {
    let motions = the_motions();
    let mut at_the_origin = None;

    for (motion_name, motion) in &motions {
        let mut part = create_chamfered_ell_shelf();

        move_mesh(&mut part, *motion);

        let mesh = try_build_the_top_rim(&part, *motion)
            .unwrap_or_else(|failure| panic!("{motion_name} refused {failure:?}"));

        let signature = signature_of(Some(&mesh));

        if *motion_name == motions[0].0 {
            at_the_origin = Some(signature);
        } else {
            let at_the_origin = at_the_origin.as_ref().expect("the first motion ran first");

            assert_eq!(
                signature.faces, at_the_origin.faces,
                "{motion_name} moved the face count"
            );
            assert_eq!(
                signature.vertices, at_the_origin.vertices,
                "{motion_name} moved the vertex count"
            );
        }
    }
}

/// C# `TheOutline`: the shelf's top outline, counter clockwise seen from +z - an L whose outer
/// corner is chamfered at 45 degrees, with one turn-zero split vertex sitting part way along the
/// long bottom edge, the mark a boolean leaves where another body's wall broke through.
fn the_outline() -> [Vector2; 8] {
    [
        Vector2::new(0.0, 0.0),
        Vector2::new(8.0, 0.0),
        Vector2::new(24.0, 0.0),
        Vector2::new(24.0, 6.0),
        Vector2::new(20.0, 10.0),
        Vector2::new(6.0, 10.0),
        Vector2::new(6.0, 24.0),
        Vector2::new(0.0, 24.0),
    ]
}

/// C# `TheTopTriangles`: how the top face is cut up - a long thin fan off the outline's first point,
/// with the one pair of triangles at the split vertex taken off the fan.
///
/// The split vertex (8, 0) stands COLLINEAR with the fan's apex (0, 0) and with (24, 0), so the fan
/// triangle that would carry it has zero area and no normal; the quad 0-1-2-3 is cut across 1-3
/// instead, which keeps the split vertex on a real triangle.
fn the_top_triangles() -> [(i32, i32, i32); 6] {
    [
        (0, 1, 3),
        (1, 2, 3),
        (0, 3, 4),
        (0, 4, 5),
        (0, 5, 6),
        (0, 6, 7),
    ]
}

/// C# `CreateChamferedEllShelf`: the outline lofted down by [`HEIGHT`], its top and bottom drawn as
/// ONE fan off the outline's first point, which is how a boolean leaves a face it did not have to
/// re-cut.
fn create_chamfered_ell_shelf() -> Mesh {
    let outline = the_outline();
    let mut mesh = Mesh::new();

    for at in &outline {
        mesh.vertices.push(Vector3Float::new_f32(
            at.x as f32,
            at.y as f32,
            HEIGHT as f32,
        ));
    }

    for at in &outline {
        mesh.vertices
            .push(Vector3Float::new_f32(at.x as f32, at.y as f32, 0.0));
    }

    let count = outline.len() as i32;

    for (a, b, c) in the_top_triangles() {
        mesh.faces.add(a, b, c, &mesh.vertices);
        mesh.faces
            .add(count + a, count + c, count + b, &mesh.vertices);
    }

    for i in 0..count {
        let next = (i + 1) % count;

        mesh.faces.add(i, count + i, count + next, &mesh.vertices);
        mesh.faces.add(i, count + next, next, &mesh.vertices);
    }

    mesh.calculate_normals();

    mesh
}

/// C# `TheTopRim`: the whole top rim as one plan, reached through the FACE GROUP pick and not
/// through a tangent walk - three feature edges meet at every corner of an L prism, so a tangent
/// walk stops at the first corner whatever tolerance it is given.
fn the_top_rim(mesh: &Mesh, features: &FeatureEdgeSet, motion: Matrix4X4) -> Rc<SurfaceBevelPlan> {
    let group = bevel_edge_group_resolver::create_from_edge(
        features,
        the_seed_rim_edge(features, motion),
        BevelSelectionKind::FaceGroup,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("the seed is a crease of the shelf");

    // The legacy build's own answer is not read here - only the plan it makes on the way, which is
    // filled in before the room is fitted, so a legacy refusal still hands the rim over.
    let outcome = try_bevel(
        mesh,
        &[group],
        RADIUS,
        SEGMENTS,
        &CancelToken::new(),
        TryBevelOptions::default(),
    );

    assert_eq!(
        outcome.plans_built.len(),
        1,
        "the top face pick planned {} chains, not one",
        outcome.plans_built.len()
    );

    Rc::clone(&outcome.plans_built[0])
}

/// C# `TheSeedRimEdge`: an edge of the shelf's top rim - both ends on the top plane, wherever the
/// part stands.
fn the_seed_rim_edge(features: &FeatureEdgeSet, motion: Matrix4X4) -> i32 {
    let was_the_top = Vector3::new(0.0, 0.0, HEIGHT).transform_position(motion);

    for (i, edge) in features.edges().iter().enumerate() {
        if (edge.start - was_the_top).dot(up(motion)).abs() < 1e-4
            && (edge.end - was_the_top).dot(up(motion)).abs() < 1e-4
        {
            return i as i32;
        }
    }

    panic!("the fixture has no feature edge on its top plane");
}

/// C# `TryBuildTheTopRim`: the shelf's top rim built and assembled through the sections path.
fn try_build_the_top_rim(part: &Mesh, motion: Matrix4X4) -> Result<Mesh, SectionBuildRefusal> {
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(part, &graph, BEND_THRESHOLD_DEGREES);
    let plan = the_top_rim(part, &features, motion);
    let mut pool = SectionPool::new(&part.vertices);

    let run = section_builder_run::try_build_plan(
        Some(plan),
        part,
        &graph,
        &mut pool,
        RADIUS,
        SEGMENTS,
        None,
    )?;

    section_assembly::try_assemble(part, &graph, &run, &mut pool)
}

/// C# `Up`: the way the shelf's top plane faces after a motion.
fn up(motion: Matrix4X4) -> Vector3 {
    (Vector3::new(0.0, 0.0, 1.0).transform_position(motion)
        - Vector3::new(0.0, 0.0, 0.0).transform_position(motion))
    .get_normal()
}

/// C# `IsInsideTheOutline`: whether a point stands inside the shelf's outline in xy, read with a
/// small slack. The outline is CONCAVE at the L's inner corner, so this is the winding number.
fn is_inside_the_outline(outline: &[Vector2], at: Vector3) -> bool {
    winding_of(outline, Vector2::new(at.x, at.y)) != 0
}

/// C# `WindingOf`: the winding number of the outline about a point, with points on the outline
/// counted in.
fn winding_of(outline: &[Vector2], at: Vector2) -> i32 {
    const SLACK: f64 = 1e-4;
    let mut winding = 0;

    for i in 0..outline.len() {
        let from = outline[i];
        let to = outline[(i + 1) % outline.len()];
        let side = (to - from).cross(at - from);

        if side.abs() <= SLACK && (at - from).dot(at - to) <= SLACK {
            // On the outline itself, which the rim's own points are.
            return 1;
        }

        if from.y <= at.y {
            if to.y > at.y && side > 0.0 {
                winding += 1;
            }
        } else if to.y <= at.y && side < 0.0 {
            winding -= 1;
        }
    }

    winding
}
