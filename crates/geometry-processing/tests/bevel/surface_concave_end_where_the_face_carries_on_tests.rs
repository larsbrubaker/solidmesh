//! The concave mirror of the end where
//! the face carries on - a bead whose crease stops at the REFLEX corner of the very face it is
//! being beaded against, with no single face standing across that corner to cut it off. The plan
//! refuses that chain and names the corner; the control (one face across the corner, which
//! `surface_end_across_the_corner` answers) still plans.
//!
//! The shape: a 20 x 20 x 10 slab with a block standing on its cap over `x in [10, 20],
//! y in [12, 20]`, leaving the cap L shaped with its reflex corner at `(10, 12, 10)`.

use std::collections::HashSet;
use std::sync::Arc;

use solidmesh::bevel::bevel_chain_linker::{self, BevelSweptChain};
use solidmesh::bevel::surface::SurfaceBevelPlan;
use solidmesh::bevel::{BevelAdvisoryArg, BevelAdvisoryCode};
use solidmesh::topology::{EdgeConvexity, FaceRegionCache, FeatureEdgeSet};
use solidmesh_math::{Vector2, Vector3};
use solidmesh_mesh::Mesh;

use crate::surface_walkback_checks::regions_of;

const RADIUS: f64 = 2.0;

/// The cap's reflex corner, which is where the crease stops and what the refusal names.
fn reflex_corner() -> Vector3 {
    Vector3::new(10.0, 12.0, 10.0)
}

#[test]
fn a_bead_ending_at_its_own_faces_reflex_corner_with_no_face_across_is_refused() {
    let part = slab_with_a_wedge_touching_down_at_the_corner();
    let (regions, features) = regions_of(&part);
    let regions = Arc::new(regions);

    let chain = chain_of(&features, the_wedges_front_crease(&features));

    assert_eq!(
        chain.edges[0].edge.convexity,
        EdgeConvexity::Concave,
        "the wedge stands ON the cap, so where its front wall meets the cap is a bead's crease"
    );

    // What makes this the case it is: the wedge's sloping top and its left wall BOTH stand at
    // the corner, and they are two planes.
    assert_eq!(
        surfaces_standing_at(&part, &regions, &chain, reflex_corner()),
        2,
        "two of the wedge's own surfaces meet the cap at that corner, so neither is 'the' face across"
    );

    let reason = match SurfaceBevelPlan::try_create(&chain, &regions, RADIUS, true) {
        Ok(_) => panic!(
            "a bead running out at a reflex corner has nothing beyond it to lean on, and a cap \
             taken in some other plane than the one the band stopped at is not an answer"
        ),
        Err(reason) => reason,
    };

    assert_eq!(
        reason.code(),
        BevelAdvisoryCode::SurfaceStripEndNotSupported,
        "the refusal is about the chain's end, not its size or its faces - {reason:?}"
    );

    assert_eq!(reason.args().len(), 1, "and it says where - {reason:?}");

    match reason.args()[0] {
        BevelAdvisoryArg::Vector3(at) => assert!(
            (at - reflex_corner()).length() < 1e-9,
            "the corner named is the one the crease stopped at - {reason:?}"
        ),
        ref other => panic!("the argument is not a point: {other:?}"),
    }
}

/// The control the refusal above is measured against: the SAME shape with the block's top left
/// flat, so that its left wall is the one surface standing at the corner.
#[test]
fn the_same_bead_with_one_face_across_the_corner_still_plans() {
    let part = slab_with_a_box_on_its_cap();
    let (regions, features) = regions_of(&part);
    let regions = Arc::new(regions);

    let chain = chain_of(&features, the_wedges_front_crease(&features));

    assert_eq!(
        surfaces_standing_at(&part, &regions, &chain, reflex_corner()),
        1,
        "a flat topped box leaves only its left wall standing at the corner"
    );

    let plan =
        SurfaceBevelPlan::try_create(&chain, &regions, RADIUS, true).unwrap_or_else(|reason| {
            panic!("one face across the corner is a plane to cut the bead off on - {reason:?}")
        });

    assert!(
        plan.start_trim.is_some() || plan.end_trim.is_some(),
        "and that plane is what the band is given to stop at"
    );
}

fn chain_of(features: &FeatureEdgeSet, edge_index: usize) -> BevelSweptChain {
    let mut chains = bevel_chain_linker::link(&[features.edges()[edge_index]]);
    assert_eq!(chains.len(), 1);
    chains.remove(0)
}

/// How many distinct surfaces stand at `corner` that are neither of the two the chain's end is
/// walked back into - read here off the mesh so the test's own claim about the fixture is checked
/// rather than assumed.
fn surfaces_standing_at(
    part: &Mesh,
    regions: &FaceRegionCache,
    chain: &BevelSweptChain,
    corner: Vector3,
) -> usize {
    let mut being_walked_back = HashSet::new();

    for side in [chain.edges[0].left_face(), chain.edges[0].right_face()] {
        if let Some(region) = regions.region(side) {
            being_walked_back.extend(region.faces().iter().copied());
        }
    }

    let mut surfaces: Vec<Vector3> = Vec::new();

    for index in 0..part.faces.len() {
        if !touches(part, index, corner) || being_walked_back.contains(&(index as i32)) {
            continue;
        }

        let normal = Vector3::from_vector3_float(part.faces[index].normal).get_normal();

        if !surfaces
            .iter()
            .any(|already| already.dot(normal) > 1.0 - 1e-6)
        {
            surfaces.push(normal);
        }
    }

    surfaces.len()
}

fn touches(part: &Mesh, face_index: usize, corner: Vector3) -> bool {
    let face = &part.faces[face_index];

    [face.v0, face.v1, face.v2].iter().any(|&vertex| {
        (Vector3::from_vector3_float(part.vertices[vertex as usize]) - corner).length() < 1e-9
    })
}

/// The crease along `y = 12, z = 10` where the standing block's front wall meets the cap, which
/// is the one edge beaded here.
fn the_wedges_front_crease(features: &FeatureEdgeSet) -> usize {
    let found: Vec<usize> = (0..features.count())
        .filter(|&i| {
            let edge = &features.edges()[i];

            (edge.start.y - 12.0).abs() < 1e-9
                && (edge.end.y - 12.0).abs() < 1e-9
                && (edge.start.z - 10.0).abs() < 1e-9
                && (edge.end.z - 10.0).abs() < 1e-9
        })
        .collect();
    assert_eq!(found.len(), 1, "expected one front crease, found {found:?}");
    found[0]
}

/// A 20 x 20 x 10 slab with a wedge standing on its cap over `x in [10, 20], y in [12, 20]`,
/// whose top is the plane `z = 10 + (x - 10) / 2 + (y - 12) / 2` - zero thickness at the cap's
/// reflex corner `(10, 12, 10)` and rising from there, so that the top and the wedge's left wall
/// both reach that corner.
fn slab_with_a_wedge_touching_down_at_the_corner() -> Arc<Mesh> {
    fn top(at: Vector2) -> Vector3 {
        Vector3::new(
            at.x,
            at.y,
            10.0 + ((at.x - 10.0) / 2.0) + ((at.y - 12.0) / 2.0),
        )
    }

    standing_on_the_slab(
        top(Vector2::new(10.0, 12.0)),
        top(Vector2::new(20.0, 12.0)),
        top(Vector2::new(20.0, 20.0)),
        top(Vector2::new(10.0, 20.0)),
    )
}

/// The same slab with a plain flat topped box on it instead, so that the only surface standing at
/// the reflex corner is the box's left wall.
fn slab_with_a_box_on_its_cap() -> Arc<Mesh> {
    standing_on_the_slab(
        Vector3::new(10.0, 12.0, 16.0),
        Vector3::new(20.0, 12.0, 16.0),
        Vector3::new(20.0, 20.0, 16.0),
        Vector3::new(10.0, 20.0, 16.0),
    )
}

/// The 20 x 20 x 10 slab with a block standing on the quarter of its cap at `x >= 10, y >= 12`,
/// roofed by the four points given - planar and in the block's own footprint order, corner
/// (10, 12) first.
fn standing_on_the_slab(
    a_top: Vector3,
    b_top: Vector3,
    c_top: Vector3,
    d_top: Vector3,
) -> Arc<Mesh> {
    let a = Vector3::new(10.0, 12.0, 10.0);
    let b = Vector3::new(20.0, 12.0, 10.0);
    let c = Vector3::new(20.0, 20.0, 10.0);
    let d = Vector3::new(10.0, 20.0, 10.0);

    let mut mesh = Mesh::new();

    // The slab's floor and its four walls. The walls carry the extra corners the cap and the
    // block need on their shared rims, or the part is a T junction rather than a solid.
    mesh.create_face(&[
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 20.0, 0.0),
        Vector3::new(20.0, 20.0, 0.0),
        Vector3::new(20.0, 0.0, 0.0),
    ]);
    mesh.create_face(&[
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(20.0, 0.0, 0.0),
        Vector3::new(20.0, 0.0, 10.0),
        Vector3::new(0.0, 0.0, 10.0),
    ]);
    mesh.create_face(&[
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, 10.0),
        Vector3::new(0.0, 20.0, 10.0),
        Vector3::new(0.0, 20.0, 0.0),
    ]);
    mesh.create_face(&[
        Vector3::new(20.0, 0.0, 0.0),
        Vector3::new(20.0, 20.0, 0.0),
        c,
        b,
        Vector3::new(20.0, 0.0, 10.0),
    ]);
    mesh.create_face(&[
        Vector3::new(0.0, 20.0, 0.0),
        Vector3::new(0.0, 20.0, 10.0),
        d,
        c,
        Vector3::new(20.0, 20.0, 0.0),
    ]);

    // The cap, less the block's footprint: an L with its reflex corner at a = (10, 12, 10),
    // fanned from (0, 0, 10), which the whole of it is visible from.
    mesh.create_face(&[
        Vector3::new(0.0, 0.0, 10.0),
        Vector3::new(20.0, 0.0, 10.0),
        b,
        a,
        d,
        Vector3::new(0.0, 20.0, 10.0),
    ]);

    // The block: front wall on the crease a-b, left wall, and the two walls flush with the
    // slab's. A wall whose roof touches the cap down at one of its own corners is a TRIANGLE
    // there, which is what `face` drops the repeated corner for.
    face(&mut mesh, &[a, b, b_top, a_top]);
    face(&mut mesh, &[b, c, c_top, b_top]);
    face(&mut mesh, &[c, d, d_top, c_top]);
    face(&mut mesh, &[d, a, a_top, d_top]);
    face(&mut mesh, &[a_top, b_top, c_top, d_top]);

    // create_face adds fresh vertices every call, so what is built above is triangle soup.
    mesh.clean_and_merge();

    Arc::new(mesh)
}

/// One face, with any corner repeated from the one before it dropped.
fn face(mesh: &mut Mesh, corners: &[Vector3]) {
    let mut kept: Vec<Vector3> = Vec::with_capacity(corners.len());

    for &corner in corners {
        if kept.is_empty() || (kept[kept.len() - 1] - corner).length() > 1e-9 {
            kept.push(corner);
        }
    }

    if kept.len() > 2 && (kept[0] - kept[kept.len() - 1]).length() < 1e-9 {
        kept.pop();
    }

    if kept.len() > 2 {
        mesh.create_face(&kept);
    }
}
