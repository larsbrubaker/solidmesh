//! A side SHARED by two of the pieces one bite leaves
//! has to come out of the bite with the same vertex sequence on both of them.
//!
//! Each piece used to decide for itself which of its own corners stood straight through a side and
//! could be left for `surface_split_sewing` to put back. On a side carrying two points a thousandth
//! apart that gives two different answers to one question: each piece keeps the point the other
//! dropped, and the two of them then walk one directed edge the same way, which is a fold the corner
//! patch reports hundreds of faces away from the bite that made it. The parameter rows are a loop.

use geometry_processing::bevel::bevel_chain_linker;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_convex_bite, surface_fan, surface_mesh_closure,
    surface_split_sewing, SurfaceBevelPlan, SurfaceBiteSharedSides, SurfaceVertexPool,
};
use indexmap::IndexSet;
use polygon_mesh::csg::manifold_import;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::{Vector3, Vector3Float};

use crate::lobed_prism_meshes;
use crate::surface_run_out_crosses_a_band_tests::lobed_regions;

const SEGMENTS: i32 = 8;

/// The two pieces the lobe wall's own corner bite leaves either side of one shared edge, measured
/// off the build at radius 0.1: the vertex one of them carries as a corner of ANOTHER side stands
/// within the ruler of the shared one, and putting it into the shared side claims an edge the
/// neighbour does not have.
///
/// Measured: the pieces are [189, 93, 563, 564] out of face (189, 93, 560) and [189, 564, 482, 475]
/// out of face (189, 560, 475), which share the edge from 189 to 564. 482 is where the second
/// piece's own cut line meets its other edge, and the face it came off is a sliver 1.5 thousandths
/// wide, so 482 stands 4.5e-4 off the line from 189 to 564 - inside the part's 5.3e-4 ruler. Grown
/// piece by piece the first piece took it in and the second did not, and the two then walked 564 to
/// 482 the same way round.
#[test]
fn two_pieces_of_one_shared_side_leave_the_bite_with_the_same_sequence() {
    // The lobe prism's OWN vertices, so the pool's ruler is the one the measurement was made with.
    let mut pool = SurfaceVertexPool::new(&lobed_prism_meshes::create().vertices);

    assert!(
        (pool.near_enough() - 5.28e-4).abs() <= 1e-5,
        "the reading below is against the ruler the lobe prism itself asks for - instead {}",
        pool.near_enough()
    );

    let corner = pool.add(Vector3::new(29.8509, 0.0, 19.9)); // 189, the shared side's near end
    let away = pool.add(Vector3::new(40.0, 0.0, 20.0)); // 93
    let top = pool.add(Vector3::new(29.9657, 0.0, 20.0)); // 563
    let far = pool.add(Vector3::new(29.8964, 0.0, 19.9431)); // 564, the shared side's far end
    let elsewhere = pool.add(Vector3::new(29.8910, 0.0, 19.9386)); // 482, a corner of the other side
    let out_of = pool.add(Vector3::new(29.8702, 0.0, 19.9191)); // 475

    let first = vec![corner, away, top, far];
    let second = vec![corner, far, elsewhere, out_of];
    let shared = SurfaceBiteSharedSides::new(vec![first.clone(), second.clone()]);

    assert!(
        surface_split_sewing::stands_on_the_side(
            &pool,
            pool.position_of(far),
            pool.position_of(corner),
            pool.position_of(elsewhere)
        )
        .0,
        "the whole difficulty is that 482 really does measure as standing on the shared side"
    );

    // Every vertex of both pieces is a candidate, which is what the bite hands the growing.
    let carried: IndexSet<i32> = [corner, away, top, far, elsewhere, out_of]
        .into_iter()
        .collect();

    let grown_first = surface_convex_bite::with_the_vertices_standing_on_it(
        &pool,
        first.clone(),
        &carried,
        Some(&shared),
        0,
    );
    let grown_second = surface_convex_bite::with_the_vertices_standing_on_it(
        &pool,
        second.clone(),
        &carried,
        Some(&shared),
        1,
    );

    assert_eq!(
        grown_first, first,
        "the piece across the shared side does not carry 482 on it, so neither does this one"
    );

    assert_eq!(
        grown_second, second,
        "and the second piece is left as the clip made it"
    );

    let mut both = Vec::new();

    surface_fan::emit(&grown_first, &mut pool, &mut both);
    surface_fan::emit(&grown_second, &mut pool, &mut both);

    let (folded, at) = surface_mesh_closure::folded_edge_count(&both);
    assert_eq!(
        folded, 0,
        "so the two of them walk the shared side opposite ways - instead {at:?}"
    );
}

/// A vertex standing on a side that belongs to NO piece of the bite is still taken in: a face the
/// bite never cut carries it as a corner, and spanning it is the T junction the growing is for.
#[test]
fn a_vertex_of_a_face_the_bite_never_cut_is_still_taken_in() {
    let scale = [
        Vector3Float::new(0.0, 0.0, 0.0),
        Vector3Float::new(20.0, 20.0, 20.0),
    ];

    let mut pool = SurfaceVertexPool::new(&scale);

    let left = pool.add(Vector3::new(0.0, 0.0, 0.0));
    let right = pool.add(Vector3::new(10.0, 0.0, 0.0));
    let up = pool.add(Vector3::new(0.0, 10.0, 0.0));
    let standing = pool.add(Vector3::new(5.0, 0.0, 0.0));

    let piece = vec![left, right, up];
    let shared = SurfaceBiteSharedSides::new(vec![piece.clone()]);

    let grown = surface_convex_bite::with_the_vertices_standing_on_it(
        &pool,
        piece,
        &[standing].into_iter().collect(),
        Some(&shared),
        0,
    );

    assert_eq!(
        grown,
        vec![left, standing, right, up],
        "nothing of this bite owns that vertex, so the side it really stands on takes it in"
    );
}

/// The lobe wall's own rim, built whole at every size around the ones that refused. The corner's
/// bites are where the shared side is measured, and the fold they used to emit came back as a
/// refusal at the corner patch.
///
/// Every size comes out closed and manifold. 0.02, 0.09 and 0.1 used to refuse by name, each on
/// one edge used by three faces: a face whose three corners are ONE LINE to within the ruler came
/// out with a pinched ring, and the triangle left claimed an edge the faces either side of it
/// already paired across. A lobe the part cannot tell from a straight line now emits nothing at all
/// (`surface_fan::covers_nothing`, `surface_fan_sliver_ring_tests`). 0.04 was the last size left open
/// before that, and on a different cause: the sewing read each split against the straight chord
/// between a face's corners, while the side had already bowed off that chord at the splits taken
/// into it (`surface_splits_on_a_segment`).
#[test]
fn the_lobe_walls_rim_builds_with_no_fold_in_it() {
    for (radius, open_edges_still_on_the_queue) in [
        (0.02, 0),
        (0.04, 0),
        (0.05, 0),
        (0.09, 0),
        (0.1, 0),
        (0.11, 0),
        (0.25, 0),
    ] {
        let prism = lobed_prism_meshes::create();
        let graph = prism.get_mesh_edge_graph();
        let regions = lobed_regions(&prism);
        let region = regions
            .region(a_facet_of_the_lobes_wall(&prism))
            .expect("a region");
        let mut chains = bevel_chain_linker::link(region.boundary_edges());
        assert_eq!(chains.len(), 1);
        let chain = chains.remove(0);

        let plan = SurfaceBevelPlan::try_create(&chain, &regions, radius, true)
            .unwrap_or_else(|why| panic!("the wall's rim plans at {radius}: {:?}", why.code()));

        let built = surface_bevel_builder::try_build_one(
            &prism,
            &graph,
            &regions,
            plan,
            SEGMENTS,
            &CancelToken::new(),
        )
        .unwrap_or_else(|reason| panic!("the rim has to build at {radius} - instead {reason:?}"));

        let faces: Vec<(i32, i32, i32)> = built
            .faces
            .iter()
            .map(|face| (face.v0, face.v1, face.v2))
            .collect();

        let (folded_count, folded) = surface_mesh_closure::folded_edge_count(&faces);
        assert_eq!(
            folded_count, 0,
            "nothing may double back at radius {radius} - instead {folded:?}"
        );

        let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
        assert_eq!(
            open, open_edges_still_on_the_queue,
            "open at {open_at:?} at radius {radius}"
        );

        if open_edges_still_on_the_queue > 0 {
            continue;
        }

        assert!(
            built.is_manifold(),
            "the mesh at radius {radius} has to be manifold"
        );

        manifold_import::import_strict(&built).unwrap_or_else(|rejected| {
            panic!(
                "the boolean kernel has to take the mesh at radius {radius} - {:?}",
                rejected.status
            )
        });
    }
}

/// Any one facet of the lobe's own wall - a side face standing well outside the flat side.
fn a_facet_of_the_lobes_wall(prism: &Mesh) -> i32 {
    (0..prism.faces.len())
        .find(|&face| {
            let facet = prism.faces[face];
            let middle = (Vector3::from_vector3_float(prism.vertices[facet.v0 as usize])
                + Vector3::from_vector3_float(prism.vertices[facet.v1 as usize])
                + Vector3::from_vector3_float(prism.vertices[facet.v2 as usize]))
                / 3.0;

            middle.y < -lobed_prism_meshes::RADIUS / 2.0
                && middle.z > 1.0
                && middle.z < lobed_prism_meshes::HEIGHT - 1.0
        })
        .expect("the prism has lost the lobe it pokes through its side") as i32
}
