//! A feature STANDING inside a band's footprint, carried out over the corner the band took until it
//! meets the fillet - Lars's ruling, and what the Onshape control builds. The fixture is the nose
//! in its notch: at fifteen the band takes the whole top face and runs off the x = 0 rim,
//! undercutting the nose's two left creases, so the nose's facets are held back by the walkback,
//! notched into the strip and extended to the trace where they meet it. The exact solid at fifteen
//! is 5563.424, and a chorded strip closes on it from below.

use std::sync::Arc;

use geometry_processing::bevel::bevel_chain_linker::BevelSweptChain;
use geometry_processing::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, SurfaceBevelPlan,
};
use geometry_processing::topology::ChainEdge;
use polygon_mesh::csg::manifold_import;
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::nose_in_notch_test_meshes;
use crate::surface_standing_feature_stretch_tests::{plan_of, top_edge_of};
use crate::surface_walkback_checks::regions_of;

/// The ridge's EXIT at each chording - where the nose's ridge leaves the strip.
fn exit_at(segments_per_quarter: i32) -> Vector3 {
    if segments_per_quarter == 8 {
        Vector3::new(3.7028, 5.0, 10.0)
    } else {
        Vector3::new(3.7598, 5.0, 10.0)
    }
}

/// The two Qs - where each extended plane's trace meets the strip's end ring.
fn qs_at(segments_per_quarter: i32) -> [Vector3; 2] {
    if segments_per_quarter == 8 {
        [
            Vector3::new(7.0848, 12.9152, 15.2768),
            Vector3::new(9.8891, 10.1109, 6.5927),
        ]
    } else {
        [
            Vector3::new(7.1197, 12.8803, 15.2536),
            Vector3::new(9.9330, 10.0670, 6.6220),
        ]
    }
}

/// Closed, manifold, one shell and taken by the boolean kernel, each failure saying which.
fn assert_closes(built: &Mesh, open_because: &str) {
    let (folded, folded_at) = surface_mesh_closure::folded_edge_count(&faces_of(built));
    assert_eq!(folded, 0, "nothing may double back - instead {folded_at:?}");
    let (open, open_at) = surface_mesh_closure::open_edge_count(built);
    assert_eq!(open, 0, "{open_because} - open at {open_at}");
    assert!(built.is_manifold(), "and close manifold");
    assert_eq!(
        shell_count(built),
        1,
        "in one piece - the nose is carried out of the part, not left beside it"
    );
}

fn assert_the_kernel_takes_it(built: &Mesh) {
    assert!(
        manifold_import::import_strict(built).is_ok(),
        "the boolean kernel has to take it"
    );
}

/// Rungs one to four together: the solid closes, the exit is ONE vertex, the Qs are one vertex each
/// and no loose end, and the volume is the chorded figure.
#[test]
fn the_nose_is_carried_out_over_the_corner_the_band_took() {
    for (segments_per_quarter, volume) in [(8, 5555.302), (16, 5561.387)] {
        let built = the_fillet_on_the_top_left_edge(
            nose_in_notch_test_meshes::create(),
            15.0,
            segments_per_quarter,
        );

        assert_closes(
            &built,
            "the extension, the notched strip and the split cap close on shared vertices",
        );
        assert_the_kernel_takes_it(&built);

        let exit = exit_at(segments_per_quarter);

        assert_eq!(
            vertices_at(&built, exit),
            1,
            "the ridge leaves the strip at ONE point, {exit}, minted once by the notch and handed to \
             both extended facets, to the strip's own tail and to nothing else"
        );
        assert!(
            carries(&built, exit) >= 4,
            "and four rims run through it - the two facets and the two sides of the notch"
        );

        for q in qs_at(segments_per_quarter) {
            assert_eq!(
                vertices_at(&built, q),
                1,
                "each trace meets the end ring at one point, {q}, shared by the facet, the strip's \
                 tail and the cap's own piece"
            );
            assert!(
                carries(&built, q) >= 3,
                "all three of those rims run through it, so it is no loose end"
            );
        }

        let measured = signed_volume(&built);
        assert!(
            (measured - volume).abs() <= 0.5,
            "chorded at {segments_per_quarter} segments to the quarter, closing on the exact \
             5563.424 from below - {measured}, expected {volume}"
        );
    }
}

/// The same notch at a size the band does NOT run off the top face's far rim at, so its end LANDS
/// on the face: 13.5 and 14, the two sizes between the nose and the 14.142 the face measures. The
/// exact volumes, worked out in closed form, are 5636.041 at 13.5 and 5611.816 at 14.
#[test]
fn the_nose_is_carried_out_over_a_landing_end_as_well() {
    for (size, segments_per_quarter, volume, exact, exit_x, upper_qx, lower_qx) in [
        (13.5, 8, 5628.921, 5636.041, 4.26152, 8.00728, 9.98891),
        (13.5, 16, 5634.250, 5636.041, 4.32628, 8.06413, 9.99434),
        (14.0, 8, 5604.352, 5611.816, 4.07528, 7.69978, 9.95563),
        (14.0, 16, 5609.937, 5611.816, 4.17013, 7.79333, 9.97737),
    ] {
        let built = the_fillet_on_the_top_left_edge(
            nose_in_notch_test_meshes::create(),
            size,
            segments_per_quarter,
        );

        assert_closes(
            &built,
            "the extension, the notched strip and the split cap close on shared vertices at a \
             LANDING end too",
        );
        assert_the_kernel_takes_it(&built);

        // The three points the tail and the split cap are sewn from: the ridge's exit on the strip,
        // and each extended plane's Q on the end ring.
        let exit = Vector3::new(exit_x, 5.0, 10.0);

        assert_eq!(
            vertices_at(&built, exit),
            1,
            "the ridge leaves the strip at ONE point, {exit}, handed to both extended facets and to \
             the strip's own tail"
        );
        assert!(
            carries(&built, exit) >= 4,
            "and four rims run through it - the two facets and the two sides of the notch"
        );

        for q in [
            Vector3::new(upper_qx, 20.0 - upper_qx, 0.0),
            Vector3::new(lower_qx, 20.0 - lower_qx, 0.0),
        ] {
            // The Qs stand ON the band's end plane x + y = 20, so the z the trace meets it at is
            // the plane's own reading: the nose's upper wall is z = (2y + 20)/3 and its lower wall
            // z = (40 - 2y)/3.
            let at = Vector3::new(
                q.x,
                q.y,
                if q.x < 9.0 {
                    ((2.0 * q.y) + 20.0) / 3.0
                } else {
                    (40.0 - (2.0 * q.y)) / 3.0
                },
            );

            assert_eq!(
                vertices_at(&built, at),
                1,
                "each trace meets the end ring at one point, {at}, shared by the facet, the strip's \
                 tail and the cap's own piece"
            );
            assert!(
                carries(&built, at) >= 3,
                "all three of those rims run through it, so it is no loose end"
            );
        }

        let measured = signed_volume(&built);

        assert!(
            (measured - volume).abs() <= 0.5,
            "chorded at {segments_per_quarter} segments to the quarter, closing on the exact \
             {exact} from below - {measured}, expected {volume}"
        );
        assert!(
            measured <= exact,
            "a chord of a convex arc lies INSIDE it, so the cut can only take more than the exact \
             {exact}, never less - built {measured}"
        );
    }
}

/// The same solid whichever way round the chain was linked: which END of the band the feature's
/// extension runs off through is the chain's own business.
#[test]
fn the_chain_walked_the_other_way_builds_the_same_solid() {
    for (size, segments_per_quarter, volume) in [
        (15.0, 8, 5555.302),
        (15.0, 16, 5561.387),
        (13.5, 8, 5628.921),
        (13.5, 16, 5634.250),
        (14.0, 8, 5604.352),
        (14.0, 16, 5609.937),
        (7.0, 8, 5920.426),
        (7.0, 16, 5922.570),
        (10.0, 8, 5796.345),
        (10.0, 16, 5800.003),
    ] {
        let part = Arc::new(nose_in_notch_test_meshes::create());
        let (regions, features) = regions_of(&part);
        let regions = Arc::new(regions);
        let index = top_edge_of(
            &features,
            Vector3::new(0.0, 0.0, 20.0),
            Vector3::new(10.0, 10.0, 20.0),
        );
        let backwards = BevelSweptChain {
            edges: vec![ChainEdge::new(index as i32, features.edges()[index], true)],
            is_closed: false,
        };

        let plan = SurfaceBevelPlan::try_create(&backwards, &regions, size, true)
            .unwrap_or_else(|reason| panic!("the backwards chain has to plan: {reason}"));

        let built = surface_bevel_builder::try_build_one(
            &part,
            &part.get_mesh_edge_graph(),
            &regions,
            plan,
            segments_per_quarter,
            &CancelToken::new(),
        )
        .unwrap_or_else(|reason| panic!("the backwards chain has to build at {size}: {reason:?}"));

        let (folded, folded_at) = surface_mesh_closure::folded_edge_count(&faces_of(&built));
        assert_eq!(
            folded, 0,
            "the strip may not be stitched through its own notch - instead {folded_at:?}"
        );
        let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
        assert_eq!(open, 0, "and it closes - open at {open_at}");
        assert!(built.is_manifold());
        assert_eq!(shell_count(&built), 1);

        let measured = signed_volume(&built);
        assert!(
            (measured - volume).abs() <= 0.5,
            "which is the same solid the forward walk builds, to the last decimal the chording \
             allows - {measured} at {size} and {segments_per_quarter}, expected {volume}"
        );
    }
}

/// The same build with the nose's upper face cut into three triangles about a point inside it -
/// which is what a boolean leaves. The ridge is where the two faces' PLANES meet and does not care
/// how either of them is triangulated, so the solid has to come out the same.
#[test]
fn the_ridge_is_where_the_planes_meet_however_the_nose_is_triangulated() {
    for (segments_per_quarter, volume) in [(8, 5555.302), (16, 5561.387)] {
        let built = the_fillet_on_the_top_left_edge(
            nose_in_notch_test_meshes::create_with_the_upper_face_fanned_from_inside(),
            15.0,
            segments_per_quarter,
        );

        let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
        assert_eq!(
            open, 0,
            "the extension closes on the same trace whichever way the nose was cut up - open at \
             {open_at}"
        );
        assert!(built.is_manifold(), "and closes manifold");
        assert_eq!(shell_count(&built), 1, "in one piece");

        assert_eq!(
            vertices_at(&built, exit_at(segments_per_quarter)),
            1,
            "and the ridge leaves the strip at the very same point it did before"
        );

        let measured = signed_volume(&built);
        assert!(
            (measured - volume).abs() <= 0.5,
            "which is the same solid as the whole-faced nose builds at {segments_per_quarter} \
             segments - {measured}, expected {volume}"
        );
    }
}

/// The sizes at which the band undercuts ONE wall of the nose only - 7 and 10 (and 9.9 and 9.99
/// either side of ten's knife edge), where the LOWER crease is never reached: the trace starts
/// where the upper crease runs out of strip, on the seam rail, and the end's section is cut in two.
#[test]
fn the_band_that_undercuts_one_wall_carries_that_wall_out_alone() {
    for (size, segments_per_quarter, volume, exact, crease_crossing_x, q_x) in [
        (7.0, 8, 5920.426, 5923.287, 9.5, 9.97566),
        (7.0, 16, 5922.570, 5923.287, 9.5, 9.98815),
        (9.9, 8, 5800.885, 5805.709, 5.15, 9.51718),
        (9.9, 16, 5804.500, 5805.709, 5.15, 9.52799),
        (9.99, 8, 5796.800, 5801.676, 5.015, 9.49763),
        (9.99, 16, 5800.454, 5801.676, 5.015, 9.50526),
        (10.0, 8, 5796.345, 5801.226, 5.0, 9.49545),
        (10.0, 16, 5800.003, 5801.226, 5.0, 9.50273),
    ] {
        let built = the_fillet_on_the_top_left_edge(
            nose_in_notch_test_meshes::create(),
            size,
            segments_per_quarter,
        );

        let (folded, folded_at) = surface_mesh_closure::folded_edge_count(&faces_of(&built));
        assert_eq!(folded, 0, "nothing may double back - instead {folded_at:?}");
        let (open, open_at) = surface_mesh_closure::open_edge_count(&built);
        assert_eq!(
            open, 0,
            "the one extended wall, the notched strip and the split cap close on shared vertices - \
             and the wall the band did NOT undercut goes back exactly as the part had it. Open at \
             {open_at}"
        );
        assert!(built.is_manifold(), "and close manifold");
        assert_eq!(shell_count(&built), 1, "in one piece");

        assert_the_kernel_takes_it(&built);

        // Where the crease runs out of strip, on the crease's own line: it climbs two in z for
        // every three along the slope from the ridge end at (5, 5, 10).
        let crossing = Vector3::new(
            crease_crossing_x,
            crease_crossing_x,
            10.0 + ((crease_crossing_x - 5.0) * 2.0 / 3.0),
        );

        assert_eq!(
            vertices_at(&built, crossing),
            1,
            "the crease leaves the strip at ONE point, {crossing} - the vertex the walkback's own \
             seam ends on, handed to the extended wall and to the strip's tail and never read a \
             second time off two planes ({size}, {segments_per_quarter})"
        );
        assert!(
            carries(&built, crossing) >= 3,
            "and the wall, the strip and the landing face all run through it"
        );

        // The Q stands on the band's end plane x + y = 20 and on the nose's upper wall,
        // z = (2y + 20)/3.
        let q = Vector3::new(q_x, 20.0 - q_x, ((2.0 * (20.0 - q_x)) + 20.0) / 3.0);

        assert_eq!(
            vertices_at(&built, q),
            1,
            "the trace meets the end ring at one point, {q}, shared by the wall, the strip's tail \
             and the cap's own piece"
        );
        assert!(carries(&built, q) >= 3, "so it is no loose end");

        assert_eq!(
            on_the_ridge_line(&built),
            2,
            "and there is NO exit: the ridge never reaches the strip at these sizes, so the only two \
             vertices on its line are the part's own two ridge ends"
        );

        let measured = signed_volume(&built);

        assert!(
            (measured - volume).abs() <= 0.5,
            "chorded at {segments_per_quarter} segments to the quarter, closing on the exact \
             {exact} from below - {measured}, expected {volume}"
        );
        assert!(
            measured <= exact,
            "a chord of a convex arc lies INSIDE it, so the cut can only take more than the exact \
             {exact}, never less - built {measured}"
        );
    }
}

/// How many vertices of the built mesh stand on the nose's RIDGE LINE, y = 5 and z = 10.
fn on_the_ridge_line(built: &Mesh) -> usize {
    // The float coordinates are read in float arithmetic, then compared as a double.
    built
        .vertices
        .iter()
        .filter(|vertex| {
            ((vertex.y - 5.0).abs() as f64) < 1e-3 && ((vertex.z - 10.0).abs() as f64) < 1e-3
        })
        .count()
}

/// The nose fixture's fillet on the left slope's top edge, built whole.
fn the_fillet_on_the_top_left_edge(part: Mesh, size: f64, segments_per_quarter: i32) -> Mesh {
    let part = Arc::new(part);
    let (regions, features) = regions_of(&part);
    let regions = Arc::new(regions);
    let plan = plan_of(
        &regions,
        &features,
        top_edge_of(
            &features,
            Vector3::new(0.0, 0.0, 20.0),
            Vector3::new(10.0, 10.0, 20.0),
        ),
        size,
        true,
    );

    surface_bevel_builder::try_build_one(
        &part,
        &part.get_mesh_edge_graph(),
        &regions,
        plan,
        segments_per_quarter,
        &CancelToken::new(),
    )
    .unwrap_or_else(|reason| panic!("the build refused the top left edge at {size}: {reason:?}"))
}

fn faces_of(built: &Mesh) -> Vec<(i32, i32, i32)> {
    built
        .faces
        .iter()
        .map(|face| (face.v0, face.v1, face.v2))
        .collect()
}

/// How many distinct VERTICES of the built mesh stand at one point, within a rounding.
fn vertices_at(built: &Mesh, at: Vector3) -> usize {
    built
        .vertices
        .iter()
        .filter(|&&vertex| (Vector3::from_vector3_float(vertex) - at).length() < 1e-3)
        .count()
}

/// How many FACES of the built mesh stand on one point.
fn carries(built: &Mesh, at: Vector3) -> usize {
    built
        .faces
        .iter()
        .filter(|face| {
            [face.v0, face.v1, face.v2].iter().any(|&vertex| {
                (Vector3::from_vector3_float(built.vertices[vertex as usize]) - at).length() < 1e-3
            })
        })
        .count()
}
