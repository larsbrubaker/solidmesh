//! The section room (`section_room::try_fit_edges`) read
//! with the OPERATION's feature reading, so a smooth wall's room is walked along the surface to a
//! real feature of the part instead of flooded across one facet of its tessellation - the tapered
//! barrel's wall pick (`tapered_barrel_test_meshes`) and "Many Holes"' fifty loops
//! (`many_holes_test_mesh`).

use std::collections::HashSet;

use solidmesh::bevel::bevel_chain_linker::link;
use solidmesh::bevel::sections::section_builder_run::try_build_loop;
use solidmesh::bevel::sections::section_room::{self, SectionRoomCache};
use solidmesh::bevel::sections::SectionPool;
use solidmesh::topology::bevel_edge_group_resolver;

use crate::many_holes_test_mesh;
use crate::tapered_barrel_test_meshes;

/// The size the barrel fixture's claim is asked at - see `section_tapered_barrel_tests`.
const A_SIZE_THE_ROOM_ALLOWS: f64 = 0.35;

/// WHAT THE ROOM NOW SAYS AT THE MOUTH THE FLOOD STOPPED AT: every loop of the barrel's wall pick
/// has room for 0.35, and the old flat-facet reading of 0.1767 is gone.
#[test]
fn the_smooth_walls_room_is_walked_along_the_surface_and_not_flooded_across_one_facet() {
    let part = tapered_barrel_test_meshes::the_working_part();
    let resolution = tapered_barrel_test_meshes::the_wall(&part);
    let graph = part.get_mesh_edge_graph();

    let mut readings: Vec<String> = Vec::new();
    let mut smallest = f64::MAX;

    for one in link(&resolution.edges)
        .iter()
        .filter(|chain| chain.is_closed)
    {
        let ring: Vec<i32> = one.edges.iter().map(|edge| edge.from_vertex()).collect();
        let mut pool = SectionPool::new(&part.vertices);

        let run = try_build_loop(
            &ring,
            Some(&resolution.features),
            &part,
            &graph,
            &mut pool,
            A_SIZE_THE_ROOM_ALLOWS,
            4,
            true,
        )
        .unwrap_or_else(|why| panic!("{why:?}"));

        // The run carries the OPERATION's feature reading - the predicate its own offsets were
        // walked with - so the room is asked with exactly what the build used.
        let is_a_feature = run.is_a_feature_edge.clone();
        let (fits, largest, refusal) = section_room::try_fit_edges(
            Some(&part),
            Some(&graph),
            Some(&run.edges),
            run.is_closed,
            A_SIZE_THE_ROOM_ALLOWS,
            None,
            is_a_feature.as_deref(),
        );

        let mut line = format!("{} stations: {largest:.4}", ring.len());
        if fits {
            line.push_str(" fits");
        } else {
            line.push_str(&format!(
                " REFUSES {}",
                refusal
                    .map(|why| format!("{:?}", why.code()))
                    .unwrap_or_default()
            ));
        }

        smallest = smallest.min(largest);

        let (_, flat_only, _) = section_room::try_fit_edges(
            Some(&part),
            Some(&graph),
            Some(&run.edges),
            run.is_closed,
            A_SIZE_THE_ROOM_ALLOWS,
            None,
            None,
        );

        line.push_str(&format!(" (flat-facet flood alone: {flat_only:.4})"));
        readings.push(line);
    }

    let reading = readings.join("; ");

    assert_eq!(readings.len(), 6, "{reading}");

    assert!(
        smallest > A_SIZE_THE_ROOM_ALLOWS,
        "every loop of the wall pick has room for the size the claim is asked at: {reading}"
    );

    assert!(
        smallest > 0.1767,
        "0.1767 was the flat-region flood stopping at a 7.5 degree ruling of the wall's own \
         tessellation, which is no feature of the part: {reading}"
    );

    // The whole reading, both ways round. The two 32-station mouths are the ones the room used to
    // refuse at 0.1767 and the two 30-station ones at 0.4382; walked along the wall instead, each
    // reaches a real feature of the part - 4.8579 and 6.3375 - which are the numbers the two
    // 48-edge END RIMS read by the flood alone, because a rim's own flat region IS bounded by
    // features. Each rim's limit is the bore mouth straight above or below it: with Pi exact that
    // mouth is centred on a wall fold, so the rim station's walk runs up the fold into the mouth's
    // lowest VERTEX and must stop there as a crease - read as a walk that could not be made, the
    // bottom rim read 14.0082.
    assert_eq!(
        reading,
        "48 stations: 4.8579 fits (flat-facet flood alone: 4.8579); \
         30 stations: 6.3375 fits (flat-facet flood alone: 0.4382); \
         48 stations: 6.3375 fits (flat-facet flood alone: 6.3375); \
         32 stations: 4.8579 fits (flat-facet flood alone: 0.1767); \
         30 stations: 6.3375 fits (flat-facet flood alone: 0.4382); \
         32 stations: 4.8579 fits (flat-facet flood alone: 0.1767)"
    );
}

/// THE ROOM COUNTS ITS CREASES THE WAY THE BUILD DOES: handed the operation's feature reading,
/// every one of "Many Holes"' fifty loops reads a size, where before the half-degree coplanarity
/// test in `section_room`'s side reading called the wall's own 0.9 and 1.2 degree tessellation
/// folds two creases and refused the side with `SurfaceCornerNotSupported` and a
/// largest-that-fits of zero. Read room-only - no build, no assembly - so the whole fifty costs one
/// flood of the wall region; `section_many_holes_tests` pins what the build and the assembly make
/// of the same loops.
#[test]
fn every_loop_of_many_holes_reads_a_room_when_the_creases_are_read_by_feature() {
    let part = many_holes_test_mesh::the_part();
    let graph = part.get_mesh_edge_graph();
    let (mut resolutions, _) =
        bevel_edge_group_resolver::resolve_all(&[many_holes_test_mesh::the_outer_wall()], &part);
    let resolution = resolutions.remove(0);

    let feature_mesh_edges: HashSet<i32> = resolution
        .features
        .edges()
        .iter()
        .map(|feature| feature.mesh_edge_index)
        .collect();
    let is_a_feature = |edge: i32| feature_mesh_edges.contains(&edge);

    let mut cache = SectionRoomCache::new(&part, &graph);
    let mut sizes: Vec<f64> = Vec::new();
    let mut read_zero: Vec<usize> = Vec::new();
    let mut fits = 0;

    for one in link(&resolution.edges)
        .iter()
        .filter(|chain| chain.is_closed)
    {
        let (fit, largest, _) = section_room::try_fit_edges(
            Some(&part),
            Some(&graph),
            Some(&one.edges),
            true,
            many_holes_test_mesh::SIZE,
            Some(&mut cache),
            Some(&is_a_feature),
        );

        if fit {
            fits += 1;
        }

        if largest <= 0.0 {
            read_zero.push(one.edges.len());
        }

        sizes.push(largest);
    }

    let reading = format!(
        "{} loops, {} read zero, {fits} fit {}, largest {:.3}",
        sizes.len(),
        read_zero.len(),
        many_holes_test_mesh::SIZE,
        sizes.iter().copied().fold(f64::MIN, f64::max)
    );

    assert_eq!(sizes.len(), 50);

    assert_eq!(
        fits, 41,
        "forty-one of the fifty loops have room for the 0.5 the owner file asks, where six did \
         while the room counted the wall's tessellation as creases: {reading}"
    );

    // The two that still read nothing are the 65-edge mouth that breaks out through the 104-edge
    // top rim and the rim itself - the pick's one degree-four vertex, where two picked loops pass
    // through one station. That is a real corner of the part, not a fold of its tessellation, and
    // it is the same pair the build refuses
    // (`section_many_holes_tests::the_touching_mouth_and_the_top_rim_both_refuse_at_their_shared_vertex`).
    read_zero.sort_unstable();
    assert_eq!(
        read_zero,
        vec![65, 104],
        "a fold of the wall's tessellation is no crease of the part, so the only loops left \
         without a size are the two that meet at the pick's degree-four vertex: {reading}"
    );
}
