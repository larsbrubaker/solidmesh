//! A face region whose rim folds back on itself at
//! a knife edge still closes. The walkback will not cut a region it cannot see the whole rim of,
//! and asks the chain linker that over the region's own boundary edges. The shape is the nose in
//! the notch, whose slopes' rims turn through the nose's knife edge at the ridge end.

use std::sync::Arc;

use solidmesh_math::Vector3;

use solidmesh::bevel::bevel_chain_linker;
use solidmesh::bevel::surface::SurfaceBevelPlan;

use crate::nose_in_notch_test_meshes;
use crate::surface_walkback_checks::regions_of;

/// Each slope of the nose-in-notch part is rimmed by one closed loop, at the knife edge included.
#[test]
fn a_slope_whose_rim_turns_through_the_noses_knife_edge_still_closes() {
    let part = nose_in_notch_test_meshes::create();
    let (regions, _) = regions_of(&Arc::new(part.clone()));

    for (name, normal) in [
        ("left", nose_in_notch_test_meshes::left_slope_normal()),
        ("right", nose_in_notch_test_meshes::right_slope_normal()),
    ] {
        // The slope is fanned into five triangles, so the face this picks is not unique - but the
        // REGION behind them has to be.
        let facets: Vec<i32> = (0..part.faces.len())
            .filter(|&i| {
                (Vector3::from_vector3_float(part.faces[i].normal) - normal).length() < 1e-6
            })
            .map(|i| i as i32)
            .collect();

        let flooded: Vec<_> = facets
            .iter()
            .map(|&i| regions.region(i).expect("a facet has a region"))
            .collect();
        let mut keys: Vec<i32> = flooded
            .iter()
            .map(|r| SurfaceBevelPlan::key_of(r))
            .collect();
        // Distinct keys; only the count is read.
        keys.sort_unstable();
        keys.dedup();

        assert_eq!(
            keys.len(),
            1,
            "the {name} slope's {} facets flood to one region, not {}",
            facets.len(),
            keys.len()
        );

        let region = &flooded[0];

        assert_eq!(
            region.open_boundary_mesh_edges().len(),
            0,
            "the part is closed, so nothing on the {name} slope's rim is an open edge"
        );

        let loops = bevel_chain_linker::link(region.boundary_edges());

        assert_eq!(
            loops.len(),
            1,
            "the {name} slope has one rim, not {}",
            loops.len()
        );
        assert_eq!(
            loops[0].edges.len(),
            region.boundary_edges().len(),
            "the whole {name} rim is in that one loop"
        );
        assert!(
            loops[0].is_closed,
            "the {name} rim comes back to where it started, through the ridge end at {}",
            loops[0]
                .edges
                .iter()
                .map(|edge| format!("{}->{}", edge.from_vertex(), edge.to_vertex()))
                .collect::<Vec<_>>()
                .join(" ")
        );
    }
}
