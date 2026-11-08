//! The rib foot's
//! mixed corner (`SurfaceTestMeshes.CreateRibOnPlate`, fillet r 1). The filler's cross-tangent on
//! each concave foot row must be the fillet's ruling - the chain's direction, x or y - not the
//! normal of the plane through the row's ends, which leaned 35 degrees off it while the rows were
//! skew. The corner cap's hole-fill watch rides on the fill's budget
//! (`hole_fill_corner_tests::fill_with_the_filler_watched`).

use std::cell::RefCell;
use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::corner_cap_hole_fill::arc_cross_tangent;
use solidmesh::bevel::level_line::BandProfile;
use solidmesh_math::Vector3;

use crate::hole_fill_corner_tests::fill_with_the_filler_watched;
use crate::level_line_feature_edges::feature_edges_over;
use crate::several_chains_fixtures::FOLD_DEGREES;
use crate::topology_test_meshes::create_rib_on_plate;

#[test]
fn a_skew_last_rows_cross_tangent_follows_the_chain() {
    let source = create_rib_on_plate();
    let positions: Vec<Vector3> = source
        .vertices
        .iter()
        .map(|&v| Vector3::from_vector3_float(v))
        .collect();
    let indices: Vec<(i32, i32, i32)> = source.faces.iter().map(|f| (f.v0, f.v1, f.v2)).collect();
    let at = |x: f64, y: f64, z: f64| {
        positions
            .iter()
            .position(|&p| (p - Vector3::new(x, y, z)).length() < 1e-9)
            .map_or(-1, |i| i as i32)
    };
    let chains = vec![
        vec![at(8.0, 5.0, 5.0), at(12.0, 5.0, 5.0)],
        vec![at(8.0, 5.0, 5.0), at(8.0, 20.0, 5.0)],
        vec![at(8.0, 5.0, 5.0), at(8.0, 5.0, 15.0)],
    ];
    let closed = [false, false, false];
    let gap = band_gap_entry_points::build_pick(
        &positions,
        &indices,
        &chains,
        &closed,
        Some(feature_edges_over(&positions, &indices, FOLD_DEGREES)),
        1.0,
        BandProfile::Fillet,
        &BandGapBuildOptions::default(),
    )
    .expect("the pick builds or refuses by name");
    assert!(gap.built());

    let corner = Vector3::new(8.0, 5.0, 5.0);
    let skew = Rc::new(RefCell::new(Vec::<Vector3>::new()));
    let seen = Rc::clone(&skew);
    fill_with_the_filler_watched(
        &gap,
        BandProfile::Fillet,
        8,
        Some(Rc::new(move |mesh, _fill, _n, _triangles, rows| {
            for row in rows {
                let p = mesh.position(row.arc[0]);
                let q = mesh.position(row.arc[row.arc.len() - 1]);
                // the two concave foot rows at the corner (the free ends' rows stand metres away)
                if row.concave && (p - corner).length() < 2.5 && (q - corner).length() < 2.5 {
                    seen.borrow_mut().push(arc_cross_tangent(
                        mesh,
                        row,
                        mesh.position(row.arc[row.arc.len() / 2]),
                        corner,
                    ));
                }
            }
        })),
    );

    let skew = skew.borrow();
    println!(
        "foot rows: {}",
        skew.iter()
            .map(|t| format!("({:.3},{:.3},{:.3})", t.x, t.y, t.z))
            .collect::<Vec<_>>()
            .join(" ")
    );
    // both concave foot chains end at this corner: the end face's (along x) and the rib side's
    // (along y)
    assert_eq!(skew.len(), 2);
    for tangent in skew.iter() {
        assert!((tangent.x.abs().max(tangent.y.abs()) - 1.0).abs() <= 1e-9);
    }
}
