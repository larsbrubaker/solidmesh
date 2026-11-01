//! Every face's region labelled in one pass (`FaceRegionCache::label_every_face`, built on
//! `FaceGroup::label_surfaces_fenced`) held to the regions the cache floods a face at a time, with
//! and without a fence: what the rolling ball's sides are read from in the model's Surface
//! builder, which must name the same sides the pick shows the user.

use std::sync::Arc;

use geometry_processing::topology::FaceRegionCache;
use polygon_mesh::Mesh;

use crate::topology_test_meshes;

const SIDES: i32 = 16;

fn cylinder() -> Arc<Mesh> {
    Arc::new(topology_test_meshes::create_cylinder(10.0, 10.0, SIDES))
}

/// The wall's faces (their normals level), in mesh order.
fn wall_faces(mesh: &Mesh) -> Vec<i32> {
    (0..mesh.faces.len() as i32)
        .filter(|&face| mesh.faces[face as usize].normal.z.abs() < 0.001)
        .collect()
}

/// The faces sharing `face`'s label, in mesh order.
fn labelled_with(labels: &[i32], face: i32) -> Vec<i32> {
    (0..labels.len() as i32)
        .filter(|&other| labels[other as usize] == labels[face as usize])
        .collect()
}

/// Labels are dense from zero and first met in ascending face order: the region of the lowest
/// face is 0, the next region to show up is 1, and so on.
fn assert_numbered_by_lowest_face(labels: &[i32]) {
    let mut next = 0;
    for &label in labels {
        assert!(
            label >= 0 && label <= next,
            "label {label} skips past {next}"
        );
        if label == next {
            next += 1;
        }
    }
}

/// Without a fence, at a tolerance where every facet is a region of its own (the faceted import
/// read at a low bend threshold) and at one where the wall is one region, the one-pass labels
/// group every face exactly as the cache's flood from that face does.
#[test]
fn one_pass_labels_are_the_regions_the_cache_floods_without_a_fence() {
    let mesh = cylinder();
    for tolerance in [2.0, 30.0] {
        let cache = FaceRegionCache::new(mesh.clone(), None, tolerance);
        let labels = cache.label_every_face();

        assert_eq!(labels.len(), mesh.faces.len());
        assert_numbered_by_lowest_face(&labels);
        for face in 0..mesh.faces.len() as i32 {
            let region = cache.region(face).expect("every face has a region");
            assert_eq!(
                labelled_with(&labels, face),
                region.faces(),
                "face {face} at {tolerance} degrees"
            );
        }

        let distinct = labels.iter().max().map_or(0, |&top| top + 1);
        let expected = if tolerance < 10.0 { SIDES + 2 } else { 3 };
        assert_eq!(
            distinct, expected,
            "a region per facet, or the wall and two caps"
        );
    }
}

/// With a fence, every face outside it is labelled with the region the cache floods from it (the
/// flood never entering the fence), and every fenced face is a region of its own. The fenced faces
/// are the wall's lowest ones, so a fenced face comes before every other wall face: the cache's
/// flood seeded on it lifts the fence for that seed and runs out over the whole wall, and a
/// labelling that shared that flood out would put the wall into the fenced face's region.
#[test]
fn one_pass_labels_give_each_fenced_face_a_region_of_its_own() {
    let mesh = cylinder();
    let wall = wall_faces(&mesh);
    let fence: Vec<i32> = wall[..2].to_vec();
    let rest_of_the_wall: Vec<i32> = wall[2..].to_vec();
    assert!(
        fence.iter().all(|&barrier| barrier < rest_of_the_wall[0]),
        "the fixture's fenced faces come before the rest of the wall"
    );

    for tolerance in [2.0, 30.0] {
        let cache =
            FaceRegionCache::with_barriers(mesh.clone(), None, tolerance, Some(fence.clone()));
        let labels = cache.label_every_face();

        assert_numbered_by_lowest_face(&labels);
        for face in 0..mesh.faces.len() as i32 {
            if fence.contains(&face) {
                assert_eq!(
                    labelled_with(&labels, face),
                    vec![face],
                    "fenced face {face}"
                );
                continue;
            }
            let region = cache.region(face).expect("every face has a region");
            assert_eq!(
                labelled_with(&labels, face),
                region.faces(),
                "face {face} at {tolerance} degrees"
            );
        }
    }

    // At 30 degrees the wall is one surface: the flood from a fenced face runs over all of it, yet
    // the wall's own faces share one label that no fenced face has.
    let cache = FaceRegionCache::with_barriers(mesh.clone(), None, 30.0, Some(fence.clone()));
    let labels = cache.label_every_face();
    let from_the_fence = cache.region(fence[0]).expect("a fenced face floods");
    assert!(from_the_fence.faces().contains(&rest_of_the_wall[0]));
    assert_eq!(
        labelled_with(&labels, rest_of_the_wall[0]),
        rest_of_the_wall
    );
    for &barrier in &fence {
        assert_ne!(
            labels[barrier as usize],
            labels[rest_of_the_wall[0] as usize]
        );
    }
}
