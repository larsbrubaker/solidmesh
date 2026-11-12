//! How much work finding where glass rests on opaque parts does, counted rather than timed: the
//! search carries into world space only the glass parts and the opaque parts near them, never an
//! opaque part far from every glass part however many there are, and a glass part the
//! glass-on-glass search already carried is not carried again.

use solidmesh_math::Matrix4X4;
use solidmesh_mesh::ray_tracer::{
    ContactSearchWork, LightSetup, PlacedPart, PlacedParts, RenderScene, ScatteringSurface,
};

use crate::glass_tray_support::{box_meshes, part_of, rig_lights};
use crate::path_tracer_support::flat_patch;

/// `glass` boxes and a floor of half side 200 at z = 0, then `far` matte boxes, each touching the
/// next, 150 and more away from every glass box. Returns the two searches' work and the face
/// counts of the glass boxes together and of the floor.
fn floor_scene(
    glass: &[([f64; 3], [f64; 3])],
    far: usize,
) -> (ContactSearchWork, ContactSearchWork, usize, usize) {
    let far_boxes: Vec<([f64; 3], [f64; 3])> = (0..far)
        .map(|index| ([20.0; 3], [150.0 + 20.0 * index as f64, 0.0, 10.0]))
        .collect();
    let mut parts: Vec<PlacedPart> = box_meshes(glass)
        .iter()
        .map(|mesh| part_of(mesh, Matrix4X4::IDENTITY))
        .collect();
    let glass_faces: usize = parts.iter().map(|part| part.surface().face_count()).sum();
    let mut surfaces = vec![ScatteringSurface::glass(); parts.len()];
    let floor = part_of(&flat_patch(200.0, 0.0), Matrix4X4::IDENTITY);
    let floor_faces = floor.surface().face_count();
    parts.push(floor);
    for mesh in box_meshes(&far_boxes) {
        parts.push(part_of(&mesh, Matrix4X4::IDENTITY));
    }
    surfaces.resize(parts.len(), ScatteringSurface::matte_plastic([0.3; 3]));
    let lights: LightSetup = rig_lights();
    let scene = RenderScene::new(PlacedParts::new(parts), surfaces, lights).unwrap();
    (
        scene.glass_contact_search_work(),
        scene.glass_rest_search_work(),
        glass_faces,
        floor_faces,
    )
}

#[test]
fn the_resting_search_carries_only_glass_and_what_lies_near_it() {
    for far in [0, 8] {
        let (contacts, rests, glass_faces, floor_faces) =
            floor_scene(&[([20.0; 3], [0.0, 0.0, 10.0])], far);
        println!("{far} far boxes: contacts {contacts:?}, rests {rests:?}");
        assert_eq!(contacts, ContactSearchWork::default(), "one glass part");
        assert_eq!(
            rests.triangles_built,
            glass_faces + floor_faces,
            "{far} far boxes: the search carried more than the glass and the floor"
        );
        assert!(rests.pairs_tested > 0, "the glass rests on the floor");
    }
}

#[test]
fn a_glass_part_is_carried_into_world_space_once_for_both_searches() {
    let touching = [
        ([20.0; 3], [-10.0, 0.0, 10.0]),
        ([20.0; 3], [10.0, 0.0, 10.0]),
    ];
    let (contacts, rests, glass_faces, floor_faces) = floor_scene(&touching, 4);
    println!("contacts {contacts:?}, rests {rests:?}");
    assert_eq!(contacts.triangles_built, glass_faces);
    assert_eq!(
        rests.triangles_built, floor_faces,
        "the resting search carried a glass part again"
    );
    assert!(contacts.pairs_tested > 0 && rests.pairs_tested > 0);
}
