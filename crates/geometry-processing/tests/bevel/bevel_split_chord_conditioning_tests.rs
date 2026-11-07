//! The split-chord pass (`bevel_split_chords`) on the raked bore whose mouth chords a boolean split
//! (`bore_test_meshes`), with the fixture numbers of `surface_raked_bore_split_chord_tests`. The
//! working copy reads the forty chords the bore has, wherever the part stands; a whole mouth is
//! left exactly as it arrived; and only the star of a removed vertex is re-cut.

use std::collections::HashSet;

use geometry_processing::bevel::bevel_split_chords;
use geometry_processing::bevel::bevel_working_mesh::{self, create_from, StampedVerdict};
use polygon_mesh::Mesh;
use vector_math::Vector3;

use crate::bore_test_meshes::create_plate_with_a_raked_bore;
use crate::surface_raked_bore_split_chord_tests::{
    the_chain_length, the_hangers_own_stand_off, the_mouth_group, the_split_part, RADIUS,
    RAKE_DEGREES, SIDES, THE_HANGERS_OWN_LAND, THE_PARTS_OWN_SPLITS, THICKNESS,
};

/// The claim: the working copy of a mouth a boolean split reads as the forty chords the bore has,
/// and says so in a conditioning advisory counting the eight it put back together.
#[test]
fn the_split_chords_are_put_back_together_as_the_one_edge_they_were() {
    let part = the_split_part(RAKE_DEGREES, Vector3::ZERO);

    assert_eq!(
        the_mouths_edges(&part, Vector3::ZERO),
        SIDES as usize + THE_PARTS_OWN_SPLITS.len(),
        "the fixture is the one the boolean split, which is the part's own forty-eight"
    );

    let mut working = create_from(&part, &mut StampedVerdict::default());

    let put = bevel_split_chords::removed_from(&mut working);

    assert_eq!(
        the_mouths_edges(&working, Vector3::ZERO),
        SIDES as usize,
        "a chord cut in two by a point that turns through nothing is one chord"
    );

    assert_eq!(
        put as usize,
        THE_PARTS_OWN_SPLITS.len(),
        "eight chords were split and eight were put back together"
    );

    assert!(working.is_manifold());

    assert_eq!(
        bevel_working_mesh::open_edge_count(&working.faces),
        0,
        "re-triangulating the fan each side of a vertex cannot open the part"
    );
}

/// The same conditioning where the part stands 28 mm out, which is the whole point of the pass:
/// the lattice is 32 times coarser there and the rule has to read the same eight splits anyway.
#[test]
fn the_same_splits_are_read_where_the_hangers_mouth_stands() {
    let part = the_split_part(RAKE_DEGREES, the_hangers_own_stand_off());

    let mut working = create_from(&part, &mut StampedVerdict::default());

    let put = bevel_split_chords::removed_from(&mut working);

    assert_eq!(
        the_mouths_edges(&working, the_hangers_own_stand_off()),
        SIDES as usize,
        "where a part stands cannot decide how many chords its mouth has"
    );

    assert_eq!(
        put as usize,
        THE_PARTS_OWN_SPLITS.len(),
        "the lattice 32 times coarser out here reads the same eight splits"
    );
}

/// THE CONTROL: the same plate with every chord whole. Nothing here is a split chord - the
/// mouth's own nine degree corners turn, and the bore wall is not flat across them - so the
/// working copy is the mesh that went in, face for face and vertex for vertex.
#[test]
fn a_whole_chorded_mouth_is_left_exactly_as_it_arrived() {
    let part = create_plate_with_a_raked_bore(
        SIDES,
        RADIUS,
        THICKNESS,
        RAKE_DEGREES,
        THE_HANGERS_OWN_LAND,
    );

    let faces = faces_of(&part);
    let vertices = part.vertices.clone();

    let mut working = create_from(&part, &mut StampedVerdict::default());

    assert_eq!(
        bevel_split_chords::removed_from(&mut working),
        0,
        "no chord of this mouth is split, so nothing is put back together"
    );

    assert_eq!(working.faces.len(), faces.len());

    assert_eq!(faces_of(&working), faces);

    for (i, vertex) in vertices.iter().enumerate() {
        assert_eq!(working.vertices[i], *vertex);
    }
}

/// WHAT THE PASS IS ALLOWED TO TOUCH: only the star of the vertex it removes. Every triangle of
/// the file that does not have the removed vertex as one of its corners comes out the other side
/// with the same three indices in the same order, the star is re-cut with the fewest triangles a
/// triangulation can have, and every triangle the pass lays winds with the side it belongs to and
/// stands taller than the lattice.
///
/// The first cut of this pass laid each side as a naive fan from the side's own first corner,
/// which is only a triangulation while the side is convex from that corner. On this fixture every
/// side is three corners long and the fan and the ear clip lay the same one triangle - so this pin
/// is here as the CONTRACT: the pass never touches a triangle it has no business in, nor lays one
/// with no height.
#[test]
fn only_the_star_of_a_removed_vertex_is_recut_and_every_other_triangle_is_the_files_own() {
    let part = the_split_part(RAKE_DEGREES, Vector3::ZERO);

    let before = faces_of(&part);

    let mut working = create_from(&part, &mut StampedVerdict::default());

    bevel_split_chords::removed_from(&mut working);

    let after = faces_of(&working);

    let mut still_used: HashSet<i32> = HashSet::new();

    for &(v0, v1, v2) in &after {
        still_used.insert(v0);
        still_used.insert(v1);
        still_used.insert(v2);
    }

    let removed: HashSet<i32> = (0..part.vertices.len() as i32)
        .filter(|vertex| !still_used.contains(vertex))
        .collect();

    assert_eq!(
        removed.len(),
        THE_PARTS_OWN_SPLITS.len(),
        "the eight splits are the only vertices this fixture loses"
    );

    let untouched: Vec<(i32, i32, i32)> = before
        .iter()
        .copied()
        .filter(|face| {
            !removed.contains(&face.0) && !removed.contains(&face.1) && !removed.contains(&face.2)
        })
        .collect();

    let carried: HashSet<(i32, i32, i32)> = after.iter().copied().collect();

    for face in &untouched {
        assert!(
            carried.contains(face),
            "({}, {}, {}) is not at any removed vertex, so the pass has no business re-cutting it",
            face.0,
            face.1,
            face.2
        );
    }

    assert_eq!(
        after.len(),
        before.len() - (2 * removed.len()),
        "a star of n faces round a removed vertex re-cuts as n - 2, which is the fewest a \
         triangulation of what it covered can be, and each split loses one each side"
    );

    let lattice = bevel_split_chords::lattice_for(&working);

    for &(v0, v1, v2) in &after {
        if carried.contains(&(v0, v1, v2)) && untouched.contains(&(v0, v1, v2)) {
            continue;
        }

        let a = Vector3::from_vector3_float(working.vertices[v0 as usize]);
        let b = Vector3::from_vector3_float(working.vertices[v1 as usize]);
        let c = Vector3::from_vector3_float(working.vertices[v2 as usize]);
        let cross = (b - a).cross(c - a);
        let longest = (b - a).length().max((c - b).length().max((a - c).length()));

        assert!(
            cross.length() / longest > lattice,
            "the triangle ({v0}, {v1}, {v2}) the pass laid has to have a height the mesh can \
             measure"
        );
    }
}

/// The faces of a mesh as their three corner indices, in order.
fn faces_of(mesh: &Mesh) -> Vec<(i32, i32, i32)> {
    mesh.faces
        .iter()
        .map(|face| (face.v0, face.v1, face.v2))
        .collect()
}

/// How many edges the pick finds in the upper mouth of a mesh's raked bore - the chain a click on
/// one of its chords resolves to, which is what the construction then reads edge by edge.
fn the_mouths_edges(mesh: &Mesh, standing_at: Vector3) -> usize {
    the_chain_length(mesh, &the_mouth_group(mesh, standing_at, RAKE_DEGREES))
}
