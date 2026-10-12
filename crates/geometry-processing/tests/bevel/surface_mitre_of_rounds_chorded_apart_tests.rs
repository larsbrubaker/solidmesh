//! Port of `SurfaceMitreOfRoundsChordedApartTests.cs`: TWO ARCS THAT WERE NOT CHORDED THE SAME
//! WAY, mitred anyway. The mitre polyline is the UNION of both strips' rail crossings ordered by
//! their angle from the face the two beads share, and each strip's end station carries the OTHER's
//! crossings as points BETWEEN two of its own rails.
//!
//! **Why the pairing works at all, and what breaks the index.** Two beads of one radius tangent to
//! the shared face both carry their rail at angle theta in the plane `r (1 - cos theta)` above it,
//! so rail theta of one crosses rail theta of the other on the mitre plane whatever the two wedges
//! are. What the construction did care about was that step `k` is the same theta in both, and
//! `bevel_arc_chording::segments_for` ROUNDS the chord count per arc, so two sweeps whose ratio does
//! not survive that rounding are chorded apart.
//!
//! **The two cases here.** Concave: the corner "Center And Right Face.mcx" dies at, 15 chords of
//! exactly 6 degrees against 11 of 6.0824. Convex: the x = 0 face rim walked at 15 segments, where
//! 90 and 135 degrees take 15 and 23 chords of 6 and 5.8696 and do not pair. C# `[Arguments]` rows
//! are a loop.

use geometry_processing::bevel::surface::surface_mesh_closure;
use geometry_processing::topology::{
    bevel_edge_group_resolver, BevelEdgeGroup, FaceGroup, FeatureEdgeSet,
};
use polygon_mesh::{CancelToken, Mesh};
use vector_math::Vector3;

use crate::mesh_measurements::{shell_count, signed_volume};
use crate::nose_in_notch_cut::anchor;
use crate::nose_in_notch_test_meshes as nose;
use crate::surface_test_build::{expect_built, try_bevel, TryBevelOptions};

/// Where the source's own boolean chords the upper right crease - its t = 0.6.
fn the_chord() -> Vector3 {
    Vector3::new(12.0, 8.0, 12.0)
}

/// THE FIXTURE'S OWN PICK: four groups, the corner at (10, 10, 6.667) chorded 6.0824 against 6,
/// and a solid that closes. Taken at two radii because the two steps are pure ANGLE - a radius that
/// changed the answer would mean the rails were not being paired by angle at all.
#[test]
fn the_fixtures_groups_build_where_the_two_arcs_were_chorded_apart() {
    for radius in [1.0, 2.0] {
        let part = nose::create_with_upper_right_crease_split_at(the_chord());

        let mesh = expect_built(
            try_bevel(
                &part,
                &the_four_groups(&part),
                radius,
                15,
                &CancelToken::new(),
                TryBevelOptions::default(),
            ),
            &format!(
                "the two arcs are chorded apart and the mitre takes the union of their rails \
                 (radius {radius})"
            ),
        );

        assert_it_is_a_sound_solid(&mesh);

        assert!(
            signed_volume(&mesh) < nose::VOLUME,
            "the three Edge picks are concave and only ever ADD, but the fourth group is the whole \
             rim of the right slope, three of whose eight edges are the cube's own convex corners - \
             so the part's {:.3} comes down on balance",
            nose::VOLUME
        );
    }
}

/// THE CONVEX SIDE, on the rim that already mitres its wide bead leaving AND arriving: the same
/// four edges of the x = 0 face, walked at 15 segments instead of 16 so that the 135 degree knife
/// edge takes 23 chords of 5.8696 against the three right angles' 15 of 6.
#[test]
fn the_closed_rim_mitres_when_its_two_widths_are_chorded_apart() {
    let part = nose::create();
    let features = FeatureEdgeSet::create(&part);

    let size = nose::SIZE;

    let groups = vec![
        anchor(
            &features,
            Vector3::new(0.0, 0.0, size),
            Vector3::new(0.0, 0.0, 0.0),
        ),
        anchor(
            &features,
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, size, 0.0),
        ),
        anchor(
            &features,
            Vector3::new(0.0, size, 0.0),
            Vector3::new(0.0, size, size),
        ),
        anchor(
            &features,
            Vector3::new(0.0, size, size),
            Vector3::new(0.0, 0.0, size),
        ),
    ];

    let mesh = expect_built(
        try_bevel(
            &part,
            &groups,
            2.0,
            15,
            &CancelToken::new(),
            TryBevelOptions::default(),
        ),
        "the rim closes at 16 segments, where both widths take 5.625 degree steps, and has to close \
         at 15 as well, where they do not",
    );

    assert_it_is_a_sound_solid(&mesh);

    assert!(
        signed_volume(&mesh) < nose::VOLUME,
        "the rim is four convex edges, and a convex bevel takes material off"
    );
}

/// And the guard on the other side: a corner whose two arcs DO take the one step is untouched by
/// any of this. The notch's centre crease below the nose against the cube's own floor edge, both
/// right angles, both 15 chords of 6 degrees - paired rail for rail on one vertex each.
#[test]
fn the_same_corner_shape_still_pairs_rail_for_rail_when_both_arcs_take_the_same_step() {
    let part = nose::create_with_upper_right_crease_split_at(the_chord());
    let features = FeatureEdgeSet::create(&part);

    let groups = vec![
        anchor(&features, nose::notch_floor(), nose::lower_crease_end()),
        anchor(&features, nose::notch_floor(), Vector3::new(20.0, 0.0, 0.0)),
    ];

    let mesh = expect_built(
        try_bevel(
            &part,
            &groups,
            1.0,
            15,
            &CancelToken::new(),
            TryBevelOptions::default(),
        ),
        "both arcs take the one step",
    );

    assert_it_is_a_sound_solid(&mesh);

    let volume = signed_volume(&mesh);

    assert!(
        (volume - 6053.6847).abs() <= 1e-3,
        "the concave centre crease is 6.667 long and only ever ADDS, the convex floor edge is \
         14.142 and only ever takes away, so the pair lands under the part's own {:.3} - this is \
         {volume}",
        nose::VOLUME
    );
}

/// Closed, manifold, one shell, and no triangle with two corners at one point.
fn assert_it_is_a_sound_solid(mesh: &Mesh) {
    let (open, open_at) = surface_mesh_closure::open_edge_count(mesh);

    assert_eq!(
        open, 0,
        "nothing may be left open, and something is at {open_at:?}"
    );

    assert!(
        mesh.is_manifold(),
        "closure and manifoldness together ARE the edge pairing: the two strips' ends only leave \
         no open edge and no edge used three times if they sewed to ONE polyline, vertex for vertex"
    );

    assert_eq!(shell_count(mesh), 1);

    assert_eq!(
        two_corners_at_one_point(mesh),
        0,
        "a triangle with two corners at one point covers nothing and shades as a crack"
    );
}

/// How many triangles have two corners at one point, which is a triangle covering nothing.
fn two_corners_at_one_point(mesh: &Mesh) -> i32 {
    let mut found = 0;

    for face in mesh.faces.iter() {
        let v0 = Vector3::from_vector3_float(mesh.vertices[face.v0 as usize]);
        let v1 = Vector3::from_vector3_float(mesh.vertices[face.v1 as usize]);
        let v2 = Vector3::from_vector3_float(mesh.vertices[face.v2 as usize]);

        if (v0 - v1).length() < 1e-9 || (v1 - v2).length() < 1e-9 || (v2 - v0).length() < 1e-9 {
            found += 1;
        }
    }

    found
}

/// The fixture's own pick, on the kernel's own copy of the part: three creases and a face.
fn the_four_groups(part: &Mesh) -> Vec<BevelEdgeGroup> {
    let features = FeatureEdgeSet::create(part);
    let graph = part.get_mesh_edge_graph();

    // The right slope, y = 20 - x: the face the fourth group floods over, whose eight-edge rim
    // re-names both of the right creases the Edge groups already picked.
    let slope = (0..part.faces.len())
        .find(|&face| {
            let f = &part.faces[face];
            let at = (Vector3::from_vector3_float(part.vertices[f.v0 as usize])
                + Vector3::from_vector3_float(part.vertices[f.v1 as usize])
                + Vector3::from_vector3_float(part.vertices[f.v2 as usize]))
                / 3.0;

            (at.x + at.y - nose::SIZE).abs() < 1e-6 && at.x > nose::SIZE / 2.0
        })
        .expect("the right slope is a face of the part") as i32;

    let region = FaceGroup::create_with_graph(
        part,
        &graph,
        slope,
        FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
    )
    .expect("the slope floods");

    assert_eq!(
        region.boundary_edges().len(),
        8,
        "the right slope's rim is eight edges with the crease chorded"
    );

    vec![
        anchor(&features, nose::left_ridge_end(), nose::right_ridge_end()),
        anchor(&features, nose::right_ridge_end(), the_chord()),
        anchor(&features, nose::right_ridge_end(), nose::lower_crease_end()),
        bevel_edge_group_resolver::create_from_face_group(
            &region,
            &region.boundary_edges()[0],
            features.flat_angle_degrees(),
        )
        .expect("the slope's rim bounds it"),
    ]
}
