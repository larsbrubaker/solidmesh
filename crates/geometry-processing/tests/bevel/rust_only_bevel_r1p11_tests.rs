//! Rust-only tests for the bevel-core package R1.P11's methods that no ported C# test class covers
//! yet: the section's constructor (`BevelSection`), a chain's sharp turns (`SectionCorner`), the
//! corner pass over a run's stations (`SectionAbsorbedSplits.BehindTheCorners`) and the split
//! blend's small readings (`SectionSplitBlend`). Each pins the C# rule on a hand-built case.

use vector_math::Vector3;

use geometry_processing::bevel::sections::section_split_blend::{self, SplitPlacement};
use geometry_processing::bevel::sections::{
    section_corner, BevelSection, SectionAbsorbedSplits, SectionPool, SectionSide,
};
use geometry_processing::topology::{ChainEdge, FeatureEdge};

/// A chain edge running from `start` to `end`.
fn edge(start: Vector3, end: Vector3) -> ChainEdge {
    ChainEdge::new(
        0,
        FeatureEdge {
            start,
            end,
            direction: (end - start).get_normal(),
            ..Default::default()
        },
        false,
    )
}

/// A section's profile has to run from its left boundary to its right by those very indices.
#[test]
fn rust_only_a_section_is_built_from_its_boundaries() {
    let build = |profile: Vec<i32>| {
        BevelSection::new(
            7,
            Vector3::ZERO,
            Vector3::new(1.0, 0.0, 0.0),
            3,
            9,
            profile,
            Vector3::ZERO,
            Vector3::ZERO,
            SectionSide::Offset,
            SectionSide::Slide,
            -1,
            4,
            0.5,
        )
    };

    let section = build(vec![3, 5, 9]).expect("its profile ends on its boundaries");
    assert_eq!(
        (
            section.chain_vertex,
            section.segments(),
            section.left_walk_face
        ),
        (7, 2, -1)
    );
    assert!(build(vec![3]).is_err());
    assert!(build(vec![3, 5, 8]).is_err());
}

/// A square turn is sharp; a turn of a few degrees is a point of a curve.
#[test]
fn rust_only_a_corner_is_a_sharp_turn() {
    let x = edge(Vector3::ZERO, Vector3::new(10.0, 0.0, 0.0));
    let y = edge(Vector3::new(10.0, 0.0, 0.0), Vector3::new(10.0, 10.0, 0.0));
    let slight = edge(Vector3::new(10.0, 0.0, 0.0), Vector3::new(20.0, 1.0, 0.0));

    assert!((section_corner::turn_degrees(&x, &y) - 90.0).abs() < 1e-9);
    assert!(section_corner::is_sharp_turn(&x, &y));
    assert!(!section_corner::is_sharp_turn(&x, &slight));
    assert!((section_corner::smooth_surface_dot() - 30f64.to_radians().cos()).abs() < 1e-15);
}

/// Round a square with an extra station just past one corner on each edge: the station whose offset
/// boundary stands behind the corner's meet is swallowed by it, on that side, and the pass reports
/// it; an open run is not read.
#[test]
fn rust_only_the_corner_swallows_the_station_behind_its_meet() {
    let corners = [
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(10.0, 0.0, 0.0),
        Vector3::new(10.0, 10.0, 0.0),
        Vector3::new(0.0, 10.0, 0.0),
    ];
    let count = corners.len();
    let edges: Vec<ChainEdge> = (0..count)
        .map(|i| edge(corners[i], corners[(i + 1) % count]))
        .collect();

    let mut pool = SectionPool::new(&[]);
    let mut sections = Vec::new();
    for (i, at) in corners.iter().enumerate() {
        // Each station's offset boundary a unit inwards; the corner at 0 meets at (1, 1), and the
        // station at (1, 0) stands its boundary at (0.5, 1) - behind that meet along the leaving edge.
        let inward = if i == 0 {
            Vector3::new(1.0, 1.0, 0.0)
        } else if i == 1 {
            Vector3::new(0.5, 1.0, 0.0)
        } else {
            *at + Vector3::new(0.0, 0.0, 1.0)
        };
        let boundary = pool.add(inward).unwrap();
        let other = pool
            .add(*at + Vector3::new(0.0, 0.0, -1.0 - i as f64))
            .unwrap();
        sections.push(
            BevelSection::new(
                i as i32,
                *at,
                Vector3::ZERO,
                boundary,
                other,
                vec![boundary, other],
                *at,
                *at,
                SectionSide::Offset,
                SectionSide::Slide,
                -1,
                -1,
                1.0,
            )
            .unwrap(),
        );
    }

    let (left, right) = SectionAbsorbedSplits::behind_the_corners(&sections, &edges, true, &pool)
        .expect("the corner at the origin swallows the station after it");
    assert_eq!(left[1], sections[0].left_boundary);
    assert_eq!(right[1], -1);
    assert!(SectionAbsorbedSplits::behind_the_corners(&sections, &edges, false, &pool).is_none());
}

/// The split blend's small readings: the nearest unsplit neighbour either way round, points and
/// numbers part way between two, how far along the chain a point stands, and the absorb switch.
#[test]
fn rust_only_the_split_blend_reads_its_neighbours() {
    let split = [false, true, true, false, true];
    assert_eq!(
        section_split_blend::nearest_that_is_not_split(&split, 1, 1),
        3
    );
    assert_eq!(
        section_split_blend::nearest_that_is_not_split(&split, 1, -1),
        0
    );
    assert_eq!(
        section_split_blend::nearest_that_is_not_split(&split, 4, 1),
        0
    );
    assert_eq!(
        section_split_blend::nearest_that_is_not_split(&[false, true], 0, 1),
        -1
    );

    let half = section_split_blend::between(Vector3::ZERO, Vector3::new(2.0, 4.0, 0.0), 0.5);
    assert_eq!(half, Vector3::new(1.0, 2.0, 0.0));
    assert_eq!(section_split_blend::between_values(1.0, 3.0, 0.25), 1.5);
    assert_eq!(
        section_split_blend::how_far_along(
            Vector3::new(3.0, 1.0, 0.0),
            Vector3::ZERO,
            Vector3::new(1.0, 0.0, 0.0)
        ),
        3.0
    );
    assert_eq!(SplitPlacement::new(0, 2, 0.5).parameter, 0.5);
    assert!(section_split_blend::absorb_split_vertices());
}

/// A vertex a boolean left part way along a straight crease - the fold between a floor and a wall -
/// reads as a split chord the repair would take out; moved off the line it does not, and says why.
#[test]
fn rust_only_a_vertex_on_a_straight_crease_is_a_split_chord() {
    use geometry_processing::bevel::bevel_split_chords;
    use polygon_mesh::Mesh;
    use vector_math::Vector3Float;

    let build = |middle: Vector3| {
        let mut mesh = Mesh::new();
        for at in [
            middle,
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, -1.0, 0.0),
            Vector3::new(2.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 1.0),
        ] {
            mesh.vertices.push(Vector3Float::from_vector3(at));
        }
        // The ring round vertex 0: floor (z = 0) from 1 through 2 to 3, wall (y = 0) from 3 through 4 to 1.
        for (b, c) in [(1, 2), (2, 3), (3, 4), (4, 1)] {
            let vertices = mesh.vertices.clone();
            mesh.faces.add(0, b, c, &vertices);
        }
        mesh
    };

    let on_the_crease = build(Vector3::new(1.0, 0.0, 0.0));
    assert!(bevel_split_chords::lattice_for(&on_the_crease) > 0.0);
    assert_eq!(bevel_split_chords::would_remove(&on_the_crease, 0), Ok(()));
    assert_eq!(
        bevel_split_chords::faces_at_each_vertex(&on_the_crease)[0]
            .as_ref()
            .map(Vec::len),
        Some(4)
    );
    assert_eq!(
        bevel_split_chords::faces_on_each_edge(&on_the_crease)[&(0, 1)].len(),
        2
    );

    let off_it = build(Vector3::new(1.0, 0.0, 0.25));
    let decline =
        bevel_split_chords::would_remove(&off_it, 0).expect_err("not on the line any more");
    assert!(
        decline.starts_with("creases:") || decline.starts_with("the line:"),
        "{decline}"
    );

    // A vertex on the open rim of this patch: one edge of its fan has a single face.
    assert_eq!(
        bevel_split_chords::would_remove(&on_the_crease, 2),
        Err("unreadable edge to 3, 1 faces on it".to_string())
    );
    assert!(bevel_split_chords::stands_on_the_line_between(
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::ZERO,
        Vector3::new(2.0, 0.0, 0.0),
        1e-9
    ));
    assert!(!bevel_split_chords::stands_on_the_line_between(
        Vector3::new(3.0, 0.0, 0.0),
        Vector3::ZERO,
        Vector3::new(2.0, 0.0, 0.0),
        1e-9
    ));
}
