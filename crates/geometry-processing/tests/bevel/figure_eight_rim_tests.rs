//! A rim that touches ITSELF. On a real part (Thingi10K 1458669, pick 0) a face's boundary loop
//! came back as a figure eight - two lobes of one face meeting at a single vertex, so the loop
//! names that vertex TWICE - and the argument check threw "chain 0 names vertex 139 twice". A crash
//! where a refusal belongs: picks are COLLECTED off a mesh, not typed by a programmer, so a
//! figure-eight rim is a statement about the part and leaves by name.

use geometry_processing::bevel::level_line::band_gap::BandGapBuildError;
use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::band_gap_reading;
use geometry_processing::bevel::level_line::{BandGapRefusal, BandProfile};
use vector_math::{Vector2, Vector3};

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::SolidWithChain;

/// The vertex the two lobes of the top face meet at - the one the rim names twice.
const SHARED_CORNER: i32 = 2;

const HEIGHT: f64 = 1.0;

/// Every entry point - the build, the room reading and the bare room - refuses by name, never
/// throws, and names the chain and the vertex it comes back through.
#[test]
fn a_figure_eight_rim_refuses_by_name_through_every_entry_point() {
    let (positions, indices, rim) = bowtie_prism();
    let chains = vec![rim];
    let closed = [true];

    let gap = band_gap_entry_points::build_pick(
        &positions,
        &indices,
        &chains,
        &closed,
        Some(feature_edges_over(&positions, &indices, 30.0)),
        0.2,
        BandProfile::Chamfer,
        &BandGapBuildOptions::default(),
    )
    .expect("a figure eight is refused by name, not thrown");
    assert_eq!(
        gap.refusal,
        BandGapRefusal::ChainTouchesItself,
        "a rim through one vertex twice is a statement about the part, not a bad argument"
    );
    assert_eq!(gap.offending_chain, 0);
    assert_eq!(gap.offending_vertex, SHARED_CORNER);
    assert_eq!(
        gap.room, 0.0,
        "no size builds this pick, so the zero is not a size statement"
    );

    let reading = band_gap_reading::read_room_of_pick(
        &positions,
        &indices,
        &chains,
        &closed,
        Some(feature_edges_over(&positions, &indices, 30.0)),
        BandProfile::Chamfer,
        1e-9,
        None,
        f64::NAN,
        0.0,
        f64::INFINITY,
    )
    .expect("a figure eight is read by name, not thrown");
    assert!(
        reading.read,
        "a fault is an answer, not a reading that ran out of work"
    );
    assert!(!reading.chain_is_simple);
    assert_eq!(reading.offending_chain, 0);
    assert_eq!(reading.offending_vertex, SHARED_CORNER);

    let room = band_gap_reading::room_of_pick(
        &positions,
        &indices,
        &chains,
        &closed,
        Some(feature_edges_over(&positions, &indices, 30.0)),
        BandProfile::Chamfer,
        1e-9,
    )
    .expect("a figure eight's room is a number, not a throw");
    assert_eq!(room, 0.0);
}

/// The same, said of the part wherever it stands: a refusal about the pick's own topology cannot
/// depend on where in the world the part is.
#[test]
fn the_refusal_is_invariant_under_a_rigid_motion() {
    let (positions, indices, rim) = bowtie_prism();
    let moved: Vec<Vector3> = positions
        .iter()
        .map(|p| {
            let turned = Vector3::new(
                (p.x * 0.9_f64.cos()) - (p.z * 0.9_f64.sin()),
                p.y,
                (p.x * 0.9_f64.sin()) + (p.z * 0.9_f64.cos()),
            );
            turned + Vector3::new(-7.25, 3.5, 11.75)
        })
        .collect();

    let gap = band_gap_entry_points::build_pick(
        &moved,
        &indices,
        &[rim],
        &[true],
        Some(feature_edges_over(&moved, &indices, 30.0)),
        0.2,
        BandProfile::Chamfer,
        &BandGapBuildOptions::default(),
    )
    .expect("a figure eight is refused by name, not thrown");

    assert_eq!(gap.refusal, BandGapRefusal::ChainTouchesItself);
    assert_eq!(gap.offending_vertex, SHARED_CORNER);
}

/// The line between the two: a chain of ONE vertex is a programmer's mistake about the CALL and
/// still throws. Only what a collected pick can honestly hand back leaves by name.
#[test]
fn a_chain_of_one_vertex_is_still_an_argument_fault() {
    let (positions, indices, _) = bowtie_prism();
    let result = band_gap_entry_points::build_pick(
        &positions,
        &indices,
        &[vec![0]],
        &[true],
        Some(feature_edges_over(&positions, &indices, 30.0)),
        0.2,
        BandProfile::Chamfer,
        &BandGapBuildOptions::default(),
    );

    assert!(matches!(result, Err(BandGapBuildError::Arguments(_))));
}

/// The smallest solid whose top face's boundary is a figure eight: two unit squares of plan meeting
/// at one corner, extruded. The corner is ONE vertex, so the top rim walks round one lobe, through
/// it, round the other, and back through it again.
///
/// The upright edge at the shared corner stands on four side walls rather than two, which is
/// exactly what a pinched region looks like and is not what the refusal rests on: the chain's own
/// topology is read before the mesh is looked at at all.
fn bowtie_prism() -> SolidWithChain {
    // Plan, counter-clockwise round the whole figure: lobe A is [0,1]x[0,1], lobe B is [1,2]x[1,2].
    let plan = [
        Vector2::new(0.0, 0.0),
        Vector2::new(1.0, 0.0),
        Vector2::new(1.0, 1.0),
        Vector2::new(2.0, 1.0),
        Vector2::new(2.0, 2.0),
        Vector2::new(1.0, 2.0),
        Vector2::new(0.0, 1.0),
    ];

    let mut positions = Vec::new();
    for q in &plan {
        positions.push(Vector3::new(q.x, q.y, HEIGHT));
    }

    for q in &plan {
        positions.push(Vector3::new(q.x, q.y, 0.0));
    }

    let n = plan.len() as i32;
    let mut indices = Vec::new();
    let mut quad = |a: i32, b: i32, c: i32, d: i32| {
        indices.push((a, b, c));
        indices.push((a, c, d));
    };

    // Each lobe of the top face, wound outward (+Z), and the same two below wound -Z.
    quad(0, 1, 2, 6);
    quad(2, 3, 4, 5);
    quad(n + 6, n + 2, n + 1, n);
    quad(n + 5, n + 4, n + 3, n + 2);

    // The eight side walls, following the rim as it walks the figure eight.
    let rim = vec![0, 1, 2, 3, 4, 5, 2, 6];
    for i in 0..rim.len() {
        let a = rim[i];
        let b = rim[(i + 1) % rim.len()];
        quad(a, n + a, n + b, b);
    }

    (positions, indices, rim)
}
