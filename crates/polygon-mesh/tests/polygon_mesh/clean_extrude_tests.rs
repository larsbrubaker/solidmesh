//! Every path extrudes to clean geometry: a closed solid (each edge walked once in each direction
//! on shared vertices), wound outward, with no face of zero area, whose caps lie flat facing out
//! and cover exactly the area the path fills, no more (no overlapping slivers) and no less (no
//! holes in the cap). The fixtures are a traced picture's outline (`test-data/ControlVertexSources/
//! Image Silhouette Trace.svgd`, the path Image to Path saved in `test-data/mcx/Image Silhouette
//! Extrude.mcx`) and small paths that each stress one way a loop set can be awkward to fill.
//!
//! The checker and the paths are shared with the other path-to-solid operations
//! (`clean_solid_support.rs`).

use agg_path::VertexStorage;
use clipper_library::ClipperError;
use polygon_mesh::processors::vertex_source_to_mesh::extrude;

use super::clean_solid_support::{
    assert_clean_solid, even_odd_area, loops_of, path_of, square_at, traced_silhouette,
    AREA_FRACTION,
};

const HEIGHT: f64 = 3.0;

/// Extrudes the path and holds it to the rule, against the path's own even-odd area, within
/// `tolerance` mm² beyond what the clean-up's point merging moves (see [`AREA_FRACTION`]). Every
/// loop of the path is also on the top of the solid by at least one of its own points, so no
/// part of the shape was dropped, however small.
fn assert_extrudes_within(
    path: &VertexStorage,
    tolerance: f64,
    what: &str,
) -> Result<(), ClipperError> {
    let mesh = extrude(path, HEIGHT)?;
    let filled_area = even_odd_area(path);
    let allowed = filled_area * AREA_FRACTION + tolerance;
    assert_clean_solid(&mesh, filled_area, HEIGHT, allowed, what);
    let on_top: Vec<(f64, f64)> = mesh
        .vertices
        .iter()
        .filter(|v| (f64::from(v.z) - HEIGHT).abs() < 1e-6)
        .map(|v| (f64::from(v.x), f64::from(v.y)))
        .collect();
    for (index, points) in loops_of(path).iter().enumerate() {
        let kept = points.iter().any(|&(x, y)| {
            on_top
                .iter()
                .any(|&(vx, vy)| (vx - x).abs() < 1e-4 && (vy - y).abs() < 1e-4)
        });
        assert!(kept, "{what}: loop {index} of the path is not on the solid");
    }
    Ok(())
}

/// [`assert_extrudes_within`] for a path whose points are all on clipper's 1 micron grid, so
/// nothing but the clean-up moves the area.
fn assert_extrudes_cleanly(path: &VertexStorage, what: &str) -> Result<(), ClipperError> {
    assert_extrudes_within(path, 1e-9, what)
}

#[test]
fn a_traced_picture_outline_extrudes_to_a_clean_solid() -> Result<(), ClipperError> {
    assert_extrudes_cleanly(&traced_silhouette(), "traced silhouette")
}

#[test]
fn deeply_nested_loops_extrude_to_a_clean_solid() -> Result<(), ClipperError> {
    // five squares inside each other, alternately wound: solid, hole, solid, hole, solid
    let squares: Vec<Vec<(f64, f64)>> = (0..5)
        .map(|i| {
            let inset = f64::from(i) * 2.0;
            square_at(inset, inset, 20.0 - 2.0 * inset, i % 2 == 1)
        })
        .collect();
    let loops: Vec<&[(f64, f64)]> = squares.iter().map(Vec::as_slice).collect();
    assert_extrudes_cleanly(&path_of(&loops), "nested squares")
}

#[test]
fn loops_wound_either_way_extrude_to_a_clean_solid() -> Result<(), ClipperError> {
    // the same nesting with every loop wound the same way: the even-odd fill still alternates
    let squares: Vec<Vec<(f64, f64)>> = (0..4)
        .map(|i| {
            let inset = f64::from(i) * 2.0;
            square_at(inset, inset, 20.0 - 2.0 * inset, false)
        })
        .collect();
    let loops: Vec<&[(f64, f64)]> = squares.iter().map(Vec::as_slice).collect();
    assert_extrudes_cleanly(&path_of(&loops), "same-wound nested squares")?;
    let reversed: Vec<Vec<(f64, f64)>> = (0..4)
        .map(|i| {
            let inset = f64::from(i) * 2.0;
            square_at(inset, inset, 20.0 - 2.0 * inset, true)
        })
        .collect();
    let loops: Vec<&[(f64, f64)]> = reversed.iter().map(Vec::as_slice).collect();
    assert_extrudes_cleanly(&path_of(&loops), "clockwise nested squares")
}

#[test]
fn loops_touching_at_a_point_extrude_to_a_clean_solid() -> Result<(), ClipperError> {
    let a = square_at(0.0, 0.0, 10.0, false);
    let b = square_at(10.0, 10.0, 10.0, false);
    assert_extrudes_cleanly(&path_of(&[&a, &b]), "squares touching at a corner")?;
    // a hole touching its outline at a point
    let outline = square_at(0.0, 0.0, 20.0, false);
    let hole = [(0.0, 10.0), (10.0, 5.0), (10.0, 15.0)];
    assert_extrudes_cleanly(&path_of(&[&outline, &hole]), "hole touching its outline")
}

#[test]
fn a_self_crossing_loop_extrudes_to_a_clean_solid() -> Result<(), ClipperError> {
    let bowtie = [(0.0, 0.0), (10.0, 10.0), (10.0, 0.0), (0.0, 10.0)];
    let path = path_of(&[&bowtie]);
    // two triangles of base 10 and height 5
    assert!((even_odd_area(&path) - 50.0).abs() < 1e-9);
    assert_extrudes_cleanly(&path, "bowtie")?;
    // A star drawn in one stroke, its points on the micron grid: under even-odd its centre is
    // empty. Each of the five crossings lands on the grid too, a fraction of a micron from where
    // the lines meet, which moves the area by up to about a micron times the edges beside it.
    let star: Vec<(f64, f64)> = (0..5)
        .map(|i| {
            let angle = std::f64::consts::TAU * f64::from(i * 2) / 5.0;
            let on_grid = |v: f64| (v * 1000.0).round_ties_even() / 1000.0;
            (on_grid(10.0 * angle.cos()), on_grid(10.0 * angle.sin()))
        })
        .collect();
    assert_extrudes_within(&path_of(&[&star]), 0.01, "pentagram")
}

#[test]
fn duplicate_consecutive_points_extrude_to_a_clean_solid() -> Result<(), ClipperError> {
    let points = [
        (0.0, 0.0),
        (10.0, 0.0),
        (10.0, 0.0),
        (10.0, 10.0),
        (10.0, 10.0),
        (10.0, 10.0),
        (0.0, 10.0),
        (0.0, 0.0),
    ];
    assert_extrudes_cleanly(&path_of(&[&points]), "repeated points")
}

#[test]
fn a_loop_too_small_to_hold_drops_out_cleanly() -> Result<(), ClipperError> {
    let outline = square_at(0.0, 0.0, 10.0, false);
    // smaller than a clipper unit: it rounds to nothing
    let speck = square_at(20.0, 20.0, 0.0002, false);
    let mesh = extrude(&path_of(&[&outline, &speck]), HEIGHT)?;
    assert_clean_solid(&mesh, 100.0, HEIGHT, 1e-9, "outline with a speck");
    // A sliver one unit tall inside a hole of the outline: its tip is within the clean-up's
    // 1.415 units of its base, so the sliver goes and the hole is left empty.
    let hole = square_at(2.0, 2.0, 6.0, true);
    let sliver = [(4.0, 4.0), (6.0, 4.0), (6.0, 4.001)];
    let mesh = extrude(&path_of(&[&outline, &hole, &sliver]), HEIGHT)?;
    assert_clean_solid(&mesh, 64.0, HEIGHT, 1e-9, "sliver in a hole");
    // the same sliver two units tall is kept
    let sliver = [(4.0, 4.0), (6.0, 4.0), (6.0, 4.002)];
    assert_extrudes_cleanly(
        &path_of(&[&outline, &hole, &sliver]),
        "thin triangle in a hole",
    )
}
