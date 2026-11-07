//! Rust-only checks of the Clipper 6 port (`clipper6`): booleans, offsets, strictly simple
//! splitting and the PolyTree on small shapes whose Clipper 6 answers are known by hand.

use clipper_library::clipper6::*;

fn square(x: i64, y: i64, size: i64) -> Path {
    vec![
        IntPoint::new(x, y),
        IntPoint::new(x + size, y),
        IntPoint::new(x + size, y + size),
        IntPoint::new(x, y + size),
    ]
}

fn run(clip_type: ClipType, subject: &Paths, clip: &Paths, fill: PolyFillType) -> Paths {
    let mut c = Clipper::new();
    c.add_paths(subject, PolyType::Subject, true).unwrap();
    c.add_paths(clip, PolyType::Clip, true).unwrap();
    let mut solution = Paths::new();
    c.execute(clip_type, &mut solution, fill, fill).unwrap();
    solution
}

#[test]
fn rust_only_union_of_overlapping_squares_is_one_ring() {
    let result = run(
        ClipType::Union,
        &vec![square(0, 0, 10)],
        &vec![square(5, 5, 10)],
        PolyFillType::NonZero,
    );
    assert_eq!(result.len(), 1);
    assert_eq!(result[0].len(), 8);
    assert_eq!(area(&result[0]), 175.0);
}

#[test]
fn rust_only_intersection_and_difference_areas() {
    let a = vec![square(0, 0, 10)];
    let b = vec![square(5, 5, 10)];
    let inter = run(ClipType::Intersection, &a, &b, PolyFillType::EvenOdd);
    assert_eq!(inter.len(), 1);
    assert_eq!(area(&inter[0]), 25.0);
    let diff = run(ClipType::Difference, &a, &b, PolyFillType::EvenOdd);
    assert_eq!(diff.len(), 1);
    assert_eq!(area(&diff[0]), 75.0);
    let xor = run(ClipType::Xor, &a, &b, PolyFillType::EvenOdd);
    assert_eq!(xor.iter().map(|p| area(p)).sum::<f64>(), 150.0);
}

#[test]
fn rust_only_collinear_points_are_dropped() {
    let mut subject = square(0, 0, 10);
    subject.insert(1, IntPoint::new(5, 0));
    let result = run(
        ClipType::Union,
        &vec![subject],
        &Paths::new(),
        PolyFillType::EvenOdd,
    );
    assert_eq!(result.len(), 1);
    assert_eq!(result[0].len(), 4);
}

#[test]
fn rust_only_frame_tree_has_outline_and_hole() {
    let mut hole = square(2, 2, 6);
    hole.reverse();
    let mut c = Clipper::new();
    c.add_paths(&[square(0, 0, 10), hole], PolyType::Subject, true)
        .unwrap();
    let mut tree = PolyTree::new();
    c.execute_tree(
        ClipType::Union,
        &mut tree,
        PolyFillType::EvenOdd,
        PolyFillType::EvenOdd,
    )
    .unwrap();
    let outer = tree.childs(PolyTree::ROOT);
    assert_eq!(outer.len(), 1);
    assert!(!tree.is_hole(outer[0]));
    let holes = tree.childs(outer[0]);
    assert_eq!(holes.len(), 1);
    assert!(tree.is_hole(holes[0]));
    assert_eq!(area(tree.contour(holes[0])), -36.0);
    assert_eq!(closed_paths_from_poly_tree(&tree).len(), 2);
}

#[test]
fn rust_only_simplify_polygon_splits_a_bow_tie() {
    let bow_tie = vec![
        IntPoint::new(0, 0),
        IntPoint::new(10, 10),
        IntPoint::new(10, 0),
        IntPoint::new(0, 10),
    ];
    let result = simplify_polygon(&bow_tie, PolyFillType::EvenOdd).unwrap();
    assert_eq!(result.len(), 2);
}

#[test]
fn rust_only_offset_grows_and_shrinks_a_square() {
    let mut offset = ClipperOffset::new(2.0, 0.25);
    offset.add_path(&square(0, 0, 100), JoinType::Miter, EndType::ClosedPolygon);
    let mut grown = Paths::new();
    offset.execute(&mut grown, 10.0).unwrap();
    assert_eq!(grown.len(), 1);
    assert_eq!(area(&grown[0]), 120.0 * 120.0);
    let mut shrunk = Paths::new();
    offset.execute(&mut shrunk, -10.0).unwrap();
    assert_eq!(shrunk.len(), 1);
    assert_eq!(area(&shrunk[0]), 80.0 * 80.0);
}

#[test]
fn rust_only_open_paths_need_a_poly_tree() {
    let mut c = Clipper::new();
    let line = vec![IntPoint::new(0, 0), IntPoint::new(10, 10)];
    assert_eq!(c.add_path(&line, PolyType::Subject, false), Ok(true));
    let mut solution = Paths::new();
    assert_eq!(
        c.execute(
            ClipType::Union,
            &mut solution,
            PolyFillType::EvenOdd,
            PolyFillType::EvenOdd
        ),
        Err(ClipperError::PolyTreeNeededForOpenPaths)
    );
    assert_eq!(
        c.add_path(&line, PolyType::Clip, false),
        Err(ClipperError::OpenPathMustBeSubject)
    );
}

#[test]
fn rust_only_open_line_is_clipped_by_a_square() {
    let mut c = Clipper::new();
    c.add_path(
        &[IntPoint::new(-5, 5), IntPoint::new(15, 5)],
        PolyType::Subject,
        false,
    )
    .unwrap();
    c.add_path(&square(0, 0, 10), PolyType::Clip, true).unwrap();
    let mut tree = PolyTree::new();
    c.execute_tree(
        ClipType::Intersection,
        &mut tree,
        PolyFillType::EvenOdd,
        PolyFillType::EvenOdd,
    )
    .unwrap();
    let open = open_paths_from_poly_tree(&tree);
    assert_eq!(open.len(), 1);
    let mut xs: Vec<i64> = open[0].iter().map(|p| p.x).collect();
    xs.sort();
    assert_eq!(xs, vec![0, 10]);
}
