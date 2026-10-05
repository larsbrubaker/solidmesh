//! Rust-only checks of the fourth bevel leaf cluster (`FieldBounds`, `SegmentBoxTree`,
//! `ScalarField`, `GapLoop`, `SectionPool`, `SurfaceBandEnd`, `SurfaceBandRunsOffOverAStretch`,
//! `SurfaceEdgeRoles`, `SurfaceLine`). No C# test class exercises them directly; their coverage
//! comes through the level-line and surface test classes of later slices.

use std::collections::HashSet;
use std::sync::Arc;

use geometry_processing::bevel::level_line::{
    field_bounds, segment_box_tree, ChainDistanceField, FieldMaximum, FieldMinimum, GapLoop,
    ScalarField,
};
use geometry_processing::bevel::sections::SectionPool;
use geometry_processing::bevel::surface::{
    SurfaceBandEnd, SurfaceBandRunsOffOverAStretch, SurfaceEdgeRoles, SurfaceLine,
};
use geometry_processing::topology::FeatureEdge;
use vector_math::{Vector3, Vector3Float};

#[test]
fn rust_only_lipschitz_bounds_bracket_a_point_distance_field() {
    // Distance to the origin is 1-Lipschitz; its bounds must bracket every sample.
    let a = Vector3::new(1.0, 0.0, 0.0);
    let b = Vector3::new(-1.0, 0.5, 0.0);
    let c = Vector3::new(0.0, 2.0, 1.0);
    let (va, vb, vc) = (a.length(), b.length(), c.length());
    let high = field_bounds::greatest_over_triangle(va, vb, vc, a, b, c);
    let low = field_bounds::least_over_triangle(va, vb, vc, a, b, c);
    let edge_high = field_bounds::greatest_along_edge(va, vb, (b - a).length());
    let edge_low = field_bounds::least_along_edge(va, vb, (b - a).length());
    for i in 0..=10 {
        for j in 0..=(10 - i) {
            let (u, v) = (f64::from(i) / 10.0, f64::from(j) / 10.0);
            let p = a + (b - a) * u + (c - a) * v;
            assert!(p.length() <= high + 1e-12 && p.length() >= low - 1e-12);
        }
        let q = a + (b - a) * (f64::from(i) / 10.0);
        assert!(q.length() <= edge_high + 1e-12 && q.length() >= edge_low - 1e-12);
    }
    assert_eq!(field_bounds::length_to_hide_a_pair(-1.0, 2.0, 0.25), 3.5);
}

#[test]
fn rust_only_the_segment_box_tree_splits_by_index_and_bounds_every_leaf() {
    let points = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    ];
    let setbacks = vec![0.5, 1.0, 2.0, 0.25];
    let reaches = vec![None, None, Some(vec![Vector3::new(1.0, 1.0, 5.0)]), None];
    let tree = segment_box_tree::build(&points, &setbacks, 4, Some(&reaches));

    // Root, left half (0..2) with its two leaves, right half (2..4) with its two leaves.
    assert_eq!(tree.len(), 7);
    assert_eq!(
        (tree[0].start, tree[0].count, tree[0].left, tree[0].right),
        (0, 4, 1, 4)
    );
    assert_eq!((tree[1].start, tree[1].count), (0, 2));
    assert_eq!((tree[2].start, tree[2].count, tree[2].left), (0, 1, -1));
    assert_eq!((tree[4].start, tree[4].count), (2, 2));
    // The closing segment 3 wraps to point 0.
    assert_eq!(tree[6].min, Vector3::new(0.0, 0.0, 0.0));
    assert_eq!(tree[6].max, Vector3::new(0.0, 1.0, 0.0));
    assert_eq!(tree[6].max_setback, 0.5);
    // The mitre reach widens segment 2's box and every box above it.
    assert_eq!(tree[5].max.z, 5.0);
    assert_eq!(tree[0].max.z, 5.0);
    assert_eq!(tree[0].max_setback, 2.0);

    let gap = segment_box_tree::box_gap(
        Vector3::ZERO,
        Vector3::ONE,
        Vector3::new(4.0, 5.0, 0.5),
        Vector3::new(6.0, 6.0, 0.7),
    );
    assert_eq!(gap, 5.0);
    assert_eq!(
        segment_box_tree::box_gap(
            Vector3::ZERO,
            Vector3::ONE,
            Vector3::ONE * 0.5,
            Vector3::ONE * 2.0
        ),
        0.0
    );
}

/// A plane field, the kind the cut is proven against before the chain field exists.
struct HeightField;

impl ScalarField for HeightField {
    fn value_at(&self, point: Vector3) -> f64 {
        point.z
    }
}

impl ChainDistanceField for HeightField {
    fn min_along_segment(&self, a: Vector3, b: Vector3) -> FieldMinimum {
        let p = if a.z <= b.z { a } else { b };
        FieldMinimum::new(p.z, p)
    }

    fn min_over_triangle(&self, a: Vector3, b: Vector3, c: Vector3) -> FieldMinimum {
        self.min_along_segment(self.min_along_segment(a, b).point, c)
    }

    fn max_along_segment(&self, a: Vector3, b: Vector3, _tolerance: f64) -> FieldMaximum {
        let p = if a.z >= b.z { a } else { b };
        FieldMaximum::new(p.z, p)
    }

    fn max_along_segment_at_least(
        &self,
        a: Vector3,
        b: Vector3,
        t: f64,
        at_least: f64,
    ) -> FieldMaximum {
        let found = self.max_along_segment(a, b, t);
        if found.value > at_least {
            found
        } else {
            FieldMaximum::new(at_least, Vector3::ZERO)
        }
    }

    fn max_over_triangle(&self, a: Vector3, b: Vector3, c: Vector3, t: f64) -> FieldMaximum {
        self.max_along_segment(self.max_along_segment(a, b, t).point, c, t)
    }

    fn max_over_triangle_bounded(
        &self,
        a: Vector3,
        b: Vector3,
        c: Vector3,
        t: f64,
        _stop_above: f64,
        at_least: f64,
    ) -> FieldMaximum {
        let found = self.max_over_triangle(a, b, c, t);
        if found.value > at_least {
            found
        } else {
            FieldMaximum::new(at_least, Vector3::ZERO)
        }
    }
}

#[test]
fn rust_only_a_field_answers_the_island_rule_through_its_seeded_search() {
    let field = HeightField;
    let (a, b, c) = (
        Vector3::ZERO,
        Vector3::new(1.0, 0.0, 2.0),
        Vector3::new(0.0, 1.0, 1.0),
    );
    assert!(
        !field.nothing_reaches_below(a, b, 0.0),
        "the default cannot say"
    );
    let peak = field.island_peak(a, b, c, 0.1, 1.5);
    assert_eq!(peak, FieldMaximum::new(2.0, b));
    let none = field.island_peak(a, b, c, 0.1, 3.0);
    assert_eq!(
        none.value, 2.9,
        "nothing beats the snap: the floor itself comes back"
    );
}

#[test]
fn rust_only_a_gap_loop_is_owned_by_its_chain_unless_told_otherwise() {
    let owned = GapLoop::new(1, true, vec![4, 5, 6], vec![0.0, 1.0, 2.0], 3, false, None);
    assert_eq!(owned.owners, vec![3]);
    assert_eq!((owned.group_tag, owned.vertex_owners.is_none()), (0, true));
    let mixed = GapLoop::new(2, false, vec![], vec![], 0, true, Some(vec![0, 2]));
    assert_eq!(mixed.owners, vec![0, 2]);
}

#[test]
fn rust_only_a_section_pool_appends_and_never_finds_near() {
    let mut pool = SectionPool::new(&[Vector3Float::new(1.0, 2.0, 3.0)]);
    assert_eq!((pool.seeded_count(), pool.count()), (1, 1));
    assert_eq!(pool.position_of(0), Vector3::new(1.0, 2.0, 3.0));
    assert_eq!(
        pool.add(Vector3::new(1.0, 2.0, 3.0)),
        Ok(1),
        "a coincident point is a new index"
    );
    assert!(pool.add(Vector3::new(0.0, f64::NAN, 0.0)).is_err());
    assert_eq!(pool.count(), 2);
}

#[test]
fn rust_only_surface_leaves_read_their_planes_and_lines() {
    let end = SurfaceBandEnd::new(Vector3::new(1.0, 0.0, 0.0), Vector3::UNIT_X);
    assert_eq!(end.signed_distance(Vector3::new(3.0, 5.0, 0.0)), 2.0);
    assert_eq!(end.signed_distance(Vector3::ZERO), -1.0);

    let x = SurfaceLine::new(Vector3::new(0.0, 0.0, 0.0), Vector3::UNIT_X);
    let y = SurfaceLine::new(Vector3::new(2.0, 3.0, 0.0), Vector3::UNIT_Y);
    assert_eq!(
        x.closest_point_to(Vector3::new(4.0, 1.0, 1.0)),
        Vector3::new(4.0, 0.0, 0.0)
    );
    assert_eq!(x.distance_to(Vector3::new(4.0, 3.0, 4.0)), 5.0);
    assert_eq!(x.try_cross(&y, 1e-9), Some(Vector3::new(2.0, 0.0, 0.0)));
    let skew = SurfaceLine::new(Vector3::new(2.0, 3.0, 0.5), Vector3::UNIT_Y);
    assert_eq!(
        x.try_cross(&skew, 0.1),
        None,
        "skew lines half a unit apart do not cross"
    );
    assert_eq!(
        x.try_cross_missed_by(&skew, 1.0),
        Some((Vector3::new(2.0, 0.0, 0.0), 0.5))
    );
    assert_eq!(x.try_cross(&x, 1.0), None, "parallel lines never cross");

    let stretch = SurfaceBandRunsOffOverAStretch::new(
        true,
        1.887,
        FeatureEdge::default(),
        Vector3::UNIT_Z,
        Vector3::ZERO,
        Arc::new(HashSet::from([7, 9])),
    );
    assert_eq!(
        (stretch.plan_index, stretch.edge_index, stretch.side),
        (-1, -1, -1)
    );
    let stamped = stretch.on(2, 3, 1);
    assert_eq!(
        (stamped.plan_index, stamped.edge_index, stamped.side),
        (2, 3, 1)
    );
    assert!(stamped.rims_it_runs_off.contains(&9) && stamped.until == 1.887);
}

#[test]
fn rust_only_edge_roles_record_cuts_beads_and_refuse_a_second_corner_on_one_end() {
    assert_eq!(
        SurfaceEdgeRoles::every_edge_has_a_strip().swallowed_count(),
        0
    );

    let mut roles = SurfaceEdgeRoles::default();
    roles.read(5, 2, true, 5, &[3, 4]);
    assert_eq!(roles.swallowed_count(), 2);
    assert!(roles.is_swallowed(3) && !roles.is_swallowed(2));
    assert_eq!(roles.try_cut_of(5), Some(2));
    assert_eq!(
        roles.try_corner_at(2, true),
        Some(5),
        "the cut meets the corner at its leaving end"
    );
    assert_eq!(
        roles.try_corner_at(5, false),
        Some(5),
        "the bead at its arriving end"
    );
    assert_eq!(roles.try_corner_at(2, false), None);

    assert!(roles.read_junction(-10, 0, true, false, &[]));
    assert_eq!(roles.try_cut_of(-10), None);
    assert_eq!(
        roles.try_the_edge_carrying_the_ring_at(-10),
        Some(0),
        "a concave junction's ring is on its bead"
    );
    assert!(
        !roles.read_junction(-11, 0, true, true, &[1]),
        "that end of edge 0 is already a corner's"
    );
    assert!(
        !roles.is_swallowed(1),
        "and a refused junction records nothing"
    );
}
