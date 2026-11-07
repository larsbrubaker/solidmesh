//! The index-ordered binary box tree a chain field query walks, and the box-to-box gap every one of
//! its bounds is read from: built once from the chain's points and setbacks, and thereafter only
//! visited, so the whole of "which segments could still beat what I have" lives in one place.

use vector_math::{ieee_math, Vector3};

/// One segment per leaf: the convex search over a segment costs hundreds of closest-point
/// evaluations, so bounding EVERY segment before searching it is worth the extra nodes many times
/// over.
const LEAF_SEGMENTS: i32 = 1;

/// One node of the segment box tree: a run of chain segments IN INDEX ORDER, their box, and the
/// greatest setback any of them asks for (what a lower bound on the field has to give away).
///
/// The runs are split by index rather than by position, so a walk that takes the left child first
/// visits the segments in exactly the order the plain loop did. That is what keeps the
/// lower-parameter tie rule - and therefore every answer - identical to the exhaustive search.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SegmentNode {
    pub start: i32,
    pub count: i32,
    /// The left child's node index, or -1 when this node is a leaf.
    pub left: i32,
    pub right: i32,
    pub min: Vector3,
    pub max: Vector3,
    pub max_setback: f64,
}

/// The whole tree as an array, root first - the shape the chain field holds. `also_reaches` is
/// `None`, or per segment the far ends of the mitre extensions that segment also answers for
/// (`None` for a segment with none) - folded into the segment's box so every prune stays sound.
pub fn build(
    points: &[Vector3],
    setbacks: &[f64],
    segment_count: i32,
    also_reaches: Option<&[Option<Vec<Vector3>>]>,
) -> Vec<SegmentNode> {
    let mut built = Vec::new();
    add_segment_node(&mut built, points, setbacks, 0, segment_count, also_reaches);
    built
}

/// The least distance between two axis-aligned boxes; zero when they touch or overlap.
pub fn box_gap(a_min: Vector3, a_max: Vector3, b_min: Vector3, b_max: Vector3) -> f64 {
    let gap = |a_low: f64, a_high: f64, b_low: f64, b_high: f64| {
        ieee_math::max_f64(0.0, ieee_math::max_f64(a_low - b_high, b_low - a_high))
    };
    let dx = gap(a_min.x, a_max.x, b_min.x, b_max.x);
    let dy = gap(a_min.y, a_max.y, b_min.y, b_max.y);
    let dz = gap(a_min.z, a_max.z, b_min.z, b_max.z);
    ((dx * dx) + (dy * dy) + (dz * dz)).sqrt()
}

fn add_segment_node(
    built: &mut Vec<SegmentNode>,
    points: &[Vector3],
    setbacks: &[f64],
    start: i32,
    count: i32,
    also_reaches: Option<&[Option<Vec<Vector3>>]>,
) -> i32 {
    let mut min = Vector3::POSITIVE_INFINITY;
    let mut max = Vector3::NEGATIVE_INFINITY;
    let mut max_setback = f64::NEG_INFINITY;
    for i in start..start + count {
        let i = i as usize;
        let next = (i + 1) % points.len();
        let from = points[i];
        let to = points[next];
        min = Vector3::component_min(min, Vector3::component_min(from, to));
        max = Vector3::component_max(max, Vector3::component_max(from, to));
        max_setback =
            ieee_math::max_f64(max_setback, ieee_math::max_f64(setbacks[i], setbacks[next]));
        if let Some(Some(reaches)) = also_reaches.map(|all| &all[i]) {
            for &reach in reaches {
                min = Vector3::component_min(min, reach);
                max = Vector3::component_max(max, reach);
            }
        }
    }

    let index = built.len() as i32;
    built.push(SegmentNode::default());

    let mut left = -1;
    let mut right = -1;
    if count > LEAF_SEGMENTS {
        let half = count / 2;
        left = add_segment_node(built, points, setbacks, start, half, also_reaches);
        right = add_segment_node(
            built,
            points,
            setbacks,
            start + half,
            count - half,
            also_reaches,
        );
    }

    built[index as usize] = SegmentNode {
        start,
        count,
        left,
        right,
        min,
        max,
        max_setback,
    };
    index
}
