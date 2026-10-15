//! A depth-first walk of a trace tree with each item's transform to world and depth, optionally
//! filtered. MatterCAD walks scene trace data with it for rectangle selection and its debug
//! bounds drawing; the filter decides per item whether the item and everything under it is
//! visited.

use vector_math::Matrix4X4;

use super::Traceable;

/// Decides whether an item (and so its subtree) is visited.
pub type BvhDescentFilter<'f> = &'f dyn Fn(&BvhIterator<'_, '_>) -> bool;

/// One item of the walk, and the walk from it.
#[derive(Clone, Copy)]
pub struct BvhIterator<'a, 'f> {
    pub transform_to_world: Matrix4X4,
    pub bvh: &'a dyn Traceable,
    pub depth: i32,
    descent_filter: Option<BvhDescentFilter<'f>>,
}

impl<'a, 'f> BvhIterator<'a, 'f> {
    /// The walk from `reference_item` at depth 0 under the identity.
    pub fn new(reference_item: &'a dyn Traceable) -> Self {
        Self::with(reference_item, Matrix4X4::IDENTITY, 0, None)
    }

    /// The walk from `reference_item` under `initial_transform` at `initial_depth`. An all-zero
    /// `initial_transform` (`Matrix4X4::default()`, "no transform given") means the identity.
    pub fn with(
        reference_item: &'a dyn Traceable,
        initial_transform: Matrix4X4,
        initial_depth: i32,
        descent_filter: Option<BvhDescentFilter<'f>>,
    ) -> Self {
        let transform_to_world = if initial_transform == Matrix4X4::default() {
            Matrix4X4::IDENTITY
        } else {
            initial_transform
        };
        BvhIterator {
            transform_to_world,
            bvh: reference_item,
            depth: initial_depth,
            descent_filter,
        }
    }

    /// The walk from `item` keeping only what `descent_filter` accepts.
    pub fn filter(item: &'a dyn Traceable, descent_filter: BvhDescentFilter<'f>) -> Self {
        Self::with(item, Matrix4X4::IDENTITY, 0, Some(descent_filter))
    }

    /// This item and every item under it that the filter lets through, depth first, parents
    /// before children, children in order.
    pub fn iter(&self) -> BvhWalk<'a, 'f> {
        BvhWalk { stack: vec![*self] }
    }
}

impl<'a, 'f> IntoIterator for BvhIterator<'a, 'f> {
    type Item = BvhIterator<'a, 'f>;
    type IntoIter = BvhWalk<'a, 'f>;

    fn into_iter(self) -> BvhWalk<'a, 'f> {
        self.iter()
    }
}

/// The pre-order walk behind [`BvhIterator::iter`].
pub struct BvhWalk<'a, 'f> {
    stack: Vec<BvhIterator<'a, 'f>>,
}

impl<'a, 'f> Iterator for BvhWalk<'a, 'f> {
    type Item = BvhIterator<'a, 'f>;

    fn next(&mut self) -> Option<Self::Item> {
        while let Some(current) = self.stack.pop() {
            // No filter visits everything.
            if current
                .descent_filter
                .is_some_and(|filter| !filter(&current))
            {
                continue;
            }

            if let Some(children) = current.bvh.children() {
                let child_transform = current.bvh.axis_to_world() * current.transform_to_world;
                for child in children.into_iter().rev() {
                    self.stack.push(BvhIterator::with(
                        child,
                        child_transform,
                        current.depth + 1,
                        current.descent_filter,
                    ));
                }
            }

            return Some(current);
        }

        None
    }
}
