//! Clipper 6's `PolyTree`/`PolyNode` (`clipper.cs`): the solution as a tree of outlines and
//! holes. Nodes live in one arena; node 0 is the tree itself (C#'s `PolyTree : PolyNode`), and
//! every other node is one of C#'s `m_AllPolys`, in the order the C# adds them.

use super::types::Path;

/// C# `PolyNode`.
#[derive(Clone, Debug, Default)]
pub struct PolyNode {
    pub parent: Option<usize>,
    pub polygon: Path,
    /// This node's position among its parent's children.
    pub index: usize,
    pub childs: Vec<usize>,
    pub is_open: bool,
}

/// C# `PolyTree`.
#[derive(Clone, Debug)]
pub struct PolyTree {
    pub nodes: Vec<PolyNode>,
}

impl Default for PolyTree {
    fn default() -> Self {
        Self::new()
    }
}

impl PolyTree {
    /// The root node's index.
    pub const ROOT: usize = 0;

    pub fn new() -> Self {
        PolyTree {
            nodes: vec![PolyNode::default()],
        }
    }

    /// C# `Clear`.
    pub fn clear(&mut self) {
        self.nodes.truncate(1);
        self.nodes[0].childs.clear();
    }

    /// Adds a detached node (C# `m_AllPolys.Add(new PolyNode())`) and returns its index.
    pub(super) fn new_node(&mut self, polygon: Path) -> usize {
        self.nodes.push(PolyNode {
            polygon,
            ..PolyNode::default()
        });
        self.nodes.len() - 1
    }

    /// C# `PolyNode.AddChild`.
    pub(super) fn add_child(&mut self, parent: usize, child: usize) {
        let cnt = self.nodes[parent].childs.len();
        self.nodes[parent].childs.push(child);
        self.nodes[child].parent = Some(parent);
        self.nodes[child].index = cnt;
    }

    /// C# `PolyNode.Contour`.
    pub fn contour(&self, node: usize) -> &Path {
        &self.nodes[node].polygon
    }

    /// C# `PolyNode.Childs`.
    pub fn childs(&self, node: usize) -> &[usize] {
        &self.nodes[node].childs
    }

    /// C# `ChildCount`.
    pub fn child_count(&self, node: usize) -> usize {
        self.nodes[node].childs.len()
    }

    /// C# `PolyNode.IsHole`: true for the root, then alternating with depth.
    pub fn is_hole(&self, node: usize) -> bool {
        let mut result = true;
        let mut parent = self.nodes[node].parent;
        while let Some(p) = parent {
            result = !result;
            parent = self.nodes[p].parent;
        }
        result
    }

    /// C# `PolyTree.GetFirst`.
    pub fn get_first(&self) -> Option<usize> {
        self.nodes[Self::ROOT].childs.first().copied()
    }

    /// C# `PolyNode.GetNext`: depth first, parents before children.
    pub fn get_next(&self, node: usize) -> Option<usize> {
        match self.nodes[node].childs.first() {
            Some(&child) => Some(child),
            None => self.get_next_sibling_up(node),
        }
    }

    fn get_next_sibling_up(&self, node: usize) -> Option<usize> {
        let parent = self.nodes[node].parent?;
        let index = self.nodes[node].index;
        if index == self.nodes[parent].childs.len() - 1 {
            self.get_next_sibling_up(parent)
        } else {
            Some(self.nodes[parent].childs[index + 1])
        }
    }

    /// C# `PolyTree.Total`: every node but the root, less the hidden outer polygon a negative
    /// offset leaves when the first child is not the first node built.
    pub fn total(&self) -> usize {
        let mut result = self.nodes.len() - 1;
        if result > 0 && self.nodes[Self::ROOT].childs.first() != Some(&1) {
            result -= 1;
        }
        result
    }
}
