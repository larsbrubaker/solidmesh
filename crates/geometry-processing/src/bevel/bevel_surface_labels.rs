//! Which logical surface each face of the working mesh belongs to, labelled once per rebuild and
//! per distinct curve tolerance: what the pair-corner crease reading asks when two faces are one
//! surface and two planes. Ports `Bevel/BevelSurfaceLabels.cs`.
//!
//! **Why the GROUP's tolerance and not a blanket one.** A labelling is only ever read to say "these
//! two faces are the same surface", and saying yes REMOVES a candidate from the cap scan.
//! Over-loose is the dangerous direction, so the tolerance used is the one the user's own pick was
//! made at (`BevelEdgeGroup::curve_tolerance_degrees`). So the memo is keyed on the tolerance; each
//! distinct one pays for its own flood, as the edge group resolver does for its feature sets.
//!
//! One instance per rebuild, built beside the working mesh and handed down to every group. Nothing
//! is labelled until a group asks, so a rebuild whose corners are all read by normal alone pays
//! nothing.

use std::cell::OnceCell;

use polygon_mesh::{Mesh, MeshEdgeGraph};

use crate::topology::FaceGroup;

/// C# `BevelSurfaceLabels`.
pub struct BevelSurfaceLabels<'a> {
    part: &'a Mesh,
    graph: Option<&'a MeshEdgeGraph>,
    // The graph built on first use where the caller had none.
    built_graph: OnceCell<MeshEdgeGraph>,
    // Keyed by tolerance with C#'s `double` key equality (NaN equals NaN, -0 equals +0). Few
    // distinct tolerances ever occur, so a list is the map; it is only looked up, never iterated.
    labelled: Vec<(f64, Vec<i32>)>,
}

impl<'a> BevelSurfaceLabels<'a> {
    /// Labels the faces of one part: the working mesh every tool of this rebuild is measured on.
    /// `graph` is that mesh's own edge graph where the caller already has one - `None` builds one
    /// on first use, and only on first use.
    pub fn new(part: &'a Mesh, graph: Option<&'a MeshEdgeGraph>) -> Self {
        BevelSurfaceLabels {
            part,
            graph,
            built_graph: OnceCell::new(),
            labelled: Vec::new(),
        }
    }

    /// The surface id of every face at one curve tolerance, built the first time it is asked for.
    pub fn at(&mut self, curve_tolerance_degrees: f64) -> &[i32] {
        let found = self
            .labelled
            .iter()
            .position(|(key, _)| same_key(*key, curve_tolerance_degrees));

        let index = match found {
            Some(index) => index,
            None => {
                let part = self.part;
                let graph = match self.graph {
                    Some(graph) => graph,
                    None => self.built_graph.get_or_init(|| part.get_mesh_edge_graph()),
                };
                let surfaces = FaceGroup::label_surfaces(part, graph, curve_tolerance_degrees);
                self.labelled.push((curve_tolerance_degrees, surfaces));
                self.labelled.len() - 1
            }
        };

        &self.labelled[index].1
    }
}

/// C# `double.Equals`, which a `Dictionary<double, _>` keys by.
fn same_key(a: f64, b: f64) -> bool {
    a == b || (a.is_nan() && b.is_nan())
}
