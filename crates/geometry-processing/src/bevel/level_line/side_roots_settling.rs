//! Puts each side's level line onto the mesh: every edge of the side that crosses the line gets
//! its root as a vertex, so no triangle of the side straddles it when the band is deleted. The
//! roots come from `edge_root`, the split from `band_cut`.

use std::collections::HashMap;

use super::band_cut::put_the_root_on_the_line;
use super::band_triangles::edges_of_side;
use super::edge_root::find_over;
use super::{LevelLineMeshError, VertexFieldReading};

/// Roots every crossed edge of side `tag` on the reading's
/// mesh; true when any root was put.
pub fn root_the_side(reading: &VertexFieldReading, tag: i32) -> Result<bool, LevelLineMeshError> {
    let roots = {
        let mesh = reading.mesh.borrow();
        find_over(&mesh, reading, edges_of_side(&mesh, tag))?
    };
    let mut rooted = false;
    for root in roots {
        put_the_root_on_the_line(reading, root.a, root.b, root.point)?;
        rooted = true;
    }

    Ok(rooted)
}

/// Takes the cut on the FINAL mesh of a pick of several
/// sides - each side is rooted again, until a whole pass roots nothing. A root put on one side's
/// line can split an edge it shares with another side in turn, which is why the pass repeats.
/// Every reading reads the one mesh being cut.
pub fn settle_every_side(
    tags: &[i32],
    readings: &HashMap<i32, VertexFieldReading>,
) -> Result<(), LevelLineMeshError> {
    let mut rooted = tags.len() > 1;
    while rooted {
        rooted = false;
        for tag in tags {
            rooted |= root_the_side(&readings[tag], *tag)?;
        }
    }

    Ok(())
}
