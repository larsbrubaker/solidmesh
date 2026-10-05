//! The open fill's row stations. Ports `Bevel/LevelLine/OpenGapFill.Stations.cs` (a partial of the
//! C# static class `OpenGapFill`): so far its row-spacing share, the mitred-turn test
//! ([`is_mitred_turn_at`]) and the thinning of rows to where the round needs them
//! ([`rows_where_the_round_needs_them`]); the stations land with later packages.

use super::{ChainField, LevelLineMesh, OpenGapFillOpenRail};

/// The least spacing between kept rows, as a share of the row's own width.
pub const MIN_ROW_SPACING_SHARE: f64 = 1.0 / 8.0;

/// True when the chain parameter `parameter` stands (within `tie`) at an interior chain vertex
/// that turns past the sharp-turn threshold and is not a smooth joint.
pub fn is_mitred_turn_at(chain: &ChainField, parameter: f64, tie: f64) -> bool {
    let points = chain.points();
    let mut at = 0.0;
    let mut k = 1;
    while k + 1 < points.len() {
        at += (points[k] - points[k - 1]).length();
        if (at - parameter).abs() <= tie {
            let into = (points[k] - points[k - 1]).get_normal();
            let out_of = (points[k + 1] - points[k]).get_normal();
            let turn = into.dot(out_of).clamp(-1.0, 1.0).acos() * 180.0 / std::f64::consts::PI;
            return turn > chain.sharp_turn_degrees && !chain.is_smooth_joint(k as i32);
        }

        k += 1;
    }

    false
}

/// The spokes that get a row: the first and last always, and every spoke between unless it stands
/// inside the chain with both ends nearer the last kept row than [`MIN_ROW_SPACING_SHARE`] of its
/// own width. Whatever kind of cell is left from the last row kept to the next spoke - both rails
/// advancing, or one - the fill takes the rail vertices between them into its base triangles.
pub fn rows_where_the_round_needs_them(
    mesh: &LevelLineMesh,
    chain: &ChainField,
    left: &OpenGapFillOpenRail,
    right: &OpenGapFillOpenRail,
    spokes: &[(i32, i32)],
) -> Vec<(i32, i32)> {
    let mut kept = vec![spokes[0]];
    let mut k = 1;
    while k + 1 < spokes.len() {
        let previous = kept[kept.len() - 1];
        let spoke = spokes[k];
        k += 1;
        let l = mesh.position(left.vertices[spoke.0 as usize]);
        let r = mesh.position(right.vertices[spoke.1 as usize]);
        let spacing = MIN_ROW_SPACING_SHARE * (r - l).length();
        let inside = left.parameters[spoke.0 as usize] > 0.0
            && left.parameters[spoke.0 as usize] < chain.length
            && right.parameters[spoke.1 as usize] > 0.0
            && right.parameters[spoke.1 as usize] < chain.length;
        if inside
            && (l - mesh.position(left.vertices[previous.0 as usize])).length() < spacing
            && (r - mesh.position(right.vertices[previous.1 as usize])).length() < spacing
        {
            continue;
        }

        kept.push(spoke);
    }

    kept.push(spokes[spokes.len() - 1]);
    kept
}
