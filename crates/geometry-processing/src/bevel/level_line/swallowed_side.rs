//! A side the band swallows whole: its mesh readings - whether a triangle touches a point
//! ([`touches`]), a side's vertices ([`side_vertices`]), the direction away from a segment on a
//! side ([`away_on_side`]) and the planes' tolerance ([`tolerance`]) - and the swallowed strip's
//! fill ([`fill`]); the swallow tests and the made-up runs are in `swallowed_side_runs.rs`.

use indexmap::IndexSet;

use vector_math::Vector3;

use super::open_gap_fill_fill::{self, OpenFillEnds};
use super::{
    BandGapResult, BandProfile, GapFillRefusal, GapFillResult, GapLoop, LevelLineMesh,
    LevelLineMeshError, LevelLineTriangle,
};

/// True when a corner of `tri` stands within the mesh's same-place backstop of `point`.
pub fn touches(mesh: &LevelLineMesh, tri: LevelLineTriangle, point: Vector3) -> bool {
    (mesh.position(tri.a) - point).length() <= mesh.same_place()
        || (mesh.position(tri.b) - point).length() <= mesh.same_place()
        || (mesh.position(tri.c) - point).length() <= mesh.same_place()
}

/// The vertices of a side's live triangles, in the order first met (the crease search takes the
/// first that qualifies, so the order decides the result).
pub fn side_vertices(mesh: &LevelLineMesh, side: i32) -> IndexSet<i32> {
    let mut vertices = IndexSet::new();
    for t in 0..mesh.triangle_count() {
        let tri = mesh.triangles()[t as usize];
        if !mesh.is_removed(t) && tri.tag == side {
            vertices.insert(tri.a);
            vertices.insert(tri.b);
            vertices.insert(tri.c);
        }
    }

    vertices
}

/// The unit direction, square to segment `a b`, from the segment into the first live triangle of
/// `side` with an edge along it (within `tolerance`) and its third corner off it; `None` where no
/// such triangle stands.
pub fn away_on_side(
    mesh: &LevelLineMesh,
    a: Vector3,
    b: Vector3,
    side: i32,
    tolerance: f64,
) -> Option<Vector3> {
    let direction = (b - a).get_normal();
    let on_segment = |q: Vector3| {
        let along = (q - a).dot(direction);
        along >= -tolerance
            && along <= (b - a).length() + tolerance
            && ((q - a) - (direction * along)).length() <= tolerance
    };

    for t in 0..mesh.triangle_count() {
        let tri = mesh.triangles()[t as usize];
        if mesh.is_removed(t) || tri.tag != side {
            continue;
        }

        for corner in 0..3 {
            let p = mesh.position(tri.corner(corner));
            let q = mesh.position(tri.corner((corner + 1) % 3));
            let third = mesh.position(tri.corner((corner + 2) % 3));
            if (p - q).length() > tolerance && on_segment(p) && on_segment(q) && !on_segment(third)
            {
                let off = third - a;
                return Some((off - (direction * off.dot(direction))).get_normal());
            }
        }
    }

    None
}

/// The planes' tolerance per unit of length: the fixture's
/// walls are float (a few 1e-7 off).
pub const ON_PLANE_PER_LENGTH: f64 = 1e-5;

/// The planes' tolerance for this chain, scaled by its largest
/// setback (at least 1).
pub fn tolerance(chain: &super::ChainField) -> f64 {
    ON_PLANE_PER_LENGTH * vector_math::ieee_math::max_f64(1.0, chain.max_setback)
}

/// The swallowed strip - each side's made-up run minted as the rail (side
/// 1 walked ascending, side 2 descending, as the cut walks a side's run with the kept surface on
/// its left), the rows laid whole between them, and the valley clip (`strip_clip`) dropping what
/// is behind either wall and growing both walls to the round. Asked only of a gap with swallowed
/// runs (the gap fill checks first); a gap without them, or without a mesh, has no
/// strip to lay and is refused as open chain ends rather than dereferenced.
pub fn fill(
    gap: &BandGapResult,
    profile: BandProfile,
    fillet_segments: i32,
) -> Result<GapFillResult, LevelLineMeshError> {
    let (Some(mesh_handle), Some(runs)) = (gap.mesh.as_ref(), gap.swallowed_runs.as_ref()) else {
        return Ok(GapFillResult {
            refusal: GapFillRefusal::OpenChainEnds,
            mesh: gap.mesh.clone(),
            ..GapFillResult::default()
        });
    };

    let mut minted = Vec::new();
    let (left, right) = {
        let mut mesh = mesh_handle.borrow_mut();
        let mut run_of = |side: i32| {
            let run = &runs[(side - 1) as usize];
            let mut vertices: Vec<i32> = run.iter().map(|r| mesh.add_vertex(r.0)).collect();
            let mut parameters: Vec<f64> = run.iter().map(|r| r.1).collect();
            minted.extend(vertices.iter().copied());
            if side == 2 {
                vertices.reverse();
                parameters.reverse();
            }

            GapLoop::new(side, false, vertices, parameters, 0, false, None)
        };
        let left = run_of(1);
        let right = run_of(2);
        (left, right)
    };

    let filled = open_gap_fill_fill::fill(
        mesh_handle,
        // The chain's own field: the rows' chain points and tolerances, the field its runs were
        // placed at (`swallowed_side_runs::runs_if_both_swallowed`).
        &gap.chains[0],
        &left,
        &right,
        profile,
        fillet_segments,
        &mut Vec::new(),
        OpenFillEnds::default(),
    )?;
    if !filled.filled() {
        return Ok(filled);
    }

    let vertices: Vec<i32> = minted
        .into_iter()
        .chain(filled.added_vertices.iter().copied())
        .collect();
    let (triangles, vertex_count) = (filled.added_triangles.len() as i32, vertices.len() as i32);
    Ok(GapFillResult {
        refusal: GapFillRefusal::None,
        mesh: filled.mesh,
        added_triangles: filled.added_triangles,
        added_vertices: vertices,
        triangles_of_chain: vec![triangles],
        vertices_of_chain: vec![vertex_count],
        ..GapFillResult::default()
    })
}
