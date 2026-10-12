//! The helpers `several_chains_tests.rs` reads (C# `SeveralChainsTests`' private `Read`,
//! `PickNamed`, `PointSet`, `TagUnder` and `ToChain`, plus the `BandGap.Build`/`RoomOf`/`ReadRoom`
//! calls its tests repeat), kept apart so the class's own file holds its tests.

use std::rc::Rc;

use geometry_processing::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use geometry_processing::bevel::level_line::band_gap_reading;
use geometry_processing::bevel::level_line::seam_edges::FeatureEdgeTest;
use geometry_processing::bevel::level_line::{
    BandGapResult, BandProfile, GapLoop, LevelLineMesh, RoomReading, WorkBudget,
};
use vector_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_room_geometry::SolidWithChain;
use crate::level_line_test_geometry::{cube_solid, cylinder_solid};
use crate::several_chains_fixtures::FOLD_DEGREES;

pub fn feature(positions: &[Vector3], indices: &[(i32, i32, i32)]) -> Option<FeatureEdgeTest> {
    Some(feature_edges_over(positions, indices, FOLD_DEGREES))
}

pub fn never() -> Option<FeatureEdgeTest> {
    Some(Rc::new(|_, _| false))
}

/// C# `BandGap.Build(positions, indices, chains, closed, feature, size, profile)`.
pub fn build(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    feature: Option<FeatureEdgeTest>,
    size: f64,
) -> BandGapResult {
    build_with(
        positions,
        indices,
        chains,
        closed,
        feature,
        size,
        &BandGapBuildOptions::default(),
    )
}

pub fn build_with(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    feature: Option<FeatureEdgeTest>,
    size: f64,
    options: &BandGapBuildOptions,
) -> BandGapResult {
    band_gap_entry_points::build_pick(
        positions,
        indices,
        chains,
        closed,
        feature,
        size,
        BandProfile::Chamfer,
        options,
    )
    .expect("the pick builds or refuses by name")
}

/// C# `BandGap.RoomOf(positions, indices, chains, closed, feature, profile)`.
pub fn room_of(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
) -> f64 {
    band_gap_reading::room_of_pick(
        positions,
        indices,
        chains,
        closed,
        feature(positions, indices),
        BandProfile::Chamfer,
        1e-9,
    )
    .expect("the pick reads")
}

/// C# `BandGap.ReadRoom(positions, indices, chains, closed, feature, Chamfer, 1e-9, null)`.
pub fn read_room(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
    closed: &[bool],
    feature: Option<FeatureEdgeTest>,
    budget: Option<Rc<WorkBudget>>,
) -> RoomReading {
    band_gap_reading::read_room_of_pick(
        positions,
        indices,
        chains,
        closed,
        feature,
        BandProfile::Chamfer,
        1e-9,
        budget,
        f64::NAN,
        0.0,
        f64::INFINITY,
    )
    .expect("the pick reads")
}

/// C# `Read`: the reading of a two-chain closed pick.
pub fn read(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chains: &[Vec<i32>],
) -> RoomReading {
    read_room(
        positions,
        indices,
        chains,
        &[true, true],
        feature(positions, indices),
        None,
    )
}

/// C# `BandGap.RoomMargin(1)`.
pub fn room_margin() -> f64 {
    band_gap_reading::room_margin(1.0, 1e-9)
}

/// C# `PickNamed`.
pub fn pick_named(pick: &str) -> SolidWithChain {
    match pick {
        "cube top rim" => {
            let (positions, indices) = cube_solid(1.0);
            (positions, indices, vec![4, 5, 6, 7])
        }
        "cylinder rim" => {
            const SIDES: i32 = 16;
            let (positions, indices) = cylinder_solid(SIDES, 1.0, 2.0);
            (positions, indices, (SIDES..2 * SIDES).collect())
        }
        _ => panic!("no such pick: {pick}"),
    }
}

/// C# `PointSet`.
pub fn point_set(built: &BandGapResult) -> Vec<String> {
    let mesh = built.mesh.as_ref().expect("a mesh").borrow();
    let mut points: Vec<String> = built
        .loops
        .iter()
        .flat_map(|l| l.vertices.iter().map(|&v| mesh.position(v)))
        .map(|p| format!("{:.9},{:.9},{:.9}", p.x, p.y, p.z))
        .collect();
    points.sort();
    points
}

/// The groups (C# `GroupBy(TagUnder).Where(g => g.Count() > 1)`) of loops standing on one tag, in
/// first-seen order.
pub fn shared_groups<'a>(mesh: &LevelLineMesh, loops: &'a [GapLoop]) -> Vec<Vec<&'a GapLoop>> {
    let mut groups: Vec<(i32, Vec<&GapLoop>)> = Vec::new();
    for gap_loop in loops {
        let tag = tag_under(mesh, gap_loop);
        match groups.iter_mut().find(|(t, _)| *t == tag) {
            Some((_, members)) => members.push(gap_loop),
            None => groups.push((tag, vec![gap_loop])),
        }
    }
    groups
        .into_iter()
        .filter(|(_, members)| members.len() > 1)
        .map(|(_, members)| members)
        .collect()
}

/// C# `TagUnder`: the tag of the triangle the loop's first step keeps.
pub fn tag_under(mesh: &LevelLineMesh, gap_loop: &GapLoop) -> i32 {
    let standing = mesh.triangles_on(
        gap_loop.vertices[0].min(gap_loop.vertices[1]),
        gap_loop.vertices[0].max(gap_loop.vertices[1]),
    );
    mesh.triangles[standing[0] as usize].tag
}

/// C# `ToChain`: the distance from a point to a closed chain read on the INPUT positions.
pub fn to_chain(point: Vector3, positions: &[Vector3], chain: &[i32]) -> f64 {
    let mut best = f64::INFINITY;
    for i in 0..chain.len() {
        let a = positions[chain[i] as usize];
        let b = positions[chain[(i + 1) % chain.len()] as usize];
        let ab = b - a;
        let d = point - a;
        let along = (d.x * ab.x) + (d.y * ab.y) + (d.z * ab.z);
        let t = if ab.length_squared() > 0.0 {
            (along / ab.length_squared()).clamp(0.0, 1.0)
        } else {
            0.0
        };
        best = best.min((point - (a + (ab * t))).length());
    }

    best
}
