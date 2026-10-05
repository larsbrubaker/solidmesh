//! Laying an OFFSET side's reach along the surface rather than straight off the chain
//! (`Bevel/Sections/SectionBuilderWalkedSide.cs`, part of the C# `internal static partial class
//! SectionBuilder`): where one such reach ended ([`WalkedEnd`]), the neighbour's end a split
//! station is joined from ([`the_neighbours_end`], [`the_faces_the_end_lies_in`]) and the place a
//! join-placed point lands on ([`try_along_the_join`], [`name_the_place`] and their helpers). And the
//! split station's own point placed on that join ([`the_joins_own_point`]), and a section with its
//! walked ends put in ([`rewritten`]).

use indexmap::IndexMap;
use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::{cs_math, Vector3};

use super::section_fan_walk::EdgeAtPair;
use super::section_split_blend::SplitPlacement;
use super::{
    section_room, section_smooth_side_boundary, section_surface_walk, section_surface_walk_join,
    BevelSection, JunctionSection, NanSectionPoint, SectionKind, SectionPool, SectionSide,
    SurfaceWalkCrossing,
};

/// C# `SectionBuilder.WalkedEnd`: where one OFFSET side's reach ended when it was laid along the
/// surface - what `BevelSection`'s `left_walk_*` / `right_walk_*` fields are filled from.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct WalkedEnd {
    /// The pool index of the boundary point, or -1 where this side was not walked at all.
    pub boundary: i32,
    /// The face that point lies in, or -1 where this side was not walked at all.
    pub face: i32,
    /// The direction the walk was travelling when it ended.
    pub direction: Vector3,
    /// Whether the point was placed along the JOIN between two neighbours' boundary points rather
    /// than walked from its own station.
    pub is_join_placed: bool,
    /// Whether the reach crossed a FOLD getting here.
    pub crossed_a_fold: bool,
    /// The mesh edge the walk ENDED on, or -1 where it ended inside its face.
    pub edge: i32,
    /// Where along that edge, 0 at vertex 0 and 1 at 1.
    pub parameter: f64,
    /// The mesh VERTEX the walk ended ON, or -1 where it ended anywhere but a corner. The edge can
    /// name only one of the folds meeting at such a corner, so the vertex is carried beside it.
    pub vertex: i32,
}

impl WalkedEnd {
    /// C#'s `WalkedEnd` constructor (its optional arguments default to not join-placed, no fold
    /// crossed, no edge, parameter 0 and no vertex: pass `false, false, -1, 0.0, -1`).
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        boundary: i32,
        face: i32,
        direction: Vector3,
        is_join_placed: bool,
        crossed_a_fold: bool,
        edge: i32,
        parameter: f64,
        vertex: i32,
    ) -> Self {
        Self {
            boundary,
            face,
            direction,
            is_join_placed,
            crossed_a_fold,
            edge,
            parameter,
            vertex,
        }
    }
}

/// Where a join-placed point stands (C# `TryAlongTheJoin`'s outs): the point, the face of the piece
/// it falls on, the join's direction there, and the mesh edge (`fold_edge`, or -1), where along it
/// (`along`) and the mesh vertex (`corner`, or -1) it stands ON.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AlongTheJoin {
    pub point: Vector3,
    pub face: i32,
    pub direction: Vector3,
    pub fold_edge: i32,
    pub along: f64,
    pub corner: i32,
}

/// C# `TheNeighboursEnd`: the neighbour station `at`'s boundary point on one side, and the faces it
/// may be read as lying in. At a JUNCTION's wall side the end the join touches is the wall end on the
/// join's own edge (the LEAVING end where the join leaves the junction, the ARRIVING end where it
/// arrives), carried onto the facet the junction recorded; everywhere else it is the section's own
/// boundary point and [`the_faces_the_end_lies_in`].
#[allow(clippy::too_many_arguments)]
pub fn the_neighbours_end(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    pool: &SectionPool,
    sections: &[BevelSection],
    junctions: Option<&IndexMap<i32, JunctionSection>>,
    at: i32,
    the_join_leaves_it: bool,
    on_the_left: bool,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
) -> (Vector3, Vec<i32>) {
    let section = &sections[at as usize];

    if let Some(junction) = junctions.and_then(|junctions| junctions.get(&at)) {
        if junction.flat_on_the_left != on_the_left {
            let end = if the_join_leaves_it {
                junction.leaving_wall_end
            } else {
                junction.arriving_wall_end
            } as usize;

            if junction.carried_onto[end] >= 0 {
                return (
                    pool.position_of(junction.samples[end]),
                    vec![junction.carried_onto[end]],
                );
            }
        }
    }

    (
        pool.position_of(if on_the_left {
            section.left_boundary
        } else {
            section.right_boundary
        }),
        the_faces_the_end_lies_in(
            mesh,
            graph,
            edge_at_pair,
            pool,
            section,
            on_the_left,
            is_a_feature_edge,
        ),
    )
}

/// C# `TheFacesTheEndLiesIn`: every face a neighbour's boundary point may be read as lying in - the
/// one face its walk ended in, or BOTH faces of the fold a slide rides, since a point on a fold is in
/// each of them. Where the point was neither walked nor slid there is no face to set out from.
///
/// An END section's slide generally runs PAST the crease's first chord - the one the section
/// recorded - so which chord holds the landing is walked for, by the very reading the smooth side's
/// boundary uses ([`section_smooth_side_boundary::try_the_crease_chord_under`]).
pub fn the_faces_the_end_lies_in(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    pool: &SectionPool,
    section: &BevelSection,
    on_the_left: bool,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
) -> Vec<i32> {
    let walk_face = if on_the_left {
        section.left_walk_face
    } else {
        section.right_walk_face
    };

    if walk_face >= 0 {
        return vec![walk_face];
    }

    let slide_edge = if on_the_left {
        section.left_slide_edge
    } else {
        section.right_slide_edge
    };
    let kind = if on_the_left {
        section.left_kind
    } else {
        section.right_kind
    };

    if kind != SectionSide::Slide || slide_edge < 0 {
        return Vec::new();
    }

    let mut chord = slide_edge;

    if section.kind == SectionKind::End {
        let landing = pool.position_of(if on_the_left {
            section.left_boundary
        } else {
            section.right_boundary
        });

        if let Ok(under) = section_smooth_side_boundary::try_the_crease_chord_under(
            mesh,
            graph,
            edge_at_pair,
            is_a_feature_edge,
            slide_edge,
            section.chain_vertex,
            landing,
        ) {
            chord = under;
        }
    }

    (0..graph.get_face_count(chord))
        .map(|k| graph.get_face(chord, k))
        .collect()
}

/// C# `TryAlongTheJoin`: the point a fraction `parameter` of the way along a surface polyline - its
/// two ends and the fold crossings between them - and the PLACE of the piece it falls on: the face,
/// and where the point stands on a fold or a corner of that face, the edge and vertex too
/// ([`name_the_place`]). A point landing on a crossing IS that crossing, by the crossing's own
/// position. `lattice` is the mesh's own grid (`SectionSurfaceWalk.GridOf`).
#[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
pub fn try_along_the_join(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    from: Vector3,
    from_face: i32,
    to: Vector3,
    crossings: &[SurfaceWalkCrossing],
    parameter: f64,
    lattice: f64,
) -> Option<AlongTheJoin> {
    let mut points = Vec::with_capacity(crossings.len() + 2);
    points.push(from);
    points.extend(crossings.iter().map(|crossing| crossing.at));
    points.push(to);

    let mut total = 0.0;

    for k in 1..points.len() {
        total += (points[k] - points[k - 1]).length();
    }

    if !(total > 0.0) {
        return None;
    }

    let wanted = parameter * total;

    let mut face = from_face;
    let mut walked = 0.0;

    for k in 1..points.len() {
        let step = points[k] - points[k - 1];
        let length = step.length();

        if walked + length >= wanted || k == points.len() - 1 {
            let fraction = if length > 0.0 {
                (wanted - walked) / length
            } else {
                0.0
            };

            // C# `Math.Max(0, Math.Min(1, fraction))`.
            let fraction = cs_math::max_f64(0.0, cs_math::min_f64(1.0, fraction));

            let mut point = points[k - 1] + (step * fraction);
            let direction = step.get_normal();

            let (fold_edge, along, corner) = name_the_place(
                mesh,
                graph,
                edge_at_pair,
                crossings,
                lattice,
                face,
                &mut point,
            );

            return Some(AlongTheJoin {
                point,
                face,
                direction,
                fold_edge,
                along,
                corner,
            });
        }

        walked += length;
        face = the_other_face(graph, crossings[k - 1].edge, face);
    }

    None
}

/// C# `NameThePlace`: names what a join-placed point stands on - and stands it exactly there - as
/// `(fold_edge, along, corner)`. A point on one of the join's own crossings takes that crossing's
/// edge and parameter; a point on a CORNER of its face takes the corner, is moved onto it, and takes
/// one of that corner's own folds beside it. Which fold that is, is rounding - the corner is the
/// reading that is not - so a consumer must read the corner first.
pub fn name_the_place(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    crossings: &[SurfaceWalkCrossing],
    lattice: f64,
    face: i32,
    point: &mut Vector3,
) -> (i32, f64, i32) {
    let mut fold_edge = -1;
    let mut along = 0.0;

    if face < 0 {
        return (fold_edge, along, -1);
    }

    if let Some(crossing) = crossings
        .iter()
        .find(|crossing| (crossing.at - *point).length() <= lattice)
    {
        fold_edge = crossing.edge;
        along = crossing.parameter;
    }

    let Some(corner) = try_the_corner_under(mesh, face, *point, lattice) else {
        return (fold_edge, along, -1);
    };

    *point = Vector3::from_vector3_float(mesh.vertices[corner as usize]);

    if fold_edge < 0 || !is_a_corner_of(graph, fold_edge, corner) {
        // A corner with no crossing to name a fold for it - the join ran through it inside one face,
        // or across a fold that is not this corner's - still stands on folds of its own. Any of the
        // face's own two meeting there will do, since the corner is the reading that carries.
        fold_edge = the_face_edge_at(mesh, edge_at_pair, face, corner);
    }

    along = if fold_edge >= 0 {
        if graph.get_vertex0(fold_edge) == corner {
            0.0
        } else {
            1.0
        }
    } else {
        0.0
    };

    (fold_edge, along, corner)
}

/// C# `IsACornerOf`: whether a mesh edge has the given vertex for one of its two ends.
pub fn is_a_corner_of(graph: &MeshEdgeGraph, edge: i32, vertex: i32) -> bool {
    graph.get_vertex0(edge) == vertex || graph.get_vertex1(edge) == vertex
}

/// C# `TheFaceEdgeAt`: one edge of a face meeting at the given corner of it, or -1. Two of the three
/// do; which one comes back is the face's own winding order and so does not move with the part's
/// placement.
pub fn the_face_edge_at(mesh: &Mesh, edge_at_pair: &EdgeAtPair, face: i32, corner: i32) -> i32 {
    let drawn = &mesh.faces[face as usize];
    let corners = [drawn.v0, drawn.v1, drawn.v2];

    for i in 0..3 {
        let one = corners[i];
        let other = corners[(i + 1) % 3];

        if one == corner || other == corner {
            if let Some(&edge) = edge_at_pair.get(&section_room::pair_key(one, other)) {
                return edge;
            }
        }
    }

    -1
}

/// C# `TryTheCornerUnder`: the corner of a face a point stands on, within the given lattice - the
/// nearest such, the later on a tie - or `None` where it stands on none.
pub fn try_the_corner_under(mesh: &Mesh, face: i32, at: Vector3, lattice: f64) -> Option<i32> {
    let drawn = &mesh.faces[face as usize];
    let mut nearest = lattice;
    let mut corner = -1;

    for candidate in [drawn.v0, drawn.v1, drawn.v2] {
        let off = (Vector3::from_vector3_float(mesh.vertices[candidate as usize]) - at).length();

        if off <= nearest {
            nearest = off;
            corner = candidate;
        }
    }

    (corner >= 0).then_some(corner)
}

/// C# `TheOtherFace`: the face on the far side of an edge from the one given, or that one where
/// there is none.
pub fn the_other_face(graph: &MeshEdgeGraph, edge: i32, face: i32) -> i32 {
    (0..graph.get_face_count(edge))
        .map(|k| graph.get_face(edge, k))
        .find(|&other| other != face)
        .unwrap_or(face)
}

/// C# `TheJoinsOwnPoint`: a SPLIT station's boundary point on one OFFSET side, placed ON the join
/// between its two unsplit neighbours' ends - the line along the surface the neighbours' cut runs
/// along - at the blend's own `placement.parameter` along it, so the split point stands on that cut
/// rather than a rounding off it. The neighbour's end is each face it may be read as lying in, tried
/// in turn until a join from one to the other is laid. A side that is not an offset, or a join that
/// cannot be laid, walks nothing (`boundary` -1). `Err` where the placed point is not a number.
#[allow(clippy::too_many_arguments)]
pub fn the_joins_own_point(
    sections: &[BevelSection],
    junctions: Option<&IndexMap<i32, JunctionSection>>,
    placement: SplitPlacement,
    at: i32,
    on_the_left: bool,
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    pool: &mut SectionPool,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
    lattice: f64,
) -> Result<WalkedEnd, NanSectionPoint> {
    let section = &sections[at as usize];
    let none = WalkedEnd::new(-1, -1, Vector3::ZERO, false, false, -1, 0.0, -1);

    let kind = if on_the_left {
        section.left_kind
    } else {
        section.right_kind
    };

    if kind != SectionSide::Offset {
        return Ok(none);
    }

    // The join LEAVES the neighbour before this station and ARRIVES at the one after it, which at a
    // junction says which of its two wall ends the join's far end is - see the_neighbours_end.
    let (from, from_faces) = the_neighbours_end(
        mesh,
        graph,
        edge_at_pair,
        pool,
        sections,
        junctions,
        placement.before,
        true,
        on_the_left,
        is_a_feature_edge,
    );
    let (to, to_faces) = the_neighbours_end(
        mesh,
        graph,
        edge_at_pair,
        pool,
        sections,
        junctions,
        placement.after,
        false,
        on_the_left,
        is_a_feature_edge,
    );

    for &from_face in &from_faces {
        for &to_face in &to_faces {
            let Ok(crossings) = section_surface_walk_join::try_join(
                mesh,
                graph,
                edge_at_pair,
                from,
                from_face,
                to,
                to_face,
                is_a_feature_edge,
            ) else {
                continue;
            };

            let Some(placed) = try_along_the_join(
                mesh,
                graph,
                edge_at_pair,
                from,
                from_face,
                to,
                &crossings,
                placement.parameter,
                lattice,
            ) else {
                continue;
            };

            return Ok(WalkedEnd::new(
                pool.add(placed.point)?,
                placed.face,
                placed.direction,
                true,
                section_surface_walk::crossed_a_fold(mesh, graph, Some(&crossings)),
                placed.fold_edge,
                placed.along,
                placed.corner,
            ));
        }
    }

    Ok(none)
}

/// C# `Rewritten`: a section with its offset sides' WALKED ends put in - each walked side's boundary
/// point as its profile's end, and the walk's face, direction, placement, fold, edge, parameter and
/// vertex recorded on the section. A section neither of whose sides was walked comes back as it was.
pub fn rewritten(section: &BevelSection, left: &WalkedEnd, right: &WalkedEnd) -> BevelSection {
    if left.face < 0 && right.face < 0 {
        return section.clone();
    }

    let mut profile = section.profile.clone();
    let last = profile.len() - 1;

    if left.boundary >= 0 {
        profile[0] = left.boundary;
    }

    if right.boundary >= 0 {
        profile[last] = right.boundary;
    }

    BevelSection {
        left_boundary: profile[0],
        right_boundary: profile[last],
        profile,
        left_walk_face: left.face,
        right_walk_face: right.face,
        left_walk_direction: left.direction,
        right_walk_direction: right.direction,
        left_boundary_is_join_placed: left.is_join_placed,
        right_boundary_is_join_placed: right.is_join_placed,
        left_walk_crossed_a_fold: left.crossed_a_fold,
        right_walk_crossed_a_fold: right.crossed_a_fold,
        left_walk_edge: left.edge,
        right_walk_edge: right.edge,
        left_walk_parameter: left.parameter,
        right_walk_parameter: right.parameter,
        left_walk_vertex: left.vertex,
        right_walk_vertex: right.vertex,
        ..section.clone()
    }
}

/// Why a section's side could not be laid: refused by name, or a point that is not a number (C#
/// throws there).
#[derive(Debug, Clone, PartialEq)]
pub enum SectionBuildRefusal {
    /// Refused, with the advisory the user is shown.
    Refused(crate::bevel::BevelAdvisory),
    /// A section point that is not a number.
    NotANumber(NanSectionPoint),
}

impl From<NanSectionPoint> for SectionBuildRefusal {
    fn from(nan: NanSectionPoint) -> Self {
        SectionBuildRefusal::NotANumber(nan)
    }
}

/// C# `SectionBuilder.TryLayOneSide`: an OFFSET side's boundary walked along the surface from the
/// station, through the face of the chain vertex's fan that holds it - the walked end, or the
/// unwalked one (`WalkedEnd(-1, -1, 0)`) where the side is not an offset or no walk starts.
/// Refused (`SizeExceedsFaceWidth`) where the walk meets a crease before it has gone the reach.
#[allow(clippy::too_many_arguments, clippy::neg_cmp_op_on_partial_ord)]
pub fn try_lay_one_side(
    section: &BevelSection,
    arriving: &crate::topology::ChainEdge,
    leaving: &crate::topology::ChainEdge,
    on_the_left: bool,
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    pool: &mut SectionPool,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
) -> Result<WalkedEnd, SectionBuildRefusal> {
    let unwalked = WalkedEnd::new(-1, -1, Vector3::ZERO, false, false, -1, 0.0, -1);
    let (kind, boundary) = if on_the_left {
        (section.left_kind, section.left_boundary)
    } else {
        (section.right_kind, section.right_boundary)
    };

    if kind != SectionSide::Offset {
        return Ok(unwalked);
    }

    let reach = pool.position_of(boundary) - section.station;
    let distance = reach.length();
    if !(distance > 0.0) {
        return Ok(unwalked);
    }

    let Ok((_, fan)) = super::section_builder::try_creases_in_the_fan_reading(
        section.chain_vertex,
        arriving,
        leaving,
        on_the_left,
        mesh,
        graph,
        edge_at_pair,
        is_a_feature_edge,
    ) else {
        return Ok(unwalked);
    };

    let Ok((face, in_face)) = super::section_surface_walk_faces::try_face_holding(
        mesh,
        section.chain_vertex,
        reach,
        &fan,
    ) else {
        return Ok(unwalked);
    };

    let walk = match super::section_surface_walk_try::try_walk(
        mesh,
        graph,
        edge_at_pair,
        section.station,
        face,
        in_face,
        distance,
        is_a_feature_edge,
    ) {
        Ok(walk) => walk,
        Err(walk) => {
            if walk.refusal == super::section_surface_walk::SurfaceWalkRefusal::CreaseReached {
                return Err(SectionBuildRefusal::Refused(
                    crate::bevel::BevelAdvisory::new(
                        crate::bevel::BevelAdvisoryCode::SizeExceedsFaceWidth,
                        vec![walk.end.into()],
                    ),
                ));
            }

            return Ok(unwalked);
        }
    };

    let drawn = pool.position_of(boundary);
    let walked_boundary = if section_surface_walk::is_the_same_point(drawn, walk.end, distance) {
        boundary
    } else {
        pool.add(walk.end)?
    };

    Ok(WalkedEnd::new(
        walked_boundary,
        walk.end_face,
        walk.end_direction,
        false,
        section_surface_walk::crossed_a_fold(mesh, graph, Some(&walk.crossings)),
        walk.end_edge,
        walk.end_parameter,
        walk.end_vertex,
    ))
}
