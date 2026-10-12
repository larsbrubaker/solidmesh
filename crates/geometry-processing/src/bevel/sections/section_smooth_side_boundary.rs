//! The boundary of a SMOOTH side - one the cut is laid on the surface for rather than drawn straight
//! off the chain (`Bevel/Sections/SectionSmoothSide.Boundary.cs`, part of the C# `internal static
//! partial class SectionSmoothSide`; a module of free functions here). The boundary is laid station by
//! station ([`try_lay_the_boundary`]) and joined along the surface ([`try_the_join`]), with the
//! crossings a join need not mint dropped; an end section's landing is found by walking down its crease
//! to the chord that holds it ([`try_the_crease_chord_under`]). A junction station's rim and the
//! crease takeovers beside it are in [`super::section_smooth_side_junction_rim`].

use indexmap::{IndexMap, IndexSet};
use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::Vector3;

use super::section_fan_walk::EdgeAtPair;
use super::section_smooth_side::{corners, SmoothSideCut, SmoothSidePlace, SmoothSideRefusal};
use super::section_smooth_side_junction_rim;
use super::section_surface_walk::grid_of;
use super::section_surface_walk_faces::try_face_holding;
use super::section_surface_walk_join::try_join;
use super::section_surface_walk_try::try_walk;
use super::{
    NanSectionPoint, SectionKind, SectionPool, SectionRun, SectionSide, SurfaceWalkCrossing,
};

/// C# `LatticeFraction`: how near an end of a ring edge an inserted point may not be taken as a new
/// point.
pub const LATTICE_FRACTION: f64 = 1e-9;

/// C# `TheLongestCreaseWalk`: how many chords of a crease a slide may run down before the walk gives
/// it up.
const THE_LONGEST_CREASE_WALK: i32 = 64;

/// The FEATURE edges leaving each vertex.
pub type CreasesAtEachVertex = IndexMap<i32, Vec<i32>>;

/// Why a smooth side's boundary or cut failed: a named refusal and where on the part it stands
/// (C#'s `out refusal, out where`), or a point that came out not a number, which C#'s pool throws on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SmoothSideFailure {
    /// The cut refused, by name, at that point.
    Refused(SmoothSideRefusal, Vector3),
    /// A point minted into the pool was NaN (C# `SectionPool.Add` throws).
    NotANumber(NanSectionPoint),
}

impl From<NanSectionPoint> for SmoothSideFailure {
    fn from(nan: NanSectionPoint) -> Self {
        Self::NotANumber(nan)
    }
}

/// C# `TryLayTheBoundary`: the boundary, laid on the surface - every station's own point, and the
/// fold crossings between consecutive points. An OFFSET station's point is its setback WALKED along
/// the surface; a SLIDE station's is the point it already slid to, which lies on a fold. A JUNCTION
/// station hands THREE points - its own rim P1, F1, P2
/// ([`section_smooth_side_junction_rim::try_lay_the_junctions_rim`]) - and where a junction's wall end
/// was carried past a crease the crossing onto that crease is minted inside its group too
/// ([`section_smooth_side_junction_rim::mint_the_crease_takeovers`]).
///
/// A place names the face its point is IN. Every place written here comes from something that
/// LANDED - the end face of a walk the run made, the end face of a join the run placed a split
/// station along, the crease chord an End slid onto, or a walk made here and now. A station the run
/// could neither walk nor place carries no face (`-1`) and is walked again here; where that walk will
/// not go the whole cut is refused BY NAME rather than the point being labelled with a face it is not
/// in - a join OUT of a station is laid in the plane of the face its place names.
#[allow(clippy::too_many_arguments)]
pub fn try_lay_the_boundary(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    pool: &mut SectionPool,
    run: &SectionRun,
    smooth_side_is_left: bool,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
    side: &IndexSet<i32>,
    facet_of: &IndexMap<i32, i32>,
    cut: &mut SmoothSideCut,
) -> Result<(), SmoothSideFailure> {
    for station in 0..run.sections.len() {
        let section = &run.sections[station];
        let mut points = Vec::new();

        // A JUNCTION owns TWO wall ends on TWO walls with the fold between them, so one point cannot
        // lay it and the freshly-walked branch below would mint a third the loft never uses. Its own
        // rim is taken instead.
        if let Some(junction) = run.junctions.get(&(station as i32)) {
            if junction.flat_on_the_left != smooth_side_is_left {
                let laid = section_smooth_side_junction_rim::try_lay_the_junctions_rim(
                    mesh,
                    graph,
                    edge_at_pair,
                    pool,
                    section,
                    junction,
                    is_a_feature_edge,
                    side,
                    cut,
                    &mut points,
                );
                cut.station_points.push(points);
                laid?;

                continue;
            }
        }

        let (kind, boundary, slide_edge) = if smooth_side_is_left {
            (
                section.left_kind,
                section.left_boundary,
                section.left_slide_edge,
            )
        } else {
            (
                section.right_kind,
                section.right_boundary,
                section.right_slide_edge,
            )
        };

        if kind == SectionSide::Slide {
            let mut chord = slide_edge;

            if section.kind == SectionKind::End {
                // An end slid down a crease that may be a POLYLINE, and the slide generally runs past
                // the first chord: which chord actually holds the landing is walked for, not assumed.
                match try_the_crease_chord_under(
                    mesh,
                    graph,
                    edge_at_pair,
                    is_a_feature_edge,
                    slide_edge,
                    section.chain_vertex,
                    pool.position_of(boundary),
                ) {
                    Ok(under) => chord = under,
                    Err(_) => {
                        cut.station_points.push(points);

                        return Err(SmoothSideFailure::Refused(
                            SmoothSideRefusal::TheEndIsNotOnItsCrease,
                            pool.position_of(boundary),
                        ));
                    }
                }
            }

            points.push(boundary);
            cut.station_points.push(points);
            cut.station_boundary.push(boundary);
            cut.places.insert(
                boundary,
                SmoothSidePlace::new(
                    -1,
                    chord,
                    parameter_on(mesh, graph, chord, pool.position_of(boundary)),
                    -1,
                ),
            );
            cut.station_faces.push(the_faces_on(graph, chord, side));

            continue;
        }

        let walk_face = if smooth_side_is_left {
            section.left_walk_face
        } else {
            section.right_walk_face
        };

        if walk_face >= 0 {
            // The run already laid this station's reach along the surface and minted the point it
            // ended on. Walking again here would mint a SECOND point for the one thing, and the loft
            // would then meet a wall cut somewhere it does not end.
            let walk_edge = if smooth_side_is_left {
                section.left_walk_edge
            } else {
                section.right_walk_edge
            };

            points.push(boundary);
            cut.station_points.push(points);
            cut.station_boundary.push(boundary);

            // Where that walk ENDED ON a fold the point lies on an edge, exactly as a slid one does,
            // and must be placed as one: the join into and out of this station meets that same fold
            // at the landing itself, and only an edge place lets `drop_the_ends_own_fold` read the two
            // as one point. The edge is what the walk recorded, never re-derived from a position.
            let walk_vertex = if smooth_side_is_left {
                section.left_walk_vertex
            } else {
                section.right_walk_vertex
            };

            let place = if walk_edge >= 0 {
                SmoothSidePlace::new(
                    -1,
                    walk_edge,
                    if smooth_side_is_left {
                        section.left_walk_parameter
                    } else {
                        section.right_walk_parameter
                    },
                    walk_vertex,
                )
            } else {
                SmoothSidePlace::new(walk_face, -1, 0.0, walk_vertex)
            };
            cut.places.insert(boundary, place);

            cut.station_faces.push(vec![walk_face]);

            if if smooth_side_is_left {
                section.left_boundary_is_join_placed
            } else {
                section.right_boundary_is_join_placed
            } {
                cut.join_placed.insert(boundary);
            }

            continue;
        }

        cut.station_points.push(points);

        let fan: Vec<i32> = side
            .iter()
            .copied()
            .filter(|&f| corners(mesh, f).contains(&section.chain_vertex))
            .collect();
        let offset = pool.position_of(boundary) - section.station;

        let Ok((face, in_face)) = try_face_holding(mesh, section.chain_vertex, offset, &fan) else {
            return Err(SmoothSideFailure::Refused(
                SmoothSideRefusal::NoFanFaceHoldsTheOffset,
                section.station,
            ));
        };

        let walk = match try_walk(
            mesh,
            graph,
            edge_at_pair,
            section.station,
            face,
            in_face,
            section.setback,
            is_a_feature_edge,
        ) {
            Ok(walk) => walk,
            Err(walk) => {
                return Err(SmoothSideFailure::Refused(
                    SmoothSideRefusal::TheWalkWasStopped,
                    walk.end,
                ));
            }
        };

        let landed = pool.add(walk.end)?;

        let last = cut.station_points.len() - 1;
        cut.station_points[last].push(landed);
        cut.station_boundary.push(landed);
        cut.places.insert(
            landed,
            SmoothSidePlace::new(walk.end_face, -1, 0.0, walk.end_vertex),
        );
        cut.station_faces.push(vec![walk.end_face]);
    }

    // Every station is laid before this runs, because a takeover is read from the ORDINARY station
    // on the far side of the junction's own group - which at a LEAVING junction is laid after it.
    section_smooth_side_junction_rim::mint_the_crease_takeovers(
        mesh,
        graph,
        edge_at_pair,
        pool,
        run,
        smooth_side_is_left,
        is_a_feature_edge,
        side,
        cut,
    )?;

    // An open boundary has one join FEWER than it has stations: nothing joins its last point back to
    // its first, because the band there is closed by the end creases instead.
    let count = cut.station_boundary.len();
    let joins = if cut.boundary_is_closed {
        count as i32
    } else {
        count as i32 - 1
    };

    // The mesh's own lattice, read once for the whole boundary: it is a reading of the PART (its
    // size and where it stands) and not of any one join, and `grid_of` walks every vertex.
    let lattice_of_the_mesh = grid_of(mesh);

    for i in 0..joins.max(0) as usize {
        let next = (i + 1) % count;
        let (from, to) = (cut.station_boundary[i], cut.station_boundary[next]);

        let Some(mut crossings) = try_the_join(
            mesh,
            graph,
            edge_at_pair,
            pool,
            from,
            &cut.station_faces[i],
            to,
            &cut.station_faces[next],
            is_a_feature_edge,
        ) else {
            return Err(SmoothSideFailure::Refused(
                SmoothSideRefusal::TheJoinWasRefused,
                pool.position_of(from),
            ));
        };

        drop_the_crossings_inside_one_facet(graph, facet_of, &mut crossings);
        drop_the_ends_own_fold(
            mesh,
            graph,
            lattice_of_the_mesh,
            cut,
            &mut crossings,
            from,
            to,
        );

        let mut minted = Vec::with_capacity(crossings.len());

        for crossing in &crossings {
            let index = pool.add(crossing.at)?;

            minted.push(index);
            cut.places.insert(
                index,
                SmoothSidePlace::new(-1, crossing.edge, crossing.parameter, -1),
            );
        }

        cut.crossings_after.push(minted);
    }

    for i in 0..cut.station_boundary.len() {
        cut.boundary.push(cut.station_boundary[i]);

        if i < cut.crossings_after.len() {
            let crossings = cut.crossings_after[i].clone();
            cut.boundary.extend(crossings);
        }
    }

    Ok(())
}

/// C# `DropTheCrossingsInsideOneFacet`: drops every crossing of a mesh edge whose two faces lie in
/// the SAME flat piece. Such an edge is a diagonal inside one facet, not a fold between two of them:
/// the point minted there is collinear with the join's own ends by construction and adds nothing to
/// the cut but a ring vertex the ears must step over - of the barrel mouth's 47 crossings, 41 are such
/// diagonals. The reading is TOPOLOGICAL - which facet each of the edge's faces was flooded into.
pub fn drop_the_crossings_inside_one_facet(
    graph: &MeshEdgeGraph,
    facet_of: &IndexMap<i32, i32>,
    crossings: &mut Vec<SurfaceWalkCrossing>,
) {
    crossings.retain(|crossing| {
        if graph.get_face_count(crossing.edge) != 2 {
            return true;
        }

        let one = facet_of.get(&graph.get_face(crossing.edge, 0));
        let other = facet_of.get(&graph.get_face(crossing.edge, 1));

        !matches!((one, other), (Some(one), Some(other)) if one == other)
    });
}

/// C# `DropTheEndsOwnFold`: drops a join's crossing of the fold one of its OWN ends stands on. A
/// SLIDE station's boundary point lies on a fold, so the straight join arriving there meets that fold
/// at the landing point itself - not a crossing to mint: the polyline vertex there IS the station's
/// point. This is IDENTITY and not tolerance.
pub fn drop_the_ends_own_fold(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    lattice: f64,
    cut: &SmoothSideCut,
    crossings: &mut Vec<SurfaceWalkCrossing>,
    from: i32,
    to: i32,
) {
    // C# indexes the places; every station point is placed before its joins are laid.
    if let (Some(first), Some(place)) = (crossings.first(), cut.places.get(&from)) {
        if is_the_ends_own_point(mesh, graph, lattice, place, first) {
            crossings.remove(0);
        }
    }

    if let (Some(last), Some(place)) = (crossings.last(), cut.places.get(&to)) {
        if is_the_ends_own_point(mesh, graph, lattice, place, last) {
            crossings.pop();
        }
    }
}

/// C# `IsTheEndsOwnPoint`: whether a join's first or last crossing is the point its own END already
/// is. On a fold that is the edge's identity. At a CORNER a point stands on every fold of that
/// vertex's fan at once, so a vertex end owns a crossing of ANY edge incident to that vertex that lands
/// AT the vertex within the mesh's lattice.
pub fn is_the_ends_own_point(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    lattice: f64,
    place: &SmoothSidePlace,
    crossing: &SurfaceWalkCrossing,
) -> bool {
    if place.vertex >= 0 {
        let corner = Vector3::from_vector3_float(mesh.vertices[place.vertex as usize]);

        if (graph.get_vertex0(crossing.edge) == place.vertex
            || graph.get_vertex1(crossing.edge) == place.vertex)
            && (crossing.at - corner).length() <= lattice
        {
            return true;
        }
    }

    place.edge >= 0 && crossing.edge == place.edge
}

/// C# `TryTheJoin`: the join between two boundary points, tried over every face either end may be
/// read as lying in - a point on a fold lies in both of the faces that fold joins, and only one of
/// them may reach the other end without leaving the side. A join is laid from the first point to the
/// second, ONE WAY, always.
#[allow(clippy::too_many_arguments)]
pub fn try_the_join(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    pool: &SectionPool,
    from: i32,
    from_faces: &[i32],
    to: i32,
    to_faces: &[i32],
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
) -> Option<Vec<SurfaceWalkCrossing>> {
    for &from_face in from_faces {
        for &to_face in to_faces {
            if let Ok(crossings) = try_join(
                mesh,
                graph,
                edge_at_pair,
                pool.position_of(from),
                from_face,
                pool.position_of(to),
                to_face,
                is_a_feature_edge,
            ) {
                return Some(crossings);
            }
        }
    }

    None
}

/// C# `TryTheCreaseChordUnder`: which CHORD of a crease an end section's landing (`at`) lies on,
/// walked for from the chain's end vertex - `Ok` where a crease leaving `from_vertex` holds it, and
/// `Err(first_chord)` (C#'s false with the chord left at the first one) where none does.
///
/// `SectionEnd` records only the crease's first chord leaving that vertex, and on a tessellated wall
/// that chord is far shorter than the slide: the raked bore's mouth starts with a chord 0.1214 long
/// and a 2 mm radius slides 0.2346, so the landing is two or three chords down. Several creases
/// generally leave an end vertex, and where the named one does not hold the landing, the others are
/// walked in turn; the one the landing stands nearest (lowest chord on a tie) wins.
#[allow(clippy::too_many_arguments)]
pub fn try_the_crease_chord_under(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
    first_chord: i32,
    from_vertex: i32,
    at: Vector3,
) -> Result<i32, i32> {
    let mut chord = -1;
    let mut nearest = f64::MAX;
    let mut creases = vec![first_chord];

    let at_each_vertex = the_creases_at_each_vertex(graph, edge_at_pair, is_a_feature_edge);

    if let Some(also_leaving) = at_each_vertex.get(&from_vertex) {
        creases.extend(
            also_leaving
                .iter()
                .copied()
                .filter(|&other| other != first_chord),
        );
    }

    for crease in creases {
        if let Some((found, off)) =
            try_the_crease_chord_down(mesh, graph, &at_each_vertex, crease, from_vertex, at)
        {
            if off < nearest || (off == nearest && found < chord) {
                nearest = off;
                chord = found;
            }
        }
    }

    if chord < 0 {
        return Err(first_chord);
    }

    Ok(chord)
}

/// C# `TryTheCreaseChordDown`: the walk itself, down ONE crease leaving the vertex, to the first chord
/// whose span holds the landing's projection - and how far the landing stands OFF that chord, which
/// is what tells one crease leaving the vertex from another. At each far vertex the walk takes the
/// feature edge that most nearly carries on the way it is going (the first such on a tie), and gives
/// up where nothing carries on.
///
/// C# builds the creases-at-each-vertex table lazily inside the walk; it is handed in here, built
/// once by the caller - the same table either way.
pub fn try_the_crease_chord_down(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    crease_at_vertex: &CreasesAtEachVertex,
    first_chord: i32,
    from_vertex: i32,
    at: Vector3,
) -> Option<(i32, f64)> {
    let mut chord = first_chord;
    let mut from = from_vertex;

    for _ in 0..THE_LONGEST_CREASE_WALK {
        let a = position(mesh, from);
        let far = other_end(graph, chord, from);
        let along = position(mesh, far) - a;

        if along.length_squared() < 1e-300 {
            return None;
        }

        let how_far = (at - a).dot(along) / along.length_squared();

        if how_far <= 1.0 {
            return Some((chord, (at - a - (along * how_far)).length()));
        }

        let leaving = crease_at_vertex.get(&far)?;

        let mut carries_on = -1;
        let mut straightest = 0.0;
        let way = along.get_normal();

        for &next in leaving {
            if next == chord {
                continue;
            }

            let other = other_end(graph, next, far);
            let direction = (position(mesh, other) - position(mesh, far)).get_normal();
            let straight = direction.dot(way);

            if carries_on < 0 || straight > straightest {
                carries_on = next;
                straightest = straight;
            }
        }

        if carries_on < 0 || straightest <= 0.0 {
            // Nothing leaves that vertex going on the way the crease was going, so the crease ENDS
            // there and the landing is past the end of it - which is not a slide this can place.
            return None;
        }

        chord = carries_on;
        from = far;
    }

    None
}

/// C# `TheCreasesAtEachVertex`: the FEATURE edges leaving each vertex, read off the edges-by-pair
/// table the operation already built, walked in that table's own order (C#'s insertion order - see
/// [`EdgeAtPair`]). Nothing where there is no feature test.
pub fn the_creases_at_each_vertex(
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
) -> CreasesAtEachVertex {
    let mut at: CreasesAtEachVertex = IndexMap::new();

    let Some(is_a_feature_edge) = is_a_feature_edge else {
        return at;
    };

    for &edge in edge_at_pair.values() {
        if !is_a_feature_edge(edge) {
            continue;
        }

        for vertex in [graph.get_vertex0(edge), graph.get_vertex1(edge)] {
            let leaving = at.entry(vertex).or_default();

            if !leaving.contains(&edge) {
                leaving.push(edge);
            }
        }
    }

    at
}

/// C# `TheFacesOn`: the faces of one edge that are on the given side.
pub fn the_faces_on(graph: &MeshEdgeGraph, edge: i32, side: &IndexSet<i32>) -> Vec<i32> {
    let mut faces = Vec::new();

    for i in 0..graph.get_face_count(edge) {
        if side.contains(&graph.get_face(edge, i)) {
            faces.push(graph.get_face(edge, i));
        }
    }

    faces
}

/// C# `ParameterOn`: where `at` projects along `edge`, 0 at the graph's vertex 0 and 1 at its vertex
/// 1, kept [`LATTICE_FRACTION`] clear of either end (0 on a degenerate edge).
pub fn parameter_on(mesh: &Mesh, graph: &MeshEdgeGraph, edge: i32, at: Vector3) -> f64 {
    let a = position(mesh, graph.get_vertex0(edge));
    let along = position(mesh, graph.get_vertex1(edge)) - a;

    if along.length_squared() < 1e-300 {
        0.0
    } else {
        ((at - a).dot(along) / along.length_squared())
            .clamp(LATTICE_FRACTION, 1.0 - LATTICE_FRACTION)
    }
}

/// The end of `edge` that is not `vertex`.
fn other_end(graph: &MeshEdgeGraph, edge: i32, vertex: i32) -> i32 {
    if graph.get_vertex0(edge) == vertex {
        graph.get_vertex1(edge)
    } else {
        graph.get_vertex0(edge)
    }
}

/// A vertex's position, read as doubles.
fn position(mesh: &Mesh, vertex: i32) -> Vector3 {
    Vector3::from_vector3_float(mesh.vertices[vertex as usize])
}
