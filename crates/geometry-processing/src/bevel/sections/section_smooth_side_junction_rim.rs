//! A JUNCTION station on a smooth side's boundary: free functions called by
//! [`super::section_smooth_side_boundary::try_lay_the_boundary`]. A junction owns TWO wall
//! ends on two walls with the fold between them, so it hands the boundary its own rim - P1, F1, P2 in
//! walk order ([`try_lay_the_junctions_rim`]) - and where a wall end was CARRIED past a feature crease,
//! the point on that crease where the ordinary stations' boundary hands over to the rim is minted into
//! the junction's group ([`mint_the_crease_takeovers`]).

use indexmap::IndexSet;
use polygon_mesh::{Mesh, MeshEdgeGraph};

use super::section_fan_walk::EdgeAtPair;
use super::section_smooth_side::{
    corners, normal_of, SmoothSideCut, SmoothSidePlace, SmoothSideRefusal,
};
use super::section_smooth_side_boundary::{
    parameter_on, the_creases_at_each_vertex, the_faces_on, try_the_crease_chord_under,
    SmoothSideFailure,
};
use super::section_surface_walk::is_the_same_point;
use super::section_surface_walk_faces::try_face_holding;
use super::section_surface_walk_try::try_walk;
use super::{BevelSection, JunctionSection, SectionPool, SectionRun, SurfaceWalkRefusal};

/// Lays a JUNCTION station's own rim on the smooth side - P1, F1, P2 in
/// walk order, taken from the ledger by pool index so that no point is minted twice and the loft
/// meets the cut by identity. P1 and P2 are the two canals' wall tangencies, each in its own wall, and
/// F1 is where the arriving canal's tangency line meets the FOLD between those walls - so each of the
/// two joins laid between them lies in ONE wall and the fold is crossed exactly once, at F1. F1 is
/// placed as an EDGE place on the chord of the fold that actually holds it, which on a tessellated
/// wall is generally not the first chord leaving the vertex.
///
/// The station's points go into `points` (already the cut's entry for this station); its
/// boundary, places and faces into `cut`.
#[allow(clippy::too_many_arguments)]
pub fn try_lay_the_junctions_rim(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    pool: &SectionPool,
    section: &BevelSection,
    junction: &JunctionSection,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
    side: &IndexSet<i32>,
    cut: &mut SmoothSideCut,
    points: &mut Vec<i32>,
) -> Result<(), SmoothSideFailure> {
    let fan: Vec<i32> = side
        .iter()
        .copied()
        .filter(|&f| corners(mesh, f).contains(&section.chain_vertex))
        .collect();
    let creases_at = the_creases_at_each_vertex(graph, edge_at_pair, is_a_feature_edge);

    let Some(first_crease) = creases_at
        .get(&section.chain_vertex)
        .and_then(|creases| creases.first().copied())
    else {
        return Err(SmoothSideFailure::Refused(
            SmoothSideRefusal::TheEndIsNotOnItsCrease,
            section.station,
        ));
    };

    let f1 = pool.position_of(junction.fold_point);

    let Ok(fold) = try_the_crease_chord_under(
        mesh,
        graph,
        edge_at_pair,
        is_a_feature_edge,
        first_crease,
        section.chain_vertex,
        f1,
    ) else {
        return Err(SmoothSideFailure::Refused(
            SmoothSideRefusal::TheEndIsNotOnItsCrease,
            f1,
        ));
    };

    let arriving_end = junction.samples[junction.arriving_wall_end as usize];
    let leaving_end = junction.samples[junction.leaving_wall_end as usize];

    for rim in [arriving_end, junction.fold_point, leaving_end] {
        points.push(rim);
        cut.station_boundary.push(rim);

        if rim == junction.fold_point {
            cut.places.insert(
                rim,
                SmoothSidePlace::new(-1, fold, parameter_on(mesh, graph, fold, f1), -1),
            );
            cut.station_faces.push(the_faces_on(graph, fold, side));

            continue;
        }

        let at = pool.position_of(rim);
        let offset = at - section.station;
        let sample = if rim == arriving_end {
            junction.arriving_wall_end
        } else {
            junction.leaving_wall_end
        };

        // A CARRIED END IS ALREADY PLACED, AND THE WALK IS THEN REDUNDANT. The junction takes a far
        // wall end onto the facet that actually holds it (`JunctionSection::carried_onto`) - the same
        // question this walk asks, answered by whoever MOVED the point, and where the two disagree the
        // carry is right by construction: the point was put on that facet's plane. Read on the
        // pierced top face (2026-09-18, station 41): P1 is carried onto face 70, the cube's flat -X
        // wall, which holds it exactly, while the walk ends in bore facet 230, whose plane it stands
        // 0.1156 off - the carry crossed the bore's MOUTH and the walk could not.
        if (sample as usize) < junction.carried_onto.len()
            && junction.carried_onto[sample as usize] >= 0
        {
            let onto = junction.carried_onto[sample as usize];
            cut.places
                .insert(rim, SmoothSidePlace::new(onto, -1, 0.0, -1));
            cut.station_faces.push(vec![onto]);

            continue;
        }

        // EACH END WALKS ITS OWN WALL'S HALF OF THE FAN. The fold between the two walls runs through
        // this fan, and a walk handed the WHOLE of it sets out through whichever face holds its
        // direction - which for P2 is the ARRIVING wall's, the fold then stopping the walk before it
        // ever reaches the leaving wall's facet the point really stands in. Which wall an end is on is
        // the JUNCTION's to say, so the fan is sided by it and nothing here reads a normal.
        let half: Vec<i32> = fan
            .iter()
            .copied()
            .filter(|&f| {
                junction.lies_in_the_arriving_wall(normal_of(mesh, f)) == (rim == arriving_end)
            })
            .collect();

        if half.is_empty() {
            return Err(SmoothSideFailure::Refused(
                SmoothSideRefusal::NoFanFaceHoldsTheOffset,
                at,
            ));
        }

        let Ok((face, in_face)) = try_face_holding(mesh, section.chain_vertex, offset, &half)
        else {
            return Err(SmoothSideFailure::Refused(
                SmoothSideRefusal::NoFanFaceHoldsTheOffset,
                at,
            ));
        };

        // THE FAN FACE HOLDS THE DIRECTION, NOT THE POINT. A wall end stands a whole radius out from
        // the junction vertex, and on a tessellated wall that is several faces past the one face of
        // the fan the offset sets out through - read on the pierced top face (2026-09-18): P1 was
        // placed in face 38, a sliver hugging the rim, which it stands 0.3796 OUTSIDE, and the join
        // into it refused JoinLeftTheStrip while the very same line WALKED arrives exactly, in face
        // 37. So the reach is WALKED, and the face the walk ends in is the place.
        let walk = match try_walk(
            mesh,
            graph,
            edge_at_pair,
            section.station,
            face,
            in_face,
            offset.length(),
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

        // The POINT stays the ledger's own - the loft meets the cut by pool index, so no second point
        // is minted for it here. Only where it is READ is taken from the walk.
        let place = if walk.end_edge >= 0 {
            SmoothSidePlace::new(
                -1,
                walk.end_edge,
                parameter_on(mesh, graph, walk.end_edge, at),
                walk.end_vertex,
            )
        } else {
            SmoothSidePlace::new(walk.end_face, -1, 0.0, walk.end_vertex)
        };
        cut.places.insert(rim, place);

        cut.station_faces.push(if walk.end_edge >= 0 {
            the_faces_on(graph, walk.end_edge, side)
        } else {
            vec![walk.end_face]
        });
    }

    Ok(())
}

/// Mints, where a junction's wall end was CARRIED past a feature crease,
/// the point on that crease where the boundary leaves the ordinary stations' facets and the
/// junction's rim takes over.
///
/// No single join spans a feature crease - a strip may not cross one. So the boundary is given the
/// crossing itself: the straight walk from the wall end towards the ordinary point stops
/// `CreaseReached` ON the crease, and that landing is minted with an EDGE place, appended INSIDE the
/// junction's own group - `[C, P1, F1, P2]` where the crease is crossed arriving, `[P1, F1, P2, C]`
/// where it is crossed leaving - so the ordinary joins either side keep their stations' readings and
/// the takeover join falls to [`SmoothSideCut::internal_crossings_of`], which the CUT consumes and no
/// loft row lays.
#[allow(clippy::too_many_arguments)]
pub fn mint_the_crease_takeovers(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    pool: &mut SectionPool,
    run: &SectionRun,
    smooth_side_is_left: bool,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
    side: &IndexSet<i32>,
    cut: &mut SmoothSideCut,
) -> Result<(), SmoothSideFailure> {
    let mut stations: Vec<i32> = run.junctions.keys().copied().collect();
    stations.sort_unstable();

    for station in stations {
        let junction = &run.junctions[&station];

        if junction.flat_on_the_left == smooth_side_is_left
            || station >= cut.station_points.len() as i32
        {
            continue;
        }

        let s = station as usize;

        if cut.station_points[s].is_empty() {
            continue;
        }

        let first = the_flattened_start_of(cut, station);
        let before = if first > 0 {
            first - 1
        } else if cut.boundary_is_closed {
            cut.station_boundary.len() as i32 - 1
        } else {
            -1
        };

        if before >= 0 {
            let ordinary = cut.station_boundary[before as usize];
            let wall_end = cut.station_points[s][0];

            if let Some((arriving, arriving_faces)) = try_the_takeover(
                mesh,
                graph,
                edge_at_pair,
                pool,
                is_a_feature_edge,
                side,
                cut,
                junction,
                junction.arriving_wall_end,
                ordinary,
                wall_end,
            )? {
                cut.station_points[s].insert(0, arriving);
                cut.station_boundary.insert(first as usize, arriving);
                cut.station_faces.insert(first as usize, arriving_faces);
            }
        }

        let first = the_flattened_start_of(cut, station);

        let after_point = first + cut.station_points[s].len() as i32;
        let after = if after_point < cut.station_boundary.len() as i32 {
            after_point
        } else if cut.boundary_is_closed {
            0
        } else {
            -1
        };

        if after >= 0 {
            let ordinary = cut.station_boundary[after as usize];
            let wall_end = cut.station_points[s][cut.station_points[s].len() - 1];

            if let Some((leaving, leaving_faces)) = try_the_takeover(
                mesh,
                graph,
                edge_at_pair,
                pool,
                is_a_feature_edge,
                side,
                cut,
                junction,
                junction.leaving_wall_end,
                ordinary,
                wall_end,
            )? {
                cut.station_points[s].push(leaving);
                cut.station_boundary.insert(after_point as usize, leaving);
                cut.station_faces
                    .insert(after_point as usize, leaving_faces);
            }
        }
    }

    Ok(())
}

/// Where a station's first point stands in the flattened boundary as it is
/// so far.
pub fn the_flattened_start_of(cut: &SmoothSideCut, station: i32) -> i32 {
    let mut at = 0;
    let mut i = 0;

    while i < station && i < cut.station_points.len() as i32 {
        at += cut.station_points[i as usize].len() as i32;
        i += 1;
    }

    at
}

/// The point on a feature crease where the join from an ordinary station into a
/// CARRIED wall end must end, with the faces of that crease on this side - `None` where the walk from
/// the wall end arrives, or the end was never carried, and the ordinary join stands.
///
/// THE WALK IS LAID OUT OF THE CARRIED FACET, not in from the ordinary station: the wall end provably
/// lies in the facet it was carried onto, and the way to the ordinary station is laid IN that facet's
/// plane. Read the other way round the line generally leaves the surface - at the pierced face's
/// first junction the direction between them is very nearly the wall's own NORMAL.
#[allow(clippy::too_many_arguments)]
pub fn try_the_takeover(
    mesh: &Mesh,
    graph: &MeshEdgeGraph,
    edge_at_pair: &EdgeAtPair,
    pool: &mut SectionPool,
    is_a_feature_edge: Option<&dyn Fn(i32) -> bool>,
    side: &IndexSet<i32>,
    cut: &mut SmoothSideCut,
    junction: &JunctionSection,
    sample: i32,
    ordinary: i32,
    wall_end: i32,
) -> Result<Option<(i32, Vec<i32>)>, SmoothSideFailure> {
    if sample as usize >= junction.carried_onto.len()
        || junction.carried_onto[sample as usize] < 0
        || wall_end != junction.samples[sample as usize]
    {
        return Ok(None);
    }

    let onto = junction.carried_onto[sample as usize];
    let from = pool.position_of(ordinary);
    let to = pool.position_of(wall_end);
    let reach = (to - from).length();

    if reach <= 0.0 {
        return Ok(None);
    }

    let normal = normal_of(mesh, onto).get_normal();
    let towards_the_station = from - to;
    let in_the_plane = towards_the_station - (normal * towards_the_station.dot(normal));

    let walk = match try_walk(
        mesh,
        graph,
        edge_at_pair,
        to,
        onto,
        in_the_plane,
        reach,
        is_a_feature_edge,
    ) {
        // The walk crosses the whole way without meeting a crease, so nothing stands between the
        // wall end and the ordinary station and the ordinary join is the whole of it.
        Ok(_) => return Ok(None),
        Err(walk) => walk,
    };

    if walk.refusal != SurfaceWalkRefusal::CreaseReached
        || walk.end_edge < 0
        || is_the_same_point(walk.end, from, reach)
        || is_the_same_point(walk.end, to, reach)
    {
        return Ok(None);
    }

    let takeover = pool.add(walk.end)?;
    let takeover_faces = the_faces_on(graph, walk.end_edge, side);
    cut.places.insert(
        takeover,
        SmoothSidePlace::new(
            -1,
            walk.end_edge,
            parameter_on(mesh, graph, walk.end_edge, walk.end),
            walk.end_vertex,
        ),
    );

    Ok(Some((takeover, takeover_faces)))
}
