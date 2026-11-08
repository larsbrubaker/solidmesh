//! THE LOFT'S JUNCTION DUTY - a junction rung stands LONGER
//! than the strip's other rung, and the ladder (`section_loft::try_emit`) closes that difference by
//! FANNING the extra samples onto the far row's wall end, never by resampling the rung; the loft
//! reads the seam's crossings by STATION (`section_loft::crossings_on`); and an absorbed gap row
//! (`SectionAbsorbedSplits`) carries a junction's takeover either way round.
//!
//! The junction is built on the pierced face's own geometry (`inner_face_test_mesh`) through
//! `section_junction_build::try_build`, and its section is lofted against the junction's own first
//! `steps + 1` samples carried along the chain edge.

use std::collections::HashSet;
use std::rc::Rc;

use solidmesh::bevel::sections::section_junction_build;
use solidmesh::bevel::sections::section_loft;
use solidmesh::bevel::sections::section_smooth_seam::SectionSmoothSeam;
use solidmesh::bevel::sections::section_smooth_side::SmoothSideCut;
use solidmesh::bevel::sections::section_split_blend::SplitPlacement;
use solidmesh::bevel::sections::{
    BevelSection, JunctionSection, SectionAbsorbedSplits, SectionPool, SectionRun, SectionSide,
};
use solidmesh::topology::{ChainEdge, FeatureEdgeSet};
use solidmesh_math::Vector3;

use crate::inner_face_test_mesh;

const RADIUS: f64 = 0.5;

const STEPS: i32 = 8;

/// The segments the synthetic gap rows are drawn in - the gap path wants at least two.
const GAP_STEPS: i32 = 4;

type Faces = Vec<(i32, i32, i32)>;

/// The extra samples close the ladder: every directed edge of the loft stands at most once, the
/// ones standing alone are exactly the strip's own rim, and the tail is the fan the spec names.
#[test]
fn the_junctions_tail_is_fanned_onto_the_far_rows_wall_end() {
    let (junction, mut pool, arriving_edge, leaving_edge) = the_junction_at_the_mouth();

    // Whichever of the two sections carries the samples PAST the near wall end is the long row;
    // which one that is depends on which way round the chain walks this station.
    let leaving_is_long =
        junction.leaving_section.segments() > junction.arriving_section.segments();
    let long_row = if leaving_is_long {
        junction.leaving_section.clone()
    } else {
        junction.arriving_section.clone()
    };
    let strip = if leaving_is_long {
        leaving_edge
    } else {
        arriving_edge
    };
    let away = if leaving_is_long {
        strip.direction().get_normal()
    } else {
        -strip.direction().get_normal()
    };

    assert!(
        long_row.segments() > STEPS,
        "one of the junction's two rungs is the other PLUS the samples past P1"
    );

    let short_row = Rc::new(the_neighbour_carried_along(
        &long_row,
        away,
        strip.to_vertex(),
        &mut pool,
        2.0,
    ));
    let run = a_run_of(long_row.clone(), short_row.clone(), strip);
    let mut faces = Faces::new();

    if let Err(failure) = section_loft::try_emit(&run, &pool, &mut faces, None) {
        panic!(
            "a junction rung is lofted, not refused - instead {:?}",
            failure.code()
        );
    }

    let apex = short_row.profile[short_row.profile.len() - 1];

    // The tail: one triangle per extra sample, each on the two samples and the far row's wall end.
    for k in (STEPS as usize)..(long_row.profile.len() - 1) {
        let wanted: HashSet<i32> = [long_row.profile[k], long_row.profile[k + 1], apex]
            .into_iter()
            .collect();

        let count = faces
            .iter()
            .filter(|face| {
                let corners: HashSet<i32> = [face.0, face.1, face.2].into_iter().collect();
                corners == wanted
            })
            .count();

        assert_eq!(
            count, 1,
            "sample {k} of the junction rung is fanned onto the far wall end once"
        );
    }

    assert_eq!(
        faces.len() as i32,
        (2 * STEPS) + (long_row.segments() - STEPS),
        "the ladder is the equal rungs' quads and one triangle per extra sample"
    );

    for face in &faces {
        let zero = pool.position_of(face.0);

        assert!(
            (pool.position_of(face.1) - zero)
                .cross(pool.position_of(face.2) - zero)
                .length()
                > 1e-12,
            "no triangle of the loft is degenerate"
        );
    }

    // Closure: every directed edge stands at most once, and the ones with no opposite are exactly
    // the strip's own rim - the long row, the chord onto the apex, the short row back and the flat
    // ends.
    let directed = directed_edges(&faces);

    let distinct: HashSet<(i32, i32)> = directed.iter().copied().collect();
    assert_eq!(
        distinct.len(),
        directed.len(),
        "no directed edge of the loft is laid twice the same way round"
    );

    let mut alone: Vec<(i32, i32)> = directed
        .iter()
        .filter(|edge| !distinct.contains(&(edge.1, edge.0)))
        .map(|edge| undirected(edge.0, edge.1))
        .collect();
    alone.sort();

    let mut rim = Vec::new();

    for k in 0..long_row.profile.len() - 1 {
        rim.push(undirected(long_row.profile[k], long_row.profile[k + 1]));
    }

    for k in 0..STEPS as usize {
        rim.push(undirected(short_row.profile[k], short_row.profile[k + 1]));
    }

    rim.push(undirected(
        long_row.profile[long_row.profile.len() - 1],
        apex,
    ));
    rim.push(undirected(long_row.profile[0], short_row.profile[0]));
    rim.sort();

    assert_eq!(
        alone, rim,
        "every edge inside the loft closes against its opposite, and only the rim stands alone"
    );
}

/// A pair of ORDINARY equal rungs is lofted exactly as it was before the junction duty: the same
/// two triangles per quad and nothing else - the tail fan is a no-op where no rung stands long.
#[test]
fn an_ordinary_pair_of_equal_rungs_is_lofted_as_it_was() {
    let (junction, mut pool, _, leaving_edge) = the_junction_at_the_mouth();
    let away = leaving_edge.direction().get_normal();
    let row = if junction.leaving_section.segments() <= STEPS {
        junction.leaving_section.clone()
    } else {
        junction.arriving_section.clone()
    };
    let first = Rc::new(the_neighbour_carried_along(
        &row,
        away,
        leaving_edge.to_vertex(),
        &mut pool,
        1.0,
    ));
    let second = Rc::new(the_neighbour_carried_along(
        &row,
        away,
        leaving_edge.to_vertex(),
        &mut pool,
        3.0,
    ));
    let run = a_run_of(first.clone(), second.clone(), leaving_edge);
    let mut faces = Faces::new();

    if let Err(failure) = section_loft::try_emit(&run, &pool, &mut faces, None) {
        panic!(
            "an ordinary ladder is unchanged - instead {:?}",
            failure.code()
        );
    }

    assert_eq!(
        faces.len() as i32,
        2 * STEPS,
        "two triangles a quad and no tail, which is what the ladder laid before"
    );

    for face in &faces {
        for corner in [face.0, face.1, face.2] {
            assert!(
                first.profile.contains(&corner) || second.profile.contains(&corner),
                "every corner the ordinary ladder lays is a sample of its own two rows"
            );
        }
    }
}

/// THE LOFT READS THE SEAM'S CROSSINGS BY STATION. A junction on the smooth side hands THREE
/// boundary points (P1, F1, P2) where an ordinary station hands one, so the flattened join index
/// and the station index part company at it: the row between stations 1 and 2 must stitch through
/// the join out of P1 - station 1's LAST point - and station 0's row through the join INTO P1,
/// station 1's first.
#[test]
fn the_rows_crossings_are_read_by_station_and_not_by_flattened_join() {
    // Stations 0 and 2 are ordinary; station 1 is a junction handing P1, F1, P2. The flattened
    // boundary is then 10, 20, 21, 22, 30 and its four joins carry one crossing apiece.
    let cut = a_cut_whose_station_one_is_a_junction();
    let seam = SectionSmoothSeam::new(-1, true, cut);

    assert_eq!(
        section_loft::crossings_on(Some(&seam), 0, 0, 1),
        Some(&vec![100]),
        "station 0's row runs to the junction's FIRST point, P1"
    );

    assert_eq!(
        section_loft::crossings_on(Some(&seam), 1, 0, 1),
        Some(&vec![103]),
        "station 1's row leaves the junction's LAST point, P2"
    );

    assert_eq!(
        seam.cut.internal_crossings_of(1),
        vec![101, 102],
        "P1 -> F1 -> P2 are the junction's own rim joins, laid by the ledger and not by a row"
    );

    assert!(
        seam.cut.internal_crossings_of(0).is_empty(),
        "an ordinary station has one point and so no internal join"
    );
}

/// An ORDINARY run reads exactly as it did: station i's row is the i'th join.
#[test]
fn an_ordinary_runs_rows_are_the_joins_they_always_were() {
    let mut cut = SmoothSideCut::default();

    for station in 0..4 {
        cut.station_points.push(vec![10 * station]);
        cut.station_boundary.push(10 * station);
        cut.crossings_after.push(vec![100 + station]);
    }

    let seam = SectionSmoothSeam::new(-1, true, cut);

    for station in 0..4 {
        assert_eq!(
            section_loft::crossings_on(Some(&seam), station, 0, 1),
            Some(&vec![100 + station])
        );
    }
}

/// THE GAP ROW CARRIES THE JUNCTION'S TAKEOVER. A junction station's group on the smooth side can
/// run PAST its wall end - the rim takes over from a feature crease at a minted point - and the
/// loft's row out of that station is the only thing that lays the stretch from the wall end to
/// that takeover. A gap row that stops at the wall end leaves the little triangle between the wall
/// end, the takeover and the next station's boundary laid by nothing, which is the pierced face's
/// seam A.
#[test]
fn an_absorbed_gap_row_runs_past_the_junctions_wall_end_to_its_takeover() {
    let mut pool = SectionPool::new(&[]);
    let sections: Vec<BevelSection> = (0..4)
        .map(|station| a_straight_section(&mut pool, station))
        .collect();

    // Station 1's group on the cut is [P1, F1, P2, C]: the junction's two wall ends with the fold
    // between them, and then C, the point the rim takes over at past the carried wall end. P2 is
    // the station's own boundary - the point the loft's row stands on - so C is the tail past it.
    let p2 = sections[1].left_boundary;
    let p1 = add(&mut pool, Vector3::new(1.0, 1.0, -0.5));
    let f1 = add(&mut pool, Vector3::new(1.0, 1.0, -0.25));
    let takeover = add(&mut pool, Vector3::new(1.4, 1.0, 0.0));

    let mut cut = SmoothSideCut::default();

    for (station, section) in sections.iter().enumerate() {
        let group = if station == 1 {
            vec![p1, f1, p2, takeover]
        } else {
            vec![section.left_boundary]
        };

        cut.station_boundary.extend(group.iter().copied());
        cut.station_points.push(group);
    }

    while cut.crossings_after.len() < cut.station_boundary.len() {
        cut.crossings_after.push(Vec::new());
    }

    let seam = SectionSmoothSeam::new(-1, true, cut);

    // Station 2 is the absorbed split, so the gap out of the junction at 1 runs 1 -> 3 and is
    // lofted through the gap path, not through the plain ladder.
    let unsplit = SplitPlacement::new(-1, -1, 0.0);

    let placed = [unsplit, unsplit, SplitPlacement::new(1, 3, 0.5), unsplit];

    let faces = loft_the_absorbed_gaps(&pool, &sections, &placed, &seam).1;
    let laid = directed_edges(&faces);
    let next = sections[2].left_boundary;

    assert_eq!(
        laid.iter()
            .filter(|&&edge| edge == (p2, takeover) || edge == (takeover, p2))
            .count(),
        1,
        "the row lays the stretch from the carried wall end to the takeover exactly once"
    );

    assert_eq!(
        laid.iter()
            .filter(|&&edge| edge == (takeover, next) || edge == (next, takeover))
            .count(),
        1,
        "and then runs on from the takeover to the next boundary point, not from P2"
    );

    assert!(
        !laid
            .iter()
            .any(|&edge| edge == (p2, next) || edge == (next, p2)),
        "the row no longer spans the takeover, which is what left seam A's triangle open"
    );
}

/// THE MIRROR: A GAP ENDING AT A JUNCTION CARRIES ITS TAKEOVER TOO. Where the gap ENDS at the
/// junction the cut walks the station's group the other way round - the rim's takeover is the
/// group's FIRST point, BEFORE the wall end the row's quad stands on - so the row's own polyline
/// cannot carry it, and the stretch from the previous station's boundary round the group's head to
/// the row's end is laid by nothing. That is the pierced face's seam at its SECOND junction.
#[test]
fn an_absorbed_gap_ending_at_a_junction_fans_its_takeover() {
    let mut pool = SectionPool::new(&[]);
    let sections: Vec<BevelSection> = (0..4)
        .map(|station| a_straight_section(&mut pool, station))
        .collect();

    // Station 2's group is [C, P1, F1, P2] - the takeover FIRST, then the junction's rim, ending
    // at the station's own boundary point, which is the point the gap's row stands on.
    let p2 = sections[2].left_boundary;
    let takeover = add(&mut pool, Vector3::new(1.6, 1.0, 0.0));
    let p1 = add(&mut pool, Vector3::new(2.0, 1.0, -0.5));
    let f1 = add(&mut pool, Vector3::new(2.0, 1.0, -0.25));

    let mut cut = SmoothSideCut::default();

    for (station, section) in sections.iter().enumerate() {
        let group = if station == 2 {
            vec![takeover, p1, f1, p2]
        } else {
            vec![section.left_boundary]
        };

        cut.station_boundary.extend(group.iter().copied());
        cut.station_points.push(group);
    }

    while cut.crossings_after.len() < cut.station_boundary.len() {
        cut.crossings_after.push(Vec::new());
    }

    let seam = SectionSmoothSeam::new(-1, true, cut);

    // Station 1 is the absorbed split, so the gap runs 0 -> 2 and ARRIVES at the junction.
    let unsplit = SplitPlacement::new(-1, -1, 0.0);

    let placed = [unsplit, SplitPlacement::new(0, 2, 0.5), unsplit, unsplit];

    let (run, faces) = loft_the_absorbed_gaps(&pool, &sections, &placed, &seam);
    let laid = directed_edges(&faces);

    // The absorbed split's own boundary point is where the row's polyline stands before the
    // junction.
    let row = run
        .absorbed
        .as_ref()
        .expect("the run carries its absorbed splits")
        .left_of(0);
    let previous = row[row.len() - 1].pool_index;

    for (one, other) in [(previous, takeover), (takeover, p1), (p1, f1), (f1, p2)] {
        assert_eq!(
            laid.iter()
                .filter(|&&edge| edge == (one, other) || edge == (other, one))
                .count(),
            1,
            "every stretch of the arriving junction's group head is laid exactly once"
        );
    }

    // The row's own chord is then an INSIDE edge of the loft: the fan closes against it.
    assert_eq!(
        laid.iter()
            .filter(|&&edge| edge == (previous, p2) || edge == (p2, previous))
            .count(),
        2,
        "the chord the row laid is closed by the fan, once each way"
    );
}

/// The absorbed run of the four straight sections lofted against the seam - the shared middle of
/// the two takeover tests, each of which asserts `absorbed` is read and the loft lays.
fn loft_the_absorbed_gaps(
    pool: &SectionPool,
    sections: &[BevelSection],
    placed: &[SplitPlacement],
    seam: &SectionSmoothSeam,
) -> (SectionRun, Faces) {
    let absorbed = SectionAbsorbedSplits::from(sections, placed, false, None, None);

    assert!(
        absorbed.is_some(),
        "one split station of four leaves three gap ends"
    );

    let run = SectionRun::new(
        sections.iter().cloned().map(Rc::new).collect(),
        false,
        vec![ChainEdge::default(); 3],
        None,
        None,
        absorbed,
        None,
    );
    let mut faces = Faces::new();

    if let Err(failure) = section_loft::try_emit(&run, pool, &mut faces, Some(seam)) {
        panic!("the absorbed gaps loft - instead {:?}", failure.code());
    }

    (run, faces)
}

fn directed_edges(faces: &Faces) -> Vec<(i32, i32)> {
    let mut directed = Vec::with_capacity(faces.len() * 3);

    for face in faces {
        directed.push((face.0, face.1));
        directed.push((face.1, face.2));
        directed.push((face.2, face.0));
    }

    directed
}

fn undirected(one: i32, other: i32) -> (i32, i32) {
    (one.min(other), one.max(other))
}

fn add(pool: &mut SectionPool, at: Vector3) -> i32 {
    pool.add(at).expect("a finite point")
}

/// The junction at the pierced face's mouth, built through the production rule on the production
/// fixture: the sharp turn `is_a_junction_station` owns at vertex 19, with the fold read off the
/// part as the one crease leaving the vertex that is not the chain.
fn the_junction_at_the_mouth() -> (JunctionSection, SectionPool, ChainEdge, ChainEdge) {
    let part = inner_face_test_mesh::the_part();
    let graph = part.get_mesh_edge_graph();
    let features = FeatureEdgeSet::create_with_graph(
        &part,
        &graph,
        inner_face_test_mesh::BEND_THRESHOLD_DEGREES,
    );
    let station = 19;

    let at_the_station: Vec<(usize, _)> = features
        .edges()
        .iter()
        .enumerate()
        .filter(|(_, edge)| edge.start_vertex == station || edge.end_vertex == station)
        .map(|(index, edge)| (index, *edge))
        .collect();

    // The operation's own reading of which mesh edges are features, which the fan walk needs.
    let feature_edges: HashSet<i32> = features
        .edges()
        .iter()
        .map(|one| one.mesh_edge_index)
        .collect();
    let is_a_feature_edge = |edge: i32| feature_edges.contains(&edge);

    let mut arriving = None;
    let mut leaving = None;

    for into in &at_the_station {
        for out_of in &at_the_station {
            if into.0 == out_of.0 {
                continue;
            }

            let one = ChainEdge::new(into.0 as i32, into.1, into.1.end_vertex != station);
            let other = ChainEdge::new(out_of.0 as i32, out_of.1, out_of.1.start_vertex != station);

            if section_junction_build::is_a_junction_station(
                &part,
                &graph,
                &one,
                &other,
                station,
                Some(&is_a_feature_edge),
                None,
            ) {
                arriving = Some(one);
                leaving = Some(other);
            }
        }
    }

    let (Some(arriving), Some(leaving)) = (arriving, leaving) else {
        panic!("no junction station reads at vertex {station}");
    };

    let mut pool = SectionPool::new(&part.vertices);

    let section = section_junction_build::try_build(
        &arriving,
        &leaving,
        the_fold_at(&features, &arriving, &leaving, station),
        &mut pool,
        RADIUS,
        STEPS,
        None,
    )
    .unwrap_or_else(|failure| panic!("the junction at {station} was refused: {failure:?}"));

    (section, pool, arriving, leaving)
}

/// The one crease leaving the station that is neither chain edge - the mouth's continuation down
/// the cube's -X wall - as a unit direction pointing away from the vertex.
fn the_fold_at(
    features: &FeatureEdgeSet,
    arriving: &ChainEdge,
    leaving: &ChainEdge,
    station: i32,
) -> Vector3 {
    let creases: Vec<_> = features
        .edges()
        .iter()
        .filter(|edge| {
            (edge.start_vertex == station || edge.end_vertex == station)
                && edge.start_vertex != arriving.from_vertex()
                && edge.end_vertex != arriving.from_vertex()
                && edge.start_vertex != leaving.to_vertex()
                && edge.end_vertex != leaving.to_vertex()
        })
        .collect();
    assert_eq!(creases.len(), 1, "one crease leaves the station");
    let crease = creases[0];

    let down = crease.direction.get_normal();

    if crease.start_vertex == station {
        down
    } else {
        -down
    }
}

/// An ORDINARY section standing where the chain would carry the junction's first `steps + 1`
/// samples along the leaving edge - the strip the loft pairs the junction rung with. Nothing of the
/// section rules is copied: the points are the junction's own, moved along the chain.
fn the_neighbour_carried_along(
    junction_row: &BevelSection,
    away: Vector3,
    chain_vertex: i32,
    pool: &mut SectionPool,
    distance: f64,
) -> BevelSection {
    let along = away * distance;
    let profile: Vec<i32> = (0..=STEPS as usize)
        .map(|k| {
            let at = pool.position_of(junction_row.profile[k]) + along;
            add(pool, at)
        })
        .collect();

    BevelSection::new(
        chain_vertex,
        junction_row.station + along,
        junction_row.tangent,
        profile[0],
        profile[profile.len() - 1],
        profile,
        junction_row.centre_from + along,
        junction_row.centre_to + along,
        junction_row.left_kind,
        junction_row.right_kind,
        -1,
        -1,
        junction_row.setback,
    )
    .expect("the profile ends on its boundaries")
}

/// A plain section standing at `station` along +X: a quarter arc from +Y round to +Z about the
/// chain, which is all the loft asks of a row - equal segments and a centre to read outward
/// against. Its LEFT boundary is the +Y end.
fn a_straight_section(pool: &mut SectionPool, station: i32) -> BevelSection {
    let centre = Vector3::new(f64::from(station), 0.0, 0.0);
    let profile: Vec<i32> = (0..=GAP_STEPS)
        .map(|k| {
            let angle = f64::from(k) * (std::f64::consts::PI / 2.0) / f64::from(GAP_STEPS);

            add(pool, centre + Vector3::new(0.0, angle.cos(), angle.sin()))
        })
        .collect();

    BevelSection::new(
        station,
        centre,
        Vector3::new(1.0, 0.0, 0.0),
        profile[0],
        profile[profile.len() - 1],
        profile,
        centre,
        centre,
        SectionSide::Offset,
        SectionSide::Offset,
        -1,
        -1,
        0.0,
    )
    .expect("the profile ends on its boundaries")
}

/// The two rungs as an OPEN run of one edge, which is the one ladder the loft has to lay.
fn a_run_of(start: Rc<BevelSection>, end: Rc<BevelSection>, edge: ChainEdge) -> SectionRun {
    SectionRun::new(vec![start, end], false, vec![edge], None, None, None, None)
}

/// Three stations whose middle one is a junction on the smooth side, with one crossing minted on
/// each of the four joins - the indices stand for pool indices and nothing here needs a mesh.
fn a_cut_whose_station_one_is_a_junction() -> SmoothSideCut {
    let mut cut = SmoothSideCut::default();

    for group in [vec![10], vec![20, 21, 22], vec![30]] {
        cut.station_boundary.extend(group.iter().copied());
        cut.station_points.push(group);
    }

    for join in 0..4 {
        cut.crossings_after.push(vec![100 + join]);
    }

    cut
}
