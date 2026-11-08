//! A rail edge that crosses the inside of a chain's turn along its middle is given a station at
//! the jump, standing for every parameter the jump skips (`rail_stations::stations_at_jumps`).

use std::collections::HashMap;

use solidmesh::bevel::level_line::{rail_stations, ChainField, LevelLineMesh};
use solidmesh_math::Vector3;

use crate::chain_field_fixtures::open_chain;

/// The L chain (0,0) -> (4,0) -> (4,3) at setback 1, and one rail edge from (2,1) to (3,2) on the
/// inside of its turn: the edge reads the first leg up to the bisector at (2.5, 1.5), parameter
/// 2.5, and the second leg from there, parameter 5.5. Without a station there, a vertex of the
/// other rail reading 2.5 to 5.5 pairs with an end of this edge that reads none of them, and the
/// zip's arcs into that end cross; with it, the station stands for the whole jump.
#[test]
fn a_jump_along_a_rail_edge_gets_a_station_standing_for_the_whole_jump() {
    let chain = open_chain(
        vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(4.0, 0.0, 0.0),
            Vector3::new(4.0, 3.0, 0.0),
        ],
        vec![1.0, 1.0, 1.0],
    );
    let mut mesh = LevelLineMesh::from_triangles(
        &[
            Vector3::new(2.0, 1.0, 0.0),
            Vector3::new(3.0, 2.0, 0.0),
            Vector3::new(2.0, 2.0, 0.0),
        ],
        &[(0, 1, 2)],
        None,
    )
    .expect("one triangle");
    let tie = 1e-9;
    let mut vertices = vec![0, 1];
    let mut parameters = vec![2.0, 6.0];
    let mut splits = HashMap::new();
    rail_stations::stations_at_jumps(
        &mut mesh,
        &chain,
        &mut vertices,
        &mut parameters,
        &[2.6],
        tie,
        0.0,
        &mut splits,
    )
    .expect("no budget");

    assert_eq!(vertices.len(), 3, "one station, at the one jump");
    assert_eq!(
        rail_stations::pieces(0, 1, &splits),
        vec![(0, vertices[1]), (vertices[1], 1)]
    );
    let station = mesh.position(vertices[1]);
    assert!(
        (station - Vector3::new(2.5, 1.5, 0.0)).length() <= 1e-6,
        "{station:?}"
    );
    assert!((parameters[1] - 2.5).abs() <= 1e-6, "{parameters:?}");

    let spans = rail_stations::tie_spans(&mesh, &chain, &vertices, &parameters, tie, 0.0)
        .expect("no budget");
    assert!((spans[1].0 - 2.5).abs() <= 1e-6, "{spans:?}");
    assert!((spans[1].1 - 5.5).abs() <= 1e-4, "{spans:?}");
    assert_eq!(spans[0], (2.0, 2.0));
    assert_eq!(spans[2], (6.0, 6.0));

    // Read again, the rail has no jump left in an edge's middle.
    rail_stations::stations_at_jumps(
        &mut mesh,
        &chain,
        &mut vertices,
        &mut parameters,
        &[2.6, 5.4],
        tie,
        0.0,
        &mut splits,
    )
    .expect("no budget");
    assert_eq!(vertices.len(), 3);
}

/// A jump no reading of the other rail falls into leaves the edge as it is: nothing would pair
/// with an end that reads none of the jump's parameters.
#[test]
fn a_jump_nothing_reads_into_gets_no_station() {
    let chain = open_chain(
        vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(4.0, 0.0, 0.0),
            Vector3::new(4.0, 3.0, 0.0),
        ],
        vec![1.0, 1.0, 1.0],
    );
    let mut mesh = LevelLineMesh::from_triangles(
        &[
            Vector3::new(2.0, 1.0, 0.0),
            Vector3::new(3.0, 2.0, 0.0),
            Vector3::new(2.0, 2.0, 0.0),
        ],
        &[(0, 1, 2)],
        None,
    )
    .expect("one triangle");
    let mut vertices = vec![0, 1];
    let mut parameters = vec![2.0, 6.0];
    let mut splits = HashMap::new();
    rail_stations::stations_at_jumps(
        &mut mesh,
        &chain,
        &mut vertices,
        &mut parameters,
        &[2.2, 5.8],
        1e-9,
        0.0,
        &mut splits,
    )
    .expect("no budget");
    assert_eq!(vertices, vec![0, 1]);
    assert!(splits.is_empty());
}

/// A chain turning twice in quick succession, (0,0) -> (4,0) -> (5,1) -> (5,4) at setback 1, and
/// one rail edge from (3,1) to (4.5,2.5) crossing both turns' bisectors: the reading jumps from
/// 3.414 to 4.586 at (3.414, 1.414) and from 4.828 to 6 at (3.586, 1.586). The other rail reads
/// into the second jump only (5.5) and between the two (4.7, which a point of the edge reads), so
/// the edge gets the one station, at the second jump.
#[test]
fn of_two_jumps_along_an_edge_only_the_one_read_into_gets_a_station() {
    let chain = open_chain(
        vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(4.0, 0.0, 0.0),
            Vector3::new(5.0, 1.0, 0.0),
            Vector3::new(5.0, 4.0, 0.0),
        ],
        vec![1.0; 4],
    );
    let mut mesh = LevelLineMesh::from_triangles(
        &[
            Vector3::new(3.0, 1.0, 0.0),
            Vector3::new(4.5, 2.5, 0.0),
            Vector3::new(3.0, 2.5, 0.0),
        ],
        &[(0, 1, 2)],
        None,
    )
    .expect("one triangle");
    let tie = 1e-9;
    let mut vertices = vec![0, 1];
    let mut parameters = vec![3.0, 5.0 + 2.0_f64.sqrt() + 1.5];
    let mut splits = HashMap::new();
    rail_stations::stations_at_jumps(
        &mut mesh,
        &chain,
        &mut vertices,
        &mut parameters,
        &[4.7, 5.5],
        tie,
        0.0,
        &mut splits,
    )
    .expect("no budget");

    assert_eq!(vertices.len(), 3, "{parameters:?}");
    let second = 3.0 + (2.0 - 2.0_f64.sqrt());
    let station = mesh.position(vertices[1]);
    assert!(
        (station - Vector3::new(second, second - 2.0, 0.0)).length() <= 1e-6,
        "{station:?}"
    );
    let spans = rail_stations::tie_spans(&mesh, &chain, &vertices, &parameters, tie, 0.0)
        .expect("no budget");
    assert!(spans[1].0 < 5.5 && spans[1].1 > 5.5, "{spans:?}");
    assert!((spans[1].1 - 6.0).abs() <= 1e-4, "{spans:?}");
}

/// A closed chain round the square (0,0)..(4,4) starting mid-side at (2,0), at setback 1, and a
/// rail inside it whose WRAP edge, from (1.5,1) back to (3,2), crosses the chain's start and then
/// the bisector at the corner (4,0): the reading there jumps from 0.4 to 3.6, a lap on from 16.4
/// to 19.6. The other rail reads 2 inside that jump, so the wrap edge gets a station at
/// (2.4, 1.6), and since its foot lies past the chain's start the rail is led by it, at 0.4.
#[test]
fn a_station_on_the_wrap_edge_past_the_chains_start_leads_the_rail() {
    let chain = ChainField::from_polyline(
        &[
            Vector3::new(2.0, 0.0, 0.0),
            Vector3::new(4.0, 0.0, 0.0),
            Vector3::new(4.0, 4.0, 0.0),
            Vector3::new(0.0, 4.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
        ],
        true,
        &[1.0; 5],
    )
    .expect("a closed square");
    assert_eq!(chain.length, 16.0);
    let mut mesh = LevelLineMesh::from_triangles(
        &[
            Vector3::new(3.0, 2.0, 0.0),
            Vector3::new(2.0, 3.2, 0.0),
            Vector3::new(1.5, 1.0, 0.0),
            Vector3::new(3.0, 1.0, 0.0),
        ],
        &[(0, 1, 2), (2, 3, 0)],
        None,
    )
    .expect("two triangles");
    let tie = 1e-9 * chain.length;
    let mut vertices = vec![0, 1, 2];
    let mut parameters = vec![4.0, 8.0, 15.5];
    let mut splits = HashMap::new();
    rail_stations::stations_at_jumps(
        &mut mesh,
        &chain,
        &mut vertices,
        &mut parameters,
        &[-14.0, 2.0, 18.0],
        tie,
        chain.length,
        &mut splits,
    )
    .expect("no budget");

    assert_eq!(vertices.len(), 4, "{parameters:?}");
    assert_eq!(&vertices[1..], &[0, 1, 2]);
    let station = mesh.position(vertices[0]);
    assert!(
        (station - Vector3::new(2.4, 1.6, 0.0)).length() <= 1e-6,
        "{station:?}"
    );
    assert!((parameters[0] - 0.4).abs() <= 1e-6, "{parameters:?}");
    assert_eq!(&parameters[1..], &[4.0, 8.0, 15.5]);
    assert_eq!(
        rail_stations::pieces(2, 0, &splits),
        vec![(2, vertices[0]), (vertices[0], 0)]
    );
}
