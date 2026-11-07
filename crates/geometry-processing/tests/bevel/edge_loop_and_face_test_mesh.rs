//! The half cylinder of "Edge Loop And Face Should Match.mcx", cut out of its own owner file - the
//! welded working mesh the bevel construction is handed (40 vertices, 76 triangles), in the space
//! the size is quoted in, with the file's three picks (the Test copy's one FaceGroup and the
//! Control copy's Edge plus TangentChain) as the anchored groups the app resolves them from.
//!
//! The part: a half cylinder radius 10 with its axis along y, flat on z = 0, spanning x -10..10,
//! y -20..0 and z 0..9.966 (the arc is inscribed at 19 facets). The rim every pick names is the
//! y = -20 END FACE's boundary: the straight diameter and the nineteen chords of the arc that close
//! back onto it. Vertex positions are the mesh's own floats written round-trip, so the copy is the
//! same mesh bit for bit. Used by `section_edge_loop_and_face_tests` and its neighbours.

#![allow(dead_code)]

use polygon_mesh::Mesh;
use vector_math::Vector3;

use geometry_processing::topology::{BevelEdgeGroup, BevelSelectionKind};

/// The size the owner file asks of all three of its rows.
pub const SIZE: f64 = 2.0;

pub const SEGMENTS: i32 = 8;

/// The operation's bend threshold, which every group is read at.
pub const BEND_THRESHOLD_DEGREES: f64 = 10.0;

/// The welded working mesh, 40 vertices and 76 triangles.
pub fn the_part() -> Mesh {
    Mesh::from_arrays(COORDINATES, TRIANGLES)
}

/// The Test copy's one pick - the y = -20 end face, as a face group off its anchors.
pub fn the_end_face() -> BevelEdgeGroup {
    BevelEdgeGroup {
        selection_kind: BevelSelectionKind::FaceGroup,
        curve_tolerance_degrees: 0.5124707395061022,
        flat_angle_degrees: BEND_THRESHOLD_DEGREES,
        anchor_midpoint: Vector3::new(0.0, -20.0, -8.74227794156468E-07),
        anchor_direction: Vector3::new(1.0, 0.0, 0.0),
        anchor_normal0: Vector3::new(0.0, -1.0, 0.0),
        anchor_plane_distance0: 20.0,
        anchor_normal1: Vector3::new(0.0, 4.3711389707823355E-08, -0.999999999999999),
        anchor_plane_distance1: 0.0,
        ..BevelEdgeGroup::default()
    }
}

/// The Control copy's first pick - the end face's straight diameter, as one
/// Edge.
pub fn the_straight_edge() -> BevelEdgeGroup {
    BevelEdgeGroup {
        selection_kind: BevelSelectionKind::Edge,
        curve_tolerance_degrees: 30.0,
        flat_angle_degrees: BEND_THRESHOLD_DEGREES,
        anchor_midpoint: Vector3::new(0.0, -20.0, -8.74227794156468E-07),
        anchor_direction: Vector3::new(1.0, 0.0, 0.0),
        anchor_normal0: Vector3::new(0.0, -1.0, 0.0),
        anchor_plane_distance0: 20.0,
        anchor_normal1: Vector3::new(0.0, 4.3711389707823355E-08, -0.999999999999999),
        anchor_plane_distance1: 0.0,
        ..BevelEdgeGroup::default()
    }
}

/// The Control copy's second pick - the arc of the same rim, as a TangentChain.
pub fn the_arc() -> BevelEdgeGroup {
    BevelEdgeGroup {
        selection_kind: BevelSelectionKind::TangentChain,
        curve_tolerance_degrees: 30.0,
        flat_angle_degrees: BEND_THRESHOLD_DEGREES,
        anchor_midpoint: Vector3::new(-9.66100025177002, -20.0, 2.4464991092681885),
        anchor_direction: Vector3::new(-0.2458108713144801, 0.0, -0.9693178093605915),
        anchor_normal0: Vector3::new(0.0, -1.0, 0.0),
        anchor_plane_distance0: 20.0,
        anchor_normal1: Vector3::new(
            -0.9693178093605914,
            -1.1721175816482915E-08,
            0.2458108713144801,
        ),
        anchor_plane_distance1: 9.965955912420668,
        ..BevelEdgeGroup::default()
    }
}

#[rustfmt::skip]
const COORDINATES: &[f64] = &[
    10.0, -20.0, -8.74227794156468E-07,
    -9.86400032043457, -20.0, 1.6459991931915283,
    -10.0, -20.0, -8.74227794156468E-07,
    -9.458000183105469, -20.0, 3.2469990253448486,
    -8.795000076293945, -20.0, 4.758998870849609,
    -7.890999794006348, -20.0, 6.141999244689941,
    -6.7729997634887695, -20.0, 7.356998920440674,
    -5.468999862670898, -20.0, 8.37199878692627,
    -4.017000198364258, -20.0, 9.157999038696289,
    -2.4549999237060547, -20.0, 9.693999290466309,
    -0.8259999752044678, -20.0, 9.965998649597168,
    0.8259999752044678, -20.0, 9.965998649597168,
    2.4549999237060547, -20.0, 9.693999290466309,
    4.017000198364258, -20.0, 9.157999038696289,
    5.468999862670898, -20.0, 8.37199878692627,
    6.7729997634887695, -20.0, 7.356998920440674,
    7.890999794006348, -20.0, 6.141999244689941,
    8.795000076293945, -20.0, 4.758998870849609,
    9.458000183105469, -20.0, 3.2469990253448486,
    9.86400032043457, -20.0, 1.6459991931915283,
    -9.86400032043457, -7.194894635631499E-08, 1.6460000276565552,
    -10.0, 0.0, 0.0,
    10.0, 0.0, 0.0,
    -9.458000183105469, -1.4193088304637058E-07, 3.246999979019165,
    -8.795000076293945, -2.0802249878215662E-07, 4.758999824523926,
    -7.890999794006348, -2.684753610537882E-07, 6.142000198364258,
    -6.7729997634887695, -3.215846788862109E-07, 7.35699987411499,
    -5.468999862670898, -3.6595173469322617E-07, 8.371999740600586,
    -4.017000198364258, -4.0030892023423803E-07, 9.157999992370605,
    -2.4549999237060547, -4.237382142946444E-07, 9.694000244140625,
    -0.8259999752044678, -4.3562769747040875E-07, 9.965999603271484,
    0.8259999752044678, -4.3562769747040875E-07, 9.965999603271484,
    2.4549999237060547, -4.237382142946444E-07, 9.694000244140625,
    4.017000198364258, -4.0030892023423803E-07, 9.157999992370605,
    5.468999862670898, -3.6595173469322617E-07, 8.371999740600586,
    6.7729997634887695, -3.215846788862109E-07, 7.35699987411499,
    7.890999794006348, -2.684753610537882E-07, 6.142000198364258,
    8.795000076293945, -2.0802249878215662E-07, 4.758999824523926,
    9.458000183105469, -1.4193088304637058E-07, 3.246999979019165,
    9.86400032043457, -7.194894635631499E-08, 1.6460000276565552,
];

#[rustfmt::skip]
const TRIANGLES: &[i32] = &[
    0, 1, 2,
    1, 0, 3,
    3, 0, 4,
    4, 0, 5,
    5, 0, 6,
    6, 0, 7,
    7, 0, 8,
    8, 0, 9,
    9, 0, 10,
    10, 0, 11,
    11, 0, 12,
    12, 0, 13,
    13, 0, 14,
    14, 0, 15,
    15, 0, 16,
    16, 0, 17,
    17, 0, 18,
    18, 0, 19,
    20, 21, 2,
    20, 2, 1,
    21, 22, 0,
    21, 0, 2,
    23, 20, 1,
    23, 1, 3,
    24, 23, 3,
    24, 3, 4,
    25, 24, 4,
    25, 4, 5,
    26, 25, 5,
    26, 5, 6,
    27, 26, 6,
    27, 6, 7,
    28, 27, 7,
    28, 7, 8,
    29, 28, 8,
    29, 8, 9,
    30, 29, 9,
    30, 9, 10,
    31, 30, 10,
    31, 10, 11,
    32, 31, 11,
    32, 11, 12,
    33, 32, 12,
    33, 12, 13,
    34, 33, 13,
    34, 13, 14,
    35, 34, 14,
    35, 14, 15,
    36, 35, 15,
    36, 15, 16,
    37, 36, 16,
    37, 16, 17,
    38, 37, 17,
    38, 17, 18,
    22, 39, 19,
    22, 19, 0,
    39, 38, 18,
    39, 18, 19,
    21, 20, 22,
    23, 22, 20,
    24, 22, 23,
    25, 22, 24,
    26, 22, 25,
    27, 22, 26,
    28, 22, 27,
    29, 22, 28,
    30, 22, 29,
    31, 22, 30,
    32, 22, 31,
    33, 22, 32,
    34, 22, 33,
    35, 22, 34,
    36, 22, 35,
    37, 22, 36,
    38, 22, 37,
    39, 22, 38,
];
