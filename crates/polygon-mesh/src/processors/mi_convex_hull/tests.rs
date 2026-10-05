//! Rust-only checks that the MIConvexHull port emits the same faces, in the same order and
//! winding, as NuGet `MIConvexHull 1.1.19.1019` (`ConvexHull.Create<TVertex, TFace>(points, .01)`,
//! the call `MeshConvexHull` makes). Expected strings were printed by a .NET 10 console app over
//! the same point sets; each face is its vertices' indices into the input.

use super::create;

fn faces_string(points: &[[f64; 3]]) -> String {
    match create(points, 0.01) {
        None => "null".to_string(),
        Some(faces) => faces
            .iter()
            .map(|f| format!("{},{},{}", f[0], f[1], f[2]))
            .collect::<Vec<_>>()
            .join(" "),
    }
}

fn grid() -> Vec<[f64; 3]> {
    let mut grid = Vec::new();
    for x in 0..5 {
        for y in 0..5 {
            for z in 0..5 {
                grid.push([x as f64, y as f64, z as f64]);
            }
        }
    }
    grid
}

#[test]
fn rust_only_cube_corners_match_miconvexhull() {
    let mut cube = Vec::new();
    for z in [-5.0, 5.0] {
        for y in [-5.0, 5.0] {
            for x in [-5.0, 5.0] {
                cube.push([x, y, z]);
            }
        }
    }
    assert_eq!(
        faces_string(&cube),
        "6,3,2 6,2,0 0,2,3 4,5,6 0,4,6 5,4,0 1,3,5 0,1,5 3,1,0 7,6,5 3,6,7 7,5,3"
    );
}

#[test]
fn rust_only_point_cloud_matches_miconvexhull() {
    let cloud: Vec<[f64; 3]> = (0..200i64)
        .map(|i| {
            [
                ((i * 7919) % 1000) as f64 / 37.0,
                ((i * 104729) % 997) as f64 / 41.0,
                ((i * 1299709) % 991) as f64 / 43.0,
            ]
        })
        .collect();
    assert_eq!(
        faces_string(&cloud),
        "135,158,181 181,158,113 39,113,158 45,135,181 181,113,45 45,86,135 86,45,0 173,62,25 \
         173,112,62 62,112,113 39,62,113 62,39,25 0,2,137 137,136,0 25,115,136 25,136,173 \
         2,119,199 199,119,0 119,2,0 74,115,117 158,117,39 117,115,39 115,74,23 158,135,111 \
         86,111,135 111,86,0 112,198,199 112,173,198 198,173,136 199,198,186 136,186,198 \
         136,137,186 115,76,39 25,76,115 25,39,76 113,112,90 113,90,45 45,90,199 41,45,199 \
         0,41,199 45,41,0 199,162,2 162,137,2 199,186,162 137,162,186 117,158,197 74,117,197 \
         197,158,111 23,74,185 185,74,0 0,23,185 136,115,68 23,68,115 136,68,0 68,23,0 0,37,111 \
         74,37,0 197,111,37 37,74,197 88,112,199 199,90,88 88,90,112"
    );
}

#[test]
fn rust_only_coplanar_grid_matches_miconvexhull() {
    assert_eq!(
        faces_string(&grid()),
        "120,104,100 120,100,10 104,120,124 124,120,24 24,104,124 104,24,4 24,10,4 4,100,104 \
         20,24,120 10,20,120 24,20,10 0,10,100 100,4,0 0,4,10"
    );
}

#[test]
fn rust_only_jitter_within_tolerance_matches_miconvexhull() {
    let jitter: Vec<[f64; 3]> = grid()
        .iter()
        .enumerate()
        .map(|(i, p)| {
            [
                p[0] + 0.001 * ((i * 13) % 7) as f64,
                p[1] + 0.002 * ((i * 5) % 3) as f64,
                p[2],
            ]
        })
        .collect();
    assert_eq!(
        faces_string(&jitter),
        "104,24,4 24,5,4 104,120,124 124,120,24 24,104,124 20,24,120 5,20,120 24,20,5 \
         120,104,100 120,100,5 4,100,104 0,5,100 100,4,0 0,4,5"
    );
}

#[test]
fn rust_only_flat_points_have_no_hull() {
    let flat: Vec<[f64; 3]> = (0..20)
        .map(|i| [(i % 5) as f64, (i / 5) as f64, 0.0])
        .collect();
    assert_eq!(faces_string(&flat), "null");
}

#[test]
fn rust_only_padding_with_no_candidate_has_no_hull() {
    let points = [
        [0.0, 0.0, 0.0],
        [1.0, 1.0, 1.0],
        [0.5, 0.2, 0.5],
        [0.3, 0.5, 0.5],
    ];
    assert_eq!(faces_string(&points), "null");
}
