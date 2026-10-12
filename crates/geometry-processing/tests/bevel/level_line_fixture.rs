//! Port of the test fixture loader `LevelLine/LevelLineFixture.cs`: a captured level-line case -
//! positions, triangles, the picked chains, the feature edges and the band's size - read from the
//! C# fixture files (copied into `tests/bevel/level_line_fixtures/`).

#![allow(dead_code)]

use std::collections::HashSet;

use vector_math::Vector3;

/// One loaded fixture.
pub struct LevelLineFixture {
    pub positions: Vec<Vector3>,
    pub indices: Vec<(i32, i32, i32)>,
    pub chains: Vec<Vec<i32>>,
    pub closed: Vec<bool>,
    pub size: f64,
    pub chamfer: bool,
    pub segments: i32,
    pub known_room: Option<f64>,
    pub seam_epsilon: f64,
    pub sharp_turn_degrees: f64,
    features: HashSet<(i32, i32)>,
}

impl LevelLineFixture {
    /// C# `IsFeatureEdge`: whether the fixture lists the edge `a`-`b` (either way round).
    pub fn is_feature_edge(&self, a: i32, b: i32) -> bool {
        self.features.contains(&if a < b { (a, b) } else { (b, a) })
    }

    /// C# `Load`: the named fixture.
    pub fn load(name: &str) -> LevelLineFixture {
        let path = format!(
            "{}/tests/bevel/level_line_fixtures/{name}.txt",
            env!("CARGO_MANIFEST_DIR")
        );
        let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));

        let mut fixture = LevelLineFixture {
            positions: Vec::new(),
            indices: Vec::new(),
            chains: Vec::new(),
            closed: Vec::new(),
            size: 0.0,
            chamfer: false,
            segments: 0,
            known_room: None,
            seam_epsilon: 0.0,
            sharp_turn_degrees: f64::INFINITY,
            features: HashSet::new(),
        };

        let number = |s: &str| {
            s.parse::<f64>()
                .unwrap_or_else(|e| panic!("{path}: {s}: {e}"))
        };
        let index = |s: &str| {
            s.parse::<i32>()
                .unwrap_or_else(|e| panic!("{path}: {s}: {e}"))
        };

        for line in text.lines() {
            if line.is_empty() || line.starts_with('#') {
                continue;
            }

            let parts: Vec<&str> = line.split(' ').collect();

            match parts[0] {
                "size" => fixture.size = number(parts[1]),
                "room" => fixture.known_room = Some(number(parts[1])),
                "seam" => fixture.seam_epsilon = number(parts[1]),
                "turn" => fixture.sharp_turn_degrees = number(parts[1]),
                "profile" => fixture.chamfer = parts[1] == "chamfer",
                "segments" => fixture.segments = index(parts[1]),
                "v" => fixture.positions.push(Vector3::new(
                    number(parts[1]),
                    number(parts[2]),
                    number(parts[3]),
                )),
                "f" => fixture
                    .indices
                    .push((index(parts[1]), index(parts[2]), index(parts[3]))),
                "chain" | "openchain" => {
                    fixture
                        .chains
                        .push(parts[1..].iter().map(|s| index(s)).collect());
                    fixture.closed.push(parts[0] == "chain");
                }
                "e" => {
                    fixture.features.insert((index(parts[1]), index(parts[2])));
                }
                other => panic!("{path}: what is \"{other}\"?"),
            }
        }

        fixture
    }
}

/// Every fixture in the fixture folder, by name, in ordinal order (C#'s
/// `Directory.GetFiles(directory, "*.txt").OrderBy(p => p, StringComparer.Ordinal)` in the
/// measurements that sweep it), less `top-center-face-control-nose`: a control mesh of bare
/// triangles in another format, which C#'s sweeps read as a fixture and throw on
/// (`docs/CSHARP_DIVERGENCES.md` 5015).
pub fn fixture_names() -> Vec<String> {
    let directory = format!(
        "{}/tests/bevel/level_line_fixtures",
        env!("CARGO_MANIFEST_DIR")
    );
    let mut names: Vec<String> = std::fs::read_dir(&directory)
        .unwrap_or_else(|e| panic!("{directory}: {e}"))
        .filter_map(|entry| entry.ok())
        .map(|entry| entry.file_name().to_string_lossy().into_owned())
        .filter(|name| name.ends_with(".txt") && name != "top-center-face-control-nose.txt")
        .collect();
    names.sort();
    names
        .into_iter()
        .map(|name| name.trim_end_matches(".txt").to_string())
        .collect()
}
