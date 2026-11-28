//! The vendored solidcore case files as data: their JSON read into steps and expectations, and the
//! check that each file is still the one `pin.json` records (byte for byte, by SHA-256, line ends
//! read as LF).
//!
//! A step is read in two passes: its `op` first, then, for an op this runner builds, the typed
//! fields. An op solidmesh has no counterpart for (`loft`, `sweep`, `pipe`, `sweep_face`,
//! `offset_faces`, `sheet_corner`, or anything new) is kept by name as [`Step::Unported`], so the
//! runner can say which op stops a case instead of refusing the file.
//!
//! A step that does not read, or a key in a case or its file outside `CASE_KEYS` and
//! `FILE_KEYS`, makes that case unreadable: a failing case that names what it could not read.
//! Expectations refuse unknown fields, so a key solidcore adds is never skipped without anyone
//! seeing it: a case whose `expect` does not read is a failing case ("its expectations are not
//! understood"), not an error for the whole run. The fields kept only for people are named and
//! ignored on purpose: each expectation's `formula`, the refusal's `error` wording and its fit's
//! `source`. A case's own `note` and `fixes` are not expectations and are not read.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use serde::de::IgnoredAny;
use serde::Deserialize;
use serde_json::Value;

use crate::lf_sha::lf_sha256;

/// A point or a direction in the case's world (mm).
pub type P3 = [f64; 3];

/// One piece of a profile in its plane's (u, v) coordinates; pieces that share end points close
/// into loops, and a profile may hold several (an outline and its holes).
#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Seg {
    Line {
        from: [f64; 2],
        to: [f64; 2],
    },
    Polygon {
        points: Vec<[f64; 2]>,
    },
    /// Counter-clockwise from `start_deg` to `end_deg`.
    Arc {
        center: [f64; 2],
        radius: f64,
        start_deg: f64,
        end_deg: f64,
    },
    Circle {
        center: [f64; 2],
        radius: f64,
    },
}

/// The plane a profile is drawn in, as a right-handed turn of XY.
#[derive(Debug, Clone, Copy, Default, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProfilePlane {
    /// u along X, v along Y, normal +Z.
    #[default]
    Xy,
    /// u along Y, v along Z, normal +X.
    Yz,
    /// u along Z, v along X, normal +Y.
    Zx,
    /// u along X, v along Z, normal -Y.
    Xz,
}

/// The axis a mirror plane is square to.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MirrorAxis {
    X,
    Y,
    Z,
}

/// A grid of positions `pitch[0] * i + pitch[1] * j`, `i` varying fastest.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Pattern {
    pub count: [usize; 2],
    pub pitch: [P3; 2],
}

fn z_axis() -> P3 {
    [0.0, 0.0, 1.0]
}

/// The ops this runner builds with solidmesh's own code. `id` names the body a step writes;
/// `a`, `b` and `body` name bodies earlier steps wrote.
#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "op", rename_all = "snake_case", deny_unknown_fields)]
pub enum Built {
    Box {
        id: String,
        origin: P3,
        size: P3,
    },
    Cylinder {
        id: String,
        origin: P3,
        radius: f64,
        height: f64,
        #[serde(default = "z_axis")]
        axis: P3,
    },
    Extrude {
        id: String,
        profile: Vec<Seg>,
        distance: f64,
        #[serde(default)]
        plane: ProfilePlane,
        #[serde(default)]
        origin: P3,
    },
    Revolve {
        id: String,
        profile: Vec<Seg>,
        #[serde(default)]
        plane: ProfilePlane,
        #[serde(default)]
        origin: P3,
        axis_point: P3,
        axis_dir: P3,
        angle_deg: f64,
    },
    Union {
        id: String,
        a: String,
        b: String,
    },
    Difference {
        id: String,
        a: String,
        b: String,
        #[serde(default)]
        pattern: Option<Pattern>,
    },
    Intersection {
        id: String,
        a: String,
        b: String,
    },
    Rotate {
        id: String,
        body: String,
        point: P3,
        axis: P3,
        degrees: f64,
    },
    Mirror {
        id: String,
        body: String,
        normal: MirrorAxis,
        #[serde(default)]
        at: f64,
    },
    Fillet {
        id: String,
        body: String,
        edges: Vec<P3>,
        radius: f64,
    },
    Chamfer {
        id: String,
        body: String,
        edges: Vec<P3>,
        distance: f64,
    },
}

/// One step of a case: built here, or an op solidmesh does not have.
#[derive(Debug, Clone)]
pub enum Step {
    Built(Built),
    Unported(String),
}

/// An analytic volume, mm^3, with the case's own absolute tolerance.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VolumeExpect {
    pub value: f64,
    pub tol: f64,
    /// How the value was derived, for people.
    #[serde(default, rename = "formula")]
    _formula: IgnoredAny,
}

/// The volume taken out of the earlier body `from`: its volume less the result's.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RemovedExpect {
    pub from: String,
    pub value: f64,
    pub tol: f64,
    /// How the value was derived, for people.
    #[serde(default, rename = "formula")]
    _formula: IgnoredAny,
}

/// The largest size that builds, which a refusal must report.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FitSizeExpect {
    pub value: f64,
    pub tol: f64,
    /// Where the value comes from, for people.
    #[serde(default, rename = "source")]
    _source: IgnoredAny,
}

/// The step that must be refused, and the largest size it must report where one is given.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RefusedExpect {
    pub step: String,
    #[serde(default)]
    pub fit_size: Option<FitSizeExpect>,
    /// solidcore's wording of the refusal; solidmesh words its own from codes.
    #[serde(default, rename = "error")]
    _error: IgnoredAny,
}

/// What the case's `result` must be; an absent field is not checked.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Expect {
    pub lumps: Option<usize>,
    pub manifold: Option<bool>,
    pub volume: Option<VolumeExpect>,
    pub removed: Option<RemovedExpect>,
    pub refused: Option<RefusedExpect>,
    /// A B-rep face count on cylinders, which a triangle mesh has no reading of.
    pub cylinder_faces_max: Option<usize>,
}

/// One case: `<fixture stem>/<name>`, its steps and what its result must be.
#[derive(Debug, Clone)]
pub struct Case {
    pub stem: String,
    pub name: String,
    pub steps: Vec<Step>,
    /// The expectations, or why they do not read.
    pub expect: Result<Expect, String>,
    /// Why the case cannot be run at all: a step that does not read, or a key in the case or
    /// its file this runner does not know. Such a case is a failing case, not an error for the
    /// whole run.
    pub unreadable: Option<String>,
}

impl Case {
    /// `<fixture stem>/<name>`, the case's key in the table and the known-failing list.
    pub fn key(&self) -> String {
        format!("{}/{}", self.stem, self.name)
    }

    /// Whether the case has a fillet or chamfer of its own.
    pub fn has_blend(&self) -> bool {
        self.steps.iter().any(|step| {
            matches!(
                step,
                Step::Built(Built::Fillet { .. }) | Step::Built(Built::Chamfer { .. })
            )
        })
    }

    /// The first op in the case solidmesh has no counterpart for.
    pub fn unported_op(&self) -> Option<&str> {
        self.steps.iter().find_map(|step| match step {
            Step::Unported(op) => Some(op.as_str()),
            Step::Built(_) => None,
        })
    }
}

/// The folder the runner reads from (`tests/solidcore_cases`, inside this crate).
pub fn cases_folder() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("solidcore_cases")
}

fn read_step(value: &Value) -> Result<Step, String> {
    let op = value
        .get("op")
        .and_then(Value::as_str)
        .ok_or_else(|| format!("a step with no op: {value}"))?;
    match op {
        "box" | "cylinder" | "extrude" | "revolve" | "union" | "difference" | "intersection"
        | "rotate" | "mirror" | "fillet" | "chamfer" => serde_json::from_value(value.clone())
            .map(Step::Built)
            .map_err(|error| format!("op {op}: {error}")),
        other => Ok(Step::Unported(other.to_string())),
    }
}

/// The keys a fixture file may carry; anything else is reported, never skipped.
pub const FILE_KEYS: [&str; 5] = ["issue", "pr", "test", "summary", "cases"];

/// The keys a case may carry (`note` and `fixes` are for people and not read).
pub const CASE_KEYS: [&str; 5] = ["name", "note", "fixes", "steps", "expect"];

/// The keys of `object` not in `known`, in the object's own order.
fn unknown_keys(object: &Value, known: &[&str]) -> Vec<String> {
    object
        .as_object()
        .map(|map| {
            map.keys()
                .filter(|key| !known.contains(&key.as_str()))
                .cloned()
                .collect()
        })
        .unwrap_or_default()
}

/// Every case of one fixture file, in file order.
pub fn read_fixture(path: &Path) -> Result<Vec<Case>, String> {
    let stem = path
        .file_stem()
        .and_then(|stem| stem.to_str())
        .ok_or_else(|| format!("{} has no file stem", path.display()))?
        .to_string();
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let file: Value = serde_json::from_str(&text).map_err(|e| format!("{stem}: {e}"))?;
    let cases = file
        .get("cases")
        .and_then(Value::as_array)
        .ok_or_else(|| format!("{stem} has no cases"))?;
    let file_unknown = unknown_keys(&file, &FILE_KEYS);
    let mut read = Vec::with_capacity(cases.len());
    for case in cases {
        let name = case
            .get("name")
            .and_then(Value::as_str)
            .ok_or_else(|| format!("{stem}: a case with no name"))?
            .to_string();
        let mut unreadable = None;
        let steps = match case.get("steps").and_then(Value::as_array) {
            None => {
                unreadable = Some("its steps are not understood: there are none".to_string());
                Vec::new()
            }
            Some(steps) => match steps.iter().map(read_step).collect::<Result<Vec<_>, _>>() {
                Ok(steps) => steps,
                Err(error) => {
                    unreadable = Some(format!("its steps are not understood: {error}"));
                    Vec::new()
                }
            },
        };
        let case_unknown = unknown_keys(case, &CASE_KEYS);
        if !case_unknown.is_empty() {
            unreadable = Some(format!(
                "it carries keys this runner does not know: {case_unknown:?} (known: {CASE_KEYS:?})"
            ));
        }
        if !file_unknown.is_empty() {
            unreadable = Some(format!(
                "its file carries keys this runner does not know: {file_unknown:?} (known: \
                 {FILE_KEYS:?})"
            ));
        }
        let expect = match case.get("expect") {
            Some(expect) => serde_json::from_value(expect.clone()).map_err(|e| format!("{e}")),
            None => Ok(Expect::default()),
        };
        read.push(Case {
            stem: stem.clone(),
            name,
            steps,
            expect,
            unreadable,
        });
    }
    Ok(read)
}

/// The fixture files `pin.json` lists, each checked against its recorded SHA-256, in name order.
/// A file on disk that the pin does not list, or one whose bytes moved, is an error: the cases
/// are solidcore's and are vendored unchanged.
pub fn pinned_fixtures() -> Result<Vec<PathBuf>, String> {
    let folder = cases_folder();
    let pin_text =
        std::fs::read_to_string(folder.join("pin.json")).map_err(|e| format!("pin.json: {e}"))?;
    let pin: Value = serde_json::from_str(&pin_text).map_err(|e| format!("pin.json: {e}"))?;
    let files = pin
        .get("files")
        .and_then(Value::as_object)
        .ok_or("pin.json has no files")?;
    let pinned: BTreeMap<String, String> = files
        .iter()
        .map(|(name, sha)| (name.clone(), sha.as_str().unwrap_or_default().to_string()))
        .collect();

    let fixtures = folder.join("fixtures");
    let mut on_disk: Vec<String> = std::fs::read_dir(&fixtures)
        .map_err(|e| format!("{}: {e}", fixtures.display()))?
        .filter_map(|entry| entry.ok())
        .filter_map(|entry| entry.file_name().to_str().map(str::to_string))
        .collect();
    on_disk.sort();
    let listed: Vec<String> = pinned.keys().cloned().collect();
    if on_disk != listed {
        return Err(format!(
            "the fixtures folder holds {on_disk:?} but pin.json lists {listed:?}"
        ));
    }

    let mut paths = Vec::with_capacity(pinned.len());
    for (name, sha) in &pinned {
        let path = fixtures.join(name);
        let bytes = std::fs::read(&path).map_err(|e| format!("{name}: {e}"))?;
        let digest = lf_sha256(&bytes);
        if &digest != sha {
            return Err(format!(
                "{name} is not the file pin.json records (sha256 {digest}, pinned {sha})"
            ));
        }
        paths.push(path);
    }
    Ok(paths)
}
