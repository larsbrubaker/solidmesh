//! Port of the test helper `ManyHolesRimReading.cs`: Many Holes' rim pick read as plain data - its
//! edges chained into runs on vertex indices (`chain`), each closed loop's shape (`shape`), and
//! every closed loop of a resolved pick filleted ALONE through the sections - room, build,
//! assembly - and measured (`read_every_loop`). Read by `section_many_holes_tests` and
//! `section_many_holes_neighbour_tests` on [`crate::many_holes_test_mesh`]. C#'s
//! `CancellationToken` parameter is left out: nothing here cancels.

#![allow(dead_code)]

use std::collections::HashSet;
use std::time::Instant;

use indexmap::IndexMap;
use polygon_mesh::{Mesh, MeshEdgeGraph};
use vector_math::{math_helper, Vector3};

use geometry_processing::bevel::bevel_chain_linker::{link, BevelSweptChain};
use geometry_processing::bevel::sections::section_builder_run::try_build_loop;
use geometry_processing::bevel::sections::{
    section_assembly, section_room, SectionPool, SectionRoomCache,
};
use geometry_processing::bevel::{BevelAdvisory, BevelAdvisoryArg};
use geometry_processing::topology::{BevelEdgeGroupResolution, FeatureEdgeSet};

use crate::mesh_measurements;

/// C# `Run`: one walk of the chained edges.
#[derive(Debug, Default, Clone)]
pub struct Run {
    pub vertices: Vec<i32>,
    pub edges: Vec<(i32, i32)>,
    pub is_closed: bool,
}

impl Run {
    pub fn first_end(&self) -> i32 {
        self.vertices[0]
    }

    pub fn last_end(&self) -> i32 {
        self.vertices[self.vertices.len() - 1]
    }
}

/// C# `Census`: every run, split into closed loops and open runs, with the vertex degrees.
#[derive(Debug, Default)]
pub struct Census {
    pub runs: Vec<Run>,
    pub closed_loops: Vec<Run>,
    pub open_runs: Vec<Run>,
    pub degree_histogram: IndexMap<usize, i32>,
    pub loose_edge_count: i32,
}

/// C# `Chain`: the edges chained on vertex indices - open runs first, walked from an end of
/// degree one so a run is never entered in its middle, then whatever is left as loops.
pub fn chain(edges: &[(i32, i32)]) -> Census {
    // Insertion order, as C#'s Dictionary iterates.
    let mut incident: IndexMap<i32, Vec<usize>> = IndexMap::new();

    for (i, &(start, end)) in edges.iter().enumerate() {
        incident.entry(start).or_default().push(i);
        incident.entry(end).or_default().push(i);
    }

    let mut census = Census::default();

    for at in incident.values() {
        *census.degree_histogram.entry(at.len()).or_insert(0) += 1;
    }

    let mut used = vec![false; edges.len()];

    let mut starts: Vec<i32> = incident
        .iter()
        .filter(|(_, at)| at.len() == 1)
        .map(|(&v, _)| v)
        .collect();
    starts.sort_unstable();

    for start in starts {
        if incident[&start].iter().all(|&e| used[e]) {
            continue;
        }

        census.runs.push(walk(edges, &incident, &mut used, start));
    }

    for i in 0..edges.len() {
        if !used[i] {
            census
                .runs
                .push(walk(edges, &incident, &mut used, edges[i].0));
        }
    }

    for run in &census.runs {
        if run.is_closed {
            census.closed_loops.push(run.clone());
        } else {
            census.open_runs.push(run.clone());
            census.loose_edge_count += run.edges.len() as i32;
        }
    }

    census
}

fn walk(
    edges: &[(i32, i32)],
    incident: &IndexMap<i32, Vec<usize>>,
    used: &mut [bool],
    start: i32,
) -> Run {
    let mut run = Run::default();

    run.vertices.push(start);

    let mut at = start;

    while let Some(&next) = incident[&at].iter().find(|&&e| !used[e]) {
        used[next] = true;
        run.edges.push(edges[next]);

        at = if edges[next].0 == at {
            edges[next].1
        } else {
            edges[next].0
        };

        run.vertices.push(at);

        if at == start {
            run.is_closed = true;
            break;
        }
    }

    run
}

/// C# `LoopShape`.
#[derive(Debug, Default, Clone, Copy)]
pub struct LoopShape {
    pub edge_count: i32,
    pub normal: Vector3,
    pub centre: Vector3,
    pub planarity: f64,
    pub mean_radius: f64,
    pub radius_spread: f64,
    pub perimeter: f64,
}

/// C# `Shape`: a run's centre, Newell normal, planarity, radii and perimeter.
pub fn shape(mesh: &Mesh, run: &Run) -> LoopShape {
    let take = run.vertices.len() - usize::from(run.is_closed);
    let points: Vec<Vector3> = run.vertices[..take]
        .iter()
        .map(|&v| Vector3::from_vector3_float(mesh.vertices[v as usize]))
        .collect();

    let mut centre = Vector3::ZERO;

    for &p in &points {
        centre += p;
    }

    centre /= points.len() as f64;

    // Newell's normal, which is the area-weighted plane of a closed polygon however it wanders.
    let mut normal = Vector3::ZERO;

    for i in 0..points.len() {
        let a = points[i];
        let b = points[(i + 1) % points.len()];

        normal += Vector3::new(
            (a.y - b.y) * (a.z + b.z),
            (a.z - b.z) * (a.x + b.x),
            (a.x - b.x) * (a.y + b.y),
        );
    }

    if normal.length_squared() > 0.0 {
        normal = normal.get_normal();
    }

    let mut result = LoopShape {
        edge_count: run.edges.len() as i32,
        centre,
        normal,
        ..LoopShape::default()
    };

    let mut radii = Vec::new();

    for &p in &points {
        result.planarity = result.planarity.max((p - centre).dot(normal).abs());
        radii.push((p - centre).length());
    }

    result.mean_radius = radii.iter().sum::<f64>() / radii.len() as f64;
    result.radius_spread = radii.iter().copied().fold(f64::MIN, f64::max)
        - radii.iter().copied().fold(f64::MAX, f64::min);

    for i in 0..points.len() {
        result.perimeter += (points[(i + 1) % points.len()] - points[i]).length();
    }

    result
}

/// C# `MeshEdges`: each undirected mesh edge with the faces on it.
pub fn mesh_edges(mesh: &Mesh) -> IndexMap<(i32, i32), Vec<i32>> {
    let mut map: IndexMap<(i32, i32), Vec<i32>> = IndexMap::new();

    for (f, face) in mesh.faces.iter().enumerate() {
        for (a, b) in [(face.v0, face.v1), (face.v1, face.v2), (face.v2, face.v0)] {
            let key = if a < b { (a, b) } else { (b, a) };
            map.entry(key).or_default().push(f as i32);
        }
    }

    map
}

/// C# `DihedralDegrees`: the angle between an edge's two faces' normals, NaN unless it has two.
pub fn dihedral_degrees(mesh: &Mesh, faces: &[i32]) -> f64 {
    if faces.len() != 2 {
        return f64::NAN;
    }

    let a = Vector3::from_vector3_float(mesh.faces[faces[0] as usize].normal);
    let b = Vector3::from_vector3_float(mesh.faces[faces[1] as usize].normal);

    math_helper::radians_to_degrees(a.dot(b).clamp(-1.0, 1.0).acos())
}

/// C# `Text(double)`: `0.#####` - at most five decimals, trailing zeros dropped.
pub fn text(value: f64) -> String {
    let fixed = format!("{value:.5}");
    let trimmed = fixed.trim_end_matches('0').trim_end_matches('.');

    if trimmed == "-0" {
        "0".to_string()
    } else {
        trimmed.to_string()
    }
}

/// C# `Text(Vector3)`.
pub fn text_of(value: Vector3) -> String {
    format!("({}, {}, {})", text(value.x), text(value.y), text(value.z))
}

/// C# `Describe`: the census as lines of text.
pub fn describe(mesh: &Mesh, census: &Census) -> String {
    let mut out = String::new();

    out.push_str(&format!(
        "runs {}: closed {}, open {}, loose edges {}\n",
        census.runs.len(),
        census.closed_loops.len(),
        census.open_runs.len(),
        census.loose_edge_count
    ));

    let mut degrees: Vec<(&usize, &i32)> = census.degree_histogram.iter().collect();
    degrees.sort_by_key(|(&degree, _)| degree);
    out.push_str(&format!(
        "degrees: {}\n",
        degrees
            .iter()
            .map(|(d, n)| format!("{d}x{n}"))
            .collect::<Vec<_>>()
            .join(", ")
    ));

    for run in &census.open_runs {
        let a = Vector3::from_vector3_float(mesh.vertices[run.first_end() as usize]);
        let b = Vector3::from_vector3_float(mesh.vertices[run.last_end() as usize]);

        out.push_str(&format!(
            "open run {} edges, ends {}{} .. {}{}, end gap {}\n",
            run.edges.len(),
            run.first_end(),
            text_of(a),
            run.last_end(),
            text_of(b),
            text((b - a).length())
        ));
    }

    out
}

/// C# `LoopAnswer`: what one loop read alone answered.
#[derive(Debug, Default)]
pub struct LoopAnswer {
    pub name: String,
    pub centre: Vector3,
    pub edge_count: i32,
    pub built: bool,
    pub build_why: Option<BevelAdvisory>,
    pub assembled: bool,
    pub assembly_why: Option<BevelAdvisory>,
    pub open_edges: i32,
    pub non_manifold_edges: i32,
    pub shells: i32,
    pub fits: bool,
    pub largest_that_fits: f64,
    pub room_why: Option<BevelAdvisory>,
    pub build_milliseconds: f64,
    pub assembly_milliseconds: f64,
    pub room_milliseconds: f64,
}

impl LoopAnswer {
    pub fn total_milliseconds(&self) -> f64 {
        self.build_milliseconds + self.assembly_milliseconds + self.room_milliseconds
    }

    /// C# `Outcome`: `build:<code>`, `assembly:<code>`, `unsound` or `sound`.
    pub fn outcome(&self) -> String {
        if !self.built {
            format!("build:{}", Self::code(self.build_why.as_ref()))
        } else if !self.assembled {
            format!("assembly:{}", Self::code(self.assembly_why.as_ref()))
        } else if self.open_edges != 0 || self.non_manifold_edges != 0 || self.shells != 1 {
            "unsound".to_string()
        } else {
            "sound".to_string()
        }
    }

    pub fn code(advisory: Option<&BevelAdvisory>) -> String {
        advisory.map_or_else(|| "none".to_string(), |a| format!("{:?}", a.code()))
    }

    pub fn code_and_station(advisory: Option<&BevelAdvisory>) -> String {
        let Some(advisory) = advisory else {
            return "none".to_string();
        };

        let station = match advisory.args().first() {
            Some(BevelAdvisoryArg::Vector3(at)) => text_of(*at),
            _ => advisory
                .args()
                .iter()
                .map(|arg| arg.to_string())
                .collect::<Vec<_>>()
                .join(","),
        };

        format!("{:?} at {station}", advisory.code())
    }
}

/// C# `ReadEveryLoop`: every closed loop of the resolution, ordered by centre (z, then y, then x)
/// so the row order is the part's own geometry, read alone; `only` names the indices to read.
pub fn read_every_loop(
    part: &Mesh,
    resolution: &BevelEdgeGroupResolution,
    radius: f64,
    segments: i32,
    mut report: Option<&mut dyn FnMut(&LoopAnswer)>,
    only: Option<&HashSet<usize>>,
) -> Vec<LoopAnswer> {
    let graph = part.get_mesh_edge_graph();

    // One cache for the whole reading: the room's flood of a face's region is its entire cost,
    // and every mouth of this barrel stands on the SAME wall region.
    let mut cache = SectionRoomCache::new(part, &graph);
    let mut answers = Vec::new();

    let loops = the_loops_in_order(part, resolution);

    for (i, one) in loops.iter().enumerate() {
        if only.is_some_and(|only| !only.contains(&i)) {
            continue;
        }

        let answer = read_one_loop(
            part,
            &graph,
            &mut cache,
            &resolution.features,
            one,
            radius,
            segments,
        );

        if let Some(report) = report.as_mut() {
            report(&answer);
        }

        answers.push(answer);
    }

    answers
}

/// The closed loops of a resolution in the loop table's order: by centre z, then y, then x (C#'s
/// stable `OrderBy(..).ThenBy(..).ThenBy(..)`).
pub fn the_loops_in_order(
    part: &Mesh,
    resolution: &BevelEdgeGroupResolution,
) -> Vec<BevelSweptChain> {
    let mut loops: Vec<BevelSweptChain> = link(&resolution.edges)
        .into_iter()
        .filter(|chain| chain.is_closed)
        .collect();

    loops.sort_by(|a, b| {
        let (a, b) = (centre(part, a), centre(part, b));
        a.z.total_cmp(&b.z)
            .then(a.y.total_cmp(&b.y))
            .then(a.x.total_cmp(&b.x))
    });

    loops
}

/// C# `Centre`: the mean of a chain's from-vertices.
pub fn centre(part: &Mesh, chain: &BevelSweptChain) -> Vector3 {
    let mut sum = Vector3::ZERO;

    for edge in &chain.edges {
        sum += Vector3::from_vector3_float(part.vertices[edge.from_vertex() as usize]);
    }

    sum / chain.edges.len() as f64
}

/// The loop table's name for a loop: `n<edges>@(<x>,<y>,<z>)` at one decimal.
pub fn name_of(part: &Mesh, chain: &BevelSweptChain) -> String {
    let at = centre(part, chain);

    format!(
        "n{}@({:.1},{:.1},{:.1})",
        chain.edges.len(),
        at.x,
        at.y,
        at.z
    )
}

fn read_one_loop(
    part: &Mesh,
    graph: &MeshEdgeGraph,
    cache: &mut SectionRoomCache,
    features: &FeatureEdgeSet,
    one: &BevelSweptChain,
    radius: f64,
    segments: i32,
) -> LoopAnswer {
    let mut answer = LoopAnswer {
        centre: centre(part, one),
        edge_count: one.edges.len() as i32,
        name: name_of(part, one),
        ..LoopAnswer::default()
    };

    let clock = Instant::now();

    // The room reads an offset side's reach with the OPERATION's own feature reading, the same set
    // the build walks with - otherwise every fold of this tessellated wall stops the reach.
    let feature_mesh_edges: HashSet<i32> =
        features.edges().iter().map(|f| f.mesh_edge_index).collect();
    let is_a_feature = |edge: i32| feature_mesh_edges.contains(&edge);

    let (fits, largest, room_why) = section_room::try_fit_edges(
        Some(part),
        Some(graph),
        Some(&one.edges),
        true,
        radius,
        Some(cache),
        Some(&is_a_feature),
    );

    answer.fits = fits;
    answer.largest_that_fits = largest;
    answer.room_why = room_why;
    answer.room_milliseconds = clock.elapsed().as_secs_f64() * 1000.0;

    let ring: Vec<i32> = one.edges.iter().map(|edge| edge.from_vertex()).collect();
    let mut pool = SectionPool::new(&part.vertices);

    let clock = Instant::now();

    let built = try_build_loop(
        &ring,
        Some(features),
        part,
        graph,
        &mut pool,
        radius,
        segments,
        true,
    );

    answer.build_milliseconds = clock.elapsed().as_secs_f64() * 1000.0;

    let run = match built {
        Ok(run) => run,
        Err(why) => {
            answer.build_why = why.advisory().cloned();

            return answer;
        }
    };

    answer.built = true;

    let clock = Instant::now();

    let assembled = section_assembly::try_assemble(part, graph, &run, &mut pool);

    answer.assembly_milliseconds = clock.elapsed().as_secs_f64() * 1000.0;

    match assembled {
        Ok(result) => {
            answer.assembled = true;
            measure(&result, &mut answer);
        }
        Err(why) => answer.assembly_why = why.advisory().cloned(),
    }

    answer
}

fn measure(result: &Mesh, answer: &mut LoopAnswer) {
    let graph = result.get_mesh_edge_graph();

    for edge in 0..graph.edge_count() {
        let faces = graph.get_face_count(edge);

        if faces == 1 {
            answer.open_edges += 1;
        } else if faces > 2 {
            answer.non_manifold_edges += 1;
        }
    }

    answer.shells = mesh_measurements::shell_count(result);
}

/// C# `Row`: one line of the loop table.
pub fn row(answer: &LoopAnswer) -> String {
    format!(
        "{:<26} {:<34} room {:>6.3} {:<24} {:>5.0}+{:>5.0}+{:>4.0} ms  {}",
        answer.name,
        answer.outcome(),
        answer.largest_that_fits,
        if answer.fits {
            "fits".to_string()
        } else {
            LoopAnswer::code(answer.room_why.as_ref())
        },
        answer.build_milliseconds,
        answer.assembly_milliseconds,
        answer.room_milliseconds,
        LoopAnswer::code_and_station(answer.build_why.as_ref().or(answer.assembly_why.as_ref()))
    )
}
