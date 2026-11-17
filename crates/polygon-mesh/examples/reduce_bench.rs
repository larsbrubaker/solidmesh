//! Measures Reduce (`Mesh::reduced_to_triangle_count`) on a mesh read from a file: the time it
//! takes, the two-way surface deviation of the result from the original
//! (`mesh_distance::surface_deviation`) as a percentage of the original's largest
//! bounding-box side (max, p99 and p99.9 over both surfaces' lattice samples, and the exact
//! largest distance from each mesh's vertices to the other surface: `vo2r` original to
//! reduced, `vr2o` reduced to original), and the open and non-manifold edges left.
//! `REDUCE_BENCH_SAMPLES` sets the samples per surface; `REDUCE_BENCH_TIME_ONLY` skips the
//! measuring, so a profiler or `time` sees the reduce alone; `REDUCE_BENCH_OUT=<path>` writes
//! the reduced mesh in the input's format, so two builds' outputs can be compared byte for byte. With `REDUCE_BENCH_WORST` set it also
//! prints each side's vertex farthest from the other surface.
//!
//! The file is raw little-endian: `u32` vertex count, `u32` triangle count, then `f64` x, y, z
//! per vertex, then `i32` a, b, c per triangle.
//!
//! Usage: `cargo run --release --example reduce_bench -p solidmesh-mesh -- <mesh.bin> <target
//! triangles> [keep|free|<reduced.bin>]` (`keep` reduces with Maintain Surface, the default; a
//! path measures that already-reduced mesh, in the same format, instead of reducing).

use std::collections::BTreeMap;
use std::io::Read;
use std::time::Instant;

use solidmesh_math::Vector3;
use solidmesh_mesh::mesh_distance::{
    point_distances, surface_deviation, DEFAULT_SAMPLES_PER_SURFACE,
};
use solidmesh_mesh::Mesh;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 3 {
        return Err(
            "usage: reduce_bench <mesh.bin> <target triangles> [keep|free|<reduced.bin>]".into(),
        );
    }
    let mut bytes = Vec::new();
    std::fs::File::open(&args[1])?.read_to_end(&mut bytes)?;
    let (vertices, faces) = parse(&bytes)?;
    let target: i32 = args[2].parse()?;
    let mode = args.get(3).map(String::as_str).unwrap_or("keep");
    let mesh = Mesh::from_arrays(&vertices, &faces);

    let start = Instant::now();
    let reduced = match mode {
        "keep" | "free" => mesh.reduced_to_triangle_count(target, mode == "keep"),
        path => {
            let mut other = Vec::new();
            std::fs::File::open(path)?.read_to_end(&mut other)?;
            let (v, f) = parse(&other)?;
            Mesh::from_arrays(&v, &f)
        }
    };
    let seconds = start.elapsed().as_secs_f64();
    if let Some(path) = std::env::var_os("REDUCE_BENCH_OUT") {
        let mut out = Vec::new();
        out.extend((reduced.vertices.len() as u32).to_le_bytes());
        out.extend((reduced.faces.len() as u32).to_le_bytes());
        for p in &reduced.vertices {
            for c in [p.x, p.y, p.z] {
                out.extend((c as f64).to_le_bytes());
            }
        }
        for f in reduced.faces.iter() {
            for i in [f.v0, f.v1, f.v2] {
                out.extend(i.to_le_bytes());
            }
        }
        std::fs::write(path, out)?;
    }
    if std::env::var_os("REDUCE_BENCH_TIME_ONLY").is_some() {
        println!("{} tris in {seconds:.2}s", reduced.faces.len());
        return Ok(());
    }
    let samples = std::env::var("REDUCE_BENCH_SAMPLES")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(DEFAULT_SAMPLES_PER_SURFACE);
    let deviation = surface_deviation(&mesh, &reduced, samples, 0.99);
    let p999 = surface_deviation(&mesh, &reduced, samples, 0.999).percentile_fraction();
    let to_reduced = vertex_hausdorff(&mesh, &reduced) / deviation.scale;
    let to_original = vertex_hausdorff(&reduced, &mesh) / deviation.scale;
    let (open, non_manifold) = edge_defects(&reduced);
    if std::env::var_os("REDUCE_BENCH_WORST").is_some() {
        print_worst("original vertex", &mesh, &reduced, deviation.scale);
        print_worst("reduced vertex", &reduced, &mesh, deviation.scale);
        print_worst_centroid("original face", &mesh, &reduced, deviation.scale);
        print_worst_centroid("reduced face", &reduced, &mesh, deviation.scale);
    }
    println!(
        "{} -> {} tris  max {:.4}%  p99 {:.4}%  p999 {:.4}%  vo2r {:.4}%  vr2o {:.4}%  open {}  non-manifold {}  {:.1}s",
        faces.len() / 3,
        reduced.faces.len(),
        100.0 * deviation.max_fraction(),
        100.0 * deviation.percentile_fraction(),
        100.0 * p999,
        100.0 * to_reduced,
        100.0 * to_original,
        open,
        non_manifold,
        seconds
    );
    Ok(())
}

/// The largest distance from a vertex of `from` to `to`'s surface: exact at every vertex, so
/// it catches a lost tip the lattice samples could step over.
fn vertex_hausdorff(from: &Mesh, to: &Mesh) -> f64 {
    let points: Vec<Vector3> = from
        .vertices
        .iter()
        .map(|v| Vector3::new(v.x as f64, v.y as f64, v.z as f64))
        .collect();
    point_distances(to, &points).into_iter().fold(0.0, f64::max)
}

/// With `REDUCE_BENCH_WORST` set: the vertex of `from` farthest from `to`'s surface, so a
/// regression can be found on the model.
fn print_worst(label: &str, from: &Mesh, to: &Mesh, scale: f64) {
    let points: Vec<Vector3> = from
        .vertices
        .iter()
        .map(|v| Vector3::new(v.x as f64, v.y as f64, v.z as f64))
        .collect();
    let distances = point_distances(to, &points);
    let mut worst = 0;
    for (i, d) in distances.iter().enumerate() {
        if *d > distances[worst] {
            worst = i;
        }
    }
    let p = points[worst];
    println!(
        "  worst {label} {worst} at ({:.4}, {:.4}, {:.4}): {:.3}%",
        p.x,
        p.y,
        p.z,
        100.0 * distances[worst] / scale
    );
}

/// With `REDUCE_BENCH_WORST` set: the face of `from` whose centroid is farthest from `to`.
fn print_worst_centroid(label: &str, from: &Mesh, to: &Mesh, scale: f64) {
    let points: Vec<Vector3> = from
        .faces
        .iter()
        .map(|f| {
            let [a, b, c] = [f.v0, f.v1, f.v2].map(|i| from.vertices[i as usize]);
            Vector3::new(
                (a.x as f64 + b.x as f64 + c.x as f64) / 3.0,
                (a.y as f64 + b.y as f64 + c.y as f64) / 3.0,
                (a.z as f64 + b.z as f64 + c.z as f64) / 3.0,
            )
        })
        .collect();
    let d = point_distances(to, &points);
    let mut worst = 0;
    for i in 0..d.len() {
        if d[i] > d[worst] {
            worst = i;
        }
    }
    let f = &from.faces[worst];
    let e = |i: i32, j: i32| {
        let (a, b) = (from.vertices[i as usize], from.vertices[j as usize]);
        (((a.x - b.x) as f64).powi(2) + ((a.y - b.y) as f64).powi(2) + ((a.z - b.z) as f64).powi(2))
            .sqrt()
    };
    println!(
        "  worst {label} centroid {worst} at {:?}: {:.3}%  edges {:.4} {:.4} {:.4}",
        points[worst],
        100.0 * d[worst] / scale,
        e(f.v0, f.v1),
        e(f.v1, f.v2),
        e(f.v2, f.v0)
    );
}

type Arrays = (Vec<f64>, Vec<i32>);

fn parse(bytes: &[u8]) -> Result<Arrays, Box<dyn std::error::Error>> {
    let word = |at: usize| -> Result<[u8; 4], Box<dyn std::error::Error>> {
        Ok(bytes.get(at..at + 4).ok_or("truncated")?.try_into()?)
    };
    let nv = u32::from_le_bytes(word(0)?) as usize;
    let nt = u32::from_le_bytes(word(4)?) as usize;
    let mut at = 8;
    let mut vertices = Vec::with_capacity(nv * 3);
    for _ in 0..nv * 3 {
        let raw: [u8; 8] = bytes.get(at..at + 8).ok_or("truncated")?.try_into()?;
        vertices.push(f64::from_le_bytes(raw));
        at += 8;
    }
    let mut faces = Vec::with_capacity(nt * 3);
    for _ in 0..nt * 3 {
        faces.push(i32::from_le_bytes(word(at)?));
        at += 4;
    }
    Ok((vertices, faces))
}

/// Edges used by one face (open) and by more than two (non-manifold).
fn edge_defects(mesh: &Mesh) -> (usize, usize) {
    let mut uses: BTreeMap<(i32, i32), usize> = BTreeMap::new();
    for f in mesh.faces.iter() {
        for (a, b) in [(f.v0, f.v1), (f.v1, f.v2), (f.v2, f.v0)] {
            *uses.entry((a.min(b), a.max(b))).or_default() += 1;
        }
    }
    let open = uses.values().filter(|&&n| n == 1).count();
    let non_manifold = uses.values().filter(|&&n| n > 2).count();
    (open, non_manifold)
}
