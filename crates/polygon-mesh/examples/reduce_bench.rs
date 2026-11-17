//! Measures Reduce (`Mesh::reduced_to_triangle_count`) on a mesh read from a file: the time it
//! takes, the two-way surface deviation of the result from the original
//! (`mesh_distance::surface_deviation`) as a percentage of the original's largest
//! bounding-box side, and the open and non-manifold edges left. With `REDUCE_BENCH_WORST` set
//! it also prints each side's vertex farthest from the other surface.
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
        return Err("usage: reduce_bench <mesh.bin> <target triangles> [keep|free]".into());
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
    let deviation = surface_deviation(&mesh, &reduced, DEFAULT_SAMPLES_PER_SURFACE, 0.99);
    let (open, non_manifold) = edge_defects(&reduced);
    if std::env::var_os("REDUCE_BENCH_WORST").is_some() {
        print_worst("original vertex", &mesh, &reduced, deviation.scale);
        print_worst("reduced vertex", &reduced, &mesh, deviation.scale);
    }
    println!(
        "{} -> {} tris  max {:.3}%  p99 {:.3}%  open {}  non-manifold {}  {:.1}s",
        faces.len() / 3,
        reduced.faces.len(),
        100.0 * deviation.max_fraction(),
        100.0 * deviation.percentile_fraction(),
        open,
        non_manifold,
        seconds
    );
    Ok(())
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
