//! `MeshSurfaceBuilder`, the time-sliced build of a `MeshSurface`: the result is the same bits at
//! any slicing and the same as before the build could be sliced, one step's work is bounded on a
//! million-face mesh, empty, soup and broken meshes build, and a dropped builder leaves nothing
//! behind.

use std::f64::consts::PI;

use solidmesh_math::{Color, Vector2Float};
use solidmesh_mesh::ray_tracer::{MeshSurface, MeshSurfaceBuilder};
use solidmesh_mesh::{FaceTextureData, Mesh};

const CREASE_30: f64 = 30.0 * PI / 180.0;

/// A welded `n` by `n` grid of quads with a bumpy height, each vertex sharing six faces.
pub(crate) fn welded_grid(n: i32) -> Mesh {
    let mut vertices = Vec::new();
    for y in 0..=n {
        for x in 0..=n {
            vertices.extend([f64::from(x), f64::from(y), f64::from((x * y) % 7) * 0.3]);
        }
    }
    let at = |x: i32, y: i32| x + y * (n + 1);
    let mut indices = Vec::new();
    for y in 0..n {
        for x in 0..n {
            indices.extend([at(x, y), at(x + 1, y), at(x + 1, y + 1)]);
            indices.extend([at(x, y), at(x + 1, y + 1), at(x, y + 1)]);
        }
    }
    Mesh::from_arrays(&vertices, &indices)
}

/// A fan of `spokes` triangles around one hub vertex, the vertex with the most faces there can
/// be, on a shallow cone so neighbouring faces meet at small angles. `colored` paints the faces
/// and puts a texture on every third one.
pub(crate) fn fan(spokes: i32, colored: bool) -> Mesh {
    let mut vertices = vec![0.0, 0.0, 1.0];
    for spoke in 0..spokes {
        let angle = f64::from(spoke) * std::f64::consts::TAU / f64::from(spokes);
        vertices.extend([10.0 * libm_cos(angle), 10.0 * libm_sin(angle), 0.0]);
    }
    let mut indices = Vec::new();
    for spoke in 0..spokes {
        indices.extend([0, 1 + spoke, 1 + (spoke + 1) % spokes]);
    }
    let mut mesh = Mesh::from_arrays(&vertices, &indices);
    if colored {
        mesh.face_colors = Some(
            (0..spokes)
                .map(|face| Color::rgb((face % 256) as u8, 40, 200))
                .collect(),
        );
        for face in (0..spokes).step_by(3) {
            let uv = |value: f64| Vector2Float::new(value, value / 2.0);
            mesh.face_textures.insert(
                face,
                FaceTextureData {
                    image: None,
                    uv0: uv(0.1),
                    uv1: uv(0.5),
                    uv2: uv(0.9),
                },
            );
        }
    }
    mesh
}

/// Exactly reproducible cosine: the fixture must not depend on the platform's trigonometry.
fn libm_cos(angle: f64) -> f64 {
    // A Taylor series to full precision over one turn, so the digest holds on every platform.
    let reduced = (angle / std::f64::consts::TAU).fract() * std::f64::consts::TAU;
    let squared = reduced * reduced;
    let (mut term, mut sum) = (1.0_f64, 1.0_f64);
    for k in 1..=20_i32 {
        term *= -squared / f64::from((2 * k - 1) * (2 * k));
        sum += term;
    }
    sum
}

fn libm_sin(angle: f64) -> f64 {
    libm_cos(angle - std::f64::consts::FRAC_PI_2)
}

/// Unwelded triangles, each with its own vertices.
fn soup() -> Mesh {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for t in 0..64 {
        let t = f64::from(t);
        let base = (vertices.len() / 3) as i32;
        vertices.extend([t, 0.0, t * 0.1, t + 1.0, 0.0, t * 0.2, t, 1.0, 0.0]);
        indices.extend([base, base + 1, base + 2]);
    }
    Mesh::from_arrays(&vertices, &indices)
}

/// A soup with a NaN corner and an infinite one, kept as they are.
fn broken() -> Mesh {
    let mut mesh = soup();
    mesh.vertices[4].x = f32::NAN;
    mesh.vertices[9].y = f32::INFINITY;
    mesh
}

/// The digest of a table's full debug form, which prints every number.
fn surface_digest(surface: &MeshSurface) -> u64 {
    let mut digest = 0xcbf2_9ce4_8422_2325_u64;
    for byte in format!("{surface:?}").into_bytes() {
        digest = (digest ^ u64::from(byte)).wrapping_mul(0x0000_0100_0000_01b3);
    }
    digest
}

/// The table for each of the kinds of mesh and crease angles the shading depends on, in a fixed
/// order, by name, so the pinned digests can be read beside it.
fn digests() -> Vec<(&'static str, u64)> {
    let empty = Mesh::new();
    let grid = welded_grid(40);
    let colored_fan = fan(257, true);
    let bare_fan = fan(257, false);
    let soup = soup();
    let broken = broken();
    let cases: [(&str, &Mesh, f64); 8] = [
        ("empty", &empty, CREASE_30),
        ("grid crease 30", &grid, CREASE_30),
        ("grid crease 0", &grid, 0.0),
        ("grid crease PI", &grid, PI),
        ("colored fan", &colored_fan, CREASE_30),
        ("bare fan", &bare_fan, CREASE_30),
        ("soup", &soup, CREASE_30),
        ("broken soup", &broken, CREASE_30),
    ];
    cases
        .iter()
        .map(|&(name, mesh, crease)| {
            (
                name,
                surface_digest(&MeshSurface::new(mesh, crease).unwrap()),
            )
        })
        .collect()
}

/// Pinned from the table as it was built before the build could be sliced, in `digests` order:
/// moving one means the shading normals changed.
const PINNED_TABLE_DIGESTS: [u64; 8] = [
    0x80904aca8a362562,
    0x94bf6e155a930139,
    0x46d329fa2255e2f3,
    0xc2f33edb508a15f2,
    0xa2c58dde70b232bc,
    0xf8e2a5c8eb9ffe50,
    0x72409c171670b36f,
    0x1593b6da27b3049d,
];

#[test]
fn the_table_is_what_it_always_was() {
    let all = digests();
    for (name, digest) in &all {
        println!("{name}: {digest:#018x},");
    }
    for ((name, digest), pinned) in all.into_iter().zip(PINNED_TABLE_DIGESTS) {
        assert_eq!(digest, pinned, "{name}: got {digest:#018x}");
    }
}

/// Runs the builder to the end in slices of `units`, and returns the table and the most any step
/// did in work units.
fn build_sliced(mesh: &Mesh, crease: f64, units: usize) -> (MeshSurface, u64) {
    let mut builder = MeshSurfaceBuilder::new(mesh, crease).unwrap();
    let mut largest = 0;
    let mut before = builder.work_done();
    loop {
        let finished = builder.step(units);
        let step_work = builder.work_done() - before;
        largest = largest.max(step_work);
        before = builder.work_done();
        if let Some(surface) = finished {
            return (surface, largest);
        }
    }
}

/// The table is the same bits sliced into steps of any size, including one unit at a time, and
/// the same as `MeshSurface::new`; every step stays within its budget and the stated overrun.
#[test]
fn slicing_a_build_changes_nothing() {
    let cases: Vec<(&str, Mesh, f64)> = vec![
        ("grid", welded_grid(32), CREASE_30),
        ("fan", fan(2_000, true), CREASE_30),
        ("large fan", fan(20_000, false), PI),
        ("soup", soup(), CREASE_30),
    ];
    for (name, mesh, crease) in cases {
        let whole = MeshSurface::new(&mesh, crease).unwrap();
        for units in [1, 3, 64, 1000, usize::MAX] {
            let (sliced, largest) = build_sliced(&mesh, crease, units);
            assert_eq!(whole, sliced, "{name} sliced into {units}");
            assert!(
                largest <= (units.min(1 << 40) + MeshSurfaceBuilder::STEP_OVERRUN_UNITS) as u64,
                "{name} at {units}: a step did {largest} units"
            );
        }
    }
}

/// The table of an empty mesh and of a mesh with broken corners builds to the same result as
/// `new` at any slicing; a broken corner is printed as itself, so the debug form compares NaNs
/// as equal.
#[test]
fn empty_and_broken_meshes_build() {
    let broken = broken();
    let empty = Mesh::new();
    for mesh in [&broken, &empty] {
        let whole = format!("{:?}", MeshSurface::new(mesh, CREASE_30).unwrap());
        for units in [1, 5, usize::MAX] {
            let (sliced, _) = build_sliced(mesh, CREASE_30, units);
            assert_eq!(whole, format!("{sliced:?}"));
        }
    }
    let empty_table = MeshSurface::new(&empty, CREASE_30).unwrap();
    assert_eq!(empty_table.face_count(), 0);
}

/// A step's work is bounded on a million-face mesh and on a mesh with one vertex holding a
/// huge fan of faces; a large build returns the same table sliced; the cost of a work unit is
/// printed for the caller's frame-budget arithmetic.
#[test]
fn a_step_on_a_large_mesh_is_bounded() {
    let grid = welded_grid(708);
    println!("grid faces {}", grid.faces.len());
    assert!(grid.faces.len() > 1_000_000);

    let start = std::time::Instant::now();
    let whole = MeshSurface::new(&grid, CREASE_30).unwrap();
    let new_time = start.elapsed();

    let units = 100_000;
    let start = std::time::Instant::now();
    let mut builder = MeshSurfaceBuilder::new(&grid, CREASE_30).unwrap();
    let mut largest_step = std::time::Duration::ZERO;
    let mut steps = 0_u64;
    let mut before = 0;
    let sliced = loop {
        let step_start = std::time::Instant::now();
        let result = builder.step(units);
        largest_step = largest_step.max(step_start.elapsed());
        steps += 1;
        let done = builder.work_done();
        assert!(
            done - before <= (units + MeshSurfaceBuilder::STEP_OVERRUN_UNITS) as u64,
            "a step did {} units",
            done - before
        );
        before = done;
        if let Some(surface) = result {
            break surface;
        }
    };
    let sliced_time = start.elapsed();
    assert_eq!(whole, sliced);
    let total_units = builder.work_done();
    let nanos_per_unit = sliced_time.as_secs_f64() * 1e9 / total_units as f64;
    println!(
        "1M grid: new {new_time:?}, sliced {sliced_time:?} in {steps} steps, largest step \
         {largest_step:?}, {total_units} units ({:.2} per face), {nanos_per_unit:.2} ns per unit",
        total_units as f64 / grid.faces.len() as f64
    );

    // A hub vertex with 200,000 faces is split across steps like everything else.
    let large_fan = fan(200_000, false);
    let (table, largest) = build_sliced(&large_fan, CREASE_30, 50_000);
    assert_eq!(table.face_count(), 200_000);
    assert!(
        largest <= (50_000 + MeshSurfaceBuilder::STEP_OVERRUN_UNITS) as u64,
        "a step on the hub vertex did {largest} units"
    );
}

/// A builder can be dropped at any point, leaving the mesh untouched and working, and can be
/// shared across frames over an `Arc<Mesh>`.
#[test]
fn a_build_can_be_dropped_midway_and_share_its_mesh() {
    let grid = welded_grid(20);
    {
        let mut builder = MeshSurfaceBuilder::new(&grid, CREASE_30).unwrap();
        assert!(builder.step(50).is_none());
        // Dropped here, half built.
    }
    let whole = MeshSurface::new(&grid, CREASE_30).unwrap();
    let shared = std::sync::Arc::new(grid.clone());
    let mut builder = MeshSurfaceBuilder::from_shared(shared, CREASE_30).unwrap();
    let mut table = None;
    while table.is_none() {
        table = builder.step(1000);
    }
    assert_eq!(table.unwrap(), whole);
    assert!(builder.step(1).is_none(), "a finished builder had more");
    assert!(MeshSurfaceBuilder::new(&grid, 7.0).is_err());
}

/// How many units a known mesh costs is part of what a caller budgets with: every stage is
/// charged, so a stage that stopped counting would show here.
#[test]
fn the_work_is_counted_exactly() {
    let mesh = welded_grid(32);
    let mut builder = MeshSurfaceBuilder::new(&mesh, CREASE_30).unwrap();
    while builder.step(usize::MAX).is_none() {}
    let units = builder.work_done();
    println!("welded_grid(32) cost {units} units");
    assert_eq!(units, 89_155, "the work a known mesh costs moved");
}
