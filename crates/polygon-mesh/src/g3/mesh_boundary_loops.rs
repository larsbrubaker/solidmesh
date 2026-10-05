//! geometry3Sharp `mesh_selection/MeshBoundaryLoops.cs`: walks a mesh's boundary edges into
//! closed loops (and open spans where a walk cannot close), splitting a loop that passes a
//! bowtie vertex more than once into simple sub-loops. Ported with its defaults - spans
//! computed (`SpanBehaviors.Compute`), failures kept as spans (`ConvertToOpenSpan`), no
//! `EdgeFilterF` - which is how the mesh repair and hole fillers construct it; with those, none
//! of C#'s throwing branches can be reached.

use super::dmesh3::DMesh3;
use super::edge_loop::{EdgeLoop, EdgeSpan};

/// g3 `MathUtil.ZeroTolerancef`.
const ZERO_TOLERANCE_F: f32 = 1e-6;
/// g3 `MathUtil.Epsilonf`.
const EPSILON_F: f32 = 1.192_092_9e-7;
/// g3 `MathUtil.Epsilon`.
const EPSILON: f64 = 2.220_446_049_250_313e-16;

/// g3 `MeshBoundaryLoops` after `Compute`.
#[derive(Debug, Clone, Default)]
pub(crate) struct MeshBoundaryLoops {
    pub(crate) loops: Vec<EdgeLoop>,
    /// Unclosed loops.
    pub(crate) spans: Vec<EdgeSpan>,
    /// Set when any open span was found.
    pub(crate) saw_open_spans: bool,
    /// Set when spans had to be added to recover from a failure.
    pub(crate) fell_back_to_spans_on_failure: bool,
}

impl MeshBoundaryLoops {
    /// C# `new MeshBoundaryLoops(mesh)` (`bAutoCompute: true`).
    pub(crate) fn new(mesh: &DMesh3) -> Self {
        let mut out = MeshBoundaryLoops::default();
        out.compute(mesh);
        out
    }

    /// C# `Compute`.
    fn compute(&mut self, mesh: &DMesh3) {
        self.loops.clear();
        self.spans.clear();

        // early-out if we don't actually have boundaries
        if mesh.is_closed() {
            return;
        }

        let ne = mesh.max_edge_id();

        // Temporary memory used to indicate when we have "used" an edge.
        let mut used_edge = vec![false; ne as usize];

        // current loop is stored here, cleared after each loop extracted
        let mut loop_edges: Vec<i32> = Vec::new();
        let mut loop_verts: Vec<i32> = Vec::new();
        let mut bowties: Vec<i32> = Vec::new();

        // Process all edges of mesh.
        for eid in 0..ne {
            if !mesh.is_edge(eid) || used_edge[eid as usize] || !mesh.is_boundary_edge(eid) {
                continue;
            }

            // ok this is start of a boundary chain
            let e_start = eid;
            used_edge[e_start as usize] = true;
            loop_edges.push(e_start);

            let mut e_cur = eid;

            // follow the chain in order of oriented edges
            let mut closed = false;
            let mut is_open_span = false;
            while !closed {
                let ev = mesh.get_oriented_boundary_edge_v(e_cur);
                let cure_b = if is_open_span {
                    ev[0]
                } else {
                    loop_verts.push(ev[0]);
                    ev[1]
                };

                let all_e = mesh.vtx_all_boundary_edges(cure_b);
                let bdry_nbrs = all_e.len();

                if bdry_nbrs < 2 {
                    // hit an 'endpoint' vertex (should only happen when Filter is on...)
                    if is_open_span {
                        closed = true;
                    } else {
                        is_open_span = true; // begin open span
                        e_cur = loop_edges[0]; // restart at other end of loop
                        loop_edges.reverse(); // do this so we can push to front
                    }
                    continue;
                }

                let e_next;
                if bdry_nbrs > 2 {
                    // found "bowtie" vertex...things just got complicated!
                    if cure_b == loop_verts[0] {
                        // The "endpoint" of the current span is the same as the beginning
                        // vertex. If we are not careful we may end up picking the wrong edge.
                        e_next = -2; // sentinel value used below
                    } else {
                        // try to find an unused outgoing edge that is oriented properly.
                        e_next = self.find_left_turn_edge(mesh, e_cur, cure_b, &all_e, &used_edge);
                        if e_next == -1 {
                            // ConvertToOpenSpan: keep what we have as an open span
                            is_open_span = true;
                            closed = true;
                            continue;
                        }
                    }
                    if !bowties.contains(&cure_b) {
                        bowties.push(cure_b);
                    }
                } else {
                    // walk forward to next available edge
                    let (e0, e1) = (all_e[0], all_e[1]);
                    e_next = if e0 == e_cur { e1 } else { e0 };
                }

                if e_next == -2 {
                    // found a bowtie vertex at the start of the loop: done
                    closed = true;
                } else if e_next == e_start {
                    // found edge at start of loop, so loop is done.
                    closed = true;
                } else if used_edge[e_next as usize] {
                    // disaster case - the next edge is already used, but it is not the start
                    // of our loop: ConvertToOpenSpan
                    is_open_span = true;
                    closed = true;
                } else {
                    // push onto accumulated list
                    loop_edges.push(e_next);
                    used_edge[e_next as usize] = true;
                    e_cur = e_next;
                }
            }

            if is_open_span {
                self.saw_open_spans = true;
                loop_edges.reverse(); // orient properly
                self.spans.push(EdgeSpan::from_edges(mesh, &loop_edges));
            } else if !bowties.is_empty() {
                // if we saw a bowtie vertex, we might need to break up this loop,
                // so call extract_subloops
                let (loops, spans) = extract_subloops(mesh, &mut loop_verts, &loop_edges, &bowties);
                self.loops.extend(loops);
                if !spans.is_empty() {
                    self.fell_back_to_spans_on_failure = true;
                    self.spans.extend(spans);
                }
            } else {
                // clean simple loop, convert to EdgeLoop instance
                self.loops.push(EdgeLoop {
                    vertices: loop_verts.clone(),
                    edges: loop_edges.clone(),
                    bowtie_vertices: Vec::new(),
                });
            }

            // reset these lists
            loop_edges.clear();
            loop_verts.clear();
            bowties.clear();
        }
    }

    /// C# `find_left_turn_edge`: at a bowtie vertex, the unused outgoing boundary edge making
    /// the smallest signed turn (in float, about the vertex normal) from the incoming edge.
    fn find_left_turn_edge(
        &self,
        mesh: &DMesh3,
        incoming_e: i32,
        bowtie_v: i32,
        bdry_edges: &[i32],
        used_edges: &[bool],
    ) -> i32 {
        // compute normal and edge [a,bowtie]
        let n = get_vtx_normal(mesh, bowtie_v);
        let other_v = mesh.edge_other_v(incoming_e, bowtie_v);
        let ab = sub(mesh.vertex(bowtie_v), mesh.vertex(other_v));

        // our winner
        let mut best_e = -1;
        let mut best_angle = f64::MAX;

        for &bdry_eid in bdry_edges {
            if used_edges[bdry_eid as usize] {
                continue; // this edge is already used
            }
            let bdry_ev = mesh.get_oriented_boundary_edge_v(bdry_eid);
            if bdry_ev[0] != bowtie_v {
                continue; // have to be able to chain to end of current edge, orientation-wise
            }

            // compute projected angle
            let bc = sub(mesh.vertex(bdry_ev[1]), mesh.vertex(bowtie_v));
            let angle = plane_angle_signed_d(to_f32(ab), to_f32(bc), to_f32(n)) as f64;

            // turn left!
            if best_angle == f64::MAX || angle < best_angle {
                best_angle = angle;
                best_e = bdry_eid;
            }
        }
        best_e
    }
}

/// C# `extract_subloops`: peels the shortest simple loop off each repeated bowtie vertex until
/// none repeats; what is left is one more loop. A walk with no simple loop is kept as a span.
fn extract_subloops(
    mesh: &DMesh3,
    loop_v: &mut [i32],
    loop_e: &[i32],
    bowties: &[i32],
) -> (Vec<EdgeLoop>, Vec<EdgeSpan>) {
    let mut loops = Vec::new();
    let mut spans = Vec::new();

    // figure out which bowties we saw are actually duplicated in loopV
    let mut dupes: Vec<i32> = bowties
        .iter()
        .copied()
        .filter(|&bv| count_in_list(loop_v, bv) > 1)
        .collect();

    // we might not actually have any duplicates, if we got luck. Early out in that case
    if dupes.is_empty() {
        loops.push(EdgeLoop {
            vertices: loop_v.to_vec(),
            edges: loop_e.to_vec(),
            bowtie_vertices: bowties.to_vec(),
        });
        return (loops, spans);
    }

    // This loop extracts subloops until we have dealt with all the duplicate vertices in loopV
    while !dupes.is_empty() {
        // Find shortest "simple" loop, ie a loop from a bowtie to itself that does not contain
        // any other bowties. This is an independent loop.
        let mut bv = 0;
        let (mut start_i, mut end_i) = (-1, -1);
        let mut bv_shortest = -1;
        let mut shortest = i32::MAX;
        for &d in &dupes {
            bv = d;
            if let Some((s, e)) = is_simple_bowtie_loop(loop_v, &dupes, bv) {
                (start_i, end_i) = (s, e);
                let len = count_span(loop_v, start_i, end_i);
                if len < shortest {
                    bv_shortest = bv;
                    shortest = len;
                }
            } else {
                let (s, e) = bowtie_loop_ends(loop_v, bv);
                (start_i, end_i) = (s, e);
            }
        }
        if bv_shortest == -1 {
            // ConvertToOpenSpan: keep what is left as one span
            let verts: Vec<i32> = loop_v.iter().copied().filter(|&v| v != -1).collect();
            spans.push(EdgeSpan {
                edges: EdgeSpan::vertices_to_edges(mesh, &verts),
                vertices: verts,
                bowtie_vertices: bowties.to_vec(),
            });
            return (loops, spans);
        }
        if bv != bv_shortest {
            bv = bv_shortest;
            // running this just to get start_i and end_i
            if let Some((s, e)) = is_simple_bowtie_loop(loop_v, &dupes, bv) {
                (start_i, end_i) = (s, e);
            }
        }

        let vertices = extract_span(loop_v, start_i, end_i, true);
        loops.push(EdgeLoop {
            edges: EdgeLoop::vertex_loop_to_edge_loop(mesh, &vertices),
            vertices,
            bowtie_vertices: bowties.to_vec(),
        });

        // If there are no more duplicates of this bowtie, we can treat it like a regular
        // vertex now
        if count_in_list(loop_v, bv) < 2 {
            if let Some(pos) = dupes.iter().position(|&d| d == bv) {
                dupes.remove(pos);
            }
        }
    }

    // Should have one loop left that contains duplicates. Extract this as a separate loop
    let left: Vec<i32> = loop_v.iter().copied().filter(|&v| v != -1).collect();
    if !left.is_empty() {
        loops.push(EdgeLoop {
            edges: EdgeLoop::vertex_loop_to_edge_loop(mesh, &left),
            vertices: left,
            bowtie_vertices: bowties.to_vec(),
        });
    }
    (loops, spans)
}

/// The `start_i` / `end_i` C#'s `is_simple_bowtie_loop` leaves in its out parameters when it
/// returns false: the first two positions of the vertex.
fn bowtie_loop_ends(loop_v: &[i32], bowtie_v: i32) -> (i32, i32) {
    let start_i = find_index(loop_v, 0, bowtie_v);
    (start_i, find_index(loop_v, start_i + 1, bowtie_v))
}

/// C# `is_simple_bowtie_loop`: the stretch between the vertex's first two occurrences (either
/// way round the loop) that passes no other bowtie.
fn is_simple_bowtie_loop(loop_v: &[i32], bowties: &[i32], bowtie_v: i32) -> Option<(i32, i32)> {
    let (start_i, end_i) = bowtie_loop_ends(loop_v, bowtie_v);
    if is_simple_path(loop_v, bowties, bowtie_v, start_i, end_i) {
        Some((start_i, end_i))
    } else if is_simple_path(loop_v, bowties, bowtie_v, end_i, start_i) {
        Some((end_i, start_i))
    } else {
        None // not a simple bowtie loop!
    }
}

/// C# `is_simple_path`: walks `i1` up to `i2` (wrapping), skipping removed (-1) entries.
fn is_simple_path(loop_v: &[i32], bowties: &[i32], bowtie_v: i32, i1: i32, i2: i32) -> bool {
    let n = loop_v.len() as i32;
    let mut i = i1;
    while i != i2 {
        let vi = loop_v[i as usize];
        if vi != -1 && vi != bowtie_v && bowties.contains(&vi) {
            return false;
        }
        i = (i + 1) % n;
    }
    true
}

/// C# `extract_span`: the live entries from `i0` up to `i1` (wrapping), marking them removed.
fn extract_span(lp: &mut [i32], i0: i32, i1: i32, mark_invalid: bool) -> Vec<i32> {
    let n = lp.len() as i32;
    let mut out = Vec::new();
    let mut i = i0;
    while i != i1 {
        if lp[i as usize] != -1 {
            out.push(lp[i as usize]);
            if mark_invalid {
                lp[i as usize] = -1;
            }
        }
        i = (i + 1) % n;
    }
    out
}

/// C# `count_span`.
fn count_span(l: &[i32], i0: i32, i1: i32) -> i32 {
    let n = l.len() as i32;
    let mut c = 0;
    let mut i = i0;
    while i != i1 {
        if l[i as usize] != -1 {
            c += 1;
        }
        i = (i + 1) % n;
    }
    c
}

/// C# `find_index`.
fn find_index(lp: &[i32], start: i32, item: i32) -> i32 {
    (start.max(0) as usize..lp.len())
        .find(|&i| lp[i] == item)
        .map_or(-1, |i| i as i32)
}

/// C# `count_in_list`.
fn count_in_list(lp: &[i32], item: i32) -> usize {
    lp.iter().filter(|&&v| v == item).count()
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn to_f32(v: [f64; 3]) -> [f32; 3] {
    v.map(|x| x as f32)
}

/// g3 `Vector3d.Normalize()`.
fn normalize_d(v: [f64; 3]) -> [f64; 3] {
    let length = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if length > EPSILON {
        let inv = 1.0 / length;
        [v[0] * inv, v[1] * inv, v[2] * inv]
    } else {
        [0.0; 3]
    }
}

fn cross_d(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// g3 `DMesh3.GetTriNormal` (`MathUtil.Normal`: both edges normalized before the cross).
pub(crate) fn get_tri_normal(mesh: &DMesh3, tid: i32) -> [f64; 3] {
    let [v1, v2, v3] = mesh.get_tri_vertices(tid as usize);
    let edge1 = normalize_d(sub(v2, v1));
    let edge2 = normalize_d(sub(v3, v2));
    normalize_d(cross_d(edge1, edge2))
}

/// C# `get_vtx_normal`: the normalized sum of the one-ring's triangle normals.
fn get_vtx_normal(mesh: &DMesh3, vid: i32) -> [f64; 3] {
    let mut n = [0.0; 3];
    for ti in mesh.vtx_triangles(vid) {
        let tn = get_tri_normal(mesh, ti);
        n = [n[0] + tn[0], n[1] + tn[1], n[2] + tn[2]];
    }
    normalize_d(n)
}

fn dot_f(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// g3 `Vector3f.Normalize()`.
fn normalize_f(v: [f32; 3]) -> [f32; 3] {
    let length = dot_f(v, v).sqrt();
    if length > EPSILON_F {
        let inv = 1.0f32 / length;
        [v[0] * inv, v[1] * inv, v[2] * inv]
    } else {
        [0.0; 3]
    }
}

/// g3 `MathUtil.PlaneAngleSignedD(Vector3f, Vector3f, Vector3f)`: the signed angle in degrees
/// from `v_from` to `v_to` projected into the plane of `plane_n`.
fn plane_angle_signed_d(v_from: [f32; 3], v_to: [f32; 3], plane_n: [f32; 3]) -> f32 {
    let df = dot_f(v_from, plane_n);
    let dt = dot_f(v_to, plane_n);
    let v_from = normalize_f(std::array::from_fn(|a| v_from[a] - df * plane_n[a]));
    let v_to = normalize_f(std::array::from_fn(|a| v_to[a] - dt * plane_n[a]));
    let c = [
        v_from[1] * v_to[2] - v_from[2] * v_to[1],
        v_from[2] * v_to[0] - v_from[0] * v_to[2],
        v_from[0] * v_to[1] - v_from[1] * v_to[0],
    ];
    if dot_f(c, c) < ZERO_TOLERANCE_F {
        // vectors are parallel
        return if dot_f(v_from, v_to) < 0.0 {
            180.0
        } else {
            0.0
        };
    }
    // C# `Math.Sign(float)` (which throws on NaN; a NaN here can only come from NaN positions)
    let d = dot_f(c, plane_n);
    let sign = if d > 0.0 {
        1.0f32
    } else if d < 0.0 {
        -1.0
    } else {
        0.0
    };
    // C# `Vector3f.AngleD`: the dot clamped, acos in double, then degrees, narrowed
    let fdot = dot_f(v_from, v_to).clamp(-1.0, 1.0);
    let angle = ((fdot as f64).acos() * (180.0 / std::f64::consts::PI)) as f32;
    sign * angle
}
