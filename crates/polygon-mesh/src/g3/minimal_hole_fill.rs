//! geometry3Sharp `mesh_ops/MinimalHoleFill.cs`: fills a boundary loop with a "minimal"
//! surface. A simple fan fill (`simple_hole_filler`) is extracted as a submesh
//! (`region_operator`), remeshed to a working density (`remesher_pro`), collapsed back down to
//! a minimal mesh, then improved by edge flips that flatten it, reduce Gaussian curvature
//! (recovering sharp edges) and improve triangle aspect ratios, before being spliced back.
//!
//! C#'s `HashSet<int>` edge sets are only added to and cleared, so they enumerate in
//! insertion order; [`OrderedSet`] keeps that order, which decides the flip order.

use std::collections::{HashMap, HashSet};

use super::dmesh3::{DMesh3, G3Exception, MeshResult, INVALID_ID};
use super::dmesh3_queries::find_tri_index;
use super::edge_loop::EdgeLoop;
use super::mesh_iterators::{boundary_edge_vertices, interior_vertices};
use super::mesh_queries::{edge_length_stats, edge_length_stats_from_edges};
use super::mesh_util::{
    check_if_collapse_creates_flip, check_if_edge_flip_creates_flip, get_edge_flip_normals,
    get_edge_flip_tris,
};
use super::region_operator::RegionOperator;
use super::remesher_pro::RemesherPro;
use super::simple_hole_filler::SimpleHoleFiller;
use super::vector3d::{aspect_ratio, distance, dot, EPSILONF, TWO_PI, ZERO_TOLERANCEF};

/// A C# `HashSet<int>` that is only added to and cleared: enumerates in insertion order.
#[derive(Debug, Default)]
struct OrderedSet {
    order: Vec<i32>,
    set: HashSet<i32>,
}

impl OrderedSet {
    fn from_iter(items: impl IntoIterator<Item = i32>) -> Self {
        let mut s = Self::default();
        for i in items {
            s.add(i);
        }
        s
    }

    fn add(&mut self, v: i32) {
        if self.set.insert(v) {
            self.order.push(v);
        }
    }

    fn clear(&mut self) {
        self.order.clear();
        self.set.clear();
    }

    fn is_empty(&self) -> bool {
        self.order.is_empty()
    }
}

/// g3 `MinimalHoleFill`.
#[derive(Debug, Clone)]
pub(crate) struct MinimalHoleFill {
    pub(crate) ignore_boundary_triangles: bool,
    pub(crate) optimize_developability: bool,
    pub(crate) optimize_triangles: bool,
    pub(crate) developability_tolerance: f64,
    /// C# `FillVertices`: final fill vertices (should be empty?)
    pub(crate) fill_vertices: Vec<i32>,
    /// C# `FillTriangles`: final fill triangles.
    pub(crate) fill_triangles: Vec<i32>,
}

impl Default for MinimalHoleFill {
    fn default() -> Self {
        Self {
            ignore_boundary_triangles: false,
            optimize_developability: true,
            optimize_triangles: true,
            developability_tolerance: 0.0001,
            fill_vertices: Vec::new(),
            fill_triangles: Vec::new(),
        }
    }
}

/// The per-fill state C# keeps in fields: the angle each boundary vertex already has in the
/// base mesh outside the fill, and the cached Gaussian curvature per fill vertex.
struct Curvature {
    exterior_angle_sums: HashMap<i32, f64>,
    curvatures: Vec<f64>,
}

impl Curvature {
    /// C# `compute_gauss_curvature`.
    fn compute_gauss_curvature(&self, fillmesh: &DMesh3, vid: i32) -> f64 {
        let mut angle_sum = self.exterior_angle_sums.get(&vid).copied().unwrap_or(0.0);
        for tid in fillmesh.vtx_triangles(vid) {
            let et = fillmesh.get_triangle(tid);
            let idx = find_tri_index(vid, &et);
            angle_sum += fillmesh.get_tri_internal_angle_r(tid, idx);
        }
        angle_sum - TWO_PI
    }

    /// C# `update_curvature`.
    fn update_curvature(&mut self, fillmesh: &DMesh3, vid: i32) {
        self.curvatures[vid as usize] = self.compute_gauss_curvature(fillmesh, vid);
    }

    /// C# `curvature_metric_cached`.
    fn metric_cached(&self, a: i32, b: i32, c: i32, d: i32) -> f64 {
        let defect_a = self.curvatures[a as usize];
        let defect_b = self.curvatures[b as usize];
        let defect_c = self.curvatures[c as usize];
        let defect_d = self.curvatures[d as usize];
        defect_a.abs() + defect_b.abs() + defect_c.abs() + defect_d.abs()
    }

    /// C# `curvature_metric_eval`.
    fn metric_eval(&self, fillmesh: &DMesh3, a: i32, b: i32, c: i32, d: i32) -> f64 {
        let defect_a = self.compute_gauss_curvature(fillmesh, a);
        let defect_b = self.compute_gauss_curvature(fillmesh, b);
        let defect_c = self.compute_gauss_curvature(fillmesh, c);
        let defect_d = self.compute_gauss_curvature(fillmesh, d);
        defect_a.abs() + defect_b.abs() + defect_c.abs() + defect_d.abs()
    }

    fn update_quad(&mut self, fillmesh: &DMesh3, ev: [i32; 2], ov: [i32; 2]) {
        self.update_curvature(fillmesh, ev[0]);
        self.update_curvature(fillmesh, ev[1]);
        self.update_curvature(fillmesh, ov[0]);
        self.update_curvature(fillmesh, ov[1]);
    }
}

/// C# `add_all_edges`: the edges of both triangles on `ei`.
fn add_all_edges(fillmesh: &DMesh3, ei: i32, edge_set: &mut OrderedSet) {
    let et = fillmesh.get_edge_t(ei);
    for t in et {
        for e in fillmesh.get_tri_edges(t) {
            edge_set.add(e);
        }
    }
}

/// C# `aspect_metric`: the flipped pair's total aspect deviation over the current pair's.
fn aspect_metric(fillmesh: &DMesh3, eid: i32) -> f64 {
    let (ta, tb, ota, otb) = get_edge_flip_tris(fillmesh, eid);
    let aspect = |t: [i32; 3]| {
        aspect_ratio(
            fillmesh.vertex(t[0]),
            fillmesh.vertex(t[1]),
            fillmesh.vertex(t[2]),
        )
    };
    let aspect_a = aspect(ta);
    let aspect_b = aspect(tb);
    let aspect_c = aspect(ota);
    let aspect_d = aspect(otb);
    let metric_ab = (aspect_a - 1.0).abs() + (aspect_b - 1.0).abs();
    let metric_cd = (aspect_c - 1.0).abs() + (aspect_d - 1.0).abs();
    metric_cd / metric_ab
}

/// C# `remove_remaining_interior_verts`: sometimes, for some reason, we have a remaining
/// interior vertex (have only ever seen one?). Try to force removal of such vertices, even if
/// it makes ugly mesh.
fn remove_remaining_interior_verts(fillmesh: &mut DMesh3) {
    // C#'s HashSet: built once, then only removed from, so it keeps the build order.
    let mut interiorv = interior_vertices(fillmesh);
    let mut prev_count = 0;
    while !interiorv.is_empty() && interiorv.len() != prev_count {
        prev_count = interiorv.len();
        let curv = interiorv.clone();
        for vid in curv {
            for e in fillmesh.vtx_edges(vid) {
                let ev = fillmesh.get_edge_v(e);
                let otherv = if ev[0] == vid { ev[1] } else { ev[0] };
                let (result, _) = fillmesh.collapse_edge(otherv, vid);
                if result == MeshResult::Ok {
                    break;
                }
            }
            if !fillmesh.is_vertex(vid) {
                interiorv.retain(|&v| v != vid);
            }
        }
    }
    // C# `Util.gBreakToDebugger()` when some remain: nothing in a release build.
}

impl MinimalHoleFill {
    /// C# `Apply()` for `new MinimalHoleFill(mesh, fillLoop)`. `Err` carries an exception C#
    /// would throw out of the fill (a broken-topology flip, a refused back-out).
    pub(crate) fn apply(
        &mut self,
        mesh: &mut DMesh3,
        fill_loop: &EdgeLoop,
    ) -> Result<bool, G3Exception> {
        // do a simple fill
        let mut simplefill = SimpleHoleFiller::new(fill_loop);
        let fill_gid = mesh.allocate_triangle_group();
        let ok = simplefill.fill(mesh, fill_gid)?;
        if !ok {
            return Ok(false);
        }
        let simple_tris = simplefill.new_triangles.clone().unwrap_or_default();

        if fill_loop.vertices.len() <= 3 {
            self.fill_triangles = simple_tris;
            self.fill_vertices = Vec::new();
            return Ok(true);
        }

        // try to guess a reasonable edge length that will give us enough geometry to work with
        // in simplify pass (C# measures the loop on the base mesh after extracting the region,
        // which does not change it)
        let loop_stats = edge_length_stats_from_edges(mesh, fill_loop.edges.iter().copied());

        // extract the simple fill mesh as a submesh, via RegionOperator, so we can backsub later
        let intial_fill_tris: HashSet<i32> = simple_tris.iter().copied().collect();
        let mut regionop = RegionOperator::new(mesh, &simple_tris)?;

        // for each boundary vertex, compute the exterior angle sum
        // we will use this to compute gaussian curvature later
        let boundaryv: HashSet<i32> = boundary_edge_vertices(&regionop.region.sub_mesh)
            .into_iter()
            .collect();
        let mut curv = Curvature {
            exterior_angle_sums: HashMap::new(),
            curvatures: Vec::new(),
        };
        if !self.ignore_boundary_triangles {
            for &sub_vid in &boundaryv {
                let mut angle_sum = 0.0;
                let base_vid = regionop.region.map_vertex_to_base_mesh(sub_vid);
                for tid in regionop.base_mesh.vtx_triangles(base_vid) {
                    if !intial_fill_tris.contains(&tid) {
                        let et = regionop.base_mesh.get_triangle(tid);
                        let idx = find_tri_index(base_vid, &et);
                        angle_sum += regionop.base_mesh.get_tri_internal_angle_r(tid, idx);
                    }
                }
                curv.exterior_angle_sums.insert(sub_vid, angle_sum);
            }
        }

        let fill_stats = edge_length_stats(&regionop.region.sub_mesh, 0)?;
        let mut remesh_target_len = loop_stats.avg;
        if fill_stats.max / remesh_target_len > 10.0 {
            remesh_target_len = fill_stats.max / 10.0;
        }

        // remesh up to target edge length, ideally gives us some triangles to work with
        {
            let mut remesh1 = RemesherPro::new(&mut regionop.region.sub_mesh);
            remesh1.remesher.smooth_speed_t = 1.0;
            remesh1.fix_all_boundary_edges();
            remesh1.remesher.set_target_edge_length(remesh_target_len);
            remesh1.fastest_remesh(25, true)?;
        }

        let fillmesh = &mut regionop.region.sub_mesh;
        collapse_and_flip_passes(fillmesh, &boundaryv)?;

        // Sometimes, for some reason, we have a remaining interior vertex (have only ever seen
        // one?) Try to force removal of such vertices, even if it makes ugly mesh
        remove_remaining_interior_verts(fillmesh);

        // enable/disable passes.
        let do_curvature_pass = self.optimize_developability;
        let do_area_pass = self.optimize_developability && self.optimize_triangles;

        flatter_passes(fillmesh)?;

        if do_curvature_pass {
            curv.curvatures = vec![0.0; fillmesh.max_vertex_id() as usize];
            for vid in fillmesh.vertex_indices().collect::<Vec<_>>() {
                curv.update_curvature(fillmesh, vid);
            }
            curvature_passes(fillmesh, &mut curv)?;
        }

        if do_area_pass {
            self.area_passes(fillmesh, &mut curv)?;
        }

        regionop.back_propropagate(true)?;
        self.fill_triangles = regionop.current_base_triangles().to_vec();
        self.fill_vertices = regionop.current_base_interior_vertices()?;

        Ok(true)
    }

    /// In this final pass, we try to improve triangle quality. We flip if the flipped
    /// triangles have better total aspect ratio, and the curvature doesn't change **too**
    /// much. The `developability_tolerance` determines what is "too much" curvature change.
    fn area_passes(&self, fillmesh: &mut DMesh3, curv: &mut Curvature) -> Result<(), G3Exception> {
        let mut remaining_edges = OrderedSet::from_iter(fillmesh.edge_indices());
        let mut updated_edges = OrderedSet::default();
        let mut area_passes = 0;
        while !remaining_edges.is_empty() && area_passes < 20 {
            area_passes += 1;
            for &ei in &remaining_edges.order {
                if fillmesh.is_boundary_edge(ei) {
                    continue;
                }

                let ev = fillmesh.get_edge_v(ei);
                let ov = fillmesh.get_edge_opposing_v(ei);

                let find_other = fillmesh.find_edge(ov[0], ov[1]);
                if find_other != INVALID_ID {
                    continue;
                }

                let total_curv_cur = curv.metric_cached(ev[0], ev[1], ov[0], ov[1]);

                let a = aspect_metric(fillmesh, ei);
                if a > 1.0 {
                    continue;
                }

                let (result, _) = fillmesh.flip_edge(ei)?;
                if result != MeshResult::Ok {
                    continue;
                }

                let total_curv_flip = curv.metric_eval(fillmesh, ev[0], ev[1], ov[0], ov[1]);

                let keep_flip =
                    (total_curv_cur - total_curv_flip).abs() < self.developability_tolerance;
                if !keep_flip {
                    fillmesh.flip_edge(ei)?;
                } else {
                    curv.update_quad(fillmesh, ev, ov);
                    add_all_edges(fillmesh, ei, &mut updated_edges);
                }
            }
            std::mem::swap(&mut remaining_edges, &mut updated_edges);
            updated_edges.clear();
        }
        Ok(())
    }
}

/// First round: collapse to minimal mesh, while flipping to try to get to ballpark minimal
/// mesh. We stop these passes as soon as we have done two rounds where we couldn't do another
/// collapse.
///
/// This is the most unstable part of the algorithm because there are strong ordering effects.
/// maybe we could sort the edges somehow??
fn collapse_and_flip_passes(
    fillmesh: &mut DMesh3,
    boundaryv: &HashSet<i32>,
) -> Result<(), G3Exception> {
    let mut zero_collapse_passes = 0;
    let mut collapse_passes = 0;
    loop {
        let go_on = collapse_passes < 20;
        collapse_passes += 1;
        if !(go_on && zero_collapse_passes < 2) {
            break;
        }

        // collapse pass
        let ne = fillmesh.max_edge_id();
        let mut collapses = 0;
        for ei in 0..ne {
            if !fillmesh.is_edge(ei) || fillmesh.is_boundary_edge(ei) {
                continue;
            }

            let ev = fillmesh.get_edge_v(ei);
            let a_bdry = boundaryv.contains(&ev[0]);
            let b_bdry = boundaryv.contains(&ev[1]);
            if a_bdry && b_bdry {
                continue;
            }

            let keepv = if a_bdry { ev[0] } else { ev[1] };
            let otherv = if keepv == ev[0] { ev[1] } else { ev[0] };
            let newv = fillmesh.vertex(keepv);
            if check_if_collapse_creates_flip(fillmesh, ei, newv) {
                continue;
            }

            let (result, _) = fillmesh.collapse_edge(keepv, otherv);
            if result == MeshResult::Ok {
                collapses += 1;
            }
        }
        if collapses == 0 {
            zero_collapse_passes += 1;
        } else {
            zero_collapse_passes = 0;
        }

        // flip pass. we flip in these cases:
        //  1) if angle between current triangles is too small (slightly more than 90 degrees, currently)
        //  2) if angle between flipped triangles is smaller than between current triangles
        //  3) if flipped edge length is shorter *and* such a flip won't flip the normal
        let ne = fillmesh.max_edge_id();
        for ei in 0..ne {
            if !fillmesh.is_edge(ei) || fillmesh.is_boundary_edge(ei) {
                continue;
            }

            let mut do_flip = false;

            let ev = fillmesh.get_edge_v(ei);
            let (n1, n2, on1, on2) = get_edge_flip_normals(fillmesh, ei);
            let dot_cur = dot(n1, n2);
            let dot_flip = dot(on1, on2);
            if dot(n1, n2) < 0.1 || dot_flip > dot_cur + EPSILONF {
                do_flip = true;
            }

            if !do_flip {
                let otherv = fillmesh.get_edge_opposing_v(ei);
                let len_e = distance(fillmesh.vertex(ev[0]), fillmesh.vertex(ev[1]));
                let len_flip = distance(fillmesh.vertex(otherv[0]), fillmesh.vertex(otherv[1]));
                if len_flip < len_e && !check_if_edge_flip_creates_flip(fillmesh, ei, 0.0) {
                    do_flip = true;
                }
            }

            if do_flip {
                fillmesh.flip_edge(ei)?;
            }
        }
    }
    Ok(())
}

/// In this pass we repeat the flipping iterations from the previous pass.
///
/// Note that because of the always-flip-if-dot-is-small case (commented), this pass will
/// frequently not converge, as some number of edges will be able to flip back and forth
/// (because neither has large enough dot). This is not ideal, but also, if we remove this
/// behavior, then we generally get worse fills. This case basically introduces a sort of
/// randomization factor that lets us escape local minima...
fn flatter_passes(fillmesh: &mut DMesh3) -> Result<(), G3Exception> {
    let mut remaining_edges = OrderedSet::from_iter(fillmesh.edge_indices());
    let mut updated_edges = OrderedSet::default();

    let mut flatter_passes = 0;
    let mut zero_flips_passes = 0;
    loop {
        let go_on = flatter_passes < 40;
        flatter_passes += 1;
        if !(go_on && zero_flips_passes < 2 && !remaining_edges.is_empty()) {
            break;
        }
        zero_flips_passes += 1;
        for &ei in &remaining_edges.order {
            if fillmesh.is_boundary_edge(ei) {
                continue;
            }

            let mut do_flip = false;

            let (n1, n2, on1, on2) = get_edge_flip_normals(fillmesh, ei);
            let dot_cur = dot(n1, n2);
            let dot_flip = dot(on1, on2);
            if flatter_passes < 20 && dot_cur < 0.1 {
                // this check causes oscillatory behavior
                do_flip = true;
            }

            if dot_flip > dot_cur + EPSILONF {
                do_flip = true;
            }

            if do_flip {
                let (result, _) = fillmesh.flip_edge(ei)?;
                if result == MeshResult::Ok {
                    zero_flips_passes = 0;
                    add_all_edges(fillmesh, ei, &mut updated_edges);
                }
            }
        }

        std::mem::swap(&mut remaining_edges, &mut updated_edges);
        updated_edges.clear();
    }
    Ok(())
}

/// In this pass we try to minimize gaussian curvature at all the vertices. This will recover
/// sharp edges, etc, and do lots of good stuff. However, this pass will not make much progress
/// if we are not already relatively close to a minimal mesh, so it really relies on the
/// previous passes getting us in the ballpark.
fn curvature_passes(fillmesh: &mut DMesh3, curv: &mut Curvature) -> Result<(), G3Exception> {
    let mut remaining_edges = OrderedSet::from_iter(fillmesh.edge_indices());
    let mut updated_edges = OrderedSet::default();
    let mut curvature_passes = 0;
    loop {
        let go_on = curvature_passes < 40;
        curvature_passes += 1;
        if !(go_on && !remaining_edges.is_empty()) {
            break;
        }
        for &ei in &remaining_edges.order {
            if fillmesh.is_boundary_edge(ei) {
                continue;
            }

            let ev = fillmesh.get_edge_v(ei);
            let ov = fillmesh.get_edge_opposing_v(ei);

            let find_other = fillmesh.find_edge(ov[0], ov[1]);
            if find_other != INVALID_ID {
                continue;
            }

            let total_curv_cur = curv.metric_cached(ev[0], ev[1], ov[0], ov[1]);
            if total_curv_cur < ZERO_TOLERANCEF {
                continue;
            }

            let (result, _) = fillmesh.flip_edge(ei)?;
            if result != MeshResult::Ok {
                continue;
            }

            let total_curv_flip = curv.metric_eval(fillmesh, ev[0], ev[1], ov[0], ov[1]);

            let keep_flip = total_curv_flip < total_curv_cur - ZERO_TOLERANCEF;
            if !keep_flip {
                fillmesh.flip_edge(ei)?;
            } else {
                curv.update_quad(fillmesh, ev, ov);
                add_all_edges(fillmesh, ei, &mut updated_edges);
            }
        }
        std::mem::swap(&mut remaining_edges, &mut updated_edges);
        updated_edges.clear();
    }
    Ok(())
}
