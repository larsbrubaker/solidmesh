//! The sweep for a run with a FINITE runout: the selected span at full size, and each tapered end
//! drawn down to a single tip vertex over a length the caller measured off the part
//! ([`try_create`]). The run and its two lengths are a [`BevelRunTaper`]; the segments and the
//! per-vertex rings are the plain sweep's
//! ([`super::bevel_chain_sweep_sections::try_build_segments`],
//! [`super::bevel_chain_sweep_rings::try_build_rings`]).
//!
//! It shares the plain sweep's segments and its per-vertex rings and then throws its ring
//! TOPOLOGY away, laying out its own stations by physical distance along the run and reading the
//! cross section at each one by interpolating between the rings at the two ends of whichever
//! segment the station lands on. That is why it wants the rings one-per-vertex: it indexes them BY
//! SEGMENT, and the passes that renumber the array for the plain sweep's mesh would leave it
//! reading the wrong ring or running off the end.

use polygon_mesh::csg::boolean_processing::{classify_boolean_operand, BooleanOperandVerdict};
use polygon_mesh::Mesh;
use vector_math::{Matrix4X4, Vector3, Vector3Float};

use super::bevel_chain_sweep_rings::try_build_rings;
use super::bevel_chain_sweep_sections::try_build_segments;
use super::{bevel_edge_tool_builder, bevel_profile_polygon};
use super::{
    BevelAdvisory, BevelAdvisoryCode, BevelChainEnds, BevelEndCap, BevelRunTaper, BevelSweptSection,
};

/// A finite runout keeps the selected span full size and closes each tapered end with one tip
/// vertex.
///
/// `back_depth` is how far past each face a section's cleared back may run, or zero for the size
/// itself - the same limit `bevel_chain_sweep::try_create` takes, and honoured here for the same
/// reason: a run whose mitre would not fit at full depth is handed a shallower one, and a tapered
/// run is no less entitled to it than a plain sweep is. (Callers normally pass `section` the tool,
/// `back_depth` zero.)
#[allow(clippy::too_many_arguments)]
pub fn try_create(
    taper: &BevelRunTaper,
    ends: &BevelChainEnds,
    size: f64,
    segments: i32,
    round_profile: bool,
    section: BevelSweptSection,
    back_depth: f64,
) -> Result<Mesh, BevelAdvisory> {
    bevel_edge_tool_builder::try_validate_size(size)?;
    let mut swept = try_build_segments(
        &taper.edges,
        size,
        segments,
        round_profile,
        section,
        back_depth,
        None,
    )?;
    let point_count = swept[0].profile.len();
    let capped = BevelChainEnds {
        start_maximum_overshoot: if taper.start_length > 0.0 {
            f64::INFINITY
        } else {
            ends.start_maximum_overshoot
        },
        end_maximum_overshoot: if taper.end_length > 0.0 {
            f64::INFINITY
        } else {
            ends.end_maximum_overshoot
        },
        ..BevelChainEnds::new(
            if taper.start_length > 0.0 {
                BevelEndCap::default()
            } else {
                ends.start.clone()
            },
            if taper.end_length > 0.0 {
                BevelEndCap::default()
            } else {
                ends.end.clone()
            },
        )
    };
    let mut full = try_build_rings(
        &mut swept,
        false,
        &capped,
        size,
        point_count,
        0.0,
        0.0,
        section,
        None,
    )?;
    let last_segment = swept.len() - 1;
    if taper.start_length > 0.0 {
        full[0] = (0..point_count)
            .map(|p| swept[0].from + swept[0].offset(p))
            .collect();
    }
    if taper.end_length > 0.0 {
        let last_ring = full.len() - 1;
        full[last_ring] = (0..point_count)
            .map(|p| swept[last_segment].to + swept[last_segment].offset(p))
            .collect();
    }
    let mut distances = vec![0.0; swept.len() + 1];
    for i in 0..swept.len() {
        distances[i + 1] = distances[i] + swept[i].length;
    }
    let selected_start = distances[taper.selected_index as usize];
    let selected_end = distances[(taper.selected_index + taper.selected_count) as usize];
    let first = selected_start - taper.start_length;
    let last = selected_end + taper.end_length;
    // A sorted set: the facet joints inside the span, both endpoints and the taper stations, in
    // ascending order with exact duplicates dropped.
    let mut stations: Vec<f64> = distances
        .iter()
        .copied()
        .filter(|&d| d >= first && d <= last)
        .collect();
    stations.push(first);
    stations.push(last);
    // Physical spacing, independent of how many facets happen to describe the wall.
    for i in 1..16 {
        if taper.start_length > 0.0 {
            stations.push(first + taper.start_length * i as f64 / 16.0);
        }
        if taper.end_length > 0.0 {
            stations.push(selected_end + taper.end_length * i as f64 / 16.0);
        }
    }
    stations.sort_by(|a, b| a.total_cmp(b));
    stations.dedup();
    let mut mesh = Mesh::new();
    let mut rings: Vec<Vec<i32>> = Vec::new();
    // A facet joint and a physical endpoint can differ only by rounding. Keep the exact
    // endpoints and coalesce nearby interior stations before creating any mesh vertices.
    let station_tolerance = size.max(last - first) * 1e-10;
    let mut distinct_stations = vec![first];
    for &station in &stations {
        if station - distinct_stations[distinct_stations.len() - 1] > station_tolerance
            && last - station > station_tolerance
        {
            distinct_stations.push(station);
        }
    }
    distinct_stations.push(last);
    for &distance in &distinct_stations {
        // A start tip can sit a rounding error before the run's first joint (the walk and the sweep
        // add the same chords in opposite orders) and find no segment. It is on the run by
        // construction, so it is read on the first segment, where `t` comes out a hair below zero
        // and the tip lands where it was meant to.
        let found = distances.iter().rposition(|&d| d <= distance).unwrap_or(0);
        let segment = last_segment.min(found);
        let t = (distance - distances[segment]) / swept[segment].length;
        let center = swept[segment].from + (swept[segment].to - swept[segment].from) * t;
        let mut factor = 1.0;
        if distance < selected_start {
            factor = (distance - first) / taper.start_length;
        }
        if distance > selected_end {
            factor = (last - distance) / taper.end_length;
        }
        factor = factor.clamp(0.0, 1.0);
        factor = factor * factor * (3.0 - 2.0 * factor);
        let is_tip = (taper.start_length > 0.0 && distance == first)
            || (taper.end_length > 0.0 && distance == last);
        let mut ring = vec![0; if is_tip { 1 } else { point_count }];
        for (p, slot) in ring.iter_mut().enumerate() {
            let position: Vector3 = center
                + ((full[segment][p] - swept[segment].from) * (1.0 - t)
                    + (full[segment + 1][p] - swept[segment].to) * t)
                    * factor;
            if !position.x.is_finite() || !position.y.is_finite() || !position.z.is_finite() {
                return Err(BevelAdvisory::new(
                    BevelAdvisoryCode::ToolNotUsableSolid,
                    vec![],
                ));
            }
            *slot = mesh.vertices.len() as i32;
            mesh.vertices.push(Vector3Float::from_vector3(position));
        }
        rings.push(ring);
    }
    for i in 0..rings.len() - 1 {
        let back = &rings[i];
        let front = &rings[i + 1];
        for p in 0..point_count {
            let next = (p + 1) % point_count;
            if back.len() == 1 {
                mesh.faces
                    .add(back[0], front[next], front[p], &mesh.vertices);
            } else if front.len() == 1 {
                mesh.faces
                    .add(back[p], back[next], front[0], &mesh.vertices);
            } else {
                mesh.faces
                    .add(back[p], back[next], front[next], &mesh.vertices);
                mesh.faces
                    .add(back[p], front[next], front[p], &mesh.vertices);
            }
        }
    }
    for start in [true, false] {
        let ring = if start {
            &rings[0]
        } else {
            &rings[rings.len() - 1]
        };
        if ring.len() == 1 {
            continue;
        }
        let profile = if start {
            &swept[0].profile
        } else {
            &swept[last_segment].profile
        };
        let cap = bevel_profile_polygon::try_triangulate(profile)
            .ok_or_else(|| BevelAdvisory::new(BevelAdvisoryCode::ToolCouldNotClose, vec![]))?;
        for i in (0..cap.len()).step_by(3) {
            let (second, third) = if start { (2, 1) } else { (1, 2) };
            mesh.faces.add(
                ring[cap[i] as usize],
                ring[cap[i + second] as usize],
                ring[cap[i + third] as usize],
                &mesh.vertices,
            );
        }
    }
    if classify_boolean_operand(&mesh, Matrix4X4::IDENTITY, false) != BooleanOperandVerdict::Clean {
        return Err(BevelAdvisory::new(
            BevelAdvisoryCode::ToolNotUsableSolid,
            vec![],
        ));
    }
    Ok(mesh)
}
