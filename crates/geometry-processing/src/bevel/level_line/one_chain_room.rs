//! The room of a pass of ONE chain as the band gap's build reads it, on the mesh tagged into its
//! two sides ([`read_in_the_build`], split from `band_gap_build_within.rs`), with the angle's
//! setbacks per unit of size. The build refuses the size exactly when it stands above the room, so
//! the room has to be read off the band the build cuts. A pass of several chains reads the pick's
//! room (`band_room_pair_room_pick`, its pair and corner readings).

use std::cell::RefCell;
use std::rc::Rc;

use solidmesh_math::Vector3;

use super::band_gap_setback_fields::PassSetbacks;
use super::band_room;
use super::{end_face_sense, BandProfile, ChainField, LevelLineMesh, RoomSource, WorkError};

/// The room of chain 0 read on `mesh`, the input tagged into its two sides, with the angle's
/// setbacks per unit of size: the level line's own reading, its convex end faces read against the
/// chain without their rays (`band_room::room_of_cut_mesh` says why).
#[allow(clippy::too_many_arguments)]
pub fn read_in_the_build(
    positions: &[Vector3],
    indices: &[(i32, i32, i32)],
    chain: &[i32],
    closed: bool,
    mesh: &Rc<RefCell<LevelLineMesh>>,
    setbacks: &PassSetbacks,
    snap: f64,
    seam_epsilon: f64,
    profile: BandProfile,
    fillet_segments: i32,
) -> Result<(f64, RoomSource), WorkError> {
    let end_rays = setbacks.end_rays[0];
    let convex_ends = end_face_sense::convex_end_faces(positions, indices, chain, end_rays);
    let without_convex_rays = (
        end_rays.0 && convex_ends.0.is_none(),
        end_rays.1 && convex_ends.1.is_none(),
    );
    let units = &setbacks.unit_setbacks[0];
    let field_at = |s: f64| {
        setbacks
            .field_of(0, s)
            .expect("the chain's polyline built a field at the size asked")
    };
    let field_without_convex_rays = |s: f64| {
        let scaled: Vec<f64> = units.iter().map(|u| u * s).collect();
        ChainField::from_polyline_bounded(
            &setbacks.chain_points[0],
            closed,
            &scaled,
            setbacks.sharp_turn_degrees(),
            without_convex_rays,
            setbacks.end_ray_bounds[0],
        )
        .expect("the chain's polyline built a field at the size asked")
    };
    let present = band_room::present(convex_ends);
    band_room::room_of_cut_mesh(
        mesh,
        &setbacks.chain_edges[0],
        &field_at,
        units,
        closed,
        snap,
        seam_epsilon,
        profile,
        fillet_segments,
        Some(&present),
        Some(&field_without_convex_rays),
    )
}
