//! The mixed corner over a chorded shared
//! face at a size whose bead RESTS ON A CREASE of it - the profile is tangent to neither facet's
//! plane, so no chord's cylinder stands exactly `2r` from the bead's axis and the point the two
//! cylinders touch at is on the crease line rather than on a facet.
//!
//! What holds is what the corner's frame always said: the touch point is where the two tangent
//! lines in the shared face cross, and on a crease landing the bead's tangent line there is the
//! crease. The cut is the chord whose cylinder comes nearest the bead's, short of `2r` by the
//! crease's deficit (0.0013 at `r = 1.5`, 0.0075 at `r = 4.5`), against a chord sagitta of 0.0308.

use std::rc::Rc;
use std::sync::Arc;

use solidmesh::bevel::bevel_chain_linker::{self, BevelSweptChain};
use solidmesh::bevel::surface::{
    surface_bevel_builder, surface_mesh_closure, surface_mixed_corner_cut_edge,
    surface_mixed_corner_read_off_the_edge_plans, surface_mixed_corner_ring_schedule,
    surface_mixed_corner_run_out, SurfaceBevelPlan, SurfaceBuildOptions, SurfaceEdgePlan,
    SurfaceLine, SurfaceMixedCorner, SurfaceMixedCornerWall, SurfaceVertexPool,
};
use solidmesh::topology::{EdgeConvexity, FaceRegionCache};
use solidmesh_math::{math_helper, Matrix4X4, Vector3};
use solidmesh_mesh::csg::boolean_processing::{classify_boolean_operand, BooleanOperandVerdict};
use solidmesh_mesh::{CancelToken, Mesh};

use crate::lobed_prism_meshes;
use crate::mesh_measurements::shell_count;

/// The first size whose roll rests on a crease at this seam: the bead's foot stands on the vertex
/// the part chord and the first whole chord share, and the cut is the part chord itself.
const RESTS_ON_THE_FIRST_CREASE: f64 = 1.5;

/// And a larger one, a crease further round: the part chord is swallowed.
const RESTS_ON_THE_SECOND_CREASE: f64 = 4.5;

fn assert_within(actual: f64, expected: f64, tolerance: f64, because: &str) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} is not {expected} within {tolerance}: {because}"
    );
}

/// The roll says so, on the section: the section names the crease and the line the bead touches
/// the wall along, which is that vertex carried along the seam.
fn the_section_names_the_crease_the_roll_rests_on(radius: f64, crease_x: f64, crease_y: f64) {
    let read = the_walls_rim(radius);
    let section = read
        .bead
        .wall_section
        .clone()
        .expect("the bead rolls along the barrel, so it has a section to report");

    assert!(
        section.rests_on_the_crease,
        "at {radius} the circle stands on the crease at the near end of facet {}",
        section.touched_facet
    );

    let crease = section.crease_line;

    assert!(
        crease.distance_to(Vector3::new(crease_x, crease_y, lobed_prism_meshes::HEIGHT)) < 1e-3,
        "the crease runs through the vertex the two facets share - instead {:?}",
        crease.point
    );

    assert_within(
        crease.direction.get_normal().dot(Vector3::UNIT_Z).abs(),
        1.0,
        1e-6,
        "along the wall's own ruling, which here is the seam",
    );

    // And it is where the ball really touches: a radius out of its axis.
    assert_within(
        read.bead.center_line.distance_to(crease.point),
        radius,
        1e-6,
        "the ball rests ON the crease, a radius from it",
    );
}

#[test]
fn the_section_names_the_crease_the_roll_rests_on_1_5() {
    the_section_names_the_crease_the_roll_rests_on(RESTS_ON_THE_FIRST_CREASE, 29.511, -1.09);
}

#[test]
fn the_section_names_the_crease_the_roll_rests_on_4_5() {
    the_section_names_the_crease_the_roll_rests_on(RESTS_ON_THE_SECOND_CREASE, 28.91, -2.54);
}

/// So no chord stands `2r` from the axis at all, and the cut is the nearest one there is, with
/// everything between it and the corner swallowed.
#[allow(clippy::too_many_arguments)]
fn the_cut_is_the_nearest_chord_because_none_stands_two_radii_out(
    radius: f64,
    deficit: f64,
    swallowed_count: usize,
    cut_from_x: f64,
    cut_from_y: f64,
    cut_to_x: f64,
    cut_to_y: f64,
) {
    let read = the_walls_rim(radius);
    let mut nearest = f64::MAX;

    for plan in read
        .planned
        .iter()
        .filter(|plan| plan.convexity() == EdgeConvexity::Convex && plan.has_center_line)
    {
        nearest =
            nearest.min((apart(plan.center_line, read.bead.center_line) - (2.0 * radius)).abs());
    }

    assert!(
        nearest > read.near_enough,
        "the nearest chord stands {nearest} off 2r, and the ruler is {}",
        read.near_enough
    );

    assert_within(nearest, deficit, 1e-5, "which is the crease's own deficit");

    assert_eq!(
        read.swallowed.len(),
        swallowed_count,
        "chords that went under the blend"
    );

    let cut = &read.planned[read.cut_index as usize];

    assert!(
        (cut.from() - Vector3::new(cut_from_x, cut_from_y, lobed_prism_meshes::HEIGHT)).length()
            < 1e-3,
        "the cut is the chord beside the crease - instead {:?} -> {:?}",
        cut.from(),
        cut.to()
    );
    assert!(
        (cut.to() - Vector3::new(cut_to_x, cut_to_y, lobed_prism_meshes::HEIGHT)).length() < 1e-3,
        "instead {:?} -> {:?}",
        cut.from(),
        cut.to()
    );

    // And it IS the nearest: no chord of the run stands nearer 2r than the one the walk chose.
    assert_within(
        (apart(cut.center_line, read.bead.center_line) - (2.0 * radius)).abs(),
        nearest,
        1e-9,
        "the walk takes the local maximum of its own measure, which is the nearest the two cylinders come",
    );
}

#[test]
fn the_cut_is_the_nearest_chord_because_none_stands_two_radii_out_1_5() {
    the_cut_is_the_nearest_chord_because_none_stands_two_radii_out(
        RESTS_ON_THE_FIRST_CREASE,
        0.001312,
        0,
        29.511,
        -1.09,
        29.772,
        0.0,
    );
}

#[test]
fn the_cut_is_the_nearest_chord_because_none_stands_two_radii_out_4_5() {
    the_cut_is_the_nearest_chord_because_none_stands_two_radii_out(
        RESTS_ON_THE_SECOND_CREASE,
        0.007496,
        1,
        28.91,
        -2.54,
        29.511,
        -1.09,
    );
}

/// How far short of `2r` a crease landing is ALLOWED to stand: the sagitta the wall's own chords
/// sag off the circle they stand in, measured by CHORD and not by facet.
fn the_deficit_is_measured_against_the_sagitta_the_walls_own_chords_sag(radius: f64, deficit: f64) {
    let read = the_walls_rim(radius);
    let sagitta = surface_mixed_corner_cut_edge::sagitta_of_the_walls_chords(
        read.bead.wall_section.as_deref(),
    );

    let barrel_sags = lobed_prism_meshes::RADIUS
        * (1.0 - math_helper::degrees_to_radians(180.0 / lobed_prism_meshes::SIDES as f64).cos());

    assert_within(
        sagitta,
        barrel_sags,
        1e-3,
        &format!("the barrel's 9 degree chords sag {barrel_sags} off it"),
    );

    assert!(
        deficit < sagitta,
        "the crease's own deficit at {radius} is a fraction of what the wall already sags"
    );
}

#[test]
fn the_deficit_is_measured_against_the_sagitta_the_walls_own_chords_sag_1_5() {
    the_deficit_is_measured_against_the_sagitta_the_walls_own_chords_sag(
        RESTS_ON_THE_FIRST_CREASE,
        0.001312,
    );
}

#[test]
fn the_deficit_is_measured_against_the_sagitta_the_walls_own_chords_sag_4_5() {
    the_deficit_is_measured_against_the_sagitta_the_walls_own_chords_sag(
        RESTS_ON_THE_SECOND_CREASE,
        0.007496,
    );
}

/// And a walk whose nearest chord stands FURTHER short than that is refused by name rather than
/// handed over to: asked at a size the wall has no chord to touch at.
#[test]
fn the_walk_refuses_a_maximum_further_short_than_the_wall_sags() {
    let read = the_walls_rim(RESTS_ON_THE_SECOND_CREASE);
    let asked_at = 6.0;

    let found = surface_mixed_corner_cut_edge::try_find(
        &read.planned,
        read.chain.is_closed,
        read.corner_index,
        read.cut_is_arriving,
        asked_at,
        read.near_enough,
    );

    assert!(
        found.is_none(),
        "no chord of this barrel comes within {} of the bead's axis - instead it took {:?}",
        2.0 * asked_at,
        found.map(|(cut_index, _)| cut_index)
    );
}

/// And the walk asks the crease question the way the corner READING asks it - of the face the two
/// edges share, not of the wall section alone.
#[test]
fn the_walk_qualifies_the_crease_by_the_shared_face_the_way_the_corner_does() {
    let read = the_walls_rim(RESTS_ON_THE_SECOND_CREASE);

    assert!(
        surface_mixed_corner_read_off_the_edge_plans::rests_on_the_crease_of_the_face_shared_with(
            &read.bead,
            &read.planned[read.cut_index as usize]
        ),
        "the crease the bead rests on is a crease of the barrel, which is the face the cut and the bead share"
    );

    assert!(
        !surface_mixed_corner_read_off_the_edge_plans::rests_on_the_crease_of_the_face_shared_with(
            &read.bead, &read.bead
        ),
        "two edges naming the same two faces share no ONE face, so there is no crease landing to read"
    );
}

/// And the touch point is on the CREASE LINE: where the cut's tangent line in the shared face
/// crosses it, a radius under the cap, a radius from the bead's axis and from the cut's centre
/// line alike.
fn the_touch_point_stands_on_the_crease_line_and_under_the_cap(radius: f64) {
    let read = the_walls_rim(radius);
    let corner = &read.corner1;
    let crease = read
        .bead
        .wall_section
        .as_ref()
        .expect("the bead has a section")
        .crease_line;

    assert!(
        crease.distance_to(corner.tangent_point) < 1e-9,
        "P stands on the line the bead touches the wall along - instead {:?}",
        corner.tangent_point
    );

    assert_within(
        corner.tangent_point.z,
        lobed_prism_meshes::HEIGHT - radius,
        1e-9,
        "a radius under the cap",
    );

    assert_within(
        corner.axis().distance_to(corner.tangent_point),
        radius,
        1e-9,
        "a radius out of the bead's axis, so the bead's own surface runs through it",
    );

    assert_within(
        corner.cut_center_line().distance_to(corner.tangent_point),
        radius,
        1e-9,
        "and a radius from the cut's centre line, so the cut's does too",
    );

    // Turn zero is then the way out to the crease rather than square out of a facet.
    assert_within(
        (corner.tangent_point - corner.axis().closest_point_to(corner.tangent_point))
            .get_normal()
            .dot(corner.towards_the_shared_face()),
        1.0,
        1e-9,
        &format!("instead {:?}", corner.towards_the_shared_face()),
    );
}

#[test]
fn the_touch_point_stands_on_the_crease_line_and_under_the_cap_1_5() {
    the_touch_point_stands_on_the_crease_line_and_under_the_cap(RESTS_ON_THE_FIRST_CREASE);
}

#[test]
fn the_touch_point_stands_on_the_crease_line_and_under_the_cap_4_5() {
    the_touch_point_stands_on_the_crease_line_and_under_the_cap(RESTS_ON_THE_SECOND_CREASE);
}

/// And the RUN-OUT past that bead dies ON the wall: every point the ball touches the cap at, over
/// the whole turn past the bead's own end, stands on or inside the wall's trace in the cap.
#[test]
fn the_run_out_dies_on_the_wall_over_a_crease_resting_bead() {
    let read = the_walls_rim(RESTS_ON_THE_FIRST_CREASE);
    let built = the_rim_built(&read, RESTS_ON_THE_FIRST_CREASE);

    let corner = &read.corner1;

    let wall = SurfaceMixedCornerWall::try_read(corner).unwrap_or_else(|unreadable| {
        panic!("the corner's wall has to read: {:?}", unreadable.code())
    });

    // Everything the run-out drew ON the cap: in the turn past the bead, within reach of the ball,
    // and standing in the cap's own plane - which is where its touch points are.
    let mut on_the_cap = 0;
    let mut worst = 0.0;
    let mut worst_at = Vector3::ZERO;

    for &stored in built.vertices.iter() {
        let at = Vector3::from_vector3_float(stored);
        let turn = surface_mixed_corner_ring_schedule::angle_of(corner, at);

        if turn < corner.turn_radians()
            || turn > corner.turn_radians() + surface_mixed_corner_run_out::clears_the_wall() + 0.05
            || (at - corner.outer_rail_centre)
                .dot(corner.out_of_the_cap_face)
                .abs()
                > read.near_enough
            || corner.axis().distance_to(at) > 2.5 * RESTS_ON_THE_FIRST_CREASE
        {
            continue;
        }

        let Some((stands_on, facing)) = wall.try_plane_at(turn) else {
            continue;
        };

        on_the_cap += 1;

        let outside = (at - stands_on).dot(facing);

        if outside > worst {
            worst = outside;
            worst_at = at;
        }
    }

    assert!(
        on_the_cap > 2,
        "the run-out has to BE there to be measured - instead {on_the_cap} points on the cap past the bead"
    );

    assert!(
        worst < read.near_enough,
        "instead {worst_at:?} stands {worst} outside the wall the run-out is trimmed by"
    );

    the_rim_is_a_clean_operand(&built);
}

/// And with both readings answering, the rim BUILDS over a crease-resting bead, with the blend on
/// the ideal torus about the axis to inside the sagitta the barrel's own chords already sag.
#[test]
fn the_rim_builds_over_a_crease_resting_bead() {
    let read = the_walls_rim(RESTS_ON_THE_SECOND_CREASE);
    let built = the_rim_built(&read, RESTS_ON_THE_SECOND_CREASE);

    the_rim_is_a_clean_operand(&built);

    let corner = &read.corner1;
    let sagitta = lobed_prism_meshes::RADIUS
        * (1.0 - math_helper::degrees_to_radians(180.0 / lobed_prism_meshes::SIDES as f64).cos());

    let mut on_the_torus = 0;
    let mut worst = 0.0_f64;

    for &stored in built.vertices.iter() {
        let at = Vector3::from_vector3_float(stored);
        let turn = surface_mixed_corner_ring_schedule::angle_of(corner, at);

        if turn < -0.05 || turn > corner.turn_radians() + 0.05 {
            continue;
        }

        let out_of_the_axis = (at - corner.axis().closest_point_to(at)).length();
        let under_the_cap = (corner.outer_rail_centre - at).dot(corner.out_of_the_cap_face);
        let from_the_centre_line = ((out_of_the_axis - corner.outer_rail_radius()).powi(2)
            + (under_the_cap - RESTS_ON_THE_SECOND_CREASE).powi(2))
        .sqrt();

        let off = (from_the_centre_line - RESTS_ON_THE_SECOND_CREASE).abs();

        if off > 0.2 || out_of_the_axis > corner.outer_rail_radius() + 1e-6 {
            // Past the outer rail is the cap's own material, which the torus runs tangent to.
            continue;
        }

        on_the_torus += 1;
        worst = worst.max(off);
    }

    assert!(
        on_the_torus > 20,
        "the blend has to BE there to be measured - instead {on_the_torus} vertices near the torus"
    );

    assert!(
        worst < sagitta,
        "the worst stands {worst} off the torus, where the barrel's own chords sag {sagitta}"
    );
}

/// Closed, manifold, one shell, and an operand the boolean kernel takes as it is.
fn the_rim_is_a_clean_operand(built: &Mesh) {
    let (open, open_at) = surface_mesh_closure::open_edge_count(built);
    assert_eq!(open, 0, "open at {open_at:?}");
    assert!(built.is_manifold());
    assert_eq!(shell_count(built), 1);

    assert_eq!(
        classify_boolean_operand(built, Matrix4X4::IDENTITY, false),
        BooleanOperandVerdict::Clean,
        "a Combine after a Fillet hands this very mesh to the kernel"
    );
}

/// The whole rim, planned as one chain at `radius` and built at eight segments.
fn the_rim_built(read: &Rim, radius: f64) -> Mesh {
    let plan = SurfaceBevelPlan::try_create(&read.chain, &read.regions, radius, true)
        .unwrap_or_else(|why| panic!("the rim has to plan at {radius}: {why}"));

    surface_bevel_builder::try_build(
        &read.prism,
        &read.prism.get_mesh_edge_graph(),
        &read.regions,
        &[plan],
        8,
        &CancelToken::new(),
        SurfaceBuildOptions {
            crossings: None,
            ran_off_over_a_stretch: None,
            lay_the_oblique_rows_anyway: false,
            planless_loops: &[],
            features: None,
            use_sections: crate::surface_test_build::use_sections(),
        },
    )
    .map(|built| built.mesh)
    .unwrap_or_else(|reason| panic!("the rim has to build at {radius}: {reason:?}"))
}

/// The length of two skew lines' common perpendicular, as the walk measures it.
pub(crate) fn apart(first: SurfaceLine, second: SurfaceLine) -> f64 {
    let between = second.point - first.point;
    let direction_dot = first.direction.dot(second.direction);
    let determinant = 1.0 - (direction_dot * direction_dot);
    let along_first = between.dot(first.direction);
    let along_second = between.dot(second.direction);
    let here = (along_first - (direction_dot * along_second)) / determinant;
    let there = ((direction_dot * along_first) - along_second) / determinant;

    ((first.point + (first.direction * here)) - (second.point + (second.direction * there)))
        .length()
}

/// One reading of the barrel's rim: the mesh it came off, its plans, and the corner at the top of
/// the right seam.
pub(crate) struct Rim {
    pub(crate) prism: Mesh,
    pub(crate) regions: Arc<FaceRegionCache>,
    pub(crate) chain: BevelSweptChain,
    pub(crate) planned: Vec<SurfaceEdgePlan>,
    /// The edge the chain ARRIVES at the corner on, which is what names the corner.
    pub(crate) corner_index: i32,
    /// Whether the convex side of the corner is that arriving edge.
    pub(crate) cut_is_arriving: bool,
    pub(crate) cut_index: i32,
    pub(crate) swallowed: Vec<i32>,
    pub(crate) bead: SurfaceEdgePlan,
    pub(crate) corner1: Rc<SurfaceMixedCorner>,
    /// The part's own ruler, which the readings measure lengths against.
    pub(crate) near_enough: f64,
}

/// The LOBE WALL as a face pick, planned at `radius`, with the corner at the top of its right seam
/// read off it - the same reading `surface_mixed_corner_over_a_curved_shared_face_tests` makes at
/// the fixture's own size.
pub(crate) fn the_walls_rim(radius: f64) -> Rim {
    let prism = lobed_prism_meshes::create();
    let graph = prism.get_mesh_edge_graph();
    let regions = Arc::new(FaceRegionCache::new(
        Arc::new(prism.clone()),
        Some(graph),
        lobed_prism_meshes::THRESHOLD_DEGREES,
    ));

    let region = regions
        .region(a_facet_of_the_lobes_wall(&prism))
        .expect("the facet has a region");
    let chains = bevel_chain_linker::link(region.boundary_edges());
    assert_eq!(chains.len(), 1);
    let chain = chains.into_iter().next().expect("one chain");

    let mut planned = Vec::with_capacity(chain.edges.len());

    for edge in &chain.edges {
        let alone = BevelSweptChain::new(vec![*edge], false);

        let one = SurfaceBevelPlan::try_create(&alone, &regions, radius, true)
            .unwrap_or_else(|why| panic!("every edge of the wall's rim plans on its own: {why}"));

        planned.push(one.edges[0].clone());
    }

    let corner = Vector3::new(
        lobed_prism_meshes::right_crossing().x,
        lobed_prism_meshes::right_crossing().y,
        lobed_prism_meshes::HEIGHT,
    );
    let near_enough = SurfaceVertexPool::near_enough_for(&prism.vertices);

    for i in 0..planned.len() {
        let here = &planned[i];
        let next = &planned[(i + 1) % planned.len()];

        if here.convexity() == next.convexity() || (here.to() - corner).length() > 1e-3 {
            continue;
        }

        let cut_is_arriving = here.convexity() == EdgeConvexity::Convex;

        let (cut_index, swallowed) = surface_mixed_corner_cut_edge::try_find(
            &planned,
            chain.is_closed,
            i as i32,
            cut_is_arriving,
            radius,
            near_enough,
        )
        .unwrap_or_else(|| panic!("the corner has a cut edge to hand over to at {radius}"));

        let bead = if cut_is_arriving { next } else { here }.clone();

        let read = SurfaceMixedCorner::try_read(
            &planned[cut_index as usize],
            &bead,
            corner,
            radius,
            near_enough,
        )
        .unwrap_or_else(|refusal| {
            panic!("the corner has to read at {radius}: {:?}", refusal.code())
        });

        return Rim {
            prism,
            regions,
            chain,
            planned,
            corner_index: i as i32,
            cut_is_arriving,
            cut_index,
            swallowed,
            bead,
            corner1: Rc::new(read),
            near_enough,
        };
    }

    panic!("the wall's rim turns over at the top of its right seam");
}

/// Any one facet of the lobe's own wall - a side face standing well outside the flat side.
fn a_facet_of_the_lobes_wall(prism: &Mesh) -> i32 {
    for face in 0..prism.faces.len() {
        let facet = prism.faces[face];
        let middle = (Vector3::from_vector3_float(prism.vertices[facet.v0 as usize])
            + Vector3::from_vector3_float(prism.vertices[facet.v1 as usize])
            + Vector3::from_vector3_float(prism.vertices[facet.v2 as usize]))
            / 3.0;

        if middle.y < -lobed_prism_meshes::RADIUS / 2.0
            && middle.z > 1.0
            && middle.z < lobed_prism_meshes::HEIGHT - 1.0
        {
            return face as i32;
        }
    }

    panic!("the prism has lost the lobe it pokes through its side");
}
