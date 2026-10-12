//! Ports `GeometryProcessingTests/BevelRecessApexRunOutTests.cs`: the apex end of "UpperLeftInner
//! should bevel.mcx" - one concave crease of a pyramidal recess, rounded at 3 mm, dying into the
//! three way concave corner where the recess's ridge starts, beaded through the kernel's own
//! sequence (`bead_sequence::full_tools`) on the fixture's own fourteen vertices, both ways up, and
//! both where the box frame puts the part and where the rebuild actually has it. The control carries
//! the bead's cylinder past the apex, lets the lower slope cut it, and the bead dies 2.293 mm past
//! the apex; the apex vertex must end up buried rather than left standing as a surface vertex the
//! union fans spurious triangles from. C#'s `[Arguments]` rows are a loop.

use polygon_mesh::Mesh;
use vector_math::Vector3;

use crate::bead_sequence::{clip_and_union, full_tools};
use crate::kernel_cut_helpers::edge_group;
use crate::mesh_measurements::{shell_count, signed_volume};
use crate::pyramid_recess_test_meshes as recess;

const RADIUS: f64 = 3.0;
const SEGMENTS: i32 = 8;

/// How far past the apex the control's bead reaches along the ridge: where the tangent line on the
/// upper slope, 2.162 off the crease, meets y = 10 - and where the kernel's capped ring lands.
const RUN_OUT_ALONG_THE_RIDGE: f64 = 2.293;

/// How far inside the bead the apex vertex is: the arc's nearest approach to the crease line is
/// the ball centre's `r / sin(55 degrees)` less `r`, 0.66, and the nearest vertex of a correct
/// result stands 0.713 off. Half of that is a bound no rounding reaches and no surviving vertex
/// passes.
const APEX_BURIED_BY: f64 = 0.35;

/// The whole apex end, either way up and at either place: the bead runs out along the ridge to
/// where the control's does, AND the raw corner vertex it ran over is buried.
#[test]
fn the_bead_runs_out_past_the_apex_and_buries_it() {
    for (mirror_z, where_the_rebuild_has_the_part) in
        [(false, false), (true, false), (false, true), (true, true)]
    {
        let offset = if where_the_rebuild_has_the_part {
            recess::fixture_offset()
        } else {
            Vector3::ZERO
        };
        let part = recess::create_from_the_fixtures_own_triangles(offset, mirror_z);
        let place = |box_frame_point: Vector3| {
            (if mirror_z {
                recess::mirror_z(box_frame_point)
            } else {
                box_frame_point
            }) + offset
        };
        let rim = place(recess::rim_end());
        let apex = place(recess::apex_end());

        let beaded = bead_the_crease(&part, rim, apex);

        assert_eq!(shell_count(&beaded), 1, "one bead on one solid");

        let along_the_ridge = furthest_along_the_ridge(&beaded, apex) - apex.x;
        let nearest_to_the_apex = beaded
            .vertices
            .iter()
            .map(|vertex| (vertex.as_vector3() - apex).length())
            .fold(f64::INFINITY, f64::min);
        let place_name = if where_the_rebuild_has_the_part {
            "the rebuild's offset"
        } else {
            "the origin"
        };
        let at = format!(
            "mirrored {mirror_z}, at {place_name}: volume {:.3}, run-out along the ridge \
             {along_the_ridge:.3}, nearest vertex to the apex {nearest_to_the_apex:.4}",
            signed_volume(&beaded)
        );

        assert!(
            (along_the_ridge - RUN_OUT_ALONG_THE_RIDGE).abs() <= 0.01,
            "the bead is capped on the lower slope and its ring reaches the ridge where the \
             control's tangent line does. {at}"
        );
        assert!(
            nearest_to_the_apex > APEX_BURIED_BY,
            "the apex vertex is 0.66 inside the bead's arc and must not survive the union as a \
             surface vertex - where it does, it is the tip of a fan of coincident and zero-area \
             triangles of the crease face under the bead, which is the clip's keep-set failing to \
             merge the void wedge's wall with the face it lies on. {at}"
        );
    }
}

/// The fixture's own triangles ARE the solid the planes describe: same volume, the cube less the
/// recess, so a reading on either builder is a reading about the same part.
#[test]
fn the_fixtures_triangles_are_the_solid_the_planes_describe() {
    let from_the_planes = signed_volume(&recess::create(false, Vector3::ZERO));
    let from_the_fixture = signed_volume(&recess::create_from_the_fixtures_own_triangles(
        Vector3::ZERO,
        false,
    ));

    assert!(
        (from_the_planes - from_the_fixture).abs() <= 0.002,
        "the planes give {from_the_planes:.4} and the fixture's triangles {from_the_fixture:.4}"
    );
    assert!(
        (from_the_fixture - 6333.343).abs() <= 0.002,
        "the regression test measures the part off the operation's own source mesh at 6333.343 - \
         got {from_the_fixture:.4}"
    );
}

/// Selects the crease from the rim to the apex and beads it exactly as `BevelFeatureMeshBuilder`
/// does - resolve, award, face fit, the cross-group readings, the tools, the bead clip, the union.
fn bead_the_crease(part: &Mesh, rim: Vector3, apex: Vector3) -> Mesh {
    let features = geometry_processing::topology::FeatureEdgeSet::create(part);
    let group = edge_group(&features, rim, apex);
    let built = full_tools(part, &[group], RADIUS, SEGMENTS);

    clip_and_union(part, &built.clippable, built.solids).0
}

/// The furthest x any vertex reaches along the ridge line through the apex (within a tenth of it
/// and short of the ridge's middle), which is where a bead that ran out past the apex died on it.
fn furthest_along_the_ridge(mesh: &Mesh, apex: Vector3) -> f64 {
    let mut furthest = f64::NAN;

    for v in mesh.vertices.iter().map(|vertex| vertex.as_vector3()) {
        if (v.y - apex.y).abs() < 0.1
            && (v.z - apex.z).abs() < 0.1
            && v.x < apex.x + 5.0
            && (furthest.is_nan() || v.x > furthest)
        {
            furthest = v.x;
        }
    }

    furthest
}
