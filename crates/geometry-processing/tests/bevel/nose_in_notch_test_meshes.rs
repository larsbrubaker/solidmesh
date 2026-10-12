//! Port of the test fixture `Fixtures/NoseInNotchTestMeshes.cs` (its `Create` with the upper face
//! whole or chorded on its right crease - `theUpperFaceFannedFromInside` is not ported yet - the named points and normals, and the crease and group picks the tests make on it): a
//! 20 mm cube with a triangular notch cut down its
//! back, and a two-faced nose standing in the notch.

#![allow(dead_code)]

use geometry_processing::bevel::BevelSweptChain;
use geometry_processing::topology::bevel_edge_group_resolver;
use geometry_processing::topology::{
    BevelEdgeGroup, BevelSelectionKind, ChainEdge, FaceGroup, FeatureEdgeSet,
};
use polygon_mesh::Mesh;
use vector_math::{Vector2, Vector3};

/// The cube's edge.
pub const SIZE: f64 = 20.0;
/// The part's volume: the cube less the notch, plus the nose.
pub const VOLUME: f64 = 8000.0 - 2000.0 + (500.0 / 9.0);
/// Where the nose's ridge stands.
pub const RIDGE_Y: f64 = 5.0;
/// The ridge's height.
pub const RIDGE_Z: f64 = 10.0;

pub fn left_ridge_end() -> Vector3 {
    Vector3::new(5.0, RIDGE_Y, RIDGE_Z)
}

pub fn right_ridge_end() -> Vector3 {
    Vector3::new(15.0, RIDGE_Y, RIDGE_Z)
}

pub fn upper_crease_end() -> Vector3 {
    Vector3::new(10.0, 10.0, 40.0 / 3.0)
}

pub fn lower_crease_end() -> Vector3 {
    Vector3::new(10.0, 10.0, 20.0 / 3.0)
}

/// The notch's floor corner.
pub fn notch_floor() -> Vector3 {
    Vector3::new(10.0, 10.0, 0.0)
}

/// The notch's ceiling corner.
pub fn notch_ceiling() -> Vector3 {
    Vector3::new(10.0, 10.0, SIZE)
}

pub fn left_slope_normal() -> Vector3 {
    Vector3::new(1.0, -1.0, 0.0).get_normal()
}

pub fn right_slope_normal() -> Vector3 {
    Vector3::new(-1.0, -1.0, 0.0).get_normal()
}

pub fn upper_nose_normal() -> Vector3 {
    Vector3::new(0.0, -2.0, 3.0).get_normal()
}

pub fn lower_nose_normal() -> Vector3 {
    Vector3::new(0.0, -2.0, -3.0).get_normal()
}

/// C# `Create()` with its defaults (unscaled, the upper face one triangle).
pub fn create() -> Mesh {
    create_scaled(1.0)
}

/// C# `Create(scale)`, the upper face one triangle.
pub fn create_scaled(scale: f64) -> Mesh {
    create_full(scale, false, None)
}

/// C# `Create(theUpperRightCreaseSplitAt: split)`: the same solid with the upper right crease - the
/// run from [`right_ridge_end`] to [`upper_crease_end`] - CHORDED into two mesh edges through
/// `split`, which is what the boolean behind "Center Flows Through.mcx" leaves on that crease (at
/// (12, 8, 12), its t = 0.6). Both faces that meet along the crease are cut at the same point, so
/// the upper nose face becomes two facets and the right slope's fan triangle across the crease
/// becomes two; only ONE of the crease's two rim edges then touches the corner it dies in.
pub fn create_with_upper_right_crease_split_at(split: Vector3) -> Mesh {
    create_full(1.0, false, Some(split))
}

/// C# `Create(scale, theUpperFaceFannedFromInside: false, theUpperRightCreaseSplitAt)`.
/// C# `Create(theUpperFaceFannedFromInside: true)`: the same solid with the nose's UPPER face cut
/// into three triangles about a point strictly inside it - which is what a boolean leaves, and the
/// shape of what "15mm Bevel Top Edge.mcx" carries. Only the facet across the upper left crease is
/// the one a band on the top left edge reaches, and it shares no EDGE with the lower face.
pub fn create_with_the_upper_face_fanned_from_inside() -> Mesh {
    create_full(1.0, true, None)
}

/// C#'s `Create` with all its options. C# refuses asking for the fan and the split together; the
/// public constructors above never ask for both.
fn create_full(
    scale: f64,
    upper_face_fanned_from_inside: bool,
    upper_right_crease_split_at: Option<Vector3>,
) -> Mesh {
    let mut mesh = Mesh::new();
    let centre = Vector2::new(SIZE / 2.0, SIZE / 2.0);

    fan(&mut mesh, 0.0, false, centre);
    fan(&mut mesh, SIZE, true, centre);

    mesh.create_face(&[
        Vector3::new(0.0, SIZE, 0.0),
        Vector3::new(0.0, SIZE, SIZE),
        Vector3::new(SIZE, SIZE, SIZE),
        Vector3::new(SIZE, SIZE, 0.0),
    ]);
    mesh.create_face(&[
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, SIZE),
        Vector3::new(0.0, SIZE, SIZE),
        Vector3::new(0.0, SIZE, 0.0),
    ]);
    mesh.create_face(&[
        Vector3::new(SIZE, 0.0, 0.0),
        Vector3::new(SIZE, SIZE, 0.0),
        Vector3::new(SIZE, SIZE, SIZE),
        Vector3::new(SIZE, 0.0, SIZE),
    ]);

    slope(&mut mesh, false, None);
    slope(&mut mesh, true, upper_right_crease_split_at);

    // The nose's two faces, each a triangle from the two ridge ends to the crease end they share.
    if upper_face_fanned_from_inside {
        // The same triangle, fanned from a point strictly INSIDE it - three facets, of which only
        // the one across the upper left crease is the one the band reaches. (Fanned from (12, 8, 12)
        // on the outline instead, one of the three would be a zero area facet.)
        let inside = Vector3::new(10.0, 7.0, 34.0 / 3.0);

        mesh.create_face(&[left_ridge_end(), right_ridge_end(), inside]);
        mesh.create_face(&[right_ridge_end(), upper_crease_end(), inside]);
        mesh.create_face(&[upper_crease_end(), left_ridge_end(), inside]);
    } else if let Some(split) = upper_right_crease_split_at {
        // The same triangle, cut from the left ridge end to the point the crease is chorded at, so
        // this face carries the same two crease edges the slope beyond it does.
        mesh.create_face(&[left_ridge_end(), split, upper_crease_end()]);
        mesh.create_face(&[left_ridge_end(), right_ridge_end(), split]);
    } else {
        mesh.create_face(&[left_ridge_end(), right_ridge_end(), upper_crease_end()]);
    }
    mesh.create_face(&[left_ridge_end(), lower_crease_end(), right_ridge_end()]);

    mesh.clean_and_merge();

    if scale != 1.0 {
        mesh.transform(vector_math::Matrix4X4::create_scale_uniform(scale));
    }

    mesh
}

fn fan(mesh: &mut Mesh, z: f64, up: bool, centre: Vector2) {
    let corner = Vector3::new(centre.x, centre.y, z);
    let around = [
        Vector3::new(0.0, 0.0, z),
        Vector3::new(0.0, SIZE, z),
        Vector3::new(SIZE, SIZE, z),
        Vector3::new(SIZE, 0.0, z),
    ];

    for i in 0..around.len() - 1 {
        if up {
            mesh.create_face(&[corner, around[i + 1], around[i]]);
        } else {
            mesh.create_face(&[corner, around[i], around[i + 1]]);
        }
    }
}

/// One slope: the rectangle from the cube's own corner edge across to the centre crease, less the
/// triangle the nose bites out of it, fanned from the ridge end that is the bite's apex.
/// `crease_split_at` chords the crease from the apex to `around[3]`; only the MIRRORED slope's is
/// ever asked for, because the two facets it makes are wound the mirrored way.
fn slope(mesh: &mut Mesh, mirrored: bool, crease_split_at: Option<Vector3>) {
    assert!(
        crease_split_at.is_none() || mirrored,
        "only the right slope's crease is chorded"
    );

    let at = |x: f64, y: f64, z: f64| {
        if mirrored {
            Vector3::new(SIZE - x, y, z)
        } else {
            Vector3::new(x, y, z)
        }
    };

    let apex = at(5.0, RIDGE_Y, RIDGE_Z);
    let around = [
        at(0.0, 0.0, 0.0),
        at(0.0, 0.0, SIZE),
        at(10.0, 10.0, SIZE),
        at(10.0, 10.0, 40.0 / 3.0),
        at(10.0, 10.0, 20.0 / 3.0),
        at(10.0, 10.0, 0.0),
    ];

    for i in 0..around.len() - 1 {
        if i == 3 {
            continue;
        }

        if let (2, Some(split)) = (i, crease_split_at) {
            // The one fan triangle whose apex-to-around[3] side IS the upper crease, cut at the
            // same point the nose's own face is, so the two sides of the crease go on sharing
            // their edges.
            mesh.create_face(&[apex, around[2], split]);
            mesh.create_face(&[split, around[2], around[3]]);
            continue;
        }

        if mirrored {
            mesh.create_face(&[apex, around[i], around[i + 1]]);
        } else {
            mesh.create_face(&[apex, around[i + 1], around[i]]);
        }
    }

    if mirrored {
        mesh.create_face(&[apex, around[around.len() - 1], around[0]]);
    } else {
        mesh.create_face(&[apex, around[0], around[around.len() - 1]]);
    }
}

/// C# `Creases(part, out names)` at scale 1: the nose's four creases, each a one-edge run from
/// its ridge end to its crease end, with their names.
pub fn creases(part: &Mesh) -> (Vec<BevelSweptChain>, Vec<String>) {
    creases_scaled(part, 1.0)
}

/// C# `Creases(part, out names, scale)`.
pub fn creases_scaled(part: &Mesh, scale: f64) -> (Vec<BevelSweptChain>, Vec<String>) {
    let features = FeatureEdgeSet::create(part);

    let wanted = [
        (
            "upper left",
            left_ridge_end() * scale,
            upper_crease_end() * scale,
        ),
        (
            "lower left",
            left_ridge_end() * scale,
            lower_crease_end() * scale,
        ),
        (
            "upper right",
            right_ridge_end() * scale,
            upper_crease_end() * scale,
        ),
        (
            "lower right",
            right_ridge_end() * scale,
            lower_crease_end() * scale,
        ),
    ];

    let mut runs = Vec::new();
    let mut names = Vec::new();

    for (name, from, to) in wanted {
        let mut found = false;

        for (i, edge) in features.edges().iter().enumerate() {
            if found {
                break;
            }

            if (edge.start - from).length() < 1e-6 * scale
                && (edge.end - to).length() < 1e-6 * scale
            {
                runs.push(BevelSweptChain::new(
                    vec![ChainEdge::new(i as i32, *edge, false)],
                    false,
                ));
                found = true;
            } else if (edge.end - from).length() < 1e-6 * scale
                && (edge.start - to).length() < 1e-6 * scale
            {
                runs.push(BevelSweptChain::new(
                    vec![ChainEdge::new(i as i32, *edge, true)],
                    false,
                ));
                found = true;
            }
        }

        assert!(
            found,
            "the fixture has no {name} crease from {from:?} to {to:?}"
        );
        names.push(name.to_string());
    }

    (runs, names)
}

/// C# `CentreBottomGroups(features, out names)` at scale 1: the three creases "5mm Bevel Center
/// Bottom.mcx" picks, as groups a rebuild can resolve - the notch's own centre crease from the floor
/// up to [`lower_crease_end`], and the two LOWER nose creases that die on the same point. All three
/// meet at that one concave junction, which is what makes this selection the inside-corner mirror
/// of a three-run boss.
///
/// One group per edge, which is what the all-edges gesture gives and what the fixture carries. The
/// two UPPER nose creases are deliberately NOT in it: their absence is why each nose bead's ridge
/// end has no partner to be bounded by and has to be ended on the plane of the face across the
/// ridge instead (`BevelPartneredBead`).
pub fn centre_bottom_groups(features: &FeatureEdgeSet) -> (Vec<BevelEdgeGroup>, Vec<String>) {
    groups_on(
        features,
        &[
            ("centre", notch_floor(), lower_crease_end()),
            ("lower left", left_ridge_end(), lower_crease_end()),
            ("lower right", right_ridge_end(), lower_crease_end()),
        ],
        1.0,
        "",
    )
}

/// C# `CentreFlowsThroughGroups(features, out names)` at scale 1: the two creases "Center Flows
/// Through.mcx" picks, as groups a rebuild can resolve - the two pieces the nose leaves of the
/// notch's own centre crease, from the floor up to [`lower_crease_end`], and from
/// [`upper_crease_end`] on up to the ceiling.
///
/// The nose's own four creases are deliberately NOT in it. That is what makes this selection the
/// INTERRUPTED crease rather than a corner: neither piece has a selected neighbour at the end where
/// the nose stops it, and what stands there is not a corner the two faces make but the nose's own
/// face, standing over the crease line the two slopes go on making behind it.
pub fn centre_flows_through_groups(
    features: &FeatureEdgeSet,
) -> (Vec<BevelEdgeGroup>, Vec<String>) {
    groups_on(
        features,
        &[
            ("below the nose", notch_floor(), lower_crease_end()),
            ("above the nose", upper_crease_end(), notch_ceiling()),
        ],
        1.0,
        "centre crease ",
    )
}

/// The shared body of C#'s two group lists: one edge group seeded on the feature edge running
/// between each named pair of points (either way round), at `scale`.
fn groups_on(
    features: &FeatureEdgeSet,
    wanted: &[(&str, Vector3, Vector3)],
    scale: f64,
    what: &str,
) -> (Vec<BevelEdgeGroup>, Vec<String>) {
    let mut groups = Vec::new();
    let mut names = Vec::new();

    for &(name, from, to) in wanted {
        let (from, to) = (from * scale, to * scale);
        let seed = features.edges().iter().position(|edge| {
            ((edge.start - from).length() < 1e-5 * scale && (edge.end - to).length() < 1e-5 * scale)
                || ((edge.start - to).length() < 1e-5 * scale
                    && (edge.end - from).length() < 1e-5 * scale)
        });

        let Some(seed) = seed else {
            panic!("the fixture has no {what}{name} crease from {from} to {to}");
        };

        groups.push(
            bevel_edge_group_resolver::create_from_edge(
                features,
                seed as i32,
                BevelSelectionKind::Edge,
                FaceGroup::DEFAULT_CURVE_TOLERANCE_DEGREES,
            )
            .expect("the edge is in the set"),
        );
        names.push(name.to_string());
    }

    (groups, names)
}
