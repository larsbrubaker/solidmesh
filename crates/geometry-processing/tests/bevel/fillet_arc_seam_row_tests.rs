//! `FilletArc.SeamRow` - a section of the incoming leg's fillet slid onto a joint's bisector plane -
//! lies on BOTH legs' fillets where the setback is constant.

use solidmesh::bevel::level_line::fillet_arc;
use solidmesh_math::Vector3;

#[test]
fn a_box_rim_mitres_seam_row_lies_on_both_cylinders() {
    // The top rim of a box (top face z = 0, side faces below) turning left at (10, 0, 0): along +x,
    // then along +y. At r 1 each leg's fillet is the cylinder of radius 1 about its axis: y = 1,
    // z = -1 for the x leg, x = 9, z = -1 for the y leg. The seam row runs from the top-face rail's
    // corner (9, 1, 0) to the side rails' corner (10, 0, -1), both on the bisector plane x + y = 10.
    let apex = Vector3::new(10.0, 0.0, 0.0);
    let row = fillet_arc::seam_row(
        apex,
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(9.0, 1.0, 0.0),
        Vector3::new(10.0, 0.0, -1.0),
        12,
    );
    for (s, p) in row.iter().enumerate() {
        assert!(
            ((p.x + p.y) - 10.0).abs() <= 1e-9,
            "point {s} {p:?} is on the bisector plane"
        );
        let off_x_leg = (((p.y - 1.0) * (p.y - 1.0)) + ((p.z + 1.0) * (p.z + 1.0))).sqrt();
        let off_y_leg = (((p.x - 9.0) * (p.x - 9.0)) + ((p.z + 1.0) * (p.z + 1.0))).sqrt();
        assert!(
            (off_x_leg - 1.0).abs() <= 1e-9,
            "point {s} {p:?} is on the x leg's fillet"
        );
        assert!(
            (off_y_leg - 1.0).abs() <= 1e-9,
            "point {s} {p:?} is on the y leg's fillet"
        );
    }
}
