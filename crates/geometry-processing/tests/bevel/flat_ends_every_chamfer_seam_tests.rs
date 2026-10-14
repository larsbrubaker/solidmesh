//! Flat ends (plan stage 2) on "Every Chamfer Works.mcx": the seam, the vertical edge and the 29
//! chord bottom rim build at the file's own 3.756 with the flag off, so they must build with it on.
//! The rim ends at (-1, 40, 0) on the x = -1 wall, which faces back across the convex chain's end
//! nearly square. Parameter rows are a loop.

use std::rc::Rc;

use crate::end_face_builds::{as_saved, build_chains, FlatEnds};
use crate::level_line_fixture::LevelLineFixture;

#[test]
fn the_seam_the_vertical_edge_and_the_bottom_rim_build() {
    for flat_ends in [false, true] {
        let fixture = Rc::new(LevelLineFixture::load("every-chamfer-seam-vertical-rim"));
        let all: Vec<usize> = (0..fixture.chains.len()).collect();
        let gap = {
            let _flat_ends = FlatEnds::set(flat_ends);
            build_chains(&fixture, &all, fixture.size, &as_saved(&fixture, None))
        };

        let chains: Vec<String> = fixture
            .chains
            .iter()
            .enumerate()
            .map(|(k, c)| {
                format!(
                    "chain {k} {:?} .. {:?} closed {}",
                    fixture.positions[c[0] as usize],
                    fixture.positions[*c.last().expect("a chain") as usize],
                    fixture.closed[k]
                )
            })
            .collect();
        println!("{}", chains.join(" / "));
        assert!(
            gap.built(),
            "flat ends {flat_ends}: {:?}, room {}",
            gap.refusal,
            gap.room
        );
    }
}
