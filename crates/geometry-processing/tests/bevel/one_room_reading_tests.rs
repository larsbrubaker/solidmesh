//! ONE reading of the room, of the part and the pick - never of the size it was asked at. Parameter
//! rows are loops over the same-named test. `LPrism` is `level_line_room_geometry::l_prism`.

use std::rc::Rc;

use solidmesh::bevel::level_line::band_gap_entry_points::{self, BandGapBuildOptions};
use solidmesh::bevel::level_line::band_gap_reading;
use solidmesh::bevel::level_line::seam_edges::FeatureEdgeTest;
use solidmesh::bevel::level_line::{
    BandGapRefusal, BandGapResult, BandProfile, RoomReading, RoomSource,
};
use solidmesh_math::Vector3;

use crate::level_line_feature_edges::feature_edges_over;
use crate::level_line_fixture::LevelLineFixture;
use crate::level_line_room_geometry::l_prism;
use crate::open_chain_fixtures;
use crate::raked_box_fixture;

/// What to read the room of, and how to build it at a size.
struct Pick {
    positions: Vec<Vector3>,
    indices: Vec<(i32, i32, i32)>,
    chain: Vec<i32>,
    closed: bool,
    feature: FeatureEdgeTest,
    profile: BandProfile,
}

impl Pick {
    fn read(&self) -> RoomReading {
        band_gap_reading::read_room(
            &self.positions,
            &self.indices,
            &self.chain,
            self.closed,
            Some(Rc::clone(&self.feature)),
            self.profile,
            1e-9,
            None,
            0.0,
            f64::INFINITY,
        )
        .expect("the room reads")
    }

    /// The room the reading names.
    fn room(&self) -> f64 {
        self.read().room
    }

    /// What set the room.
    fn room_set_by(&self) -> RoomSource {
        self.read().room_set_by
    }

    /// One band build at `size`, under `known_room` when given.
    fn build(&self, size: f64, known_room: Option<f64>) -> BandGapResult {
        band_gap_entry_points::build(
            &self.positions,
            &self.indices,
            &self.chain,
            self.closed,
            Some(Rc::clone(&self.feature)),
            size,
            self.profile,
            &BandGapBuildOptions {
                chord_tolerance: 0.0,
                snap: 1e-9,
                known_room,
                ..BandGapBuildOptions::default()
            },
        )
        .expect("the pick builds or refuses by name")
    }
}

/// The fixtures the room reading is run on.
const NAMES: [&str; 7] = [
    "internal-fillet-face8-r1",
    "internal-fillet-face8-r2",
    "cylinder-rim",
    "l-prism-rim",
    "raked-box-rim",
    "fade-one-edge",
    "fade-three-edges",
];

/// The pick a test row names.
fn pick_named(name: &str) -> Pick {
    match name {
        "cylinder-rim" => {
            let (positions, indices) = open_chain_fixtures::cylinder();
            let sides = open_chain_fixtures::SIDES;
            let feature =
                feature_edges_over(&positions, &indices, open_chain_fixtures::FOLD_DEGREES);
            Pick {
                positions,
                indices,
                chain: (sides..sides + sides).collect(),
                closed: true,
                feature,
                profile: BandProfile::Fillet,
            }
        }
        "l-prism-rim" => {
            let (positions, indices, rim) = l_prism();
            let feature = feature_edges_over(&positions, &indices, 45.0);
            Pick {
                positions,
                indices,
                chain: rim,
                closed: true,
                feature,
                profile: BandProfile::Fillet,
            }
        }
        "raked-box-rim" => {
            let (positions, indices, rim) = raked_box_fixture::pick(false, None);
            // The raked box's feature edges, at its own fold threshold.
            let feature = feature_edges_over(&positions, &indices, raked_box_fixture::FOLD_DEGREES);
            Pick {
                positions,
                indices,
                chain: rim,
                closed: true,
                feature,
                profile: BandProfile::Fillet,
            }
        }
        "fade-one-edge" | "fade-three-edges" => {
            let (positions, indices) = open_chain_fixtures::cylinder();
            let edges = if name == "fade-one-edge" { 1 } else { 3 };
            let feature =
                feature_edges_over(&positions, &indices, open_chain_fixtures::FOLD_DEGREES);
            Pick {
                positions,
                indices,
                chain: open_chain_fixtures::top_rim_run(edges),
                closed: false,
                feature,
                profile: BandProfile::Fillet,
            }
        }
        _ => {
            let fixture = Rc::new(LevelLineFixture::load(name));
            let lookup = Rc::clone(&fixture);
            Pick {
                positions: fixture.positions.clone(),
                indices: fixture.indices.clone(),
                chain: fixture.chains[0].clone(),
                closed: true,
                feature: Rc::new(move |a, b| lookup.is_feature_edge(a, b)),
                profile: if fixture.chamfer {
                    BandProfile::Chamfer
                } else {
                    BandProfile::Fillet
                },
            }
        }
    }
}

/// Every size under the room read is not refused by SIZE,
/// and a size above it is, naming that very number to the bit.
#[test]
fn the_build_decides_on_the_room_that_was_read() {
    for name in NAMES {
        let pick = pick_named(name);
        let room = pick.room();

        assert!(
            room.is_finite(),
            "{name}: every pick here runs out of something"
        );
        assert!(room > 0.0, "{name}");

        for rung in [0.5, 0.99, 1.0] {
            let under = pick.build(rung * room, None);
            assert!(
                under.built(),
                "{name}: {rung} of the room {room} is a size the room says builds, and it answered {:?} naming {}",
                under.refusal,
                under.room
            );
        }

        let over = pick.build(1.1 * room, None);
        assert_eq!(over.refusal, BandGapRefusal::Size, "{name}");
        assert_eq!(
            over.room, room,
            "{name}: one reading: the build named {} where the room read is {room}",
            over.room
        );
    }
}

/// A build that reads its own room reports the same
/// room, and the same thing setting it, as the reading asked for on its own.
#[test]
fn a_builds_own_reading_carries_the_readings_fields() {
    for name in NAMES {
        let pick = pick_named(name);
        let room = pick.room();
        let built = pick.build(0.5 * room, None);

        assert_eq!(built.room, room, "{name}");
        assert_eq!(
            built.room_set_by,
            pick.room_set_by(),
            "{name}: what set the room is a fact about the part, so the build and the reading agree on it"
        );
    }
}

/// A build handed the room and a build that reads
/// it for itself answer the same number.
#[test]
fn handed_the_room_or_reading_it_the_answer_is_the_same() {
    for name in NAMES {
        let pick = pick_named(name);
        let room = pick.room();

        let read = pick.build(0.99 * room, None);
        let told = pick.build(0.99 * room, Some(room));

        assert_eq!(told.refusal, read.refusal, "{name}");
        assert_eq!(told.room, read.room, "{name}");
    }
}
