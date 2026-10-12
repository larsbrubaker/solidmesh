//! The `Section*` vertex-centric construction test classes (`SectionAssembly`, `SectionRoom`,
//! `SectionBuilder.TryBuildLoop`/`TryBuildRun` readings), one file each, grouped under one module
//! so the `bevel` area root stays inside the file-size gate. They reach the shared fixtures through
//! `crate::`, exactly as the root's own modules do.

#[path = "section_fan_remesh_tests.rs"]
mod section_fan_remesh_tests;
#[path = "section_many_holes_neighbour_tests.rs"]
mod section_many_holes_neighbour_tests;
#[path = "section_open_mouth_arc_tests.rs"]
mod section_open_mouth_arc_tests;
#[path = "section_room_tests.rs"]
mod section_room_tests;
#[path = "section_several_runs_tests.rs"]
mod section_several_runs_tests;
#[path = "section_split_chord_tests.rs"]
mod section_split_chord_tests;
#[path = "section_tapered_barrel_mouth_reading_tests.rs"]
mod section_tapered_barrel_mouth_reading_tests;
