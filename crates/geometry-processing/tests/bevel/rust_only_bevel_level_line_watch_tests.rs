//! Rust-only pins of `LevelLineRouting.Watch` and `WatchRoom` (`level_line_routing::watch`,
//! `watch_room`): the two test seams the app route hands its pick and its known room to. A watch
//! set on one thread sees the passes that thread's work runs on another, carried there by a
//! `LevelLineChoice`, and no other thread's.

use std::sync::{Arc, Mutex};

use geometry_processing::bevel::level_line::level_line_routing;
use geometry_processing::bevel::level_line::LevelLineChainWatch;
use vector_math::Vector3;

fn one_pick() -> LevelLineChainWatch {
    LevelLineChainWatch {
        pick: None,
        positions: vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)],
        indices: vec![(0, 1, 0)],
        chains: vec![vec![0, 1]],
        closed: vec![false],
        feature_edges: vec![(0, 1)],
        size: 0.5,
        fillet_segments: 8,
        round_profile: true,
        seam_epsilon: 0.0,
        sharp_turn_degrees: f64::INFINITY,
    }
}

#[test]
fn rust_only_the_chain_and_room_watches_are_handed_what_the_route_gives_across_threads() {
    let picks: Arc<Mutex<Vec<LevelLineChainWatch>>> = Arc::new(Mutex::new(Vec::new()));
    let rooms: Arc<Mutex<Vec<Option<f64>>>> = Arc::new(Mutex::new(Vec::new()));

    let seen = picks.clone();
    level_line_routing::set_watch(Some(Arc::new(move |watch: &LevelLineChainWatch| {
        seen.lock().unwrap().push(watch.clone())
    })));
    let seen = rooms.clone();
    level_line_routing::set_watch_room(Some(Arc::new(move |room: Option<f64>| {
        seen.lock().unwrap().push(room)
    })));

    // A thread this one handed nothing to has no watch.
    let elsewhere = std::thread::spawn(|| {
        (
            level_line_routing::watch().is_some(),
            level_line_routing::watch_room().is_some(),
        )
    })
    .join()
    .unwrap();
    assert_eq!(
        elsewhere,
        (false, false),
        "another test's builds are not watched"
    );

    // The route runs on a worker thread; the watch was set on this one and carried there.
    let choice = level_line_routing::LevelLineChoice::current();
    std::thread::spawn(move || {
        let _carried = choice.enter();
        if let Some(watch) = level_line_routing::watch() {
            watch(&one_pick());
        }
        if let Some(watch) = level_line_routing::watch_room() {
            watch(Some(0.25));
            watch(None);
        }
    })
    .join()
    .unwrap();

    level_line_routing::set_watch(None);
    level_line_routing::set_watch_room(None);

    assert_eq!(*picks.lock().unwrap(), vec![one_pick()]);
    assert_eq!(*rooms.lock().unwrap(), vec![Some(0.25), None]);
    assert!(level_line_routing::watch().is_none());
    assert!(level_line_routing::watch_room().is_none());
}
