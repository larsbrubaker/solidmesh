//! The sections switch and the route record as builds on threads side by side see them: a test's
//! pin holds for its own thread's builds, and the route a build took is read by the thread that
//! asked for it, so two tests pinning opposite constructions at once each read their own answer.
//! The route builds are the straight bore mouth of `section_routing_tests`.

use std::sync::{Arc, Barrier, Mutex};

use geometry_processing::bevel::sections::section_routing::{
    self, set_last_route, set_route_observed, SectionRoutingContext,
};
use vector_math::Matrix4X4;

use crate::section_routing_tests::{try_bevel_the_bore_mouth, SIZE};
use crate::surface_test_build::{pinned_to_the_legacy_construction, pinned_to_the_sections};

/// Pins the switch to `on`, waits until the other thread has pinned its own, builds the bore
/// mouth, waits until the other thread has built too, then answers the route this thread reads.
fn route_of_a_build_pinned(on: bool, pinned: &Barrier, built: &Barrier) -> Option<String> {
    let _pin = if on {
        pinned_to_the_sections()
    } else {
        pinned_to_the_legacy_construction()
    };
    pinned.wait();
    let mouth = try_bevel_the_bore_mouth(40, Matrix4X4::IDENTITY, SIZE, true);
    built.wait();
    assert!(mouth.is_ok(), "the straight bore mouth builds either way");
    section_routing::last_route()
}

#[test]
fn two_threads_pinning_opposite_constructions_each_route_and_read_their_own() {
    let pinned = Arc::new(Barrier::new(2));
    let built = Arc::new(Barrier::new(2));
    let threads: Vec<_> = [true, false]
        .into_iter()
        .map(|on| {
            let pinned = Arc::clone(&pinned);
            let built = Arc::clone(&built);
            (
                on,
                std::thread::spawn(move || route_of_a_build_pinned(on, &pinned, &built)),
            )
        })
        .collect();
    for (on, thread) in threads {
        let route = thread.join().expect("the building thread runs");
        let expected = if on {
            "sections"
        } else {
            "legacy: the flag is off"
        };
        assert_eq!(
            route.as_deref(),
            Some(expected),
            "the build pinned {on} routes as its own test asked, and its test reads that route"
        );
    }
}

#[test]
fn a_carried_context_routes_and_reports_as_the_thread_that_asked() {
    set_last_route(None);
    let heard = Arc::new(Mutex::new(Vec::<String>::new()));
    let into = Arc::clone(&heard);
    set_route_observed(Some(Arc::new(move |route: Option<&str>| {
        into.lock()
            .unwrap()
            .push(route.unwrap_or_default().to_string());
    })));
    let _pin = pinned_to_the_sections();

    let context = SectionRoutingContext::current();
    let (inside, after) = std::thread::spawn(move || {
        let inside = {
            let _carried = context.enter();
            set_last_route(Some("from the worker".to_string()));
            section_routing::use_sections()
        };
        (inside, section_routing::last_route())
    })
    .join()
    .expect("the worker runs");

    let here = section_routing::last_route();
    set_route_observed(None);
    assert!(inside, "the worker reads the switch its asker pinned");
    assert_eq!(
        here.as_deref(),
        Some("from the worker"),
        "the route a worker wrote is the asker's to read"
    );
    assert_eq!(
        heard.lock().unwrap().as_slice(),
        ["from the worker".to_string()],
        "the asker's observer hears the worker's route"
    );
    assert_eq!(
        after, None,
        "once the carried context ends, the worker's own record is untouched"
    );
}

#[test]
fn a_context_entered_with_the_switch_set_holds_only_while_it_is_entered() {
    let _pin = pinned_to_the_sections();
    {
        let _off = SectionRoutingContext::current()
            .with_use_sections(false)
            .enter();
        assert!(
            !section_routing::use_sections(),
            "the inner build routes legacy"
        );
    }
    assert!(
        section_routing::use_sections(),
        "the thread's own setting is back once the inner build's scope ends"
    );
}
