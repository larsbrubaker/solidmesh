//! The CPU time the calling thread has used, for the cost rungs that port a C# `Stopwatch` bound
//! (`SurfacePlanCostTests`): a wall clock read beside the rest of the suite counts the time the
//! thread spent waiting for a core, so it fails on a loaded machine with the work unchanged; the
//! thread's own CPU time counts only the work. Test support only, not a C# port. Unix reads
//! `clock_gettime(CLOCK_THREAD_CPUTIME_ID)` (libc), Windows `GetThreadTimes` (windows-sys).

use std::time::Duration;

/// The CPU time (user and system) the calling thread has used so far.
#[cfg(unix)]
pub fn thread_cpu_time() -> Duration {
    let mut now = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: `now` is a valid, writable `timespec` that outlives the call, and
    // CLOCK_THREAD_CPUTIME_ID is a clock every supported Unix (Linux, macOS 10.12+) provides;
    // `clock_gettime` writes only through the pointer it is given.
    let status = unsafe { libc::clock_gettime(libc::CLOCK_THREAD_CPUTIME_ID, &mut now) };
    assert_eq!(status, 0, "clock_gettime(CLOCK_THREAD_CPUTIME_ID) failed");
    Duration::new(now.tv_sec as u64, now.tv_nsec as u32)
}

/// The CPU time (user and kernel) the calling thread has used so far.
#[cfg(windows)]
pub fn thread_cpu_time() -> Duration {
    use windows_sys::Win32::Foundation::FILETIME;
    use windows_sys::Win32::System::Threading::{GetCurrentThread, GetThreadTimes};

    let mut creation = FILETIME::default();
    let mut exit = FILETIME::default();
    let mut kernel = FILETIME::default();
    let mut user = FILETIME::default();
    // SAFETY: `GetCurrentThread` returns a pseudo-handle for the calling thread that needs no
    // closing and is valid for the whole call, and the four `FILETIME`s are valid, writable and
    // outlive it; `GetThreadTimes` writes only through the pointers it is given.
    let ok = unsafe {
        GetThreadTimes(
            GetCurrentThread(),
            &mut creation,
            &mut exit,
            &mut kernel,
            &mut user,
        )
    };
    assert_ne!(ok, 0, "GetThreadTimes failed");
    // FILETIME counts 100 ns ticks.
    let ticks = |t: FILETIME| (u64::from(t.dwHighDateTime) << 32) | u64::from(t.dwLowDateTime);
    Duration::from_nanos((ticks(kernel) + ticks(user)) * 100)
}
