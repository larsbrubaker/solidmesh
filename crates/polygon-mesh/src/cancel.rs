//! Cooperative cancellation for the long mesh operations (STL load and save today).
//!
//! The same shape as `step-p21`'s `CancelToken` (a port of C#'s `CancellationToken`): a shared
//! flag the caller sets and the work polls at its checkpoints. polygon-mesh keeps its own copy
//! rather than depending on the STEP kernel for one atomic bool.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

/// A cancel flag shared between the caller and the work: cloning is cheap (one `Arc`), and
/// cancelling any clone cancels them all. The default token is never cancelled unless someone
/// calls [`CancelToken::cancel`] on it or a clone (C# `CancellationToken.None`).
#[derive(Debug, Clone, Default)]
pub struct CancelToken(Arc<AtomicBool>);

impl CancelToken {
    /// A fresh, uncancelled token.
    pub fn new() -> Self {
        Self::default()
    }

    /// Requests cancellation; every clone sees it.
    pub fn cancel(&self) {
        self.0.store(true, Ordering::Relaxed);
    }

    /// Whether cancellation was requested (C# `IsCancellationRequested`).
    pub fn is_cancelled(&self) -> bool {
        self.0.load(Ordering::Relaxed)
    }
}
