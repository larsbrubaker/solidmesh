//! Cooperative cancellation for the long mesh operations (STL load and save, booleans).
//!
//! The same shape as `step-p21`'s `CancelToken`: a shared flag the caller sets and the work polls
//! at its checkpoints. It is a thin wrapper over manifold-rust's own `CancelToken` (also one shared
//! atomic flag) so that the token a caller hands a boolean reaches the kernel itself, with no
//! per-boolean registration to leak. Work that has nothing to report but the cancel itself
//! answers [`Cancelled`].

/// A cancel flag shared between the caller and the work: cloning is cheap (one `Arc`), and
/// cancelling any clone cancels them all. The default token is never cancelled unless someone calls
/// [`CancelToken::cancel`] on it or a clone.
#[derive(Debug, Clone, Default)]
pub struct CancelToken(manifold_rust::cancel::CancelToken);

impl CancelToken {
    /// A fresh, uncancelled token.
    pub fn new() -> Self {
        Self::default()
    }

    /// Requests cancellation; every clone sees it.
    pub fn cancel(&self) {
        self.0.cancel();
    }

    /// Whether cancellation was requested.
    pub fn is_cancelled(&self) -> bool {
        self.0.is_cancelled()
    }

    /// The kernel's view of the same flag, for the booleans in [`crate::csg`].
    pub(crate) fn kernel_token(&self) -> &manifold_rust::cancel::CancelToken {
        &self.0
    }
}

/// The answer of work that was cancelled through its [`CancelToken`] and has nothing else to
/// say: its result is not to be used.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Cancelled;

impl std::fmt::Display for Cancelled {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("the operation was cancelled")
    }
}

impl std::error::Error for Cancelled {}
