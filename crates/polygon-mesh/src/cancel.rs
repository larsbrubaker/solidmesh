//! Cooperative cancellation for the long mesh operations (STL load and save, booleans).
//!
//! The same shape as `step-p21`'s `CancelToken`: a shared flag the caller sets and the work polls
//! at its checkpoints. It is a thin wrapper over manifold-rust's own `CancelToken` (also one shared
//! atomic flag) so that the token a caller hands a boolean reaches the kernel itself, with no
//! per-boolean registration to leak.

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
