//! The picture a mesh face wears, as the mesh kernel sees it: [`TexturePixels`].
//!
//! The kernel never reads or draws a picture's pixels. It shares one picture between many faces
//! (`Arc<dyn TexturePixels>`), tells two pictures apart by allocation (`Arc::ptr_eq`, which
//! ignores the trait object's vtable, so the identity is the same as through the concrete type),
//! and folds a picture's size and bytes into the mesh's long hash. That is all this trait offers.
//! agg-path's byte image implements it, and agg-path's `texture_image` and `texture_image_arc`
//! downcast back through [`TexturePixels::as_any`] / [`TexturePixels::into_any_arc`] for code that
//! draws.

use std::any::Any;
use std::fmt::Debug;
use std::sync::Arc;

/// A picture's size and raw bytes, plus the downcasts back to its concrete type.
pub trait TexturePixels: Any + Send + Sync + Debug {
    /// Width in pixels.
    fn width(&self) -> i32;

    /// Height in pixels.
    fn height(&self) -> i32;

    /// The pixel bytes in the picture's own layout. The kernel only hashes them, so any stable
    /// layout works; the same picture must always return the same bytes.
    fn pixel_bytes(&self) -> &[u8];

    /// The picture as `Any`, for borrowing it back as its concrete type.
    fn as_any(&self) -> &dyn Any;

    /// The shared picture as `Any`, for getting the concrete `Arc` back (same allocation).
    fn into_any_arc(self: Arc<Self>) -> Arc<dyn Any + Send + Sync>;
}
