//! [`RenderWork`], a plain count of what a render pass did: rays by kind and a histogram of how
//! far paths went, so a later change can compare tree designs on real renders and compute how
//! much of the work is coherent. Counts only; the renderer has no clock.

/// How many bounce counts the histogram keeps; paths that scattered this many times or more
/// share the last bin.
pub const BOUNCE_HISTOGRAM_BINS: usize = 16;

/// What one or more render passes cost. Plain counters, summed with [`RenderWork::add`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RenderWork {
    /// Camera rays: the most coherent rays there are.
    pub camera_rays: u64,
    /// Shadow rays from a camera ray's first hit, which share the camera rays' coherence.
    pub first_hit_shadow_rays: u64,
    /// Shadow rays from later bounces.
    pub later_shadow_rays: u64,
    /// Rays continuing a path after a bounce: incoherent.
    pub bounce_rays: u64,
    /// Rays continuing a straight line past a part that was not there.
    pub pass_through_rays: u64,
    /// Traces a shadow ray takes after its first, the first being counted as a shadow ray: one
    /// past each face of glass it crosses when the sun goes straight through glass
    /// (`SunThroughGlass`).
    pub sun_line_rays: u64,
    /// Pixels that received a sample.
    pub pixels_rendered: u64,
    /// Samples dropped because a value overflowed or turned to NaN (see
    /// [`AccumulatedPixel::add_sample`](super::AccumulatedPixel::add_sample)).
    pub dropped_samples: u64,
    /// Whether the cancel token stopped this pass before it finished.
    pub cancelled: bool,
    /// Paths by how many times they scattered.
    pub bounce_histogram: [u64; BOUNCE_HISTOGRAM_BINS],
}

impl RenderWork {
    /// Every ray of every kind.
    pub fn total_rays(&self) -> u64 {
        self.camera_rays
            + self.first_hit_shadow_rays
            + self.later_shadow_rays
            + self.bounce_rays
            + self.pass_through_rays
            + self.sun_line_rays
    }

    /// The fraction of rays that are coherent (camera rays and shadow rays from their first hit),
    /// or 0 when there were none.
    pub fn coherent_fraction(&self) -> f64 {
        let total = self.total_rays();
        if total == 0 {
            0.0
        } else {
            // Ray counts stay below 2^53, so both convert exactly.
            (self.camera_rays + self.first_hit_shadow_rays) as f64 / total as f64
        }
    }

    /// `other` added into this count.
    pub fn add(&mut self, other: &RenderWork) {
        self.camera_rays += other.camera_rays;
        self.first_hit_shadow_rays += other.first_hit_shadow_rays;
        self.later_shadow_rays += other.later_shadow_rays;
        self.bounce_rays += other.bounce_rays;
        self.pass_through_rays += other.pass_through_rays;
        self.sun_line_rays += other.sun_line_rays;
        self.pixels_rendered += other.pixels_rendered;
        self.dropped_samples += other.dropped_samples;
        self.cancelled |= other.cancelled;
        for (bin, count) in self
            .bounce_histogram
            .iter_mut()
            .zip(other.bounce_histogram.iter())
        {
            *bin += count;
        }
    }

    /// Records a finished path that scattered `scatters` times.
    pub(super) fn record_path(&mut self, scatters: u32) {
        // A bounce count is far below usize's range; the last bin takes every path at or past it.
        let bin = (scatters as usize).min(BOUNCE_HISTOGRAM_BINS - 1);
        self.bounce_histogram[bin] += 1;
    }
}
