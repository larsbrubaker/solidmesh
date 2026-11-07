//! How a mesh is written: ASCII or binary, CSG handling, metadata and a progress callback.

use std::fmt;
use std::sync::Arc;

/// How overlapping volumes are combined on export (used by the AMF/3MF writers).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum CsgOption {
    #[default]
    SimpleInsertVolumes,
    DoCsgMerge,
}

/// Text or binary output.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum OutputType {
    Ascii,
    #[default]
    Binary,
}

/// A progress callback: ratio done (0..1) and a status message.
pub type ReportProgress = Arc<dyn Fn(f64, &str) + Send + Sync>;

/// Output options for the mesh writers.
#[derive(Clone, Default)]
pub struct MeshOutputSettings {
    pub output_type_setting: OutputType,
    /// Key/value metadata in insertion order (a repeated key replaces the earlier value in
    /// place, so a caller setting a key twice gets the last value at the first position).
    pub meta_data_key_value: Vec<(String, String)>,
    pub csg_option_state: CsgOption,
    pub report_progress: Option<ReportProgress>,
}

impl MeshOutputSettings {
    /// Binary output, no metadata.
    pub fn new() -> Self {
        Self::default()
    }

    /// Default settings with the given CSG option.
    pub fn with_csg_option(csg_option: CsgOption) -> Self {
        MeshOutputSettings {
            csg_option_state: csg_option,
            ..Self::default()
        }
    }

    /// The given output type, metadata from alternating key/value strings, and an optional
    /// progress callback.
    pub fn with_output_type(
        output_type_setting: OutputType,
        meta_data_key_value_pairs: &[&str],
        report_progress: Option<ReportProgress>,
    ) -> Self {
        let mut settings = MeshOutputSettings {
            output_type_setting,
            report_progress,
            ..Self::default()
        };
        for pair in meta_data_key_value_pairs.as_chunks::<2>().0 {
            settings.set_meta_data(pair[0], pair[1]);
        }
        settings
    }

    fn set_meta_data(&mut self, key: &str, value: &str) {
        match self.meta_data_key_value.iter_mut().find(|(k, _)| k == key) {
            Some(entry) => entry.1 = value.to_string(),
            None => self
                .meta_data_key_value
                .push((key.to_string(), value.to_string())),
        }
    }
}

impl fmt::Debug for MeshOutputSettings {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MeshOutputSettings")
            .field("output_type_setting", &self.output_type_setting)
            .field("meta_data_key_value", &self.meta_data_key_value)
            .field("csg_option_state", &self.csg_option_state)
            .field("report_progress", &self.report_progress.is_some())
            .finish()
    }
}
