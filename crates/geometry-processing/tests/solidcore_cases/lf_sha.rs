//! The one hash the solidcore cases are pinned by, shared by the runner's pin check
//! (`case_file.rs`) and the sync tool (`examples/sync_solidcore_cases.rs`, through `#[path]`), so
//! the two can never disagree about whether a file changed.
//!
//! It is SHA-256 of the file with every CRLF read as LF: the files are pinned as solidcore wrote
//! them, with LF line ends, and a checkout that turns those into CRLF has not changed a case.

use sha2::{Digest, Sha256};

/// The lower-case hex SHA-256 of `bytes` with every CRLF read as LF.
pub fn lf_sha256(bytes: &[u8]) -> String {
    let mut lf = Vec::with_capacity(bytes.len());
    for (i, byte) in bytes.iter().enumerate() {
        // 13 is CR and 10 is LF.
        if *byte == 13 && bytes.get(i + 1) == Some(&10) {
            continue;
        }
        lf.push(*byte);
    }
    Sha256::digest(&lf)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}
