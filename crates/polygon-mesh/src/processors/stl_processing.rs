//! STL reading (ASCII and binary). Ports the load half of agg-sharp
//! `PolygonMesh/Processors/StlProcessing.cs`.
//!
//! C# writes the parse as an iterator that pauses every [`FACES_PER_YIELD`] faces so its async
//! entry point can hand the browser's UI thread back between chunks. Here that walk is
//! [`StlParse`]: the caller drives it chunk by chunk with [`StlParse::next_chunk`] (the web
//! shell between frames), or runs it out with [`parse_file_contents`]. Progress is reported and
//! cancellation polled at every chunk boundary (see divergence 40). Where C# returned `null`
//! (too short, truncated, cancelled, an exception) these return an [`StlError`].

use std::collections::HashMap;
use std::fmt;
use std::io::Read;
use std::path::Path;

use vector_math::{number_format, Vector3, Vector3Float};

use crate::cancel::CancelToken;
use crate::mesh::{f64_key, Mesh};

/// How many faces the parse reads before it offers the caller its thread back: roughly what
/// one frame of wasm parsing gets through, so a progress bar moves while a large STL comes in.
pub const FACES_PER_YIELD: i32 = 5000;

/// A progress callback: ratio done (0..1) and a status message (C# `Action<double, string>`).
pub type Progress<'a> = &'a mut dyn FnMut(f64, &str);

/// Why an STL produced no mesh.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StlError {
    /// The file is not named `.stl` ([`load_file`] only).
    NotStl,
    /// 80 bytes or fewer: not even a binary header.
    TooShort,
    /// A binary STL with no triangles, or fewer bytes than its triangle count needs.
    Truncated,
    /// An ASCII `vertex` line with fewer than three numbers.
    Malformed { line: String },
    /// The cancel token was cancelled.
    Cancelled,
    /// The reader or file failed.
    Io(String),
}

impl fmt::Display for StlError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            StlError::NotStl => f.write_str("not an .stl file"),
            StlError::TooShort => f.write_str("the file is too short to be an STL"),
            StlError::Truncated => f.write_str("the binary STL has no triangles or is truncated"),
            StlError::Malformed { line } => write!(f, "malformed STL line: {line}"),
            StlError::Cancelled => f.write_str("cancelled"),
            StlError::Io(message) => write!(f, "cannot read the STL: {message}"),
        }
    }
}

impl std::error::Error for StlError {}

/// What one [`StlParse::next_chunk`] step produced.
#[derive(Debug)]
pub enum StlChunk {
    /// A chunk boundary: this many faces read so far; call again.
    Continue { faces_read: i32 },
    /// The parse finished.
    Done(Mesh),
}

enum Format {
    Ascii {
        text: String,
        position: usize,
    },
    Binary {
        num_triangles: u32,
        triangle: u32,
        position: usize,
    },
}

/// An STL parse in progress over an in-memory file.
pub struct StlParse<'a> {
    data: &'a [u8],
    format: Format,
    mesh: Mesh,
    position_to_index: HashMap<(u64, u64, u64), i32>,
    vector_index: i32,
    corners: [Vector3; 3],
    finished: bool,
}

/// Loads `file_name` if its extension is `.stl` (any case).
pub fn load_file(
    file_name: &Path,
    cancel: &CancelToken,
    report_progress: Option<Progress<'_>>,
) -> Result<Mesh, StlError> {
    let is_stl = file_name
        .extension()
        .and_then(|e| e.to_str())
        .is_some_and(|e| e.eq_ignore_ascii_case("stl"));
    if !is_stl {
        return Err(StlError::NotStl);
    }
    let data = std::fs::read(file_name).map_err(|e| StlError::Io(e.to_string()))?;
    parse_file_contents(&data, cancel, report_progress)
}

/// Reads `reader` to its end and parses it.
pub fn load<R: Read>(
    mut reader: R,
    cancel: &CancelToken,
    report_progress: Option<Progress<'_>>,
) -> Result<Mesh, StlError> {
    let mut data = Vec::new();
    reader
        .read_to_end(&mut data)
        .map_err(|e| StlError::Io(e.to_string()))?;
    parse_file_contents(&data, cancel, report_progress)
}

/// Parses a whole STL, running the chunk boundaries out.
pub fn parse_file_contents(
    data: &[u8],
    cancel: &CancelToken,
    mut report_progress: Option<Progress<'_>>,
) -> Result<Mesh, StlError> {
    let mut parse = StlParse::new(data)?;
    loop {
        // Re-borrow the callback for each step without giving it up.
        let progress = report_progress.as_mut().map(|p| &mut **p as Progress<'_>);
        if let StlChunk::Done(mesh) = parse.next_chunk(cancel, progress)? {
            return Ok(mesh);
        }
    }
}

impl<'a> StlParse<'a> {
    /// Starts a parse: checks the size and decides ASCII or binary from the first 160 bytes.
    pub fn new(data: &'a [u8]) -> Result<Self, StlError> {
        if data.len() <= 80 {
            return Err(StlError::TooShort);
        }

        let mut first160_bytes = [0u8; 160];
        let read = data.len().min(160);
        first160_bytes[..read].copy_from_slice(&data[..read]);
        let byte_order_mark = [0xEF, 0xBB, 0xBF];
        let start_of_string = if first160_bytes[..3] == byte_order_mark {
            3
        } else {
            0
        };
        let first160 = String::from_utf8_lossy(&first160_bytes[start_of_string..]).to_lowercase();

        let format = if first160.starts_with("solid") && first160.contains("facet") {
            // StreamReader skips a UTF-8 byte order mark.
            let body = data.strip_prefix(&byte_order_mark[..]).unwrap_or(data);
            Format::Ascii {
                text: String::from_utf8_lossy(body).into_owned(),
                position: 0,
            }
        } else {
            // the number of triangles follows the 80 byte header
            let num_triangles = u32::from_le_bytes([data[80], data[81], data[82], data[83]]);
            let bytes_for_triangles = u64::from(num_triangles) * (3 * 4 + 3 * 4 * 3 + 2);
            if (data.len() as u64) < 84 + bytes_for_triangles || num_triangles < 1 {
                return Err(StlError::Truncated);
            }
            Format::Binary {
                num_triangles,
                triangle: 0,
                position: 84,
            }
        };

        Ok(StlParse {
            data,
            format,
            mesh: Mesh::new(),
            position_to_index: HashMap::new(),
            vector_index: 0,
            corners: [Vector3::ZERO; 3],
            finished: false,
        })
    }

    /// Reads up to [`FACES_PER_YIELD`] more faces. At each chunk boundary progress is
    /// reported ("Loading Polygons") and `cancel` polled; at the end progress is reported as
    /// `(1, "")` and the mesh returned.
    pub fn next_chunk(
        &mut self,
        cancel: &CancelToken,
        mut report_progress: Option<Progress<'_>>,
    ) -> Result<StlChunk, StlError> {
        if self.finished {
            return Ok(StlChunk::Done(Mesh::new()));
        }
        let faces_at_start = self.mesh.faces.len() as i32;
        let done = match self.format {
            Format::Ascii { .. } => self.ascii_chunk(faces_at_start)?,
            Format::Binary { .. } => self.binary_chunk(faces_at_start),
        };

        if done {
            if let Some(report) = report_progress.as_mut() {
                report(1.0, "");
            }
            if cancel.is_cancelled() {
                return Err(StlError::Cancelled);
            }
            self.finished = true;
            return Ok(StlChunk::Done(std::mem::take(&mut self.mesh)));
        }

        if let Some(report) = report_progress.as_mut() {
            report(self.progress_ratio(), "Loading Polygons");
        }
        if cancel.is_cancelled() {
            return Err(StlError::Cancelled);
        }
        Ok(StlChunk::Continue {
            faces_read: self.mesh.faces.len() as i32,
        })
    }

    fn progress_ratio(&self) -> f64 {
        match self.format {
            Format::Ascii { ref text, position } => position as f64 / text.len().max(1) as f64,
            Format::Binary {
                num_triangles,
                triangle,
                ..
            } => f64::from(triangle) / f64::from(num_triangles),
        }
    }

    fn get_index(&mut self, position: Vector3) -> i32 {
        let key = (
            f64_key(position.x),
            f64_key(position.y),
            f64_key(position.z),
        );
        let vertices = &mut self.mesh.vertices;
        *self.position_to_index.entry(key).or_insert_with(|| {
            vertices.push(Vector3Float::from_vector3(position));
            vertices.len() as i32 - 1
        })
    }

    fn add_triangle(&mut self, corners: [Vector3; 3]) {
        if !Vector3::collinear(corners[0], corners[1], corners[2]) {
            let iv0 = self.get_index(corners[0]);
            let iv1 = self.get_index(corners[1]);
            let iv2 = self.get_index(corners[2]);
            self.mesh.faces.add(iv0, iv1, iv2, &self.mesh.vertices);
        }
    }

    /// Returns true at the end of the text.
    fn ascii_chunk(&mut self, faces_at_start: i32) -> Result<bool, StlError> {
        loop {
            let Some(line) = self.next_ascii_line() else {
                return Ok(true);
            };
            let line = line.trim();
            if line.len() >= 6 && line.as_bytes()[..6].eq_ignore_ascii_case(b"vertex") {
                let vector = convert(line)?;
                self.corners[self.vector_index as usize] = vector;
                self.vector_index += 1;
                if self.vector_index == 3 {
                    self.add_triangle(self.corners);
                    self.vector_index = 0;
                }
            }
            if self.mesh.faces.len() as i32 - faces_at_start >= FACES_PER_YIELD {
                return Ok(false);
            }
        }
    }

    /// The next line (C# `StreamReader.ReadLine`: ends at "\n", "\r" or "\r\n").
    fn next_ascii_line(&mut self) -> Option<String> {
        let Format::Ascii { text, position } = &mut self.format else {
            return None;
        };
        if *position >= text.len() {
            return None;
        }
        let rest = &text[*position..];
        let end = rest.find(['\r', '\n']).unwrap_or(rest.len());
        let line = rest[..end].to_string();
        let mut next = *position + end;
        if rest[end..].starts_with("\r\n") {
            next += 2;
        } else if end < rest.len() {
            next += 1;
        }
        *position = next;
        Some(line)
    }

    /// Returns true after the last triangle.
    fn binary_chunk(&mut self, faces_at_start: i32) -> bool {
        loop {
            let Format::Binary {
                num_triangles,
                triangle,
                position,
            } = &mut self.format
            else {
                return true;
            };
            if *triangle >= *num_triangles {
                return true;
            }
            // skip the normal
            let mut current = *position + 3 * 4;
            let mut corners = [Vector3::ZERO; 3];
            for corner in corners.iter_mut() {
                let f = |offset: usize| {
                    let at = current + offset * 4;
                    f64::from(f32::from_le_bytes([
                        self.data[at],
                        self.data[at + 1],
                        self.data[at + 2],
                        self.data[at + 3],
                    ]))
                };
                *corner = Vector3::new(f(0), f(1), f(2));
                current += 3 * 4;
            }
            current += 2; // skip the attribute
            *position = current;
            *triangle += 1;

            self.add_triangle(corners);
            if self.mesh.faces.len() as i32 - faces_at_start >= FACES_PER_YIELD {
                return false;
            }
        }
    }
}

/// The three numbers after "vertex" (C# `Convert`): each runs from the first non-space to the
/// next space; one that does not parse is 0, as `double.TryParse` leaves it.
fn convert(line: &str) -> Result<Vector3, StlError> {
    let bytes = line.as_bytes();
    let mut current_position = "vertex".len();
    let mut values = [0.0; 3];
    for value in values.iter_mut() {
        while bytes.get(current_position) == Some(&b' ') {
            current_position += 1;
        }
        // C# indexes past the end here and the load fails.
        if current_position >= bytes.len() {
            return Err(StlError::Malformed {
                line: line.to_string(),
            });
        }
        let start = current_position;
        while current_position < bytes.len() && bytes[current_position] != b' ' {
            current_position += 1;
        }
        let number = String::from_utf8_lossy(&bytes[start..current_position]);
        *value = parse_stl_number(&number);
    }
    Ok(Vector3::new(values[0], values[1], values[2]))
}

/// C# `double.TryParse` with `AllowDecimalPoint | AllowExponent | AllowLeadingSign` and the
/// invariant culture: like `NumberStyles.Float` but without surrounding white space.
fn parse_stl_number(number: &str) -> f64 {
    if number.chars().any(char::is_whitespace) {
        return 0.0;
    }
    number_format::parse_f64(number)
}
