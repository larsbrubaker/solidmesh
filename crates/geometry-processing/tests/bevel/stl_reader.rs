//! The one thing the Thingi10K sweep needs that the kernel has no business owning: an STL out of a
//! zip, welded into the positions and index triples `BandGap::build` asks for.
//!
//! The weld is by EXACT position - the doubles are widened from the file's own floats, so two
//! corners that were written as the same three floats compare equal and nothing else is merged.
//! That is what makes a facet soup manifold again without moving a single point, and it is why a
//! model whose author wrote one corner twice with a one-ulp difference comes out NOT closed and is
//! recorded as such rather than quietly healed. +0 and -0 weld, being the same point, and every NaN
//! welds to every NaN so a broken corner reads as one vertex rather than many.

use std::collections::HashMap;
use std::io::Read;
use std::path::Path;

use solidmesh_math::Vector3;

use crate::sweep_watchdog::Thrown;

type Corner = (f32, f32, f32);

/// A welded mesh: the positions and the triangles' index triples.
pub type Welded = (Vec<Vector3>, Vec<(i32, i32, i32)>);

/// Reads the single STL inside a zip file and welds it.
pub fn read_zip(zip_path: &Path) -> Result<Welded, Thrown> {
    let file = std::fs::File::open(zip_path)?;
    let mut archive = zip::ZipArchive::new(file)
        .map_err(|error| Thrown::new("InvalidDataException", error.to_string()))?;
    for index in 0..archive.len() {
        let mut entry = archive
            .by_index(index)
            .map_err(|error| Thrown::new("InvalidDataException", error.to_string()))?;
        if entry.size() == 0 {
            continue;
        }

        let mut bytes = Vec::new();
        entry.read_to_end(&mut bytes)?;
        return Ok(weld(&facets(&bytes)?));
    }

    Err(Thrown::new(
        "InvalidDataException",
        format!("{} holds no mesh", zip_path.display()),
    ))
}

/// Every triangle's three corners, in file order, as the floats the file carries.
fn facets(bytes: &[u8]) -> Result<Vec<[Corner; 3]>, Thrown> {
    if looks_binary(bytes) {
        let count = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
        let single = |at: usize| {
            f32::from_le_bytes([bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]])
        };
        let mut facets = Vec::with_capacity(count);
        let mut at = 84;
        for _ in 0..count {
            // the facet normal, which is not trusted: the winding is the truth.
            at += 12;
            let mut corners = [(0.0, 0.0, 0.0); 3];
            for corner in &mut corners {
                *corner = (single(at), single(at + 4), single(at + 8));
                at += 12;
            }

            // attribute byte count
            at += 2;
            facets.push(corners);
        }

        return Ok(facets);
    }

    ascii_facets(&String::from_utf8_lossy(bytes))
}

/// A binary STL is exactly its header, its count and that many fixed-size facets.
fn looks_binary(bytes: &[u8]) -> bool {
    if bytes.len() < 84 {
        return false;
    }

    let count = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as u64;
    84 + 50 * count == bytes.len() as u64
}

fn ascii_facets(text: &str) -> Result<Vec<[Corner; 3]>, Thrown> {
    let bad = |what: &str| Thrown::new("FormatException", what.to_string());
    let mut facets = Vec::new();
    let mut corners: Vec<Corner> = Vec::with_capacity(3);
    for line in text.split('\n') {
        let trimmed = line.trim();
        if trimmed.len() < 6 || !trimmed[..6].eq_ignore_ascii_case("vertex") {
            continue;
        }

        let mut parts = trimmed.split_whitespace().skip(1);
        let mut next = || -> Result<f32, Thrown> {
            parts
                .next()
                .ok_or_else(|| bad("a vertex line with fewer than three coordinates"))?
                .parse::<f32>()
                .map_err(|error| bad(&error.to_string()))
        };
        corners.push((next()?, next()?, next()?));

        if corners.len() == 3 {
            facets.push([corners[0], corners[1], corners[2]]);
            corners.clear();
        }
    }

    Ok(facets)
}

/// The bits a corner is welded by: exact float equality, with every NaN one value and -0 as 0.
fn weld_key(corner: Corner) -> (u32, u32, u32) {
    let bits = |v: f32| {
        if v.is_nan() {
            f32::NAN.to_bits()
        } else if v == 0.0 {
            0
        } else {
            v.to_bits()
        }
    };
    (bits(corner.0), bits(corner.1), bits(corner.2))
}

fn weld(facets: &[[Corner; 3]]) -> Welded {
    let mut index: HashMap<(u32, u32, u32), i32> = HashMap::with_capacity(facets.len() * 2);
    let mut positions = Vec::with_capacity(facets.len());
    let mut indices = Vec::with_capacity(facets.len());

    let mut of = |corner: Corner| -> i32 {
        *index.entry(weld_key(corner)).or_insert_with(|| {
            positions.push(Vector3::new(
                corner.0 as f64,
                corner.1 as f64,
                corner.2 as f64,
            ));
            positions.len() as i32 - 1
        })
    };

    for facet in facets {
        indices.push((of(facet[0]), of(facet[1]), of(facet[2])));
    }

    (positions, indices)
}
