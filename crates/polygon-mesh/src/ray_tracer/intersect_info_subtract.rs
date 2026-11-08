//! View-space CSG on hit lists: subtracting one from another, reading a saved list back, and the
//! distance order both are sorted in. [`super::Difference`] gathers every front and back face hit
//! of its primary and subtract shapes along a ray, sorts both by distance, and removes from the
//! primary's list the spans the subtract shapes cover, adding faces where a cut opens the primary.
//!
//! Malformed lists are an `Err` that carries the cause; nothing is written to disk, because a
//! library has no business writing into the working directory and the browser has none.

use std::cmp::Ordering;

use solidmesh_math::{IntersectionType, Ray};

use super::{IntersectInfo, RayTracerError};

/// Nearer first, and at the same distance a front face before a back face. Two different hit
/// types that are not a front/back pair (a `None` in the list) are an error: the list is
/// malformed.
pub fn compare_intersect_info_on_distance(
    a: &IntersectInfo<'_>,
    b: &IntersectInfo<'_>,
) -> Result<Ordering, RayTracerError> {
    if a.distance_to_hit > b.distance_to_hit {
        return Ok(Ordering::Greater);
    } else if a.distance_to_hit < b.distance_to_hit {
        return Ok(Ordering::Less);
    }
    if a.hit_type != b.hit_type {
        return match (a.hit_type, b.hit_type) {
            (IntersectionType::FrontFace, IntersectionType::BackFace) => Ok(Ordering::Less),
            (IntersectionType::BackFace, IntersectionType::FrontFace) => Ok(Ordering::Greater),
            _ => Err(RayTracerError::BadHitList(
                "hit types must be front or back faces to sort",
            )),
        };
    }
    Ok(Ordering::Equal)
}

/// Sorts `list` with [`compare_intersect_info_on_distance`]. The sort is stable, so hits tied
/// on distance and type keep the order the iterator produced them in.
pub(crate) fn sort_on_distance(list: &mut [IntersectInfo<'_>]) -> Result<(), RayTracerError> {
    let mut error = None;
    list.sort_by(|a, b| {
        compare_intersect_info_on_distance(a, b).unwrap_or_else(|e| {
            error = Some(e);
            Ordering::Equal
        })
    });
    match error {
        Some(e) => Err(e),
        None => Ok(()),
    }
}

impl<'a> IntersectInfo<'a> {
    /// A copy of the hit's type, object, position, normal and distance, with the color left
    /// unset.
    pub fn copy_hit(copy_info: &IntersectInfo<'a>) -> Self {
        IntersectInfo {
            hit_type: copy_info.hit_type,
            closest_hit_object: copy_info.closest_hit_object,
            hit_position: copy_info.hit_position,
            normal_at_hit: copy_info.normal_at_hit,
            distance_to_hit: copy_info.distance_to_hit,
            ..IntersectInfo::new()
        }
    }

    /// Reads a count line, then that many `HitType, distance` lines, into `list_to_populate`
    /// (the debugging dump format of saved hit lists). Unparsable numbers read as 0; a hit
    /// type other than `FrontFace`/`BackFace`, or a missing line, is an error.
    pub fn read_in_list<'s>(
        list_to_populate: &mut Vec<IntersectInfo<'a>>,
        lines: &mut impl Iterator<Item = &'s str>,
    ) -> Result<(), RayTracerError> {
        let count: i32 = lines
            .next()
            .and_then(|line| line.trim().parse().ok())
            .unwrap_or(0);
        for _ in 0..count {
            let mut info = IntersectInfo::new();
            let line = lines
                .next()
                .ok_or(RayTracerError::BadHitList("the list ended early"))?;
            let mut strings = line.split(',');
            let hit_type = strings.next().unwrap_or("");
            if hit_type == "FrontFace" {
                info.hit_type = IntersectionType::FrontFace;
            } else {
                if hit_type != "BackFace" {
                    return Err(RayTracerError::BadHitList("Has to be back or front."));
                }

                info.hit_type = IntersectionType::BackFace;
            }

            info.distance_to_hit = strings
                .next()
                .and_then(|text| text.trim().parse().ok())
                .unwrap_or(0.0);
            list_to_populate.push(info);
        }

        Ok(())
    }

    /// `all_primary` with the spans covered by `all_subtract` removed, into `result`. Both
    /// lists are sorted by distance and alternate front and back faces.
    pub fn subtract(
        all_primary: &[IntersectInfo<'a>],
        all_subtract: &[IntersectInfo<'a>],
        result: &mut Vec<IntersectInfo<'a>>,
    ) -> Result<(), RayTracerError> {
        result.clear();
        result.extend_from_slice(all_primary);

        let mut next_subtract_index = 0;
        while let Some((region_start_index, region_end_index)) =
            get_remove_region(all_subtract, next_subtract_index)?
        {
            subtract_remove_region(
                result,
                &all_subtract[region_start_index],
                &all_subtract[region_end_index],
            )?;

            next_subtract_index = region_end_index + 1;
        }

        Ok(())
    }
}

fn subtract_remove_region<'a>(
    remove_from: &mut Vec<IntersectInfo<'a>>,
    start_remove_info: &IntersectInfo<'a>,
    end_remove_info: &IntersectInfo<'a>,
) -> Result<(), RayTracerError> {
    if start_remove_info.hit_type != IntersectionType::FrontFace
        || end_remove_info.hit_type != IntersectionType::BackFace
    {
        return Err(RayTracerError::BadHitList(
            "These should always be set right.",
        ));
    }

    let start = start_remove_info.distance_to_hit - Ray::SAME_SURFACE_OFFSET;
    let end = end_remove_info.distance_to_hit + Ray::SAME_SURFACE_OFFSET;

    let mut inside_count = 0.0;
    // A signed index: removing an entry steps it back to re-check the same slot.
    let mut primary_index: isize = 0;
    while (primary_index as usize) < remove_from.len() {
        let index = primary_index as usize;
        let primary_info = remove_from[index];
        if primary_info.hit_type == IntersectionType::FrontFace {
            inside_count += 1.0;

            if primary_info.distance_to_hit < start {
                // We are in front of the remove start. If the next backface is behind the
                // remove start, add a back face at the remove start. There should always be a
                // back face after a front face; a missing one is a malformed list.
                let next = remove_from
                    .get(index + 1)
                    .ok_or(RayTracerError::BadHitList(
                        "a front face has no matching back face",
                    ))?;
                if next.distance_to_hit > start {
                    let mut new_back_face = IntersectInfo::copy_hit(start_remove_info);
                    new_back_face.hit_type = IntersectionType::BackFace;
                    remove_from.insert(index + 1, new_back_face); // it goes after the current index
                    primary_index += 1;
                }
            } else if primary_info.distance_to_hit >= start && primary_info.distance_to_hit < end {
                // the front face is within the remove so remove it.
                remove_from.remove(index);
                // need to check the same index again.
                primary_index -= 1;
            } else if primary_info.distance_to_hit >= end {
                // we have gone past the remove region just return.
                return Ok(());
            }
        } else if primary_info.hit_type == IntersectionType::BackFace {
            if inside_count == 0.0 {
                return Err(RayTracerError::BadHitList(
                    "You should not have a back face without a matching front face.",
                ));
            }
            inside_count -= 1.0;
            let within = primary_info.distance_to_hit > start && primary_info.distance_to_hit < end;
            if inside_count == 0.0 {
                if within {
                    // the back face is within the remove so remove it.
                    remove_from.remove(index);
                    // need to check the same index again.
                    primary_index -= 1;
                } else if primary_info.distance_to_hit >= end {
                    // the back face is past the remove distance.
                    // Add the remove back face as a front face to the primary
                    // We should be guaranteed that the front face is within the remove distance
                    // because if it was we should have returned above.
                    let mut new_front_face = IntersectInfo::copy_hit(end_remove_info);
                    new_front_face.hit_type = IntersectionType::FrontFace;
                    remove_from.insert(index, new_front_face);
                    primary_index += 1;
                }
            } else if within {
                // the back face is within the remove so remove it.
                remove_from.remove(index);
                // need to check the same index again.
                primary_index -= 1;
            }
        } else {
            return Err(RayTracerError::BadHitList(
                "There should be no 'none's in the hit types.",
            ));
        }

        primary_index += 1;
    }

    Ok(())
}

/// The next span of `subtract_list` from `start_index` that is inside the subtract shapes:
/// `Some((start, end))` indices of its first front face and last back face, `None` when there
/// is none left.
fn get_remove_region(
    subtract_list: &[IntersectInfo<'_>],
    start_index: usize,
) -> Result<Option<(usize, usize)>, RayTracerError> {
    let mut region_start_index = start_index;
    let mut inside_count = 0.0;
    let subtract_count = subtract_list.len();
    for subtract_index in start_index..subtract_count {
        let subtract_info = &subtract_list[subtract_index];
        if subtract_info.hit_type == IntersectionType::FrontFace {
            if inside_count == 0.0 {
                region_start_index = subtract_index;
            }
            inside_count += 1.0;
        } else if subtract_info.hit_type == IntersectionType::BackFace {
            if inside_count == 0.0 {
                return Err(RayTracerError::BadHitList(
                    "You should not have a back face without a matching front face.",
                ));
            }
            inside_count -= 1.0;
            if inside_count == 0.0 {
                // let's check that there is not another entry aligned exactly with this exit
                let next_index = subtract_index + 1;
                if next_index >= subtract_count
                    || subtract_list[subtract_index].distance_to_hit + Ray::SAME_SURFACE_OFFSET
                        < subtract_list[next_index].distance_to_hit
                {
                    // we have our subtract region
                    return Ok(Some((region_start_index, subtract_index)));
                }
            }
        } else {
            return Err(RayTracerError::BadHitList(
                "There should be no 'none's in the hit types.",
            ));
        }
    }

    Ok(None)
}
