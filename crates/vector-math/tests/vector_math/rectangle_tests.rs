// RectangleDouble and RectangleInt: unions holding both rectangles, y-up clipping (top is the
// larger y, and clipping a destination trims its source alike), and equal rectangles hashing
// equal.

use solidmesh_math::{RectangleDouble, RectangleInt};
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

fn hash_of<T: Hash>(value: &T) -> u64 {
    let mut hasher = DefaultHasher::new();
    value.hash(&mut hasher);
    hasher.finish()
}

#[test]
fn unite_rectangles_holds_both_rectangles() {
    let mut united = RectangleDouble::new(0.0, 0.0, 0.0, 0.0);
    united.unite_rectangles(
        RectangleDouble::new(0.0, 0.0, 2.0, 10.0),
        RectangleDouble::new(1.0, -1.0, 3.0, 5.0),
    );
    assert_eq!(united, RectangleDouble::new(0.0, -1.0, 3.0, 10.0));

    let mut united = RectangleInt::new(0, 0, 0, 0);
    united.unite_rectangles(
        RectangleInt::new(0, 0, 2, 10),
        RectangleInt::new(1, -1, 3, 5),
    );
    assert_eq!(united, RectangleInt::new(0, -1, 3, 10));
}

#[test]
fn clip_rect_treats_top_as_the_larger_y() {
    let bounding = RectangleInt::new(0, 0, 10, 10);
    let mut dest = RectangleInt::new(-5, -5, 5, 20);
    assert!(RectangleInt::clip_rect(bounding, &mut dest));
    assert_eq!(dest, RectangleInt::new(0, 0, 5, 10));

    let mut above = RectangleInt::new(0, 12, 5, 20);
    assert!(!RectangleInt::clip_rect(bounding, &mut above));

    let mut source = RectangleInt::new(100, 100, 110, 125);
    let mut dest = RectangleInt::new(0, -5, 10, 20);
    assert_eq!(
        RectangleInt::clip_rects(bounding, &mut source, &mut dest),
        Ok(true)
    );
    assert_eq!(dest, RectangleInt::new(0, 0, 10, 10));
    // Trimming 10 off dest's top and 5 off its bottom trims source alike.
    assert_eq!(source, RectangleInt::new(100, 105, 110, 115));
}

#[test]
fn test_get_hash_code() {
    {
        let a = RectangleDouble::new(10.0, 11.0, 12.0, 13.0);
        let b = RectangleDouble::new(10.0, 11.0, 12.0, 13.0);
        assert!(hash_of(&a) == hash_of(&b));
    }
    {
        let a = RectangleInt::new(10, 11, 12, 13);
        let b = RectangleInt::new(10, 11, 12, 13);
        assert!(hash_of(&a) == hash_of(&b));
    }
}
