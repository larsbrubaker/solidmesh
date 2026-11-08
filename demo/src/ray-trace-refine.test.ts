// Tests for src/ray-trace-refine.ts: the Ray Tracing page's two decisions about when an image
// counts, which keep a still page from flickering or restarting. A result only draws while
// moving or if it is exactly what the page now wants; a resize only restarts refinement when
// the image's size really changed. Run with `bun test` from demo/.

import { describe, expect, test } from 'bun:test';
import { mayDraw, sameSize, sizeChanged } from './ray-trace-refine.ts';

const preview = [0.85, 0.42, 1, 0, 0, 91, 194, 180, 0.3, 0, 0, 0, 5, 1, 110, 118, true, 0];
const full = [0.85, 0.42, 1, 0, 0, 91, 194, 180, 0.3, 0, 0, 0, 5, 1, 567, 597, false, 45];

describe('mayDraw', () => {
  test('while the camera moves every result draws, even one the camera has already passed', () => {
    expect(mayDraw(preview, full, true)).toBe(true);
  });

  test('once still, the request the page is refining draws', () => {
    expect(mayDraw(full, [...full], false)).toBe(true);
  });

  test('once still, a preview still in flight when the finger lifted does not draw', () => {
    expect(mayDraw(preview, full, false)).toBe(false);
  });

  test('once still, a result for another camera, look or size does not draw', () => {
    expect(mayDraw(full, Object.assign([...full], { 0: 0.9 }), false)).toBe(false); // camera
    expect(mayDraw(full, Object.assign([...full], { 8: 0.35 }), false)).toBe(false); // look
    expect(mayDraw(full, Object.assign([...full], { 15: 620 }), false)).toBe(false); // size
  });

  test('the first request has nothing to compare with and does not draw once still', () => {
    expect(mayDraw(full, null, false)).toBe(false);
  });
});

describe('sizeChanged', () => {
  const size = { w: 567, h: 597 };

  test('the first measurement is a change', () => {
    expect(sizeChanged(null, size)).toBe(true);
  });

  test('a resize notification that leaves the image the same size is not a change', () => {
    expect(sizeChanged(size, { ...size })).toBe(false);
    expect(sameSize(size, { ...size })).toBe(true);
  });

  test('a different width or height is a change', () => {
    expect(sizeChanged(size, { w: 568, h: 597 })).toBe(true);
    expect(sizeChanged(size, { w: 567, h: 590 })).toBe(true);
  });
});
