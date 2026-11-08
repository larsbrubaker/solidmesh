// Tests for src/orbit-gesture.ts, the Ray Tracing page's pointer rule: one finger orbits, two
// fingers moving together orbit by their midpoint, spreading or pinching zooms, and a finger
// landing, lifting or being cancelled never moves the camera. Run with `bun test` from demo/; the
// class is pure, so no DOM is involved.

import { describe, expect, test } from 'bun:test';
import {
  ORBIT_RADIANS_PER_PX, OrbitGesture, PITCH_MAX, PITCH_MIN, ZOOM_MAX, ZOOM_MIN, applyDelta,
} from './orbit-gesture.ts';

/** A camera at rest, away from every clamp. */
const camera = () => ({ yaw: 1, pitch: 0.8, zoom: 2 });

function orbitOf(start: ReturnType<typeof camera>, dx: number, dy: number) {
  return { yaw: start.yaw - dx * ORBIT_RADIANS_PER_PX, pitch: start.pitch + dy * ORBIT_RADIANS_PER_PX };
}

describe('one finger', () => {
  test('drags to orbit by the distance moved', () => {
    const g = new OrbitGesture();
    g.down(1, 100, 100);
    g.move(1, 130, 110);
    const d = g.take();
    expect(d.dx).toBe(30);
    expect(d.dy).toBe(10);
    expect(d.zoom).toBe(1);
  });

  test('a press and a release without moving changes nothing', () => {
    const g = new OrbitGesture();
    g.down(1, 100, 100);
    g.up(1);
    expect(g.take()).toEqual({ dx: 0, dy: 0, zoom: 1 });
  });

  test('moves of a pointer that is not down are ignored (a mouse hovering)', () => {
    const g = new OrbitGesture();
    g.move(1, 100, 100);
    g.move(1, 150, 120);
    expect(g.take()).toEqual({ dx: 0, dy: 0, zoom: 1 });
    expect(g.fingers).toBe(0);
  });
});

describe('two fingers', () => {
  test('a second finger landing gives no movement, wherever it lands', () => {
    const g = new OrbitGesture();
    g.down(1, 100, 100);
    g.take();
    g.down(2, 300, 240);
    expect(g.take()).toEqual({ dx: 0, dy: 0, zoom: 1 });
  });

  test('alternating reports from two stationary fingers do not orbit', () => {
    // The rule that kept one last position per page: each report was the gap between the fingers.
    const g = new OrbitGesture();
    g.down(1, 100, 200);
    g.down(2, 260, 240);
    for (let i = 0; i < 8; i++) {
      g.move(1, 100, 200);
      expect(g.take()).toEqual({ dx: 0, dy: 0, zoom: 1 });
      g.move(2, 260, 240);
      expect(g.take()).toEqual({ dx: 0, dy: 0, zoom: 1 });
    }
  });

  test('read one report at a time, a shared drag never jumps by the gap between the fingers', () => {
    const g = new OrbitGesture();
    g.down(1, 100, 100);
    g.down(2, 220, 160);
    g.move(1, 125, 118);
    const first = g.take();
    expect(Math.abs(first.dx)).toBeLessThanOrEqual(12.5);
    expect(Math.abs(first.dy)).toBeLessThanOrEqual(9);
    g.move(2, 245, 178);
    const second = g.take();
    expect(Math.abs(second.dx)).toBeLessThanOrEqual(12.5);
    expect(Math.abs(second.dy)).toBeLessThanOrEqual(9);
  });

  test('moved together by (dx, dy) they orbit exactly as one finger moved by (dx, dy) would', () => {
    const one = new OrbitGesture();
    one.down(1, 100, 100);
    one.move(1, 125, 118);
    const single = one.take();

    const two = new OrbitGesture();
    two.down(1, 100, 100);
    two.down(2, 220, 160);
    two.move(1, 125, 118);
    two.move(2, 245, 178);
    const d = two.take();
    expect(d.dx).toBeCloseTo(single.dx, 10);
    expect(d.dy).toBeCloseTo(single.dy, 10);
    expect(d.zoom).toBeCloseTo(1, 10);
  });

  test('moved together in several frames, the orbit adds up to the whole movement', () => {
    const g = new OrbitGesture();
    g.down(1, 100, 100);
    g.down(2, 220, 160);
    let dx = 0;
    let dy = 0;
    for (let i = 1; i <= 12; i++) {
      g.move(1, 100 + i * 4, 100 + i * 2);
      g.move(2, 220 + i * 4, 160 + i * 2);
      const d = g.take();
      dx += d.dx;
      dy += d.dy;
    }
    expect(dx).toBeCloseTo(48, 9);
    expect(dy).toBeCloseTo(24, 9);
  });

  test('spreading zooms by the ratio of the finger distances and does not orbit', () => {
    const g = new OrbitGesture();
    g.down(1, 100, 200);
    g.down(2, 200, 200);
    g.move(1, 80, 200);
    g.move(2, 220, 200);
    const d = g.take();
    expect(d.zoom).toBeCloseTo(140 / 100, 12);
    expect(d.dx).toBe(0);
    expect(d.dy).toBe(0);
  });

  test('pinching with one finger held still zooms and does not orbit', () => {
    const g = new OrbitGesture();
    g.down(1, 100, 200);
    g.down(2, 200, 200);
    g.move(2, 160, 200);
    const d = g.take();
    expect(d.zoom).toBeCloseTo(60 / 100, 12);
    expect(d.dx).toBe(0);
    expect(d.dy).toBe(0);
  });

  test('lifting a finger gives no movement, and the other continues as one-finger orbit', () => {
    const g = new OrbitGesture();
    g.down(1, 100, 100);
    g.down(2, 240, 180);
    g.move(1, 104, 102);
    g.move(2, 244, 182);
    g.take();
    g.up(2);
    expect(g.take()).toEqual({ dx: 0, dy: 0, zoom: 1 });
    expect(g.fingers).toBe(1);
    g.move(1, 114, 107);
    const d = g.take();
    expect(d.dx).toBe(10);
    expect(d.dy).toBe(5);
    expect(d.zoom).toBe(1);
  });

  test('a cancelled finger is forgotten like a lifted one', () => {
    const g = new OrbitGesture();
    g.down(1, 100, 100);
    g.down(2, 240, 180);
    g.up(1); // pointercancel and lostpointercapture both end up here
    expect(g.fingers).toBe(1);
    expect(g.take()).toEqual({ dx: 0, dy: 0, zoom: 1 });
    g.up(1);
    g.up(2);
    expect(g.fingers).toBe(0);
    expect(g.take()).toEqual({ dx: 0, dy: 0, zoom: 1 });
  });

  test('movement before a change of fingers still counts, before the rebaseline', () => {
    const g = new OrbitGesture();
    g.down(1, 100, 100);
    g.move(1, 110, 104);
    g.down(2, 300, 300);
    g.move(2, 300, 300);
    const d = g.take();
    expect(d.dx).toBe(10);
    expect(d.dy).toBe(4);
    expect(d.zoom).toBe(1);
  });

  test('a third finger is ignored; only two count', () => {
    const g = new OrbitGesture();
    g.down(1, 100, 100);
    g.down(2, 200, 100);
    g.down(3, 500, 500);
    g.move(3, 40, 40);
    expect(g.take()).toEqual({ dx: 0, dy: 0, zoom: 1 });
  });

  test('two fingers at the same point cannot zoom by a divide by zero', () => {
    const g = new OrbitGesture();
    g.down(1, 100, 100);
    g.down(2, 100, 100);
    g.move(2, 150, 100);
    const d = g.take();
    expect(Number.isFinite(d.zoom)).toBe(true);
  });
});

describe('applyDelta', () => {
  test('turns a delta into the same camera a one-finger drag always made', () => {
    const start = camera();
    const next = applyDelta(start, { dx: 40, dy: -20, zoom: 1 });
    expect(next).toEqual({ ...orbitOf(start, 40, -20), zoom: 2 });
  });

  test('keeps the pitch and zoom limits', () => {
    const start = camera();
    expect(applyDelta(start, { dx: 0, dy: 1e6, zoom: 1 }).pitch).toBe(PITCH_MAX);
    expect(applyDelta(start, { dx: 0, dy: -1e6, zoom: 1 }).pitch).toBe(PITCH_MIN);
    expect(applyDelta(start, { dx: 0, dy: 0, zoom: 1000 }).zoom).toBe(ZOOM_MAX);
    expect(applyDelta(start, { dx: 0, dy: 0, zoom: 0.001 }).zoom).toBe(ZOOM_MIN);
  });

  test('a pinch multiplies the zoom by the ratio, within the clamp', () => {
    expect(applyDelta(camera(), { dx: 0, dy: 0, zoom: 1.5 }).zoom).toBeCloseTo(3, 12);
    expect(ZOOM_MIN).toBe(0.4);
    expect(ZOOM_MAX).toBe(8);
    expect(PITCH_MIN).toBe(0.06);
    expect(PITCH_MAX).toBe(1.5);
  });
});
