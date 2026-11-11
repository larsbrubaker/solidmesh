// The Ray Tracing page's pointer rule (src/demos/ray-trace.ts; src/ray-trace-camera-input.ts feeds
// it the page's pointer events), kept apart from the DOM so it can be tested: pointers go down,
// move and come up, and `take()` says how far the camera should orbit and how much it should zoom
// since last asked. Tested in src/orbit-gesture.test.ts.
//
// One finger orbits by its movement. Two fingers orbit by the movement of the point midway
// between them and zoom by the ratio of their distances, so a pair moved together turns the part
// exactly as one finger would. A step in which the fingers mainly spread or pinch, including a
// pinch with one finger held still, zooms and does not orbit (`ORBIT_OVER_SPREAD`). Fingers past
// the first two are tracked but ignored.
//
// Each finger's position is kept separately, because a phone reports the fingers one at a time.
// Movement is banked at every change of finger count and baselines restart there, so a finger
// landing, lifting or being cancelled never moves the camera however far it is from the others.
// The page asks once per animation frame, which also puts the reports of both fingers in the
// same step: reports read one by one would see a pinch's midpoint wobble and a shared drag's
// distance wobble.

/** Orbit radians per CSS px of drag. */
export const ORBIT_RADIANS_PER_PX = 0.008;
export const PITCH_MIN = 0.06;
export const PITCH_MAX = 1.5;
export const ZOOM_MIN = 0.4;
export const ZOOM_MAX = 8;

/** Fingers closer than this (CSS px) have no meaningful distance to take a ratio of. */
const MIN_FINGER_GAP_PX = 1;
/**
 * How many times the fingers' spreading (half the change of the distance between them) their
 * shared movement must be to orbit. A pinch with one finger still moves the midpoint exactly as
 * far as it spreads, so a ratio of 1 would orbit it; a finger drifting off the line between the
 * fingers needs the margin.
 */
const ORBIT_OVER_SPREAD = 1.5;

export interface OrbitDelta {
  /** Orbit movement in CSS px, positive right. */
  dx: number;
  /** Orbit movement in CSS px, positive down. */
  dy: number;
  /** Zoom multiplier: 1 is no change. */
  zoom: number;
}

export interface OrbitCamera {
  yaw: number;
  pitch: number;
  zoom: number;
}

interface Point {
  x: number;
  y: number;
}

/** The camera after `delta`, within the page's pitch and zoom limits. */
export function applyDelta(camera: OrbitCamera, delta: OrbitDelta): OrbitCamera {
  return {
    yaw: camera.yaw - delta.dx * ORBIT_RADIANS_PER_PX,
    pitch: Math.max(PITCH_MIN, Math.min(PITCH_MAX, camera.pitch + delta.dy * ORBIT_RADIANS_PER_PX)),
    zoom: Math.max(ZOOM_MIN, Math.min(ZOOM_MAX, camera.zoom * delta.zoom)),
  };
}

export class OrbitGesture {
  private points = new Map<number, Point>();
  /** Where each finger was when the movement in `banked` was last brought up to date. */
  private base = new Map<number, Point>();
  private banked: OrbitDelta = { dx: 0, dy: 0, zoom: 1 };

  /** How many pointers are down. */
  get fingers(): number {
    return this.points.size;
  }

  /** Whether this pointer is down. */
  has(id: number): boolean {
    return this.points.has(id);
  }

  down(id: number, x: number, y: number): void {
    this.bank();
    this.points.set(id, { x, y });
    this.rebaseline();
  }

  /** Moves a pointer that is down; a pointer that isn't (a hovering mouse) is ignored. */
  move(id: number, x: number, y: number): void {
    const point = this.points.get(id);
    if (!point) return;
    point.x = x;
    point.y = y;
  }

  /** A pointer came up, was cancelled or lost its capture; an unknown one is ignored. */
  up(id: number): void {
    this.bank();
    this.points.delete(id);
    this.rebaseline();
  }

  /** The movement since the last call. */
  take(): OrbitDelta {
    this.bank();
    const delta = this.banked;
    this.banked = { dx: 0, dy: 0, zoom: 1 };
    return delta;
  }

  private rebaseline(): void {
    this.base = new Map([...this.points].map(([id, p]) => [id, { ...p }]));
  }

  /** Adds the movement of the first two fingers since their baseline, then moves it. */
  private bank(): void {
    const ids = [...this.points.keys()].slice(0, 2);
    if (ids.length === 1) this.bankOne(ids[0]);
    else if (ids.length === 2) this.bankTwo(ids[0], ids[1]);
    this.rebaseline();
  }

  private bankOne(id: number): void {
    const now = this.points.get(id)!;
    const was = this.base.get(id)!;
    this.banked.dx += now.x - was.x;
    this.banked.dy += now.y - was.y;
  }

  private bankTwo(a: number, b: number): void {
    const aNow = this.points.get(a)!;
    const bNow = this.points.get(b)!;
    const aWas = this.base.get(a)!;
    const bWas = this.base.get(b)!;
    const midX = (aNow.x + bNow.x - aWas.x - bWas.x) / 2;
    const midY = (aNow.y + bNow.y - aWas.y - bWas.y) / 2;
    const gapNow = Math.hypot(aNow.x - bNow.x, aNow.y - bNow.y);
    const gapWas = Math.hypot(aWas.x - bWas.x, aWas.y - bWas.y);
    if (gapNow >= MIN_FINGER_GAP_PX && gapWas >= MIN_FINGER_GAP_PX) this.banked.zoom *= gapNow / gapWas;
    const shared = Math.hypot(midX, midY);
    if (shared > ORBIT_OVER_SPREAD * (Math.abs(gapNow - gapWas) / 2)) {
      this.banked.dx += midX;
      this.banked.dy += midY;
    }
  }
}
