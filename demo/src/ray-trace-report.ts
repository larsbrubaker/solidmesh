// The Ray Tracing page's "Copy timings" report, with no browser in it so `bun test` can run it
// (src/ray-trace-report.test.ts): the collectors the page feeds while it renders (`Durations`,
// `StillTimings`, `PreviewStretch`) and the plain text a person on any device copies and pastes
// back (`timingsReport`), so a developer who cannot see the device can tell which build it runs,
// which renderer drew the picture, how big and how fast it is, and where the time went.
// src/demos/ray-trace.ts collects the numbers and owns the button; src/ray-trace-rules.ts holds
// the page's other decisions.
//
// Collecting must cost nothing noticeable while rendering: each collector keeps running totals
// and, for a median, the most recent MEDIAN_KEPT values, so memory stays bounded however long a
// still refines. A step counts in a still only for the part of it after the still began
// (`insideStill`), so a shader build that runs on from the first preview into the first still is
// counted there for what it cost the still, and the report says it began earlier. A change of the
// renderer drawing is a fresh still that says how far the other renderer had got (`stillFor`).
// Every number is written with `toFixed` or as a plain integer, never through the
// locale, so reports from different devices compare line for line.

import type { GpuFailure, Route } from './ray-trace-rules.ts';
import { rendererName } from './ray-trace-rules.ts';
import { sameArgs } from './ray-trace-refine.ts';

/** How many of the most recent values a median is taken over. */
export const MEDIAN_KEPT = 512;

/** A count, total and largest of some durations or sizes, and the median of the latest few. */
export class Durations {
  count = 0;
  total = 0;
  max = 0;
  private kept: number[] = [];
  private next = 0;

  add(value: number): void {
    this.count += 1;
    this.total += value;
    if (value > this.max) this.max = value;
    if (this.kept.length < MEDIAN_KEPT) this.kept.push(value);
    else this.kept[this.next] = value;
    this.next = (this.next + 1) % MEDIAN_KEPT;
  }

  /** The median of the most recent MEDIAN_KEPT values (the mean of the middle two of an even count); 0 when empty. */
  get median(): number {
    if (this.kept.length === 0) return 0;
    const sorted = [...this.kept].sort((a, b) => a - b);
    const mid = sorted.length >> 1;
    return sorted.length % 2 === 1 ? sorted[mid] : (sorted[mid - 1] + sorted[mid]) / 2;
  }
}

/** How much of a step from `startedAt` to `endedAt` falls after a still began at `stillStart` (ms, never negative). */
export function insideStill(startedAt: number, endedAt: number, stillStart: number): number {
  return Math.max(0, endedAt - Math.max(startedAt, stillStart));
}

/**
 * One kind of step a still waited on, each counted only for the part after the still began: a
 * shader build the first preview started runs on into the first still, and a step started under an
 * earlier still with the same request ends inside a later one. Steps that began before the still
 * are also kept whole, so the report can say how long they took in all.
 */
export class StepTimes {
  /** The part of each counted step inside the still. */
  readonly inside = new Durations();
  /** Of those, the steps that began before the still: how many, their whole time and their inside part. */
  straddled = 0;
  straddledWhole = 0;
  straddledInside = 0;

  /** Counts a step unless it ended before the still began; says whether it counted. */
  add(startedAt: number, endedAt: number, stillStart: number): boolean {
    if (endedAt < stillStart) return false;
    const inside = insideStill(startedAt, endedAt, stillStart);
    this.inside.add(inside);
    if (startedAt < stillStart) {
      this.straddled += 1;
      this.straddledWhole += endedAt - startedAt;
      this.straddledInside += inside;
    }
    return true;
  }
}

/** The renderer a still was restarted from, mid-way, and how far that one had got. */
export interface Restart {
  from: Route;
  samples: number;
  wallMs: number;
}

export type StepKind = 'device' | 'pack' | 'shader' | 'pilot' | 'slices' | 'reads' | 'decodes' | 'views';

/**
 * One still's refinement from the moment the page asked for it, by the renderer it heads to
 * (`heading`): a change of renderer is a fresh still (`stillFor`). Times are `performance.now()` ms.
 */
export class StillTimings {
  /** Whole passes over the picture finished, and the passes' pixels and own render time. */
  samples = 0;
  pixelPasses = 0;
  renderMs = 0;
  rays = 0;
  /** When the last pass reached the cap; null while it refines. */
  finishedAt: number | null = null;
  /** The renderer that drew its passes; null before the first. */
  route: Route | null = null;
  /** One time-boxed CPU call each, a part of a pass or a whole one. */
  readonly passes = new Durations();
  /** The GPU route's steps; a counted slice's pixels are kept beside its time. */
  readonly steps: Record<StepKind, StepTimes> = {
    device: new StepTimes(), pack: new StepTimes(), shader: new StepTimes(), pilot: new StepTimes(), slices: new StepTimes(),
    reads: new StepTimes(), decodes: new StepTimes(), views: new StepTimes(),
  };
  readonly slicePixels = new Durations();

  constructor(
    readonly startedAt: number, readonly args: readonly unknown[], readonly width: number, readonly height: number,
    readonly heading: Route = 'cpu', readonly restarted: Restart | null = null,
  ) {}

  /** A GPU step from `startedAt` to `endedAt`; `pixels` for a slice. */
  record(kind: StepKind, startedAt: number, endedAt: number, pixels = 0): void {
    if (this.steps[kind].add(startedAt, endedAt, this.startedAt) && kind === 'slices') this.slicePixels.add(pixels);
  }

  /**
   * A pass of this still was drawn at `now`, its render taking `ms`: `samples` finished in all,
   * `cap` the most it takes. Only the part of `ms` since the still began counts as its time.
   */
  drew(route: Route, samples: number, pixelPasses: number, ms: number, rays: number, cap: number, now: number): void {
    const inside = insideStill(now - ms, now, this.startedAt);
    this.route = route;
    this.samples = samples;
    this.pixelPasses += pixelPasses;
    this.renderMs += inside;
    this.rays += rays;
    if (route === 'cpu') this.passes.add(inside);
    if (samples >= cap && this.finishedAt === null) this.finishedAt = now;
  }
}

/**
 * The still for a request heading to `heading`: `previous` while the request and the renderer are
 * the same, else a fresh one. When only the renderer changed (Auto falling back to the CPU
 * mid-still) the fresh one says how far the previous renderer had got.
 */
export function stillFor(
  previous: StillTimings | null, args: readonly unknown[], width: number, height: number, heading: Route, now: number,
): StillTimings {
  const same = previous !== null && sameArgs(previous.args, args);
  if (previous && same && previous.heading === heading) return previous;
  const restarted = previous && same && previous.route !== null
    ? { from: previous.route, samples: previous.samples, wallMs: (previous.finishedAt ?? now) - previous.startedAt }
    : null;
  return new StillTimings(now, args, width, height, heading, restarted);
}

/** The latest stretch of moving previews: frames shown and the time between them. */
export class PreviewStretch {
  frames = 0;
  route: Route | null = null;
  readonly intervals = new Durations();
  readonly pixels = new Durations();
  private lastShown: number | null = null;

  shown(route: Route, width: number, height: number, now: number): void {
    this.frames += 1;
    this.route = route;
    this.pixels.add(width * height);
    if (this.lastShown !== null) this.intervals.add(now - this.lastShown);
    this.lastShown = now;
  }
}

/** What the page knows when the person presses Copy timings. */
export interface TimingsInputs {
  /** The site build's identity, injected when the site was built (src/site-build.ts). */
  build: string;
  userAgent: string;
  pixelRatio: number;
  areaWidth: number;
  areaHeight: number;
  webGpu: boolean;
  /** The adapter the page opened, or null when it has opened none. */
  adapter: string | null;
  part: string;
  detail: string;
  triangles: number | null;
  view: string;
  renderer: string;
  material: string;
  light: string;
  /** Who drew the picture on screen, and the sentence the page shows about the route. */
  shown: Route;
  routeMessage: string;
  failure: GpuFailure | null;
  cap: number;
  /** The GPU route's most recent part pack and shader build, wherever they fell (ms). */
  lastPackMs: number | null;
  lastShaderMs: number | null;
  /** The still on screen, or null while the camera moves or before one is asked for. */
  still: StillTimings | null;
  previews: PreviewStretch | null;
  now: number;
}

const ms = (value: number) => `${value.toFixed(1)} ms`;
const int = (value: number) => String(Math.round(value));

/** A rate in the largest fitting unit, `unit` per second. */
function rate(perSecond: number, unit: string): string {
  if (perSecond >= 1e6) return `${(perSecond / 1e6).toFixed(2)} M ${unit}/s`;
  if (perSecond >= 1e3) return `${(perSecond / 1e3).toFixed(1)} k ${unit}/s`;
  return `${perSecond.toFixed(2)} ${unit}/s`;
}

/** What a step's line adds when some of its steps began before the still. */
function straddleNote(t: StepTimes): string {
  if (t.straddled === 0) return '';
  if (t.inside.count === 1) return ` (it began before this still and took ${ms(t.straddledWhole)} in all)`;
  return `; ${t.straddled} began before this still and took ${ms(t.straddledWhole)} in all, ${ms(t.straddledInside)} of it inside`;
}

/** A step of the GPU route in one line: how often, the time inside this still, the median when it repeats. */
function step(t: StepTimes, median = true): string {
  const d = t.inside;
  if (d.count === 0) return 'none';
  const many = `${d.count}, total ${ms(d.total)}${median ? `, median ${ms(d.median)}` : ''}`;
  return (d.count === 1 ? ms(d.total) : many) + straddleNote(t);
}

/** A once-per-part step: its time in this still, or the last one's time from before it began. */
function oncePerPart(t: StepTimes, last: number | null): string {
  if (t.inside.count > 0) return `${ms(t.inside.total)} in this still${straddleNote(t)}`;
  return last === null ? 'none yet' : `none in this still (the last took ${ms(last)}, before it began)`;
}

/**
 * What the steps above leave of the wall time. Every step is cut to the still's own time and only
 * the live GPU run records, one step after another, so this is not below zero; if it ever is, steps
 * overlapped, and the report says so rather than printing a remainder of nothing.
 */
export function remainderLine(left: number): string {
  // Below a twentieth of a millisecond is the clock's rounding, not an overlap.
  if (left >= -0.05) return `  Remainder of the wall time, not covered above: ${ms(Math.max(0, left))}`;
  return `  Remainder of the wall time: none; the steps above add up to ${ms(-left)} more than the wall time, so some of them overlapped`;
}

function gpuLines(still: StillTimings, wall: number, inputs: TimingsInputs): string[] {
  const t = still.steps;
  const covered = Object.values(t).reduce((sum, steps) => sum + steps.inside.total, 0);
  const slices = t.slices.inside.count === 0
    ? 'none'
    : `${t.slices.inside.count}, total wait ${ms(t.slices.inside.total)}; median ${int(still.slicePixels.median)} pixels, ${ms(t.slices.inside.median)}; largest ${int(still.slicePixels.max)} pixels, ${ms(t.slices.inside.max)}${straddleNote(t.slices)}`;
  return [
    'Where the time went (GPU route):',
    `  Opening the graphics device: ${step(t.device)}`,
    `  Part pack: ${oncePerPart(t.pack, inputs.lastPackMs)}`,
    `  Shader build (pipeline creation): ${oncePerPart(t.shader, inputs.lastShaderMs)}`,
    `  Pilot: ${step(t.pilot)}`,
    `  GPU slices: ${slices}`,
    `  Read-backs: ${step(t.reads)}`,
    `  Decodes in the worker: ${step(t.decodes)}`,
    `  View requests to the worker: ${step(t.views, false)}`,
    remainderLine(wall - covered),
  ];
}

function cpuLines(still: StillTimings): string[] {
  const rays = still.renderMs > 0 ? rate((still.rays / still.renderMs) * 1000, 'rays') : 'none yet';
  return [
    'Where the time went (CPU route):',
    `  Time-boxed calls to the worker, each part of a pass: ${still.passes.count}, total ${ms(still.passes.total)}, median ${ms(still.passes.median)}`,
    `  Rays per second: ${rays}`,
  ];
}

function adapterText(inputs: TimingsInputs): string {
  if (!inputs.webGpu) return 'no WebGPU in this browser';
  return inputs.adapter ?? 'WebGPU present, no adapter opened yet (the GPU route has not run)';
}

/** The text 'Copy timings' puts on the clipboard. */
export function timingsReport(inputs: TimingsInputs): string {
  const lines = [
    `Build: ${inputs.build}`,
    'solidmesh Ray Tracing timings',
    `Device: ${inputs.userAgent}; device pixel ratio ${inputs.pixelRatio.toFixed(2)}; picture area ${int(inputs.areaWidth)} x ${int(inputs.areaHeight)} CSS pixels; GPU adapter: ${adapterText(inputs)}`,
    `Settings: part ${inputs.part}; detail ${inputs.detail}; ${inputs.triangles === null ? 'triangles not built yet' : `${int(inputs.triangles)} triangles`}; view ${inputs.view}; renderer picked ${inputs.renderer}; material ${inputs.material}; light ${inputs.light}`,
    `Renderer in use: ${rendererName(inputs.shown)}${inputs.routeMessage ? ` (the page says: ${inputs.routeMessage})` : ''}`,
    `GPU failure: ${inputs.failure ? `${inputs.failure.kind}: ${inputs.failure.detail}` : 'none'}`,
  ];
  const still = inputs.still;
  if (inputs.shown === 'none') {
    lines.push('Picture: nothing drawn, because the GPU picked cannot draw it');
  } else if (!still || still.route === null) {
    lines.push('Picture: no still drawn yet (the camera is moving, or the first pass has not come back)');
  } else {
    const end = still.finishedAt ?? inputs.now;
    const wall = end - still.startedAt;
    const pixels = still.width * still.height;
    const state = still.finishedAt === null ? 'still refining' : 'finished';
    const perSecond = wall > 0 ? (still.samples / wall) * 1000 : 0;
    const pixelRate = wall > 0 ? (still.pixelPasses / wall) * 1000 : 0;
    const ownRate = still.renderMs > 0 ? (still.pixelPasses / pixels / still.renderMs) * 1000 : 0;
    lines.push(`Picture: ${still.width} x ${still.height} pixels by the ${rendererName(still.route)}; ${still.samples} of ${inputs.cap} passes (${state}); wall time ${ms(wall)}; ${perSecond.toFixed(2)} passes/s and ${rate(pixelRate, 'pixel-passes')} over the wall time; ${ownRate.toFixed(2)} passes/s over the renderer's own time (the Samples/s readout)`);
    if (still.restarted) {
      const r = still.restarted;
      lines.push(`Restarted: after the ${rendererName(r.from)} stopped, this picture started again on the ${rendererName(still.route)}; the ${rendererName(r.from)} had drawn ${r.samples} of ${inputs.cap} passes in ${ms(r.wallMs)}`);
    }
    lines.push(...(still.route === 'gpu' ? gpuLines(still, wall, inputs) : cpuLines(still)));
  }
  const previews = inputs.previews;
  if (!previews || previews.frames === 0) {
    lines.push('Moving previews: none yet (orbit the part, then copy again to time them)');
  } else {
    const between = previews.intervals.count === 0
      ? 'one frame, so no time between frames'
      : `median ${ms(previews.intervals.median)} and slowest ${ms(previews.intervals.max)} from one frame shown to the next`;
    lines.push(`Moving previews: ${previews.frames} frames by the ${rendererName(previews.route ?? 'none')}; ${between}; median size ${int(previews.pixels.median)} pixels`);
  }
  return lines.join('\n');
}
