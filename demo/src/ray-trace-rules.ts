// The Ray Tracing page's decisions that need no browser, so `bun test` can run them
// (src/ray-trace-rules.test.ts): which renderer draws (`chooseRoute`), the words the page says
// when the GPU can't (`fallbackMessage`), the pass count over the picture (`passCountText`, the
// same words MatterCAD's Realistic view shows), which pixel and sample each GPU slice covers
// (`SampleWalk`), how big a slice may be (`SliceCosts`) and the small pilot image a still is
// measured with first (`pilotSize`), when an image read back from the GPU is
// worth drawing (`displayDue`), what the badge says while the GPU route gets ready
// (`GpuStageBadge`) and when a crashed worker gets another try (`TrapRecovery`).
// src/demos/ray-trace.ts is the page and src/ray-trace-gpu.ts its WebGPU calls.

import { FIRST_SLICE_RAYS, TARGET_MS, WORKGROUP_RAYS } from './gpu-bench-rules.ts';

export type RendererChoice = 'auto' | 'cpu' | 'gpu';
export type Route = 'cpu' | 'gpu';

/**
 * Why the GPU route stopped: the device was lost (`lost`), the scene doesn't fit this GPU
 * (`too-large`, the library's own words in `detail`), the browser has WebGPU but no usable adapter
 * (`no-adapter`), the driver refused the shader (`shader`), the worker crashed packing the part
 * (`trapped`), or it failed some other way (`error`).
 */
export interface GpuFailure {
  kind: 'lost' | 'too-large' | 'no-adapter' | 'shader' | 'trapped' | 'error';
  detail: string;
}

export interface RouteDecision {
  route: Route;
  /** What the page says about it, empty when there is nothing to say. */
  message: string;
}

/** Chosen GPU on a browser with no WebGPU: says so and what the person can do. */
export const NO_WEBGPU_MESSAGE =
  'This browser does not have WebGPU, so the picture is drawn on the CPU. To use your graphics card, open this page in a current version of Chrome or Edge.';

/** A sentence's `detail` without the period it may already end with. */
function clause(detail: string): string {
  return detail.trim().replace(/\.+$/, '');
}

/** What the page says when `failure` has moved the picture to the CPU. */
export function fallbackMessage(failure: GpuFailure): string {
  switch (failure.kind) {
    case 'lost':
      return 'Your graphics card stopped responding, so the picture moved to the CPU and kept going there. To try the GPU again, pick CPU and then GPU in Renderer, or reload the page.';
    case 'too-large':
      return `This part does not fit your graphics card, so it is drawn on the CPU: ${clause(failure.detail)}. A smaller Detail may fit.`;
    case 'no-adapter':
      return 'This browser has WebGPU but found no usable graphics adapter, so the picture is drawn on the CPU. Updating your graphics driver may help.';
    case 'shader':
      return 'Your graphics driver refused the path tracing shader, so the picture is drawn on the CPU. Updating the driver may fix it.';
    case 'trapped':
      return 'Preparing this part for your graphics card failed, most likely out of memory, so it is drawn on the CPU. A smaller Detail may fit.';
    case 'error':
      return `The graphics card could not run this render, so it is drawn on the CPU: ${clause(failure.detail)}. Reload the page to try the GPU again.`;
  }
}

/**
 * Which renderer draws, given the Renderer menu, whether the browser has WebGPU, and any failure
 * of the GPU route on this page. Auto uses the GPU wherever it can and says nothing when there is
 * no WebGPU or no usable adapter: nothing went wrong and nothing was asked for. A failure holds
 * until the person picks a renderer again, so a lost device is never retried in a loop.
 */
export function chooseRoute(choice: RendererChoice, webGpu: boolean, failure: GpuFailure | null): RouteDecision {
  if (choice === 'cpu') return { route: 'cpu', message: '' };
  if (!webGpu) return { route: 'cpu', message: choice === 'gpu' ? NO_WEBGPU_MESSAGE : '' };
  if (failure?.kind === 'no-adapter' && choice === 'auto') return { route: 'cpu', message: '' };
  if (failure) return { route: 'cpu', message: fallbackMessage(failure) };
  return { route: 'gpu', message: '' };
}

/**
 * The pass count over the picture, `samples` of `cap` finished: "Rendering 24 of 128" while it
 * refines and nothing once it has finished, as MatterCAD's Realistic view shows it.
 */
export function passCountText(samples: number, cap: number): string {
  return samples >= cap ? '' : `Rendering ${samples} of ${cap}`;
}

export interface PixelSlice {
  first: number;
  count: number;
  sample: number;
}

/**
 * The order the GPU gets its work: pixels in order within a sample, samples in order, until `cap`
 * samples. Each pixel of each sample is given exactly once, because the accumulation buffer can't
 * refuse a repeat. `next` takes however many pixels the adaptive slice size allows this time.
 */
export class SampleWalk {
  private first = 0;
  private sample = 0;

  constructor(readonly pixels: number, readonly cap: number) {}

  get done(): boolean {
    return this.sample >= this.cap;
  }

  /** Whole samples finished so far. */
  get samples(): number {
    return this.sample;
  }

  /** The next pixel to be given. */
  get position(): number {
    return this.first;
  }

  /** The start of a sample rather than part way through one. */
  get atSampleStart(): boolean {
    return this.first === 0;
  }

  /** The next slice of at most `size` pixels, or null when every sample is given. */
  next(size: number): PixelSlice | null {
    if (this.done || this.pixels === 0) return null;
    const count = Math.min(size, this.pixels - this.first);
    const slice = { first: this.first, count, sample: this.sample };
    this.first += count;
    if (this.first >= this.pixels) {
      this.first = 0;
      this.sample += 1;
    }
    return slice;
  }
}

/** Horizontal bands of the image `SliceCosts` keeps an estimate for. */
export const COST_BANDS = 32;
/**
 * A dispatch shorter than this (ms) says nothing about what its pixels cost: its wait is mostly
 * the dispatch's own fixed cost and its jitter. It still proves an upper bound.
 */
export const NOISE_FLOOR_MS = 8;
/**
 * No slice is issued whose cost by the proven bounds exceeds this (ms). A quarter of a second is
 * eight frames' target, enough room for a costly band to be sliced without one dispatch a row, and
 * an eighth of the two seconds after which an operating system resets a GPU that is held up.
 */
export const DISPATCH_CAP_MS = 250;
/** Kept free under the cap for a wait the GPU's work didn't cause (ms): slices aim at the rest. */
export const STRAY_ALLOWANCE_MS = 50;
/** How much costlier a carried bound or estimate is assumed to be, for a view that moved. */
export const CARRIED_COST_MARGIN = 2;
/** The smallest time a measurement counts as (ms): the browser's clock is no finer. */
const MIN_MEASURED_MS = 0.1;
/** About how many pixels a pilot image has. */
export const PILOT_PIXELS = 16384;

/**
 * The pilot of a `width` x `height` still: about PILOT_PIXELS in the same shape, with at least
 * one row a band, so its slices touch every part of the picture before a full-size slice can.
 */
export function pilotSize(width: number, height: number): { w: number; h: number } {
  const scale = Math.min(1, Math.sqrt(PILOT_PIXELS / (width * height)));
  const h = Math.min(height, Math.max(COST_BANDS, Math.round(height * scale)));
  const w = Math.max(1, Math.min(width, Math.round(width * scale)));
  return { w, h };
}

/**
 * One proven stretch of the image: the dispatch that covered it waited `ms`, so its pixels
 * `[from, to)` cost at most that together, `rate` ms each taken evenly.
 */
interface Proof {
  from: number;
  to: number;
  rate: number;
  ms: number;
}

/**
 * How many pixels the next GPU slice may cover. What a pixel costs depends on where it is, sky
 * next to glass by a factor of a hundred or more, and a dispatch also costs a few milliseconds of
 * its own, which jitter. So two things are kept apart:
 *
 * - **Upper bounds.** Every dispatch, however short, proves that its pixels cost at most its whole
 *   wait over their count; the fixed cost is never taken off. The image is covered by the newest
 *   such proofs, and no slice is issued whose cost by them exceeds DISPATCH_CAP_MS (less
 *   STRAY_ALLOWANCE_MS for a wait the GPU didn't cause). A stretch nothing has proven may be
 *   entered by FIRST_SLICE_RAYS pixels at a time, no more, which is how a cold image (the pilot)
 *   covers itself. Within one proven stretch the cost is taken as even: what is not bounded is a
 *   costly feature inside a stretch that was proven by a cheap average.
 * - **Estimates** of each band's cost, learned only from dispatches over NOISE_FLOOR_MS, aim a
 *   slice at TARGET_MS. Where the bands ahead have none yet, the slice doubles while the last came
 *   back under half the target, and holds otherwise.
 *
 * A still starts from its pilot's proofs and estimates, scaled to its size (same rays a pixel, so
 * the same cost a pixel), and a preview from the last preview's, with CARRIED_COST_MARGIN for the
 * camera having moved: the cap then holds where no stretch's cost more than doubled since that
 * frame, and is corrected by the first dispatch over it.
 */
export class SliceCosts {
  private proofs: Proof[] = [];
  private estimates = new Array<number>(COST_BANDS).fill(0);
  private last = FIRST_SLICE_RAYS;
  private lastMs = 0;
  private largest = 0;

  /** `carry` is the pilot (margin 1) or the previous preview (CARRIED_COST_MARGIN). */
  constructor(
    private readonly pixels: number,
    private readonly maxPixels: number,
    carry?: { from: SliceCosts; margin: number },
  ) {
    if (!carry) return;
    const { from, margin } = carry;
    const scale = pixels / from.pixels;
    this.proofs = from.proofs.map(proof => ({
      from: Math.round(proof.from * scale),
      to: proof.to === from.pixels ? pixels : Math.round(proof.to * scale),
      rate: proof.rate * margin,
      ms: proof.ms * scale * margin,
    })).filter(proof => proof.to > proof.from);
    this.estimates = from.estimates.map(estimate => estimate * margin);
    this.last = Math.max(FIRST_SLICE_RAYS, Math.round(from.largest * scale));
  }

  private band(pixel: number): number {
    return Math.min(COST_BANDS - 1, Math.floor((pixel * COST_BANDS) / this.pixels));
  }

  private bandStart(band: number): number {
    return band === 0 ? 0 : this.bandEnd(band - 1);
  }

  private bandEnd(band: number): number {
    return Math.min(this.pixels, Math.ceil(((band + 1) * this.pixels) / COST_BANDS));
  }

  private clamped(pixels: number): number {
    const whole = Math.floor(pixels / WORKGROUP_RAYS) * WORKGROUP_RAYS;
    return Math.max(WORKGROUP_RAYS, Math.min(whole, Math.max(WORKGROUP_RAYS, this.maxPixels)));
  }

  /**
   * Pixels from `first` the estimates fit into TARGET_MS, and whether that used the whole target;
   * it stops early at a band with no estimate, where growth decides instead.
   */
  private aim(first: number): { reach: number; whole: boolean } {
    let budget: number = TARGET_MS;
    let at = first;
    while (at < this.pixels) {
      const band = this.band(at);
      const rate = this.estimates[band];
      if (rate === 0) return { reach: at - first, whole: false };
      const end = this.bandEnd(band);
      if (budget / rate < end - at) return { reach: at + budget / rate - first, whole: true };
      budget -= rate * (end - at);
      at = end;
    }
    return { reach: at - first, whole: true };
  }

  /**
   * The most band `band` can cost a pixel by its proofs, taking each proof's whole wait as if it
   * all fell in the band (a strict bound); Infinity if part of the band is unproven.
   */
  private provenRate(band: number): number {
    const from = this.bandStart(band);
    const to = this.bandEnd(band);
    let covered = 0;
    let ms = 0;
    for (const proof of this.proofs) {
      if (proof.to <= from || proof.from >= to) continue;
      covered += Math.min(to, proof.to) - Math.max(from, proof.from);
      ms += proof.ms;
    }
    return covered < to - from ? Infinity : ms / (to - from);
  }

  /** Pixels from `first` the proven bounds allow under the cap. */
  private allowed(first: number): number {
    let budget = DISPATCH_CAP_MS - STRAY_ALLOWANCE_MS;
    let at = first;
    let index = this.proofs.findIndex(proof => proof.to > at);
    while (at < this.pixels) {
      const proof = index >= 0 && index < this.proofs.length ? this.proofs[index] : null;
      if (!proof || proof.from > at) {
        // Unproven: one starting-size step in, and no further.
        const gapEnd = proof ? proof.from : this.pixels;
        at += Math.min(FIRST_SLICE_RAYS, gapEnd - at);
        break;
      }
      if (budget / proof.rate < proof.to - at) {
        at += budget / proof.rate;
        break;
      }
      budget -= proof.rate * (proof.to - at);
      at = proof.to;
      index += 1;
    }
    return at - first;
  }

  /** The most pixels a slice starting at `first` should cover. */
  sizeAt(first: number): number {
    const aimed = this.aim(first);
    const grown = this.lastMs < TARGET_MS / 2 ? this.last * 2 : this.last;
    const size = this.clamped(Math.min(aimed.whole ? aimed.reach : Math.max(aimed.reach, grown), this.allowed(first)));
    // Less than a workgroup left over would be a dispatch of its own for nothing.
    const rest = this.pixels - first;
    return rest - size < WORKGROUP_RAYS && rest <= this.maxPixels ? rest : size;
  }

  /** Tells it that pixels `[first, first + count)` took `ms`. */
  report(first: number, count: number, ms: number): void {
    const end = first + count;
    // The newest proof of a stretch replaces what covered it; a stretch only partly covered keeps
    // its own proof for the rest.
    const kept: Proof[] = [];
    for (const proof of this.proofs) {
      if (proof.to <= first || proof.from >= end) kept.push(proof);
      else {
        if (proof.from < first) kept.push({ ...proof, to: first });
        if (proof.to > end) kept.push({ ...proof, from: end });
      }
    }
    kept.push({ from: first, to: end, rate: Math.max(ms, MIN_MEASURED_MS) / count, ms: Math.max(ms, MIN_MEASURED_MS) });
    this.proofs = kept.sort((a, b) => a.from - b.from);

    if (ms >= NOISE_FLOOR_MS) {
      // Bands already estimated keep their shape, scaled so the slice's total matches its wait.
      const observed = ms / count;
      let expected = 0;
      for (let at = first; at < end;) {
        const band = this.band(at);
        const until = Math.min(end, this.bandEnd(band));
        expected += (this.estimates[band] > 0 ? this.estimates[band] : observed) * (until - at);
        at = until;
      }
      const scale = ms / expected;
      for (let band = this.band(first); band <= this.band(end - 1); band++) {
        this.estimates[band] = (this.estimates[band] > 0 ? this.estimates[band] : observed) * scale;
      }
    }
    // No band can be estimated above what its proofs allow, so an estimate a stray wait inflated,
    // or a margin carried, comes back down as soon as short dispatches prove the band cheaper.
    for (let band = this.band(first); band <= this.band(end - 1); band++) {
      const proven = this.provenRate(band);
      if (this.estimates[band] > proven) this.estimates[band] = proven;
    }
    // A slice cut short by the end of the image says nothing about how big one may be.
    const cut = end === this.pixels && count < this.last;
    if (!cut) {
      this.last = count;
      this.lastMs = ms;
    }
    this.largest = Math.max(this.largest, count);
  }
}

/** The badge while the part is packed in the worker. */
export const GPU_PACKING_BADGE = 'Packing the part for your graphics card…';
/** The badge during the shader compile, which is most of the wait on a first start. */
export const GPU_COMPILING_BADGE = 'Preparing your graphics card. The first time can take a while…';

/** Where the badge's words go. */
export interface BadgeView {
  show(text: string): void;
  hide(): void;
}

/**
 * The busy badge while the GPU route gets ready. The run says what stage it is at; the badge shows
 * only while the GPU route is what the page is on (`active`), so leaving for the CPU mid-compile
 * hides it at once and coming back shows it again. It hides only what it put up itself, so it
 * never takes down another message, such as the part being rebuilt.
 */
export class GpuStageBadge {
  private stage: string | null = null;
  private showing = false;

  constructor(private readonly view: BadgeView) {}

  /** The run's current stage, or null when it has none. */
  set(stage: string | null, active: boolean): void {
    this.stage = stage;
    this.refresh(active);
  }

  /** Whether the GPU route is what the page is on now. */
  refresh(active: boolean): void {
    if (this.stage !== null && active) {
      this.view.show(this.stage);
      this.showing = true;
    } else if (this.showing) {
      this.view.hide();
      this.showing = false;
    }
  }
}

/** When the worker crashed and could not be brought back. */
export const WORKER_TRAPPED_MESSAGE =
  'This part could not be built in the memory this browser has. Choose a smaller Detail or reload the page.';

/**
 * Whether a crashed worker gets another try: once, until a pass has succeeded, so a build that
 * crashes every time stops instead of looping. One crash can reach the page through several calls
 * at once, and the worker instance it names makes those one crash.
 */
export class TrapRecovery {
  private lastInstance = -1;
  private spent = false;
  stopped = false;

  /** What to do about a crash of worker `instance`. */
  trapped(instance: number): 'retry' | 'stop' | 'seen' {
    if (instance === this.lastInstance) return 'seen';
    this.lastInstance = instance;
    if (this.spent) {
      this.stopped = true;
      return 'stop';
    }
    this.spent = true;
    return 'retry';
  }

  /** A pass worked, so the worker is healthy again. */
  succeeded(): void {
    this.spent = false;
    this.stopped = false;
  }

  /** A different scene starts over. */
  reset(): void {
    this.succeeded();
  }
}

/** The shortest gap between two pictures read back from the GPU (ms) while it refines. */
export const DISPLAY_MIN_MS = 120;
/** A readback costs real time; refining gets at least this many times the cost of showing it. */
export const DISPLAY_COST_RATIO = 4;

/**
 * Whether to read the accumulation back and draw it now: the last sample of an image always, a
 * preview only when its frame is whole, and a refining image no more often than DISPLAY_MIN_MS
 * and DISPLAY_COST_RATIO times what the last readback cost.
 */
export function displayDue(
  now: number, lastDisplay: number, lastCost: number, finished: boolean, preview: boolean, sampleFinished: boolean,
): boolean {
  if (finished) return true;
  if (preview) return sampleFinished;
  return now - lastDisplay >= Math.max(DISPLAY_MIN_MS, lastCost * DISPLAY_COST_RATIO);
}
