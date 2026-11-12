// The Ray Tracing page's decisions that need no browser, so `bun test` can run them
// (src/ray-trace-rules.test.ts): which renderer draws (`chooseRoute`, and `viewRoute` for the
// view on screen), the words the page says when Auto falls back to the CPU (`fallbackMessage`) and
// when a GPU picked explicitly can't draw (`gpuUnavailableMessage`; the page then draws nothing
// rather than quietly using the CPU), the pass count over the picture (`passCountText`, the same
// words MatterCAD's Realistic view shows, and `passPillText`, which adds the renderer drawing it),
// what the stats call the renderer (`rendererName`) and which renderer the picture on screen came
// from (`shownRoute`), what the page does on reaching `none` (`NothingDrawn`: blank the picture,
// drop a waiting pass, let the thumbnail capture go, log once), which pixel and sample each GPU
// slice covers (`SampleWalk`), how big a slice may be (`SliceCosts`) and the small pilot image a
// still is measured with first (`pilotSize`), when an image read back from the GPU is worth
// drawing (`displayDue`), what the badge says while the GPU route gets ready
// (`GpuStageBadge`) and when a crashed worker gets another try (`TrapRecovery`).
// src/demos/ray-trace.ts is the page and src/ray-trace-gpu.ts its WebGPU calls.

import { FIRST_SLICE_RAYS, TARGET_MS, WORKGROUP_RAYS } from './gpu-bench-rules.ts';

export type RendererChoice = 'auto' | 'cpu' | 'gpu';
/** Who draws the picture; `none` when the person picked GPU and the GPU can't draw it. */
export type Route = 'cpu' | 'gpu' | 'none';

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

/** How every sentence for a GPU picked that can't draw ends: the way to a picture on the CPU. */
const PICK_CPU = 'pick Auto or CPU in Renderer to draw it on the CPU';
/**
 * The retry that works for a failure a new start may fix. Picking GPU while GPU is picked changes
 * nothing, so the way to re-arm it is another Renderer and then GPU.
 */
const RETRY = 'To try the GPU again, pick CPU and then GPU in Renderer, or reload the page; to draw it on the CPU, pick Auto or CPU';

/** Chosen GPU on a browser with no WebGPU: says so and what the person can do. */
export const NO_WEBGPU_MESSAGE =
  `This browser does not have WebGPU, so the GPU cannot draw the picture. To use your graphics card, open this page in a current version of Chrome or Edge, or ${PICK_CPU}.`;

/** A sentence's `detail` without the period it may already end with. */
function clause(detail: string): string {
  return detail.trim().replace(/\.+$/, '');
}

/**
 * What the page says when the person picked GPU and `failure` stopped it. Nothing is drawn on the
 * CPU behind their back, so each sentence says how to get a picture there, beside the advice that
 * might get the GPU going.
 */
export function gpuUnavailableMessage(failure: GpuFailure): string {
  switch (failure.kind) {
    case 'lost':
      return `Your graphics card stopped responding, so the GPU stopped drawing the picture. ${RETRY}.`;
    case 'too-large':
      return `This part does not fit your graphics card, so the GPU cannot draw it: ${clause(failure.detail)}. A smaller Detail may fit, or ${PICK_CPU}.`;
    case 'no-adapter':
      return `This browser has WebGPU but found no usable graphics adapter, so the GPU cannot draw the picture. Updating your graphics driver may help, or ${PICK_CPU}.`;
    case 'shader':
      return `This device's graphics driver could not build the path tracing program, so the GPU cannot draw the picture. To draw it on the CPU, which is slower, pick Auto or CPU in Renderer.`;
    case 'trapped':
      return `Preparing this part for your graphics card failed, most likely out of memory, so the GPU cannot draw it. A smaller Detail may fit, or ${PICK_CPU}.`;
    case 'error':
      return `The graphics card could not run this render: ${clause(failure.detail)}. ${RETRY}.`;
  }
}

/** What the page says when `failure` has moved Auto's picture to the CPU. */
export function fallbackMessage(failure: GpuFailure): string {
  switch (failure.kind) {
    case 'lost':
      return 'Your graphics card stopped responding, so the picture moved to the CPU and kept going there. To try the GPU again, pick CPU and then GPU in Renderer, or reload the page.';
    case 'too-large':
      return `This part does not fit your graphics card, so it is drawn on the CPU: ${clause(failure.detail)}. A smaller Detail may fit.`;
    case 'no-adapter':
      return 'This browser has WebGPU but found no usable graphics adapter, so the picture is drawn on the CPU. Updating your graphics driver may help.';
    case 'shader':
      return "This device's graphics driver could not build the path tracing program, so the picture is drawn on the CPU, which is slower.";
    case 'trapped':
      return 'Preparing this part for your graphics card failed, most likely out of memory, so it is drawn on the CPU. A smaller Detail may fit.';
    case 'error':
      return `The graphics card could not run this render, so it is drawn on the CPU: ${clause(failure.detail)}. Reload the page to try the GPU again.`;
  }
}

/**
 * Which renderer draws the path traced picture, given the Renderer menu, whether the browser has
 * WebGPU, and any failure of the GPU route on this page. Auto uses the GPU wherever it can, falls
 * back to the CPU, and says nothing when there is no WebGPU or no usable adapter: nothing went
 * wrong and nothing was asked for. GPU gets the GPU or nothing (`none`), with a sentence saying
 * why: a person who asked for the GPU must be able to tell it isn't drawing. A failure holds until
 * the person picks a renderer again, so a lost device is never retried in a loop.
 */
export function chooseRoute(choice: RendererChoice, webGpu: boolean, failure: GpuFailure | null): RouteDecision {
  if (choice === 'cpu') return { route: 'cpu', message: '' };
  if (choice === 'gpu') {
    if (!webGpu) return { route: 'none', message: NO_WEBGPU_MESSAGE };
    if (failure) return { route: 'none', message: gpuUnavailableMessage(failure) };
    return { route: 'gpu', message: '' };
  }
  if (!webGpu || failure?.kind === 'no-adapter') return { route: 'cpu', message: '' };
  if (failure) return { route: 'cpu', message: fallbackMessage(failure) };
  return { route: 'gpu', message: '' };
}

/**
 * Which renderer draws the view on screen. Normals and Depth (`pathTraced` false) are hit data read
 * on the CPU whatever Renderer says, so they keep drawing when GPU is picked, and a GPU picked
 * that can't draw has nothing to say over them: its sentence is about the path traced picture.
 */
export function viewRoute(choice: RendererChoice, webGpu: boolean, failure: GpuFailure | null, pathTraced: boolean): RouteDecision {
  const decision = chooseRoute(choice, webGpu, failure);
  if (pathTraced) return decision;
  return { route: 'cpu', message: decision.route === 'none' ? '' : decision.message };
}

/**
 * Which renderer the pill and the stats name: the one that drew the picture on screen (`shown`,
 * null when nothing is), not the one the page is heading to, so a fallback or a new pick doesn't
 * relabel the old picture before the new renderer's first pass lands. Nothing is named once the
 * page has stopped drawing (`heading` is `none`): its picture is blanked.
 */
export function shownRoute(shown: Route | null, heading: Route): Route {
  return heading === 'none' ? 'none' : (shown ?? heading);
}

/** What the page does on reaching `none`, when it has just stopped drawing. */
export interface StopActions {
  /** Logged with `console.warn` for someone debugging remotely, or null when a failure already was. */
  warn: string | null;
}

/** Logged when GPU is picked on a browser that has no WebGPU. */
export const NO_WEBGPU_WARNING = 'Renderer is GPU but this browser has no WebGPU, so nothing is path traced.';

/**
 * The moment the page stops drawing. On reaching `none` from any other route it blanks the picture
 * (whatever was on screen came from another renderer or another view), drops a path traced pass
 * still waiting for the worker, and lets the thumbnail capture go, since nothing more will draw;
 * `reached` says so by returning the actions, and null while nothing changes. A GPU failure is
 * logged where it happens; no WebGPU is no failure, so it is logged here, once per Renderer pick.
 */
export class NothingDrawn {
  private last: Route | null = null;
  private warned = false;

  /** A Renderer pick: no WebGPU is worth logging again. */
  picked(): void {
    this.warned = false;
  }

  /** The route a kick found for the view on screen, and whether the browser has WebGPU. */
  reached(route: Route, webGpu: boolean): StopActions | null {
    const entered = route === 'none' && this.last !== 'none';
    this.last = route;
    if (!entered) return null;
    const warn = !webGpu && !this.warned ? NO_WEBGPU_WARNING : null;
    if (warn) this.warned = true;
    return { warn };
  }
}

/** What the stats call the renderer drawing the picture. */
export function rendererName(route: Route): string {
  switch (route) {
    case 'gpu':
      return 'GPU';
    case 'cpu':
      return 'CPU';
    case 'none':
      return "None: the GPU can't draw it";
  }
}

/**
 * The pass count over the picture, `samples` of `cap` finished: "Rendering 24 of 128" while it
 * refines and nothing once it has finished, as MatterCAD's Realistic view shows it.
 */
export function passCountText(samples: number, cap: number): string {
  return samples >= cap ? '' : `Rendering ${samples} of ${cap}`;
}

/**
 * The pill over the picture: the pass count and who is drawing it, "Rendering 24 of 128 on the
 * GPU", so the person can tell without scrolling to the stats; nothing once finished or when
 * nothing is drawing.
 */
export function passPillText(samples: number, cap: number, route: Route): string {
  const count = passCountText(samples, cap);
  if (count === '' || route === 'none') return '';
  return `${count} on the ${route === 'gpu' ? 'GPU' : 'CPU'}`;
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
/**
 * The most the earlier whole waits of the stretches one slice merges may sum to (ms). While the
 * same pixels cost about the same next pass, the sum bounds the merged slice's wait (a floor is
 * paid once, not once a stretch), so the aim can sit well above a phone's floors; it is set so
 * the sum, 30 percent more for a pass that runs slower, and the 25 ms a slice waits after a
 * picture is shown still stay under DISPATCH_CAP_MS with room.
 */
export const SLICE_AIM_MS = 100;
/**
 * A merged slice that waited more than this many times the sum that justified it did not save a
 * floor: the device may cost a long dispatch more than its parts (it time-slices them), so a cut
 * is made at the boundary nearest the merge's middle; halves that together waited less than the
 * whole over this factor are cut apart the same way.
 */
export const MERGE_TOLERANCE = 1.15;
/**
 * How many passes a new cut holds (`CUT_FIRST_PASSES`) before merging across it is tried again,
 * and the most a cut that keeps coming back can hold (`CUT_MOST_PASSES`). A cut is a guess, from
 * one or two waits that jitter or a spike may have made: a guess jitter made is gone after one
 * retry, while one a device keeps earning is retried less and less often (the lifetime doubles),
 * so the retries cost a superlinear device a long slice every 64 passes at most.
 */
export const CUT_FIRST_PASSES = 4;
export const CUT_MOST_PASSES = 64;
/**
 * An own proof that waited more than this (ms) is halved rather than taken whole: half again the
 * aim, so that a stretch whose floor alone is near the aim (a slow phone's) is still taken whole,
 * while a slice that came back long is cut before a pass three times slower could take it past
 * twice the cap.
 */
export const SPLIT_OVER_MS = SLICE_AIM_MS * 1.5;

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
  /** Traced by this picture at this size, rather than carried from the pilot or a preview. */
  own: boolean;
  /** What is left of a proof a later slice covered only part of. */
  rest?: boolean;
  /** One of two halves that together waited less than the proof they were cut from: cut again. */
  again?: boolean;
  /** The wait of the proof of exactly this stretch before it, if there was one (ms). */
  prev?: number;
}

/**
 * How many pixels the next GPU slice may cover. What a pixel costs depends on where it is, sky
 * next to glass by a factor of a hundred or more, and a slice's wait is not its pixels' work alone:
 * on a phone's GPU it is a floor set by what the slice's pixels show (about 3 ms for sky, 13 to
 * 20 ms over most of a part, 25 to 33 ms over its costliest rows, measured on a Pixel 7) plus work
 * in proportion to its pixels. The floor follows the pixels, not the submission (an empty dispatch
 * returns in about 4 ms), and it is paid once a slice, so only whole waits of real slices are used.
 * A picture's stretches are covered two ways:
 *
 * - **The first coverage** of a stretch, from what the pilot or the last preview proved. Every
 *   dispatch proves that its pixels cost at most its whole wait over their count, and the image is
 *   covered by the newest such proofs; no slice is issued whose cost by them exceeds
 *   DISPATCH_CAP_MS (less STRAY_ALLOWANCE_MS for a wait the GPU didn't cause). A stretch nothing
 *   has proven may be entered by FIRST_SLICE_RAYS pixels at a time, no more, which is how a cold
 *   image (the pilot) covers itself. Within one carried proof the cost is taken as even: what is
 *   not bounded is a costly feature inside a stretch that was proven by a cheap average. Estimates
 *   of each band's cost, learned only from dispatches over NOISE_FLOOR_MS, aim a slice at
 *   TARGET_MS; where the bands ahead have none yet, the slice doubles while the last came back
 *   under half the target, and holds otherwise. A preview is one pass, so it is all first coverage.
 * - **Stretches this picture has traced** (its own proofs, each within the cap). A slice over
 *   them waits about the sum of their whole waits or less while the same pixels cost about the
 *   same next pass (a merged slice pays one floor where its parts paid several), with no
 *   assumption that cost is even, so it takes whole own proofs while that sum stays within
 *   SLICE_AIM_MS, and ends on a proof's end. Merging continues pass by pass until waits sit between
 *   about half the aim and the aim. A stretch whose wait alone is over the aim is taken alone; one
 *   over SPLIT_OVER_MS (a pass that ran slow, or the wait after a picture is shown) is halved and
 *   what is left of it taken whole, so its next slice is bounded by its own wait, at most the cap.
 *   One over the cap proves only that its stretch is costly somewhere, so it is covered again by
 *   the first rule, which cuts it to slices that isolate the cost. Where a device costs a long
 *   dispatch more than its parts, a merge that came back over MERGE_TOLERANCE times its sum is cut
 *   at its middle, and halves that together waited clearly less than the whole did (and than it
 *   did the pass before, so one spike does not count) are cut apart. A cut holds for
 *   CUT_FIRST_PASSES passes; then merging across it is tried again, and the cut goes if that merge
 *   comes back within its sum or returns for twice as long, up to CUT_MOST_PASSES, if not. What holds: a slice over own stretches is issued with a bound of at most the aim, or
 *   of one stretch's own wait under the cap, and passes the cap only where its pixels cost more
 *   this pass than its bound said they did; where a stretch's floor alone is near the aim, few
 *   stretches merge and the slices there stay near the size the first coverage made them.
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
  /** Boundaries a merge has been refused across, for the rest of this picture. */
  private cuts = new Map<number, { until: number; life: number }>();
  /** Whole passes this image has reported, the clock cuts lapse by. */
  private passes = 0;
  /** The merge just sized: its stretch, the sum that justified it and the boundaries inside. */
  private merging: { first: number; end: number; sum: number; inner: number[] } | null = null;
  /** The proof just halved: its stretch, its wait, and the first half's wait once measured. */
  private halving: { first: number; mid: number; end: number; ms: number; half: number | null } | null = null;

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
      own: false,
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

  /**
   * The own proof `first` lies in, when it was traced by this picture and within the cap: a
   * stretch over the cap is costly somewhere and its cost is not known to be even, so it is
   * covered again as a first coverage is.
   */
  private owned(first: number): number {
    const index = this.proofs.findIndex(proof => proof.to > first);
    if (index < 0) return -1;
    const proof = this.proofs[index];
    return proof.from <= first && proof.own && proof.ms <= DISPATCH_CAP_MS ? index : -1;
  }

  /**
   * Pixels from `first` over own proofs by strict sums: whole proofs while the sum of their whole
   * waits stays within SLICE_AIM_MS, never across a refused boundary. It ends on a proof's end,
   * so the next slice starts aligned; the first proof is taken whole even past the aim, and one
   * over SPLIT_OVER_MS, or cut again because its halves saved time, is halved, what is left of a
   * halved one then taken whole, so the half that held the cost shows itself next pass.
   */
  private strictReach(index: number, first: number): { reach: number; sum: number; inner: number[] } {
    const lead = this.proofs[index];
    let reach: number;
    let sum = 0;
    const inner: number[] = [];
    if ((lead.ms > SPLIT_OVER_MS || lead.again) && (lead.rest || lead.to - first >= 2 * WORKGROUP_RAYS)) {
      reach = lead.rest
        ? lead.to - first
        : Math.max(WORKGROUP_RAYS, Math.floor((lead.to - first) / 2 / WORKGROUP_RAYS) * WORKGROUP_RAYS);
      sum = lead.ms;
    } else {
      let at = first;
      for (let i = index; i < this.proofs.length; i++) {
        const proof = this.proofs[i];
        if (i > index && (proof.from !== at || this.live(at))) break;
        if (!proof.own || proof.ms > SPLIT_OVER_MS) break;
        if (i > index && sum + proof.ms > SLICE_AIM_MS) break;
        if (i > index) inner.push(at);
        sum += proof.ms;
        at = proof.to;
      }
      reach = at - first;
    }
    // A proof made by a refused merge still ends at the boundaries it was refused across.
    for (const at of this.cuts.keys()) if (at > first && at < first + reach && this.live(at)) reach = at - first;
    return { reach, sum, inner: inner.filter(at => at < first + reach) };
  }

  /** Whether a cut holds at `at` this pass. */
  private live(at: number): boolean {
    const cut = this.cuts.get(at);
    return cut !== undefined && this.passes < cut.until;
  }

  /** Cuts at `at`: for CUT_FIRST_PASSES, or twice as long as last time it was cut, up to CUT_MOST_PASSES. */
  private cut(at: number): void {
    const before = this.cuts.get(at);
    const life = before ? Math.min(CUT_MOST_PASSES, before.life * 2) : CUT_FIRST_PASSES;
    this.cuts.set(at, { until: this.passes + life, life });
  }

  /** How many cuts hold this pass. */
  get liveCuts(): number {
    let count = 0;
    for (const at of this.cuts.keys()) if (this.live(at)) count += 1;
    return count;
  }

  /** The aim in force for a slice starting at `first` (ms), for the timings report. */
  aimAt(first: number): number {
    return this.owned(first) >= 0 ? SLICE_AIM_MS : TARGET_MS;
  }

  /**
   * The most pixels a slice starting at `first` should cover. Over own stretches it notes the
   * merge or the halving the size makes, which `report` of the same slice then judges, so each
   * call is to be followed by the report of the slice it sized.
   */
  sizeAt(first: number): number {
    const own = this.owned(first);
    if (own >= 0) {
      const { reach, sum, inner } = this.strictReach(own, first);
      const whole = Math.floor(reach / WORKGROUP_RAYS) * WORKGROUP_RAYS;
      let size = Math.min(Math.max(WORKGROUP_RAYS, this.maxPixels), first + reach === this.pixels ? reach : Math.max(WORKGROUP_RAYS, whole));
      // Less than a workgroup left over would be a dispatch of its own for nothing.
      const rest = this.pixels - first;
      if (rest - size < WORKGROUP_RAYS && rest <= this.maxPixels) size = rest;
      this.merging = inner.length > 0 ? { first, end: first + size, sum, inner: inner.filter(at => at < first + size) } : null;
      const lead = this.proofs[own];
      if (this.halving?.mid !== first) {
        this.halving = lead.from === first && !lead.rest && first + size < lead.to
          ? { first, mid: first + size, end: lead.to, ms: Math.min(lead.ms, lead.prev ?? lead.ms), half: null }
          : null;
      }
      return size;
    }
    this.merging = null;
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
    const merged = this.merging;
    this.merging = null;
    const halved = this.halving;
    let again = false;
    if (halved && halved.first === first && halved.mid === end) halved.half = ms;
    else if (halved && halved.half !== null && halved.mid === first && halved.end === end) {
      // Both halves together waited clearly less than the whole did, and than it did the pass
      // before (so one spike does not count): this device may cost a long dispatch more than
      // its parts, so the halves are cut again and not merged back while the cut holds.
      again = halved.half + ms < halved.ms / MERGE_TOLERANCE;
      if (again) this.cut(halved.mid);
      this.halving = null;
    } else this.halving = null;
    if (merged && merged.first === first && merged.end === end) {
      const retried = merged.inner.filter(at => this.cuts.has(at));
      if (ms > merged.sum * MERGE_TOLERANCE) {
        // A lapsed cut the merge tried again comes back for twice as long; otherwise only the
        // boundary nearest the middle is cut, so a refusal jitter caused costs one boundary and a
        // device that does cost long dispatches more is cut down a halving a pass.
        if (retried.length > 0) for (const at of retried) this.cut(at);
        else {
          const middle = (first + end) / 2;
          let nearest = merged.inner[0];
          for (const at of merged.inner) if (Math.abs(at - middle) < Math.abs(nearest - middle)) nearest = at;
          this.cut(nearest);
        }
      } else for (const at of retried) this.cuts.delete(at);
    }
    // The newest proof of a stretch replaces what covered it; a stretch only partly covered keeps
    // its own proof for the rest.
    const kept: Proof[] = [];
    let prev: number | undefined;
    for (const proof of this.proofs) {
      if (proof.from === first && proof.to === end) prev = proof.ms;
      if (proof.to <= first || proof.from >= end) kept.push(proof);
      else {
        if (proof.from < first) kept.push({ ...proof, to: first, rest: true });
        if (proof.to > end) kept.push({ ...proof, from: end, rest: true });
      }
    }
    kept.push({ from: first, to: end, rate: Math.max(ms, MIN_MEASURED_MS) / count, ms: Math.max(ms, MIN_MEASURED_MS), own: true, prev });
    if (again) for (const proof of kept) if (proof.from >= halved!.first && proof.to <= halved!.end) proof.again = true;
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
    if (end === this.pixels) this.passes += 1;
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
