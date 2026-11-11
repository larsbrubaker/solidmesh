// Tests for src/ray-trace-rules.ts: the renderer choice, the fallback words and the words for a
// GPU picked that can't draw, the pass count text and the renderer the pill names, what the page
// does when it stops drawing, and the walk that gives every pixel of every sample to the GPU
// exactly once.

import { describe, expect, test } from 'bun:test';
import { readFileSync } from 'fs';
import { join } from 'path';
import { SliceSizer } from './gpu-bench-rules.ts';
import {
  CARRIED_COST_MARGIN, COST_BANDS, DISPATCH_CAP_MS, DISPLAY_MIN_MS, GPU_COMPILING_BADGE, type GpuFailure,
  GpuStageBadge, NO_WEBGPU_MESSAGE, NO_WEBGPU_WARNING, NothingDrawn, PILOT_PIXELS, SampleWalk, SliceCosts,
  TrapRecovery, WORKER_TRAPPED_MESSAGE, chooseRoute, displayDue, fallbackMessage, gpuUnavailableMessage,
  passCountText, passPillText, pilotSize, rendererName, shownRoute, viewRoute,
} from './ray-trace-rules.ts';

const lost: GpuFailure = { kind: 'lost', detail: 'device lost: reset' };
const tooLarge: GpuFailure = {
  kind: 'too-large',
  detail: 'the face buffer would need 5 bytes but this GPU allows 1 byte in one storage binding.',
};

describe('chooseRoute', () => {
  test('Auto uses the GPU where the browser has WebGPU', () => {
    expect(chooseRoute('auto', true, null)).toEqual({ route: 'gpu', message: '' });
  });

  test('Auto without WebGPU stays on the CPU and says nothing', () => {
    expect(chooseRoute('auto', false, null)).toEqual({ route: 'cpu', message: '' });
  });

  test('GPU without WebGPU says so in words and draws nothing on the CPU', () => {
    expect(chooseRoute('gpu', false, null)).toEqual({ route: 'none', message: NO_WEBGPU_MESSAGE });
    expect(NO_WEBGPU_MESSAGE).toContain('WebGPU');
    expect(NO_WEBGPU_MESSAGE).toContain('Chrome or Edge');
  });

  test('CPU is always the CPU, even after a failure and with no words', () => {
    expect(chooseRoute('cpu', true, lost)).toEqual({ route: 'cpu', message: '' });
  });

  test('a lost device moves Auto to the CPU and does not retry by itself', () => {
    const decision = chooseRoute('auto', true, lost);
    expect(decision.route).toBe('cpu');
    expect(decision.message).toBe(fallbackMessage(lost));
  });

  test('a lost device under GPU stops the picture rather than moving it to the CPU', () => {
    expect(chooseRoute('gpu', true, lost)).toEqual({ route: 'none', message: gpuUnavailableMessage(lost) });
  });

  test('a scene too large for the GPU under GPU says why and draws nothing on the CPU', () => {
    const decision = chooseRoute('gpu', true, tooLarge);
    expect(decision.route).toBe('none');
    expect(decision.message).not.toContain('drawn on the CPU');
    expect(decision.message).toContain(tooLarge.detail.replace(/\.$/, ''));
    expect(decision.message).toContain('smaller Detail');
  });

  test('a scene too large for the GPU under Auto uses the CPU and says why', () => {
    const decision = chooseRoute('auto', true, tooLarge);
    expect(decision.route).toBe('cpu');
    expect(decision.message).toContain('drawn on the CPU');
    expect(decision.message).toContain(tooLarge.detail.replace(/\.$/, ''));
  });
});

describe('a GPU picked explicitly gets the GPU or is told why not', () => {
  const kinds: GpuFailure['kind'][] = ['lost', 'too-large', 'no-adapter', 'shader', 'trapped', 'error'];
  const failures: (GpuFailure | 'no WebGPU')[] = [
    ...kinds.map(kind => ({ kind, detail: kind === 'too-large' ? tooLarge.detail : 'the driver said no.' })),
    'no WebGPU',
  ];
  const decide = (choice: 'auto' | 'gpu', failure: GpuFailure | 'no WebGPU') =>
    failure === 'no WebGPU' ? chooseRoute(choice, false, null) : chooseRoute(choice, true, failure);
  const name = (failure: GpuFailure | 'no WebGPU') => (failure === 'no WebGPU' ? failure : failure.kind);

  for (const failure of failures) {
    test(`${name(failure)}: GPU never goes to the CPU, and says how to get a picture`, () => {
      const decision = decide('gpu', failure);
      expect(decision.route).toBe('none');
      expect(decision.message).not.toBe('');
      expect(decision.message).toContain('Renderer');
      expect(decision.message).toMatch(/Auto|CPU/);
      expect(decision.message).not.toContain('drawn on the CPU');
      expect(decision.message).not.toContain('..');
    });

    test(`${name(failure)}: Auto still falls back to the CPU with today's words`, () => {
      const decision = decide('auto', failure);
      expect(decision.route).toBe('cpu');
      const silent = failure === 'no WebGPU' || failure.kind === 'no-adapter';
      expect(decision.message).toBe(silent ? '' : fallbackMessage(failure as GpuFailure));
    });
  }

  test('the advice each failure already gave is kept', () => {
    expect(gpuUnavailableMessage({ kind: 'too-large', detail: 'x' })).toContain('smaller Detail');
    expect(gpuUnavailableMessage({ kind: 'trapped', detail: '' })).toContain('smaller Detail');
    expect(gpuUnavailableMessage({ kind: 'no-adapter', detail: '' })).toContain('graphics driver');
    expect(gpuUnavailableMessage({ kind: 'no-adapter', detail: '' })).not.toMatch(/reload/i);
    expect(gpuUnavailableMessage({ kind: 'shader', detail: '' })).not.toMatch(/reload|updat/i);
    expect(gpuUnavailableMessage({ kind: 'lost', detail: '' })).toMatch(/reload/i);
    expect(gpuUnavailableMessage({ kind: 'error', detail: 'bad.' })).toContain(': bad.');
  });

  test('Normals and Depth keep drawing on the CPU under GPU, without the path traced sentence', () => {
    expect(viewRoute('gpu', true, lost, false)).toEqual({ route: 'cpu', message: '' });
    expect(viewRoute('gpu', false, null, false)).toEqual({ route: 'cpu', message: '' });
    expect(viewRoute('auto', true, null, false)).toEqual({ route: 'cpu', message: '' });
    expect(viewRoute('gpu', true, lost, true)).toEqual(chooseRoute('gpu', true, lost));
    // Auto's sentence over Normals and Depth is as it was.
    expect(viewRoute('auto', true, lost, false)).toEqual({ route: 'cpu', message: fallbackMessage(lost) });
  });

  test('the stats say what is really drawing', () => {
    expect(rendererName('gpu')).toBe('GPU');
    expect(rendererName('cpu')).toBe('CPU');
    expect(rendererName('none')).toContain('GPU');
    expect(rendererName('none')).not.toContain('CPU');
  });

  test('a lost device or another error says the way to retry that works while GPU is picked', () => {
    for (const kind of ['lost', 'error'] as const) {
      const text = gpuUnavailableMessage({ kind, detail: 'bad.' });
      expect(text).toContain('To try the GPU again, pick CPU and then GPU in Renderer, or reload the page');
      expect(text).toContain('to draw it on the CPU, pick Auto or CPU.');
    }
    expect(gpuUnavailableMessage(lost)).toBe(
      'Your graphics card stopped responding, so the GPU stopped drawing the picture. To try the GPU again, pick CPU and then GPU in Renderer, or reload the page; to draw it on the CPU, pick Auto or CPU.');
  });
});

describe('shownRoute', () => {
  test('names the renderer that drew the picture on screen, not the one the page heads to', () => {
    // Auto fell back while the GPU's picture is still up, and CPU picked mid-refine.
    expect(shownRoute('gpu', 'cpu')).toBe('gpu');
    expect(shownRoute('cpu', 'gpu')).toBe('cpu');
    expect(passPillText(24, 128, shownRoute('gpu', 'cpu'))).toBe('Rendering 24 of 128 on the GPU');
  });

  test('before anything is drawn it names where the page is heading; once stopped, nothing', () => {
    expect(shownRoute(null, 'gpu')).toBe('gpu');
    expect(shownRoute(null, 'cpu')).toBe('cpu');
    expect(shownRoute('cpu', 'none')).toBe('none');
    expect(rendererName(shownRoute(null, 'none'))).toBe(rendererName('none'));
  });
});

describe('NothingDrawn', () => {
  test('reaching none from a drawing route blanks, drops and releases once', () => {
    const stop = new NothingDrawn();
    expect(stop.reached('cpu', true)).toBeNull();
    expect(stop.reached('none', true)).toEqual({ warn: null });
    // Every later kick while stopped changes nothing.
    expect(stop.reached('none', true)).toBeNull();
    // Normals drew on the CPU in between: coming back blanks its picture again.
    expect(stop.reached('cpu', true)).toBeNull();
    expect(stop.reached('none', true)).toEqual({ warn: null });
  });

  test('the first kick of a page that cannot draw counts as reaching none', () => {
    expect(new NothingDrawn().reached('none', true)).toEqual({ warn: null });
  });

  test('no WebGPU is logged once per Renderer pick, not on every stop', () => {
    const stop = new NothingDrawn();
    expect(stop.reached('none', false)).toEqual({ warn: NO_WEBGPU_WARNING });
    stop.reached('cpu', false);
    expect(stop.reached('none', false)).toEqual({ warn: null });
    stop.picked();
    stop.reached('cpu', false);
    expect(stop.reached('none', false)).toEqual({ warn: NO_WEBGPU_WARNING });
    expect(NO_WEBGPU_WARNING).toContain('WebGPU');
  });
});

describe('fallbackMessage', () => {
  test('a lost GPU says what happened and what the person can do', () => {
    const text = fallbackMessage(lost);
    expect(text).toContain('stopped responding');
    expect(text).toContain('CPU');
    expect(text).toContain('Renderer');
    expect(text).toContain('reload');
  });

  test('a too-large scene keeps the library words and does not double a period', () => {
    const text = fallbackMessage(tooLarge);
    expect(text).toContain('in one storage binding.');
    expect(text).not.toContain('..');
  });

  test('another failure says where it is drawn now and how to retry', () => {
    const text = fallbackMessage({ kind: 'error', detail: 'no adapter.' });
    expect(text).toBe('The graphics card could not run this render, so it is drawn on the CPU: no adapter. Reload the page to try the GPU again.');
  });
});

describe('passCountText', () => {
  test('reads the way MatterCAD\'s Realistic view reads while refining', () => {
    expect(passCountText(24, 128)).toBe('Rendering 24 of 128');
    expect(passCountText(0, 128)).toBe('Rendering 0 of 128');
  });

  test('says nothing once finished', () => {
    expect(passCountText(128, 128)).toBe('');
    expect(passCountText(200, 128)).toBe('');
  });
});

describe('passPillText', () => {
  test('names the renderer drawing the picture while it refines', () => {
    expect(passPillText(24, 128, 'gpu')).toBe('Rendering 24 of 128 on the GPU');
    expect(passPillText(24, 128, 'cpu')).toBe('Rendering 24 of 128 on the CPU');
  });

  test('says nothing once finished, or when nothing is drawing', () => {
    expect(passPillText(128, 128, 'gpu')).toBe('');
    expect(passPillText(128, 128, 'cpu')).toBe('');
    expect(passPillText(24, 128, 'none')).toBe('');
  });
});

describe('SampleWalk', () => {
  /** Runs a walk with a changing slice size and counts each (pixel, sample) it gives. */
  function coverage(pixels: number, cap: number, sizes: number[]) {
    const seen = new Map<string, number>();
    const walk = new SampleWalk(pixels, cap);
    let i = 0;
    for (let slice = walk.next(sizes[0]); slice; slice = walk.next(sizes[++i % sizes.length])) {
      expect(slice.first + slice.count).toBeLessThanOrEqual(pixels);
      for (let p = slice.first; p < slice.first + slice.count; p++) {
        const key = `${p}/${slice.sample}`;
        seen.set(key, (seen.get(key) ?? 0) + 1);
      }
    }
    return { seen, walk };
  }

  test('every pixel of every sample is given exactly once, however the slices are cut', () => {
    for (const sizes of [[64], [64, 128, 64], [4096, 1], [1000000]]) {
      const { seen, walk } = coverage(5000, 3, sizes);
      expect(walk.done).toBe(true);
      expect(seen.size).toBe(15000);
      for (const count of seen.values()) expect(count).toBe(1);
    }
  });

  test('the last slice of a sample is cut to the image, and the walk reports whole samples', () => {
    const walk = new SampleWalk(100, 2);
    expect(walk.next(64)).toEqual({ first: 0, count: 64, sample: 0 });
    expect(walk.samples).toBe(0);
    expect(walk.atSampleStart).toBe(false);
    expect(walk.next(64)).toEqual({ first: 64, count: 36, sample: 0 });
    expect(walk.samples).toBe(1);
    expect(walk.atSampleStart).toBe(true);
    walk.next(100);
    expect(walk.done).toBe(true);
    expect(walk.next(64)).toBeNull();
  });

  test('a preview walk ends after its one sample', () => {
    const walk = new SampleWalk(10, 1);
    walk.next(100);
    expect(walk.done).toBe(true);
  });
});

describe('displayDue', () => {
  test('the last sample always draws', () => {
    expect(displayDue(0, 0, 0, true, false, true)).toBe(true);
    expect(displayDue(0, 0, 1000, true, true, false)).toBe(true);
  });

  test('a preview draws only when its frame is whole', () => {
    expect(displayDue(10000, 0, 0, false, true, false)).toBe(false);
    expect(displayDue(10000, 0, 0, false, true, true)).toBe(true);
  });

  test('a refining image waits the minimum and pays for its readback', () => {
    expect(displayDue(DISPLAY_MIN_MS - 1, 0, 0, false, false, true)).toBe(false);
    expect(displayDue(DISPLAY_MIN_MS, 0, 0, false, false, false)).toBe(true);
    expect(displayDue(390, 0, 100, false, false, true)).toBe(false);
    expect(displayDue(400, 0, 100, false, false, false)).toBe(true);
  });
});

describe('the words for failures the person cannot fix by reloading', () => {
  const noAdapter: GpuFailure = { kind: 'no-adapter', detail: 'no adapter' };
  const shader: GpuFailure = { kind: 'shader', detail: 'error: bad token' };

  test('no usable adapter stays silent under Auto, as no WebGPU does', () => {
    expect(chooseRoute('auto', true, noAdapter)).toEqual({ route: 'cpu', message: '' });
  });

  test('no usable adapter under GPU says so and does not advise a reload', () => {
    const decision = chooseRoute('gpu', true, noAdapter);
    expect(decision.route).toBe('none');
    expect(decision.message).toContain('graphics driver');
    expect(decision.message).not.toMatch(/reload/i);
  });

  test('a refused shader names the driver and asks for nothing this page lacks', () => {
    const text = fallbackMessage(shader);
    expect(text).toContain('drawn on the CPU');
    expect(text).toContain('driver');
    expect(text).not.toMatch(/reload|results text|report/i);
    // A phone's driver is not something its owner can update.
    expect(text).not.toMatch(/updat/i);
    expect(text).toContain('slower');
  });

  test('a trapped worker during the GPU pack says what to do', () => {
    const text = fallbackMessage({ kind: 'trapped', detail: '' });
    expect(text).toContain('drawn on the CPU');
    expect(text).toContain('smaller Detail');
    expect(WORKER_TRAPPED_MESSAGE).toContain('smaller Detail');
  });
});

describe('GpuStageBadge', () => {
  function fake() {
    const log: string[] = [];
    return { log, view: { show: (t: string) => { log.push(`show ${t}`); }, hide: () => { log.push('hide'); } } };
  }

  test('every way out of the run clears the badge', () => {
    const { log, view } = fake();
    const badge = new GpuStageBadge(view);
    badge.set(GPU_COMPILING_BADGE, true);
    badge.set(null, true);
    expect(log).toEqual([`show ${GPU_COMPILING_BADGE}`, 'hide']);
  });

  test('picking CPU mid-compile hides it at once, and picking the GPU back shows it again', () => {
    const { log, view } = fake();
    const badge = new GpuStageBadge(view);
    badge.set(GPU_COMPILING_BADGE, true);
    badge.refresh(false);
    badge.refresh(true);
    expect(log).toEqual([`show ${GPU_COMPILING_BADGE}`, 'hide', `show ${GPU_COMPILING_BADGE}`]);
  });

  test('it never hides a message someone else put up', () => {
    const { log, view } = fake();
    const badge = new GpuStageBadge(view);
    badge.set(GPU_COMPILING_BADGE, true);
    badge.refresh(false); // the part is rebuilding: its own badge goes up next
    log.length = 0;
    badge.set(null, false);
    expect(log).toEqual([]);
  });

  test('tells the truth about the wait', () => {
    expect(GPU_COMPILING_BADGE).toContain('graphics card');
    expect(GPU_COMPILING_BADGE).toContain('first time');
  });
});

describe('hidden means hidden', () => {
  const css = readFileSync(join(import.meta.dir, '..', 'styles', 'main.css'), 'utf8');
  const rule = (selector: string) => {
    const at = css.indexOf(`${selector} {`);
    expect(at).toBeGreaterThanOrEqual(0);
    return css.slice(at, css.indexOf('}', at));
  };

  test('a badge or pill that is not showing is out of the accessibility tree', () => {
    expect(rule('.busy-badge')).toContain('visibility: hidden');
    expect(rule('.busy-badge.visible')).toContain('visibility: visible');
    expect(rule('.pass-pill')).toContain('visibility: hidden');
    expect(rule('.pass-pill.visible')).toContain('visibility: visible');
    expect(rule('.busy-badge.visible ~ .pass-pill')).toContain('visibility: hidden');
    expect(rule('.route-note')).toContain('visibility: hidden');
    expect(rule('.route-note.visible')).toContain('visibility: visible');
  });

  test('why the picture is on the CPU sits over the picture, seen without scrolling on a phone', () => {
    expect(rule('.route-note')).toContain('position: absolute');
    expect(rule('.route-note')).toContain('bottom: 10px');
    const page = readFileSync(join(import.meta.dir, 'demos', 'ray-trace.ts'), 'utf8');
    expect(page).toContain("routeNote.className = 'route-note';");
    expect(page).toContain('area.appendChild(routeNote);');
    expect(page).not.toContain('controls.appendChild(routeNote)');
  });
});

describe('TrapRecovery', () => {
  test('a trap retries once, the same worker trap counts once, and a second trap stops', () => {
    const recovery = new TrapRecovery();
    expect(recovery.trapped(1)).toBe('retry');
    expect(recovery.trapped(1)).toBe('seen');
    expect(recovery.stopped).toBe(false);
    expect(recovery.trapped(2)).toBe('stop');
    expect(recovery.stopped).toBe(true);
  });

  test('a good pass earns another retry; a new scene clears a stop', () => {
    const recovery = new TrapRecovery();
    recovery.trapped(1);
    recovery.succeeded();
    expect(recovery.trapped(2)).toBe('retry');
    recovery.trapped(3);
    expect(recovery.stopped).toBe(true);
    recovery.reset();
    expect(recovery.stopped).toBe(false);
    expect(recovery.trapped(4)).toBe('retry');
  });
});

describe('SliceCosts', () => {
  /** The image sizes pinned: a large square, and the browser's own 811 x 398 canvas. */
  const SIZES: [number, number][] = [[1000, 1000], [811, 398]];

  /**
   * What a dispatch costs besides its pixels, as a real device has it: 2 to 5 ms, drawn from a
   * seeded stream so a run is repeatable, and a stray 10 ms on about one dispatch in forty.
   */
  function fixedCosts(seed = 7) {
    let state = seed >>> 0;
    const next = () => {
      state = (state + 0x6d2b79f5) >>> 0;
      let t = state;
      t = Math.imul(t ^ (t >>> 15), t | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
    return () => 2 + 3 * next() + (next() < 0.025 ? 10 : 0);
  }

  /** ms a pixel at row fraction `row` (0 at the bottom of the image). */
  type Scene = (row: number) => number;

  /** Runs `samples` samples of a `width` x `height` image; the slowest dispatch and count of each. */
  function image(costs: SliceCosts, width: number, height: number, scene: Scene, samples: number,
    fixed: () => number, extra: (sample: number, slice: number) => number = () => 0) {
    const pixels = width * height;
    const worst: number[] = [];
    const slices: number[] = [];
    for (let sample = 0; sample < samples; sample++) {
      let longest = 0;
      let count = 0;
      for (let first = 0; first < pixels; count++) {
        const size = Math.min(costs.sizeAt(first), pixels - first);
        expect(size % 64 === 0 || first + size === pixels).toBe(true);
        let ms = fixed() + extra(sample, count);
        for (let p = first; p < first + size; p += 64) {
          ms += scene(Math.floor(p / width) / height) * Math.min(64, first + size - p);
        }
        costs.report(first, size, ms);
        longest = Math.max(longest, ms);
        first += size;
      }
      worst.push(longest);
      slices.push(count);
    }
    return { worst, slices };
  }

  /** A still: its pilot, then `samples` samples at full size starting from what the pilot proved. */
  function still(width: number, height: number, scene: Scene, samples: number,
    extra?: (sample: number, slice: number) => number) {
    const fixed = fixedCosts();
    const p = pilotSize(width, height);
    const pilot = new SliceCosts(p.w * p.h, p.w * p.h);
    const ran = image(pilot, p.w, p.h, scene, 1, fixed);
    const full = new SliceCosts(width * height, width * height, { from: pilot, margin: 1 });
    const result = image(full, width, height, scene, samples, fixed, extra);
    return { pilotSlices: ran.slices[0], pilotWorst: ran.worst[0], ...result };
  }

  /** A fast desktop GPU: the whole image is 30 ms of pixel work, evenly spread. */
  const desktop = (width: number, height: number): Scene => () => 30 / (width * height);
  /** Sky and glass at 400 to 1 per pixel, split at row fraction 0.3, glass below or above. */
  const glassBelow: Scene = row => (row < 0.3 ? 0.04 : 0.0001);
  const glassAbove: Scene = row => (row < 0.5 ? 0.0001 : 0.04);

  test('the size a uniform sizer reaches on sky would blow up on the wrap to the part', () => {
    const sizer = new SliceSizer(1_000_000);
    const fixed = fixedCosts();
    let first = 0;
    let longest = 0;
    for (let i = 0; i < 4000; i++) {
      const size = Math.min(sizer.current, 1_000_000 - first);
      let ms = fixed();
      for (let p = first; p < first + size; p += 64) ms += glassBelow(Math.floor(p / 1000) / 1000) * 64;
      sizer.report(size, ms);
      longest = Math.max(longest, ms);
      first = (first + size) % 1_000_000;
    }
    expect(longest).toBeGreaterThan(2000);
  });

  for (const [width, height] of SIZES) {
    describe(`at ${width} x ${height}`, () => {
      test('a fast uniform device runs at most two dispatches a pass by the fifth sample', () => {
        const { slices, worst } = still(width, height, desktop(width, height), 10);
        for (const steady of slices.slice(4)) expect(steady).toBeLessThanOrEqual(2);
        for (const ms of worst) expect(ms).toBeLessThanOrEqual(DISPATCH_CAP_MS);
      });

      test('a cold still reaches its first full sample in at most 20 dispatches, pilot included', () => {
        const { pilotSlices, slices } = still(width, height, desktop(width, height), 1);
        expect(pilotSlices + slices[0]).toBeLessThanOrEqual(20);
      });

      test('sky and glass at 400 to 1, in either order, never pass the cap', () => {
        for (const scene of [glassBelow, glassAbove]) {
          const { pilotWorst, worst } = still(width, height, scene, 6);
          expect(pilotWorst).toBeLessThanOrEqual(DISPATCH_CAP_MS);
          for (const ms of worst) expect(ms).toBeLessThanOrEqual(DISPATCH_CAP_MS);
        }
      });

      test('a stray wait the GPU did not cause does not hold the slices small', () => {
        const spike = (sample: number, slice: number) => (sample === 5 && slice === 0 ? 10 : 0);
        const { slices } = still(width, height, desktop(width, height), 10, spike);
        for (const steady of slices.slice(7)) expect(steady).toBeLessThanOrEqual(2);
      });

      test('previews while the camera moves settle at three dispatches or fewer', () => {
        const fixed = fixedCosts();
        const w = Math.round(width * 0.5);
        const h = Math.round(height * 0.5);
        const scene = desktop(w * 4, h * 4);
        let costs = new SliceCosts(w * h, w * h);
        const counts = [image(costs, w, h, scene, 1, fixed).slices[0]];
        for (let frame = 0; frame < 5; frame++) {
          costs = new SliceCosts(w * h, w * h, { from: costs, margin: CARRIED_COST_MARGIN });
          const { slices, worst } = image(costs, w, h, scene, 1, fixed);
          counts.push(slices[0]);
          expect(worst[0]).toBeLessThanOrEqual(DISPATCH_CAP_MS);
        }
        // The very first preview has nothing to start from and steps through; the next two grow
        // from those small proofs; from the fourth on, each is at most three dispatches.
        expect(counts[0]).toBeLessThanOrEqual(Math.ceil((w * h) / 2048) + 1);
        for (const warm of counts.slice(3)) expect(warm).toBeLessThanOrEqual(3);
      });
    });
  }

  test('what is not bounded: a costly feature inside a stretch a cheap average proved', () => {
    // Glass in 5 of the 125 rows one pilot slice proved: the pilot proves that stretch at its
    // average, and the still's first slice through it, sized by that average, lands on all the
    // glass. It exceeds the cap by about the share of the stretch the glass fills, once: from the
    // second sample every stretch is proven at full size. Pinned at the factor it reaches here.
    const thin: Scene = row => (row >= 0.12 && row < 0.125 ? 0.2 : 0.0001);
    const { worst } = still(1000, 1000, thin, 3);
    expect(worst[0]).toBeGreaterThan(DISPATCH_CAP_MS);
    expect(worst[0]).toBeLessThanOrEqual(DISPATCH_CAP_MS * 5);
    for (const ms of worst.slice(1)) expect(ms).toBeLessThanOrEqual(DISPATCH_CAP_MS);
  });

  test('a pilot is about PILOT_PIXELS in the same shape, with a row for every band', () => {
    const { w, h } = pilotSize(811, 398);
    expect(Math.abs(w * h - PILOT_PIXELS)).toBeLessThan(PILOT_PIXELS * 0.05);
    expect(Math.abs(w / h - 811 / 398)).toBeLessThan(0.05);
    expect(pilotSize(4000, 100).h).toBe(COST_BANDS);
    expect(pilotSize(100, 50)).toEqual({ w: 100, h: 50 });
  });

  test('a cold image enters unproven pixels a starting step at a time, whole workgroups', () => {
    const costs = new SliceCosts(1_000_000, 1_000_000);
    expect(costs.sizeAt(0)).toBe(2048);
    costs.report(0, 2048, 0);
    expect(costs.sizeAt(2048)).toBe(2048);
  });

  test('the device limit on one dispatch holds', () => {
    const costs = new SliceCosts(100_000, 4096, { from: new SliceCosts(100_000, 100_000), margin: 1 });
    costs.report(0, 100_000, 3);
    expect(costs.sizeAt(0)).toBeLessThanOrEqual(4096);
  });
});
