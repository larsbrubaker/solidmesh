// Tests of the Ray Tracing page's "Copy timings" report (src/ray-trace-report.ts): the exact text
// for each route and state, the medians and rates it computes, and that no number goes through
// the locale.

import { describe, expect, test } from 'bun:test';
import {
  Durations, MEDIAN_KEPT, PreviewStretch, StillTimings, type TimingsInputs, insideStill, remainderLine, stillFor,
  timingsReport,
} from './ray-trace-report.ts';

const UA = 'Mozilla/5.0 (Linux; Android 14; Pixel 7)';

function inputs(over: Partial<TimingsInputs>): TimingsInputs {
  return {
    build: 'abc1234, built 2026-11-11T20:00:00.000Z',
    userAgent: UA, pixelRatio: 2.625, areaWidth: 411.4, areaHeight: 300,
    webGpu: true, adapter: 'arm mali-g710',
    part: 'Torus', detail: 'About 100,000 triangles', triangles: 101234, view: 'Path traced',
    renderer: 'GPU', material: 'Glass', light: 'Studio (sky and soft sun)',
    shown: 'gpu', routeMessage: '', failure: null, cap: 128,
    lastPackMs: 250, lastShaderMs: 4200, still: null, previews: null, now: 0,
    ...over,
  };
}

const HEAD = [
  'Build: abc1234, built 2026-11-11T20:00:00.000Z',
  'solidmesh Ray Tracing timings',
  `Device: ${UA}; device pixel ratio 2.63; picture area 411 x 300 CSS pixels; GPU adapter: arm mali-g710`,
  'Settings: part Torus; detail About 100,000 triangles; 101234 triangles; view Path traced; renderer picked GPU; material Glass; light Studio (sky and soft sun)',
];

describe('Durations', () => {
  test('counts, totals, the largest and the median', () => {
    const d = new Durations();
    expect(d.median).toBe(0);
    for (const v of [5, 1, 9, 3]) d.add(v);
    expect([d.count, d.total, d.max, d.median]).toEqual([4, 18, 9, 4]);
    d.add(2);
    expect(d.median).toBe(3);
  });

  test('keeps only the latest values for the median, but every one in the totals', () => {
    const d = new Durations();
    for (let i = 0; i < MEDIAN_KEPT; i++) d.add(1000);
    for (let i = 0; i < MEDIAN_KEPT; i++) d.add(1);
    expect(d.count).toBe(2 * MEDIAN_KEPT);
    expect(d.max).toBe(1000);
    expect(d.median).toBe(1);
  });
});

describe('timingsReport', () => {
  test('a finished still on the GPU, with previews: every line, build first', () => {
    const still = new StillTimings(1000, [], 400, 300);
    still.record('pilot', 1000, 1030);
    still.record('views', 1030, 1034);
    still.record('views', 1034, 1040);
    still.record('slices', 1040, 1050, 20000);
    still.record('slices', 1050, 1080, 60000);
    still.record('slices', 1080, 1100, 40000);
    still.record('reads', 1100, 1105);
    still.record('reads', 1105, 1112);
    still.record('decodes', 1112, 1120);
    still.record('decodes', 1120, 1132);
    still.drew('gpu', 1, 120000, 60, 0, 1, 1200);
    still.drew('gpu', 1, 0, 0, 0, 1, 1300);
    const previews = new PreviewStretch();
    previews.shown('gpu', 100, 80, 0);
    previews.shown('gpu', 120, 90, 40);
    previews.shown('gpu', 110, 80, 140);
    const text = timingsReport(inputs({ still, previews, cap: 1, now: 5000 }));
    expect(text).toBe([
      ...HEAD,
      'Renderer in use: GPU',
      'GPU failure: none',
      'Picture: 400 x 300 pixels by the GPU; 1 of 1 passes (finished); wall time 200.0 ms; 5.00 passes/s and 600.0 k pixel-passes/s over the wall time; 16.67 passes/s over the renderer\'s own time (the Samples/s readout)',
      'Where the time went (GPU route):',
      '  Opening the graphics device: none',
      '  Part pack: none in this still (the last took 250.0 ms, before it began)',
      '  Shader build (pipeline creation): none in this still (the last took 4200.0 ms, before it began)',
      '  Pilot: 30.0 ms',
      '  GPU slices: 3, total wait 60.0 ms; median 40000 pixels, 20.0 ms; largest 60000 pixels, 30.0 ms',
      '  Read-backs: 2, total 12.0 ms, median 6.0 ms',
      '  Decodes in the worker: 2, total 20.0 ms, median 10.0 ms',
      '  View requests to the worker: 2, total 10.0 ms',
      '  Remainder of the wall time, not covered above: 68.0 ms',
      'Moving previews: 3 frames by the GPU; median 70.0 ms and slowest 100.0 ms from one frame shown to the next; median size 8800 pixels',
    ].join('\n'));
  });

  test('an unfinished still on the CPU, no previews yet, CPU route lines', () => {
    const still = new StillTimings(0, [], 200, 100);
    still.drew('cpu', 1, 20000, 40, 100000, 128, 50);
    still.drew('cpu', 2, 20000, 60, 150000, 128, 120);
    still.drew('cpu', 3, 20000, 50, 120000, 128, 180);
    const text = timingsReport(inputs({
      still, now: 500, renderer: 'CPU', shown: 'cpu', webGpu: false, adapter: null, lastPackMs: null, lastShaderMs: null,
    }));
    expect(text).toBe([
      HEAD[0], HEAD[1],
      `Device: ${UA}; device pixel ratio 2.63; picture area 411 x 300 CSS pixels; GPU adapter: no WebGPU in this browser`,
      HEAD[3].replace('renderer picked GPU', 'renderer picked CPU'),
      'Renderer in use: CPU',
      'GPU failure: none',
      'Picture: 200 x 100 pixels by the CPU; 3 of 128 passes (still refining); wall time 500.0 ms; 6.00 passes/s and 120.0 k pixel-passes/s over the wall time; 20.00 passes/s over the renderer\'s own time (the Samples/s readout)',
      'Where the time went (CPU route):',
      '  Time-boxed calls to the worker, each part of a pass: 3, total 150.0 ms, median 50.0 ms',
      '  Rays per second: 2.47 M rays/s',
      'Moving previews: none yet (orbit the part, then copy again to time them)',
    ].join('\n'));
  });

  test('GPU picked and the GPU cannot draw: says nothing is drawn, with the failure and the sentence', () => {
    const text = timingsReport(inputs({
      shown: 'none', adapter: null, routeMessage: 'The GPU lost its device.',
      failure: { kind: 'lost', detail: 'device was lost' },
    }));
    expect(text.split('\n').slice(4)).toEqual([
      'Renderer in use: None: the GPU can\'t draw it (the page says: The GPU lost its device.)',
      'GPU failure: lost: device was lost',
      'Picture: nothing drawn, because the GPU picked cannot draw it',
      'Moving previews: none yet (orbit the part, then copy again to time them)',
    ]);
    expect(text.split('\n')[2]).toEndWith('GPU adapter: WebGPU present, no adapter opened yet (the GPU route has not run)');
  });

  test('pack and shader inside the still count against its wall time', () => {
    const still = new StillTimings(0, [], 10, 10);
    still.record('pack', 0, 100);
    still.record('shader', 100, 400);
    still.drew('gpu', 1, 100, 1, 0, 128, 10);
    const lines = timingsReport(inputs({ still, now: 1000 })).split('\n');
    expect(lines).toContain('  Part pack: 100.0 ms in this still');
    expect(lines).toContain('  Shader build (pipeline creation): 300.0 ms in this still');
    expect(lines).toContain('  Remainder of the wall time, not covered above: 600.0 ms');
  });

  test('numbers never go through the locale: no grouping separators, a point for decimals', () => {
    const still = new StillTimings(0, [], 1600, 1000);
    still.drew('cpu', 1, 1600000, 12345.678, 98765432, 128, 20000);
    const text = timingsReport(inputs({ still, now: 20000, triangles: 1234567, detail: 'About 1 million triangles' }));
    expect(text).toContain('1234567 triangles');
    expect(text).toContain('wall time 20000.0 ms');
    expect(text).not.toMatch(/\d,\d{3}/);
  });
});

describe('steps that straddle the start of a still', () => {
  test('only the part of a step after the still began counts', () => {
    expect(insideStill(0, 100, 40)).toBe(60);
    expect(insideStill(50, 100, 40)).toBe(50);
    expect(insideStill(0, 30, 40)).toBe(0);
  });

  test('a shader build begun for the first preview and ending inside the first still', () => {
    // The preview asked for the pack at 0 and the shader at 100; the camera was still at 140, and
    // the shader build ended at 4300.
    const still = new StillTimings(140, [], 10, 10, 'gpu');
    still.record('pack', 0, 100);
    still.record('shader', 100, 4300);
    still.record('slices', 4300, 4400, 100);
    still.drew('gpu', 1, 100, 100, 0, 1, 4400);
    const lines = timingsReport(inputs({ still, cap: 1, lastPackMs: 100, lastShaderMs: 4200, now: 9000 })).split('\n');
    expect(lines).toContain('  Part pack: none in this still (the last took 100.0 ms, before it began)');
    expect(lines).toContain('  Shader build (pipeline creation): 4160.0 ms in this still (it began before this still and took 4200.0 ms in all)');
    expect(lines).toContain('  Remainder of the wall time, not covered above: 0.0 ms');
  });

  test('a step counted under a later still with the same request counts only from that still', () => {
    const args = [1, 2, 3];
    const earlier = new StillTimings(0, args, 10, 10, 'gpu');
    const later = stillFor(null, args, 10, 10, 'gpu', 500);
    expect(later).not.toBe(earlier);
    later.record('decodes', 400, 520);
    later.record('decodes', 520, 540);
    later.record('reads', 100, 300);
    later.drew('gpu', 1, 100, 20, 0, 1, 540);
    const lines = timingsReport(inputs({ still: later, cap: 1, now: 600 })).split('\n');
    expect(lines).toContain('  Decodes in the worker: 2, total 40.0 ms, median 20.0 ms; 1 began before this still and took 120.0 ms in all, 20.0 ms of it inside');
    expect(lines).toContain('  Read-backs: none');
    expect(lines).toContain('  Remainder of the wall time, not covered above: 0.0 ms');
  });

  test('the remainder is never negative, and wall time counts from the still', () => {
    const still = new StillTimings(1000, [], 10, 10, 'gpu');
    still.record('pilot', 0, 1100);
    still.drew('gpu', 1, 100, 1100, 0, 1, 1100);
    const text = timingsReport(inputs({ still, cap: 1, now: 2000 }));
    expect(text).toContain('wall time 100.0 ms');
    expect(text).toContain('  Pilot: 100.0 ms (it began before this still and took 1100.0 ms in all)');
    expect(text).toContain('  Remainder of the wall time, not covered above: 0.0 ms');
  });
});

describe('a change of renderer mid-still', () => {
  test('the same request on another renderer is a fresh still that says how far the first got', () => {
    const args = [7, 8];
    const onGpu = stillFor(null, args, 200, 100, 'gpu', 0);
    expect(stillFor(onGpu, args, 200, 100, 'gpu', 50)).toBe(onGpu);
    onGpu.record('slices', 0, 300, 20000);
    onGpu.drew('gpu', 12, 240000, 300, 0, 128, 1500);
    const onCpu = stillFor(onGpu, args, 200, 100, 'cpu', 1500);
    expect(onCpu).not.toBe(onGpu);
    expect(onCpu.restarted).toEqual({ from: 'gpu', samples: 12, wallMs: 1500 });
    onCpu.drew('cpu', 1, 20000, 40, 100000, 128, 1600);
    const text = timingsReport(inputs({
      still: onCpu, now: 2500, renderer: 'Auto', shown: 'cpu', routeMessage: 'The GPU stopped, so the CPU draws it.',
      failure: { kind: 'lost', detail: 'device was lost' },
    }));
    expect(text.split('\n').slice(4)).toEqual([
      'Renderer in use: CPU (the page says: The GPU stopped, so the CPU draws it.)',
      'GPU failure: lost: device was lost',
      'Picture: 200 x 100 pixels by the CPU; 1 of 128 passes (still refining); wall time 1000.0 ms; 1.00 passes/s and 20.0 k pixel-passes/s over the wall time; 25.00 passes/s over the renderer\'s own time (the Samples/s readout)',
      'Restarted: after the GPU stopped, this picture started again on the CPU; the GPU had drawn 12 of 128 passes in 1500.0 ms',
      'Where the time went (CPU route):',
      '  Time-boxed calls to the worker, each part of a pass: 1, total 40.0 ms, median 40.0 ms',
      '  Rays per second: 2.50 M rays/s',
      'Moving previews: none yet (orbit the part, then copy again to time them)',
    ]);
  });

  test('a new request is a fresh still with no restart line', () => {
    const first = stillFor(null, [1], 10, 10, 'gpu', 0);
    first.drew('gpu', 3, 300, 30, 0, 128, 100);
    const next = stillFor(first, [2], 10, 10, 'cpu', 200);
    expect(next.restarted).toBeNull();
    next.drew('cpu', 1, 100, 10, 1000, 128, 300);
    expect(timingsReport(inputs({ still: next, shown: 'cpu', now: 400 }))).not.toContain('Restarted');
  });
});

describe('the remainder of the wall time', () => {
  test('steps that overlap are named, never hidden behind a remainder of nothing', () => {
    expect(remainderLine(12.34)).toBe('  Remainder of the wall time, not covered above: 12.3 ms');
    expect(remainderLine(-0.01)).toBe('  Remainder of the wall time, not covered above: 0.0 ms');
    expect(remainderLine(-3000)).toBe('  Remainder of the wall time: none; the steps above add up to 3000.0 ms more than the wall time, so some of them overlapped');
  });

  test('two shader builds recorded in one still say they overlapped', () => {
    const still = new StillTimings(0, [], 10, 10, 'gpu');
    still.record('shader', 0, 4000);
    still.record('shader', 0, 4000);
    still.drew('gpu', 1, 100, 1, 0, 1, 5000);
    const text = timingsReport(inputs({ still, cap: 1, now: 6000 }));
    expect(text).toContain('so some of them overlapped');
  });
});
