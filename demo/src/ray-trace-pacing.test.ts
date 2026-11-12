// Tests for how `SliceCosts` (src/ray-trace-rules.ts) paces the GPU slices of a picture once its
// stretches have been traced: simulated devices with a seeded jitter so a run repeats, driving the
// production class the way src/ray-trace-gpu.ts does (a pilot, then a still that starts from it).
// The phone's device follows the law measured on a Pixel 7: a slice waits a floor set by the
// costliest rows it touches plus work in proportion to its pixels. Devices that change between
// passes (jitter, spikes, a sudden slow-down, long dispatches costing more than their parts)
// check the headroom the strict sums keep. Where a test states what the rule before strict sums
// did, that number was measured by running the same test's helpers against that rule. The last
// tests give exact waits pass by pass instead, to pin single steps of the rule: how a cut lapses,
// is deleted or comes back for longer, and the last stretch taken with the slice before it.

import { describe, expect, test } from 'bun:test';
import { CUT_FIRST_PASSES, DISPATCH_CAP_MS, MERGE_TOLERANCE, SLICE_AIM_MS, SliceCosts, pilotSize } from './ray-trace-rules.ts';

/** A seeded stream of numbers in [0, 1). */
function stream(seed: number): () => number {
  let state = seed >>> 0;
  return () => {
    state = (state + 0x6d2b79f5) >>> 0;
    let t = state;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** The wait of pixels `[first, first + count)` of a `width` x `height` image in pass `pass` (ms; -1 is the pilot). */
type Device = (first: number, count: number, width: number, height: number, pass: number) => number;

interface Run {
  /** Each pass's slice count and sum of waits (ms), and every wait with its pass. */
  slices: number[];
  passMs: number[];
  waits: number[];
  passOf: number[];
  /** Cuts holding at the end of each pass. */
  cuts: number[];
}

function image(costs: SliceCosts, width: number, height: number, device: Device, samples: number, pilot = false): Run {
  const pixels = width * height;
  const run: Run = { slices: [], passMs: [], waits: [], passOf: [], cuts: [] };
  for (let sample = 0; sample < samples; sample++) {
    let count = 0;
    let total = 0;
    for (let first = 0; first < pixels;) {
      const size = Math.min(costs.sizeAt(first), pixels - first);
      expect(size % 64 === 0 || first + size === pixels).toBe(true);
      const ms = device(first, size, width, height, pilot ? -1 : sample);
      costs.report(first, size, ms);
      run.waits.push(ms);
      run.passOf.push(sample);
      count += 1;
      total += ms;
      first += size;
    }
    run.slices.push(count);
    run.passMs.push(total);
    run.cuts.push(costs.liveCuts);
  }
  return run;
}

/** A still of `samples` passes after its pilot. */
function still(width: number, height: number, device: Device, samples: number): Run {
  const p = pilotSize(width, height);
  const pilot = new SliceCosts(p.w * p.h, p.w * p.h);
  image(pilot, p.w, p.h, device, 1, true);
  return image(new SliceCosts(width * height, width * height, { from: pilot, margin: 1 }), width, height, device, samples);
}

const median = (values: number[]) => [...values].sort((a, b) => a - b)[values.length >> 1];
const sum = (values: number[]) => values.reduce((a, b) => a + b, 0);
/** Waits of passes `from` on. */
const laterWaits = (run: Run, from = 1) => run.waits.filter((_, i) => run.passOf[i] >= from);
const PHONE: [number, number] = [567, 597];

/**
 * The phone's law, `slow` times slower: a floor by row (3 ms over the sky in the top 30 percent,
 * 13 to 20 ms over the part, 25 to 33 ms over its base in the last 15 percent) paid once a slice
 * at the costliest row it touches, plus 0.4 microseconds a sky pixel and 1.4 a part pixel (about
 * 450 ms a pass), 15 percent jitter, and 25 ms more on the slice after a picture is shown, every
 * 5 to 16 slices.
 */
function phone(slow = 1, seed = 11): Device {
  const next = stream(seed);
  const floor = (r: number) => (r < 0.3 ? 3 : r >= 0.85 ? 25 + 8 * ((r - 0.85) / 0.15) : 13 + 7 * ((r - 0.3) / 0.55));
  const work = (r: number) => (r < 0.3 ? 0.0004 : 0.0014);
  let untilShown = 8;
  return (first, count, width, height) => {
    const row = (p: number) => Math.floor(p / width) / height;
    let f = floor(row(first + count - 1));
    for (let p = first; p < first + count; p += width) f = Math.max(f, floor(row(p)));
    let k = 0;
    for (let p = first; p < first + count; p += 64) k += work(row(p)) * Math.min(64, first + count - p);
    let ms = (f + k) * (1 + 0.15 * (2 * next() - 1)) * slow;
    if (--untilShown < 0) {
      ms += 25;
      untilShown = 5 + Math.floor(next() * 12);
    }
    return ms;
  };
}

/** `pass` ms of work a whole picture, evenly spread, plus `fixed` ms a slice, 15 percent jitter. */
function even(pass: number, fixed: number, seed = 5): Device {
  const next = stream(seed);
  return (_first, count, width, height) => (fixed + (pass * count) / (width * height)) * (1 + 0.15 * (2 * next() - 1));
}

/** A scene by row cost (ms a pixel at row fraction `r`), plus 2 to 5 ms a dispatch and a stray 10 ms one time in forty. */
function scene(cost: (r: number) => number, seed: number): Device {
  const next = stream(seed);
  return (first, count, width, height) => {
    let ms = 2 + 3 * next() + (next() < 0.025 ? 10 : 0);
    for (let p = first; p < first + count; p += 64) ms += cost(Math.floor(p / width) / height) * Math.min(64, first + count - p);
    return ms;
  };
}

/** `device` changing between passes: each wait up to `noise` either way, `spike` of them `spikeX` times, and `stepX` times from pass `stepAt`. */
function changing(device: Device, o: { noise?: number; spike?: number; spikeX?: number; stepAt?: number; stepX?: number }, seed: number): Device {
  const next = stream(seed + 1000);
  return (first, count, width, height, pass) => {
    let ms = device(first, count, width, height, pass) * (1 + (o.noise ?? 0) * (2 * next() - 1));
    if (next() < (o.spike ?? 0)) ms *= o.spikeX ?? 3;
    if (o.stepAt !== undefined && pass >= o.stepAt) ms *= o.stepX ?? 2;
    return ms;
  };
}

const thin = (r: number) => (r >= 0.12 && r < 0.125 ? 0.2 : 0.0001);
const band = (r: number) => (r >= 0.45 && r < 0.55 ? 0.1 : 0.001);
const below = (r: number) => (r < 0.3 ? 0.04 : 0.0001);
const above = (r: number) => (r < 0.5 ? 0.0001 : 0.04);
const SCENES: [string, number, number, (r: number) => number][] = [
  ['thin glass', 1000, 1000, thin],
  ['a hundred times the cost on a tenth of the rows', 567, 597, band],
  ['glass below', 1000, 1000, below],
  ['glass above', 811, 398, above],
];

/** Over `seeds` seeds: how many waits of passes 2 on passed the cap, the worst, and the mean time a pass. */
function overSeeds(make: (seed: number) => Device, width: number, height: number, passes: number, seeds = 12) {
  let over = 0;
  let worst = 0;
  let passMs = 0;
  for (let seed = 1; seed <= seeds; seed++) {
    const run = still(width, height, make(seed), passes);
    for (const ms of laterWaits(run)) {
      worst = Math.max(worst, ms);
      if (ms > DISPATCH_CAP_MS) over += 1;
    }
    passMs += sum(run.passMs.slice(1)) / (passes - 1) / seeds;
  }
  return { over, worst, passMs };
}

describe('a traced picture is sliced by the strict sum of what its stretches waited', () => {
  test("the phone's law: a few slices a pass by the sixth pass, none over the cap", () => {
    // Before (30 ms aimed by band rates): 154, 250, 398, ... and 691 slices and 22 s a pass at
    // the steady state of this model, which is harsher than the phone (it drew 62 slices and
    // 1.9 s a pass). Now passes 1 to 8 take 154, 54, 23, 15, 11, 9, 9, 9 slices and 4.66, 1.97,
    // 1.03, 0.77, 0.67, 0.58, 0.55, 0.61 s, settling at 7 slices and 532 ms a pass, the longest
    // wait 133 ms, 74.8 s for 128 passes; the first pass is the rule before's, unchanged.
    const run = still(...PHONE, phone(), 128);
    expect(run.slices[0]).toBe(154);
    expect(run.passMs[2]).toBeLessThan(1100);
    for (const slices of run.slices.slice(5)) expect(slices).toBeLessThanOrEqual(12);
    expect(median(run.passMs.slice(8))).toBeLessThan(560);
    for (const ms of run.waits) expect(ms).toBeLessThanOrEqual(150);
  });

  test('a desktop whose whole pass costs 20 ms: one slice a pass from the second', () => {
    // As before: 3 slices in the first pass, then 1.
    const run = still(...PHONE, even(20, 0), 16);
    for (const slices of run.slices.slice(1)) expect(slices).toBe(1);
  });

  test('a desktop that adds 26 ms to every slice: one slice a pass by the sixth pass', () => {
    // Before: 123, 74, 50, 39, 31, 26, ... and still 10 slices (288 ms) at the sixteenth pass,
    // 12.9 s for 16. Now 123, 40, 13, 5, 2, then 1 slice of about 45 ms, 5.3 s for 16.
    const run = still(...PHONE, even(20, 26), 16);
    for (const slices of run.slices.slice(5)) expect(slices).toBe(1);
    expect(run.passMs[15]).toBeLessThan(60);
  });

  test('waits exactly in proportion to pixels, 3 s a pass: as fast as before, slices under the aim', () => {
    // Before: 100 slices of 30 ms a pass (worst 45 ms), 47,952 ms for 16 passes. Now 32 slices
    // of about 98 ms, 48,007 ms (0.1 percent more), the longest 116 ms.
    const run = still(...PHONE, even(3000, 0), 16);
    expect(sum(run.passMs)).toBeLessThan(47_952 * 1.01);
    expect(median(run.slices.slice(2))).toBeLessThanOrEqual(33);
    for (const ms of laterWaits(run)) expect(ms).toBeLessThanOrEqual(SLICE_AIM_MS * 1.2);
  });

  test('the phone slowed three times improves on the rule before, under the cap', () => {
    // Floors up to about 100 ms, at the aim: few stretches merge. Before: 3,645 slices and 219 s
    // a pass from the second pass on. Now 2,427 in the second and 1,593 by the eighth, 117 s a
    // pass, worst 139 ms; making the first pass itself quick on such a device is separate work.
    const run = still(...PHONE, phone(3), 8);
    expect(run.slices[7]).toBeLessThan(1700);
    for (const ms of run.waits) expect(ms).toBeLessThanOrEqual(DISPATCH_CAP_MS);
  });
});

describe('costly scenes are covered no worse than before', () => {
  // Passes 2 to 8 over 12 seeds; the first pass is the rule before's, unchanged.
  const pinned: Record<string, [number, number]> = {
    // The cap passes and the worst wait now. Before: thin glass 22 passes, worst 1,037 ms; none
    // on the rest, worst 203, 183 and 194 ms.
    'thin glass': [13, 1017],
    'a hundred times the cost on a tenth of the rows': [0, 105],
    'glass below': [0, 97],
    'glass above': [0, 141],
  };
  for (const [name, width, height, cost] of SCENES) {
    test(`${name}: passes 2 to 8 pass the cap no more often or further than before`, () => {
      const { over, worst } = overSeeds(seed => scene(cost, seed), width, height, 8);
      expect(over).toBeLessThanOrEqual(pinned[name][0]);
      expect(worst).toBeLessThanOrEqual(pinned[name][1] + 1);
    });
  }
});

describe('a device that changes between passes keeps the cap', () => {
  // Passes 2 to 16 over 8 seeds; each pins the counts and worst it gives now and states the rule
  // before's, measured with these helpers.
  const noisy: Record<string, [number, number]> = {
    // Every wait up to 30 percent either way, independently. Before: thin glass 39, worst
    // 1,153 ms; the costly band 0 (236), glass below 1 (261), glass above 0 (186). Now as pinned:
    // thin glass's worst, inside the stretch a cheap average proved, is 2 percent further.
    'thin glass': [12, 1176],
    'a hundred times the cost on a tenth of the rows': [0, 169],
    'glass below': [0, 171],
    'glass above': [0, 183],
  };
  for (const [name, width, height, cost] of SCENES) {
    test(`${name}, each wait 30 percent either way: no more cap passes than before`, () => {
      const { over, worst } = overSeeds(seed => changing(scene(cost, seed), { noise: 0.3 }, seed), width, height, 16, 8);
      expect(over).toBeLessThanOrEqual(noisy[name][0]);
      expect(worst).toBeLessThanOrEqual(noisy[name][1] + 1);
    });
  }

  test('one wait in fifty three times longer on steady waits: passed more often than before, but well under twice the cap', () => {
    // A slice near the aim taken three times over passes the cap where the rule before's 30 ms
    // slices did not. Over 8 seeds and passes 2 to 16, before: the costly band 1 (worst 295 ms),
    // glass below 0 (207), glass above 0 (194). Now 14 (283), 23 (290) and 18 (309): under a
    // percent of the slices,
    // and on otherwise steady waits the worst about 1.2 times the cap, as an own proof is at
    // most half again the aim before it is halved.
    const pinned = [[14, 283], [23, 290], [18, 309]];
    SCENES.slice(1).forEach(([, width, height, cost], i) => {
      const { over, worst } = overSeeds(seed => changing(scene(cost, seed), { spike: 0.02, spikeX: 3 }, seed), width, height, 16, 8);
      expect(over).toBeLessThanOrEqual(pinned[i][0]);
      expect(worst).toBeLessThanOrEqual(pinned[i][1] + 1);
    });
  });

  test('spikes on top of 30 percent jitter: about half a percent of slices over the cap, the worst under twice it', () => {
    // A spike on a slice the jitter already made long goes furthest. Over 8 seeds and passes 2
    // to 16, before: the costly band 3 (worst 419 ms), glass below 2 (313), glass above 1 (297).
    // Now 34 (425), 124 (428) and 71 (459): about half a percent of the slices, the worst 1.7 to
    // 1.8 times the cap.
    const pinned = [[34, 425], [124, 428], [71, 459]];
    SCENES.slice(1).forEach(([, width, height, cost], i) => {
      const { over, worst } = overSeeds(seed => changing(scene(cost, seed), { noise: 0.3, spike: 0.02, spikeX: 3 }, seed), width, height, 16, 8);
      expect(over).toBeLessThanOrEqual(pinned[i][0]);
      expect(worst).toBeLessThanOrEqual(pinned[i][1] + 1);
    });
  });

  for (const [label, change, noneMs, allowed] of [
    ['one wait in fifty three times longer', { spike: 0.02, spikeX: 3 }, 590, 1.1],
    ['those spikes and 30 percent jitter', { noise: 0.3, spike: 0.02, spikeX: 3 }, 542, 1.15],
  ] as const) {
    test(`the phone model with ${label}: a long still does not slow as it goes`, () => {
      // A spike must not leave a cut that lasts: cuts lapse and are tried again. Mean of passes
      // 96 to 127 over 4 seeds against the same rule with neither refusal nor cutting again: 610
      // against 590 ms with spikes, 617 against 542 with jitter as well (the cuts jitter makes
      // while they hold cost 14 percent there). With cuts that never lapsed the live cuts grew
      // through the still (to 25 by pass 127) and passes slowed by half; now at most 4 hold.
      let late = 0;
      let lastCuts = 0;
      for (let seed = 1; seed <= 4; seed++) {
        const run = still(...PHONE, changing(phone(1, seed), change, seed), 128);
        late += sum(run.passMs.slice(96)) / 32 / 4;
        lastCuts = Math.max(lastCuts, ...run.cuts.slice(64));
        for (const ms of run.waits) expect(ms).toBeLessThanOrEqual(DISPATCH_CAP_MS * 2);
      }
      expect(late).toBeLessThanOrEqual(noneMs * allowed);
      expect(lastCuts).toBeLessThanOrEqual(4);
    });
  }

  test('a sudden slow-down to twice the cost from the sixth pass: one pass near twice the aim, then back', () => {
    // Before: the phone's slices kept shrinking, 588 to 3,140 a pass and 38 to 141 s a pass, worst
    // 125 ms. Now 9 a pass become 9 to 14, the worst 183 ms in the pass of the step, 165 after.
    const run = still(...PHONE, changing(phone(), { stepAt: 6, stepX: 2 }, 1), 12);
    for (const ms of run.waits) expect(ms).toBeLessThanOrEqual(DISPATCH_CAP_MS);
    const after = run.waits.filter((_, i) => run.passOf[i] >= 9);
    expect(Math.max(...after)).toBeLessThanOrEqual(SLICE_AIM_MS * 2);
  });

  for (const [exponent, mainMs] of [[1.1, 559], [1.3, 550], [1.6, 546]] as const) {
    test(`waits that grow as pixels to the power ${exponent}: no slower than before beyond a few percent, steady, under the cap`, () => {
      // A merged slice waits more than its parts: a merge that came back over the sum stops being
      // merged, and halves that together waited less than the whole are cut again, each cut
      // tried again when it lapses. Before: 559, 550 and 546 ms a pass, none over the cap. Now,
      // the mean of passes 25 to 128: 590 (5.5 percent more), 566 and 437; a pass that retries a
      // lapsed cut costs up to 612 ms at the power 1.3, 11 percent over the rule before.
      const device: Device = (_first, count) => 3 + 30 * Math.pow(count / 20000, exponent);
      const run = still(...PHONE, device, 128);
      const later = run.passMs.slice(24);
      expect(sum(later) / later.length).toBeLessThanOrEqual(mainMs * 1.06);
      expect(Math.max(...run.passMs.slice(16))).toBeLessThanOrEqual(mainMs * 1.12);
      for (const ms of laterWaits(run)) expect(ms).toBeLessThanOrEqual(DISPATCH_CAP_MS);
    });
  }
});

/** A still of one row whose first pass traced the given slices and waits. */
function traced(slices: [number, number][]): SliceCosts {
  const pixels = sum(slices.map(([count]) => count));
  const pilot = new SliceCosts(pixels, pixels);
  pilot.report(0, pixels, 5);
  const costs = new SliceCosts(pixels, pixels, { from: pilot, margin: 1 });
  let first = 0;
  for (const [count, ms] of slices) {
    costs.report(first, count, ms);
    first += count;
  }
  return costs;
}

describe('the strict sum', () => {
  test('takes whole traced stretches while their waits sum within the aim, ending on a stretch', () => {
    expect(SLICE_AIM_MS).toBe(100);
    const costs = traced([[640, 30], [640, 40], [640, 30], [640, 40]]);
    expect(costs.sizeAt(0)).toBe(1920);
    expect(costs.aimAt(0)).toBe(SLICE_AIM_MS);
  });

  test('a stretch past the aim is taken alone, never with a neighbour', () => {
    expect(traced([[640, 60], [640, 60], [640, 60]]).sizeAt(0)).toBe(640);
    expect(traced([[640, 140], [640, 10]]).sizeAt(0)).toBe(640);
  });

  test('a traced stretch over half again the aim is halved, and what is left of it taken whole', () => {
    const costs = traced([[1280, 180], [640, 10]]);
    expect(costs.sizeAt(0)).toBe(640);
    costs.report(0, 640, 90);
    expect(costs.sizeAt(640)).toBe(640);
  });

  test('a traced stretch over the cap is covered again by the first rule', () => {
    const costs = traced([[6400, 400], [640, 10]]);
    expect(costs.aimAt(0)).toBe(30);
    expect(costs.sizeAt(0)).toBeLessThan(3200);
  });

  test('a merge that came back over the sum is not merged across again', () => {
    const costs = traced([[640, 30], [640, 30], [640, 30]]);
    expect(costs.sizeAt(0)).toBe(1920);
    costs.report(0, 1920, 120);
    // The merged proof is cut at the boundary nearest its middle.
    expect(costs.sizeAt(0)).toBe(640);
  });

  test('stretches carried from the pilot are not summed', () => {
    const pilot = new SliceCosts(2560, 2560);
    for (let first = 0; first < 2560; first += 640) pilot.report(first, 640, 20);
    const costs = new SliceCosts(2560, 2560, { from: pilot, margin: 1 });
    expect(costs.aimAt(0)).toBe(30);
    // Summed as own proofs, the four would make one slice of 2,560 pixels.
    expect(costs.sizeAt(0)).toBeLessThan(2560);
  });
});

describe('cuts lapse and are tried again', () => {
  // Waits chosen pass by pass rather than a device, so each step reads as what the rule did with
  // it. `liveCuts` counts the cuts holding for the coming pass. After a merge comes back long, the
  // two stretches it is cut back into wait half of it each: halves that waited clearly less than
  // the merge would be cut again by the halving rule, which the device tests above exercise.

  /**
   * One whole pass of `costs`, its slices waiting `waits` in order: the slice sizes it took, which
   * show where its boundaries held.
   */
  function pass(costs: SliceCosts, pixels: number, waits: number[]): number[] {
    const sizes: number[] = [];
    for (let first = 0; first < pixels;) {
      expect(sizes.length).toBeLessThan(waits.length);
      const size = costs.sizeAt(first);
      costs.report(first, size, waits[sizes.length]);
      sizes.push(size);
      first += size;
    }
    expect(sizes.length).toBe(waits.length);
    return sizes;
  }

  /**
   * `count` passes of two 640-pixel slices waiting `half` ms each while the cut between them
   * holds; it has lapsed after the last of them.
   */
  function held(costs: SliceCosts, count: number, half: number): void {
    for (let i = 0; i < count; i++) {
      expect(costs.liveCuts).toBe(1);
      expect(pass(costs, 1280, [half, half])).toEqual([640, 640]);
    }
    expect(costs.liveCuts).toBe(0);
  }

  /**
   * Two stretches of 640 that waited 20 ms each, whose merge came back at 50 (over 46, the sum
   * times the tolerance): cut once at 640, and held through the three passes after the pass that
   * cut it, so the fourth after tries the merge again.
   */
  function cutOnce(): SliceCosts {
    expect([SLICE_AIM_MS, MERGE_TOLERANCE, CUT_FIRST_PASSES]).toEqual([100, 1.15, 4]);
    const costs = traced([[640, 20], [640, 20]]);
    expect(costs.liveCuts).toBe(0);
    expect(pass(costs, 1280, [50])).toEqual([1280]);
    held(costs, CUT_FIRST_PASSES - 1, 25);
    return costs;
  }

  test('a merge that comes back over its sum makes one cut, tried again four passes after the pass that cut it', () => {
    const costs = cutOnce();
    expect(pass(costs, 1280, [50])).toEqual([1280]);
  });

  test('a retry that comes back within its sum deletes the cut, so a later failure there is cut for four passes again', () => {
    const costs = cutOnce();
    // 50 ms is within the sum of 50 times the tolerance: the cut is gone and the stretch is one.
    expect(pass(costs, 1280, [50])).toEqual([1280]);
    expect(costs.liveCuts).toBe(0);
    expect(pass(costs, 1280, [50])).toEqual([1280]);
    // A slow pass (over half again the aim) halves the stretch at 640 again, with nothing cut.
    expect(pass(costs, 1280, [160])).toEqual([1280]);
    expect(pass(costs, 1280, [25, 25])).toEqual([640, 640]);
    expect(costs.liveCuts).toBe(0);
    // The merge across 640 comes back long: a new cut, as short-lived as the first was.
    expect(pass(costs, 1280, [60])).toEqual([1280]);
    held(costs, CUT_FIRST_PASSES - 1, 30);
    expect(pass(costs, 1280, [60])).toEqual([1280]);
  });

  test('a retry that comes back over its sum cuts the same boundary for 8 passes, then 16', () => {
    const costs = cutOnce();
    // 60 ms is over the sum of 50 times the tolerance: 640 is cut again, for twice as long.
    expect(pass(costs, 1280, [60])).toEqual([1280]);
    held(costs, 2 * CUT_FIRST_PASSES - 1, 30);
    // Tried again 8 passes after: 70 ms is over the sum of 60 times the tolerance.
    expect(pass(costs, 1280, [70])).toEqual([1280]);
    held(costs, 4 * CUT_FIRST_PASSES - 1, 35);
    expect(pass(costs, 1280, [70])).toEqual([1280]);
  });

  test('a retry across two lapsed cuts that comes back over re-cuts both, not the boundary nearest its middle', () => {
    // Four stretches of 640: the first pass merges them in pairs and both pairs come back long,
    // cutting 640 and 1920.
    const costs = traced([[640, 35], [640, 35], [640, 35], [640, 35]]);
    expect(pass(costs, 2560, [90, 90])).toEqual([1280, 1280]);
    expect(costs.liveCuts).toBe(2);
    expect(pass(costs, 2560, [40, 40, 40, 40])).toEqual([640, 640, 640, 640]);
    // The middle two merge across 1280, which is not cut, and come back within their sum.
    expect(pass(costs, 2560, [25, 50, 25])).toEqual([640, 1280, 640]);
    expect(costs.liveCuts).toBe(2);
    expect(pass(costs, 2560, [25, 50, 25])).toEqual([640, 1280, 640]);
    // Both cuts lapse together: one merge of the whole row (sum 100) comes back at 120.
    expect(costs.liveCuts).toBe(0);
    expect(pass(costs, 2560, [120])).toEqual([2560]);
    // Both retried boundaries are cut again, and 1280 between them stays free.
    expect(costs.liveCuts).toBe(2);
    expect(pass(costs, 2560, [40, 80, 40])).toEqual([640, 1280, 640]);
  });

  test('a last traced stretch that would leave under a workgroup is taken in the slice before it', () => {
    // 20 pixels after a stretch too long to merge with them: one dispatch of 1,300, not 1,280 and 20.
    const costs = traced([[1280, 120], [20, 5]]);
    expect(pass(costs, 1300, [120])).toEqual([1300]);
  });
});
