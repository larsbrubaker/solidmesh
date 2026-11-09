// Tests for src/gpu-bench-rules.ts: a submission's size stays safe for a slow GPU and the report
// carries every number a person needs to paste back.

import { describe, expect, test } from 'bun:test';
import {
  FIRST_SLICE_RAYS, OVER_MS, SliceSizer, TARGET_MS, WORKGROUP_RAYS, formatRate, formatRatio, formatScatter, reportText, SCATTER_CONDITIONING_NOTE,
} from './gpu-bench-rules.ts';

describe('the scattering check', () => {
  const clean = { compared: 6765, exact: 6765, ties: 0, disagreed: 0, examples: '' };
  const inputs = {
    adapter: 'test', timing: 'test', triangles: 1, nodeBytes: 1, triangleBytes: 1, maxBinding: 1, depth: 1, rows: [],
  };

  test('says how many cases agree and how many ties there are', () => {
    const text = formatScatter({ compared: 100, exact: 97, ties: 1, disagreed: 2, examples: 'x' });
    expect(text).toContain('100 scattering cases');
    expect(text).toContain('97 agree');
    expect(text).toContain('1 ties');
    expect(text).toContain('2 disagreements');
  });

  test('the report carries the check and its examples when there is one', () => {
    const report = reportText({
      ...inputs,
      scatter: { ...clean, disagreed: 1, examples: 'case 42: pdf: CPU 1, GPU 2' },
    });
    expect(report).toContain('Scattering model, GPU against the CPU reference');
    expect(report).toContain('case 42: pdf: CPU 1, GPU 2');
    expect(report).toContain(SCATTER_CONDITIONING_NOTE);
  });

  test('a run without a scattering check reports none', () => {
    expect(reportText(inputs)).not.toContain('Scattering model');
  });
});

describe('SliceSizer', () => {
  test('starts small and in whole workgroups', () => {
    const sizer = new SliceSizer(1_000_000);
    expect(sizer.current).toBe(FIRST_SLICE_RAYS);
    expect(sizer.current % WORKGROUP_RAYS).toBe(0);
  });

  test('a fast device grows by at most double per submission', () => {
    const sizer = new SliceSizer(1_000_000);
    sizer.report(sizer.current, 0.001);
    expect(sizer.current).toBe(FIRST_SLICE_RAYS * 2);
  });

  test('converges to the target time on a steady device', () => {
    const sizer = new SliceSizer(10_000_000);
    const raysPerMs = 5_000;
    for (let i = 0; i < 20; i++) sizer.report(sizer.current, sizer.current / raysPerMs);
    expect(sizer.current / raysPerMs).toBeLessThanOrEqual(TARGET_MS);
    expect(sizer.current / raysPerMs).toBeGreaterThan(TARGET_MS * 0.8);
  });

  test('a submission past the limit is cut hard, so a phone keeps its device', () => {
    const sizer = new SliceSizer(1_000_000);
    for (let i = 0; i < 8; i++) sizer.report(sizer.current, 1);
    const before = sizer.current;
    sizer.report(before, OVER_MS + 1);
    expect(sizer.current).toBeLessThanOrEqual(before / 4);
  });

  test('a slice clipped by the end of the ray buffer does not shrink the next one', () => {
    const sizer = new SliceSizer(1_000_000);
    for (let i = 0; i < 6; i++) sizer.report(sizer.current, 1);
    const before = sizer.current;
    // Few rays, proportionally little time: the rate says the slice size is still fine.
    sizer.report(64, 64 * (TARGET_MS / before));
    expect(sizer.current).toBeGreaterThanOrEqual(before * 0.9);
  });

  test('never exceeds the ray set or goes below one workgroup', () => {
    const tiny = new SliceSizer(10);
    expect(tiny.current).toBe(WORKGROUP_RAYS);
    const capped = new SliceSizer(4096);
    for (let i = 0; i < 10; i++) capped.report(capped.current, 0.01);
    expect(capped.current).toBe(4096);
    for (let i = 0; i < 20; i++) capped.report(capped.current, 500);
    expect(capped.current).toBe(WORKGROUP_RAYS);
  });
});

describe('the report', () => {
  const row = {
    name: 'Camera rays (coherent, closest hit)',
    rays: 1000,
    gpuRaysPerSecond: 50e6,
    gpuMs: 20,
    submissions: 3,
    wasmRaysPerSecond: 2.5e6,
    compared: 172800,
    exact: 172798,
    ties: 1,
    disagreed: 1,
    examples: 'ray 7: reference triangle 3 at t 2, GPU nothing',
  };

  test('has the adapter, sizes, rates, ratio, agreement and disagreements', () => {
    const text = reportText({
      adapter: 'Test GPU',
      timing: 'timestamp-query',
      triangles: 100000,
      nodeBytes: 112,
      triangleBytes: 40,
      maxBinding: 134217728,
      depth: 10,
      rows: [row],
    });
    expect(text).toContain('Test GPU');
    expect(text).toContain('timestamp-query');
    expect(text).toContain('100,000');
    expect(text).toContain('50.00 M rays/s');
    expect(text).toContain('20.0x');
    expect(text).toContain('172798 exact, 1 ties, 1 disagreements');
    expect(text).toContain('ray 7: reference triangle 3');
    expect(text).toContain('not like for like');
  });

  test('rates and ratios read sensibly at every scale', () => {
    expect(formatRate(1_500)).toBe('1.5 k rays/s');
    expect(formatRate(900)).toBe('900 rays/s');
    expect(formatRatio(1, 0)).toBe('n/a');
  });
});
