// The parts of the GPU Ray Tracing page with no browser in them, so `bun test` can run them
// (src/gpu-bench-rules.test.ts): how large one submission to the GPU may be, and the plain-text
// report a person can paste back. src/gpu-bench.ts makes the WebGPU calls; src/demos/gpu-trace.ts
// is the page.
//
// A long submission can make the browser reset the GPU and lose the device, and phones are the
// slow end, so a submission's size adapts to how long the previous one took: toward TARGET_MS,
// never more than doubling, cut hard after one that ran past OVER_MS. Sizes are whole workgroups.

/** The time one submission aims for (ms), well under the ~100 ms a phone's GPU can bear. */
export const TARGET_MS = 30;
/** A submission that takes this long (ms) is cut hard rather than eased toward the target. */
export const OVER_MS = 80;
/** Rays in one workgroup of the shader (`@workgroup_size(64)` in trace.wgsl). */
export const WORKGROUP_RAYS = 64;
/** Where a dispatch starts: small enough for the slowest device to answer quickly. */
export const FIRST_SLICE_RAYS = 2048;

export class SliceSizer {
  private size: number;

  constructor(private readonly maxRays: number) {
    this.size = this.clamped(FIRST_SLICE_RAYS);
  }

  private clamped(rays: number): number {
    const whole = Math.floor(rays / WORKGROUP_RAYS) * WORKGROUP_RAYS;
    return Math.max(WORKGROUP_RAYS, Math.min(whole, Math.max(WORKGROUP_RAYS, this.maxRays)));
  }

  /** The rays the next submission should trace. */
  get current(): number {
    return this.size;
  }

  /** Tells the sizer that a submission of `rays` took `ms`. */
  report(rays: number, ms: number): void {
    if (ms >= OVER_MS) {
      this.size = this.clamped(rays / 4);
      return;
    }
    // A measurement of no time at all (a coarse clock on a tiny slice) only says "go bigger".
    const ideal = ms > 0 ? (rays * TARGET_MS) / ms : this.size * 2;
    this.size = this.clamped(Math.min(ideal, this.size * 2));
  }
}

export interface KindRow {
  name: string;
  rays: number;
  gpuRaysPerSecond: number;
  gpuMs: number;
  submissions: number;
  wasmRaysPerSecond: number;
  compared: number;
  exact: number;
  ties: number;
  disagreed: number;
  examples: string;
}

export interface ReportInputs {
  adapter: string;
  timing: string;
  triangles: number;
  nodeBytes: number;
  triangleBytes: number;
  maxBinding: number;
  depth: number;
  rows: KindRow[];
}

/** A rate as text in the largest fitting unit, with its unit. */
export function formatRate(raysPerSecond: number): string {
  if (raysPerSecond >= 1e6) return `${(raysPerSecond / 1e6).toFixed(2)} M rays/s`;
  if (raysPerSecond >= 1e3) return `${(raysPerSecond / 1e3).toFixed(1)} k rays/s`;
  return `${raysPerSecond.toFixed(0)} rays/s`;
}

/** The GPU's rate as a multiple of the reference's, as text. */
export function formatRatio(gpu: number, wasm: number): string {
  return wasm > 0 ? `${(gpu / wasm).toFixed(1)}x` : 'n/a';
}

/** The report for 'Copy results'. */
export function reportText(inputs: ReportInputs): string {
  const lines = [
    'solidmesh GPU Ray Tracing benchmark',
    `Adapter: ${inputs.adapter}`,
    `Timing: ${inputs.timing}`,
    `Triangles: ${inputs.triangles.toLocaleString('en-US')}`,
    `Node buffer: ${inputs.nodeBytes} bytes; triangle buffer: ${inputs.triangleBytes} bytes; storage binding limit: ${inputs.maxBinding} bytes; tree depth: ${inputs.depth}`,
    '',
  ];
  for (const row of inputs.rows) {
    lines.push(`${row.name}: ${row.rays} rays, ${row.submissions} submissions, ${row.gpuMs.toFixed(0)} ms`);
    lines.push(`  GPU ${formatRate(row.gpuRaysPerSecond)}; wasm reference (1 thread) ${formatRate(row.wasmRaysPerSecond)}; ratio ${formatRatio(row.gpuRaysPerSecond, row.wasmRaysPerSecond)}`);
    lines.push(`  agreement with the reference on ${row.compared} rays: ${row.exact} exact, ${row.ties} ties, ${row.disagreed} disagreements`);
    if (row.examples) lines.push(`  ${row.examples.split('\n').join('\n  ')}`);
  }
  lines.push('', 'For scale, not like for like: the library\'s CPU path tracer measured about 4.2 M rays/s on 8 threads at 100k triangles, on a different scene and ray mix.');
  return lines.join('\n');
}
