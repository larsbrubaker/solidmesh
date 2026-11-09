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

/** The result of the scattering check (`gpu_bench_scatter_compare`). */
export interface ScatterRow {
  compared: number;
  exact: number;
  ties: number;
  disagreed: number;
  examples: string;
}

/** One scene of the render check (`gpu_bench_render_check`). */
export interface RenderRow {
  name: string;
  /** Every tier is clean and the GPU dropped no more samples than the CPU did. */
  passed: boolean;
  regions: number;
  exceeded: number;
  maxZ: number;
  worst: string;
  lineGroups: number;
  linesExceeded: number;
  linesMaxZ: number;
  linesWorst: string;
  pooledGroups: number;
  pooledExceeded: number;
  pooledMaxZ: number;
  pooledWorst: string;
  uniformExceeded: number;
  uniformMaxZ: number;
  uniformWorst: string;
  gpuDropped: number;
  cpuDropped: number;
  closedFormExceeded: number;
  cpuClosedFormExceeded: number;
  meanRelativeDifference: number;
  detectableUniform: number;
  detectableWhole: number;
  detectableRegionMedian: number;
  /** Samples per pixel over pixels rendered per second, the passes' GPU wall time. */
  samplesPerSecond: number;
  /** What the per-pass time counts: 'GPU timestamp queries' or 'submission wall time'. */
  timing: string;
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
  scatter?: ScatterRow;
  render?: RenderRow[];
  renderThreshold?: number;
  renderPooledThreshold?: number;
}

/** Beside the scattering check in the report, so a reader of a zero there knows its reach. */
export const SCATTER_CONDITIONING_NOTE = '  Note: at view cosines at or below about 1e-3 on narrow lobes the evaluate and pdf outputs depend so sharply on their inputs that f32 can only match them loosely, so a zero above does not vouch for them there.';

/** The scattering check in one line, for the page and the report. */
export function formatScatter(row: ScatterRow): string {
  return `${row.compared} scattering cases: ${row.exact} agree within f32, ${row.ties} ties, ${row.disagreed} disagreements`;
}

/** A render check scene in one line: how many groups exceeded, at each stated z. */
export function formatRender(row: RenderRow, threshold: number, pooledThreshold: number): string {
  const verdict = row.passed ? '' : 'FAILED: ';
  return `${verdict}${row.exceeded} of ${row.regions} regions and ${row.linesExceeded} of ${row.lineGroups} pixel lines over z = ${threshold} (largest ${Math.max(row.maxZ, row.linesMaxZ).toFixed(2)}); ${row.pooledExceeded} of ${row.pooledGroups} pooled groups and ${row.uniformExceeded} of 3 channels of the uniform-bias fit over z = ${pooledThreshold} (largest ${Math.max(row.pooledMaxZ, row.uniformMaxZ).toFixed(2)})`;
}

/** Samples per second, in the largest fitting unit. */
export function formatSamples(samplesPerSecond: number): string {
  if (samplesPerSecond >= 1e6) return `${(samplesPerSecond / 1e6).toFixed(2)} M samples/s`;
  if (samplesPerSecond >= 1e3) return `${(samplesPerSecond / 1e3).toFixed(1)} k samples/s`;
  return `${samplesPerSecond.toFixed(0)} samples/s`;
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
  if (inputs.scatter) {
    lines.push(`Scattering model, GPU against the CPU reference: ${formatScatter(inputs.scatter)}`);
    if (inputs.scatter.examples) lines.push(`  ${inputs.scatter.examples.split('\n').join('\n  ')}`);
    lines.push(SCATTER_CONDITIONING_NOTE);
  }
  if (inputs.render && inputs.render.length > 0) {
    const threshold = inputs.renderThreshold ?? 0;
    const pooled = inputs.renderPooledThreshold ?? 0;
    lines.push('', `Whole renders by the GPU path loop against the CPU path tracer, compared at four scales (fixed seed, thresholds z = ${threshold}, pooled z = ${pooled}):`);
    for (const row of inputs.render) {
      lines.push(`  ${row.name}: ${formatRender(row, threshold, pooled)}; GPU ${formatSamples(row.samplesPerSecond)} (${row.timing}); dropped samples: GPU ${row.gpuDropped}, CPU ${row.cpuDropped}; mean relative difference ${(row.meanRelativeDifference * 100).toFixed(3)}%`);
      lines.push(`    can detect a bias of: ${(row.detectableUniform * 100).toFixed(2)}% uniform, ${(row.detectableWhole * 100).toFixed(2)}% whole image, ${(row.detectableRegionMedian * 100).toFixed(2)}% in a median region`);
      if (row.closedFormExceeded > 0) lines.push(`    ${row.closedFormExceeded} channel-groups of the GPU differ from the closed-form answer`);
      if (row.cpuClosedFormExceeded > 0) lines.push(`    ${row.cpuClosedFormExceeded} channel-groups of the CPU reference differ from the closed-form answer`);
      if (row.gpuDropped > row.cpuDropped) lines.push(`    the GPU dropped ${row.gpuDropped} samples the CPU kept (${row.cpuDropped})`);
      if (row.exceeded > 0) lines.push(`    worst: ${row.worst}`);
      if (row.linesExceeded > 0) lines.push(`    worst line: ${row.linesWorst}`);
      if (row.pooledExceeded > 0) lines.push(`    worst pooled: ${row.pooledWorst}`);
      if (row.uniformExceeded > 0) lines.push(`    uniform bias: ${row.uniformWorst}`);
    }
  }
  lines.push('', 'For scale, not like for like: the library\'s CPU path tracer measured about 4.2 M rays/s on 8 threads at 100k triangles, on a different scene and ray mix.');
  return lines.join('\n');
}
