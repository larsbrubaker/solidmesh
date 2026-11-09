// The WebGPU calls of the GPU Ray Tracing page (src/demos/gpu-trace.ts): opening the device with
// the adapter's real limits, uploading a scene the worker built (demo/wasm/src/gpu_bench.rs), and
// dispatching the library's compute shader (crates/polygon-mesh/src/ray_tracer/flat_tree/
// trace.wgsl) in slices sized by src/gpu-bench-rules.ts. `runScatterCheck` runs the library's
// scattering model over the worker's fixed batch once (demo/wasm/src/gpu_bench_scatter.rs).
//
// Rays per second is rays traced divided by GPU time. GPU time comes from timestamp queries around
// each compute pass when the adapter has them, and otherwise from the wall time between
// `queue.submit` and `onSubmittedWorkDone` for that submission, which also counts queueing. The
// page reports which was used. Each submission waits for the one before it, so the uniform
// holding the slice is written once per submission.
//
// Types are `any` because Bun's types carry no WebGPU declarations and this demo has no others.

import { SliceSizer, WORKGROUP_RAYS } from './gpu-bench-rules.ts';

/** Bytes per ray and per hit: `RAY_STRIDE` and `HIT_STRIDE` in the library's flat_tree/shader.rs. */
const RAY_BYTES = 48;
const HIT_BYTES = 28;
/** Bytes per scattering result: `SCATTER_RESULT_STRIDE` in the library's flat_tree/scatter_shader.rs. */
const SCATTER_RESULT_BYTES = 48;

/** GPUBufferUsage and GPUMapMode, numeric so this file needs no WebGPU type declarations. */
const MAP_READ = 0x0001;
const COPY_SRC = 0x0004;
const COPY_DST = 0x0008;
const UNIFORM = 0x0040;
const STORAGE = 0x0080;
const QUERY_RESOLVE = 0x0200;
const MAP_MODE_READ = 0x0001;
const COMPUTE_STAGE = 0x0004;

export function hasWebGpu(): boolean {
  return !!(navigator as any).gpu;
}

export interface GpuContext {
  device: any;
  /** The adapter's name and vendor, as far as the browser says. */
  adapter: string;
  timestamps: boolean;
  /** The largest storage binding the device allows, in bytes. */
  maxBinding: number;
  maxWorkgroupsPerDimension: number;
}

/** What gpu_bench_prepare answers. */
export interface PreparedScene {
  shader: string;
  nodes: Uint8Array;
  triangles: Uint8Array;
  rays: [Uint8Array, Uint8Array, Uint8Array];
}

export interface Measured {
  rays: number;
  gpuMs: number;
  submissions: number;
  /** Submissions timed by the wall clock because their timestamps were unusable. */
  wallTimedSubmissions: number;
}

/** The device, with the adapter's real limits; throws an Error whose message is for the page. */
export async function openGpu(onLost: (message: string) => void): Promise<GpuContext> {
  const gpu = (navigator as any).gpu;
  const adapter = await gpu.requestAdapter({ powerPreference: 'high-performance' });
  if (!adapter) {
    throw new Error('This browser has WebGPU but found no usable graphics adapter. Update your graphics driver, or try current Chrome or Edge.');
  }
  const timestamps = adapter.features.has('timestamp-query');
  const device = await adapter.requestDevice({
    requiredFeatures: timestamps ? ['timestamp-query'] : [],
    requiredLimits: {
      maxStorageBufferBindingSize: adapter.limits.maxStorageBufferBindingSize,
      maxBufferSize: adapter.limits.maxBufferSize,
    },
  });
  device.lost.then((info: any) => onLost(String(info.message || info.reason || 'unknown reason')));
  const info = adapter.info ?? (await adapter.requestAdapterInfo?.()) ?? {};
  const described = [info.vendor, info.architecture, info.device, info.description]
    .filter((part: string) => part && part.length > 0)
    .join(' / ');
  return {
    device,
    adapter: described || 'not reported by this browser',
    timestamps,
    maxBinding: Math.min(adapter.limits.maxStorageBufferBindingSize, adapter.limits.maxBufferSize),
    maxWorkgroupsPerDimension: adapter.limits.maxComputeWorkgroupsPerDimension,
  };
}

function uploadBuffer(device: any, bytes: Uint8Array, usage: number): any {
  // WebGPU refuses an empty storage binding: a set with no rays still gets one record's space.
  const buffer = device.createBuffer({
    size: Math.max(bytes.byteLength, RAY_BYTES),
    usage,
    mappedAtCreation: true,
  });
  new Uint8Array(buffer.getMappedRange()).set(bytes);
  buffer.unmap();
  return buffer;
}

interface RaySet {
  count: number;
  hits: any;
  bindGroup: any;
  pipeline: any;
}

export class TraceRun {
  private sets: RaySet[] = [];
  private uniform: any;
  private buffers: any[] = [];
  private querySet: any = null;
  private resolveBuffer: any = null;
  private timeBuffer: any = null;
  private wallTimed = 0;

  private constructor(private context: GpuContext) {}

  /** Uploads a scene and compiles its pipelines; throws an Error with text for the page. */
  static async create(context: GpuContext, scene: PreparedScene): Promise<TraceRun> {
    const run = new TraceRun(context);
    const device = context.device;

    const module = device.createShaderModule({ code: scene.shader });
    const info = await module.getCompilationInfo();
    const errors = info.messages.filter((m: any) => m.type === 'error');
    if (errors.length > 0) {
      throw new Error(`The GPU driver refused the ray tracing shader: ${errors.map((m: any) => m.message).join('; ')}. This is a bug in the page; please report it with the results text.`);
    }

    const storage = (binding: number, type: string) => ({ binding, visibility: COMPUTE_STAGE, buffer: { type } });
    const layout = device.createBindGroupLayout({
      entries: [
        storage(0, 'read-only-storage'),
        storage(1, 'read-only-storage'),
        storage(2, 'read-only-storage'),
        storage(3, 'storage'),
        { binding: 4, visibility: COMPUTE_STAGE, buffer: { type: 'uniform' } },
      ],
    });
    const pipelineLayout = device.createPipelineLayout({ bindGroupLayouts: [layout] });
    const pipeline = (entryPoint: string) =>
      device.createComputePipelineAsync({ layout: pipelineLayout, compute: { module, entryPoint } });
    const closest = await pipeline('trace_closest_main');
    const any = await pipeline('trace_any_main');

    const nodes = uploadBuffer(device, scene.nodes, STORAGE);
    const triangles = uploadBuffer(device, scene.triangles, STORAGE);
    run.uniform = device.createBuffer({ size: 16, usage: UNIFORM | COPY_DST });
    run.buffers.push(nodes, triangles, run.uniform);

    scene.rays.forEach((bytes, kind) => {
      const count = bytes.byteLength / RAY_BYTES;
      const rays = uploadBuffer(device, bytes, STORAGE);
      const hits = device.createBuffer({ size: Math.max(count, 1) * HIT_BYTES, usage: STORAGE | COPY_SRC });
      run.buffers.push(rays, hits);
      run.sets.push({
        count,
        hits,
        pipeline: kind === 1 ? any : closest,
        bindGroup: device.createBindGroup({
          layout,
          entries: [
            { binding: 0, resource: { buffer: nodes } },
            { binding: 1, resource: { buffer: triangles } },
            { binding: 2, resource: { buffer: rays } },
            { binding: 3, resource: { buffer: hits } },
            { binding: 4, resource: { buffer: run.uniform } },
          ],
        }),
      });
    });

    if (context.timestamps) {
      run.querySet = device.createQuerySet({ type: 'timestamp', count: 2 });
      run.resolveBuffer = device.createBuffer({ size: 16, usage: QUERY_RESOLVE | COPY_SRC });
      run.timeBuffer = device.createBuffer({ size: 16, usage: MAP_READ | COPY_DST });
      run.buffers.push(run.resolveBuffer, run.timeBuffer);
    }
    return run;
  }

  /** How many rays set `kind` holds. */
  rayCount(kind: number): number {
    return this.sets[kind].count;
  }

  /** The most rays one submission may trace on this device. */
  private maxSlice(kind: number): number {
    return Math.min(this.sets[kind].count, this.context.maxWorkgroupsPerDimension * WORKGROUP_RAYS);
  }

  /** One submission tracing rays `[first, first + count)`; answers its GPU time in ms. */
  private async dispatch(kind: number, first: number, count: number): Promise<number> {
    const set = this.sets[kind];
    const device = this.context.device;
    device.queue.writeBuffer(this.uniform, 0, new Uint32Array([first, count, 0, 0]));
    const encoder = device.createCommandEncoder();
    const pass = encoder.beginComputePass(
      this.querySet
        ? { timestampWrites: { querySet: this.querySet, beginningOfPassWriteIndex: 0, endOfPassWriteIndex: 1 } }
        : {},
    );
    pass.setPipeline(set.pipeline);
    pass.setBindGroup(0, set.bindGroup);
    pass.dispatchWorkgroups(Math.ceil(count / WORKGROUP_RAYS));
    pass.end();
    if (this.querySet) {
      encoder.resolveQuerySet(this.querySet, 0, 2, this.resolveBuffer, 0);
      encoder.copyBufferToBuffer(this.resolveBuffer, 0, this.timeBuffer, 0, 16);
    }
    const before = performance.now();
    device.queue.submit([encoder.finish()]);
    await device.queue.onSubmittedWorkDone();
    const wallMs = performance.now() - before;
    if (!this.querySet) return wallMs;

    await this.timeBuffer.mapAsync(MAP_MODE_READ);
    const times = new BigUint64Array(this.timeBuffer.getMappedRange().slice(0));
    this.timeBuffer.unmap();
    const passMs = Number(times[1] - times[0]) / 1e6;
    // Some drivers report zero or reversed timestamps for a pass; the wall time is still honest.
    if (passMs > 0) return passMs;
    this.wallTimed += 1;
    return wallMs;
  }

  /**
   * Traces set `kind` in slices, wrapping around it, until `targetMs` of GPU time have passed.
   * `shouldStop` is asked between submissions, so leaving the page stops the work.
   */
  async measure(kind: number, targetMs: number, shouldStop: () => boolean): Promise<Measured> {
    const set = this.sets[kind];
    const result: Measured = { rays: 0, gpuMs: 0, submissions: 0, wallTimedSubmissions: 0 };
    if (set.count === 0) return result;
    const sizer = new SliceSizer(this.maxSlice(kind));
    this.wallTimed = 0;
    let first = 0;
    while (result.gpuMs < targetMs && !shouldStop()) {
      const count = Math.min(sizer.current, set.count - first);
      const ms = await this.dispatch(kind, first, count);
      sizer.report(count, ms);
      result.rays += count;
      result.gpuMs += ms;
      result.submissions += 1;
      first = (first + count) % set.count;
    }
    result.wallTimedSubmissions = this.wallTimed;
    return result;
  }

  /** Traces all of set `kind` once, in slices, and answers its hit records in ray order. */
  async readHits(kind: number, shouldStop: () => boolean): Promise<Uint8Array> {
    const set = this.sets[kind];
    const device = this.context.device;
    const bytes = set.count * HIT_BYTES;
    if (bytes === 0) return new Uint8Array(0);
    const sizer = new SliceSizer(this.maxSlice(kind));
    let first = 0;
    while (first < set.count) {
      if (shouldStop()) throw new Error('stopped');
      const count = Math.min(sizer.current, set.count - first);
      sizer.report(count, await this.dispatch(kind, first, count));
      first += count;
    }
    const staging = device.createBuffer({ size: bytes, usage: MAP_READ | COPY_DST });
    const encoder = device.createCommandEncoder();
    encoder.copyBufferToBuffer(set.hits, 0, staging, 0, bytes);
    device.queue.submit([encoder.finish()]);
    await staging.mapAsync(MAP_MODE_READ);
    const copy = new Uint8Array(staging.getMappedRange().slice(0));
    staging.unmap();
    staging.destroy();
    return copy;
  }

  destroy() {
    for (const buffer of this.buffers) buffer.destroy();
    this.buffers = [];
    this.querySet?.destroy();
    this.querySet = null;
  }
}

/** What gpu_bench_scatter_prepare answers. */
export interface ScatterPrepared {
  shader: string;
  surfaces: Uint8Array;
  cases: Uint8Array;
  albedo: Uint8Array;
  caseCount: number;
}

/**
 * Runs the scattering check once: one dispatch over the fixed batch (about 6,000 small cases, far
 * under a slice's worth of work), and its result records in case order for the worker to compare.
 * Throws an Error with text for the page.
 */
export async function runScatterCheck(context: GpuContext, prepared: ScatterPrepared, shouldStop: () => boolean): Promise<Uint8Array> {
  const device = context.device;
  const module = device.createShaderModule({ code: prepared.shader });
  const info = await module.getCompilationInfo();
  const errors = info.messages.filter((m: any) => m.type === 'error');
  if (errors.length > 0) {
    throw new Error(`The GPU driver refused the scattering shader: ${errors.map((m: any) => m.message).join('; ')}. This is a bug in the page; please report it with the results text.`);
  }
  const pipeline = await device.createComputePipelineAsync({ layout: 'auto', compute: { module, entryPoint: 'scatter_check_main' } });

  const resultBytes = prepared.caseCount * SCATTER_RESULT_BYTES;
  const created: any[] = [];
  const upload = (bytes: Uint8Array, usage: number) => {
    const buffer = uploadBuffer(device, bytes, usage);
    created.push(buffer);
    return buffer;
  };
  try {
    const surfaces = upload(prepared.surfaces, STORAGE);
    const cases = upload(prepared.cases, STORAGE);
    const albedo = upload(prepared.albedo, STORAGE);
    const uniform = upload(new Uint8Array(new Uint32Array([0, prepared.caseCount, 0, 0]).buffer), UNIFORM);
    const results = device.createBuffer({ size: resultBytes, usage: STORAGE | COPY_SRC });
    const staging = device.createBuffer({ size: resultBytes, usage: MAP_READ | COPY_DST });
    created.push(results, staging);

    const group0 = device.createBindGroup({
      layout: pipeline.getBindGroupLayout(0),
      entries: [
        { binding: 0, resource: { buffer: surfaces } },
        { binding: 1, resource: { buffer: cases } },
        { binding: 2, resource: { buffer: results } },
        { binding: 3, resource: { buffer: uniform } },
      ],
    });
    const group1 = device.createBindGroup({
      layout: pipeline.getBindGroupLayout(1),
      entries: [{ binding: 0, resource: { buffer: albedo } }],
    });

    if (shouldStop()) throw new Error('stopped');
    const encoder = device.createCommandEncoder();
    const pass = encoder.beginComputePass();
    pass.setPipeline(pipeline);
    pass.setBindGroup(0, group0);
    pass.setBindGroup(1, group1);
    pass.dispatchWorkgroups(Math.ceil(prepared.caseCount / WORKGROUP_RAYS));
    pass.end();
    encoder.copyBufferToBuffer(results, 0, staging, 0, resultBytes);
    device.queue.submit([encoder.finish()]);
    await staging.mapAsync(MAP_MODE_READ);
    const copy = new Uint8Array(staging.getMappedRange().slice(0));
    staging.unmap();
    return copy;
  } finally {
    for (const buffer of created) buffer.destroy();
  }
}
