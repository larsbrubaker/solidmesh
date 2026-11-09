// The Ray Tracing page's WebGPU work (src/demos/ray-trace.ts is the page): the library's path loop
// (PATH_WGSL, packed by demo/wasm/src/ray_trace_gpu.rs) run on the GPU one short slice of pixels
// at a time, adding into an accumulation buffer the page reads back to draw. The pipeline, bind
// groups and dispatch come from src/gpu-bench.ts, where the GPU Ray Tracing page's render check
// uses them too.
//
// A slice is sized from what the image has been proven and measured to cost (`SliceCosts`,
// src/ray-trace-rules.ts), and a still first runs a small pilot of itself (`pilot`) so that no
// full-size slice enters a stretch nothing has measured, because the operating system resets a GPU that a long dispatch holds
// up, and that comes to a lost device on phones and on desktops. Each submission waits for the one
// before it, so the slice uniform is written once per submission. `SampleWalk`
// (src/ray-trace-rules.ts) gives each pixel of each sample exactly once, since the accumulation
// buffer can't refuse a repeat. Accumulation is allocated for the largest image asked for and the
// preview's smaller images use the start of it, so a preview changing size costs no reallocation.
//
// `abort()` makes any wait in progress give up, so a lost device can't leave the page hanging on a
// promise that never settles. Types are `any` because Bun's types carry no WebGPU declarations.

import {
  ACCUMULATION_BYTES, CHANNEL_BYTES, COPY_DST, COPY_SRC, type GpuContext, MAP_MODE_READ, MAP_READ, STORAGE, UNIFORM,
  createPathPipeline, dispatchPath, pathBindGroups, uploadBuffer,
} from './gpu-bench.ts';
import { WORKGROUP_RAYS } from './gpu-bench-rules.ts';
import { CARRIED_COST_MARGIN, SampleWalk, SliceCosts } from './ray-trace-rules.ts';

/** Bytes of the `path_view` uniform (`VIEW_RECORD_BYTES` in the library's flat_tree/gpu_view.rs). */
const VIEW_BYTES = 272;
/** Bytes of the `path_slice` uniform (`PIXEL_SLICE_BYTES` in the library's flat_tree/gpu_scene.rs). */
const SLICE_BYTES = 16;

/** What ray_trace_gpu_prepare answers for a scene that fits. */
export interface GpuPack {
  fits: true;
  shader: string;
  entryPoint: string;
  nodes: Uint8Array;
  triangles: Uint8Array;
  faces: Uint8Array;
  srgb: Uint8Array;
  albedo: Uint8Array;
}

/** A cold pilot, a still that refines, or a preview while the camera moves. */
export type ImageKind = 'pilot' | 'still' | 'preview';

/** What one slice did. */
export interface SliceDone {
  /** Wall time of its submission (ms). */
  ms: number;
  /** Pixels it traced. */
  count: number;
  /** A whole sample was finished by this slice. */
  finishedSample: boolean;
}

export class GpuPathRenderer {
  private walk: SampleWalk | null = null;
  private costs: SliceCosts | null = null;
  /** What the current image is, and the last preview's costs, for the next preview. */
  private kind: ImageKind = 'still';
  private lastPreview: SliceCosts | null = null;
  private width = 0;
  private height = 0;
  private capacity = 0;
  private groups: any[] = [];
  private buffers: any[] = [];
  private shared!: { nodes: any; triangles: any; albedo: any; faces: any; srgb: any; channels: any };
  private viewUniform: any;
  private sliceUniform: any;
  private parts: any = null;
  private accumulation: any = null;
  private staging: any = null;
  private stop: (reason: Error) => void = () => {};
  private readonly stopped: Promise<never>;

  private constructor(private context: GpuContext, private pipeline: any) {
    this.stopped = new Promise<never>((_, reject) => {
      this.stop = reject;
    });
    // Nothing need be waiting on it when it rejects.
    this.stopped.catch(() => {});
  }

  /** Compiles the shader and uploads the buffers that never change; throws an Error for the page. */
  static async create(context: GpuContext, pack: GpuPack): Promise<GpuPathRenderer> {
    const device = context.device;
    const pipeline = await createPathPipeline(device, pack.shader, pack.entryPoint, 'path tracing shader');
    const renderer = new GpuPathRenderer(context, pipeline);
    const made = (buffer: any) => {
      renderer.buffers.push(buffer);
      return buffer;
    };
    renderer.shared = {
      nodes: made(uploadBuffer(device, pack.nodes, STORAGE)),
      triangles: made(uploadBuffer(device, pack.triangles, STORAGE)),
      albedo: made(uploadBuffer(device, pack.albedo, STORAGE)),
      faces: made(uploadBuffer(device, pack.faces, STORAGE)),
      srgb: made(uploadBuffer(device, pack.srgb, UNIFORM)),
      // A plain view never reads the channels, but the pipeline binds the buffer; one record does.
      channels: made(device.createBuffer({ size: CHANNEL_BYTES, usage: STORAGE })),
    };
    renderer.viewUniform = made(device.createBuffer({ size: VIEW_BYTES, usage: UNIFORM | COPY_DST }));
    renderer.sliceUniform = made(device.createBuffer({ size: SLICE_BYTES, usage: UNIFORM | COPY_DST }));
    return renderer;
  }

  /** Whole samples finished for the current image. */
  get samples(): number {
    return this.walk?.samples ?? 0;
  }

  /** Every sample of the current image is given. */
  get done(): boolean {
    return this.walk?.done ?? false;
  }

  /** Mid-way through a sample: a preview finishes its frame before the camera moves on. */
  get midSample(): boolean {
    return this.walk !== null && !this.walk.done && !this.walk.atSampleStart;
  }

  /**
   * Starts a new image: writes its view and parts and clears its accumulation. A pilot starts
   * cold; a still starts from the pilot just run; a preview from the last preview, with a margin.
   */
  setImage(view: Uint8Array, parts: Uint8Array, width: number, height: number, cap: number, kind: ImageKind): void {
    const device = this.context.device;
    const pixels = width * height;
    const earlier = this.costs;
    if (this.kind === 'preview' && earlier) this.lastPreview = earlier;
    const carry = kind === 'still' && this.kind === 'pilot' && earlier
      ? { from: earlier, margin: 1 }
      : kind === 'preview' && this.lastPreview
        ? { from: this.lastPreview, margin: CARRIED_COST_MARGIN }
        : undefined;
    this.kind = kind;
    this.ensureAccumulation(parts.byteLength, pixels);
    this.width = width;
    this.height = height;
    this.walk = new SampleWalk(pixels, cap);
    this.costs = new SliceCosts(
      pixels,
      Math.min(pixels, this.context.maxWorkgroupsPerDimension * WORKGROUP_RAYS),
      carry,
    );
    device.queue.writeBuffer(this.viewUniform, 0, view);
    device.queue.writeBuffer(this.parts, 0, parts);
    const encoder = device.createCommandEncoder();
    encoder.clearBuffer(this.accumulation, 0, pixels * ACCUMULATION_BYTES);
    device.queue.submit([encoder.finish()]);
  }

  /** Grows the accumulation and rebinds what binds it; a smaller image reuses what is there. */
  private ensureAccumulation(partBytes: number, pixels: number): void {
    const device = this.context.device;
    if (!this.parts) {
      this.parts = device.createBuffer({ size: partBytes, usage: STORAGE | COPY_DST });
      this.buffers.push(this.parts);
    }
    if (pixels <= this.capacity) return;
    this.accumulation?.destroy();
    this.staging?.destroy();
    this.buffers = this.buffers.filter(buffer => buffer !== this.accumulation && buffer !== this.staging);
    const bytes = pixels * ACCUMULATION_BYTES;
    this.accumulation = device.createBuffer({ size: bytes, usage: STORAGE | COPY_SRC | COPY_DST });
    this.staging = device.createBuffer({ size: bytes, usage: MAP_READ | COPY_DST });
    this.buffers.push(this.accumulation, this.staging);
    this.capacity = pixels;
    const { nodes, triangles, albedo, faces, srgb, channels } = this.shared;
    this.groups = pathBindGroups(device, this.pipeline, {
      nodes, triangles, albedo, faces, parts: this.parts, accumulation: this.accumulation,
      view: this.viewUniform, slice: this.sliceUniform, channels, srgb,
    });
  }

  /**
   * Runs a still's pilot: one sample of a small image of the same view and paths, not shown, so
   * the still's first full-size slices are sized by what every part of the picture costs.
   */
  async pilot(view: Uint8Array, parts: Uint8Array, width: number, height: number): Promise<void> {
    this.setImage(view, parts, width, height, 1, 'pilot');
    while (await this.step()) { /* every slice of the one sample */ }
  }

  /** Traces the next slice and waits for the GPU; null when every sample is given. */
  async step(): Promise<SliceDone | null> {
    if (!this.walk || !this.costs) return null;
    const slice = this.walk.next(this.costs.sizeAt(this.walk.position));
    if (!slice) return null;
    const device = this.context.device;
    device.queue.writeBuffer(this.sliceUniform, 0, new Uint32Array([slice.first, slice.count, slice.sample, 0]));
    const encoder = device.createCommandEncoder();
    dispatchPath(encoder, this.pipeline, this.groups, slice.count);
    const before = performance.now();
    device.queue.submit([encoder.finish()]);
    await Promise.race([device.queue.onSubmittedWorkDone(), this.stopped]);
    const ms = performance.now() - before;
    this.costs.report(slice.first, slice.count, ms);
    return { ms, count: slice.count, finishedSample: this.walk.atSampleStart };
  }

  /** The accumulation so far, a `vec4<f32>` per pixel of the current image. */
  async read(): Promise<Uint8Array> {
    const device = this.context.device;
    const bytes = this.width * this.height * ACCUMULATION_BYTES;
    const encoder = device.createCommandEncoder();
    encoder.copyBufferToBuffer(this.accumulation, 0, this.staging, 0, bytes);
    device.queue.submit([encoder.finish()]);
    await Promise.race([this.staging.mapAsync(MAP_MODE_READ, 0, bytes), this.stopped]);
    const copy = new Uint8Array(this.staging.getMappedRange(0, bytes).slice(0));
    this.staging.unmap();
    return copy;
  }

  /** Makes every wait in progress give up. */
  abort(reason: string): void {
    this.stop(new Error(reason));
  }

  destroy(): void {
    this.abort('the renderer was stopped');
    for (const buffer of this.buffers) buffer.destroy();
    this.buffers = [];
  }
}
