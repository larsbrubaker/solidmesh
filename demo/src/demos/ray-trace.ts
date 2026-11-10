// Ray Tracing page: a solid made by the library and path traced, either in WebAssembly by the
// library's PathTracer (demo/wasm/src/ray_trace.rs) or on the GPU by its WGSL path loop
// (demo/wasm/src/ray_trace_gpu.rs and src/ray-trace-gpu.ts), and drawn on a 2D canvas either way.
// The Renderer menu picks which (src/ray-trace-rules.ts has the rule and the words the page says
// about it). Dragging orbits, pinching or the wheel zooms (src/orbit-gesture.ts is the rule); the
// Material and Light menus are the library's presets.
//
// The page has its own worker (`SideWorker`), so the BVH of a million triangles builds off the
// UI thread and terminating the worker on leaving the page gives that memory back. One wasm call
// is in flight at a time, through the site's `Coalescer`: every control change and every result
// calls `kick()`, which asks for what the page needs right now, and a newer request replaces the
// waiting one. While the camera moves a pass renders a whole frame at low resolution with short
// paths, the resolution adapting so a frame stays near PREVIEW_TARGET_MS; once still for STILL_MS
// it renders full resolution in time-boxed passes, accumulating samples up to SAMPLE_CAP. A
// material or light change refines from scratch, on the part and tree the worker already holds.
//
// Once still, the page stays at one steady refinement (src/ray-trace-refine.ts). Each result
// carries the request it answers, and a still page draws only a result for the request it is
// refining, so a preview in flight when the finger lifted never replaces it. The canvas is
// positioned out of the layout (`.image-canvas`, styles/main.css): its pixel size must not feed
// the height of the area that decides its pixel size. A resize changes the image only once it
// settles, and only if the size in pixels really changed.
//
// The GPU route (`runGpu`) runs on this thread because that is where the WebGPU device is, and it
// asks the worker for the three things the worker holds: the pack once per part, the view and part
// records when the camera or look changes, and the decode of what the GPU accumulated, through the
// library's display code, so the picture is the CPU route's. One run at a time; a newer request
// replaces what it is working towards, between slices. A preview frame finishes before the camera
// moves on, so a slow GPU still shows a whole frame. A still first runs a small pilot of itself,
// not shown, so the size of its first full-size slices is measured rather than guessed
// (`SliceCosts` in src/ray-trace-rules.ts). A failure sends the picture to the CPU with a
// sentence over the picture's bottom edge (`.route-note`) saying what happened, and holds until
// the person picks a renderer again, so a lost device is never retried in a loop; the Normals and
// Depth views read hit data on the CPU either way.
//
// A wasm trap (out of memory packing a large part, say) kills the worker and everything it held,
// so the page forgets its prepared part and builds it again on the fresh worker, once
// (`TrapRecovery`), before going on with the CPU route.
//
// The thumbnail capture (src/capture.ts) reads the canvas registered with `setCaptureCanvas`, and
// the page holds it off (`holdBusy`) until the first full-resolution image has CAPTURE_SAMPLES.

import { demoFrameMarkup } from '../demo-shell.ts';
import { Coalescer, SideWorker, WorkerTrapped, holdBusy } from '../mesh-runner.ts';
import { OrbitGesture, applyDelta } from '../orbit-gesture.ts';
import { type ImageSize, mayDraw, sameArgs, sizeChanged } from '../ray-trace-refine.ts';
import {
  GPU_COMPILING_BADGE, GPU_PACKING_BADGE, type GpuFailure, GpuStageBadge, type RendererChoice, TrapRecovery,
  WORKER_TRAPPED_MESSAGE, chooseRoute, displayDue, passCountText, pilotSize,
} from '../ray-trace-rules.ts';
import { GpuPathRenderer, type GpuPack } from '../ray-trace-gpu.ts';
import { type GpuContext, NoGraphicsAdapter, ShaderRefused, hasWebGpu, openGpu } from '../gpu-bench.ts';
import { setCaptureCanvas } from '../capture.ts';
import { createColor, createDropdown, createSlider } from '../controls.ts';
import { loadSetting, saveSetting } from '../settings.ts';

const ROUTE = 'ray-trace';

/** Samples per pixel the refinement stops at. */
const SAMPLE_CAP = 128;
/** A full-resolution pass stops at this budget (ms); a preview frame runs to its end. */
const PASS_BUDGET_MS = 45;
/** How long the camera must be still before refinement starts (ms). */
const STILL_MS = 140;
/** How long the viewer's size must be steady before the image takes the new size (ms). */
const SIZE_SETTLE_MS = 140;
/** The time the preview resolution adapts toward (ms). */
const PREVIEW_TARGET_MS = 40;
/** Largest full-resolution image, so a large display never makes a sample take too long. */
const MAX_FULL_PIXELS = 1_000_000;
/** The thumbnail is taken once the first full-resolution image has this many samples. */
const CAPTURE_SAMPLES = 8;
const SEED = 1;

interface Pass {
  pixels: Uint8Array;
  width: number;
  height: number;
  samples: number;
  rays: number;
  pixelSamples: number;
  ms: number;
}

interface Prepared {
  part: number;
  detail: number;
  triangles: number;
  mesh_ms: number;
  bvh_ms: number;
  note: string;
}

type Result = Pass | string;

/** A worker's answer with the request that produced it. */
interface Answered {
  result: Result;
  args: unknown[];
}

/** A color control's `#rrggbb` as its three 0..=255 channels. */
function colorBytes(hex: string): [number, number, number] {
  const value = parseInt(hex.slice(1), 16);
  return [(value >> 16) & 255, (value >> 8) & 255, value & 255];
}

function fmt(n: number, digits = 1): string {
  return n.toLocaleString(undefined, { minimumFractionDigits: digits, maximumFractionDigits: digits });
}

function fmtRays(perSecond: number): string {
  if (perSecond >= 1e6) return `${fmt(perSecond / 1e6, 2)} M/s`;
  if (perSecond >= 1e3) return `${fmt(perSecond / 1e3, 0)} k/s`;
  return `${fmt(perSecond, 0)} /s`;
}

export function init(container: HTMLElement): () => void {
  container.innerHTML = demoFrameMarkup(
    'Ray Tracing',
    `A part made by the library, lit the way light really behaves: this is a path tracer, following
    light as it bounces off the part and the floor, running in your browser on your graphics card
    where it can and in WebAssembly where it can't. Pick a material and see plastic catch a soft highlight, metal mirror the sky and
    glass bend the floor behind it. Drag to orbit, pinch or scroll to zoom. Moving shows a small, fast image;
    when you stop it sharpens, sample by sample. Normals and Depth show the raw hit data, so you can
    see these are real intersections with the mesh.`,
  );
  const area = container.querySelector('#viewer-container') as HTMLElement;
  const controls = container.querySelector('#controls') as HTMLElement;
  const busy = container.querySelector('.busy-badge') as HTMLElement;

  const canvas = document.createElement('canvas');
  canvas.className = 'image-canvas';
  canvas.style.touchAction = 'none';
  canvas.style.cursor = 'grab';
  area.insertBefore(canvas, busy);
  const context = canvas.getContext('2d')!;
  setCaptureCanvas(canvas);

  const s = {
    part: loadSetting(ROUTE, 'part', '0'),
    detail: loadSetting(ROUTE, 'detail', '2'),
    renderer: loadSetting(ROUTE, 'renderer', 'auto'),
    mode: loadSetting(ROUTE, 'mode', '0'),
    material: loadSetting(ROUTE, 'material', '0'),
    color: loadSetting(ROUTE, 'color', '#5bc2b4'),
    roughness: loadSetting(ROUTE, 'roughness', 0.3),
    metallic: loadSetting(ROUTE, 'metallic', 0),
    transmission: loadSetting(ROUTE, 'transmission', 0),
    light: loadSetting(ROUTE, 'light', '0'),
    sun: loadSetting(ROUTE, 'sun', 5),
  };

  const camera = { yaw: 0.85, pitch: 0.42, zoom: 1 };
  // The image the worker is asked for next: preview while moving, full resolution once still.
  let moving = true;
  let stillTimer: ReturnType<typeof setTimeout> | null = null;
  let sizeTimer: ReturnType<typeof setTimeout> | null = null;
  // The full-resolution size the page is using, changed only when a resize has settled.
  let imageSize: ImageSize | null = null;
  // The arguments of the last pass asked for: the one result a still page wants to draw.
  let currentArgs: unknown[] | null = null;
  let previewScale = 0.22;
  let prepared: Prepared | null = null;
  let preparingKey = '';
  let lastPass: Pass | null = null;
  let raysTotal = 0;
  let pixelSamplesTotal = 0;
  let msTotal = 0;
  let disposed = false;
  let releaseCapture: (() => void) | null = holdBusy();

  // The GPU route. `gpuGeneration` changes when a run must stop; a run that wakes to find it
  // changed does nothing more, whatever it was waiting on.
  let gpu: GpuContext | null = null;
  let gpuRenderer: GpuPathRenderer | null = null;
  let gpuFailure: GpuFailure | null = null;
  let gpuPackedKey = '';
  let gpuWanted: unknown[] | null = null;
  let gpuRendering: unknown[] | null = null;
  let gpuGeneration = 0;
  let gpuRunning = false;
  let lastGpuDisplay = 0;
  let lastGpuDisplayCost = 0;
  const gpuStage = new GpuStageBadge({
    show: text => {
      busy.textContent = text;
      busy.classList.add('visible');
    },
    hide: () => busy.classList.remove('visible'),
  });
  const trapRecovery = new TrapRecovery();

  const stats = document.createElement('div');
  stats.className = 'info-readout';
  const note = document.createElement('div');
  note.className = 'note-line';
  const error = document.createElement('div');
  error.className = 'error-line';
  // Why the picture fell back to the CPU, over the picture's bottom edge: on a phone the controls
  // and the readout sit below the picture, out of sight, and this is the one thing to see.
  const routeNote = document.createElement('div');
  routeNote.className = 'route-note';
  area.appendChild(routeNote);
  const pill = document.createElement('div');
  pill.className = 'pass-pill';
  area.appendChild(pill);

  const worker = new SideWorker();

  const fullSize = () => {
    const scale = Math.min(window.devicePixelRatio || 1, 1.5);
    let w = Math.max(16, Math.round(area.clientWidth * scale));
    let h = Math.max(16, Math.round(area.clientHeight * scale));
    if (w * h > MAX_FULL_PIXELS) {
      const down = Math.sqrt(MAX_FULL_PIXELS / (w * h));
      w = Math.round(w * down);
      h = Math.round(h * down);
    }
    return { w, h };
  };

  const preparedKey = () => `${s.part}|${s.detail}`;

  /** Asks the worker for what the page needs now; a newer request replaces a waiting one. */
  const kick = () => {
    if (disposed) return;
    if (!prepared || `${prepared.part}|${prepared.detail}` !== preparedKey()) {
      // The part's own badge goes up next, or is already up.
      gpuStage.refresh(false);
      if (preparingKey === preparedKey() || trapRecovery.stopped) return;
      preparingKey = preparedKey();
      busy.textContent = 'Building the part and its ray tracing tree…';
      busy.classList.add('visible');
      coalescer.request('ray_trace_prepare', [Number(s.part), Number(s.detail)]);
      return;
    }
    gpuStage.refresh(usingGpu());
    imageSize ??= fullSize();
    const full = imageSize;
    const size = moving
      ? { w: Math.max(32, Math.round(full.w * previewScale)), h: Math.max(32, Math.round(full.h * previewScale)) }
      : full;
    const color = colorBytes(s.color);
    currentArgs = [
      camera.yaw, camera.pitch, camera.zoom, Number(s.mode), Number(s.material),
      color[0], color[1], color[2], s.roughness, s.metallic, s.transmission,
      Number(s.light), s.sun, SEED, size.w, size.h, moving,
      moving ? 0 : PASS_BUDGET_MS,
    ];
    if (usingGpu()) {
      gpuWanted = currentArgs;
      startGpu();
      return;
    }
    // A CPU pass makes whatever the GPU route was rendering stale.
    gpuRendering = null;
    coalescer.request('ray_trace_pass', currentArgs);
  };

  const route = () => chooseRoute(s.renderer as RendererChoice, hasWebGpu(), gpuFailure);
  /** The GPU route draws only the path traced view: Normals and Depth read hits on the CPU. */
  const usingGpu = () => s.mode === '0' && route().route === 'gpu';

  const finishMoving = () => {
    stillTimer = null;
    moving = false;
    resetRates();
    kick();
  };

  const nudge = () => {
    moving = true;
    if (stillTimer) clearTimeout(stillTimer);
    stillTimer = setTimeout(finishMoving, STILL_MS);
    kick();
  };

  const resetRates = () => {
    raysTotal = 0;
    pixelSamplesTotal = 0;
    msTotal = 0;
  };

  const updateStats = () => {
    const pass = lastPass;
    const raysPerSecond = msTotal > 0 ? (raysTotal / msTotal) * 1000 : 0;
    // A sample is one pass over every pixel of the image, however the passes slice it.
    const samplesPerSecond = pass && msTotal > 0
      ? (pixelSamplesTotal / (pass.width * pass.height) / msTotal) * 1000
      : 0;
    const onGpu = usingGpu();
    // The shader's paths are not counted, so the GPU route shows no rays per second.
    const raysRow = onGpu ? '' : `<span class="k">Rays/s</span><span class="v">${raysPerSecond ? fmtRays(raysPerSecond) : '…'}</span>`;
    stats.innerHTML = `<div class="stats-grid">
      <span class="k">Triangles</span><span class="v">${prepared ? prepared.triangles.toLocaleString() : '…'}</span>
      <span class="k">Build mesh</span><span class="v">${prepared ? `${fmt(prepared.mesh_ms, 0)} ms` : '…'}</span>
      <span class="k">Build tree</span><span class="v">${prepared ? `${fmt(prepared.bvh_ms, 0)} ms` : '…'}</span>
      <span class="k">Renderer</span><span class="v">${onGpu ? 'GPU' : 'CPU'}</span>
      <span class="k">Samples</span><span class="v">${pass ? `${pass.samples} / ${SAMPLE_CAP}` : '…'}</span>
      <span class="k">Samples/s</span><span class="v">${samplesPerSecond ? fmt(samplesPerSecond, 2) : '…'}</span>
      ${raysRow}
      <span class="k">Image</span><span class="v">${pass ? `${pass.width} x ${pass.height}` : '…'}</span>
    </div>`;
    note.textContent = prepared ? prepared.note : '';
    routeNote.textContent = route().message;
    routeNote.classList.toggle('visible', routeNote.textContent !== '');
    pill.textContent = passCountText(moving || !pass ? 0 : pass.samples, SAMPLE_CAP);
    pill.classList.toggle('visible', pill.textContent !== '');
  };

  const draw = (pass: Pass) => {
    if (canvas.width !== pass.width || canvas.height !== pass.height) {
      canvas.width = pass.width;
      canvas.height = pass.height;
    }
    const bytes = new Uint8ClampedArray(pass.pixels.buffer, pass.pixels.byteOffset, pass.pixels.length);
    context.putImageData(new ImageData(bytes, pass.width, pass.height), 0, 0);
  };

  const onPrepared = (result: Prepared) => {
    prepared = result;
    preparingKey = '';
    stopGpu();
    resetRates();
    busy.classList.remove('visible');
    moving = true;
    if (stillTimer) clearTimeout(stillTimer);
    stillTimer = setTimeout(finishMoving, STILL_MS);
    updateStats();
    kick();
  };

  const onPass = (pass: Pass, args: unknown[]) => {
    // A pass the page no longer wants: its replacement is already queued, so nothing is lost.
    if (!mayDraw(args, currentArgs, moving)) return;
    trapRecovery.succeeded();
    draw(pass);
    lastPass = pass;
    raysTotal += pass.rays;
    pixelSamplesTotal += pass.pixelSamples;
    msTotal += pass.ms;
    if (moving && pass.ms > 0) {
      const adjust = Math.sqrt(PREVIEW_TARGET_MS / pass.ms);
      previewScale = Math.min(0.6, Math.max(0.05, previewScale * Math.min(1.5, Math.max(0.6, adjust))));
    }
    if (!moving && pass.samples >= CAPTURE_SAMPLES && releaseCapture) {
      releaseCapture();
      releaseCapture = null;
    }
    updateStats();
    if (!moving && pass.samples < SAMPLE_CAP) kick();
  };

  const coalescer = new Coalescer<Answered>(
    ({ result, args }) => {
      error.textContent = '';
      if (typeof result === 'string') onPrepared(JSON.parse(result) as Prepared);
      else onPass(result, args);
    },
    e => {
      if (e instanceof WorkerTrapped) {
        workerTrapped(e, false);
        return;
      }
      error.textContent = e.message;
      busy.classList.remove('visible');
      preparingKey = '';
      releaseCapture?.();
      releaseCapture = null;
    },
    () => {},
    (fn, args) => worker.call<Result>(fn, args).then(result => ({ result, args })),
  );

  const dropRenderer = () => {
    gpuRenderer?.destroy();
    gpuRenderer = null;
    gpuPackedKey = '';
    gpuRendering = null;
  };

  /** Ends the run in progress and drops its renderer, so the next kick starts afresh. */
  const stopGpu = () => {
    gpuGeneration += 1;
    gpuRunning = false;
    gpuStage.set(null, false);
    dropRenderer();
  };

  const closeDevice = () => {
    const context = gpu;
    gpu = null;
    context?.device.destroy();
  };

  /** Hands the picture to the CPU and says what happened; the failure holds until `choose`. */
  const failGpu = (failure: GpuFailure) => {
    gpuFailure = failure;
    stopGpu();
    if (failure.kind !== 'too-large') closeDevice();
    updateStats();
    kick();
  };

  /**
   * The worker crashed and took the part with it: forget the part, so the next request builds it
   * on the fresh worker, and go on on the CPU if that was the GPU route's doing.
   */
  const workerTrapped = (e: WorkerTrapped, onGpu: boolean) => {
    if (onGpu) gpuFailure = { kind: 'trapped', detail: e.message };
    const next = trapRecovery.trapped(e.instance);
    stopGpu();
    if (next === 'seen') {
      updateStats();
      return;
    }
    prepared = null;
    preparingKey = '';
    if (next === 'stop') {
      busy.classList.remove('visible');
      error.textContent = WORKER_TRAPPED_MESSAGE;
      releaseCapture?.();
      releaseCapture = null;
      updateStats();
      return;
    }
    error.textContent = '';
    updateStats();
    kick();
  };

  const openDevice = async () => {
    const context: GpuContext = await openGpu(message => {
      if (gpu === context) failGpu({ kind: 'lost', detail: message });
    });
    return context;
  };

  /** What the request `args` is for the worker: 17 numbers, the preview flag as 0 or 1. */
  const traceNumbers = (args: unknown[]) => Float64Array.from(args.slice(0, 17).map(Number));

  const runGpu = async (generation: number) => {
    const live = () => !disposed && generation === gpuGeneration && usingGpu();
    if (!gpu) {
      const context = await openDevice();
      if (!live()) {
        context.device.destroy();
        return;
      }
      gpu = context;
      // A validation error drops the dispatch and leaves the picture black, with no other sign.
      context.device.addEventListener('uncapturederror', (e: any) => {
        if (gpu === context) failGpu({ kind: 'error', detail: String(e.error?.message ?? 'the GPU reported an error') });
      });
    }
    // Moving: one slice at a time, a whole frame then the next camera. Refining: slices until
    // the cap, drawn at the cadence `displayDue` sets.
    let frameMs = 0;
    let framePixels = 0;
    for (;;) {
      if (!live() || !prepared || `${prepared.part}|${prepared.detail}` !== preparedKey()) return;
      const want = gpuWanted;
      if (!want) return;
      if (!gpuRenderer || gpuPackedKey !== preparedKey()) {
        dropRenderer();
        gpuStage.set(GPU_PACKING_BADGE, usingGpu());
        const pack = await worker.call<GpuPack | { fits: false; reason: string }>(
          'ray_trace_gpu_prepare', [gpu.maxBinding, traceNumbers(want)]);
        if (!live()) return;
        if (!pack.fits) {
          failGpu({ kind: 'too-large', detail: pack.reason });
          return;
        }
        gpuStage.set(GPU_COMPILING_BADGE, usingGpu());
        const renderer = await GpuPathRenderer.create(gpu, pack);
        if (!live()) {
          renderer.destroy();
          return;
        }
        gpuRenderer = renderer;
        gpuPackedKey = preparedKey();
        gpuStage.set(null, usingGpu());
      }
      const renderer = gpuRenderer;
      const finishingPreview = gpuRendering !== null && gpuRendering[16] === true && renderer.midSample;
      if (!finishingPreview && (gpuRendering === null || !sameArgs(want, gpuRendering))) {
        type ViewAnswer = { fits: true; view: Uint8Array; parts: Uint8Array } | { fits: false; reason: string };
        if (want[16] !== true) {
          // A still first measures a small copy of itself, so no full-size slice is a guess.
          const pilot = pilotSize(want[14] as number, want[15] as number);
          const pilotArgs = [...want];
          pilotArgs[14] = pilot.w;
          pilotArgs[15] = pilot.h;
          const pilotView = await worker.call<ViewAnswer>('ray_trace_gpu_view', [traceNumbers(pilotArgs)]);
          if (!live()) return;
          if (!pilotView.fits) {
            failGpu({ kind: 'too-large', detail: pilotView.reason });
            return;
          }
          if (gpuWanted !== want) continue;
          // The pilot overwrites the accumulation, so nothing rendered before it is current.
          gpuRendering = null;
          await renderer.pilot(pilotView.view, pilotView.parts, pilot.w, pilot.h);
          if (!live()) return;
        }
        const answer = await worker.call<ViewAnswer>(
          'ray_trace_gpu_view', [traceNumbers(want)]);
        if (!live()) return;
        if (!answer.fits) {
          failGpu({ kind: 'too-large', detail: answer.reason });
          return;
        }
        // The camera moved on while the view was computed; start from the newest request.
        if (gpuWanted !== want) continue;
        const preview = want[16] === true;
        renderer.setImage(answer.view, answer.parts, want[14] as number, want[15] as number,
          preview ? 1 : SAMPLE_CAP, preview ? 'preview' : 'still');
        gpuRendering = want;
        frameMs = 0;
        framePixels = 0;
      }
      if (renderer.done) return;
      // What is being rendered: a preview that is finishing is not yet the newest request.
      const rendering = gpuRendering as unknown[];
      const width = rendering[14] as number;
      const height = rendering[15] as number;
      const slice = await renderer.step();
      if (!live()) return;
      if (!slice) return;
      frameMs += slice.ms;
      framePixels += slice.count;
      if (!displayDue(performance.now(), lastGpuDisplay, lastGpuDisplayCost, renderer.done, rendering[16] === true, slice.finishedSample)) continue;
      const started = performance.now();
      const bytes = await renderer.read();
      if (!live()) return;
      const decoded = await worker.call<{ pixels: Uint8Array }>('ray_trace_gpu_decode', [bytes, width, height], [bytes.buffer as ArrayBuffer]);
      if (!live()) return;
      lastGpuDisplay = performance.now();
      lastGpuDisplayCost = lastGpuDisplay - started;
      onPass({ pixels: decoded.pixels, width, height, samples: renderer.samples, rays: 0, pixelSamples: framePixels, ms: frameMs }, rendering);
      frameMs = 0;
      framePixels = 0;
    }
  };

  /** Starts a GPU run unless one is going, which sees the newer `gpuWanted` between slices. */
  const startGpu = () => {
    if (gpuRunning || disposed) return;
    gpuRunning = true;
    const generation = gpuGeneration;
    runGpu(generation)
      .catch((e: Error) => {
        // The worker is gone whichever run was waiting on it.
        if (e instanceof WorkerTrapped) {
          workerTrapped(e, true);
          return;
        }
        // Stopped on purpose, or already handled as a lost device.
        if (generation !== gpuGeneration) return;
        if (e instanceof NoGraphicsAdapter) failGpu({ kind: 'no-adapter', detail: e.message });
        else if (e instanceof ShaderRefused) {
          // The page's sentence is for the person; the driver's own words are for a remote debugger.
          console.warn('The graphics driver refused the path tracing shader:', e.driverText);
          failGpu({ kind: 'shader', detail: e.driverText });
        }
        else failGpu({ kind: 'error', detail: e.message });
      })
      .finally(() => {
        // Every way out of a run clears its badge; a stopped run's was cleared when it stopped.
        if (generation !== gpuGeneration) return;
        gpuRunning = false;
        gpuStage.set(null, false);
      });
  };

  const rerender = () => {
    resetRates();
    nudge();
  };

  /** A scene too large for the GPU is about the scene, so a different one gets a fresh try. */
  const newScene = () => {
    if (gpuFailure?.kind === 'too-large' || gpuFailure?.kind === 'trapped') gpuFailure = null;
    trapRecovery.reset();
    rerender();
  };

  controls.appendChild(createDropdown('Part', [
    { value: '0', text: 'Boolean (box minus ball)' },
    { value: '1', text: 'Torus' },
    { value: '2', text: 'Rounded box' },
    { value: '3', text: 'Implicit blend' },
  ], s.part, v => { s.part = v; saveSetting(ROUTE, 'part', v); newScene(); }));
  controls.appendChild(createDropdown('Detail', [
    { value: '0', text: 'About 10,000 triangles' },
    { value: '1', text: 'About 30,000 triangles' },
    { value: '2', text: 'About 100,000 triangles' },
    { value: '3', text: 'About 300,000 triangles' },
    { value: '4', text: 'About 1,000,000 triangles' },
  ], s.detail, v => { s.detail = v; saveSetting(ROUTE, 'detail', v); newScene(); }));
  controls.appendChild(createDropdown('View', [
    { value: '0', text: 'Path traced' },
    { value: '1', text: 'Normals (smooth hit data)' },
    { value: '2', text: 'Depth (hit data)' },
  ], s.mode, v => { s.mode = v; saveSetting(ROUTE, 'mode', v); showLookControls(); rerender(); }));
  controls.appendChild(createDropdown('Renderer', [
    { value: 'auto', text: 'Auto' },
    { value: 'cpu', text: 'CPU' },
    { value: 'gpu', text: 'GPU' },
  ], s.renderer, v => {
    s.renderer = v;
    saveSetting(ROUTE, 'renderer', v);
    // A person picking the renderer is the one thing that re-arms a GPU that failed.
    gpuFailure = null;
    rerender();
  }));

  // The look controls show only for the path traced view; Custom's sliders and the Sun size
  // only when they mean something.
  const lookControls = document.createElement('div');
  const materialMenu = createDropdown('Material', [
    { value: '0', text: 'Matte plastic' },
    { value: '1', text: 'Glossy plastic' },
    { value: '2', text: 'Brushed metal' },
    { value: '3', text: 'Polished metal' },
    { value: '4', text: 'Glass' },
    { value: '5', text: 'Rubber' },
    { value: '6', text: 'Custom' },
  ], s.material, v => { s.material = v; saveSetting(ROUTE, 'material', v); showLookControls(); rerender(); });
  const colorPicker = createColor('Color', s.color, v => { s.color = v; saveSetting(ROUTE, 'color', v); rerender(); });
  const customSliders = document.createElement('div');
  const lightMenu = createDropdown('Light', [
    { value: '0', text: 'Studio (sky and soft sun)' },
    { value: '1', text: 'Overcast (no sun)' },
  ], s.light, v => { s.light = v; saveSetting(ROUTE, 'light', v); showLookControls(); rerender(); });
  const sunSlider = createSlider('Sun size ', 1, 20, s.sun, 1, v => {
    s.sun = v;
    saveSetting(ROUTE, 'sun', v);
    rerender();
  });
  const slider = (label: string, key: 'roughness' | 'metallic' | 'transmission') =>
    createSlider(label, 0, 1, s[key], 0.05, v => { s[key] = v; saveSetting(ROUTE, key, v); rerender(); });
  customSliders.appendChild(slider('Roughness ', 'roughness'));
  customSliders.appendChild(slider('Metallic ', 'metallic'));
  customSliders.appendChild(slider('Transmission ', 'transmission'));
  lookControls.appendChild(materialMenu);
  lookControls.appendChild(colorPicker);
  lookControls.appendChild(customSliders);
  lookControls.appendChild(lightMenu);
  lookControls.appendChild(sunSlider);
  const showLookControls = () => {
    const pathTraced = s.mode === '0';
    lookControls.style.display = pathTraced ? 'contents' : 'none';
    customSliders.style.display = s.material === '6' ? 'contents' : 'none';
    sunSlider.style.display = s.light === '0' ? '' : 'none';
  };
  showLookControls();
  controls.appendChild(lookControls);
  controls.appendChild(stats);
  controls.appendChild(note);
  controls.appendChild(error);
  updateStats();

  // Camera input: drag to orbit, two fingers to orbit or pinch, wheel to zoom. Pointer reports are
  // collected and applied once per animation frame, so a two-finger gesture is one step.
  const gesture = new OrbitGesture();
  let gestureFrame: number | null = null;
  const applyGesture = () => {
    gestureFrame = null;
    const delta = gesture.take();
    if (delta.dx === 0 && delta.dy === 0 && delta.zoom === 1) return;
    Object.assign(camera, applyDelta(camera, delta));
    nudge();
  };
  const scheduleGesture = () => {
    if (gestureFrame === null) gestureFrame = requestAnimationFrame(applyGesture);
  };
  const showGrab = () => {
    canvas.style.cursor = gesture.fingers > 0 ? 'grabbing' : 'grab';
  };
  canvas.addEventListener('pointerdown', e => {
    gesture.down(e.pointerId, e.clientX, e.clientY);
    // a pointer the browser no longer tracks cannot be captured, which must not end the gesture
    try { canvas.setPointerCapture(e.pointerId); } catch { /* not an active pointer */ }
    scheduleGesture();
    showGrab();
  });
  canvas.addEventListener('pointermove', e => {
    if (!gesture.has(e.pointerId)) return;
    gesture.move(e.pointerId, e.clientX, e.clientY);
    scheduleGesture();
  });
  const endPointer = (e: PointerEvent) => {
    gesture.up(e.pointerId);
    scheduleGesture();
    showGrab();
  };
  canvas.addEventListener('pointerup', endPointer);
  canvas.addEventListener('pointercancel', endPointer);
  // Lifted outside the canvas with capture lost: the pointer must not stay "down" forever.
  canvas.addEventListener('lostpointercapture', endPointer);
  canvas.addEventListener('wheel', e => {
    e.preventDefault();
    camera.zoom = Math.max(0.4, Math.min(8, camera.zoom * Math.exp(-e.deltaY * 0.0012)));
    nudge();
  }, { passive: false });

  const settleSize = () => {
    sizeTimer = null;
    const measured = fullSize();
    if (!sizeChanged(imageSize, measured)) return;
    imageSize = measured;
    if (!prepared) return;
    resetRates();
    kick();
  };
  const resize = new ResizeObserver(() => {
    if (sizeTimer) clearTimeout(sizeTimer);
    sizeTimer = setTimeout(settleSize, SIZE_SETTLE_MS);
  });
  resize.observe(area);

  kick();
  return () => {
    disposed = true;
    stopGpu();
    closeDevice();
    if (stillTimer) clearTimeout(stillTimer);
    if (sizeTimer) clearTimeout(sizeTimer);
    if (gestureFrame !== null) cancelAnimationFrame(gestureFrame);
    resize.disconnect();
    coalescer.dispose();
    worker.dispose();
    setCaptureCanvas(null);
    releaseCapture?.();
  };
}
