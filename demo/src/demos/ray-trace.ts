// Ray Tracing page: a solid made by the library, path traced in WebAssembly by the library's
// PathTracer and drawn on a 2D canvas (demo/wasm/src/ray_trace.rs). Dragging orbits, pinching or
// the wheel zooms (src/orbit-gesture.ts is the rule); the Material and Light menus are the
// library's presets.
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
// The thumbnail capture (src/capture.ts) reads the canvas registered with `setCaptureCanvas`, and
// the page holds it off (`holdBusy`) until the first full-resolution image has CAPTURE_SAMPLES.

import { demoFrameMarkup } from '../demo-shell.ts';
import { Coalescer, SideWorker, holdBusy } from '../mesh-runner.ts';
import { OrbitGesture, applyDelta } from '../orbit-gesture.ts';
import { type ImageSize, mayDraw, sizeChanged } from '../ray-trace-refine.ts';
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
    light as it bounces off the part and the floor, running in WebAssembly in your browser with no
    graphics card. Pick a material and see plastic catch a soft highlight, metal mirror the sky and
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

  const stats = document.createElement('div');
  stats.className = 'info-readout';
  const note = document.createElement('div');
  note.className = 'note-line';
  const error = document.createElement('div');
  error.className = 'error-line';

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
      if (preparingKey === preparedKey()) return;
      preparingKey = preparedKey();
      busy.textContent = 'Building the part and its ray tracing tree…';
      busy.classList.add('visible');
      coalescer.request('ray_trace_prepare', [Number(s.part), Number(s.detail)]);
      return;
    }
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
    coalescer.request('ray_trace_pass', currentArgs);
  };

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
    stats.innerHTML = `<div class="stats-grid">
      <span class="k">Triangles</span><span class="v">${prepared ? prepared.triangles.toLocaleString() : '…'}</span>
      <span class="k">Build mesh</span><span class="v">${prepared ? `${fmt(prepared.mesh_ms, 0)} ms` : '…'}</span>
      <span class="k">Build tree</span><span class="v">${prepared ? `${fmt(prepared.bvh_ms, 0)} ms` : '…'}</span>
      <span class="k">Samples</span><span class="v">${pass ? `${pass.samples} / ${SAMPLE_CAP}` : '…'}</span>
      <span class="k">Samples/s</span><span class="v">${samplesPerSecond ? fmt(samplesPerSecond, 2) : '…'}</span>
      <span class="k">Rays/s</span><span class="v">${raysPerSecond ? fmtRays(raysPerSecond) : '…'}</span>
      <span class="k">Image</span><span class="v">${pass ? `${pass.width} x ${pass.height}` : '…'}</span>
    </div>`;
    note.textContent = prepared ? prepared.note : '';
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
      error.textContent = e.message;
      busy.classList.remove('visible');
      preparingKey = '';
      releaseCapture?.();
      releaseCapture = null;
    },
    () => {},
    (fn, args) => worker.call<Result>(fn, args).then(result => ({ result, args })),
  );

  const rerender = () => {
    resetRates();
    nudge();
  };

  controls.appendChild(createDropdown('Part', [
    { value: '0', text: 'Boolean (box minus ball)' },
    { value: '1', text: 'Torus' },
    { value: '2', text: 'Rounded box' },
    { value: '3', text: 'Implicit blend' },
  ], s.part, v => { s.part = v; saveSetting(ROUTE, 'part', v); rerender(); }));
  controls.appendChild(createDropdown('Detail', [
    { value: '0', text: 'About 10,000 triangles' },
    { value: '1', text: 'About 30,000 triangles' },
    { value: '2', text: 'About 100,000 triangles' },
    { value: '3', text: 'About 300,000 triangles' },
    { value: '4', text: 'About 1,000,000 triangles' },
  ], s.detail, v => { s.detail = v; saveSetting(ROUTE, 'detail', v); rerender(); }));
  controls.appendChild(createDropdown('View', [
    { value: '0', text: 'Path traced' },
    { value: '1', text: 'Normals (smooth hit data)' },
    { value: '2', text: 'Depth (hit data)' },
  ], s.mode, v => { s.mode = v; saveSetting(ROUTE, 'mode', v); showLookControls(); rerender(); }));

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
