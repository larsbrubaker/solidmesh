// Ray Tracing page: a solid made by the library, traced in WebAssembly through the library's BVH
// and drawn on a 2D canvas (demo/wasm/src/ray_trace.rs). Dragging orbits, the wheel zooms.
//
// The page has its own worker (`SideWorker`), so the BVH of a million triangles builds off the
// UI thread and terminating the worker on leaving the page gives that memory back. One wasm call
// is in flight at a time, through the site's `Coalescer`: every control change and every result
// calls `kick()`, which asks for what the page needs right now, and a newer request replaces the
// waiting one. While the camera moves a pass renders a whole frame at low resolution, the
// resolution adapting so a frame stays near PREVIEW_TARGET_MS; once still for STILL_MS it renders
// full resolution in time-boxed passes, accumulating samples up to SAMPLE_CAP.
//
// The thumbnail capture (src/capture.ts) reads the canvas registered with `setCaptureCanvas`, and
// the page holds it off (`holdBusy`) until the first full-resolution image has CAPTURE_SAMPLES.

import { demoFrameMarkup } from '../demo-shell.ts';
import { Coalescer, SideWorker, holdBusy } from '../mesh-runner.ts';
import { setCaptureCanvas } from '../capture.ts';
import { createDropdown, createSlider } from '../controls.ts';
import { loadSetting, saveSetting } from '../settings.ts';

const ROUTE = 'ray-trace';

/** Samples per pixel the refinement stops at. */
const SAMPLE_CAP = 128;
/** A full-resolution pass stops at this budget (ms); a preview frame runs to its end. */
const PASS_BUDGET_MS = 45;
/** How long the camera must be still before refinement starts (ms). */
const STILL_MS = 140;
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
    `A solid made by the library, traced in WebAssembly through the library's ray tracing tree, one
    ray at a time onto a plain 2D canvas. Drag to orbit, scroll to zoom. Moving shows a fast low
    resolution image; when you stop it sharpens, sample by sample. Normals and Depth show the raw
    hit data, so you can see these are real intersections with the mesh.`,
  );
  const area = container.querySelector('#viewer-container') as HTMLElement;
  const controls = container.querySelector('#controls') as HTMLElement;
  const busy = container.querySelector('.busy-badge') as HTMLElement;

  const canvas = document.createElement('canvas');
  canvas.style.touchAction = 'none';
  canvas.style.cursor = 'grab';
  area.insertBefore(canvas, busy);
  const context = canvas.getContext('2d')!;
  setCaptureCanvas(canvas);

  const s = {
    part: loadSetting(ROUTE, 'part', '0'),
    detail: loadSetting(ROUTE, 'detail', '2'),
    mode: loadSetting(ROUTE, 'mode', '0'),
    reflect: loadSetting(ROUTE, 'reflect', 0.2),
  };

  const camera = { yaw: 0.85, pitch: 0.42, zoom: 1 };
  // The image the worker is asked for next: preview while moving, full resolution once still.
  let moving = true;
  let stillTimer: ReturnType<typeof setTimeout> | null = null;
  let previewScale = 0.22;
  let prepared: Prepared | null = null;
  let preparingKey = '';
  let lastPass: Pass | null = null;
  let raysTotal = 0;
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
    const full = fullSize();
    const size = moving
      ? { w: Math.max(32, Math.round(full.w * previewScale)), h: Math.max(32, Math.round(full.h * previewScale)) }
      : full;
    coalescer.request('ray_trace_pass', [
      camera.yaw, camera.pitch, camera.zoom, Number(s.mode), s.reflect, SEED, size.w, size.h,
      moving ? 0 : PASS_BUDGET_MS,
    ]);
  };

  const finishMoving = () => {
    stillTimer = null;
    moving = false;
    kick();
  };

  const nudge = () => {
    moving = true;
    if (stillTimer) clearTimeout(stillTimer);
    stillTimer = setTimeout(finishMoving, STILL_MS);
    kick();
  };

  const updateStats = () => {
    const pass = lastPass;
    const raysPerSecond = msTotal > 0 ? (raysTotal / msTotal) * 1000 : 0;
    stats.innerHTML = `<div class="stats-grid">
      <span class="k">Triangles</span><span class="v">${prepared ? prepared.triangles.toLocaleString() : '…'}</span>
      <span class="k">Build mesh</span><span class="v">${prepared ? `${fmt(prepared.mesh_ms, 0)} ms` : '…'}</span>
      <span class="k">Build BVH</span><span class="v">${prepared ? `${fmt(prepared.bvh_ms, 0)} ms` : '…'}</span>
      <span class="k">Samples</span><span class="v">${pass ? `${pass.samples} / ${SAMPLE_CAP}` : '…'}</span>
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
    raysTotal = 0;
    msTotal = 0;
    busy.classList.remove('visible');
    moving = true;
    if (stillTimer) clearTimeout(stillTimer);
    stillTimer = setTimeout(finishMoving, STILL_MS);
    updateStats();
    kick();
  };

  const onPass = (pass: Pass) => {
    draw(pass);
    lastPass = pass;
    raysTotal += pass.rays;
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

  const coalescer = new Coalescer<Result>(
    r => {
      error.textContent = '';
      if (typeof r === 'string') onPrepared(JSON.parse(r) as Prepared);
      else onPass(r);
    },
    e => {
      error.textContent = e.message;
      busy.classList.remove('visible');
      preparingKey = '';
      releaseCapture?.();
      releaseCapture = null;
    },
    () => {},
    (fn, args) => worker.call<Result>(fn, args),
  );

  const rerender = () => {
    raysTotal = 0;
    msTotal = 0;
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
    { value: '0', text: 'Shaded' },
    { value: '1', text: 'Normals (hit data)' },
    { value: '2', text: 'Depth (hit data)' },
  ], s.mode, v => { s.mode = v; saveSetting(ROUTE, 'mode', v); rerender(); }));
  controls.appendChild(createSlider('Reflectivity ', 0, 1, s.reflect, 0.05, v => {
    s.reflect = v;
    saveSetting(ROUTE, 'reflect', v);
    rerender();
  }));
  controls.appendChild(stats);
  controls.appendChild(note);
  controls.appendChild(error);
  updateStats();

  // Camera input: drag to orbit, wheel to zoom.
  let dragging = false;
  let lastX = 0;
  let lastY = 0;
  canvas.addEventListener('pointerdown', e => {
    dragging = true;
    lastX = e.clientX;
    lastY = e.clientY;
    canvas.setPointerCapture(e.pointerId);
    canvas.style.cursor = 'grabbing';
  });
  canvas.addEventListener('pointermove', e => {
    if (!dragging) return;
    camera.yaw -= (e.clientX - lastX) * 0.008;
    camera.pitch = Math.max(0.06, Math.min(1.5, camera.pitch + (e.clientY - lastY) * 0.008));
    lastX = e.clientX;
    lastY = e.clientY;
    nudge();
  });
  const stopDrag = () => {
    dragging = false;
    canvas.style.cursor = 'grab';
  };
  canvas.addEventListener('pointerup', stopDrag);
  canvas.addEventListener('pointercancel', stopDrag);
  canvas.addEventListener('wheel', e => {
    e.preventDefault();
    camera.zoom = Math.max(0.4, Math.min(8, camera.zoom * Math.exp(-e.deltaY * 0.0012)));
    nudge();
  }, { passive: false });

  const resize = new ResizeObserver(() => { if (prepared) nudge(); });
  resize.observe(area);

  kick();
  return () => {
    disposed = true;
    if (stillTimer) clearTimeout(stillTimer);
    resize.disconnect();
    coalescer.dispose();
    worker.dispose();
    setCaptureCanvas(null);
    releaseCapture?.();
  };
}
