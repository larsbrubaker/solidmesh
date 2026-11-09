// GPU Ray Tracing page: measures how fast a WebGPU compute shader traces rays through the
// library's flat tree. The part-on-a-floor scene is built by the worker (demo/wasm/src/
// gpu_bench.rs), the WebGPU calls are src/gpu-bench.ts, and what is measured is the three ray
// sets of gpu_bench.rs, each traced by the GPU and by the same traversal in wasm on one thread.
//
// The run is on a button, not on arrival: it holds the GPU for seconds, and that should be asked
// for. The picture canvas has a fixed pixel size and `.image-canvas` keeps it out of the layout
// (styles/main.css), so its size can never feed back into the page and restart work. The page
// has no thumbnail or home card (src/routes.ts lists those): the thumbnail capture cannot run a
// GPU benchmark.

import { demoFrameMarkup } from '../demo-shell.ts';
import { createButton, createDropdown } from '../controls.ts';
import { SideWorker } from '../mesh-runner.ts';
import { loadSetting, saveSetting } from '../settings.ts';
import { type GpuContext, type PreparedScene, type ScatterPrepared, TraceRun, hasWebGpu, openGpu, runScatterCheck } from '../gpu-bench.ts';
import { type KindRow, type ScatterRow, formatRate, formatRatio, formatScatter, reportText } from '../gpu-bench-rules.ts';

const ROUTE = 'gpu-trace';

/** GPU time spent measuring each ray set (ms). */
const MEASURE_MS = 1500;
/** Time spent measuring the wasm reference on each ray set (ms). */
const REFERENCE_MS = 500;

/** Matches IMAGE_WIDTH and IMAGE_HEIGHT in demo/wasm/src/gpu_bench.rs. */
const PICTURE_WIDTH = 480;
const PICTURE_HEIGHT = 360;

const KIND_NAMES = [
  'Camera rays (coherent, closest hit)',
  'Shadow rays to a light (any hit)',
  'Random bounce rays (incoherent, closest hit)',
];

interface PreparedAnswer {
  shader: string;
  nodes: Uint8Array;
  triangles: Uint8Array;
  rays0: Uint8Array;
  rays1: Uint8Array;
  rays2: Uint8Array;
  triangleCount: number;
  nodeBytes: number;
  triangleBytes: number;
  depth: number;
  note: string;
  meshMs: number;
  treeMs: number;
}

export function init(container: HTMLElement): () => void {
  container.innerHTML = demoFrameMarkup(
    'GPU Ray Tracing',
    `How fast can your graphics card trace rays? This page builds a part on a floor, packs it into the
    library's flat tree and traces the same rays with a WebGPU compute shader and in WebAssembly on one
    CPU thread, then checks that they agree. Press Run; it uses your GPU for several seconds.`,
  );
  const area = container.querySelector('#viewer-container') as HTMLElement;
  const controls = container.querySelector('#controls') as HTMLElement;
  const busy = container.querySelector('.busy-badge') as HTMLElement;
  controls.classList.add('gpu-bench-column');

  const canvas = document.createElement('canvas');
  canvas.className = 'image-canvas';
  canvas.width = PICTURE_WIDTH;
  canvas.height = PICTURE_HEIGHT;
  canvas.style.objectFit = 'contain';
  area.insertBefore(canvas, busy);
  const context2d = canvas.getContext('2d')!;

  const settings = {
    part: loadSetting(ROUTE, 'part', '1'),
    detail: loadSetting(ROUTE, 'detail', '2'),
  };
  controls.appendChild(createDropdown('Scene size', [
    { value: '0', text: 'About 10,000 triangles' },
    { value: '1', text: 'About 30,000 triangles' },
    { value: '2', text: 'About 100,000 triangles' },
    { value: '3', text: 'About 300,000 triangles' },
    { value: '4', text: 'About 1,000,000 triangles' },
  ], settings.detail, v => { settings.detail = v; saveSetting(ROUTE, 'detail', v); }));
  controls.appendChild(createDropdown('Part', [
    { value: '1', text: 'Torus' },
    { value: '0', text: 'Box with a ball carved out' },
    { value: '2', text: 'Rounded box' },
    { value: '3', text: 'Smooth blend' },
  ], settings.part, v => { settings.part = v; saveSetting(ROUTE, 'part', v); }));

  const status = document.createElement('div');
  status.className = 'note-line';
  const error = document.createElement('div');
  error.className = 'error-line';
  const results = document.createElement('div');
  results.className = 'info-readout';
  const report = document.createElement('textarea');
  report.readOnly = true;
  report.rows = 9;
  report.style.width = '100%';
  report.style.fontSize = '12px';
  report.style.display = 'none';

  const worker = new SideWorker();
  let disposed = false;
  let lost = false;
  let gpu: GpuContext | null = null;
  let run: TraceRun | null = null;

  const stopped = () => disposed || lost;

  const onLost = (message: string) => {
    if (disposed) return;
    lost = true;
    gpu = null;
    run = null;
    busy.classList.remove('visible');
    runButton.disabled = false;
    status.textContent = '';
    error.textContent = `The graphics device stopped working (${message}), so the run ended. Press Run to try again; if this keeps happening, pick a smaller scene or close other pages using the GPU.`;
  };

  const benchmark = async () => {
    runButton.disabled = true;
    error.textContent = '';
    busy.classList.add('visible');
    try {
      status.textContent = 'Starting WebGPU...';
      if (!gpu) {
        gpu = await openGpu(onLost);
        lost = false;
      }
      run?.destroy();
      run = null;
      const context = gpu;

      status.textContent = 'Building the scene and its tree...';
      const answer = await worker.call<PreparedAnswer>('gpu_bench_prepare', [
        Number(settings.part), Number(settings.detail), context.maxBinding,
      ]);
      if (disposed) return;
      const scene: PreparedScene = {
        shader: answer.shader,
        nodes: answer.nodes,
        triangles: answer.triangles,
        rays: [answer.rays0, answer.rays1, answer.rays2],
      };
      status.textContent = 'Uploading the scene and compiling the shader...';
      const created = await TraceRun.create(context, scene);
      if (stopped()) { created.destroy(); return; }
      run = created;

      const rows: KindRow[] = [];
      let timing = context.timestamps ? 'GPU timestamp queries around each compute pass' : 'wall time around each submission (onSubmittedWorkDone); this browser has no timestamp queries, so queueing is counted too';
      const hits: Uint8Array[] = [];
      for (let kind = 0; kind < KIND_NAMES.length; kind++) {
        if (stopped()) return;
        status.textContent = `Measuring: ${KIND_NAMES[kind]}...`;
        const measured = await run.measure(kind, MEASURE_MS, stopped);
        if (stopped()) return;
        if (measured.wallTimedSubmissions > 0) {
          timing += `; ${measured.wallTimedSubmissions} submissions fell back to wall time (the GPU's timestamps were unusable)`;
        }

        status.textContent = `Timing the wasm reference: ${KIND_NAMES[kind]}...`;
        const reference = await worker.call<{ rays: number; ms: number }>('gpu_bench_reference', [kind, REFERENCE_MS]);
        if (stopped()) return;

        status.textContent = `Reading back the GPU's hits: ${KIND_NAMES[kind]}...`;
        const gpuHits = await run.readHits(kind, stopped);
        if (stopped()) return;
        hits.push(gpuHits);
        const agreement = await worker.call<{ compared: number; exact: number; ties: number; disagreed: number; examples: string }>(
          'gpu_bench_compare', [kind, gpuHits.slice(0)],
        );
        if (stopped()) return;
        rows.push({
          name: KIND_NAMES[kind],
          rays: measured.rays,
          gpuRaysPerSecond: measured.gpuMs > 0 ? measured.rays / (measured.gpuMs / 1000) : 0,
          gpuMs: measured.gpuMs,
          submissions: measured.submissions,
          wasmRaysPerSecond: reference.ms > 0 ? reference.rays / (reference.ms / 1000) : 0,
          compared: agreement.compared,
          exact: agreement.exact,
          ties: agreement.ties,
          disagreed: agreement.disagreed,
          examples: agreement.examples,
        });
      }

      status.textContent = 'Checking the scattering model on the GPU against the CPU...';
      const scatterPrepared = await worker.call<ScatterPrepared>('gpu_bench_scatter_prepare', []);
      if (stopped()) return;
      const scatterResults = await runScatterCheck(context, scatterPrepared, stopped);
      if (stopped()) return;
      const scatter = await worker.call<ScatterRow>('gpu_bench_scatter_compare', [scatterResults.slice(0)]);
      if (stopped()) return;

      status.textContent = 'Drawing the picture from the GPU\'s hits...';
      const picture = await worker.call<{ pixels: Uint8Array }>('gpu_bench_picture', [hits[0], hits[1]]);
      if (stopped()) return;
      const image = new ImageData(new Uint8ClampedArray(picture.pixels.buffer as ArrayBuffer), PICTURE_WIDTH, PICTURE_HEIGHT);
      context2d.putImageData(image, 0, 0);

      const inputs = {
        adapter: context.adapter,
        timing,
        triangles: answer.triangleCount,
        nodeBytes: answer.nodeBytes,
        triangleBytes: answer.triangleBytes,
        maxBinding: context.maxBinding,
        depth: answer.depth,
        rows,
        scatter,
      };
      const mib = (bytes: number) => `${(bytes / (1024 * 1024)).toFixed(1)} MiB`;
      results.innerHTML = `<div class="stats-grid">
        <span class="k">Adapter</span><span class="v">${context.adapter.replace(/</g, '&lt;')}</span>
        <span class="k">Triangles</span><span class="v">${answer.triangleCount.toLocaleString()}</span>
        <span class="k">Tree buffers</span><span class="v">${mib(answer.nodeBytes)} + ${mib(answer.triangleBytes)}</span>
        <span class="k">Binding limit</span><span class="v">${mib(context.maxBinding)}</span>
        <span class="k">GPU clock</span><span class="v">${context.timestamps ? 'timestamp queries' : 'submission wall time'}</span>
      </div>` + rows.map(row => `<div class="stats-grid" style="margin-top:8px">
        <span class="k">${row.name}</span><span class="v">${formatRate(row.gpuRaysPerSecond)}</span>
        <span class="k">wasm, 1 thread</span><span class="v">${formatRate(row.wasmRaysPerSecond)} (GPU ${formatRatio(row.gpuRaysPerSecond, row.wasmRaysPerSecond)})</span>
        <span class="k">Agreement</span><span class="v ${row.disagreed === 0 ? 'good' : 'bad'}">${row.exact.toLocaleString()} exact, ${row.ties.toLocaleString()} ties, ${row.disagreed.toLocaleString()} wrong of ${row.compared.toLocaleString()}</span>
      </div>`).join('') + `<div class="stats-grid" style="margin-top:8px">
        <span class="k">Scattering model</span><span class="v ${scatter.disagreed === 0 ? 'good' : 'bad'}">${formatScatter(scatter)}</span>
      </div>`;
      report.value = reportText(inputs);
      report.style.display = 'block';
      copyButton.style.display = 'block';
      const disagreements = rows.reduce((sum, row) => sum + row.disagreed, 0);
      if (disagreements > 0) {
        status.textContent = `Done, but the GPU and the reference disagree on ${disagreements.toLocaleString()} rays: copy the results and report this, because it is a bug.`;
      } else if (scatter.disagreed > 0) {
        status.textContent = `Done, but the scattering model disagrees on ${scatter.disagreed.toLocaleString()} cases: copy the results and report this, because it is a bug.`;
      } else {
        status.textContent = 'Done. The GPU agrees with the reference on every ray and scattering case checked. Copy the results to share them.';
      }
    } catch (e) {
      if (disposed) return;
      if (!lost) {
        status.textContent = '';
        error.textContent = (e as Error).message;
      }
    } finally {
      if (!disposed) {
        busy.classList.remove('visible');
        runButton.disabled = false;
      }
    }
  };

  const runButton = createButton('Run benchmark', () => { void benchmark(); });
  const copyButton = createButton('Copy results', () => {
    void navigator.clipboard?.writeText(report.value).then(
      () => { status.textContent = 'Results copied.'; },
      () => { report.select(); status.textContent = 'Could not reach the clipboard: the text is selected, so copy it by hand.'; },
    );
    if (!navigator.clipboard) {
      report.select();
      status.textContent = 'This page cannot reach the clipboard: the text is selected, so copy it by hand.';
    }
  });
  copyButton.style.display = 'none';

  controls.appendChild(runButton);
  controls.appendChild(status);
  controls.appendChild(error);
  controls.appendChild(results);
  controls.appendChild(copyButton);
  controls.appendChild(report);

  if (!hasWebGpu()) {
    runButton.disabled = true;
    error.textContent = 'This browser does not support WebGPU, so there is nothing to measure here. Try current Chrome or Edge.';
  }

  return () => {
    disposed = true;
    run?.destroy();
    gpu?.device.destroy();
    worker.dispose();
  };
}
