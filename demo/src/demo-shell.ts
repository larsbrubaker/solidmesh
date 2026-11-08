// The frame every demo page shares: the header (title and what the page shows), the 3D viewer,
// the control column, and below the controls the readout every page reports - the operation's
// time in ms, triangle and vertex counts, volume and area, whether the result is a closed
// solid - plus the library's note, any error, and the STL download of the current result.
//
// A page builds its own controls into `page.controls` and calls `page.request(fn, args)` with a
// wasm export of demo/wasm (src/lib.rs); requests coalesce (src/mesh-runner.ts), so controls can
// call it on every input event.

import { ThreeViewer } from './three-viewer.ts';
import { Coalescer, lastResultStl, type RunResult } from './mesh-runner.ts';
import { createButton, createCheckbox } from './controls.ts';
import { loadSetting, saveSetting } from './settings.ts';

export interface PageSpec {
  route: string;
  title: string;
  /** One or two sentences of HTML under the title. */
  description: string;
  /** Show the operands / source ghost by default. */
  ghostDefault?: boolean;
  /** Label of the ghost in the legend. */
  ghostLabel?: string;
}

export interface DemoPage {
  viewer: ThreeViewer;
  controls: HTMLElement;
  request(fn: string, args: unknown[]): void;
  /** Called with each result after the shell has shown it. */
  onResult?: (r: RunResult) => void;
  dispose(): void;
}

function fmt(n: number, digits = 1): string {
  return n.toLocaleString(undefined, { minimumFractionDigits: digits, maximumFractionDigits: digits });
}

export function createDemoPage(container: HTMLElement, spec: PageSpec): DemoPage {
  container.innerHTML = `
    <div class="demo-page">
      <div class="demo-header">
        <h2>${spec.title}</h2>
        <p>${spec.description}</p>
      </div>
      <div class="demo-layout">
        <div class="demo-canvas-area" id="viewer-container"><div class="busy-badge">Computing&hellip;</div></div>
        <div class="demo-controls" id="controls"></div>
      </div>
    </div>`;
  const viewerEl = container.querySelector('#viewer-container') as HTMLElement;
  const controls = container.querySelector('#controls') as HTMLElement;
  const busy = container.querySelector('.busy-badge') as HTMLElement;
  const viewer = new ThreeViewer(viewerEl);

  // The readout and display toggles go after the page's own controls: appended once the page
  // has built them (next microtask).
  const tail = document.createElement('div');
  tail.style.display = 'flex';
  tail.style.flexDirection = 'column';
  tail.style.gap = '10px';
  const stats = document.createElement('div');
  stats.className = 'info-readout';
  const note = document.createElement('div');
  note.className = 'note-line';
  const error = document.createElement('div');
  error.className = 'error-line';

  let showGhost = loadSetting(spec.route, 'ghost', spec.ghostDefault ?? false);
  let lastResult: RunResult | null = null;

  const showGhostNow = () => {
    const m = lastResult?.mesh;
    viewer.setGhost(showGhost && m ? m.ghostPositions : null, showGhost && m ? m.ghostIndices : null);
  };

  if (spec.ghostLabel) {
    tail.appendChild(createCheckbox(`Show ${spec.ghostLabel}`, showGhost, v => {
      showGhost = v;
      saveSetting(spec.route, 'ghost', v);
      showGhostNow();
    }));
  }
  tail.appendChild(createCheckbox('Feature edges', loadSetting(spec.route, 'edges', true), v => {
    saveSetting(spec.route, 'edges', v);
    viewer.setEdges(v);
  }));
  tail.appendChild(createCheckbox('Wireframe', loadSetting(spec.route, 'wire', false), v => {
    saveSetting(spec.route, 'wire', v);
    viewer.setWireframe(v);
  }));
  viewer.setEdges(loadSetting(spec.route, 'edges', true));
  viewer.setWireframe(loadSetting(spec.route, 'wire', false));
  const legend = document.createElement('div');
  legend.className = 'legend';
  legend.innerHTML = `<span class="swatch" style="background:#2a9d8f"></span>outside
    <span class="swatch" style="background:#e9a23b"></span>inside (back faces)`;
  tail.appendChild(legend);
  tail.appendChild(stats);
  tail.appendChild(note);
  tail.appendChild(error);
  const stl = createButton('Download STL', async () => {
    try {
      const bytes = await lastResultStl();
      const url = URL.createObjectURL(new Blob([bytes], { type: 'model/stl' }));
      const a = document.createElement('a');
      a.href = url;
      a.download = `solidmesh-${spec.route}.stl`;
      a.click();
      setTimeout(() => URL.revokeObjectURL(url), 1000);
    } catch (e) {
      error.textContent = `STL export: ${(e as Error).message}`;
    }
  });
  tail.appendChild(stl);
  queueMicrotask(() => controls.appendChild(tail));

  const page: DemoPage = {
    viewer,
    controls,
    request: () => {},
    dispose: () => {},
  };

  const coalescer = new Coalescer(
    (r) => {
      lastResult = r;
      const m = r.mesh;
      viewer.setMesh(m.positions, m.indices);
      showGhostNow();
      const closed = m.closed
        ? '<span class="v good">yes</span>'
        : `<span class="v bad">no (${m.openEdges} open edges)</span>`;
      stats.innerHTML = `<div class="stats-grid">
        <span class="k">Time</span><span class="v">${fmt(r.ms)} ms</span>
        <span class="k">Triangles</span><span class="v">${m.numTri.toLocaleString()}</span>
        <span class="k">Vertices</span><span class="v">${m.numVert.toLocaleString()}</span>
        <span class="k">Volume</span><span class="v">${fmt(m.volume)} mm³</span>
        <span class="k">Area</span><span class="v">${fmt(m.surfaceArea)} mm²</span>
        <span class="k">Closed solid</span>${closed}
      </div>`;
      note.textContent = m.note;
      error.textContent = '';
      page.onResult?.(r);
    },
    (e) => {
      error.textContent = e.message;
    },
    (b) => busy.classList.toggle('visible', b),
  );

  page.request = (fn, args) => coalescer.request(fn, args);
  page.dispose = () => {
    coalescer.dispose();
    viewer.dispose();
  };
  return page;
}
