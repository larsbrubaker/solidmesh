// Measure & Export page: the showcase part (demo/wasm/src/showcase.rs, `showcase_scene`: a
// revolved hub, a smooth SDF fillet, a bevelled star boss, a bore and bolt holes, combined by
// booleans) with its measures - volume and area from the library, the bounding box, and the mass
// it would print at in PLA - and the library's binary STL of it to download.

import { createDemoPage } from '../demo-shell.ts';
import { createSlider } from '../controls.ts';
import { loadSetting, saveSetting } from '../settings.ts';

const ROUTE = 'measure-export';

/** PLA, g/cm³: the filament most parts made from an STL like this are printed in. */
const PLA_DENSITY = 1.24;

export function init(container: HTMLElement): () => void {
  const page = createDemoPage(container, {
    route: ROUTE,
    title: 'Measure &amp; Export',
    description: `One part built from most of the library: a revolved hub, a smooth implicit
      fillet where the neck meets the flange, a bevelled star boss, and a bore and bolt circle cut
      by one n-ary boolean. Its volume and area come from the mesh; <b>Download STL</b> writes it
      with the library's STL writer, ready for a slicer.`,
  });

  const s = {
    holes: loadSetting(ROUTE, 'holes', 6),
    flange: loadSetting(ROUTE, 'flange', 26),
    blend: loadSetting(ROUTE, 'blend', 3),
    detail: loadSetting(ROUTE, 'detail', 64),
  };
  const run = () => page.request('showcase_scene', [s.holes, s.flange, s.blend, s.detail]);
  const set = <K extends keyof typeof s>(key: K) => (v: number) => {
    s[key] = v;
    saveSetting(ROUTE, key, v);
    run();
  };

  const c = page.controls;
  c.appendChild(createSlider('Bolt holes ', 0, 12, s.holes, 1, set('holes')));
  c.appendChild(createSlider('Flange radius (mm) ', 18, 36, s.flange, 0.5, set('flange')));
  c.appendChild(createSlider('Fillet radius (mm) ', 0, 6, s.blend, 0.25, set('blend')));
  c.appendChild(createSlider('Segments per circle ', 16, 128, s.detail, 4, set('detail')));
  const extra = document.createElement('div');
  extra.className = 'info-readout';
  c.appendChild(extra);

  page.onResult = (r) => {
    const p = r.mesh.positions;
    const min = [Infinity, Infinity, Infinity];
    const max = [-Infinity, -Infinity, -Infinity];
    for (let i = 0; i < p.length; i += 3) {
      for (let k = 0; k < 3; k++) {
        min[k] = Math.min(min[k], p[i + k]);
        max[k] = Math.max(max[k], p[i + k]);
      }
    }
    const size = max.map((v, k) => (v - min[k]).toFixed(1)).join(' × ');
    const grams = (r.mesh.volume / 1000) * PLA_DENSITY;
    extra.innerHTML = `<div class="stats-grid">
      <span class="k">Bounding box</span><span class="v">${size} mm</span>
      <span class="k">Mass in PLA</span><span class="v">${grams.toFixed(1)} g</span>
    </div>`;
  };

  run();
  return () => page.dispose();
}
