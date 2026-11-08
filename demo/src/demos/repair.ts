// Mesh Repair page: a closed solid broken on purpose (holes, flipped faces, unwelded triangle
// soup) and mended by the library's Repair operation, auto or advanced (demo/wasm/src/repair.rs,
// `repair_scene`). Back faces draw in saffron, so the flipped patches and the holes show before
// the repair and are gone after it.

import { createDemoPage } from '../demo-shell.ts';
import { createButton, createCheckbox, createDropdown, createSlider } from '../controls.ts';
import { loadSetting, saveSetting } from '../settings.ts';

const ROUTE = 'repair';

export function init(container: HTMLElement): () => void {
  const page = createDemoPage(container, {
    route: ROUTE,
    title: 'Mesh Repair',
    description: `Real meshes arrive broken: holes, faces wound the wrong way, vertices that were
      never welded. Break a closed solid here, then let the repair weld it, orient its faces and
      fill its holes until it is a closed solid again. Saffron is a face seen from behind: an
      inside-out patch, or the inside showing through a hole.`,
  });

  const s = {
    shape: loadSetting(ROUTE, 'shape', '0'),
    holes: loadSetting(ROUTE, 'holes', 3),
    flips: loadSetting(ROUTE, 'flips', 40),
    soup: loadSetting(ROUTE, 'soup', true),
    seed: loadSetting(ROUTE, 'seed', 1),
    mode: loadSetting(ROUTE, 'mode', '2'),
    fill: loadSetting(ROUTE, 'fill', true),
    orient: loadSetting(ROUTE, 'orient', true),
    weldEdges: loadSetting(ROUTE, 'weldEdges', true),
  };
  const run = () => page.request('repair_scene', [
    Number(s.shape), s.holes, s.flips, s.soup, s.seed, Number(s.mode), s.fill, s.orient, s.weldEdges,
  ]);
  const set = <K extends keyof typeof s>(key: K) => (v: (typeof s)[K]) => {
    s[key] = v;
    saveSetting(ROUTE, key, v as string | number | boolean);
    advanced.style.display = s.mode === '2' ? 'flex' : 'none';
    run();
  };

  const c = page.controls;
  c.appendChild(createDropdown('Solid', [
    { value: '0', text: 'Sphere' },
    { value: '1', text: 'Rounded box' },
    { value: '2', text: 'Torus' },
  ], s.shape, set('shape')));
  c.appendChild(createSlider('Holes ', 0, 12, s.holes, 1, set('holes')));
  c.appendChild(createSlider('Flipped faces ', 0, 300, s.flips, 1, set('flips')));
  c.appendChild(createCheckbox('Unwelded (triangle soup)', s.soup, set('soup')));
  c.appendChild(createButton('Break it differently', () => set('seed')(s.seed + 1)));
  c.appendChild(createDropdown('Show', [
    { value: '0', text: 'The broken mesh' },
    { value: '1', text: 'Auto repair' },
    { value: '2', text: 'Advanced repair' },
  ], s.mode, set('mode')));
  const advanced = document.createElement('div');
  advanced.style.display = s.mode === '2' ? 'flex' : 'none';
  advanced.style.flexDirection = 'column';
  advanced.style.gap = '8px';
  advanced.appendChild(createCheckbox('Orient faces', s.orient, set('orient')));
  advanced.appendChild(createCheckbox('Weld edges (cracks, seams)', s.weldEdges, set('weldEdges')));
  advanced.appendChild(createCheckbox('Fill holes', s.fill, set('fill')));
  c.appendChild(advanced);

  run();
  return () => page.dispose();
}
