// Implicit & SDF page: solids as signed distance fields, blended smoothly and meshed by the
// library's marching cubes or octree dual contouring (demo/wasm/src/implicit.rs,
// `implicit_scene`), plus the boolean's implicit processing mode on two mesh operands.

import { createDemoPage } from '../demo-shell.ts';
import { createDropdown, createSlider } from '../controls.ts';
import { loadSetting, saveSetting } from '../settings.ts';

const ROUTE = 'implicit';

export function init(container: HTMLElement): () => void {
  const page = createDemoPage(container, {
    route: ROUTE,
    title: 'Implicit &amp; SDF',
    description: `Shapes described by signed distance functions (sphere, box, cylinder) combined
      with a smooth blend whose radius you set, then meshed into a closed solid by marching cubes
      or dual contouring. The third scene is the boolean's implicit mode: two meshes resampled as
      distance grids and combined, an approach that never fails whatever the operands.`,
    ghostLabel: 'mesh operands',
  });

  const s = {
    scene: loadSetting(ROUTE, 'scene', '0'),
    blend: loadSetting(ROUTE, 'blend', 4),
    res: loadSetting(ROUTE, 'res', 96),
    mesher: loadSetting(ROUTE, 'mesher', '0'),
  };
  const run = () => page.request('implicit_scene', [Number(s.scene), s.blend, s.res, Number(s.mesher)]);
  const set = <K extends keyof typeof s>(key: K) => (v: (typeof s)[K]) => {
    s[key] = v;
    saveSetting(ROUTE, key, v as string | number);
    blendBox.style.display = s.scene === '2' ? 'none' : '';
    if (key === 'scene') page.viewer.resetFrame();
    run();
  };

  const c = page.controls;
  c.appendChild(createDropdown('Scene', [
    { value: '0', text: 'Blob: sphere + box + post, smooth union' },
    { value: '1', text: 'Drilled boss: smooth union and cuts' },
    { value: '2', text: 'Mesh operands: implicit boolean' },
  ], s.scene, set('scene')));
  c.appendChild(createDropdown('Mesher', [
    { value: '0', text: 'Marching cubes' },
    { value: '1', text: 'Dual contouring (octree)' },
  ], s.mesher, set('mesher')));
  const blendBox = createSlider('Blend radius (mm) ', 0, 10, s.blend, 0.25, set('blend'));
  blendBox.style.display = s.scene === '2' ? 'none' : '';
  c.appendChild(blendBox);
  c.appendChild(createSlider('Grid cells across ', 32, 192, s.res, 16, set('res')));

  run();
  return () => page.dispose();
}
