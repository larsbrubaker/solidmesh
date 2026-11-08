// Minkowski & Hull page: the Minkowski sum (dilation) and difference (erosion) of a solid by a
// tool, and the convex hull of a cloud of separate shells (demo/wasm/src/morph.rs:
// `minkowski_scene`, `hull_scene`). The solid or the hull's source is the ghost.

import { createDemoPage } from '../demo-shell.ts';
import { createButton, createDropdown, createSlider } from '../controls.ts';
import { loadSetting, saveSetting } from '../settings.ts';

const ROUTE = 'minkowski-hull';

export function init(container: HTMLElement): () => void {
  const page = createDemoPage(container, {
    route: ROUTE,
    title: 'Minkowski &amp; Hull',
    description: `The Minkowski sum sweeps a tool over every point of a solid (rounding it
      outward by the tool's shape); the difference keeps only where the tool still fits inside.
      The convex hull wraps any set of shells in the smallest convex solid.`,
    ghostDefault: true,
    ghostLabel: 'source',
  });

  const s = {
    mode: loadSetting(ROUTE, 'mode', 'sum'),
    solid: loadSetting(ROUTE, 'solid', '1'),
    tool: loadSetting(ROUTE, 'tool', '0'),
    size: loadSetting(ROUTE, 'size', 4),
    segments: loadSetting(ROUTE, 'segments', 24),
    source: loadSetting(ROUTE, 'source', '0'),
    count: loadSetting(ROUTE, 'count', 60),
    seed: loadSetting(ROUTE, 'seed', 1),
  };
  const run = () => {
    if (s.mode === 'hull') page.request('hull_scene', [Number(s.source), s.count, s.seed]);
    else page.request('minkowski_scene', [Number(s.solid), Number(s.tool), s.size, s.mode === 'diff', s.segments]);
  };
  const set = <K extends keyof typeof s>(key: K) => (v: (typeof s)[K]) => {
    s[key] = v;
    saveSetting(ROUTE, key, v as string | number);
    if (key === 'mode' || key === 'solid' || key === 'source') page.viewer.resetFrame();
    show();
    run();
  };

  const morphBox = document.createElement('div');
  const hullBox = document.createElement('div');
  for (const box of [morphBox, hullBox]) {
    box.style.display = 'flex';
    box.style.flexDirection = 'column';
    box.style.gap = '12px';
  }
  const show = () => {
    morphBox.style.display = s.mode === 'hull' ? 'none' : 'flex';
    hullBox.style.display = s.mode === 'hull' ? 'flex' : 'none';
  };

  const c = page.controls;
  c.appendChild(createDropdown('Operation', [
    { value: 'sum', text: 'Minkowski sum (dilate)' },
    { value: 'diff', text: 'Minkowski difference (erode)' },
    { value: 'hull', text: 'Convex hull' },
  ], s.mode, set('mode')));
  morphBox.appendChild(createDropdown('Solid', [
    { value: '0', text: 'Star prism' },
    { value: '1', text: 'L bracket' },
    { value: '2', text: 'Cube' },
    { value: '3', text: 'Hex nut' },
  ], s.solid, set('solid')));
  morphBox.appendChild(createDropdown('Tool', [
    { value: '0', text: 'Sphere' },
    { value: '1', text: 'Cube' },
    { value: '2', text: 'Octahedron' },
  ], s.tool, set('tool')));
  morphBox.appendChild(createSlider('Tool size (mm) ', 0.5, 10, s.size, 0.5, set('size')));
  morphBox.appendChild(createSlider('Ball segments ', 4, 32, s.segments, 2, set('segments')));
  hullBox.appendChild(createDropdown('Source', [
    { value: '0', text: 'Point cloud (tiny octahedra)' },
    { value: '1', text: 'Scattered spheres and cubes' },
    { value: '2', text: 'Star prism' },
  ], s.source, set('source')));
  hullBox.appendChild(createSlider('Count / star points ', 3, 300, s.count, 1, set('count')));
  hullBox.appendChild(createButton('New random seed', () => set('seed')(s.seed + 1)));
  c.appendChild(morphBox);
  c.appendChild(hullBox);

  show();
  run();
  return () => page.dispose();
}
