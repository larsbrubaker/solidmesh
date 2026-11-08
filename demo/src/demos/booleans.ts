// Booleans page: union, difference and intersection of two primitives through the library's
// boolean (demo/wasm/src/booleans.rs, `boolean_scene`), with the operands as a ghost. The
// "Messy input" menu turns operand B inside out and/or makes operand A self-intersecting; the
// winding rule and orientation repair then decide what counts as solid, and the note reports how
// the kernel classified each operand.

import { createDemoPage } from '../demo-shell.ts';
import { createCheckbox, createDropdown, createSlider } from '../controls.ts';
import { loadSetting, saveSetting } from '../settings.ts';

const ROUTE = 'booleans';

export const SHAPES = [
  { value: '0', text: 'Cube' },
  { value: '1', text: 'Sphere (revolved)' },
  { value: '2', text: 'Cylinder (extruded)' },
  { value: '3', text: 'Rounded box' },
  { value: '4', text: 'Torus (revolved)' },
  { value: '5', text: 'Icosahedron' },
  { value: '6', text: 'Star prism' },
];

export function init(container: HTMLElement): () => void {
  const page = createDemoPage(container, {
    route: ROUTE,
    title: 'Booleans',
    description: `Union, difference and intersection of two solids on the exact kernel
      (manifold-rust underneath). Switch on <em>Messy input</em> to feed it an inside-out or a
      self-intersecting operand: the robust path still returns a closed solid, and the winding
      rule or orientation repair decides what the bad operand means.`,
    ghostDefault: true,
    ghostLabel: 'operands',
  });

  const s = {
    a: loadSetting(ROUTE, 'a', '0'),
    b: loadSetting(ROUTE, 'b', '1'),
    op: loadSetting(ROUTE, 'op', '1'),
    x: loadSetting(ROUTE, 'x', 8),
    y: loadSetting(ROUTE, 'y', 6),
    z: loadSetting(ROUTE, 'z', 6),
    rot: loadSetting(ROUTE, 'rot', 20),
    messy: loadSetting(ROUTE, 'messy', '0'),
    nonzero: loadSetting(ROUTE, 'nonzero', false),
    repair: loadSetting(ROUTE, 'repair', false),
  };

  const run = () => page.request('boolean_scene', [
    Number(s.a), Number(s.b), Number(s.op), s.x, s.y, s.z, s.rot,
    Number(s.messy), s.nonzero, s.repair,
  ]);
  const set = <K extends keyof typeof s>(key: K) => (v: (typeof s)[K]) => {
    s[key] = v;
    saveSetting(ROUTE, key, v as string | number | boolean);
    run();
  };

  const c = page.controls;
  c.appendChild(createDropdown('Operation', [
    { value: '0', text: 'Union (A + B)' },
    { value: '1', text: 'Difference (A - B)' },
    { value: '2', text: 'Intersection (A & B)' },
  ], s.op, set('op')));
  c.appendChild(createDropdown('Operand A', SHAPES, s.a, set('a')));
  c.appendChild(createDropdown('Operand B', SHAPES, s.b, set('b')));
  c.appendChild(createSlider('B offset X (mm) ', -25, 25, s.x, 0.5, set('x')));
  c.appendChild(createSlider('B offset Y (mm) ', -25, 25, s.y, 0.5, set('y')));
  c.appendChild(createSlider('B offset Z (mm) ', -25, 25, s.z, 0.5, set('z')));
  c.appendChild(createSlider('B turn (deg) ', 0, 180, s.rot, 1, set('rot')));
  c.appendChild(createDropdown('Messy input', [
    { value: '0', text: 'Clean operands' },
    { value: '1', text: 'B inside out' },
    { value: '2', text: 'A self-intersecting' },
    { value: '3', text: 'Both' },
  ], s.messy, set('messy')));
  c.appendChild(createCheckbox('Nonzero winding (inside-out counts as solid)', s.nonzero, set('nonzero')));
  c.appendChild(createCheckbox('Repair orientation first', s.repair, set('repair')));

  run();
  return () => page.dispose();
}
