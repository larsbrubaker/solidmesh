// Outline to Solid page: an editable 2D profile (src/profile-editor.ts) extruded - square-ended or
// with rounded / chamfered bevels at either end - or revolved about the z axis, by the library's
// outline_to_mesh contours-in-mm calls (demo/wasm/src/outline.rs: `extrude_scene`,
// `revolve_scene`). Presets give a starting profile; every drag re-runs the operation.

import { createDemoPage } from '../demo-shell.ts';
import { createCheckbox, createDropdown, createSlider } from '../controls.ts';
import { loadSetting, saveSetting } from '../settings.ts';
import { ProfileEditor, flatten, type Contour } from '../profile-editor.ts';

const ROUTE = 'outline';

function circle(cx: number, cy: number, r: number, n: number, clockwise = false): Contour {
  const pts: Contour = [];
  for (let i = 0; i < n; i++) {
    const a = (clockwise ? -1 : 1) * (2 * Math.PI * i) / n;
    pts.push([+(cx + r * Math.cos(a)).toFixed(2), +(cy + r * Math.sin(a)).toFixed(2)]);
  }
  return pts;
}

function star(points: number, outer: number, inner: number): Contour {
  const pts: Contour = [];
  for (let i = 0; i < points * 2; i++) {
    const a = Math.PI / 2 + (Math.PI * i) / points;
    const r = i % 2 === 0 ? outer : inner;
    pts.push([+(r * Math.cos(a)).toFixed(2), +(r * Math.sin(a)).toFixed(2)]);
  }
  return pts;
}

function gear(teeth: number, outer: number, root: number): Contour {
  const pts: Contour = [];
  for (let i = 0; i < teeth; i++) {
    const a = (2 * Math.PI * i) / teeth;
    const w = Math.PI / teeth;
    for (const [da, r] of [[0, root], [w * 0.3, outer], [w * 0.9, outer], [w * 1.2, root]] as const) {
      pts.push([+(r * Math.cos(a + da)).toFixed(2), +(r * Math.sin(a + da)).toFixed(2)]);
    }
  }
  return pts;
}

const EXTRUDE_PRESETS: Record<string, () => Contour[]> = {
  star: () => [star(5, 20, 9)],
  plate: () => [
    [[-20, -12], [20, -12], [20, 12], [-20, 12]],
    circle(-10, 0, 5, 24, true),
    circle(10, 0, 5, 24, true),
  ],
  gear: () => [gear(12, 20, 16), circle(0, 0, 6, 24, true)],
  bracket: () => [[[-18, -18], [18, -18], [18, -8], [-8, -8], [-8, 18], [-18, 18]]],
  bowtie: () => [[[-18, -14], [18, 14], [18, -14], [-18, 14]]],
};

const REVOLVE_PRESETS: Record<string, () => Contour[]> = {
  goblet: () => [[[0, 0], [14, 0], [14, 2], [3, 4], [2.5, 18], [12, 26], [16, 40], [14, 40], [10, 28], [0, 24]]],
  ring: () => [[[16, 0], [28, 0], [28, 6], [22, 10], [16, 6]]],
  bead: () => [circle(22, 20, 8, 32)],
  knob: () => [[[0, 0], [20, 0], [20, 6], [12, 8], [10, 18], [16, 26], [0, 30]]],
};

export function init(container: HTMLElement): () => void {
  const page = createDemoPage(container, {
    route: ROUTE,
    title: 'Outline to Solid',
    description: `Draw a profile in mm and turn it into a closed solid: extrude it (with a rounded
      or chamfered bevel at either end) or revolve it about the z axis. Drag points, double-click
      an edge to add one, right-click a point to remove it. Holes, touching loops and even a
      self-crossing bowtie extrude to a closed solid of the area they fill.`,
  });

  const s = {
    mode: loadSetting(ROUTE, 'mode', 'extrude'),
    preset: loadSetting(ROUTE, 'preset', 'star'),
    height: loadSetting(ROUTE, 'height', 12),
    top: loadSetting(ROUTE, 'top', 3),
    bottom: loadSetting(ROUTE, 'bottom', 0),
    steps: loadSetting(ROUTE, 'steps', 6),
    round: loadSetting(ROUTE, 'round', true),
    rsteps: loadSetting(ROUTE, 'rsteps', 64),
    start: loadSetting(ROUTE, 'start', 0),
    end: loadSetting(ROUTE, 'end', 360),
  };

  const run = () => {
    const { coords, counts } = flatten(editor.getContours());
    if (s.mode === 'extrude') {
      page.request('extrude_scene', [coords, counts, s.height, s.top, s.bottom, s.steps, s.round]);
    } else {
      page.request('revolve_scene', [coords, counts, s.rsteps, s.start, s.end]);
    }
  };

  const editorHost = document.createElement('div');
  let editor = new ProfileEditor({ view: [-25, -25, 50], axis: false, onChange: run });
  const extrudeBox = document.createElement('div');
  const revolveBox = document.createElement('div');
  for (const box of [extrudeBox, revolveBox]) {
    box.style.display = 'flex';
    box.style.flexDirection = 'column';
    box.style.gap = '12px';
  }
  const presetHost = document.createElement('div');

  const set = <K extends keyof typeof s>(key: K) => (v: (typeof s)[K]) => {
    s[key] = v;
    saveSetting(ROUTE, key, v as string | number | boolean);
    run();
  };

  function buildEditor() {
    const revolve = s.mode === 'revolve';
    const presets = revolve ? REVOLVE_PRESETS : EXTRUDE_PRESETS;
    if (!(s.preset in presets)) s.preset = Object.keys(presets)[0];
    editor = new ProfileEditor({
      view: revolve ? [-4, -4, 48] : [-25, -25, 50],
      axis: revolve,
      onChange: run,
    });
    editor.setContours(presets[s.preset]());
    editorHost.replaceChildren(editor.element);
    presetHost.replaceChildren(createDropdown('Profile preset',
      Object.keys(presets).map(k => ({ value: k, text: k[0].toUpperCase() + k.slice(1) })),
      s.preset, v => {
        s.preset = v;
        saveSetting(ROUTE, 'preset', v);
        editor.setContours(presets[v]());
        run();
      }));
    extrudeBox.style.display = revolve ? 'none' : 'flex';
    revolveBox.style.display = revolve ? 'flex' : 'none';
    page.viewer.resetFrame();
  }

  const c = page.controls;
  c.appendChild(createDropdown('Operation', [
    { value: 'extrude', text: 'Extrude' },
    { value: 'revolve', text: 'Revolve about z' },
  ], s.mode, v => {
    s.mode = v;
    saveSetting(ROUTE, 'mode', v);
    buildEditor();
    run();
  }));
  c.appendChild(presetHost);
  c.appendChild(editorHost);
  const hint = document.createElement('div');
  hint.className = 'hint';
  hint.textContent = 'Drag points · double-click an edge to add · right-click to remove';
  c.appendChild(hint);

  extrudeBox.appendChild(createSlider('Height (mm) ', 1, 40, s.height, 0.5, set('height')));
  extrudeBox.appendChild(createSlider('Top bevel (mm) ', 0, 10, s.top, 0.25, set('top')));
  extrudeBox.appendChild(createSlider('Bottom bevel (mm) ', 0, 10, s.bottom, 0.25, set('bottom')));
  extrudeBox.appendChild(createSlider('Bevel steps (1 = chamfer) ', 1, 16, s.steps, 1, set('steps')));
  extrudeBox.appendChild(createCheckbox('Round joins at corners', s.round, set('round')));
  revolveBox.appendChild(createSlider('Steps around ', 3, 128, s.rsteps, 1, set('rsteps')));
  revolveBox.appendChild(createSlider('Start angle (deg) ', 0, 360, s.start, 1, set('start')));
  revolveBox.appendChild(createSlider('End angle (deg) ', 0, 360, s.end, 1, set('end')));
  c.appendChild(extrudeBox);
  c.appendChild(revolveBox);

  buildEditor();
  run();
  return () => page.dispose();
}
