// The Ray Tracing page's settings and menus (src/demos/ray-trace.ts is the page): what the person
// picked, loaded from and saved to local storage (`loadRaySettings`), and the Part, Detail, View,
// Renderer, Material, Color, Light and Sun controls that change it (`appendRayControls`). The page
// says what a change means through `RayControlActions`: a new part or detail is a new scene, a
// Renderer pick re-arms a GPU that failed, and any other change renders afresh. The menus' words
// (`menuText`) also name the settings in the page's "Copy timings" report.
//
// The look controls show only for the path traced view; Custom's sliders and the Sun size only
// when they mean something.

import { createColor, createDropdown, createSlider } from './controls.ts';
import { loadSetting, saveSetting } from './settings.ts';

const ROUTE = 'ray-trace';

export function loadRaySettings() {
  return {
    part: loadSetting(ROUTE, 'part', '0'),
    detail: loadSetting(ROUTE, 'detail', '2'),
    renderer: loadSetting(ROUTE, 'renderer', 'auto'),
    mode: loadSetting(ROUTE, 'mode', '0'),
    material: loadSetting(ROUTE, 'material', '0'),
    color: loadSetting(ROUTE, 'color', '#5bc2b4'),
    roughness: loadSetting(ROUTE, 'roughness', 0.3),
    metallic: loadSetting(ROUTE, 'metallic', 0),
    transmission: loadSetting(ROUTE, 'transmission', 0),
    light: loadSetting(ROUTE, 'light', '0'),
    sun: loadSetting(ROUTE, 'sun', 5),
  };
}

export type RaySettings = ReturnType<typeof loadRaySettings>;

type Option = { value: string; text: string };

export const PART_OPTIONS: Option[] = [
  { value: '0', text: 'Boolean (box minus ball)' },
  { value: '1', text: 'Torus' },
  { value: '2', text: 'Rounded box' },
  { value: '3', text: 'Implicit blend' },
];
export const DETAIL_OPTIONS: Option[] = [
  { value: '0', text: 'About 10,000 triangles' },
  { value: '1', text: 'About 30,000 triangles' },
  { value: '2', text: 'About 100,000 triangles' },
  { value: '3', text: 'About 300,000 triangles' },
  { value: '4', text: 'About 1,000,000 triangles' },
];
export const VIEW_OPTIONS: Option[] = [
  { value: '0', text: 'Path traced' },
  { value: '1', text: 'Normals (smooth hit data)' },
  { value: '2', text: 'Depth (hit data)' },
];
export const RENDERER_OPTIONS: Option[] = [
  { value: 'auto', text: 'Auto' },
  { value: 'cpu', text: 'CPU' },
  { value: 'gpu', text: 'GPU' },
];
export const MATERIAL_OPTIONS: Option[] = [
  { value: '0', text: 'Matte plastic' },
  { value: '1', text: 'Glossy plastic' },
  { value: '2', text: 'Brushed metal' },
  { value: '3', text: 'Polished metal' },
  { value: '4', text: 'Glass' },
  { value: '5', text: 'Rubber' },
  { value: '6', text: 'Custom' },
];
export const LIGHT_OPTIONS: Option[] = [
  { value: '0', text: 'Studio (sky and soft sun)' },
  { value: '1', text: 'Overcast (no sun)' },
];

/** The words a menu shows for `value`, or the value itself if no option has it. */
export function menuText(options: Option[], value: string): string {
  return options.find(option => option.value === value)?.text ?? value;
}

export interface RayControlActions {
  /** A different part or detail: a scene the GPU has not refused yet. */
  newScene: () => void;
  /** Any other change to the picture. */
  rerender: () => void;
  /** The person picked a renderer, the one thing that re-arms a GPU that failed. */
  pickedRenderer: () => void;
}

/** Appends the page's controls to `controls`; each change updates `s`, saves it and tells `actions`. */
export function appendRayControls(controls: HTMLElement, s: RaySettings, actions: RayControlActions): void {
  const { newScene, rerender } = actions;
  controls.appendChild(createDropdown('Part', PART_OPTIONS, s.part,
    v => { s.part = v; saveSetting(ROUTE, 'part', v); newScene(); }));
  controls.appendChild(createDropdown('Detail', DETAIL_OPTIONS, s.detail,
    v => { s.detail = v; saveSetting(ROUTE, 'detail', v); newScene(); }));
  controls.appendChild(createDropdown('View', VIEW_OPTIONS, s.mode,
    v => { s.mode = v; saveSetting(ROUTE, 'mode', v); showLookControls(); rerender(); }));
  controls.appendChild(createDropdown('Renderer', RENDERER_OPTIONS, s.renderer, v => {
    s.renderer = v;
    saveSetting(ROUTE, 'renderer', v);
    actions.pickedRenderer();
    rerender();
  }));

  const lookControls = document.createElement('div');
  const materialMenu = createDropdown('Material', MATERIAL_OPTIONS, s.material,
    v => { s.material = v; saveSetting(ROUTE, 'material', v); showLookControls(); rerender(); });
  const colorPicker = createColor('Color', s.color, v => { s.color = v; saveSetting(ROUTE, 'color', v); rerender(); });
  const customSliders = document.createElement('div');
  const lightMenu = createDropdown('Light', LIGHT_OPTIONS, s.light,
    v => { s.light = v; saveSetting(ROUTE, 'light', v); showLookControls(); rerender(); });
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
  function showLookControls() {
    const pathTraced = s.mode === '0';
    lookControls.style.display = pathTraced ? 'contents' : 'none';
    customSliders.style.display = s.material === '6' ? 'contents' : 'none';
    sunSlider.style.display = s.light === '0' ? '' : 'none';
  }
  showLookControls();
  controls.appendChild(lookControls);
}
