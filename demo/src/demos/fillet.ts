// Fillet & Chamfer page: MatterCAD's fillet workflow on a part the library builds (or an uploaded
// STL). Hover the part to see what a click would pick, click to add it to the list, give each pick
// its own radius, choose a fillet or a chamfer, and the part is rebuilt with every pick rounded.
//
// Picking runs in wasm in a worker of its own (`SideWorker`, so a hover never waits behind a
// build): `fillet_prepare` readies the part and answers its feature edges and corners,
// `fillet_pick` answers what a ray (from Three.js, in the part's space) picks in the current mode
// as a geometric signature the page keeps, plus the segments to draw. The build is
// `fillet_scene` in the site's worker (demo/wasm/src/fillet.rs): one `bevel_part` call per
// distinct radius, each on the previous call's result, with a row per pick that the list shows -
// built, or refused in the kernel's own words with the size that would fit.

import { createDemoPage } from '../demo-shell.ts';
import { createButton, createDropdown, createSlider } from '../controls.ts';
import { loadSetting, saveSetting } from '../settings.ts';
import { callMesh, holdBusy, SideWorker } from '../mesh-runner.ts';

const ROUTE = 'fillet';
const UPLOAD = 5;
const GROUP_NUMBERS = 17;

const PARTS = [
  { value: '0', text: 'L-bracket' },
  { value: '1', text: 'Plate with pocket and holes' },
  { value: '2', text: 'Revolved hub' },
  { value: '3', text: 'Measure & Export part' },
  { value: '4', text: 'Box' },
  { value: '5', text: 'Uploaded STL' },
];

const MODES = [
  { mode: 0, text: 'Chain', hint: 'Click an edge: its whole tangent chain is picked.' },
  { mode: 1, text: 'Edge', hint: 'Click an edge: that straight edge alone is picked.' },
  { mode: 2, text: 'Face rim', hint: 'Click a face: the edges around it are picked.' },
  { mode: 3, text: 'Point', hint: 'Click near a sharp corner to round the corner itself.' },
];

const PICK_COLORS = [0x3a86ff, 0x8338ec, 0x06a77d, 0xff006e, 0x00b4d8, 0xfb8500, 0x6a4c93];
const REFUSED = 0xd62828;
const HOVER = 0xffb703;
const FEATURE = 0x14213d;

interface Pick {
  group: number[];
  segments: number[];
  point: number[] | null;
  label: string;
  size: number;
  color: number;
}

interface Row {
  built: boolean;
  edges: number;
  pass: number;
  text: string;
  fits: number | null;
  code: string | null;
}

interface PickAnswer {
  hit: boolean;
  group?: number[];
  segments?: number[];
  point?: number[] | null;
  label?: string;
}

/** The picks each part starts with: where to click (target, seen from target + from), how, how big. */
const STARTING_PICKS: Record<number, { target: number[]; from: number[]; mode: number; size: number }[]> = {
  0: [
    { target: [-16.9, 0, 8], from: [20, 0, 20], mode: 0, size: 4 },
    { target: [-17.1, 0, 40], from: [20, 0, 20], mode: 0, size: 2 },
    { target: [24.9, 0, 8], from: [20, 0, 20], mode: 0, size: 2 },
  ],
  1: [
    { target: [18.1, 0, 10], from: [15, 0, 15], mode: 0, size: 1.5 },
    { target: [31.1, 16, 10], from: [10, 1, 15], mode: 0, size: 0.8 },
    { target: [34.9, 0, 10], from: [15, 0, 15], mode: 0, size: 3 },
  ],
  2: [
    { target: [14.1, 0, 8], from: [15, 0, 15], mode: 0, size: 3 },
    { target: [29.9, 0, 8], from: [15, 0, 15], mode: 0, size: 2 },
    { target: [10, 0, 26], from: [0, 0, 30], mode: 2, size: 1 },
  ],
  3: [{ target: [10.1, 0, 6], from: [15, 0, 15], mode: 0, size: 2 }],
  4: [{ target: [0, 0, 20], from: [0, 0, 30], mode: 2, size: 3 }],
};

/** Which picks select the same edges (or corner), wherever on them the click landed. */
function signature(p: { segments: number[]; point: number[] | null }): string {
  if (p.point) return p.point.map(v => v.toFixed(4)).join(',');
  const keys: string[] = [];
  for (let i = 0; i + 5 < p.segments.length; i += 6) {
    const a = p.segments.slice(i, i + 3).map(v => v.toFixed(4)).join(',');
    const b = p.segments.slice(i + 3, i + 6).map(v => v.toFixed(4)).join(',');
    keys.push(a < b ? `${a}|${b}` : `${b}|${a}`);
  }
  return keys.sort().join(';');
}

function escapeHtml(s: string): string {
  return s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
}

export function init(container: HTMLElement): () => void {
  const page = createDemoPage(container, {
    route: ROUTE,
    title: 'Fillet &amp; Chamfer',
    description: `MatterCAD's fillet tool on a part the library builds: hover to see what a click
      picks - a tangent <b>chain</b>, one <b>edge</b>, a <b>face rim</b> or a <b>point</b> - click
      to add it, and give each pick its own radius. Each pick has one radius (a radius that varies
      along an edge is not offered); picks with different radii are rounded one radius after
      another by <code>bevel_part</code>, and a pick that does not fit is refused with the size that
      would.`,
  });
  const viewer = page.viewer;
  const side = new SideWorker();
  let disposed = false;

  const s = {
    part: loadSetting(ROUTE, 'part', 0) as number,
    mode: loadSetting(ROUTE, 'mode', 0) as number,
    round: loadSetting(ROUTE, 'round', true) as boolean,
    segments: loadSetting(ROUTE, 'segments', 6) as number,
    size: loadSetting(ROUTE, 'size', 2) as number,
  };
  if (s.part === UPLOAD) s.part = 0; // an upload does not outlive the page
  let picks: Pick[] = [];
  let rows: (Row | null)[] = [];
  let features: number[] = [];
  let corners: number[] = [];
  let scale = 50;
  let ready = false;
  let hover: PickAnswer | null = null;

  // ----- controls
  const c = page.controls;
  const partSelect = createDropdown('Part', PARTS, String(s.part), v => {
    if (Number(v) === UPLOAD) {
      fileInput.click();
      (partSelect.querySelector('select') as HTMLSelectElement).value = String(s.part);
      return;
    }
    loadPart(Number(v), true);
  });
  c.appendChild(partSelect);
  const fileInput = document.createElement('input');
  fileInput.type = 'file';
  fileInput.accept = '.stl';
  fileInput.style.display = 'none';
  fileInput.addEventListener('change', () => uploadFile());
  c.appendChild(fileInput);

  const modeRow = document.createElement('div');
  modeRow.className = 'control-group';
  modeRow.innerHTML = '<label>Pick mode</label><div class="segmented"></div><div class="hint-line"></div>';
  const segmented = modeRow.querySelector('.segmented') as HTMLElement;
  const hint = modeRow.querySelector('.hint-line') as HTMLElement;
  const modeButtons = MODES.map(m => {
    const b = document.createElement('button');
    b.textContent = m.text;
    b.addEventListener('click', () => setMode(m.mode));
    segmented.appendChild(b);
    return b;
  });
  c.appendChild(modeRow);

  c.appendChild(createDropdown('Operation', [
    { value: 'fillet', text: 'Fillet (round)' },
    { value: 'chamfer', text: 'Chamfer (flat)' },
  ], s.round ? 'fillet' : 'chamfer', v => {
    s.round = v === 'fillet';
    saveSetting(ROUTE, 'round', s.round);
    drawCorners();
    run();
  }));
  c.appendChild(createSlider('Segments per round ', 1, 16, s.segments, 1, v => {
    s.segments = v;
    saveSetting(ROUTE, 'segments', v);
    run();
  }));
  c.appendChild(createSlider('Radius of a new pick (mm) ', 0.25, 10, s.size, 0.25, v => {
    s.size = v;
    saveSetting(ROUTE, 'size', v);
  }));

  const listBox = document.createElement('div');
  listBox.className = 'pick-list';
  c.appendChild(listBox);
  c.appendChild(createButton('Clear picks', () => {
    picks = [];
    rows = [];
    afterPicksChanged();
  }));

  // ----- drawing
  function drawFeatures() {
    viewer.setTubes('features', features, FEATURE, scale * 0.0018, { opacity: 0.35 });
  }
  function drawCorners() {
    if (s.mode !== 3) {
      viewer.clearOverlay('corners');
      return;
    }
    const shown: number[] = [];
    for (let i = 0; i + 3 < corners.length; i += 4) {
      if (s.round && corners[i + 3] === 0) continue;
      shown.push(corners[i], corners[i + 1], corners[i + 2]);
    }
    viewer.setBalls('corners', shown, FEATURE, scale * 0.008);
  }
  function drawPicks() {
    picks.forEach((p, i) => {
      const color = rows[i] && !rows[i]!.built ? REFUSED : p.color;
      viewer.setTubes(`pick${i}`, p.segments, color, scale * 0.005, { onTop: true, opacity: 0.8 });
      viewer.setBalls(`pickball${i}`, p.point, color, scale * 0.014);
    });
    for (let i = picks.length; i < picks.length + 8; i++) {
      viewer.clearOverlay(`pick${i}`);
      viewer.clearOverlay(`pickball${i}`);
    }
  }
  function drawHover() {
    viewer.setTubes('hover', hover?.hit ? hover.segments ?? null : null, HOVER, scale * 0.006, { onTop: true });
    viewer.setBalls('hoverball', hover?.hit ? hover.point ?? null : null, HOVER, scale * 0.016);
  }

  function setMode(mode: number) {
    s.mode = mode;
    saveSetting(ROUTE, 'mode', mode);
    modeButtons.forEach((b, i) => b.classList.toggle('active', MODES[i].mode === mode));
    hint.textContent = MODES.find(m => m.mode === mode)!.hint;
    hover = null;
    drawHover();
    drawCorners();
  }

  // ----- the list
  function renderList() {
    if (picks.length === 0) {
      listBox.innerHTML = '<div class="hint-line">No picks yet: hover the part and click.</div>';
      return;
    }
    listBox.innerHTML = '';
    picks.forEach((p, i) => {
      const row = rows[i];
      const item = document.createElement('div');
      item.className = 'pick-row' + (row && !row.built ? ' refused' : '');
      const color = row && !row.built ? REFUSED : p.color;
      const status = !row ? '' : row.built
        ? `<span class="ok">built${picks.length > 1 ? `, call ${row.pass + 1}` : ''}</span>`
        : '<span class="bad">refused</span>';
      item.innerHTML = `<div class="pick-head">
          <span class="swatch" style="background:#${color.toString(16).padStart(6, '0')}"></span>
          <span class="pick-label">${i + 1}. ${escapeHtml(p.label)}</span>${status}
          <button class="pick-remove" title="Remove this pick">&times;</button>
        </div>`;
      const slider = createSlider(s.round ? 'Radius (mm) ' : 'Setback (mm) ', 0.25, 20, p.size, 0.25, v => {
        p.size = v;
        savePicks();
        run();
      });
      item.appendChild(slider);
      if (row && row.text) {
        const text = document.createElement('div');
        text.className = row.built ? 'pick-note' : 'pick-note bad';
        text.textContent = row.text;
        item.appendChild(text);
      }
      if (row && !row.built && row.fits && row.fits > 0) {
        const fits = row.fits;
        item.appendChild(createButton(`Use ${fits} mm`, () => {
          p.size = fits;
          savePicks();
          renderList();
          run();
        }));
      }
      (item.querySelector('.pick-remove') as HTMLButtonElement).addEventListener('click', () => {
        picks.splice(i, 1);
        rows.splice(i, 1);
        afterPicksChanged();
      });
      listBox.appendChild(item);
    });
  }

  function savePicks() {
    if (s.part === UPLOAD) return;
    saveSetting(ROUTE, `picks${s.part}`, JSON.stringify(picks));
  }

  function afterPicksChanged() {
    savePicks();
    renderList();
    drawPicks();
    run();
  }

  // ----- the build
  function run() {
    if (!ready) return;
    const numbers = new Float64Array(picks.length * (GROUP_NUMBERS + 1));
    picks.forEach((p, i) => {
      numbers.set(p.group, i * (GROUP_NUMBERS + 1));
      numbers[i * (GROUP_NUMBERS + 1) + GROUP_NUMBERS] = p.size;
    });
    page.request('fillet_scene', [s.part, numbers, s.round, s.segments]);
  }

  page.onResult = (r) => {
    try {
      const data = JSON.parse(r.mesh.data) as { passes: number; rows: Row[] };
      // a result for an older list (picks added or removed since) is replaced by the next one
      if (data.rows.length === picks.length) {
        rows = data.rows;
        renderList();
        drawPicks();
      }
    } catch { /* a result without rows */ }
  };

  // ----- loading a part
  async function loadPart(part: number, fresh: boolean) {
    const release = holdBusy();
    try {
      ready = false;
      s.part = part;
      if (part !== UPLOAD) saveSetting(ROUTE, 'part', part);
      picks = [];
      rows = [];
      hover = null;
      drawHover();
      drawPicks();
      const prepared = JSON.parse(await side.call<string>('fillet_prepare', [part])) as
        { features: number[]; corners: number[] };
      if (disposed) return;
      features = prepared.features;
      corners = prepared.corners;
      let min = [Infinity, Infinity, Infinity];
      let max = [-Infinity, -Infinity, -Infinity];
      for (let i = 0; i + 2 < features.length; i += 3) {
        for (let k = 0; k < 3; k++) {
          min[k] = Math.min(min[k], features[i + k]);
          max[k] = Math.max(max[k], features[i + k]);
        }
      }
      scale = features.length ? Math.hypot(max[0] - min[0], max[1] - min[1], max[2] - min[2]) : 50;
      drawFeatures();
      drawCorners();
      viewer.resetFrame();
      const saved = part === UPLOAD ? null : loadSetting<string | null>(ROUTE, `picks${part}`, null);
      if (saved && !fresh) {
        picks = JSON.parse(saved);
      } else if (STARTING_PICKS[part]) {
        for (const start of STARTING_PICKS[part]) {
          const origin = start.target.map((v, k) => v + start.from[k]);
          const dir = start.target.map((v, k) => v - origin[k]);
          const answer = JSON.parse(await side.call<string>('fillet_pick',
            [part, ...origin, ...dir, start.mode, 0.002, s.round])) as PickAnswer;
          if (disposed) return;
          if (answer.hit) addPick(answer, start.size, false);
        }
      }
      ready = true;
      renderList();
      drawPicks();
      run();
    } catch (e) {
      console.error('Fillet page: preparing the part failed', e);
    } finally {
      release();
    }
  }

  async function uploadFile() {
    const file = fileInput.files?.[0];
    fileInput.value = '';
    if (!file) return;
    const bytes = new Uint8Array(await file.arrayBuffer());
    try {
      await side.call<number>('fillet_upload', [bytes]);
      await callMesh<number>('fillet_upload', [bytes]);
      (partSelect.querySelector('select') as HTMLSelectElement).value = String(UPLOAD);
      await loadPart(UPLOAD, true);
    } catch (e) {
      listBox.innerHTML = `<div class="pick-note bad">${escapeHtml((e as Error).message)}</div>`;
    }
  }

  function addPick(answer: PickAnswer, size: number, rebuild = true) {
    const used = new Set(picks.map(p => p.color));
    const color = PICK_COLORS.find(c => !used.has(c)) ?? PICK_COLORS[picks.length % PICK_COLORS.length];
    picks.push({
      group: answer.group!,
      segments: answer.segments ?? [],
      point: answer.point ?? null,
      label: answer.label ?? 'pick',
      size,
      color,
    });
    rows.push(null);
    if (rebuild) afterPicksChanged();
  }

  // ----- hover and click
  const canvas = viewer.canvas;
  let hoverBusy = false;
  let hoverNext: { x: number; y: number } | null = null;
  let down: { x: number; y: number } | null = null;

  async function hoverAt(x: number, y: number) {
    if (!ready) return;
    if (hoverBusy) {
      hoverNext = { x, y };
      return;
    }
    hoverBusy = true;
    try {
      const ray = viewer.rayAt(x, y);
      const answer = JSON.parse(await side.call<string>('fillet_pick',
        [s.part, ...ray.origin, ...ray.direction, s.mode, ray.pixelSize, s.round])) as PickAnswer;
      if (disposed) return;
      hover = answer;
      drawHover();
      canvas.style.cursor = answer.hit ? 'pointer' : '';
    } catch (e) {
      console.warn("Fillet page: the pick failed", e);
      hover = null;
    } finally {
      hoverBusy = false;
      if (hoverNext) {
        const next = hoverNext;
        hoverNext = null;
        hoverAt(next.x, next.y);
      }
    }
  }

  const onMove = (e: PointerEvent) => {
    if (e.buttons !== 0) return;
    hoverAt(e.clientX, e.clientY);
  };
  const onDown = (e: PointerEvent) => { down = { x: e.clientX, y: e.clientY }; };
  const onUp = async (e: PointerEvent) => {
    if (!down || e.button !== 0) return;
    const moved = Math.hypot(e.clientX - down.x, e.clientY - down.y);
    down = null;
    if (moved > 4 || !ready) return;
    if (e.pointerType !== 'mouse') {
      // a tap has no hover before it: pick where it landed
      const ray = viewer.rayAt(e.clientX, e.clientY);
      hover = JSON.parse(await side.call<string>('fillet_pick',
        [s.part, ...ray.origin, ...ray.direction, s.mode, ray.pixelSize * 2, s.round])) as PickAnswer;
      if (disposed) return;
      drawHover();
    }
    if (!hover?.hit) return;
    const key = signature({ segments: hover.segments ?? [], point: hover.point ?? null });
    const same = picks.findIndex(p => signature(p) === key);
    if (same >= 0) {
      picks.splice(same, 1);
      rows.splice(same, 1);
      afterPicksChanged();
    } else {
      addPick(hover, s.size);
    }
  };
  const onLeave = () => {
    hover = null;
    drawHover();
  };
  canvas.addEventListener('pointermove', onMove);
  canvas.addEventListener('pointerdown', onDown);
  canvas.addEventListener('pointerup', onUp);
  canvas.addEventListener('pointerleave', onLeave);

  setMode(s.mode);
  renderList();
  loadPart(s.part, false);

  return () => {
    disposed = true;
    canvas.removeEventListener('pointermove', onMove);
    canvas.removeEventListener('pointerdown', onDown);
    canvas.removeEventListener('pointerup', onUp);
    canvas.removeEventListener('pointerleave', onLeave);
    side.dispose();
    page.dispose();
  };
}
