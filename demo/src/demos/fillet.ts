// Fillet & Chamfer page: MatterCAD's fillet workflow on a part the library builds (or an uploaded
// STL). Hover the part to see what a click would pick, click to add it to the list, choose a
// fillet or a chamfer and one radius for the operation, and the part is rebuilt with every pick
// rounded.
//
// Picking runs in wasm in a worker of its own (`SideWorker`, so a hover never waits behind a
// build): `fillet_prepare` readies the part and answers its feature edges and corners,
// `fillet_pick` answers what a ray (from Three.js, in the part's space) picks in the current mode
// as a geometric signature the page keeps, plus the segments to draw. The build is
// `fillet_scene` in the site's worker (demo/wasm/src/fillet.rs): every pick at the one radius in
// one `bevel_part` call, as MatterCAD's Fillet does, with a row per pick that the list shows -
// built, or refused in the kernel's own words with the size that would fit.
//
// The picks are drawn on the part only where they help: a pick's overlay sits on the edge as it
// was picked, which a round replaces, so once a pick has built it is hidden unless "Show picks"
// is on (then thin and see-through) or its row in the list is hovered, focused or tapped (then
// bold). A pick still waiting for its build, or refused, has no round to hide and stays drawn.
// The original part's feature edges (what can be picked) follow the shell's Feature edges box.

import { createDemoPage } from '../demo-shell.ts';
import { createButton, createCheckbox, createDropdown, createSlider } from '../controls.ts';
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

/**
 * A pick's overlay: thin (a see-through line `line` px wide whatever the zoom, so it never
 * covers a round) or bold (a tube, sizes as fractions of the part's).
 */
const THIN = { line: 2, ball: 0.006, opacity: 0.6 };
const BOLD = { tube: 0.005, ball: 0.014, opacity: 0.9 };

interface Pick {
  group: number[];
  segments: number[];
  point: number[] | null;
  label: string;
  color: number;
}

interface Row {
  built: boolean;
  edges: number;
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

/** The picks each part starts with: where to click (target, seen from target + from), and how. */
const STARTING_PICKS: Record<number, { target: number[]; from: number[]; mode: number }[]> = {
  0: [
    { target: [-16.9, 0, 8], from: [20, 0, 20], mode: 0 },
    { target: [-17.1, 0, 40], from: [20, 0, 20], mode: 0 },
    { target: [24.9, 0, 8], from: [20, 0, 20], mode: 0 },
  ],
  1: [
    { target: [18.1, 0, 10], from: [15, 0, 15], mode: 0 },
    { target: [31.1, 16, 10], from: [10, 1, 15], mode: 0 },
    { target: [34.9, 0, 10], from: [15, 0, 15], mode: 0 },
  ],
  2: [
    { target: [14.1, 0, 8], from: [15, 0, 15], mode: 0 },
    { target: [29.9, 0, 8], from: [15, 0, 15], mode: 0 },
    { target: [10, 0, 26], from: [0, 0, 30], mode: 2 },
  ],
  3: [{ target: [10.1, 0, 6], from: [15, 0, 15], mode: 0 }],
  4: [{ target: [0, 0, 20], from: [0, 0, 30], mode: 2 }],
};

/**
 * The radius each part starts with: one every starting pick builds at (the plate's pocket and
 * slot leave room for 1.5 mm, not 2), large enough to read as a round in the thumbnail.
 */
const STARTING_RADIUS: Record<number, number> = { 0: 3, 1: 1.5, 2: 2, 3: 2, 4: 3 };

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

/** Saved picks as this page writes them; anything else (an older page's fields) is dropped. */
function savedPicks(text: string): Pick[] | null {
  try {
    const parsed = JSON.parse(text);
    if (!Array.isArray(parsed)) return null;
    return parsed
      .filter(p => Array.isArray(p?.group) && p.group.length === GROUP_NUMBERS)
      .map(p => ({
        group: p.group,
        segments: Array.isArray(p.segments) ? p.segments : [],
        point: Array.isArray(p.point) ? p.point : null,
        label: String(p.label ?? 'pick'),
        color: Number(p.color ?? PICK_COLORS[0]),
      }));
  } catch {
    return null;
  }
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
      to add it, and set one radius for the operation. Every pick is rounded at that radius in one
      call to <code>bevel_part</code>, and a pick that does not fit is refused with the size that
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
    radius: STARTING_RADIUS[0],
    showPicks: loadSetting(ROUTE, 'showPicks', false) as boolean,
  };
  if (s.part === UPLOAD) s.part = 0; // an upload does not outlive the page
  let picks: Pick[] = [];
  let rows: (Row | null)[] = [];
  let features: number[] = [];
  let corners: number[] = [];
  let scale = 50;
  let ready = false;
  let hover: PickAnswer | null = null;
  // the pick drawn bold: its row is under the mouse, has keyboard focus, or was tapped
  const lit = { hovered: -1, focused: -1, tapped: -1 };

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
    radiusLabel();
    drawCorners();
    run();
  }));
  const radiusSlider = createSlider('Radius (mm) ', 0.25, 20, s.radius, 0.25, v => {
    s.radius = v;
    saveRadius();
    run();
  });
  c.appendChild(radiusSlider);
  c.appendChild(createSlider('Segments per round ', 1, 16, s.segments, 1, v => {
    s.segments = v;
    saveSetting(ROUTE, 'segments', v);
    run();
  }));

  const listBox = document.createElement('div');
  listBox.className = 'pick-list';
  c.appendChild(listBox);
  c.appendChild(createButton('Clear picks', () => {
    picks = [];
    rows = [];
    afterPicksChanged();
  }));

  page.addDisplayToggle(createCheckbox('Show picks', s.showPicks, v => {
    s.showPicks = v;
    saveSetting(ROUTE, 'showPicks', v);
    drawPicks();
  }));

  /** The radius slider's label (a chamfer's size is its setback) and value, to match `s`. */
  function radiusLabel() {
    const label = radiusSlider.querySelector('label') as HTMLLabelElement;
    (label.firstChild as Text).textContent = s.round ? 'Radius (mm) ' : 'Setback (mm) ';
    (radiusSlider.querySelector('input') as HTMLInputElement).value = String(s.radius);
    (radiusSlider.querySelector('.slider-value') as HTMLElement).textContent = String(s.radius);
  }
  function saveRadius() {
    if (s.part !== UPLOAD) saveSetting(ROUTE, `radius${s.part}`, s.radius);
  }

  // ----- drawing
  function drawFeatures() {
    viewer.setTubes('features', features, FEATURE, scale * 0.0018, { opacity: 0.35, withEdges: true });
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
      const row = rows[i];
      const refused = !!row && !row.built;
      const bold = i === lit.hovered || i === lit.focused || i === lit.tapped;
      // a built pick's edge is gone under its round: draw it only when asked to
      if (!bold && !refused && row && !s.showPicks) {
        viewer.clearOverlay(`pick${i}`);
        viewer.clearOverlay(`pickball${i}`);
        return;
      }
      const color = refused ? REFUSED : p.color;
      const opts = { onTop: true, opacity: bold ? BOLD.opacity : THIN.opacity };
      if (bold) viewer.setTubes(`pick${i}`, p.segments, color, scale * BOLD.tube, opts);
      else viewer.setLines(`pick${i}`, p.segments, color, THIN.line, opts);
      viewer.setBalls(`pickball${i}`, p.point, color, scale * (bold ? BOLD.ball : THIN.ball), opts);
    });
    for (let i = picks.length; i < picks.length + 8; i++) {
      viewer.clearOverlay(`pick${i}`);
      viewer.clearOverlay(`pickball${i}`);
    }
    listBox.querySelectorAll('.pick-row').forEach((item, i) => item.classList.toggle('lit',
      i === lit.hovered || i === lit.focused || i === lit.tapped));
  }
  function drawHover() {
    viewer.setTubes('hover', hover?.hit ? hover.segments ?? null : null, HOVER, scale * 0.006, { onTop: true });
    viewer.setBalls('hoverball', hover?.hit ? hover.point ?? null : null, HOVER, scale * 0.016);
  }
  function unlight() {
    lit.hovered = lit.focused = lit.tapped = -1;
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
        ? '<span class="ok">built</span>'
        : '<span class="bad">refused</span>';
      item.innerHTML = `<div class="pick-head">
          <span class="swatch" style="background:#${color.toString(16).padStart(6, '0')}"></span>
          <span class="pick-label">${i + 1}. ${escapeHtml(p.label)}</span>${status}
          <button class="pick-remove" title="Remove this pick">&times;</button>
        </div>`;
      if (row && row.text) {
        const text = document.createElement('div');
        text.className = row.built ? 'pick-note' : 'pick-note bad';
        text.textContent = row.text;
        item.appendChild(text);
      }
      if (row && !row.built && row.fits && row.fits > 0) {
        const fits = row.fits;
        item.appendChild(createButton(`Use ${fits} mm for all picks`, () => {
          s.radius = fits;
          saveRadius();
          radiusLabel();
          run();
        }));
      }
      (item.querySelector('.pick-remove') as HTMLButtonElement).addEventListener('click', () => {
        picks.splice(i, 1);
        rows.splice(i, 1);
        afterPicksChanged();
      });
      // which overlay is this row's: bold while the row is under the mouse, focused from the
      // keyboard, or tapped (a second tap, or a tap on another row, lets it go)
      item.addEventListener('pointerenter', e => {
        if (e.pointerType !== 'mouse') return;
        lit.hovered = i;
        drawPicks();
      });
      item.addEventListener('pointerleave', e => {
        if (e.pointerType !== 'mouse' || lit.hovered !== i) return;
        lit.hovered = -1;
        drawPicks();
      });
      item.addEventListener('pointerup', e => {
        if (e.pointerType === 'mouse' || (e.target as HTMLElement).closest('button')) return;
        lit.tapped = lit.tapped === i ? -1 : i;
        drawPicks();
      });
      item.addEventListener('focusin', e => {
        if (!(e.target as HTMLElement).matches(':focus-visible')) return;
        lit.focused = i;
        drawPicks();
      });
      item.addEventListener('focusout', () => {
        if (lit.focused !== i) return;
        lit.focused = -1;
        drawPicks();
      });
      listBox.appendChild(item);
    });
    const tip = document.createElement('div');
    tip.className = 'hint-line';
    tip.textContent = 'Hover or tap a pick to see its edges on the part.';
    listBox.appendChild(tip);
  }

  function savePicks() {
    if (s.part === UPLOAD) return;
    saveSetting(ROUTE, `picks${s.part}`, JSON.stringify(picks));
  }

  function afterPicksChanged() {
    unlight();
    savePicks();
    renderList();
    drawPicks();
    run();
  }

  // ----- the build
  function run() {
    if (!ready) return;
    const numbers = new Float64Array(picks.length * GROUP_NUMBERS);
    picks.forEach((p, i) => numbers.set(p.group, i * GROUP_NUMBERS));
    page.request('fillet_scene', [s.part, numbers, s.radius, s.round, s.segments]);
  }

  page.onResult = (r) => {
    try {
      const data = JSON.parse(r.mesh.data) as { rows: Row[] };
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
      unlight();
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
      const start = STARTING_RADIUS[part] ?? 2;
      s.radius = fresh || part === UPLOAD ? start : loadSetting(ROUTE, `radius${part}`, start);
      radiusLabel();
      saveRadius();
      const saved = part === UPLOAD || fresh
        ? null
        : savedPicks(loadSetting<string>(ROUTE, `picks${part}`, ''));
      if (saved) {
        picks = saved;
      } else if (STARTING_PICKS[part]) {
        for (const start of STARTING_PICKS[part]) {
          const origin = start.target.map((v, k) => v + start.from[k]);
          const dir = start.target.map((v, k) => v - origin[k]);
          const answer = JSON.parse(await side.call<string>('fillet_pick',
            [part, ...origin, ...dir, start.mode, 0.002, s.round])) as PickAnswer;
          if (disposed) return;
          if (answer.hit) addPick(answer, false);
        }
        savePicks();
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

  function addPick(answer: PickAnswer, rebuild = true) {
    const used = new Set(picks.map(p => p.color));
    const color = PICK_COLORS.find(c => !used.has(c)) ?? PICK_COLORS[picks.length % PICK_COLORS.length];
    picks.push({
      group: answer.group!,
      segments: answer.segments ?? [],
      point: answer.point ?? null,
      label: answer.label ?? 'pick',
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
      // a tap has no hover before it: pick where it landed, and draw nothing of it afterwards
      // (no pointer stays there to take the highlight away)
      const ray = viewer.rayAt(e.clientX, e.clientY);
      hover = JSON.parse(await side.call<string>('fillet_pick',
        [s.part, ...ray.origin, ...ray.direction, s.mode, ray.pixelSize * 2, s.round])) as PickAnswer;
      if (disposed) return;
    }
    const tapped = hover;
    if (e.pointerType !== 'mouse') {
      hover = null;
      drawHover();
    }
    if (!tapped?.hit) return;
    const key = signature({ segments: tapped.segments ?? [], point: tapped.point ?? null });
    const same = picks.findIndex(p => signature(p) === key);
    if (same >= 0) {
      picks.splice(same, 1);
      rows.splice(same, 1);
      afterPicksChanged();
    } else {
      addPick(tapped);
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
  radiusLabel();
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
