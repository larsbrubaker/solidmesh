// A small SVG editor for 2D contours in mm, used by the Outline to Solid page: drag a point to
// move it, double-click an edge to add a point there, right-click a point to remove it (a contour
// keeps at least three). y points up, as in the kernel; with `axis` set, the revolve axis (x = 0)
// is drawn and points cannot cross it.
//
// The contours are filled even-odd, which is how the library reads a profile: a contour inside
// another is a hole.

export type Contour = [number, number][];

export interface EditorOptions {
  /** The visible square, in mm: [min x, min y, size]. */
  view: [number, number, number];
  /** Draw the revolve axis and keep points at x >= 0. */
  axis: boolean;
  onChange: (contours: Contour[]) => void;
}

const SVG_NS = 'http://www.w3.org/2000/svg';

function el<K extends keyof SVGElementTagNameMap>(tag: K, attrs: Record<string, string | number>) {
  const e = document.createElementNS(SVG_NS, tag);
  for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, String(v));
  return e;
}

export class ProfileEditor {
  readonly element: SVGSVGElement;
  private layer: SVGGElement;
  private contours: Contour[] = [];
  private drag: { c: number; p: number } | null = null;

  constructor(private options: EditorOptions) {
    const [x0, y0, size] = options.view;
    this.element = el('svg', {
      class: 'profile-editor',
      // The group flips y, so the view box spans -(y0 + size)..-y0 in SVG units.
      viewBox: `${x0} ${-(y0 + size)} ${size} ${size}`,
    });
    this.layer = el('g', { transform: 'scale(1,-1)' });
    this.element.appendChild(this.layer);
    this.element.addEventListener('pointermove', e => this.onMove(e));
    this.element.addEventListener('pointerup', () => { this.drag = null; });
    this.element.addEventListener('pointerleave', () => { this.drag = null; });
    this.element.addEventListener('contextmenu', e => e.preventDefault());
  }

  /** The point under the pointer, in mm. */
  private toModel(e: MouseEvent): [number, number] {
    const pt = this.element.createSVGPoint();
    pt.x = e.clientX;
    pt.y = e.clientY;
    const m = this.layer.getScreenCTM();
    if (!m) return [0, 0];
    const p = pt.matrixTransform(m.inverse());
    const snap = (v: number) => Math.round(v * 2) / 2;
    return [this.options.axis ? Math.max(0, snap(p.x)) : snap(p.x), snap(p.y)];
  }

  setContours(contours: Contour[]) {
    this.contours = contours.map(c => c.map(([x, y]) => [x, y] as [number, number]));
    this.draw();
  }

  getContours(): Contour[] {
    return this.contours;
  }

  private changed() {
    this.draw();
    this.options.onChange(this.contours);
  }

  private onMove(e: PointerEvent) {
    if (!this.drag) return;
    const p = this.toModel(e);
    const pts = this.contours[this.drag.c];
    const cur = pts[this.drag.p];
    if (cur[0] === p[0] && cur[1] === p[1]) return;
    pts[this.drag.p] = p;
    this.changed();
  }

  private draw() {
    const [x0, y0, size] = this.options.view;
    this.layer.replaceChildren();
    const step = size / 10;
    for (let i = 0; i <= 10; i++) {
      const x = x0 + i * step;
      const y = y0 + i * step;
      this.layer.appendChild(el('line', { class: 'grid-line', x1: x, y1: y0, x2: x, y2: y0 + size }));
      this.layer.appendChild(el('line', { class: 'grid-line', x1: x0, y1: y, x2: x0 + size, y2: y }));
    }
    if (this.options.axis) {
      this.layer.appendChild(el('line', { class: 'axis-line', x1: 0, y1: y0, x2: 0, y2: y0 + size }));
    }
    const d = this.contours
      .map(c => 'M' + c.map(([x, y]) => `${x},${y}`).join('L') + 'Z')
      .join(' ');
    const path = el('path', { class: 'contour', d });
    path.addEventListener('dblclick', e => this.insertAt(e));
    this.layer.appendChild(path);
    const r = size / 90;
    this.contours.forEach((c, ci) => c.forEach(([x, y], pi) => {
      const h = el('circle', { class: 'handle', cx: x, cy: y, r });
      h.addEventListener('pointerdown', e => {
        if (e.button === 2) {
          if (c.length > 3) {
            c.splice(pi, 1);
            this.changed();
          }
          return;
        }
        this.drag = { c: ci, p: pi };
        this.element.setPointerCapture(e.pointerId);
      });
      this.layer.appendChild(h);
    }));
  }

  /** Adds a point on the edge nearest the double-click. */
  private insertAt(e: MouseEvent) {
    const [px, py] = this.toModel(e);
    let best = { c: -1, i: -1, d: Infinity };
    this.contours.forEach((c, ci) => c.forEach(([ax, ay], i) => {
      const [bx, by] = c[(i + 1) % c.length];
      const dx = bx - ax, dy = by - ay;
      const len2 = dx * dx + dy * dy || 1;
      const t = Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / len2));
      const qx = ax + t * dx - px, qy = ay + t * dy - py;
      const dist = qx * qx + qy * qy;
      if (dist < best.d) best = { c: ci, i, d: dist };
    }));
    if (best.c < 0) return;
    this.contours[best.c].splice(best.i + 1, 0, [px, py]);
    this.changed();
  }
}

/** The contours as the wasm call takes them: flat xy pairs and the point count of each. */
export function flatten(contours: Contour[]): { coords: Float64Array; counts: Uint32Array } {
  const coords = new Float64Array(contours.reduce((n, c) => n + c.length * 2, 0));
  let k = 0;
  for (const c of contours) for (const [x, y] of c) { coords[k++] = x; coords[k++] = y; }
  return { coords, counts: Uint32Array.from(contours.map(c => c.length)) };
}
