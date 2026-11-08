// The README hero: a scene of solids all computed by the library in this browser - the showcase
// part in front, and behind it a boolean with a messy operand, a smooth implicit blend, a revolved
// goblet and a Minkowski-rounded bracket - rendered at the size of README_HERO.png (1234 x 749).
//
// `renderHero` builds it into a fixed-size viewer; src/capture.ts calls it and saves the frame
// through the dev server. The route #/hero (not in the sidebar) shows the same scene, so the
// composition can be checked before it is captured.

import { ThreeViewer } from '../three-viewer.ts';
import { runMesh, type MeshResult } from '../mesh-runner.ts';

export const HERO_WIDTH = 1234;
export const HERO_HEIGHT = 749;

interface Piece {
  fn: string;
  args: unknown[];
  color: number;
  /** Where the piece's footprint centre goes, z on the floor. */
  at: [number, number];
}

const GOBLET: [number, number][] = [
  [0, 0], [14, 0], [14, 2], [3, 4], [2.5, 18], [12, 26], [16, 40], [14, 40], [10, 28], [0, 24],
];

const PIECES: Piece[] = [
  { fn: 'showcase_scene', args: [6, 26, 3, 96], color: 0x2a9d8f, at: [0, -6] },
  // A rounded box minus an inside-out sphere, read with the nonzero winding rule.
  { fn: 'boolean_scene', args: [3, 1, 1, 9, -7, 8, 20, 1, true, false], color: 0x3d5a80, at: [-74, 46] },
  { fn: 'implicit_scene', args: [0, 5, 128, 0], color: 0xe76f51, at: [-26, 78] },
  {
    fn: 'revolve_scene',
    args: [Float64Array.from(GOBLET.flat()), Uint32Array.of(GOBLET.length), 96, 0, 360],
    color: 0xe9c46a, at: [26, 80],
  },
  { fn: 'minkowski_scene', args: [1, 0, 3, false, 32], color: 0x8d6cab, at: [74, 46] },
];

function bounds(m: MeshResult) {
  const p = m.positions;
  const min = [Infinity, Infinity, Infinity];
  const max = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < p.length; i += 3) {
    for (let k = 0; k < 3; k++) {
      min[k] = Math.min(min[k], p[i + k]);
      max[k] = Math.max(max[k], p[i + k]);
    }
  }
  return { min, max };
}

/** Builds the hero scene into `container` at the hero size; resolves once it is drawn. */
export async function renderHero(container: HTMLElement): Promise<ThreeViewer> {
  const viewer = new ThreeViewer(container, {
    width: HERO_WIDTH,
    height: HERO_HEIGHT,
    pixelRatio: 1,
  });
  viewer.setEdges(true);
  for (const piece of PIECES) {
    const { mesh } = await runMesh(piece.fn, piece.args);
    const { min, max } = bounds(mesh);
    const offset: [number, number, number] = [
      piece.at[0] - (min[0] + max[0]) / 2,
      piece.at[1] - (min[1] + max[1]) / 2,
      -min[2],
    ];
    viewer.addMesh(mesh.positions, mesh.indices, piece.color, offset);
  }
  viewer.frame([0.1, -1, 0.62], 0.78);
  // Two frames so the canvas holds the finished scene before anyone reads it.
  await new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)));
  viewer.renderNow();
  return viewer;
}

export function init(container: HTMLElement): () => void {
  container.innerHTML = `<div class="demo-page">
    <div class="demo-header"><h2>README hero</h2>
    <p>The scene the README image is captured from (${HERO_WIDTH} &times; ${HERO_HEIGHT}).</p></div>
    <div id="viewer-container" style="width:${HERO_WIDTH}px;height:${HERO_HEIGHT}px;max-width:100%;overflow:auto"></div>
  </div>`;
  const host = container.querySelector('#viewer-container') as HTMLElement;
  let viewer: ThreeViewer | null = null;
  let disposed = false;
  renderHero(host).then(v => {
    if (disposed) v.dispose();
    else viewer = v;
  });
  return () => {
    disposed = true;
    viewer?.dispose();
  };
}
