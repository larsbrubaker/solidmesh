// The localhost-only captures (how to run them: capture-thumbs.ts at the demo root).
//
// Thumbnails: visit every demo route, wait until the mesh worker is idle and the result drawn,
// scale the viewer canvas to 320 x 240 and POST it as JPEG to the dev server, which writes
// demo/public/thumbs/<route>.jpg.
//
// Hero: render the hero scene (src/demos/hero.ts) at 1234 x 749 on a stage over the page, write
// the name and tagline into its corner, and POST it as PNG, which the dev server writes to the
// repository root as README_HERO.png.
//
// Both buttons are shown only when the page is served from localhost; GitHub Pages has no server
// to save to.

import { whenIdle } from './mesh-runner.ts';
import { HERO_HEIGHT, HERO_WIDTH, renderHero } from './demos/hero.ts';

/** The routes with a thumbnail on the home page, in sidebar order. */
export const DEMO_ROUTES = [
  'booleans', 'outline', 'implicit', 'minkowski-hull', 'repair', 'measure-export',
];

export function isLocalhost(): boolean {
  return ['localhost', '127.0.0.1', '[::1]'].includes(location.hostname);
}

const frames = (n: number) => new Promise<void>(resolve => {
  const step = (left: number) => (left <= 0 ? resolve() : requestAnimationFrame(() => step(left - 1)));
  step(n);
});

async function post(path: string, body: unknown): Promise<string> {
  const resp = await fetch(path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  const json = await resp.json();
  return json.ok ? `${json.size} bytes` : `failed: ${json.error}`;
}

export async function captureThumbnails(navigate: (route: string) => Promise<void>, btn: HTMLButtonElement) {
  btn.disabled = true;
  btn.classList.add('capturing');
  const back = location.hash;
  const results: string[] = [];
  for (const route of DEMO_ROUTES) {
    btn.textContent = `Capturing ${route}...`;
    await navigate(route);
    // The page queues its first request on load; wait for it (and any follow-up) to finish.
    await frames(2);
    await whenIdle();
    await frames(30);
    const canvas = document.querySelector('#viewer-container canvas') as HTMLCanvasElement | null;
    if (!canvas) {
      results.push(`${route}: no canvas`);
      continue;
    }
    const tmp = document.createElement('canvas');
    tmp.width = 320;
    tmp.height = 240;
    const ctx = tmp.getContext('2d')!;
    // Cover-crop the viewer into 4:3.
    const scale = Math.max(320 / canvas.width, 240 / canvas.height);
    const w = 320 / scale;
    const h = 240 / scale;
    ctx.drawImage(canvas, (canvas.width - w) / 2, (canvas.height - h) / 2, w, h, 0, 0, 320, 240);
    try {
      results.push(`${route}: ${await post('/api/save-thumb', { name: route, data: tmp.toDataURL('image/jpeg', 0.85) })}`);
    } catch (e) {
      results.push(`${route}: upload failed - ${e}`);
    }
  }
  location.hash = back || '#/';
  btn.disabled = false;
  btn.classList.remove('capturing');
  btn.textContent = 'Update Thumbnails';
  console.log('Thumbnail capture complete:', results);
}

export async function captureHero(btn: HTMLButtonElement) {
  btn.disabled = true;
  btn.classList.add('capturing');
  btn.textContent = 'Rendering hero...';
  const stage = document.createElement('div');
  stage.className = 'hero-stage';
  document.body.appendChild(stage);
  try {
    const viewer = await renderHero(stage);
    const out = document.createElement('canvas');
    out.width = HERO_WIDTH;
    out.height = HERO_HEIGHT;
    const ctx = out.getContext('2d')!;
    ctx.drawImage(viewer.canvas, 0, 0, HERO_WIDTH, HERO_HEIGHT);
    ctx.fillStyle = '#14213d';
    ctx.font = '700 44px -apple-system, "Segoe UI", Roboto, sans-serif';
    ctx.fillText('solid', 36, 70);
    const solidWidth = ctx.measureText('solid').width;
    ctx.fillStyle = '#2a9d8f';
    ctx.fillText('mesh', 36 + solidWidth, 70);
    ctx.fillStyle = '#4a5568';
    ctx.font = '400 20px -apple-system, "Segoe UI", Roboto, sans-serif';
    ctx.fillText('Booleans, outlines to solids, implicit blends, Minkowski sums and repair', 38, 102);
    ctx.fillText('on triangle meshes, in pure Rust. Every solid here was computed by the library.', 38, 128);
    viewer.dispose();
    const result = await post('/api/save-hero', { data: out.toDataURL('image/png') });
    console.log(`README_HERO.png: ${result}`);
    btn.textContent = `Hero saved (${result})`;
  } catch (e) {
    console.error('Hero capture failed:', e);
    btn.textContent = 'Hero capture failed';
  } finally {
    stage.remove();
    btn.disabled = false;
    btn.classList.remove('capturing');
    setTimeout(() => { btn.textContent = 'Capture README Hero'; }, 4000);
  }
}
