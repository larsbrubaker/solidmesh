// The localhost-only thumbnail capture (how to run it, and the README hero command:
// capture-thumbs.ts at the demo root).
//
// Visit every demo route, wait until the mesh worker is idle (the page's result computed and
// shown), draw the viewer once more right then (or, for a page that draws on its own canvas and
// registers it with `setCaptureCanvas`, take that canvas), scale it to 320 x 240 and POST it as
// JPEG to the dev server, which writes demo/public/thumbs/<route>.jpg.
//
// Nothing here waits on animation frames: a browser stops or slows requestAnimationFrame for a
// page that is hidden or behind another window, which once made each page take ~25 s and a page
// wait indefinitely. The capture draws the frame itself (`ThreeViewer.renderNow`), and a page
// that is still not idle after PAGE_TIMEOUT_MS is captured as it stands, with a console warning.
//
// The button is shown only when the page is served from localhost; GitHub Pages has no server to
// save to.

import { whenIdleWithin } from './mesh-runner.ts';
import { activeViewer } from './three-viewer.ts';
import { DEMO_ROUTES } from './routes.ts';

/** How long one page may compute before it is captured anyway. */
const PAGE_TIMEOUT_MS = 30_000;

export function isLocalhost(): boolean {
  return ['localhost', '127.0.0.1', '[::1]'].includes(location.hostname);
}

async function post(path: string, body: unknown): Promise<string> {
  const resp = await fetch(path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  const json = await resp.json();
  return json.ok ? `${json.size} bytes` : `failed: ${json.error}`;
}

let pageCanvas: HTMLCanvasElement | null = null;

/** A page drawing on its own canvas instead of the 3D viewer registers it here for the capture. */
export function setCaptureCanvas(canvas: HTMLCanvasElement | null) {
  pageCanvas = canvas;
}

/** The page's canvas, or the viewer's drawn now, cover-cropped into a 320 x 240 JPEG data URL. */
function thumbnail(): string | null {
  const viewer = activeViewer();
  if (!pageCanvas && !viewer) return null;
  if (viewer && !pageCanvas) viewer.renderNow();
  const canvas = pageCanvas ?? viewer!.canvas;
  const tmp = document.createElement('canvas');
  tmp.width = 320;
  tmp.height = 240;
  const ctx = tmp.getContext('2d')!;
  const scale = Math.max(320 / canvas.width, 240 / canvas.height);
  const w = 320 / scale;
  const h = 240 / scale;
  ctx.drawImage(canvas, (canvas.width - w) / 2, (canvas.height - h) / 2, w, h, 0, 0, 320, 240);
  return tmp.toDataURL('image/jpeg', 0.85);
}

export async function captureThumbnails(navigate: (route: string) => Promise<void>, btn: HTMLButtonElement) {
  btn.disabled = true;
  btn.classList.add('capturing');
  const back = location.hash;
  const results: string[] = [];
  for (const route of DEMO_ROUTES) {
    btn.textContent = `Capturing ${route}...`;
    const start = performance.now();
    // The page queues its first request while it loads, so by now the runner counts it.
    await navigate(route);
    if (!(await whenIdleWithin(PAGE_TIMEOUT_MS))) {
      console.warn(`Thumbnail ${route}: still computing after ${PAGE_TIMEOUT_MS / 1000} s; capturing it as it stands.`);
    }
    const data = thumbnail();
    if (!data) {
      results.push(`${route}: no viewer`);
      continue;
    }
    try {
      const saved = await post('/api/save-thumb', { name: route, data });
      results.push(`${route}: ${saved} in ${Math.round(performance.now() - start)} ms`);
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
