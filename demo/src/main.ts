// The demo site's entry point: the hash router (#/<route>) that lazily loads each page from
// src/demos/, the home page with a card per page, the mobile sidebar toggle, and on localhost the
// thumbnail capture (src/capture.ts): the "Update Thumbnails" button, or ?capture=thumbs to run it
// on load. ?hero hides the localhost-only button, so the README hero (a screenshot of the home
// page, capture-hero.ts) shows the page as visitors see it.
//
// The wasm itself is loaded by the mesh worker (src/mesh-worker.ts) on a page's first request, so
// the home page costs no wasm download.

import { captureThumbnails, isLocalhost } from './capture.ts';
import { DEMO_ROUTES } from './routes.ts';

type DemoInit = (container: HTMLElement) => (() => void) | void;
const demoModules: Record<string, () => Promise<{ init: DemoInit }>> = {
  'fillet': () => import('./demos/fillet.ts' as any),
  'booleans': () => import('./demos/booleans.ts' as any),
  'outline': () => import('./demos/outline.ts' as any),
  'implicit': () => import('./demos/implicit.ts' as any),
  'minkowski-hull': () => import('./demos/minkowski-hull.ts' as any),
  'repair': () => import('./demos/repair.ts' as any),
  'measure-export': () => import('./demos/measure-export.ts' as any),
  'ray-trace': () => import('./demos/ray-trace.ts' as any),
};

const CARDS: Record<string, { title: string; text: string }> = {
  'fillet': {
    title: 'Fillet & Chamfer',
    text: 'Pick edge chains, single edges, face rims or corners, set one radius, and watch them all rounded live.',
  },
  'booleans': {
    title: 'Booleans',
    text: 'Union, difference and intersection on the exact kernel, and the robust path for inside-out or self-intersecting operands.',
  },
  'outline': {
    title: 'Outline to Solid',
    text: 'Draw a profile in mm; extrude it with rounded or chamfered bevels, or revolve it about z.',
  },
  'implicit': {
    title: 'Implicit & SDF',
    text: 'Signed distance fields with smooth blends, meshed by marching cubes or dual contouring.',
  },
  'minkowski-hull': {
    title: 'Minkowski & Hull',
    text: 'Dilate or erode a solid by a tool, or wrap a cloud of shells in its convex hull.',
  },
  'repair': {
    title: 'Mesh Repair',
    text: 'Punch holes, flip faces and unweld a solid, then weld, orient and fill it closed again.',
  },
  'measure-export': {
    title: 'Measure & Export',
    text: 'A part built from most of the library: volume, area, mass, and its STL to download.',
  },
  'ray-trace': {
    title: 'Ray Tracing',
    text: 'A solid path traced in WebAssembly by the library: real shadows, metal that mirrors the sky and glass that bends the floor behind it, up to a million triangles.',
  },
};

let currentCleanup: (() => void) | null = null;

const menuToggle = document.getElementById('menu-toggle')!;
const sidebar = document.getElementById('sidebar')!;
const sidebarOverlay = document.getElementById('sidebar-overlay')!;

function setSidebar(open: boolean) {
  sidebar.classList.toggle('open', open);
  menuToggle.classList.toggle('open', open);
  sidebarOverlay.classList.toggle('visible', open);
}

menuToggle.addEventListener('click', () => setSidebar(!sidebar.classList.contains('open')));
sidebarOverlay.addEventListener('click', () => setSidebar(false));
document.querySelectorAll('.nav-link').forEach(link => link.addEventListener('click', () => setSidebar(false)));

function getRoute(): string {
  return window.location.hash.slice(2) || 'home';
}

function updateNav(route: string) {
  document.querySelectorAll('.nav-link').forEach(el => {
    el.classList.toggle('active', (el as HTMLElement).dataset.route === route);
  });
}

function escapeHtml(s: string): string {
  return s.replace(/&/g, '&amp;').replace(/</g, '&lt;');
}

function renderHome(container: HTMLElement) {
  const cards = DEMO_ROUTES.map(route => `
    <a href="#/${route}" class="feature-card">
      <div class="card-thumb" style="background-image:url(public/thumbs/${route}.jpg)"></div>
      <h3>${escapeHtml(CARDS[route].title)}</h3>
      <p>${escapeHtml(CARDS[route].text)}</p>
    </a>`).join('');
  container.innerHTML = `
    <div class="home-page">
      <div class="github-badge">
        <a href="https://github.com/larsbrubaker/solidmesh" target="_blank" rel="noopener" class="github-badge-link">
          <svg height="20" viewBox="0 0 16 16" width="20" fill="currentColor"><path d="M8 0C3.58 0 0 3.58 0 8c0 3.54 2.29 6.53 5.47 7.59.4.07.55-.17.55-.38 0-.19-.01-.82-.01-1.49-2.01.37-2.53-.49-2.69-.94-.09-.23-.48-.94-.82-1.13-.28-.15-.68-.52-.01-.53.63-.01 1.08.58 1.23.82.72 1.21 1.87.87 2.33.66.07-.52.28-.87.51-1.07-1.78-.2-3.64-.89-3.64-3.95 0-.87.31-1.59.82-2.15-.08-.2-.36-1.02.08-2.12 0 0 .67-.21 2.2.82.64-.18 1.32-.27 2-.27.68 0 1.36.09 2 .27 1.53-1.04 2.2-.82 2.2-.82.44 1.1.16 1.92.08 2.12.51.56.82 1.27.82 2.15 0 3.07-1.87 3.75-3.65 3.95.29.25.54.73.54 1.48 0 1.07-.01 1.93-.01 2.2 0 .21.15.46.55.38A8.013 8.013 0 0016 8c0-4.42-3.58-8-8-8z"/></svg>
          <span>larsbrubaker/solidmesh</span>
        </a>
      </div>
      <div class="hero">
        <h1>solid<span>mesh</span></h1>
        <p>
          Solid modeling on triangle meshes, in pure Rust: fillets and chamfers, booleans, outlines
          to solids, implicit blends, Minkowski sums, convex hulls and repair, on the meshes real
          pipelines contain.
          Every page below runs the library itself in your browser, compiled to WebAssembly.
        </p>
      </div>
      <div class="feature-grid">${cards}</div>
      <div class="about-section">
        <h2>About solidmesh</h2>
        <p>
          solidmesh is the geometry kernel behind MatterCAD, moved out of the application so anyone
          can use it. It works on clean modeled parts and on what real pipelines hand over (STL and
          3MF downloads, scans, triangle soup), builds for <code>wasm32-unknown-unknown</code>,
          contains no C or C++, and gives the same answer on every platform. Booleans run on
          <a href="https://github.com/larsbrubaker/manifold-rust" target="_blank" rel="noopener">manifold-rust</a>.
          Fillets and chamfers, the library's signature feature, come through one call,
          <code>bevel::part::bevel_part</code>.
        </p>
        <p style="margin-top: 12px">
          By <strong>Lars Brubaker</strong>, sponsored by
          <a href="https://www.matterhackers.com" target="_blank" rel="noopener">MatterHackers</a>.
        </p>
        <div class="stats-row">
          <div class="stat"><div class="stat-value">Rust</div><div class="stat-label">No C or C++</div></div>
          <div class="stat"><div class="stat-value">f64</div><div class="stat-label">Deterministic</div></div>
          <div class="stat"><div class="stat-value">Closed</div><div class="stat-label">In and out</div></div>
          <div class="stat"><div class="stat-value">WASM</div><div class="stat-label">Browser ready</div></div>
        </div>
      </div>
    </div>`;
}

async function navigate(route: string) {
  const container = document.getElementById('main-content')!;
  if (currentCleanup) {
    currentCleanup();
    currentCleanup = null;
  }
  updateNav(route);
  if (route === 'home') {
    renderHome(container);
    return;
  }
  const loader = demoModules[route];
  if (!loader) {
    container.innerHTML = `<div class="home-page"><h2>Page not found</h2><p>Unknown route: ${escapeHtml(route)}</p></div>`;
    return;
  }
  container.innerHTML = `<div class="home-page" style="display:flex;align-items:center;justify-content:center;height:80vh;"><p style="color:var(--text-muted)">Loading demo...</p></div>`;
  try {
    const mod = await loader();
    container.innerHTML = '';
    const cleanup = mod.init(container);
    if (cleanup) currentCleanup = cleanup;
  } catch (e) {
    console.error('Failed to load demo:', e);
    container.innerHTML = `<div class="home-page"><h2>Error loading demo</h2><pre style="color:#c0392b">${escapeHtml(String(e))}</pre></div>`;
  }
}

window.addEventListener('hashchange', () => navigate(getRoute()));
navigate(getRoute());

const params = new URLSearchParams(location.search);
const thumbsBtn = document.getElementById('capture-thumbs-btn') as HTMLButtonElement;
// The capture moves between routes with replaceState, which fires no hashchange, so each page is
// loaded exactly once and the capture knows when it is up.
const captureNavigate = async (route: string) => {
  history.replaceState(null, '', `#/${route}`);
  await navigate(route);
};

if (isLocalhost() && !params.has('hero')) {
  thumbsBtn.style.display = 'block';
  thumbsBtn.addEventListener('click', () => captureThumbnails(captureNavigate, thumbsBtn));
}

// Scripted capture (capture-thumbs.ts): ?capture=thumbs runs it on load and retitles the page
// when it is done.
if (isLocalhost() && params.get('capture') === 'thumbs') {
  captureThumbnails(captureNavigate, thumbsBtn).then(() => { document.title = 'thumbnails captured'; });
}
