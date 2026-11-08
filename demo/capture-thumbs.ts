// Thumbnail capture, and how the README hero is made.
//
// Thumbnails (demo/public/thumbs/<route>.jpg, the pictures on the home page's cards) are rendered
// by the running demo itself, from geometry the library computes in the browser, and saved by the
// dev server (POST /api/save-thumb in server.ts). The capture code is src/capture.ts: for each
// page in src/routes.ts it navigates, waits until the mesh worker is idle (at most 30 s, then it
// warns in the console and captures the page as it stands), draws the viewer once and saves a
// 320 x 240 JPEG. It takes about a second per page.
//
// Setup (once): wasm-pack (`cargo install wasm-pack --locked`) and Bun (`winget install
// Oven-sh.Bun`, or bun.sh); rust-toolchain.toml pins the wasm32 target. Then, from demo/:
//   bun install
//   bun run build:wasm
//   bun run dev                       # http://localhost:3030 (PORT=... to change)
//
// Capture the thumbnails, either way:
//   bun run capture:thumbs            # this script: headless Edge or Chrome opens
//                                     # http://localhost:3030/?capture=thumbs, which runs the
//                                     # capture on load; the script waits for every page's file
//   or click "Update Thumbnails" in the sidebar of http://localhost:3030 (localhost only)
//
// Then the README hero (README_HERO.png at the repository root, 1234 x 749, a screenshot of the
// home page with its cards and thumbnails, like the other rust-apps libraries' heroes):
//   bun run capture:hero              # capture-hero.ts, after the thumbnails exist
// which runs `<browser> --headless=new --screenshot=<repo>/README_HERO.png --window-size=1234,749
// --hide-scrollbars http://localhost:3030/?hero` (?hero hides the localhost-only button).
//
// Both scripts need the dev server running and find Edge or Chrome themselves (browser.ts; set
// $BROWSER to choose one, $PORT for another port). Commit demo/public/thumbs/*.jpg and
// README_HERO.png afterwards. A new page gets a thumbnail by adding its route to src/routes.ts
// and a card to src/main.ts's CARDS.

import { statSync } from "fs";
import { join } from "path";
import { SITE, findBrowser, headlessArgs, killTree, requireDevServer } from "./browser.ts";
import { DEMO_ROUTES } from "./src/routes.ts";

const THUMBS = join(import.meta.dir, "public", "thumbs");
const TIMEOUT_MS = 4 * 60_000;

function savedSince(route: string, start: number): boolean {
  try {
    return statSync(join(THUMBS, `${route}.jpg`)).mtimeMs >= start;
  } catch {
    return false;
  }
}

if (import.meta.main) {
  await requireDevServer();
  const browser = findBrowser();
  const start = Date.now() - 1000;
  const { args, cleanup } = headlessArgs(["--window-size=1280,800", `${SITE}/?capture=thumbs`]);
  console.log(`Capturing thumbnails with ${browser}...`);
  const proc = Bun.spawn([browser, ...args], { stdout: "ignore", stderr: "ignore" });
  const done = new Set<string>();
  while (done.size < DEMO_ROUTES.length && Date.now() - start < TIMEOUT_MS) {
    for (const route of DEMO_ROUTES) {
      if (!done.has(route) && savedSince(route, start)) {
        done.add(route);
        console.log(`  ${route}.jpg (${((Date.now() - start) / 1000).toFixed(1)} s)`);
      }
    }
    await Bun.sleep(250);
  }
  // The last file can still be landing when its mtime appears; let the write finish.
  await Bun.sleep(500);
  killTree(proc);
  await proc.exited;
  cleanup();
  const missing = DEMO_ROUTES.filter(r => !done.has(r));
  if (missing.length) {
    console.error(`Not captured within ${TIMEOUT_MS / 1000} s: ${missing.join(", ")}`);
    process.exit(1);
  }
  console.log(`All ${DEMO_ROUTES.length} thumbnails saved to ${THUMBS}`);
}
