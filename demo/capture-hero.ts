// The README hero: a 1234 x 749 screenshot of the demo's home page (sidebar, title and the grid
// of page cards with their thumbnails), written to README_HERO.png at the repository root - the
// same kind of hero as the other rust-apps libraries' READMEs.
//
// Run from demo/, with the dev server up and the thumbnails captured (capture-thumbs.ts):
//   bun run capture:hero
// It runs a headless Edge or Chrome (browser.ts) as
//   <browser> --headless=new --screenshot=<repo>/README_HERO.png --window-size=1234,749
//             --hide-scrollbars --virtual-time-budget=10000 http://localhost:3030/?hero
// ?hero hides the localhost-only capture button, and the virtual-time budget lets the page's
// script render the cards and their thumbnails load before the shot.

import { existsSync, readFileSync, rmSync } from "fs";
import { join, resolve } from "path";
import { SITE, findBrowser, headlessArgs, requireDevServer } from "./browser.ts";
import { DEMO_ROUTES } from "./src/routes.ts";

const OUT = resolve(import.meta.dir, "..", "README_HERO.png");
const WIDTH = 1234;
const HEIGHT = 749;

await requireDevServer();
const missing = DEMO_ROUTES.filter(r => !existsSync(join(import.meta.dir, "public", "thumbs", `${r}.jpg`)));
if (missing.length) {
  console.error(`Thumbnails missing (${missing.join(", ")}): run \`bun run capture:thumbs\` first.`);
  process.exit(1);
}

const browser = findBrowser();
const { args, cleanup } = headlessArgs([
  `--screenshot=${OUT}`,
  `--window-size=${WIDTH},${HEIGHT}`,
  "--hide-scrollbars",
  "--virtual-time-budget=10000",
  `${SITE}/?hero`,
]);
rmSync(OUT, { force: true });
console.log(`Screenshotting ${SITE}/?hero with ${browser}...`);
const proc = Bun.spawn([browser, ...args], { stdout: "ignore", stderr: "ignore" });
await proc.exited;
cleanup();

if (!existsSync(OUT)) {
  console.error("The browser wrote no screenshot.");
  process.exit(1);
}
// A PNG's IHDR holds the width and height, big-endian, at bytes 16 and 20.
const png = readFileSync(OUT);
const w = png.readUInt32BE(16);
const h = png.readUInt32BE(20);
console.log(`Saved ${OUT}: ${w} x ${h}, ${png.length} bytes`);
if (w !== WIDTH || h !== HEIGHT) {
  console.error(`Expected ${WIDTH} x ${HEIGHT}.`);
  process.exit(1);
}
