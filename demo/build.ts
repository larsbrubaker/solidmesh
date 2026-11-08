// Static build for GitHub Pages: bundles src/main.ts (with its lazily loaded pages) and the mesh
// worker into dist/, then copies index.html, the styles, the wasm package (public/pkg, built
// first by wasm-pack), the thumbnails and the favicon beside them. .github/workflows/
// deploy-demo.yml runs it as `bun run build` and uploads dist/.
//
// Usage: bun run build.ts

import { cpSync, existsSync, mkdirSync, readFileSync, rmSync, writeFileSync } from "fs";
import { join } from "path";

const ROOT = import.meta.dir;
const DIST = join(ROOT, "dist");

rmSync(DIST, { recursive: true, force: true });
mkdirSync(DIST, { recursive: true });

if (!existsSync(join(ROOT, "public/pkg/solidmesh_wasm.js"))) {
  console.error("public/pkg is missing: run `bun run build:wasm` (wasm-pack) first.");
  process.exit(1);
}

async function bundle(entry: string, splitting: boolean) {
  const result = await Bun.build({
    entrypoints: [join(ROOT, entry)],
    outdir: DIST,
    splitting,
    target: "browser",
    format: "esm",
    minify: true,
    naming: "[dir]/[name].[ext]",
    // Three.js comes from the CDN through index.html's import map.
    external: ["three", "three/addons/*"],
  });
  if (!result.success) {
    console.error(`Bundling ${entry} failed:`);
    for (const msg of result.logs) console.error(msg);
    process.exit(1);
  }
  console.log(`  ${entry}: ${result.outputs.length} files`);
}

console.log("Bundling TypeScript...");
await bundle("src/main.ts", true);
// The worker is its own entry point: Bun.build does not rewrite `new Worker(new URL(...))`, so
// mesh-runner.ts loads it by the fixed name mesh-worker.js next to main.js.
await bundle("src/mesh-worker.ts", false);

console.log("Copying index.html, styles, wasm, thumbnails...");
const html = readFileSync(join(ROOT, "index.html"), "utf-8").replace(
  '<script type="module" src="./src/main.ts"></script>',
  '<script type="module" src="./main.js"></script>',
);
writeFileSync(join(DIST, "index.html"), html);
cpSync(join(ROOT, "styles"), join(DIST, "styles"), { recursive: true });
cpSync(join(ROOT, "public/pkg"), join(DIST, "public/pkg"), { recursive: true });
// Thumbnails are captured on localhost and committed (capture-thumbs.ts); until the first capture
// there are none, and the home cards show without pictures.
if (existsSync(join(ROOT, "public/thumbs"))) {
  cpSync(join(ROOT, "public/thumbs"), join(DIST, "public/thumbs"), { recursive: true });
}
cpSync(join(ROOT, "public/favicon.svg"), join(DIST, "public/favicon.svg"));
// GitHub Pages would otherwise run Jekyll over the site.
writeFileSync(join(DIST, ".nojekyll"), "");

console.log(`Build complete -> ${DIST}`);
