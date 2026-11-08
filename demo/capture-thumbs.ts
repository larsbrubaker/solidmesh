// Thumbnail and README hero capture: how it works and how to run it. (Documentation only; the
// capture code is src/capture.ts, the saving is server.ts.)
//
// Both images are rendered by the running demo itself, from geometry the library computes in the
// browser, and saved by the dev server. The two buttons that start them appear in the sidebar
// footer only when the page is served from localhost; they never show on GitHub Pages.
//
// Setup (once): rustup target wasm32-unknown-unknown (rust-toolchain.toml pins it), wasm-pack
// (`cargo install wasm-pack --locked`) and Bun (`winget install Oven-sh.Bun`, or bun.sh).
//
// Run, from demo/:
//   bun install
//   bun run build:wasm          # wasm-pack build wasm --target web --out-dir ../public/pkg
//   bun run dev                 # serves http://localhost:3030 (PORT=... to change)
// then open http://localhost:3030 in a desktop browser (a window at least 1280 px wide, so the
// fixed-size hero stage fits) and:
//
//   "Update Thumbnails"   visits every page in src/capture.ts's DEMO_ROUTES, waits until its mesh
//                         is computed and drawn, and saves a 320 x 240 JPEG of the viewer to
//                         demo/public/thumbs/<route>.jpg (POST /api/save-thumb). The home page's
//                         cards show them.
//   "Capture README Hero" renders src/demos/hero.ts's scene (the showcase part, a messy boolean,
//                         a smooth implicit blend, a revolved goblet, a Minkowski-rounded bracket)
//                         at 1234 x 749, writes the name and tagline into its corner, and saves it
//                         as README_HERO.png at the repository root (POST /api/save-hero).
//                         #/hero shows the same scene without saving, to check the composition.
//
// Then commit demo/public/thumbs/*.jpg and README_HERO.png.
//
// To give a new page a thumbnail: add its route to DEMO_ROUTES (and a card to main.ts's CARDS),
// then click "Update Thumbnails" again.

export {};
