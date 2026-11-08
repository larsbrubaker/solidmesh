// The demo pages that get a home-page card and a thumbnail, in sidebar order. Shared by the
// router's home page (src/main.ts), the in-page capture (src/capture.ts) and the capture scripts
// at the demo root (capture-thumbs.ts, capture-hero.ts), which run under Bun outside the page.

export const DEMO_ROUTES = [
  'fillet', 'booleans', 'outline', 'implicit', 'minkowski-hull', 'repair', 'measure-export',
  'ray-trace',
];
