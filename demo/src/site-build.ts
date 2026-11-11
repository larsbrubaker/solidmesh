// Which build of the site this is: the commit and build time demo/site-build-id.ts works out,
// baked in as `__SITE_BUILD__` by demo/build.ts (the static site) and demo/server.ts (local runs).
// The Ray Tracing page's "Copy timings" report puts it on its first line. Code run some other way
// (a test, say) has no identity baked in and says so.

declare const __SITE_BUILD__: string | undefined;

export const SITE_BUILD: string = typeof __SITE_BUILD__ === 'string'
  ? __SITE_BUILD__
  : 'unknown: this code was not bundled by demo/build.ts or served by demo/server.ts';
