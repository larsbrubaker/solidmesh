// The headless browser the capture scripts (capture-thumbs.ts, capture-hero.ts) drive: a locally
// installed Chromium-family browser (Microsoft Edge or Google Chrome, or Chromium on Linux), run
// with `--headless=new` and a throwaway profile so it never touches the user's own. $BROWSER
// names a browser executable to use instead of searching.
//
// Also the dev-server check both scripts start with: they capture from the running site
// (`bun run dev`), on $PORT or 3030.

import { existsSync, mkdtempSync, rmSync } from "fs";
import { tmpdir } from "os";
import { join } from "path";

export const PORT = parseInt(process.env.PORT || "3030");
export const SITE = `http://localhost:${PORT}`;

/** The browser executable to drive, or an error naming what was tried. */
export function findBrowser(): string {
  const fromEnv = process.env.BROWSER;
  if (fromEnv) {
    if (existsSync(fromEnv)) return fromEnv;
    throw new Error(`$BROWSER is set to ${fromEnv}, which does not exist.`);
  }
  const pf = process.env["ProgramFiles"] || "C:\\Program Files";
  const pf86 = process.env["ProgramFiles(x86)"] || "C:\\Program Files (x86)";
  const local = process.env["LOCALAPPDATA"] || "";
  const candidates = [
    join(pf86, "Microsoft", "Edge", "Application", "msedge.exe"),
    join(pf, "Microsoft", "Edge", "Application", "msedge.exe"),
    join(pf, "Google", "Chrome", "Application", "chrome.exe"),
    join(pf86, "Google", "Chrome", "Application", "chrome.exe"),
    join(local, "Google", "Chrome", "Application", "chrome.exe"),
    "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
    "/Applications/Microsoft Edge.app/Contents/MacOS/Microsoft Edge",
    "/Applications/Chromium.app/Contents/MacOS/Chromium",
  ];
  for (const c of candidates) if (existsSync(c)) return c;
  for (const name of ["google-chrome", "google-chrome-stable", "chromium", "chromium-browser", "microsoft-edge"]) {
    const found = Bun.which(name);
    if (found) return found;
  }
  throw new Error(`No Edge or Chrome found (tried ${candidates.join(", ")} and the PATH); set $BROWSER.`);
}

/** Exits with a message unless the dev server answers. */
export async function requireDevServer() {
  try {
    const resp = await fetch(SITE);
    if (resp.ok) return;
  } catch {
    // reported below
  }
  console.error(`No demo server at ${SITE}: start it with \`bun run dev\` (PORT=${PORT}) first.`);
  process.exit(1);
}

/** Headless flags with a fresh profile; `cleanup` removes the profile afterwards. */
export function headlessArgs(extra: string[]): { args: string[]; cleanup: () => void } {
  const profile = mkdtempSync(join(tmpdir(), "solidmesh-capture-"));
  const args = [
    "--headless=new",
    `--user-data-dir=${profile}`,
    "--no-first-run",
    "--no-default-browser-check",
    "--disable-extensions",
    ...extra,
  ];
  return {
    args,
    cleanup: () => {
      try {
        rmSync(profile, { recursive: true, force: true });
      } catch {
        // A browser helper can hold the profile for a moment after exit; it is only temp files.
      }
    },
  };
}

/** Stops a browser and the helper processes it started. */
export function killTree(proc: { pid: number; kill: () => void }) {
  if (process.platform === "win32") {
    Bun.spawnSync(["taskkill", "/PID", String(proc.pid), "/T", "/F"], { stdout: "ignore", stderr: "ignore" });
  } else {
    proc.kill();
  }
}
