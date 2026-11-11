// Development server for the demo site (started by dev.ts, or `bun run server.ts`): serves
// index.html, transpiles src/*.ts per request, serves styles/ and public/ (the wasm package and
// thumbnails), and falls back to index.html for client-side routes. The transpiled code carries
// the build identity (the commit plus "dev server", site-build-id.ts) as build.ts's bundle does,
// worked out again each time src/site-build.ts is served.
//
// It also takes the localhost-only thumbnail capture src/capture.ts makes (see capture-thumbs.ts):
//   POST /api/save-thumb  { name, data: JPEG data URL }  -> demo/public/thumbs/<name>.jpg
// The server listens on localhost only, so nothing outside this machine can write those files.
//
// Port: $PORT, else 3030 (manifold-rust's demo keeps 3000, so both can run side by side).

import { existsSync, mkdirSync, readFileSync, statSync, writeFileSync } from "fs";
import { extname, join, resolve } from "path";
import { siteBuildDefine } from "./site-build-id.ts";

const PORT = parseInt(process.env.PORT || "3030");
const ROOT = import.meta.dir;

const MIME_TYPES: Record<string, string> = {
  ".html": "text/html",
  ".css": "text/css",
  ".js": "application/javascript",
  ".ts": "application/javascript",
  ".wasm": "application/wasm",
  ".json": "application/json",
  ".svg": "image/svg+xml",
  ".png": "image/png",
  ".jpg": "image/jpeg",
  ".ico": "image/x-icon",
};
const BINARY_EXTS = new Set([".wasm", ".jpg", ".png", ".ico"]);

const transpiler = new Bun.Transpiler({ loader: "ts", target: "browser" });
/** The build identity is worked out afresh each time it is served, so a commit or an edit made
 * while the server runs shows on the next page load. */
const IDENTITY_FILE = "/src/site-build.ts";

function tryServe(pathname: string): { content: string | Uint8Array; mime: string } | null {
  if (pathname === "/" || pathname === "/index.html") {
    return { content: readFileSync(join(ROOT, "index.html"), "utf-8"), mime: "text/html" };
  }
  const filePath = join(ROOT, pathname);
  // Never serve outside demo/.
  if (!resolve(filePath).startsWith(resolve(ROOT))) return null;
  if (!existsSync(filePath) || !statSync(filePath).isFile()) return null;
  const ext = extname(filePath);
  if (pathname.startsWith("/src/") && ext === ".ts") {
    const source = readFileSync(filePath, "utf-8");
    const code = pathname === IDENTITY_FILE
      ? new Bun.Transpiler({ loader: "ts", target: "browser", define: siteBuildDefine("dev") }).transformSync(source)
      : transpiler.transformSync(source);
    return { content: code, mime: "application/javascript" };
  }
  const mime = MIME_TYPES[ext] || "application/octet-stream";
  if (BINARY_EXTS.has(ext)) return { content: new Uint8Array(readFileSync(filePath)), mime };
  return { content: readFileSync(filePath, "utf-8"), mime };
}

/** The bytes of a base64 image data URL of the given type, or null. */
function imageBytes(data: unknown, type: "jpeg" | "png"): Buffer | null {
  if (typeof data !== "string") return null;
  const prefix = `data:image/${type};base64,`;
  if (!data.startsWith(prefix)) return null;
  return Buffer.from(data.slice(prefix.length), "base64");
}

function json(body: unknown, status = 200): Response {
  return new Response(JSON.stringify(body), {
    status,
    headers: { "Content-Type": "application/json" },
  });
}

async function saveThumb(req: Request): Promise<Response> {
  const { name, data } = (await req.json()) as { name: unknown; data: unknown };
  if (typeof name !== "string" || !/^[a-z0-9_-]+$/.test(name)) {
    return json({ ok: false, error: "invalid name" }, 400);
  }
  const bytes = imageBytes(data, "jpeg");
  if (!bytes) return json({ ok: false, error: "expected a JPEG data URL" }, 400);
  const dir = join(ROOT, "public", "thumbs");
  mkdirSync(dir, { recursive: true });
  const out = join(dir, `${name}.jpg`);
  writeFileSync(out, bytes);
  console.log(`[thumbs] Saved ${out} (${bytes.length} bytes)`);
  return json({ ok: true, size: bytes.length });
}

const server = Bun.serve({
  port: PORT,
  hostname: "localhost",
  async fetch(req) {
    const url = new URL(req.url);
    try {
      if (req.method === "POST" && url.pathname === "/api/save-thumb") return await saveThumb(req);
    } catch (e: any) {
      return json({ ok: false, error: String(e?.message ?? e) }, 500);
    }
    const resolved = tryServe(url.pathname);
    if (resolved) {
      return new Response(resolved.content, {
        headers: { "Content-Type": resolved.mime, "Cache-Control": "no-cache" },
      });
    }
    return new Response(readFileSync(join(ROOT, "index.html"), "utf-8"), {
      headers: { "Content-Type": "text/html", "Cache-Control": "no-cache" },
    });
  },
});

console.log(`solidmesh demo running at http://localhost:${server.port}`);
