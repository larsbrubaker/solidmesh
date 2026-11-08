// The mesh worker: owns the wasm instance (public/pkg/solidmesh_wasm.js) and runs every scene
// builder off the UI thread, so a long Minkowski sum or a fine implicit grid never freezes the
// page. src/mesh-runner.ts is the main-thread side.
//
// Messages in:
//   { type: 'init', pkgUrl }                 first; the page computes the package URL, because
//                                            this file's own location differs between the dev
//                                            server (/src/) and the built site (/)
//   { type: 'run', id, fn, args }            call wasm export `fn(...args)`; keep the result
//   { type: 'stl', id }                      the last result as a binary STL
//   { type: 'call', id, fn, args }           call wasm export `fn(...args)` for a plain value (a
//                                            JSON string, a count): the Fillet page's picking
//                                            and upload, which keep state in this worker's wasm
// Messages out (one per run / stl / call, with its id):
//   { type: 'result', id, ok: true, ms, mesh } where mesh holds typed arrays and the measures
//   { type: 'call', id, ok: true, value, ms }
//   { type: 'stl', id, ok: true, bytes }
//   { type: ..., id, ok: false, error, fatal } (fatal: the wasm instance trapped and is unusable)
// `ms` is the wall time of the wasm call alone, measured here, which is what the pages show.

let pkgUrl: string | null = null;
let wasmReady: Promise<any> | null = null;
// The last result's wasm handle, kept for its STL; freed when the next result replaces it.
let last: any = null;

/** One wasm instantiation, shared by every caller (memoized on the promise). */
function ensureWasm(): Promise<any> {
  if (!wasmReady) {
    if (!pkgUrl) throw new Error('mesh worker not initialized');
    const url = pkgUrl;
    wasmReady = (async () => {
      const mod = await import(/* @vite-ignore */ url);
      await mod.default();
      return mod;
    })();
  }
  return wasmReady;
}

function errorText(e: unknown): string {
  if (e instanceof Error) return e.message;
  return String(e);
}

self.onmessage = async (ev: MessageEvent) => {
  const msg = ev.data;
  if (msg.type === 'init') {
    pkgUrl = msg.pkgUrl;
    return;
  }
  if (msg.type === 'run') {
    try {
      const wasm = await ensureWasm();
      const fn = wasm[msg.fn];
      if (typeof fn !== 'function') throw new Error(`no wasm export ${msg.fn}`);
      const start = performance.now();
      const data = fn(...msg.args);
      const ms = performance.now() - start;
      const mesh = {
        positions: data.positions as Float32Array,
        indices: data.indices as Uint32Array,
        ghostPositions: data.has_ghost ? (data.ghost_positions as Float32Array) : null,
        ghostIndices: data.has_ghost ? (data.ghost_indices as Uint32Array) : null,
        numTri: data.num_tri as number,
        numVert: data.num_vert as number,
        volume: data.volume as number,
        surfaceArea: data.surface_area as number,
        closed: data.closed as boolean,
        openEdges: data.open_edges as number,
        note: data.note as string,
        data: data.data as string,
      };
      if (last) last.free();
      last = data;
      const transfer: Transferable[] = [mesh.positions.buffer, mesh.indices.buffer];
      if (mesh.ghostPositions && mesh.ghostIndices) {
        transfer.push(mesh.ghostPositions.buffer, mesh.ghostIndices.buffer);
      }
      (self as any).postMessage({ type: 'result', id: msg.id, ok: true, ms, mesh }, transfer);
    } catch (e) {
      // A trap (a Rust panic) leaves the instance unusable; the runner then starts a new worker.
      const fatal = e instanceof WebAssembly.RuntimeError;
      (self as any).postMessage({ type: 'result', id: msg.id, ok: false, fatal, error: errorText(e) });
    }
    return;
  }
  if (msg.type === 'call') {
    try {
      const wasm = await ensureWasm();
      const fn = wasm[msg.fn];
      if (typeof fn !== 'function') throw new Error(`no wasm export ${msg.fn}`);
      const start = performance.now();
      const value = fn(...msg.args);
      const ms = performance.now() - start;
      (self as any).postMessage({ type: 'call', id: msg.id, ok: true, value, ms });
    } catch (e) {
      const fatal = e instanceof WebAssembly.RuntimeError;
      (self as any).postMessage({ type: 'call', id: msg.id, ok: false, fatal, error: errorText(e) });
    }
    return;
  }
  if (msg.type === 'stl') {
    try {
      if (!last) throw new Error('nothing to export yet');
      const bytes = last.stl() as Uint8Array;
      (self as any).postMessage({ type: 'stl', id: msg.id, ok: true, bytes }, [bytes.buffer]);
    } catch (e) {
      (self as any).postMessage({ type: 'stl', id: msg.id, ok: false, error: errorText(e) });
    }
  }
};
