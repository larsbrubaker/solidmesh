// The main-thread side of src/mesh-worker.ts: one worker for the whole site, a promise per call,
// and per-page coalescing so a slider dragged faster than the kernel computes only ever queues
// the latest setting (a superseded request is dropped, never computed).
//
// `whenIdle()` resolves once nothing is computing or queued (`whenIdleWithin` gives up after a
// timeout); the thumbnail capture (src/capture.ts) waits on it instead of guessing how long a page
// takes. src/mesh-runner.test.ts tests the coalescing, the idle tracking and a side worker's trap.
//
// `callMesh` asks the same worker for a plain value; `SideWorker` is a second worker with its own
// wasm instance, for calls that must not queue behind a build (the Fillet page's hover picking),
// and `holdBusy` keeps the capture waiting while a page prepares its first request there. A side
// worker whose instance traps rejects its calls with `WorkerTrapped`, so a page knows what it kept
// there is gone, and starts a fresh instance on the next call.

export interface MeshResult {
  positions: Float32Array;
  indices: Uint32Array;
  ghostPositions: Float32Array | null;
  ghostIndices: Uint32Array | null;
  numTri: number;
  numVert: number;
  volume: number;
  surfaceArea: number;
  closed: boolean;
  openEdges: number;
  note: string;
  /** A page's own answer beside the mesh, as JSON (the Fillet page's rows); empty elsewhere. */
  data: string;
}

export interface RunResult {
  mesh: MeshResult;
  /** Wall time of the wasm call, measured in the worker. */
  ms: number;
}

type Pending = { resolve: (v: any) => void; reject: (e: Error) => void };

let worker: Worker | null = null;
let nextId = 1;
const pending = new Map<number, Pending>();
let inFlight = 0;
let idleWaiters: (() => void)[] = [];

function settleIdle() {
  if (inFlight > 0) return;
  const waiters = idleWaiters;
  idleWaiters = [];
  for (const w of waiters) w();
}

/** A new worker running src/mesh-worker.ts, told where the wasm package is. */
function spawnWorker(): Worker {
  // In dev the server transpiles src/*.ts and import.meta.url ends in .ts; the build bundles the
  // worker as mesh-worker.js beside main.js.
  const url = import.meta.url.endsWith('.ts')
    ? new URL('./mesh-worker.ts', import.meta.url)
    : new URL('./mesh-worker.js', window.location.href);
  const w = new Worker(url, { type: 'module' });
  w.postMessage({
    type: 'init',
    pkgUrl: new URL('./public/pkg/solidmesh_wasm.js', window.location.href).href,
  });
  return w;
}

function ensureWorker(): Worker {
  if (worker) return worker;
  const w = spawnWorker();
  w.onmessage = (ev: MessageEvent) => {
    const msg = ev.data;
    const p = pending.get(msg.id);
    if (!p) return;
    pending.delete(msg.id);
    inFlight--;
    if (msg.ok) {
      if (msg.type === 'stl') p.resolve(msg.bytes);
      else if (msg.type === 'call') p.resolve(msg.value);
      else p.resolve({ mesh: msg.mesh, ms: msg.ms });
    }
    else {
      if (msg.fatal) restartWorker();
      p.reject(new Error(msg.error));
    }
    settleIdle();
  };
  w.onerror = (ev) => {
    console.error('mesh worker error', ev);
  };
  worker = w;
  return w;
}

/** Drops a worker whose wasm instance trapped; the next call starts a fresh one. */
function restartWorker() {
  worker?.terminate();
  worker = null;
  for (const [id, p] of pending) {
    p.reject(new Error('the wasm instance was restarted'));
    pending.delete(id);
    inFlight--;
  }
}

function post<T>(message: Record<string, unknown>): Promise<T> {
  const w = ensureWorker();
  const id = nextId++;
  inFlight++;
  return new Promise<T>((resolve, reject) => {
    pending.set(id, { resolve, reject });
    w.postMessage({ ...message, id });
  });
}

/** Runs wasm export `fn(...args)` in the worker. */
export function runMesh(fn: string, args: unknown[]): Promise<RunResult> {
  return post<RunResult>({ type: 'run', fn, args });
}

/** The worker's last result as binary STL bytes. */
export function lastResultStl(): Promise<Uint8Array> {
  return post<Uint8Array>({ type: 'stl' });
}

/** Runs wasm export `fn(...args)` in the worker for a plain value, in turn with its builds. */
export function callMesh<T>(fn: string, args: unknown[]): Promise<T> {
  return post<T>({ type: 'call', fn, args });
}

/**
 * A `SideWorker` call that ended because its wasm instance crashed. What the instance held is
 * gone, and the next call starts a fresh one with nothing in it; `instance` says which crashed.
 */
export class WorkerTrapped extends Error {
  constructor(message: string, readonly instance: number) {
    super(message);
  }
}

/**
 * A worker of its own with its own wasm instance, for calls that must not wait behind a build:
 * the Fillet page picks in one while the site's worker rounds the part. Not counted by
 * `whenIdle()`; `dispose()` ends it.
 */
export class SideWorker {
  private worker: Worker | null = null;
  private nextId = 1;
  private pending = new Map<number, Pending>();
  private instance = 0;

  // The worker to start; a test supplies its own.
  constructor(private readonly spawn: () => Worker = spawnWorker) {}

  private ensure(): Worker {
    if (this.worker) return this.worker;
    const w = this.spawn();
    this.instance += 1;
    w.onmessage = (ev: MessageEvent) => {
      const msg = ev.data;
      const p = this.pending.get(msg.id);
      if (!p) return;
      this.pending.delete(msg.id);
      if (msg.ok) p.resolve(msg.value);
      else if (msg.fatal) {
        const trapped = new WorkerTrapped(msg.error, this.instance);
        this.abandon(trapped);
        p.reject(trapped);
      } else p.reject(new Error(msg.error));
    };
    this.worker = w;
    return w;
  }

  /** Runs wasm export `fn(...args)` for a plain value; `transfer` lists buffers to hand over. */
  call<T>(fn: string, args: unknown[], transfer: Transferable[] = []): Promise<T> {
    const w = this.ensure();
    const id = this.nextId++;
    return new Promise<T>((resolve, reject) => {
      this.pending.set(id, { resolve, reject });
      w.postMessage({ type: 'call', id, fn, args }, transfer);
    });
  }

  /** Ends the worker and rejects every call still waiting on it with `error`. */
  private abandon(error: Error) {
    this.worker?.terminate();
    this.worker = null;
    for (const [id, p] of this.pending) {
      p.reject(error);
      this.pending.delete(id);
    }
  }

  dispose() {
    this.abandon(new Error('the side worker was stopped'));
  }
}

/**
 * Counts as computing until the returned release is called: a page that prepares its first
 * request elsewhere (the Fillet page picks its starting edges in a `SideWorker`) holds the
 * capture off until that request is queued.
 */
export function holdBusy(): () => void {
  inFlight++;
  let released = false;
  return () => {
    if (released) return;
    released = true;
    inFlight--;
    settleIdle();
  };
}

/** Resolves once nothing is computing or queued. */
export function whenIdle(): Promise<void> {
  if (inFlight === 0) return Promise.resolve();
  return new Promise(resolve => idleWaiters.push(resolve));
}

/**
 * True once nothing is computing or queued, false if that has not happened within `ms`: a
 * capture waiting on a page whose request never returns moves on instead of hanging.
 */
export function whenIdleWithin(ms: number): Promise<boolean> {
  return new Promise(resolve => {
    const timer = setTimeout(() => resolve(false), ms);
    whenIdle().then(() => {
      clearTimeout(timer);
      resolve(true);
    });
  });
}

/**
 * One page's request queue: at most one call in flight and one waiting; a newer request
 * replaces the waiting one. Results of a disposed page are dropped. `R` is what `run` answers: a
 * `RunResult` for a mesh build, or (the Ray Tracing page) whatever its side worker returns.
 */
export class Coalescer<R = RunResult> {
  private running = false;
  private next: { fn: string; args: unknown[] } | null = null;
  private disposed = false;

  constructor(
    private onResult: (r: R) => void,
    private onError: (e: Error) => void,
    private onBusy: (busy: boolean) => void,
    // The call that computes a request; the worker unless a test supplies its own.
    private run: (fn: string, args: unknown[]) => Promise<R> = runMesh as unknown as (fn: string, args: unknown[]) => Promise<R>,
  ) {}

  request(fn: string, args: unknown[]) {
    this.next = { fn, args };
    if (!this.running) this.pump();
  }

  private async pump() {
    if (!this.next || this.disposed) return;
    const job = this.next;
    this.next = null;
    this.running = true;
    this.onBusy(true);
    // Counted as in flight across the gap between two queued jobs, so whenIdle() cannot resolve
    // between them.
    inFlight++;
    try {
      const result = await this.run(job.fn, job.args);
      if (!this.disposed) this.onResult(result);
    } catch (e) {
      if (!this.disposed) this.onError(e as Error);
    } finally {
      this.running = false;
      if (this.next && !this.disposed) this.pump();
      else if (!this.disposed) this.onBusy(false);
      inFlight--;
      settleIdle();
    }
  }

  dispose() {
    this.disposed = true;
    this.next = null;
  }
}
