// The main-thread side of src/mesh-worker.ts: one worker for the whole site, a promise per call,
// and per-page coalescing so a slider dragged faster than the kernel computes only ever queues
// the latest setting (a superseded request is dropped, never computed).
//
// `whenIdle()` resolves once nothing is computing or queued (`whenIdleWithin` gives up after a
// timeout); the thumbnail capture (src/capture.ts) waits on it instead of guessing how long a page
// takes. src/mesh-runner.test.ts tests the coalescing and the idle tracking.

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

function ensureWorker(): Worker {
  if (worker) return worker;
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
  w.onmessage = (ev: MessageEvent) => {
    const msg = ev.data;
    const p = pending.get(msg.id);
    if (!p) return;
    pending.delete(msg.id);
    inFlight--;
    if (msg.ok) p.resolve(msg.type === 'stl' ? msg.bytes : { mesh: msg.mesh, ms: msg.ms });
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
 * replaces the waiting one. Results of a disposed page are dropped.
 */
export class Coalescer {
  private running = false;
  private next: { fn: string; args: unknown[] } | null = null;
  private disposed = false;

  constructor(
    private onResult: (r: RunResult) => void,
    private onError: (e: Error) => void,
    private onBusy: (busy: boolean) => void,
    // The call that computes a request; the worker unless a test supplies its own.
    private run: (fn: string, args: unknown[]) => Promise<RunResult> = runMesh,
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
