// Tests for src/mesh-runner.ts's request coalescing and idle tracking, which the thumbnail capture
// (src/capture.ts) waits on: the capture must see "idle" exactly when every queued request has
// been computed and shown, and must get a timeout rather than a hang when one never finishes.
// Run with `bun test` from demo/; the runner is injected, so no worker or wasm is involved.

import { describe, expect, test } from 'bun:test';
import { Coalescer, SideWorker, WorkerTrapped, whenIdleWithin, type RunResult } from './mesh-runner.ts';

/** A runner whose calls finish only when the test says so. */
function manualRunner() {
  const calls: { fn: string; finish: (ok: boolean) => void }[] = [];
  const run = (fn: string) => new Promise<RunResult>((resolve, reject) => {
    calls.push({
      fn,
      finish: ok => (ok ? resolve({ ms: 1, mesh: {} as RunResult['mesh'] }) : reject(new Error('boom'))),
    });
  });
  return { calls, run };
}

const tick = () => new Promise(r => setTimeout(r, 0));

describe('Coalescer and idle tracking', () => {
  test('idle once the running request and the latest queued one are done', async () => {
    const { calls, run } = manualRunner();
    const shown: string[] = [];
    const c = new Coalescer(() => shown.push('result'), () => shown.push('error'), () => {}, run);
    c.request('a', []);
    c.request('b', []);
    c.request('c', []);
    expect(await whenIdleWithin(20)).toBe(false);
    calls[0].finish(true);
    await tick();
    // 'b' was superseded by 'c' while 'a' ran: it is never computed.
    expect(calls.map(x => x.fn)).toEqual(['a', 'c']);
    expect(await whenIdleWithin(20)).toBe(false);
    calls[1].finish(true);
    expect(await whenIdleWithin(1000)).toBe(true);
    expect(shown).toEqual(['result', 'result']);
  });

  test('a failed request still leaves the runner idle', async () => {
    const { calls, run } = manualRunner();
    const c = new Coalescer(() => {}, () => {}, () => {}, run);
    c.request('a', []);
    calls[0].finish(false);
    expect(await whenIdleWithin(1000)).toBe(true);
  });

  test('a page disposed mid-request is idle once its request returns', async () => {
    const { calls, run } = manualRunner();
    const c = new Coalescer(() => {}, () => {}, () => {}, run);
    c.request('a', []);
    c.dispose();
    calls[0].finish(true);
    expect(await whenIdleWithin(1000)).toBe(true);
  });

  test('a dropped waiting request is never computed, and only a call to the named function drops', async () => {
    const { calls, run } = manualRunner();
    const shown: string[] = [];
    const c = new Coalescer(() => shown.push('result'), () => shown.push('error'), () => {}, run);
    c.request('prepare', []);
    c.request('pass', []);
    c.dropWaiting('other');
    c.dropWaiting('pass');
    calls[0].finish(true);
    expect(await whenIdleWithin(1000)).toBe(true);
    expect(calls.map(x => x.fn)).toEqual(['prepare']);
    expect(shown).toEqual(['result']);
    c.request('pass', []);
    c.request('prepare', []);
    c.dropWaiting('pass');
    calls[1].finish(true);
    await tick();
    expect(calls.map(x => x.fn)).toEqual(['prepare', 'pass', 'prepare']);
    calls[2].finish(true);
    expect(await whenIdleWithin(1000)).toBe(true);
  });

  test('a request that never finishes times out instead of hanging', async () => {
    const { calls, run } = manualRunner();
    const c = new Coalescer(() => {}, () => {}, () => {}, run);
    c.request('never', []);
    expect(await whenIdleWithin(30)).toBe(false);
    // Let it go, so the module's idle count is clean for whatever runs next.
    c.dispose();
    calls[0].finish(true);
    expect(await whenIdleWithin(1000)).toBe(true);
  });
});

describe('SideWorker after a trap', () => {
  /** A worker that answers only when told to. */
  function fakeWorkers() {
    const made: { sent: any[]; onmessage: ((ev: any) => void) | null; terminated: boolean; postMessage: (m: any) => void; terminate: () => void }[] = [];
    const spawn = () => {
      const w = {
        sent: [] as any[],
        onmessage: null as ((ev: any) => void) | null,
        terminated: false,
        postMessage(m: any) { this.sent.push(m); },
        terminate() { this.terminated = true; },
      };
      made.push(w);
      return w as unknown as Worker;
    };
    return { made, spawn };
  }

  test('a fatal answer rejects as WorkerTrapped, naming the instance, and the next call has a new worker', async () => {
    const { made, spawn } = fakeWorkers();
    const side = new SideWorker(spawn);
    const call = side.call('pack', []);
    made[0].onmessage!({ data: { id: made[0].sent[0].id, ok: false, fatal: true, error: 'unreachable' } });
    const error = await call.catch(e => e);
    expect(error).toBeInstanceOf(WorkerTrapped);
    const first = (error as WorkerTrapped).instance;
    expect(made[0].terminated).toBe(true);

    const next = side.call('prepare', []);
    expect(made.length).toBe(2);
    made[1].onmessage!({ data: { id: made[1].sent[0].id, ok: false, fatal: true, error: 'unreachable' } });
    expect(((await next.catch(e => e)) as WorkerTrapped).instance).not.toBe(first);
  });

  test('other calls waiting on the trapped worker are told it trapped too', async () => {
    const { made, spawn } = fakeWorkers();
    const side = new SideWorker(spawn);
    const trapped = side.call('pack', []);
    const waiting = side.call('decode', []);
    made[0].onmessage!({ data: { id: made[0].sent[0].id, ok: false, fatal: true, error: 'unreachable' } });
    await trapped.catch(() => {});
    expect(await waiting.catch(e => e)).toBeInstanceOf(WorkerTrapped);
  });

  test('an ordinary refusal is not a trap', async () => {
    const { made, spawn } = fakeWorkers();
    const side = new SideWorker(spawn);
    const call = side.call('pack', []);
    made[0].onmessage!({ data: { id: made[0].sent[0].id, ok: false, fatal: false, error: 'no' } });
    const error = await call.catch(e => e);
    expect(error).not.toBeInstanceOf(WorkerTrapped);
    expect((error as Error).message).toBe('no');
    expect(made[0].terminated).toBe(false);
  });
});
