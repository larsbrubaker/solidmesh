// Tests for src/mesh-runner.ts's request coalescing and idle tracking, which the thumbnail capture
// (src/capture.ts) waits on: the capture must see "idle" exactly when every queued request has
// been computed and shown, and must get a timeout rather than a hang when one never finishes.
// Run with `bun test` from demo/; the runner is injected, so no worker or wasm is involved.

import { describe, expect, test } from 'bun:test';
import { Coalescer, whenIdleWithin, type RunResult } from './mesh-runner.ts';

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
