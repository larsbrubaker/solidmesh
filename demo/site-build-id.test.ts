// Tests of the site build's identity text (site-build-id.ts): a clean commit, a build whose only
// difference from the commit is the Cargo.lock cargo rewrote, and real uncommitted changes.

import { describe, expect, test } from "bun:test";
import { changedPaths, identityText } from "./site-build-id.ts";

const TIME = "2026-11-11T20:00:00.000Z";

describe("identityText", () => {
  test("a clean commit is the bare hash", () => {
    expect(identityText("abc1234", [], "build", TIME)).toBe("abc1234, built 2026-11-11T20:00:00.000Z");
    expect(identityText("abc1234", [], "dev", TIME)).toBe("abc1234, dev server, served 2026-11-11T20:00:00.000Z");
  });

  test("only Cargo.lock changed: the source is the commit's, the dependencies are newer", () => {
    expect(identityText("abc1234", ["Cargo.lock"], "build", TIME))
      .toBe("abc1234, built 2026-11-11T20:00:00.000Z; built with newer dependencies than the commit's Cargo.lock names");
  });

  test("other files changed: uncommitted changes, and the lock said apart", () => {
    expect(identityText("abc1234", ["demo/src/demos/ray-trace.ts"], "dev", TIME))
      .toBe("abc1234 plus uncommitted changes, dev server, served 2026-11-11T20:00:00.000Z");
    expect(identityText("abc1234", ["Cargo.lock", "demo/build.ts"], "build", TIME))
      .toBe("abc1234 plus uncommitted changes, built 2026-11-11T20:00:00.000Z; built with newer dependencies than the commit's Cargo.lock names");
  });

  test("no commit found says so", () => {
    expect(identityText(null, [], "build", TIME)).toStartWith("commit unknown (neither git nor GITHUB_SHA");
  });
});

describe("changedPaths", () => {
  test("reads modified, untracked, renamed and quoted entries", () => {
    expect(changedPaths(" M Cargo.lock\n?? demo/new.ts\nR  old.ts -> demo/moved.ts\n M \"demo/a b.ts\"\n"))
      .toEqual(["Cargo.lock", "demo/new.ts", "demo/moved.ts", "demo/a b.ts"]);
    expect(changedPaths("")).toEqual([]);
  });
});

test("changedPaths reads Windows line endings", () => {
  expect(changedPaths(" M Cargo.lock\r\n M demo/build.ts\r\n")).toEqual(["Cargo.lock", "demo/build.ts"]);
});
