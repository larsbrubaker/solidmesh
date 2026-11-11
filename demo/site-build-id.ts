// The site build's identity, worked out where the site is built and baked into the bundle as
// `__SITE_BUILD__` (read by src/site-build.ts): build.ts uses it for the static site, server.ts for
// local runs, so the Ray Tracing page's "Copy timings" report can say which build a device ran.
// The commit comes from `git rev-parse` (CI checks the repo out), else from $GITHUB_SHA; when
// neither is there the identity says so instead of failing the build.
//
// What differs from the commit (`git status --porcelain`) is named, so the hash never claims code
// it isn't (`identityText`, tested in site-build-id.test.ts). Cargo.lock is apart: the deploy
// checks out each sibling's current main and wasm-pack runs cargo without --locked, so a sibling
// that moved ahead rewrites the lock on a clean checkout. That is said separately, as newer
// dependencies, and never as uncommitted changes to the site's source.

import { spawnSync } from "child_process";

/** The lock file cargo may rewrite during a build; not the site's source. */
const LOCK_FILE = "Cargo.lock";
const NO_COMMIT = "commit unknown (neither git nor GITHUB_SHA was available where the site was built)";

/** The short hash of the checked-out commit, or null when it can't be found. */
function commit(): string | null {
  const git = spawnSync("git", ["rev-parse", "--short", "HEAD"], { cwd: import.meta.dir, encoding: "utf-8" });
  const hash = git.status === 0 ? git.stdout.trim() : "";
  if (hash) return hash;
  const sha = process.env.GITHUB_SHA?.trim();
  return sha ? sha.slice(0, 7) : null;
}

/** The repo-relative paths `git status --porcelain` lists, a rename by its new path. */
export function changedPaths(porcelain: string): string[] {
  return porcelain.split("\n").filter(line => line.trim() !== "").map(line => {
    const renamed = line.replace(/\r$/, "").slice(3).split(" -> ");
    return renamed[renamed.length - 1].replace(/^"(.*)"$/, "$1");
  });
}

/** What differs from the commit; none where git can't say (no checkout). */
function workingTreeChanges(): string[] {
  const git = spawnSync("git", ["status", "--porcelain"], { cwd: import.meta.dir, encoding: "utf-8" });
  return git.status === 0 ? changedPaths(git.stdout) : [];
}

/**
 * The identity text: "abc1234, built <time>" for a site build or "abc1234, dev server, served
 * <time>" locally; "plus uncommitted changes" after the hash when the site's source differs from
 * the commit, and a separate note when the Cargo.lock cargo resolved does.
 */
export function identityText(hash: string | null, changed: string[], kind: "build" | "dev", time: string): string {
  const lockChanged = changed.includes(LOCK_FILE);
  const sourceChanged = changed.some(path => path !== LOCK_FILE);
  const head = hash === null ? NO_COMMIT : sourceChanged ? `${hash} plus uncommitted changes` : hash;
  const when = kind === "build" ? `built ${time}` : `dev server, served ${time}`;
  const lock = lockChanged ? "; built with newer dependencies than the commit's Cargo.lock names" : "";
  return `${head}, ${when}${lock}`;
}

export function siteBuildId(kind: "build" | "dev"): string {
  return identityText(commit(), workingTreeChanges(), kind, new Date().toISOString());
}

/** The `define` entry that bakes the identity into the code. */
export function siteBuildDefine(kind: "build" | "dev"): Record<string, string> {
  return { __SITE_BUILD__: JSON.stringify(siteBuildId(kind)) };
}
