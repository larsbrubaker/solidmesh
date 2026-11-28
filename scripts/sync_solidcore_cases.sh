#!/usr/bin/env bash
# Brings the vendored solidcore fillet and chamfer cases
# (crates/geometry-processing/tests/solidcore_cases/) up to a solidcore commit, then replays them.
#
# It takes solidcore's fixtures/solidcraft/ at that commit (read straight from git objects, so
# the checkout's working tree is never touched), and the `sync_solidcore_cases` example does the
# file work: copy in the files that are new or changed and hold a fillet or chamfer case, delete
# the vendored files gone upstream with their known-failing and unported entries, and rewrite
# pin.json. It refuses an upstream with no case files, a sync that would remove more than half the
# vendored files (unless --allow-mass-removal) and file or case names that are not plain. Then the
# `solidcore_cases` test runs and writes its results as JSON (SOLIDCORE_CASES_RESULTS), and the
# example writes an `untriaged: ...` known-failing entry for each new or changed case that fails
# (a changed case keeps its old reason after the note), removes the entry of each listed case that
# now passes and holds the unported cases. A last run of the test shows the result. The report
# (files and cases added, changed and removed; newly passing; newly failing, untriaged; unported
# ops) is printed, and with --report also written as JSON for an automated weekly run. The
# example's state and the run's results stay in a temporary folder removed on exit. Nothing from
# solidcore is run: its files are only read.
#
# Usage, from anywhere (Git Bash on Windows):
#   scripts/sync_solidcore_cases.sh [--checkout DIR] [--rev REV | --pinned] [--report FILE]
#                                   [--allow-mass-removal]
#     --checkout DIR  a solidcore clone to read; without it one is cloned or fetched into
#                     $SOLIDCORE_CACHE (default $HOME/.cache/solidmesh/solidcore)
#     --rev REV       the commit to sync to (default origin/main)
#     --pinned        the commit pin.json already records: with nothing changed upstream at it,
#                     the sync changes no file (pin.json comes back byte for byte)
#     --report FILE   also write the report as JSON
#     --allow-mass-removal  let the sync remove more than half the vendored files
# CARGO_BUILD_JOBS is passed through to cargo as usual.

set -euo pipefail

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cases="$repo/crates/geometry-processing/tests/solidcore_cases"
checkout=""
rev="origin/main"
pinned=0
report=""
mass=()
while [ $# -gt 0 ]; do
    case "$1" in
        --checkout) checkout="$2"; shift 2 ;;
        --rev) rev="$2"; shift 2 ;;
        --pinned) pinned=1; shift ;;
        --report) report="$2"; shift 2 ;;
        --allow-mass-removal) mass=(--allow-mass-removal); shift ;;
        *) echo "unknown argument: $1 (see the header of $0)" >&2; exit 2 ;;
    esac
done

if [ -z "$checkout" ]; then
    checkout="${SOLIDCORE_CACHE:-$HOME/.cache/solidmesh/solidcore}"
    if [ -d "$checkout/.git" ]; then
        git -C "$checkout" fetch --quiet origin
    else
        mkdir -p "$(dirname "$checkout")"
        git clone --quiet https://github.com/tangency-labs/solidcore.git "$checkout"
    fi
fi
if [ "$pinned" = 1 ]; then
    rev="$(sed -n 's/^  "commit": "\([0-9a-f]*\)",$/\1/p' "$cases/pin.json")"
    [ -n "$rev" ] || { echo "pin.json records no commit" >&2; exit 1; }
fi

commit="$(git -C "$checkout" rev-parse --verify "$rev^{commit}")"
short="$(git -C "$checkout" rev-parse --short "$commit")"
date="$(TZ=UTC git -C "$checkout" show -s --format=%cd --date=format-local:%Y-%m-%dT%H:%M:%SZ "$commit")"

work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
mkdir -p "$work/solidcraft"
git -C "$checkout" ls-tree --name-only "$commit" fixtures/solidcraft/ | while read -r path; do
    case "$path" in
        *.json) git -C "$checkout" cat-file blob "$commit:$path" > "$work/solidcraft/$(basename "$path")" ;;
    esac
done

cd "$repo"
cargo run --quiet --example sync_solidcore_cases -- plan \
    --upstream "$work/solidcraft" --commit "$commit" --short "$short" --date "$date" \
    --state "$work/state.json" "${mass[@]}"
# The first run is expected to fail where cases moved; its results file is what apply reads.
SOLIDCORE_CASES_RESULTS="$work/results.json" cargo nextest run --workspace --test solidcore_cases --no-capture > "$work/run.txt" 2>&1 || true
if [ ! -s "$work/results.json" ]; then
    cat "$work/run.txt"
    echo "the solidcore_cases run wrote no results (its output is above): nothing applied" >&2
    exit 1
fi
apply=(apply --results "$work/results.json" --state "$work/state.json")
[ -n "$report" ] && apply+=(--report "$report")
cargo run --quiet --example sync_solidcore_cases -- "${apply[@]}"

echo
echo "after the sync:"
if cargo nextest run --workspace --test solidcore_cases --no-capture > "$work/final.txt" 2>&1; then
    grep ' cases: ' "$work/final.txt"
else
    grep ' cases: ' "$work/final.txt" || true
    sed -n '/cases moved:/,/^note:/p' "$work/final.txt"
    echo "the cases above still move: triage them by hand" >&2
    exit 1
fi
