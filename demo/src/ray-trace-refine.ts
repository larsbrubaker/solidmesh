// The Ray Tracing page's decisions about whether an image still counts, kept out of the page so
// they can be tested (src/ray-trace-refine.test.ts). They keep a page that has gone still at one
// steady refinement: a result computed for anything other than what the page now wants is not
// drawn, and a resize notification restarts the image only when its pixel size really changed.

export interface ImageSize {
  w: number;
  h: number;
}

export function sameSize(a: ImageSize, b: ImageSize): boolean {
  return a.w === b.w && a.h === b.h;
}

/** Whether a resize measured at `measured` is a real change from the image's `current` size. */
export function sizeChanged(current: ImageSize | null, measured: ImageSize): boolean {
  return current === null || !sameSize(current, measured);
}

/**
 * Whether a finished pass may be shown. While the camera moves every result is the next frame of
 * the movement; once still only a result for exactly the request being refined draws, so a
 * preview still in flight when the finger lifted, or a pass for an older camera, look or size,
 * cannot replace the refining image.
 */
export function mayDraw(passArgs: readonly unknown[], currentArgs: readonly unknown[] | null,
  moving: boolean): boolean {
  if (moving) return true;
  return currentArgs !== null && passArgs.length === currentArgs.length
    && passArgs.every((value, i) => value === currentArgs[i]);
}
