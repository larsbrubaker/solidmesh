// The Ray Tracing page's camera input (src/demos/ray-trace.ts is the page): drag to orbit, two
// fingers to orbit or pinch, the wheel to zoom. Pointer reports are collected and applied once
// per animation frame, so a two-finger gesture is one step (src/orbit-gesture.ts is the rule).
// Every applied change tells the page through `moved`, which starts a preview and the wait for
// the camera to be still.

import { OrbitGesture, applyDelta } from './orbit-gesture.ts';

export interface Camera {
  yaw: number;
  pitch: number;
  zoom: number;
}

/** Listens on `canvas` and moves `camera`; returns what stops a pending frame on leaving the page. */
export function attachCameraInput(canvas: HTMLCanvasElement, camera: Camera, moved: () => void): () => void {
  const gesture = new OrbitGesture();
  let gestureFrame: number | null = null;
  const applyGesture = () => {
    gestureFrame = null;
    const delta = gesture.take();
    if (delta.dx === 0 && delta.dy === 0 && delta.zoom === 1) return;
    Object.assign(camera, applyDelta(camera, delta));
    moved();
  };
  const scheduleGesture = () => {
    if (gestureFrame === null) gestureFrame = requestAnimationFrame(applyGesture);
  };
  const showGrab = () => {
    canvas.style.cursor = gesture.fingers > 0 ? 'grabbing' : 'grab';
  };
  canvas.addEventListener('pointerdown', e => {
    gesture.down(e.pointerId, e.clientX, e.clientY);
    // a pointer the browser no longer tracks cannot be captured, which must not end the gesture
    try { canvas.setPointerCapture(e.pointerId); } catch { /* not an active pointer */ }
    scheduleGesture();
    showGrab();
  });
  canvas.addEventListener('pointermove', e => {
    if (!gesture.has(e.pointerId)) return;
    gesture.move(e.pointerId, e.clientX, e.clientY);
    scheduleGesture();
  });
  const endPointer = (e: PointerEvent) => {
    gesture.up(e.pointerId);
    scheduleGesture();
    showGrab();
  };
  canvas.addEventListener('pointerup', endPointer);
  canvas.addEventListener('pointercancel', endPointer);
  // Lifted outside the canvas with capture lost: the pointer must not stay "down" forever.
  canvas.addEventListener('lostpointercapture', endPointer);
  canvas.addEventListener('wheel', e => {
    e.preventDefault();
    camera.zoom = Math.max(0.4, Math.min(8, camera.zoom * Math.exp(-e.deltaY * 0.0012)));
    moved();
  }, { passive: false });
  return () => {
    if (gestureFrame !== null) cancelAnimationFrame(gestureFrame);
  };
}
