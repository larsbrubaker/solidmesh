// The Three.js viewer every page draws its result in: z up (the kernel's convention), orbit
// controls, flat shading with the solid's feature edges drawn dark, back faces in saffron so an
// inside-out patch or a hole shows at a glance, and an optional translucent ghost (a boolean's
// operands, a hull's source). The Fillet page also draws overlays on it (edges as tubes, corners
// as balls, `setTubes` / `setBalls`) and casts the pointer's ray into the part (`rayAt`).
//
// The newest live viewer is `activeViewer()`: src/capture.ts draws it with `renderNow` and reads
// its canvas for the thumbnail (preserveDrawingBuffer keeps the frame readable).

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export const COLORS = {
  solid: 0x2a9d8f,
  back: 0xe9a23b,
  ghost: 0x8a94a6,
  edge: 0x14213d,
};

let active: ThreeViewer | null = null;

/** The viewer of the page on screen, or null between pages. */
export function activeViewer(): ThreeViewer | null {
  return active;
}

function geometry(positions: Float32Array, indices: Uint32Array): THREE.BufferGeometry {
  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geom.setIndex(new THREE.BufferAttribute(indices, 1));
  geom.computeBoundingBox();
  geom.computeBoundingSphere();
  return geom;
}

export class ThreeViewer {
  private renderer: THREE.WebGLRenderer;
  private scene = new THREE.Scene();
  private camera: THREE.PerspectiveCamera;
  private controls: OrbitControls;
  private content = new THREE.Group();
  private ghost = new THREE.Group();
  private grid: THREE.GridHelper | null = null;
  private animId = 0;
  private disposed = false;
  private hasFramed = false;
  private showEdges = true;
  private showWireframe = false;
  private resizeObserver: ResizeObserver | null = null;

  constructor(private container: HTMLElement) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setClearColor(0xf4f6f9);
    container.appendChild(this.renderer.domElement);

    THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
    this.camera = new THREE.PerspectiveCamera(40, 1, 0.1, 5000);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(60, -90, 60);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.1;

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x8899aa, 1.4));
    const key = new THREE.DirectionalLight(0xffffff, 1.6);
    key.position.set(40, -60, 100);
    this.scene.add(key);
    const rim = new THREE.DirectionalLight(0xffffff, 0.5);
    rim.position.set(-60, 80, -20);
    this.scene.add(rim);
    this.scene.add(this.content);
    this.scene.add(this.ghost);

    this.resizeObserver = new ResizeObserver(() => this.fitContainer());
    this.resizeObserver.observe(container);
    this.fitContainer();
    active = this;
    this.animate();
  }

  private setSize(w: number, h: number) {
    this.renderer.setSize(w, h);
    this.camera.aspect = w / Math.max(h, 1);
    this.camera.updateProjectionMatrix();
  }

  private fitContainer() {
    this.setSize(this.container.clientWidth, this.container.clientHeight);
  }

  private animate() {
    if (this.disposed) return;
    this.animId = requestAnimationFrame(() => this.animate());
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  /** Renders one frame now, at the container's current size, for a capture that cannot wait for
   *  an animation frame (a hidden page gets none). */
  renderNow() {
    this.fitContainer();
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  get canvas(): HTMLCanvasElement {
    return this.renderer.domElement;
  }

  private static disposeGroup(group: THREE.Group) {
    for (const child of [...group.children]) {
      group.remove(child);
      child.traverse((obj: THREE.Object3D) => {
        const m = obj as THREE.Mesh;
        if (m.geometry) m.geometry.dispose();
        const mat = m.material as THREE.Material | THREE.Material[] | undefined;
        if (Array.isArray(mat)) mat.forEach(x => x.dispose());
        else mat?.dispose();
      });
    }
  }

  private buildSolid(positions: Float32Array, indices: Uint32Array, color: number): THREE.Group {
    const geom = geometry(positions, indices);
    const group = new THREE.Group();
    const front = new THREE.MeshStandardMaterial({
      color, roughness: 0.45, metalness: 0.05, flatShading: true, side: THREE.FrontSide,
      polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1,
    });
    const back = new THREE.MeshStandardMaterial({
      color: COLORS.back, roughness: 0.6, flatShading: true, side: THREE.BackSide,
    });
    group.add(new THREE.Mesh(geom, front));
    group.add(new THREE.Mesh(geom, back));
    if (this.showWireframe) {
      const wire = new THREE.WireframeGeometry(geom);
      group.add(new THREE.LineSegments(wire, new THREE.LineBasicMaterial({
        color: COLORS.edge, transparent: true, opacity: 0.18,
      })));
    } else if (this.showEdges) {
      const edges = new THREE.EdgesGeometry(geom, 25);
      group.add(new THREE.LineSegments(edges, new THREE.LineBasicMaterial({
        color: COLORS.edge, transparent: true, opacity: 0.55,
      })));
    }
    return group;
  }

  private last: { positions: Float32Array; indices: Uint32Array; color: number } | null = null;

  /** Shows one result, replacing whatever was shown. */
  setMesh(positions: Float32Array, indices: Uint32Array, color = COLORS.solid) {
    ThreeViewer.disposeGroup(this.content);
    this.last = { positions, indices, color };
    this.content.add(this.buildSolid(positions, indices, color));
    if (!this.hasFramed) this.frame();
  }

  /** Shows the translucent ghost, or clears it with null. */
  setGhost(positions: Float32Array | null, indices: Uint32Array | null) {
    ThreeViewer.disposeGroup(this.ghost);
    if (!positions || !indices || indices.length === 0) return;
    const geom = geometry(positions, indices);
    const mat = new THREE.MeshBasicMaterial({
      color: COLORS.ghost, transparent: true, opacity: 0.12, depthWrite: false,
      side: THREE.DoubleSide,
    });
    this.ghost.add(new THREE.Mesh(geom, mat));
    this.ghost.add(new THREE.LineSegments(new THREE.EdgesGeometry(geom, 25),
      new THREE.LineBasicMaterial({ color: COLORS.ghost, transparent: true, opacity: 0.5 })));
  }

  setEdges(on: boolean) {
    this.showEdges = on;
    this.rebuild();
  }

  setWireframe(on: boolean) {
    this.showWireframe = on;
    this.rebuild();
  }

  private rebuild() {
    if (!this.last) return;
    const { positions, indices, color } = this.last;
    ThreeViewer.disposeGroup(this.content);
    this.content.add(this.buildSolid(positions, indices, color));
  }

  /** Aims the camera at everything shown, from a three-quarter view, and sizes the grid. */
  private frame() {
    const box = new THREE.Box3().setFromObject(this.content);
    if (box.isEmpty()) return;
    const sphere = box.getBoundingSphere(new THREE.Sphere());
    const radius = Math.max(sphere.radius, 1);
    const fov = (this.camera.fov * Math.PI) / 180;
    const fitHeight = radius / Math.sin(fov / 2);
    const fitWidth = fitHeight / Math.min(this.camera.aspect, 1);
    const dist = Math.max(fitHeight, fitWidth) * 1.25;
    const dir = new THREE.Vector3(0.55, -0.85, 0.6).normalize();
    this.controls.target.copy(sphere.center);
    this.camera.position.copy(sphere.center).addScaledVector(dir, dist);
    this.camera.near = dist / 100;
    this.camera.far = dist * 10;
    this.camera.updateProjectionMatrix();
    this.controls.update();

    if (this.grid) {
      this.scene.remove(this.grid);
      this.grid.geometry.dispose();
    }
    const size = Math.pow(10, Math.ceil(Math.log10(radius * 4)));
    this.grid = new THREE.GridHelper(size, 20, 0xc5ccd8, 0xe2e6ee);
    this.grid.rotation.x = Math.PI / 2;
    this.grid.position.set(sphere.center.x, sphere.center.y, box.min.z - 0.01);
    this.scene.add(this.grid);
    this.hasFramed = true;
  }

  /** Re-frames on the next mesh (a page whose result changes size a lot). */
  resetFrame() {
    this.hasFramed = false;
  }

  private overlays = new Map<string, THREE.Group>();

  /**
   * Draws `segments` (six numbers per segment, in the part's space) as tubes of `radius` mm, or
   * clears them with null: the Fillet page's feature edges, hovered pick and picks, each under its
   * own key. `onTop` draws them through the part, for a hover that must show behind a face.
   */
  setTubes(key: string, segments: Float32Array | number[] | null, color: number, radius: number,
    opts: { onTop?: boolean; opacity?: number } = {}) {
    this.clearOverlay(key);
    if (!segments || segments.length < 6) return;
    const count = Math.floor(segments.length / 6);
    const geom = new THREE.CylinderGeometry(1, 1, 1, 6, 1, true);
    const mat = new THREE.MeshBasicMaterial({
      color, transparent: (opts.opacity ?? 1) < 1 || !!opts.onTop, opacity: opts.opacity ?? 1,
      depthTest: !opts.onTop,
    });
    const tubes = new THREE.InstancedMesh(geom, mat, count);
    const a = new THREE.Vector3();
    const b = new THREE.Vector3();
    const up = new THREE.Vector3(0, 1, 0);
    const m = new THREE.Matrix4();
    const q = new THREE.Quaternion();
    const s = new THREE.Vector3();
    for (let i = 0; i < count; i++) {
      a.set(segments[i * 6], segments[i * 6 + 1], segments[i * 6 + 2]);
      b.set(segments[i * 6 + 3], segments[i * 6 + 4], segments[i * 6 + 5]);
      const dir = b.clone().sub(a);
      const length = dir.length();
      q.setFromUnitVectors(up, length > 0 ? dir.normalize() : up);
      s.set(radius, length, radius);
      m.compose(a.clone().add(b).multiplyScalar(0.5), q, s);
      tubes.setMatrixAt(i, m);
    }
    tubes.renderOrder = opts.onTop ? 10 : 1;
    this.overlayGroup(key).add(tubes);
  }

  /** Draws a ball of `radius` mm at each point (three numbers each), or clears them with null. */
  setBalls(key: string, points: Float32Array | number[] | null, color: number, radius: number) {
    this.clearOverlay(key);
    if (!points || points.length < 3) return;
    const count = Math.floor(points.length / 3);
    const balls = new THREE.InstancedMesh(new THREE.SphereGeometry(1, 12, 8),
      new THREE.MeshBasicMaterial({ color }), count);
    const m = new THREE.Matrix4();
    for (let i = 0; i < count; i++) {
      m.makeScale(radius, radius, radius);
      m.setPosition(points[i * 3], points[i * 3 + 1], points[i * 3 + 2]);
      balls.setMatrixAt(i, m);
    }
    this.overlayGroup(key).add(balls);
  }

  private overlayGroup(key: string): THREE.Group {
    let group = this.overlays.get(key);
    if (!group) {
      group = new THREE.Group();
      this.overlays.set(key, group);
      this.scene.add(group);
    }
    return group;
  }

  clearOverlay(key: string) {
    const group = this.overlays.get(key);
    if (group) ThreeViewer.disposeGroup(group);
  }

  /**
   * The ray under a pointer at client coordinates: its origin and unit direction in the scene's
   * (the part's) space, and how many mm one pixel spans at 1 mm from the eye.
   */
  rayAt(clientX: number, clientY: number): { origin: number[]; direction: number[]; pixelSize: number } {
    const rect = this.renderer.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((clientX - rect.left) / Math.max(rect.width, 1)) * 2 - 1,
      -((clientY - rect.top) / Math.max(rect.height, 1)) * 2 + 1,
    );
    const caster = new THREE.Raycaster();
    caster.setFromCamera(ndc, this.camera);
    const { origin, direction } = caster.ray;
    const pixelSize = (2 * Math.tan((this.camera.fov * Math.PI) / 360)) / Math.max(rect.height, 1);
    return { origin: [origin.x, origin.y, origin.z], direction: [direction.x, direction.y, direction.z], pixelSize };
  }

  /** The radius of the bounding sphere of what is shown, for sizing tubes and balls. */
  get contentRadius(): number {
    const box = new THREE.Box3().setFromObject(this.content);
    return box.isEmpty() ? 10 : box.getBoundingSphere(new THREE.Sphere()).radius;
  }

  clear() {
    ThreeViewer.disposeGroup(this.content);
    ThreeViewer.disposeGroup(this.ghost);
    for (const group of this.overlays.values()) ThreeViewer.disposeGroup(group);
    this.last = null;
  }

  dispose() {
    this.disposed = true;
    if (active === this) active = null;
    cancelAnimationFrame(this.animId);
    this.resizeObserver?.disconnect();
    this.clear();
    this.controls.dispose();
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }
}
