// The Three.js viewer every page draws its result in: z up (the kernel's convention), orbit
// controls, flat shading with the solid's feature edges drawn dark, back faces in saffron so an
// inside-out patch or a hole shows at a glance, and an optional translucent ghost (a boolean's
// operands, a hull's source). The hero page places several results side by side with addMesh.
//
// preserveDrawingBuffer is on so src/capture.ts can read the canvas for thumbnails and the hero.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export const COLORS = {
  solid: 0x2a9d8f,
  back: 0xe9a23b,
  ghost: 0x8a94a6,
  edge: 0x14213d,
};

function geometry(positions: Float32Array, indices: Uint32Array): THREE.BufferGeometry {
  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geom.setIndex(new THREE.BufferAttribute(indices, 1));
  geom.computeBoundingBox();
  geom.computeBoundingSphere();
  return geom;
}

export interface ViewerOptions {
  /** Fixed canvas size in CSS pixels (the hero); otherwise the container's size. */
  width?: number;
  height?: number;
  pixelRatio?: number;
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

  constructor(private container: HTMLElement, private options: ViewerOptions = {}) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    this.renderer.setPixelRatio(options.pixelRatio ?? window.devicePixelRatio);
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

    if (options.width && options.height) {
      this.setSize(options.width, options.height);
    } else {
      this.resizeObserver = new ResizeObserver(() => this.fitContainer());
      this.resizeObserver.observe(container);
      this.fitContainer();
    }
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

  /** Renders one frame now, for a capture that cannot wait for the next animation frame. */
  renderNow() {
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

  /** Adds a result at `offset` beside the others (the hero's layout). */
  addMesh(positions: Float32Array, indices: Uint32Array, color: number, offset: [number, number, number]) {
    const solid = this.buildSolid(positions, indices, color);
    solid.position.set(...offset);
    this.content.add(solid);
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
  frame(direction: [number, number, number] = [0.55, -0.85, 0.6], tightness = 1.25) {
    const box = new THREE.Box3().setFromObject(this.content);
    if (box.isEmpty()) return;
    const sphere = box.getBoundingSphere(new THREE.Sphere());
    const radius = Math.max(sphere.radius, 1);
    const fov = (this.camera.fov * Math.PI) / 180;
    const fitHeight = radius / Math.sin(fov / 2);
    const fitWidth = fitHeight / Math.min(this.camera.aspect, 1);
    const dist = Math.max(fitHeight, fitWidth) * tightness;
    const dir = new THREE.Vector3(...direction).normalize();
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

  clear() {
    ThreeViewer.disposeGroup(this.content);
    ThreeViewer.disposeGroup(this.ghost);
    this.last = null;
  }

  dispose() {
    this.disposed = true;
    cancelAnimationFrame(this.animId);
    this.resizeObserver?.disconnect();
    this.clear();
    this.controls.dispose();
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }
}
