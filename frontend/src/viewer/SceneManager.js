import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import PointCloudManager from "./PointCloudManager";
import RobotManager from "./RobotManager"
import { PLYLoader } from "three/examples/jsm/loaders/PLYLoader.js"

export default class SceneManager {
  constructor(container) {
    this.container = container;

    // 🔹 Scene dulu
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x202433);

    const width = container.clientWidth;
    const height = container.clientHeight;

    this.camera = new THREE.PerspectiveCamera(60, width / height, 0.1, 2000);

    this.camera.position.set(20, -20, 20);
    this.camera.lookAt(0, 0, 0);
    this.camera.up.set(0, 0, 1);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(width, height);
    this.renderer.setPixelRatio(window.devicePixelRatio);

    container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;

    // Grid
    const gridHelper = new THREE.GridHelper(50, 50);
    gridHelper.rotation.x = Math.PI / 2;
    gridHelper.material.opacity = 0.4;
    gridHelper.material.transparent = true;
    this.scene.add(gridHelper);

    const axesHelper = new THREE.AxesHelper(5);
    this.scene.add(axesHelper);


    // 🔹 Baru sekarang buat clock & pointcloud
    this.clock = new THREE.Clock();
    this.pointCloud = new PointCloudManager(this.scene);

    this.keys = {};

    window.addEventListener("keydown", (e) => {
      this.keys[e.key.toLowerCase()] = true;
    });

    window.addEventListener("keyup", (e) => {
      this.keys[e.key.toLowerCase()] = false;
    });

    // Robot manager
    this.robot = new RobotManager(this.scene)

    this.animate = this.animate.bind(this);
    this.onResize = this.onResize.bind(this);

    window.addEventListener("resize", this.onResize);

    // Load PLY model
    const loader = new PLYLoader(); 
    loader.load("/full_subsampled.ply", (geometry) => {
      geometry.computeVertexNormals();
      const material = new THREE.PointsMaterial({
        size: 0.05,
        vertexColors: true
      });
      const points = new THREE.Points(geometry, material);
      points.position.set(0, 0, 0);
      this.scene.add(points);
    });

    this.animate();
  }

  animate() {
    requestAnimationFrame(this.animate)

    const delta = this.clock.getDelta()
    const speed = 5
    const rotSpeed = 2

    let move = 0
    let rotate = 0

    if (this.keys["w"]) move = speed
    if (this.keys["s"]) move = -speed
    if (this.keys["a"]) rotate = rotSpeed
    if (this.keys["d"]) rotate = -rotSpeed

    // Simulasi pose
    this.robotYaw = (this.robotYaw || 0) + rotate * delta

    const dx = Math.cos(this.robotYaw) * move * delta
    const dy = Math.sin(this.robotYaw) * move * delta

    this.robotX = (this.robotX || 0) + dx
    this.robotY = (this.robotY || 0) + dy

    this.robot.updatePose(this.robotX, this.robotY, this.robotYaw)

    this.controls.update()
    this.renderer.render(this.scene, this.camera)
  }

  onResize() {
    const width = this.container.clientWidth;
    const height = this.container.clientHeight;

    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();

    this.renderer.setSize(width, height);
  }

  dispose() {
    window.removeEventListener("resize", this.onResize);

    if (this.controls) this.controls.dispose();

    if (this.renderer) {
      this.renderer.dispose();
      this.renderer.domElement.remove();
    }

    if (this.scene) this.scene.clear();
  }

  generateDummy(count) {
    const points = new Float32Array(count * 3);
    for (let i = 0; i < count; i++) {
      points[i * 3] = (Math.random() - 0.5) * 100; // x
      points[i * 3 + 1] = (Math.random() - 0.5) * 100; // y
      points[i * 3 + 2] = (Math.random() - 0.5) * 100; // z
    }
    return points;
  }
}
