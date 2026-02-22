import * as THREE from "three"

export default class PointCloudManager {
  constructor(scene, maxPoints = 200000) {
    this.scene = scene;
    this.maxPoints = maxPoints;

    // Pre-allocate buffer besar supaya tidak realloc terus
    this.positions = new Float32Array(this.maxPoints * 3);

    this.geometry = new THREE.BufferGeometry();
    this.geometry.setAttribute(
      "position",
      new THREE.BufferAttribute(this.positions, 3)
    );

    // Awalnya tidak gambar apa-apa
    this.geometry.setDrawRange(0, 0);

    this.material = new THREE.PointsMaterial({
      size: 0.05,
      color: 0x00ffcc,
    });

    this.points = new THREE.Points(this.geometry, this.material);
    this.scene.add(this.points);
  }

  // Ini dipanggil saat WebSocket terima data ROS
  update(pointArray) {
    const count = Math.min(pointArray.length / 3, this.maxPoints);

    for (let i = 0; i < count * 3; i++) {
      this.positions[i] = pointArray[i];
    }

    this.geometry.setDrawRange(0, count);
    this.geometry.attributes.position.needsUpdate = true;
  }

  dispose() {
    this.geometry.dispose();
    this.material.dispose();
    this.scene.remove(this.points);
  }
}
