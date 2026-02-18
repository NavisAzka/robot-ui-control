import * as THREE from "three"

export default class PointCloudManager {
  constructor(scene) {
    this.scene = scene

    const geometry = new THREE.BufferGeometry()

    const pointCount = 10000
    const radius = 5
    const height = 2

    const positions = new Float32Array(pointCount * 3)

    for (let i = 0; i < pointCount; i++) {
      const angle = Math.random() * Math.PI * 2
      const r = radius + (Math.random() - 0.5) * 0.2

      positions[i * 3 + 0] = Math.cos(angle) * r
      positions[i * 3 + 1] = Math.sin(angle) * r
      positions[i * 3 + 2] = (Math.random() - 0.5) * height
    }

    geometry.setAttribute(
      "position",
      new THREE.BufferAttribute(positions, 3)
    )

    const material = new THREE.PointsMaterial({
      size: 0.05,
      color: 0x00ffcc
    })

    this.points = new THREE.Points(geometry, material)
    this.scene.add(this.points)

    this.velocity = new THREE.Vector3()
  }

  update(delta) {
    this.points.position.addScaledVector(this.velocity, delta)
  }

  setVelocity(x, y) {
    this.velocity.x = x
    this.velocity.y = y
  }

  dispose() {
    this.points.geometry.dispose()
    this.points.material.dispose()
    this.scene.remove(this.points)
  }
}
