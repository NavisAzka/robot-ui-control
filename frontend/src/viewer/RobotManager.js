import * as THREE from "three"

export default class RobotManager {
  constructor(scene) {
    this.scene = scene

    const bodyGeometry = new THREE.CylinderGeometry(0.7, 0.7, 1, 12)
    const bodyMaterial = new THREE.MeshStandardMaterial({
      color: 0xff4444
    })

    this.body = new THREE.Mesh(bodyGeometry, bodyMaterial)
    this.body.rotation.x = Math.PI / 2
    this.body.position.z = 0.15

    // Heading arrow
    const arrowGeometry = new THREE.ConeGeometry(0.3, 0.5, 16)
    const arrowMaterial = new THREE.MeshStandardMaterial({
      color: 0xffff00
    })

    this.arrow = new THREE.Mesh(arrowGeometry, arrowMaterial)
    this.arrow.position.set(0.8, 0, 0.15)
    this.arrow.rotation.z = -Math.PI / 2

    this.robot = new THREE.Group()
    this.robot.add(this.body)
    this.robot.add(this.arrow)

    scene.add(this.robot)

    // Lighting (biar kelihatan)
    const light = new THREE.DirectionalLight(0xffffff, 1)
    light.position.set(10, 10, 20)
    scene.add(light)
  }

  updatePose(x, y, yaw) {
    this.robot.position.set(x, y, 0)
    this.robot.rotation.z = yaw
  }

  dispose() {
    this.scene.remove(this.robot)
  }
}
