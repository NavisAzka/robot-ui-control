import { useEffect, useRef } from "react"
import SceneManager from "./SceneManager"

export default function Viewer() {
  const mountRef = useRef(null)
  const sceneRef = useRef(null)

  useEffect(() => {
    if (!mountRef.current) return

    sceneRef.current = new SceneManager(mountRef.current)

    return () => {
      if (sceneRef.current) {
        sceneRef.current.dispose()
      }
    }
  }, [])

  return (
    <div
      ref={mountRef}
      style={{
        width: "100%",
        height: "100%",
        overflow: "hidden"
      }}
    />
  )
}
