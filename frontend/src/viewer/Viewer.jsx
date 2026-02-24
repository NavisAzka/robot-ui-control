import { forwardRef, useEffect, useRef, useImperativeHandle } from "react"
import SceneManager from "./SceneManager"

const Viewer = forwardRef((props, ref) => {
  const containerRef = useRef()
  const sceneRef = useRef()

  useEffect(() => {
    sceneRef.current = new SceneManager(containerRef.current)

    return () => {
      sceneRef.current?.dispose()
    }
  }, [])

  useImperativeHandle(ref, () => ({
    sendService(value) {
      sceneRef.current?.sendService(value)
    }
  }))

  return (
    <div
      ref={containerRef}
      style={{ width: "100%", height: "100%" }}
    />
  )
})

export default Viewer