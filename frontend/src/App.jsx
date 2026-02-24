import { useRef } from "react"
import Viewer from "./viewer/Viewer"

function App() {
  const viewerRef = useRef(null)

  return (
    <div style={{ width: "100vw", height: "100vh" }}>
      
      <div
        style={{
          position: "absolute",
          top: 20,
          left: 20,
          zIndex: 10,
          display: "flex",
          gap: "10px"
        }}
      >
        <button onClick={() => viewerRef.current?.sendService(true)}>
          Robot ON
        </button>

        <button onClick={() => viewerRef.current?.sendService(false)}>
          Robot OFF
        </button>
      </div>

      <Viewer ref={viewerRef} />

    </div>
  )
}

export default App