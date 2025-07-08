import React, { useRef, useState, useMemo, useEffect } from "react";
import { Canvas, useLoader, useThree } from "@react-three/fiber";
import { OrbitControls as DreiOrbitControls, Edges, Bounds } from "@react-three/drei";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader";
import * as THREE from "three";

// Safe OrbitControls wrapper to avoid camera-readiness crashes
function OrbitControlsWrapper() {
  const { camera, gl } = useThree();
  const controlsRef = useRef();
  useEffect(() => {
    if (controlsRef.current) controlsRef.current.update();
  }, []);
  return <DreiOrbitControls ref={controlsRef} args={[camera, gl.domElement]} />;
}

// STL mesh loader with normalized geometry and reactive updates
function STLModel({ fileURL, color = "#00ffc6", scale = 1 }) {
  const loader = useMemo(() => new STLLoader(), []);
  const meshRef = useRef();
  const [geometry, setGeometry] = useState(null);

  useEffect(() => {
    if (!fileURL) return;

    loader.load(
      fileURL,
      (rawGeometry) => {
        const geom = rawGeometry.clone();
        geom.center();
        geom.computeVertexNormals();
        geom.computeBoundingBox();
        geom.toNonIndexed();

        const size = geom.boundingBox.getSize(new THREE.Vector3());
        const maxAxis = Math.max(size.x, size.y, size.z);
        geom.scale(1 / maxAxis, 1 / maxAxis, 1 / maxAxis);

        setGeometry(geom);
      },
      undefined,
      (error) => {
        console.error("❌ Failed to load STL:", error);
        setGeometry(null);
      }
    );
  }, [fileURL, loader]);

  if (!geometry) return null;

  return (
    <mesh
      ref={meshRef}
      geometry={geometry}
      castShadow
      receiveShadow
      scale={[scale, scale, scale]}
    >
      <meshStandardMaterial
        color={color}
        flatShading
        metalness={0.1}
        roughness={0.7}
        side={THREE.DoubleSide}
      />
      <Edges threshold={15} color="white" />
    </mesh>
  );
}

// Main viewer canvas
export default function LiveModel({ fileURL, color = "#00ffc6", scale = 1 }) {
  return (
    <div style={{ height: "500px", width: "100%", marginTop: "1rem" }}>
      <Canvas shadows camera={{ position: [3, 3, 3], fov: 60 }}>
        <ambientLight intensity={0.6} />
        <directionalLight position={[5, 10, 7.5]} intensity={1.4} castShadow />
        <spotLight position={[0, 5, 2]} angle={0.35} penumbra={1} intensity={2.0} castShadow />
        <OrbitControlsWrapper />
        <Bounds observe margin={1.2} fit clip>
          <STLModel fileURL={fileURL} color={color} scale={scale} />
        </Bounds>
      </Canvas>
    </div>
  );
}
