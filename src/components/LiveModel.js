import React, { useRef, useState, useMemo, useEffect } from "react";
import { Canvas, useLoader, useThree } from "@react-three/fiber";
import { OrbitControls as DreiOrbitControls, Edges, Bounds } from "@react-three/drei";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader";
import * as THREE from "three";

// Safe OrbitControls wrapper
function OrbitControlsWrapper() {
  const { camera, gl } = useThree();
  const controlsRef = useRef();
  useEffect(() => {
    if (controlsRef.current) controlsRef.current.update();
  }, []);
  return <DreiOrbitControls ref={controlsRef} args={[camera, gl.domElement]} />;
}

// Model renderer that reacts to fileURL changes
function STLModel({ fileURL, finalColor }) {
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

        const size = geom.boundingBox.getSize(new THREE.Vector3());
        const maxAxis = Math.max(size.x, size.y, size.z);
        geom.scale(1 / maxAxis, 1 / maxAxis, 1 / maxAxis);

        geom.toNonIndexed();
        geom.computeVertexNormals();
        setGeometry(geom);
      },
      undefined,
      (error) => {
        console.error("Failed to load STL:", error);
        setGeometry(null);
      }
    );
  }, [fileURL, loader]);

  if (!geometry) return null;

  return (
    <mesh ref={meshRef} geometry={geometry} castShadow receiveShadow>
      <meshStandardMaterial
        color={finalColor}
        flatShading
        metalness={0.1}
        roughness={0.7}
        side={THREE.DoubleSide}
      />
      <Edges threshold={15} color="white" />
    </mesh>
  );
}

export default function LiveModel({ fileURL, color = "#00ffc6" }) {
  // Clean up object URL on unmount
  useEffect(() => {
    return () => {
      if (fileURL?.startsWith("blob:")) {
        URL.revokeObjectURL(fileURL);
      }
    };
  }, [fileURL]);

  return (
    <div style={{ height: "500px", width: "100%", marginTop: "2rem" }}>
      <Canvas shadows camera={{ position: [3, 3, 3], fov: 60 }}>
        <ambientLight intensity={0.6} />
        <directionalLight position={[5, 10, 7.5]} intensity={1.2} castShadow />
        <spotLight position={[0, 5, 2]} angle={0.35} penumbra={1} intensity={1.8} castShadow />
        <OrbitControlsWrapper />
        <Bounds observe margin={1.2} fit clip>
          <STLModel fileURL={fileURL} finalColor={color} />
        </Bounds>
      </Canvas>
    </div>
  );
}
