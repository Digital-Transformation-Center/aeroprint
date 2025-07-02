import React, { useRef, useState, useEffect } from "react";
import { Canvas, useLoader } from "@react-three/fiber";
import { OrbitControls, Edges, Bounds } from "@react-three/drei";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader";
import PropTypes from "prop-types"; 

function STLModel({ fileURL, finalColor }) {
  const geometry = useLoader(STLLoader, fileURL);
  const meshRef = useRef();
  const [revealCount, setRevealCount] = useState(0);
  const totalVertices = geometry.attributes.position.count;

  useEffect(() => {
    geometry.setDrawRange(0, 0);
    let start = performance.now();
    let frameId;

    const animate = (now) => {
      const elapsed = now - start;
      const progress = Math.min(elapsed / 5000, 1);
      const currentCount = Math.floor(progress * totalVertices);

      geometry.setDrawRange(0, currentCount);
      setRevealCount(currentCount);

      if (progress < 1) {
        frameId = requestAnimationFrame(animate);
      } else {
        geometry.setDrawRange(0, totalVertices);
        setRevealCount(totalVertices);
      }
    };

    frameId = requestAnimationFrame(animate);
    return () => cancelAnimationFrame(frameId);
  }, [geometry, totalVertices]);

  return (
    <mesh ref={meshRef} geometry={geometry}>
      <meshStandardMaterial
        color={revealCount === totalVertices ? finalColor : "#0070f3"}
        transparent
        opacity={0.95}
        polygonOffset
        polygonOffsetFactor={-1}
      />
      <Edges threshold={15} color="white" />
    </mesh>
  );
}

export default function LiveModel({ fileURL, color = "#00ffc6" }) {
  return (
    <div style={{ height: "500px", width: "100%", marginTop: "2rem" }}>
      <Canvas camera={{ position: [4 , 4, 4] }}>
        <ambientLight intensity={0.3} />
        <directionalLight position={[5, 10, 7.5]} intensity={1.5} />
        <spotLight
          position={[0, 3, 2]}
          angle={0.3}
          penumbra={1}
          intensity={2}
          castShadow
        />

        <OrbitControls
          enableZoom={true}
          enableRotate={true}
          enableDamping
          dampingFactor={0.1}
          minDistance={0.1}
          maxDistance={500}
          rotateSpeed={1.5}
        />

        <OrbitControls target={[0, 0, 0]} />

        <Bounds observe margin={1.5} fit>
          <STLModel fileURL={fileURL} finalColor={color} />
        </Bounds>
      </Canvas>
    </div>
  );
}
