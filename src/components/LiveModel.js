import React, { useRef } from "react";
import { Canvas, useFrame } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";
import * as THREE from "three";

function RotatingMesh({ progress }) {
  const meshRef = useRef();
  const scale = 1 + progress / 100; // grows from 1 → 2

  useFrame(() => {
    if (meshRef.current) {
      meshRef.current.rotation.y += 0.003;
      meshRef.current.rotation.x += 0.001;
    }
  });

  return (
    <mesh ref={meshRef} scale={[scale, scale, scale]}>
      <boxGeometry args={[2, 2, 2]} />
      <meshStandardMaterial color={progress < 100 ? "#00ffc6" : "#ff007a"} />
    </mesh>
  );
}

function LiveModel({ progress }) {
  return (
    <div style={{ height: "400px", width: "100%", marginTop: "2rem" }}>
      <Canvas camera={{ position: [4, 4, 4] }}>
        <ambientLight />
        <pointLight position={[10, 10, 10]} />
        <OrbitControls />
        <RotatingMesh progress={progress} />
      </Canvas>
    </div>
  );
}

export default LiveModel;
