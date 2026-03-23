"use client";

import { useRef, useMemo } from "react";
import { Canvas, useFrame, useLoader } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import * as THREE from "three";

function STLModel() {
  const rawGeometry = useLoader(STLLoader, "/hilbert-resistor.stl");
  const meshRef = useRef<THREE.Mesh>(null!);

  const { geo, scale } = useMemo(() => {
    const geo = rawGeometry.clone();
    geo.computeBoundingBox();
    geo.center();

    const bbox = geo.boundingBox!;
    const size = new THREE.Vector3();
    bbox.getSize(size);
    const maxDim = Math.max(size.x, size.y, size.z);
    const scale = (3 / maxDim) * 1.3 * 1.3;

    return { geo, scale };
  }, [rawGeometry]);

  useFrame((_state, delta) => {
    if (meshRef.current) {
      meshRef.current.rotation.z += delta * 0.3;
    }
  });

  return (
    <mesh ref={meshRef} geometry={geo} scale={scale}>
      <meshStandardMaterial color="#1245A8" flatShading />
    </mesh>
  );
}

export default function ModelViewer() {
  return (
    <div className="absolute inset-[-25%] h-[150%] w-[150%]" style={{ cursor: "grab" }}>
      <Canvas
        camera={{ position: [3, 2, 5], fov: 50 }}
        gl={{ alpha: true }}
        style={{ background: "transparent" }}
      >
        <ambientLight intensity={0.5} />
        <spotLight
          position={[10, 10, 10]}
          angle={0.15}
          penumbra={1}
          decay={0}
          intensity={Math.PI}
        />
        <pointLight
          position={[-10, -10, -10]}
          decay={0}
          intensity={Math.PI}
        />
        <STLModel />
        <OrbitControls enableZoom={false} enablePan={false} />
      </Canvas>
    </div>
  );
}
