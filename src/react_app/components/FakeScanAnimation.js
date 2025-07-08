import React, { useEffect, useState } from "react";

const SCAN_STEPS = [
  "🌀 Aligning sensors...",
  "📡 Capturing geometry...",
  "🧠 Reconstructing mesh...",
  "💾 Finalizing scan file...",
];

export default function FakeScanAnimation({ onComplete, duration = 3000 }) {
  const [currentStep, setCurrentStep] = useState(0);
  const [progress, setProgress] = useState(0);

  useEffect(() => {
    const interval = setInterval(() => {
      setCurrentStep((prev) => Math.min(prev + 1, SCAN_STEPS.length - 1));
      setProgress((prev) => Math.min(prev + 25, 100));
    }, duration / SCAN_STEPS.length);

    const timeout = setTimeout(() => {
      onComplete?.();
    }, duration);

    return () => {
      clearInterval(interval);
      clearTimeout(timeout);
    };
  }, [duration, onComplete]);

  return (
    <div style={{ textAlign: "center", padding: "3rem" }}>
      <p style={{ fontSize: "1.2rem", marginBottom: "1rem" }}>
        {SCAN_STEPS[currentStep]}
      </p>
      <div
        style={{
          height: "12px",
          background: "#222",
          borderRadius: "10px",
          overflow: "hidden",
          boxShadow: "0 0 8px #00ffc655",
        }}
      >
        <div
          style={{
            width: `${progress}%`,
            height: "100%",
            background: "linear-gradient(to right, #00ffc6, #0070f3)",
            transition: "width 0.4s ease-out",
          }}
        />
      </div>
    </div>
  );
}