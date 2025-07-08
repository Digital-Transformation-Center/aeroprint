import React, { useState, useEffect, useMemo } from "react";
import { io } from "socket.io-client";
import styled from "styled-components";
import ScanUpload from "../components/ScanUpload";
import { useScans } from "../context/ScanContext";
import LiveModel from "../components/LiveModel";
import { Link } from "react-router-dom";

const Container = styled.div`
  padding: 40px;
  color: ${(p) => p.theme.colors.text};
`;

function Scan() {
  const { addScan } = useScans();
  const [scanFile, setScanFile] = useState(null);
  const [scanURL, setScanURL] = useState("");
  const [uploading, setUploading] = useState(false);
  const [printing, setPrinting] = useState(false);
  const [status, setStatus] = useState("");
  const [progress, setProgress] = useState(0);

  useEffect(() => {
    const socket = io("http://localhost:4000");
    socket.on("scan-progress", (data) => setStatus(data.message));
    socket.on("scan-complete", () =>
      setStatus("✅ Real scan complete! Ready for upload.")
    );
    return () => socket.disconnect();
  }, []);

  const handleStartScan = async () => {
    setStatus("🛫 Launching drone scan...");
    try {
      await fetch("http://localhost:4000/start-scan", { method: "POST" });
      setStatus("📡 Scan initiated. Listening for updates...");
    } catch (err) {
      console.error(err);
      setStatus("❌ Failed to start scan.");
    }
  };

  const handleScanUpload = async (file) => {
    const formData = new FormData();
    formData.append("scan", file);
    setUploading(true);
    setStatus("");
    setScanFile(file);
    setProgress(0);

    try {
      const res = await fetch("http://localhost:4000/upload", {
        method: "POST",
        body: formData,
      });
      const data = await res.json();
      setScanURL(data.url);
      addScan({
        name: file.name,
        url: data.url,
        timestamp: new Date().toISOString(),
        color: "#00ffc6",
      });
      setStatus("🛰️ Scan received. Ready to print.");
    } catch (err) {
      console.error(err);
      setStatus("❌ Upload failed.");
    } finally {
      setUploading(false);
    }
  };

  const handlePrint = async () => {
    if (!scanURL) return;
    setPrinting(true);
    setStatus("🖨️ Starting print...");
    setProgress(0);

    const filename = scanURL.split("/").pop();
    try {
      await fetch("http://localhost:4000/print", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ filename }),
      });
    } catch (err) {
      console.error("Backend print call failed:", err);
    }

    let current = 0;
    const interval = setInterval(() => {
      current += Math.floor(Math.random() * 10) + 5;
      if (current >= 100) {
        current = 100;
        clearInterval(interval);
        setStatus("✅ Print complete!");
        setPrinting(false);
      }
      setProgress(current);
    }, 300);
  };

  const previewURL = useMemo(() => {
    return scanFile ? URL.createObjectURL(scanFile) : null;
  }, [scanFile]);

  useEffect(() => {
    return () => {
      if (previewURL) URL.revokeObjectURL(previewURL);
    };
  }, [previewURL]);

  return (
    <Container>
      <Link
        to="/"
        style={{
          position: "absolute",
          top: "20px",
          right: "40px",
          textDecoration: "none",
          backgroundColor: "#1a1d24",
          color: "#00ffc6",
          padding: "10px 20px",
          borderRadius: "8px",
          fontWeight: "bold",
          boxShadow: "0 0 10px rgba(0,255,198,0.3)",
        }}
      >
        ⬅ Back to Home
      </Link>

      <h1>🚀 Launch Scan</h1>
      <p>Simulate uploading a 3D scan from the drone.</p>

      <ScanUpload onUpload={handleScanUpload} />
      {uploading && <p>Uploading scan...</p>}

      <button
        onClick={handleStartScan}
        style={{
          margin: "20px 0",
          padding: "12px 28px",
          fontSize: "16px",
          borderRadius: "8px",
          backgroundColor: "#0070f3",
          color: "white",
          border: "none",
          cursor: "pointer",
        }}
      >
        Start Drone Scan
      </button>

      {scanFile && (
        <div style={{ marginTop: "30px" }}>
          <h3>🖨️ Print Ready</h3>
          <p><strong>File:</strong> {scanFile.name}</p>

          <button
            onClick={handlePrint}
            disabled={printing}
            style={{
              marginTop: "12px",
              padding: "12px 24px",
              fontSize: "16px",
              borderRadius: "8px",
              backgroundColor: "#00ffc6",
              border: "none",
              cursor: "pointer",
            }}
          >
            {printing ? "Printing..." : "Send to Printer"}
          </button>

          {printing && (
            <div
              style={{ marginTop: "20px", width: "100%", maxWidth: "400px" }}
            >
              <div
                style={{
                  height: "12px",
                  backgroundColor: "#2c2f36",
                  borderRadius: "8px",
                  overflow: "hidden",
                }}
              >
                <div
                  style={{
                    width: `${progress}%`,
                    height: "100%",
                    backgroundColor: "#00ffc6",
                    transition: "width 0.3s ease",
                  }}
                />
              </div>
              <p style={{ marginTop: "8px" }}>{progress}%</p>
            </div>
          )}

          <div style={{ marginTop: "40px" }}>
            <LiveModel fileURL={previewURL} />
          </div>
        </div>
      )}

      {status && <p style={{ marginTop: "20px" }}>{status}</p>}
    </Container>
  );
}

export default Scan;
