import React, { useState } from "react";
import styled from "styled-components";
import ScanUpload from "../components/ScanUpload";

const Container = styled.div`
  padding: 40px;
  color: ${(p) => p.theme.colors.text};
`;

function Scan() {
  const [scanFile, setScanFile] = useState(null);
  const [scanURL, setScanURL] = useState("");
  const [uploading, setUploading] = useState(false);
  const [printing, setPrinting] = useState(false);
  const [status, setStatus] = useState("");

  const handleScanUpload = async (file) => {
    const formData = new FormData();
    formData.append("scan", file);
    setUploading(true);
    setStatus("");

    try {
      const res = await fetch("http://localhost:4000/upload", {
        method: "POST",
        body: formData,
      });
      const data = await res.json();
      setScanFile(file);
      setScanURL(data.url);
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
    setStatus("");

    const filename = scanURL.split("/").pop();

    try {
      const res = await fetch("http://localhost:4000/print", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ filename }),
      });
      const data = await res.json();
      setStatus("✅ " + data.message);
    } catch (err) {
      console.error(err);
      setStatus("❌ Print failed.");
    } finally {
      setPrinting(false);
    }
  };

  return (
    <Container>
      <h1>🚀 Launch Scan</h1>
      <p>Simulate uploading a 3D scan from the drone.</p>

      <ScanUpload onUpload={handleScanUpload} />
      {uploading && <p>Uploading scan...</p>}

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
        </div>
      )}

      {status && <p style={{ marginTop: "20px" }}>{status}</p>}
    </Container>
  );
}

export default Scan;
