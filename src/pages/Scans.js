import React, { useState, useEffect, useContext } from "react";
import { useScans } from "../context/ScanContext";
import Modal from "react-modal";
import LiveModel from "../components/LiveModel";
import { useNavigate, Link } from "react-router-dom";

Modal.setAppElement("#root");

function Scans() {
  const { scans } = useScans();
  const [previewURL, setPreviewURL] = useState(null);
  const navigate = useNavigate();

  return (
    <div style={{ padding: "2rem", color: "white" }}>
      <div style={{ marginBottom: "1rem" }}>
        <Link
          to="/"
          style={{
            backgroundColor: "#1a1d24",
            color: "#00ffc6",
            textDecoration: "none",
            padding: "8px 16px",
            borderRadius: "6px",
            boxShadow: "0 0 10px rgba(0,255,198,0.25)",
          }}
        >
          ⬅ Back to Home
        </Link>
      </div>

      <h2>📁 Recent Scans</h2>

      {scans.length === 0 ? (
        <p>No scans uploaded yet.</p>
      ) : (
        scans
          .slice()
          .reverse()
          .map((scan, index) => (
            <div
              key={index}
              style={{
                margin: "1rem 0",
                padding: "1rem",
                backgroundColor: "#2a2a2a",
                borderRadius: "8px",
              }}
            >
              <h4>{scan.name}</h4>
              <p>{new Date(scan.timestamp).toLocaleString()}</p>

              <div style={{ display: "flex", gap: "10px", flexWrap: "wrap" }}>
                <button
                  onClick={() => setPreviewURL(scan.url)}
                  style={{
                    padding: "8px 16px",
                    backgroundColor: "#00ffc6",
                    border: "none",
                    borderRadius: "6px",
                    cursor: "pointer",
                  }}
                >
                  👁️ Preview
                </button>

                <a href={scan.url} download>
                  <button
                    style={{
                      padding: "8px 16px",
                      backgroundColor: "#0070f3",
                      border: "none",
                      borderRadius: "6px",
                      color: "white",
                      cursor: "pointer",
                    }}
                  >
                    📥 Download
                  </button>
                </a>

                <button
                  onClick={() =>
                    navigate(`/edit/${encodeURIComponent(scan.url)}`)
                  }
                  style={{
                    padding: "8px 16px",
                    backgroundColor: "#888",
                    border: "none",
                    borderRadius: "6px",
                    color: "white",
                    cursor: "pointer",
                  }}
                >
                  ✏️ Edit Scan
                </button>
              </div>
            </div>
          ))
      )}

      <Modal
        isOpen={!!previewURL}
        onRequestClose={() => setPreviewURL(null)}
        style={{
          content: {
            backgroundColor: "#1e1e1e",
            padding: "20px",
            borderRadius: "12px",
            width: "80%",
            maxWidth: "600px",
            margin: "auto",
            color: "white",
          },
          overlay: {
            backgroundColor: "rgba(0,0,0,0.75)",
          },
        }}
      >
        <h3>🧱 3D Preview</h3>
        <LiveModel fileURL={previewURL} />
        <button
          onClick={() => setPreviewURL(null)}
          style={{
            marginTop: "12px",
            padding: "8px 16px",
            backgroundColor: "#444",
            color: "white",
            border: "none",
            borderRadius: "6px",
            cursor: "pointer",
          }}
        >
          Close
        </button>
      </Modal>
    </div>
  );
}

export default Scans;
