import React from "react";
import { useScans } from "../context/ScanContext";

function Scans() {
  const { scans } = useScans();

  return (
    <div style={{ padding: "2rem", color: "white" }}>
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
              {scan.url && (
                <>
                  <a
                    href={scan.url}
                    target="_blank"
                    rel="noreferrer"
                    style={{ color: "#00ffc6" }}
                  >
                    View Scan
                  </a>
                  <div style={{ marginTop: "8px" }}>
                    <a
                      href={scan.url}
                      target="_blank"
                      rel="noopener noreferrer"
                      style={{ color: "#00b3ff" }}
                    >
                      🔗 Direct File Link
                    </a>
                  </div>
                  <div style={{ marginTop: "8px" }}>
                    <button
                      style={{
                        padding: "6px 12px",
                        backgroundColor: "#444",
                        color: "white",
                        border: "none",
                        borderRadius: "4px",
                        cursor: "pointer",
                      }}
                      onClick={() => {
                        window.open(
                          `/viewer?model=${encodeURIComponent(scan.url)}`,
                          "_blank"
                        );
                      }}
                    >
                      👁️ Preview
                    </button>
                  </div>
                </>
              )}
            </div>
          ))
      )}
    </div>
  );
}

export default Scans;
