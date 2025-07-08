import React, { useState, useEffect } from "react";
import LiveModel from "../components/LiveModel";
import FakeScanAnimation from "../components/FakeScanAnimation";
import Modal from "react-modal";

Modal.setAppElement("#root");

export default function Demo() {
  const [step, setStep] = useState("scan"); // scan → preview → edit → done
  const [demoData, setDemoData] = useState({
    fileURL: "/models/DTCcoaster.stl",
    name: "DTC Coaster",
    color: "#00ffc6",
    scale: 1,
    profile: "Standard",
  });

  const [printing, setPrinting] = useState(false);
  const [demoComplete, setDemoComplete] = useState(false);

  useEffect(() => {
    if (step === "scan") {
      const timeout = setTimeout(() => setStep("preview"), 3000);
      return () => clearTimeout(timeout);
    }
  }, [step]);

  const handleEditSave = () => {
    console.log("✅ Demo updated:", demoData);
    setStep("done");
    setDemoComplete(false);
  };

  return (
    <div style={{ padding: "2rem", color: "white", maxWidth: "900px", margin: "0 auto", position: "relative" }}>
      {/* ⬅ Back Button Top-Right */}
      <div style={styles.backContainer}>
        <a href="/" style={styles.backButton}>⬅ Back</a>
      </div>

      <h2>🚀 Demo Mode</h2>

      {step === "scan" && (
        <FakeScanAnimation duration={3000} onComplete={() => setStep("preview")} />
      )}

      {step === "preview" && (
        <>
          <h3>{demoData.name}</h3>
          <LiveModel fileURL={demoData.fileURL} color={demoData.color} scale={demoData.scale} />
          <button style={styles.btn} onClick={() => setStep("edit")}>✏️ Edit Scan</button>
        </>
      )}

      {step === "edit" && (
        <Modal isOpen style={styles.modal}>
          <h3>✏️ Edit Demo Scan</h3>

          <label>Name</label>
          <input
            value={demoData.name}
            onChange={(e) => setDemoData({ ...demoData, name: e.target.value })}
            style={styles.input}
          />

          <label>Color</label>
          <input
            type="color"
            value={demoData.color}
            onChange={(e) => setDemoData({ ...demoData, color: e.target.value })}
            style={styles.input}
          />

          <label>Scale: {demoData.scale.toFixed(2)}×</label>
          <input
            type="range"
            min="0.1"
            max="2"
            step="0.01"
            value={demoData.scale}
            onChange={(e) => setDemoData({ ...demoData, scale: parseFloat(e.target.value) })}
            style={{ width: "100%", marginBottom: "1rem" }}
          />

          <label>Print Profile</label>
          <select
            value={demoData.profile}
            onChange={(e) => setDemoData({ ...demoData, profile: e.target.value })}
            style={styles.input}
          >
            <option>Standard</option>
            <option>Draft</option>
            <option>High Detail</option>
            <option>ABS</option>
            <option>PLA</option>
          </select>

          <LiveModel fileURL={demoData.fileURL} color={demoData.color} scale={demoData.scale} />

          <div style={styles.row}>
            <button onClick={handleEditSave} style={styles.btn}>✅ Save</button>
            <button onClick={() => setStep("preview")} style={styles.btnSecondary}>Cancel</button>
          </div>
        </Modal>
      )}

      {step === "done" && (
        <>
          <h3>🎉 Your scan is ready!</h3>
          <p>Exporting with <strong>{demoData.profile}</strong> profile...</p>
          <LiveModel fileURL={demoData.fileURL} color={demoData.color} scale={demoData.scale} />

          {!printing && !demoComplete && (
            <button
              style={styles.btn}
              onClick={() => {
                setPrinting(true);
                setTimeout(() => {
                  setPrinting(false);
                  setDemoComplete(true);
                }, 3000);
              }}
            >
              🖨️ Send to Printer
            </button>
          )}

          {printing && (
            <div style={{ marginTop: "1rem", fontSize: "1.1rem" }}>
              <p>🛠️ Sending to printer…</p>
              <div style={styles.progressBar} />
            </div>
          )}

          {demoComplete && (
            <div style={{ marginTop: "2rem" }}>
              <h2>✅ Demo Complete</h2>
              <p>Your part is ready to print. Thanks for trying AeroPrint!</p>
              <button style={styles.btn} onClick={() => setStep("scan")}>🔁 Try Again</button>
            </div>
          )}
        </>
      )}
    </div>
  );
}

const styles = {
  input: {
    display: "block",
    width: "100%",
    padding: "0.5rem",
    marginBottom: "1rem",
    borderRadius: "6px",
  },
  btn: {
    padding: "10px 20px",
    backgroundColor: "#00ffc6",
    border: "none",
    borderRadius: "6px",
    color: "#000",
    cursor: "pointer",
    fontWeight: "bold",
  },
  btnSecondary: {
    padding: "10px 20px",
    backgroundColor: "#444",
    border: "none",
    borderRadius: "6px",
    color: "white",
    cursor: "pointer",
  },
  row: {
    display: "flex",
    gap: "1rem",
    marginTop: "1.2rem",
  },
  modal: {
    content: {
      background: "#1e1e1e",
      padding: "20px",
      borderRadius: "10px",
      maxWidth: "700px",
      margin: "auto",
      color: "white",
    },
    overlay: {
      backgroundColor: "rgba(0,0,0,0.7)",
    },
  },
  progressBar: {
    marginTop: "10px",
    height: "12px",
    width: "100%",
    background: "#222",
    borderRadius: "10px",
    overflow: "hidden",
    boxShadow: "0 0 6px #00ffc655",
    backgroundImage: "linear-gradient(90deg, #00ffc6 0%, #0070f3 100%)",
    backgroundSize: "200% auto",
    animation: "pulse 1.5s linear infinite",
  },
  backContainer: {
    position: "absolute",
    top: "1rem",
    right: "1rem",
    zIndex: 1000,
  },
  backButton: {
    backgroundColor: "#1a1d24",
    color: "#00ffc6",
    textDecoration: "none",
    padding: "8px 16px",
    borderRadius: "6px",
    fontWeight: "bold",
    boxShadow: "0 0 8px rgba(0,255,198,0.25)",
  },
};
