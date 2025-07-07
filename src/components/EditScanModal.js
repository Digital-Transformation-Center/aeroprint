import React, { useState } from "react";
import Modal from "react-modal";
import LiveModel from "./LiveModel";

export default function EditScanModal({ scan, onClose }) {
  const [name, setName] = useState(scan.name);
  const [color, setColor] = useState(scan.color || "#00ffc6");
  const [scale, setScale] = useState(scan.scale || 1);
  const [profile, setProfile] = useState(scan.profile || "Standard");

  const handleSave = () => {
    console.log("🔧 Updated scan:", { name, color, scale, profile });
    onClose();
  };

  return (
    <Modal isOpen onRequestClose={onClose} style={modalStyles}>
      <h2>✏️ Edit Scan</h2>

      <label>🔤 Name</label>
      <input value={name} onChange={(e) => setName(e.target.value)} style={styles.input} />

      <label>🎨 Color</label>
      <input type="color" value={color} onChange={(e) => setColor(e.target.value)} style={styles.colorInput} />

      <label>📏 Scale: {scale.toFixed(2)}×</label>
      <input type="range" min="0.1" max="2" step="0.01" value={scale} onChange={(e) => setScale(parseFloat(e.target.value))} style={styles.slider} />

      <label>🧪 Print Profile</label>
      <select value={profile} onChange={(e) => setProfile(e.target.value)} style={styles.input}>
        <option>Standard</option>
        <option>Draft</option>
        <option>High Detail</option>
        <option>ABS</option>
        <option>PLA</option>
      </select>

      <div style={{ marginTop: "1rem" }}>
        <LiveModel fileURL={scan.url} color={color} scale={scale} />
      </div>

      <div style={styles.buttonRow}>
        <button onClick={handleSave}>💾 Save</button>
        <button onClick={onClose}>Cancel</button>
      </div>
    </Modal>
  );
}

const styles = {
  input: {
    display: "block",
    width: "100%",
    marginBottom: "1rem",
    padding: "0.5rem",
    borderRadius: "6px",
    fontSize: "1rem",
  },
  slider: {
    width: "100%",
    marginBottom: "1rem",
  },
  colorInput: {
    width: "50px",
    height: "40px",
    padding: "0",
    marginBottom: "1rem",
    border: "none",
    background: "none",
    cursor: "pointer",
  },
  buttonRow: {
    marginTop: "1rem",
    display: "flex",
    gap: "1rem",
  },
};

const modalStyles = {
  content: {
    background: "#1e1e1e",
    padding: "20px",
    borderRadius: "12px",
    width: "90%",
    maxWidth: "700px",
    margin: "auto",
    color: "white",
  },
  overlay: {
    backgroundColor: "rgba(0,0,0,0.8)",
  },
};
