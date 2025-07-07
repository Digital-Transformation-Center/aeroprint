import React, { useState } from "react";
import { useScans } from "../context/ScanContext";
import { useNavigate, Link } from "react-router-dom";
import Modal from "react-modal";
import LiveModel from "../components/LiveModel";

Modal.setAppElement("#root");

export default function Scans() {
  const { scans } = useScans();
  const navigate = useNavigate();
  const [previewURL, setPreviewURL] = useState(null);
  const [editScan, setEditScan] = useState(null);
  const [editedValues, setEditedValues] = useState({
    name: "",
    color: "#00ffc6",
    scale: 1,
    profile: "Standard",
  });

  const openEditModal = (scan) => {
    setEditScan(scan);
    setEditedValues({
      name: scan.name,
      color: scan.color || "#00ffc6",
      scale: scan.scale || 1,
      profile: scan.profile || "Standard",
    });
  };

  const closeEditModal = () => {
    setEditScan(null);
    setEditedValues({
      name: "",
      color: "#00ffc6",
      scale: 1,
      profile: "Standard",
    });
  };

  const handleSave = () => {
    console.log("Saving scan edits:", {
      ...editScan,
      ...editedValues,
    });
    // TODO: Save changes to backend or context
    closeEditModal();
  };

  return (
    <div style={{ padding: "2rem", color: "white" }}>
      <div style={{ marginBottom: "1rem" }}>
        <Link to="/" style={styles.backButton}>⬅ Back to Home</Link>
      </div>

      <h2>📁 Recent Scans</h2>

      {scans.length === 0 ? (
        <p>No scans uploaded yet.</p>
      ) : (
        scans
          .slice()
          .reverse()
          .map((scan, index) => (
            <div key={index} style={styles.scanCard}>
              <h4>{scan.name}</h4>
              <p>{new Date(scan.timestamp).toLocaleString()}</p>

              <div style={styles.buttonRow}>
                <button onClick={() => setPreviewURL(scan.url)} style={styles.btnPreview}>👁️ Preview</button>
                <a href={scan.url} download>
                  <button style={styles.btnDownload}>📥 Download</button>
                </a>
                <button onClick={() => openEditModal(scan)} style={styles.btnEdit}>✏️ Edit Scan</button>
              </div>
            </div>
          ))
      )}

      {/* Preview Modal */}
      <Modal
        isOpen={!!previewURL}
        onRequestClose={() => setPreviewURL(null)}
        style={styles.modalStyles}
      >
        <h3>🧱 3D Preview</h3>
        <LiveModel fileURL={previewURL} />
        <button onClick={() => setPreviewURL(null)} style={styles.btnClose}>Close</button>
      </Modal>

      {/* Edit Modal */}
      <Modal
        isOpen={!!editScan}
        onRequestClose={closeEditModal}
        style={styles.modalStyles}
      >
        <h3>✏️ Edit Scan</h3>
        <label>🔤 Name</label>
        <input
          value={editedValues.name}
          onChange={(e) => setEditedValues({ ...editedValues, name: e.target.value })}
          style={styles.input}
        />

        <label>🎨 Color</label>
        <input
          type="color"
          value={editedValues.color}
          onChange={(e) => setEditedValues({ ...editedValues, color: e.target.value })}
          style={styles.colorInput}
        />

        <label>📏 Scale: {editedValues.scale.toFixed(2)}×</label>
        <input
          type="range"
          min="0.1"
          max="2"
          step="0.01"
          value={editedValues.scale}
          onChange={(e) => setEditedValues({ ...editedValues, scale: parseFloat(e.target.value) })}
          style={styles.slider}
        />

        <label>🧪 Print Profile</label>
        <select
          value={editedValues.profile}
          onChange={(e) => setEditedValues({ ...editedValues, profile: e.target.value })}
          style={styles.input}
        >
          <option>Standard</option>
          <option>Draft</option>
          <option>High Detail</option>
          <option>ABS</option>
          <option>PLA</option>
        </select>

        <div style={{ marginTop: "1rem" }}>
          <LiveModel fileURL={editScan?.url} color={editedValues.color} scale={editedValues.scale} />
        </div>

        <div style={styles.buttonRow}>
          <button onClick={handleSave} style={styles.btnPreview}>💾 Save</button>
          <button onClick={closeEditModal} style={styles.btnClose}>Cancel</button>
        </div>
      </Modal>
    </div>
  );
}

const styles = {
  backButton: {
    backgroundColor: "#1a1d24",
    color: "#00ffc6",
    textDecoration: "none",
    padding: "8px 16px",
    borderRadius: "6px",
    boxShadow: "0 0 10px rgba(0,255,198,0.25)",
  },
  scanCard: {
    margin: "1rem 0",
    padding: "1rem",
    backgroundColor: "#2a2a2a",
    borderRadius: "8px",
  },
  buttonRow: {
    display: "flex",
    gap: "10px",
    flexWrap: "wrap",
    marginTop: "0.8rem",
  },
  btnPreview: {
    padding: "8px 16px",
    backgroundColor: "#00ffc6",
    border: "none",
    borderRadius: "6px",
    cursor: "pointer",
  },
  btnDownload: {
    padding: "8px 16px",
    backgroundColor: "#0070f3",
    border: "none",
    borderRadius: "6px",
    color: "white",
    cursor: "pointer",
  },
  btnEdit: {
    padding: "8px 16px",
    backgroundColor: "#888",
    border: "none",
    borderRadius: "6px",
    color: "white",
    cursor: "pointer",
  },
  btnClose: {
    padding: "8px 16px",
    backgroundColor: "#444",
    color: "white",
    border: "none",
    borderRadius: "6px",
    cursor: "pointer",
  },
  modalStyles: {
    content: {
      backgroundColor: "#1e1e1e",
      padding: "20px",
      borderRadius: "12px",
      width: "90%",
      maxWidth: "700px",
      margin: "auto",
      color: "white",
    },
    overlay: {
      backgroundColor: "rgba(0,0,0,0.75)",
    },
  },
  input: {
    width: "100%",
    padding: "0.5rem",
    borderRadius: "6px",
    marginBottom: "1rem",
    fontSize: "1rem",
  },
  colorInput: {
    width: "50px",
    height: "40px",
    padding: 0,
    border: "none",
    marginBottom: "1rem",
    background: "none",
    cursor: "pointer",
  },
  slider: {
    width: "100%",
    marginBottom: "1rem",
  },
};
