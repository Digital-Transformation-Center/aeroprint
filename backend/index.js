const express = require("express");
const multer = require("multer");
const cors = require("cors");
const path = require("path");

const app = express();
const PORT = 4000;

app.get("/", (req, res) => {
  res.send("👋 AeroPrint backend is running.");
});

app.use(cors());
app.use(express.static("uploads"));

// Setup storage for uploaded scans
const storage = multer.diskStorage({
  destination: (req, file, cb) => cb(null, "uploads"),
  filename: (req, file, cb) => cb(null, Date.now() + "_" + file.originalname),
});
const upload = multer({ storage });

// Upload route
app.post("/upload", upload.single("scan"), (req, res) => {
  if (!req.file) return res.status(400).send("No file uploaded.");
  const fileURL = `http://localhost:${PORT}/${req.file.filename}`;
  res.json({ url: fileURL });
});

// Print simulation
app.post("/print", express.json(), (req, res) => {
  const { filename } = req.body;
  if (!filename) return res.status(400).json({ error: "Filename required" });
  console.log(`🖨️ Simulating print for: ${filename}`);
  res.json({ message: `Printing started for ${filename}` });
});

app.listen(PORT, () => console.log(`🚀 Backend running on http://localhost:${PORT}`));
