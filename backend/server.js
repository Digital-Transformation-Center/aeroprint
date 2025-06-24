const express = require("express");
const http = require("http");
const { Server } = require("socket.io");
const cors = require("cors");
const multer = require("multer");
const path = require("path");

const app = express();
const server = http.createServer(app);
const io = new Server(server, {
  cors: {
    origin: "http://localhost:3000",
    methods: ["GET", "POST"]
  }
});

// Middleware setup
app.use(cors());
app.use(express.json());

// Multer upload setup
const storage = multer.diskStorage({
  destination: "uploads/",
  filename: (req, file, cb) => {
    cb(null, `${Date.now()}-${file.originalname}`);
  },
});
const upload = multer({ storage });
app.use("/uploads", express.static("uploads")); // Serve uploaded files

// Simulated drone scan endpoint
app.post("/start-scan", (req, res) => {
  console.log("Starting drone scan...");

  const steps = [
    "Drone capturing...",
    "Generating point cloud...",
    "Building mesh...",
    "Cleaning artifacts...",
    "Finalizing scan."
  ];
  let i = 0;
  const interval = setInterval(() => {
    if (i < steps.length) {
      io.emit("scan-progress", { message: steps[i], timestamp: Date.now() });
      i++;
    } else {
      io.emit("scan-complete");
      clearInterval(interval);
    }
  }, 1000);

  res.json({ message: "Scan started." });
});

// Upload endpoint
app.post("/upload", upload.single("scan"), (req, res) => {
  if (!req.file) {
    return res.status(400).json({ error: "No file uploaded." });
  }

  console.log("Received file:", req.file.originalname);
  res.json({
    url: `http://localhost:4000/uploads/${req.file.filename}`,
  });
});

// Optional: Print simulation route
app.post("/print", (req, res) => {
  const { filename } = req.body;
  console.log(`Starting 3D print of: ${filename}`);
  res.json({ message: "Print started" });
});

server.listen(4000, () => {
  console.log("Server running on http://localhost:4000");
});
