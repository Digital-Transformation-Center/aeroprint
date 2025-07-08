
from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
import os
from werkzeug.utils import secure_filename
from datetime import datetime

app = Flask(__name__)
CORS(app)

# Use /uploads as the upload folder (mounted as a Docker volume)
UPLOAD_FOLDER = os.environ.get("UPLOAD_FOLDER", "/uploads")
os.makedirs(UPLOAD_FOLDER, exist_ok=True)
app.config["UPLOAD_FOLDER"] = UPLOAD_FOLDER

@app.route("/")
def home():
    return "👋 AeroPrint backend API is running."

@app.route("/api/upload", methods=["POST"])
def upload_file():
    if "scan" not in request.files:
        return "No file uploaded.", 400
    file = request.files["scan"]
    if file.filename == "":
        return "No selected file.", 400
    filename = f"{int(datetime.now().timestamp())}_{secure_filename(file.filename)}"
    file_path = os.path.join(app.config["UPLOAD_FOLDER"], filename)
    file.save(file_path)
    file_url = f"/uploads/{filename}"
    return jsonify({"url": file_url, "filename": filename})

@app.route("/api/print", methods=["POST"])
def simulate_print():
    data = request.get_json()
    filename = data.get("filename")
    if not filename:
        return jsonify({"error": "Filename required"}), 400
    print(f"🖨️ Simulating print for: {filename}")
    return jsonify({"message": f"Printing started for {filename}"})

@app.route("/uploads/<path:filename>")
def serve_uploaded_file(filename):
    return send_from_directory(app.config["UPLOAD_FOLDER"], filename)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=4000)
