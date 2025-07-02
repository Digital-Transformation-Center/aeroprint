from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
import os
from werkzeug.utils import secure_filename
from datetime import datetime

app = Flask(__name__)
CORS(app)

UPLOAD_FOLDER = os.path.join(os.getcwd(), "uploads")
os.makedirs(UPLOAD_FOLDER, exist_ok=True)
app.config["UPLOAD_FOLDER"] = UPLOAD_FOLDER

@app.route("/")
def home():
    return "👋 AeroPrint backend is running."

@app.route("/upload", methods=["POST"])
def upload_file():
    if "scan" not in request.files:
        return "No file uploaded.", 400
    file = request.files["scan"]
    if file.filename == "":
        return "No selected file.", 400
    filename = f"{int(datetime.now().timestamp())}_{secure_filename(file.filename)}"
    file_path = os.path.join(app.config["UPLOAD_FOLDER"], filename)
    file.save(file_path)
    file_url = f"http://localhost:4000/{filename}"
    return jsonify({"url": file_url})

@app.route("/print", methods=["POST"])
def simulate_print():
    data = request.get_json()
    filename = data.get("filename")
    if not filename:
        return jsonify({"error": "Filename required"}), 400
    print(f"🖨️ Simulating print for: {filename}")
    return jsonify({"message": f"Printing started for {filename}"})

@app.route("/<path:filename>")
def serve_uploaded_file(filename):
    return send_from_directory(app.config["UPLOAD_FOLDER"], filename)

if __name__ == "__main__":
    app.run(port=4000)
