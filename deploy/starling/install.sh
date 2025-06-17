#!/bin/bash
# install.sh - Script to copy, enable, and start a systemd service file.

# --- Configuration ---
# IMPORTANT: Replace 'your-service-name.service' with the actual name of your .service file.
SERVICE_FILE="aeroprint-chatter.service" # Example: 'my-web-app.service' or 'ros-talker-app.service'

SERVICE_PATH="/etc/systemd/system/${SERVICE_FILE}"

# --- Pre-checks ---
echo "Starting systemd service installation for ${SERVICE_FILE}..."

# Check if running as root
if [ "$EUID" -ne 0 ]; then
  echo "Error: This script must be run as root. Please use 'sudo ./install.sh'"
  exit 1
fi

# Check if the .service file exists in the current directory
if [ ! -f "./${SERVICE_FILE}" ]; then
  echo "Error: Service file '${SERVICE_FILE}' not found in the current directory."
  echo "Please ensure '${SERVICE_FILE}' is in the same folder as this script."
  exit 1
fi

# --- Copy Service File ---
echo "Copying '${SERVICE_FILE}' to '${SERVICE_PATH}'..."
cp "./${SERVICE_FILE}" "${SERVICE_PATH}" || { echo "Failed to copy service file."; exit 1; }

# Set appropriate permissions for the service file
chmod 644 "${SERVICE_PATH}" || { echo "Failed to set permissions on service file."; exit 1; }

# --- Reload Systemd, Enable, and Start Service ---
echo "Reloading systemd daemon to recognize new service..."
systemctl daemon-reload || { echo "Failed to reload systemd daemon."; exit 1; }

echo "Enabling '${SERVICE_FILE}' to start on boot..."
systemctl enable "${SERVICE_FILE}" || { echo "Failed to enable service."; exit 1; }

echo "Starting '${SERVICE_FILE}' now..."
systemctl start "${SERVICE_FILE}" || { echo "Failed to start service. Check 'sudo systemctl status ${SERVICE_FILE}' for details."; exit 1; }

echo "Installation and startup complete for ${SERVICE_FILE}!"
echo "---"
echo "To check service status: sudo systemctl status ${SERVICE_FILE}"
echo "To view service logs: sudo journalctl -u ${SERVICE_FILE} -f"
echo "To stop the service: sudo systemctl stop ${SERVICE_FILE}"
echo "To disable the service (prevent autostart on boot): sudo systemctl disable ${SERVICE_FILE}"
