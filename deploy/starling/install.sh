#!/bin/bash
# install.sh - Script to copy, enable, and start a systemd service file.

# --- Configuration ---
# IMPORTANT: Replace 'your-service-name.service' with the actual name of your .service file.
AEROPRINT_CHATTER_SERVICE_FILE="aeroprint-chatter.service" # Example: 'my-web-app.service' or 'ros-talker-app.service'
WAIT_FOR_IP_SERVICE_FILE="wait-for-ip.service" # Example: 'wait-for-ip.service'

AEROPRINT_CHATTER_SERVICE_PATH="/etc/systemd/system/${AEROPRINT_CHATTER_SERVICE_FILE}"
WAIT_FOR_IP_SERVICE_PATH="/etc/systemd/system/${WAIT_FOR_IP_SERVICE_FILE}"

# --- Pre-checks ---
echo "Starting systemd service installation for ${AEROPRINT_CHATTER_SERVICE_FILE}..."

# Check if running as root
if [ "$EUID" -ne 0 ]; then
  echo "Error: This script must be run as root. Please use 'sudo ./install.sh'"
  exit 1
fi

# Check if the .service file exists in the current directory
if [ ! -f "./${AEROPRINT_CHATTER_SERVICE_FILE}" ]; then
  echo "Error: Service file '${AEROPRINT_CHATTER_SERVICE_FILE}' not found in the current directory."
  echo "Please ensure '${AEROPRINT_CHATTER_SERVICE_FILE}' is in the same folder as this script."
  exit 1
fi

# --- Copy Service File ---
echo "Copying '${AEROPRINT_CHATTER_SERVICE_FILE}' to '${AEROPRINT_CHATTER_SERVICE_PATH}'..."
cp "./${AEROPRINT_CHATTER_SERVICE_FILE}" "${AEROPRINT_CHATTER_SERVICE_PATH}" || { echo "Failed to copy service file."; exit 1; }

# Set appropriate permissions for the service file
chmod 644 "${AEROPRINT_CHATTER_SERVICE_PATH}" || { echo "Failed to set permissions on service file."; exit 1; }

echo "Starting systemd service installation for '${WAIT_FOR_IP_SERVICE_FILE}'..."

if [ ! -f "./${WAIT_FOR_IP_SERVICE_FILE}" ]; then
  echo "Error: Service file '${WAIT_FOR_IP_SERVICE_FILE}' not found in the current directory."
  echo "Please ensure '${WAIT_FOR_IP_SERVICE_FILE}' is in the same folder as this script."
  exit 1
fi
# --- Copy Wait-for-IP Service File ---
echo "Copying '${WAIT_FOR_IP_SERVICE_FILE}' to '${WAIT_FOR_IP_SERVICE_PATH}'..."
cp "./${WAIT_FOR_IP_SERVICE_FILE}" "${WAIT_FOR_IP_SERVICE_PATH}" || { echo "Failed to copy wait-for-ip service file."; exit 1; }
# Set appropriate permissions for the wait-for-ip service file
chmod 644 "${WAIT_FOR_IP_SERVICE_PATH}" || { echo "Failed to set permissions on wait-for-ip service file."; exit 1; }

# --- Reload Systemd, Enable, and Start Service ---
echo "Reloading systemd daemon to recognize new service..."
systemctl daemon-reload || { echo "Failed to reload systemd daemon."; exit 1; }

echo "Enabling '${AEROPRINT_CHATTER_SERVICE_FILE}' to start on boot..."
systemctl enable "${AEROPRINT_CHATTER_SERVICE_FILE}" || { echo "Failed to enable service."; exit 1; }

echo "Starting '${AEROPRINT_CHATTER_SERVICE_FILE}' now..."
systemctl start "${AEROPRINT_CHATTER_SERVICE_FILE}" || { echo "Failed to start service. Check 'sudo systemctl status ${AEROPRINT_CHATTER_SERVICE_FILE}' for details."; exit 1; }

echo "Enabling '${WAIT_FOR_IP_SERVICE_FILE}' to start on boot..."
systemctl enable "${WAIT_FOR_IP_SERVICE_FILE}" || { echo "Failed to enable wait-for-ip service."; exit 1; }
echo "Starting '${WAIT_FOR_IP_SERVICE_FILE}' now..."
systemctl start "${WAIT_FOR_IP_SERVICE_FILE}" || { echo "Failed to start wait-for-ip service. Check 'sudo systemctl status ${WAIT_FOR_IP_SERVICE_FILE}' for details."; exit 1; }

echo "Installation and startup complete for ${AEROPRINT_CHATTER_SERVICE_FILE}!"
echo "---"
echo "To check service status: sudo systemctl status ${AEROPRINT_CHATTER_SERVICE_FILE}"
echo "To view service logs: sudo journalctl -u ${AEROPRINT_CHATTER_SERVICE_FILE} -f"
echo "To stop the service: sudo systemctl stop ${AEROPRINT_CHATTER_SERVICE_FILE}"
echo "To disable the service (prevent autostart on boot): sudo systemctl disable ${AEROPRINT_CHATTER_SERVICE_FILE}"
