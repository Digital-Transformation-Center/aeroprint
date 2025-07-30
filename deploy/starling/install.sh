#!/bin/bash

# --- Configuration ---
AEROPRINT_CHATTER_SERVICE_FILE="aeroprint-chatter.service" # Example: 'my-web-app.service' or 'ros-talker-app.service'
VOXL_MPA_SERVICE_FILE="voxl-mpa.service" # Example: 'voxl-mpa.service'
VOXL_MPA_EXEC_FILE="voxl_mpa.sh" # Example: 'exec/voxl_mpa.sh'
WAIT_FOR_IP_SERVICE_FILE="wait-for-ip.service" # Example: 'wait-for-ip.service'

AEROPRINT_CHATTER_SERVICE_PATH="/etc/systemd/system/${AEROPRINT_CHATTER_SERVICE_FILE}"
WAIT_FOR_IP_SERVICE_PATH="/etc/systemd/system/${WAIT_FOR_IP_SERVICE_FILE}"
VOXL_MPA_SERVICE_PATH="/etc/systemd/system/${VOXL_MPA_SERVICE_FILE}"

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

echo "Starting systemd service installation for '${VOXL_MPA_SERVICE_FILE}'..."

echo "Copying '${VOXL_MPA_EXEC_FILE}' to '/usr/local/bin/${VOXL_MPA_EXEC_FILE}'..."
cp "./exec/${VOXL_MPA_EXEC_FILE}" "/usr/local/bin/${VOXL_MPA_EXEC_FILE}" || { echo "Failed to copy VOXL MPA executable file."; exit 1; }
chmod +x /usr/local/bin/voxl_mpa.sh

if [ ! -f "./${VOXL_MPA_SERVICE_FILE}" ]; then
  echo "Error: Service file '${VOXL_MPA_SERVICE_FILE}' not found in the current directory."
  echo "Please ensure '${VOXL_MPA_SERVICE_FILE}' is in the same folder as this script."
  exit 1
fi
# --- Copy VOXL MPA Service File ---
echo "Copying '${VOXL_MPA_SERVICE_FILE}' to '${VOXL_MPA_SERVICE_PATH}'..."
cp "./${VOXL_MPA_SERVICE_FILE}" "${VOXL_MPA_SERVICE_PATH}" || { echo "Failed to copy VOXL MPA service file."; exit 1; }

chmod 644 "${VOXL_MPA_SERVICE_PATH}" || { echo "Failed to set permissions on VOXL MPA service file."; exit 1; }

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

echo "Enabling '${VOXL_MPA_SERVICE_FILE}' to start on boot..."
systemctl enable "${VOXL_MPA_SERVICE_FILE}" || { echo "Failed to enable VOXL MPA service."; exit 1; }
echo "Starting '${VOXL_MPA_SERVICE_FILE}' now..."
systemctl start "${VOXL_MPA_SERVICE_FILE}" || { echo "Failed to start VOXL MPA service. Check 'sudo systemctl status ${VOXL_MPA_SERVICE_FILE}' for details."; exit 1; }

echo "Enabling '${WAIT_FOR_IP_SERVICE_FILE}' to start on boot..."
systemctl enable "${WAIT_FOR_IP_SERVICE_FILE}" || { echo "Failed to enable wait-for-ip service."; exit 1; }
echo "Starting '${WAIT_FOR_IP_SERVICE_FILE}' now..."
systemctl start "${WAIT_FOR_IP_SERVICE_FILE}" || { echo "Failed to start wait-for-ip service. Check 'sudo systemctl status ${WAIT_FOR_IP_SERVICE_FILE}' for details."; exit 1; }

echo "Installation and startup complete for ${AEROPRINT_CHATTER_SERVICE_FILE}!"
echo "---"
echo "Setting up AeroPrint Paths..."
# --- Set up AeroPrint Paths ---
if [ ! -d "/etc/aeroprint" ]; then
  mkdir /etc/aeroprint
  mkdir /etc/aeroprint/logs
  mkdir /etc/aeroprint/config
  mkdir /etc/aeroprint/artifacts
fi


