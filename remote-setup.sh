#!/bin/bash
set -euo pipefail

# Ask for user
read -p "What user should this service run as? [pi]: " SERVICE_USER
SERVICE_USER=${SERVICE_USER:-pi}
echo "Service will run as user: $SERVICE_USER"

# Get the absolute path to the binary
BINARY_PATH="$(pwd)/target/release/device-server"

# Create temp file and modify service file with correct paths
echo "Creating service file with correct paths..."
sed "s|User=pi|User=${SERVICE_USER}|g; s|ExecStart=.*|ExecStart=${BINARY_PATH}|g" rules/remote-xum1541.service > /tmp/remote-xum1541.service

echo "Installing remote-xum1541.service"
sudo rm -f /etc/systemd/system/remote-xum1541.service
sudo cp /tmp/remote-xum1541.service /etc/systemd/system/remote-xum1541.service
rm /tmp/remote-xum1541.service

echo "Reloading systemd"
sudo systemctl daemon-reload
echo "Enabling remote-xum1541.service"
sudo systemctl enable remote-xum1541.service
echo "Starting remote-xum1541.service"
sudo systemctl start remote-xum1541.service
echo "Querying status of remote-xum1541.service"
sudo systemctl status remote-xum1541.service
