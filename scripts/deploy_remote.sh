#!/bin/bash
set -e

PI="${1:-pi@openscan.local}"
SITE=$(ssh "$PI" 'ls -d /opt/openscan3/current/venv/lib/python3.*/site-packages/openscan_firmware' 2>/dev/null | head -1)

if [ -z "$SITE" ]; then
    echo "ERROR: Could not find openscan_firmware in Pi venv. Is openscan3 installed?" >&2
    exit 1
fi

echo "Deploying to $PI:$SITE"

scp openscan_firmware/config/remote.py                         "$PI:$SITE/config/"
scp openscan_firmware/config/motor.py                          "$PI:$SITE/config/"
scp openscan_firmware/models/scanner.py                        "$PI:$SITE/models/"
scp openscan_firmware/controllers/hardware/motors.py           "$PI:$SITE/controllers/hardware/"
scp openscan_firmware/controllers/hardware/remote_motors.py    "$PI:$SITE/controllers/hardware/"
scp openscan_firmware/controllers/hardware/remote_lights.py    "$PI:$SITE/controllers/hardware/"
scp openscan_firmware/controllers/device.py                    "$PI:$SITE/controllers/"

scp settings/device/example_remote_nodemcu.json "$PI:/etc/openscan3/device/"

echo "Restarting openscan3..."
ssh "$PI" sudo systemctl restart openscan3

echo "Done. Check logs: ssh $PI journalctl -u openscan3 -f"
