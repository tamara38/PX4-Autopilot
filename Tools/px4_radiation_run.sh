#!/bin/bash
set -e

LOGFILE="/tmp/px4_sitl_run.log"
rm -f "$LOGFILE"

cd ~/PX4-Autopilot   # prilagodi putanju svom PX4 repo-u

PX4_GZ_WORLD=radiation_field make px4_sitl gz_x500 2>&1 | tee "$LOGFILE" &
SITL_PID=$!

echo "Waiting for source to be found..."

until grep -q "SOURCE FOUND" "$LOGFILE" 2>/dev/null; do
    sleep 0.5
done

echo "Source found — waiting for CSV flush..."
sleep 2

echo "Shutting down simulation..."
pkill -f gz || true
pkill -f px4 || true


echo "Generating heatmap..."
python3 src/modules/radiation_detector/radiation_heatmap.py  # prilagodi putanju
