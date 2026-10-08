#!/bin/bash
set -e  # stop on first error

# Directory where THIS script is located
SKETCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Compiling..."
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old "$SKETCH_DIR"
echo "Uploading..."
arduino-cli upload -p /dev/arduino_nano --fqbn arduino:avr:nano:cpu=atmega328old "$SKETCH_DIR"

echo "**Upload complete**"