#!/bin/bash
set -e

echo "Building and serving locally for testing..."

# Build first
./build.sh

# Serve the dist directory
echo "Starting local server..."
echo "Visit http://localhost:8000 to test"
cd dist
python3 -m http.server 8000
