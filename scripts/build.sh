#!/bin/bash
set -e

echo "Building WASM for GitHub Pages deployment..."

# Add wasm32 target if not already present
rustup target add wasm32-unknown-unknown

# Build the project for WASM target
echo "Building Rust project for WASM..."
cargo build --target wasm32-unknown-unknown --release

# Copy built WASM file to dist
echo "Copying WASM file to dist/"
mkdir -p dist
cp target/wasm32-unknown-unknown/release/sph_simulation.wasm dist/

# Copy static assets to dist
echo "Copying static assets to dist/"
cp -r static/* dist/

echo "✅ Build complete! Files ready in dist/:"
ls -la dist/

echo ""
echo "To test locally:"
echo "  cd dist && python3 -m http.server 8000"
echo "  # Then visit http://localhost:8000"
