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
cp target/wasm32-unknown-unknown/release/sph_simulation.wasm dist/

# Copy and update index.html for dist
echo "Updating index.html for deployment..."
cp index.html dist/index.html

echo "✅ Build complete! Files ready in dist/:"
ls -la dist/

echo ""
echo "To test locally:"
echo "  cd dist && python3 -m http.server 8000"
echo "  # Then visit http://localhost:8000"
