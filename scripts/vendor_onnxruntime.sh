#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
VENDOR_DIR="$PROJECT_ROOT/vendor"
TMP_DIR="$PROJECT_ROOT/.onnxruntime_vendor_tmp"

GPU_URL="https://files.pythonhosted.org/packages/ba/75/7d6dafa54255a978b0698cfe3d073208e6b0df311b15468b3cf9e33e6053/onnxruntime_gpu-1.19.2-cp39-cp39-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl"
CPU_URL="https://files.pythonhosted.org/packages/12/f4/39c395c98e9ecccb0751f80897a5d065d5070c69823b0c94e95b371b568c/onnxruntime-1.19.2-cp39-cp39-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl"

GPU_WHEEL="$TMP_DIR/onnxruntime_gpu.whl"
CPU_WHEEL="$TMP_DIR/onnxruntime_cpu.whl"

GPU_TARGET="$VENDOR_DIR/onnxruntime_gpu_py39"
CPU_TARGET="$VENDOR_DIR/onnxruntime_cpu_py39"

rm -rf "$TMP_DIR"
rm -rf "$GPU_TARGET"
rm -rf "$CPU_TARGET"

mkdir -p "$TMP_DIR"
mkdir -p "$GPU_TARGET"
mkdir -p "$CPU_TARGET"

echo "Downloading GPU wheel..."
python3 - "$GPU_URL" "$GPU_WHEEL" <<'PY'
import sys
import urllib.request

url = sys.argv[1]
path = sys.argv[2]

with urllib.request.urlopen(url) as response, open(path, "wb") as output_file:
    output_file.write(response.read())
PY

echo "Downloading CPU wheel..."
python3 - "$CPU_URL" "$CPU_WHEEL" <<'PY'
import sys
import urllib.request

url = sys.argv[1]
path = sys.argv[2]

with urllib.request.urlopen(url) as response, open(path, "wb") as output_file:
    output_file.write(response.read())
PY

echo "Extracting GPU wheel..."
python3 -m zipfile -e "$GPU_WHEEL" "$GPU_TARGET"

echo "Extracting CPU wheel..."
python3 -m zipfile -e "$CPU_WHEEL" "$CPU_TARGET"

rm -rf "$TMP_DIR"

echo
echo "Vendored ONNX Runtime into:"
echo "  $GPU_TARGET"
echo "  $CPU_TARGET"
