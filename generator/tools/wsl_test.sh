#!/bin/bash
# Linux build check (needs neither numpy nor Pillow): build the helpers with gcc, list the exported functions, call
# them through ctypes on the fixed test grid (compare with a Windows run), and run read_check on a pack
# (argument: out/<mission>; optional).
set -e
cd "$(dirname "$0")/.."
python3 gen/build_core.py
echo "--- exported functions of build/s30_core.so"
nm -D --defined-only build/s30_core.so | awk '$2 == "T" {print $3}' | sort | tr '\n' ' '
echo
echo "--- OpenMP linked:"
ldd build/s30_core.so | grep -i gomp || echo "no libgomp"
echo "--- ctypes self-test"
python3 tools/core_selftest.py
if [ -n "$1" ]; then
  m=$(basename "$1")
  echo "--- read_check on $1 (Linux build)"
  ./build/read_check "$1/$m.routes.bin" "$1/$m.map.bin"
fi
