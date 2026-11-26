#!/bin/bash
# What a Linux machine has for the generator: python3, gcc, numpy, Pillow, CPUs, memory, libgomp (read only;
# installs nothing).
uname -a
head -2 /etc/os-release
echo "python3: $(command -v python3) $(python3 --version 2>&1)"
echo "gcc: $(command -v gcc) $(gcc --version 2>/dev/null | head -1)"
python3 -c 'import numpy; print("numpy", numpy.__version__)' 2>&1 | tail -1
python3 -c 'import PIL; print("pillow", PIL.__version__)' 2>&1 | tail -1
echo "cpus: $(nproc)"
free -g | head -2
echo "libgomp: $(ldconfig -p 2>/dev/null | grep -c libgomp)"
