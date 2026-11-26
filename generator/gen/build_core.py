"""Builds the C helpers into the tree's build/ folder (Windows and Linux).

  s30_core: routes_core.c (EDT, Dijkstra / A*, geodesic discs), s30_vis.c (line of sight), lanes_core.c (cover search,
            chain); OpenMP on.
            Windows: MSVC  cl /O2 /openmp /LD  -> build/s30_core.dll   (located with vswhere)
            Linux:   gcc   -O2 -fopenmp -shared -fPIC -lm  -> build/s30_core.so
  read_check: read_check.c (the library file check) -> build/read_check.exe (MSVC) / build/read_check (gcc -O2 -lm)
A target is rebuilt when it is missing or older than one of its sources. Override the compiler with CC (Linux).
"""
import os
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))
import paths  # noqa: E402

SRC = [HERE / "routes_core.c", HERE / "s30_vis.c", HERE / "lanes_core.c"]
RC_SRC = HERE / "read_check.c"
WINDOWS = os.name == "nt"


def _msvc():
    if shutil.which("cl"):
        return "cl", os.environ.copy()
    vswhere = Path(os.environ["ProgramFiles(x86)"]) / "Microsoft Visual Studio" / "Installer" / "vswhere.exe"
    install = subprocess.check_output([vswhere, "-latest", "-products", "*", "-requires",
                                       "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property", "installationPath"],
                                      text=True).strip()
    vcvars = Path(install) / "VC" / "Auxiliary" / "Build" / "vcvars64.bat"
    shell = subprocess.check_output(f'call "{vcvars}" >nul && set', shell=True, text=True)
    env = dict(line.split("=", 1) for line in shell.splitlines() if "=" in line and not line.startswith("="))
    return shutil.which("cl", path=next(value for name, value in env.items() if name.upper() == "PATH")), env


def _fresh(target, sources):
    return target.exists() and all(target.stat().st_mtime > s.stat().st_mtime for s in sources)


def lib_path():
    return paths.BUILD / ("s30_core.dll" if WINDOWS else "s30_core.so")


def read_check_path():
    return paths.BUILD / ("read_check.exe" if WINDOWS else "read_check")


def build():
    out = lib_path()
    if _fresh(out, SRC):
        return out
    paths.BUILD.mkdir(parents=True, exist_ok=True)
    if WINDOWS:
        cl, env = _msvc()
        subprocess.check_call([cl, "/nologo", "/O2", "/openmp", "/LD"] + [str(s) for s in SRC] +
                              ["/Fe:" + str(out), "/Fo:" + str(paths.BUILD) + os.sep], cwd=str(paths.BUILD), env=env)
    else:
        cc = os.environ.get("CC", "gcc")
        subprocess.check_call([cc, "-O2", "-fopenmp", "-shared", "-fPIC", "-o", str(out)] + [str(s) for s in SRC] + ["-lm"],
                              cwd=str(paths.BUILD))
    return out


def build_read_check():
    out = read_check_path()
    if _fresh(out, [RC_SRC]):
        return out
    paths.BUILD.mkdir(parents=True, exist_ok=True)
    if WINDOWS:
        cl, env = _msvc()
        subprocess.check_call([cl, "/nologo", "/O2", str(RC_SRC), "/Fe:" + str(out), "/Fo:" + str(paths.BUILD) + os.sep],
                              cwd=str(paths.BUILD), env=env)
    else:
        cc = os.environ.get("CC", "gcc")
        subprocess.check_call([cc, "-O2", "-o", str(out), str(RC_SRC), "-lm"], cwd=str(paths.BUILD))
    return out


if __name__ == "__main__":
    print(build())
    print(build_read_check())
