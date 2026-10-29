"""Builds and runs test/od/od_test.cpp, a host test of components_next/canopen/od.cpp against CANopenNode's
301/CO_ODinterface.c (run: pytest test/). CANopenNode is the checkout next to this repo, or $CANOPENNODE_PATH."""

import os
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
COMPONENT = ROOT / "components_next" / "canopen"
CANOPENNODE = Path(os.environ.get("CANOPENNODE_PATH", ROOT.parent / "CANopenNode"))
INCLUDES = [ROOT / "test" / "od" / "stubs", CANOPENNODE, COMPONENT / "canopennode_port", COMPONENT]
SANITIZERS = ["-fsanitize=address,undefined", "-fno-sanitize-recover=all"]


def _build(cc, cxx, tmp_path, extra):
    inc = [f"-I{p}" for p in INCLUDES]
    obj = tmp_path / "CO_ODinterface.o"
    c = subprocess.run(
        [cc, "-std=c11", "-Wall", "-Wextra", "-g", *extra, *inc, "-c", str(CANOPENNODE / "301" / "CO_ODinterface.c"),
         "-o", str(obj)],
        capture_output=True, text=True,
    )
    if c.returncode != 0:
        return c
    exe = tmp_path / "od_test"
    cmd = [cxx, "-std=c++17", "-Wall", "-Wextra", "-Werror", "-g", *extra, *inc,
           str(ROOT / "test" / "od" / "od_test.cpp"), str(COMPONENT / "od.cpp"), str(obj), "-o", str(exe)]
    return subprocess.run(cmd, capture_output=True, text=True)


def test_od_cpp(tmp_path):
    cc = shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")
    cxx = shutil.which("c++") or shutil.which("g++") or shutil.which("clang++")
    if not cc or not cxx:
        pytest.skip("no C/C++ compiler")
    if not (CANOPENNODE / "301" / "CO_ODinterface.c").is_file():
        pytest.skip(f"CANopenNode not found at {CANOPENNODE}")
    build = _build(cc, cxx, tmp_path, SANITIZERS)
    if build.returncode != 0 and "sanitize" in build.stderr:
        build = _build(cc, cxx, tmp_path, [])  # toolchain without sanitizer runtimes
    assert build.returncode == 0, build.stderr
    run = subprocess.run([str(tmp_path / "od_test")], capture_output=True, text=True, timeout=60)
    assert run.returncode == 0, run.stdout + run.stderr
    assert run.stdout.strip() == "OK"
