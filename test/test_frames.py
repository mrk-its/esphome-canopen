"""Builds and runs test/frames/frames_test.cpp, a host test of the frame decoders in components_next/canopen/types.h
(run: pytest test/)."""

import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
SANITIZERS = ["-fsanitize=address,undefined", "-fno-sanitize-recover=all"]


def _compile(cxx, out, extra):
    cmd = [cxx, "-std=c++17", "-Wall", "-Wextra", "-Werror", "-g", *extra,
           f"-I{ROOT / 'components_next' / 'canopen'}", str(ROOT / "test" / "frames" / "frames_test.cpp"),
           "-o", str(out)]
    return subprocess.run(cmd, capture_output=True, text=True)


def test_frames_cpp(tmp_path):
    cxx = shutil.which("c++") or shutil.which("g++") or shutil.which("clang++")
    if not cxx:
        pytest.skip("no C++ compiler")
    exe = tmp_path / "frames_test"
    build = _compile(cxx, exe, SANITIZERS)
    if build.returncode != 0 and "sanitize" in build.stderr:
        build = _compile(cxx, exe, [])  # toolchain without sanitizer runtimes
    assert build.returncode == 0, build.stderr
    run = subprocess.run([str(exe)], capture_output=True, text=True, timeout=60)
    assert run.returncode == 0, run.stdout + run.stderr
    assert run.stdout.strip() == "OK"
