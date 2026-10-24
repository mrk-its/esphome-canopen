"""Builds and runs test/logs/logs_test.cpp, a host test of components/canopen/logs.cpp (run: pytest test/)."""

import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
SOURCES = [ROOT / "test" / "logs" / "logs_test.cpp", ROOT / "components" / "canopen" / "logs.cpp"]
INCLUDES = [ROOT / "test" / "logs" / "stubs", ROOT / "components" / "canopen"]
SANITIZERS = ["-fsanitize=address,undefined", "-fno-sanitize-recover=all"]


def _compile(cxx, out, extra):
    cmd = [cxx, "-std=c++17", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter", "-g", *extra]
    cmd += [f"-I{p}" for p in INCLUDES] + [str(s) for s in SOURCES] + ["-o", str(out)]
    return subprocess.run(cmd, capture_output=True, text=True)


def test_logs_cpp(tmp_path):
    cxx = shutil.which("c++") or shutil.which("g++") or shutil.which("clang++")
    if not cxx:
        pytest.skip("no C++ compiler")
    exe = tmp_path / "logs_test"
    build = _compile(cxx, exe, SANITIZERS)
    if build.returncode != 0 and "sanitize" in build.stderr:
        build = _compile(cxx, exe, [])  # toolchain without sanitizer runtimes
    assert build.returncode == 0, build.stderr
    run = subprocess.run([str(exe)], capture_output=True, text=True, timeout=60)
    assert run.returncode == 0, run.stdout + run.stderr
    assert run.stdout.strip() == "OK"
