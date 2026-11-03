"""Builds and runs the host test of the OD writer frame decoder (run: pytest test/):
test/od_writer/od_writer_frame_test.cpp, with components/canopen_entities/od_writer_frame.h."""

import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
SOURCE = ROOT / "test" / "od_writer" / "od_writer_frame_test.cpp"
MAPPER = ROOT / "components" / "canopen_entities"
SANITIZERS = ["-fsanitize=address,undefined", "-fno-sanitize-recover=all"]


def _compile(cxx: str, out: Path, extra: list[str]) -> subprocess.CompletedProcess:
    cmd = [cxx, "-std=c++17", "-Wall", "-Wextra", "-Werror", "-g", *extra, f"-I{MAPPER}", str(SOURCE), "-o", str(out)]
    return subprocess.run(cmd, capture_output=True, text=True)


def test_od_writer_frame_cpp(tmp_path: Path) -> None:
    cxx = shutil.which("c++") or shutil.which("g++") or shutil.which("clang++")
    if not cxx:
        pytest.skip("no C++ compiler")
    exe = tmp_path / "od_writer_frame_test"
    result = _compile(cxx, exe, SANITIZERS)
    if result.returncode != 0 and "sanitize" in result.stderr:
        result = _compile(cxx, exe, [])  # toolchain without sanitizer runtimes
    assert result.returncode == 0, result.stderr
    run = subprocess.run([str(exe)], capture_output=True, text=True, timeout=60)
    assert run.returncode == 0, run.stdout + run.stderr
    assert run.stdout.strip() == "OK"
