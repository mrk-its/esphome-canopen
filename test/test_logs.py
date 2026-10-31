"""Builds and runs the host test of the remote logs (run: pytest test/): test/logs/log_buffer_test.cpp against
components/canopen_entities/log_buffer.cpp, with the stub headers in test/logs/stubs."""

import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
STUBS = ROOT / "test" / "logs" / "stubs"
MAPPER = ROOT / "components" / "canopen_entities"
BUILDS = {
    "canopen_entities": ([ROOT / "test" / "logs" / "log_buffer_test.cpp", MAPPER / "log_buffer.cpp"], [STUBS, MAPPER]),
}
SANITIZERS = ["-fsanitize=address,undefined", "-fno-sanitize-recover=all"]


def _compile(cxx: str, out: Path, sources: list[Path], includes: list[Path], extra: list[str]) -> subprocess.CompletedProcess:
    cmd = [cxx, "-std=c++17", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter", "-g", *extra]
    cmd += [f"-I{p}" for p in includes] + [str(s) for s in sources] + ["-o", str(out)]
    return subprocess.run(cmd, capture_output=True, text=True)


@pytest.mark.parametrize("build", BUILDS)
def test_logs_cpp(tmp_path: Path, build: str) -> None:
    cxx = shutil.which("c++") or shutil.which("g++") or shutil.which("clang++")
    if not cxx:
        pytest.skip("no C++ compiler")
    sources, includes = BUILDS[build]
    exe = tmp_path / "logs_test"
    result = _compile(cxx, exe, sources, includes, SANITIZERS)
    if result.returncode != 0 and "sanitize" in result.stderr:
        result = _compile(cxx, exe, sources, includes, [])  # toolchain without sanitizer runtimes
    assert result.returncode == 0, result.stderr
    run = subprocess.run([str(exe)], capture_output=True, text=True, timeout=60)
    assert run.returncode == 0, run.stdout + run.stderr
    assert run.stdout.strip() == "OK"
