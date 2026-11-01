"""components/canopen is a copy of the hub (scripts/sync_hub.py): it must match what SYNCED_FROM records (run: pytest
test/). Edits belong in the ESPHome fork (~/repos/esphome, canopen-full), followed by a re-sync."""

import importlib.util
import os
from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).parent.parent
_spec = importlib.util.spec_from_file_location(
    "sync_hub", ROOT / "scripts" / "sync_hub.py"
)
sync_hub = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sync_hub)

FORK = Path(os.environ.get("ESPHOME_FORK", Path.home() / "repos" / "esphome"))


def test_copy_unmodified():
    marker = sync_hub.read_marker()
    assert sync_hub.tree_digest(sync_hub.DEST) == marker["digest"], (
        "components/canopen differs from the synced copy: edit the hub in the ESPHome fork and run scripts/sync_hub.py"
    )


def test_copy_matches_commit(tmp_path):
    """The recorded commit really gives these files (needs the fork checkout with that commit)."""
    marker = sync_hub.read_marker()
    assert "worktree" not in marker, (
        "components/canopen was synced from a working tree: commit the hub in the fork and run scripts/sync_hub.py"
    )
    commit = marker["commit"]
    known = subprocess.run(
        ["git", "-C", str(FORK), "cat-file", "-e", f"{commit}^{{commit}}"],
        capture_output=True,
        check=False,
    )
    if known.returncode != 0:
        pytest.skip(f"commit {commit[:10]} not in {FORK}")
    sync_hub.extract(FORK, commit, tmp_path)
    assert sync_hub.tree_digest(tmp_path) == sync_hub.tree_digest(sync_hub.DEST)
