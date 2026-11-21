"""Child process of remote.py: loads a remote node's YAML, prints the parts canopen_entities reads as JSON.

    python -P remote_loader.py <remote yaml> <local config path or ""> <top-level key>...

Runs ESPHome's pre-validation passes on the remote YAML (packages, substitutions, package merge, !extend /
!remove: steps 0..1.2 of esphome.config.validate_config()) without validating it. In a process of its own because
those passes keep state in ESPHome globals (the yaml_util secret sets, CORE.data entries of packages and of the
YAML loader) that would otherwise mix the remote node into the local node's run.

Self-contained: no imports from the component package (run as a script). Exit 1 with the error on stderr.
"""

import json
from pathlib import Path
import sys


def load(remote_path: Path, local_config_path: Path | None) -> dict:
    from esphome import yaml_util
    from esphome.components.packages import do_packages_pass, merge_packages
    from esphome.components.substitutions import do_substitution_pass
    from esphome.config import resolve_extend_remove
    from esphome.core import CORE

    # as in the local run: !secret falls back to the local node's secrets.yaml, git packages are cached in its
    # .esphome directory (CORE.data_dir)
    CORE.config_path = local_config_path or remote_path
    config = yaml_util.load_yaml(remote_path)
    if "packages" in config:
        config = do_packages_pass(config)
    config = do_substitution_pass(config)
    if "packages" in config:
        config = merge_packages(config)
    resolve_extend_remove(config)
    return config


def main(argv: list[str]) -> int:
    remote_path = Path(argv[1])
    local_config_path = Path(argv[2]) if argv[2] else None
    keys = argv[3:]
    try:
        config = load(remote_path, local_config_path)
    except Exception as err:  # noqa: BLE001 - any loader error goes to the parent as text
        print(f"{type(err).__name__}: {err}", file=sys.stderr)
        return 1
    # !lambda and other ESPHome objects as text: canopen_entities only checks whether such keys are present
    json.dump({key: config[key] for key in keys if key in config}, sys.stdout, default=str)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
