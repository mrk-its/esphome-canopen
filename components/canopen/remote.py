"""Remote CANopen nodes, described by their own ESPHome YAML (`canopen: remote_nodes:`).

The remote YAML is not validated: only the pre-validation passes ESPHome runs
(packages, substitutions, !extend / !remove) are applied, then its `canopen:`
section and the configs of the exposed entities are read raw. That is enough
for layout.py to compute the remote node's TPDO layout.
"""

from pathlib import Path

from . import layout

# top-level config keys of entity domains supported by layout.py
ENTITY_DOMAINS = {
    layout.DOMAIN_SENSOR,
    layout.DOMAIN_NUMBER,
    layout.DOMAIN_BINARY_SENSOR,
    layout.DOMAIN_SWITCH,
    layout.DOMAIN_LIGHT,
    layout.DOMAIN_COVER,
    layout.DOMAIN_ALARM,
}

_cache = {}


class RemoteError(Exception):
    pass


def load_remote_config(path):
    """Load a node's YAML and apply ESPHome's pre-validation passes (see
    esphome.config.validate_config(), steps 0..1.2)."""
    # ESPHome internals, imported lazily: only configs with remote_nodes need them
    from esphome import yaml_util
    from esphome.components.packages import do_packages_pass, merge_packages
    from esphome.components.substitutions import do_substitution_pass
    from esphome.config import resolve_extend_remove

    # clear_secrets=False: don't drop secrets already loaded for the local config
    config = yaml_util.load_yaml(Path(path), clear_secrets=False)
    if "packages" in config:
        config = do_packages_pass(config)
    config = do_substitution_pass(config)
    if "packages" in config:
        config = merge_packages(config)
    resolve_extend_remove(config)
    return config


def _declares_nested_id(value, entity_id):
    """`id: entity_id` nested in a component config, ignoring automations
    (where `id:` is a reference, e.g. `on_press: - light.turn_on: {id: ...}`)."""
    if isinstance(value, dict):
        if str(value.get("id")) == entity_id:
            return True
        return any(
            _declares_nested_id(v, entity_id)
            for k, v in value.items()
            if not (str(k).startswith("on_") or k in ("then", "else"))
        )
    if isinstance(value, list):
        return any(_declares_nested_id(v, entity_id) for v in value)
    return False


def _items(config, domain):
    items = config.get(domain) or []
    return items if isinstance(items, list) else [items]


def find_entity(config, entity_id):
    """(domain, platform config) of the entity declared with `entity_id` in a raw config.

    The domain is the top-level key the entity is declared under and the platform
    config is that list item. Ids nested in a component config, e.g. `sensor:
    - platform: debug, free: {id: ...}`, belong to the same domain.
    """
    entity_id = str(entity_id)
    for domain in ENTITY_DOMAINS:
        for item in _items(config, domain):
            if isinstance(item, dict) and str(item.get("id")) == entity_id:
                return domain, item
    for domain in ENTITY_DOMAINS:
        for item in _items(config, domain):
            if _declares_nested_id(item, entity_id):
                return domain, item
    raise layout.LayoutError(
        f"ID '{entity_id}' is not declared in any of: {', '.join(sorted(ENTITY_DOMAINS))}"
    )


def _int(value):
    """Integer option of a raw config: the substitution pass may turn it into a string."""
    return value if isinstance(value, int) else int(str(value), 0)


def _normalize_node_config(canopen):
    """Coerce the raw `canopen:` options layout.py reads to ints."""
    canopen = {**canopen, "node_id": _int(canopen["node_id"])}
    entities = []
    for entity in canopen.get("entities") or ():
        entity = {**entity, "index": _int(entity["index"])}
        tpdo = entity.get("tpdo")
        if isinstance(tpdo, dict):
            entity["tpdo"] = {**tpdo, "number": _int(tpdo["number"])}
        elif tpdo is not None:
            entity["tpdo"] = _int(tpdo)
        if entity.get("size") is not None:
            entity["size"] = _int(entity["size"])
        entities.append(entity)
    canopen["entities"] = entities
    canopen["template_entities"] = [
        {**t, "index": _int(t["index"]), "tpdo": _int(t.get("tpdo", -1))}
        for t in canopen.get("template_entities") or ()
    ]
    return canopen


class RemoteNode:
    def __init__(self, name, path, config):
        self.name = name
        self.path = path
        canopen = config.get("canopen")
        if isinstance(canopen, list):
            if len(canopen) != 1:
                raise RemoteError(f"{path}: expected exactly one canopen node, got {len(canopen)}")
            canopen = canopen[0]
        if not isinstance(canopen, dict) or "node_id" not in canopen:
            raise RemoteError(f"{path}: no 'canopen:' section with node_id")
        try:
            canopen = _normalize_node_config(canopen)
        except (KeyError, TypeError, ValueError) as err:
            raise RemoteError(f"{path}: invalid canopen config: {err!r}") from err
        self.node_id = canopen["node_id"]
        self.entities = list(canopen.get("entities") or ())
        try:
            self.states = layout.node_states(
                canopen, lambda entity: find_entity(config, entity["id"])
            )
            self.mapping = layout.tpdo_mapping(self.states)
        except layout.LayoutError as err:
            raise RemoteError(f"{path}: {err}") from err
        self.layout_hash = layout.layout_hash(self.mapping)

    def entity(self, entity_id=None, index=None):
        """Remote `canopen.entities` item, by id or by index."""
        for entity in self.entities:
            if entity_id is not None and str(entity["id"]) == str(entity_id):
                return entity
            if index is not None and entity["index"] == index:
                return entity
        what = f"entity '{entity_id}'" if entity_id is not None else f"entity index {index}"
        raise RemoteError(
            f"{what} is not exposed by {self.name} (not in canopen.entities of {self.path})"
        )

    def state(self, entity_id=None, index=None, state=None):
        """Locate a remote entity state in the remote TPDOs.

        Returns dict(node_id, index, entity, state, tpdo, offset, size); `state`
        defaults to the entity's first state ("state" / "value").
        """
        entity = self.entity(entity_id, index)
        entity_name = str(entity["id"])
        names = [
            label.split(".", 1)[1]
            for idx, _, _, label in self.states
            if idx == entity["index"]
        ]
        if state is None:
            state = names[0]
        if state not in names:
            raise RemoteError(
                f"{self.name} entity '{entity_name}' has no state '{state}' (states: {', '.join(names)})"
            )
        tpdo = layout.tpdo_number(entity)
        if tpdo < 0:
            raise RemoteError(
                f"{self.name} entity '{entity_name}' is not sent in any TPDO, add 'tpdo:' to it in {self.path}"
            )
        offset = 0
        for idx, state_tpdo, size, label in self.states:
            if state_tpdo != tpdo:
                continue
            if idx == entity["index"] and label.split(".", 1)[1] == state:
                return {
                    "node_id": self.node_id,
                    "index": entity["index"],
                    "entity": entity_name,
                    "state": state,
                    "tpdo": tpdo,
                    "offset": offset,
                    "size": size,
                }
            offset += size
        raise AssertionError("state not found in its TPDO")


def get_remote_node(name, path):
    """RemoteNode for a YAML file, cached by path and modification time."""
    path = Path(path).resolve()
    key = (name, path, path.stat().st_mtime_ns)
    if key not in _cache:
        try:
            config = load_remote_config(path)
        except Exception as err:  # YAML, packages or substitution errors of the remote file
            raise RemoteError(f"{path}: can't load node '{name}': {err}") from err
        _cache[key] = RemoteNode(name, path, config)
    return _cache[key]
