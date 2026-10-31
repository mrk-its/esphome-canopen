"""Remote CANopen nodes, described by their own ESPHome YAML (`canopen_entity_mapper: remote_nodes:`).

The remote YAML is not validated: only the pre-validation passes ESPHome runs
(packages, substitutions, !extend / !remove) are applied, then its node id and
exposed entities and the configs of those entities are read raw. That is enough
for layout.py to compute the remote node's TPDO layout. Two formats: a node on
the canopen hub (`canopen: node_id` + `canopen_entity_mapper: entities`) and a
node on the old canopen-stack component (`canopen: node_id, entities`).

Also the RPDO plan of a node (rpdo_plan()): ESPHome-free, for the tests.
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


def _float(value):
    return value if isinstance(value, (int, float)) else float(str(value))


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
        for key in ("min_value", "max_value"):
            if entity.get(key) is not None:
                entity[key] = _float(entity[key])
        entities.append(entity)
    canopen["entities"] = entities
    canopen["template_entities"] = [
        {**t, "index": _int(t["index"]), "tpdo": _int(t.get("tpdo", -1))}
        for t in canopen.get("template_entities") or ()
    ]
    return canopen


MAPPER_KEY = "canopen_entity_mapper"


def _single(config, key, path):
    section = config.get(key)
    if isinstance(section, list):
        if len(section) != 1:
            raise RemoteError(
                f"{path}: expected exactly one '{key}:', got {len(section)}"
            )
        section = section[0]
    return section


def node_section(config, path):
    """The node's `canopen:` options layout.py reads (node_id, entities), from either format."""
    canopen = _single(config, "canopen", path)
    if not isinstance(canopen, dict) or "node_id" not in canopen:
        raise RemoteError(f"{path}: no 'canopen:' section with node_id")
    mapper = _single(config, MAPPER_KEY, path)
    if mapper is None:
        # old component: entities and the OD writer option in canopen:
        return canopen, layout.flag({"x": canopen.get("pdo_od_writer", True)}, "x")
    if not isinstance(mapper, dict):
        raise RemoteError(f"{path}: invalid '{MAPPER_KEY}:' section")
    node = {"node_id": canopen["node_id"], "entities": mapper.get("entities") or []}
    return node, layout.flag({"x": mapper.get("od_writer", True)}, "x")


class RemoteNode:
    def __init__(self, name, path, config):
        self.name = name
        self.path = path
        canopen, self.od_writer = node_section(config, path)
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
            # entity index -> (domain, [(state name, size, kind)])
            self.entity_states = {}
            for entity in self.entities:
                domain, platform_config = find_entity(config, entity["id"])
                caps = layout.entity_caps(domain, platform_config, entity.get("caps"))
                self.entity_states[entity["index"]] = (
                    domain,
                    layout.entity_state_kinds(domain, caps, entity.get("size")),
                )
        except layout.LayoutError as err:
            raise RemoteError(f"{path}: {err}") from err
        self.layout_hash = layout.layout_hash(self.mapping)

    def tpdo_size(self, tpdo):
        """Bytes the remote node sends in TPDO `tpdo`."""
        return sum((word & 0xFF) // 8 for word in self.mapping[tpdo])

    def entity(self, entity_id=None, index=None):
        """Remote `canopen.entities` item, by id or by index."""
        for entity in self.entities:
            if entity_id is not None and str(entity["id"]) == str(entity_id):
                return entity
            if index is not None and entity["index"] == index:
                return entity
        what = (
            f"entity '{entity_id}'"
            if entity_id is not None
            else f"entity index {index}"
        )
        raise RemoteError(
            f"{what} is not exposed by {self.name} (not in canopen.entities of {self.path})"
        )

    def domain(self, entity_id=None, index=None):
        return self.entity_states[self.entity(entity_id, index)["index"]][0]

    def state_names(self, entity_id=None, index=None):
        """State names of a remote entity; a state's position is its command index."""
        entity = self.entity(entity_id, index)
        return [name for name, _, _ in self.entity_states[entity["index"]][1]]

    def scaling(self, entity):
        """(min_value, max_value) of a scaled (size 1 / 2) sensor / number, defaults as in __init__.py."""
        size = entity.get("size")
        default_max = 254 if size == 1 else 65534  # 255 / 65535 reserved for NaN
        return entity.get("min_value", 0.0), entity.get("max_value", default_max)

    def state(self, entity_id=None, index=None, state=None):
        """Locate a remote entity state in the remote TPDOs.

        Returns dict(node_id, index, entity, state, tpdo, offset, size, domain,
        kind, cmd), plus min_value / max_value for scaled values; `state`
        defaults to the entity's first state ("state" / "value").
        """
        entity = self.entity(entity_id, index)
        entity_name = str(entity["id"])
        domain, state_kinds = self.entity_states[entity["index"]]
        names = [name for name, _, _ in state_kinds]
        if state is None:
            state = names[0]
        if state not in names:
            raise RemoteError(
                f"{self.name} entity '{entity_name}' has no state '{state}' (states: {', '.join(names)})"
            )
        cmd = names.index(state)
        _, size, kind = state_kinds[cmd]
        info = {"domain": domain, "kind": kind, "cmd": cmd}
        if kind == layout.KIND_VALUE and size != 4:
            info["min_value"], info["max_value"] = self.scaling(entity)
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
                    **info,
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
        except (
            Exception
        ) as err:  # YAML, packages or substitution errors of the remote file
            raise RemoteError(f"{path}: can't load node '{name}': {err}") from err
        _cache[key] = RemoteNode(name, path, config)
    return _cache[key]


# ---- RPDO plan (ESPHome-free) ----

# a gap in a remote TPDO: CANopenNode's dummy entry (index < 0x20, subindex 0), any length
DUMMY_INDEX = 0x0005
REMOTE_STATE_INDEX = 0x4000  # OD index of slot 0, see OBJECT_DICTIONARY.md
RPDO_MAX_ENTRIES = 8  # mapping entries per PDO (CiA 301)


def entity_commands_index(entity_index):
    return 0x2002 + 0x10 * entity_index


class PlanError(Exception):
    pass


def rpdo_plan(sources):
    """RPDOs of a node: [(node_id, tpdo, [(od index, subindex, bits)])], one per remote (node_id, tpdo).

    `sources`: dicts with node_id, tpdo, offset, size (bytes), tpdo_size (bytes the remote TPDO has, None if
    unknown: raw `rpdo:` entries without `tpdo_size`), a target: `slot` (remote state slot n) or
    `entity_index` + `cmd` (a local entity command), and a `label` for errors. Entries are sorted by offset;
    gaps and the rest of the remote TPDO are dummy entries, so the RPDO has the TPDO's length (CANopenNode
    reports longer frames with an EMCY). With no known tpdo_size the RPDO ends at the last mapped byte.
    """
    groups = {}
    for source in sources:
        groups.setdefault((source["node_id"], source["tpdo"]), []).append(source)
    plan = []
    for (node_id, tpdo), group in sorted(groups.items()):
        group.sort(key=lambda s: s["offset"])
        entries = []
        offset = 0
        prev = None
        for source in group:
            if source["offset"] < offset:
                raise PlanError(
                    f"{source['label']} overlaps {prev['label']} in TPDO {tpdo} of node 0x{node_id:02x}"
                )
            if source["offset"] > offset:
                entries.append((DUMMY_INDEX, 0, (source["offset"] - offset) * 8))
            if "slot" in source:
                entries.append(
                    (REMOTE_STATE_INDEX + source["slot"], 1, source["size"] * 8)
                )
            else:
                entries.append(
                    (
                        entity_commands_index(source["entity_index"]),
                        source["cmd"] + 1,
                        source["size"] * 8,
                    )
                )
            offset = source["offset"] + source["size"]
            prev = source
        sizes = {s["tpdo_size"] for s in group if s["tpdo_size"] is not None}
        if len(sizes) > 1:
            labels = ", ".join(
                f"{s['label']}: {s['tpdo_size']} B"
                for s in group
                if s["tpdo_size"] is not None
            )
            raise PlanError(
                f"TPDO {tpdo} of node 0x{node_id:02x} has different sizes: {labels}"
            )
        tpdo_size = sizes.pop() if sizes else offset
        if tpdo_size < offset:
            raise PlanError(
                f"{prev['label']} is outside the {tpdo_size} B TPDO {tpdo} of node 0x{node_id:02x}"
            )
        if tpdo_size > offset:
            entries.append((DUMMY_INDEX, 0, (tpdo_size - offset) * 8))
        if len(entries) > RPDO_MAX_ENTRIES:
            raise PlanError(
                f"RPDO from node 0x{node_id:02x} TPDO {tpdo} needs {len(entries)} mapping entries "
                f"(max {RPDO_MAX_ENTRIES})"
            )
        plan.append((node_id, tpdo, entries))
    return plan


def co_link_words(entries):
    """(index, sub, bits) entries as layout.co_link() words."""
    return [layout.co_link(index, sub, bits) for index, sub, bits in entries]
