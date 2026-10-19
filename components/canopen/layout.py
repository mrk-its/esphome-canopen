"""TPDO layout of CANopen entities, computed from the YAML config.

This module is the single source of truth for which OD states an entity has
and how they are mapped into TPDOs. The C++ side receives the capability bits
from codegen and adds states in the order defined here; at boot it hashes the
actual TPDO mapping (0x1A00..0x1A07) and compares it with `layout_hash()`.

No ESPHome imports here, so it can be unit-tested on its own.
"""

DOMAIN_SENSOR = "sensor"
DOMAIN_NUMBER = "number"
DOMAIN_BINARY_SENSOR = "binary_sensor"
DOMAIN_SWITCH = "switch"
DOMAIN_LIGHT = "light"
DOMAIN_COVER = "cover"
DOMAIN_ALARM = "alarm_control_panel"

TPDO_COUNT = 8

# Light capability bits (0x2001:N bits 16..23), see OBJECT_DICTIONARY.md
LIGHT_CAP_ON_OFF = 1
LIGHT_CAP_BRIGHTNESS = 2
LIGHT_CAP_COLOR_TEMPERATURE = 4

# Cover capability bits
COVER_CAP_POSITION = 1
COVER_CAP_TILT = 2

CAPS_NAMES = {
    DOMAIN_LIGHT: {
        "on_off": LIGHT_CAP_ON_OFF,
        "brightness": LIGHT_CAP_BRIGHTNESS,
        "color_temperature": LIGHT_CAP_COLOR_TEMPERATURE,
    },
    DOMAIN_COVER: {
        "position": COVER_CAP_POSITION,
        "tilt": COVER_CAP_TILT,
    },
}

def flag(conf, key):
    """Boolean option of a validated or raw (possibly substituted, so string) config."""
    value = conf.get(key, False)
    if isinstance(value, str):
        return value.strip().lower() in ("true", "yes", "on", "1")
    return bool(value)


# Supported color modes of light platforms, mirroring each platform's
# get_traits() in ESPHome.
LIGHT_PLATFORM_COLOR_MODES = {
    "binary": lambda conf: {"ON_OFF"},
    "monochromatic": lambda conf: {"BRIGHTNESS"},
    "cwww": lambda conf: {"COLD_WARM_WHITE"},
    "color_temperature": lambda conf: {"COLOR_TEMPERATURE"},
    "hbridge": lambda conf: {"COLD_WARM_WHITE"},
    "shelly_dimmer": lambda conf: {"BRIGHTNESS"},
    "sonoff_d1": lambda conf: {"BRIGHTNESS"},
    "rgb": lambda conf: {"RGB"},
    "rgbw": lambda conf: (
        {"RGB", "WHITE"} if flag(conf, "color_interlock") else {"RGB_WHITE"}
    ),
    "rgbww": lambda conf: (
        {"RGB", "COLD_WARM_WHITE"}
        if flag(conf, "color_interlock")
        else {"RGB_COLD_WARM_WHITE"}
    ),
    "rgbct": lambda conf: (
        {"RGB", "COLOR_TEMPERATURE"}
        if flag(conf, "color_interlock")
        else {"RGB_COLOR_TEMPERATURE", "COLOR_TEMPERATURE"}
    ),
}

# Cover capabilities of cover platforms, mirroring each platform's get_traits().
COVER_PLATFORM_CAPS = {
    "time_based": lambda conf: COVER_CAP_POSITION,
    "current_based": lambda conf: COVER_CAP_POSITION,
    "endstop": lambda conf: COVER_CAP_POSITION,
    "feedback": lambda conf: COVER_CAP_POSITION,
    "template": lambda conf: (
        (
            COVER_CAP_POSITION
            if flag(conf, "has_position") or "position_action" in conf
            else 0
        )
        | (COVER_CAP_TILT if "tilt_action" in conf else 0)
    ),
    # third-party: https://github.com/bruxy70/Venetian-Blinds-Control
    "venetian_blinds": lambda conf: COVER_CAP_POSITION | COVER_CAP_TILT,
}


# sizes of `template_entities` state types (TYPE_TO_CANOPEN_TYPE in __init__.py)
TEMPLATE_STATE_SIZES = {
    "uint8": 1,
    "uint16": 2,
    "uint32": 4,
    "int8": 1,
    "int16": 2,
    "int32": 4,
}


class LayoutError(Exception):
    def __init__(self, message, entity_pos=None):
        super().__init__(message)
        # position of the offending entity in the node's `entities` list, if known
        self.entity_pos = entity_pos


# Color capabilities of ESPHome color modes (light::ColorMode in color_mode.h),
# ON_OFF left out: every mode has it.
COLOR_MODE_CAPABILITIES = {
    "ON_OFF": set(),
    "BRIGHTNESS": {"BRIGHTNESS"},
    "WHITE": {"BRIGHTNESS", "WHITE"},
    "COLOR_TEMPERATURE": {"BRIGHTNESS", "COLOR_TEMPERATURE"},
    "COLD_WARM_WHITE": {"BRIGHTNESS", "COLD_WARM_WHITE"},
    "RGB": {"BRIGHTNESS", "RGB"},
    "RGB_WHITE": {"BRIGHTNESS", "RGB", "WHITE"},
    "RGB_COLOR_TEMPERATURE": {"BRIGHTNESS", "RGB", "WHITE", "COLOR_TEMPERATURE"},
    "RGB_COLD_WARM_WHITE": {"BRIGHTNESS", "RGB", "COLD_WARM_WHITE"},
}


def light_caps_from_color_modes(color_modes):
    """Same rules as the trait check in LightStateEntity::setup().

    Brightness and color temperature bits are exclusive (Home Assistant
    doesn't allow `brightness` combined with other color modes); color
    temperature implies the brightness state.
    """
    capabilities = set().union(*(COLOR_MODE_CAPABILITIES[mode] for mode in color_modes))
    caps = 0
    if "ON_OFF" in color_modes:
        caps |= LIGHT_CAP_ON_OFF
    if capabilities & {"COLOR_TEMPERATURE", "COLD_WARM_WHITE"}:
        caps |= LIGHT_CAP_COLOR_TEMPERATURE
    elif "BRIGHTNESS" in capabilities:
        caps |= LIGHT_CAP_BRIGHTNESS
    return caps


def caps_from_names(domain, names):
    bits = CAPS_NAMES[domain]
    caps = 0
    for name in names:
        if name not in bits:
            raise LayoutError(
                f"Unknown {domain} capability '{name}', valid: {', '.join(bits)}"
            )
        caps |= bits[name]
    return caps


def entity_caps(domain, platform_config, caps_override=None):
    """Capability bits of a light / cover entity, None for other domains.

    `caps_override` (list of capability names from the `caps:` option) takes
    precedence; otherwise caps are derived from the platform config.
    """
    if domain not in CAPS_NAMES:
        if caps_override is not None:
            raise LayoutError(f"'caps' is not supported for {domain} entities")
        return None
    if caps_override is not None:
        return caps_from_names(domain, caps_override)
    platform = (platform_config or {}).get("platform")
    if domain == DOMAIN_LIGHT:
        modes = LIGHT_PLATFORM_COLOR_MODES.get(platform)
        if modes is None:
            raise LayoutError(_unknown_platform_msg(domain, platform))
        return light_caps_from_color_modes(modes(platform_config))
    caps = COVER_PLATFORM_CAPS.get(platform)
    if caps is None:
        raise LayoutError(_unknown_platform_msg(domain, platform))
    return caps(platform_config)


def _unknown_platform_msg(domain, platform):
    return (
        f"Can't determine capabilities of {domain} platform '{platform}', "
        f"set them explicitly with 'caps: [...]' "
        f"(valid: {', '.join(CAPS_NAMES[domain])})"
    )


# State kinds: how a state is encoded on the wire (see OBJECT_DICTIONARY.md)
KIND_BOOL = "bool"  # 0 / 1
KIND_PERCENTAGE = "percentage"  # UINT8 0..254 -> 0..1, 255 unknown
KIND_COLOR_TEMP = "color_temp"  # UINT8 0..254 -> 100..1000 mireds
KIND_COVER_STATE = "cover_state"  # 0 open, 1 opening, 2 closed, 3 closing
KIND_ALARM_STATE = "alarm_state"  # 0..9
KIND_VALUE = "value"  # sensor / number: REAL32, or UINT8 / UINT16 scaled to min_value..max_value

# C++ RemoteStateKind (remote_entities.h)
KIND_CODES = {
    KIND_BOOL: 0,
    KIND_PERCENTAGE: 1,
    KIND_COLOR_TEMP: 2,
    KIND_COVER_STATE: 3,
    KIND_ALARM_STATE: 4,
    KIND_VALUE: 5,
}


def entity_state_kinds(domain, caps=None, size=None):
    """(name, size in bytes, kind) of the entity's OD states, in subindex order.

    Must match the order of od_add_state() calls in entities.cpp. Commands are
    added in the same order, so a state's position is also its command index.
    """
    if domain in (DOMAIN_SENSOR, DOMAIN_NUMBER):
        size = 4 if size is None else size
        if size not in (1, 2, 4):
            raise LayoutError(f"Unsupported {domain} size: {size}")
        return [("value", size, KIND_VALUE)]
    if domain in (DOMAIN_BINARY_SENSOR, DOMAIN_SWITCH):
        return [("state", 1, KIND_BOOL)]
    if domain == DOMAIN_ALARM:
        return [("state", 1, KIND_ALARM_STATE)]
    if domain == DOMAIN_LIGHT:
        states = [("state", 1, KIND_BOOL)]
        if caps & (LIGHT_CAP_BRIGHTNESS | LIGHT_CAP_COLOR_TEMPERATURE):
            states.append(("brightness", 1, KIND_PERCENTAGE))
        if caps & LIGHT_CAP_COLOR_TEMPERATURE:
            states.append(("color_temp", 1, KIND_COLOR_TEMP))
        return states
    if domain == DOMAIN_COVER:
        states = [("state", 1, KIND_COVER_STATE)]
        if caps & COVER_CAP_POSITION:
            states.append(("position", 1, KIND_PERCENTAGE))
        if caps & COVER_CAP_TILT:
            states.append(("tilt", 1, KIND_PERCENTAGE))
        return states
    raise LayoutError(f"Unsupported entity domain: {domain}")


def entity_states(domain, caps=None, size=None):
    """(name, size in bytes) of the entity's OD states, in subindex order."""
    return [(name, size) for name, size, _ in entity_state_kinds(domain, caps, size)]


def entity_state_sizes(domain, caps=None, size=None):
    return [size for _, size in entity_states(domain, caps, size)]


def tpdo_number(entity_config):
    """`tpdo:` of an entity: int, or {number: ...} mapping; -1 when not set."""
    tpdo = entity_config.get("tpdo", -1)
    return tpdo["number"] if isinstance(tpdo, dict) else tpdo


def node_states(node_config, resolve_entity):
    """(entity_index, tpdo, size, label) of all OD states of a node.

    In the order they are added to the OD at runtime: template entity states
    (added in codegen), then entities by index (CanopenComponent::setup()).
    Labels are "<entity id>.<state name>".

    `node_config` is a `canopen:` node config (validated or raw YAML);
    `resolve_entity(entity_config)` returns (domain, platform_config) of the
    entity the config refers to.
    """
    states = []
    for tmpl_entity in node_config.get("template_entities") or ():
        for num, state in enumerate(tmpl_entity.get("states") or (), 1):
            size = TEMPLATE_STATE_SIZES[state["type"]]
            label = f"template{tmpl_entity['index']}.{num}"
            states.append((tmpl_entity["index"], tmpl_entity.get("tpdo", -1), size, label))
    entities = list(enumerate(node_config.get("entities") or ()))
    for pos, entity_config in sorted(entities, key=lambda x: x[1]["index"]):
        try:
            domain, platform_config = resolve_entity(entity_config)
            caps = entity_caps(domain, platform_config, entity_config.get("caps"))
            entity_state_list = entity_states(domain, caps, entity_config.get("size"))
        except LayoutError as err:
            raise LayoutError(f"entity '{entity_config['id']}': {err}", pos) from err
        tpdo = tpdo_number(entity_config)
        states.extend(
            (entity_config["index"], tpdo, size, f"{entity_config['id']}.{name}")
            for name, size in entity_state_list
        )
    return states


def co_link(index, sub, bits):
    """CO_LINK() from canopen-stack: PDO mapping entry."""
    return ((index & 0xFFFF) << 16) | ((sub & 0xFF) << 8) | (bits & 0xFF)


def entity_od_index(entity_index):
    """ENTITY_INDEX() from canopen.h."""
    return 0x2000 + entity_index * 0x10


def tpdo_mapping(states):
    """TPDO mapping entries (0x1A00 + n, subindex 1..) in OD creation order.

    `states` is an iterable of (entity_index, tpdo_number, size[, label]) in the
    order od_add_state() is called at runtime; tpdo_number < 0 means not mapped.
    Returns a list of TPDO_COUNT lists of CO_LINK words.
    """
    mapping = [[] for _ in range(TPDO_COUNT)]
    state_sub = {}
    for entity_index, tpdo, size, *_ in states:
        sub = state_sub.get(entity_index, 0) + 1
        state_sub[entity_index] = sub
        if tpdo < 0:
            continue
        if tpdo >= TPDO_COUNT:
            raise LayoutError(f"Invalid TPDO number {tpdo} (0..{TPDO_COUNT - 1})")
        mapping[tpdo].append(co_link(entity_od_index(entity_index) + 1, sub, size * 8))
    for tpdo, entries in enumerate(mapping):
        if sum(word & 0xFF for word in entries) > 64:
            raise LayoutError(
                f"TPDO {tpdo} exceeds 8 bytes: {tpdo_offsets_str(entries)}"
            )
    return mapping


def tpdo_offsets_str(entries):
    return ", ".join(
        f"0x{word >> 16:04x}:{(word >> 8) & 0xFF} ({(word & 0xFF) // 8} B)"
        for word in entries
    )


def tpdo_cob_id(tpdo, node_id):
    """Default TPDO COB-IDs, see od_setup_tpdo() in canopen.cpp."""
    base = 0x180 + 0x100 * tpdo if tpdo < 4 else 0x200 + 0x100 * (tpdo - 4)
    return base + node_id


def layout_table(states, node_id):
    """Human-readable TPDO layout, one line per TPDO.

    `states` as for tpdo_mapping(), with labels.
    """
    lines = []
    for tpdo in range(TPDO_COUNT):
        offset, fields = 0, []
        for _, state_tpdo, size, *label in states:
            if state_tpdo != tpdo:
                continue
            name = label[0] if label else "?"
            fields.append(f"[{offset}{'' if size == 1 else f'..{offset + size - 1}'}] {name}")
            offset += size
        if fields:
            lines.append(
                f"TPDO{tpdo} (COB-ID 0x{tpdo_cob_id(tpdo, node_id):03x}, {offset} B): "
                + ", ".join(fields)
            )
    return lines


def fnv1a_32(data):
    h = 0x811C9DC5
    for b in data:
        h ^= b
        h = (h * 0x01000193) & 0xFFFFFFFF
    return h


def layout_hash(mapping):
    """FNV-1a 32 over (count, entries...) of every TPDO, as little-endian u32.

    Must match CanopenComponent::compute_tpdo_layout_hash().
    """
    words = []
    for entries in mapping:
        words.append(len(entries))
        words.extend(entries)
    return fnv1a_32(b"".join(w.to_bytes(4, "little") for w in words))
