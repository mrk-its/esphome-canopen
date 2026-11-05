#!/usr/bin/env python3
"""Non-blank lines of the upstream `canopen` hub per planned upstream PR (docs/canopennode.md, "New split").

Usage: scripts/pr_split_lines.py [path to esphome/components/canopen]   (default: ~/repos/esphome, branch canopen-full)
       scripts/pr_split_lines.py --annotate FILE                        (print FILE with the PR of every line)

Lines are classified by markers, not line numbers, so the script keeps working while the files change:
- whole files belong to one PR (FILES);
- C++: the `// --- <title> ---` feature banners of canopen.h / od.h (BANNERS, up to the next banner, `protected:`
  or the end of the class), `#ifdef USE_CANOPEN_*` blocks (IFDEFS), and keyword lines (CPP_KEYWORDS, e.g. the
  setup() hook calls in canopen.cpp);
- types.h: each top-level declaration with the comment block before it (TYPES);
- __init__.py: top-level blocks by name (PY_BLOCKS); inside the shared blocks (CONFIG_SCHEMA, to_code(), ...),
  keyword lines (PY_KEYWORDS) with their bracketed continuation or indented body.
Anything else is PR 1. Results are right to a few lines; check new code with --annotate.
"""

from __future__ import annotations

import collections
from pathlib import Path
import re
import sys

PRS = {
    1: "SDO client, sdo_read / sdo_write (hub without node_id)",
    2: "Raw frames, heartbeat / EMCY triggers",
    3: "Node core: node_id, runtime OD, NMT, HB producer, SDO server, identity; local loopback",
    4: "Node monitoring (HB consumer), NMT master",
    5: "OD handler API, profile API (CANopenNode)",
    6: "PDOs, store / restore",
    7: "SDO block transfer",
    8: "OTA on CiA 302-3 objects",
}

FILES = {
    "canopen.cpp": 1,
    "driver_can.cpp": 1,
    "types.h": 1,
    "canopen.h": 1,
    "__init__.py": 1,
    "sdo_client.cpp": 1,
    "automation.h": 1,
    "frames.cpp": 2,
    # PR 1 sets up 0x1280 for the client without the runtime OD (a fixed entry, not in this tree, ~30 lines)
    "od.cpp": 3,
    "od.h": 3,
    "device_info.cpp": 3,
    "node_monitor.cpp": 4,
    "nmt_master.cpp": 4,
    "od_handler.h": 5,
    "od_handler.cpp": 5,
    "canopen_node.h": 5,
    "canopen_node.cpp": 5,
    "pdo.cpp": 6,
    "pdo_storage.cpp": 6,
}
OTA_PR = 8

# banner title (start) -> PR; the first match wins
BANNERS = [
    ("Identity", 3),
    ("Object dictionary", 5),  # get_od() / write_od() / read_od(): the API for other components
    ("Node monitoring", 4),
    ("Raw frames", 2),
    ("SDO client", 1),
    ("NMT master", 4),
    ("PDO", 6),
    ("Handlers", 5),
]

IFDEFS = {
    "USE_CANOPEN_NODE": 3,
    "USE_CANOPEN_SDO_CLIENT": 1,
    "USE_CANOPEN_NMT_MASTER": 4,
    "USE_CANOPEN_PDO": 6,
    "CANOPEN_SDO_BLOCK_SIZE": 7,
}

# C++ functions (a column-0 line ending in `{` up to the column-0 `}`) by their signature -> PR
FUNCS = [
    (r"nmt_state_to_string", 2),  # with NMTState: the heartbeat trigger's state
    (r"to_nmt_state|set_nmt_state", 3),
]

# C++ keyword lines (in PR 1 files)
CPP_KEYWORDS = [
    (r"node_monitor", 4),
    (r"next_local", 3),
    (r"init_frame_listeners_|dispatch_frame_", 2),
    (r"\bextension", 5),  # od.h / od.cpp: an entry's handler
    (r"is_client_only|NO_NODE_ID|heartbeat_interval|hb_producer|start_operational|nmt_state|NMTState|err_reg_|"
     r"cob_emcy_|NMT state|ObjectDictionary od_|#include \"od.h\"", 3),
]

# A line belongs to the newest PR among its function, ifdefs, banner and keyword.

# types.h: keyword in a top-level declaration -> PR (first match)
TYPES = [
    (r"NodeEvent|node_event", 4),
    (r"NMTCommand", 4),
    (r"PdoMapping", 6),
    (r"FrameCallback|Emergency|decode_|NMTState|nmt_state", 2),
    (r"SDO", 1),
]

# __init__.py top-level names -> PR
PY_BLOCKS = {
    "_enable_once": 1,
    "enable_sdo_client": 1,
    "validate_sdo_value": 1,
    "SDO_WRITE_SCHEMA": 1,
    "sdo_write_to_code": 1,
    "SDO_READ_SCHEMA": 1,
    "sdo_read_to_code": 1,
    "NO_NODE_ID": 3,
    "_hub_configs": 3,
    "IDENTITY_SCHEMA": 3,
    "NODE_OPTIONS": 3,
    "validate_client_only_options": 3,
    "validate_client_only": 3,
    "KEY_LOCAL_NODES": 3,
    "HB_CLIENT_SCHEMA": 4,
    "validate_heartbeat_clients": 4,
    "NMT_COMMANDS": 4,
    "enable_nmt_master": 4,
    "NMT_SCHEMA": 4,
    "_nmt_builder": 4,
    "nmt_to_code": 4,
    "CANOPEN_NODE_SCHEMA": 5,
    "require_node_id": 5,
    "final_validate_canopen_node": 5,
    "register_node": 5,
    "enable_pdo": 6,
    "SDO_BLOCK_BUFFER_SIZE": 7,
    "enable_sdo_block": 7,
}

# keyword lines inside the shared blocks (imports, CONFIG_SCHEMA, to_code(), ...); the first match wins
PY_KEYWORDS = [
    (r"HEARTBEAT_CLIENTS|ON_NODE_EVENT|monitor_node|NodeEvent|validate_heartbeat_clients|events of the heartbeat", 4),
    (r"ON_HEARTBEAT|ON_EMCY|Emergency|every heartbeat / EMCY|add_on_heartbeat|add_on_emcy", 2),
    (r"set_next_local|LOCAL_NODES|^\s*local = |if local:|local\.append|same canbus|CONF_CANBUS_ID\] ==", 3),
    (r"SDO_BLOCK_SIZE|sdo_block|block transfer", 7),
    (r"CANopenNode = |base class of components that drive", 5),
    (r"NMTCommand|CONF_COMMAND", 4),
    (r"NMTState = ", 2),
    (
        r"NODE_ID|HEARTBEAT_INTERVAL|NMT_STARTUP|NMT_STATE|IDENTITY|DEVICE_NAME|HARDWARE_VERSION|SOFTWARE_VERSION|"
        r"VENDOR_ID|PRODUCT_CODE|REVISION_NUMBER|SERIAL_NUMBER|set_identity|USE_CANOPEN_NODE|CONF_SDO_CLIENT|"
        r"sdo_client applies|validate_client_only|NO_NODE_ID|EsphomeError|0x1016 \(consumer|CONF_TIMEOUT|"
        r"_final_validate|FINAL_VALIDATE|for other in fv",
        3,
    ),
]


# tests (tests/components/canopen): file -> PR, and TEST names -> PR (a test goes with the newest feature it uses)
TEST_FILES = {
    "fake_canbus.h": 1,
    "__init__.py": 1,
    "common.yaml": 1,
    "sdo_reply_test.cpp": 1,
    "frames_test.cpp": 2,
    "od_test.cpp": 3,
    "client_only_test.cpp": 3,  # its server is a local node
    "loopback_test.cpp": 3,
    "canopen_node_test.cpp": 5,
    "pdo_test.cpp": 6,
    "storage_test.cpp": 6,
    "sdo_block_test.cpp": 7,
}
TEST_NAMES = [
    (r"CanopenSdoBlock", 7),
    (
        r"CanopenOD, (Handler|ManyEntries|Errors|AfterFinalize|ValueSize|Read)",
        5,
    ),  # od_test: these use OdObject handlers or the OD access of od_handler.cpp
]


def match(rules, text, default):
    for pattern, pr in rules:
        if re.search(pattern, text):
            return pr
    return default


def classify_cpp(lines: list[str], default: int) -> list[int]:
    out = []
    banner = None  # PR of the current banner section
    func = None  # PR of the current function (FUNCS)
    ifdef_stack: list[int | None] = []
    for line in lines:
        s = line.strip()
        m = re.match(r"// --- (.*) ---", s)
        if m:
            banner = match([(re.escape(t), p) for t, p in BANNERS], m.group(1), None)
        elif s in ("public:", "protected:") or s == "};":
            banner = None
        if (
            line[:1] not in ("", " ", "#", "/", "}")
            and line.rstrip().endswith("{")
            and not line.startswith(("namespace", "extern"))
        ):
            func = match(FUNCS, line, None)
        if s.startswith("#if"):
            ifdef_stack.append(match([(k, p) for k, p in IFDEFS.items()], s, None))
        candidates = [p for p in ifdef_stack if p is not None]
        for p in (banner, func, match(CPP_KEYWORDS, s, None) if default == 1 else None):
            if p is not None:
                candidates.append(p)
        out.append(max(candidates) if candidates else default)
        if s.startswith("#endif") and ifdef_stack:
            ifdef_stack.pop()
        if line.startswith("}"):
            func = None
    return out


def classify_types(lines: list[str]) -> list[int]:
    out = [1] * len(lines)
    decl = re.compile(r"^(enum|struct|class|using|inline|const |template)")
    i = 0
    while i < len(lines):
        if decl.match(lines[i]):
            # the declaration up to the next blank line at depth 0, with the comment block before it
            start = i
            while start > 0 and lines[start - 1].startswith("//"):
                start -= 1
            end = i
            depth = 0
            while end < len(lines):
                depth += lines[end].count("{") - lines[end].count("}")
                if depth <= 0 and (end + 1 >= len(lines) or not lines[end + 1].strip()):
                    break
                end += 1
            pr = match(TYPES, "".join(lines[i : end + 1]), 1)
            for k in range(start, end + 1):
                out[k] = pr
            i = end + 1
        else:
            i += 1
    return out


def indent(line: str) -> int:
    return len(line) - len(line.lstrip())


def classify_py(lines: list[str]) -> list[int]:
    out = [1] * len(lines)
    # top-level blocks: from a column-0 line to the next one (comments before a block belong to it)
    starts: list[int] = []
    for i, line in enumerate(lines):
        if not line or line[0].isspace() or line.startswith((")", "]", "}", "#")):
            continue
        # a decorated def belongs to its decorator's block
        if (
            starts
            and lines[starts[-1]].startswith("@")
            and line.startswith(("def ", "async def "))
        ):
            continue
        starts.append(i)
    for n, start in enumerate(starts):
        end = starts[n + 1] if n + 1 < len(starts) else len(lines)
        first = start
        while first > 0 and lines[first - 1].startswith("#"):
            first -= 1
        head = lines[start]
        if head.startswith("@"):  # decorator: the name is on the def line
            head = next(
                (x for x in lines[start:end] if x.startswith(("def ", "async def "))),
                head,
            )
        m = re.match(r"(?:async )?(?:def )?([A-Za-z_][A-Za-z0-9_]*)", head)
        pr = PY_BLOCKS.get(m.group(1)) if m else None
        if pr is not None:
            for k in range(first, end):
                out[k] = pr
            continue
        # a shared block: keyword lines with their continuation
        k = start
        while k < end:
            line = lines[k]
            pr = match(PY_KEYWORDS, line, None)
            if pr is None and line.rstrip().endswith(("(", "[", "{")) and k + 1 < end:
                pr = match(
                    PY_KEYWORDS, lines[k + 1], None
                )  # opener of an item whose keyword is inside
            if pr is None:
                k += 1
                continue
            first = k
            if line.strip().startswith("#"):
                # a comment about the next statement: it goes with the comment
                k += 1
                while k < end and (
                    not lines[k].strip() or lines[k].strip().startswith("#")
                ):
                    k += 1
                if k >= end:
                    break
                line = lines[k]
            # comment lines right before the statement belong to it
            while first > start and lines[first - 1].strip().startswith("#"):
                first -= 1
            for c in range(first, k):
                out[c] = pr
            base = indent(line)
            depth = 0
            j = k
            while j < end:
                depth += sum(lines[j].count(c) for c in "([{") - sum(
                    lines[j].count(c) for c in ")]}"
                )
                out[j] = pr
                if depth <= 0:
                    break
                j += 1
            if lines[j].rstrip().endswith(":"):  # an if / for: its indented body
                j += 1
                while j < end and (not lines[j].strip() or indent(lines[j]) > base):
                    out[j] = pr
                    j += 1
                j -= 1
            k = j + 1
    return out


def classify(path: Path, rel: str) -> list[int]:
    lines = path.read_text(encoding="utf-8").splitlines()
    if rel.startswith("ota/"):
        return [OTA_PR] * len(lines)
    default = FILES.get(rel, 1)
    if rel == "types.h":
        return classify_types(lines)
    if rel.endswith(".py"):
        return classify_py(lines)
    if default != 1:
        return [default] * len(lines)
    return classify_cpp(lines, default)


def classify_test(path: Path) -> list[int]:
    lines = path.read_text(encoding="utf-8").splitlines()
    if path.parent.name == "ota" or path.name.startswith("test-ota."):
        return [OTA_PR] * len(lines)
    default = TEST_FILES.get(path.name, 1)
    out = [default] * len(lines)
    if path.suffix != ".cpp":
        return out
    tests = [i for i, line in enumerate(lines) if re.match(r"TEST(_F)?\(", line)]
    for n, start in enumerate(tests):
        first = start
        while first > 0 and lines[first - 1].startswith("//"):
            first -= 1
        end = tests[n + 1] if n + 1 < len(tests) else len(lines)
        while end > start and lines[end - 1].startswith("//"):
            end -= 1  # the next test's comment
        pr = match(TEST_NAMES, lines[start], default)
        for k in range(first, end):
            out[k] = pr
    return out


def is_comment(line: str) -> bool:
    s = line.strip()
    return s.startswith(("//", "/*", "*", "# ")) or s == "#"


def main() -> None:
    args = sys.argv[1:]
    if args and args[0] == "--annotate":
        path = Path(args[1])
        rel = path.name
        lines = path.read_text(encoding="utf-8").splitlines()
        for line, pr in zip(lines, classify(path, rel)):
            print(f"{pr if line.strip() else ' '} | {line}")
        return
    root = Path(
        args[0] if args else Path.home() / "repos/esphome/esphome/components/canopen"
    )
    files = sorted(
        p
        for p in root.rglob("*")
        if p.suffix in (".cpp", ".h", ".py") and "__pycache__" not in p.parts
    )
    counts: dict[int, collections.Counter] = collections.defaultdict(
        collections.Counter
    )
    unknown = []
    for path in files:
        rel = str(path.relative_to(root))
        if rel not in FILES and not rel.startswith("ota/"):
            unknown.append(rel)
        kind = path.suffix.lstrip(".")
        for line, pr in zip(
            path.read_text(encoding="utf-8").splitlines(), classify(path, rel)
        ):
            if not line.strip():
                continue
            counts[pr][kind] += 1
            counts[pr]["total"] += 1
            if is_comment(line):
                counts[pr]["comment"] += 1
    print("| PR | Content | .cpp | .h | .py | Total | Comment-only |")
    print("|---|---|---|---|---|---|---|")
    for pr in sorted(counts):
        c = counts[pr]
        print(
            f"| {pr} | {PRS[pr]} | {c['cpp']} | {c['h']} | {c['py']} | **{c['total']}** | {c['comment']} |"
        )
    print(f"\nall: {sum(c['total'] for c in counts.values())} non-blank lines")
    tests_dir = root.parents[2] / "tests/components/canopen"
    if tests_dir.is_dir():
        tests: dict[int, collections.Counter] = collections.defaultdict(
            collections.Counter
        )
        for path in sorted(
            p
            for p in [*tests_dir.iterdir(), *(tests_dir / "ota").glob("*")]
            if p.is_file() and p.suffix in (".cpp", ".h", ".py", ".yaml")
        ):
            name = (
                path.name
                if path.name in TEST_FILES or not path.name.startswith("test.")
                else "common.yaml"
            )
            for line, pr in zip(
                path.read_text(encoding="utf-8").splitlines(), classify_test(path)
            ):
                if line.strip():
                    tests[pr][name] += 1
        print("\ntests (non-blank lines):")
        for pr in sorted(tests):
            parts = ", ".join(f"{k} {v}" for k, v in sorted(tests[pr].items()))
            print(f"  PR {pr}: {sum(tests[pr].values())} ({parts})")
    if unknown:
        print(f"not in FILES (counted by markers as PR 1 files): {', '.join(unknown)}")


if __name__ == "__main__":
    main()
