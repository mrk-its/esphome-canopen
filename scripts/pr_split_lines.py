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
    1: "Node core, SDO server, identity",
    2: "Node monitoring, raw frames, loopback",
    3: "SDO client, NMT master, actions",
    4: "OD handler API, profile API",
    5: "PDOs, store/restore",
    6: "SDO block transfer",
}

FILES = {
    "canopen.cpp": 1,
    "driver_can.cpp": 1,
    "device_info.cpp": 1,
    "od.cpp": 1,
    "od.h": 1,
    "types.h": 1,
    "canopen.h": 1,
    "__init__.py": 1,
    "canopennode_port/CO_driver_target.h": 1,
    "node_monitor.cpp": 2,
    "frames.cpp": 2,
    "sdo_client.cpp": 3,
    "nmt_master.cpp": 3,
    "automation.h": 3,
    "od_handler.h": 4,
    "od_handler.cpp": 4,
    "canopen_node.h": 4,
    "canopen_node.cpp": 4,
    "pdo.cpp": 5,
    "pdo_storage.cpp": 5,
}

# banner title (start) -> PR
BANNERS = [
    ("Identity", 1),
    ("Object dictionary", 4),  # get_od() / write_od(): the API for other components
    ("Node monitoring", 2),
    ("Raw frames", 2),
    ("SDO client", 3),
    ("NMT master", 3),
    ("PDO", 5),
    ("Handlers", 4),
]

IFDEFS = {
    "USE_CANOPEN_SDO_CLIENT": 3,
    "USE_CANOPEN_NMT_MASTER": 3,
    "USE_CANOPEN_PDO": 5,
    "CANOPEN_SDO_BLOCK_SIZE": 6,
}

# C++ lines outside banners / ifdefs (in PR 1 files)
CPP_KEYWORDS = [
    (r"node_monitor|init_frame_listeners_|dispatch_frame_|next_local", 2),
    (r"\bextension", 4),  # od.h / od.cpp: an entry's handler
]

# types.h: keyword in a top-level declaration -> PR
TYPES = [
    (r"NodeEvent|node_event", 2),
    (r"FrameCallback|Emergency|decode_", 2),
    (r"NMTCommand", 3),
    (r"SDO", 3),
    (r"PdoMapping", 5),
]

# __init__.py top-level names -> PR
PY_BLOCKS = {
    "NMT_COMMANDS": 3,
    "_enable_once": 3,
    "enable_sdo_client": 3,
    "enable_nmt_master": 3,
    "SDO_BLOCK_BUFFER_SIZE": 6,
    "enable_sdo_block": 6,
    "enable_pdo": 5,
    "CANOPEN_NODE_SCHEMA": 4,
    "final_validate_canopen_node": 4,
    "register_node": 4,
    "KEY_LOCAL_NODES": 2,
    "HB_CLIENT_SCHEMA": 2,
    "validate_heartbeat_clients": 2,
    "validate_sdo_value": 3,
    "SDO_WRITE_SCHEMA": 3,
    "sdo_write_to_code": 3,
    "NMT_SCHEMA": 3,
    "_nmt_builder": 3,
    "nmt_to_code": 3,
}

PY_KEYWORDS = [
    (
        r"HEARTBEAT_CLIENTS|ON_NODE_EVENT|ON_HEARTBEAT|ON_EMCY|monitor_node|NodeEvent|Emergency|"
        r"set_next_local|LOCAL_NODES|^\s*local = |if local:|local\.append|same canbus|validate_heartbeat_clients|"
        r"every heartbeat / EMCY|events of the heartbeat",
        2,
    ),
    (r"RESPONSE_TIMEOUT|SDOWriteAction|SDOResult|NMTCommand", 3),
    (r"SDO_BLOCK_SIZE|sdo_block|block transfer", 6),
    (r"CANopenNode = |base class of components that drive", 4),
]


# tests (tests/components/canopen): file -> PR, and TEST names -> PR (a test goes with the newest feature it uses)
TEST_FILES = {
    "od_test.cpp": 1,
    "fake_canbus.h": 1,
    "__init__.py": 1,
    "common.yaml": 1,
    "frames_test.cpp": 2,
    "loopback_test.cpp": 2,
    "sdo_reply_test.cpp": 3,
    "canopen_node_test.cpp": 4,
    "pdo_test.cpp": 5,
    "storage_test.cpp": 5,
    "sdo_block_test.cpp": 6,
}
TEST_NAMES = [
    (r"CanopenSdoBlock", 6),
    (r"Sdo|SDO", 3),
    (
        r"CanopenOD, (Handler|ManyEntries|Errors|AfterFinalize)",
        4,
    ),  # od_test: these use OdObject handlers
]


def match(rules, text, default):
    for pattern, pr in rules:
        if re.search(pattern, text):
            return pr
    return default


def classify_cpp(lines: list[str], default: int) -> list[int]:
    out = []
    banner = None  # PR of the current banner section
    ifdef_stack: list[int | None] = []
    for line in lines:
        s = line.strip()
        m = re.match(r"// --- (.*) ---", s)
        if m:
            banner = match([(re.escape(t), p) for t, p in BANNERS], m.group(1), None)
        elif s in ("public:", "protected:") or s == "};":
            banner = None
        if s.startswith("#if"):
            ifdef_stack.append(match([(k, p) for k, p in IFDEFS.items()], s, None))
        pr = next((p for p in reversed(ifdef_stack) if p is not None), None)
        if pr is None:
            pr = banner
        if pr is None:
            pr = match(CPP_KEYWORDS, s, default) if default == 1 else default
        out.append(pr)
        if s.startswith("#endif") and ifdef_stack:
            ifdef_stack.pop()
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
        rel = (
            path.name
            if path.parent.name != "canopennode_port"
            else f"canopennode_port/{path.name}"
        )
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
        if rel not in FILES:
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
            for p in tests_dir.iterdir()
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
