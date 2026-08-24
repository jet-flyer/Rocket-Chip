"""Grok+Claude combined overlay vs owner disposition.

Finding packs stay frozen. This writes L2P5_GROK_CLAUDE_OVERLAY.md.
Owner walk stays a separate chunk. IDs are never merged.
"""
from __future__ import annotations

import importlib.util
import json
import re
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
LOG = HERE / "L2P5_DISPOSITION_LOG.md"
CLUSTERS = HERE / "L2P5_WN_CLUSTERS.md"
OUT_MD = HERE / "L2P5_GROK_CLAUDE_OVERLAY.md"
OUT_JSON = HERE / "_grok_claude_overlay.json"

spec = importlib.util.spec_from_file_location("join", HERE / "_gen_three_walk_join.py")
join = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(join)

WN_ROW = re.compile(
    r"^\| (WN-\d+) \| (REMEDIATE|ACCEPT|DEFER) \| (closed|labeled) \| (.*) \|$"
)
SEC_H = re.compile(r"^## (.+?) \(\d+\)")
CLUSTER_IDS = re.compile(
    r"^\*\*(.+?):\*\* ((?:WN-\d+(?:, WN-\d+)*)+)\s*$", re.M
)

# Owner-bucket names that rewrote comments / suppressions in Phase 3.
POLICY_SITTINGS = {
    "In-source NOLINT / suppressions",
    "Doxygen + header comment-density policy",
    "Process archaeology in comments",
}

COMMENT_TITLE = re.compile(
    r"banner|essay|density|doxygen|tombstone|IVP|council|archaeology|"
    r"process|Stage |comment ratio|@brief|@param|NOLINT|narrative|history tag",
    re.I,
)

LIVE_CONTRACT = re.compile(
    r"\bdisagree|contradict|does not match|does not exist|no such |"
    r"unimplemented|never fires|fail-open|plain bool|no barrier|no atomic|"
    r"wrong[- ]core|NVIC|literal 0|named, not declared|opposite overflow|"
    r"drop-oldest|drop-newest|cannot both|false on numeric|not backed by|"
    r"dual-owned|unenforceable|silently depends|overstates|misstate|"
    r"states opposite|is false|no consumer|wrong byte|collision at|"
    r"same AGL|erase-only|cached alias|clobbers|2-of-N|"
    r"labeled unused|does not implement|not the source|"
    r"stale (number|contract|byte|size|pointer|comment)",
    re.I,
)

ESSAY_NOISE = re.compile(
    r"too long|comment density|comment ratio|Doxygen|@brief|@param|@file|"
    r"\bIVP-|\bcouncil\b|Stage [0-9TL]|tombstone|process archaeology|"
    r"banner hosts|narrative|ticket tag|W-6|superfluous|essay",
    re.I,
)

DEFER_BUCKETS = {
    "starcom",
    "early-impl",
    "rc-os",
    "codegen",
    "rf-legal",
}

# Hand-placed triples from L2P5_THREE_WALK_COMPARE.md vs the log.
CURATED_TRIPLES = [
    {
        "issue": "`RC_ASSERT` defined, zero call sites",
        "wn": "WN-007",
        "gwf": ["GWF-016"],
        "cw": ["CW-B01-05"],
        "expect": "cleared",
    },
    {
        "issue": "Unknown / unmatched board → silent Feather pin map",
        "wn": "WN-020",
        "gwf": ["GWF-023"],
        "cw": ["CW-B02-06"],
        "expect": "cleared",
    },
    {
        "issue": "Phantom `version_string()`; no such API",
        "wn": "WN-011",
        "gwf": ["GWF-102"],
        "cw": ["CW-B06-07"],
        "expect": "cleared",
    },
    {
        "issue": "ICM-20948 lazy mag re-init on the 1 kHz read path",
        "wn": "WN-089",
        "gwf": ["GWF-146"],
        "cw": ["CW-B10-02"],
        "expect": "defer-home",
    },
    {
        "issue": "Deprecated health aliases live, no consumers, wrong byte",
        "wn": "WN-051",
        "gwf": ["GWF-080"],
        "cw": ["CW-B05-12"],
        "expect": "cleared",
    },
    {
        "issue": "`g_imu` Core 0 init / Core 1 use; handoff not on the header",
        "wn": "WN-002",
        "gwf": ["GWF-006"],
        "cw": [],
        "expect": "cleared",
        "note": "Claude's live hit on this leaf is the init-flag bools, not this handle.",
    },
]

CURATED_NEIGHBORHOOD = [
    {
        "issue": "Flash-layout banner / `flash_layout_valid()` stale numbers",
        "wn": "WN-060",
        "gwf": ["GWF-096", "GWF-097", "GWF-098"],
        "cw": ["CW-B06-03", "CW-B06-04"],
        "expect": "affected",
        "note": "Sitting 13 made constexprs the code SSOT. Re-read whether agent stale-number P is gone.",
    },
    {
        "issue": "DPS310 OS/rate table / fabricated MaxRate",
        "wn": "WN-090",
        "gwf": ["GWF-147"],
        "cw": ["CW-B10-05"],
        "expect": "affected",
        "note": "Owner asked home; agents named a numeric lie. Comment sitting trimmed the table — check the numbers.",
    },
    {
        "issue": "Log overflow header/body clash (drop-oldest vs drop-newest)",
        "wn": "WN-003",
        "gwf": ["GWF-009", "GWF-284"],
        "cw": ["CW-B01-03"],
        "expect": "affected",
        "note": "WN-201 also. Archaeology sitting rewrote comments; the ring policy may still be a live P.",
    },
    {
        "issue": "LED split homes / collision at 28",
        "wn": "WN-177",
        "gwf": ["GWF-087"],
        "cw": ["CW-B21-01"],
        "expect": "affected",
        "note": "Owner closed pointer-at-led_patterns.h. Agents named a numeric collision. Different P — re-read.",
    },
]

CURATED_AGREE_OWNER_SILENT = [
    ("eskf_to_fused_state() does not exist", "GWF-093", "CW-B06-05", "phantom-symbol"),
    ("kRcLogRingBytes named, not declared", "GWF-011", "CW-B01-04", "phantom-symbol"),
    ("Dashboard Temp: is literal 0", "GWF-487", "CW-B44-01", "dashboard-cli"),
    ("Dashboard Alt and Baro print the same AGL value", "GWF-488", "CW-B44-02", "dashboard-cli"),
    ("Station distance stale uses MET vs station uptime", "GWF-481", "CW-B43-01", "dashboard-cli"),
    ("Boot-summary FAIL count vs FAIL list disagree", "GWF-485", "CW-B43-05", "dashboard-cli"),
    ("PSRAM hard gate is erase-only", "GWF-309", "CW-B26-03", "psram-qmi"),
    ("psram_self_test readback through cached alias", "GWF-310", "CW-B26-04", "psram-qmi"),
    ("ring_init clobbers the header ring_recover reads", "GWF-288", "CW-B25-06", "log-ring"),
    ("Failed ring_init still marks initialized", "GWF-292", "CW-B25-05", "log-ring"),
    ("core1_i2c_pause can return without ack / fail-open", "GWF-383", "CW-B34-03", "cross-core"),
    ("Pause/resume do not nest", "GWF-386", "CW-B34-02", "cross-core"),
    ("Anomalous-boot 2-of-N cannot fire", "GWF-349", "CW-B31-08", "safety-ssot"),
    ("FlightMetadata 14B vs sizeof 16", "GWF-079", "CW-B05-11", "packaging"),
    ("PMTK314 comment lists GSV; literal disables it", "GWF-132", "CW-B09-07", "gps-uart"),
    ("UART RX ring: volatile claimed sufficient", "GWF-135", "CW-B09-04", "gps-uart"),
    ("gps_uart_reinit() wrong-core NVIC (live on vehicle)", "GWF-135", "CW-B09-03", "gps-uart"),
    ("Calibration session objects, both cores, no barrier", "GWF-222", "CW-B18-01", "cross-core"),
    ("Seqlock + six atomics declared in two homes", "GWF-007", "CW-X4-09", "cross-core"),
]

CURATED_DISAGREE = [
    {
        "p": "Is the unfenced QMI direct-mode window in `psram_configure_qmi` a live IRQ hazard?",
        "gwf": "GWF-311",
        "g_answer": "Yes (high). Same `qmi_hw` is IRQ-fenced in `psram_detect`, unfenced in configure. **Kept.**",
        "cw": "CW-B26-05",
        "c_answer": (
            "REFUTED. Only caller is `psram_init` from `main` before USB / QF tick / Core 1; "
            "no IRQ armed. **Refute fails.** `init_gps_early()` → `sleep_ms` arms the default "
            "alarm-pool TIMER IRQ before `psram_init`. `DIRECT_CSR.EN` disconnects XIP."
        ),
        "rule": (
            "Settled 2026-08-23: Grok. `psram_configure_qmi` takes the same IRQ fence as "
            "`psram_detect` for the EN window. Frozen packs stay frozen."
        ),
    },
]

BUCKETS: list[tuple[str, str, str]] = [
    ("disagree-stop", "Grok vs Claude complete disagree", "Stop. Owner settles before any code."),
    ("dashboard-cli", "Dashboard / station CLI display lies", "Owner-silent two-agree. Print/path bugs, not RC_OS menu structure."),
    ("psram-qmi", "PSRAM init / QMI window", "Hard-gate, cached alias, QMI 1-vs-1. Do not silent-touch QMI."),
    ("log-ring", "Log ring init / recover / named sizes", "ring_init vs recover, failed-init flag, kRcLogRingBytes."),
    ("gps-uart", "GPS UART / PMTK / wrong-core NVIC", "Live on the vehicle (Claude Reconciliation §1)."),
    ("cross-core", "Cross-core publication / fail-open", "Plain bools, missing barriers, pause fail-open, dual-home atomics."),
    ("phantom-symbol", "Phantom / missing symbols", "Named in comments or callers; not declared, or declared unused."),
    ("class-design", "Class-design / published guts", "Claude lens re-run: AO/driver headers publish the innards. Unique-heavy; still one sitting."),
    ("nolint", "In-source NOLINT / suppressions", "Same policy as owner bucket. Skip if already closed on that leaf."),
    ("doxygen-density", "Doxygen + header comment-density", "Sitting 5 dropped authored Doxygen. Remaining = generated or new."),
    ("archaeology", "Process archaeology in comments", "Sitting 13 trimmed essays. Essay-only leftovers are cleared-policy, not a second archaeology sitting."),
    ("comment-contract", "Comment still disagrees with the body", "Re-read after sittings 5/13. Fix the live lie or delete the sentence. Not essay density."),
    ("hw-agnostic", "HW leakage vs domain code", "Sitting 4 closed most. Remaining = new agent P or DEFER riders."),
    ("starcom", "Starcom / radio-telem supersession", "Park with owner Starcom DEFER. No half-polish."),
    ("codegen", "Generated files / codegen hygiene", "No silent regen of mission_profile_data.h (R-4)."),
    ("spdx", "SPDX / third-party license", "Owner sitting closed authored SPDX. Remaining = new hits only."),
    ("rf-legal", "RF / regulatory config hazards", "WN-100 DEFER home."),
    ("safety-ssot", "Safety / ops SSOT (Go/No-Go, pyro, guards)", "Live invariants, not comment style."),
    ("early-impl", "Early-impl / design re-eval", "Park with owner early-impl DEFER. No seqlock/I2C/ICM/PCM rewrite."),
    ("rc-os", "RC_OS / CLI structure", "Park with owner RC_OS DEFER. Dashboard *lies* are not this bucket."),
    ("test-in-flight", "Test / inject / debug in the flight tree", "Owner sitting closed. Remaining = new agent P."),
    ("packaging", "File earn-rent / naming / packaging", "Owner sitting closed KEEP/fold. Remaining = new agent P."),
    ("version-config", "Version / identity / config.h grab-bag", "config.h gone. R-9 version bump still owner-scheduled."),
    ("fusion-invariants", "Fusion / math / cal live invariants", "Numerical contracts, not comment style."),
    ("fn-ptr", "P10-9 function pointers", "Owner GPS/watchdog remediates closed; QP-tied still DEFER."),
    ("one-off", "One-off / leftover", "Does not share a later sitting with a pile above."),
]


def has(blob: str, *pats: str) -> bool:
    return any(re.search(p, blob, re.I) for p in pats)


def parse_log() -> dict[str, dict]:
    text = LOG.read_text(encoding="utf-8")
    bucket = "(none)"
    out: dict[str, dict] = {}
    for line in text.splitlines():
        sm = SEC_H.match(line)
        if sm:
            bucket = sm.group(1).strip()
            continue
        m = WN_ROW.match(line)
        if not m:
            continue
        wid, label, state, close = m.group(1), m.group(2), m.group(3), m.group(4)
        out[wid] = {
            "id": wid,
            "label": label,
            "state": state,
            "close": close.strip(),
            "bucket": bucket,
            "closed_remediate": label == "REMEDIATE" and state == "closed",
            "defer": label == "DEFER",
        }
    return out


def parse_cluster_index() -> dict[str, str]:
    text = CLUSTERS.read_text(encoding="utf-8")
    out: dict[str, str] = {}
    for m in CLUSTER_IDS.finditer(text):
        name = m.group(1).rstrip()
        for wid in re.findall(r"WN-\d+", m.group(2)):
            out[wid] = name
    return out


def blob_of(n: dict, body_n: int = 800) -> str:
    return " ".join(
        [
            n.get("title") or "",
            n.get("claim") or "",
            n.get("issue") or "",
            (n.get("body") or "")[:body_n],
        ]
    )


def comment_shaped(n: dict) -> bool:
    kind = (n.get("kind") or "").lower()
    title = n.get("title") or ""
    if kind in {"comment", "doxygen", "spine"}:
        return True
    if "doxygen" in kind or "comment" in kind:
        return True
    if COMMENT_TITLE.search(title):
        return True
    return False


def live_shaped(n: dict) -> bool:
    kind = (n.get("kind") or "").lower()
    blob = blob_of(n)
    if kind in {
        "concurrency",
        "invariant",
        "ownership",
        "control-flow",
        "class-design",
        "contract",
        "assertions",
        "contract-surface",
    }:
        return True
    if LIVE_CONTRACT.search(blob):
        return True
    return False


def essay_only(n: dict) -> bool:
    blob = blob_of(n, 400)
    if LIVE_CONTRACT.search(blob_of(n)):
        return False
    if ESSAY_NOISE.search(blob) or COMMENT_TITLE.search(n.get("title") or ""):
        return True
    return False


def assign_bucket(n: dict, defer_home: str | None) -> str:
    path = (n.get("leaf") or "") + " " + (n.get("path") or "")
    title = n.get("title") or ""
    claim = n.get("claim") or ""
    body = (n.get("body") or "")[:1800]
    blob = " ".join([path, title, claim, body])
    kind = n.get("kind") or ""

    # Display lies on CLI leaves are not RC_OS structure, even if WN-325 DEFER
    # lives on the same file.
    if has(
        blob,
        r"rc_os_dashboard|Temp: is literal|same AGL|FAIL count vs FAIL|"
        r"distance.*stale|station uptime|dashboard Alt",
    ) or (
        "rc_os_dashboard" in path
        and has(blob, r"Temp|Alt|Baro|FAIL|stale|literal 0")
    ):
        return "dashboard-cli"

    if defer_home:
        dh = defer_home.lower()
        if "starcom" in dh:
            return "starcom"
        if "early-impl" in dh or "design re-eval" in dh:
            return "early-impl"
        if "rc_os" in dh or "cli structure" in dh:
            return "rc-os"
        if "codegen" in dh or "generated" in dh:
            return "codegen"
        if "rf /" in dh or "regulatory" in dh:
            return "rf-legal"

    if kind.lower() == "class-design":
        return "class-design"
    if "pio_backup_timer" in path:
        return "early-impl"
    if "station_idle_tick" in path:
        return "cross-core"
    if "rfm95w" in path or "pcm_frame" in path:
        return "starcom"
    if "icm20948" in path and not has(blob, r"lazy mag|re-init"):
        return "early-impl"
    if has(blob, r"qmi_hw|psram_configure_qmi|psram_detect|psram_self_test|hard gate"):
        return "psram-qmi"
    if has(blob, r"ring_init|ring_recover|kRcLogRingBytes|drop-oldest|drop-newest|drop-on-full"):
        return "log-ring"
    if has(
        blob,
        r"rc_os_dashboard|Temp: is literal|same AGL|FAIL count vs FAIL|"
        r"distance.*stale|station uptime|dashboard Alt",
    ):
        return "dashboard-cli"
    if "rc_os_dashboard" in path:
        if has(blob, r"Temp|Alt|Baro|FAIL|stale|literal 0"):
            return "dashboard-cli"
    if has(blob, r"gps_uart_reinit|PMTK314|UART RX ring|wrong-core NVIC|g_gpsTransport"):
        return "gps-uart"
    if "sensor_seqlock" in path:
        if has(blob, r"two homes|declared in two|six atomics"):
            return "cross-core"
        return "early-impl"
    if has(
        blob,
        r"plain bool|no atomic|no barrier|fail-open|do not nest|"
        r"two homes|declared in two|g_baroInitialized|g_gpsInitialized|"
        r"g_sensorPhaseActive|calibration session|who-may-write|who-actually-writes",
    ):
        return "cross-core"
    if has(blob, r"does not exist|named, not declared|no such API|phantom"):
        return "phantom-symbol"
    if has(blob, r"\bNOLINT"):
        return "nolint"
    if has(blob, r"P10-9|function.?pointer|\bfn-ptr\b|kick_watchdog|g_gpsFn"):
        return "fn-ptr"
    if has(blob, r"AUTO-GENERATED|Do not edit") or has(title, r"generated|codegen|regen"):
        return "codegen"
    if has(title, r"SPDX|third-party|license"):
        return "spdx"
    if has(title, r"regulatory|FCC|Part 15") or has(blob, r"\bWN-100\b"):
        return "rf-legal"
    if has(
        blob,
        r"Go/No-Go|go_nogo|FIRE_PYRO|anomalous-boot 2-of-N|2-of-N cannot|"
        r"phase bitmask|phase_bit cannot|HealthCritical|mpu_setup_stack_guard",
    ):
        return "safety-ssot"
    if "safety/" in path:
        return "safety-ssot"
    if "log/rc_log" in path or "logging/ring_buffer" in path:
        return "log-ring"
    if has(blob, r"DeviceRole|never defined|signal-name lookup"):
        return "phantom-symbol"
    if has(
        blob,
        r"NeoPixel driver is written from both cores|two writer families on one LED|"
        r"I2C sensor reader is bound into the CLI|"
        r"g_crash_record is non-volatile|Core 1 mutates the Core-0-owned calibration",
    ):
        return "cross-core"
    if has(blob, r"\bStarcom\b") or has(
        path, r"radio_config|radio_scheduler|telemetry_encoder|mavlink_rx|ao_radio|ao_rf_manager|ao_telemetry|rfm95w|pcm_frame"
    ):
        if has(blob, r"Starcom|supersession|CCSDS|PCM layout|MAVLink"):
            return "starcom"
        if any(
            x in path
            for x in (
                "radio_scheduler",
                "ao_radio",
                "ao_rf_manager",
                "ao_telemetry",
                "telemetry_encoder",
                "mavlink_rx",
            )
        ):
            return "starcom"
    if has(blob, r"HW-agnostic|PICO_BOARD|SKU|pin map|Feather pin|board pack"):
        return "hw-agnostic"
    if has(blob, r"fault.inject|test_mode|debug sub-menu"):
        return "test-in-flight"
    if "cli/" in path or "ao_rcos" in path:
        if has(blob, r"Temp:|dashboard|FAIL count|literal 0"):
            return "dashboard-cli"
        return "rc-os"
    if has(
        title,
        r"seqlock still|SensorSnapshot|early-impl|prior.art|bespoke driver|PCM|I2C bus",
    ):
        return "early-impl"
    if has(blob, r"version_string|kFirmwareVersion|TIER_|FEATURE_|config\.h"):
        return "version-config"
    if has(title, r"earn rent|sparse|vague|filename|packaging|folder"):
        return "packaging"
    if kind.lower() in {"invariant"} and has(
        title, r"default|rate|Hz|mission|wind|ZUPT|sigma|noise|WMM|sentinel"
    ):
        return "fusion-invariants"
    if "fusion/" in path or "calibration/" in path or "math/" in path:
        if live_shaped(n) and not comment_shaped(n):
            return "fusion-invariants"
    if has(blob, r"code 28|LedPhaseValue|kOff is zero|kNone is not"):
        return "packaging"
    if has(title, r"Doxygen|comment-density|comment density|comment ratio"):
        return "doxygen-density"
    if comment_shaped(n) and LIVE_CONTRACT.search(blob):
        return "comment-contract"
    if comment_shaped(n) and has(blob, r"IVP|Stage |council|tombstone|essay|banner"):
        return "archaeology"
    if comment_shaped(n):
        if essay_only(n):
            return "archaeology"
        return "comment-contract"
    if kind.lower() == "contract" and LIVE_CONTRACT.search(blob):
        return "comment-contract"
    return "one-off"


def classify_vs_owner(
    n: dict,
    leaf_wns: list[dict],
    log: dict[str, dict],
    best: tuple[dict | None, int, list[str]],
) -> tuple[str, str]:
    """Return (axis, reason). axis in cleared / affected / defer-home / untouched."""
    best_w, score, toks = best
    closed_on_leaf = [
        w
        for w in leaf_wns
        if log.get(w["id"], {}).get("closed_remediate")
    ]
    defer_on_leaf = [w for w in leaf_wns if log.get(w["id"], {}).get("defer")]

    if best_w:
        rec = log.get(best_w["id"])
        if rec and rec["closed_remediate"] and score >= 6:
            return "cleared", f"same-P {best_w['id']} ({score}) toks={toks[:6]}"
        if rec and rec["defer"] and score >= 4:
            return "defer-home", f"{best_w['id']} {rec['bucket']} ({score})"
        if rec and rec["closed_remediate"] and score >= 4:
            if comment_shaped(n) and rec["bucket"] in POLICY_SITTINGS:
                return "cleared", f"policy {best_w['id']} {rec['bucket']} ({score})"
            return "affected", f"{best_w['id']} closed but P may differ ({score}) toks={toks[:6]}"

    if essay_only(n) and closed_on_leaf:
        pol = [
            w
            for w in closed_on_leaf
            if log[w["id"]]["bucket"] in POLICY_SITTINGS
        ]
        ids = ",".join(w["id"] for w in (pol or closed_on_leaf)[:3])
        return "cleared", f"essay-only leftover after policy sitting {ids}"

    if comment_shaped(n) and not live_shaped(n) and closed_on_leaf:
        pol = [
            w
            for w in closed_on_leaf
            if log[w["id"]]["bucket"] in POLICY_SITTINGS
        ]
        if pol and COMMENT_TITLE.search(n.get("title") or ""):
            ids = ",".join(w["id"] for w in pol[:3])
            return "cleared", f"policy-leaf comment sitting {ids}"

    if closed_on_leaf:
        ids = ",".join(w["id"] for w in closed_on_leaf[:4])
        return "affected", f"leaf remediates {ids}; agent P not the same WN"
    if defer_on_leaf and not closed_on_leaf:
        w = defer_on_leaf[0]
        rec = log[w["id"]]
        return "defer-home", f"leaf parked {w['id']} {rec['bucket']}"
    return "untouched", "no closed REMEDIATE and no DEFER WN on this leaf"


def best_wn(n: dict, leaf_wns: list[dict]) -> tuple[dict | None, int, list[str]]:
    gs = set(n.get("idents") or [])
    best_w = None
    best_ws = 0
    best_wt: list[str] = []
    for w in leaf_wns:
        ws, wt = join.overlap_score(gs, set(w.get("idents") or []))
        if ws > best_ws:
            best_ws, best_w, best_wt = ws, w, wt
    return best_w, best_ws, best_wt


def pair_key(a: str, b: str) -> tuple[str, str]:
    return (a, b) if a < b else (b, a)


def md_escape(s: str) -> str:
    return (s or "").replace("|", "/").replace("\n", " ").strip()


def short(s: str, n: int = 90) -> str:
    s = md_escape(s)
    return s if len(s) <= n else s[: n - 1] + "…"


def main() -> None:
    leaves = join.load_leaves()
    idx = join.build_leaf_index(leaves)
    owner_text = join.OWNER.read_text(encoding="utf-8")
    grok_text = join.GROK.read_text(encoding="utf-8")
    claude_text = join.CLAUDE.read_text(encoding="utf-8")
    wn = join.parse_owner(owner_text, leaves, idx)
    gwf = join.parse_grok(grok_text, leaves, idx)
    cw_all = join.parse_claude(claude_text, leaves, idx)
    cw_live = [
        n
        for n in cw_all
        if not n["duplicate"] and n["verdict"] not in ("REFUTED",)
    ]
    cw_refuted = [
        n
        for n in cw_all
        if (not n["duplicate"]) and n["verdict"] == "REFUTED"
    ]
    log = parse_log()
    cluster_of = parse_cluster_index()

    by_leaf: dict[str, dict[str, list]] = defaultdict(
        lambda: {"WN": [], "GWF": [], "CW": [], "CW_REF": []}
    )
    for n in wn:
        by_leaf[n["leaf"]]["WN"].append(n)
    for n in gwf:
        by_leaf[n["leaf"]]["GWF"].append(n)
    for n in cw_live:
        by_leaf[n["leaf"]]["CW"].append(n)
    for n in cw_refuted:
        by_leaf[n["leaf"]]["CW_REF"].append(n)

    # Grok+Claude live pairs
    pairs = []
    gwf_paired: set[str] = set()
    cw_paired: set[str] = set()
    for leaf, b in by_leaf.items():
        for g in b["GWF"]:
            gs = set(g["idents"])
            for c in b["CW"]:
                sc, toks = join.overlap_score(gs, set(c["idents"]))
                if sc < 4:
                    continue
                pairs.append(
                    {
                        "leaf": leaf,
                        "gwf": g["id"],
                        "cw": c["id"],
                        "score": sc,
                        "toks": toks,
                        "g_title": g["title"],
                        "c_title": c["title"],
                    }
                )
                gwf_paired.add(g["id"])
                cw_paired.add(c["id"])

    # Disagree: live Grok vs REFUTED Claude on same leaf
    auto_disagree = []
    for leaf, b in by_leaf.items():
        for g in b["GWF"]:
            gs = set(g["idents"])
            for c in b["CW_REF"]:
                sc, toks = join.overlap_score(gs, set(c["idents"]))
                if sc < 6:
                    continue
                # LED 0/kOff and "ownership map" are same-facts / different-P, not is/isn't.
                if c["id"] in {"CW-B29-06", "CW-B01-02"}:
                    continue
                auto_disagree.append(
                    {
                        "leaf": leaf,
                        "gwf": g["id"],
                        "cw": c["id"],
                        "score": sc,
                        "toks": toks,
                        "g_title": g["title"],
                        "c_title": c["title"],
                    }
                )

    curated_ids = set()
    for row in CURATED_TRIPLES + CURATED_NEIGHBORHOOD:
        curated_ids.update(row.get("gwf") or [])
        curated_ids.update(row.get("cw") or [])
    for _iss, g, c, _b in CURATED_AGREE_OWNER_SILENT:
        curated_ids.add(g)
        curated_ids.add(c)
    curated_ids.add("GWF-311")
    curated_ids.add("CW-B26-05")

    items: list[dict] = []

    def add_item(n: dict, pack: str) -> dict:
        leaf = n["leaf"]
        leaf_wns = by_leaf[leaf]["WN"]
        best = best_wn(n, leaf_wns)
        axis, reason = classify_vs_owner(n, leaf_wns, log, best)
        vs_other = "unique"
        if pack == "GWF" and n["id"] in gwf_paired:
            vs_other = "agree"
        if pack == "CW" and n["id"] in cw_paired:
            vs_other = "agree"
        if n["id"] in {"GWF-311", "CW-B26-05"}:
            vs_other = "disagree"
        defer_home = None
        if axis == "defer-home":
            m = re.search(r"(WN-\d+)", reason)
            if m and m.group(1) in log:
                defer_home = log[m.group(1)]["bucket"]
        rec = {
            "pack": pack,
            "id": n["id"],
            "leaf": leaf,
            "title": n.get("title") or "",
            "kind": n.get("kind") or "",
            "sev": n.get("severity") or n.get("verdict") or "",
            "axis": axis,
            "vs_other": vs_other,
            "reason": reason,
            "best_wn": best[0]["id"] if best[0] else None,
            "best_score": best[1],
            "bucket": assign_bucket(n, defer_home),
        }
        items.append(rec)
        return rec

    for n in gwf:
        add_item(n, "GWF")
    for n in cw_live:
        add_item(n, "CW")

    # Force curated bucket/axis where we know better than ident overlap.
    by_id = {r["id"]: r for r in items}
    for row in CURATED_TRIPLES + CURATED_NEIGHBORHOOD:
        expect = row["expect"]
        axis = {
            "cleared": "cleared",
            "affected": "affected",
            "defer-home": "defer-home",
        }[expect]
        for nid in (row.get("gwf") or []) + (row.get("cw") or []):
            if nid in by_id:
                by_id[nid]["axis"] = axis
                by_id[nid]["reason"] = f"curated vs {row['wn']}: {row['issue']}"
                by_id[nid]["best_wn"] = row["wn"]
                if axis == "defer-home":
                    rec = log.get(row["wn"], {})
                    by_id[nid]["bucket"] = assign_bucket(
                        {"leaf": by_id[nid]["leaf"], "title": by_id[nid]["title"],
                         "claim": "", "body": "", "kind": by_id[nid]["kind"],
                         "path": by_id[nid]["leaf"]},
                        rec.get("bucket"),
                    )
                by_id[nid]["vs_other"] = "agree" if (row.get("gwf") and row.get("cw")) else by_id[nid]["vs_other"]
    for _iss, g, c, buck in CURATED_AGREE_OWNER_SILENT:
        for nid in (g, c):
            if nid not in by_id:
                continue
            if by_id[nid]["axis"] == "cleared":
                by_id[nid]["axis"] = "untouched"
                by_id[nid]["reason"] = "curated owner-silent two-agree; not the closed WN P"
            by_id[nid]["vs_other"] = "agree"
            by_id[nid]["bucket"] = buck
    if "GWF-311" in by_id:
        by_id["GWF-311"]["vs_other"] = "disagree"
        by_id["GWF-311"]["bucket"] = "disagree-stop"
        by_id["GWF-311"]["axis"] = "untouched"
        by_id["GWF-311"]["reason"] = "curated 1-vs-1 QMI; owner silent"
    # CW-B26-05 is REFUTED so not in cw_live; record it only in the disagree section.

    for rec in items:
        if rec["id"] == "GWF-311":
            continue
        if rec["bucket"] in DEFER_BUCKETS and rec["axis"] != "cleared":
            rec["axis"] = "defer-home"
            rec["reason"] = f"park with owner {rec['bucket']} home"
        elif rec["axis"] == "defer-home" and rec["bucket"] not in DEFER_BUCKETS:
            rec["axis"] = "affected"
            rec["reason"] = (
                rec["reason"] + " (leaf DEFER is a different P; keep in this bucket)"
            )

    def count(pred) -> int:
        return sum(1 for r in items if pred(r))

    n_gwf, n_cw = len(gwf), len(cw_live)
    lines: list[str] = []

    def emit(s: str = "") -> None:
        lines.append(s)

    emit("# L2-P5 Grok + Claude combined overlay")
    emit("")
    emit("**Not a merge of IDs. Not a record. Finding packs stay frozen.**")
    emit("Owner walk (`WN-001–327`) stays a **separate** chunk — that work is")
    emit("already labeled and the do-now remediates are on `main`. This overlay")
    emit("is chunks 2+3 together, because the two agent packs named many of the")
    emit("same propositions.")
    emit("")
    emit("| Pack | File | Role |")
    emit("|------|------|------|")
    emit("| Owner | `L2P5_WALK_FINDINGS.md` | Already dispositioned. Used here only as the previous sitting. |")
    emit("| Grok | `L2P5_GROK_WALK_FINDINGS.md` | `GWF-001–498`. One vote. |")
    emit("| Claude | `L2P5_CLAUDE_WALK_FINDINGS_ALIGNED.md` | One vote. Skip lane duplicates and REFUTED (except the disagree bin). |")
    emit("")
    emit("Join key: **itinerary leaf + the proposition** (same helper as")
    emit("`L2P5_THREE_WALK_COMPARE.md`). Silence is not “this is fine.”")
    emit("")
    emit("Generator: `_gen_grok_claude_overlay.py`. Re-run after rule tweaks.")
    emit("Review **buckets**, not each row. Default for remaining live rows is")
    emit("still REMEDIATE; do not auto-ACCEPT.")
    emit("")
    emit("---")
    emit("")
    emit("## How to read the four bins")
    emit("")
    emit("Every live Grok row and every live Claude row is tagged on two axes,")
    emit("then dropped into a bucket (the same idea as the owner 16).")
    emit("")
    emit("| Axis | Bin | Meaning |")
    emit("|------|-----|---------|")
    emit("| vs owner disposition | **cleared** | Same P as a closed REMEDIATE WN, or a comment/NOLINT/Doxygen note on a leaf that sitting already rewrote. Skip. |")
    emit("| vs owner disposition | **affected** | That leaf was remediates, but the agent P is a different claim (or the comment sitting may have only half-fixed it). Re-read before skip or fix. |")
    emit("| vs owner disposition | **defer-home** | Same P (or same leaf, no closed remediates) as an owner DEFER. Park with that home. Do not reopen Starcom / RC_OS structure / early-impl. |")
    emit("| vs owner disposition | **untouched** | Owner did not remediate this leaf, or did not name this P. Extra walks working. |")
    emit("| vs the other agent | **agree** | Grok and Claude independently named the same underlying issue. |")
    emit("| vs the other agent | **disagree** | True is/isn’t split. Stop. |")
    emit("| vs the other agent | **unique** | Only one pack asserted P. Candidate, not a fight. |")
    emit("")
    emit("A row can be **agree and cleared** (both found it; owner already fixed")
    emit("it). The work queue is: not-cleared rows, by bucket, skip `defer-home`")
    emit("the same way owner skipped those sittings.")
    emit("")
    emit("---")
    emit("")
    emit("## Counts")
    emit("")
    emit("| | n |")
    emit("|--|--:|")
    emit(f"| Grok kept | {n_gwf} |")
    emit(f"| Claude live (no lane-dup, no REFUTED) | {n_cw} |")
    emit(f"| Combined rows in this overlay | {len(items)} |")
    emit(f"| Grok+Claude ident pairs (score≥4) | {len(pairs)} |")
    emit(f"| Cleared | {count(lambda r: r['axis']=='cleared')} |")
    emit(f"| Affected | {count(lambda r: r['axis']=='affected')} |")
    emit(f"| Defer-home | {count(lambda r: r['axis']=='defer-home')} |")
    emit(f"| Untouched | {count(lambda r: r['axis']=='untouched')} |")
    emit(f"| Both agree | {count(lambda r: r['vs_other']=='agree')} |")
    emit(f"| Disagree | {count(lambda r: r['vs_other']=='disagree')} |")
    emit(f"| Unique | {count(lambda r: r['vs_other']=='unique')} |")
    emit(f"| Remaining work (not cleared, not defer-home) | {count(lambda r: r['axis'] in ('affected','untouched'))} |")
    emit("")
    emit("Owner log parsed: "
         f"{sum(1 for v in log.values() if v['closed_remediate'])} closed REMEDIATE, "
         f"{sum(1 for v in log.values() if v['defer'])} DEFER.")
    emit("")
    emit("---")
    emit("")
    emit("## Completely disagree")
    emit("")
    emit("Across Grok and Claude there was **one** curated is/isn’t split.")
    emit("**Settled 2026-08-23:** Grok. Fence configure the same way as detect.")
    emit("")
    for d in CURATED_DISAGREE:
        emit(f"**P:** (the claim) {d['p']}")
        emit("")
        emit("| Pack | ID | Answer |")
        emit("|------|----|--------|")
        emit(f"| Grok | `{d['gwf']}` | {d['g_answer']} |")
        emit(f"| Claude | `{d['cw']}` | {d['c_answer']} |")
        emit("| Owner | — | Silent. Nearby WNs on this leaf (WN-215–217) are density / board-coupling / test-permanence. |")
        emit("")
        emit(f"**Rule:** {d['rule']}")
        emit("")
    emit("`0` / `kOff` in the LED resolver is the same facts, different “is this")
    emit("a finding?” call — not this bin.")
    emit("")
    if auto_disagree:
        emit("Auto ident-overlap of a live Grok row vs a **REFUTED** Claude row")
        emit("(score≥6). Not automatically a second fight — check before promoting:")
        emit("")
        emit("| Leaf | Grok | Claude | score | toks |")
        emit("|------|------|--------|------:|------|")
        for d in sorted(auto_disagree, key=lambda x: -x["score"])[:25]:
            emit(
                f"| `{short(d['leaf'], 40)}` | `{d['gwf']}` {short(d['g_title'], 40)} "
                f"| `{d['cw']}` {short(d['c_title'], 40)} | {d['score']} | "
                f"{', '.join(d['toks'][:6])} |"
            )
        emit("")
    else:
        emit("No additional live-Grok vs REFUTED-Claude ident pairs at score≥6.")
        emit("")

    emit("---")
    emit("")
    emit("## Fully cleared by the owner disposition")
    emit("")
    emit("Skip these in the agent chunk. Frozen packs still contain the original")
    emit("text — “cleared” means the **work** is done, not that the walk file")
    emit("was edited.")
    emit("")
    emit("### Curated triples (from the three-walk compare)")
    emit("")
    emit("| Issue | Owner | Owner close | Grok | Claude | Bin |")
    emit("|-------|-------|-------------|------|--------|-----|")
    for row in CURATED_TRIPLES:
        rec = log.get(row["wn"], {})
        close = short(rec.get("close", rec.get("label", "")), 50)
        emit(
            f"| {row['issue']} | `{row['wn']}` {rec.get('label','')} "
            f"{rec.get('state','')} | {close} | "
            f"{', '.join('`'+x+'`' for x in row['gwf']) or '—'} | "
            f"{', '.join('`'+x+'`' for x in row['cw']) or '—'} | "
            f"**{row['expect']}** |"
        )
        if row.get("note"):
            emit(f"| | | {row['note']} | | | |")
    emit("")
    emit("### Auto-cleared (same-P or comment/NOLINT/Doxygen policy sitting)")
    emit("")
    emit("Conservative: ident overlap with a **closed REMEDIATE** WN, or a")
    emit("comment-shaped title on a leaf that sitting 1/5/13 already rewrote.")
    emit("If a later re-read shows the live contract is still wrong, move the")
    emit("row to **affected** — do not treat this list as certification.")
    emit("")

    cleared = [r for r in items if r["axis"] == "cleared"]
    by_buck: dict[str, list] = defaultdict(list)
    for r in cleared:
        by_buck[r["bucket"]].append(r)
    emit(f"Auto + curated cleared: **{len(cleared)}** rows.")
    emit("")
    for cid, name, _later in BUCKETS:
        members = sorted(by_buck.get(cid, []), key=lambda x: (x["pack"], x["id"]))
        if not members:
            continue
        ids = ", ".join(f"`{m['id']}`" for m in members)
        emit(f"- **{name}** ({len(members)}): {ids}")
        emit("")

    emit("---")
    emit("")
    emit("## Affected by the owner disposition")
    emit("")
    emit("The file (or a nearby WN) was remediates, but the agent claim is not")
    emit("the same P — or the comment sitting may have deleted the essay and")
    emit("left a live lie. **Re-read the current tree** before skip or fix.")
    emit("")
    emit("### Curated same-neighborhood (owner asked home; agents named a lie)")
    emit("")
    emit("| Issue | Owner | Close | Grok | Claude |")
    emit("|-------|-------|-------|------|--------|")
    for row in CURATED_NEIGHBORHOOD:
        rec = log.get(row["wn"], {})
        emit(
            f"| {row['issue']} | `{row['wn']}` {rec.get('label','')} "
            f"{rec.get('state','')} | {short(rec.get('close',''), 45)} | "
            f"{', '.join('`'+x+'`' for x in row['gwf'])} | "
            f"{', '.join('`'+x+'`' for x in row['cw'])} |"
        )
        emit(f"| | | {row['note']} | | |")
    emit("")
    emit("### Auto-affected")
    emit("")
    affected = [r for r in items if r["axis"] == "affected"]
    emit(f"**{len(affected)}** rows. Compact IDs here; titles live in Combined remaining buckets.")
    emit("")
    by_buck = defaultdict(list)
    for r in affected:
        by_buck[r["bucket"]].append(r)
    for cid, name, _later in BUCKETS:
        members = sorted(by_buck.get(cid, []), key=lambda x: (x["pack"], x["id"]))
        if not members:
            continue
        ids = ", ".join(f"`{m['id']}`" for m in members)
        emit(f"- **{name}** ({len(members)}): {ids}")
        emit("")

    emit("---")
    emit("")
    emit("## Covered by an owner DEFER (park, do not reopen)")
    emit("")
    emit("Same homes as the owner chunk: Starcom, RC_OS structure, early-impl")
    emit("rewrites, codegen audit, WN-100. Agent polish on those surfaces waits")
    emit("with the owner DEFER. Dashboard **display lies** are *not* RC_OS")
    emit("structure — they stay in the dashboard bucket.")
    emit("")
    defer_rows = [r for r in items if r["axis"] == "defer-home"]
    emit(f"**{len(defer_rows)}** rows.")
    emit("")
    by_buck = defaultdict(list)
    for r in defer_rows:
        by_buck[r["bucket"]].append(r)
    for cid, name, _later in BUCKETS:
        members = sorted(by_buck.get(cid, []), key=lambda x: (x["pack"], x["id"]))
        if not members:
            continue
        ids = ", ".join(f"`{m['id']}`" for m in members)
        emit(f"- **{name}** ({len(members)}): {ids}")
        emit("")

    emit("---")
    emit("")
    emit("## Both agree (and still live)")
    emit("")
    emit("Grok and Claude named the same underlying issue. Owner was silent, or")
    emit("the overlapping WN did not close this P. This is why the extra walks")
    emit("were run. Medium-high trust of the *claim* — still not a rank.")
    emit("")
    emit("### From the three-walk compare (owner silent)")
    emit("")
    emit("| Claim | Grok | Claude | Bucket |")
    emit("|-------|------|--------|--------|")
    for iss, g, c, buck in CURATED_AGREE_OWNER_SILENT:
        bname = next((nm for cid, nm, _ in BUCKETS if cid == buck), buck)
        g_ax = by_id.get(g, {}).get("axis", "?")
        c_ax = by_id.get(c, {}).get("axis", "?")
        emit(
            f"| {iss} | `{g}` ({g_ax}) | `{c}` ({c_ax}) | {bname} |"
        )
    emit("")
    emit("GWF-135 is one Grok row covering two Claude UART claims (volatile ring")
    emit("+ wrong-core NVIC). Do not mint a second GWF.")
    emit("")
    emit("### Other ident pairs still not cleared / not defer-home")
    emit("")
    live_pairs = []
    for p in pairs:
        g_rec = by_id.get(p["gwf"])
        c_rec = by_id.get(p["cw"])
        if not g_rec or not c_rec:
            continue
        if g_rec["axis"] == "cleared" and c_rec["axis"] == "cleared":
            continue
        if g_rec["axis"] == "defer-home" and c_rec["axis"] == "defer-home":
            continue
        live_pairs.append((p, g_rec, c_rec))
    emit(f"**{len(live_pairs)}** pairs (a row may appear in more than one pair).")
    emit("")
    emit("| Leaf | Grok | ax | Claude | ax | score |")
    emit("|------|------|----|--------|----|------:|")
    for p, g_rec, c_rec in sorted(live_pairs, key=lambda t: -t[0]["score"]):
        emit(
            f"| `{short(p['leaf'], 36)}` | `{p['gwf']}` {short(p['g_title'], 32)} "
            f"| {g_rec['axis']} | `{p['cw']}` {short(p['c_title'], 32)} "
            f"| {c_rec['axis']} | {p['score']} |"
        )
    emit("")

    emit("---")
    emit("")
    emit("## Unique rows that still group")
    emit("")
    emit("One pack asserted P; the other sitting did not. Unique is why extra")
    emit("walks were run. They still sit in the same buckets when the theme")
    emit("matches — do not run them as 400 isolated nits.")
    emit("")
    unique_live = [
        r
        for r in items
        if r["vs_other"] == "unique" and r["axis"] in ("affected", "untouched")
    ]
    emit(f"**{len(unique_live)}** unique remaining (not cleared, not defer-home).")
    emit("Titles live in Combined remaining buckets — do not treat unique as a")
    emit("separate queue.")
    emit("")
    by_buck = defaultdict(list)
    for r in unique_live:
        by_buck[r["bucket"]].append(r)
    emit("| n | Bucket |")
    emit("|--:|--------|")
    for cid, name, _later in BUCKETS:
        members = by_buck.get(cid, [])
        if not members:
            continue
        emit(f"| {len(members)} | {name} |")
    emit("")

    emit("---")
    emit("")
    emit("## Combined remaining buckets")
    emit("")
    emit("Same sitting style as the owner 16: one theme per sitting, related")
    emit("notes in the same bucket even if they are not one bug. Do not open a")
    emit("fourth taxonomy. New buckets below are **agent-only themes the owner")
    emit("walk did not file** — that is the extra-walks harvest — not a new")
    emit("process.")
    emit("")
    emit("Inside a bucket: skip `cleared`, park `defer-home`, re-read `affected`,")
    emit("do-now = `untouched` + surviving `affected`. Test in groups of 2–4")
    emit("(R-10). **QMI disagree settled** (fence configure with detect).")
    emit("")
    emit("**Per remaining item:** one-line summary of the claim + a suggested")
    emit("label (REMEDIATE / re-read skip / DEFER-home) before `src/` in that")
    emit("group. Do not auto-ACCEPT. See plan Phase 4.")
    emit("")
    emit("| n remaining | Bucket | Later sitting |")
    emit("|--:|--------|---------------|")
    remaining_by: dict[str, list] = defaultdict(list)
    for r in items:
        if r["axis"] in ("affected", "untouched"):
            remaining_by[r["bucket"]].append(r)
    for cid, name, later in BUCKETS:
        members = remaining_by.get(cid, [])
        if not members and cid != "disagree-stop":
            continue
        n = len(members)
        if cid == "disagree-stop":
            n = max(n, 1)
        emit(f"| {n} | {name} | {later} |")
    emit("")
    emit("### ID lists (cite this in the plan)")
    emit("")
    for cid, name, later in BUCKETS:
        members = sorted(
            remaining_by.get(cid, []), key=lambda x: (x["pack"], x["id"])
        )
        if cid == "disagree-stop":
            emit(f"**{name}:** `GWF-311` vs `CW-B26-05` (REFUTED, not in live Claude count)")
            emit("")
            emit(later)
            emit("")
            continue
        if not members:
            continue
        emit(f"**{name}** ({len(members)}) — {later}")
        emit("")
        emit("| ID | vs owner | vs other | Leaf | Title |")
        emit("|----|----------|----------|------|-------|")
        for m in members:
            emit(
                f"| `{m['id']}` | {m['axis']} | {m['vs_other']} | "
                f"`{short(m['leaf'], 32)}` | {short(m['title'], 48)} |"
            )
        emit("")

    emit("---")
    emit("")
    emit("## Out of this document")
    emit("")
    emit("- Actual REMEDIATE / ACCEPT / DEFER labels on GWF/CW rows (next sitting).")
    emit("- Hardware gates for any code that later lands.")
    emit("- Editing the frozen finding packs.")
    emit("- Mixing owner WNs back into this queue.")
    emit("")
    emit("When the owner opens the first agent-bucket sitting: work remaining")
    emit("rows in that bucket, in groups of 2–4, on the disposition worktree.")
    emit("Do not start those `src/` edits on `main`.")
    emit("")

    OUT_MD.write_text("\n".join(lines) + "\n", encoding="utf-8")
    payload = {
        "counts": {
            "gwf": n_gwf,
            "cw_live": n_cw,
            "items": len(items),
            "pairs": len(pairs),
            "cleared": count(lambda r: r["axis"] == "cleared"),
            "affected": count(lambda r: r["axis"] == "affected"),
            "defer_home": count(lambda r: r["axis"] == "defer-home"),
            "untouched": count(lambda r: r["axis"] == "untouched"),
            "agree": count(lambda r: r["vs_other"] == "agree"),
            "disagree": count(lambda r: r["vs_other"] == "disagree"),
            "unique": count(lambda r: r["vs_other"] == "unique"),
            "remaining": count(lambda r: r["axis"] in ("affected", "untouched")),
        },
        "items": items,
        "pairs": pairs,
        "auto_disagree": auto_disagree,
        "remaining_by_bucket": {
            cid: [m["id"] for m in remaining_by.get(cid, [])] for cid, *_ in BUCKETS
        },
    }
    OUT_JSON.write_text(json.dumps(payload, indent=1), encoding="utf-8")
    print("wrote", OUT_MD)
    print("wrote", OUT_JSON)
    print(json.dumps(payload["counts"], indent=2))


if __name__ == "__main__":
    main()
