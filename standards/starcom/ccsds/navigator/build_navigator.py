#!/usr/bin/env python3
"""CCSDS Prox-1 navigator generator (docs tooling only).

Reads the four CCSDS PDFs stored in standards/starcom/ccsds/ with PyMuPDF and writes
  data/standard.json        (text of the standards only: clauses, shall sentences, PICS, cross-refs)
  data/implementation.json  (explicit clause citations found in this repository's code)
  data/evidence.json        (hand maintained; created EMPTY if missing, never overwritten)
  index.html                (self contained page; JSON embedded)
  ../../../starcom/docs/SHALL_MATRIX.md

Deterministic: same PDFs + same repo files => same output (no timestamps).
Usage:  python build_navigator.py [--repo-root DIR] [--pdf-dir DIR] [--skip-impl] [--include-docs] [--no-html | --html-only]
"""
import argparse
import collections
import json
import os
import re
import sys

try:
    import pymupdf
except ImportError:  # older PyMuPDF
    import fitz as pymupdf

HERE = os.path.dirname(os.path.abspath(__file__))

BOOKS = [
    # id, pdf file name (order = stack order in the UI: top layer first)
    ("133.0-B-2", "CCSDS-133.0-B-2-EC2.pdf"),
    ("211.0-B-6", "CCSDS-211.0-B-6-EC1.pdf"),
    ("211.2-B-3", "CCSDS-211.2-B-3.pdf"),
    ("211.1-B-4", "CCSDS-211.1-B-4-EC1.pdf"),
]
DUKE_RAW = {"211.0-B-6": 312, "211.1-B-4": 40, "211.2-B-3": 94, "133.0-B-2": 123}

MONTHS = ("January|February|March|April|May|June|July|August|September|October|"
          "November|December")
RE_FOOT_DOC = re.compile(r"^CCSDS \d{3}\.\d-B-\d+\s*$")
RE_FOOT_PAGE = re.compile(r"^Page ([ivxlcdm]+|[A-Z]-\d+|\d+-\d+)\s*$")
RE_FOOT_DATE = re.compile(r"^(%s) \d{4}\s*$" % MONTHS)
RE_HEAD = re.compile(r"^CCSDS RECOMMENDED STANDARD FOR")
RE_NUM_BODY = re.compile(r"^\d+(?:\.\d+)*$")
RE_NUM_ANNEX = re.compile(r"^[A-Z]\d+(?:\.\d+)*$")
RE_ANNEX_HEAD = re.compile(r"^ANNEX ([A-Z])$")
MODALS = (("shall", re.compile(r"\bshall\b", re.I)),
          ("should", re.compile(r"\bshould\b", re.I)),
          ("may", re.compile(r"\bmay\b")))
ABBREV = ("e.g", "i.e", "etc", "vs", "approx", "No", "Fig", "Ref", "cf", "al")


# --------------------------------------------------------------------------- helpers
def norm(t):
    t = t.replace("\xa0", " ").replace("\u200b", "")
    return re.sub(r"\s+", " ", t).strip()


def join_text(parts):
    """Join line texts of one paragraph. A line ending in a letter+hyphen followed by a
    lower-case word, or ending in an em dash, is joined without a space (compound hyphen
    / dash kept as printed)."""
    out = ""
    for p in parts:
        p = norm(p)
        if not p:
            continue
        if not out:
            out = p
        elif (out.endswith("-") and len(out) > 1 and out[-2].isalnum() and p[0].islower()) \
                or out.endswith("\u2014") or p[0] in "^_":
            out += p
        else:
            out += " " + p
    return out


class Line:
    """One text line of the PDF (content-stream order) with geometry and bold info."""

    def __repr__(self):
        return "Line(p%d %.0f,%.0f %r)" % (self.page, self.x0, self.y0, self.text[:50])


def span_text(spans):
    """Text of one PDF line. Spans set in a clearly smaller size (<= 90% of the line's main
    size) and raised / lowered against the line's baseline are written as ^sup / _sub, so
    that equation exponents and subscripts keep their position (X^32, M_0, M_{n-33}).
    Short alphanumeric runs are written bare, anything else in braces. Only the position
    information of the PDF is used; no character is added other than ^ _ { }."""
    vis = [s for s in spans if s["text"].strip()]
    if not vis:
        return "".join(s["text"] for s in spans)
    base = max(s["size"] for s in vis)
    main = [s for s in vis if s["size"] >= base - 0.2]
    by = main[0]["origin"][1]
    out, run, mode = "", "", None

    def flush():
        nonlocal out, run, mode
        if run:
            out = out.rstrip(" ") + mode + (run if (re.fullmatch(r"[A-Za-z0-9]+", run) or len(run) == 1) else "{" + run + "}")
        run, mode = "", None

    for s in spans:
        t = s["text"]
        kind = None
        if t.strip() and s["size"] <= 0.9 * base:
            dy = s["origin"][1] - by
            kind = "^" if dy < -1.0 else ("_" if dy > 1.0 else None)
        if kind and (mode in (None, kind)):
            mode = kind
            run += t.strip()
            continue
        flush()
        if kind:
            mode = kind
            run = t.strip()
        else:
            out += t
    flush()
    return out


def page_lines(doc):
    """All text lines of the document in content-stream order, with positions/bold."""
    pages = []
    for pi, page in enumerate(doc):
        lines = []
        for b in page.get_text("dict")["blocks"]:
            if b["type"] != 0:
                continue
            for l in b["lines"]:
                spans = [s for s in l["spans"]]
                text = norm(span_text(spans))
                if not text:
                    continue
                ln = Line()
                ln.page = pi + 1
                ln.x0, ln.y0, ln.x1, ln.y1 = l["bbox"]
                ln.text = text
                vis = [s for s in spans if s["text"].strip()]
                ln.bold_first = bool(vis and vis[0]["flags"] & 16)
                ln.all_bold = bool(vis) and all(s["flags"] & 16 for s in vis)
                # the leading run of bold text
                bt = ""
                for s in spans:
                    if s["flags"] & 16 or not s["text"].strip():
                        bt += s["text"]
                    else:
                        break
                ln.bold_text = norm(bt)
                ln.size = round(vis[0]["size"], 1) if vis else 0
                ln.maxsize = max([x["size"] for x in vis] or [0])
                ln.spans = [(s["text"], bool(s["flags"] & 16)) for s in spans]
                ln.hf = False
                ln.cell = None
                ln.arial = bool(vis) and all("Arial" in x["font"] for x in vis) \
                    and not re.fullmatch(r"[\d\s]{1,3}", text)
                ln.rot = tuple(round(v) for v in l["dir"]) != (1, 0)
                lines.append(ln)
        pages.append(lines)
    return pages


def page_rules(page):
    """Horizontal / vertical rule segments (table borders) of a page."""
    hs, vs = [], []
    for dr in page.get_drawings():
        for it in dr["items"]:
            if it[0] == "l":
                a, b = it[1], it[2]
                if abs(a.y - b.y) < 0.6 and abs(a.x - b.x) > 8:
                    hs.append((round(a.y, 1), min(a.x, b.x), max(a.x, b.x)))
                elif abs(a.x - b.x) < 0.6 and abs(a.y - b.y) > 5:
                    vs.append((round(a.x, 1), min(a.y, b.y), max(a.y, b.y)))
            elif it[0] == "re":
                r = it[1]
                if r.height < 2.5 and r.width > 8:
                    hs.append((round((r.y0 + r.y1) / 2, 1), r.x0, r.x1))
                elif r.width < 2.5 and r.height > 5:
                    vs.append((round((r.x0 + r.x1) / 2, 1), r.y0, r.y1))
    return hs, vs


def table_regions(page):
    """Ruled tables on a page: list of dict(bbox, ys, xs). Connected clusters of rules."""
    hs, vs = page_rules(page)
    segs = [("h", y, x0, x1, y, y) for (y, x0, x1) in hs] + \
           [("v", x, y0, y1, 0, 0) for (x, y0, y1) in vs]
    boxes = []
    for s in segs:
        if s[0] == "h":
            boxes.append([s[2] - 1.5, s[1] - 1.5, s[3] + 1.5, s[1] + 1.5, s])
        else:
            boxes.append([s[1] - 1.5, s[2] - 1.5, s[1] + 1.5, s[3] + 1.5, s])
    n = len(boxes)
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    order = sorted(range(n), key=lambda i: boxes[i][0])
    for ai in range(n):
        a = order[ai]
        for bi in range(ai + 1, n):
            b = order[bi]
            if boxes[b][0] > boxes[a][2]:
                break
            if boxes[a][1] <= boxes[b][3] and boxes[b][1] <= boxes[a][3]:
                parent[find(a)] = find(b)
    groups = collections.defaultdict(list)
    for i in range(n):
        groups[find(i)].append(boxes[i])
    regions = []
    for g in groups.values():
        hh = [b[4] for b in g if b[4][0] == "h"]
        vv = [b[4] for b in g if b[4][0] == "v"]
        if len(hh) < 2 or len(vv) < 2:
            continue
        x0 = min(b[0] for b in g) + 1.5
        y0 = min(b[1] for b in g) + 1.5
        x1 = max(b[2] for b in g) - 1.5
        y1 = max(b[3] for b in g) - 1.5
        if (x1 - x0) < 40 or (y1 - y0) < 15:
            continue
        ys = sorted(set(round(s[1], 1) for s in hh))
        xs = sorted(set(round(s[1], 1) for s in vv))
        regions.append({"bbox": (x0, y0, x1, y1), "ys": ys, "xs": xs,
                        "nh": len(hh), "nv": len(vv)})
    return regions


def split_sentences(text):
    """Split a paragraph into sentences at '. ' '? ' '! ' followed by an upper-case letter,
    digit, quote or bracket. Abbreviations (e.g. i.e. etc.) never end a sentence."""
    out, start = [], 0
    for m in re.finditer(r"[.?!][\u2019\u201d)\]]?\s+(?=[A-Z0-9\u2018\u201c(\[])", text):
        end = m.end()
        before = text[start:m.start() + 1]
        last = re.split(r"\s", before)[-1].rstrip(".")
        if last.lower() in ABBREV or re.fullmatch(r"[A-Za-z]", last):
            continue
        out.append(text[start:m.start() + len(m.group(0).rstrip())])
        start = end
    if start < len(text):
        out.append(text[start:])
    return [s.strip() for s in out if s.strip()]


# --------------------------------------------------------------------------- page/line prep
def prepare_pages(doc):
    pages = page_lines(doc)
    labels = []
    for pl in pages:
        label = None
        for ln in pl:
            m = RE_FOOT_PAGE.match(ln.text)
            if m:
                label = m.group(1)
        labels.append(label)
        if label is None:
            continue  # cover pages: keep every line
        for i, ln in enumerate(pl):
            t = ln.text
            if RE_HEAD.match(t):
                ln.hf = True
                if t.rstrip().endswith("\u2014") and i + 1 < len(pl):
                    pl[i + 1].hf = True
            elif RE_FOOT_DOC.match(t) or RE_FOOT_DATE.match(t) or RE_FOOT_PAGE.match(t):
                ln.hf = True
    return pages, labels


def key_tuple(key):
    if key.startswith("ANNEX "):
        return (1, key[6:], ())
    if re.match(r"^[A-Z]\d", key):
        return (1, key[0], tuple(int(x) for x in re.split(r"\.", key[1:])))
    return (0, "", tuple(int(x) for x in key.split(".")))


def parent_key(key, known):
    if key.startswith("ANNEX "):
        return None
    if re.match(r"^[A-Z]\d", key):
        parts = key[1:].split(".")
        letter = key[0]
        while len(parts) > 1:
            parts = parts[:-1]
            k = letter + ".".join(parts)
            if k in known:
                return k
        return "ANNEX " + letter if ("ANNEX " + letter) in known else None
    parts = key.split(".")
    while len(parts) > 1:
        parts = parts[:-1]
        k = ".".join(parts)
        if k in known:
            return k
    return None


class Blk:
    """A paragraph-like block of text (clause paragraph, list item, note, table cell...)."""

    def __init__(self, kind, clause, ctx):
        self.kind = kind          # clause|heading|text|item|note|cell|caption|figtext
        self.clause = clause
        self.ctx = ctx            # clause|note|list-item|table|front-matter|figure ...
        self.num = None           # clause number token if the block opens a clause
        self.parts = []           # (text, page)
        self.last = None          # last Line
        self.label = None         # e.g. note number

    def add(self, ln, text):
        self.parts.append((text, ln.page))
        self.last = ln

    def joined(self):
        """Return (text, [(offset, page)])."""
        out, offs = "", []
        for p, pg in self.parts:
            p = norm(p)
            if not p:
                continue
            if not out:
                offs.append((0, pg))
                out = p
                continue
            if (out.endswith("-") and len(out) > 1 and out[-2].isalnum() and p[0].islower()) \
                    or out.endswith("\u2014") or p[0] in "^_":
                offs.append((len(out), pg))
                out += p
            else:
                offs.append((len(out) + 1, pg))
                out += " " + p
        return out, offs


def build_items(pages, regions_by_page, first_page):
    """Flatten lines of all pages in stream order; replace each ruled table by one 'table'
    item whose lines are grouped in cells."""
    items = []
    for pi, pl in enumerate(pages):
        regs = regions_by_page[pi]
        seen = {}
        for ln in pl:
            if ln.hf:
                continue
            yc = (ln.y0 + ln.y1) / 2
            rid = None
            if not ln.rot:
                for k, r in enumerate(regs):
                    x0, y0, x1, y1 = r["bbox"]
                    if x0 - 2 <= ln.x0 <= x1 and y0 - 2 <= yc <= y1 + 2:
                        rid = k
                        break
            if rid is None:
                items.append(("line", ln))
            else:
                if rid not in seen:
                    seen[rid] = {"page": pi + 1, "region": regs[rid], "lines": []}
                    items.append(("table", seen[rid]))
                seen[rid]["lines"].append(ln)
    return items


def table_cells(tbl):
    """Group lines of a ruled table into cells -> list of (row, col, [Line])"""
    r = tbl["region"]

    def clus(v):
        out = []
        for a in sorted(v):
            if not out or a - out[-1] > 1.5:
                out.append(a)
        return out
    ys, xs = clus(r["ys"]), clus(r["xs"])
    cells = collections.OrderedDict()
    for ln in sorted(tbl["lines"], key=lambda l: (round((l.y0 + l.y1) / 2), l.x0)):
        yc = (ln.y0 + ln.y1) / 2
        row = max(0, sum(1 for y in ys if y <= yc + 0.5) - 1)
        col = max(0, sum(1 for x in xs if x <= ln.x0 + 2) - 1)
        cells.setdefault((row, col), []).append(ln)
    out = []
    for (row, col) in sorted(cells):
        ls = sorted(cells[(row, col)], key=lambda l: (l.y0, l.x0))
        out.append((row, col, ls))
    return out, ys, xs


# --------------------------------------------------------------------------- book parser
RE_LIST = re.compile(r"^(?:[a-z]|[ivx]{1,4})\)\s")
RE_BULLET = re.compile(r"^[\u2013\u2022]\s")
RE_NOTE = re.compile(r"^NOTES?(?:\s*[\u2013\u2014-]|:|$)")
RE_CAPTION = re.compile(r"^(Figure|Table) [A-Z]?\d*-\d+\b")
TERMINAL = (".", "?", "!", ":", ";")


def first_token(ln):
    return ln.text.split(" ", 1)[0]


def after_token_bold(ln, tok):
    """Is the first visible text after the leading clause number (tok) bold?"""
    need, seen = len(tok), 0
    for text, bold in ln.spans:
        for ch in text.replace("\xa0", " "):
            if seen < need:
                if not ch.isspace():
                    seen += 1
                continue
            if not ch.isspace():
                return bold
    return None


class BookParse:
    pass


def parse_book(book_id, path):
    doc = pymupdf.open(path)
    pages, labels = prepare_pages(doc)
    regions = [table_regions(p) for p in doc]
    bp = BookParse()
    bp.id = book_id
    bp.pages = pages
    bp.labels = labels
    bp.npages = len(doc)
    # first body page = first page whose printed label is 1-1
    body0 = next(i for i, pl in enumerate(pages) for ln in pl
                 if ln.bold_first and re.match(r"^1\s+INTRODUCTION\s*$", ln.text))
    bp.body0 = body0
    items = build_items(pages, regions, body0)
    bp.items = items
    bp.tables = []
    blocks = []
    nodes = collections.OrderedDict()      # clause key -> dict
    state = {"clause": None, "last_tuple": None, "annex": None, "in_notes": False, "cur": None,
             "lead": None, "warn": []}
    bp.warnings = state["warn"]
    margin = {}

    def label_of(pg):
        return labels[pg - 1]

    def close():
        state["cur"] = None

    def new_block(kind, ln, ctx=None):
        front = ln.page - 1 < body0
        if ctx is None:
            ctx = "front-matter" if front else kind
        b = Blk(kind, None if front else state["clause"], ctx)
        b.page0 = ln.page
        blocks.append(b)
        state["cur"] = b
        return b

    def start_clause(key, title_parts, ln, kind):
        t = key_tuple(key)
        known = nodes
        node = {"key": key, "title": None, "page": label_of(ln.page), "pdf_page": ln.page,
                "parent": parent_key(key, known), "kind": kind}
        nodes[key] = node
        state["last_tuple"] = t
        state["clause"] = key
        state["in_notes"] = False
        return node

    n = len(items)
    i = 0
    while i < n:
        typ, it = items[i]
        i += 1
        if typ == "table":
            close()
            cells, ys, xs = table_cells(it)
            tb = {"page": it["page"], "cells": [], "ys": ys, "xs": xs,
                  "clause": state["clause"]}
            bp.tables.append(tb)
            front = it["page"] - 1 < body0
            for row, col, ls in cells:
                b = Blk("cell", None if front else state["clause"],
                        "front-matter" if front else "table")
                b.page0 = it["page"]
                for l in ls:
                    b.add(l, l.text)
                b.row, b.col = row, col
                blocks.append(b)
                tb["cells"].append((row, col, b))
            continue
        ln = it
        front = ln.page - 1 < body0
        t = ln.text
        cur = state["cur"]

        if ln.rot or (ln.arial and not front) or (0 < ln.maxsize < 9.0 and not front):
            close()
            b = new_block("figtext", ln, "figure")
            b.add(ln, t)
            close()
            continue

        # ---- clause / annex markers (body only)
        if not front and ln.bold_first:
            tok = first_token(ln)
            is_num = bool((RE_NUM_BODY.match(tok) and state["annex"] is None) or
                          (RE_NUM_ANNEX.match(tok) and tok[0] == state["annex"]))
            m_annex = RE_ANNEX_HEAD.match(ln.bold_text.strip()) if t.startswith("ANNEX ") else None
            if m_annex:
                key = "ANNEX " + m_annex.group(1)
                if key not in nodes:
                    state["annex"] = m_annex.group(1)
                    node = start_clause(key, None, ln, "annex")
                    parts = [t[len(key):].strip()] if t[len(key):].strip() else []
                    # title lines: following bold lines on the same page
                    while i < n and items[i][0] == "line" and items[i][1].page == ln.page \
                            and items[i][1].all_bold and not (
                                RE_NUM_BODY.match(first_token(items[i][1])) or
                                RE_NUM_ANNEX.match(first_token(items[i][1]))):
                        parts.append(items[i][1].text)
                        i += 1
                    node["title"] = join_text(parts) or None
                    close()
                    b = new_block("heading", ln)
                    b.num = key
                    b.add(ln, node["title"] or "")
                    close()
                    continue
            elif is_num and tok == ln.bold_text.split(" ")[0]:
                key = tok
                tk = key_tuple(key)
                ok = key not in nodes and (state["last_tuple"] is None or
                                           (tk[0], tk[1]) != (state["last_tuple"][0], state["last_tuple"][1])
                                           or tk[2] > state["last_tuple"][2])
                if state["last_tuple"] is not None and state["last_tuple"][0] == 1 and tk[0] == 0:
                    ok = False
                rest0 = t[len(tok):].strip()
                if ok and "." not in key and rest0 and not rest0.isupper():
                    ok = False      # figure/table text such as "4 octets"
                if ok and tk[0] == 0 and "." not in key:
                    # top level section must follow the previous top level number
                    prev_top = max([int(k) for k in nodes if re.fullmatch(r"\d+", k)] or [0])
                    ok = int(key) == prev_top + 1
                if ok:
                    rest = t[len(tok):].strip()
                    frag = [rest] if rest else []
                    bold_rest = after_token_bold(ln, tok) if rest else None
                    # title / text may sit on following line(s) at the same height
                    j = i
                    while j < n and items[j][0] == "line" and items[j][1].page == ln.page \
                            and abs(items[j][1].y0 - ln.y0) <= 3 and items[j][1].x0 >= ln.x1 - 2:
                        l2 = items[j][1]
                        frag.append(l2.text)
                        if bold_rest is None:
                            bold_rest = l2.bold_first
                        j += 1
                    i = j
                    close()
                    is_heading = bool(bold_rest) and bool(frag)
                    if is_heading:
                        node = start_clause(key, None, ln, "heading")
                        b = new_block("heading", ln)
                        b.num = key
                        b.add(ln, " ".join(frag))
                        b.last = ln
                        state["cur"] = b          # heading may wrap onto next line
                        b.head_open = True
                        node["_blk"] = b
                    else:
                        node = start_clause(key, None, ln, "paragraph")
                        b = new_block("clause", ln)
                        b.num = key
                        for f in frag:
                            b.add(ln, f)
                        b.last = ln
                        node["_blk"] = b
                    continue
                else:
                    state["warn"].append("p%d: bold numeric start not accepted as clause: %r" % (ln.page, t[:60]))

        # ---- continuation of an open block?
        cont = False
        if cur is not None:
            p = cur.last
            if ln.page == p.page:
                dy = ln.y0 - p.y0
                cont = (-3 <= dy <= (1.6 if re.search(r"[\^_]", p.text) else 1.45) * max(p.maxsize, 8))
                if cur.kind == "heading" and not getattr(cur, "head_open", False):
                    cont = False
            else:
                if cur.kind in ("heading", "caption", "figtext", "cell"):
                    cont = False
                else:
                    txt = norm(" ".join(x for x, _ in cur.parts))
                    cont = not txt.endswith(TERMINAL) and bool(txt)
            if cont and cur.kind == "heading":
                cont = ln.all_bold or (ln.page == p.page and abs(ln.x0 - p.x0) < 200)
            # new marker lines never continue
        starts_marker = bool(RE_LIST.match(t) or RE_BULLET.match(t) or RE_NOTE.match(t) or
                             RE_CAPTION.match(t))
        in_notes_num = False
        if state["in_notes"] and not front:
            m = re.match(r"^(\d{1,2})(?:\s+|$)", t)
            left = min([l.x0 for l in pages[ln.page - 1] if not l.hf and not l.rot] or [ln.x0])
            if m and ln.x0 <= left + 6:
                in_notes_num = True
        if cont and cur is not None and (starts_marker or in_notes_num):
            cont = False
        if cont:
            if cur.kind == "heading":
                pass
            cur.add(ln, t)
            continue

        # ---- start a new block
        close()
        if RE_CAPTION.match(t):
            b = new_block("caption", ln, "figure")
            b.add(ln, t)
            continue
        if RE_NOTE.match(t):
            state["in_notes"] = True
            b = new_block("note", ln, "note")
            m = re.match(r"^NOTES?\s*[\u2013\u2014-]?\s*:?\s*(.*)$", t)
            b.label = "NOTE" if t.startswith("NOTE") and not t.strip() == "NOTES" else "NOTES"
            b.add(ln, m.group(1))
            if t.strip() == "NOTES":
                b.parts = []
                b.label = "NOTES"
            continue
        if in_notes_num:
            m = re.match(r"^(\d{1,2})(?:\s+(.*))?$", t)
            b = new_block("note", ln, "note")
            b.label = "NOTE " + m.group(1)
            if m.group(2):
                b.add(ln, m.group(2))
            else:
                # text on the following same-height line
                if i < n and items[i][0] == "line" and abs(items[i][1].y0 - ln.y0) <= 3 \
                        and items[i][1].page == ln.page:
                    b.add(items[i][1], items[i][1].text)
                    i += 1
            continue
        if RE_LIST.match(t) or RE_BULLET.match(t):
            b = new_block("item", ln, "list-item")
            b.add(ln, t)
            continue
        if state["in_notes"] and not front:
            state["in_notes"] = False
        b = new_block("text", ln)
        b.add(ln, t)
    for b in blocks:
        if b.ctx == "front-matter" and b.parts and re.match(r"^o\s", b.parts[0][0]):
            b.parts[0] = (b.parts[0][0][2:], b.parts[0][1])      # bullet glyph rendered as 'o'
    blocks = [b for b in blocks if norm(" ".join(x for x, _ in b.parts)) or b.num]
    bp.blocks = blocks
    bp.nodes = nodes
    for k, nd in nodes.items():
        b = nd.pop("_blk", None)
        if nd["kind"] in ("heading", "annex") and nd["title"] is None and b is not None:
            nd["title"] = b.joined()[0] or None
    return bp


# --------------------------------------------------------------------------- sentences
def modal_counts(text):
    return {name: len(rx.findall(text)) for name, rx in MODALS if rx.search(text)}


def extract_sentences(bp):
    """Every sentence containing shall/should/may, with clause, page, context."""
    out = []
    lead = None
    for bi, b in enumerate(bp.blocks):
        text, offs = b.joined()
        if b.kind in ("clause", "heading", "text", "caption"):
            lead = text if text.endswith(":") else None
        if b.kind == "figtext":
            continue
        pos = 0
        for sent in split_sentences(text):
            start = text.find(sent, pos)
            pos = start + len(sent)
            cnt = modal_counts(sent)
            if not cnt:
                continue
            pg0 = [pg for (o, pg) in offs if o <= start][-1]
            pg1 = [pg for (o, pg) in offs if o <= start + len(sent) - 1][-1]
            primary = "shall" if "shall" in cnt else ("should" if "should" in cnt else "may")
            last_in_block = text[start + len(sent):].strip() == ""
            cont = []
            if last_in_block and b.kind in ("clause", "text") and not sent.endswith((".", "?", "!")):
                for nb in bp.blocks[bi + 1:]:
                    if nb.kind == "item" and nb.clause == b.clause:
                        cont.append(nb.joined()[0])
                    else:
                        break
            out.append({"block": bi, "continues_in_items": cont, "clause": b.clause if b.clause else None, "ctx": b.ctx,
                        "page": bp.labels[pg0 - 1], "pdf_page": pg0,
                        "page_end": bp.labels[pg1 - 1] if pg1 != pg0 else None,
                        "pdf_page_end": pg1 if pg1 != pg0 else None,
                        "text": sent, "kind": primary, "counts": cnt,
                        "lead_in": lead if b.kind == "item" else None})
    return out


# --------------------------------------------------------------------------- PICS (Annex A)
RE_ITEM = re.compile(r"^(?:[A-Z]{1,6}-)?\d+(?:\.\d+)*$")
ROLES5 = ["Item", "Description", "Reference", "Status", "Support"]
ROLES6 = ["Item", "Description", "Reference", "Status", "Values Allowed", "Support"]
RE_REFPART = re.compile(r"^(\d{1,2}(?:\.\d{1,3})*|[A-Z]\d{1,2}(?:\.\d{1,3})*)(?:\s+[a-z]\))?$")
RE_REFTOK = re.compile(r"(?<![\w.\[])(\d{1,2}(?:\.\d{1,3})+|[A-Z]\d{1,2}(?:\.\d{1,3})*|\d{1,2})(?![\w])")


def extract_pics(bp):
    """Annex A Requirements List rows exactly as printed, one dict per item row.
    Returns (rows, notes)."""
    rows, notes = [], []
    group_cur = None
    for tb in bp.tables:
        if not tb["clause"] or not tb["clause"].startswith("A2"):
            continue
        cells = {(r, c): b.joined()[0] for r, c, b in tb["cells"]}
        if not cells:
            continue
        ncols = max(c for _, c in cells) + 1
        hrow = next((r for r, c in sorted(cells) if c == 0 and cells[(r, c)] == "Item"), None)
        if hrow is None:
            continue
        hdr = " ".join(cells.get((hrow, c), "") for c in range(ncols))
        hdr_ns = hdr.replace(" ", "")
        if not ("Description" in hdr and "Reference" in hdr_ns and "Status" in hdr):
            continue
        roles = ROLES6 if "Values" in hdr else ROLES5
        if ncols != len(roles):
            bp.warnings.append("PICS table p%d has %d columns, expected %d" % (tb["page"], ncols, len(roles)))
            continue
        for r in range(hrow):                    # group title printed above the header row
            t = " ".join(cells.get((r, c), "") for c in range(ncols)).strip()
            if t:
                group_cur = t
        for r in sorted({r for r, _ in cells if r > hrow}):
            vals = {roles[c]: cells.get((r, c), "") for c in range(ncols)}
            item = vals["Item"]
            if RE_ITEM.match(item):
                ref = vals["Reference"]
                ext = re.search(r"[Rr]eference\s+\[(\w+)\]", ref)
                toks, other = [], []
                if not ext:
                    refn = re.sub(r"(\d\.)\s+(\d)", r"\1\2", ref)       # clause number wrapped in the cell
                    refn = re.sub(r"\b(?:[Tt]ables?|[Ff]igures?)\s+[A-Z]?\d*-\d+(?:\s*,\s*[A-Z]?\d*-\d+)*", " ", refn)
                    for part in re.split(r"[,;]|\u2013", refn):
                        part = part.strip()
                        m = RE_REFPART.match(part)
                        if m:
                            toks.append(m.group(1))
                        elif part:
                            other.append(part)
                rows.append({
                    "item": item, "description": vals["Description"], "reference": ref,
                    "status": vals["Status"], "values_allowed": vals.get("Values Allowed"),
                    "support": vals["Support"], "group": group_cur,
                    "page": bp.labels[tb["page"] - 1], "pdf_page": tb["page"],
                    "ref_external": ext.group(1) if ext else None,
                    "ref_clauses": toks, "ref_other": other,
                    "does_not_apply_when": None, "option_tag": None})
            elif not any(vals[k] for k in vals if k not in ("Item", "Description")):
                group_cur = (item + " " + vals["Description"]).strip()
            else:
                bp.warnings.append("PICS p%d row not classified: %r" % (tb["page"], vals))
    for b in bp.blocks:
        if b.clause == "A2.2" and b.kind in ("text", "item", "note", "clause"):
            t = b.joined()[0]
            if t:
                notes.append({"text": t, "page": bp.labels[b.page0 - 1], "pdf_page": b.page0})
    return rows, notes


# --------------------------------------------------------------------------- cross references
UNITS = r"(?:dB|dBm|Hz|kHz|MHz|GHz|ms|s|sec|seconds?|kbps|bps|bits?|octets?|bytes?|%|m|km|W|ppm|degrees?)\b"
RE_CL = re.compile(r"(?<![\w.\[#/-])((?:\d{1,2}(?:\.\d{1,3}){1,6})|(?:[A-Z]\d{1,2}(?:\.\d{1,3}){0,5}))"
                   r"(?![\w]|\.\d|-\d)(?!\s*" + UNITS + ")")
RE_SEC = re.compile(r"\b(?:sub)?sections?\s+(\d{1,2})\b(?![\d.-])")
RE_ANNEX = re.compile(r"\bannex(?:es)?\s+([A-Z])\b(?!\d)")
RE_FIGTAB = re.compile(r"\b([Ff]igure|[Tt]able)s?\s+((?:[A-Z]?\d*-\d+)(?:(?:,|,?\s+and|\s+through|\s+or|\s+to)\s+[A-Z]?\d*-\d+)*)")
STRONG_CUE = re.compile(r"(?:see|in|of|per|by|with|sections?|subsections?|clause|paragraphs?|annex|"
                        r"according to|specified in|defined in|described in)\s+$", re.I)
RE_REFBR = re.compile(r"\[([A-Z]?\d{1,2})\]")
CUE_BEFORE = re.compile(r"(?:see|in|of|to|and|or|per|by|with|from|through|annex|section|sections|subsection|subsections|"
                        r"clause|paragraph|paragraphs|according to|above|below|under|at|,|\()\s*$", re.I)


def build_refs_table(bp):
    refs = collections.OrderedDict()
    for b in bp.blocks:
        t = b.joined()[0]
        m = re.match(r"^\[([A-Z]?\d{1,2})\]\s+(.*)$", t)
        if m and b.kind in ("text", "clause") and b.ctx != "front-matter" and b.clause is not None \
                and (bp.nodes[b.clause]["title"] or "").upper().find("REFERENCES") >= 0:
            refs[m.group(1)] = {"label": "[%s]" % m.group(1), "text": m.group(2),
                                "page": bp.labels[b.page0 - 1], "pdf_page": b.page0}
    return refs


def figure_table_index(bp):
    out = collections.OrderedDict()
    for b in bp.blocks:
        if b.kind == "caption":
            t = b.joined()[0]
            m = RE_CAPTION.match(t)
            lab = t.split(":")[0].strip()
            out[lab.lower()] = {"label": lab, "caption": t, "page": bp.labels[b.page0 - 1],
                                "pdf_page": b.page0, "clause": b.clause}
    return out


def all_sentences(bp):
    """(block_index, sentence, page_label, pdf_page) for every sentence of running text."""
    for bi, b in enumerate(bp.blocks):
        if b.kind in ("figtext", "caption"):
            continue
        text, offs = b.joined()
        pos = 0
        for sent in split_sentences(text):
            start = text.find(sent, pos)
            pos = start + len(sent)
            pg = [p for (o, p) in offs if o <= start][-1]
            yield bi, b, sent, bp.labels[pg - 1], pg


def extract_xrefs(bp, refs, figtab):
    out = []
    for bi, b, sent, plabel, ppdf in all_sentences(bp):
        if b.ctx == "front-matter" or b.clause is None:
            continue
        # skip the reference list entries themselves
        if re.match(r"^\[[A-Z]?\d{1,2}\]\s", sent) and b.kind == "text":
            continue
        if b.kind == "cell" and b.clause.startswith("A2"):
            continue                                  # PICS rows are handled by extract_pics
        found = []   # (start, end, rec)

        def ext_of(s, e):
            """Reference [X] named immediately after / before the mention (same phrase)."""
            after = sent[e:e + 60]
            if re.match(r"^\s+of\s+this\s+(?:document|Recommended Standard)", after):
                return None
            m = re.match(r"^\s*,?\s*(?:of|in)\s+(?:the\s+)?(?:[\w-]+\s+){0,2}?[Rr]eference\s+\[([A-Z]?\d{1,2})\]", after)
            if m:
                return m.group(1)
            before = sent[max(0, s - 60):s]
            m = re.search(r"[Rr]eference\s+\[([A-Z]?\d{1,2})\]\s*,?\s*(?:(?:sub)?sections?\s+|annex\s+)?$", before)
            if m:
                return m.group(1)
            return None

        for m in RE_CL.finditer(sent):
            tok = m.group(1)
            before = sent[max(0, m.start() - 22):m.start()]
            if tok[0].isdigit():
                if tok.split(".")[0] == "0":
                    continue
                if tok not in bp.nodes and not STRONG_CUE.search(before):
                    continue
            else:
                if tok not in bp.nodes or not STRONG_CUE.search(before):
                    continue
            ex = ext_of(m.start(), m.end())
            found.append((m.start(), m.end(), {"kind": "clause", "target": tok, "external_ref": ex,
                                               "mention": m.group(0)}))
        for m in RE_SEC.finditer(sent):
            ex = ext_of(m.start(), m.end())
            found.append((m.start(), m.end(), {"kind": "clause", "target": m.group(1), "external_ref": ex,
                                               "mention": m.group(0)}))
        for m in RE_ANNEX.finditer(sent):
            ex = ext_of(m.start(), m.end())
            found.append((m.start(), m.end(), {"kind": "clause", "target": "ANNEX " + m.group(1),
                                               "external_ref": ex, "mention": m.group(0)}))
        for m in RE_FIGTAB.finditer(sent):
            kind = m.group(1).lower()
            for lab in re.findall(r"[A-Z]?\d*-\d+", m.group(2)):
                found.append((m.start(), m.end(), {"kind": kind, "target": "%s %s" % (m.group(1).capitalize(), lab),
                                                   "external_ref": ext_of(m.start(), m.end()),
                                                   "mention": m.group(0)}))
        for m in RE_REFBR.finditer(sent):
            found.append((m.start(), m.end(), {"kind": "reference", "target": m.group(1), "external_ref": None,
                                               "mention": m.group(0)}))
        # drop duplicates produced by two patterns (e.g. 'section 3' + '3.2')
        seen = set()
        for s, e, rec in sorted(found, key=lambda x: (x[0], x[1])):
            key = (rec["kind"], rec["target"], s)
            if key in seen:
                continue
            seen.add(key)
            if rec["kind"] == "clause":
                if rec["external_ref"]:
                    rec["resolved"] = None
                    rec["scope"] = "external"
                else:
                    rec["scope"] = "internal"
                    rec["resolved"] = rec["target"] in bp.nodes
                    if rec["target"] == b.clause:
                        continue
            elif rec["kind"] == "reference":
                rec["scope"] = "reference-list"
                rec["resolved"] = rec["target"] in refs
            else:
                rec["scope"] = "internal"
                rec["resolved"] = rec["target"].lower() in figtab
            rec.update({"source_clause": b.clause, "sentence": sent, "page": plabel, "pdf_page": ppdf,
                        "block": bi})
            out.append(rec)
    return out


# --------------------------------------------------------------------------- front matter
def parse_cover(bp):
    ls = sorted(bp.pages[0], key=lambda l: (l.y0, l.x0))
    lines = [l.text for l in ls]
    try:
        top = next(l for l in ls if l.text.startswith("Recommendation for Space Data System"))
        bot = next(l for l in ls if l.text.startswith("RECOMMENDED STANDARD"))
    except StopIteration:
        return {}
    title_lines = [l.text for l in ls if top.y0 < l.y0 < bot.y0 and l.text != "BLUE BOOK"]
    doc = next((t for t in lines if re.match(r"^CCSDS \d{3}\.\d-B-\d+$", t)), None)
    date = next((t for t in lines if RE_FOOT_DATE.match(t)), None)
    colour = next((t for t in lines if t.endswith("BOOK")), None)
    auth = " ".join(ln.text for pl in bp.pages[:6] for ln in pl)
    m = re.search(r"Issue:\s*(Recommended Standard, Issue \d+)\s*Date:\s*(\w+ \d{4})", auth)
    return {"title": join_text(title_lines), "series_line": top.text, "status_line": bot.text,
            "designation": doc, "colour": colour, "date": date,
            "issue": m.group(1) if m else None, "authority_date": m.group(2) if m else None}


def document_control(bp):
    out = []
    for pi, pl in enumerate(bp.pages[:bp.body0]):
        if any(ln.text.strip() == "DOCUMENT CONTROL" for ln in pl):
            for b in bp.blocks:
                if b.page0 == pi + 1 or (b.kind == "cell" and b.page0 == pi + 1):
                    t = b.joined()[0]
                    if t and t != "DOCUMENT CONTROL":
                        out.append(t)
            return {"page": bp.labels[pi], "pdf_page": pi + 1, "text": out}
    return None


# --------------------------------------------------------------------------- assemble standard.json
def assemble_book(bid, pdf_path):
    bp = parse_book(bid, pdf_path)
    sents = extract_sentences(bp)
    pics, pics_notes = extract_pics(bp)
    refs = build_refs_table(bp)
    figtab = figure_table_index(bp)
    xrefs = extract_xrefs(bp, refs, figtab)
    cover = parse_cover(bp)
    # ids
    by_block = collections.defaultdict(list)
    for n, s in enumerate(sents, 1):
        s["id"] = "%s/s%03d" % (bid, n)
        by_block[s["block"]].append(s["id"])
    # clause contents
    content = collections.OrderedDict((k, []) for k in bp.nodes)
    cur_table = None
    gfx = collections.OrderedDict()
    for bi, b in enumerate(bp.blocks):
        if b.kind == "figtext" and b.clause is not None and b.ctx != "front-matter":
            gfx[(b.clause, b.page0)] = gfx.get((b.clause, b.page0), 0) + 1
        if b.ctx == "front-matter" or b.clause is None or b.kind in ("figtext",):
            cur_table = None
            continue
        if b.kind == "cell":
            if b.clause.startswith("A2"):
                continue
            prev = bp.blocks[bi - 1] if bi else None
            if cur_table is None or prev is None or prev.kind != "cell" or cur_table["clause"] != b.clause:
                cur_table = {"t": "table", "rows": [], "page": bp.labels[b.page0 - 1], "pdf_page": b.page0,
                             "clause": b.clause}
                content[b.clause].append(cur_table)
            while len(cur_table["rows"]) <= b.row:
                cur_table["rows"].append({})
            cur_table["rows"][b.row][b.col] = b.joined()[0]
            continue
        cur_table = None
        if b.kind == "heading":
            continue
        text = b.joined()[0]
        if not text:
            continue
        kind = {"clause": "p", "text": "p", "item": "item", "note": "note", "caption": "caption"}[b.kind]
        item = {"t": kind, "text": text, "page": bp.labels[b.page0 - 1], "pdf_page": b.page0}
        if b.num:
            item["num"] = b.num
        if b.label:
            item["label"] = b.label
        if by_block.get(bi):
            item["sids"] = by_block[bi]
        content[b.clause].append(item)
    for k, items in content.items():
        for it in items:
            if it["t"] == "table":
                it["rows"] = [[r.get(c, "") for c in range(max(r) + 1)] if r else [] for r in it["rows"]]
                it.pop("clause", None)
    clauses = []
    for k, nd in bp.nodes.items():
        cl = {"key": k, "title": nd["title"], "page": nd["page"], "pdf_page": nd["pdf_page"],
              "parent": nd["parent"], "kind": nd["kind"], "content": content[k]}
        om = [{"pdf_page": pp, "page": bp.labels[pp - 1], "lines": n} for (ck, pp), n in gfx.items() if ck == k]
        if om:
            cl["graphic_text_omitted"] = om
        clauses.append(cl)
    out_sents = []
    for s in sents:
        core = re.sub(r"\s+(?:and|or)$", "", s["text"])
        incomplete = ("shall" in s["counts"] and not core.endswith((".", "?", "!", ":", ";", ","))
                      and not s["continues_in_items"])
        cont = None
        if incomplete:
            # the sentence runs on through display equations: collect the following blocks
            # (verbatim) up to the one that ends with a full stop (at most 6 blocks)
            parts = []
            for nb in bp.blocks[s["block"] + 1:s["block"] + 7]:
                parts.append(nb.joined()[0])
                if parts[-1].endswith("."):
                    break
            if parts and parts[-1].endswith("."):
                cont = join_text(parts)
                incomplete = False
            elif parts:
                cont = parts[0]
        out_sents.append({"id": s["id"], "book": bid, "clause": s["clause"], "ctx": s["ctx"],
                          "page": s["page"], "pdf_page": s["pdf_page"], "page_end": s["page_end"],
                          "pdf_page_end": s["pdf_page_end"], "text": s["text"], "kind": s["kind"],
                          "counts": s["counts"], "continues_in_items": s["continues_in_items"],
                          "lead_in": s["lead_in"], "incomplete": incomplete,
                          "continues_in_next_block": cont,
                          "does_not_apply_when": None, "option_tag": None})
    xr = []
    for n, x in enumerate(xrefs, 1):
        xr.append({"id": "%s/x%03d" % (bid, n), "book": bid, "source_clause": x["source_clause"],
                   "sentence": x["sentence"], "page": x["page"], "pdf_page": x["pdf_page"],
                   "mention": x["mention"], "target_kind": x["kind"], "target": x["target"],
                   "scope": x["scope"], "external_ref": x["external_ref"], "resolved": x["resolved"]})
    for k in refs.values():
        pass
    book = {
        "id": bid, "pdf": os.path.basename(pdf_path), "pdf_pages": bp.npages,
        "title": cover.get("title"), "designation": cover.get("designation"),
        "colour": cover.get("colour"), "date": cover.get("date"), "issue": cover.get("issue"),
        "cover": cover, "document_control": document_control(bp),
        "clauses": clauses, "sentences": out_sents, "pics": pics, "pics_notes": pics_notes,
        "crossrefs": xr, "references": list(refs.values()),
        "figures_tables": list(figtab.values()),
    }
    return book, bp


# --------------------------------------------------------------------------- implementation scan
CODE_EXT = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".inl", ".ino", ".py", ".cmake", ".ps1", ".sh",
            ".s", ".S", ".ld", ".js", ".ts"}
CODE_NAMES = {"CMakeLists.txt"}
DOC_EXT = {".md", ".txt", ".rst"}
SCAN_ROOTS = ["src", "starcom", "test", "tests", "include"]
SKIP_DIRS = {"graphify-out", "standards", "third_party", ".git", "build", "build_host", "node_modules",
             "__pycache__", "docs"}
RE_BOOK = re.compile(r"(?:CCSDS\s*)?\b(133\.0|211\.0|211\.1|211\.2)(?:-B-(\d+))?")
RE_OTHERDOC = re.compile(r"\b(?:131\.0|132\.0|231\.0|232\.0|232\.1|732\.1|301\.0|133\.1|355\.0|727\.0|"
                         r"[0-9]{3}\.[0-9])(?:-[A-Z]-\d+)?\b")
RE_CITE_CLAUSE = re.compile(r"(?:§\s*|\b(?:[Ss]ection|[Ss]ubsection|[Cc]lause)s?\s+)?"
                            r"(?<![\w.])(\d{1,2}(?:\.\d{1,3})+|(?<=§)\d{1,2}|(?<=§ )\d{1,2}|"
                            r"(?<=[Ss]ection )\d{1,2}|(?<=[Ss]ubsection )\d{1,2})(?![\w]|\.\d)")
RE_CITE_ANNEX = re.compile(r"\b[Aa]nnex\s+([A-Z])\b(?:\s+(?:§\s*)?([A-Z]\d{1,2}(?:\.\d{1,3})*))?")
RE_CITE_FT = re.compile(r"\b(?:([Tt]able|[Ff]ig(?:ure)?)\.?\s*(\d{1,2}-\d{1,2})|(T)(\d{1,2}-\d{1,2}))\b")


def iter_code_files(repo):
    for root in SCAN_ROOTS:
        base = os.path.join(repo, root)
        if not os.path.isdir(base):
            continue
        for dp, dns, fns in os.walk(base):
            dns[:] = sorted(d for d in dns if d not in SKIP_DIRS and not d.startswith("build"))
            for fn in sorted(fns):
                ext = os.path.splitext(fn)[1]
                if ext in CODE_EXT or fn in CODE_NAMES:
                    yield os.path.join(dp, fn)


def scan_implementation(repo, nodes_by_book, include_docs=False):
    cites = []
    no_target = []
    stats = {"files_scanned": 0, "files_with_citations": 0}
    files = list(iter_code_files(repo))
    if include_docs:
        for root in SCAN_ROOTS:
            base = os.path.join(repo, root)
            for dp, dns, fns in os.walk(base):
                dns[:] = sorted(d for d in dns if d not in SKIP_DIRS - {"docs"} and not d.startswith("build"))
                for fn in sorted(fns):
                    if os.path.splitext(fn)[1] in DOC_EXT:
                        files.append(os.path.join(dp, fn))
    for path in files:
        try:
            if os.path.getsize(path) > 2_000_000:
                continue
            with open(path, "r", encoding="utf-8", errors="replace") as fh:
                lines = fh.read().splitlines()
        except OSError:
            continue
        stats["files_scanned"] += 1
        rel = os.path.relpath(path, repo).replace(os.sep, "/")
        hit = False
        for ln_no, line in enumerate(lines, 1):
            bks = list(RE_BOOK.finditer(line))
            if not bks:
                continue
            for bi, m in enumerate(bks):
                seg_end = bks[bi + 1].start() if bi + 1 < len(bks) else len(line)
                other = RE_OTHERDOC.search(line, m.end(), seg_end)
                # another standard (e.g. 131.0, 232.1) ends this citation segment
                while other and other.group(0)[:5] in ("133.0", "211.0", "211.1", "211.2"):
                    other = RE_OTHERDOC.search(line, other.end(), seg_end)
                if other:
                    seg_end = other.start()
                seg = line[m.end():seg_end]
                book = {"133.0": "133.0-B-2", "211.0": "211.0-B-6", "211.1": "211.1-B-4",
                        "211.2": "211.2-B-3"}[m.group(1)]
                targets = []
                for c in RE_CITE_CLAUSE.finditer(seg):
                    targets.append(("clause", c.group(1), c.group(0).strip()))
                for a in RE_CITE_ANNEX.finditer(seg):
                    if a.group(2):
                        targets.append(("clause", a.group(2), a.group(0).strip()))
                    targets.append(("clause", "ANNEX " + a.group(1), a.group(0).strip()))
                for f in RE_CITE_FT.finditer(seg):
                    if f.group(1):
                        word = "Table" if f.group(1).lower().startswith("t") else "Figure"
                        targets.append((word.lower(), "%s %s" % (word, f.group(2)), f.group(0)))
                    else:
                        targets.append(("table", "Table " + f.group(4), f.group(0)))
                seen = set()
                if not targets:
                    no_target.append({"book": book, "file": rel, "line": ln_no, "text": line.strip()[:240]})
                for kind, tgt, matched in targets:
                    if (kind, tgt) in seen:
                        continue
                    seen.add((kind, tgt))
                    rec = {"book": book, "target_kind": kind, "target": tgt, "matched": matched,
                           "file": rel, "line": ln_no, "text": line.strip()[:240]}
                    if kind == "clause":
                        rec["resolved"] = tgt in nodes_by_book.get(book, ())
                    else:
                        rec["resolved"] = None
                    cites.append(rec)
                    hit = True
        if hit:
            stats["files_with_citations"] += 1
    cites.sort(key=lambda r: (r["file"], r["line"], r["book"], r["target"]))
    return cites, no_target, stats


# --------------------------------------------------------------------------- SHALL_MATRIX.md
def md_cell(t):
    return t.replace("\\", "\\\\").replace("|", "\\|").replace("*", "\\*").replace("_", "\\_").replace("\n", " ")


def write_matrix(path, standard):
    L = []
    L.append("# SHALL matrix: CCSDS Proximity-1 / Space Packet books\n")
    L.append("Generated by `standards/starcom/ccsds/navigator/build_navigator.py` from "
             "`standards/starcom/ccsds/navigator/data/standard.json` (the stored CCSDS PDFs). Do not edit by hand; "
             "re-run the generator. Quotes are the sentence text as printed in the book, with the page where the "
             "sentence starts. The Status column is `Unchecked` for every row; it is not decided by the generator. "
             "`Does not apply when` and `Option tag` are empty and are filled in by people.\n")
    L.append("| Book | Title as printed on cover | Issue (authority page) / date (cover) | Rows |")
    L.append("|---|---|---|---|")
    for b in standard["books"]:
        n = sum(1 for s in b["sentences"] if "shall" in s["counts"])
        L.append("| CCSDS %s | %s | %s / %s | %d |" % (b["id"], md_cell(b["title"] or ""), md_cell(b["issue"] or "none printed"),
                                                      md_cell(b["date"] or "none printed"), n))
    L.append("")
    L.append("Each row is one sentence containing the word `shall`. A sentence that is a lead-in to a list is "
             "followed by its list items in the same quote. `p.` is the printed page label; `PDF p.` is the page "
             "number in the PDF file. In equations `^` marks a superscript and `\\_` a subscript "
             "(`X^32` is X with superscript 32); the backslashes before `*`, `_` and `|` only keep Markdown from "
             "formatting the quote.\n")
    L.append("| ID | Book | Clause | Verbatim quote | Page | Status | Does not apply when | Option tag |")
    L.append("|---|---|---|---|---|---|---|---|")
    for b in standard["books"]:
        for s in b["sentences"]:
            if "shall" not in s["counts"]:
                continue
            q = s["text"]
            if s["continues_in_items"]:
                q += " " + " ".join(s["continues_in_items"])
            if s["continues_in_next_block"]:
                q += " [the sentence continues in the following paragraph(s) of the PDF: " + s["continues_in_next_block"] + "]"
            if s["lead_in"]:
                q += " [lead-in paragraph: " + s["lead_in"] + "]"
            clause = s["clause"] if s["clause"] else "unattributed"
            pg = "p. %s (PDF p.%d)" % (s["page"] or "no printed label", s["pdf_page"])
            if s["page_end"] or s["pdf_page_end"]:
                pg += ", continues to PDF p.%d" % s["pdf_page_end"]
            L.append("| %s | %s | %s | \"%s\" | %s | Unchecked |  |  |" % (
                s["id"], b["id"], clause, md_cell(q), pg))
    L.append("")
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(L))


# --------------------------------------------------------------------------- page
def embed(obj):
    """JSON safe to embed in a <script type="application/json"> element."""
    t = json.dumps(obj, ensure_ascii=True, separators=(",", ":"))
    return t.replace("</", "<\\/").replace("<!--", "<\\!--")


def write_page(out_dir):
    d = os.path.join(out_dir, "data")
    with open(os.path.join(HERE, "navigator_template.html"), "r", encoding="utf-8") as fh:
        tpl = fh.read()
    for name, ph in (("standard", "__STANDARD_JSON__"), ("implementation", "__IMPLEMENTATION_JSON__"),
                     ("evidence", "__EVIDENCE_JSON__")):
        with open(os.path.join(d, name + ".json"), "r", encoding="utf-8") as fh:
            tpl = tpl.replace(ph, embed(json.load(fh)))
    with open(os.path.join(out_dir, "index.html"), "w", encoding="utf-8", newline="\n") as fh:
        fh.write(tpl)


# --------------------------------------------------------------------------- main
def dump_json(path, obj):
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(obj, fh, ensure_ascii=False, separators=(",", ":"))
        fh.write("\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo-root", default=os.path.abspath(os.path.join(HERE, "..", "..", "..", "..")))
    ap.add_argument("--pdf-dir", default=os.path.abspath(os.path.join(HERE, "..")))
    ap.add_argument("--out-dir", default=HERE)
    ap.add_argument("--matrix", default=None, help="SHALL_MATRIX.md path (default <repo>/starcom/docs/SHALL_MATRIX.md)")
    ap.add_argument("--skip-impl", action="store_true", help="do not (re)scan the repo for code citations")
    ap.add_argument("--include-docs", action="store_true", help="also scan .md/.txt/.rst for citations")
    ap.add_argument("--no-html", action="store_true", help="write data + SHALL_MATRIX.md but not index.html")
    ap.add_argument("--html-only", action="store_true",
                    help="only rebuild index.html from the existing data/*.json (use after editing evidence.json)")
    a = ap.parse_args()
    data_dir = os.path.join(a.out_dir, "data")
    os.makedirs(data_dir, exist_ok=True)
    if a.html_only:
        write_page(a.out_dir)
        return

    books, parses = [], {}
    for bid, fname in BOOKS:
        book, bp = assemble_book(bid, os.path.join(a.pdf_dir, fname))
        books.append(book)
        parses[bid] = bp
    standard = {"generator": "build_navigator.py", "books": books,
                "note": "Only text printed in the CCSDS books. does_not_apply_when / option_tag are null: filled by people."}
    dump_json(os.path.join(data_dir, "standard.json"), standard)

    nodes_by_book = {b["id"]: {c["key"] for c in b["clauses"]} for b in books}
    impl_path = os.path.join(data_dir, "implementation.json")
    if not a.skip_impl:
        cites, no_target, st = scan_implementation(a.repo_root, nodes_by_book, a.include_docs)
        dump_json(impl_path, {"scan_roots": SCAN_ROOTS, "skipped_dirs": sorted(SKIP_DIRS), "stats": st,
                              "citations": cites, "mentions_without_clause": no_target})
    elif not os.path.exists(impl_path):
        dump_json(impl_path, {"scan_roots": SCAN_ROOTS, "skipped_dirs": sorted(SKIP_DIRS), "stats": {},
                              "citations": [], "mentions_without_clause": []})
    ev_path = os.path.join(data_dir, "evidence.json")
    if not os.path.exists(ev_path):
        with open(ev_path, "w", encoding="utf-8", newline="\n") as fh:
            fh.write("{}\n")
    matrix = a.matrix or os.path.join(a.repo_root, "starcom", "docs", "SHALL_MATRIX.md")
    write_matrix(matrix, standard)
    if not a.no_html:
        write_page(a.out_dir)
    # summary for the log
    for b in books:
        sh = [s for s in b["sentences"] if "shall" in s["counts"]]
        occ = sum(s["counts"]["shall"] for s in sh)
        print("%s: clauses=%d shall-sentences=%d shall-words=%d (Duke raw %d) pics=%d xrefs=%d refs=%d" % (
            b["id"], len(b["clauses"]), len(sh), occ, DUKE_RAW[b["id"]], len(b["pics"]), len(b["crossrefs"]),
            len(b["references"])))


if __name__ == "__main__":
    main()
