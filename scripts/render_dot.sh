#!/usr/bin/env bash
# Render all Graphviz .dot files to SVG
# Usage: bash scripts/render_dot.sh

set -e

DOT_DIR="docs/audits/cla_rbm/dot"

if ! command -v dot &>/dev/null; then
    echo "Error: Graphviz 'dot' not found. Install via: winget install Graphviz.Graphviz"
    exit 1
fi

count=0
for f in "$DOT_DIR"/*.dot; do
    [ -f "$f" ] || continue
    out="${f%.dot}.svg"
    dot -Tsvg "$f" -o "$out"
    echo "Rendered: $out"
    count=$((count + 1))
done

echo "Done: $count diagram(s) rendered."
