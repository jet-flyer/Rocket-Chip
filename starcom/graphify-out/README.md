# Starcom knowledge graph

Dedicated map of `starcom/` + `standards/starcom/`. The repo-root graph ignores this tree (`.graphifyignore` has `starcom/` and `**/*.pdf`), so SAD/ICD queries against `graphify-out/graph.json` miss it.

Host semantic pass 2026-08-27 (no Gemini key).

```
graphify query "How does decode_pltu find the CRC?" --graph starcom/graphify-out/graph.json
graphify path "PLTU" "211.2 Annex C CRC-32" --graph starcom/graphify-out/graph.json
```

Open `graph.html` for the interactive view. `GRAPH_REPORT.md` is the audit.

Rebuild is a sitting, not the post-commit hook. Intermediates (AST dumps, extract JSON) stay untracked.
