# Local LLM Companion — Research Collection

**Purpose:** Evaluate self-hosted (local) language models as a *companion* to frontier cloud
models for RocketChip and Starcom work — not as a replacement. The target role is
straightforward, high-volume, verifiable work (file operations, applying already-decided code
changes, audit remediation execution) where the thinking has already been done by a cloud model
or by the repo owner.

**Status:** Research only. No code, no tooling, no procurement decisions committed. Nothing here
is an adopted standard or an approved workstream.

---

## How this document works (read before editing)

This is a **multi-agent collection point**. Claude, Grok, Gemini and others are expected to add
research here over time.

1. **Append-only by default.** Add a new dated, attributed section. Do not restructure the
   document or re-order existing sections.
2. **Never edit another agent's section.** If you disagree with a finding, add your own section
   that cites and rebuts it — do not silently amend the original. This is the
   `CROSS_AGENT_REVIEW.md` protocol applied to a research doc.
3. **You may edit your own prior sections** (correct, extend, retract) — mark retractions
   explicitly rather than deleting, so the reasoning trail survives.
4. **Attribute every section** with `Brand Model (Harness)` + date, matching the CHANGELOG
   author convention (`docs/agents/SESSION_CHECKLIST.md`, CHANGELOG header).
5. **Label your sourcing.** Per LL Entry 37/38 discipline: mark each material claim as
   primary-source-verified, secondary, or unverified. Do not let a plausible secondary summary
   pass as authoritative — see the Unverified Claims Log at the bottom, which already contains
   one likely-fabricated model.

---

## Section 1 — Initial scoping and model landscape

**Author:** Claude Opus 5 (Code) · **Date:** 2026-07-28

### 1.1 Context and hardware baseline

The repo owner installed **Odysseus** (open-source self-hosted AI workspace, AGPL-3.0; supports
Ollama / vLLM / llama.cpp / OpenRouter backends) and asked which model best fits the companion
role, given an Nvidia GPU.

Hardware, **measured on the machine this session** (not assumed):

| Component | Value |
|---|---|
| GPU | NVIDIA GeForce RTX 4070 Ti — **12 GB VRAM**, Ada, compute capability 8.9 |
| System RAM | **61.6 GB**, DDR5-**6000**, 2×32 GB dual channel (~70–85 GB/s effective) |
| Driver | 610.74 |

**The controlling insight:** 12 GB of VRAM is restrictive, but 62 GB of fast DDR5 is not. That
combination favors **Mixture-of-Experts (MoE) models with expert tensors offloaded to system
RAM**. An MoE model only reads its *active* parameters per token, so streaming experts over DDR5
costs far less than it would for a dense model, where every parameter is read every token.

Practical consequence: **use the llama.cpp backend, not vLLM.** vLLM effectively requires the
model to fit in VRAM; llama.cpp does proper hybrid CPU/GPU expert offload (`--n-cpu-moe`, tuned
until VRAM use approaches ~11 GB). Odysseus's hardware-aware recommender optimizes for
"fits in VRAM" and will likely steer toward small dense models, missing this entirely.

### 1.2 Model landscape (primary-source verified)

All figures below are from Hugging Face model cards or vendor documentation, not roundup articles.

| | Qwen3.6-35B-A3B | Qwen3-Coder-Next | Gemma 4 26B A4B |
|---|---|---|---|
| Released | 2026-04-16 | 2026-02-03 | 2026-04-02 |
| Total / active params | 35B / 3B | 80B / 3B | 26B / 3.8B |
| SWE-bench Verified | **73.4** | 70.6 | — |
| SWE-bench Pro | **49.5** | 44.3 | — |
| Terminal-Bench 2.0 | **51.5** | 36.2 | — |
| Context | 262,144 (ext. ~1M) | 262,144 | 262,144 |
| License | Apache 2.0 | Apache 2.0 | Apache 2.0 |
| Q4_K_M size | 22.1 GB | ~46 GB | ~15 GB (QAT Q4_0) |

**Finding: bigger is not better here.** `Qwen3-Coder-Next` is 2.3× the total parameters of
`Qwen3.6-35B-A3B` and loses on every agentic benchmark — by 15 points on Terminal-Bench 2.0,
which is the benchmark closest to "execute a decided change in a real shell." Spending the extra
24 GB of RAM buys a worse model *and* a speed penalty.

**Therefore the quality dial on this hardware is quantization, not parameter count.** Available
quants for Qwen3.6-35B-A3B: Q4_K_M 22.1 GB · UD-Q4_K_XL 22.4 GB · Q5_K_M 26.5 GB · Q6_K 29.3 GB.
Q6_K still leaves ~32 GB of RAM headroom. Expected throughput with expert offload on this box:
roughly 20–30 tok/s (memory-bandwidth bound; the DDR5-6000 is why it lands there and not at 10).

Configuration notes for Qwen3.6-35B-A3B, from the model card:
- **Thinking mode is ON by default** and emits `<think>` blocks. For the executor role this is
  waste — set `"enable_thinking": false`. Non-thinking sampling params: `temperature=0.7,
  top_p=0.80, top_k=20, presence_penalty=1.5, repetition_penalty=1.0`. For pure edit application,
  drop temperature further (~0–0.2); the card's defaults are tuned for general use.
- **Tool-call parser is `qwen3_coder`.** Set it explicitly. A wrong or approximated tool-call
  template is the most common cause of a model that looks fine in chat but silently emits
  malformed tool arguments.
- The model is **multimodal with a vision encoder**, which is dead weight for this role. vLLM
  exposes `--language-model-only` to skip it and reclaim memory for KV cache.

Out of reach locally, noted for completeness: **Kimi K3** (2.8T total, 2026-07-16), **GLM-5.2**
(744B/40B, 2026-06-16), **MiniMax M3** (2026-06-01). All multi-H200 class.

### 1.3 What quantization-aware training (QAT) means

Relevant because Gemma 4 26B A4B ships Google-authored QAT checkpoints, which is a genuine
efficiency advantage on a 12 GB card.

**Post-training quantization (PTQ)** — the normal path — trains in bf16, then afterward rounds
every weight to a coarse 4-bit grid. The model was never told this would happen, so the rounding
error is uncompensated damage.

**Quantization-aware training (QAT)** simulates the rounding *inside the forward pass* during the
final training phase, while letting gradients flow through as if it were continuous (a
straight-through estimator). The optimizer can therefore see the rounding error and compensate —
nudging weights toward positions where rounding costs less, and adapting the rest of the network
to absorb the residual. The result is a model trained to be good *as a 4-bit model*, rather than
a bf16 model damaged on export.

Google's published figure: for Q4_0, QAT **reduces the perplexity drop by 54%** vs PTQ
(llama.cpp perplexity evaluation).

Two caveats that matter operationally:

1. **The benefit is welded to the format it was trained against (Q4_0).** This produces a
   counterintuitive result — Q4_0 is an older, cruder llama.cpp format normally *worse* than
   Q4_K_M, but QAT-Q4_0 can beat PTQ-Q4_K_M. The trap: requantizing the QAT checkpoint upward to
   Q5_K_M or Q6_K discards most of the advantage and lands *worse* than a plain PTQ quant of the
   same size. Use Google's own `gemma-4-26B-A4B-it-qat-q4_0-gguf`; do not upconvert it.
2. **QAT moves the quality-per-GB frontier; it does not add capability.** QAT-Q4 ≈ bf16 for the
   same model. It does not make a 26B competitive with a much larger one.

Gemma 4 also has **native function calling** trained in (unlike Gemma 3, which lacked a tool
template). Caution: its tool-call format uses nonstandard delimiters (`<|tool_call>` /
`<tool_call|>`, escaped strings as `<|"|>`) and there are open parser bugs across backends
(mlx-lm #1096: `tool_calls` silently empty; open-webui #23863: needs a mode toggle). Verify tool
calls actually parse in Odysseus before trusting it with a repo.

### 1.4 The governing principle — verifiability, not difficulty

The most useful result of this session. The right question for delegating work to a weaker model
is **not** "is this task easy?" but **"can a machine check the output more cheaply than doing the
task?"**

- **Generative tasks** (apply this fix) fail by producing a *wrong artifact*. It compiles or it
  doesn't; tests pass or they don't; there is a diff to review. Every failure mode leaves
  something for a checkpoint to catch.
- **Search / recall tasks** (find all violations of this rule) fail by producing *nothing* — the
  false negative. There is no artifact to review, and discovering what was missed costs about as
  much as doing the audit again.

This is why "the local model audits, the frontier model checks at checkpoints" is structurally
weaker than it looks: the checkpoint works beautifully for the fixes and does not naturally work
at all for the audit.

Two aggravating factors on this hardware: practical context is ~32–64K, so large files get
chunked (silently killing any rule whose violation spans a chunk boundary), and rules that look
black-and-white stop being so somewhere around file 40 — the edge cases are where an audit earns
its keep, and edge-case judgment is where a 3B-active model is weakest.

**The fix — invert the error economics.** Do not ask a local model to audit. Ask it to be a
**high-recall, low-precision candidate generator**, explicitly instructed that false positives
are fine and expected. A frontier model then adjudicates the candidate list, which is cheap
(rule + file + line + snippet in, yes/no out, no repo-wide reading). This puts the *cheap* error
where the weak model is.

This maps directly onto the existing `RULE_VERIFIABILITY_TRIAGE.md` taxonomy — which turns out to
be a routing table for *models*, not just for rules.

### 1.5 Where a local model fits in the existing RocketChip pipeline

Framed against the project's own formulation of why tools fail — **"tool sees the *what*, blind
to the *why*"**. A small local model is blind to the *why* in the same way; it just fails more
fluently. So it belongs wherever the answer **already exists in writing somewhere in the repo**
and the work is finding, matching, or transcribing it — never where the answer must be judged
into existence. `REFERENCE-BEFORE-JUDGMENT` (triage doc, 2026-06-12) is precisely the filter that
separates those two.

**Good fits, roughly in priority order:**

1. **Det-by-reference resolution at scale.** Given a candidate site plus the resolving artifacts
   (deviation register, exemption sets, LL entries, precedence chain, platform constraints),
   answer "is this already decided elsewhere?" Bounded input, an answer that provably exists in
   text, verifiable by citation. LL Entry 1 (mandated `static` large objects) is the canonical
   case — a scope-checker flags it, and resolving it is a *lookup*, not a judgment.
2. **The §CM conversion artifacts — specifically `bugprone-unused-return-value.CheckedFunctions`.**
   Enumerating every project function whose return conveys error information across the 186-file
   itinerary is mechanical enumeration with a config file as output, verified by clang-tidy
   actually running. Also the highest bite×population item in the triage (Rule 7 / the LL28/31/41
   dropped-bus-return class). *(Note: per the 2026-06-24 handoff, much of the P10-7 canary was
   subsequently closed via `[[nodiscard]]` contracts with `CheckedFunctions` scoped to
   `flash_safe_execute` — verify current state before treating this as open work.)*
3. **Triaging a diagnostic burst.** After flipping a gate (e.g. the measured 56 `-Wconversion` /
   `-Wsign-conversion` findings), the resulting wave is bounded, enumerated, high-volume, and
   sample-verifiable. This is the ideal "read instrumentation" job.
4. **NOLINT / deviation rationale drafting** for confirmed false positives — permanent, reviewable
   in-repo justification written while the reasoning is fresh. Serves the Grey→Det-by-reference
   conversion move directly.
5. **Cross-artifact drift detection.** Comparing what a doc *claims* against what the config or
   repo *is*. The project has a documented history of exactly this failure class (CHANGELOG
   missing for `5a6cf87`; a WB temp-record overwritten; itinerary membership 125-vs-186; citation
   counts wrong — JPL is 31 not 102, JSF 233 not 221). Given the severity model puts over-claiming
   at critical, a standing pass here has real value.

**Must not be used for — each grounded in a burn already taken:**

- **As a source of rule text or rule numbers.** The record already contains a WebFetch summarizer
  fabricating P10 Rule 1 as single-exit (false), and JSF ranges entered from memory that needed
  flagging against non-sequential JSF numbering. A local model does this *more* often and *more*
  fluently. Verbatim text comes from pypdf over the real PDFs; the model never supplies a citation.
- **The Manual bucket.** Assertion meaningfulness, the `shared_state.h` cross-core
  plain-bool-vs-atomic question, comments that restate a referenced doc. All *why*. Delegating
  these does not merely fail — it produces confident dispositions that *look like coverage*.
- **Provenance / precedence judgments.** The CCG-is-a-reference-not-an-adopted-source re-anchoring
  is exactly the subtle governance distinction a small model flattens.
- **Graph parity.** Already deterministic in `graphify_curate.py` / `graphify_verify.py`, and the
  equal≠parity lesson (the 1-of-370 bridge) demands content checks, not counts.
- **The pedagogical/explanatory layer.** Local models are worst at grounded explanation with
  primary sourcing, which is the property the deliverables most need.

**Validation design.** Two different mechanisms are required, because the two stages fail
differently:
- *Generation stage* → **salt the corpus with known violations** and measure per-rule recall.
  Gives a real number per rule instead of a hope, and tells you which rules to route where.
- *Triage stage* → **salt with known false positives.** The failure mode here is the inverse:
  the model defers to the tool and rubber-stamps "true positive" because clang-tidy said so.
  clang-tidy false positives are frequently precisely the cases needing domain knowledge the model
  lacks (a cast safe because of an invariant three files away; a raw loop deliberate for a wire
  format; a "magic number" that is a spec-mandated field width). If it flags 100% of known
  false-positives as real, it is deferring rather than reading — and that is invisible from
  reviewing its output alone, because each judgment looks reasonable in isolation.
- Force a **three-way** classification (true / false / uncertain) with escalation framed as cheap
  and correct. Watch the escalation rate as a health metric: ~2% means overconfidence, ~60% means
  it isn't earning its keep.
- Constrain output at the decoder. llama.cpp supports **GBNF grammars**, making schema violation
  impossible rather than merely unlikely.

**Also worth doing first:** push every rule that *can* be mechanized down to clang-tidy,
clang-query / AST matchers, cppcheck, or ripgrep. Each one so converted gets 100% recall, zero
tokens, and CI reproducibility — strictly better than any model on any hardware. This is the
triage's own "conversion move," and it is arguably the local model's best use: **executing the
conversion at volume**, so that neither model ever has to judge that rule again.

**Renames specifically:** LL Entry 44 already settled this — mass identifier renaming uses
`clang-refactor local-rename` (AST), because `clang-tidy --fix` and text/`sed` each shipped a
distinct class of subtle bug. Nothing here revises that. A local model's role in a rename is the
residue an AST tool cannot see: strings, log messages, comments, docs, config keys, filenames,
serialized formats.

### 1.6 Broader-ecosystem uses not specific to the audit pipeline

The framing error worth naming: local models are not simply *cheaper* cloud models. The real
difference is that **zero marginal cost enables workflows that would never be run against a
metered API** — things that run on every commit, every file save, every build, continuously.

1. **FIM (fill-in-the-middle) autocomplete** — the single most common real-world local-model use,
   and a *different model slot*: a specialized 1–3B FIM-trained model, not the 35B. Sub-second,
   no network round trip; thinking models are disqualified because latency is the product.
2. **Local embeddings + rerankers as infrastructure.** `Qwen3-Embedding-0.6B` (~1.5 GB, Apache
   2.0, 70.7 MTEB, covers programming languages); `BGE-M3` + `BGE-reranker-v2-m3` is the standard
   production pairing; `Qwen3-Reranker-0.6B` is instruction-aware, letting relevance be *defined*
   per query. Directly relevant to the graphify semantic cache and the 370 doc→code bridges.
   These are a different *function* (they emit vectors, not text) — not a smaller LLM competing
   for the same budget, and they run acceptably on CPU, preserving VRAM.
3. **Datasheet RAG.** RP2350, SX1276, Pico SDK docs, CCSDS Blue Books. A documented pattern in
   embedded practice specifically. See the AGENT_WHITEBOARD Research/Deferred row added
   2026-07-28 for the caveats.
4. **Continuous background review on hooks that already exist.** Tooling exists (`roborev` for
   per-commit background review with a daemon; `llm-code-review` as an Ollama-backed pre-commit
   hook). The project already runs `post-commit` with graphify curate/verify. A local reviewer is
   economically trivial there in a way a cloud call is not.
5. **The toolchain's own structured output** — an entire family currently unread because reading
   it is tedious: `.map` files and linker symbol tables; `-fstack-usage` reports (stack depth is
   safety-relevant in flight code and nobody audits it); `arm-none-eabi-size` deltas per commit
   (silent flash/RAM creep); serial/RTT trace and log streams off the bench; C++ template error
   walls and CMake failures. Bounded, repetitive, cloud-expensive purely by volume.
6. **Spec-table → code with the compiler as verifier.** CCSDS Blue Book bit layouts and RP2350 /
   SX1276 register bitfields → C++ structs + `static_assert`. Transcription from a table is what
   small models are good at, and the output is machine-checkable — the property that makes
   delegation safe. **Bounded claim:** `static_assert` proves *layout*, not *semantics*. See the
   whiteboard row for the full caveat.

### 1.7 Open questions — carried to the next session

1. **Quantization as a choice dial.** Repo owner wants to work through what quant levels actually
   mean for capability vs. footprint, against the model-budget question. Not yet discussed in
   depth.
2. **"Big + surgical small" vs. one large model.** Owner's stated preference: a single more
   capable model over many small ones, *unless* shakedowns show diminishing returns; big-plus-
   surgical-small is appealing "if practical." Practicality is unresolved and is a shakedown
   question, not a spec-sheet question. Note that item 2 in §1.6 (embeddings) is a different
   function rather than a competing model, so it does not sit on the same tradeoff axis; FIM
   autocomplete genuinely does.
3. **Shakedown design.** No benchmark harness has been designed. The salted-corpus approach in
   §1.5 is the proposed validation method and the owner has confirmed it will be part of it.
4. **Odysseus specifics.** Backend selection, offload flags, and model-slot configuration have not
   been inspected on the actual install.

### 1.8 Unverified claims log

Recorded per LL Entry 37/38 discipline, so a later reader does not re-import them.

- **"DeepSeek-Coder V3 (Distilled), 16B dense, 40.5% SWE-bench Verified, runs in 12 GB."**
  Appears as the *top recommendation* in several "best local LLM 2026" roundups (InsiderLLM and
  others). **Could not be traced to any DeepSeek primary source** — no model card, no release
  announcement, no repository. Assessed as likely SEO-generated. Do not go looking for these
  weights on the strength of those articles.
- **"Qwen3-Coder-Next is a dense model with all 79.7B parameters active."** Claimed in a Medium
  comparison article; **contradicted by the Hugging Face model card**, which states 80B total /
  3B active MoE. The card governs.
- Throughput figures (~20–30 tok/s) in §1.2 are **estimates** derived from active-parameter count
  and measured memory bandwidth. Not benchmarked on this machine.

### 1.9 Sources

Primary (model cards / vendor documentation):
- https://huggingface.co/Qwen/Qwen3.6-35B-A3B
- https://huggingface.co/unsloth/Qwen3.6-35B-A3B-GGUF (quantization sizes)
- https://huggingface.co/Qwen/Qwen3-Coder-Next
- https://unsloth.ai/docs/models/qwen3-coder-next (local-run guidance)
- https://ai.google.dev/gemma/docs/core/model_card_4
- https://huggingface.co/google/gemma-4-26B-A4B-it-qat-q4_0-gguf
- https://ai.google.dev/gemma/docs/capabilities/text/function-calling-gemma4
- https://blog.google/innovation-and-ai/technology/developers-tools/quantization-aware-training-gemma-4/
- https://developers.googleblog.com/en/gemma-3-quantized-aware-trained-state-of-the-art-ai-to-consumer-gpus/

Secondary (ecosystem / tooling survey — treat as pointers, not authority):
- https://openrouter.ai/blog/insights/the-open-weight-models-that-matter-june-2026/
- https://d-central.tech/local-embedding-models/
- https://zeroentropy.dev/articles/open-source-alternatives-to-cohere-rerank/
- https://resources.altium.com/p/building-local-llm-datasheet-extractor-ic-driver-development
- https://github.com/kenn-io/roborev
- https://github.com/massimilianoviola/llm-code-review
- https://modelfit.io/tools/continue-dev/
- https://explore.n1n.ai/blog/odysseus-self-hosted-ai-workspace-llm-tools-2026-06-08

---

<!-- NEXT AGENT: append your section below this line. Do not edit sections above. -->

---

## Section 2 — Odysseus install reality + hybrid workflow + RocketChip fit (session handoff)

**Author:** Grok 4.5 (Build CLI) · **Date:** 2026-07-28  
**Role of this section:** Handover from the Odysseus bring-up / architecture discussion session. Complements §1 (model landscape + verifiability principle). Does **not** re-pick a primary companion model — that is the explicit next-session topic with the owner.

### 2.1 What was actually measured / fixed on the Odysseus host (primary, this machine)

Install path: `C:\Apps\odysseus`. Compose project name `odysseus`.

| Observation | Detail | Confidence |
|---|---|---|
| Host GPU | RTX 4070 Ti, driver 610.74, `nvidia-smi` OK | Primary |
| Host RAM | ~61.6 GB physical | Primary |
| Docker GPU | `docker run --rm --gpus all nvidia/cuda:… nvidia-smi` sees the 4070 Ti; runtime `nvidia` present | Primary |
| Main app image | Was **missing** when helpers (chromadb/searxng/ntfy) were already up — UI dead on :7000 until `docker compose up -d --build odysseus` | Primary |
| GPU inside app container (before fix) | No DeviceRequests; scan could not see GPU | Primary |
| GPU after NVIDIA overlay | `DeviceRequests` nvidia + `nvidia-smi -L` inside container → RTX 4070 Ti | Primary |
| `.env` COMPOSE_FILE (Windows) | Colon form `yml:docker/gpu…` is treated as a **single broken path** on Windows Docker CLI. Working form: `COMPOSE_FILE=docker-compose.yml;docker/gpu.nvidia.yml` | Primary |
| RAM inside container / Cookbook scan | `/proc/meminfo` **~30 GiB**, not 61 GB — Docker Desktop/WSL2 VM budget (no `.wslconfig` on host). Scan is honest about the container, not broken | Primary |
| Ollama | Host process listening; **zero models pulled** at check time | Primary |
| No built-in chat brain | Odysseus is a workspace; welcome copy “pick a model or just type” does **not** imply a free default model — send without a model gets a “no session / pick model” path | Primary (code + behavior) |

**Operational notes for next session:**

1. Closing the Docker Desktop **window** is fine; **Quit** Docker kills the stack and the UI.
2. After reboot: wait for Docker healthy, then `cd C:\Apps\odysseus; docker compose up -d` (overlay now in `.env`).
3. Optional: raise WSL RAM via `%UserProfile%\.wslconfig` (e.g. `memory=48GB`) + `wsl --shutdown` + Docker restart if Cookbook should rank against more host RAM. Manual HW override in Cookbook also exists for “what if” ranking without giving the container more memory.
4. Host **Ollama** sees full host RAM/GPU; Docker Odysseus talking to `http://host.docker.internal:11434/v1` is the usual Windows path. Cookbook **Serve** inside the container is a separate GPU-engine path (see §2.3).

### 2.2 MCP is not “cloud-only”

**Finding:** MCP (Model Context Protocol) is a **tool/plugin transport standard** (local stdio servers and remote servers). Local hosts (Claude Desktop, Cursor, VS Code, Odysseus, custom agents) can attach local MCP servers for filesystem, git, browser, shell, or a **local worker wrapped as a tool**. Cloud hosts can call the same local servers when the desktop/CLI bridge allows it.

**Implication for companion design:** Hybrid cloud↔local does not require inventing a private RPC. First glue is often **CLI + JSON work orders + verify commands**; MCP is the cleaner long-term “USB-C” for the same tools. Odysseus already registers built-in MCP servers (RAG, memory, browser, etc.) — pattern is native to that stack.

Sourcing: secondary (protocol docs / ecosystem); no RocketChip MCP server implemented this session.

### 2.3 Inference engines vs “chat quality” (what the words mean)

For the companion role, these are **serving layers**, not different personalities:

| Tool | Role | When it matters here |
|---|---|---|
| **Ollama** | Easy local OpenAI-compatible server on Windows | Fastest path to *try* a model with Odysseus |
| **llama.cpp / Cookbook Serve** | GGUF + hybrid CPU/GPU offload | Aligns with §1 MoE-offload story on 12 GB VRAM + lots of RAM |
| **vLLM** | High-throughput multi-request server | Many concurrent agent steps; often wants model mostly in VRAM — weaker fit for large MoE expert-offload than llama.cpp (§1.1) |
| **TensorRT-LLM** | NVIDIA-compiled engines, max tok/s for a **fixed** model | Later, after model choice is locked |
| **NIM** | NVIDIA packaged microservices (model + optimized stack) | Lowest DIY for NVIDIA-curated deploys; heavier product surface |

**Slim Odysseus image note (not a bug):** GPU **passthrough** (`nvidia-smi` in container) ≠ CUDA **serve engine** installed. Cookbook → Dependencies installs CUDA-capable llama.cpp/vLLM-class engines into persisted `./data/local`. Passthrough was fixed this session; engines still need install when using in-container Serve.

### 2.4 Nemotron / “NVIDIA-specialized” models (secondary / community)

Not “secret CUDA unlocks.” Distinction:

- **Most open models** use Tensor Cores via a CUDA backend (Ollama/llama.cpp/vLLM kernels) regardless of brand.
- **Nemotron** (esp. Nemotron 3 family) is NVIDIA’s open line oriented toward **agent throughput, hybrid MoE efficiency, long context** — architecture/runtime packaging, not exclusive Tensor Core access.
- **Community pattern (LocalLLM / hands-on, secondary):** often **very fast** prompt processing / tok/s; many hobbyists still prefer **Qwen / Gemma** class for coding/daily “smarts” at similar practical VRAM. Super/Ultra-class sizes are not 12 GB desktop targets without heavy quant/offload and careful choice of Nano-class variants.

Does **not** overturn §1’s primary-card ranking of Qwen3.6-35B-A3B / Gemma QAT for the executor role; Nemotron is optional A/B if agent throughput is the experiment.

### 2.5 Hybrid cloud + local: mainstream architecture, not a white rabbit

Cross-check against industry/research practice (secondary survey this session):

- **Routing:** one model per task by complexity/sensitivity.  
- **Cascading:** cheap first, escalate if quality low — or reverse (smart plan → cheap execute).  
- **Hybrid cloud–local guides** (2026): route by privacy, complexity, availability.  
- **Coding agents:** cloud as lead / local as implementer; Claude Code or Codex **harness** pointed at Ollama; multi-model local portfolios with frontier fallback.  
- **Research:** hybrid LLM routing papers report large reductions in expensive API calls when easy work stays local.

**Composable, not one product button.** “Grok plans audit; local agent applies work orders” is the same pattern with RocketChip-shaped contracts.

### 2.6 RocketChip-specific: where this workflow is *best* (and not)

Maps §1.4–1.5 verifiability principle onto **actual scars and process** in this repo.

#### Best fit (high ROI companion / local worker)

1. **Audit → approved work orders → apply + verify**  
   Code trimming, dead-code inventories, standards remediations that are already decided. Cloud/owner: triage. Local: edit + `ctest` / scoped build. Matches CODE_TRIMMING-style “identify now, implement later with checklist.”

2. **Cross-artifact consistency after a decision**  
   Symbol-removal walks (callees + protected-doc refs — SESSION_CHECKLIST), stale log-token vs `bench_sim` regex (LL Entry 36 class), CHANGELOG/doc mention sweeps when the fact change is already approved.

3. **Rename *residue* only**  
   LL Entry 44: mass identifier renames = **`clang-refactor local-rename` (AST)**, not sed/LLM. Local model: comments, docs, log strings, configs after AST tool. Never freeform bulk rename.

4. **Diagnostic bursts / mechanical enumeration**  
   clang-tidy waves, conversion findings, “list every CheckedFunctions candidate” with config as output — §1.5 items remain correct.

5. **Dual build / matrix smoke after bulk edits**  
   Station vs vehicle compile parity when paths demand it — scripted, high volume.

6. **Lab procedure runner (HW-adjacent software)**  
   See §2.7.

#### Medium fit (strict contract only)

- Small bugfixes with failing test + file list + no drive-by.  
- RC_OS table fill-ins after cloud designs the table (not freeform “clean the CLI”).  
- Host tests for specified behavior.

#### Poor fit (keep cloud + human + HW gate)

- ESKF / fusion / pyro / timing / “pick a number” without research (CODING_STANDARDS prior-art rule).  
- Architecture / council / Stage 17 / IVP design.  
- Ambiguous mass edits (exactly what broke under partial clang-tidy / sed renames).  
- Root-cause of intermittent electrical faults (IVP-140 cable class).

### 2.7 Hardware validation, soaks, benchmarks — what is automatable

Repo already encodes the right split in `standards/HW_GATE_DISCIPLINE.md` and VERIFICATION_OVERVIEW.

| Layer | Agent fit | Notes |
|---|---|---|
| **A. Procedure runner** | Excellent | OpenOCD, flash exact artifact, 3-boot protocol, run `bench_sim` / soak / capture serial |
| **B. Evidence compare** | Excellent | Assert **named positive-control signals** (RegVersion=0x12, DAC ACK, GPS PMTK, FD pyro line…); check binary banner/hash; record flash method (probe vs picotool — LL scar) |
| **C. Physical / causal judgment** | Poor alone | Cables, probe exclusivity, “code vs bus,” first bring-up |

**Datasheets + HARDWARE.md + IVP text** make **A+B** checklist-complete enough for solid progress (lab tech with a work order). They do **not** make unsupervised “fix the board until green” safe.

**Recommended work-order shape for HW gates (next session design, not implemented):**

```text
binary path | flash method (probe) | script | required regexes/signals |
boot_count=3 | report path for commit citation
```

**Benchmarks:** host mat/UD and on-target scrapers → local collects numbers vs `docs/benchmarks/*`; human/cloud owns “ship for flight.” Dynamic motion validation stays human-led.

**Anti-pattern:** soft gates (“soak stable,” “no crash”) without positive control — project already forbids treating those as sole evidence.

### 2.8 Cookbook “score” and how to measure companion improvement

**Score (from Odysseus `services/hwfit/fit.py`, primary read of install):** composite  
`quality×w + speed×w + fit×w + context×w` with use-case weights (e.g. coding vs general). Quality is **heuristic** (size bands + name/arch bonuses + quant penalty), not a live SWE-bench run on this machine. Use other sort columns / sub-scores when the single number fights the job (savant worker vs chat).

**Measuring “LLM + tools” improvement for RocketChip (recommended harness, not built):**

- Task suite: work orders from real audits (renames residue, doc sync, checklist fixes).  
- Metrics: success rate, verify pass (`ctest` / `rg` / dual build), human interventions, wall time, cloud tokens, retries.  
- **Baselines:** (1) script/codemod only for pure renames; (2) cloud-only; (3) cloud plan + local apply.  
- Salted recall (§1.5) for generation; salted false-positives for triage rubber-stamping.

Public chat Elo is a weak proxy for this repo’s companion role.

### 2.9 Open questions for next session (owner + agent)

Carry forward §1.7 and add:

1. **Primary companion model + quant for this box** — deep-dive with owner (Qwen3.6-35B-A3B vs Gemma QAT vs others; enable_thinking off; tool parser). §1 is the starting landscape; no procurement this session.  
2. **Backend choice on real Odysseus install:** Ollama host vs Cookbook llama.cpp Serve vs both; confirm expert-offload flags if using llama.cpp path.  
3. **`.wslconfig` RAM raise** — owner decision (Cookbook ranking vs host Ollama-only).  
4. **Work-order schema v0** for audit apply + optional HW gate runner (JSON fields, verify commands, escalate rules).  
5. **Nemotron Nano-class A/B** only if throughput experiment is wanted; not required to start.

### 2.10 Sources / confidence

| Claim class | Sourcing |
|---|---|
| Docker/GPU/RAM measurements, COMPOSE_FILE Windows behavior | Primary on owner machine 2026-07-27/28 |
| Odysseus no default model / hwfit score structure | Primary (container behavior + `fit.py` / `hardware.py` on install) |
| MCP local/remote; hybrid routing/cascade industry | Secondary (protocol docs, 2026 hybrid/routing literature and agent ecosystem) |
| Nemotron user sentiment | Secondary (community / reviews) — do not treat as benchmark gospel |
| RocketChip fit / HW gate rules | Primary repo docs (`HW_GATE_DISCIPLINE`, LESSONS_LEARNED, CODE_TRIMMING, SESSION_CHECKLIST) |

### 2.11 Handover checklist (for the next agent / session)

Use this section + §1 as the handoff package. Do not re-discover Odysseus GPU from zero.

- [ ] Read §1 (verifiability principle, model cards, must-not-use list) and this §2.  
- [ ] Confirm Docker Desktop up; `docker compose -f C:\Apps\odysseus\… ps` shows `odysseus` + helpers; `docker compose exec odysseus nvidia-smi -L` if GPU work.  
- [ ] Owner agenda: **model + quant selection** first; optional WSL RAM; optional work-order schema.  
- [ ] Research-only until owner adopts tooling — no silent standards changes.  
- [ ] Append further findings as **new dated sections**; do not edit §1 or §2 body except the original author.  
- [ ] If implementing anything: pure-software vs HW-gate paths per SESSION_CHECKLIST; renames via clang-refactor only.

---

<!-- NEXT AGENT: append your section below this line. Do not edit sections above. -->
