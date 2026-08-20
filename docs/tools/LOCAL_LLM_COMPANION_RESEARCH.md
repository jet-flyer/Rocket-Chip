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

## Section 3 — Quantization as a choice dial (pedagogical)

**Author:** Grok 4.6 (Build CLI) · **Date:** 2026-08-19  
**Picks up:** §1.7 item 1 / WB “quant levels as a choice dial.” Does **not** pick a model or buy weights. Complements §1.2–1.3 (does not rewrite them).

### 3.1 What is being rounded

A trained weight is a real number. The usual training format is **16-bit float** (fp16 / bf16): about two bytes per parameter. A 35B-parameter dense model in fp16 is ~70 GB of *weights alone*, before context (KV cache).

**Quantization** replaces each weight with a value from a coarser grid so it takes fewer bits. Q4 ≈ 4 bits per weight ≈ ¼ the weight file of fp16. The *architecture* (layers, attention, MoE routing) is unchanged. You are not training a smaller brain; you are storing the same brain more coarsely.

Analogy that is fair and that breaks in a useful place: **MP3**. MP3 throws away parts of the signal humans barely hear. Quantization throws away parts of the weight the *loss function* barely used. Both look “almost the same” on typical input. Both fail on the rare, high-precision bit — a cymbal transient, or a tool-call JSON field.

RocketChip-shaped analogy: storing a sensor sample as `uint8_t` when the spec only needs 8 bits is fine; storing a **CRC polynomial** or a **CCSDS bit-field width** as a rounded float is not. Quantization error is not uniform across “kinds of thinking.” Chat prose is robust. Structured tokens (tool names, file paths, exact numbers) are brittle.

### 3.2 How to read a GGUF name

`Q4_K_M` is three decisions, not one number.

| Piece | Meaning | Sourcing |
|---|---|---|
| **Q4** | Nominal ~4 bits per weight. Effective bits are usually a little *higher* because some tensors stay wider. | llama.cpp `QUANT_OPTIONS` / format comments. **Primary** for the *names*. |
| **K** | “K-quant”: super-block + mixed precision inside the block (llama.cpp PR [#1684](https://github.com/ggerganov/llama.cpp/pull/1684)). Older `Q4_0` / `Q4_1` are cruder uniform blocks. | **Primary** (PR). |
| **S / M / L** | How *much* mixed precision: Small / Medium / Large. M keeps more sensitive tensors at higher bits than S, so M is usually the default 4-bit pick. | **Primary** (llama.cpp option text). |
| **IQ…** | “I-quant”: uses an **importance matrix** from calibration text (`llama-imatrix`) so bits are spent on weights that move the loss more. Smaller at the same nominal bit-width; quality depends on that calibration. | **Primary** (llama.cpp imatrix docs). **Secondary** for “always better/worse than K.” |
| **UD- / Unsloth** | Someone else’s extra mix (e.g. UD-Q4_K_XL). Treat as a *vendor preset* of the above, not a new physics. | Secondary until the card states the recipe. |

**QAT** (already §1.3): the model was *trained* knowing it would be 4-bit. That advantage is welded to the format it was trained for (Gemma’s Q4_0). Do not “upgrade” a QAT file to Q6 and expect a free lunch.

Effective bits-per-weight for K-quants are **not integers**. Academic survey of llama.cpp K-quants on Llama-3.1-8B (arXiv [2601.14277](https://arxiv.org/html/2601.14277v1), 2026): Q4_K_M ≈ **4.5** bpw, Q5_K_M ≈ **5.5**, Q6_K ≈ **6.5**. **Secondary** (one paper, one model) — use as a picture of *why* “Q4” is not “exactly 4,” not as RocketChip numbers.

### 3.3 What you actually lose (and what PPL does not tell you)

**Perplexity (PPL)** asks: how surprised is the model by ordinary text? Lower is better. llama.cpp can measure it (`llama-perplexity`). It is the *standard proxy* for quant damage because it is cheap and repeatable.

Typical *shape* of the curve (WikiText-style PPL vs fp16; **secondary** community + older GPTQ study arXiv [2402.16775](https://arxiv.org/html/2402.16775v1) — not this box, not Qwen3.6):

- **Q8 / Q6:** PPL change is usually *inside measurement noise*. Blind llama.cpp votes (discussion [#5962](https://github.com/ggml-org/llama.cpp/discussions/5962)) could not tell Q5/Q6 from each other with confidence except that **IQ1** was obviously worse.
- **Q4_K_M:** small PPL bump; this is why it is the default. You notice it on *edge* cases, not on “write a paragraph.”
- **Q3 and especially Q2:** PPL can explode; 2-bit GPTQ models in that 2024 study **stopped following instructions** and emitted incoherent text. Floor, not a dial step.

**The pedagogical point:** PPL is an *average over next-token prediction*. The companion job is **not** “sound like Wikipedia.” It is “emit a valid tool call + a correct file edit + a `ctest` that still passes.” That fails as a *discrete* error (wrong path, dropped brace, hallucinated symbol). A 1.5% PPL change can be invisible in chat and fatal in a JSON argument.

So the dial for *this repo* is not “lowest PPL that fits.” It is:

> At this quant, does the model still **apply an already-decided work order** with a machine-checkable artifact?

That is exactly §1.4–1.5. We do **not** have that number yet. Throughput in §1.2 (~20–30 tok/s) is already labeled an estimate. Same honesty here: **no quant ranking for the executor role is claimed until the salted-corpus harness (§1.5 / next item) exists.**

What we *can* say without a harness:

1. **Do not go below Q4_K_M** for a model you expect to follow a contract. Q3/Q2 are a different *kind* of damage (coherence), not “a bit more lossy.”
2. **Q8 is almost never worth 2× the bytes** for this role. The leftover RAM is better spent on **context / KV cache** (see §3.4) or on *not* evicting experts from RAM so hard that generation stutters.
3. **Q5_K_M / Q6_K are the “I have RAM, spend it on weights” steps** — only interesting if a shakedown shows Q4 breaking tool calls or exact identifiers. On this box they still fit (Qwen3.6-35B-A3B: Q4_K_M 22.1 GB · Q5_K_M 26.5 · Q6_K 29.3 per §1.2 card sizes).
4. **IQ4_XS is a size trick, not a free upgrade.** Smaller than Q4_K_M; more sensitive to imatrix quality. Fine as an *experiment*, not as the first dial notch.

### 3.4 The second budget: KV cache (context), not just weights

The GGUF file is the *weights*. While generating, the model also stores **keys and values for every token in the window**. That grows with context length and sits in **VRAM** unless you offload it too.

Consequence for the dial, on a **12 GB card + 62 GB RAM** (§1.1):

- Weights of a 35B-class MoE can live mostly in **RAM** (expert offload). That is why Q4 vs Q6 is “do I spend 7 GB more *system* RAM,” not “does it fit on the GPU.”
- **Context** still wants GPU-side space. A fat quant that leaves no VRAM for KV *hurts the job* (chunking files, exactly the audit failure mode in §1.4) more than a slightly lossier weight file.
- So “higher quant is always better if it fits in RAM” is false. The executor reads *files*. Window size is part of capability.

This is the same “budget is shared” idea as PIO SMs on the WB (watchdog vs beacon vs I²C). Bits spent on weights are bits (and bandwidth) not spent on context.

### 3.5 How to use the dial (not a purchase)

Think of three notches, then *stop* until a shakedown:

| Notch | When you turn it | What you are buying |
|---|---|---|
| **Q4_K_M** | Default for the first executor try | Fits easily; community default; leave VRAM for KV |
| **Q5_K_M or Q6_K** | Only if Q4 fails *checkable* things (malformed tools, wrong identifiers) and you have already turned thinking off / fixed the tool parser (§1.2) | Weight fidelity. Measure tok/s — you may have bought quality and sold speed. |
| **Q8 / fp16** | Almost never on this box for a 35B-class model | Diminishing returns; starves context or forces a smaller *model* |

**Gemma QAT-Q4_0** is a *parallel* notch, not a higher one: same 4-bit *class*, trained for it (§1.3). Compare it to Qwen-Q4 as “different 4-bit,” not as “worse than Qwen-Q6.”

**Do not** pick a notch because a blog said “95% of quality.” That is the same failure as picking a timeout without a datasheet. The datasheet here is a **work-order suite + `ctest` / compile**, which is item 3 on the original open list.

### 3.6 What this does *not* settle

- Which *model* (Qwen3.6-35B-A3B vs Gemma QAT vs other) — still §1.2 + a shakedown.
- Big + surgical-small vs one large — still item 2; FIM/embeddings are other *functions* (§1.6), not extra notches on this dial.
- Odysseus backend flags — still item 4. The dial assumes llama.cpp-class GGUF; Ollama names (`q4_K_M`) are the same family.

### 3.7 Sourcing

| Claim | Label |
|---|---|
| What Q/K/S/M/IQ *are* | **Primary** — llama.cpp PR #1684, `QUANT_OPTIONS`, imatrix docs |
| Qwen3.6-35B-A3B GGUF sizes | **Primary** — already §1.2 (Unsloth/HF cards) |
| QAT welded to Q4_0 | **Primary** — already §1.3 (Google) |
| 2-bit can destroy instruction following | **Secondary** — arXiv 2402.16775 (GPTQ, 2024, not this model) |
| Q4_K_M ≈ 4.5 bpw | **Secondary** — arXiv 2601.14277 (Llama-3.1-8B) |
| “Q6 ≈ indistinguishable in blind chat votes” | **Secondary** — llama.cpp #5962 (Mistral 7B, large error bars) |
| “Q4 loses 1–3% PPL / MMLU” blog tables | **Unverified for this model** — do not cite as our number |
| Executor-role ranking of Q4 vs Q6 on *this* box | **Not measured** — harness not built |

---

## Section 4 — Quant dial, off-box empirical (not this machine)

**Author:** Grok 4.6 (Build CLI) · **Date:** 2026-08-19  
**Picks up:** §1.7 item 1 / WB quant dial, after §3 (teaching frame). **Does not close the item.** Complements §1–§3; does not rewrite them. No procurement. No this-box run.

**Role of this section:** New numbers that §1–§3 did not have — task-level quant comparisons, KV-cache K vs V, and install-RAM vs file-size. Written so the owner can argue with them. Owner discussion is the point of the row.

### 4.1 What is new vs §1–§3

§1 listed GGUF *file* sizes and said the quality dial on this box is quant, not parameter count. §3 explained what the names mean and said we had no executor ranking. This section is the first set of **task / tail** numbers for those names.

None of it is measured on the 4070 Ti. Label is **secondary** unless noted. The job we care about is still §1.4: apply an already-decided work order with a checkable artifact.

### 4.2 Weight quant: a plateau on SWE *resolved*, not on format errors

Source: Hugging Face `unsloth/Qwen3.6-35B-A3B-GGUF` discussion **#10**, user `jerobnd`, first **100** SWE-bench Verified samples, **mini-swe-agent**, 250-turn cap, **one pass**. Unsloth (`shimmyshimmer`, 2026-04-18) cautioned: n=100, no repeats, “isn't the best metric.” Still the only head-to-head *agent* table we have for these exact GGUFs.

| Model | Quant | resolved / 100 | unresolved | error | incomplete |
|---|---|---|---|---|---|
| Qwen3.5-35B-A3B | Q4_K_M | **59** | 25 | 14 | 2 |
| Qwen3.5-35B-A3B | UD-Q6_K_XL | **59** | 29 | 6 | 6 |
| Qwen3.5-35B-A3B | Q8_0 | **59** | 30 | 8 | 3 |
| Qwen3.5-122B-A10B | UD-Q5_K_XL | 69 | 28 | 0 | 3 |
| Qwen3.5-27B | UD-Q4_K_XL | **71** | 26 | 2 | 1 |
| Qwen3.6-35B-A3B | UD-Q8_K_XL | 53 | 26 | 18 | 3 |
| Qwen3.6-35B-A3B | Q5_K_M (AesSedai) | 51 | 29 | 18 | 2 |

**Error** = output did not start with `diff --git` (failed the system prompt). **Incomplete** = hit 250 turns.

**Finding A — climbing Q4→Q8 bought zero extra SWE resolves on 3.5-35B.** Same 59. What moved was the *error* column (14 → 6 → 8): format / contract, not “smarter code.” That is the opposite of the blog story “higher bits = more capability.” For a RocketChip executor whose failures are supposed to be compile/`ctest`/diff, this is the interesting number.

**Finding B — Qwen3.6 Q8 ≈ Qwen3.6 Q5 on this harness, and both lose to 3.5-Q4.** 53 and 51 resolved, both with 18 format errors. Official Qwen card (primary, §1.2 / HF model card): SWE-bench Verified **73.4** (3.6-35B) vs **70.0** (3.5-35B). The quantized mini-swe run inverts that. Possible causes, not ranked: (1) chat template / `diff --git` contract; (2) tester used thinking-mode *precise coding* `temp=0.6` while Qwen’s SWE footnote is `temp=1.0, top_p=0.95, 200K ctx`; (3) 3.6’s card gains do not survive GGUF + this agent. Unsloth did not refute the table; they asked for more samples.

**Finding C — a smaller dense 27B beat the 35B MoE at Q4 on the same 100.** 71 vs 59. That punches a hole in “quality dial is quant not param count” *if* you only compare MoEs. It does **not** by itself pick 27B for this repo — n=100, one pass, different architecture — but it is now a live model question sitting *inside* the quant row. Same discussion also has `islameissa` (clinical 18-prompt suite, GPT-5.4 judge, 10 runs): Qwen3.5-27B UD-Q4_K_XL **77.0** overall vs Qwen3.6-35B **74.5** vs Qwen3.5-35B **72.0**. Different domain; same direction.

HumanEval is a different axis (`LadyJun`, 164 tests, pass@1): 3.6 UD-Q6 **93.29 / 90.24** vs 3.5 UD-Q6 **98.78 / 93.9**. Same comment: 3.6 tool calling “never failed.” Gemma 4 31B dense sat at 100 / 94.51 on that suite. Executor job is closer to mini-SWE + tools than HumanEval.

**What this does *not* prove:** Q4_K_M is good enough on *this* box, for *our* work orders. Treat the table as a *split* (capability plateau vs format-error movement), not a ranking of which file to keep. Owner has storage; “first download” is not the constraint.

### 4.3 PPL / KLD can rank the dial backwards

Unsloth, primary (their Qwen3.5 GGUF-benchmarks page + Reddit `1rgel19`, 2026-02): they still publish KLD Pareto (Qwen3.6-35B-A3B: Unsloth first in **21 of 22** sizes on mean KLD) and also say **PPL/KLD can invert real evals**. Cited case: Unsloth Dynamic IQ2_XXS beat AesSedai IQ3_S on LiveCodeBench v6 / MMLU-Pro while being ~11 GB smaller; AesSedai PPL 0.2441 vs Unsloth 0.3552 and KLD 8.28 vs 9.03 (lower = closer to BF16). UD-Q4_K_XL beat other Q4s while ~8 GB smaller on that plot.

Other Unsloth facts that change *which named file* to try, not the bit number:

- **MXFP4 retired** from most mixed quants (Q2/Q3/Q4_K_XL); kept for `MXFP4_MOE`. Q4_K is 4.5 bpw vs MXFP4 4.25; they prefer Q4_K on sensitive tensors.
- **Do not cheapen `attn_*` or `ssm_out`.** `ffn_up_exps` / `ffn_gate_exps` tolerate ~3-bit; `ffn_down_exps` less so. This is why UD- / `_XL` exists — it is a *tensor recipe*, not “Q4 plus marketing.”
- Imatrix helps low bits; **I-quants ~5–10% slower** decode (`iq3_xxs` tg128 ≈ 85 vs `q4_k` ≈ 90 in their table).
- Mar 5 2026 Qwen3.5 update: UD-Q4_K_XL **max KLD 5.894 → 2.877 (−51%)**, file 19.2 → 20.7 GB. Recipe date matters; an old XL is not the current XL.

**Implication for us:** Unsloth UD-Q4_K_XL (or current Q4_K_M) is the interesting 4-bit *file*, not a random Q4_0. KLD is a rough signal, not the executor metric. That agrees with §1.4 and §3.3; we now have Unsloth saying it about their own plots.

### 4.4 The second dial is KV cache, and it is a VRAM dial

Weights of a 35B MoE can live in **system RAM** on this box (§1.1). The KV cache for the window mostly wants **VRAM**. 12 GB is the scarce side. So Q4 vs Q6 is “spend 7 GB more DDR5”; KV q8 vs q4 is “how much context fits on the 4070 Ti without wrecking tool JSON.”

Unsloth Qwen3.6 run page (primary): if output is gibberish, try `--cache-type-k bf16 --cache-type-v bf16` (and do not use CUDA 13.2). That is them pointing at **cache type**, not weight bits.

**Asymmetric K vs V — llama.cpp discussion #23470** (sanmai, Qwen2.5-7B Q4_0 GGUF, WikiText, vs f16 KV; **secondary**, old Qwen, ctx 512):

| flags | mean KLD | same top-p | note in thread |
|---|---|---|---|
| `-ctk q4_0 -ctv q4_0` | **5.51** | 11.6% | “out of touch with f16”; PPL 8 → **1589** |
| `-ctk q4_0 -ctv f16` | 5.49 | 11.8% | almost the same collapse — damage is **K** |
| `-ctk f16 -ctv q4_0` | 0.0040 | 96.9% | V at 4-bit nearly free |
| `-ctk q8_0 -ctv q4_0` | 0.0048 | 96.7% | |
| `-ctk q8_0 -ctv q8_0` | 0.0018 | 98.0% | |

Follow-up in the same thread (`chambejp`, 2026-08-16, 500 ARC questions, grammar-constrained, answer-*churn* vs f16): q8_0 K+V changed ≤4/500; q4_0 K+V was model-dependent (Qwen3.6-27B 2/500, Qwen2.5-7B **375/500** and score 92%→24%). On the sensitive model, **q4 K alone reproduced the collapse; q4 V alone changed 1/500.**

**Qwen3.6-27B long-context KV table** — Anbeeld (2026, BeeLlama/llama.cpp fork, Q5_K_S weights, 64k, vs bf16 KV). PPL almost flat through q4_0 (5.480 → 5.488). Tail is not:

| K / V | size vs bf16 | 99.9% “precision” vs bf16 (their exp formula) |
|---|---|---|
| bf16 / bf16 | 100% | 100% |
| q8_0 / q8_0 | 53.1% | 94.6% |
| q8_0 / q5_1 | 45.3% | 94.2% |
| q5_0 / q5_0 | 34.4% | 92.7% |
| q5_0 / q4_1 | 32.8% | 92.7% |
| q4_0 / q4_0 | 28.1% | 89.8% |

Author’s own caveat: this is PPL/KLD, not tool-call success; they discarded JSON-schema tests because everything scored 100% at the scale they tried. Community reports still match the *direction*: Reddit 2026-08 (Qwen3.6-27B UD-Q5_K_XL) “Q4 KV is turning qwen 3.6 into mental”; split `-ctk q8_0 -ctv q4_0` called a long-context sweet spot. llama.cpp collaborator note: some mixed pairs **silently fall back to CPU** unless the FlashAttention quant combo is compiled in (`GGML_CUDA_FA_ALL_QUANTS`) — so a “better” asymmetric pair can *look* like a speed regression.

**Qeios preprint RGD04F (2026-08, secondary):** Qwen3-4B key-precision ablation — Q4 and Q8 keys similar *likelihood*; GSM8K 200-example pilot still favored **Q8 keys by 13 points**. Same mismatch class as Unsloth’s PPL-vs-LiveCodeBench.

**Implication for this box:** default KV on llama.cpp is f16/bf16. That is the quality floor. The first *compression* to try for context is **q8_0 on K** (both, or K=q8 V=q5/q4), not q4/q4. Do not spend the 12 GB on a fatter *weight* file if that is what starves KV. NVFP4 is Blackwell-only (Unsloth 2026-07-10); 4070 Ti is Ada 8.9 — ignore NVFP4 speed claims.

### 4.5 File size vs “runs in N GB” vs this install

Unsloth Qwen3.6 run page, **primary**, “total memory: RAM + VRAM, or unified”:

| | 3-bit | 4-bit | 6-bit | 8-bit | BF16 |
|---|---|---|---|---|---|
| 35B-A3B weights | 17 GB | **23 GB** | **30 GB** | **38 GB** | 70 GB |
| same + MTP (~+1 GB) | 18 | 24 | 31 | 39 | 71 |

HF GGUF blobs (primary, Unsloth card): UD-Q4_K_M **22.1 GB**, UD-Q4_K_XL **22.4**, UD-Q5_K_M **26.5**, UD-Q6_K **29.3**, UD-Q6_K_XL **31.8**, Q8_0 **36.9**, UD-Q8_K_XL **38.5**. Matches §1.2.

This machine (§1.1 / §2.1): **61.6 GB** host RAM + **12 GB** VRAM. Docker Odysseus `/proc/meminfo` was **~30 GiB**. So:

- **Host** llama.cpp / host Ollama: Q4 (23) and Q6 (30) are easy; Q8 (38) still fits with KV + OS if you are not also stuffing the container.  
- **In-container Cookbook Serve:** 30 GiB is roughly the Q6 ceiling before the VM, not the weights, is the limit. Q8 is the wrong experiment on that path until `.wslconfig` is raised (§2.9 item 3).

MTP (Unsloth, primary): MoE speedup **~1.15–1.2×** vs dense **1.4–2×**; they recommend `--spec-draft-n-max 2`; more drafts drop accept rate 83%→50% at 4. No claimed accuracy change. Speed knob, not a quality notch. ~1 GB extra.

### 4.6 Adjacent knobs that impersonate “need more bits”

These are not quant, but they produce the same *visible* failure (bad JSON, looping, “dumber”) that people then try to fix by climbing Q4→Q8.

1. **Thinking on, for an executor.** `islameissa` 10-run suite: Qwen3.5-27B standard **77.0** vs thinking **71.4** (code 82.7→78.2). Gemma thinking *helped* code. Qwen3.6 thinks **by default**; §1.2 already said turn it off for this role (`enable_thinking: false`). jerobnd’s 3.6 mini-SWE used thinking-mode 0.6; Qwen’s SWE footnote is 1.0. Template/sampling may explain more of the 59 vs 53 gap than Q5 vs Q8.
2. **Tool parser / Jinja / `qwen3_coder`.** Unsloth has repeatedly shipped chat-template fixes that applied to *all* uploaders (Qwen3.5 GGUF page; older Qwen3-Coder Roo threads). A Q6 that loops in an agent is often the template, not 6-bit.
3. **CUDA 13.2** — Unsloth: gibberish; use <13.2 or 13.3.

### 4.7 Working hypotheses (for the owner to break)

**Retraction (same day):** 4.7 originally framed this as “which file to download first.” Owner clarified storage is not the constraint, and the question is **what changing the bit width does**, not which quant is best. Hypotheses below are about *effects*, not a shopping list.

1. **On this job, Q4→Q8 mostly changes contract-failure rate, not “how many bugs it can independently solve.”** Motivated by Finding A’s split (resolved stuck at 59; errors 14→6). Incomplete test; direction is the claim.
2. **Q8 vs Q6 is unlikely to be observable on tedious apply-the-diff work** once thinking is off and the tool parser is correct. Cost is RAM + bandwidth, not a smarter executor.
3. **Q3/Q2 is a different *kind* of damage** (coherence / ignore-the-schema), not “Q4 but a bit worse.”
4. **KV-cache bit width can change the same failure mode as weight quant** (malformed tools, lost JSON keys) and hits VRAM, which is the scarce side on this box.
5. **Qwen3.6 vs 3.5 is a template/harness question before it is a quant question.** Card says 3.6 wins SWE; mini-SWE says 3.6 Q8 loses to 3.5 Q4 with format errors.

### 4.8 Still unmeasured (this is why the WB row stays)

- Any quant on the **4070 Ti + 62 GB** with Odysseus as actually installed (§2). Throughput in §1.2 is still an estimate.  
- Executor-shaped tasks from **this repo** (work orders + `ctest` / dual build / `rg`). Salted harness is still §1.7 item 3.  
- Whether KV q8 vs f16 changes *tool-call* rate on Qwen3.6-35B-A3B (Anbeeld explicitly did not show that).  
- Whether UD-Q4_K_XL vs Q4_K_M matters on agent tasks (KLD says XL; mini-SWE used Q4_K_M for the 59).

### 4.9 Questions for the owner (this session)

**Retraction:** Q1 was “first download / one 4-bit file.” Wrong question (storage is fine; owner wants what the dial *changes*).

1. For the executor job, is the useful split **capability vs contract-failure** (resolved vs `diff --git` / tool JSON), or do you want a different “what changed” axis (speed, context window, exact identifiers, `ctest` miss rate)?  
2. Is 27B-beats-35B on that 100 a *model* tangent we should park until the quant-effects discussion is done?  
3. Serve path still matters for *what you can observe*: host 62 GB vs Docker ~30 GiB changes whether Q8 even runs, not whether it is “better.”  
4. Next research fork on this row: keep pulling **what each notch changes for tool/edit work**, not bake-off winners.

### 4.10 Sources / confidence

| Claim | Label |
|---|---|
| Unsloth 35B-A3B RAM+VRAM table; MTP +1 GB; gibberish → bf16 KV; CUDA 13.2; NVFP4 = Blackwell; MTP 1.15–1.2× MoE | **Primary** — https://unsloth.ai/docs/models/qwen3.6 |
| GGUF blob sizes | **Primary** — https://huggingface.co/unsloth/Qwen3.6-35B-A3B-GGUF |
| KLD 21/22 Pareto; MXFP4 retire; attn/ssm sensitivity; PPL/KLD invert vs LiveCodeBench; I-quant 5–10% slower | **Primary** (Unsloth) — https://unsloth.ai/docs/models/qwen3.5/gguf-benchmarks |
| mini-SWE first 100 table; HumanEval LadyJun; islameissa 18-prompt suite | **Secondary** — HF discussion #10; n=100 / 164 / 18 as stated; Unsloth caution on n=100 |
| Official SWE 73.4 vs 70.0 | **Primary** — Qwen3.6-35B-A3B model card (already §1.2) |
| KV K vs V KLD on Qwen2.5-7B; ARC churn | **Secondary** — llama.cpp #23470 |
| Qwen3.6-27B 64k KV ladder | **Secondary** — https://anbeeld.com/articles/kv-cache-quantization-benchmarks-for-long-context (fork of llama.cpp; author says not tool-call proof) |
| GSM8K Q8-keys +13 | **Secondary** — Qeios RGD04F |
| Docker ~30 GiB vs host 62 GB | **Primary** — already §2.1 |
| Odysseus Cookbook quant in the score | **Primary** — `C:\Apps\odysseus\services\hwfit\fit.py` + `models.py` (this install). See §4.11. |

### 4.11 Cookbook columns: quant label vs score (this install)

**Correction:** the scan row has **separate** columns. Owner took `quant` as a factual storage-format tag (same class as params), not as the derived `score`. That reading matches the UI: params tooltip is “original total model parameters”; quant suffix tooltip is “full storage format”; VRAM tooltip is “**Estimated** loaded footprint for this quant/backend/context”; score is its own cell (`hwfit-c-score`). Do not treat the quant *cell* as a quality number.

What the **quant cell** is: a GGUF/HF **format name** (`Q4_K_M`, `AWQ-4bit`, `FP8`, …). For prequantized HF repos it is the catalog’s native format. For generic GGUF rows, Cookbook **defaults the evaluated format to `Q4_K_M`** unless you pick a quant filter (`fit.py`: “Default: Q4_K_M (user's stated preference)”). So the label is a real format, but on those rows it is “the recipe this row is priced at,” not a lab measurement of the weights.

The **score** (already §2.8) is the derived thing. It *consumes* that format name. Composite = `quality×w + speed×w + fit×w + context×w`. Coding weights: **0.50 / 0.20 / 0.15 / 0.15**. The format name is plugged into **three** slots of the *score*, only one of which is “quality”:

| Slot | What it uses quant for | Knowable without running the model? |
|---|---|---|
| **Fit** | `estimate_memory_gb` from params × bytes-per-param. Hierarchy tries `Q8_0 → Q6_K → Q5_K_M → Q4_K_M → Q3_K_M → Q2_K` until it fits VRAM, else RAM (offload), else halve context. | **Yes** (file size vs RAM/VRAM). This is the real reason the column exists. |
| **Speed** | tok/s ≈ bandwidth / (active_params × bpp). Smaller quant → fewer bytes streamed → higher *estimated* tok/s. CPU fallback also has `QUANT_SPEED_MULT` (Q4 1.15, Q8 0.8). | **Mostly yes** as an order-of-magnitude. Calibrated on an RX 9060 XT in a comment; not this 4070 Ti. |
| **Quality** | Hardcoded `QUANT_QUALITY_PENALTY` added to a **parameter-count band** + name bonuses (`qwen` +2, `qwen3.6` +9). **Not a bench.** | The *penalty table* is knowable (it is the table). Whether it matches *this job* is **not**. |

Penalty table (points on the 0–100 quality subscore):

| quant | penalty |
|---|---|
| Q8_0 / BF16 / FP8 | 0 |
| Q6_K | −1 |
| Q5_K_M | −2 |
| Q4_K_M / Q4_0 | **−5** |
| Q3_K_M | −8 |
| Q2_K | −12 |
| QAT-INT4 | −1 (they know QAT is not PTQ Q4) |

A 35B model’s quality *base* is 89 before bonuses. Q4 vs Q8 is a **5-point fudge**, same for every model, every task. Coding use-case does not change the quant penalty; it only boosts models tagged “coding” (+6).

**Implication:** Cookbook is answering “will this *size* run, and shall I dock the score a bit for 4-bit.” It is not answering “does Q4 vs Q6 change tool-call rate on RocketChip work orders.” §2.8 already said not to treat the single score as a SWE-bench. The quant column is the fit ladder plus that dock.

---

## Section 5 — Owner shortlist to try (download later)

**Author:** Grok 4.6 (Build CLI) · **Date:** 2026-08-20  
**Does not close** the quant row or adopt a companion. Owner asked to record a **try-later quiver** after Gemma / Mistral / NVIDIA research, with Qwen kept because it still leads on agentic coding. No weights downloaded this session.

**Job this shortlist is for** (owner, this session): background worker; tedious/expensive/straightforward work; **do not stray from directions**; coding housekeeping (not full function generation); snappiness and eloquence do not matter. Cloud/frontier still does the thinking.

### 5.1 What a “model” is (and what llama.cpp is)

These are **not programs**. A local LLM is a large array of learned numbers (tensors / a “weight matrix”). Training produced those numbers. Inference **reads** them and predicts the next token. The software that does that read is a **runtime** — here **llama.cpp** (Odysseus Cookbook Serve) or Ollama. **GGUF** is the file format: the same tensors, stored at a named bit-width (`Q4_K_M`, `Q8_0`, Google’s `q4_0` QAT, …). Quant (§4) is how coarsely those numbers are stored, not a different architecture.

### 5.2 MoE vs dense (why “35B” is not “24B but bigger”)

**Dense:** every parameter is used on every token. Devstral Small 2 is **24B dense** — all ~24B fire each step. Cost scales with total size.

**MoE (Mixture of Experts):** the file still holds *all* experts (e.g. Qwen3.6-**35B**-A**3B** = 35B stored, **~3B active** per token). A router picks a handful of experts (Qwen: 8 routed + 1 shared of 256). You pay RAM/disk for 35B, but **bandwidth per token** is closer to a 3B model. That is why §1.1 said 12 GB VRAM + 62 GB DDR5 favors MoE with expert offload: experts live in RAM; only active ones stream.

Nemotron 3 Nano / 3.5 Lightning are also **30B total / 3B active** MoE (hybrid Mamba + attention). Super 120B-A12B and Ultra 550B-A55B are the same idea at sizes that do **not** belong on a 4070 Ti.

### 5.3 NVIDIA GPU ≠ Nemotron exclusive

An RTX card runs **any** CUDA-backed GGUF (Qwen, Gemma, Devstral, Nemotron). Nemotron is NVIDIA’s **open-weight family**, not a hardware unlock. Nano/Lightning **can** run here. Super/Ultra are the “dedicated LLM box / many-GPU” sizes. Nemotron is **not** the first pick for this job just because the GPU is NVIDIA.

### 5.4 Paper scores vs this job (why Qwen stays in the quiver)

SWE-bench Verified ≈ “independently fix GitHub issues.” That is **harder** than apply-an-already-decided work order. It is still the closest public coding-agent number.

| Model | Kind | SWE-bench Verified | Source |
|---|---|---|---|
| Qwen3.6-35B-A3B | MoE 35B/3B | **73.4** / NVIDIA table **70.1** | Qwen card / NVIDIA Lightning card |
| Devstral Small 2 24B | **Mistral** dense 24B | **68.0** | Mistral card |
| Nemotron 3 Super 120B | NVIDIA MoE | 63.1 | NVIDIA Lightning card |
| Gemma 4 26B-A4B | Google MoE | 57.4 (NVIDIA) / 17.4 (Qwen table — different harness) | those two cards |
| Nemotron 3.5 Lightning 30B-A3B | NVIDIA MoE 30B/3B | 51.6 | NVIDIA card |
| Nemotron 3 Nano 30B-A3B | NVIDIA MoE | 34.1 | NVIDIA card |

Owner preference was Gemma / Mistral / NVIDIA first. **Correction recorded:** for *this* job, that trio did not catch Qwen. If Chinese origin is acceptable **because it is open-weight and local**, keep Qwen. If it is not, **Devstral Small 2** is the non-Chinese coding-agent stand-in (not Gemma 26B-A4B, not Nano).

Ministral 3 14B is Mistral’s **edge** line, not the coding savant. Skip for this quiver.

### 5.5 Download shortlist (four + one format note)

Try on **host** llama.cpp/Ollama first if Q6/Q8 is in play (Docker Odysseus saw **~30 GiB** RAM). Default weight format unless noted: current Unsloth **UD-Q4_K_XL** or **Q4_K_M**; climb bits only if *checkable* contract failures remain (thinking off, correct tool parser). Gemma is the exception: **Google QAT-Q4_0**, do not upconvert (§1.3).

| # | Try | Why it’s here | Hugging Face (GGUF) |
|---|---|---|---|
| 1 | **Qwen3.6-35B-A3B** | Still the local coding-agent leader; MoE fits this box | `unsloth/Qwen3.6-35B-A3B-GGUF` |
| 2 | **Devstral Small 2 24B Instruct 2512** | Mistral agentic-coding model; 68% SWE; dense 24B | `unsloth/Devstral-Small-2-24B-Instruct-2512-GGUF` |
| 3 | **Gemma 4 31B IT QAT Q4_0** | Stronger Gemma than 26B-A4B; official QAT 4-bit | `google/gemma-4-31B-it-qat-q4_0-gguf` |
| 4 | **Nemotron 3.5 Lightning 30B-A3B** | Current NVIDIA 30B-class; optional A/B, not because of the 4070 Ti | `unsloth/NVIDIA-Nemotron-3.5-Lightning-30B-A3B-GGUF` |

**Also discussed, not in the four:** Gemma 4 26B-A4B QAT (`google/gemma-4-26B-A4B-it-qat-q4_0-gguf`) — easier MoE fit, weaker coder; Qwen3.5/3.6 **27B dense** — one incomplete mini-SWE had 27B-Q4 beating 35B-Q4; Nemotron 3 **Nano** (older/weaker SWE) and **Super/Ultra** (wrong size).

Executor knobs that are **not** the GGUF name: `enable_thinking: false` for this role (§1.2); tool parser (`qwen3_coder` on Qwen; Mistral tool parser on Devstral); KV cache at least q8 on **K** if you compress context (§4.4).

### 5.6 Cookbook (live 2026-08-20)

UI at `http://127.0.0.1:7000`, Cookbook columns sortable: FIT · MODEL · VRAM · PARAM · QUANT · CTX · SPEED · SCORE · MODE. Detected: 4070 Ti **12.0 GB**, RAM **28.2 / 30.1 GB** (container, not host 62 GB). QUANT cell = format name (default GGUF **Q4_K_M**); SCORE is separate. Owner can download via Cookbook **or** Hugging Face directly — the table in §5.5 is the HF source of truth.

### 5.7 Sources

| Claim | Label |
|---|---|
| MoE 35B/3B, 256 experts | **Primary** — already §1.2 Qwen card |
| Devstral Small 2 SWE 68.0, 24B, Apache 2.0, 256k | **Primary** — https://huggingface.co/mistralai/Devstral-Small-2-24B-Instruct-2512 |
| Lightning / Nano / Super SWE table | **Primary** — nvidia/NVIDIA-Nemotron-3.5-Lightning-30B-A3B-BF16 model card |
| GGUF repo IDs and quant filenames | **Primary** — Hugging Face `/api/models/{id}` 2026-08-20 |
| Gemma QAT welded to Q4_0 | **Primary** — already §1.3 |
| Cookbook live columns / 30.1 GB RAM | **Primary** — Odysseus UI this session |

---

<!-- NEXT AGENT: append your section below this line. Do not edit sections above. -->
