# Graph Report - C:\Users\pow-w\Documents\Rocket-Chip  (2026-08-27)

## Corpus Check
- 59 files · ~67,644 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 115 nodes · 197 edges · 20 communities (15 shown, 5 thin omitted)
- Extraction: 82% EXTRACTED · 18% INFERRED · 0% AMBIGUOUS · INFERRED: 35 edges (avg confidence: 0.79)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- [[_COMMUNITY_PLTU codec tests|PLTU codec tests]]
- [[_COMMUNITY_D-5 heap trap|D-5 heap trap]]
- [[_COMMUNITY_PLTU C++ types|PLTU C++ types]]
- [[_COMMUNITY_PLTU envelope claims|PLTU envelope claims]]
- [[_COMMUNITY_CRC-32 and CMake|CRC-32 and CMake]]
- [[_COMMUNITY_FPGA and PIO ports|FPGA and PIO ports]]
- [[_COMMUNITY_Version-3 fields|Version-3 fields]]
- [[_COMMUNITY_Decode length and IVP|Decode length and IVP]]
- [[_COMMUNITY_COP-P and COP-1|COP-P and COP-1]]
- [[_COMMUNITY_PLCW and CLCW|PLCW and CLCW]]
- [[_COMMUNITY_ASM and 211.2|ASM and 211.2]]
- [[_COMMUNITY_Sans-IO core|Sans-I/O core]]
- [[_COMMUNITY_Duplex section 6|Duplex section 6]]
- [[_COMMUNITY_MIB parameters|MIB parameters]]
- [[_COMMUNITY_Error type|Error type]]
- [[_COMMUNITY_Result type|Result type]]
- [[_COMMUNITY_FPGA assurance handbooks|FPGA assurance handbooks]]

## God Nodes (most connected - your core abstractions)
1. `encode_pltu()` - 19 edges
2. `decode_pltu()` - 18 edges
3. `as_span()` - 16 edges
4. `main()` - 16 edges
5. `crc32()` - 12 edges
6. `PLTU` - 11 edges
7. `Version-3 Transfer Frame` - 7 edges
8. `load_be32()` - 6 edges
9. `test_one_data_octet()` - 6 edges
10. `test_codecs_allocate_nothing()` - 6 edges

## Surprising Connections (you probably didn't know these)
- `Version-3 Transfer Frame` --cites--> `211.0 Version-3 header`  [EXTRACTED]
  starcom/docs/SAD.md → standards/starcom/ccsds/CCSDS-211.0-B-6.pdf
- `PLTU` --cites--> `211.2 PLTU`  [EXTRACTED]
  starcom/docs/SAD.md → standards/starcom/ccsds/CCSDS-211.2-B-3.pdf
- `USLP Version-4` --cites--> `732.1 USLP`  [EXTRACTED]
  starcom/docs/SAD.md → standards/starcom/ccsds/CCSDS-732.1-B-3.pdf
- `PLTU repeater` --cites--> `133.0 §2.4 store-and-forward`  [INFERRED]
  starcom/AGENT_WHITEBOARD.md → standards/starcom/ccsds/CCSDS-133.0-B-2.pdf
- `FPGA port seam` --cites--> `211.1 Physical Layer`  [INFERRED]
  starcom/AGENT_WHITEBOARD.md → standards/starcom/ccsds/CCSDS-211.1-B-4.pdf

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **PLTU envelope ASM + frame + CRC-32** — docs_sad_asm, docs_sad_pltu, docs_sad_crc32, ccsds_ccsds_211_2_b_3_pltu [EXTRACTED 1.00]
- **Increment 0+1 first codec sitting** — docs_icd_crc32, docs_icd_decode_pltu, docs_icd_encode_pltu, docs_ivp_d5_trap, docs_icd_starcom_target [EXTRACTED 1.00]
- **Later PIO/FPGA/PHY ports** — starcom_agent_whiteboard_pio, starcom_agent_whiteboard_fpga, ccsds_ccsds_211_1_b_4_phy, fpga_readme_later_port [INFERRED 0.85]

## Communities (20 total, 5 thin omitted)

### Community 0 - "PLTU codec tests"
Cohesion: 0.31
Nodes (21): byte, crc32(), decode_pltu(), encode_pltu(), as_span(), main(), test_asm_in_crc(), test_codecs_allocate_nothing() (+13 more)

### Community 1 - "D-5 heap trap"
Cohesion: 0.21
Nodes (12): comparison D-1 to D-5, D-5 no-heap-after-init, D-5 heap trap, nothrow_t, size_t, bump(), heap_trap_arm(), operator delete() (+4 more)

### Community 2 - "PLTU C++ types"
Cohesion: 0.24
Nodes (8): load_be32(), PltuView, frame, store_be32(), kPltuCrcSize, Result, span, uint32_t

### Community 3 - "PLTU envelope claims"
Cohesion: 0.29
Nodes (8): 133.0 §2.4 store-and-forward, PLTU conformance claim, D-4 PLTU wraps V-3 XOR USLP, encode_pltu(), PLTU, USLP Version-4, Claude CCSDS PLTU research, PLTU repeater

### Community 4 - "CRC-32 and CMake"
Cohesion: 0.29
Nodes (7): 131.0 TM coding, 211.2 Annex C CRC-32, crc32(), Starcom::starcom, PLTU CRC-32, starcom static library, First codec TUs

### Community 5 - "FPGA and PIO ports"
Cohesion: 0.29
Nodes (7): 211.1 Physical Layer, Starcom port, NASA-HDBK-4008 HDL practice, NASA-HDBK-4011, FPGA as later Starcom port, FPGA port seam, PIO port seam

### Community 6 - "Version-3 fields"
Cohesion: 0.40
Nodes (6): 133.0 Space Packet, Frame Sequence Number, PCID, Port ID, Space Packet, Version-3 Transfer Frame

### Community 7 - "Decode length and IVP"
Cohesion: 0.33
Nodes (6): 211.0 Version-3 header, 211.2 §3.6.4 TFVN length, 732.1 USLP, decode_pltu(), IVP increment 0+1, T/A/R/I verification methods

### Community 8 - "COP-P and COP-1"
Cohesion: 0.50
Nodes (4): 211.0 COP-P §7, 232.1 COP-1, COP-1, COP-P

### Community 9 - "PLCW and CLCW"
Cohesion: 0.50
Nodes (4): 211.0 PLCW, 232.0 CLCW, CLCW 32-bit, PLCW 16-bit

### Community 10 - "ASM and 211.2"
Cohesion: 0.50
Nodes (4): 211.2 ASM FAF320, 211.2 PLTU, ASM FAF320, Starcom Blue Book shelf

### Community 11 - "Sans-I/O core"
Cohesion: 0.50
Nodes (4): sans-I/O, starcom::ccsds core, Pitchfork library layout, Starcom stack

## Knowledge Gaps
- **25 isolated node(s):** `frame`, `Starcom stack`, `Management Information Base`, `Frame Sequence Number`, `Result<T>` (+20 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **5 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `encode_pltu()` connect `PLTU codec tests` to `D-5 heap trap`, `PLTU C++ types`, `PLTU envelope claims`?**
  _High betweenness centrality (0.196) - this node is a cross-community bridge._
- **Why does `PLTU` connect `PLTU envelope claims` to `ASM and 211.2`, `CRC-32 and CMake`, `Version-3 fields`, `Decode length and IVP`?**
  _High betweenness centrality (0.194) - this node is a cross-community bridge._
- **Why does `decode_pltu()` connect `PLTU codec tests` to `PLTU C++ types`, `Decode length and IVP`?**
  _High betweenness centrality (0.101) - this node is a cross-community bridge._
- **Are the 11 inferred relationships involving `encode_pltu()` (e.g. with `test_codecs_allocate_nothing()` and `test_decode_ignores_trailing()`) actually correct?**
  _`encode_pltu()` has 11 INFERRED edges - model-reasoned connections that need verification._
- **Are the 10 inferred relationships involving `decode_pltu()` (e.g. with `test_codecs_allocate_nothing()` and `test_decode_ignores_trailing()`) actually correct?**
  _`decode_pltu()` has 10 INFERRED edges - model-reasoned connections that need verification._
- **Are the 4 inferred relationships involving `crc32()` (e.g. with `test_asm_in_crc()` and `test_codecs_allocate_nothing()`) actually correct?**
  _`crc32()` has 4 INFERRED edges - model-reasoned connections that need verification._
- **What connects `frame`, `Starcom stack`, `Management Information Base` to the rest of the system?**
  _28 weakly-connected nodes found - possible documentation gaps or missing edges._