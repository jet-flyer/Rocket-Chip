# FPGA / space-electronics references

This shelf is FPGA / space-electronics references for Rocket Chip. Mission-style rename of `docs/hardware/FPGA/` is TBD — do not invent a mission name. **Not a cert kit.**

Index only. Do **not** copy PDFs here. NASA/ESA handbooks already live in `standards/starcom/fpga/`. CCSDS Blue Books stay under `standards/starcom/ccsds/` (comms, not FPGA form-factor).

## In-tree PDFs (`standards/starcom/fpga/`)

Point at the existing files; do not duplicate them.

| Document | Path |
|----------|------|
| NASA-HDBK-4011 VHDL Style Handbook | [`standards/starcom/fpga/NASA-HDBK-4011.pdf`](../../../../standards/starcom/fpga/NASA-HDBK-4011.pdf) |
| NASA-HDBK-4008 PLD Handbook | [`standards/starcom/fpga/NASA-HDBK-4008.pdf`](../../../../standards/starcom/fpga/NASA-HDBK-4008.pdf) |
| ESA ASIC/001 VHDL Modelling Guidelines | [`standards/starcom/fpga/ESA-ASIC-001.pdf`](../../../../standards/starcom/fpga/ESA-ASIC-001.pdf) |
| GSFC 500-PG-8700.2.7 Design of Space Flight FPGAs | [`standards/starcom/fpga/GSFC-500-PG-8700.2.7.pdf`](../../../../standards/starcom/fpga/GSFC-500-PG-8700.2.7.pdf) |

Catalog, provenance, and why we have them: [`standards/starcom/README.md`](../../../../standards/starcom/README.md) and [`standards/starcom/fpga/README.md`](../../../../standards/starcom/fpga/README.md).

## URL-only (gated; no PDF in tree)

Already listed in `standards/starcom/README.md`. Do not scrape ECSS logins.

| Document | URL |
|----------|-----|
| ECSS-E-ST-20-40C ASIC, FPGA and IP Core engineering (11 October 2023) | <https://ecss.nl/standard/ecss-e-st-20-40c-asic-fpga-and-ip-core-engineering-11-october-2023/> |
| ECSS-E-HB-20-40A radiation-effects mitigation handbook (11 October 2023) | <https://ecss.nl/home/ecss-e-hb-20-40a-engineering-techniques-for-radiation-effects-mitigation-in-asics-and-fpgas-handbook/> |

## Digilent Pmod

Current is **1.3.1** (Digilent, 2020-10-28). We do not redistribute the PDF.

- Spec page: <https://digilent.com/reference/pmod/specification>
- 1.3.1 PDF: <https://digilent.com/reference/_media/reference/pmod/pmod-interface-specification-1_3_1.pdf>
- 1.3.0 PDF (older filename still published): <https://digilent.com/reference/_media/reference/pmod/pmod-interface-specification-1_3_0.pdf>

Our note: [`PMOD.md`](PMOD.md)

## CCSDS

Blue Books stay under [`standards/starcom/ccsds/`](../../../../standards/starcom/ccsds/). Comms, not FPGA form-factor. Do not relocate them here.
