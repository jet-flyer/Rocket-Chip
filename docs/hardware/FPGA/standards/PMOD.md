# Digilent Pmod

Connector / protocol note for Rocket Chip FPGA boards. Spec is Digilent Pmod Interface Specification **1.3.1** (current; 2020-10-28). We do not redistribute the PDF.

- Spec: <https://digilent.com/reference/pmod/specification>
- 1.3.1 PDF: <https://digilent.com/reference/_media/reference/pmod/pmod-interface-specification-1_3_1.pdf>
- 1.3.0 PDF (older filename still published): <https://digilent.com/reference/_media/reference/pmod/pmod-interface-specification-1_3_0.pdf>

Not a cert kit. Not for the Core stamp T8.

## Mechanical

0.1" headers.

- **6-pin (1x6):** 4 I/O + GND + VCC.
- **12-pin (2x6):** two stacked 6-pin = 8 I/O + 2 GND + 2 VCC. Row spacing is 0.1" **between** the two rows (this is why UPduino DIP is not a PMOD).

Host vs peripheral pin numbering is mirrored / non-standard IDC. Easy to reverse.

## Electrical

3.3 V LVCMOS/LVTTL. ~100 mA assumed per port (spec: do not assume more than approximately 100 mA). Not impedance-controlled; not for SERDES/video.

Protocol types on the same 8 I/O: GPIO, SPI (CS MOSI MISO SCK), UART, I2C, I2S, H-bridge. That is why FPGA boards use it: bitstream picks the protocol; MCU boards usually hard-assign SPI vs UART.

## Boards in play

### pico2-ice

<https://pico2-ice.tinyvision.ai/md_pmods.html> — 2 rows x 6 columns; 4 ports:

- ICE PMOD A (iCE40 only)
- ICE PMOD B (iCE40 only)
- RP-ICE PMOD (both)
- RP PMOD (RP2350 only)

RP pinout arranged SPI-compatible; other protocols PIO / bit-bang. Solder example: female 2x6 right-angle. Tindie SKU is soldered vs not (4 headers). Getting started photo: `pico_ice_pmod_install.jpg`.

### Galaxia

Ten real Digilent 2x6 90 deg PMODs.

### UPduino

**Not** a 2x6. Pin-order 1x6 on DIP (27, 26, 25, 23, GND, 3V3).

### Forgix / Teensy / Feather

Not PMOD.

## Why we might add PMODs later

If a larger FPGA / bench carrier (not the rocket stamp) needs a universal 8-I/O expansion. One Pmod is 8 I/O, not a Feather pinout. Not for the Core stamp T8.

Settled policy: PIO first; T8 not on base RC for comms; T8 gated on stamp; Forgix eval; UVLO analog; no ESKF on FPGA.
