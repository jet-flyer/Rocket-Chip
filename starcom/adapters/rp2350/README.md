# RP2350 adapters

Generic SPI/GPIO radio port, PIO-shaped PLTU bit pipe, and PHY tier declaration (`PhyDecl`). Host tests use a fake bus / fake PIO / uncoded PLTU. Board pins, SX1276 / RFM types, AO, `hardware/pio.h`, and FPGA bitstreams stay out. `PhyTier::compliant` is not offered. Convolutional / LDPC is increment 19.
