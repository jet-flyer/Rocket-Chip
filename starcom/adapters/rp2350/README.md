# RP2350 adapters

Generic SPI/GPIO radio port (`BusOps` + `radio_bus_shift_tx` / `radio_bus_shift_rx`) and PIO-shaped PLTU bit pipe (`pio_shift_out` / `pio_shift_in`). Host tests use a fake bus / fake PIO. Board pins, SX1276 / RFM types, AO, and `hardware/pio.h` stay in the consumer. Not 211.1 residual-carrier PM. PHY tiers are increment 18.
