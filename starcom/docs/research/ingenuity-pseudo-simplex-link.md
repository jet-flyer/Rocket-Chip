# Ingenuity link and pseudo-simplex main mode

Session notes from 2026-10-05, moved off the whiteboard on 2026-11-09. Research only. Nothing here is a product decision. Do not adopt this radio or this protocol from this note. Do not write `starcom/docs/DESIGN.md` from this file.

## Two flight modes (not started)

Passive rocket mode uses Proximity-1 simplex (211.0-B-6 §6.4.4). No hail. One direction.

The main mode may be pseudo-simplex. That name is ours. The pattern is the IRIG 106 rocket pattern: one-way telemetry in flight, two-way on the ground, one fixed pair, settings already known. This note does not select an IRIG 106 waveform. It does not replace the Proximity-1 frames.

Look next at the Ingenuity–Perseverance link. It is a fixed pair with that on/off pattern. It is not Proximity-1 and not the rover's Electra relay.

## Known so far (not a design)

- Two identical COTS IEEE 802.15.4 radios, SiFlex 02 (LS Research), about 900 MHz (reported near 914 MHz). One is on the helicopter. One is the Mars Helicopter Base Station on Perseverance. The computers talk to the radios over UART.
- Balaram et al., AIAA 2018-0023, section F: one-way data during the short flights, at 20 kbit/s or 250 kbit/s, design range about 1 km. A secure two-way mode when landed. NTRS 20190026832. DOI 10.2514/6.2018-0023.
- N5BF, "Mars Helicopter Telecom," EME 2024 slides (`https://www.ok2kkw.com/eme/Trenton/papers/N5BF-Mars_Helicopter_Telecom_EME2024.pdf`): the protocol was modified from Zigbee and adapted to a two-point high-throughput link. Packet or ack loss triggers retries. Flight software driver name on those slides is ISF. Helicopter-side telecom mass about 13 g. TX under 3 W. RX under 0.2 W.
- Chahat et al., IEEE Antennas and Propagation Magazine, Dec 2020, "The Mars Helicopter Telecommunication Link." Antennas and the link budget. Not the protocol.

## Still unread

Open these before any mode text is written:

- The actual frame, ack, and retry rules of the modified 802.15.4 stack.
- Whether flight "one-way" is the radio receiving off, or the same radio with data in one direction.
- Who starts a contact, and whether the radios are powered only at scheduled times.
- What of that is useful for a fixed pad-to-rocket pair, and what is Zigbee-only.
