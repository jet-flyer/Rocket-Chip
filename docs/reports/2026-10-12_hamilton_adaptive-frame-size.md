# Adaptive frame size on the Starcom link: books and prior art

- **Date:** 2026-10-12
- **Author:** Hamilton (compiled from Duke, Goddard and Buzz in the Rocket Chip group chat)
- **Scope:** Whether Starcom can change Proximity-1 frame length during a session as link quality changes, what the CCSDS books say, how other systems do it, and what the SX1276 radio allows. Post-pivot work (Nathan, 2026-10-12).
- **Status:** Frozen once committed. A later report replaces this one.

**[Fact]** means a quoted or cited source says it. **[Inference]** means the named agent's judgment; not checked on hardware.

## 1. Answer
- **[Fact]** No CCSDS book forbids changing frame length during a session, and none gives a rule for choosing it (Duke).
- **[Inference]** It fits as a small sender-side feature: keep MIB `Maximum_Frame_Length` fixed, vary the size actually sent below it, choose it from COP-P ACK and CRC-fail history, and resend queued frames unchanged (Duke, Goddard).

## 2. CCSDS books (Duke)
- **[Fact]** 211.0-B-6 §2.1.6: the PLTU "is flexibly sized to fit its data content, that is, a variable-length frame containing variable-length packets". §3.2.3.1 b): the data field shall "be of variable length". §3.2.2.10.2: every frame header carries a length count, 5 to 2048 octets.
- **[Fact]** 211.0-B-6 §3.2.3.1 c): data field at most `Maximum_Frame_Length` less 5 octets. §4.4.2.2 b): frames are built "using the value of the MIB parameters Maximum_Packet_Size and Maximum_Frame_Length".
- **[Fact]** 211.0-B-6 Annex C, `Maximum_Frame_Length`: "Link efficiency at various data rates may require varying frame lengths." Not tagged "Session static". No directive sets it over the link; it is not a SET TRANSMITTER, SET RECEIVER or SET PL EXTENSIONS parameter. PICS DLL-167 makes it mandatory, so the plan must state our value.
- **[Fact]** 211.0-B-6 §7.2.3.1.7 defines "Resend SEQ (Sequence Controlled) Frame"; note 3 under the FOP-P table: a "Progressive Retransmission" resends "the frames on the Sent queue".
- **[Fact]** 211.2-B-3 §3.2 Note 1: "The length of a PLTU depends on the length of the Transfer Frame it contains". §3.6.4 a): the receiver "shall use the Proximity-1 Frame Length Field … to locate the position of the CRC-32 field".
- **[Fact]** 210.0-G-2 (Green Book) §3.3.3.2.1: "The frame size is variable as long as it is smaller than the maximum allowed on the link." The configuration-control list there leaves out frame length. §2 c)4): asymmetric traffic can use "different forward and return data rates, frame lengths, and go-back-n values". §4.2.4.3: frame length is a user-set throughput parameter; pp. 4-37: "the user frame size on the slowest link needs to be reduced"; Figs 4-15 to 4-17.
- **[Fact]** 732.1-B-3 (USLP) §5.1: managed parameters "tend to be static for long periods of time" (general USLP text, not a Prox-1 rule).
- **[Fact]** Nothing found in 211.1-B-4; no book says what happens to queued frames when size changes. Lunar drafts not checked.
- Sources: [210.0-G-2](https://public.ccsds.org/Pubs/210x0g2e1.pdf) (URL not checked), [211.0-B-6](https://public.ccsds.org/Pubs/211x0b6e1.pdf), [211.2-B-3](https://public.ccsds.org/Pubs/211x2b3.pdf), [732.1-B-3](https://public.ccsds.org/Pubs/732x1b3e1.pdf).

## 3. Prior art outside CCSDS (Goddard)
- **[Fact]** Modiano 1999, Wireless Networks 5(4):279-286, https://doi.org/10.1023/a:1019111430288: sender picks size from ACK/resend history. Two-size threshold variant: https://opensiuc.lib.siu.edu/cgi/viewcontent.cgi?article=1071&context=ece_confs&httpsredir=1
- **[Fact]** Martins & Alves 1990, IEEE Trans. Commun. 38(6):737-739, https://doi.org/10.1109/26.57462. Yao 1995, IEEE Trans. Commun. 43(1):20-23, https://doi.org/10.1109/26.385946.
- **[Fact]** Adaptive Go-Back-N on Gilbert-Elliott channels (adapt copies, not length; threshold logic carries over): Lu et al., https://www.ece.uvic.ca/~wslu/Publications/Lu-Journal/Paper48.pdf; Chakraborty et al. 2005, https://www.sciencedirect.com/science/article/abs/pii/S1389128604003263; Annamalai et al. PIMRC 1997, https://doi.org/10.1109/pimrc.1997.631038.
- **[Fact]** DPLC (802.15.4/TinyOS): with a hidden interferer, loss rose but RSSI/LQI did not show it, so it estimates link quality from reception statistics. https://tns.thss.tsinghua.edu.cn/sun/publications/2010.15.pdf, https://tns.thss.tsinghua.edu.cn/sun/publications/2014.2.pdf
- **[Fact]** Adaptive frame size predictor, IEEE JSAC 2005, https://dl.acm.org/doi/10.1109/JSAC.2004.839400. DRACER, INFOCOM 2008: "the success of a packet is a good indicator of the next packet's success", https://people.eecs.berkeley.edu/~pister/publications/2008/Lanzisera%20Infocomm08.pdf
- **[Fact]** 802.11-2012: "If a fragment requires retransmission, its frame body content and length shall remain fixed for the lifetime of the MSDU" (quoted at https://mrncciew.com/2014/11/03/cwap-802-11-fragmentation/).
- **[Fact]** Linux minstrel_ht: A-MSDU cap 500/1600/3200 bytes by rate and 75% success threshold; cap held "to avoid the complexity of having to de-aggregate packets in the queue". https://wireless.docs.kernel.org/en/latest/en/developers/documentation/mac80211/ratecontrol/minstrel.html
- **[Fact]** LTE HARQ resends the same transport block size (TS 36.213 Table 7.1.7.1-1; TS 36.321 §5.3.2.2 Note 2). RLC AM re-segments with extra header fields (TS 36.322 §5.2.1, https://www.etsi.org/deliver/etsi_ts/136300_136399/136322/09.00.00_60/ts_136322v090000p.pdf).
- **[Fact]** BLE Data Length Extension: 27 to 251 bytes, negotiated, not quality-driven (https://punchthrough.com/maximizing-ble-throughput-part-3-data-length-extension-dle-2/).
- **[Fact]** LoRaWAN US902-928 uplink max payload DR0 19, DR1 61, DR2 133, DR3-4 230 bytes (RP002-1.0.5; https://www.thethingsnetwork.org/docs/lorawan/regional-parameters/us915/).
- **[Fact]** AX.25 PACLEN negotiable in v2.2; smaller frames on noisy links; at 9600 bps PACLEN 128 to 256 raised efficiency 10% to 20% (WB2OSZ, https://raw.githubusercontent.com/wb2osz/direwolf/dev/doc/Why-is-9600-only-twice-as-fast-as-1200.pdf).
- **[Fact]** 802.15.4g SUN FSK PSDU up to 2047 octets; no length-adaptation guidance found. No smallsat found that varies frame length.

## 4. Radio limits (Buzz, SX1276 Rev 6 §4.2.13, not board-checked)
- **[Fact]** 64-byte FIFO. Variable-length packet mode: up to 255 bytes, length byte after sync word, so size can change per frame with no register writes; costs one byte duplicating the Prox-1 length field.
- **[Fact]** Fixed-length mode: up to 2047 bytes, but the receiver must read the Prox-1 length mid-frame and set PayloadLength, plus FIFO streaming: new driver work.

## 5. Design direction (all [Inference])
- Keep `Maximum_Frame_Length` fixed for the session; vary only the sent size below it (Duke).
- Set size when a frame is built; resend Sent-queue frames unchanged; never re-split (Duke, Goddard).
- 2-3 steps under 255 bytes, e.g. 69 (current nav), about 128, about 250, in variable-length mode (Goddard, Buzz).
- Drive from ACK and CRC-fail history, RSSI secondary; drop fast, climb slowly (Goddard).
- Best length estimate \( L \approx \sqrt{H/p} \): about 140 bytes at p = 1e-3 and 450 at 1e-4, assuming 20 bytes overhead (Goddard). Weak links land under 255, so long-frame driver work waits for data.
- If a fixed-block code is added, size steps should be codeblock multiples (Goddard).

## 6. Open items
- Bench: frame error rate vs frame size, sub-255 first (raise the 128-byte driver cap).
- Pick our `Maximum_Frame_Length` value for the PICS.
- Check the lunar Prox-1 drafts.
