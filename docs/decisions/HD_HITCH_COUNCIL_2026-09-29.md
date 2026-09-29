# Half-duplex hitch — council record

**Date:** 2026-09-29
**Panel:** JPL, professor, cubesat, rocketeer. The user asked for this set only, so each persona had more room. An earlier pass that also included ArduPilot and Capstone was restarted and is not this decision.
**Source:** The persona text below is the session record of that panel. It is not a later rewrite.

The user accepted the consensus, then corrected two points. Those corrections are the standing direction.

## Corrections after the panel

The 1.2 s rhythm is not a target. No perceptible hitch is the goal. A hitch every 1.2 s is not something to keep. The user had called that cadence acceptable only to point at the length of the hole as the real defect.

Command lockout is not required if the hitch goes away. Simplex, a second radio, and full duplex stay fallbacks for after a correct half-duplex turn is measured and is still too visible. Do not build the countdown unless that measurement says the turn itself cannot be made short enough.

## What the panel was answering

Figure 6-2 (book page 6-26) is the half-duplex machine. The rows are Table 6-10, with COMM_CHANGE in 6-11 and termination in 6-12. Full duplex is Figure 6-1 and Tables 6-7 through 6-9. S is a state. E is an event. The code follows the row, including its actions. A label on the drawing does not override the row. MIB times are local and are not in the figure.

The hitch is that turn running long. A paper turn is about one station packet, near 100–200 ms. The bench hole was about half a second.

## Panel

**JPL.** Implement Table 6-10 one for one, which is Figure 6-2. Leave Figure 6-1 out of this radio. Full duplex is the machine whose data state keeps the receiver on while the transmitter is on. Half duplex is the machine that turns the receiver off to send. Mixing the arrows is how you get a token that nobody accepts. The three broken rows are a missed-token failure, and the book already named the backup: both transmitters come on. Fix the rows, size the listen window to the contact the table actually runs, and time one yield on the pad. Pass is a hole of about one station packet, repeating every 1.2 s, with COP-P still locked. The 1.2 s rhythm can stay. A half-second hole cannot. The command-lockout clock is a flight-director and pad procedure sitting on top of that machine. It is not a new MAC state. At expiry, reject ordinary commands and keep the abort path on the token. A lockout that also kills the uplink has no cutoff, and that is the failure mode the Shuttle count was built to avoid.

**Professor.** S and E are the state and the event in one transition function. S50 on E38 goes to S50 with persistence set. S60 on E49 goes to S51. The number spaces look similar and they are not two protocols. The 30s and 40s belong to full duplex. The 50s and 60s, and events from E29 up, belong to half duplex. One for one means those rows, including the actions, not a redraw of page 6-26 from memory. The code's extra E38 behavior, "if a status report is owed, grant another whole send," is a transition the table does not contain. Delete it. The token has to be tested after carrier lock has moved S62 to S61, because that is the only order in which the first frame of a contact can be E49. The idle pattern is the action of S51 and S52, so those states remain even though this modem cannot radiate the PN sequence. Shorten the timers to the preamble. Do not delete the states to flatter the chip.

**Cubesat.** The cadence is command latency. 1.2 s to get a pad command through is acceptable on a vehicle whose flight termination is onboard. The duration is lost telemetry. A correct turn is one status packet and a token each way, about 100–200 ms, and the average stays near 9 Hz. That is worth doing before anyone buys a second radio. A second SX1276 is a few grams and it does remove the hole, and it is the step after a pad measurement still shows half a second. FSK on this same chip shortens each packet and spends range to do it. It does not repair E38. Leave it alone until the table matches the air.

**Rocketeer.** I want the call the count already uses. Something on the pad that says "command lockout in 30 seconds" and keeps saying it, and that I can hold or abort. Thirty seconds is long enough to hear and long enough to hit the button. Arm is not that call. I still disarm from the pad after arm. When the count expires I expect the radio to keep working, because abort has to get to the rocket until the motor is burning, and after that the onboard termination is what I trust. Do not go silent to make the horizon pretty during the count. Make each blink short. If we ever stop turning the radio around, do it after it has left the rail, and turn the token back on when it is on the ground so I can safe it in the field.

**Consensus, as accepted that day.** Keep half duplex. Make the data-services rows of Table 6-10 match the book, and shorten the hitch to about one station packet. Prove that on the pad. The user also asked for the announced lockout countdown. The later correction above withdraws that until a correct turn is still too visible. Simplex is not that procedure.

## Where that stands

Commit `40b83fc` puts the row fixes on `main`. E38 sets persistence and does not reload Send_Duration. A pass token that is the first frame in S62 takes carrier lock, then E49. E42 does not count a failed pass. E50 does. Vehicle receive covers one station contact. Station receive and the carrier-loss hold cover one vehicle contact. Vehicle send stays 1200 ms. S51, S52, and S58 still put no octets on the air. That stays in the adapter, because the SX1276 cannot radiate the 211.2 idle pattern.

Pad, same tree, station glass: 11.4 s, 7.45 Hz, median gap 154 ms, repeated holes about 616–618 ms. The line before each hole showed MAC A/s50 or s51. The next nav line showed MAC A/s60. COP-P stayed up. The hitch is not gone. The station send timer is 65 ms of `now_ms()`, so those holes are not that timer counted in 10 ms ticks.

Opening-shock is a separate uncommitted set and is not part of this decision.
