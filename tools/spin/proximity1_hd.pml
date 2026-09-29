/*
 * proximity1_hd.pml — 211.0-B-6 Figure 6-2 / Table 6-10, simplified
 *
 * One half-duplex session after hail. Two peers. Discrete ticks.
 * Rows coded here: E38 E39 E40 E41 E42 E43 E44 E45 E46 E47 E48 E49 E50.
 * Hail (E29–E37), COMM_CHANGE (table 6-11), and termination (table 6-12)
 * are the same variables in mac.cpp and are not this file.
 *
 * Snn is a state. Enn is the event that leaves it. Full duplex is Figure
 * 6-1 and Tables 6-7..6-9 (S40 and neighbors, receiver stays on). Those
 * states are not in this model.
 *
 * This file does not replace starcom/src/ccsds/mac.cpp. It is the book
 * machine to diff against that file. Three flight behaviors are not rows:
 *   1. mac.cpp E38 reloads Send_Duration while NEED_PLCW is set
 *      (macWaitExpiredHalf, S50). Compile -DCODE_E38 to model that.
 *   2. pump_handle_air calls macOnToken before carrier lock, so a token
 *      that arrives in S62 is not E49. This model does E47 then E49.
 *   3. S51 and S52 are timed and the SX1276 radiates nothing. Compile
 *      -DCODE_MISS: those states emit nothing, and Receive_Duration is
 *      that one silent tick, so E50 runs before the first frame while
 *      the station Send_Duration is still going. -DSHORT_WINDOW keeps
 *      the radiation, so the row taken is E44, then E49.
 *
 * A zero-length silence is not a row. The table turns the radio around.
 * p_gap_bounded says the silence is one peer ceremony, not a second send.
 * How often the turn happens is Send_Duration, a MIB value, not a goal.
 *
 * The count (seq_*) is the pad procedure above the MAC: its own clock,
 * hold, abort/cutoff, then ordinary commands refused. Abort stays open.
 * Duplex stays half. It is not simplex and it is not the flight T- clock.
 *
 * Run: tools/spin/run_hd_book_spin.sh
 */

#define S50 50
#define S51 51
#define S52 52
#define S56 56
#define S58 58
#define S60 60
#define S61 61
#define S62 62

#define NONE  0
#define TOKEN 1
#define PLCW  2
#define SDU   3
#define TONE  4

#ifdef CODE_MISS
/* Silent open. The listen is that one tick, so it ends with no carrier. */
#define V_RECV 1
#define S_SEND 4
#elif defined(SHORT_WINDOW)
/* Peer still radiates. E44 reloads; E48 stays unused. */
#define V_RECV 2
#define S_SEND 1
#else
#define V_RECV 12
#define S_SEND 1
#endif
#define S_RECV 12
#define V_SEND 4
#define MAX_GAP 14
#define HORIZON 24

#define SEQ_IDLE  0
#define SEQ_COUNT 1
#define SEQ_HOLD  2
#define SEQ_LOCK  3
#define SEQ_ABORT 4

byte vst = S51
byte sst = S60
byte vwait = 1
byte swait = S_RECV
bool varmed = true
bool sarmed = true
bool vloaded = true
bool sloaded = false
bool vtx = true
bool stx = false
bool vpers = false
bool spers = false
bool vneed = true
bool sneed = true
bool vcarrier = false
bool scarrier = false
byte vgap = 0
byte turns = 0

byte seq = SEQ_COUNT
byte seq_left = 3
bool ordinary_ok = true
bool abort_open = true
bool half = true
bool saw_user = false

inline emit(st, pers, need, kind) {
	kind = NONE
	if
	:: st == S56 -> kind = TOKEN
	:: st == S50 && need -> kind = PLCW
	:: st == S50 && !pers && !need -> kind = SDU
#ifdef CODE_MISS
	:: st == S58 -> kind = TONE
	:: st == S51 || st == S52 -> kind = NONE
#else
	:: st == S51 || st == S52 || st == S58 -> kind = TONE
#endif
	:: else -> kind = NONE
	fi
}

/* Token: E47 before E49, so a token in S62 still hands off. */
inline on_token(st, wait, loaded, tx) {
	if
	:: st == S62 -> st = S61
	:: else -> skip
	fi
	if
	:: st == S60 || st == S61 ->
		st = S51
		wait = 1
		loaded = true
		tx = true
	:: else -> skip
	fi
}

inline on_frame(st) {
	if
	:: st == S62 -> st = S60   /* E47 then E46 */
	:: st == S61 -> st = S60   /* E46 */
	:: else -> skip
	fi
}

inline on_tone(st) {
	if
	:: st == S62 -> st = S61   /* E47, no frame yet */
	:: else -> skip
	fi
}

/* E38 sets persistence. It does not grant another Send_Duration. */
inline e38(st, pers, need, wait, loaded, send) {
	pers = true
#ifdef CODE_E38
	if
	:: need ->
		wait = send
		loaded = true
		pers = false
	:: else ->
		st = S56
	fi
#else
	if
	:: !need -> st = S56
	:: else -> skip
	fi
#endif
}

inline expire(st, pers, need, wait, armed, loaded, tx, send, recv) {
	if
	:: armed && !loaded && wait > 0 -> wait = wait - 1
	:: else -> skip
	fi
	if
	:: armed && !loaded && wait == 0 ->
		armed = false
		if
		:: st == S51 ->          /* E40 */
			st = S52
			wait = 1
			loaded = true
			armed = true
		:: st == S52 ->          /* E41 */
			st = S50
			wait = send
			loaded = true
			armed = true
		:: st == S50 ->          /* E38, then E39 when the PLCW is done */
			e38(st, pers, need, wait, loaded, send)
		:: st == S58 ->          /* E43 */
			st = S62
			pers = false
			tx = false
			wait = recv
			loaded = true
			armed = true
			turns = turns + 1
		:: else -> skip
		fi
	:: else -> skip
	fi
}

/* Receive-timer rows need the peer's carrier, so they are not in expire().
   E44 and E45 reload this peer's Receive_Duration. */
inline expire_recv(st, carrier, wait, armed, loaded, tx, recv) {
	if
	:: armed && !loaded && wait > 0 && (st == S60 || st == S61 || st == S62) ->
		wait = wait - 1
	:: else -> skip
	fi
	if
	:: armed && !loaded && wait == 0 && (st == S60 || st == S61 || st == S62) ->
		armed = false
		if
		:: st == S60 && carrier ->   /* E44 */
			wait = recv
			loaded = true
			armed = true
		:: st == S60 && !carrier ->  /* E48 missed token */
			st = S51
			wait = 1
			loaded = true
			armed = true
			tx = true
		:: st == S61 && carrier ->   /* E45 */
			wait = recv
			loaded = true
			armed = true
		:: (st == S61 || st == S62) && !carrier ->  /* E50 */
			st = S51
			wait = 1
			loaded = true
			armed = true
			tx = true
		:: else -> skip
		fi
	:: else -> skip
	fi
}

active proctype session() {
	byte vem
	byte sem
	byte n

	n = 0
	do
	:: n >= HORIZON -> break
	:: n < HORIZON ->
		atomic {
			/* A wait loaded last tick starts counting now. A wait loaded
			   by the event below (E49, the token's own tail) does not. */
			vloaded = false
			sloaded = false
			/* Book: persistence blocks user data, so the token follows. */
			if
			:: vst == S50 && vpers && !vneed -> vst = S56
			:: else -> skip
			fi
			if
			:: sst == S50 && spers && !sneed -> sst = S56
			:: else -> skip
			fi

			emit(vst, vpers, vneed, vem)
			emit(sst, spers, sneed, sem)
			assert(!(vem == TOKEN && sem == TOKEN))

			if
			:: vem == TOKEN -> on_token(sst, swait, sloaded, stx); scarrier = true
			:: vem == PLCW || vem == SDU -> on_frame(sst); scarrier = true
			:: vem == TONE -> on_tone(sst); scarrier = true
			:: else -> scarrier = false
			fi
			if
			:: sem == TOKEN -> on_token(vst, vwait, vloaded, vtx); vcarrier = true
			:: sem == PLCW || sem == SDU -> on_frame(vst); vcarrier = true
			:: sem == TONE -> on_tone(vst); vcarrier = true
			:: else -> vcarrier = false
			fi

			if
			:: vem == TOKEN ->
				vst = S58
				vwait = 1
				vloaded = true
				varmed = true
			:: vem == PLCW -> vneed = false
			:: else -> skip
			fi
			if
			:: sem == TOKEN ->
				sst = S58
				swait = 1
				sloaded = true
				sarmed = true
			:: sem == PLCW -> sneed = false
			:: else -> skip
			fi

			/* Guards must be exclusive. Promela if takes any true guard. */
			if
			:: vem == SDU ->
				vgap = 0
				saw_user = true
			:: else ->
				if
				:: vgap < 255 -> vgap = vgap + 1
				:: else -> assert(false)
				fi
			fi

			/* Send-column timers. Receive column is expire_recv. */
			if
			:: vst == S60 || vst == S61 || vst == S62 ->
				expire_recv(vst, vcarrier, vwait, varmed, vloaded, vtx, V_RECV)
			:: else ->
				expire(vst, vpers, vneed, vwait, varmed, vloaded, vtx, V_SEND, V_RECV)
			fi
			if
			:: sst == S60 || sst == S61 || sst == S62 ->
				expire_recv(sst, scarrier, swait, sarmed, sloaded, stx, S_RECV)
			:: else ->
				expire(sst, spers, sneed, swait, sarmed, sloaded, stx, S_SEND, S_RECV)
			fi

			/* Pad count. Not the flight T- clock. Not a MAC event. */
			if
			:: seq == SEQ_COUNT ->
				if
				:: seq_left > 1 -> seq = SEQ_HOLD
				:: seq = SEQ_ABORT
					ordinary_ok = true
					abort_open = true
					half = true
				:: seq_left = seq_left - 1
					if
					:: seq_left == 0 ->
						seq = SEQ_LOCK
						ordinary_ok = false
						abort_open = true
						half = true
					:: else -> skip
					fi
				fi
			:: seq == SEQ_HOLD ->
				if
				:: seq = SEQ_COUNT
				:: seq = SEQ_ABORT
					ordinary_ok = true
					abort_open = true
					half = true
				:: skip
				fi
			:: else -> skip
			fi

			n = n + 1
		}
	od
	assert(saw_user)
	assert(half)
}

ltl p_no_dual_s50 { [] !(vst == S50 && sst == S50) }
ltl p_gap_bounded { [] (vgap <= MAX_GAP) }
ltl p_lock_keeps_abort {
	[] (seq == SEQ_LOCK -> (abort_open && half && !ordinary_ok))
}
ltl p_hold_not_commit { [] (seq == SEQ_HOLD -> ordinary_ok) }
ltl p_abort_reopens {
	[] (seq == SEQ_ABORT -> (ordinary_ok && abort_open && half))
}
