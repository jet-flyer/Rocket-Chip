#include "starcom/ccsds/mac.hpp"

namespace starcom::ccsds {
namespace {

void loadWait(MacSession& m, Tick dur) noexcept {
  m.wait_left = dur;
  m.wait_armed = dur != 0;
}

void clearWait(MacSession& m) noexcept {
  m.wait_left = 0;
  m.wait_armed = false;
}

void notify(MacSession& m, MacNotify n) noexcept { m.notify = n; }

void applyTable66(MacSession& m) noexcept {
  m.transmit_on = false;
  m.modulation = false;
  m.persistence = false;
  m.ss = 0;
  m.x = 0;
  m.y = 0;
  m.z = 0;
  clearWait(m);
  m.carrier_loss_left = 0;
  m.plcw_left = 0;
  m.hail_life_left = 0;
  m.resync_wait_left = 0;
  m.resync_life_left = 0;
  m.mac_frame_pending = false;
  m.mac_queue_len = 0;
  m.token_fail_n = 0;
  m.fifo_empty = true;
  m.need_plcw = true;
  m.need_status_report = true;
}

struct StateFx {
  MacState state;
  MacMode mode;
  bool tx;
  std::uint8_t ss;
};

constexpr StateFx kStateFx[] = {
    {MacState::s1, MacMode::inactive, false, 0},
    {MacState::s2, MacMode::connecting_l, false, 0},
    {MacState::s80, MacMode::active, false, 0},
    {MacState::s31, MacMode::connecting_t, true, 1},
    {MacState::s11, MacMode::connecting_t, true, 1},
    {MacState::s32, MacMode::connecting_t, true, 2},
    {MacState::s12, MacMode::connecting_t, true, 2},
    {MacState::s33, MacMode::connecting_t, true, 3},
    {MacState::s13, MacMode::connecting_t, true, 3},
    {MacState::s34, MacMode::connecting_t, true, 4},
    {MacState::s14, MacMode::connecting_t, true, 4},
    {MacState::s35, MacMode::connecting_t, false, 5},
    {MacState::s36, MacMode::connecting_t, false, 5},
    {MacState::s41, MacMode::active, true, 1},
    {MacState::s51, MacMode::active, true, 1},
    {MacState::s42, MacMode::active, true, 2},
    {MacState::s52, MacMode::active, true, 2},
    {MacState::s40, MacMode::active, true, 0},
    {MacState::s50, MacMode::active, true, 0},
    {MacState::s48, MacMode::active, true, 6},
    {MacState::s56, MacMode::active, true, 6},
    {MacState::s45, MacMode::active, true, 4},
    {MacState::s58, MacMode::active, true, 4},
    {MacState::s54, MacMode::active, true, 3},
    {MacState::s55, MacMode::active, true, 7},
    {MacState::s60, MacMode::active, false, 0},
    {MacState::s61, MacMode::active, false, 1},
    {MacState::s62, MacMode::active, false, 2},
};

void applyState(MacSession& m, MacState s) noexcept {
  m.state = s;
  if (s == MacState::s71) {
    m.duplex = MacDuplex::simplex_transmit;
    m.mode = MacMode::active;
    m.transmit_on = true;
    m.ss = 0;
    return;
  }
  if (s == MacState::s72) {
    m.duplex = MacDuplex::simplex_receive;
    m.mode = MacMode::active;
    m.transmit_on = false;
    m.ss = 0;
    return;
  }
  for (StateFx const& row : kStateFx) {
    if (row.state == s) {
      m.mode = row.mode;
      m.transmit_on = row.tx;
      m.ss = row.ss;
      return;
    }
  }
}

void startHailLife(MacSession& m) noexcept {
  m.hail_life_left = m.mib.hail_lifetime;
}

void queueSetVr(MacSession& m) noexcept {
  if (m.copp == nullptr) {
    return;
  }
  const auto n = encodeSetVr(m.mac_queue, m.copp->fop.nn_r, m.copp->pcid);
  if (!n) {
    return;
  }
  m.mac_queue_len = *n;
  m.mac_frame_pending = true;
  m.persistence = true;
}

void enterS1(MacSession& m, Tick /*now*/, bool end) noexcept {
  applyTable66(m);
  applyState(m, MacState::s1);
  if (m.copp != nullptr) {
    farmPInit(m.copp->farm);
    fopPInit(m.copp->fop, m.copp->fop.mib);
  }
  if (end) {
    notify(m, MacNotify::end_session);
  }
}

bool tickDec(Tick& t, Tick dt) noexcept {
  if (t == 0 || dt == 0) {
    return false;
  }
  if (dt >= t) {
    t = 0;
    return true;
  }
  t = static_cast<Tick>(t - dt);
  return t == 0;
}

void macTickPlcw(MacSession& m, Tick dt) noexcept {
  if (m.plcw_left != 0 && tickDec(m.plcw_left, dt)) {
    m.need_plcw = true;
    m.plcw_left = m.mib.plcw_repeat_interval;
  }
}

bool macTickHail(MacSession& m, Tick now, Tick dt) noexcept {
  if (m.hail_life_left != 0 && tickDec(m.hail_life_left, dt)) {
    notify(m, MacNotify::hail_fail);
    enterS1(m, now, true);
    return true;
  }
  return false;
}

void macTickResync(MacSession& m, Tick dt) noexcept {
  if (m.resync_life_left != 0 && tickDec(m.resync_life_left, dt)) {
    notify(m, MacNotify::resync_fail);
    m.persistence = false;
    m.mac_frame_pending = false;
    if (m.copp != nullptr) {
      m.copp->fop.resync = false;
      m.copp->fop.state = FopPState::s1_active;
    }
  }
  if (m.resync_wait_left != 0 && tickDec(m.resync_wait_left, dt) &&
      m.copp != nullptr && m.copp->fop.resync) {
    queueSetVr(m);
    m.resync_wait_left = m.mib.resync_waiting_period;
  }
}

bool macTickCarrierLoss(MacSession& m, Tick dt) noexcept {
  if (m.carrier_loss_left == 0 || !tickDec(m.carrier_loss_left, dt)) {
    return false;
  }
  if (m.duplex == MacDuplex::full && m.state == MacState::s40) {
    if (m.role == MacRole::caller) {  // E80
      applyState(m, MacState::s80);
      loadWait(m, m.mib.drop_carrier_duration);
    } else {  // E82
      applyState(m, MacState::s2);
    }
    m.transmit_on = false;
    return true;
  }
  if (m.duplex == MacDuplex::half && m.state == MacState::s60) {  // E85
    applyState(m, MacState::s2);
    m.transmit_on = false;
    return true;
  }
  return false;
}

void macWaitExpiredConnect(MacSession& m) noexcept {
  switch (m.state) {
    case MacState::s31:  // E4
      applyState(m, MacState::s32);
      m.modulation = true;
      loadWait(m, m.mib.acquisition_idle_duration);
      break;
    case MacState::s32:  // E5
      applyState(m, MacState::s33);
      m.mac_frame_pending = true;
      m.fifo_empty = false;
      break;
    case MacState::s34:  // E7
      applyState(m, MacState::s35);
      m.transmit_on = false;
      loadWait(m, m.mib.hail_wait_duration);
      break;
    case MacState::s35:  // E8
      applyState(m, MacState::s31);
      m.modulation = false;
      m.transmit_on = true;
      loadWait(m, m.mib.carrier_only_duration);
      notify(m, MacNotify::hail_repeat);
      break;
    case MacState::s11:  // E32
      applyState(m, MacState::s12);
      m.modulation = true;
      loadWait(m, m.mib.acquisition_idle_duration);
      break;
    case MacState::s12:  // E33
      applyState(m, MacState::s13);
      m.mac_frame_pending = true;
      m.fifo_empty = false;
      break;
    case MacState::s14:  // E35
      applyState(m, MacState::s36);
      m.modulation = false;
      m.transmit_on = false;
      loadWait(m, m.mib.hail_wait_duration);
      break;
    case MacState::s36:  // E36
      applyState(m, MacState::s11);
      m.transmit_on = true;
      loadWait(m, m.mib.carrier_only_duration);
      notify(m, MacNotify::hail_repeat);
      break;
    default:
      break;
  }
}

void macWaitExpiredFull(MacSession& m, Tick now) noexcept {
  switch (m.state) {
    case MacState::s41:  // E10
      applyState(m, MacState::s42);
      m.modulation = true;
      loadWait(m, m.mib.acquisition_idle_duration);
      break;
    case MacState::s42:  // E11
      applyState(m, MacState::s40);
      break;
    case MacState::s80:
      if (m.duplex == MacDuplex::full) {  // E81
        applyState(m, MacState::s31);
      } else {  // E84
        applyState(m, MacState::s11);
      }
      m.persistence = true;
      m.transmit_on = true;
      loadWait(m, m.mib.carrier_only_duration);
      startHailLife(m);
      break;
    case MacState::s45:  // E26
      enterS1(m, now, true);
      break;
    case MacState::s48:
      if (m.y == 3) {  // E18
        m.y = 1;
      } else if (m.y == 5) {  // E20
        m.y = 0;
        m.persistence = false;
        m.need_plcw = true;
        m.modulation = false;
        applyState(m, MacState::s41);
        loadWait(m, m.mib.carrier_only_duration);
        notify(m, MacNotify::comm_change_ok);
      }
      break;
    default:
      break;
  }
}

void macWaitExpiredHalf(MacSession& m, Tick now) noexcept {
  switch (m.state) {
    case MacState::s50:  // E38
      m.persistence = true;
      m.y = 0;
      break;
    case MacState::s51:  // E40
      applyState(m, MacState::s52);
      m.modulation = true;
      loadWait(m, m.mib.acquisition_idle_duration);
      break;
    case MacState::s52:  // E41
      applyState(m, MacState::s50);
      loadWait(m, m.mib.send_duration);
      break;
    case MacState::s58:
      if (m.y == 2) {  // E67
        m.y = 3;
        applyState(m, MacState::s62);
        loadWait(m, m.mib.receive_duration);
      } else {  // E43
        applyState(m, MacState::s62);
        m.persistence = false;
        m.modulation = false;
        loadWait(m, m.mib.receive_duration);
      }
      break;
    case MacState::s55:  // E57
      enterS1(m, now, true);
      break;
    case MacState::s60:
      if (m.carrier_acquired) {  // E44
        loadWait(m, m.mib.receive_duration);
        notify(m, MacNotify::sender_overran);
      } else {  // E48
        applyState(m, MacState::s51);
        loadWait(m, m.mib.carrier_only_duration);
      }
      break;
    case MacState::s61:
      if (m.carrier_acquired) {  // E45
        loadWait(m, m.mib.receive_duration);
        notify(m, MacNotify::no_data_this_contact);
      } else {  // E50
        applyState(m, MacState::s51);
        loadWait(m, m.mib.carrier_only_duration);
        notify(m, MacNotify::no_data_this_contact);
      }
      break;
    case MacState::s62:
      if (!m.carrier_acquired) {  // E50
        applyState(m, MacState::s51);
        loadWait(m, m.mib.carrier_only_duration);
        notify(m, MacNotify::no_carrier_this_contact);
      }
      break;
    default:
      break;
  }
}

void macOnWaitExpired(MacSession& m, Tick now) noexcept {
  switch (m.state) {
    case MacState::s31:
    case MacState::s32:
    case MacState::s34:
    case MacState::s35:
    case MacState::s11:
    case MacState::s12:
    case MacState::s14:
    case MacState::s36:
      macWaitExpiredConnect(m);
      break;
    case MacState::s41:
    case MacState::s42:
    case MacState::s80:
    case MacState::s45:
    case MacState::s48:
      macWaitExpiredFull(m, now);
      break;
    default:
      macWaitExpiredHalf(m, now);
      break;
  }
}

}  // namespace

void macInit(MacSession& m, MacMib const& mib, MacDuplex duplex,
              CoppEndpoint* copp) noexcept {
  m = MacSession{};
  m.mib = mib;
  m.duplex = duplex;
  m.copp = copp;
  applyTable66(m);
  applyState(m, MacState::s1);
}

void macSetInitializeMode(MacSession& m, Tick now) noexcept {
  enterS1(m, now, true);
}

void macSetDuplex(MacSession& m, MacDuplex duplex) noexcept {
  if (m.state != MacState::s1) {
    return;  // session duplex change is SET CONTROL PARAMETERS
  }
  m.duplex = duplex;
}

void macSetSduPending(MacSession& m, bool pending) noexcept {
  m.sdu_pending = pending;
}

void macSetMode(MacSession& m, MacMode mode, Tick now) noexcept {
  if (mode == MacMode::inactive) {
    enterS1(m, now, true);  // E28 / E61 / E73
    return;
  }
  if (m.state != MacState::s1) {
    return;
  }
  if (mode == MacMode::connecting_l) {
    applyState(m, MacState::s2);  // E1 / E29
    if (m.duplex == MacDuplex::half) {
      m.need_plcw = true;
    }
    m.role = MacRole::responder;
    return;
  }
  if (mode == MacMode::connecting_t) {
    m.role = MacRole::caller;
    m.persistence = true;
    startHailLife(m);
    if (m.duplex == MacDuplex::full) {
      applyState(m, MacState::s31);  // E2
    } else if (m.duplex == MacDuplex::half) {
      applyState(m, MacState::s11);  // E31
    } else {
      return;
    }
    loadWait(m, m.mib.carrier_only_duration);
    return;
  }
  if (mode == MacMode::active) {
    if (m.duplex == MacDuplex::simplex_transmit) {
      applyState(m, MacState::s71);  // E71
      m.modulation = true;
    } else if (m.duplex == MacDuplex::simplex_receive) {
      applyState(m, MacState::s72);  // E72
    }
  }
}

void macOnHailReceived(MacSession& m, Tick now) noexcept {
  (void)now;
  if (m.state == MacState::s2 && m.duplex == MacDuplex::full) {
    applyState(m, MacState::s41);  // E3
    m.need_plcw = true;
    m.transmit_on = true;
    loadWait(m, m.mib.carrier_only_duration);
    notify(m, MacNotify::hail_ok);
    return;
  }
  if (m.state == MacState::s2 && m.duplex == MacDuplex::half) {
    applyState(m, MacState::s51);  // E30
    m.transmit_on = true;
    loadWait(m, m.mib.carrier_only_duration);
    notify(m, MacNotify::hail_ok);
  }
}

void macOnValidFrame(MacSession& m, Tick now) noexcept {
  (void)now;
  if (m.state == MacState::s35) {  // E9
    applyState(m, MacState::s41);
    m.modulation = false;
    m.persistence = false;
    loadWait(m, m.mib.carrier_only_duration);
    notify(m, MacNotify::hail_ok);
    return;
  }
  if (m.state == MacState::s36) {  // E37
    applyState(m, MacState::s60);
    m.persistence = false;
    loadWait(m, m.mib.receive_duration);
    notify(m, MacNotify::hail_ok);
    return;
  }
  if (m.state == MacState::s48 && m.z == 1) {  // E17
    m.y = 0;
    m.z = 0;
    m.persistence = false;
    m.modulation = false;
    applyState(m, MacState::s41);
    loadWait(m, m.mib.carrier_only_duration);
    notify(m, MacNotify::comm_change_ok);
    return;
  }
  if (m.state == MacState::s61 && m.y != 3) {  // E46
    applyState(m, MacState::s60);
    return;
  }
  if (m.state == MacState::s61 && m.y == 3) {  // E68
    m.y = 0;
    m.persistence = false;
    applyState(m, MacState::s60);
    notify(m, MacNotify::comm_change_ok);
  }
}

void macOnFifoEmpty(MacSession& m, Tick now) noexcept {
  (void)now;
  m.fifo_empty = true;
  if (m.state == MacState::s33) {  // E6
    applyState(m, MacState::s34);
    loadWait(m, m.mib.tail_idle_duration);
    return;
  }
  if (m.state == MacState::s13) {  // E34
    applyState(m, MacState::s14);
    loadWait(m, m.mib.tail_idle_duration);
    return;
  }
  if (m.state == MacState::s48 && m.y == 2) {  // E15
    m.y = 3;
    loadWait(m, m.mib.persistence_wait_time);
    return;
  }
  if (m.state == MacState::s48 && m.y == 4) {  // E19
    m.y = 5;
    loadWait(m, m.mib.tail_idle_duration);
  }
}

void macOnNoFramesPending(MacSession& m, Tick now) noexcept {
  (void)now;
  m.no_frames_pending = true;
  if (m.state == MacState::s40 && m.x == 5) {  // E25
    applyState(m, MacState::s45);
    loadWait(m, m.mib.tail_idle_duration);
    return;
  }
  if (m.state == MacState::s48 && m.y == 1) {  // E14
    m.y = 2;
    m.mac_frame_pending = true;
    return;
  }
  if (m.state == MacState::s50 && m.y == 0 && !m.need_plcw) {  // E39
    applyState(m, MacState::s56);
    m.mac_frame_pending = true;
    return;
  }
  if (m.state == MacState::s56 && m.y == 0) {  // E42
    applyState(m, MacState::s58);
    loadWait(m, m.mib.tail_idle_duration);
    ++m.token_fail_n;
    return;
  }
  if (m.state == MacState::s50 && m.y == 2) {  // E65
    applyState(m, MacState::s56);
    m.mac_frame_pending = true;
    return;
  }
  if (m.state == MacState::s56 && m.y == 2) {  // E66
    applyState(m, MacState::s58);
    loadWait(m, m.mib.tail_idle_duration);
    return;
  }
  if (m.state == MacState::s50 && m.x == 1) {  // E53
    m.x = 2;
    m.mac_frame_pending = true;
    return;
  }
  if (m.state == MacState::s50 && m.x == 4) {  // E55
    m.x = 5;
    applyState(m, MacState::s54);
    m.mac_frame_pending = true;
    return;
  }
  if (m.state == MacState::s54 && m.x == 5) {  // E56
    m.x = 0;
    applyState(m, MacState::s55);
    loadWait(m, m.mib.tail_idle_duration);
  }
}

void macLocalNoMoreData(MacSession& m, Tick now) noexcept {
  (void)now;
  if (m.duplex == MacDuplex::full && m.state == MacState::s40) {
    if (m.x == 0) {  // E21
      m.x = 2;
      m.mac_frame_pending = true;
    } else if (m.x == 4) {  // E24
      m.x = 5;
      m.mac_frame_pending = true;
    }
    return;
  }
  if (m.duplex == MacDuplex::half) {
    if (m.x == 0) {  // E51
      m.x = 1;
    } else if (m.x == 3) {  // E52
      m.x = 4;
    }
  }
}

void macOnRnmd(MacSession& m, Tick now) noexcept {
  if (m.duplex == MacDuplex::full && m.state == MacState::s40) {
    if (m.x == 0) {  // E22
      m.x = 4;
    } else if (m.x == 2) {  // E23
      m.x = 5;
    }
    return;
  }
  if (m.duplex == MacDuplex::half) {
    if ((m.state == MacState::s60 || m.state == MacState::s61) && m.x == 2) {
      enterS1(m, now, true);  // E58
      return;
    }
    if ((m.state == MacState::s60 || m.state == MacState::s61) && m.x == 0) {
      m.x = 3;
      applyState(m, MacState::s51);
      loadWait(m, m.mib.carrier_only_duration);
      return;
    }
    if ((m.state == MacState::s60 || m.state == MacState::s61) && m.x == 1) {
      m.x = 4;
      applyState(m, MacState::s51);
      loadWait(m, m.mib.carrier_only_duration);
    }
  }
}

void macLocalCommChange(MacSession& m, Tick now) noexcept {
  (void)now;
  if (m.duplex == MacDuplex::full && m.state == MacState::s40 && m.y == 0) {
    applyState(m, MacState::s48);  // E12
    m.y = 1;
    m.persistence = true;
    return;
  }
  if (m.duplex == MacDuplex::half) {
    if (m.state == MacState::s50) {  // E63
      m.y = 2;
      m.persistence = true;
    } else if (m.y == 0) {  // E62
      m.y = 1;
    }
  }
}

void macOnRemoteCommChange(MacSession& m, Tick now) noexcept {
  (void)now;
  if (m.duplex == MacDuplex::full && m.state == MacState::s40 && m.y == 0) {
    applyState(m, MacState::s48);  // E13
    m.y = 4;
    m.persistence = true;
    return;
  }
  if (m.duplex == MacDuplex::half &&
      (m.state == MacState::s60 || m.state == MacState::s61)) {  // E69
    applyState(m, MacState::s51);
    m.need_plcw = true;
    loadWait(m, m.mib.carrier_only_duration);
  }
}

void macOnToken(MacSession& m, Tick now) noexcept {
  (void)now;
  if (m.state == MacState::s60 || m.state == MacState::s61) {  // E49
    applyState(m, MacState::s51);
    loadWait(m, m.mib.carrier_only_duration);
  }
}

void macSetCarrierAcquired(MacSession& m, bool acquired, Tick now) noexcept {
  const bool was = m.carrier_acquired;
  m.carrier_acquired = acquired;
  if (acquired) {
    m.carrier_loss_left = 0;
    if (m.state == MacState::s62) {  // E47
      applyState(m, MacState::s61);
    }
    return;
  }
  if (was && !acquired && m.mode == MacMode::active &&
      (m.duplex == MacDuplex::full ||
       (m.duplex == MacDuplex::half && !m.transmit_on))) {
    m.carrier_loss_left = m.mib.carrier_loss_timer_duration;
    (void)now;
  }
}

void macSetSymbolInlock(MacSession& m, bool inlock, Tick now) noexcept {
  m.symbol_inlock = inlock;
  if (!inlock && m.state == MacState::s48 &&
      (m.y == 1 || m.y == 2 || m.y == 3)) {  // E16
    m.z = 1;
  }
  if (inlock && m.state == MacState::s35) {
    macOnValidFrame(m, now);  // Hail_Response option
  }
}

void macTick(MacSession& m, Tick now) noexcept {
  Tick dt = 0;
  if (now > m.last_now) {
    dt = now - m.last_now;
  }
  m.last_now = now;
  if (m.copp != nullptr) {
    coppTick(*m.copp, now);
    macDriveSetVr(m, now);
  }
  macTickPlcw(m, dt);
  if (macTickHail(m, now, dt)) {
    return;
  }
  macTickResync(m, dt);
  if (macTickCarrierLoss(m, dt)) {
    return;  // one event; do not also expire Drop_Carrier_Duration this tick
  }
  if (!m.wait_armed || m.wait_left == 0) {
    if (m.mib.maximum_failed_token_passes != 0 &&
        m.token_fail_n >= m.mib.maximum_failed_token_passes &&
        m.state == MacState::s50) {  // E83
      applyState(m, MacState::s80);
      loadWait(m, m.mib.drop_carrier_duration);
      m.transmit_on = false;
    }
    return;
  }
  if (!tickDec(m.wait_left, dt)) {
    return;
  }
  m.wait_armed = false;
  macOnWaitExpired(m, now);
}

MacNotify macPollNotify(MacSession& m) noexcept {
  const MacNotify n = m.notify;
  m.notify = MacNotify::none;
  return n;
}

MacPhy macPhy(MacSession const& m) noexcept {
  MacPhy p{};
  p.transmit = m.transmit_on;
  p.modulation = m.modulation && m.transmit_on;
  switch (m.state) {
    case MacState::s1:
      p.receive = false;
      break;
    case MacState::s11:
    case MacState::s12:
    case MacState::s13:
    case MacState::s14:
    case MacState::s50:
    case MacState::s51:
    case MacState::s52:
    case MacState::s54:
    case MacState::s55:
    case MacState::s56:
    case MacState::s58:
    case MacState::s71:
      p.receive = false;
      break;
    default:
      p.receive = true;
      break;
  }
  return p;
}

MacFifoSource macFifoSource(MacSession const& m) noexcept {
  if (!m.transmit_on || !m.modulation) {
    if (m.ss == 1 && m.transmit_on) {
      return MacFifoSource::carrier_only;
    }
    return MacFifoSource::none;
  }
  if (m.ss == 2 || m.ss == 4 || m.ss == 7) {
    return MacFifoSource::idle;
  }
  if (m.ss == 0 || m.ss == 3 || m.ss == 6) {
    if (m.mac_frame_pending) {
      return MacFifoSource::spdu;
    }
    if (m.persistence) {
      return MacFifoSource::idle;
    }
    if (m.need_plcw || m.need_status_report) {
      return MacFifoSource::plcw;
    }
    if (m.sdu_pending) {
      return MacFifoSource::sdu;
    }
    return MacFifoSource::idle;
  }
  if (m.ss == 1) {
    return MacFifoSource::carrier_only;
  }
  return MacFifoSource::none;
}

Result<std::size_t> encodeSetVr(std::span<std::byte> out, std::uint8_t seq_ctrl_fsn,
                                  Pcid /*pcid*/) noexcept {
  // B1.5: 16 bits. Bit 0–7 SEQ_CTRL_FSN, 8–12 spare 0, 13–15 type 011.
  // PCID is associated with the FOP-P on the MAC that queues this (7.2.3.2.2 b).
  if (out.size() < 2) {
    return tl::unexpected(Error::buffer_too_small);
  }
  out[0] = static_cast<std::byte>(seq_ctrl_fsn);
  out[1] = static_cast<std::byte>(kSetVrDirectiveType);
  return std::size_t{2};
}

Result<std::uint8_t> decodeSetVr(std::span<const std::byte> octets,
                                   Pcid* /*pcid_out*/) noexcept {
  if (octets.size() < 2) {
    return tl::unexpected(Error::truncated);
  }
  const auto b1 = static_cast<std::uint8_t>(octets[1]);
  if ((b1 & 0x07u) != kSetVrDirectiveType || (b1 & 0xF8u) != 0) {
    return tl::unexpected(Error::truncated);
  }
  return static_cast<std::uint8_t>(octets[0]);
}

void macOnSetVrDirective(MacSession& m, std::uint8_t seq_ctrl_fsn) noexcept {
  if (m.copp != nullptr) {
    farmPSetVr(m.copp->farm, seq_ctrl_fsn);  // RE2
  }
}

void macDriveSetVr(MacSession& m, Tick now) noexcept {
  (void)now;
  if (m.copp == nullptr || !m.copp->fop.resync) {
    return;
  }
  if (!m.mac_frame_pending) {
    queueSetVr(m);
    m.resync_wait_left = m.mib.resync_waiting_period;
    if (m.resync_life_left == 0) {
      m.resync_life_left = m.mib.resync_lifetime;
    }
  }
}

void macOnPlcw(MacSession& m, Plcw16 const& w, bool format_ok, Tick now) noexcept {
  (void)now;
  if (m.copp == nullptr) {
    return;
  }
  const bool resyncing = m.copp->fop.resync;
  fopPOnPlcw(m.copp->fop, w, format_ok);
  // 7.2.3.2.1 c / FOP-P SE2 S2: only a *valid* PLCW is Resync_Response.
  if (resyncing && !m.copp->fop.resync) {
    m.persistence = false;
    m.mac_frame_pending = false;
    m.mac_queue_len = 0;
    m.resync_wait_left = 0;
    m.resync_life_left = 0;
    notify(m, MacNotify::resync_ok);
  }
}

}  // namespace starcom::ccsds
