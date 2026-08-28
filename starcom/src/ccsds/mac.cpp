#include "starcom/ccsds/mac.hpp"

namespace starcom::ccsds {
namespace {

void load_wait(MacSession& m, Tick dur) noexcept {
  m.wait_left = dur;
  m.wait_armed = dur != 0;
}

void clear_wait(MacSession& m) noexcept {
  m.wait_left = 0;
  m.wait_armed = false;
}

void notify(MacSession& m, MacNotify n) noexcept { m.notify = n; }

void table_6_6(MacSession& m) noexcept {
  m.transmit_on = false;
  m.modulation = false;
  m.persistence = false;
  m.ss = 0;
  m.x = 0;
  m.y = 0;
  m.z = 0;
  clear_wait(m);
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

void apply_state(MacSession& m, MacState s) noexcept {
  m.state = s;
  switch (s) {
    case MacState::s1:
      m.mode = MacMode::inactive;
      m.transmit_on = false;
      m.ss = 0;
      break;
    case MacState::s2:
      m.mode = MacMode::connecting_l;
      m.transmit_on = false;
      m.ss = 0;
      break;
    case MacState::s80:
      m.mode = MacMode::active;
      m.transmit_on = false;
      m.ss = 0;
      break;
    case MacState::s31:
      m.mode = MacMode::connecting_t;
      m.transmit_on = true;
      m.ss = 1;
      break;
    case MacState::s32:
      m.mode = MacMode::connecting_t;
      m.transmit_on = true;
      m.ss = 2;
      break;
    case MacState::s33:
      m.mode = MacMode::connecting_t;
      m.transmit_on = true;
      m.ss = 3;
      break;
    case MacState::s34:
      m.mode = MacMode::connecting_t;
      m.transmit_on = true;
      m.ss = 4;
      break;
    case MacState::s35:
      m.mode = MacMode::connecting_t;
      m.transmit_on = false;
      m.ss = 5;
      break;
    case MacState::s41:
      m.mode = MacMode::active;
      m.transmit_on = true;
      m.ss = 1;
      break;
    case MacState::s42:
      m.mode = MacMode::active;
      m.transmit_on = true;
      m.ss = 2;
      break;
    case MacState::s40:
      m.mode = MacMode::active;
      m.transmit_on = true;
      m.ss = 0;
      break;
    case MacState::s48:
      m.mode = MacMode::active;
      m.transmit_on = true;
      m.ss = 6;
      break;
    case MacState::s45:
      m.mode = MacMode::active;
      m.transmit_on = true;
      m.ss = 4;
      break;
    case MacState::s11:
      m.mode = MacMode::connecting_t;
      m.transmit_on = true;
      m.ss = 1;
      break;
    case MacState::s12:
      m.mode = MacMode::connecting_t;
      m.transmit_on = true;
      m.ss = 2;
      break;
    case MacState::s13:
      m.mode = MacMode::connecting_t;
      m.transmit_on = true;
      m.ss = 3;
      break;
    case MacState::s14:
      m.mode = MacMode::connecting_t;
      m.transmit_on = true;
      m.ss = 4;
      break;
    case MacState::s36:
      m.mode = MacMode::connecting_t;
      m.transmit_on = false;
      m.ss = 5;
      break;
    case MacState::s51:
      m.mode = MacMode::active;
      m.transmit_on = true;
      m.ss = 1;
      break;
    case MacState::s52:
      m.mode = MacMode::active;
      m.transmit_on = true;
      m.ss = 2;
      break;
    case MacState::s50:
      m.mode = MacMode::active;
      m.transmit_on = true;
      m.ss = 0;
      break;
    case MacState::s54:
      m.mode = MacMode::active;
      m.transmit_on = true;
      m.ss = 3;
      break;
    case MacState::s55:
      m.mode = MacMode::active;
      m.transmit_on = true;
      m.ss = 7;
      break;
    case MacState::s56:
      m.mode = MacMode::active;
      m.transmit_on = true;
      m.ss = 6;
      break;
    case MacState::s58:
      m.mode = MacMode::active;
      m.transmit_on = true;
      m.ss = 4;
      break;
    case MacState::s60:
      m.mode = MacMode::active;
      m.transmit_on = false;
      m.ss = 0;
      break;
    case MacState::s61:
      m.mode = MacMode::active;
      m.transmit_on = false;
      m.ss = 1;
      break;
    case MacState::s62:
      m.mode = MacMode::active;
      m.transmit_on = false;
      m.ss = 2;
      break;
    case MacState::s71:
      m.mode = MacMode::active;
      m.duplex = MacDuplex::simplex_transmit;
      m.transmit_on = true;
      m.ss = 0;
      break;
    case MacState::s72:
      m.mode = MacMode::active;
      m.duplex = MacDuplex::simplex_receive;
      m.transmit_on = false;
      m.ss = 0;
      break;
  }
}

void start_hail_life(MacSession& m) noexcept {
  m.hail_life_left = m.mib.hail_lifetime;
}

void queue_set_vr(MacSession& m) noexcept {
  if (m.copp == nullptr) {
    return;
  }
  const auto n = encode_set_vr(m.mac_queue, m.copp->fop.nn_r, m.copp->pcid);
  if (!n) {
    return;
  }
  m.mac_queue_len = *n;
  m.mac_frame_pending = true;
  m.persistence = true;
}

void enter_s1(MacSession& m, Tick /*now*/, bool end) noexcept {
  table_6_6(m);
  apply_state(m, MacState::s1);
  if (m.copp != nullptr) {
    farm_p_init(m.copp->farm);
    fop_p_init(m.copp->fop, m.copp->fop.mib);
  }
  if (end) {
    notify(m, MacNotify::end_session);
  }
}

}  // namespace

void mac_init(MacSession& m, MacMib const& mib, MacDuplex duplex,
              CoppEndpoint* copp) noexcept {
  m = MacSession{};
  m.mib = mib;
  m.duplex = duplex;
  m.copp = copp;
  table_6_6(m);
  apply_state(m, MacState::s1);
}

void mac_set_initialize_mode(MacSession& m, Tick now) noexcept {
  enter_s1(m, now, true);
}

void mac_set_duplex(MacSession& m, MacDuplex duplex) noexcept {
  if (m.state != MacState::s1) {
    return;  // session duplex change is SET CONTROL PARAMETERS
  }
  m.duplex = duplex;
}

void mac_set_sdu_pending(MacSession& m, bool pending) noexcept {
  m.sdu_pending = pending;
}

void mac_set_mode(MacSession& m, MacMode mode, Tick now) noexcept {
  if (mode == MacMode::inactive) {
    enter_s1(m, now, true);  // E28 / E61 / E73
    return;
  }
  if (m.state != MacState::s1) {
    return;
  }
  if (mode == MacMode::connecting_l) {
    apply_state(m, MacState::s2);  // E1 / E29
    if (m.duplex == MacDuplex::half) {
      m.need_plcw = true;
    }
    m.role = MacRole::responder;
    return;
  }
  if (mode == MacMode::connecting_t) {
    m.role = MacRole::caller;
    m.persistence = true;
    start_hail_life(m);
    if (m.duplex == MacDuplex::full) {
      apply_state(m, MacState::s31);  // E2
    } else if (m.duplex == MacDuplex::half) {
      apply_state(m, MacState::s11);  // E31
    } else {
      return;
    }
    load_wait(m, m.mib.carrier_only_duration);
    return;
  }
  if (mode == MacMode::active) {
    if (m.duplex == MacDuplex::simplex_transmit) {
      apply_state(m, MacState::s71);  // E71
      m.modulation = true;
    } else if (m.duplex == MacDuplex::simplex_receive) {
      apply_state(m, MacState::s72);  // E72
    }
  }
}

void mac_on_hail_received(MacSession& m, Tick now) noexcept {
  (void)now;
  if (m.state == MacState::s2 && m.duplex == MacDuplex::full) {
    apply_state(m, MacState::s41);  // E3
    m.need_plcw = true;
    m.transmit_on = true;
    load_wait(m, m.mib.carrier_only_duration);
    notify(m, MacNotify::hail_ok);
    return;
  }
  if (m.state == MacState::s2 && m.duplex == MacDuplex::half) {
    apply_state(m, MacState::s51);  // E30
    m.transmit_on = true;
    load_wait(m, m.mib.carrier_only_duration);
    notify(m, MacNotify::hail_ok);
  }
}

void mac_on_valid_frame(MacSession& m, Tick now) noexcept {
  (void)now;
  if (m.state == MacState::s35) {  // E9
    apply_state(m, MacState::s41);
    m.modulation = false;
    m.persistence = false;
    load_wait(m, m.mib.carrier_only_duration);
    notify(m, MacNotify::hail_ok);
    return;
  }
  if (m.state == MacState::s36) {  // E37
    apply_state(m, MacState::s60);
    m.persistence = false;
    load_wait(m, m.mib.receive_duration);
    notify(m, MacNotify::hail_ok);
    return;
  }
  if (m.state == MacState::s48 && m.z == 1) {  // E17
    m.y = 0;
    m.z = 0;
    m.persistence = false;
    m.modulation = false;
    apply_state(m, MacState::s41);
    load_wait(m, m.mib.carrier_only_duration);
    notify(m, MacNotify::comm_change_ok);
    return;
  }
  if (m.state == MacState::s61 && m.y != 3) {  // E46
    apply_state(m, MacState::s60);
    return;
  }
  if (m.state == MacState::s61 && m.y == 3) {  // E68
    m.y = 0;
    m.persistence = false;
    apply_state(m, MacState::s60);
    notify(m, MacNotify::comm_change_ok);
  }
}

void mac_on_fifo_empty(MacSession& m, Tick now) noexcept {
  (void)now;
  m.fifo_empty = true;
  if (m.state == MacState::s33) {  // E6
    apply_state(m, MacState::s34);
    load_wait(m, m.mib.tail_idle_duration);
    return;
  }
  if (m.state == MacState::s13) {  // E34
    apply_state(m, MacState::s14);
    load_wait(m, m.mib.tail_idle_duration);
    return;
  }
  if (m.state == MacState::s48 && m.y == 2) {  // E15
    m.y = 3;
    load_wait(m, m.mib.persistence_wait_time);
    return;
  }
  if (m.state == MacState::s48 && m.y == 4) {  // E19
    m.y = 5;
    load_wait(m, m.mib.tail_idle_duration);
  }
}

void mac_on_no_frames_pending(MacSession& m, Tick now) noexcept {
  (void)now;
  m.no_frames_pending = true;
  if (m.state == MacState::s40 && m.x == 5) {  // E25
    apply_state(m, MacState::s45);
    load_wait(m, m.mib.tail_idle_duration);
    return;
  }
  if (m.state == MacState::s48 && m.y == 1) {  // E14
    m.y = 2;
    m.mac_frame_pending = true;
    return;
  }
  if (m.state == MacState::s50 && m.y == 0 && !m.need_plcw) {  // E39
    apply_state(m, MacState::s56);
    m.mac_frame_pending = true;
    return;
  }
  if (m.state == MacState::s56 && m.y == 0) {  // E42
    apply_state(m, MacState::s58);
    load_wait(m, m.mib.tail_idle_duration);
    ++m.token_fail_n;
    return;
  }
  if (m.state == MacState::s50 && m.y == 2) {  // E65
    apply_state(m, MacState::s56);
    m.mac_frame_pending = true;
    return;
  }
  if (m.state == MacState::s56 && m.y == 2) {  // E66
    apply_state(m, MacState::s58);
    load_wait(m, m.mib.tail_idle_duration);
    return;
  }
  if (m.state == MacState::s50 && m.x == 1) {  // E53
    m.x = 2;
    m.mac_frame_pending = true;
    return;
  }
  if (m.state == MacState::s50 && m.x == 4) {  // E55
    m.x = 5;
    apply_state(m, MacState::s54);
    m.mac_frame_pending = true;
    return;
  }
  if (m.state == MacState::s54 && m.x == 5) {  // E56
    m.x = 0;
    apply_state(m, MacState::s55);
    load_wait(m, m.mib.tail_idle_duration);
  }
}

void mac_local_no_more_data(MacSession& m, Tick now) noexcept {
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

void mac_on_rnmd(MacSession& m, Tick now) noexcept {
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
      enter_s1(m, now, true);  // E58
      return;
    }
    if ((m.state == MacState::s60 || m.state == MacState::s61) && m.x == 0) {
      m.x = 3;
      apply_state(m, MacState::s51);
      load_wait(m, m.mib.carrier_only_duration);
      return;
    }
    if ((m.state == MacState::s60 || m.state == MacState::s61) && m.x == 1) {
      m.x = 4;
      apply_state(m, MacState::s51);
      load_wait(m, m.mib.carrier_only_duration);
    }
  }
}

void mac_local_comm_change(MacSession& m, Tick now) noexcept {
  (void)now;
  if (m.duplex == MacDuplex::full && m.state == MacState::s40 && m.y == 0) {
    apply_state(m, MacState::s48);  // E12
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

void mac_on_remote_comm_change(MacSession& m, Tick now) noexcept {
  (void)now;
  if (m.duplex == MacDuplex::full && m.state == MacState::s40 && m.y == 0) {
    apply_state(m, MacState::s48);  // E13
    m.y = 4;
    m.persistence = true;
    return;
  }
  if (m.duplex == MacDuplex::half &&
      (m.state == MacState::s60 || m.state == MacState::s61)) {  // E69
    apply_state(m, MacState::s51);
    m.need_plcw = true;
    load_wait(m, m.mib.carrier_only_duration);
  }
}

void mac_on_token(MacSession& m, Tick now) noexcept {
  (void)now;
  if (m.state == MacState::s60 || m.state == MacState::s61) {  // E49
    apply_state(m, MacState::s51);
    load_wait(m, m.mib.carrier_only_duration);
  }
}

void mac_set_carrier_acquired(MacSession& m, bool acquired, Tick now) noexcept {
  const bool was = m.carrier_acquired;
  m.carrier_acquired = acquired;
  if (acquired) {
    m.carrier_loss_left = 0;
    if (m.state == MacState::s62) {  // E47
      apply_state(m, MacState::s61);
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

void mac_set_symbol_inlock(MacSession& m, bool inlock, Tick now) noexcept {
  m.symbol_inlock = inlock;
  if (!inlock && m.state == MacState::s48 &&
      (m.y == 1 || m.y == 2 || m.y == 3)) {  // E16
    m.z = 1;
  }
  if (inlock && m.state == MacState::s35) {
    mac_on_valid_frame(m, now);  // Hail_Response option
  }
}

void mac_tick(MacSession& m, Tick now) noexcept {
  Tick dt = 0;
  if (now > m.last_now) {
    dt = now - m.last_now;
  }
  m.last_now = now;
  if (m.copp != nullptr) {
    copp_tick(*m.copp, now);
    mac_drive_set_vr(m, now);
  }

  auto dec = [dt](Tick& t) -> bool {
    if (t == 0 || dt == 0) {
      return false;
    }
    if (dt >= t) {
      t = 0;
      return true;
    }
    t = static_cast<Tick>(t - dt);
    return t == 0;
  };

  if (m.plcw_left != 0 && dec(m.plcw_left)) {
    m.need_plcw = true;
    m.plcw_left = m.mib.plcw_repeat_interval;
  }

  if (m.hail_life_left != 0 && dec(m.hail_life_left)) {
    notify(m, MacNotify::hail_fail);
    enter_s1(m, now, true);
    return;
  }

  if (m.resync_life_left != 0 && dec(m.resync_life_left)) {
    notify(m, MacNotify::resync_fail);
    m.persistence = false;
    m.mac_frame_pending = false;
    if (m.copp != nullptr) {
      m.copp->fop.resync = false;
      m.copp->fop.state = FopPState::s1_active;
    }
  }

  if (m.resync_wait_left != 0 && dec(m.resync_wait_left) && m.copp != nullptr &&
      m.copp->fop.resync) {
    queue_set_vr(m);
    m.resync_wait_left = m.mib.resync_waiting_period;
  }

  if (m.carrier_loss_left != 0 && dec(m.carrier_loss_left)) {
    if (m.duplex == MacDuplex::full && m.state == MacState::s40) {
      if (m.role == MacRole::caller) {  // E80
        apply_state(m, MacState::s80);
        load_wait(m, m.mib.drop_carrier_duration);
        m.transmit_on = false;
      } else {  // E82
        apply_state(m, MacState::s2);
        m.transmit_on = false;
      }
      return;  // one event; do not also expire Drop_Carrier_Duration this tick
    } else if (m.duplex == MacDuplex::half && m.state == MacState::s60) {  // E85
      apply_state(m, MacState::s2);
      m.transmit_on = false;
      return;
    }
  }

  if (!m.wait_armed || m.wait_left == 0) {
    if (m.mib.maximum_failed_token_passes != 0 &&
        m.token_fail_n >= m.mib.maximum_failed_token_passes &&
        m.state == MacState::s50) {  // E83
      apply_state(m, MacState::s80);
      load_wait(m, m.mib.drop_carrier_duration);
      m.transmit_on = false;
    }
    return;
  }
  if (!dec(m.wait_left)) {
    return;
  }
  m.wait_armed = false;

  switch (m.state) {
    case MacState::s31:  // E4
      apply_state(m, MacState::s32);
      m.modulation = true;
      load_wait(m, m.mib.acquisition_idle_duration);
      break;
    case MacState::s32:  // E5
      apply_state(m, MacState::s33);
      m.mac_frame_pending = true;
      m.fifo_empty = false;
      break;
    case MacState::s34:  // E7
      apply_state(m, MacState::s35);
      m.transmit_on = false;
      load_wait(m, m.mib.hail_wait_duration);
      break;
    case MacState::s35:  // E8
      apply_state(m, MacState::s31);
      m.modulation = false;
      m.transmit_on = true;
      load_wait(m, m.mib.carrier_only_duration);
      notify(m, MacNotify::hail_repeat);
      break;
    case MacState::s41:  // E10
      apply_state(m, MacState::s42);
      m.modulation = true;
      load_wait(m, m.mib.acquisition_idle_duration);
      break;
    case MacState::s42:  // E11
      apply_state(m, MacState::s40);
      break;
    case MacState::s80:
      if (m.duplex == MacDuplex::full) {  // E81
        apply_state(m, MacState::s31);
      } else {  // E84
        apply_state(m, MacState::s11);
      }
      m.persistence = true;
      m.transmit_on = true;
      load_wait(m, m.mib.carrier_only_duration);
      start_hail_life(m);
      break;
    case MacState::s45:  // E26
      enter_s1(m, now, true);
      break;
    case MacState::s48:
      if (m.y == 3) {  // E18
        m.y = 1;
      } else if (m.y == 5) {  // E20
        m.y = 0;
        m.persistence = false;
        m.need_plcw = true;
        m.modulation = false;
        apply_state(m, MacState::s41);
        load_wait(m, m.mib.carrier_only_duration);
        notify(m, MacNotify::comm_change_ok);
      }
      break;
    case MacState::s11:  // E32
      apply_state(m, MacState::s12);
      m.modulation = true;
      load_wait(m, m.mib.acquisition_idle_duration);
      break;
    case MacState::s12:  // E33
      apply_state(m, MacState::s13);
      m.mac_frame_pending = true;
      m.fifo_empty = false;
      break;
    case MacState::s14:  // E35
      apply_state(m, MacState::s36);
      m.modulation = false;
      m.transmit_on = false;
      load_wait(m, m.mib.hail_wait_duration);
      break;
    case MacState::s36:  // E36
      apply_state(m, MacState::s11);
      m.transmit_on = true;
      load_wait(m, m.mib.carrier_only_duration);
      notify(m, MacNotify::hail_repeat);
      break;
    case MacState::s50:  // E38
      m.persistence = true;
      m.y = 0;
      break;
    case MacState::s51:  // E40
      apply_state(m, MacState::s52);
      m.modulation = true;
      load_wait(m, m.mib.acquisition_idle_duration);
      break;
    case MacState::s52:  // E41
      apply_state(m, MacState::s50);
      load_wait(m, m.mib.send_duration);
      break;
    case MacState::s58:
      if (m.y == 2) {  // E67
        m.y = 3;
        apply_state(m, MacState::s62);
        load_wait(m, m.mib.receive_duration);
      } else {  // E43
        apply_state(m, MacState::s62);
        m.persistence = false;
        m.modulation = false;
        load_wait(m, m.mib.receive_duration);
      }
      break;
    case MacState::s55:  // E57
      enter_s1(m, now, true);
      break;
    case MacState::s60:
      if (m.carrier_acquired) {  // E44
        load_wait(m, m.mib.receive_duration);
        notify(m, MacNotify::sender_overran);
      } else {  // E48
        apply_state(m, MacState::s51);
        load_wait(m, m.mib.carrier_only_duration);
      }
      break;
    case MacState::s61:
      if (m.carrier_acquired) {  // E45
        load_wait(m, m.mib.receive_duration);
        notify(m, MacNotify::no_data_this_contact);
      } else {  // E50
        apply_state(m, MacState::s51);
        load_wait(m, m.mib.carrier_only_duration);
        notify(m, MacNotify::no_data_this_contact);
      }
      break;
    case MacState::s62:
      if (!m.carrier_acquired) {  // E50
        apply_state(m, MacState::s51);
        load_wait(m, m.mib.carrier_only_duration);
        notify(m, MacNotify::no_carrier_this_contact);
      }
      break;
    default:
      break;
  }
}

MacNotify mac_poll_notify(MacSession& m) noexcept {
  const MacNotify n = m.notify;
  m.notify = MacNotify::none;
  return n;
}

MacPhy mac_phy(MacSession const& m) noexcept {
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

MacFifoSource mac_fifo_source(MacSession const& m) noexcept {
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

Result<std::size_t> encode_set_vr(std::span<std::byte> out, std::uint8_t seq_ctrl_fsn,
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

Result<std::uint8_t> decode_set_vr(std::span<const std::byte> octets,
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

void mac_on_set_vr_directive(MacSession& m, std::uint8_t seq_ctrl_fsn) noexcept {
  if (m.copp != nullptr) {
    farm_p_set_vr(m.copp->farm, seq_ctrl_fsn);  // RE2
  }
}

void mac_drive_set_vr(MacSession& m, Tick now) noexcept {
  (void)now;
  if (m.copp == nullptr || !m.copp->fop.resync) {
    return;
  }
  if (!m.mac_frame_pending) {
    queue_set_vr(m);
    m.resync_wait_left = m.mib.resync_waiting_period;
    if (m.resync_life_left == 0) {
      m.resync_life_left = m.mib.resync_lifetime;
    }
  }
}

void mac_on_plcw(MacSession& m, Plcw16 const& w, bool format_ok, Tick now) noexcept {
  (void)now;
  if (m.copp == nullptr) {
    return;
  }
  const bool resyncing = m.copp->fop.resync;
  fop_p_on_plcw(m.copp->fop, w, format_ok);
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
