/**
 * Realtime provider â€” WebSocket feed of seq/rssi/snr.
 * Keeps a short ring buffer so request()+subscribe both feed plots
 * (oMCT TelemetryCollection uses both under a realtime conductor).
 */
(function (global) {
  const NAMESPACE = 'rocket-chip.hello';
  const BUFFER_MS = 120000;
  // Subscribed on connect so buffers fill before a view opens; never unsubscribed.
  const ALWAYS = ['seq', 'rssi', 'snr', 'baro', 'baro_alt_m', 'alt_m', 'batt_v', 'vvel_mps',
    'speed_mps', 'gps_fix', 'gps_sats', 'flight_state', 'lq_pct', 'lat', 'lon'];

  function RcRealtimePlugin(options) {
    options = options || {};
    const wsUrl = options.wsUrl || 'ws://localhost:8091/';

    return function install(openmct) {
      var socket = null;
      var listeners = {};
      var buffers = { seq: [], rssi: [], snr: [] };
      var reconnectTimer = null;

      function trimBuffer(id, now) {
        var arr = buffers[id];
        if (!arr) return;
        var cut = now - BUFFER_MS;
        while (arr.length && arr[0].timestamp < cut) arr.shift();
      }

      function ingest(point) {
        if (!point || !point.id) return;
        var id = point.id;
        if (!buffers[id]) buffers[id] = [];
        var datum = {
          timestamp: Number(point.timestamp),
          utc: Number(point.timestamp),
          value: point.value
        };
        global.__rcWsLastMs = Date.now();
        buffers[id].push(datum);
        trimBuffer(id, datum.timestamp);
        // Several views can watch one id (alpha + its condition set + plots).
        (listeners[id] || []).slice().forEach(function (cb) {
          try { cb(datum); } catch (e) { console.warn('[rc-rt] callback', e); }
        });
      }

      function connect() {
        if (socket && (socket.readyState === WebSocket.OPEN || socket.readyState === WebSocket.CONNECTING)) {
          return;
        }
        try {
          socket = new WebSocket(wsUrl);
        } catch (e) {
          console.warn('[rc-rt] construct failed', e);
          scheduleReconnect();
          return;
        }
        socket.onopen = function () {
          console.info('[rc-rt] connected', wsUrl);
          Object.keys(listeners).forEach(function (id) {
            try { socket.send('subscribe ' + id); } catch (e) {}
          });
          // Also subscribe all channels so buffers fill even before plot opens
          ALWAYS.forEach(function (id) {
            try { socket.send('subscribe ' + id); } catch (e) {}
          });
        };
        socket.onmessage = function (event) {
          var point;
          try { point = JSON.parse(event.data); } catch (e) { return; }
          ingest(point);
        };
        socket.onclose = function () {
          console.warn('[rc-rt] closed; reconnecting');
          scheduleReconnect();
        };
        socket.onerror = function () {
          console.warn('[rc-rt] socket error');
        };
      }

      function scheduleReconnect() {
        if (reconnectTimer) return;
        reconnectTimer = setTimeout(function () {
          reconnectTimer = null;
          connect();
        }, 1500);
      }

      connect();

      openmct.telemetry.addProvider({
        supportsSubscribe: function (domainObject) {
          return domainObject.type === 'rocket-chip.telemetry' &&
            domainObject.identifier.namespace === NAMESPACE;
        },
        subscribe: function (domainObject, callback) {
          var id = domainObject.identifier.key;
          (listeners[id] = listeners[id] || []).push(callback);
          connect();
          if (socket && socket.readyState === WebSocket.OPEN) {
            try { socket.send('subscribe ' + id); } catch (e) {}
          }
          // Replay last buffered point so the plot isn't empty until the next sample
          var buf = buffers[id];
          if (buf && buf.length) {
            try { callback(buf[buf.length - 1]); } catch (e) {}
          }
          return function unsubscribe() {
            var arr = listeners[id] || [];
            var i = arr.indexOf(callback);
            if (i >= 0) arr.splice(i, 1);
            if (arr.length) return;
            delete listeners[id];
            if (ALWAYS.indexOf(id) >= 0) return;
            if (socket && socket.readyState === WebSocket.OPEN) {
              try { socket.send('unsubscribe ' + id); } catch (e) {}
            }
          };
        }
      });
    };
  }

  global.RcRealtimePlugin = RcRealtimePlugin;
})(typeof window !== 'undefined' ? window : globalThis);