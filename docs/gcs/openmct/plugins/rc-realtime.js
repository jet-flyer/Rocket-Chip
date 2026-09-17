/**
 * Realtime provider — WebSocket feed of seq/rssi/snr.
 * Keeps a short ring buffer so request()+subscribe both feed plots
 * (oMCT TelemetryCollection uses both under a realtime conductor).
 */
(function (global) {
  const NAMESPACE = 'rocket-chip.hello';
  const BUFFER_MS = 120000;

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
        buffers[id].push(datum);
        trimBuffer(id, datum.timestamp);
        var cb = listeners[id];
        if (cb) {
          try { cb(datum); } catch (e) { console.warn('[rc-rt] callback', e); }
        }
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
          ['seq', 'rssi', 'snr', 'baro'].forEach(function (id) {
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
          listeners[id] = callback;
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
            delete listeners[id];
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