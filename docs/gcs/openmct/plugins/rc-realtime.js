/**
 * Realtime provider — WebSocket feed of seq/rssi/snr (same objects as CSV historical).
 * Expects WS messages: {"id":"rssi","timestamp":<ms>,"value":-54}
 * Default URL: ws://localhost:8091/ (see realtime/stream_station.py)
 */
(function (global) {
  const NAMESPACE = 'rocket-chip.hello';

  function RcRealtimePlugin(options) {
    options = options || {};
    const wsUrl = options.wsUrl || 'ws://localhost:8091/';

    return function install(openmct) {
      var socket = null;
      var listeners = {};
      var reconnectTimer = null;

      function connect() {
        if (socket && (socket.readyState === WebSocket.OPEN || socket.readyState === WebSocket.CONNECTING)) {
          return;
        }
        try {
          socket = new WebSocket(wsUrl);
        } catch (e) {
          console.warn('[rc-rt] WebSocket construct failed', e);
          scheduleReconnect();
          return;
        }
        socket.onopen = function () {
          console.info('[rc-rt] connected', wsUrl);
          Object.keys(listeners).forEach(function (id) {
            try { socket.send('subscribe ' + id); } catch (e) {}
          });
        };
        socket.onmessage = function (event) {
          var point;
          try { point = JSON.parse(event.data); } catch (e) { return; }
          if (!point || !point.id) return;
          var cb = listeners[point.id];
          if (cb) cb(point);
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
          if (socket && socket.readyState === WebSocket.OPEN) {
            try { socket.send('subscribe ' + id); } catch (e) {}
          } else {
            connect();
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