/**
 * Historical CSV provider + optional live poll of the same file.
 * When the conductor window is near "now", refetch CSV every second so a
 * rolling live.csv from stream_station.py feeds the plot without WS.
 */
(function (global) {
  const NAMESPACE = 'rocket-chip.hello';

  function num(v) {
    if (v === undefined || v === null || v === '') return NaN;
    var n = Number(v);
    return Number.isFinite(n) ? n : NaN;
  }

  function parseCsv(text) {
    // Skip leading # comment lines (Big Daddy glance header)
    const rawLines = text.split(/\r?\n/);
    const lines = [];
    for (let i = 0; i < rawLines.length; i++) {
      var L = rawLines[i];
      if (!L || !L.trim()) continue;
      if (L.trim().charAt(0) === '#') continue;
      lines.push(L);
    }
    if (lines.length < 2) return [];
    const header = lines[0].split(',').map(function (h) { return h.trim(); });
    const rows = [];
    for (let i = 1; i < lines.length; i++) {
      const cols = lines[i].split(',');
      const raw = {};
      header.forEach(function (h, idx) { raw[h] = (cols[idx] || '').trim(); });
      const ts = Date.parse(raw.timestamp);
      if (Number.isNaN(ts)) continue;
      const row = { timestamp: ts };
      header.forEach(function (h) {
        if (h === 'timestamp' || h === 'flight_state_name') return;
        var n = num(raw[h]);
        if (!Number.isNaN(n)) row[h] = n;
      });
      // Live scrape aliases <-> Big Daddy names
      if (row.rssi === undefined) row.rssi = num(raw.rssi_dbm);
      if (row.snr === undefined) row.snr = num(raw.snr_db);
      if (row.baro_alt_m === undefined) row.baro_alt_m = num(raw.baro);
      if (row.baro === undefined && row.baro_alt_m !== undefined) row.baro = row.baro_alt_m;
      rows.push(row);
    }
    return rows;
  }

  function applyFixedBounds(openmct, rows) {
    if (!rows || !rows.length) return;
    var start = rows[0].timestamp;
    var end = rows[rows.length - 1].timestamp;
    if (!(end >= start)) return;
    // If data ends near now (live rolling file), skip auto Fixed — stay on realtime conductor
    if (Math.abs(Date.now() - end) < 15000) {
      console.info('[rc-csv] skip auto Fixed; CSV looks live');
      return;
    }
    var pad = Math.max(250, Math.floor((end - start) * 0.02) || 250);
    start -= pad;
    end += pad;
    try {
      if (typeof openmct.time.stopClock === 'function') openmct.time.stopClock();
      openmct.time.setTimeSystem('utc');
      openmct.time.bounds({ start: start, end: end });
      console.info('[rc-csv] Fixed bounds', new Date(start).toISOString(), '->', new Date(end).toISOString());
    } catch (e) {
      console.warn('[rc-csv] could not set Fixed bounds', e);
    }
  }

  function RcCsvHistoricalPlugin(options) {
    options = options || {};
    const csvUrl = options.csvUrl || '../fixtures/omct_big_daddy_glance.csv';

    return function install(openmct) {
      let cache = null;
      let loadPromise = null;
      let boundsApplied = false;
      let lastFetch = 0;

      function loadRows(force) {
        var now = Date.now();
        if (!force && cache && (now - lastFetch) < 800) {
          return Promise.resolve(cache);
        }
        if (!force && loadPromise) return loadPromise;
        var url = csvUrl + (csvUrl.indexOf('?') >= 0 ? '&' : '?') + 't=' + now;
        loadPromise = fetch(url, { cache: 'no-store' })
          .then(function (r) {
            if (!r.ok) throw new Error('CSV fetch failed ' + r.status + ' ' + csvUrl);
            return r.text();
          })
          .then(function (text) {
            cache = parseCsv(text);
            lastFetch = Date.now();
            loadPromise = null;
            console.info('[rc-csv] loaded', cache.length, 'rows from', csvUrl);
            return cache;
          })
          .catch(function (e) {
            loadPromise = null;
            throw e;
          });
        return loadPromise;
      }

      function tryApplyBounds() {
        if (boundsApplied) return;
        loadRows(true).then(function (rows) {
          if (boundsApplied) return;
          boundsApplied = true;
          applyFixedBounds(openmct, rows);
        }).catch(function (e) { console.error(e); });
      }

      if (typeof openmct.on === 'function') openmct.on('start', tryApplyBounds);
      setTimeout(tryApplyBounds, 800);

      // While clock is running, nudge bounds slightly so plots re-request
      setInterval(function () {
        try {
          loadRows(true);
          // Nudge bounds so TelemetryCollection re-requests while clock runs
          if (typeof openmct.time.bounds === 'function') {
            var b = openmct.time.bounds();
            if (b && b.start != null) {
              openmct.time.bounds({ start: b.start, end: b.end });
            }
          }
        } catch (e) {}
      }, 1000);

      openmct.telemetry.addProvider({
        supportsRequest: function (domainObject) {
          return domainObject.type === 'rocket-chip.telemetry' &&
            domainObject.identifier.namespace === NAMESPACE;
        },
        request: function (domainObject, requestOptions) {
          const key = domainObject.identifier.key;
          const force = requestOptions.end >= (Date.now() - 120000);
          return loadRows(force).then(function (rows) {
            const start = requestOptions.start;
            const end = requestOptions.end;
            return rows
              .filter(function (row) {
                return row.timestamp >= start && row.timestamp <= end;
              })
              .map(function (row) {
                return { timestamp: row.timestamp, utc: row.timestamp, value: row[key] };
              });
          });
        }
      });
    };
  }

  global.RcCsvHistoricalPlugin = RcCsvHistoricalPlugin;
})(typeof window !== 'undefined' ? window : globalThis);